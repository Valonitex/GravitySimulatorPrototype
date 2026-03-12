//hi

#include "EndBrace.h"


double dt = (1.0f / 120.0f);

void drawGrid(const std::vector<std::vector<char>> livyud)
{
	for (int i = 0; i < livyud.size(); i++)
	{
		for (int j = 0; j < livyud[i].size(); j++)
		{
			std::cout << livyud[i][j] << " ";
		}
		LOG("");
	};
}

class vectorP
{
public:
	double icap;
	double jcap;
	//double inclineT;
	//double inclineC;

public:
	vectorP(double i=0, double j=0)
		:icap(i), jcap(j)
	{
		//inclineT = (icap != 0) ? (jcap / icap) : 0;
		//inclineC = (icap != 0) ? (icap / mag) : 0;

	}
	void getInfo()
	{
		LOG(icap << "\n" << jcap << "\n");
	}
	void updateValues()
	{
		//inclineT = (icap != 0) ? (jcap / icap) : 0;
		//inclineC = (icap != 0) ? (icap / mag) : 0;
	}
	double magSq() const
	{
		return icap*icap + jcap*jcap;
	}
	double mag() const
	{
		return sqrt(magSq());
	}

	vectorP operator+=(const vectorP& other)
	{

		icap += other.icap;
		jcap += other.jcap;
		//updateValues();

		return (*this);
	}

	vectorP operator-=(const vectorP& other)
	{
		icap -= other.icap;
		jcap -= other.jcap;
		//updateValues();

		return (*this);
	}

	vectorP operator+(const vectorP& other) const 
	{
		return (vectorP(icap + other.icap, jcap + other.jcap));
	}

	vectorP operator-(const vectorP& other) const //const coz matrix mul told me 
	{
		return (vectorP(icap - other.icap, jcap - other.jcap));
	}

	vectorP operator*(double t) const 
	{
		return (vectorP(icap * t, jcap * t));
	}
	 
	vectorP operator/(double t) const
	{
		return (vectorP(icap / t, jcap / t));
	}

	vectorP operator/=(double t)
	{
		icap /= t;
		jcap /= t;
		//updateValues();

		return (*this);
	}

	vectorP& operator=(const vectorP& other)
	{
		icap = other.icap;
		jcap = other.jcap;
		//updateValues();

		return *this;
	}
	bool operator!=(const vectorP& other) const
	{
		if (icap != other.icap || jcap != other.jcap)
		{
			return true;
		}
		return false;
	}
	bool operator==(const vectorP& other) const
	{
		if (icap == other.icap && jcap == other.jcap)
		{
			return true;
		}
		return false;
	}

	vectorP negate()
	{
		return vectorP(-(icap), -(jcap));
	}

	vectorP round()
	{
		return vectorP(std::round(icap), std::round(jcap));
	}

	void negate_inp()
	{
		icap *= -1;
		jcap *= -1;
	}
};

std::ostream& operator<<(std::ostream& stream, const vectorP& other)
{
	stream << other.icap << "," << other.jcap ;
	return stream;
}

class Body
{
public:
	bool dead;
	bool movability;
	vectorP m_posVec;
	vectorP m_velVec;
	vectorP m_accVec;
	vectorP m_forVec;
	vectorP m_forRes;
	double m_radius;
	double m_Mass;

public:
	Body(double m, double r, bool stat = true, vectorP pos = { 0,0 }, vectorP vel = { 0,0 }, vectorP f = { 0,0 }) //stat = static
		:dead(false),movability(stat), m_posVec(pos), m_velVec(vel), m_forVec(f), m_accVec(0.0f, 0.0f)
	{
		if (m <= 0 || r<=0)
			throw std::invalid_argument("Mass/Radius be positive");
		m_Mass = m;
		m_radius = r;

		m_accVec = m_forVec / m_Mass;
	}

	void updateVal()
	{
		m_forVec = m_forRes;
		m_accVec = m_forVec / m_Mass;
		m_forRes = (0, 0);
	}

	void forsum(vectorP force)
	{
		m_forRes += force;
	}

	void GetVal()
	{
		LOG(m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << m_forVec << "\n" << "---------");
	}

	std::unique_ptr<Body> clone() const
	{
		return std::make_unique<Body>(m_Mass, m_radius, movability, m_posVec, m_velVec, m_forVec);
	}

};

namespace physics {
	constexpr double G = 6.67430e-11;

	vectorP displacement(const Body& a,const Body& b)
	{
		return (a.m_posVec - b.m_posVec);
	}

	void pull(Body& a, Body& b)
	{
		vectorP disp = physics::displacement(a, b);
		double eps = 0.1;

		double distSq = disp.magSq() + eps * eps;
		double invdist =  1.0/sqrt(distSq);
		double denom = invdist * invdist * invdist ;

		vectorP pullvec = (disp) * ((physics::G * a.m_Mass * b.m_Mass) * denom);

		b.forsum(pullvec);
		pullvec.negate_inp();
		a.forsum(pullvec);
		
		//LOG(pullvec);
	}

	std::vector<std::unique_ptr<Body>> checkCol(std::vector<std::unique_ptr<Body>>& bodies)
	{
		int n = bodies.size();

		// Build collision graph using Union-Find
		std::vector<int> parent(n);
		for (int i = 0; i < n; i++) parent[i] = i;

		std::function<int(int)> find = [&](int x) {
			return parent[x] == x ? x : parent[x] = find(parent[x]);
			};

		auto unite = [&](int x, int y) {
			parent[find(x)] = find(y);
			};

		// Detect all collisions and union them
		for (int i = 0; i < n - 1; i++)
		{
			if (!bodies[i]) continue;  // ✓ Skip if already moved

			Body& boda = *bodies[i];
			for (int j = i + 1; j < n; j++)
			{
				if (!bodies[j]) continue;  // ✓ Skip if already moved

				Body& bodb = *bodies[j];
				double disp = displacement(boda, bodb).mag();
				double mindisp = boda.m_radius + bodb.m_radius;

				if (disp < mindisp)
				{
					unite(i, j);
				}
			}
		}

		// Group bodies by collision cluster
		std::map<int, std::vector<int>> clusters;
		for (int i = 0; i < n; i++)
		{
			int root = find(i);
			clusters[root].push_back(i);
		}

		// Merge each cluster with 2+ bodies
		std::vector<std::unique_ptr<Body>> deadBodies;
		for (auto& [root, indices] : clusters)
		{
			if (indices.size() >= 2)
			{
				// Calculate merged properties
				double totalMass = 0;
				vectorP weightedPos(0, 0);
				vectorP weightedVel(0, 0);
				vectorP totalForce(0, 0);
				double volumeSum = 0;

				for (int idx : indices)
				{
					Body& b = *bodies[idx];
					totalMass += b.m_Mass;
					weightedPos += b.m_posVec * b.m_Mass;
					weightedVel += b.m_velVec * b.m_Mass;
					totalForce += b.m_forVec;
					volumeSum += b.m_radius * b.m_radius * b.m_radius;

					b.dead = true;
					deadBodies.push_back(std::move(bodies[idx]));
				}

				vectorP newPos = weightedPos / totalMass;
				vectorP newVel = weightedVel / totalMass;
				vectorP newFor = totalForce / indices.size();
				double newRadius = pow(volumeSum, 1.0 / 3.0);

				auto newBody = std::make_unique<Body>(totalMass, newRadius, true, newPos, newVel, newFor);
				bodies.push_back(std::move(newBody));
			}
		}

		// Remove dead bodies
		auto it = bodies.begin();
		while (it != bodies.end())
		{
			if ((*it)->dead)
				it = bodies.erase(it);
			else
				++it;
		}

		return deadBodies;
	}

	void resolve(std::vector<std::unique_ptr<Body>>& bodies)
	{
		int s = bodies.size();
		for (int i = 0; i < (s-1); i++)
		{
			auto& bodya = *bodies[i];
			for (int j = i + 1; j < bodies.size() ; j++)
			{
				physics::pull( bodya, *(bodies[j]));
			}
			bodies[i]->updateVal();
		}

		bodies[s - 1]->updateVal();
	}

	void move(std::vector<std::unique_ptr<Body>>& bodies)
	{
		resolve(bodies);
		int s = bodies.size();
		std::vector<vectorP> oldacc(s);
		double dtb2 = dt / 2;
		double dt2b2 = dt * dtb2;
		for (int i = 0; i < s; i++)
		{
			if(bodies[i]->movability != false)
			{
				vectorP temp = bodies[i]->m_accVec;
				bodies[i]->m_posVec += /*bodies[i]->m_posVec*/  bodies[i]->m_velVec * dt + (temp * dt2b2);
				oldacc[i] = temp;
			}

		}
		resolve(bodies);
		for (int i = 0; i < s; i++)
		{
			if (bodies[i]->movability != false)
			{
				bodies[i]->m_velVec += (oldacc[i] + bodies[i]->m_accVec) * dtb2;
			}
		}
		
	}
}

struct BodyInput
{
	double x, y;
	double xv, yv;

	vectorP pos;
	vectorP vel;

	double mass, radius;
	int movable; //if its kept bool and a value other than 0 ir 1 is given it goes haywire , atleast for my poor code
};

BodyInput getValBod()
{
	BodyInput temp;

	int x;
	std::cout << "Position Coord (X) :";
	std::cin >> x;

	int y;
	std::cout << "Position Coord (Y) :";
	std::cin >> y;

	temp.pos = vectorP(x, y);

	std::cout << "Velocity Vec (X):";
	std::cin >> x;

	std::cout << "Velocity Vec (Y):";
	std::cin >> y;

	temp.vel = vectorP(x, y);

	do
	{
		std::cout << "Mass :";
		std::cin >> temp.mass;
	} while (temp.mass <= 0);

	do
	{
		std::cout << "Radius :";
		std::cin >> temp.radius;
	} while (temp.radius <= 0); 

	do
	{
		std::cout << "Movability (0/1):";
		std::cin >> temp.movable;
	} while (temp.movable < 0 || temp.movable > 1); //!= doesnt work because then both need to be satisfied which can nevver happen

	return temp;
}

void getValArr(std::vector<std::unique_ptr<Body>>& bodies)
{
	int bsize = bodies.size();
	for (int i = 0; i < bsize; i++)
	{
		LOG("["<<i<<"]");
		bodies[i]->GetVal();
	}
}

namespace create
{
	void bodyAdd(std::vector<std::unique_ptr<Body>>& bodies)
	{
		BodyInput temp = getValBod();

		auto bod = std::make_unique<Body>(temp.mass, temp.radius, temp.movable, temp.pos, temp.vel);
		bodies.push_back(std::move(bod));
	}
	/*std::unique_ptr<Body> bodyExtract(BodyInput temp)
	{
		return(std::make_unique<Body>(temp.mass, temp.radius, temp.movable, vectorP(temp.x, temp.y), vectorP(temp.xv, temp.yv)));
	}*/
}

int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	std::vector<std::unique_ptr<Body>> bodys;
	std::vector<std::unique_ptr<Body>> delBods;
	std::vector<std::vector<std::unique_ptr<Body>>> colPairs;

	int operation;

	do

	{
		std::cout << "0:Exit\n1:Add Body\n2:Edit Body\n3:Delete Body\n4:Run\n5:View\n-----------------\n Choose:";
		std::cin >> operation;


		if (operation == 1)
		{

			LOG("-----")
			create::bodyAdd(bodys);
		}

		if (operation == 2)
		{
			LOG("-----")
			getValArr(bodys);
			int bsize = bodys.size();

			int b;
			do
			{
				std::cout << "Choose:";
				std::cin >> b;
			} while (b>bsize || b<0);

			Body& fn = *bodys[b]; //reference to a body fn = fornow??? i guess
			BodyInput temp = getValBod();
			fn.m_posVec = temp.pos;
			fn.m_velVec = temp.vel;
			fn.m_Mass = temp.mass;
			fn.m_radius = temp.radius;
			fn.movability = temp.movable; 

		}

		if (operation == 3)
		{
			LOG("-----")
			getValArr(bodys);
			int bsize = bodys.size();

			int b;
			do
			{
				std::cout << "Choose:";
				std::cin >> b;
			} while (b >= bsize || b < 0);

			delBods.push_back(std::move(bodys[b]));
			bodys.erase(bodys.begin() + b);
		}

		if (operation == 5)
		{
			vectorP vector(6, 8);
			vectorP vector2(3, 4);
			vectorP vector4(2, 2); //actual is 2,-2

			auto star = std::make_unique<Body>(1000000000000.0f, 0.1f, true, vector);
			auto perf = std::make_unique<Body>(10.0f, 0.1f, true, vector2, vector4);

			bodys.push_back(std::move(star));
			bodys.push_back(std::move(perf));

			LOG("-----")
			LOG("Alive\n------------")
				for (int i = 0; i < bodys.size(); i++)
				{
					bodys[i]->GetVal();
				}
			LOG("Dead\n------------");

			for (auto& pair : colPairs)
			{
				int psize = pair.size();
				if (psize >= 2)
				{
					for (int i = 0; i < psize; i++)
					{
						pair[i]->GetVal();
					}
				}
			}
			LOG("Deleted\n-------------");
			for (int i = 0; i < delBods.size(); i++)
			{
				delBods[i]->GetVal();
			}
		}
		
		/*vectorP vectorN(0, 0);
		vectorP poso(14, 8);*/


		
		/*auto z = std::make_unique<Body>(1000000000000.0f, 0.1f, false, poso);

		
		bodys.push_back(std::move(z));*/

		if (operation == 4)
		{
			LOG("-----")
			float dur;
			std::cout << "Runtime:";
			std::cin >> dur;

			int stat;
			do
			{
				std::cout << "Grid / Raw Data / No data ? (0/1/2) :";
				std::cin >> stat;
			} while (stat < 0 || stat > 2);


			int fps;
			if(stat==0)
			{
				std::cout << "Per how many frames ? (calculated at 120fps):";
				std::cin >> fps;
			}

			std::vector<std::unique_ptr<Body>> bodOs;
			bodOs.reserve(bodys.size());
			for (const auto& b : bodys) // reference is important as otherwise itll try to copy a unique_ptr into b
				bodOs.push_back(b ? b->clone() : nullptr);
			
			std::vector<vectorP> posOs(bodys.size());


			for (int i = 0; i < bodys.size(); i++)
			{
				posOs[i] = bodys[i]->m_posVec;
			}  //i dont need this becasue of my poor design choices , at start every posOs = 0 and since
			// i check every posOs for every body each posOs is valid because its also checked for 0 wait WTFF,nvm
			// i needed that because what it i dont have a body at 0,0  


			std::vector<char> livyur(21, '.');
			std::vector<std::vector<char>> livyud(21, livyur);

			//std::vector<char> livyurc(21, '.'); 
			//std::vector<std::vector<char>> livyudc(21, livyurc);

			int rerun = 1;

			do 
			{
				bodys.clear();
				bodys.reserve(bodOs.size());
				for (const auto& b : bodOs)
					bodys.push_back(b ? b->clone() : nullptr);

				std::chrono::duration<double> duration(dur);

				auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

				auto start = clock::now();
				auto end = start + duration;
				auto nextFrame = start;
				int frame = 0;

				while (clock::now() < end && bodys.size() > 0)
				{
					
					if (stat == 0)
					{
						posOs.resize(bodys.size());
						for (int i = 0; i < bodys.size(); i++)
						{

							vectorP tbpv = bodys[i]->m_posVec.round();  //tbpv = temporary bodies postition vector

							if (tbpv.icap < 0 || tbpv.icap > 20 || tbpv.jcap < 0 || tbpv.jcap > 20)
							{
								tbpv = (0, 0); // AHHH ts so goated as its tbps in 0 its ovec is 0 and since 
								//the coords dont match ovec 0,0 will still be "." ahahhaahah
							}

							posOs[i] = tbpv;
						}
					}

					frame++;
					//auto t0 = clock::now();

					physics::move(bodys);
					auto killed = (physics::checkCol(bodys));
					if (!killed.empty())
					{
						colPairs.push_back(std::move(killed));
					}
					if (stat == 0)
					{
						posOs.resize(bodys.size());

						for (int i = 0; i < posOs.size(); i++)
						{
							vectorP Ovec = posOs[i];
							bool booly = false;
							for (int j = 0; j < bodys.size(); j++)
							{
								booly = (Ovec == bodys[j]->m_posVec.round());
								if (booly == true)
								{
									break;
								}
							}
							if (booly == false)
							{
								livyud[Ovec.jcap][Ovec.icap] = '.';
							}
							else
							{
								livyud[Ovec.jcap][Ovec.icap] = 'O';
							}
						}
					}

					if (stat == 1)
					{
						posOs.resize(bodys.size());
						for (int i = 0; i < bodys.size(); i++)
						{
							bodys[i]->GetVal();
						}
					}

					if (stat == 0 && frame % fps == 0)
					{
						drawGrid(livyud);
						LOG("----------------------------");
					}


					/*auto t1 = clock::now();
					auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
					double dt_ms = dt * 1000000.0f;
					std::cout << "work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n";*/

					nextFrame += dt_duration;
					std::this_thread::sleep_until(nextFrame);

				}

				std::cout << "Rerun ? (0/1)";
				std::cin >> rerun;

				if (rerun == 1)
				{
					std::vector<char> dots(21, '.');
					for (int i = 0; i < 21; i++)
					{
						livyud[i] = dots;
					}
				}

			} while (rerun == 1);

			drawGrid(livyud);
			LOG("Alive\n--------------")
				for (int i = 0; i < bodys.size(); i++)
				{
					bodys[i]->GetVal();
				}
			LOG("Dead\n------------");

			for (auto& pair : colPairs)
			{
				int psize = pair.size();
				if (psize >= 2)
				{
					for (int i = 0; i < psize; i++)
					{
						pair[i]->GetVal();
					}
				}
			}
			LOG("Deleted\n-------------");
			for (int i = 0; i < delBods.size(); i++)
			{
				delBods[i]->GetVal();
			}

		}

		LOG("---------------")
	} while (operation != 0);

	std::cin.get();
}
