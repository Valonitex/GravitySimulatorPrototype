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
		updateValues();

		return (*this);
	}

	vectorP operator-=(const vectorP& other)
	{
		icap -= other.icap;
		jcap -= other.jcap;
		updateValues();

		return (*this);
	}

	vectorP operator+(const vectorP& other)
	{
		return (vectorP(icap + other.icap, jcap + other.jcap));
	}

	vectorP operator-(const vectorP& other)
	{
		return (vectorP(icap - other.icap, jcap - other.jcap));
	}

	vectorP operator*(double t)
	{
		return (vectorP(icap * t, jcap * t));
	}

	vectorP operator/(double t)
	{
		return (vectorP(icap / t, jcap / t));
	}

	vectorP operator/=(double t)
	{
		icap /= t;
		jcap /= t;
		updateValues();

		return (*this);
	}

	vectorP& operator=(const vectorP& other)
	{
		icap = other.icap;
		jcap = other.jcap;
		updateValues();

		return *this;
	}
	bool operator!=(const vectorP& other)
	{
		if (icap != other.icap && jcap != other.jcap)
		{
			return true;
		}
		return false;
	}
	bool operator==(const vectorP& other)
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
	vectorP m_posVec;
	vectorP m_posVec0;
	vectorP m_velVec;
	vectorP m_accVec;
	vectorP m_forVec;
	vectorP m_forRes;
	double m_radius;
	double m_Mass;

public:
	Body(double m,double r, vectorP pos = { 0,0 }, vectorP vel = { 0,0 }, vectorP f = { 0,0 })
		:dead(false) ,m_posVec(pos),m_posVec0(0.0f,0.0f), m_velVec(vel), m_forVec(f), m_accVec(0.0f, 0.0f)
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

	void Move()
	{
		m_velVec += m_accVec * dt;

		m_posVec += m_velVec * dt;

	}

	void GetVal()
	{
		LOG(m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << m_forVec << "\n" << "---------");
	}

};

namespace physics {
	constexpr double G = 6.67430e-11;

	vectorP displacement(Body a, Body b)
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
	void checkCol(std::vector<std::unique_ptr<Body>>& bodies)
	{
		for (int i = 0; i < (bodies.size() - 1); i++)
		{
			Body& boda = *bodies[i];
			for (int j = i+1; j < bodies.size(); j++)
			{
				Body& bodb = *bodies[j];
				double disp = displacement(boda, bodb).mag();
				double mindisp = boda.m_radius + bodb.m_radius;

				if (disp < mindisp)
				{
					boda.dead = true;
					LOG(i << j << "gone poof");
					boda.GetVal();
					bodb.GetVal();
					bodb.dead = true;
				}
			}
		}
		bodies.erase(
			std::remove_if(
				bodies.begin(),
				bodies.end(),
				[](const std::unique_ptr<Body>& b) {
					return b->dead;
				}
			),
			bodies.end()
		);
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
			vectorP temp = bodies[i]->m_accVec;
			bodies[i]->m_posVec0 = bodies[i]->m_posVec;
			bodies[i]->m_posVec += /*bodies[i]->m_posVec*/  bodies[i]->m_velVec * dt + (temp * dt2b2);
			oldacc[i]=temp;

		}
		resolve(bodies);
		for (int i = 0; i < s; i++)
		{
			bodies[i]->m_velVec += (oldacc[i] + bodies[i]->m_accVec) * dtb2;
		}
		
	}
}



int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	vectorP vector(6, 8);
	vectorP vector2(3, 4);
	vectorP vector4(2, -2);
	vectorP vectorN(0, 0);
	vectorP poso(9, 9);


	auto a = std::make_unique<Body>(1000000000000.0f, 0.1f, vector);
	auto b = std::make_unique<Body>(10.0f, 0.1f, vector2,vector4);
	//auto c = std::make_unique<Body>(100.0f, 0.1f, poso);
	auto c = std::make_unique<Body>(100.0f, 0.1f, vector4);
	auto d = std::make_unique<Body>(1.0f, 0.1f, vectorN);


	std::vector<std::unique_ptr<Body>> bodys;
	bodys.push_back(std::move(a));
	bodys.push_back(std::move(b));
	//bodys.push_back(std::move(c));
	//bodys.push_back(std::move(d));

	std::chrono::duration<double> duration(1000.0f);

	auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

	auto start = clock::now();
	auto end = start + duration;
	auto nextFrame = start;
	int frame = 0;


	std::vector<char> livyur(21,'.');
	std::vector<std::vector<char>> livyud(21, livyur);

	std::vector<vectorP> posOs(bodys.size());

	for (int i = 0; i < bodys.size(); i++)
	{
		posOs[i] = bodys[i]->m_posVec;
	}  //i dont need this becasue of my poor design choices , at start every posOs = 0 and since
	// i check every posOs for every body each posOs is valid because its also checked for 0 wait WTFF,nvm
	// i needed that because what it i dont have a body at 0,0  


	while (clock::now() < end && bodys.size() > 0)
	{
		for (int i = 0; i < bodys.size(); i++)
		{
			vectorP tbpv = bodys[i]->m_posVec.round();  //tbpv = temporary bodies postition vector

			if (tbpv.icap < 0 || tbpv.icap > 20 || tbpv.jcap < 0 || tbpv.jcap > 20)
			{
				//bodys[i]->GetVal();
				tbpv = (0, 0); // AHHH ts so goated as its tbps in 0 its ovec is 0 and since 
				//the coords dont match ovec 0,0 will still be "." ahahhaahah
			}
			
			posOs[i] = tbpv;
		}

		frame++;
		//auto t0 = clock::now();

		physics::move(bodys);
		physics::checkCol(bodys);

		for (int i = 0; i < posOs.size(); i++)
		{
			vectorP Ovec = posOs[i];
			bool booly = false;
			for (int j = 0; j < bodys.size(); j++)
			{
				bool boly = (Ovec == bodys[j]->m_posVec.round());
				booly = boly;
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
		
		
		if (frame % 10 == 0)
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
	
	drawGrid(livyud);
	for (int i = 0; i < bodys.size(); i++)
	{
		bodys[i]->GetVal();
	}
	std::cin.get();
}
