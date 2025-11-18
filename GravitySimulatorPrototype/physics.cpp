#include "EndBrace.h"

double dt = (1.0f / 360.0f);

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

	vectorP negate()
	{
		return vectorP(-(icap), -(jcap));
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
	vectorP m_posVec;
	vectorP m_velVec;
	vectorP m_accVec;
	vectorP m_forVec;
	vectorP m_forRes;
	double m_radius;
	double m_Mass;

public:
	Body(double m,double r, vectorP pos = { 0,0 }, vectorP vel = { 0,0 }, vectorP f = { 0,0 })
		:m_posVec(pos), m_velVec(vel), m_forVec(f), m_accVec(0.0f, 0.0f)
	{
		if (m <= 0 || r<=0)
			throw std::invalid_argument("Mass/Radius be positive");
		m_Mass = m;
		m_radius = r;

		m_accVec = m_forVec / m_Mass;
	}

	void updateVal(/*vectorP force*/)
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
		double eps = 1e-2;

		double distSq = disp.magSq() + eps * eps;
		double dist = sqrt(distSq);
		double denom = distSq * dist;

		vectorP pullvec = (disp) * ((physics::G * a.m_Mass * b.m_Mass) / denom);

		a.forsum(pullvec.negate());
		a.updateVal();
		b.forsum(pullvec);
		b.updateVal();
		//LOG(pullvec);
	}
	bool checkCol(const Body& a, const Body& b)
	{
		if (displacement(a, b).mag() <= a.m_radius + b.m_radius)
		{
			LOG("poof");
			return false;
		}
		return true;
	}
	void resolve(std::vector<std::reference_wrapper<Body>>& bodies)
	{
		for (int i = 0; i < (bodies.size()-1); i++)
		{
			for (int j = i + 1; j < bodies.size() ; j++)
			{
				physics::pull(bodies[i], bodies[j]);
			}
		}
	}
}



int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	vectorP vector(6, 8);
	vectorP vector2(3, 4);
	vectorP vector4(20, 20);

	Body a(1000000000000.0f,0.1f, vector);
	Body b(10.0f,0.1f, vector2);
	Body c(100.0f, 0.1f, vector4);

	std::vector<std::reference_wrapper<Body>> bodys = { a,b,c };

	std::chrono::duration<double> duration(1.0f);

	auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

	auto start = clock::now();
	auto end = start + duration;
	auto nextFrame = start;


	while (clock::now() < end)
	{
		//auto t0 = clock::now();

		//a.GetVal();
		b.GetVal();
		c.GetVal();

		//physics::pull(a, b);

		physics::resolve(bodys);
		
		a.Move();
		b.Move();

		if (physics::checkCol(a, b) == false)
			break;

		/*auto t1 = clock::now();
		auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
		double dt_ms = dt * 1000000.0f;
		std::cout << "work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n";*/


		nextFrame += dt_duration;
		std::this_thread::sleep_until(nextFrame);
	}

	/*physics::pull(a, b);

	LOG(b.m_forVec <<"\n"<< b.m_accVec);
	LOG(a.m_forVec << "\n" << a.m_accVec);*/

	//a.GetVal();
	b.GetVal();

	std::cin.get();
}