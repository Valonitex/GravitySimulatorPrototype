#include "EndBrace.h"

double dt = (1.0f/120.0f);

class vectorP
{
public:
	double icap;
	double jcap;
	double mag;
	double inclineT;
	double inclineC;

public:
	vectorP(double i , double j)
		:icap(i) , jcap(j)
	{

		mag = double(pow((pow(icap, 2) + pow(jcap, 2)), 0.5));
		inclineT = (icap != 0) ? (jcap / icap) : 0;
		inclineC = (icap != 0) ? (icap / mag) : 0;

	}
	void getInfo()
	{
		LOG(icap<< "\n" << jcap<<"\n" << mag<<"\n" << inclineT<<"\n");
	}
	void updateValues()
	{
		mag = double(pow((pow(icap, 2) + pow(jcap, 2)), 0.5));
		inclineT = (icap != 0) ? (jcap / icap) : 0;
		inclineC = (icap != 0) ? (icap / mag) : 0;
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

	vectorP operator*( double t)
	{
		return (vectorP(icap * t, jcap * t));
	}

	vectorP operator/(double t)
	{
		return (vectorP(icap / t, jcap / t));
	}

	vectorP operator/=( double t)
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
	stream << other.icap << "," << other.jcap << "," << other.mag << "," << other.inclineT;
	return stream;
}

class Body
{
public:
	vectorP m_posVec;
	vectorP m_velVec;
	vectorP m_accVec;
	vectorP m_forVec;
	double m_Mass;

public:
	Body(double m, vectorP pos= {0,0}, vectorP vel = {0,0}, vectorP f = {0,0})
		:m_posVec(pos), m_velVec(vel), m_forVec(f) , m_accVec(0.0f,0.0f)
	{
		if (m <= 0)
			throw std::invalid_argument("Mass be positive");
		m_Mass = m;

		m_accVec = m_forVec / m_Mass;
	}

	void updateVal( vectorP force)
	{
		m_forVec = force;
		m_accVec = force / m_Mass;
	}

	void Move()
	{
		vectorP temp(m_accVec * dt);
		m_velVec += temp;

		vectorP temp2(m_velVec * dt);
		m_posVec += temp2;

	}

	void GetVal()
	{
		LOG(m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << m_forVec <<"\n"<< "---------");
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
		vectorP pullvec = (disp / disp.mag) * ((physics::G * a.m_Mass * b.m_Mass) / (disp.mag * disp.mag));

		a.updateVal(pullvec.negate());

		b.updateVal(pullvec);

		//LOG(pullvec);
	}
}



int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	 vectorP vector(6, 8);
	 vectorP vector2(3, 4);
	 vectorP vector3(20, 20);

	 Body a(1000000000000.0f,vector);
	 Body b(10.0f,vector2);

	 

	 std::chrono::duration<double> duration(1.0f);

	 auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

	 auto start = clock::now();
	 auto end = start + duration;
	 auto nextFrame = start;


	 while (clock::now() < end)
	 {

		 /*a.GetVal();
		 b.GetVal();*/

		 //auto t0 = clock::now();
		 physics::pull(a, b);
		 a.Move();
		 b.Move();
 
		 nextFrame += dt_duration;
		 std::this_thread::sleep_until(nextFrame);


		 /*auto t1 = clock::now();
		 auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
		 double dt_ms = dt * 1000.0f;
		 std::cout << "work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n";*/
	 } 

	 /*physics::pull(a, b);

	 LOG(b.m_forVec <<"\n"<< b.m_accVec);
	 LOG(a.m_forVec << "\n" << a.m_accVec);*/

	 a.GetVal();
	 b.GetVal();

	std::cin.get();
}