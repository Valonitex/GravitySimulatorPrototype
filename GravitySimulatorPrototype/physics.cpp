#include "EndBrace.h"

double dt = (1.0f/720.0f);

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
		//vectorP *temp = new vectorP(icap += other.icap, jcap += other.jcap);

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


	vectorP operator*( double t)
	{
		vectorP *temp = new vectorP(icap * t, jcap * t);
		

		return (*temp);
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

		m_accVec = m_forVec /= m_Mass;
	}


	void Move()
	{
		/*vectorP temp(m_accVec *= dt);
		vectorP temp2(temp *= dt);
		m_posVec = temp2;*/
		vectorP temp(m_accVec * dt);
		m_velVec += temp;

		vectorP temp2(m_velVec * dt);
		m_posVec += temp2;

	}

	void GetVal()
	{
		LOG(m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << "---------");
	}
};

namespace prereq {
	constexpr double G = 6.67430e-11;

	double distance(Body a, Body b)
	{
		vectorP temp = a.m_posVec -= b.m_posVec;
		return temp.mag;
	}
}


int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	 vectorP vector(6, 8);
	 vectorP vector2(3, 4);
	 vectorP vector3(20, 20);

	 Body a(10.0f, vector, vector2, vector3);

	 std::chrono::duration<double> duration(1.0f);
	 //std::chrono::duration<double> dt_dur(1.0f/60.0f);

	 auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

	 auto start = clock::now();
	 auto end = start + duration;
	 auto nextFrame = start;


	 while (clock::now() < end)
	 {

		 auto t0 = clock::now();

		 a.Move();

		// std::this_thread::sleep_for(std::chrono::duration<double>(dt));

		 nextFrame += dt_duration;
		 std::this_thread::sleep_until(nextFrame);


		 auto t1 = clock::now();
		 auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
		 double dt_ms = dt * 1000.0f;
		 std::cout << "work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n";
	 }



	 a.GetVal();

	 


	/*vectorP temp(vector2 * dt);
	vector2.getInfo();

	vector3 += temp;
	vector3.getInfo();


	vector2 * dt;
	vector2.getInfo();

	LOG(vector2 * dt);

	vector2.getInfo();*/


	std::cin.get();
}