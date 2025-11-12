#include "EndBrace.h"

float dt = 1/60;

class vectorP
{
public:
	float icap;
	float jcap;
	float mag;
	float inclineT;
	float inclineC;

public:
	vectorP(float i , float j)
		:icap(i) , jcap(j)
	{

		mag = float(pow((pow(icap, 2) + pow(jcap, 2)), 0.5));
		inclineT = (icap != 0) ? (jcap / icap) : 0;
		inclineC = (icap != 0) ? (icap / mag) : 0;

	}
	void getInfo()
	{
		LOG(icap<< "\n" << jcap<<"\n" << mag<<"\n" << inclineT<<"\n");
	}
	void updateValues()
	{
		mag = float(pow((pow(icap, 2) + pow(jcap, 2)), 0.5));
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


	vectorP operator*=(const float t)
	{
		icap = icap * t;
		jcap = jcap * t;
		updateValues();

		return (*this);
	}

	vectorP operator/=(const float t)
	{
		icap = icap / t;
		jcap = jcap / t;
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
	float m_Mass;

public:
	Body(float m, vectorP pos= {0,0}, vectorP vel = {0,0}, vectorP f = {0,0})
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

		this->m_velVec += (this->m_accVec *= 1/60);
		this->m_posVec += (this->m_velVec *= 1/60);
	}

	void GetVal()
	{
		LOG(m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << "---------");
	}
};

namespace prereq {
	constexpr double G = 6.67430e-11;

	float distance(Body a, Body b)
	{
		vectorP temp = a.m_posVec -= b.m_posVec;
		return temp.mag;
	}
}


int main()
{
	using namespace std::literals::chrono_literals;

	static vectorP vector(6, 8);
	static vectorP vector2(3, 4);
	static vectorP vector3(20, 20);

	static Body a(10.0f, vector, vector2, vector3);

	while (true)
	{

		a.GetVal();
		a.Move();

		std::this_thread::sleep_for(1s);
	}
	


	std::cin.get();
}