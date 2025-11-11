#include "EndBrace.h"

double dt = 1 / 60;

namespace constants {
	constexpr double G = 6.67430e-11;
}

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

	void operator+=(const vectorP& other)
	{
		icap += other.icap;
		jcap += other.jcap;
		updateValues();
	}

	void operator-=(const vectorP& other)
	{
		icap -= other.icap;
		jcap -= other.jcap;
		updateValues();
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
	Body(float m, vectorP pos, vectorP vel, vectorP f)
		:m_posVec(pos), m_velVec(vel), m_forVec(f) , m_accVec(0.0f,0.0f)
	{
		if (m <= 0)
			throw std::invalid_argument("Mass be positive");
		m_Mass = m;

		m_accVec = m_forVec /= m_Mass;
	}

	void Move()
	{
		
	}
};


int main()
{
	using namespace std::literals::chrono_literals;

	static vectorP vector(6, 8);
	vectorP vector2(4, 5);

	while (vector.mag < 1000)
	{
		vector += vector2;
		LOG(vector);
		std::this_thread::sleep_for(100ms);
	}
	


	std::cin.get();
}