#include "EndBrace.h"

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

};

std::ostream& operator<<(std::ostream& stream, const vectorP& other)
{
	stream << other.icap << "," << other.jcap << "," << other.mag << "," << other.inclineT;
	return stream;
}


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