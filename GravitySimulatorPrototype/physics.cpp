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
		inclineT = float(jcap / icap);
		inclineC = float(icap / mag);

	}
	void getInfo()
	{
		LOG(icap<< "\n" << jcap<<"\n" << mag<<"\n" << inclineT<<"\n");
	}

};

vectorP resol(vectorP& one, vectorP& other)
{
	 vectorP nuy( one.icap + other.icap, one.jcap + other.jcap);
	 return nuy;
}


int main()
{
	vectorP vector(6, 8);
	vectorP vector2(-6, -8);

	vectorP nayi = resol(vector , vector2);
	nayi.getInfo();


	std::cin.get();
}