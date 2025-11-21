#include "physics.hpp"
#include <iostream>

int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	vectorP vector(6, 8);
	vectorP vector2(3, 4);
	vectorP vector4(20, 20);
	vectorP vectorN(0, 0);


	auto a = std::make_unique<Body>(1000000000000.0f, 0.1f, vector);
	auto b = std::make_unique<Body>(10.0f, 0.1f, vector2);
	auto c = std::make_unique<Body>(100.0f, 0.1f, vector4);
	auto d = std::make_unique<Body>(1.0f, 0.1f, vectorN);


	std::vector<std::unique_ptr<Body>> bodys;
	bodys.push_back(std::move(a));
	bodys.push_back(std::move(b));
	bodys.push_back(std::move(c));
	bodys.push_back(std::move(d));

	std::chrono::duration<double> duration(5.0f);

	auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

	auto start = clock::now();
	auto end = start + duration;
	auto nextFrame = start;


	while (clock::now() < end && bodys.size() > 0)
	{
		//auto t0 = clock::now();

		/*bodys[0]->GetVal();
		bodys[1]->GetVal();
		bodys[2]->GetVal();
		bodys[3]->GetVal();*/

		physics::move(bodys);
		physics::checkCol(bodys);


		/*auto t1 = clock::now();
		auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
		double dt_ms = dt * 1000000.0f;
		std::cout << "work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n";*/


		nextFrame += dt_duration;
		std::this_thread::sleep_until(nextFrame);


		for (int i = 0; i < bodys.size(); i++)
		{
			bodys[i]->GetVal();
		}
	}

	for (int i = 0; i < bodys.size(); i++)
	{
		bodys[i]->GetVal();
	}


	std::cin.get();
}