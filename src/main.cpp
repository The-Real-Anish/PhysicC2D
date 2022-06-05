#include "main.hpp"
#include<iostream>
#include<cstdlib>
#include<ctime>
#include<cmath>
#include<time.h>
int main()
{
	clock_t begin = clock();
	Physicc2D::Generator<Physicc2D::BoundingVolume::OBB> gen(40000);
	gen.Generate();
	gen.ShowCollisions();
	//TODO : Implement a randomized OBB generator by using rand() and srand() (done above)
	clock_t end = clock();
	std::cout << "Execution time: " << (double)(end - begin)/CLOCKS_PER_SEC << 's' << std::endl;
	return 0;
	//bv1.SetBV(rec1);
	//bv2.SetBV(rec2);
	//if(bv1.OverlapsWith(bv2)) std::cout << "They overlap!" << std::endl;
	//else std::cout << "They don't overlap..." << std::endl;
	//std::cout << "The area of the first BV is " << bv1.getArea() << std::endl;
	//std::cout << "The area of the second BV is " << bv2.getArea() << std::endl;
	//return 0;
}