#ifndef __MAIN_H__
#define __MAIN_H__

#include "bv.hpp"
#include<iostream>
#include<string>

const int MAX = 1000;

namespace Physicc2D{
    template<typename T>
    class Generator{
    
        private:

        int n;
        //TODO : Implement a randomized OBB generator by using rand() and srand() (done above)
        glm::vec2 lb[MAX], ub[MAX], ax[MAX];
        //Physicc2D::BoundingVolume::OBB rec1(a, b, o1), rec2(c, d, o2);
        Physicc2D::BoundingVolume::BaseBV<T> bv[MAX];

        public:

        Generator(int n1){
            n = n1;
        };

        void Generate(std::string s){
            srand (static_cast <unsigned> (time(0)));
            for(int i = 0; i < n; i++){
                lb[i].x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                lb[i].y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub[i].x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub[i].y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                if(s.compare("OBB")){
                    ax[i].x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    ax[i].y = sqrt(1 - ax[i].x*ax[i].x);
                    bv[i].SetBV(lb[i], ub[i], ax[i]);
                }
                else if(s.compare("AABB")) bv[i].SetBV(lb[i], ub[i]);
            }
        }
        void ShowCollisions(){
            for(int i = 0; i < n; i++){
                /*std::cout << "first lb = (" << lb[i].x << ", " << lb[i].y << ")" << std::endl;
                std::cout << "first ub = (" << ub[i].x << ", " << ub[i].y << ")" << std::endl;
                std::cout << "first axis = (" << ax[i].x << ", " << ax[i].y << ")" << std::endl;
                std::cout << "second lb = (" << lb[i + 1].x << ", " << lb[i + 1].y << ")" << std::endl;
                std::cout << "second ub = (" << ub[i + 1].x << ", " << ub[i + 1].y << ")" << std::endl;
                std::cout << "second axis = (" << ax[i + 1].x << ", " << ax[i + 1].y << ")" << std::endl;*/
                for(int j = 1; j < n; j++)
                if(bv[i].OverlapsWith(bv[j])){} /*std::cout << "They overlap! :)" << std::endl;
                else std::cout << "They don't overlap :(" << std::endl;*/
            }
        }
    };
}

#endif