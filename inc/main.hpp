#include "bv.hpp"
#include<iostream>

const int MAX = 40000;

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

        void Generate(){
            srand (static_cast <unsigned> (time(0)));
            for(int i = 0; i < n; i++){
                lb[i].x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                lb[i].y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub[i].x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub[i].y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ax[i].x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                ax[i].y = sqrt(1 - ax[i].x*ax[i].x);
                bv[i].SetBV(lb[i], ub[i], ax[i]);
            }
        }
        void ShowCollisions(){
            for(int i = 0; i < n - 1; i++){
                if(bv[i].OverlapsWith(bv[i + 1])){}
            }
        }
    };
}