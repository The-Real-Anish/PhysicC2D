#include "main.hpp"
#include<iostream>
#include<cstdlib>
#include<ctime>
#include<cmath>
#include<time.h>

template<typename T>
void Physicc2D::Generator<T>::Generate(std::string s){
            srand (static_cast <unsigned> (time(0)));
            std::vector<Physicc2D::RigidBody> list;
            std::vector<Physicc2D::RigidBody> *rbList = new std::vector<Physicc2D::RigidBody>;
            for(int i = 0; i < n; i++){
                glm::vec2* lb = new glm::vec2;
                glm::vec2* ub = new glm::vec2;
                glm::vec2* axis = new glm::vec2;
                lb->x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                lb->y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub->x = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                ub->y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                if(s.compare("OBB")){
                    while(ub->y <= lb->y){
                        lb->y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                        ub->y = -100 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(200)));
                    }
                    axis->x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    axis->y = sqrt(1 - axis->x*axis->x);
                    Physicc2D::BoundingVolume::BaseBV<T> *bv = new Physicc2D::BoundingVolume::BaseBV<T>(*lb, *ub, *axis);
                    //bv[i].SetBV(lb[i], ub[i], ax[i]);
                    bv->Correct();
                    Physicc2D::RigidBody *rb = new Physicc2D::RigidBody;
                    rb->setBV<T>(bv);
                    rbList->push_back(*rb);
                    delete bv;
                }
                else if(s.compare("AABB")){
                    Physicc2D::BoundingVolume::BaseBV<T> *bv = new Physicc2D::BoundingVolume::BaseBV<T>(*lb, *ub);
                    //bv[i].SetBV(lb[i], ub[i]);
                    bv->Correct();
                    Physicc2D::RigidBody *rb = new Physicc2D::RigidBody;
                    rb->setBV<T>(bv);
                    rbList->push_back(*rb);
                    delete bv;
                }
                delete lb, ub, axis;
            }
            Physicc2D::BVH<T> *newbvh = new Physicc2D::BVH<T>(*rbList);
            bvh = newbvh;
            bvh->buildmyTree();
			rbList->clear();
            delete rbList;
        }

template<typename T>
int Physicc2D::Generator<T>::ShowCollisions(){
		std::vector<Physicc2D::BroadPhase::potentialContact> collidingBodies =
        Physicc2D::BroadPhase::getPotentialContacts<T>(bvh->returnHead());
        return collidingBodies.size();
	}

int main(){
	Physicc2D::Generator<Physicc2D::BoundingVolume::OBB> gen(1000);
	gen.Generate("OBB");
	clock_t begin = clock();
	std::cout << gen.ShowCollisions() << std::endl;
	//TODO : Implement a randomized OBB generator by using rand() and srand() (done above)
	clock_t end = clock();
	//std::cout << gen.returnCollisions() << std::endl;
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