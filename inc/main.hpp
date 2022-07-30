#ifndef __MAIN_H__
#define __MAIN_H__

#include "bvh.hpp"
#include "broadphase.hpp"
#include<iostream>
#include<vector>
#include<string>
#include<cstring>

namespace Physicc2D{
    template<typename T>
    class Generator{
    
        private:

        int n;
        //TODO : Implement a randomized OBB generator by using rand() and srand() (done above)
        //glm::vec2 lb[MAX], ub[MAX], ax[MAX];
        //Physicc2D::BoundingVolume::OBB rec1(a, b, o1), rec2(c, d, o2);
        //Physicc2D::BoundingVolume::BaseBV<T> bv[MAX];
        //Physicc2D::BVH<T>* bvh = NULL;
        Physicc2D::BVH<T> *bvh;

        public:
        Generator(int n1){
            n = n1;
        };

        void Generate(std::string s);
        /*void Generate(std::string s){
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
            bvh->buildTree();
            delete rbList;
        }*/
        int ShowCollisions();
        /*int ShowCollisions(){
            std::vector<Physicc2D::BroadPhase::potentialContact> collidingBodies =
            Physicc2D::BroadPhase::getPotentialContacts<T>(bvh->returnHead());
            return collidingBodies.size();
            /*new int a = 0;
            for(int i = 0; i < n; i++){
                //std::cout << "first lb = (" << lb[i].x << ", " << lb[i].y << ")" << std::endl;
                //std::cout << "first ub = (" << ub[i].x << ", " << ub[i].y << ")" << std::endl;
                //std::cout << "first axis = (" << ax[i].x << ", " << ax[i].y << ")" << std::endl;
                //std::cout << "second lb = (" << lb[i + 1].x << ", " << lb[i + 1].y << ")" << std::endl;
                //std::cout << "second ub = (" << ub[i + 1].x << ", " << ub[i + 1].y << ")" << std::endl;
                //std::cout << "second axis = (" << ax[i + 1].x << ", " << ax[i + 1].y << ")" << std::endl;
                for(int j = 1; j < n; j++)
                if(bv[i].OverlapsWith(bv[j])){
                    a++;
                } std::cout << "They overlap! :)" << std::endl;
                else std::cout << "They don't overlap :(" << std::endl;
            //}
        }*/

        //int returnCollisions(){
          //  return a/2;
        //}
    };
}

#endif