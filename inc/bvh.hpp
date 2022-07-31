#ifndef __BVH_H__
#define __BVH_H__

#include "bv.hpp"
#include "rigidbody.hpp"
#include <cstring>
#include <utility>
#include <algorithm>
#include <stack>
#include <vector>
#include <memory>

namespace Physicc2D{
    
    template<typename T>
    struct BVHNode{
		T volume;
		std::weak_ptr<RigidBody> body;

		std::shared_ptr<BVHNode> parent;
		std::shared_ptr<BVHNode> left;
		std::shared_ptr<BVHNode> right;
	};

    template<typename T>
    class BVH{

        private:
        
        T computeBV(std::size_t start, std::size_t end){
            BoundingVolume::AABB bv(rigidBodyList[start].getAABB());
        //enclosing AABB for all the rigidbodies is calculated
		for (std::size_t i = start + 1; i != end; i++)
		{
			bv = bv.Enclose(rigidBodyList[i].getAABB());
		}

        if(std::is_same<T, BoundingVolume::OBB>::value){
            BoundingVolume::OBB o_bv;
            o_bv.axis = (bv.upperbound - bv.lowerbound)/glm::length(bv.upperbound - bv.lowerbound);
            o_bv.Check();
            glm::vec2 p_axis;
            if(o_bv.axis.x < 0) p_axis = glm::vec2(o_bv.axis.y, -o_bv.axis.x);
            else p_axis = glm::vec2(-o_bv.axis.y, o_bv.axis.x);
            int ub_max = 0, other_u = 0, other_l = 0, lb_min = 0;
            glm::vec2 center((bv.upperbound.x + bv.lowerbound.x)/2.0, (bv.upperbound.y + bv.lowerbound.y)/2.0);
            
            for (std::size_t i = start + 1; i != end; i++)
            {
                if(glm::dot(o_bv.axis, rigidBodyList[i].getAABB().upperbound) >
                   glm::dot(o_bv.axis, rigidBodyList[ub_max].getAABB().upperbound))
                ub_max = i;
                if(glm::dot(p_axis,
                   glm::vec2(rigidBodyList[i].getAABB().lowerbound.x, rigidBodyList[i].getAABB().upperbound.y))
                   >
                   glm::dot(p_axis,
                   glm::vec2(rigidBodyList[other_u].getAABB().lowerbound.x, rigidBodyList[other_u].getAABB().upperbound.y)))
                other_u = i;
                if(glm::dot(o_bv.axis, rigidBodyList[i].getAABB().lowerbound) <
                   glm::dot(o_bv.axis, rigidBodyList[lb_min].getAABB().lowerbound)) lb_min = i;
                if(glm::dot(p_axis,
                            glm::vec2(rigidBodyList[i].getAABB().upperbound.x, rigidBodyList[i].getAABB().lowerbound.y))
                   >
                   glm::dot(p_axis,
                            glm::vec2(rigidBodyList[other_l].getAABB().upperbound.x, rigidBodyList[other_l].getAABB().lowerbound.y)))
                other_l = i;
            }
            
            o_bv.upperbound = center + (o_bv.axis * glm::dot(o_bv.axis, rigidBodyList[ub_max].getAABB().upperbound - center))
                                     + (p_axis * glm::dot(p_axis,
                                                 glm::vec2(rigidBodyList[other_u].getAABB().lowerbound.x, rigidBodyList[other_u].getAABB().upperbound.y) - center));
            o_bv.lowerbound = center + (o_bv.axis * glm::dot(o_bv.axis, rigidBodyList[lb_min].getAABB().lowerbound - center))
                                     + (p_axis * glm::dot(p_axis,
                                                 glm::vec2(rigidBodyList[other_l].getAABB().upperbound.x, rigidBodyList[other_l].getAABB().lowerbound.y) - center));
            
            T O_bv(o_bv.lowerbound, o_bv.upperbound, o_bv.axis);
            return O_bv;
        }
        else{
            T a_bv(bv.lowerbound, bv.upperbound);
            return a_bv;
        }
        }
        void sort(glm::vec2 axis, std::size_t start, std::size_t end){
            //if we are using OBBs, use the axis of the parent OBB
        if(std::is_same<T, BoundingVolume::OBB>::value){
            std::sort(std::next(rigidBodyList.begin(), start),
			 	      std::next(rigidBodyList.begin(), end + 1),
					  [&axis](const RigidBody& rigidBody1, const RigidBody& rigidBody2) {
					      return glm::dot(axis, rigidBody1.getCentroid())
				     	       > glm::dot(axis, rigidBody2.getCentroid());
					  });
        }
        //if we are using AABBs and the x-axis
        else if(axis.y == 0){
            std::sort(std::next(rigidBodyList.begin(), start),
					  std::next(rigidBodyList.begin(), end + 1),
					  [](const RigidBody& rigidBody1, const RigidBody& rigidBody2) {
						 return rigidBody1.getCentroid().x
							  > rigidBody2.getCentroid().x;
					});
        }
        //if we are using AABBs and the y-axis
        else{
            std::sort(std::next(rigidBodyList.begin(), start),
					  std::next(rigidBodyList.begin(), end + 1),
					  [](const RigidBody& rigidBody1, const RigidBody& rigidBody2) {
						 return rigidBody1.getCentroid().y
							  > rigidBody2.getCentroid().y;
					});
        }
        }

        glm::vec2 cuttingAxis(std::size_t start, std::size_t end){
            //As this function calls the computeBV function, we do not need to directly call
        //computeBV in our buildTree function
        BoundingVolume::AABB bv = rigidBodyList[start].getAABB();
        for (std::size_t i = start + 1; i != end; i++)
		{
			bv = bv.Enclose(rigidBodyList[i].getAABB());
		}
        //if an AABB, then the axis must be either x-axis or y-axis
        if(std::is_same<T, BoundingVolume::AABB>::value){
            //if spread is greater along x-axis, return x-axis
            if(bv.upperbound.x - bv.lowerbound.x > bv.upperbound.y - bv.lowerbound.y) return glm::vec2(1.f, 0.f);
            //else return y-axis
            else return glm::vec2(0.f, 1.f);
        }
        //if an OBB, we use the axis of the parent OBB as our cutting axis
        //else return bv.axis;
        else return (bv.upperbound - bv.lowerbound)/glm::length(bv.upperbound - bv.lowerbound);
        }

        void buildTree(std::shared_ptr<BVHNode<T>> node, std::size_t start, std::size_t end){
            if(start == end){
            node->volume.upperbound = rigidBodyList[start].getAABB().upperbound;
            node->volume.lowerbound = rigidBodyList[start].getAABB().lowerbound;
            node->body = std::make_shared<RigidBody>(rigidBodyList[start]);
            node->left = nullptr;
            node->right = nullptr;
        }
        else{
       
        node->volume = computeBV(start, end);
        
        sort(cuttingAxis(start, end), start, end);

        auto leftNode = std::make_shared<BVHNode<T>>();
		auto rightNode = std::make_shared<BVHNode<T>>();
        
        node->left = leftNode;
		node->right = rightNode;

		leftNode->parent = node;
		rightNode->parent = node;

        //Cutting Axis(CA) is currently median CA
        //TODO: Implement mean CA and compare results with median CA
		buildTree(leftNode, start, start + (end - start) / 2);
		buildTree(rightNode, start + 1 + (end - start) / 2, end);
        }
        }
        
        public:

        std::shared_ptr<BVHNode<T>> head;
        std::vector<RigidBody> rigidBodyList;

        BVH(std::vector<Physicc2D::RigidBody> rb_list) : rigidBodyList(rb_list){
            head = std::make_shared<BVHNode<T>>();
        }
        
        BVHNode<T> returnHead(){
            return *head;
        }
        //makes a binary tree of BVs
        void buildmyTree(){
            buildTree(head, 0, rigidBodyList.size() - 1);
        }

        //adds the elements to a vector, converting the tree
        //to a linear data structure
        std::vector<std::weak_ptr<RigidBody>> convert(){
            std::stack<BVHNode<T>*> s;
		std::vector<std::weak_ptr<RigidBody>> tree;
		BVHNode<T>* currentNode = head.get();
		while (!s.empty() || currentNode != nullptr)
		{

			while (currentNode != nullptr)
			{
				s.push(currentNode);
				currentNode = currentNode->left.get();
			}

			currentNode = s.top();
			s.pop();

			if (currentNode->left == nullptr && currentNode->right == nullptr)
			{
				tree.push_back(currentNode->body);
			}
			currentNode = currentNode->right.get();
		}
		return tree;
        }
    };
}

#endif
