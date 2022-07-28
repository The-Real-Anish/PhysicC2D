#ifndef __BVH_H__
#define __BVH_H__

#include "bv.hpp"
#include "rigidbody.hpp"
#include <cstring>
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
        std::shared_ptr<BVHNode<T>> head;
        std::vector<RigidBody> rigidBodyList;
        T computeBV(std::size_t start, std::size_t end);
        void sort(glm::vec2& axis, std::size_t start, std::size_t end);
        glm::vec2 cuttingAxis(std::size_t start, std::size_t end);
        void buildTree(std::shared_ptr<BVHNode<T>> node, std::size_t start, std::size_t end);
        
        public:

        BVH(std::vector<Physicc2D::RigidBody> rb_list) : rigidBodyList(rb_list){
            head = std::make_shared<BVHNode<T>>();
        }
        
        BVHNode<T> returnHead(){
            return *head;
        }
        //makes a binary tree of BVs
        inline void buildTree(){
			buildTree(head, 0, rigidBodyList.size() - 1);
		}

        //adds the elements to a vector, converting the tree
        //to a linear data structure
        std::vector<std::weak_ptr<RigidBody>> convert();
    };
}

#endif
