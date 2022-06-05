#include "bv.hpp"
#include "rigidbody.hpp"
#include "vector"
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
        std::vector<RigidBody> Rigidbodylist;
        enum Axis {X, Y, Z,};

        T computeBV(std::size_t start, std::size_t end);
        void Buildtree(std::shared_ptr<BVHNode<T>> node, std::size_t start, std::size_t end);
		void sort(Axis axis, std::size_t start, std::size_t end);
		Axis Getcuttingaxis(std::size_t start, std::size_t end);

        public:
        BVH(std::vector<RigidBody> rb_list) : Rigidbodylist(rb_list){
            head = std::make_shared<BVHNode>;
        }
        
        //makes a binary tree of BVs
        inline void Buildtree(){
			Buildtree(head, 0, Rigidbodylist.size() - 1);
		}

        //adds the elements to a vector, converting the tree
        //to a linear data structure
        std::vector<std::weak_ptr<RigidBody>> convert();
    };
}


