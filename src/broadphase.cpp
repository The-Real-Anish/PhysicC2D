#include "broadphase.hpp"

namespace Physicc2D{
    
    namespace BroadPhase{

        //implementation of getPotentialContacts hidden by keeping another namespace
        
        namespace BroadPhaseImpl{

			template<typename T>
            inline bool isLeaf(BVHNode<T>* node){
				return (node->left == nullptr) && (node->right == nullptr);
			}

			template<typename T>
            void getPotentialContactsWith(BVHNode<T>* node1, BVHNode<T>* node2,
                                 std::vector<potentialContact>& collisionArray){
				
                if (isLeaf(node1) && isLeaf(node2)){
					//If both nodes are leaves, then we have a PotentialContact, and we add the collision pair to the collisionArray
					collisionArray.push_back(PotentialContact(node1->body, node2->body));
				}
                else if (!isLeaf(node1) && isLeaf(node2)){
					//recurse through both the left and right child of the non-leaf node (i.e. node1)
					getPotentialContactsWith(node1->left.get(), node2, collisionArray);
					getPotentialContactsWith(node1->right.get(), node2, collisionArray);
				}
                else if (isLeaf(node1) && !isLeaf(node2)){
					//If only of the nodes is a leaf, then make sure that node2 is the leaf node and that we recurse through node1
					getPotentialContactsWith(node2, node1, collisionArray);
                    /*TODO: Find out if just copying similar code to the 2nd case above, for this 3rd case,
                    will give a faster or slower performance*/
				}
                else{
					/** Both nodes are non-leaf nodes */

					//Recurse through node1, with node2 constant.
					getPotentialContactsWith(node1->left.get(), node2, collisionArray);
					getPotentialContactsWith(node1->right.get(), node2, collisionArray);

					//Recurse through node2, with node1 constant.
					getPotentialContactsWith(node1, node2->left.get(), collisionArray);
					getPotentialContactsWith(node1, node2->right.get(), collisionArray);
				}
			}

			template<typename T>
            void imp_getPotentialContacts(BVHNode<T>* node, std::vector<potentialContact>& collisionArray){
				
                if (isLeaf(node)) return;
				
				imp_getPotentialContacts(node->left.get(), collisionArray);
				imp_getPotentialContacts(node->right.get(), collisionArray);

				if ((node->left)->volume.OverlapsWith((node->right)->volume)){
					getPotentialContactsWith(node->left.get(), node->right.get(), collisionArray);
				}
			}
        }

		template<typename T>
        std::vector<potentialContact> getPotentialContacts<T>(BVHNode<T>* node){
            std::vector<potentialContact> pc_list;
            BroadPhaseImpl::imp_getPotentialContacts<T>(node, pc_list);
            return pc_list;
        }
    }
}