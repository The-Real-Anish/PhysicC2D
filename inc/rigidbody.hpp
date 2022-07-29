#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include "glm/glm.hpp"
#include "bv.hpp"
#include <memory>
namespace Physicc2D{
    
    //similar to 3D. this class describes the properties of a rigidbody
    /*Currently no collider is implemented as Boxcolliders and OBBs are the same
	and orientation cannot be deduced from Spherecolliders, making the OBBs effectively an AABB*/
	class RigidBody{

		private:
        float m_mass;
		glm::vec2 m_velocity;
		float m_gravityScale;
        glm::vec2 m_force;
		BoundingVolume::AABB enclosingAABB;
		//Once I figure out how to implement more complex colliders I'll make a code for a tight-fitting OBB
		BoundingVolume::OBB enclosingOBB;
        
        public:
			RigidBody(float mass, const glm::vec2& velocity, float gravityScale);
			RigidBody(const RigidBody& other) = default;
			RigidBody(){}

			[[nodiscard]] inline glm::vec2 getCentroid() const{
				return glm::vec2((enclosingAABB.upperbound.x + enclosingAABB.lowerbound.x)/2.0,
								 (enclosingAABB.upperbound.y + enclosingAABB.lowerbound.y)/2.0);
			}
            [[nodiscard]] inline glm::vec2 getVelocity() const{
				return m_velocity;
			}

			inline void setVelocity(const glm::vec2& velocity){
				m_velocity = velocity;
			}

			inline void setGravityScale(const float gravityScale){
				m_gravityScale = gravityScale;

			}
			[[nodiscard]] inline BoundingVolume::AABB getAABB(){
				return enclosingAABB;
			}
			[[nodiscard]] inline BoundingVolume::OBB getOBB(){
				return enclosingOBB;
			} 
			
			inline void setAABB(BoundingVolume::AABB aabb){
				enclosingAABB.Set(aabb.lowerbound, aabb.upperbound);
			}
			
			inline void setAABB(glm::vec2& ub, glm::vec2& lb){
				enclosingAABB.Set(lb, ub);
			}

			inline void setOBB(glm::vec2& ub, glm::vec2& lb, glm::vec2& ax){
				enclosingOBB.Set(lb, ub, ax);
			}

			inline void setOBB(BoundingVolume::OBB obb){
				enclosingOBB.Set(obb.lowerbound, obb.upperbound, obb.axis);
			}

			template<typename T>
			inline void setBV(BoundingVolume::BaseBV<T> *bv){
				if(std::is_same<T, BoundingVolume::AABB>::value) enclosingAABB.Set(bv->volume.lowerbound, bv->volume.upperbound);
				else{
					enclosingOBB.Set(bv->volume.lowerbound, bv->volume.upperbound);
					enclosingOBB.SetAxis(bv->volume.axis);
				}
			}

    };
}

#endif