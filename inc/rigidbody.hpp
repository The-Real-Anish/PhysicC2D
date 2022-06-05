#include "glm/glm.hpp"

namespace Physicc2D{
    
    //similar to 3D. this class describes the properties of a rigidbody
    //as collider
    class RigidBody{

		private:
        glm::vec2 m_position;
        float m_mass;
		glm::vec2 m_velocity;
		float m_gravityScale;
        glm::vec2 m_force;
        
        public:
			RigidBody(float mass, const glm::vec2& velocity, float gravityScale);
			RigidBody(const RigidBody& other) = default;

			[[nodiscard]] inline glm::vec2 getPosition() const{
				return m_position;
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
            inline void setPosition(const glm::vec2& position){
                m_position = position;
            }
    };
}