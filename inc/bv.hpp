#ifndef __BV_H__
#define __BV_H__

#include "glm/glm.hpp"
#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec2.hpp"

namespace Physicc2D{
    
    namespace BoundingVolume{
        
        struct AABB{
            glm::vec2 lowerbound;
            glm::vec2 upperbound;
            
            //constructors
            AABB() = default;
            AABB(const AABB& aabb) = default;
            AABB(glm::vec2 LB, glm::vec2 UB) : lowerbound(LB), upperbound(UB) {};
            
            //functions
            inline void Set(glm::vec2& LB, glm::vec2& UB){
                lowerbound = LB;
                upperbound = UB;
            }
            
            
            inline void Set(glm::vec2& LB, glm::vec2& UB, glm::vec2& ax){
                lowerbound = LB;
                upperbound = UB;
            }
            /*This isn't really needed*/

            inline float Area() const{
                return (upperbound.x - lowerbound.x)
				     * (upperbound.y - lowerbound.y);
            }

            inline bool Overlaps(const AABB& aabb){
                 return (this->lowerbound.x <= aabb.upperbound.x
							&& this->upperbound.x >= aabb.lowerbound.x)
						&& (this->lowerbound.y <= aabb.upperbound.y
							&& this->upperbound.y >= aabb.lowerbound.y);
            }

            inline AABB Enclose(const AABB& aabb) const{
				return {glm::min(this->lowerbound, aabb.lowerbound),
						glm::max(this->upperbound, aabb.upperbound)};
			}
        };
        
        struct OBB{
            glm::vec2 lowerbound;
            glm::vec2 upperbound;
            //the axis determines orientation. ||axis|| must always be 1
            glm::vec2 axis;
            //bool is_AABB;
            
            //constructors
            OBB() = default;
            OBB(const OBB& obb) = default;
            OBB(glm::vec2& LB, glm::vec2& UB, glm::vec2& ax)
               //bool AABB)
             : lowerbound(LB), upperbound(UB), axis(ax)
               //is_AABB(AABB)
               {};
            
            //functions
            inline void Set(glm::vec2& LB, glm::vec2& UB, glm::vec2& ax){
                lowerbound = LB;
                upperbound = UB;
                axis = ax;
            }

            //The current template implementation doesn't like it when this function is missing.
            //Ideally it should never be called
            inline void Set(glm::vec2& LB, glm::vec2& UB){
                lowerbound = LB;
                upperbound = UB;
            }

            inline float Area() const{
                return glm::dot(axis, upperbound - lowerbound)
                     * glm::length(glm::cross(glm::vec3(axis.x, axis.y, 0), 
                                              glm::vec3(upperbound.x - lowerbound.x,
                                                        upperbound.y - lowerbound.y,
                                                        0)));
            }

            inline bool Overlaps(OBB& obb){
                const glm::vec2 X(1.f,0.f), Y(0.f, 1.f);
                return 
                //first checks whether the vertices of the passed obb lie inside our obb(this)
                    (inside(obb.lowerbound)
                  || inside(obb.upperbound)
                  || inside(obb.lowerbound.x + glm::dot(obb.upperbound - obb.lowerbound, obb.axis)*glm::dot(obb.axis, X),
                            obb.lowerbound.y + glm::dot(obb.upperbound - obb.lowerbound, obb.axis)*glm::dot(obb.axis, Y))
                  || inside(upperbound.x - glm::dot(obb.upperbound - obb.lowerbound, obb.axis)*glm::dot(obb.axis, X),
                               upperbound.y - glm::dot(obb.upperbound - obb.lowerbound, obb.axis)*glm::dot(obb.axis, Y))
                //then checks whether the vertices of our obb(this) lie inside the passed obb
                  || obb.inside(lowerbound)
                  || obb.inside(upperbound)
                  || obb.inside(lowerbound.x + glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, X),
                                    lowerbound.y + glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, Y))
                  || obb.inside(upperbound.x - glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, X),
                                   upperbound.y - glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, Y)));
            }
            
            //This should never be called 
            inline OBB Enclose(const OBB& obb){
                return *this;
            }
            
            inline void Check(){
                //checks if axis is misaligned
                    if(glm::acos(glm::dot(axis, upperbound - lowerbound)/
                            (glm::length(upperbound - lowerbound))) > 0.7854){
                    //Rotate the axis clockwise
                    if(axis.x < 0) axis = glm::vec2(axis.y, -axis.x);
                    //Rotate the axis counterclockwise
                    else axis = glm::vec2(-axis.y, axis.x);
                }
            }
            //returns whether or not a point is inside an OBB
            inline bool inside(const glm::vec2& a){ 
                //cos = glm::dot(axis, X), sin = glm::dot(axis, Y)
                //(the angle in question is the orientation of the axis)
                
                //a third vertice of the OBB
                //lowerbound.x + (upperbound.x - lowerbound.x)*cos*cos + (upperbound.y - lowerbound.y)*cos*sin,
                //was my original implementation.
                const glm::vec2 X(1.f,0.f), Y(0.f, 1.f);
                glm::vec2 adj(lowerbound.x + glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, X), //(upperbound.x - lowerbound.x)*cos*cos + (upperbound.y - lowerbound.y)*cos*sin,
                              lowerbound.y + glm::dot(upperbound - lowerbound, axis)*glm::dot(axis, Y));
                return (glm::dot(a - adj, lowerbound - adj) > 0
                     && glm::dot(a - adj, lowerbound - adj) < glm::dot(lowerbound - adj, lowerbound - adj)
                     && glm::dot(a - adj, upperbound - adj) > 0
                     && glm::dot(a - adj, upperbound - adj) < glm::dot(upperbound - adj, upperbound - adj));
            }

            inline bool inside(const float& m, const float& n){
                glm::vec2 a(m, n);
                return inside(a);
            }
        };
        
        template <typename T>
        class BaseBV{
            private:
            
            T volume;
            
            public:
            
            //constructors
            BaseBV() = default;
            BaseBV(const BaseBV& bv) = default;
            BaseBV(const glm::vec2& LB, const glm::vec2& UB){
				this->volume = {LB, UB};
			}
            BaseBV(const glm::vec2& LB, const glm::vec2& UB, const glm::vec2& ax){
				this->volume = {LB, UB, ax};
			}

            //functions
            
            inline void SetBV(const T& volume){
                this->volume = volume;
            }
            inline void SetBV(glm::vec2& LB, glm::vec2& UB){
                volume.Set(LB, UB);
            }
            inline void SetBV(glm::vec2& LB, glm::vec2& UB, glm::vec2& ax){
			    volume.Set(LB, UB, ax);
			}

            inline float getArea() const{
                return volume.Area();
            }
            
            //the idea is that only a set of just AABBs or just OBBs will ever be used
            //thus, there is no question of checking if an AABB and OBB intersect
            inline bool OverlapsWith(BaseBV& bv){
                return this->volume.Overlaps(bv.volume);
            }
            //this function should ideally only be called for AABBs.
            inline BaseBV EnclosingBV(const BaseBV& bv){
                return BaseBV(this->volume.Enclose(bv.volume));
            }
            
        };
    }
}
#endif