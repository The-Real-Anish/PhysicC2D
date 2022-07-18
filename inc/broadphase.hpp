#ifndef __BROADPHASE_H__
#define __BROADPHASE_H__

#include "rigidbody.hpp"
#include "bvh.hpp"
#include "core/assert.hpp"
#include <memory>

namespace Physicc2D{

    namespace BroadPhase{

        struct potentialContact{

            //Will store a pair of rigidbodies that may be colliding

        potentialContact(std::weak_ptr<RigidBody> body1, std::weak_ptr<RigidBody> body2): rb1(body1), rb2(body2)
		{}

		std::weak_ptr<RigidBody> rb1, rb2;
        }

        

    }
}

#endif