#pragma once
#include <util/CollisionInfo.h>
#include "RigidBody.hpp"

struct CollisionContact {
    RigidBody* a;
    RigidBody* b;
    CollisionInfo collision_info;
    CollisionContact(RigidBody*const& a, RigidBody*const& b, const CollisionInfo& collision_info);
};
