#include "CollisionContact.h"

CollisionContact::CollisionContact(RigidBody *const& a, RigidBody *const& b, const CollisionInfo &collision_info): a{a}, b{b}, collision_info{collision_info}{}
