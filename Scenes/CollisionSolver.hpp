#pragma once
#include <vector>
#include "CollisionContact.hpp"

class CollisionSolver {
public:
    void resolveContacts(std::vector<CollisionContact>& contacts, float duration);
protected:
    virtual void resolveContact(const CollisionContact& contact, float duration) = 0;
};

class ImpulseSolver : public CollisionSolver {
private:
    void resolveContact(const CollisionContact& contacts, float duration) override;
};
