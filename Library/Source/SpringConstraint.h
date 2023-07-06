#pragma once

#include "Constraint.h"

class SpringConstraint : public Constraint
{
public:
    SpringConstraint(int i0a, int i1a, float restLengtha) :
        Constraint(ConstraintType::SPRING),
        i0(i0a),
        i1(i1a),
        restLength(restLengtha)
    {

    }

    int i0;
    int i1;

    float restLength;
};
