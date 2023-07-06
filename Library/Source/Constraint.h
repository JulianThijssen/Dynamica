#pragma once

#include "Types.h"

enum class ConstraintType
{
    SPRING, TETRAHEDRON
};

class Constraint
{
public:
    Constraint(ConstraintType type) :
        _type(type)
    {}

    ConstraintType getType() const
    {
        return _type;
    }

    Eigen::SparseMatrix<float>& getRHM()
    {
        return RHM;
    }

    const Eigen::SparseMatrix<float>& getRHM() const
    {
        return RHM;
    }

protected:
    ConstraintType _type;

    // [3n x 6]
    Eigen::SparseMatrix<float> RHM;
};
