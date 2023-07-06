#pragma once

#include "Constraint.h"

class TetConstraint : public Constraint
{
public:
    TetConstraint(int i0a, int i1a, int i2a, int i3a, FMatrix& x);

    void getDeformationGradient(FMatrix3x3& F, const FVectorX& x) const;

    void singularValueDecomp(FMatrix3x3& U, FVector3& SIGMA, FMatrix3x3& V, const FMatrix3x3& A) const;

    void computeVolumePreservingVertexPositions(FMatrix& verticesAfter, const FMatrix& vertices) const;

    int i0;
    int i1;
    int i2;
    int i3;

    FMatrix3x3 m_Dr;
    FMatrix3x3 m_Dr_inv;
};
