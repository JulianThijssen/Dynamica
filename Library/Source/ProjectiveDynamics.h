#pragma once

#include "Types.h"
#include "SpringConstraint.h"
#include "TetConstraint.h"

#include <vector>
#include <iostream>

class State
{
public:
    int n;
    int dim;

    // Matrix of particle positions (n x 3)
    FMatrix q;
    // Matrix of particle velocities (n x 3)
    FMatrix v;

    FMatrix p;

    Eigen::SparseMatrix<float> M;
    Eigen::SparseMatrix<float> Minv;

    // Vector of external forces
    //std::vector<float> fext;
    FMatrix fint;
    
    FMatrix fext;
};

class Simulation
{
public:
    void init();

    void update();

    FMatrix collisionDetection(const FMatrix& q);

    State state;

    std::vector<Constraint*> constraints;

private:
    //FMatrix S;
    FMatrix sn;
    FMatrix qn1;
    FMatrix vn1;

    FMatrix inertia;

    Eigen::LLT<FMatrix> lltM;



    int numIterations = 10;

    double t = 0;
    const double dt = 0.016;
};
