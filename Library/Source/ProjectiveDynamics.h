#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>

using FMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

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

class Constraint
{
public:
    int i0;
    int i1;
    float restLength;

    // [3n x 6]
    Eigen::SparseMatrix<float> RHM;
};

class Simulation
{
public:
    void init();

    void update();

    State state;

    std::vector<Constraint> constraints;

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
