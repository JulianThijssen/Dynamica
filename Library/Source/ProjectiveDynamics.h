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

class TetConstraint : public Constraint
{
public:
    TetConstraint(int i0a, int i1a, int i2a, int i3a) :
        Constraint(ConstraintType::TETRAHEDRON),
        i0(i0a),
        i1(i1a),
        i2(i2a),
        i3(i3a)
    {

    }

    int i0;
    int i1;
    int i2;
    int i3;
};

class Simulation
{
public:
    void init();

    void update();

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
