#include "ProjectiveDynamics.h"

//#include <Eigen/Triplet>

void Simulation::init()
{
    state.n = 3;
    state.dim = 3;

    state.q.resize(state.n * state.dim, 1);
    state.v.resize(state.n * state.dim, 1);
    state.p.resize(2 * state.dim, 1);
    state.fint.resize(state.n * state.dim, 1);
    state.fext.resize(state.n * state.dim, 1);

    sn.resize(state.n * state.dim, 1);

    for (int i = 0; i < state.n * state.dim; i++)
    {
        state.q(i, 0) = 0;
        state.v(i, 0) = 0;
        state.fint(i, 0) = 0;
        state.fext(i, 0) = 0;
    }

    state.q(0, 0) = -1.0f;
    state.q(1, 0) = 1.0f;
    state.q(3, 0) = 0.0f;
    state.q(4, 0) = 0.0f;
    state.q(6, 0) = 1.0f;
    state.q(7, 0) = 1.0f;

    state.M.resize(state.n * state.dim, state.n * state.dim);
    state.M.setIdentity();
    state.Minv.resize(state.n * state.dim, state.n * state.dim);
    state.Minv.setIdentity();

    //qn1 = sn;

    constraints.push_back(Constraint{ 0, 1, 0.5f });
    constraints.push_back(Constraint{ 1, 2, 0.5f });

    // Create cholesky decomposed Y matrix
    FMatrix Y;
    Y = state.M / (dt * dt);

    for (int i = 0; i < constraints.size(); i++)
    {
        Constraint& c = constraints[i];

        std::vector<Eigen::Triplet<float, int>> triplets;
        triplets.push_back(Eigen::Triplet<float, int>(0, c.i0 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(1, c.i0 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(2, c.i0 * 3 + 2, 1));
        triplets.push_back(Eigen::Triplet<float, int>(3, c.i1 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(4, c.i1 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(5, c.i1 * 3 + 2, 1));

        Eigen::SparseMatrix<float> S_i;
        S_i.resize(6, state.n * state.dim);
        S_i.setFromTriplets(triplets.begin(), triplets.end());

        float w_i = 1;

        Eigen::SparseMatrix<float> A_i;
        A_i.resize(6, 6);
        A_i.setIdentity();

        Eigen::SparseMatrix<float> S_iT = S_i.transpose();
        Eigen::SparseMatrix<float> A_iT = A_i.transpose();

        S_iT.applyThisOnTheLeft(A_iT);
        A_iT.applyThisOnTheLeft(A_i);
        A_i.applyThisOnTheLeft(S_i);

        Y += w_i * S_i;
    }
    lltM.compute(Y);

    // Create right hand matrices
    for (int i = 0; i < constraints.size(); i++)
    {
        Constraint& c = constraints[i];

        std::vector<Eigen::Triplet<float, int>> triplets;
        triplets.push_back(Eigen::Triplet<float, int>(0, c.i0 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(1, c.i0 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(2, c.i0 * 3 + 2, 1));
        triplets.push_back(Eigen::Triplet<float, int>(3, c.i1 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(4, c.i1 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(5, c.i1 * 3 + 2, 1));

        // Selector matrix 6x3n
        Eigen::SparseMatrix<float> S_i;
        S_i.resize(6, state.n * state.dim);
        S_i.setFromTriplets(triplets.begin(), triplets.end());

        // [6 x 6]
        Eigen::SparseMatrix<float> A_i;
        A_i.resize(6, 6);
        A_i.setIdentity();

        Eigen::SparseMatrix<float> B_i;
        B_i.resize(6, 6);
        B_i.setIdentity();

        // [3n x 6]
        Eigen::SparseMatrix<float> S_iT = S_i.transpose();
        // Check: Transpose here or not?
        Eigen::SparseMatrix<float> A_iT = A_i.transpose();

        // [3n x 6] * [6 x 6] = [3n x 6]
        S_iT.applyThisOnTheLeft(A_iT);
        // [3n x 6] * [6 x 6] = [3n x 6]
        A_iT.applyThisOnTheLeft(B_i);

        float w_i = 1;

        c.RHM = w_i * B_i;
    }
}

void Simulation::update()
{
    // Simulation
    inertia = state.q + state.v * dt;

    //state.fint = 0;
    //for (int i = 0; i < state.n; i++)
    //{
        // Gradients of 
        //fint += Grad_q(Wi(q));
        // Wi(q) : function from state to non-negative scalar, which is the energy due to i-th constraint
    //}

    sn = inertia + (dt * dt) * (state.Minv) * state.fext;
    qn1 = sn;

    Eigen::SparseMatrix<float> coeff = state.M / (dt * dt);
    coeff.applyThisOnTheLeft(sn);

    // Compute next state
    //float vn = state.v(i, d) + dt * state.Minv(i, i) * (state.fint + state.fext(d));
    //float qn = state.q(i, d) + vn * dt;

    // Local step
    // argmin(p_i) = sum_i [ || A_i S_i q - B_i p_i ||^2 + I_c_i(p_i) ]

    for (int i = 0; i < numIterations; i++)
    {
        FMatrix b = sn;

        // Loop over all constraints
        for (Constraint& c : constraints)
        {
            // Project on constraint set (C_i, qn+1)
            FMatrix v0 = qn1.block<3, 1>(c.i0*3, 0);
            FMatrix v1 = qn1.block<3, 1>(c.i1*3, 0);

            FMatrix dir = v1 - v0;
            float restLength = c.restLength;
            double len = dir.norm();
            float dlen = len - restLength;
            FMatrix displacement = (0.5 * dlen) * (dir / len);

            state.p.block<3, 1>(0, 0) = v0 + displacement;
            state.p.block<3, 1>(3, 0) = v1 - displacement;

            //float constraintEnergy = 0;
            //for (int i = 0; i < 1; i++)
            //{
            //    constraintEnergy += (state.q.row(i) - state.p.row(i)).squaredNorm() + 0;
            //}

            // [3n x 6] * [6 x 1] = [3n x 1]
            c.RHM.applyThisOnTheLeft(state.p);

            b += state.p;
        }
        // Solve linear system (s_n, p_1, p_2, p_3, ...)

            // Global step
        //1.0f / (dt * dt) * M;
        //for (int i = 0; i < 1; i++)
        //{
        //            
        //}
        qn1 = lltM.solve(b);
    }

    // Update state
    vn1 = (qn1 - state.q) / dt;
    state.q = qn1;
    state.v = vn1;

    // Damping
    state.v *= (1.0 - 0.01);
}
