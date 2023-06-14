#include "ProjectiveDynamics.h"

void Simulation::init()
{
    state.n = 2;
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
    state.q(3, 0) = 1.0f;

    state.M.resize(state.n * state.dim, state.n * state.dim);
    state.M.setIdentity();
    state.Minv.resize(state.n * state.dim, state.n * state.dim);
    state.Minv.setIdentity();

    //qn1 = sn;

    constraints.push_back(Constraint{ 0, 1, 1.0f });

    // Create cholesky decomposed Y matrix
    FMatrix Y;
    Y = state.M / (dt * dt);

    for (int i = 0; i < constraints.size(); i++)
    {
        Constraint& c = constraints[i];

        Eigen::SparseMatrix<float> S_i;
        Eigen::SparseMatrix<float> A_i;
        float w_i;

        w_i = 1;
        A_i.resize(6, 6);
        A_i.setIdentity();
        S_i.resize(6, 6);
        S_i.setIdentity();

        Eigen::SparseMatrix<float> S_iT = S_i.transpose();
        Eigen::SparseMatrix<float> A_iT = A_i.transpose();

        S_iT.applyThisOnTheLeft(A_iT);
        A_iT.applyThisOnTheLeft(A_i);
        A_i.applyThisOnTheLeft(S_i);

        Y += w_i * S_i;
    }
    lltM.compute(Y);
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

            b.block<6, 1>(0, 0) += state.p;
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
