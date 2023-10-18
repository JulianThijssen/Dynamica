#include "ProjectiveDynamics.h"

//#include <Eigen/Triplet>

Eigen::SparseMatrix<float> CreateSelectorMatrix(const Constraint* c, const State& state)
{
    Eigen::SparseMatrix<float> S_i;
    std::vector<Eigen::Triplet<float, int>> triplets;

    // Selector matrix 6x3n
    if (c->getType() == ConstraintType::SPRING)
    {
        const SpringConstraint* sc = static_cast<const SpringConstraint*>(c);
        triplets.push_back(Eigen::Triplet<float, int>(0, sc->i0 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(1, sc->i0 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(2, sc->i0 * 3 + 2, 1));
        triplets.push_back(Eigen::Triplet<float, int>(3, sc->i1 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(4, sc->i1 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(5, sc->i1 * 3 + 2, 1));

        S_i.resize(6, state.n * state.dim);
        S_i.setFromTriplets(triplets.begin(), triplets.end());
    }
    // Selector matrix 12x3n
    if (c->getType() == ConstraintType::TETRAHEDRON)
    {
        const TetConstraint* tc = static_cast<const TetConstraint*>(c);
        triplets.push_back(Eigen::Triplet<float, int>(0, tc->i0 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(1, tc->i0 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(2, tc->i0 * 3 + 2, 1));

        triplets.push_back(Eigen::Triplet<float, int>(3, tc->i1 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(4, tc->i1 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(5, tc->i1 * 3 + 2, 1));

        triplets.push_back(Eigen::Triplet<float, int>(6, tc->i2 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(7, tc->i2 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(8, tc->i2 * 3 + 2, 1));

        triplets.push_back(Eigen::Triplet<float, int>(9, tc->i3 * 3 + 0, 1));
        triplets.push_back(Eigen::Triplet<float, int>(10, tc->i3 * 3 + 1, 1));
        triplets.push_back(Eigen::Triplet<float, int>(11, tc->i3 * 3 + 2, 1));

        S_i.resize(12, state.n * state.dim);
        S_i.setFromTriplets(triplets.begin(), triplets.end());
    }

    return S_i;
}

Eigen::SparseMatrix<float> springA;
Eigen::SparseMatrix<float> springB;

Eigen::SparseMatrix<float> tetA;
Eigen::SparseMatrix<float> tetB;

void Simulation::init()
{
    state.n = state.q.rows()/3;
    state.dim = 3;

    state.v.resize(state.n * state.dim, 1);
    state.p.resize(2 * state.dim, 1);
    state.fint.resize(state.n * state.dim, 1);
    state.fext.resize(state.n * state.dim, 1);

    sn.resize(state.n * state.dim, 1);

    for (int i = 0; i < state.n * state.dim; i++)
    {
        state.v(i, 0) = 0;
        state.fint(i, 0) = 0;
        state.fext(i, 0) = 0;
    }

    state.M.resize(state.n * state.dim, state.n * state.dim);
    state.M.setIdentity();
    state.Minv.resize(state.n * state.dim, state.n * state.dim);
    state.Minv.setIdentity();

    // Set up A and B matrices
    {
        std::vector<Eigen::Triplet<float, int>> triplets;

        triplets.push_back(Eigen::Triplet<float, int>(0, 0, 0.5));
        triplets.push_back(Eigen::Triplet<float, int>(1, 1, 0.5));
        triplets.push_back(Eigen::Triplet<float, int>(2, 2, 0.5));

        triplets.push_back(Eigen::Triplet<float, int>(0, 3, -0.5));
        triplets.push_back(Eigen::Triplet<float, int>(1, 4, -0.5));
        triplets.push_back(Eigen::Triplet<float, int>(2, 5, -0.5));

        triplets.push_back(Eigen::Triplet<float, int>(3, 0, -0.5));
        triplets.push_back(Eigen::Triplet<float, int>(4, 1, -0.5));
        triplets.push_back(Eigen::Triplet<float, int>(5, 2, -0.5));

        triplets.push_back(Eigen::Triplet<float, int>(3, 3, 0.5));
        triplets.push_back(Eigen::Triplet<float, int>(4, 4, 0.5));
        triplets.push_back(Eigen::Triplet<float, int>(5, 5, 0.5));

        springA.resize(6, 6);
        springA.setFromTriplets(triplets.begin(), triplets.end());

        springB = springA;
    }

    {
        std::vector<Eigen::Triplet<float, int>> triplets;

        float v0 = 2.0f / 3.0f;
        float v1 = -2.0f / 3.0f;

        for (int k = 0; k < 3; k++)
        {
            for (int i = 0; i < 12; i++)
            {
                int j = (k * 4) + (i % 4);
                float v = i == j ? v0 : v1;
                triplets.push_back(Eigen::Triplet<float, int>(i, j, v));
            }
        }

        tetA.resize(12, 12);
        tetA.setFromTriplets(triplets.begin(), triplets.end());

        tetB = tetA;
    }

    // Create cholesky decomposed Y matrix
    FMatrix Y;
    Y = state.M / (dt * dt);

    for (int i = 0; i < constraints.size(); i++)
    {
        Constraint* c = constraints[i];

        Eigen::SparseMatrix<float> S_i = CreateSelectorMatrix(c, state);
        Eigen::SparseMatrix<float> A_i;

        float w_i = 1;

        if (c->getType() == ConstraintType::SPRING)
        {
            A_i = springA;
        }
        if (c->getType() == ConstraintType::TETRAHEDRON)
        {
            A_i = tetA;
        }

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
        Constraint* c = constraints[i];

        Eigen::SparseMatrix<float> S_i = CreateSelectorMatrix(c, state);
        Eigen::SparseMatrix<float> A_i;
        Eigen::SparseMatrix<float> B_i;

        if (c->getType() == ConstraintType::SPRING)
        {
            A_i = springA;
            B_i = springB;
        }
        if (c->getType() == ConstraintType::TETRAHEDRON)
        {
            A_i = tetA;
            B_i = tetB;
        }

        // [3n x 6]
        Eigen::SparseMatrix<float> S_iT = S_i.transpose();
        // Check: Transpose here or not?
        Eigen::SparseMatrix<float> A_iT = A_i.transpose();

        // [3n x 6] * [6 x 6] = [3n x 6]
        S_iT.applyThisOnTheLeft(A_iT);
        // [3n x 6] * [6 x 6] = [3n x 6]
        A_iT.applyThisOnTheLeft(B_i);

        float w_i = 1;

        c->getRHM() = w_i * B_i;
    }
}

void Simulation::update()
{
    //state.v.vector3(0) += Eigen::Matrix<float, 3, 1>(0, -0.04f, 0);
    state.fext.setZero();
    for (unsigned int i = 0; i < state.n; i++)
        state.fext(i * 3 + 1) += -0.1f;
    state.fext = state.M * state.fext;

    // Simulation
    inertia = state.q + state.v * dt;

    sn = inertia + (dt * dt) * (state.Minv) * state.fext;
    qn1 = sn;

    Eigen::SparseMatrix<float> coeff = state.M / (dt * dt);
    coeff.applyThisOnTheLeft(sn);

    // Local step
    for (int i = 0; i < numIterations; i++)
    {
        FMatrix b = sn;

        // Loop over all constraints
        for (const Constraint* c : constraints)
        {
            if (c->getType() == ConstraintType::SPRING)
            {
                const SpringConstraint* sc = static_cast<const SpringConstraint*>(c);
                // Project on constraint set (C_i, qn+1)
                FMatrix v0 = qn1.vector3(sc->i0);
                FMatrix v1 = qn1.vector3(sc->i1);

                FMatrix dir = v1 - v0;
                float restLength = sc->restLength;
                double len = dir.norm();
                float dlen = len - restLength;
                FMatrix displacement = (0.5 * dlen) * (dir / len);

                state.p.resize(6, 1);
                state.p.vector3(0) = v0 + displacement;
                state.p.vector3(1) = v1 - displacement;
            }
            else if (c->getType() == ConstraintType::TETRAHEDRON)
            {
                const TetConstraint* tc = static_cast<const TetConstraint*>(c);

                FMatrix vertices;
                vertices.resize(12, 1);
                vertices.vector3(0) = qn1.vector3(tc->i0);
                vertices.vector3(1) = qn1.vector3(tc->i1);
                vertices.vector3(2) = qn1.vector3(tc->i2);
                vertices.vector3(3) = qn1.vector3(tc->i3);

                FMatrix verticesAfter;
                verticesAfter.resize(12, 1);
                tc->computeVolumePreservingVertexPositions(verticesAfter, vertices);

                state.p.resize(12, 1);
                state.p.vectorX(12, 0) = verticesAfter.vectorX(12, 0);
            }
            // [3n x 6] * [6 x 1] = [3n x 1]
            c->getRHM().applyThisOnTheLeft(state.p);

            b += state.p;
        }
        // Solve linear system (s_n, p_1, p_2, p_3, ...)

        qn1 = lltM.solve(b);
    }

    // Update state
    vn1 = (qn1 - state.q) / dt;
    state.q = qn1;
    state.v = vn1;

    FMatrix penetration = collisionDetection(state.q);
    state.q -= penetration;

    // Damping
    state.v *= (1.0 - 0.01);
}

FMatrix Simulation::collisionDetection(const FMatrix& q)
{
    FMatrix penetration;
    penetration.resize(state.n * state.dim, 1);
    penetration.setZero();
    FVector3 normal;
    double dist;

    FVector3 vn, vt, vel;

    float friction = 0.88f;
    float restitution = 0.4f;

    float floor = -1.5f;

    for (unsigned int i = 0; i < state.n; i++)
    {
        FVector3 xi = q.vector3(i);

        if (xi(1, 0) < floor)
        {
            float dist = fabs(xi(1, 0) - floor);
            normal(0, 0) = 0;
            normal(1, 0) = -1;
            normal(2, 0) = 0;
            penetration.vector3(i) += dist * normal;

            vel = state.v.vector3(i);

            vn = vel.dot(normal) / normal.dot(normal) * normal;

            vt = vel - vn;

            vn = vn * restitution;
            vt = vt * friction;

            vn = vn * -1.0f;

            state.v.vector3(i) = vn + vt;
        }
    }

    return penetration;
}
