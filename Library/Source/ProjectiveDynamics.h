#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>
#include <iostream>

using FMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using FMatrix3x3 = Eigen::Matrix<float, 3, 3>;
using FVector3 = Eigen::Matrix<float, 3, 1>;
#define vector3(a) block<3,1>(3*(a), 0)

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
    void ThreeVector3ToMatrix3(FMatrix3x3& m, FVector3& v1, FVector3& v2, FVector3& v3)
    {
        m.block<3, 1>(0, 0) = v1;
        m.block<3, 1>(0, 1) = v2;
        m.block<3, 1>(0, 2) = v3;
    }

    TetConstraint(int i0a, int i1a, int i2a, int i3a, FMatrix& x) :
        Constraint(ConstraintType::TETRAHEDRON),
        i0(i0a),
        i1(i1a),
        i2(i2a),
        i3(i3a)
    {
        FVector3 v1 = x.vector3(i0) - x.vector3(i3);
        FVector3 v2 = x.vector3(i1) - x.vector3(i3);
        FVector3 v3 = x.vector3(i2) - x.vector3(i3);

        ThreeVector3ToMatrix3(m_Dr, v1, v2, v3);

        m_Dr_inv = m_Dr.inverse();
    }

    double clamp(double n, double lower, double upper) const
    {
        return std::max(lower, std::min(n, upper));
    }

    void getDeformationGradient(FMatrix3x3& F, const Eigen::Matrix<float, Eigen::Dynamic, 1>& x) const
    {
        // TODO
        FVector3 x1, x2, x3, x4;
        //x1 = x.block_vector(m_p[0]);
        //x2 = x.block_vector(m_p[1]);
        //x3 = x.block_vector(m_p[2]);
        //x4 = x.block_vector(m_p[3]);
        x1 = x.vector3(0);
        x2 = x.vector3(1);
        x3 = x.vector3(2);
        x4 = x.vector3(3);

        FMatrix3x3 Ds;
        Ds(0, 0) = x1.x() - x4.x();     Ds(0, 1) = x2.x() - x4.x();     Ds(0, 2) = x3.x() - x4.x();
        Ds(1, 0) = x1.y() - x4.y();     Ds(1, 1) = x2.y() - x4.y();     Ds(1, 2) = x3.y() - x4.y();
        Ds(2, 0) = x1.z() - x4.z();     Ds(2, 1) = x2.z() - x4.z();     Ds(2, 2) = x3.z() - x4.z();
        //Ds(0,0) = x1.x() - x4.x();	Ds(1,0) = x2.x() - x4.x();	Ds(2,0) = x3.x() - x4.x();
        //Ds(0,1) = x1.y() - x4.y();	Ds(1,1) = x2.y() - x4.y();	Ds(2,1) = x3.y() - x4.y();
        //Ds(0,2) = x1.z() - x4.z();	Ds(1,2) = x2.z() - x4.z();	Ds(2,2) = x3.z() - x4.z();

        F = m_Dr_inv;
        Ds.applyThisOnTheLeft(F);
    }

    void singularValueDecomp(FMatrix3x3& U, Eigen::Matrix<float, 3, 1, 0, 3, 1>& SIGMA, FMatrix3x3& V, const FMatrix3x3& A) const
    {
        Eigen::JacobiSVD<FMatrix3x3> svd;
        svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        U = svd.matrixU();
        V = svd.matrixV();
        SIGMA = svd.singularValues();

        float detU = U.determinant();
        float detV = V.determinant();

        // make sure that both U and V are rotation matrices without reflection
        if (detU < 0)
        {
            U.block<3, 1>(0, 2) *= -1;
            SIGMA[2] *= -1;
        }
        if (detV < 0)
        {
            V.block<3, 1>(0, 2) *= -1;
            SIGMA[2] *= -1;
        }
    }

    void computeVolumePreservingVertexPositions(FMatrix& verticesAfter, const FMatrix& vertices) const
    {
        // debug
        bool tet_did_collide = false;

        // compute the deformation gradient, F, for the tetrahedron's current deformation
        FMatrix3x3 F;
        getDeformationGradient(F, vertices);

        // run SVD to get U, S, and V from the deformation gradient
        FMatrix3x3 U, V;
        FVector3 SIGMA;
        singularValueDecomp(U, SIGMA, V, F);

        // compute determinant of F (the deformation gradient)
        double det_F = F.determinant();

        // prevent inverted tetrahedra
        if (det_F < 0.0) {

            // TODO: maybe check for a negative determinant for F after we clamp the SIGMA values instead of here

            // debug
            tet_did_collide = true;

            // negate smallest value of SIGMA; SIGMA is gauranteed to be sorted in decreasing order
            SIGMA[2] *= -1.0;

            // TODO: the re-sorting below might be unnecessary

            // re-sort SIGMA so the values are in decreasing order
            double high_val = SIGMA[0];
            double mid_val = SIGMA[1];
            double low_val = SIGMA[2];

            // set low_val with certainty
            // swap mid_val and low_val if mid_val < low_val
            if (mid_val < low_val) {
                double temp = low_val;
                low_val = mid_val;
                mid_val = temp;
            }
            // swap high_val and low_val if high_val < low_val
            if (high_val < low_val) {
                double temp = low_val;
                low_val = high_val;
                high_val = temp;
            }

            // sort mid_val and high_val
            // we already know low_val
            if (high_val < mid_val) {
                double temp = mid_val;
                mid_val = high_val;
                high_val = temp;
            }

            SIGMA[0] = high_val;
            SIGMA[1] = mid_val;
            SIGMA[2] = low_val;
        }

        // compute a new S, S*, by clamping the existing s values (stresses) on the main diagonal of S
        double min = 0.95;
        double max = 1.05;
        FMatrix3x3 SIGMA_new;

        SIGMA_new << clamp(SIGMA(0, 0), min, max), 0.0, 0.0,
            0.0, clamp(SIGMA(1, 0), min, max), 0.0,
            0.0, 0.0, clamp(SIGMA(2, 0), min, max);

        // TODO: maybe check for a negative determinant for F here instead of before we clamp the SIGMA values

        // TODO: maybe invert the last column of U to ensure the determinant of F_new is positive b/c I saw that in a paper

        // compute a new deformation gradient, F*, using the existing U and V rotation matrices along with S*
        FMatrix3x3 F_new = V.transpose();
        SIGMA_new.applyThisOnTheLeft(F_new);
        U.applyThisOnTheLeft(F_new);

        // compute deformed basis matrix from F* and rest state basis matrix
        FMatrix3x3 deformed_basis = m_Dr;
        F_new.applyThisOnTheLeft(deformed_basis);

        // use deformed basis matrix to compute new positions of the tetrahedron's vertices
        FVector3 tet_centroid = (vertices.vector3(0) + vertices.vector3(1) + vertices.vector3(2) + vertices.vector3(3)) / 4.0;
        verticesAfter.vector3(3) = tet_centroid - (deformed_basis.col(0) + deformed_basis.col(1) + deformed_basis.col(2)) / 4.0;
        verticesAfter.vector3(0) = verticesAfter.vector3(3) + deformed_basis.col(0);
        verticesAfter.vector3(1) = verticesAfter.vector3(3) + deformed_basis.col(1);
        verticesAfter.vector3(2) = verticesAfter.vector3(3) + deformed_basis.col(2);

        // debug
        //if ( tet_did_collide ) {
        //	std::cout << "\n" << std::endl;
        //	std::cout << "current[0] = ( " << current_vertex_positions.block_vector( 0 )[0] << ", " << current_vertex_positions.block_vector( 0 )[1] << ", " << current_vertex_positions.block_vector( 0 )[2] << " )" << std::endl;
        //	std::cout << "current[1] = ( " << current_vertex_positions.block_vector( 1 )[0] << ", " << current_vertex_positions.block_vector( 1 )[1] << ", " << current_vertex_positions.block_vector( 1 )[2] << " )" << std::endl;
        //	std::cout << "current[2] = ( " << current_vertex_positions.block_vector( 2 )[0] << ", " << current_vertex_positions.block_vector( 2 )[1] << ", " << current_vertex_positions.block_vector( 2 )[2] << " )" << std::endl;
        //	std::cout << "current[3] = ( " << current_vertex_positions.block_vector( 3 )[0] << ", " << current_vertex_positions.block_vector( 3 )[1] << ", " << current_vertex_positions.block_vector( 3 )[2] << " )" << std::endl;

        //	std::cout << "\n" << std::endl;
        //	std::cout << "new[0] = ( " << new_vertex_positions.block_vector( 0 )[0] << ", " << new_vertex_positions.block_vector( 0 )[1] << ", " << new_vertex_positions.block_vector( 0 )[2] << " )" << std::endl;
        //	std::cout << "new[1] = ( " << new_vertex_positions.block_vector( 1 )[0] << ", " << new_vertex_positions.block_vector( 1 )[1] << ", " << new_vertex_positions.block_vector( 1 )[2] << " )" << std::endl;
        //	std::cout << "new[2] = ( " << new_vertex_positions.block_vector( 2 )[0] << ", " << new_vertex_positions.block_vector( 2 )[1] << ", " << new_vertex_positions.block_vector( 2 )[2] << " )" << std::endl;
        //	std::cout << "new[3] = ( " << new_vertex_positions.block_vector( 3 )[0] << ", " << new_vertex_positions.block_vector( 3 )[1] << ", " << new_vertex_positions.block_vector( 3 )[2] << " )" << std::endl;
        //}
    }

    int i0;
    int i1;
    int i2;
    int i3;

    FMatrix3x3 m_Dr;
    FMatrix3x3 m_Dr_inv;
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
