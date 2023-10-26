////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2023 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.16.1
//
////////////////////////////////////////////////////////////////////////////////

#ifndef EXAMPLES_GLFW_SPHERE_CMATRIXGL_H
#define EXAMPLES_GLFW_SPHERE_CMATRIXGL_H

// Eigen library headers
#include "Eigen/Eigen"

////////////////////////////////////////////////////////////////////////////////
///
/// Matrix class used to express position or translation.
/// On the OpenGL side 4x4 matrices are required to perform all geometric
/// transformations. cMatrixGL provides a structure which encapsulates all the
/// necessary functionalities to generate 4x4 OpenGL transformation matrices by
/// passing 3D position vectors and rotation matrices. cMatrixGL also provides
/// OpenGL calls to push, multiply and pop matrices off the OpenGL stack.
/// OpenGL Matrices are COLUMN major.
///
////////////////////////////////////////////////////////////////////////////////

struct cMatrixGL
{
 private:

    /// Transformation matrix
    double m_matrix[4][4];

 public:

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method returns a pointer to the matrix array in memory.
    ///
    /// \return
    /// A pointer to the first element of the 2 dimensional matrix array.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    const double* get() const
    {
        return m_matrix[0];
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method creates an OpenGL transformation matrix from a position vector.
    ///
    /// \param a_position
    /// A vector containing the transform position
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void set(const Eigen::Vector3d& a_position)
    {
        m_matrix[0][0] = 1.0;
        m_matrix[1][0] = 0.0;
        m_matrix[2][0] = 0.0;
        m_matrix[3][0] = a_position(0);

        m_matrix[0][1] = 0.0;
        m_matrix[1][1] = 1.0;
        m_matrix[2][1] = 0.0;
        m_matrix[3][1] = a_position(1);

        m_matrix[0][2] = 0.0;
        m_matrix[1][2] = 0.0;
        m_matrix[2][2] = 1.0;
        m_matrix[3][2] = a_position(2);

        m_matrix[0][3] = 0.0;
        m_matrix[1][3] = 0.0;
        m_matrix[2][3] = 0.0;
        m_matrix[3][3] = 1.0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method creates an OpenGL transformation matrix from a rotation matrix
    /// and a position vector.
    ///
    /// \param a_position
    /// A vector containing the transform position
    ///
    /// \param a_rotation
    /// A vector containing the transform rotation
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void set(const Eigen::Vector3d& a_position,
             const double a_rotation[3][3])
    {
        m_matrix[0][0] = a_rotation[0][0];
        m_matrix[1][0] = a_rotation[0][1];
        m_matrix[2][0] = a_rotation[0][2];
        m_matrix[3][0] = a_position(0);

        m_matrix[0][1] = a_rotation[1][0];
        m_matrix[1][1] = a_rotation[1][1];
        m_matrix[2][1] = a_rotation[1][2];
        m_matrix[3][1] = a_position(1);

        m_matrix[0][2] = a_rotation[2][0];
        m_matrix[1][2] = a_rotation[2][1];
        m_matrix[2][2] = a_rotation[2][2];
        m_matrix[3][2] = a_position(2);

        m_matrix[0][3] = 0.0;
        m_matrix[1][3] = 0.0;
        m_matrix[2][3] = 0.0;
        m_matrix[3][3] = 1.0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method creates an OpenGL transformation matrix from an Eigen rotation
    /// matrix and an Eigen translation vector.
    ///
    /// \param a_position
    /// A vector containing the transform position
    ///
    /// \param a_rotation
    /// A vector containing the transform rotation
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void set(const Eigen::Vector3d& a_position,
             const Eigen::Matrix3d& a_rotation)
    {
        m_matrix[0][0] = a_rotation(0, 0);
        m_matrix[1][0] = a_rotation(0, 1);
        m_matrix[2][0] = a_rotation(0, 2);
        m_matrix[3][0] = a_position(0);

        m_matrix[0][1] = a_rotation(1, 0);
        m_matrix[1][1] = a_rotation(1, 1);
        m_matrix[2][1] = a_rotation(1, 2);
        m_matrix[3][1] = a_position(1);

        m_matrix[0][2] = a_rotation(2, 0);
        m_matrix[1][2] = a_rotation(2, 1);
        m_matrix[2][2] = a_rotation(2, 2);
        m_matrix[3][2] = a_position(2);

        m_matrix[0][3] = 0.0;
        m_matrix[1][3] = 0.0;
        m_matrix[2][3] = 0.0;
        m_matrix[3][3] = 1.0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method pushes the current OpenGL matrix on the stack and multiplies it
    /// with this \ref cMatrixGL matrix.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void glMatrixPushMultiply()
    {
        glPushMatrix();
        glMultMatrixd(m_matrix[0]);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method pops the current OpenGL matrix off the stack.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void glMatrixPop()
    {
        glPopMatrix();
    }
};

#endif  // EXAMPLES_GLFW_SPHERE_CMATRIXGL_H
