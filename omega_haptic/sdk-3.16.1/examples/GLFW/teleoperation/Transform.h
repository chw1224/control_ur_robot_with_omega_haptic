////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2023 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.16.1
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TRANSFORM_H
#define TRANSFORM_H

// C library headers
#define _USE_MATH_DEFINES
#include <math.h>

// C++ library headers
#include <mutex>

// Eigen library headers
#include "Eigen/Eigen"

////////////////////////////////////////////////////////////////////////////////
///
/// This class implements a thread-safe, isometric affine transformation
/// (https://en.wikipedia.org/wiki/Affine_transformation).
/// Conceptually, this is a 4x4 matrix with its last row set to (0, 0, 0, 1).
/// This class however stores the rotation and translation members explicitly
/// to make the maths more explicit.
///
/// \note
/// The \ref rotation() and \ref setRotation() methods use rotation matrices to
/// rotations. The \ref angles() and \ref setAngles() methods use the Tait-Bryan
/// XYZ intrinsic convention to represent rotations.
///
////////////////////////////////////////////////////////////////////////////////

class Transform
{
 private:

    /// Member access mutex
    mutable std::mutex m_lock;

    /// Position vector
    Eigen::Vector3d m_position;

    /// Rotation matrix
    Eigen::Matrix3d m_rotation;

 public:

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method converts Tait-Bryan XYZ angles to a rotation matrix.
    ///
    /// \return
    /// A rotation matrix matching \c a_angles.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    static Eigen::Matrix3d rotation(const Eigen::Vector3d& a_angles)
    {
        Eigen::Matrix3d rotation;

        double cosx = cos(a_angles(0));
        double sinx = sin(a_angles(0));
        double cosy = cos(a_angles(1));
        double siny = sin(a_angles(1));
        double cosz = cos(a_angles(2));
        double sinz = sin(a_angles(2));

        rotation(0,0) = cosy * cosz;
        rotation(0,1) = -cosy * sinz;
        rotation(0,2) = siny;
        rotation(1,0) = cosz * sinx * siny + cosx * sinz;
        rotation(1,1) = cosx * cosz - sinx * siny * sinz;
        rotation(1,2) = -cosy * sinx;
        rotation(2,0) = -cosx * cosz * siny + sinx * sinz;
        rotation(2,1) = cosz * sinx + cosx * siny * sinz;
        rotation(2,2) = cosx * cosy;

        return rotation;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method converts a rotation matrix to Tait-Bryan XYZ angles.
    ///
    /// \return
    /// Tait-Bryan XYZ angles matching \c a_rotation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    static Eigen::Vector3d angles(const Eigen::Matrix3d& a_rotation)
    {
        Eigen::Vector3d angles;
        if (a_rotation(0,2) < 1.0)
        {
            if (a_rotation(0,2) > -1.0)
            {
                angles(1) = asin(a_rotation(0,2));
                angles(0) = atan2(-a_rotation(1,2), a_rotation(2,2));
                angles(2) = atan2(-a_rotation(0,1), a_rotation(0,0));
            }
            else
            {
                angles(1) = -M_PI / 2.0;
                angles(0) = -atan2(a_rotation(1,0), a_rotation(1,1));
                angles(2) = 0.0;
            }
        }
        else
        {
            angles(1) = M_PI / 2.0;
            angles(0) = atan2(a_rotation(1,0), a_rotation(1,1));
            angles(2) = 0.0;
        }

        return angles;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method converts a rotation matrix to a quaternion.
    ///
    /// \return
    /// A quaternion matching \c a_rotation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    static Eigen::Quaterniond quaternion(const Eigen::Matrix3d& a_rotation)
    {
        Eigen::Quaterniond quaternion { a_rotation };

        return quaternion;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method converts a rotation matrix to an angle-axis representation.
    ///
    /// \return
    /// An angle-axis object matching \c a_rotation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    static Eigen::AngleAxisd angleAxis(const Eigen::Matrix3d& a_rotation)
    {
        Eigen::AngleAxisd angleAxis { a_rotation };

        return angleAxis;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method prevents a rotation represented by an angle-axis object from
    /// "flipping" over to the other hemisphere. It does so by keeping track of
    /// its history in the \c a_previousAngleAxis reference. The main purpose of
    /// this method is to return a rotation that is guaranteed not to have any
    /// discontinuity over time.
    ///
    /// \param a_angleAxis
    /// The angle-axis object that must be prevented from flipping over.
    ///
    /// \param a_previousAngleAxis
    /// The historical reference used by the method to assess whether \c
    /// a_angleAxis has flipped over. It is entirely managed by this method,
    /// and should generally not be modified outside of it.
    ///
    /// \return
    /// An angle-axis object that is as close to \c a_angleAxis as possible without
    /// flipping over with respect to its past values.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    static Eigen::AngleAxisd noFlip(const Eigen::AngleAxisd& a_angleAxis,
                                    Eigen::AngleAxisd& a_previousAngleAxis)
    {
        Eigen::AngleAxisd angleAxis = a_angleAxis;

        // If the rotation axis changes direction, we have flipped. When that happens,
        // revert the axis and keep the last angle that we saw before the flip.
        // Inverting the axis of the input angle-axis guarantees that there will be no
        // "jump" if we revert to an unflipped input angle-axis some (angular) distance
        // from the angle-axis at which we entered the flipped state.
        constexpr double AngularEpsilon = M_PI * 1.0 / 180.0;
        if (angleAxis.angle() > AngularEpsilon)
        {
            if (angleAxis.axis().dot(a_previousAngleAxis.axis()) < 0)
            {
                angleAxis.axis() = -a_angleAxis.axis();
                angleAxis.angle() = a_previousAngleAxis.angle();
            }
        }

        // Store the unflipped value in the reference rotation for the next run.
        // We do this for convenience so that the caller does not have to systematically
        // update the reference value after every call.
        a_previousAngleAxis = angleAxis;

        return angleAxis;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// Default constructor
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Transform()
    {
        zero();
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// Copy constructor
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Transform(const Transform& a_rhs)
    {
        std::lock_guard<std::mutex> rhs(a_rhs.m_lock);

        m_position = a_rhs.m_position;
        m_rotation = a_rhs.m_rotation;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// Thread-safe assignment operator
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Transform& operator=(const Transform& a_rhs)
    {
        if (this != &a_rhs)
        {
            std::lock(m_lock, a_rhs.m_lock);
            std::lock_guard<std::mutex> lhs(m_lock, std::adopt_lock);
            std::lock_guard<std::mutex> rhs(a_rhs.m_lock, std::adopt_lock);

            m_position = a_rhs.m_position;
            m_rotation = a_rhs.m_rotation;
        }

        return *this;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// Set a null transform.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void zero()
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_position.setZero();
        m_rotation.setIdentity();
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current position and rotation.
    ///
    /// \param a_x
    /// Desired position on the X axis in [m]
    ///
    /// \param a_y
    /// Desired position on the Y axis in [m]
    ///
    /// \param a_z
    /// Desired position on the Z axis in [m]
    ///
    /// \param a_rotation
    /// The desired rotation matrix
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void set(double a_x,
             double a_y,
             double a_z,
             double a_rotation[3][3])
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_position << a_x, a_y, a_z;
        m_rotation << a_rotation[0][0], a_rotation[0][1], a_rotation[0][2],
                      a_rotation[1][0], a_rotation[1][1], a_rotation[1][2],
                      a_rotation[2][0], a_rotation[2][1], a_rotation[2][2];
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the transform from its 4x4 matrix representation.
    ///
    /// \param a_matrix
    /// The desired 4x4 transformation matrix
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setMatrix(const Eigen::Matrix4d& a_matrix)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_rotation = a_matrix.block<3, 3>(0, 0);
        m_position = a_matrix.block<1, 3>(0, 3);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current position from a position vector.
    ///
    /// \param a_position
    /// A vector that contains the desired position in [m]
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setPosition(const Eigen::Vector3d& a_position)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_position = a_position;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current rotation from a rotation matrix.
    ///
    /// \param a_rotation
    /// The desired rotation matrix
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setRotation(const Eigen::Matrix3d& a_rotation)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_rotation = a_rotation;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current rotation from Tait-Bryan XYZ angles.
    ///
    /// \param a_angles
    /// A vector that contains the desired rotation as Tait-Bryan XYZ angles in
    /// [rad]
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setAngles(const Eigen::Vector3d& a_angles)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_rotation = rotation(a_angles);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current rotation from a quaternion.
    ///
    /// \param a_quaternion
    /// A quaternion object that represents the desired rotation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setQuaternion(const Eigen::Quaterniond& a_quaternion)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_rotation = Eigen::Matrix3d { a_quaternion };
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the current rotation from an angle-axis representation.
    ///
    /// \param a_angleAxis
    /// An angle-axis object that represents the desired rotation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    void setAngleAxis(const Eigen::AngleAxisd& a_angleAxis)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        m_rotation = Eigen::Matrix3d { a_angleAxis };
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method returns the corresponding 4x4 matrix representation.
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix4d matrix() const
    {
        Eigen::Matrix4d transform;
        transform << m_rotation(0, 0), m_rotation(0, 1), m_rotation(0, 2), m_position(0),
                     m_rotation(1, 0), m_rotation(1, 1), m_rotation(1, 2), m_position(1),
                     m_rotation(2, 0), m_rotation(2, 1), m_rotation(2, 2), m_position(2),
                                  0.0,              0.0,              0.0,           1.0;

        return transform;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method retrieves the current position vector.
    ///
    /// \return
    /// The position vector in [m]
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Vector3d position() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return m_position;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method retrieves the current rotation matrix.
    ///
    /// \return
    /// The rotation matrix
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix3d rotation() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return m_rotation;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method retrieves the current rotation as Tait-Bryan XYZ angles.
    ///
    /// \return
    /// The Tait-Bryan XYZ angles in [rad]
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Vector3d angles() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return angles(m_rotation);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method retrieves the current rotation as a quaternion.
    ///
    /// \return
    /// A quaternion
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Quaterniond quaternion() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return quaternion(m_rotation);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///
    /// This method retrieves the current rotation as an angle-axis object.
    ///
    /// \return
    /// An angle-axis object
    ///
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::AngleAxisd angleAxis() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return angleAxis(m_rotation);
    }
};

#endif  // TRANSFORM_H
