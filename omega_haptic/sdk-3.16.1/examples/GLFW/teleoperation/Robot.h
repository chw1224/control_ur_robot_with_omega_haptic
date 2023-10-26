////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2023 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.16.1
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOT_H
#define ROBOT_H

// C library headers
#include <mutex>

// Eigen library headers
#include "Eigen/Eigen"

// Application headers
#include "Transform.h"

////////////////////////////////////////////////////////////////////////////////
///
/// This class implements a dummy slave robot whose end-effector can be sent to
/// any position/rotation target within pre-defined position and angular
/// workspace limits. The dummy robot moves instantly to any new target it is
/// given.
///
////////////////////////////////////////////////////////////////////////////////

class Robot
{
 private:

    /// Member access mutex
    mutable std::mutex m_lock;

    /// Flag to enforce workspace limits
    static constexpr bool EnforceWorkspaceLimits = true;

    /// Linear workspace radius in [m]
    static constexpr double LinearWorkspaceLimit = 0.05;

    /// Angular workspace angle in [rad]
    static constexpr double AngularWorkspaceLimit = M_PI * 90.0 / 180.0;

    // Current slave robot position
    Transform m_current;

 public:

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// This method sets the target that the slave robot needs to try and reach.
    /// If \ref EnforceWorkspaceLimits is set to \b true, simple workspace limits
    /// are applied as follows:
    /// \li the linear workspace is limited to a sphere of
    ///     radius \ref LinearWorkspaceLimit,
    /// \li the angular workspace is limited to a cone of
    ///     angle \ref AngularWorkspaceLimit.
    ///
    /// \param a_target
    /// The target position that the robot should try and reach
    ///
    ///////////////////////////////////////////////////////////////////////////////

    void setTarget(const Transform& a_target)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        // Apply workspace limits if required.
        if (EnforceWorkspaceLimits)
        {
            // Compute robot actual position.
            // Limit the linear workspace to a sphere centered at 0.0.
            Eigen::Vector3d currentPosition = a_target.position();
            if (currentPosition.norm() > LinearWorkspaceLimit)
            {
                currentPosition *= LinearWorkspaceLimit / currentPosition.norm();
            }

            // Compute robot actual rotation.
            // We make the assumption that the robot roll axis is along X,
            // and we limit the YZ rotation of the robot tool to a cone.
            Eigen::Matrix3d currentRotation = a_target.rotation();
            Eigen::Vector3d coneAxis = Eigen::Vector3d::UnitX();
            Eigen::Vector3d rollAxis = currentRotation.col(0).normalized();
            double robotToolAngle = std::acos(coneAxis.dot(rollAxis));
            if (robotToolAngle > AngularWorkspaceLimit / 2.0)
            {
                // Compute cross product between world roll axis and robot axis.
                // This produces a unit vector around which we need to rotate to
                // counter the excess rotation.
                Eigen::Vector3d counterRotationAxis = rollAxis.cross(coneAxis);

                // Compute the counter rotation angle we need to apply to stay within
                // the angular workspace limit.
                double counterRotationAngle = robotToolAngle - AngularWorkspaceLimit / 2.0;

                // Construct a rotation that brings the robot back within the angular
                // workspace limit.
                Eigen::Matrix3d counterRotation { Eigen::AngleAxisd(counterRotationAngle, counterRotationAxis) };

                // Apply the counter rotation to keep the current rotation within the
                // angular workspace limit.
                currentRotation = counterRotation * currentRotation.eval();
            }

            // Store the current position.
            m_current.setPosition(currentPosition);
            m_current.setRotation(currentRotation);
        }

        // Otherwise, do not enforce workspace limits.
        else
        {
            m_current.setPosition(a_target.position());
            m_current.setRotation(a_target.rotation());
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// This method returns the current slave robot position in the world.
    ///
    /// \return
    /// The current slave robot position
    ///
    ///////////////////////////////////////////////////////////////////////////////

    const Transform& current() const
    {
        std::lock_guard<std::mutex> lock(m_lock);

        return m_current;
    }
};

#endif  // ROBOT_H
