////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2023 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.16.1
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///
/// This example illustrates how to retrieve a Force Dimension haptic device
/// base and wrist Jacobian matrixes. It uses the Jacobian matrixes to translate
/// a Cartesian spring-damper force and torque model to joint torques, and
/// applies those joint torques to the haptic device.
///
/// The spring-damper model parameters are defined and documented in the haptic
/// loop and can be adjusted to modify the behavior of the application.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <cmath>
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "dhdc.h"

////////////////////////////////////////////////////////////////////////////////
///
/// This function computes the transpose of a given matrix.
///
/// \param a_matrix
/// The matrix to transpose
///
/// \param a_result
/// The transposed matrix
///
////////////////////////////////////////////////////////////////////////////////

inline void transpose(const double a_matrix[3][3],
                      double a_result[3][3])
{
    a_result[0][0] = a_matrix[0][0];
    a_result[1][0] = a_matrix[0][1];
    a_result[2][0] = a_matrix[0][2];

    a_result[0][1] = a_matrix[1][0];
    a_result[1][1] = a_matrix[1][1];
    a_result[2][1] = a_matrix[1][2];

    a_result[0][2] = a_matrix[2][0];
    a_result[1][2] = a_matrix[2][1];
    a_result[2][2] = a_matrix[2][2];
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function decomposes a rotation matrix into its axis angle
/// representation.
///
/// \param a_matrix
/// The matrix to decompose
///
/// \param a_axis
/// The rotation axis vector
///
/// \param a_angle
/// The rotation angle in [rad], in the (0.0, M_PI) range.
///
/// \note
/// This implementation returns the axis-angle combination with the shortest
/// angular path. As a result, the axis angle will change direction when the
/// rotation crosses over 180 deg.
///
////////////////////////////////////////////////////////////////////////////////

inline void axisAngle(const double a_matrix[3][3],
                      double a_axis[3],
                      double& a_angle)
{
    // Compute the angle.
    a_angle = acos((a_matrix[0][0] + a_matrix[1][1 ] + a_matrix[2][2] - 1.0) / 2.0);

    // Handle the singularity.
    if (a_angle < 1e-6)
    {
        a_axis[0] = 1.0;
        a_axis[1] = 0.0;
        a_axis[2] = 0.0;

        return;
    }

    // Compute the axis.
    a_axis[0] = (a_matrix[2][1] - a_matrix[1][2]) / std::sqrt(std::pow(a_matrix[2][1] - a_matrix[1][2], 2.0) + std::pow(a_matrix[0][2] - a_matrix[2][0], 2.0) + std::pow(a_matrix[1][0] - a_matrix[0][1], 2.0));
    a_axis[1] = (a_matrix[0][2] - a_matrix[2][0]) / std::sqrt(std::pow(a_matrix[2][1] - a_matrix[1][2], 2.0) + std::pow(a_matrix[0][2] - a_matrix[2][0], 2.0) + std::pow(a_matrix[1][0] - a_matrix[0][1], 2.0));
    a_axis[2] = (a_matrix[1][0] - a_matrix[0][1]) / std::sqrt(std::pow(a_matrix[2][1] - a_matrix[1][2], 2.0) + std::pow(a_matrix[0][2] - a_matrix[2][0], 2.0) + std::pow(a_matrix[1][0] - a_matrix[0][1], 2.0));

    // Normalize the axis.
    double axisNorm = std::sqrt(std::pow(a_axis[0], 2.0) + std::pow(a_axis[1], 2.0) + std::pow(a_axis[2], 2.0));
    if (axisNorm > 1e-6)
    {
        a_axis[0] /= axisNorm;
        a_axis[1] /= axisNorm;
        a_axis[2] /= axisNorm;
    }
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char *argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Jacobian Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Enable expert mode to allow operations in joint space.
    dhdEnableExpertMode();

    // Open the first available haptic device.
    if (dhdOpen() < 0)
    {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display the device type.
    std::cout << dhdGetSystemName() << " device detected" << std::endl << std::endl;

    // Enable button emulation on devices with an active gripper.
    if (dhdHasActiveGripper() && dhdEmulateButton(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable button emulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display user instructions.
    std::cout << "press [space] to toggle the virtual spring" << std::endl;
    std::cout << "      'q' to quit" << std::endl << std::endl;

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Allocate and initialize haptic loop control variables.
    bool centeringActive = false;

    // Allocate and initialize device joint angles.
    double jointAngles[DHD_MAX_DOF] = {};

    // Allocate and initialize device position (base and wrist).
    double px = 0;
    double py = 0;
    double pz = 0;
    double rotation[3][3] = {};
    double axis[3] = { 1.0, 0.0, 0.0 };
    double angle = {};

    // Allocate and initialize device velocity (base and wrist).
    double vx = 0;
    double vy = 0;
    double vz = 0;
    double wx = 0;
    double wy = 0;
    double wz = 0;

    // Allocate and initialize device force and torque.
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;

    // Allocate and initialize device joint torques.
    double jointTorques[DHD_MAX_DOF] = {};

    // Allocate and initialize Jacobian matrices and their transposed counterparts.
    double Jb[3][3] = {};
    double JbT[3][3] = {};
    double Jw[3][3] = {};
    double JwT[3][3] = {};

    // Allocate and initialize display control variables.
    double lastDisplayUpdateTime = dhdGetTime();

    // Run the haptic loop.
    bool running = true;
    while (running)
    {
        // Retrieve the haptic device position.
        if (dhdGetPositionAndOrientationFrame(&px, &py, &pz, rotation) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Represent the rotation in axis angle form
        axisAngle(rotation, axis, angle);

        // Retrieve the haptic device velocity.
        if (dhdGetLinearVelocity(&vx, &vy, &vz) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetAngularVelocityRad(&wx, &wy, &wz) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the device joint angles.
        if (dhdGetJointAngles(jointAngles) < 0)
        {
            std::cout << "error: failed to read joint angles (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Compute the base Jacobian.
        if (dhdDeltaJointAnglesToJacobian(jointAngles[0], jointAngles[1], jointAngles[2], Jb) < DHD_NO_ERROR)
        {
            std::cout << "error: failed to compute base Jacobian matrix (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Compute the wrist Jacobian.
        if (dhdHasWrist() && dhdWristJointAnglesToJacobian(jointAngles[3], jointAngles[4], jointAngles[5], Jw) < DHD_NO_ERROR)
        {
            std::cout << "error: failed to compute wrist Jacobian matrix (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the joint torques required to compensate for gravity.
        if (dhdJointAnglesToGravityJointTorques(jointAngles, jointTorques) < 0)
        {
            std::cout << "error: failed to compute gravity compensation joint torques (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // If the button is pressed, enable centering force.
        if (centeringActive)
        {
            // Base position spring stiffness in [N/m]
            constexpr double Kp = 100.0;

            // Base position spring damping in [N/(m/s)]
            constexpr double Kv = 20.0;

            // Base position maximum force in [N]
            constexpr double MaxForce = 5.0;

            // Compute Cartesian force to apply.
            fx = -1.0 * Kp * px - Kv * vx;
            fy = -1.0 * Kp * py - Kv * vy;
            fz = -1.0 * Kp * pz - Kv * vz;

            // Make sure the Cartesian maximum force is not exceeded.
            double forceMagnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (forceMagnitude > MaxForce)
            {
                double forceRatio = MaxForce / forceMagnitude;
                fx *= forceRatio;
                fy *= forceRatio;
                fz *= forceRatio;
            }

            // Compute base joint torques (Q = Jacobian_transposed * F)
            transpose(Jb, JbT);
            jointTorques[0] += JbT[0][0] * fx + JbT[0][1] * fy + JbT[0][2] * fz;
            jointTorques[1] += JbT[1][0] * fx + JbT[1][1] * fy + JbT[1][2] * fz;
            jointTorques[2] += JbT[2][0] * fx + JbT[2][1] * fy + JbT[2][2] * fz;

            // Wrist joint angular spring stiffness in [Nm/rad]
            constexpr double Kr = 0.2;

            // Wrist joint angular spring damping in [Nm/(rad/s)]
            constexpr double Kw = 0.05;

            // Wrist joint maximum torque in [Nm]
            constexpr double MaxTorque = 0.05;

            // Compute Cartesian torque to apply.
            tx = -1.0 * Kr * angle * axis[0] - Kw * wx;
            ty = -1.0 * Kr * angle * axis[1] - Kw * wy;
            tz = -1.0 * Kr * angle * axis[2] - Kw * wz;

            // Make sure the Cartesian maximum torque is not exceeded.
            double torqueMagnitude = std::sqrt(tx * tx + ty * ty + tz * tz);
            if (torqueMagnitude > MaxTorque)
            {
                double torqueRatio = MaxTorque / torqueMagnitude;
                tx *= torqueRatio;
                ty *= torqueRatio;
                tz *= torqueRatio;
            }

            // Compute wrist joint torques (Q = Jacobian_transposed * F)
            transpose(Jw, JwT);
            jointTorques[3] += JwT[0][0] * tx + JwT[0][1] * ty + JwT[0][2] * tz;
            jointTorques[4] += JwT[1][0] * tx + JwT[1][1] * ty + JwT[1][2] * tz;
            jointTorques[5] += JwT[2][0] * tx + JwT[2][1] * ty + JwT[2][2] * tz;
        }

        // Apply joint torques on base (0x07), wrist (0x38) and gripper (0x40) axes.
        if (dhdSetJointTorques(jointTorques, 0x7f) < 0)
        {
            std::cout << "error: failed to apply joint torques (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the haptic device actual force and torque rendered.
        if (dhdGetForceAndTorque(&fx, &fy, &fz, &tx, &ty, &tz) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read force (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Periodically display the haptic device status and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << ((centeringActive) ? "ACTIVE" : "------") << "  |  ";
            std::cout << std::showpos << std::fixed << std::setprecision(3);
            std::cout << "f (" << std::setw(6) << fx << " " << std::setw(6) << fy << " " << std::setw(6) << fz << ") N  |  ";
            std::cout << "t (" << std::setw(6) << tx << " " << std::setw(6) << ty << " " << std::setw(6) << tz << ") Nm  |  ";
            std::cout << "freq " << std::noshowpos << std::setprecision(2) << std::setw(4) << dhdGetComFreq() << " kHz  \r";
            std::cout.flush();
        }

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Toggle the centering force.
                case ' ':
                {
                    centeringActive = !centeringActive;
                    break;
                }

                // Quit the application.
                case 'q':
                {
                    std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
                    running = false;
                    break;
                }
            }
        }
    }

    // Close the connection to the haptic device.
    if (dhdClose() < 0)
    {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Report success.
    std::cout << "connection closed" << std::endl;
    return 0;
}
