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
/// This example illustrates how to apply a gentle spring-damper force model to
/// keep a Force Dimension haptic device in a given location.
///
/// It allows the haptic device to move freely in a gravity compensation loop
/// while the user button on the end-effector is pressed. When the user releases
/// the button, the current position and rotation are stored and the spring-
/// damper force model is applied to gently hold the device in that position,
/// and bring it back to it if an outside perturbation moves it away from its
/// holding position.
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
#include "drdc.h"

////////////////////////////////////////////////////////////////////////////////
///
/// This function implements an equivalent of C++17 std::clamp().
///
/// \note
/// We provide this function as we cannot guarantee that this example will be
/// built with C++17 support. It can be seamlessly replaced with std::clamp()
/// if C++17 support is enabled.
///
////////////////////////////////////////////////////////////////////////////////

template<typename T>
constexpr const T& clamp(const T& a_val, const T& a_minVal, const T& a_maxVal)
{
    return (a_val < a_minVal) ? a_minVal : (a_maxVal < a_val) ? a_maxVal : a_val;
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
    std::cout << "Device Hold Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Enable expert mode to allow wrist joint torques control.
    dhdEnableExpertMode();

    // Open the first available haptic device.
    if (drdOpen() < 0)
    {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Make sure the device is compatible with the robotics library (DRD).
    if (!drdIsSupported())
    {
        std::cout << "unsupported device type " << dhdGetSystemName() << std::endl;
        std::cout << "exiting..." << std::endl;
        dhdSleep(2.0);
        drdClose();
        return -1;
    }

    // Display the device type.
    std::cout << dhdGetSystemName() << " device detected" << std::endl << std::endl;

    // Perform automatic initialization of the device if required.
    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        std::cout << "error: failed to initialize device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    else if (drdStart() < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move to the center of the workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter) < 0)
    {
        std::cout << "error: failed to move the device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Stop the regulation thread but leaves the forces enabled on the device.
    if (drdStop(true) < 0)
    {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable button emulation on devices with an active gripper.
    if (dhdHasActiveGripper() && dhdEmulateButton(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable button emulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl << std::endl;

    // Allocate and initialize display control variables.
    double lastDisplayUpdateTime = dhdGetTime();

    // Allocate and initialize user button state.
    bool previousUserButton = false;

    // Allocate and initialize device position (base, wrist and gripper).
    double px = 0;
    double py = 0;
    double pz = 0;
    double ra = 0;
    double rb = 0;
    double rg = 0;
    double pg = 0;

    // Allocate and initialize device velocity (base, wrist and gripper).
    double jointAnglesVelocity[DHD_MAX_DOF] = {};
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wa = 0.0;
    double wb = 0.0;
    double wg = 0.0;
    double vg = 0.0;

    // Allocate and initialize device holding position (base, wrist and gripper).
    double pxHold = 0;
    double pyHold = 0;
    double pzHold = 0;
    double raHold = 0;
    double rbHold = 0;
    double rgHold = 0;
    double pgHold = 0;

    // Allocate and initialize device force and torque (base, wrist and gripper).
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double qa = 0.0;
    double qb = 0.0;
    double qg = 0.0;
    double fg = 0.0;

    // Run the haptic loop.
    while (true)
    {
        // Retrieve base, wrist and gripper position.
        if (dhdGetPositionAndOrientationDeg(&px, &py, &pz, &ra, &rb, &rg) < 0)
        {
            std::cout << "error: failed to retrieve device position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperGap(&pg) < 0)
        {
            std::cout << "error: failed to retrieve device gripper opening (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the base an gripper velocity.
        if (dhdGetLinearVelocity(&vx, &vy, &vz) < 0)
        {
            std::cout << "error: failed to retrieve linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperLinearVelocity(&vg) < 0)
        {
            std::cout << "error: failed to retrieve gripper velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the wrist joint angles velocity.
        if (dhdGetJointVelocities(jointAnglesVelocity) < 0)
        {
            std::cout << "error: failed to retrieve angular velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        wa = jointAnglesVelocity[3];
        wb = jointAnglesVelocity[4];
        wg = jointAnglesVelocity[5];

        // Detect user button events.
        bool userButton = (dhdGetButton(0) != 0);
        bool userButtonPressed = (userButton && !previousUserButton);
        bool userButtonReleased = (!userButton && previousUserButton);
        previousUserButton = userButton;

        // When the button is released, store the current position as a holding target.
        if (userButtonReleased)
        {
            // Store base position.
            pxHold = px;
            pyHold = py;
            pzHold = pz;

            // Store wrist joint angles.
            raHold = ra;
            rbHold = rb;
            rgHold = rg;

            // Store gripper opening.
            pgHold = pg;
        }

        // While the button is pressed, let the device move freely.
        if (userButton)
        {
            fx = 0.0;
            fy = 0.0;
            fz = 0.0;
            qa = 0.0;
            qb = 0.0;
            qg = 0.0;
            fg = 0.0;
        }

        // If the button is not pressed, apply a light spring to hold the device in place.
        else
        {
            // Base position spring stiffness in [N/m]
            constexpr double Kp = 100.0;

            // Base position spring damping in [N/(m/s)]
            constexpr double Kv = 20.0;

            // Base position maximum force in [N]
            constexpr double MaxForce = 2.0;

            // Wrist joint angular spring stiffness in [Nm/deg]
            constexpr double Kr = 0.01;

            // Wrist joint angular spring damping in [Nm/(deg/s)]
            constexpr double Kw = 0.04;

            // Wrist joint maximum torque in [Nm]
            constexpr double MaxTorque = 0.02;

            // Gripper spring stiffness in [N/m]
            constexpr double Kgp = 100.0;

            // Gripper spring damping in [N/(m/s)]
            constexpr double Kgv = 5.0;

            // Gripper maximum force in [N]
            constexpr double MaxGripperForce = 1.0;

            // Compute base position spring.
            fx = Kp * (pxHold - px) - Kv * vx;
            fy = Kp * (pyHold - py) - Kv * vy;
            fz = Kp * (pzHold - pz) - Kv * vz;

            // Make sure the Cartesian maximum force is not exceeded.
            double forceMagnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (forceMagnitude > MaxForce)
            {
                double forceRatio = MaxForce / forceMagnitude;
                fx *= forceRatio;
                fy *= forceRatio;
                fz *= forceRatio;
            }

            // Compute wrist joint springs, and limit each joint torque to the set maximum.
            qa = clamp(Kr * (raHold - ra) - Kw * wa, -MaxTorque, MaxTorque);
            qb = clamp(Kr * (rbHold - rb) - Kw * wb, -MaxTorque, MaxTorque);
            qg = clamp(Kr * (rgHold - rg) - Kw * wg, -MaxTorque, MaxTorque);

            // Compute gripper position spring, and limit it to its set maximum.
            fg = clamp(Kgp * (pgHold - pg) - Kgv * vg, -MaxGripperForce, MaxGripperForce);
        }

        // Apply the required forces and torques.
        if (dhdSetForceAndWristJointTorquesAndGripperForce(fx, fy, fz, qa, qb, qg, fg) < 0)
        {
            std::cout << "error: failed to apply forces (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Periodically display the haptic device status and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;       
            
            if (userButton)
            {
                std::cout << "current status: in use   \r";
            }
            else
            {
                std::cout << "current status: holding  \r";
            }
            std::cout.flush();
        }

        // Process user input.
        if (dhdKbHit() && dhdKbGet() == 'q')
        {
            std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
            break;
        }
    }

    // Close the connection to the haptic device.
    if (drdClose() < 0)
    {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Report success.
    std::cout << "connection closed" << std::endl;
    return 0;
}
