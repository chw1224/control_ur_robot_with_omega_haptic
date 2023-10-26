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
/// This example illustrates how to control a Force Dimension haptic device like
/// a robot using the Force Dimension robotic SDK.
///
/// It starts by initializing the haptic device if required, than slowly moves
/// the end- effector to a series of randomly selected target points. It then
/// repeats the same procedure but at a higher velocity. Finally, it drives the
/// end-effector along a trajectory that draws a spiral on the surface of a
/// sphere. The user can request the application to exit at any moment during
/// execution.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "drdc.h"

// [deg] to [rad] conversion macro
#define D2R(a) (M_PI * (a) / 180.0)

////////////////////////////////////////////////////////////////////////////////

int
main(int argc,
     char* argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Robotic Control Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

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

    // Display user instructions.
    std::cout << "press 'a' to abort" << std::endl << std::endl;

    // Allocate and initialize display control variables.
    double lastDisplayUpdateTime = dhdGetTime();

    // Define a series of target coordinates distributed over each workspace axis ([8] cm for positions, [30] deg for rotations).
    constexpr int NumTargetPositions = 9;
    constexpr int NumTargetRotations = 9;
    double targetPositions[NumTargetPositions] = { -0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03, 0.04 };
    double targetRotations[NumTargetRotations] = { D2R(-30.0), D2R(-20.0), D2R(-10.0), 0.0, D2R(10.0), D2R(20.0), D2R(30.0) };

    // Move within the cubic robotic workspace at default acceleration and speed
    // (which are conservative).
    int numTargetPoints = 0;
    while (numTargetPoints < 6)
    {
        // Arbitrarily throttle the monitoring of the robot to 20 Hz.
        dhdSleep(0.05);

        // If the device is not moving (meaning that any previous drdMoveTo() command has finished),
        // generate a new random target within the workspace and order the device to go there.
        while (!drdIsMoving ())
        {
            double target[DHD_MAX_DOF] = {};
            target[0] = 0.02 + targetPositions[rand() % NumTargetPositions];
            target[1] = targetPositions[rand() % NumTargetPositions];
            target[2] = targetPositions[rand() % NumTargetPositions];
            if (dhdHasActiveWrist())
            {
                target[3] = targetRotations[rand() % NumTargetRotations];
                target[4] = targetRotations[rand() % NumTargetRotations];
                target[5] = targetRotations[rand() % NumTargetRotations];
            }
            if (drdMoveTo(target, false) == 0)
            {
                numTargetPoints += 1;
            }
        }

        // Check for exit requests from the user.
        if (dhdKbHit() && dhdKbGet()=='a')
        {
            std::cout << std::endl << std::endl << "aborting at user's request" << std::endl;
            return 0;
        }

        // Make sure that the regulation thread is still running.
        if (!drdIsRunning ())
        {
            std::cout << "error: regulation thread not running" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Periodically display the regulation performance and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << "regulation thread running at " << std::fixed << std::setprecision(2) << std::setw(4) << drdGetCtrlFreq() << " kHz \r";
            std::cout.flush();
        }
    }

    // Retrieve the current linear motion parameters.
    double accelerationLinear = 0.0;
    double velocityLinear = 0.0;
    double jerkLinear = 0.0;
    double accelerationAngular = 0.0;
    double velocityAngular = 0.0;
    double jerkAngular = 0.0;
    if (drdGetPosMoveParam(&accelerationLinear, &velocityLinear, &jerkLinear) < 0)
    {
        std::cout << "error: failed to retrieve linear motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    if (drdGetRotMoveParam(&accelerationAngular, &velocityAngular, &jerkAngular) < 0)
    {
        std::cout << "error: failed to retrieve angular motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Define the motion parameters scaling factor.
    constexpr double MotionScalingFactor = 10.0;

    // Scale the motion parameters to make the device move faster.
    if (drdSetPosMoveParam(MotionScalingFactor * accelerationLinear, MotionScalingFactor * velocityLinear, MotionScalingFactor * jerkLinear) < 0)
    {
        std::cout << "error: failed to set linear motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    if (drdSetRotMoveParam(MotionScalingFactor * accelerationAngular, MotionScalingFactor * velocityAngular, MotionScalingFactor * jerkAngular) < 0)
    {
        std::cout << "error: failed to set angular motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move within the cubic robotic workspace at higher acceleration and speed.
    numTargetPoints = 0;
    while (numTargetPoints < 6)
    {
        // Arbitrarily throttle the monitoring of the robot to 20 Hz.
        dhdSleep(0.02);

        // If the device is not moving (meaning that any previous drdMoveTo() command has finished),
        // generate a new random target within the workspace and order the device to go there.
        while (!drdIsMoving ())
        {
            double target[DHD_MAX_DOF] = {};
            target[0] = 0.02 + targetPositions[rand() % NumTargetPositions];
            target[1] = targetPositions[rand() % NumTargetPositions];
            target[2] = targetPositions[rand() % NumTargetPositions];
            if (dhdHasActiveWrist())
            {
                target[3] = targetRotations[rand() % NumTargetRotations];
                target[4] = targetRotations[rand() % NumTargetRotations];
                target[5] = targetRotations[rand() % NumTargetRotations];
            }
            if (drdMoveTo(target, false) == 0)
            {
                numTargetPoints += 1;
            }
        }

        // Check for exit requests from the user.
        if (dhdKbHit() && dhdKbGet()=='a')
        {
            std::cout << std::endl << std::endl << "aborting at user's request" << std::endl;
            return 0;
        }

        // Make sure that the regulation thread is still running.
        if (!drdIsRunning ())
        {
            std::cout << "error: regulation thread not running" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Periodically display the regulation performance and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << "regulation thread running at " << std::fixed << std::setprecision(2) << std::setw(4) << drdGetCtrlFreq() << " kHz \r";
            std::cout.flush();
        }
    }

    // Restore the original motion control parameters.
    if (drdSetPosMoveParam(accelerationLinear, velocityLinear, jerkLinear) < 0)
    {
        std::cout << "error: failed to restore linear motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    if (drdSetRotMoveParam(accelerationAngular, velocityAngular, jerkAngular) < 0)
    {
        std::cout << "error: failed to restore angular motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Track a sphere in real-time: this shows how to asynchronously send the robot along a
    // given trajectory. The trajectory is interpolated along the target points according to
    // a set of physical parameters such as max allowed acceleration, max allowed velocity
    // and max allowed deceleration time these parameters are controlled by the
    // drdSetPoseTrackParam() function.

    // Sphere center on X in [m]
    constexpr double SphereCenterX = 0.02;

    // Sphere center on Y in [m]
    constexpr double SphereCenterY = 0.0;

    // Sphere center on Z in [m]
    constexpr double SphereCenterZ = 0.0;

    // Sphere radius in [m]
    constexpr double SphereRadius = 0.03;

    // Spiral trajectory angular velocity in [rad/s]
    constexpr double RadialVelocity = 2.0 * M_PI;

    // Spiral trajectory vertical velocity in [m/s]
    constexpr double VerticalVelocity = M_PI / 10.0;

    // Move to the top of the sphere.
    if (drdMoveToPos(SphereCenterX, SphereCenterY, SphereCenterZ + SphereRadius) < 0)
    {
        std::cout << "error: failed to move to the top of the sphere (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Realign the wrist in its nominal position.
    if (drdMoveToRot(0.0, 0.0, 0.0) < 0)
    {
        std::cout << "error: failed to center the wrist (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable the trajectory interpolator and adjust its parameters.
    if (drdEnableFilter(true) < 0)
    {
        std::cout << "error: failed to enable track filter (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    if (drdSetPosTrackParam(2.0, 1.0, -1.0) < 0)
    {
        std::cout << "error: failed to set tracking motion parameters (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Start tracking a trajectory on the surface of the sphere.
    // A new point along the trajectory is sent to the robotic controller at about 20 Hz,
    // but thanks to the trajectory interpolator, the movement is smooth.
    double timeStart = dhdGetTime();
    while (true)
    {
        // Arbitrarily throttle the update of the target to 20 Hz.
        dhdSleep(0.05);

        // generate next sphere target point
        double dt = drdGetTime() - timeStart;
        double alpha = RadialVelocity * dt;
        double beta = M_PI / 2.0 + (VerticalVelocity * dt);
        double targetX = SphereCenterX + SphereRadius * cos(alpha) * cos(beta);
        double targetY = SphereCenterY + SphereRadius * sin(alpha) * cos(beta);
        double targetZ = SphereCenterZ + SphereRadius * sin(beta);
        if (drdTrackPos(targetX, targetY, targetZ) < 0)
        {
            std::cout << "error: failed to track target (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Make sure that the regulation thread is still running.
        if (!drdIsRunning())
        {
            std::cout << "error: regulation thread not running" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Evaluate exit conditions.
        if (beta > 5.0 * M_PI / 2.0)
        {
            std::cout << std::endl << std::endl << "robotic operation succeeded" << std::endl;
            break;
        }
        if (dhdKbHit() && dhdKbGet() == 'a')
        {
            std::cout << std::endl << std::endl << "aborting at user's request" << std::endl;
            return 0;
        }

        // Periodically display the regulation performance and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << "regulation thread running at " << std::fixed << std::setprecision(2) << std::setw(4) << drdGetCtrlFreq() << " kHz \r";
            std::cout.flush();
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
