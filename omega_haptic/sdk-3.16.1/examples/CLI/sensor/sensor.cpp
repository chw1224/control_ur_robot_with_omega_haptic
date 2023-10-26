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
/// This example illustrates how to use the Force Dimension robotic SDK to turn
/// a Force Dimension haptic device into a force sensor.
///
/// It uses the robotic SDK to keep the haptic device end-effector at the center
/// of the workspace, and retrieve the regulation force applied by the robotic
/// regulation thread to measure the force applied at the device's end-effector.
/// The stiffness of the robotic regulation constraint can be adjusted by the
/// user using keyboard input.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "drdc.h"

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char* argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Force Sensor Emulation Example " << dhdGetSDKVersionStr() << std::endl;
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

    // Disable the integral term of the PID (we do not need precise positioning).
    if (drdSetEncIGain(0.0) < 0)
    {
        std::cout << "error: failed to disable integral term (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display user instructions.
    std::cout << "press 'z' to zero the sensor" << std::endl;
    std::cout << "      'k/K' to decrease/increase the sensor stiffness Kp" << std::endl;
    std::cout << "      'q' to quit demo" << std::endl << std::endl;

    // Allocate and initialize control loop variables.
    double Kp = drdGetEncPGain();
    const double MaxKp = 1.5 * Kp;
    double fxOffset = 0.0;
    double fyOffset = 0.0;
    double fzOffset = 0.0;
    double lastDisplayUpdateTime = dhdGetTime();

    // Prepare UI layout.
    std::cout << "  Kp  |  force    |  fx    fy    fz" << std::endl;
    std::cout << "------|-----------|----------------------" << std::endl;

    // Continuously measure and display the force applied by the device to maintain
    // the current position, excluding gravity compensation.
    bool running = true;
    while (running)
    {
        // Synchronize with the regulation thread.
        drdWaitForTick();

        // Retrieve the force applied by the device (exluding gravity compensation).
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        if (dhdGetForce(&fx, &fy, &fz) < 0)
        {
            std::cout << "error: failed to disable integral term (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Compensate for the sensor zero.
        double fxSensor = fx - fxOffset;
        double fySensor = fy - fyOffset;
        double fzSensor = fz - fzOffset;
        double f = std::sqrt(fxSensor * fxSensor + fySensor * fySensor + fzSensor * fzSensor);

        // Periodically display the force measured and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << std::noshowpos << std::fixed << std::setprecision(2) << std::setw(5) << Kp << " | ";
            std::cout << std::showpos << std::setprecision(2) << std::setw(5) << f << " [N] |";
            std::cout << std::showpos << std::setprecision(2) << std::setw(6) << fxSensor << " " << fxSensor << " " << fzSensor << " [N]  \r";
            std::cout.flush();
        }

        // Make sure that the regulation thread is still running.
        if (!drdIsRunning())
        {
            std::cout << "error: regulation thread not running" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Store the sensor current value as the zero offset.
                case 'z':
                {
                    fxOffset = fx;
                    fyOffset = fy;
                    fzOffset = fz;
                    break;
                }

                // Decrease the sensor stiffness.
                case 'k':
                {
                    Kp = std::max(0.0, Kp - 0.1);
                    break;
                }

                // Increase the sensor stiffness.
                case 'K':
                {
                    Kp = std::min(MaxKp, Kp + 0.1);
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

            // Apply Kp in case it was modified.
            if (drdSetEncPGain(Kp) < 0)
            {
                std::cout << "error: failed to change sensor stiffness (" << dhdErrorGetLastStr() << ")" << std::endl;
                dhdSleep(2.0);
                break;
            }
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
