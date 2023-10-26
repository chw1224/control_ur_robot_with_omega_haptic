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
/// This example illustrates how to use the robotic library to move the base,
/// wrist or gripper axes of a Force Dimension haptic device back to a given
/// position (the center of the workspace in this case).
///
/// It starts by putting the haptic device in a gravity compensation loop, and
/// engages the robotic regulation selectively on the relevant axes group (base,
/// wrist or gripper) when a keyboard input from the user requests it.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "drdc.h"

////////////////////////////////////////////////////////////////////////////////
///
/// This function centers selected axes groups on the device.
///
/// \param a_moveBase
/// If \b true, center the device base position.
/// If \b false, leave the base free to be moved by the user.
///
/// \param a_moveWrist
/// If \b true, center the device wrist rotation.
/// If \b false, leave the wrist free to be moved by the user.
///
/// \param a_moveGripper
/// If \b true, close the device gripper.
/// If \b false, leave the gripper free to be moved by the user.
///
/// \return
/// 0 on success, -1 on failure
///
////////////////////////////////////////////////////////////////////////////////

int moveToCenter(double a_moveBase,
                 double a_moveWrist,
                 double a_moveGripper)
{
    // Enable/disable base regulation.
    if (drdRegulatePos(a_moveBase) < 0)
    {
        std::cout << "error: failed to set base regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable/disable wrist regulation.
    if (dhdHasActiveWrist() && drdRegulateRot(a_moveWrist) < 0)
    {
        std::cout << "error: failed to set wrist regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable/disable gripper regulation.
    if (dhdHasActiveGripper() && drdRegulateGrip(a_moveGripper) < 0)
    {
        std::cout << "error: failed to set gripper regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Start robotic regulation.
    if (drdStart() < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move to the center of the workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter, true) < 0)
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

    return 0;
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
    std::cout << "Automatic Centering Example " << dhdGetSDKVersionStr() << std::endl;
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

    // Stop the regulation thread but leaves the forces enabled on the device.
    if (drdStop(true) < 0)
    {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl;
    std::cout << "      'c' to re-center the end-effector (all axes)" << std::endl;
    std::cout << "      'p' to re-center the base positions only" << std::endl;
    std::cout << "      'r' to re-center the wrist rotation only" << std::endl;
    std::cout << "      'g' to close the gripper only" << std::endl << std::endl;

    // Allocate and initialize haptic loop variables.
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double lastDisplayUpdateTime = dhdGetTime();

    // Run the haptic loop.
    bool running = true;
    while (running)
    {
        // Apply zero force on the haptic device.
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR)
        {
            std::cout << "error: failed to render force (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the haptic device position.
        if (dhdGetPosition(&px, &py, &pz) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the haptic device force.
        if (dhdGetForce(&fx, &fy, &fz) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read forces (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Periodically display the haptic device status and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << std::showpos << std::fixed << std::setprecision(3);
            std::cout << "p (" << std::setw(6) << px << " " << std::setw(6) << py << " " << std::setw(6) << pz << ") m  |  ";
            std::cout << "f (" << std::setw(6) << fx << " " << std::setw(6) << fy << " " << std::setw(6) << fz << ") N  |  ";
            std::cout << "freq " << std::noshowpos << std::setprecision(2) << std::setw(4) << dhdGetComFreq() << " kHz  \r";
            std::cout.flush();
        }

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Center base, wrist and gripper.
                case 'c':
                {
                    if (moveToCenter(true, true, true) < 0)
                    {
                        running = false;
                    }
                    break;
                }

                // Center the base.
                case 'p':
                {
                    if (moveToCenter(true, false, false) < 0)
                    {
                        running = false;
                    }
                    break;
                }

                // Center the wrist.
                case 'r':
                {
                    if (moveToCenter(false, true, false) < 0)
                    {
                        running = false;
                    }
                    break;
                }

                // Center the gripper.
                case 'g':
                {
                    if (moveToCenter(false, false, true) < 0)
                    {
                        running = false;
                    }
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
