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
/// This example illustrates how to write a simple haptic control loop that
/// places a Force Dimension haptic device in gravity compensation.
///
/// It serves as the "hello world" application of the Force Dimension SDK, and
/// can be used as a starting point to develop haptic applications.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "dhdc.h"

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char *argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Gravity Compensation Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Open the first available haptic device.
    if (dhdOpen() < 0)
    {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display the device type.
    std::cout << dhdGetSystemName() << " device detected" << std::endl << std::endl;

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl << std::endl;

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Allocate and initialize haptic loop variables.
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double lastDisplayUpdateTime = dhdGetTime();

    // Run the haptic loop.
    while (true)
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
        if (dhdKbHit() && dhdKbGet() == 'q')
        {
            std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
            break;
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
