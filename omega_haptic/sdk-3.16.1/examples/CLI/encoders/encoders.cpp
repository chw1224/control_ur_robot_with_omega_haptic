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
/// low-level encoders using the expert functions of the SDK. If the haptic
/// device is initialized, it places it in a gravity compensation loop.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <iomanip>
#include <iostream>

// Force Dimension SDK library header
#include "dhdc.h"

///////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char *argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Encoder Reading Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Enable expert mode to allow access to the encoder values.
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

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl << std::endl;

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Determine the number of encoders to display based on the device capabilities.
    int numEncoders = 3;
    if (dhdHasWrist())
    {
        numEncoders += 3;
    }
    if (dhdHasWrist())
    {
        numEncoders += 1;
    }

    // Allocate and initialize the encoders buffer.
    int encoders[DHD_MAX_DOF] = {};
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

        // Retrieve encoders.
        if (dhdGetEnc(encoders) < 0)
        {
            std::cout << "error: failed to read encoders (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Periodically the haptic device encoders and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;

            std::cout << std::internal << std::showpos << std::setfill('0');
            for (int index = 0; index < numEncoders; index++)
            {
                std::cout << std::setw(7) << encoders[index] << " ";
            }
            std::cout << "  \r";
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
