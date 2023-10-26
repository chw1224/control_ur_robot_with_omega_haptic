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
/// This example illustrates how to write a multithreaded application that runs
/// one haptic control loop for each available Force Dimension haptic device,
/// each in its own thread.
///
/// It starts by opening a connection to all available haptic devices, and then
/// starts a gravity compensation loop in a separate thread for each of them.
/// All threads execute independently until the user requests the application
/// to exit.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <atomic>
#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

// Force Dimension SDK library header
#include "dhdc.h"

/// Global running flag shared by all threads
std::atomic<bool> s_running { true };

/// Global running threads count
std::atomic<size_t> s_runningThreadsCount { 0 };

////////////////////////////////////////////////////////////////////////////////
///
/// This function implements a simple haptic thread that puts a given haptic
/// device, identified by then handle passed as argument, into gravity
/// compensation.
///
/// \param a_arg
/// A pointer to the device ID that identifies the haptic device
///
////////////////////////////////////////////////////////////////////////////////

void* gravityCompensationThread(void* a_arg)
{
    // Increment the global running thread counter.
    s_runningThreadsCount += 1;

    // Retrieve the device index as argument.
    int deviceId = *(reinterpret_cast<int*>(a_arg));

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON, deviceId) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
        s_running = false;
    }

    // Run haptic loop.
    while (s_running)
    {
        // Apply null force.
        if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, deviceId) < DHD_NO_ERROR)
        {
            std::cout << "error: failed to set force on device id " << deviceId << " (" << dhdErrorGetLastStr() << ")" << std::endl;
            s_running = false;
            break;
        }
    }

    // Close the connection to the haptic device.
    if (dhdClose(deviceId) < 0)
    {
        std::cout << "warning: failed to close the connection on device id " << deviceId << "(" << dhdErrorGetLastStr() << ")" << std::endl;
    }

    // Decrement the global running thread counter.
    s_runningThreadsCount -= 1;

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char* argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display version information.
    std::cout << "Multithreaded Gravity Compensation Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // check for devices (limit to 10 devices)
    int deviceCount = dhdGetDeviceCount();
    if (deviceCount < 1)
    {
        std::cout << "error: no device found (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display the device type.
    std::cout << deviceCount << " device(s) detected" << std::endl;

    // Allocate resources and start threads.
    std::vector<int> deviceIdsList;
    for (int index = 0; index < deviceCount; index++ )
    {
        int deviceId = dhdOpenID(index);
        if (deviceId >= 0)
        {
            uint16_t serialNumber = 0;
            std::string serialNumberString;
            if (dhdGetSerialNumber(&serialNumber, deviceId) == DHD_NO_ERROR)
            {
                std::ostringstream serialNumberStringStream;
                serialNumberStringStream << "S/N " << serialNumber;
                serialNumberString = serialNumberStringStream.str();
            }
            std::cout << "starting gravity compensation thread for " << dhdGetSystemName() << " " << serialNumberString << std::endl;

            deviceIdsList.push_back(deviceId);
            if (dhdStartThread(gravityCompensationThread, &(deviceIdsList.back()), DHD_THREAD_PRIORITY_HIGH) < 0)
            {
                std::cout << "error: failed to start thread for " << dhdGetSystemName() << " (" << dhdErrorGetLastStr() << ")" << std::endl;
                dhdSleep(2.0);
                return -1;
            }
        }
    }

    // Wait for all threads to start.
    while (s_runningThreadsCount < deviceIdsList.size())
    {
        dhdSleep(0.1);

        // Detect a failure in any of the threads.
        if (!s_running)
        {
            std::cout << "error: at least one thread quit with an error" << std::endl;
            dhdSleep(2.0);
            return -1;
        }
    }

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl << std::endl;

    // UI thread (default priority)
    while (s_running)
    {
        // Do not hog a CPU core.
        dhdSleep(0.1);

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Quit the application.
                case 'q':
                {
                    s_running = false;
                    break;
                }
            }
        }
    }

    // Wait for all threads to finish.
    while (s_runningThreadsCount > 0)
    {
        dhdSleep(0.1);
    }

    // Report success.
    std::cout << "all threads finished" << std::endl;
    return 0;
}
