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
/// This example illustrates how to write a simple teleoperation application
/// using the Force Dimension SDK.
/// It requires two Force Dimension haptic devices:
///
/// 1. the first haptic device acts as a master device and is held by the user
///
/// 2. the second haptic device serves as a slave and reproduces the scaled down
///    movement of the master device.
///
/// To engage the teleoperation, the user needs to press and hold the user
/// button on the master device's end-effector. Any force applied to the slave
/// device is rendered as haptic feedback on the master device.
///
/// The movement and force scaling factors are defined and documented in the
/// haptic loop and can be adjusted to modify the behavior of the application.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <cmath>
#include <iostream>
#include <iomanip>

// Force Dimension SDK library header
#include "drdc.h"

////////////////////////////////////////////////////////////////////////////////

int main()
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    // Display information.
    std::cout << std::endl;
    std::cout << "Mirror Teleoperation Example " << dhdGetSDKVersionStr() << ")" << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Make sure there are enough haptic devices connected.
    int deviceCount = dhdGetAvailableCount();
    if (deviceCount < 2)
    {
        std::cout << "error: not enough devices available" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Open the first available haptic device as the master.
    int masterId = drdOpen();
    if (masterId < 0)
    {
        std::cout << "error: failed to open master device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Open the second available haptic device as the slave.
    int slaveId = drdOpen();
    if (slaveId < 0)
    {
        std::cout << "error: failed to open slave device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Start regulation on the slave robot.
    if (drdStart(slaveId) < 0)
    {
        std::cout << "error: failed to start robotic regulation on the slave device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move the slave device to the center of its workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter, true, slaveId) < 0)
    {
        std::cout << "error: failed to move slave device to the center of its workspace (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable force on the master haptic device.
    if (dhdEnableForce(DHD_ON, masterId) < 0)
    {
        std::cout << "error: failed to enable force on master device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable button emulation on the master device if available.
    if (dhdHasGripper(masterId) && dhdEmulateButton(DHD_ON, masterId) < 0)
    {
        std::cout << "error: failed to enable button emulation on master device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Display the device type.
    std::cout << "using " << dhdGetSystemName(masterId) << " as the master device" << std::endl;
    std::cout << "using " << dhdGetSystemName(slaveId) << " as the slave device" << std::endl << std::endl;

    // Display instructions.
    std::cout << "hold BUTTON to engage teleoperation, or" << std::endl;
    std::cout << "press [space] to engage/disengage teleoperation, or" << std::endl;
    std::cout << "press 'q' to quit." << std::endl << std::endl;

    // Declare state variables used to retrieve data from the master haptic device.
    bool teleoperationEngaged = false;
    bool engageOverride = false;
    double pxMasterOrigin = 0.0;
    double pyMasterOrigin = 0.0;
    double pzMasterOrigin = 0.0;
    double pxSlaveOrigin = 0.0;
    double pySlaveOrigin = 0.0;
    double pzSlaveOrigin = 0.0;

    // Teleoperation control loop
    bool running = true;
    while (running)
    {
        // Retrieve the master haptic device position.
        double pxMaster = 0.0;
        double pyMaster = 0.0;
        double pzMaster = 0.0;
        if (dhdGetPosition(&pxMaster, &pyMaster, &pzMaster, masterId) < 0)
        {
            std::cout << "error: failed to get master position (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }

        // Retrieve the master haptic device velocity.
        double vxMaster = 0.0;
        double vyMaster = 0.0;
        double vzMaster = 0.0;
        if (dhdGetLinearVelocity(&vxMaster, &vyMaster, &vzMaster, masterId) < 0)
        {
            std::cout << "error: failed to retrieve master linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }

        // Retrieve the slave device position.
        double pxSlave = 0.0;
        double pySlave = 0.0;
        double pzSlave = 0.0;
        if (dhdGetPosition(&pxSlave, &pySlave, &pzSlave, slaveId) < 0)
        {
            std::cout << "error: failed to get slave position (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }

        // Read the status of the user button.
        bool buttonEngaged = (dhdGetButton(0, masterId) != DHD_OFF) || engageOverride;

        // Handle teloperation engage/disengage events.
        if (buttonEngaged && !teleoperationEngaged)
        {
            // Store the current haptic device position as a reference starting point for teleoperation.
            pxMasterOrigin = pxMaster;
            pyMasterOrigin = pyMaster;
            pzMasterOrigin = pzMaster;

            // Store the current robot position as a reference starting point for teleoperation.
            pxSlaveOrigin = pxSlave;
            pySlaveOrigin = pySlave;
            pzSlaveOrigin = pzSlave;

            // Engage teloperation.
            teleoperationEngaged = true;

            std::cout << "teleoperation engaged     \r";
        }
        else if (!buttonEngaged && teleoperationEngaged)
        {
            // Disengage teleoperation.
            teleoperationEngaged = false;

            std::cout << "teleoperation disengaged  \r";
        }

        // By default, we set no force and no torque on the device.
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;

        // If teleoperation is engaged, reproduce the relative movement of the master (with respect
        // to its starting point of reference) on the slave (with respect to its starting point of
        // reference), applying motion scaling along the way.
        if (teleoperationEngaged)
        {
            // Scaling factor between master translation and slave translation.
            // A value greater than 1.0 means that the slave's movement will be
            // larger than the master's.
            constexpr double LinearScaling = 0.5;

            // Master linear damping when teleoperation is engaged in [N/(m/s)]
            constexpr double LinearDamping = 2.0;

            // Linear stiffness of the virtual spring between desired and actual slave positions in [N/m].
            // We normalize it by the linear scaling so that the apparent stiffness (from the haptic device's
            // user perspective) remains the same, regardless of the movement scaling.
            constexpr double LinearStiffness = 1000.0 / LinearScaling;

            // Compute the slave linear movement to match the master's, with scaling.
            double pxSlaveDesiredTarget = pxSlaveOrigin + LinearScaling * (pxMaster - pxMasterOrigin);
            double pySlaveDesiredTarget = pySlaveOrigin + LinearScaling * (pyMaster - pyMasterOrigin);
            double pzSlaveDesiredTarget = pzSlaveOrigin + LinearScaling * (pzMaster - pzMasterOrigin);

            // Slave spherical workspace radius in [m].
            constexpr double SlaveWorkspaceRadius = 0.04;

            // Keep the slave within a safe workspace.
            double pxSlaveTarget = pxSlaveDesiredTarget;
            double pySlaveTarget = pySlaveDesiredTarget;
            double pzSlaveTarget = pzSlaveDesiredTarget;
            double slaveDistanceFromCenter = std::sqrt(pxSlaveDesiredTarget * pxSlaveDesiredTarget + pySlaveDesiredTarget * pySlaveDesiredTarget + pzSlaveDesiredTarget * pzSlaveDesiredTarget);
            if (slaveDistanceFromCenter > SlaveWorkspaceRadius)
            {
                double slaveScalingFactor = SlaveWorkspaceRadius / slaveDistanceFromCenter;
                pxSlaveTarget *= slaveScalingFactor;
                pySlaveTarget *= slaveScalingFactor;
                pzSlaveTarget *= slaveScalingFactor;
            }

            // Send the desired target position to the slave robot.
            if (drdTrackPos(pxSlaveTarget, pySlaveTarget, pzSlaveTarget, slaveId) < 0)
            {
                std::cout << "error: failed to set slave tracking target (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }

            // Compute the virtual linear spring force.
            double fxSpring = -1.0 * LinearStiffness * (pxSlaveDesiredTarget - pxSlave);
            double fySpring = -1.0 * LinearStiffness * (pySlaveDesiredTarget - pySlave);
            double fzSpring = -1.0 * LinearStiffness * (pzSlaveDesiredTarget - pzSlave);

            // Compute the damping force.
            double fxDamping = -1.0 * LinearDamping * vxMaster;
            double fyDamping = -1.0 * LinearDamping * vyMaster;
            double fzDamping = -1.0 * LinearDamping * vzMaster;

            // Combine all force and torque contributions.
            fx = fxSpring + fxDamping;
            fy = fySpring + fyDamping;
            fz = fzSpring + fzDamping;
        }

        // Send force to the haptic device.
        if (dhdSetForceAndTorqueAndGripperForce(fx, fy, fz, 0.0, 0.0, 0.0, 0.0, masterId) < 0)
        {
            std::cout << "error: failed to set haptic force (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }

        // Make sure that the regulation thread is still running.
        if (!drdIsRunning(slaveId))
        {
            std::cout << "error: regulation thread not running on slave device" << std::endl;
            dhdSleep(2.0);
            return -1;
        }

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Toggle teleoperation status.
                case ' ':
                {
                    engageOverride = ! engageOverride;
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

    // Close the connection to the haptic devices.
    if (drdClose(masterId) < 0)
    {
        std::cout << "error: failed to close the connection to the master device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    if (drdClose(slaveId) < 0)
    {
        std::cout << "error: failed to close the connection to the slave device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Report success.
    std::cout << "connections closed" << std::endl;
    return 0;
}
