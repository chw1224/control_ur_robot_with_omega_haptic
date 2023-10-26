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
/// This example illustrates how to constrain a Force Dimension haptic device on
/// an arbitrary segment defined using the user button on the haptic device end-
/// effector.
///
/// The constraint force model parameters are defined and documented in the
/// haptic loop and can be adjusted to modify the behavior of the application.
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
/// This function computes the projection of a 'point' onto a segment defined
/// by two points 'A' and 'B'. The projection is returned as the point
/// 'projection'.
///
////////////////////////////////////////////////////////////////////////////////

void projectPointOnSegment(double a_point[3],
                           double a_A[3],
                           double a_B[3],
                           double a_projection[3])
{
    // Compute Ap vector.
    double Ap[] =
    {
     a_point[0] - a_A[0],
     a_point[1] - a_A[1],
     a_point[2] - a_A[2]
    };

    // Compute AB segment vector.
    double AB[3] =
    {
     a_B[0] - a_A[0],
     a_B[1] - a_A[1],
     a_B[2] - a_A[2]
    };

    // Compute the segment norm, and return the current input point if it is too small.
    double norm = std::sqrt(AB[0] * AB[0] + AB[1] * AB[1] + AB[2] * AB[2]);
    if (norm <= 1e-6)
    {
        a_projection[0] = a_point[0];
        a_projection[1] = a_point[1];
        a_projection[2] = a_point[2];

        return;
    }

    // Compute the segment direction unit vector.
    double direction[3] =
    {
     AB[0] / norm,
     AB[1] / norm,
     AB[2] / norm,
    };

    // Compute the projection ratio.
    double projectionRatio = Ap[0] * direction[0] + Ap[1] * direction[1] + Ap[2] * direction[2];

    // Compute the point projection on the segment.
    if (projectionRatio < 0.0)
    {
        a_projection[0] = a_A[0];
        a_projection[1] = a_A[1];
        a_projection[2] = a_A[2];
    }
    else if (projectionRatio > norm)
    {
        a_projection[0] = a_B[0];
        a_projection[1] = a_B[1];
        a_projection[2] = a_B[2];
    }
    else
    {
        a_projection[0] = a_A[0] + projectionRatio * direction[0];
        a_projection[1] = a_A[1] + projectionRatio * direction[1];
        a_projection[2] = a_A[2] + projectionRatio * direction[2];
    }
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function computes the projection of a 'force' onto a direction
/// defined by two points 'A' and 'B'. The projection is returned as the point
/// 'projectedForce'.
///
////////////////////////////////////////////////////////////////////////////////

void projectForceOnDirection(double a_force[3],
                             double a_A[3],
                             double a_B[3],
                             double a_projectedForce[3])
{
    // Compute AB segment vector.
    double AB[3] =
    {
     a_B[0] - a_A[0],
     a_B[1] - a_A[1],
     a_B[2] - a_A[2]
    };

    // Compute the segment norm, and return the current input point if it is too small.
    double norm = std::sqrt(AB[0] * AB[0] + AB[1] * AB[1] + AB[2] * AB[2]);
    if (norm <= 1e-6)
    {
        a_projectedForce[0] = 0.0;
        a_projectedForce[1] = 0.0;
        a_projectedForce[2] = 0.0;

        return;
    }

    // Compute the segment direction unit vector.
    double direction[3] =
    {
     AB[0] / norm,
     AB[1] / norm,
     AB[2] / norm,
    };

    // Compute the projection ratio.
    double projectionRatio = a_force[0] * direction[0] + a_force[1] * direction[1] + a_force[2] * direction[2];

    // Compute the force projection on the segment.
    a_projectedForce[0] = projectionRatio * direction[0];
    a_projectedForce[1] = projectionRatio * direction[1];
    a_projectedForce[2] = projectionRatio * direction[2];
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
    std::cout << "Segment Constraint Example " << dhdGetSDKVersionStr() << std::endl;
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
    std::cout << "press BUTTON or 'p' to define the segment extremities" << std::endl;
    std::cout << "                'c' to clear the current segment" << std::endl;
    std::cout << "                'q' to quit" << std::endl << std::endl;
    std::cout << "select first segment point  \r";
    std::cout.flush();

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
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

    // Allocate and initialize haptic loop variables.
    double numPoints = 0;
    double A[3] = {};
    double B[3] = {};
    double position[3] = {};
    double velocity[3] = {};
    double projectedPosition[3] = {};
    double force[3] = {};
    double projectedForce[3] = {};
    bool previousUserButton = false;

    // Run haptic loop.
    bool running = true;
    while(running)
    {
        // Retrieve the device position.
        if (dhdGetPosition(&(position[0]), &(position[1]), &(position[2])) < 0)
        {
            std::cout << "error: failed to retrieve device position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the device velocity.
        if (dhdGetLinearVelocity(&(velocity[0]), &(velocity[1]), &(velocity[2])) < 0)
        {
            std::cout << "error: failed to retrieve linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // If a segment is defined, compute the force required to keep the device on the segment.
        if (numPoints >= 2)
        {
            /// Guidance spring stiffness in [N/m]
            constexpr double Kp = 2000.0;

            /// Guidance spring damping in [N/(m/s)]
            constexpr double Kv = 20.0;

            // Compute the projection of the device position onto the segment.
            projectPointOnSegment(position, A, B, projectedPosition);

            // Compute the guidance force, modeled as a spring-damper system that pulls
            // the device towards its projection on the constraint segment.
            force[0] = Kp * (projectedPosition[0] - position[0]) - Kv * velocity[0];
            force[1] = Kp * (projectedPosition[1] - position[1]) - Kv * velocity[1];
            force[2] = Kp * (projectedPosition[2] - position[2]) - Kv * velocity[2];

            // Project the guidance force onto the vector defined by the device position and its projection;
            // this removes all unwanted force components (e.g. damping along the "free" direction).
            projectForceOnDirection(force, position, projectedPosition, projectedForce);
        }

        // If no segment is defined, apply a null force.
        else
        {
            projectedForce[0] = 0.0;
            projectedForce[1] = 0.0;
            projectedForce[2] = 0.0;
        }

        // Apply the required force.
        if (dhdSetForceAndTorqueAndGripperForce (projectedForce[0], projectedForce[1], projectedForce[2], 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR)
        {
            std::cout << "error: failed to apply forces (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Detect user button event.
        bool userButton = (dhdGetButton(0) != 0);
        bool addPoint = (!userButton && previousUserButton);
        previousUserButton = userButton;

        // Process user input.
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
                // Add a segment point.
                case 'p':
                {
                    addPoint = true;
                    break;
                }

                // Clear the current segment definition.
                case 'c':
                {
                    numPoints = 0;
                    std::cout << "select first segment point  \r";
                    std::cout.flush();
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

        // Define segment if required.
        if (addPoint)
        {
            if (numPoints == 0)
            {
                dhdGetPosition(&(A[0]), &(A[1]), &(A[2]));
                numPoints = 1;

                std::cout << "select second segment point \r";
                std::cout.flush();
            }
            else if (numPoints == 1)
            {
                dhdGetPosition(&(B[0]), &(B[1]), &(B[2]));
                numPoints = 2;

                std::cout << "segment constraint active   \r";
                std::cout.flush();
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
