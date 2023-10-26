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
/// This example implements a simple teleoperation control loop.
/// The robot movement is relative to the haptic device location when the user
/// button on the haptic device is held pressed. Alternatively, the space bar
/// can be used to toggle the teleoperation state between engaged and
/// disengaged.
///
/// The teleoperation logic is encapsulated in the \ref
/// teleoperationControlLoop() function. Motion scaling, haptic stiffness and
/// damping can be modified at build time by changing the values in the \ref
/// teleoperationControlLoop() function. The rest of the code handles mostly
/// the graphic display and the limited keyboard interaction.
///
/// The haptic feedback conveys the limits of the dummy robot workspace defined
/// in the Robot.h header. The dummy robot \ref Robot::setTarget()
/// implementation contains simple workspace limits to provide linear and
/// angular limits that the teleoperation loop uses to compute haptic feedback.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>

// Platform-specific headers
#if defined(WIN32) | defined(WIN64)
#include <windows.h>
#endif

// FD SDK library headers
#include "drdc.h"

// GLFW library headers
#include <GLFW/glfw3.h>

// GLU headers
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif

// Project headers
#include "Robot.h"
#include "Transform.h"

/// Global simulation running flag
std::atomic<bool> simulationRunning = true;

/// Global teleoperation engagement override
std::atomic<bool> engageOverride = false;

/// Global master haptic input device transform
Transform masterCurrent;

/// Global slave robot target transform
Transform slaveTarget;

/// Global slave robot current transform
Transform slaveCurrent;

/// Global dummy robot object
Robot slaveRobot;

////////////////////////////////////////////////////////////////////////////////
///
/// This function renders the scene using OpenGL.
///
////////////////////////////////////////////////////////////////////////////////

int updateGraphics()
{
    // Rendering settings
    constexpr float HapticSphereRadius = 0.01f;
    constexpr float TargetSphereRadius = 0.005f;
    constexpr float RobotSphereRadius = 0.01f;
    constexpr float HapticFrameSize = 0.04f;
    constexpr float TargetFrameSize = 0.02f;
    constexpr float RobotFrameSize = 0.04f;

    // Initialize rendering settings.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 1.0);

    // Declare sphere with frame rendering lambda.
    auto renderSphere = [](double* a_transform,
                           float a_red, float a_green, float a_blue,
                           float a_radius,
                           float a_frameSize)
    {
        glPushMatrix();
        glMultMatrixd(a_transform);
        glEnable(GL_COLOR_MATERIAL);
        glColor3f(a_red, a_green, a_blue);
        GLUquadricObj* sphere = gluNewQuadric();
        gluSphere(sphere, a_radius, 32, 32);
        gluDeleteQuadric(sphere);
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3d(0.00, 0.00, 0.00);
        glVertex3d(a_frameSize, 0.00, 0.00);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3d(0.00, 0.00, 0.00);
        glVertex3d(0.00, a_frameSize, 0.00);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3d(0.00, 0.00, 0.00);
        glVertex3d(0.00, 0.00, a_frameSize);
        glEnd();
        glEnable(GL_LIGHTING);
        glPopMatrix();
    };

    // Display a sphere and frame for the master haptic device.
    renderSphere(masterCurrent.matrix().data(), 0.5f, 0.5f, 0.5f, HapticSphereRadius, HapticFrameSize);

    // Display a smaller sphere and frame for the slave robot target.
    renderSphere(slaveTarget.matrix().data(), 0.2f, 0.2f, 0.2f, TargetSphereRadius, TargetFrameSize);

    // Display a sphere and frame for the slave robot.
    renderSphere(slaveCurrent.matrix().data(), 0.5f, 0.5f, 0.5f, RobotSphereRadius, RobotFrameSize);

    // Wait for all rendering operations to complete.
    glFinish();

    // Report any issue.
    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
    {
        return -1;
    }

    // Report success.
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void teleoperationControlLoop(int a_deviceId)
{
    /// Scaling factor between master translation and slave translation.
    /// A value greater than 1.0 means that the slave's movement will be
    /// larger than the master's.
    constexpr double LinearScaling = 0.5;

    /// Scaling factor between master rotation and slave rotation.
    /// A value greater than 1.0 means that the slave's movement will be
    /// larger than the master's.
    constexpr double AngularScaling = 0.5;

    /// Master linear damping when teleoperation is engaged in [N/(m/s)]
    constexpr double LinearDamping = 10.0;

    /// Master angular damping when teleoperation is engaged in [Nm/(rad/s)]
    constexpr double AngularDamping = 0.01;

    /// Linear stiffness of the virtual spring between desired and actual slave positions in [N/m].
    /// We normalize it by the linear scaling so that the apparent stiffness (from the haptic device's
    /// user perspective) remains the same, regardless of the movement scaling.
    constexpr double LinearStiffness = 4000.0 / LinearScaling;

    /// Angular stiffness of the virtual spring between desired and actual slave positions in [Nm/rad].
    /// We normalize it by the angular scaling so that the apparent stiffness (from the haptic device's
    /// user perspective) remains the same, regardless of the movement scaling.
    constexpr double AngularStiffness = 4.0 / AngularScaling;

    std::cout << "teleoperation thread running" << std::endl;

    // Declare state variables used to retrieve data from the master haptic device.
    double masterX = 0.0;
    double masterY = 0.0;
    double masterZ = 0.0;
    double masterRotation[3][3] = {};
    double masterLinearVelocityX = 0.0;
    double masterLinearVelocityY = 0.0;
    double masterLinearVelocityZ = 0.0;
    double masterAngularVelocityX = 0.0;
    double masterAngularVelocityY = 0.0;
    double masterAngularVelocityZ = 0.0;

    // Declare all teleoperation control variables.
    bool teleoperationEngaged = false;
    Eigen::Vector3d masterStartPosition;
    Eigen::Matrix3d masterStartRotation;
    Eigen::Vector3d slaveStartPosition;
    Eigen::Matrix3d slaveStartRotation;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
    Eigen::AngleAxisd previousMasterAngularMove;

    // Stop robotic regulation of the master haptic device, but leave it with forces enabled.
    if (drdStop(true, a_deviceId) < 0)
    {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        simulationRunning = false;
        return;
    }

    // Enable button emulation on devices featuring a gripper.
    if (dhdHasActiveGripper(a_deviceId) && dhdEmulateButton(DHD_ON, a_deviceId) < 0)
    {
        std::cout << "error: failed to enable button emulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        simulationRunning = false;
        return;
    }

    // Teleoperation control loop
    while (simulationRunning)
    {
        // Retrieve the haptic device position and velocity.
        if (dhdGetPositionAndOrientationFrame(&masterX, &masterY, &masterZ, masterRotation, a_deviceId) < 0)
        {
            std::cout << "error: failed to get haptic position (" << dhdErrorGetLastStr() << ")" << std::endl;
            simulationRunning = false;
            break;
        }
        if (dhdGetLinearVelocity(&masterLinearVelocityX, &masterLinearVelocityY, &masterLinearVelocityZ, a_deviceId) < 0)
        {
            std::cout << "error: failed to retrieve master linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            simulationRunning = false;
            break;
        }
        if (dhdGetAngularVelocityRad(&masterAngularVelocityX, &masterAngularVelocityY, &masterAngularVelocityZ, a_deviceId) < 0)
        {
            std::cout << "error: failed to retrieve master angular velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            simulationRunning = false;
            break;
        }

        // Put master haptic device state into useful control variables.
        masterCurrent.set(masterX, masterY, masterZ, masterRotation);
        Eigen::Vector3d masterLinearVelocity { masterLinearVelocityX, masterLinearVelocityY, masterLinearVelocityZ };
        Eigen::Vector3d masterAngularVelocity { masterAngularVelocityX, masterAngularVelocityY, masterAngularVelocityZ };

        // Update the slave robot current position.
        slaveCurrent = slaveRobot.current();

        // Read the status of the user button.
        bool buttonEngaged = (dhdGetButton(0) != DHD_OFF) || engageOverride;

        // Handle teloperation engage/disengage events.
        if (buttonEngaged && !teleoperationEngaged)
        {
            // Store the current haptic device position as a reference starting point for teleoperation.
            masterStartPosition = masterCurrent.position();
            masterStartRotation = masterCurrent.rotation();

            // Store the current robot position as a reference starting point for teleoperation.
            slaveStartPosition = slaveCurrent.position();
            slaveStartRotation = slaveCurrent.rotation();

            // Engage teloperation.
            teleoperationEngaged = true;
        }
        else if (!buttonEngaged && teleoperationEngaged)
        {
            // Disengage teleoperation.
            teleoperationEngaged = false;
        }

        // If teleoperation is not engaged, we set no force and no torque on the device.
        if (!teleoperationEngaged)
        {
            force.setZero();
            torque.setZero();
        }

        // If teleoperation is engaged, reproduce the relative movement of the master (with respect
        // to its starting point of reference) on the slave (with respect to its starting point of
        // reference), applying motion scaling along the way.
        else
        {
            // Compute the slave linear movement to match the master's, with scaling.
            Eigen::Vector3d masterLinearMove = masterCurrent.position() - masterStartPosition;
            Eigen::Vector3d slaveLinearMove = LinearScaling * masterLinearMove;

            // Compute the slave angular movement to match the master's. We also prevent
            // the angle to "flip" around to the other hemisphere to avoid control discontinuities.
            Eigen::AngleAxisd masterAngularMove = Transform::angleAxis(masterCurrent.rotation() * masterStartRotation.transpose());
            Eigen::AngleAxisd masterAngularMoveNoFlip = Transform::noFlip(masterAngularMove, previousMasterAngularMove);
            Eigen::Matrix3d slaveAngularMove { Eigen::AngleAxisd { AngularScaling * masterAngularMoveNoFlip.angle(), masterAngularMoveNoFlip.axis() } };

            // Compute the slave absolute position by adding its relative movement to its point of reference.
            slaveTarget.setPosition(slaveStartPosition + slaveLinearMove);
            slaveTarget.setRotation(slaveAngularMove * slaveStartRotation);

            // Send the desired target position to the slave robot.
            slaveRobot.setTarget(slaveTarget);

            // Compute the virtual linear spring force.
            Eigen::Vector3d linearSpringLength = slaveTarget.position() - slaveCurrent.position();
            Eigen::Vector3d linearSpringForce = -1.0 * LinearStiffness * linearSpringLength;

            // Compute the virtual angular spring torque.
            Eigen::Matrix3d angularSpringLength = slaveTarget.rotation() * slaveCurrent.rotation().transpose();
            Eigen::Vector3d angularSpringTorque = -1.0 * AngularStiffness * Transform::angles(angularSpringLength);

            // Compute the master's angular move limit torque (caused by the "no flipping" condition).
            Eigen::Matrix3d masterSpringLength = Eigen::Matrix3d { masterAngularMove } * Eigen::Matrix3d { masterAngularMoveNoFlip }.transpose();
            Eigen::Vector3d masterSpringTorque = -1.0 * AngularStiffness * Transform::angles(masterSpringLength);

            // Compute the damping force.
            Eigen::Vector3d dampingForce = -1.0 * LinearDamping * masterLinearVelocity;

            // Compute damping torque.
            Eigen::Vector3d dampingTorque = -1.0 * AngularDamping * masterAngularVelocity;

            // Combine all force and torque contributions.
            force = linearSpringForce + dampingForce;
            torque = angularSpringTorque + masterSpringTorque + dampingTorque;
        }

        // Send force to the haptic device.
        if (dhdSetForceAndTorqueAndGripperForce(force(0), force(1), force(2), torque(0), torque(1), torque(2), 0.0, a_deviceId) < 0)
        {
            std::cout << "error: failed to set haptic force (" << dhdErrorGetLastStr() << ")" << std::endl;
            simulationRunning = false;
            break;
        }
    }

    // Turn all forces off on exit.
    dhdEnableForce(DHD_OFF, a_deviceId);

    // Let everyone know that we are stopping.
    simulationRunning = false;
    std::cout << "teleoperation thread stopped" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function is called when the GLFW window is resized.
///
/// \note See GLFW documentation for a description of the parameters.
///
////////////////////////////////////////////////////////////////////////////////

void onWindowResized(GLFWwindow* a_window,
                     int a_width,
                     int a_height)
{
    double glAspect = (static_cast<double>(a_width) / static_cast<double>(a_height));

    glViewport(0, 0, a_width, a_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, glAspect, 0.01, 10);
    gluLookAt(0.2, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function is called when a key is pressed in the GLFW window.
///
/// \note See GLFW documentation for a description of the parameters.
///
////////////////////////////////////////////////////////////////////////////////

void onKeyPressed(GLFWwindow* a_window,
                  int a_key,
                  int a_scanCode,
                  int a_action,
                  int a_modifiers)
{
    // Ignore all keyboard events that are not key presses.
    if (a_action != GLFW_PRESS)
    {
        return;
    }

    // Map keys to events and process accordingly.
    switch (a_key)
    {
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
        {
            simulationRunning = false;
            break;
        }

        case GLFW_KEY_SPACE:
        {
            engageOverride = !engageOverride;
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function is called when the GLFW library reports an error.
///
/// \note See GLFW documentation for a description of the parameters.
///
////////////////////////////////////////////////////////////////////////////////

void onError(int a_error,
             const char* a_description)
{
    std::cout << "error: " << a_description << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void userInterfaceLoop()
{
    // Initialize GLFW library.
    if (!glfwInit())
    {
        std::cout << "error: failed to initialized GLFW" << std::endl;
        simulationRunning = false;
        return;
    }

    // GLFW window handle
    GLFWwindow *window = NULL;

    // Set error callback.
    glfwSetErrorCallback(onError);

    // Compute desired size of window.
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int width = static_cast<int>(0.8 * mode->height);
    int height = static_cast<int>(0.5 * mode->height);
    int x = static_cast<int>(0.5 * (mode->width - width));
    int y = static_cast<int>(0.5 * (mode->height - height));

    // Configure OpenGL rendering.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_STEREO, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

    // Apply platform-specific settings.
#ifdef MACOSX

    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);

#endif

    // Create window and display context.
    window = glfwCreateWindow(width, height, "Force Dimension - Virtual Teleoperation Example", NULL, NULL);
    if (!window)
    {
        std::cout << "error: failed create GLFW window" << std::endl;
        simulationRunning = false;
        return;
    }

    // Configure window.
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, onKeyPressed);
    glfwSetWindowSizeCallback(window, onWindowResized);
    glfwSetWindowPos(window, x, y);
    glfwSwapInterval(1);
    glfwShowWindow(window);

    // Adjust initial window size.
    onWindowResized(window, width, height);

    // set material properties
    GLfloat mat_ambient[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_diffuse[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

    // set light source
    GLfloat ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0);
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0);

    GLfloat lightPos[] = { 2.0, 0.0, 0.0, 1.0f };
    GLfloat lightDir[] = { -1.0, 0.0, 0.0, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, lightDir);
    glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 180);
    glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 1.0);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Main graphic loop
    while (simulationRunning && !glfwWindowShouldClose(window))
    {
        // Throttle rendering at 40 Hz.
        std::this_thread::sleep_for(std::chrono::milliseconds(25));

        updateGraphics();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Close window.
    glfwDestroyWindow(window);

    // Terminate GLFW library.
    glfwTerminate();

    // Ask teleoperation to stop as well.
    simulationRunning = false;
}

////////////////////////////////////////////////////////////////////////////////

int main()
{
    // Display information.
    std::cout << std::endl;
    std::cout << "Force Dimension - Virtual Teleoperation Example " << dhdGetSDKVersionStr() << ")" << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Open haptic device.
    int deviceId = drdOpen();
    if (deviceId < 0)
    {
        std::cout << "error: failed to open haptic device (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Get haptic device serial number.
    uint16_t serialNumber = 0;
    if (dhdGetSerialNumber(&serialNumber, deviceId) < 0)
    {
        std::cout << "warning: failed to retrieve device serial number (" << dhdErrorGetLastStr() << ")" << std::endl;
    }

    // Initialize haptic device if required.
    std::cout << "initializing haptic device...\r";
    std::cout.flush();
    if (!drdIsInitialized(deviceId) && drdAutoInit(deviceId) < 0)
    {
        std::cout << "error: failed to initialize device (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Identify the device.
    std::cout << "connected to " << dhdGetSystemName(deviceId) << " S/N " << std::setw(5) << std::setfill('0') << serialNumber << std::endl;

    // Start robotic regulation if required.
    if (!drdIsRunning(deviceId) && drdStart(deviceId) < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Display instructions.
    std::cout << std::endl;
    std::cout << "Hold the user button on the haptic device to engage teleoperation, or" << std::endl;
    std::cout << "press ' ' in the UI window to engage/disengage teleoperation, or" << std::endl;
    std::cout << "press 'q' in the UI window to quit." << std::endl;
    std::cout << std::endl;

    // Align the master haptic device with the slave robot initial position.
    // In this example, the slave robot initial position is located at the center of the workspace.
    double initialPosition[DHD_MAX_DOF] = {};
    if (drdMoveTo(initialPosition, true, deviceId) < 0)
    {
        std::cout << "error: failed to align master with slave (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Start the teleoperation thread.
    std::thread teleoperationThread(teleoperationControlLoop, deviceId);

    // Run the user interface.
    userInterfaceLoop();

    // Wait for both threads to terminate.
    teleoperationThread.join();

    // Close the connection to the haptic device.
    if (drdClose(deviceId) < 0)
    {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Report success.
    std::cout << "connection closed" << std::endl;
    return 0;
}
