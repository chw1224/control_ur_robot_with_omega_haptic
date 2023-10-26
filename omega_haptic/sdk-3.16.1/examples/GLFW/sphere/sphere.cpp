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
/// This example illustrates how to combine a haptic rendering loop with a
/// graphic (OpenGL) rendering loop for the same object, in this case a sphere.
/// The sphere object is rendered visually in a GLFW window, while a separate
/// thread haptically renders the interaction between a tool representing the
/// haptic device and the sphere.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

// Platform specific headers
#if defined(WIN32) || defined(WIN64)
#define NOMINMAX
#include "windows.h"
#endif

// GLU library headers
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif

// Force Dimension SDK library header
#include "dhdc.h"

// GLFW library header
#include <GLFW/glfw3.h>

// Project headers
#include "CMatrixGL.h"
#include "FontGL.h"

////////////////////////////////////////////////////////////////////////////////
///
/// Constants
///
////////////////////////////////////////////////////////////////////////////////

/// Position of the sphere in the scene
const Eigen::Vector3d SpherePosition(0.0, 0.0, 0.0);

/// Sphere radius in [m]
constexpr double SphereRadius = 0.03;

/// Tool size in [m[
constexpr double ToolRadius = 0.005;

/// Sphere stiffness in [N/m]
constexpr double LinearStiffness = 1000.0;

/// GLFW swap interval setting
constexpr int SwapInterval = 1;

////////////////////////////////////////////////////////////////////////////////
///
/// Global variables
///
////////////////////////////////////////////////////////////////////////////////

/// Simulation running flag
bool simulationRunning = true;

/// Simulation closing synchronization flag
bool simulationFinished = false;

/// Number of tools
int toolCount = 0;

/// Position of the tools in the scene
Eigen::Vector3d toolPosition[2];

/// Rotation available flag
bool useRotation = false;

/// Gripper available flag
bool useGripper = false;

/// Text overlay flag
bool showRefreshRate = true;

/// GLFW window handle
GLFWwindow* window = nullptr;

/// GLFW window width in [pixel]
int windowWidth = 0;

/// GLFW window height in [pixel]
int windowHeight = 0;

////////////////////////////////////////////////////////////////////////////////
///
/// This function renders the scene using OpenGL.
///
////////////////////////////////////////////////////////////////////////////////

int updateGraphics()
{
    // Clean up.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Retrieve the haptic device orientation frame (identity for 3-dof devices).
    double deviceRotation[3][3] = {};
    if (dhdGetOrientationFrame(deviceRotation) < 0)
    {
        return -1;
    }

    // Create a sphere object.
    cMatrixGL matrix;
    matrix.set(SpherePosition);
    matrix.glMatrixPushMultiply();
    GLUquadricObj* sphere = gluNewQuadric();

    // Render the sphere.
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.1f, 0.3f, 0.5f);
    gluSphere(sphere, SphereRadius, 32, 32);
    matrix.glMatrixPop();

    // Render all tools.
    for (int index = 0; index < toolCount; index++)
    {
        matrix.set(toolPosition[index], deviceRotation);
        matrix.glMatrixPushMultiply();
        sphere = gluNewQuadric();

        glColor3f(0.8f, 0.8f, 0.8f);
        gluSphere(sphere, ToolRadius, 32, 32);

        // Render a small frame for each tool.
        if (useRotation)
        {
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            glColor3f(0.45f, 0.45f, 0.45f);
            glVertex3d(0.00,  0.000,  0.000);
            glVertex3d(0.02,  0.000,  0.000);
            glVertex3d(0.02, -0.004,  0.000);
            glVertex3d(0.02,  0.004,  0.000);
            glVertex3d(0.02,  0.000, -0.004);
            glVertex3d(0.02,  0.000,  0.004);
            glEnd();
            glEnable(GL_LIGHTING);
        }

        matrix.glMatrixPop();
    }

    // Render text overlay.
    if (showRefreshRate)
    {
        static double lastFrequencyUpdateTime = dhdGetTime();
        static char frequencyString[16] = "0.000 kHz";

        // Periodically update the haptic refresh rate.
        double time = dhdGetTime();
        if (time - lastFrequencyUpdateTime > 0.1)
        {
            double frequency = dhdGetComFreq();
            lastFrequencyUpdateTime = time;
            snprintf(frequencyString, 10, "%0.03f kHz", frequency);
        }

        // Render the string.
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(0.0f, -0.01f, -0.1f);
        for (char* character = frequencyString; *character != '\0'; character++)
        {
            render_character(*character, HELVETICA12);
        }
        glEnable(GL_LIGHTING);
    }

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
///
/// This function implements the haptics loop.
///
/// \param a_userData
/// Pointer to user data passed to the haptic thread
///
/// \return
/// A pointer to a return data structure
///
////////////////////////////////////////////////////////////////////////////////

void* hapticsLoop(void* a_userData)
{
    // Allocate and initialize haptic loop variables.
    Eigen::Vector3d force;
    force.setZero();
    Eigen::Vector3d forceTool[2];
    forceTool[0].setZero();
    forceTool[1].setZero();

    // Enable force rendering on the haptic device.
    if (dhdEnableForce(DHD_ON) < 0)
    {
        std::cout << "error: failed to enable force rendering (" << dhdErrorGetLastStr() << ")" << std::endl;
        simulationRunning = false;
        return nullptr;
    }

    // Run the haptic loop.
    while (simulationRunning)
    {
        // Adapt the behavior to device capabilities.
        double px, py, pz;
        if (useGripper)
        {
            // Retrieve the haptic device position.
            if (dhdGetGripperThumbPos(&px, &py, &pz) < DHD_NO_ERROR)
            {
                std::cout << std::endl << "error: failed to read gripper thumb position (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }
            toolPosition[0] << px, py, pz;

            // Retrieve the haptic device position.
            if (dhdGetGripperFingerPos(&px, &py, &pz) < DHD_NO_ERROR)
            {
                std::cout << std::endl << "error: failed to read gripper finger position (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }
            toolPosition[1] << px, py, pz;
        }
        else
        {
            // Retrieve the haptic device position.
            if (dhdGetPosition(&px, &py, &pz) < DHD_NO_ERROR)
            {
                std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }
            toolPosition[0] << px, py, pz;
        }

        // Compute the interaction force between the sphere and each tool.
        for (int index = 0; index < toolCount; index++)
        {
            // Compute the penetration distance.
            Eigen::Vector3d direction = (toolPosition[index] - SpherePosition).normalized();
            double penetrationDistance = (toolPosition[index] - SpherePosition).norm() - SphereRadius - ToolRadius;

            // Compute the penetration force.
            if (penetrationDistance < 0.0)
            {
                forceTool[index] = -1.0 * penetrationDistance * LinearStiffness * direction;
            }
            else
            {
                forceTool[index].setZero();
            }
        }

        // Compute the force to render on the haptic device.
        double gripperForceMagnitude = 0.0;

        // If the haptic device does not have a gripper, use the current tool force.
        if (!useGripper)
        {
            force = forceTool[0];
        }

        // If the haptic device has a gripper, compute the projected force on each gripper finger.
        else
        {
            // Compute the total force.
            force = forceTool[0] + forceTool[1];
            Eigen::Vector3d gripperDirection = toolPosition[1] - toolPosition[0];

            // If the total force is not null, project it on both gripper fingers.
            if (gripperDirection.norm() > 0.00001)
            {
                // Project the mobile gripper finger force (forceGlobal[1]) onto the gripper opening vector (gripperDirection).
                gripperDirection.normalize ();
                Eigen::Vector3d gripperForce = (forceTool[1].dot(gripperDirection) / (gripperDirection.squaredNorm())) * gripperDirection;
                gripperForceMagnitude = gripperForce.norm();

                // Compute the direction of the force based on the angle between the gripper
                // force vector (gripperForce) and the gripper opening vector (gripperDirection)
                if (force.norm() > 0.001)
                {
                    double cosAngle = gripperDirection.dot(gripperForce) / (gripperDirection.norm() * gripperForce.norm());
                    cosAngle = std::min(1.0, cosAngle);
                    cosAngle = std::max(-1.0, cosAngle);
                    double angle = acos(cosAngle);
                    if ((angle > M_PI / 2.0) || (angle < -M_PI / 2.0))
                    {
                        gripperForceMagnitude = -gripperForceMagnitude;
                    }
                }
            }

            // Invert the gripper force direction for left-handed devices.
            if (dhdIsLeftHanded())
            {
                gripperForceMagnitude = -gripperForceMagnitude;
            }
        }

        // Only enable haptic rendering once the device is in free space.
        static bool safeToRenderHaptics = false;
        if (!safeToRenderHaptics)
        {
            if (force.norm() == 0.0 && gripperForceMagnitude == 0.0)
            {
                safeToRenderHaptics = true;
            }
            else
            {
                force.setZero();
                gripperForceMagnitude = 0.0;
            }
        }

        // Apply all forces at once.
        if (dhdSetForceAndGripperForce(force(0), force(1), force(2), gripperForceMagnitude) < 0)
        {
            std::cout << "error: failed to render force (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }
    }

    // Flag the simulation as having finished.
    simulationRunning = false;
    simulationFinished = true;
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function is called when the application exits.
///
////////////////////////////////////////////////////////////////////////////////

void onExit()
{
    // Wait for the haptic loop to finish.
    simulationRunning = false;
    while (!simulationFinished)
    {
        dhdSleep(0.1);
    }

    // Close the connection to the haptic device.
    if (dhdClose() < 0)
    {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        return;
    }

    // Report success.
    std::cout << "connection closed" << std::endl;
    return;
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
    windowWidth = a_width;
    windowHeight = a_height;
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

    // Detect exit requests.
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        exit(0);
    }

    // Toggle the display of the refresh rate.
    if (a_key == GLFW_KEY_R)
    {
        showRefreshRate = !showRefreshRate;
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
///
/// This function initializes the GLFW window.
///
/// \return
/// 0 on success, -1 on failure.
///
////////////////////////////////////////////////////////////////////////////////

int initializeGLFW()
{
    // Initialize the GLFW library.
    if (!glfwInit())
    {
        return -1;
    }

    // Set the error callback.
    glfwSetErrorCallback(onError);

    // Compute the desired window size.
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    windowWidth = static_cast<int>(0.8 *  mode->height);
    windowHeight = static_cast<int>(0.5 * mode->height);
    int x = static_cast<int>(0.5 * (mode->width - windowWidth));
    int y = static_cast<int>(0.5 * (mode->height - windowHeight));

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

    // Create the GLFW window and OpenGL display context.
    window = glfwCreateWindow(windowWidth, windowHeight, "Force Dimension - OpenGL Sphere Example", nullptr, nullptr);
    if (!window)
    {
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, onKeyPressed);
    glfwSetWindowSizeCallback(window, onWindowResized);
    glfwSetWindowPos(window, x, y);
    glfwSwapInterval(SwapInterval);
    glfwShowWindow(window);

    // Adjust initial window size
    onWindowResized(window, windowWidth, windowHeight);

    // Define material properties.
    GLfloat mat_ambient[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_diffuse[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

    // Define light sources.
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

    // Use depth buffering for hidden surface elimination.
    glEnable(GL_DEPTH_TEST);

    // Clear the OpenGL color buffer.
    glClearColor(0.0, 0.0, 0.0, 1.0);

    // Report success.
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function initializes the haptic device.
///
/// \return
/// 0 on success, -1 on failure.
///
////////////////////////////////////////////////////////////////////////////////

int initializeHaptics()
{
    // Open the first available haptic device.
    if (dhdOpen() < 0)
    {
        return -1;
    }

    // Store the haptic device properties.
    useRotation = dhdHasWrist();
    useGripper = dhdHasGripper();
    toolCount = (useGripper) ? 2 : 1;

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function initializes the simulation.
///
/// \return
/// 0 on success, -1 on failure.
///
////////////////////////////////////////////////////////////////////////////////

int initializeSimulation()
{
    // Initialize all tool positions.
    for (int index = 0; index < toolCount; index++)
    {
        toolPosition[index].setZero();
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char* argv[])
{
    // Display version information.
    std::cout << "OpenGL Sphere Example " << dhdGetSDKVersionStr() << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Find and open a connection to a haptic device.
    if (initializeHaptics() < 0)
    {
        std::cout << "error: failed to initialize haptics" << std::endl;
        return -1;
    }

    // Open a GLFW window to render the simulation.
    if (initializeGLFW() < 0)
    {
        std::cout << "error: failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Initialize simulation objects.
    if (initializeSimulation() < 0)
    {
        std::cout << "error: failed to initialize the simulation" << std::endl;
        return -1;
    }

    // Create a high priority haptic thread.
#if defined(WIN32) || defined(WIN64)

    DWORD threadHandle;
    CreateThread(nullptr, 0, (LPTHREAD_START_ROUTINE)(hapticsLoop), nullptr, 0x0000, &threadHandle);
    SetThreadPriority(&threadHandle, THREAD_PRIORITY_ABOVE_NORMAL);

#else

    pthread_t threadHandle;
    pthread_create(&threadHandle, nullptr, hapticsLoop, nullptr);
    struct sched_param schedulerParameters;
    memset(&schedulerParameters, 0, sizeof(struct sched_param));
    schedulerParameters.sched_priority = 10;
    pthread_setschedparam(threadHandle, SCHED_RR, &schedulerParameters);

#endif

    // Register a callback that stops the haptic thread when the application exits.
    atexit(onExit);

    // Display the device type.
    std::cout << dhdGetSystemName() << " device detected" << std::endl << std::endl;

    // Display user instructions.
    std::cout << "press 'r' to toggle display of the haptic rate" << std::endl;
    std::cout << "      'q' to quit" << std::endl << std::endl;

    // Main graphic loop
    while (simulationRunning && !glfwWindowShouldClose(window))
    {
        // Render graphics.
        if (updateGraphics() < 0)
        {
            std::cout << "error: failed to update graphics" << std::endl;
            break;
        }

        // Swap buffers.
        glfwSwapBuffers(window);

        // Process GLFW events.
        glfwPollEvents();
    }

    // Close the GLFW window.
    glfwDestroyWindow(window);

    // Close the GLFW library.
    glfwTerminate();

    // Report success.
    return 0;
}
