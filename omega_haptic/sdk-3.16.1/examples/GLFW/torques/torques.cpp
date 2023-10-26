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
/// This example illustrates how to compute both force and torque when
/// with virtual objects. A Force Dimension haptic device controls the position
/// and rotation of a cube that can interact with a fixed point located in the
/// middle of the virtual scene. Interactions between the cube and the fixed
/// point generate both force and torque, which are applied to the haptic device
/// by the haptic loop running as a separate thread. The cube and fixed point
/// are rendered visually in a GLFW window.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
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

/// Cube stiffness in [N/m]
constexpr double Stiffness = 2000.0;

/// Torque gain
constexpr double TorqueGain = 2.0;

/// Cube size in [m]
constexpr float CubeSize = 0.06f;

/// Cube half-size in [m]
constexpr float CubeHalfSize = CubeSize / 2.0f;

/// Normals for the 6 faces of a cube
const GLfloat CubeNormals[6][3] =
{
 { -1.0, 0.0, 0.0 },
 { 0.0, 1.0, 0.0 },
 { 1.0, 0.0, 0.0 },
 { 0.0, -1.0, 0.0 },
 { 0.0, 0.0, 1.0 },
 { 0.0, 0.0, -1.0 }
};

/// Vertex indices for the 6 faces of a cube
const GLint CubeFaces[6][4] =
{
 { 0, 1, 2, 3 },
 { 3, 2, 6, 7 },
 { 7, 6, 5, 4 },
 { 4, 5, 1, 0 },
 { 5, 6, 2, 1 },
 { 7, 4, 0, 3 }
};

// Cube vertices
const GLfloat CubeVertices[8][3] =
{
 { -1.0 * CubeHalfSize, -1.0 * CubeHalfSize,  1.0 * CubeHalfSize },
 { -1.0 * CubeHalfSize, -1.0 * CubeHalfSize, -1.0 * CubeHalfSize },
 { -1.0 * CubeHalfSize,  1.0 * CubeHalfSize, -1.0 * CubeHalfSize },
 { -1.0 * CubeHalfSize,  1.0 * CubeHalfSize,  1.0 * CubeHalfSize },
 {  1.0 * CubeHalfSize, -1.0 * CubeHalfSize,  1.0 * CubeHalfSize },
 {  1.0 * CubeHalfSize, -1.0 * CubeHalfSize, -1.0 * CubeHalfSize },
 {  1.0 * CubeHalfSize,  1.0 * CubeHalfSize, -1.0 * CubeHalfSize },
 {  1.0 * CubeHalfSize,  1.0 * CubeHalfSize,  1.0 * CubeHalfSize }
};

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

/// Text overlay flag
bool showRefreshRate = true;

/// GLFW window handle
GLFWwindow* window = nullptr;

/// GLFW window width in [pixel]
int windowWidth = 0;

/// GLFW window height in [pixel]
int windowHeight = 0;

/// Torque rendering flag
bool renderTorque = true;

/// Force rendering flag
bool renderForce = true;

////////////////////////////////////////////////////////////////////////////////
///
/// This function renders the scene using OpenGL.
///
////////////////////////////////////////////////////////////////////////////////

int updateGraphics()
{
    // Clean up.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Retrieve the haptic device position.
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    if (dhdGetPosition(&px, &py, &pz) < 0)
    {
        return -1;
    }
    Eigen::Vector3d devicePosition;
    devicePosition << px, py, pz;

    // Retrieve the haptic device orientation frame (identity for 3-dof devices).
    double deviceRotation[3][3] = {};
    if (dhdGetOrientationFrame(deviceRotation) < 0)
    {
        return -1;
    }

    // Create a cube object.
    cMatrixGL matrix;
    matrix.set(devicePosition, deviceRotation);
    matrix.glMatrixPushMultiply();

    // Render the cube object.
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.1f, 0.3f, 0.5f);
    for (int index = 0; index < 6; index++)
    {
        glBegin(GL_QUADS);
        glNormal3fv(&CubeNormals[index][0]);
        glVertex3fv(&CubeVertices[CubeFaces[index][0]][0]);
        glVertex3fv(&CubeVertices[CubeFaces[index][1]][0]);
        glVertex3fv(&CubeVertices[CubeFaces[index][2]][0]);
        glVertex3fv(&CubeVertices[CubeFaces[index][3]][0]);
        glEnd();
    }
    matrix.glMatrixPop();

    // Render a fixed sphere at center of workspace.
    glColor3f(0.8f, 0.8f, 0.8f);
    GLUquadricObj* sphere = gluNewQuadric();
    gluSphere(sphere, 0.005, 64, 64);

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
/// This function implements the interaction force and torque between the cube
/// and the sphere.
///
/// \param a_position
/// Cube position
///
/// \param a_rotation
/// Cube rotation
///
/// \param a_force
/// Resulting interaction force
///
/// \param a_torque
/// Resulting interaction torque
///
////////////////////////////////////////////////////////////////////////////////

void computeForceAndTorque(const Eigen::Vector3d& a_position,
                           const Eigen::Matrix3d& a_rotation,
                           Eigen::Vector3d& a_force,
                           Eigen::Vector3d& a_torque)
{
    // By default, render null force and torque.
    Eigen::Vector3d localForce(0, 0, 0);
    Eigen::Vector3d localTorque(0, 0, 0);

    // Compute the position and rotation of the device in the local coordinates of the cube.
    Eigen::Matrix3d localRotationTransposed = a_rotation.transpose();
    Eigen::Vector3d localPosition = -1.0 * localRotationTransposed * a_position;

    // Compute the interaction force and torque if the cube interacts with the sphere.
    if ((localPosition(0) < CubeHalfSize) && (localPosition(0) > -CubeHalfSize) &&
        (localPosition(1) < CubeHalfSize) && (localPosition(1) > -CubeHalfSize) &&
        (localPosition(2) < CubeHalfSize) && (localPosition(2) > -CubeHalfSize))
    {
        // Determine the cube face that has the maximum penetration.
        double maxPenetration = CubeHalfSize;
        Eigen::Vector3d normalDirection(0, 0, 1);
        for (int axis = 0; axis < 3; axis++)
        {
            std::vector<double> directionsList = { -1.0, 1.0 };
            for (double currentDirection : directionsList)
            {
                // Test penetration in the current direction of the current axis.
                // If this specific direction is the largest penetration so far, store
                // the penetration and normal direction as the one to render.
                double penetration = std::abs(currentDirection * CubeHalfSize - localPosition(axis));
                if (penetration < maxPenetration)
                {
                    maxPenetration = penetration;
                    normalDirection << 0.0, 0.0, 0.0;
                    normalDirection(axis) = currentDirection;
                }
            }
        }

        // Compute the reaction force.
        localForce = -maxPenetration * Stiffness * normalDirection;
        Eigen::Vector3d normalForce = -1.0 * localForce;
        localTorque = -TorqueGain * localPosition.cross(normalForce);
    }

    // Convert the resulting force to global coordinates.
    a_force = a_rotation * localForce;
    a_torque = a_rotation * localTorque;
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
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double rot[3][3] = {};
    Eigen::Vector3d deviceForce;
    Eigen::Vector3d deviceTorque;
    Eigen::Vector3d devicePosition;
    Eigen::Matrix3d deviceRotation;

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
        // Retrieve the haptic device position and rotation.
        if (dhdGetPositionAndOrientationFrame(&px, &py, &pz, rot) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }
        devicePosition << px, py, pz;
        deviceRotation << rot[0][0], rot[0][1], rot[0][2],
                          rot[1][0], rot[1][1], rot[1][2],
                          rot[2][0], rot[2][1], rot[2][2];

        // Compute interaction force and torque.
        computeForceAndTorque(devicePosition, deviceRotation, deviceForce, deviceTorque);

        // Only enable haptic rendering once the device is in free space.
        static bool safeToRenderHaptics = false;
        if (!safeToRenderHaptics)
        {
            if (deviceForce.norm() == 0)
            {
                safeToRenderHaptics = true;
            }
            else
            {
                deviceForce.setZero();
                deviceTorque.setZero();
            }
        }

        // Disable torque if required.
        if (!renderTorque)
        {
            deviceTorque.setZero();
        }

        // Send force and torque to the haptic device.
        dhdSetForceAndTorqueAndGripperForce(deviceForce(0), deviceForce(1), deviceForce(2),
                                            deviceTorque(0), deviceTorque(1), deviceTorque(2),
                                            0.0);
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

    // Toggle torque rendering.
    if (a_key == GLFW_KEY_T)
    {
        renderTorque = !renderTorque;
    }

    // Toggle force rendering.
    if (a_key == GLFW_KEY_F)
    {
        renderForce = !renderForce;
        if (dhdEnableForce((renderForce) ? DHD_ON : DHD_OFF) < 0)
        {
            std::cout << "warning: failed to change force status (" << dhdErrorGetLastStr() << ")" << std::endl;
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
    window = glfwCreateWindow(windowWidth, windowHeight, "Force Dimension - Torques Example", nullptr, nullptr);
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

    // Define a single OpenGL light source.
    GLfloat lightAmbient[]  = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat lightDiffuse[]  = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat lightSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightPosition[] = {1.0f, 0.5f, 0.8f, 0.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    // Use depth buffering for hidden surface elimination.
    glEnable(GL_DEPTH_TEST);

    // Clear the OpenGL color buffer.
    glClearColor(0.0, 0.0, 0.0, 1.0);

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

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char* argv[])
{
    // Display version information.
    std::cout << "OpenGL Torques Example " << dhdGetSDKVersionStr() << std::endl;
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
    std::cout << "      't' to enable or disable torque on wrist actuated devices" << std::endl;
    std::cout << "      'f' to enable or disable force" << std::endl;
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

