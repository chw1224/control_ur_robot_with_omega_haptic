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
/// This example illustrates how to compute the interaction between several
/// Force Dimension haptic devices and a torus that spins freely around its
/// center when forces are applied to it.
///
/// A single thread is used to compute the interaction force and torque on all
/// available haptic devices, as well as to compute the dynamics of the torus.
/// Holding any of the haptic devices end-effector user button stops the torus
/// from spinning and holds it in place.
///
////////////////////////////////////////////////////////////////////////////////

// C++ library headers
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

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
/// Haptic device data structure
///
////////////////////////////////////////////////////////////////////////////////

struct HapticDevice
{
    /// Haptic device handle
    int deviceId;

    /// Number of tools provided by this haptic device
    int numTools;

    /// Positions of the haptic device tool(s)
    Eigen::Vector3d toolPosition[2];

    /// Haptic device rotation
    Eigen::Matrix3d rotation;

    /// \b true if the haptic device has a wrist
    bool useRotation;

    /// \b true if the haptic device has a gripper
    bool useGripper;

    /// Constructor
    HapticDevice()
    : deviceId { -1 },
      numTools {},
      useRotation { false },
      useGripper{ false }
    {}
};

////////////////////////////////////////////////////////////////////////////////
///
/// Constants
///
////////////////////////////////////////////////////////////////////////////////

/// Torus stiffness in [N/m]
constexpr double Stiffness = 1000.0;

/// Torus apparent mass in arbitrary unit
constexpr double Mass = 100.0;

/// Torus angular damping in [Nm/(rad/s)]
constexpr double Kv = 1.0;

/// Torus outer radius in [m]
constexpr float TorusRadius0 = 0.05f;

/// Torus inner radius in [m]
constexpr float TorusRadius1 = 0.027f;

/// Haptic tool sphere radius in [m]
constexpr double ToolRadius = 0.005;

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

/// Vector of available haptic devices
std::vector<HapticDevice> devicesList;

/// Torus position
Eigen::Vector3d torusPosition;

/// Torus rotation
Eigen::Matrix3d torusRotation;

////////////////////////////////////////////////////////////////////////////////
///
/// This function renders a torus using OpenGL.
///
/// \param a_majorRadius
/// The outer radius of the torus in [m]
///
/// \param a_minorRadius
/// The inner radius of the torus in [m]
///
/// \param a_majorNumSegments
/// The number of segments to use to generate polygons along the torus major
/// direction
///
/// \param a_minorNumSegments
/// The number of segments to use to generate polygons along the torus minor
/// direction
///
////////////////////////////////////////////////////////////////////////////////

void DrawTorus(float a_outerRadius,
               float a_innerRadius,
               int a_majorNumSegments,
               int a_minorNumSegments)
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    double majorStep = 2.0 * M_PI / a_majorNumSegments;
    for (int majorIndex = 0; majorIndex < a_majorNumSegments; ++majorIndex)
    {
        double a0 = majorIndex * majorStep;
        double a1 = a0 + majorStep;
        GLdouble x0 = cos(a0);
        GLdouble y0 = sin(a0);
        GLdouble x1 = cos(a1);
        GLdouble y1 = sin(a1);

        glBegin(GL_TRIANGLE_STRIP);

        double minorStep = 2.0 * M_PI / a_minorNumSegments;
        for (int minorIndex = 0; minorIndex <= a_minorNumSegments; ++minorIndex)
        {
            double b = minorIndex * minorStep;
            GLdouble c = cos(b);
            GLdouble r = a_innerRadius * c + a_outerRadius;
            GLdouble z = a_innerRadius * sin(b);

            glNormal3d(x0 * c, y0 * c, z / a_innerRadius);
            glTexCoord2d(majorIndex / static_cast<GLdouble>(a_majorNumSegments), minorIndex / static_cast<GLdouble>(a_minorNumSegments));
            glVertex3d(x0 * r, y0 * r, z);

            glNormal3d(x1 * c, y1 * c, z / a_innerRadius);
            glTexCoord2d((majorIndex + 1) / static_cast<GLdouble>(a_majorNumSegments), minorIndex / static_cast<GLdouble>(a_minorNumSegments));
            glVertex3d(x1 * r, y1 * r, z);
        }

        glEnd();
    }
}

////////////////////////////////////////////////////////////////////////////////
///
/// This function renders the scene using OpenGL.
///
////////////////////////////////////////////////////////////////////////////////

int updateGraphics()
{
    // Clean up.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the torus.
    static const GLfloat mat_ambient0[] = { 0.1f, 0.1f, 0.3f };
    static const GLfloat mat_diffuse0[] = { 0.1f, 0.3f, 0.5f };
    static const GLfloat mat_specular0[] = { 0.5f, 0.5f, 0.5f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular0);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

    // Render the torus at its current position and rotation.
    cMatrixGL matrix;
    matrix.set(torusPosition, torusRotation);
    matrix.glMatrixPushMultiply();
    DrawTorus(TorusRadius0, TorusRadius1, 64, 64);
    matrix.glMatrixPop();

    // Configure OpenGL for tool rendering.
    static const GLfloat mat_ambient1[] = { 0.4f, 0.4f, 0.4f };
    static const GLfloat mat_diffuse1[] = { 0.8f, 0.8f, 0.8f };
    static const GLfloat mat_specular1[] = { 0.5f, 0.5f, 0.5f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient1);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse1);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular1);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

    // Render all tools for all devices.
    size_t devicesCount = devicesList.size();
    for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
    {
        // Render all tools for the current device.
        HapticDevice& currentDevice = devicesList[deviceIndex];
        for (int toolIndex = 0; toolIndex < currentDevice.numTools; toolIndex++)
        {
            matrix.set(currentDevice.toolPosition[toolIndex], currentDevice.rotation);
            matrix.glMatrixPushMultiply();
            GLUquadricObj* sphere = gluNewQuadric();
            gluSphere(sphere, ToolRadius, 32, 32);

            // Render a small frame if the haptic device supports rotations.
            if (currentDevice.useRotation)
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
    double timePrevious = dhdGetTime();
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double rot[3][3] = {};
    Eigen::Vector3d toolPosition[2];
    Eigen::Vector3d toolLocalPosition;
    Eigen::Vector3d forceLocal;
    Eigen::Vector3d force;
    Eigen::Vector3d forceTool[2];
    forceTool[0].setZero();
    forceTool[1].setZero();
    Eigen::Vector3d torusAngularVelocity;
    torusAngularVelocity.setZero();

    // Enable force on all devices.
    size_t devicesCount = devicesList.size();
    for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
    {
        dhdEnableForce(DHD_ON, devicesList[deviceIndex].deviceId);
    }

    // Run the haptic loop.
    while (simulationRunning)
    {
        // Compute time step.
        double time = dhdGetTime();
        double timeStep = time - timePrevious;
        timePrevious = time;

        // Process each device in turn.
        for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
        {
            // Shortcut to the current device.
            HapticDevice& currentDevice = devicesList[deviceIndex];

            // Select the active device to receive all subsequent DHD commands.
            dhdSetDevice(currentDevice.deviceId);

            // Retrieve the device orientation frame (identity for 3-dof devices).
            if (dhdGetOrientationFrame(rot) < 0)
            {
                std::cout << std::endl << "error: failed to read rotation (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }
            currentDevice.rotation << rot[0][0], rot[0][1], rot[0][2],
                                      rot[1][0], rot[1][1], rot[1][2],
                                      rot[2][0], rot[2][1], rot[2][2];

            // Retrieve the position of all tools attached to devices.
            // Devices equipped with grippers provide 2 tools, while others only provide 1.
            if (currentDevice.useGripper)
            {
                if (dhdGetGripperThumbPos(&px, &py, &pz) < 0)
                {
                    std::cout << std::endl << "error: failed to read rotation (" << dhdErrorGetLastStr() << ")" << std::endl;
                    break;
                }
                toolPosition[0] << px, py, pz;

                if (dhdGetGripperFingerPos(&px, &py, &pz) < 0)
                {
                    std::cout << std::endl << "error: failed to read rotation (" << dhdErrorGetLastStr() << ")" << std::endl;
                    break;
                }
                toolPosition[1] << px, py, pz;
            }
            else
            {
                if (dhdGetPosition(&px, &py, &pz) < 0)
                {
                    std::cout << std::endl << "error: failed to read rotation (" << dhdErrorGetLastStr() << ")" << std::endl;
                    break;
                }
                toolPosition[0] << px, py, pz;
            }

            // Compute the interaction between the torus and each tool.
            for (int toolIndex = 0; toolIndex < currentDevice.numTools; toolIndex++)
            {
                // Compute the position of the tool in the local coordinates of the torus.
                toolLocalPosition = torusRotation.transpose() * (toolPosition[toolIndex] - torusPosition);

                // Project the tool position onto the torus plane (z = 0).
                Eigen::Vector3d toolProjection = toolLocalPosition;
                toolProjection(2) = 0.0;

                // Search for the nearest point on the torus medial axis.
                forceLocal.setZero();
                if (toolLocalPosition.squaredNorm() > 1e-10)
                {
                    Eigen::Vector3d pointAxisTorus = TorusRadius0 * toolProjection.normalized();

                    // Compute eventual penetration of the tool inside the torus.
                    Eigen::Vector3d torusToolDirection = toolLocalPosition - pointAxisTorus;

                    // If the tool is inside the torus, compute the force which is proportional to the tool penetration.
                    double distance = torusToolDirection.norm();
                    if ((distance < (TorusRadius1 + ToolRadius)) && (distance > 0.001))
                    {
                        forceLocal = ((TorusRadius1 + ToolRadius) - distance) * Stiffness * torusToolDirection.normalized();
                        toolLocalPosition = pointAxisTorus + (TorusRadius1 + ToolRadius) * torusToolDirection.normalized();
                    }

                    // Otherwise, the tool is outside the torus and we have a null force.
                    else
                    {
                        forceLocal.setZero();
                    }
                }

                // Convert the tool reaction force and position to world coordinates.
                forceTool[toolIndex] = torusRotation * forceLocal;
                currentDevice.toolPosition[toolIndex] = torusRotation * toolLocalPosition;

                // Update the torus angular velocity.
                torusAngularVelocity += -1.0 / Mass * timeStep * (currentDevice.toolPosition[toolIndex] - torusPosition).cross(forceTool[toolIndex]);
            }

            // Compute the force to render on the haptic device.
            Eigen::Vector3d force;
            double gripperForceMagnitude = 0.0;

            // If the haptic device does not have a gripper, use the current tool force.
            if (!currentDevice.useGripper)
            {
                force = forceTool[0];
            }

            // If the haptic device has a gripper, compute the projected force on each gripper tool.
            else
            {
                // Compute the total force.
                force = forceTool[0] + forceTool[1];
                Eigen::Vector3d gripperDirection = toolPosition[1] - toolPosition[0];

                // If the total force is not null, project it on both gripper tools.
                if (gripperDirection.norm() > 0.00001)
                {
                    // Project the mobile gripper tool force (forceTool[1]) onto the gripper opening vector (gripperDirection).
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
            dhdSetForceAndGripperForce(force(0), force(1), force(2), gripperForceMagnitude);
        }

        // Stop the torus rotation if any of the devices button is pressed.
        for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
        {
            if (dhdGetButton(0, devicesList[deviceIndex].deviceId) != DHD_OFF)
            {
                torusAngularVelocity.setZero();
            }
        }

        // Add damping to slow down the torus rotation over time.
        torusAngularVelocity *= (1.0 - Kv * timeStep);

        // Compute the next pose of the torus.
        if (torusAngularVelocity.norm() > 1e-10)
        {
            Eigen::Matrix3d torusRotationIncrement;
            torusRotationIncrement = Eigen::AngleAxisd(torusAngularVelocity.norm(), torusAngularVelocity.normalized());
            torusRotation = torusRotationIncrement * torusRotation;
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

    // Close the connection to all the haptic devices.
    size_t devicesCount = devicesList.size();
    for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
    {
        if (dhdClose() < 0)
        {
            std::cout << "error: failed to close the connection to device ID " << devicesList[deviceIndex].deviceId << " (" << dhdErrorGetLastStr() << ")" << std::endl;
            return;
        }
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
    window = glfwCreateWindow(windowWidth, windowHeight, "Force Dimension - OpenGL Torus Example", nullptr, nullptr);
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
    int devicesCount = dhdGetAvailableCount();
    if (devicesCount < 1)
    {
        std::cout << "error: no device found" << std::endl;
        return -1;
    }

    // Open a connection to each available haptic device and store its properties.
    for (int deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
    {
        int deviceId = dhdOpenID(deviceIndex);
        if (deviceId >= 0)
        {
            devicesList.push_back(HapticDevice {});
            HapticDevice& currentDevice = devicesList.back();
            currentDevice.deviceId = deviceId;
            currentDevice.useRotation = dhdHasWrist(deviceId);
            currentDevice.useGripper = dhdHasGripper(deviceId);
            currentDevice.numTools = (currentDevice.useGripper) ? 2 : 1;

            // Enable button emulation on devices with an active gripper.
            if (dhdHasActiveGripper() && dhdEmulateButton(DHD_ON) < 0)
            {
                std::cout << "error: failed to enable button emulation (" << dhdErrorGetLastStr() << ")" << std::endl;
                return -1;
            }

            // Display the device type.
            std::cout << dhdGetSystemName() << " device detected" << std::endl;
        }
    }

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
    size_t devicesCount = devicesList.size();
    for (size_t deviceIndex = 0; deviceIndex < devicesCount; deviceIndex++)
    {
        for (int toolIndex = 0; toolIndex < devicesList[deviceIndex].numTools; toolIndex++)
        {
            devicesList[deviceIndex].toolPosition[toolIndex].setZero();
        }
    }

    // Initialize the torus position.
    torusPosition.setZero();
    torusRotation.Identity();
    torusRotation = Eigen::AngleAxisd(M_PI * 45.0 / 180.0, Eigen::Vector3d(0.0, 1.0, -1.0));

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc,
         char* argv[])
{
    // Display version information.
    std::cout << "OpenGL Torus Example " << dhdGetSDKVersionStr() << std::endl;
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

    // Display user instructions.
    std::cout << std::endl;
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
