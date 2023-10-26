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
/// This example illustrates how to constrain a Force Dimension haptic device
/// range of motion on its base, wrist or gripper. The range constraints can be
/// enabled, disabled and adjusted using keyboard input at runtime.
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

// CHAI3D geometry and math library headers
#include "CGeometry.h"

// Project headers
#include "FontGL.h"

////////////////////////////////////////////////////////////////////////////////
///
/// Constants
///
////////////////////////////////////////////////////////////////////////////////

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

/// Haptic device position
chai3d::cVector3d devicePosition;

/// Haptic device rotation
chai3d::cMatrix3d deviceRotation;

/// Haptic device gripper angle in [deg]
double deviceGripperAngleDeg = 0.0;

/// Haptic device linear velocity [m/s]
chai3d::cVector3d deviceLinearVelocity;

/// Haptic device angular velocity in [rad/s]
chai3d::cVector3d deviceAngularVelocity;

/// Haptic device input switches state
uint32_t deviceSwitches = 0x00000000;

/// Force to render on the haptic device in [N]
chai3d::cVector3d deviceForce;

/// Torque to render on the haptic device in [Nm]
chai3d::cVector3d deviceTorque;

/// Force to render on the haptic device gripper in [N]
double deviceGripperForce = 0.0;

/// Haptic device roll angle in [rad]
double deviceRollAngleRad = 0.0;

/// Haptic device force/torque saturation flag
bool flagSaturation = false;

/// Hold position constraint
chai3d::cVector3d holdPosition;

/// Hold position constraint flag
bool flagHoldPosition = false;

/// Hold position constraint safety flag
bool flagHoldPositionReady = false;

/// Workspace constraint
double workspaceRadius = 0.05;

/// Workspace constraint flag
bool flagWorkspace = true;

/// Workspace constraint safety flag
bool flagWorkspaceReady = false;

/// Cone workspace limit in [rad]
double coneAngleLimit = chai3d::cDegToRad(10.0);

/// Roll workspace limit in [rad]
double rollAngleLimit = chai3d::cDegToRad(40.0);

/// Cone principal axis
chai3d::cVector3d coneAxis;

/// Roll principal axis
chai3d::cVector3d rollAxis;

/// Cone rotation
chai3d::cMatrix3d coneRotation;

/// Cone constraint flag
bool flagCone = false;

/// Cone constraint safety flag
bool flagConeReady = false;

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
    ////////////////////////////////////////////////////////////////////////////
    //
    // Initialize rendering
    //
    ////////////////////////////////////////////////////////////////////////////

    GLUquadricObj *sphere;
    chai3d::cTransform mat;

    // Initialize the OpenGL settings.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 1.0);

    ////////////////////////////////////////////////////////////////////////////
    //
    // Haptic device rendering
    //
    ////////////////////////////////////////////////////////////////////////////

    // Display the haptic device.
    mat.set(devicePosition, deviceRotation);
    glPushMatrix();
    glMultMatrixd((const double *)mat.getData());

    // Render a sphere.
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.5f, 0.5f, 0.5f);
    sphere = gluNewQuadric();
    gluSphere(sphere, 0.005, 32, 32);

    // Render a frame.
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3d(0.00, 0.00, 0.00);
    glVertex3d(0.02, 0.00, 0.00);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3d(0.00, 0.00, 0.00);
    glVertex3d(0.00, 0.02, 0.00);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3d(0.00, 0.00, 0.00);
    glVertex3d(0.00, 0.00, 0.02);
    glEnd();
    glEnable(GL_LIGHTING);

    // Render the gripper.
    glDisable(GL_LIGHTING);
    glBegin(GL_LINE_STRIP);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3d(0.0, 0.0, 0.0);
    for (int index = -10; index <= 10; index++)
    {
        double angle = 0.1 * (double)(index) * deviceGripperAngleDeg;
        double px = 0.1 * chai3d::cCosDeg(angle);
        double py = 0.1 * chai3d::cSinDeg(angle);
        glVertex3d(-px, py, 0.0);
    }
    glVertex3d(0.0, 0.0, 0.0);
    glColor3f(0.5f, 0.5f, 0.5f);
    glVertex3d(-0.1, 0.0, 0.0);
    glEnd();
    glEnable(GL_LIGHTING);
    glPopMatrix();

    ////////////////////////////////////////////////////////////////////////////
    //
    // Cone rendering
    //
    ////////////////////////////////////////////////////////////////////////////

    // Render cone and roll constraint.
    if (flagConeReady)
    {
        // Render the cone.
        mat.set(devicePosition, coneRotation);
        glPushMatrix();
        glMultMatrixd((const double *)mat.getData());

        // Render the cone axis.
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(-0.1, 0.0, 0.0);
        glEnd();

        // Render the gripper fan.
        glBegin(GL_LINE_STRIP);
        for (int index = 0; index <= 36; index++)
        {
            double SIZE = 0.1;
            double radius = SIZE * chai3d::cTanRad(coneAngleLimit);
            double angle = 10 * (double)(index);
            double px = radius * chai3d::cCosDeg(angle);
            double py = radius * chai3d::cSinDeg(angle);
            glVertex3d(-SIZE, px, py);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        glPopMatrix();

        // Render the roll disk and limits.
        mat.set(devicePosition, deviceRotation);
        glPushMatrix();
        glMultMatrixd((const double *)mat.getData());

        // Render lines and polygons in two passes.
        for (int pass = 0; pass < 2; pass++)
        {
            if (pass == 0)
            {
                glColor3f(1.0f, 1.0f, 1.0f);
                glDepthMask(GL_TRUE);
                glBegin(GL_LINE_STRIP);
            }
            else if (pass == 1)
            {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glColor4f(1.0f, 1.0f, 1.0f, 0.3f);
                glDepthMask(GL_FALSE);
                glBegin(GL_TRIANGLE_FAN);
            }

            glVertex3d(0.0, 0.0, 0.0);
            double angleMin = -rollAngleLimit + deviceRollAngleRad;
            double angleMax = rollAngleLimit + deviceRollAngleRad;
            for (int index = 0; index <= 36; index++)
            {
                double angle = angleMax + ((chai3d::C_TWO_PI - (angleMax - angleMin)) / 36.0) * (double)(index);
                double px = 0.02 * chai3d::cCosRad(angle);
                double py = 0.02 * chai3d::cSinRad(angle);
                glVertex3d(0, -py, px);
            }
            glVertex3d(0.0, 0.0, 0.0);
            glEnd();
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }

        glEnable(GL_LIGHTING);
        glPopMatrix();
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // Refresh rate rendering
    //
    ////////////////////////////////////////////////////////////////////////////

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

    ////////////////////////////////////////////////////////////////////////////
    //
    // Switches status rendering
    //
    ////////////////////////////////////////////////////////////////////////////

    // Render the status of the switches.
    glDisable(GL_LIGHTING);
    glColor3f(1.0, 1.0, 1.0);
    glRasterPos3f(0.0f, -0.03f, 0.1f);
    for (int index = 0; index < 16; index++)
    {
        // Display bit status.
        if (chai3d::cCheckBit(deviceSwitches, index))
        {
            render_character('1', HELVETICA12);
        }
        else
        {
            render_character('0', HELVETICA12);
        }

        // Display space character.
        render_character(' ', HELVETICA12);
    }
    glEnable(GL_LIGHTING);

    ////////////////////////////////////////////////////////////////////////////
    //
    // Finalize rendering
    //
    ////////////////////////////////////////////////////////////////////////////

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
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double rot[3][3] = {};
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

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
        ////////////////////////////////////////////////////////////////////////
        //
        // Retrieve haptic device data.
        //
        ////////////////////////////////////////////////////////////////////////

        // Retrieve the haptic device position and rotation.
        if (dhdGetPositionAndOrientationFrame(&px, &py, &pz, rot) < DHD_NO_ERROR)
        {
            std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }
        devicePosition.set(px, py, pz);
        deviceRotation.set(rot[0][0], rot[0][1], rot[0][2],
                           rot[1][0], rot[1][1], rot[1][2],
                           rot[2][0], rot[2][1], rot[2][2]);

        // Read the gripper angle.
        if (dhdGetGripperAngleDeg(&deviceGripperAngleDeg) < 0)
        {
            std::cout << std::endl << "error: failed to read gripper angle (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }

        // Read the linear velocity.
        if (dhdGetLinearVelocity(&vx, &vy, &vz) < 0)
        {
            std::cout << std::endl << "error: failed to read linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }
        deviceLinearVelocity.set(vx, vy, vz);

        // Read the angular velocity.
        if (dhdGetAngularVelocityRad(&wx, &wy, &wz) < 0)
        {
            std::cout << std::endl << "error: failed to read gripper angle (" << dhdErrorGetLastStr() << ")" << std::endl;
            break;
        }
        deviceAngularVelocity.set(wx, wy, wz);

        // Read user switches.
        deviceSwitches = dhdGetButtonMask();

        ////////////////////////////////////////////////////////////////////////
        //
        // Compute interaction forces.
        //
        ////////////////////////////////////////////////////////////////////////

        // Apply zero force by default.
        deviceForce.zero();
        deviceTorque.zero();
        deviceGripperForce = 0.0;

        // Define stiffness and damping used to constrain the device position.
        constexpr double Kp = 2000.0;  // N/m
        constexpr double Kv = 10.0;    // N/(m/s)

        // Compute the force required to hold device in position if required.
        if (flagHoldPosition)
        {
            if (flagHoldPositionReady)
            {
                // Compute reaction force.
                chai3d::cVector3d force = -Kp * (devicePosition - holdPosition) - Kv * deviceLinearVelocity;

                // Add all forces together.
                deviceForce = deviceForce + force;
            }
            else
            {
                holdPosition = devicePosition;
                flagHoldPositionReady = true;
            }
        }

        // Compute the workspace constraint if required.
        if (flagWorkspace)
        {
            double distance = devicePosition.length();
            if (flagWorkspaceReady)
            {
                double depth = distance - workspaceRadius;
                if (depth > 0.0)
                {
                    // Compute surface normal vector.
                    chai3d::cVector3d normal = -chai3d::cNormalize(devicePosition);

                    // Compute reaction force.
                    chai3d::cVector3d force = Kp * depth * normal;

                    // Compute damping term parallel to normal.
                    chai3d::cVector3d damping = -Kv * chai3d::cProject(deviceLinearVelocity, normal);

                    // Add all forces together.
                    deviceForce = deviceForce + (force + damping);
                }
            }
            else
            {
                if (distance < workspaceRadius)
                {
                    flagWorkspaceReady = true;
                }
            }
        }

        // Compute the cone constraint if required.
        if (flagCone)
        {
            if (flagConeReady)
            {
                ////////////////////////////////////////////////////////////////
                //
                // Cone constraint
                //
                ////////////////////////////////////////////////////////////////

                // Compute tool axis and angle.
                chai3d::cVector3d toolAxis = -deviceRotation.getCol0();
                double toolAngle = chai3d::cAngle(toolAxis, coneAxis);

                // Compute constraint.
                if (toolAngle > coneAngleLimit)
                {
                    // Compute the cross product between cone axis and direction
                    // of haptic device. This gives us the torque direction.
                    chai3d::cVector3d torqueDirection = chai3d::cNormalize(cCross(coneAxis, toolAxis));

                    // Compute angle difference.
                    double deltaAngle = toolAngle - coneAngleLimit;

                    // Compute torque.
                    chai3d::cVector3d torque = -10.0 * deltaAngle * torqueDirection;

                    // Compute damping.
                    chai3d::cVector3d damping = -0.02 * chai3d::cProject(deviceAngularVelocity, torqueDirection);

                    // Add torque from cone constraint.
                    deviceTorque = deviceTorque + (torque + damping);
                }

                ////////////////////////////////////////////////////////////////
                //
                // Roll constraint
                //
                ////////////////////////////////////////////////////////////////

                // Compute roll axis and angle.
                chai3d::cVector3d toolRoll = deviceRotation.getCol2();
                chai3d::cVector3d ProjectedRollAxis = chai3d::cProjectPointOnPlane(rollAxis, chai3d::cVector3d(0, 0, 0), toolAxis);
                double currentRollAngle = chai3d::cAngle(toolRoll, ProjectedRollAxis);

                // Compute the cross product between cone axis and direction
                // of haptic device. This gives us the torque direction.
                chai3d::cVector3d t = chai3d::cNormalize(coneAxis);

                // Compute torque sign.
                double sign = 1.0;
                chai3d::cVector3d c = cCross(toolRoll, ProjectedRollAxis);
                if (chai3d::cAngle(t, c) < chai3d::C_PI_DIV_2)
                {
                    sign = -1.0;
                }

                // Store value.
                deviceRollAngleRad = sign * currentRollAngle;

                // Compute constraint.
                if (currentRollAngle > rollAngleLimit)
                {
                    // Compute angle difference.
                    double deltaAngle = currentRollAngle - rollAngleLimit;

                    // Compute torque.
                    chai3d::cVector3d torque = -4.0 * sign * deltaAngle * t;

                    // Compute damping.
                    chai3d::cVector3d damping = -0.02 * chai3d::cProject(deviceAngularVelocity, t);

                    // Add torque from cone constraint.
                    deviceTorque = deviceTorque + (torque + damping);
                }
            }
            else
            {
                coneAxis = -deviceRotation.getCol0();
                rollAxis = deviceRotation.getCol2();
                coneRotation = deviceRotation;
                flagConeReady = true;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        //
        // Send force and torque to the haptic device.
        //
        ////////////////////////////////////////////////////////////////////////

        // Limit the torque to a maximum value.
        constexpr double MaxTorque = 0.3;
        if (deviceTorque.length() > MaxTorque)
        {
            deviceTorque = MaxTorque * chai3d::cNormalize(deviceTorque);
        }

        // Apply force and torque all at once.
        int error = dhdSetForceAndTorqueAndGripperForce(deviceForce(0), deviceForce(1), deviceForce(2),
                                                        deviceTorque(0), deviceTorque(1), deviceTorque(2),
                                                        deviceGripperForce);

        // Check for if any force saturation occurred.
        if (error == 2)
        {
            flagSaturation = true;
        }
        else
        {
            flagSaturation = false;
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

    // Toggle hold device in position.
    if (a_key == GLFW_KEY_H)
    {
        flagHoldPosition = !flagHoldPosition;
        flagHoldPositionReady = false;
    }

    // Toggle workspace constraint.
    if (a_key == GLFW_KEY_W)
    {
        flagWorkspace = !flagWorkspace;
        flagWorkspaceReady = false;
    }

    // Toggle cone constraint.
    if (a_key == GLFW_KEY_C)
    {
        flagCone = !flagCone;
        flagConeReady = false;
    }

    // Reduce cone angle.
    if (a_key == GLFW_KEY_1)
    {
        coneAngleLimit = chai3d::cMax(coneAngleLimit - chai3d::cDegToRad(1.0), chai3d::cDegToRad(0.0));
    }

    // Increase cone angle.
    if (a_key == GLFW_KEY_2)
    {
        coneAngleLimit = chai3d::cMin(coneAngleLimit + chai3d::cDegToRad(1.0), chai3d::cDegToRad(35.0));
    }

    // Reduce roll angle.
    if (a_key == GLFW_KEY_3)
    {
        rollAngleLimit = chai3d::cMax(rollAngleLimit - chai3d::cDegToRad(1.0), chai3d::cDegToRad(0.0));
    }

    // Increase roll angle.
    if (a_key == GLFW_KEY_4)
    {
        rollAngleLimit = chai3d::cMin(rollAngleLimit + chai3d::cDegToRad(1.0), chai3d::cDegToRad(180.0));
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

    // Adjust initial window size.
    onWindowResized(window, windowWidth, windowHeight);

    // Set material properties.
    static const GLfloat mat_ambient[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_diffuse[] = { 0.5f, 0.5f, 0.5f };
    GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

    // Set light sources.
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

    // Enable button emulation on devices with an active gripper.
    if (dhdHasActiveGripper() && dhdEmulateButton(DHD_ON) < 0)
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
    std::cout << "OpenGL Constraints Example " << dhdGetSDKVersionStr() << std::endl;
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
    std::cout << "press 'h' to toggle hold device in position" << std::endl;
    std::cout << "      'w' to toggle spherical workspace limit" << std::endl;
    std::cout << "      'c' to toggle conical rotational limit" << std::endl;
    std::cout << "      '1' to increase cone angle" << std::endl;
    std::cout << "      '2' to decrease cone angle" << std::endl;
    std::cout << "      '3' to increase roll angle" << std::endl;
    std::cout << "      '4' to decrease roll angle" << std::endl;
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
