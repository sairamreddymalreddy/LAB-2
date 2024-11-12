#include "stdafx.h"

// standard

#include <math.h>
#include<stdlib.h>
// glut
#include <GL/glut.h>

// Include standard headers
#include <assert.h>





// Include standard headers
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

// Include GLUT and OpenGL
#define GLUT_DISABLE_ATEXIT_HACK


// Define constants
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#include "Quaternion.h"
#include "Interpolation.h"

struct UInput {
    double x;
    double y;
    double z;

    double fXr;
    double fYr;
    double fZr;

    double qXr;
    double qYr;
    double qZr;
    double qWr;
};

// Global variables
int g_screenWidth = 800;
int g_screenHeight = 600;

double dt = 0.016; // Default value, will be read from input file
UInput* g_UInput = nullptr;
// Number of control points
int n = 0;

double g_positionX = 0;
double g_positionY = 0;
double g_positionZ = 0;

double prevPositionX = 0.0;
double prevPositionY = 0.0;
double prevPositionZ = 0.0;

double movementDirX = 0.0;
double movementDirY = 0.0;
double movementDirZ = -1.0; // Default facing negative Z-axis

double g_rotationX = 0;
double g_rotationY = 0;
double g_rotationZ = 0;

double lastFrameTime = 0.0;
double totalTime = 0.0;

GLfloat g_rotation[16];

SplineType g_splineType;
AngleInput g_angleInput;

double leftlegAngle = 0;
double rightlegAngle = 0;

double legSwingSpeed = 3.0;      // Adjusted speed of leg swing
double legSwingAmplitude = 33.0; // Maximum swing angle

std::vector<UInput> controlPoints; // For plotting control points

bool isMovingAlongX = true; // True if moving along X-axis, false if along Z-axis

//building cuboid for torso body
void drawCuboid(float width, float height, float depth) {
    glPushMatrix();
    glScalef(width, height, depth);
    glutSolidCube(1.0f);
    glPopMatrix();
}

//taking the input from a file
void getUInputFromFile(const std::string& filename) {
    std::ifstream input(filename);

    if (!input.is_open()) {
        printf("Error: Failed to open the input file.\n");
        exit(1);
    }

    // Read dt from the first line
    input >> dt;

    // Read number of keyframes is defining
    input >> n;

    g_UInput = new UInput[n];

    std::string splineType;
    input >> splineType;
    if (splineType == "Bspline") {
        g_splineType = Bspline;
    }
    else {
        g_splineType = CatMullRom;
    }

    std::string angleType;
    input >> angleType;
    if (angleType == "Quaternion") {
        g_angleInput = Quaternions;
    }
    else {
        g_angleInput = Fixed;
    }

    controlPoints.clear();

    for (int i = 0; i < n; i++) {
        input >> g_UInput[i].x >> g_UInput[i].y >> g_UInput[i].z;

        if (g_angleInput == Fixed) {
            // Read rotation angles but ignore them
            input >> g_UInput[i].fXr >> g_UInput[i].fYr >> g_UInput[i].fZr;
        }
        else {
            // Read quaternion values but ignore them
            input >> g_UInput[i].qXr >> g_UInput[i].qYr >> g_UInput[i].qZr >> g_UInput[i].qWr;
        }

        // Store control points for plotting
        controlPoints.push_back(g_UInput[i]);
    }

    input.close();
}

void init(void) {
    // Initialize any required resources
}

void createRotationMatrixFromDirection(double dirX, double dirY, double dirZ, GLfloat* rotationMatrix) {
    // Assume up vector is (0, 1, 0)
    double upX = 0.0, upY = 1.0, upZ = 0.0;

    // Compute right vector = up x direction
    double rightX = upY * dirZ - upZ * dirY;
    double rightY = upZ * dirX - upX * dirZ;
    double rightZ = upX * dirY - upY * dirX;

    // Normalize right vector
    double rightLength = sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
    if (rightLength > 0.0) {
        rightX /= rightLength;
        rightY /= rightLength;
        rightZ /= rightLength;
    }

    // Recompute up vector = direction x right
    upX = dirY * rightZ - dirZ * rightY;
    upY = dirZ * rightX - dirX * rightZ;
    upZ = dirX * rightY - dirY * rightX;

    // Fill rotation matrix
    rotationMatrix[0] = (GLfloat)rightX;
    rotationMatrix[1] = (GLfloat)rightY;
    rotationMatrix[2] = (GLfloat)rightZ;
    rotationMatrix[3] = 0.0f;

    rotationMatrix[4] = (GLfloat)upX;
    rotationMatrix[5] = (GLfloat)upY;
    rotationMatrix[6] = (GLfloat)upZ;
    rotationMatrix[7] = 0.0f;

    rotationMatrix[8] = (GLfloat)(-dirX);
    rotationMatrix[9] = (GLfloat)(-dirY);
    rotationMatrix[10] = (GLfloat)(-dirZ);
    rotationMatrix[11] = 0.0f;

    rotationMatrix[12] = 0.0f;
    rotationMatrix[13] = 0.0f;
    rotationMatrix[14] = 0.0f;
    rotationMatrix[15] = 1.0f;
}

void CatMullRomInterpolate(double t, int currentKeyframeIndex, int nextKeyframeIndex) {
    // Perform Catmull-Rom interpolation for position
    double interpolatedX = Interpolation::CatmullRomInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].x,
        g_UInput[currentKeyframeIndex].x,
        g_UInput[nextKeyframeIndex].x,
        g_UInput[(nextKeyframeIndex + 1) % n].x);

    double interpolatedY = Interpolation::CatmullRomInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].y,
        g_UInput[currentKeyframeIndex].y,
        g_UInput[nextKeyframeIndex].y,
        g_UInput[(nextKeyframeIndex + 1) % n].y);

    double interpolatedZ = Interpolation::CatmullRomInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].z,
        g_UInput[currentKeyframeIndex].z,
        g_UInput[nextKeyframeIndex].z,
        g_UInput[(nextKeyframeIndex + 1) % n].z);

    // Update global position
    g_positionX = interpolatedX;
    g_positionY = interpolatedY;
    g_positionZ = interpolatedZ;

    // Create rotation matrix from movement direction
    createRotationMatrixFromDirection(movementDirX, movementDirY, movementDirZ, g_rotation);

    // Set translation components in the rotation matrix
    g_rotation[12] = (GLfloat)g_positionX;
    g_rotation[13] = (GLfloat)g_positionY;
    g_rotation[14] = (GLfloat)g_positionZ;
}

void BsplineInterpolate(double t, int currentKeyframeIndex, int nextKeyframeIndex) {
    // Perform Bspline interpolation for position
    double interpolatedX = Interpolation::BSplinePositionInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].x,
        g_UInput[currentKeyframeIndex].x,
        g_UInput[nextKeyframeIndex].x,
        g_UInput[(nextKeyframeIndex + 1) % n].x);

    double interpolatedY = Interpolation::BSplinePositionInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].y,
        g_UInput[currentKeyframeIndex].y,
        g_UInput[nextKeyframeIndex].y,
        g_UInput[(nextKeyframeIndex + 1) % n].y);

    double interpolatedZ = Interpolation::BSplinePositionInterpolation(t,
        g_UInput[(currentKeyframeIndex - 1 + n) % n].z,
        g_UInput[currentKeyframeIndex].z,
        g_UInput[nextKeyframeIndex].z,
        g_UInput[(nextKeyframeIndex + 1) % n].z);

    // Update global position
    g_positionX = interpolatedX;
    g_positionY = interpolatedY;
    g_positionZ = interpolatedZ;

    // Create rotation matrix from movement direction
    createRotationMatrixFromDirection(movementDirX, movementDirY, movementDirZ, g_rotation);

    // Set translation components in the rotation matrix
    g_rotation[12] = (GLfloat)g_positionX;
    g_rotation[13] = (GLfloat)g_positionY;
    g_rotation[14] = (GLfloat)g_positionZ;
}

void update(void) {
    double keyframeDuration = 2.0; // seconds per segment
    int totalSegments = n - 1;
    double segmentTime = fmod(totalTime, keyframeDuration * totalSegments);
    int currentKeyframeIndex = static_cast<int>(segmentTime / keyframeDuration);
    int nextKeyframeIndex = (currentKeyframeIndex + 1) % n;
    double t = (segmentTime - currentKeyframeIndex * keyframeDuration) / keyframeDuration;

    t = std::min(1.0, std::max(0.0, t));

    // Store previous position
    prevPositionX = g_positionX;
    prevPositionY = g_positionY;
    prevPositionZ = g_positionZ;

    // Compute the new position and orientation
    if (g_splineType == CatMullRom) {
        CatMullRomInterpolate(t, currentKeyframeIndex, nextKeyframeIndex);
    }
    else {
        BsplineInterpolate(t, currentKeyframeIndex, nextKeyframeIndex);
    }

    // Compute movement direction vector
    double dirX = g_positionX - prevPositionX;
    double dirY = g_positionY - prevPositionY;
    double dirZ = g_positionZ - prevPositionZ;

    // Normalize the direction vector
    double length = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (length > 0.0) {
        dirX /= length;
        dirY /= length;
        dirZ /= length;
    }
    else {
        // Default direction if movement is zero
        dirX = 0.0;
        dirY = 0.0;
        dirZ = -1.0; // Facing negative Z-axis
    }

    // Store movement direction
    movementDirX = dirX;
    movementDirY = dirY;
    movementDirZ = dirZ;

    // Determine dominant movement axis
    if (fabs(movementDirX) > fabs(movementDirZ)) {
        // Moving along X-axis
        isMovingAlongX = true;
    }
    else {
        // Moving along Z-axis
        isMovingAlongX = false;
    }

    // Leg animation using sine wave
    double walkCycle = totalTime * legSwingSpeed;
    leftlegAngle = legSwingAmplitude * sin(walkCycle);
    rightlegAngle = legSwingAmplitude * sin(walkCycle + M_PI); // Opposite phase
}

void renderGround() {
    // Draw ground as a large rectangle
    glColor3f(0.3f, 0.9f, 0.3f); // Green color
    glBegin(GL_QUADS);
    glVertex3f(-100.0f, -5.0f, -100.0f);
    glVertex3f(100.0f, -5.0f, -100.0f);
    glVertex3f(100.0f, -5.0f, 100.0f);
    glVertex3f(-100.0f, -5.0f, 100.0f);
    glEnd();
}

void renderControlPoints() {
    // Draw control points as small spheres
    glColor3f(1.0f, 0.0f, 0.0f); // Red color
    for (const auto& point : controlPoints) {
        glPushMatrix();
        glTranslatef(point.x, point.y, point.z);
        glutSolidSphere(0.3f, 10, 10);
        glPopMatrix();
    }
}

void render(void) {
    // Clear buffer
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Light blue background
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render state
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Light source attributes
    GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

    // Surface material attributes
    GLfloat material_Ka[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat material_Kd[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat material_Ks[] = { 0.9f, 0.9f, 0.9f, 1.0f };
    GLfloat material_Ke[] = { 0.1f, 0.0f, 0.1f, 1.0f };
    GLfloat material_Se = 10.0f;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Applying camera transforms
    gluLookAt(0.0f, 40.0f, 50.0f,   // Eye position
        0.0f, 0.0f, 0.0f,    // Look-at point
        0.0f, 1.0f, 0.0f);   // Up direction

    // Render ground
    glDisable(GL_LIGHTING); // Disable lighting for ground
    renderGround();
    glEnable(GL_LIGHTING);  // Re-enable lighting

    // Render control points
    glDisable(GL_LIGHTING); // Disable lighting for control points
    renderControlPoints();
    glEnable(GL_LIGHTING);  // Re-enable lighting

    // Determine the leg swing axis based on movement direction
    double swingAxisX = 0.0;
    double swingAxisY = 0.0;
    double swingAxisZ = 0.0;

    if (isMovingAlongX) {
        // Moving along X-axis, swing legs along X-axis because we change the direction of torso into direction of movement
        // Rotate around X-axis
        swingAxisX = 1.0;
    }
    else {
        // Moving along Z-axis, swing legs along X-axis
        // Rotate around Z-axis
        swingAxisX = 1.0;
    }

    // Render Torso
    glPushMatrix();
    glMultMatrixf(g_rotation);

    // Adjust the body to be a cuboid
    //glColor3f(0.8f, 0.6f, 0.4f); // Brown color for torso
    drawCuboid(3.0f, 5.0f, 3.0f); // width, height, depth

    // Left Leg
    glPushMatrix();
    glTranslatef(-0.5f, -2.0f, 0.0f);
    glRotatef(leftlegAngle, (GLfloat)swingAxisX, (GLfloat)swingAxisY, (GLfloat)swingAxisZ);
    glColor3f(0.4f, 0.4f, 0.8f);
    drawCuboid(0.5f, 5.0f, 0.5f);
    glPopMatrix();

    // Right Leg
    glPushMatrix();
    glTranslatef(0.5f, -2.0f, 0.0f);
    glRotatef(rightlegAngle, (GLfloat)swingAxisX, (GLfloat)swingAxisY, (GLfloat)swingAxisZ);
    glColor3f(0.4f, 0.4f, 0.8f);
    drawCuboid(0.5f, 5.0f, 0.5f);
    glPopMatrix();

    glPopMatrix();

    // Disable lighting
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);

    // Swap back and front buffers
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
    // Handle keyboard input if needed
    if (key == 27) { // ESC key
        exit(0);
    }
}

void reshape(int w, int h) {
    // Screen size
    g_screenWidth = w;
    g_screenHeight = h;

    // Viewport
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    // Projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (GLfloat)w / (GLfloat)h, 1.0f, 2000.0f);
}

void timer(int value) {
    // Get current time
    double currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0; // time in seconds
    double frameTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;
    totalTime += dt; // Use dt from input file

    update();

    // Render
    glutPostRedisplay();

    // Reset timer
    glutTimerFunc(16, timer, 0);
}

int main(int argc, char** argv) {
    // Get input from file
    getUInputFromFile("input.txt");

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_screenWidth, g_screenHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Walking Torso");

    // Init
    init();

    // Set callback functions
    glutDisplayFunc(render);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, timer, 0);

    // Main loop
    glutMainLoop();

    delete[] g_UInput;

    return 0;
}
