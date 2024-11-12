#include "stdafx.h"
#include<math.h>
#include<cmath>
#include "Quaternion.h"



void eulerToQuaternion(double roll, double pitch, double yaw, Quaternion& q) {
    // Calculate half angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    // Calculate quaternion components
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

void quaternionToMatrix(const Quaternion& q, GLfloat* rotation)
{
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;

    double xx = x * x;
    double xy = x * y;
    double xz = x * z;
    double xw = x * w;

    double yy = y * y;
    double yz = y * z;
    double yw = y * w;

    double zz = z * z;
    double zw = z * w;

    rotation[0] = 1 - 2 * (yy + zz);
    rotation[1] = 2 * (xy + zw);
    rotation[2] = 2 * (xz - yw);
    rotation[3] = 0;

    rotation[4] = 2 * (xy - zw);
    rotation[5] = 1 - 2 * (xx + zz);
    rotation[6] = 2 * (yz + xw);
    rotation[7] = 0;

    rotation[8] = 2 * (xz + yw);
    rotation[9] = 2 * (yz - xw);
    rotation[10] = 1 - 2 * (xx + yy);
    rotation[11] = 0;

    rotation[12] = 0;
    rotation[13] = 0;
    rotation[14] = 0;
    rotation[15] = 1;
}
GLfloat* eulerToMatrix(double roll, double pitch, double yaw)
{
    // Precompute trigonometric values for efficiency
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Matrix4x4 g_rotationMatrix;

    // Fill the transformation matrix
    g_rotationMatrix.m[0][0] = cos_yaw * cos_pitch;
    g_rotationMatrix.m[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    g_rotationMatrix.m[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    g_rotationMatrix.m[0][3] = 0;

    g_rotationMatrix.m[1][0] = sin_yaw * cos_pitch;
    g_rotationMatrix.m[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    g_rotationMatrix.m[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    g_rotationMatrix.m[1][3] = 0;

    g_rotationMatrix.m[2][0] = -sin_pitch;
    g_rotationMatrix.m[2][1] = cos_pitch * sin_roll;
    g_rotationMatrix.m[2][2] = cos_pitch * cos_roll;
    g_rotationMatrix.m[2][3] = 0;

    g_rotationMatrix.m[3][0] = 0.0;
    g_rotationMatrix.m[3][1] = 0.0;
    g_rotationMatrix.m[3][2] = 0.0;
    g_rotationMatrix.m[3][3] = 1.0;

    GLfloat rotation[16];

    int index = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rotation[index] = g_rotationMatrix.m[row][col];
            index++;
        }
    }

    return rotation;
}