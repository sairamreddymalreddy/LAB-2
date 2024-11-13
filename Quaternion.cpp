#include "stdafx.h"
#include<math.h>
#include<cmath>
#include "Quaternion.h"



void fixedToQuaternion(double roll, double pitch, double yaw, Quaternion& q) {
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
void fixedToMatrix(double roll, double pitch, double yaw, GLfloat* rotation)
{
    // Precompute trigonometric values for efficiency
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    

    // Fill the transformation matrix
    rotation[0] = cos_yaw * cos_pitch;
    rotation[1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    rotation[2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    rotation[3] = 0;

    rotation[4] = sin_yaw * cos_pitch;
    rotation[5] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    rotation[6] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    rotation[7] = 0;

    rotation[8] = -sin_pitch;
    rotation[9] = cos_pitch * sin_roll;
    rotation[10] = cos_pitch * cos_roll;
    rotation[11] = 0;

    rotation[12]= 0.0;
    rotation[13] = 0.0;
    rotation[14] = 0.0;
    rotation[15] = 1.0;

}