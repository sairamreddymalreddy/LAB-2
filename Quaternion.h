#pragma once
#include <GL/glut.h>

struct Quaternion
{
    double x, y, z, w;
};

struct Matrix4x4 {
    double m[4][4];
};


void fixedToQuaternion(double roll, double pitch, double yaw, Quaternion& q);
void fixedToMatrix(double roll, double pitch, double yaw, GLfloat* rotation);
void quaternionToMatrix(const Quaternion& q, GLfloat* rotation);

