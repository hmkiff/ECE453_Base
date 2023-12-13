// Harry K. Dec 4, 2023
#ifndef VECTOR_UTIL_H_
#define VECTOR_UTIL_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


struct VEC2D {
    double i;
    double j;
};

struct MAT2{
    double a;
    double b;
    double c;
    double d;
};

#define DTR 0.01745329  // Degrees to Radians

// returns the sign of a float
int signf(float num);

// Add two 2D vectors
struct VEC2D addVec(struct VEC2D vec_a, struct VEC2D vec_b);

// Subtract 2D vector B FROM 2D vector A
struct VEC2D subVec(struct VEC2D vec_a, struct VEC2D vec_b);

// Multiply all values in 2DVector A by a scalar
struct VEC2D scalarMultVec(struct VEC2D vec_a, double scalar);

// Dot product of two 2D vectors
double dot2DVec(struct VEC2D vec_a, struct VEC2D vec_b);

// Rotates a 2D vector using a 2x2 Rotation matrix
struct VEC2D rotVec(struct MAT2 mat, struct VEC2D vec);

struct VEC2D squareVec(struct VEC2D vec_a);

double sumVec(struct VEC2D vec_a);


#endif