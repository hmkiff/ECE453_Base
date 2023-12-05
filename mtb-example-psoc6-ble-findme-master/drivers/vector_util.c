#include "vector_util.h"

int signf(float num){
    if(num>0){
        return  1;
    } else if(num < 0){
        return -1;
    } else {
        return 0;
    }
}

// Add two 2D vectors
struct VEC2D addVec(struct VEC2D vec_a, struct VEC2D vec_b){
    struct VEC2D rslt;
    rslt.i = vec_a.i + vec_b.i;
    rslt.j = vec_a.j + vec_b.j;
    return rslt;
}

// Subtract 2D vector B FROM 2D vector A
struct VEC2D subVec(struct VEC2D vec_a, struct VEC2D vec_b){
    struct VEC2D rslt;
    rslt.i = vec_a.i - vec_b.i;
    rslt.j = vec_a.j - vec_b.j;
    return rslt;    
}

// Multiply all values in 2DVector A by a scalar
struct VEC2D scalarMultVec(struct VEC2D vec_a, double scalar){
    struct VEC2D rslt;
    rslt.i = vec_a.i * scalar;
    rslt.j = vec_a.j * scalar;
    return rslt; 
}

// 2DVector dot product
double dot2DVec(struct VEC2D vec_a, struct VEC2D vec_b){
    return (vec_a.i * vec_b.i) + (vec_a.j * vec_b.j);
}

struct VEC2D rotVec(struct MAT2 mat, struct VEC2D vec)
{
    struct VEC2D out;
    out.i = (vec.i * mat.a) + (vec.j*mat.b);
    out.j = (vec.i * mat.c) + (vec.j*mat.d);
    return out;
}

struct VEC2D squareVec(struct VEC2D vec_a)
{
    struct VEC2D out;
    out.i = vec_a.i * vec_a.i;
    out.j = vec_a.j * vec_a.j;
    return out;
}

double sumVec(struct VEC2D vec_a)
{
    return (vec_a.i + vec_a.j);
}
