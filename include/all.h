#ifndef _ALL_H
#define _ALL_H

#include <iostream>
#include <fstream>
#include <string>

#include "Types.h"

/* Ceres */
#include "ceres/ceres.h"
#include "ceres/types.h"
#include "ceres/rotation.h"

/* Logging */
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "Eigen/Core"


typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;


/* Rotations */
///Rotation around X,Y,Z (Angles[3]). 
///The second and third rotations are done around the rotated axes of the object's frame
template <typename T>
void Rot3D(const T* const Angles, T R[3][3] )
{
    R[0][0] = cos(Angles[2])*cos(Angles[1]);
    R[0][1] = -sin(Angles[2])*cos(Angles[1]);
    R[0][2] = sin(Angles[1]);
         
    R[1][0] = cos(Angles[2])*sin(Angles[1])*sin(Angles[0]) + sin(Angles[2])*cos(Angles[0]);
    R[1][1] = -sin(Angles[2])*sin(Angles[1])*sin(Angles[0]) + cos(Angles[2])*cos(Angles[0]);
    R[1][2] = -cos(Angles[1])*sin(Angles[0]);
         
    R[2][0] = -cos(Angles[2])*sin(Angles[1])*cos(Angles[0]) + sin(Angles[2])*sin(Angles[0]);
    R[2][1] = sin(Angles[2])*sin(Angles[1])*cos(Angles[0]) + cos(Angles[2])*sin(Angles[0]);
    R[2][2] = cos(Angles[1])*cos(Angles[0]);
 
}



template <typename T>
void Rot3D(const T* const Angles, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = cos(Angles[2])*cos(Angles[1]);
    R(0,1) = -sin(Angles[2])*cos(Angles[1]);
    R(0,2) = sin(Angles[1]);

    R(1,0) = cos(Angles[2])*sin(Angles[1])*sin(Angles[0]) + sin(Angles[2])*cos(Angles[0]);
    R(1,1) = -sin(Angles[2])*sin(Angles[1])*sin(Angles[0]) + cos(Angles[2])*cos(Angles[0]);
    R(1,2) = -cos(Angles[1])*sin(Angles[0]);

    R(2,0) = -cos(Angles[2])*sin(Angles[1])*cos(Angles[0]) + sin(Angles[2])*sin(Angles[0]);
    R(2,1) = sin(Angles[2])*sin(Angles[1])*cos(Angles[0]) + cos(Angles[2])*sin(Angles[0]);
    R(2,2) = cos(Angles[1])*cos(Angles[0]);

}

template <typename T>
void Rot3D(const Eigen::Matrix<T,3,1>& Angles, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = cos(Angles[2])*cos(Angles[1]);
    R(0,1) = -sin(Angles[2])*cos(Angles[1]);
    R(0,2) = sin(Angles[1]);

    R(1,0) = cos(Angles[2])*sin(Angles[1])*sin(Angles[0]) + sin(Angles[2])*cos(Angles[0]);
    R(1,1) = -sin(Angles[2])*sin(Angles[1])*sin(Angles[0]) + cos(Angles[2])*cos(Angles[0]);
    R(1,2) = -cos(Angles[1])*sin(Angles[0]);

    R(2,0) = -cos(Angles[2])*sin(Angles[1])*cos(Angles[0]) + sin(Angles[2])*sin(Angles[0]);
    R(2,1) = sin(Angles[2])*sin(Angles[1])*cos(Angles[0]) + cos(Angles[2])*sin(Angles[0]);
    R(2,2) = cos(Angles[1])*cos(Angles[0]);

}

template <typename T>
void RotX(const T* const Angle, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = 1;
    R(0,1) = 0;
    R(0,2) = 0;

    R(1,0) = 0;
    R(1,1) = cos(Angle);
    R(1,2) = -sin(Angle);

    R(2,0) = 0;
    R(2,1) = sin(Angle);
    R(2,2) = cos(Angle);
}


template <typename T>
void RotY(const T* const Angle, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = cos(Angle);
    R(0,1) = 0;
    R(0,2) = sin(Angle);

    R(1,0) = 0;
    R(1,1) = 1;
    R(1,2) = 0;

    R(2,0) = -sin(Angle);
    R(2,1) = 0;
    R(2,2) = cos(Angle);

}


template <typename T>
void RotZ(const T* const Angle, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = cos(Angle);
    R(0,1) = -sin(Angle);
    R(0,2) = 0;

    R(1,0) = sin(Angle);
    R(1,1) = cos(Angle);
    R(1,2) = 0;

    R(2,0) = 0;
    R(2,1) = 0;
    R(2,2) = 1;
}


/* Some useful matrix operations */
/* POut = R * (PIn - Tr)  */
template <typename T>
void RotTr (T R[3][3],T Tr[3] ,T PIn[3] ,T POut[3])
{
    T PIn_Tr[3] = { PIn[0] - Tr[0],
                    PIn[1] - Tr[1], 
                    PIn[2] - Tr[2]};

    POut[0] = R[0][0]*PIn_Tr[0] + R[0][1]*PIn_Tr[1] + R[0][2]*PIn_Tr[2];
    POut[1] = R[1][0]*PIn_Tr[0] + R[1][1]*PIn_Tr[1] + R[1][2]*PIn_Tr[2];
    POut[2] = R[2][0]*PIn_Tr[0] + R[2][1]*PIn_Tr[1] + R[2][2]*PIn_Tr[2];

}

template <typename T>
bool FileReadOK(FILE *fptr, const char *format, T *value)
{
    int OK = fscanf(fptr, format, value);
    if (OK != 1)
        return false;
    else
        return true; 
}

#endif //_ALL_H
