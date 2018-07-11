#include "all.h"

///Rotation around X,Y,Z (Angles[3]). 
///The second and third rotations are done around the rotated axes of the object's frame
template <typename T>
static void Rot3D(const T* Angles, Eigen::Matrix<T,3,3>& R)
{
    R(0,0) = cos(Angles[2])*cos(Angles[1]);
    R(0,1) = -sin(Angles[2])*cos(Angles[1]);
    R(0,2) = sin(Angles[1]);

    R(1,0) = cos(Angles[2])*sin(Angles[1])*sin(Angles[0]) + sin(Angles[2])*cos(Angle[0]);
    R(1,1) = -sin(Angles[2])*sin(Angles[1])*sin(Angles[0]) + cos(Angles[2])*cos(Angle[0]);
    R(1,2) = -cos(Angle[1])*sin(Angle[0]);
    
    R(2,0) = -cos(Angles[2])*sin(Angles[1])*cos(Angle[0]) + sin(Angles[2])*sin(Angles[0]);
    R(2,1) = sin(Angles[2])*sin(Angles[1])*cos(Angle[0]) + cos(Angle[2])*sin(Angles[0]);
    R(2,2) = cos(Angles[1])*cos(Angles[0]);
    
}

template <typename T>
static void RotX(const T* Angle, Eigen::Matrix<T,3,3>& R)
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
static void RotY(const T* Angle, Eigen::Matrix<T,3,3>& R)
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
static void RotZ(const T* Angle, Eigen::Matrix<T,3,3>& R)
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


