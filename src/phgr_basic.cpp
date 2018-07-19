#include "all.h"

///Rotation around X,Y,Z (Angles[3]). 
///The second and third rotations are done around the rotated axes of the object's frame


/* POut = R * (PIn - Tr)  */
template <typename T>
static void RotTr (T[3][3] R,T[3] Tr,T[3] PIn,T[3] POut)
{
    T[3] PIn_Tr = { PIn[0] - Tr[0], 
                    PIn[1] - Tr[1], 
                    PIn[2] - Tr[2]};

    POut[0] = R[0][0]*PIn_Tr[0] + R[0][1]*PIn_Tr[1] + R[0][2]*PIn_Tr[2];
    POut[1] = R[1][0]*PIn_Tr[0] + R[1][1]*PIn_Tr[1] + R[1][2]*PIn_Tr[2];
    POut[2] = R[2][0]*PIn_Tr[0] + R[2][1]*PIn_Tr[1] + R[2][2]*PIn_Tr[2];

}

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
static void Rot3D(const T* Angles, T[3][3] R)
{
    R[0][0] = cos(Angles[2])*cos(Angles[1]);
    R[0][1] = -sin(Angles[2])*cos(Angles[1]);
    R[0][2] = sin(Angles[1]);
         
    R[1][0] = cos(Angles[2])*sin(Angles[1])*sin(Angles[0]) + sin(Angles[2])*cos(Angle[0]);
    R[1][1] = -sin(Angles[2])*sin(Angles[1])*sin(Angles[0]) + cos(Angles[2])*cos(Angle[0]);
    R[1][2] = -cos(Angle[1])*sin(Angle[0]);
         
    R[2][0] = -cos(Angles[2])*sin(Angles[1])*cos(Angle[0]) + sin(Angles[2])*sin(Angles[0]);
    R[2][1] = sin(Angles[2])*sin(Angles[1])*cos(Angle[0]) + cos(Angle[2])*sin(Angles[0]);
    R[2][2] = cos(Angles[1])*cos(Angles[0]);
    
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


