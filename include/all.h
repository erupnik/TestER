#ifndef _ALL_H
#define _ALL_H

#include <iostream>
#include <fstream>
#include <string>


/* Ceres */
#include "ceres/ceres.h"
#include "ceres/types.h"
#include "ceres/rotation.h"

/* Logging */
#include "gflags/gflags.h"
#include "glog/logging.h"

/* Rotations, definitions in Rotations.h */
template <typename T>
static void Rot3D(const T* Angle, ceres::MatrixAdapter<T, 3, 3>& R);

template <typename T>
static void RotX(const T* Angle, ceres::MatrixAdapter<T, 3, 3>& R);

template <typename T>
static void RotY(const T* Angle, ceres::MatrixAdapter<T, 3, 3>& R);

template <typename T>
static void RotZ(const T* Angle, ceres::MatrixAdapter<T, 3, 3>& R);

#endif //_ALL_H
