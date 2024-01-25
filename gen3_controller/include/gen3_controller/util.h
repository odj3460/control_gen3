// util.h

#pragma once

#include <termios.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>

void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);

Eigen::Vector3d quintic_spline(
    double time, double time_0, double time_f, double x_0, double x_dot_0,
    double x_ddot_0, double x_f, double x_dot_f, double x_ddot_f);

double cubic(double time, double time_0, double time_f, double x_0,
             double x_f, double x_dot_0, double x_dot_f);

double cubicDot(double time, double time_0, double time_f, double x_0,
                double x_f, double x_dot_0, double x_dot_f);

Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation);

Eigen::Matrix3d rotationCubic(double time, double time_0, double time_f,
                              const Eigen::Matrix3d &rotation_0,
                              const Eigen::Matrix3d &rotation_f);

Eigen::Vector3d rotationCubicDot(double time, double time_0, double time_f,
                                 const Eigen::Matrix3d &rotation_0,
                                 const Eigen::Matrix3d &rotation_f);

double lowPassFilter(double input, double prev, double dt,
                     double cut_off_frequency);

// Eigen default type definition
namespace Eigen {
  #define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)    \
    typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
    typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
    typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

  typedef double rScalar;

  EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
  EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
}