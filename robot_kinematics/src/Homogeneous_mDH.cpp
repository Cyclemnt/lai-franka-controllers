//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Homogeneous_mDH.cpp
//
// Code generation for function 'Homogeneous_mDH'
//

// Include files
#include <robot_kinematics/Homogeneous_mDH.h>
#include <cmath>

namespace robot_kinematics {

// Function Definitions
void Homogeneous_mDH(const double mDH_row[4], double T[16])
{
  double ca;
  double ct;
  double sa;
  double st;
  //  ----------------------------------------------------------------------------
  //  %
  //                                   DESCRIPTION %
  //  ----------------------------------------------------------------------------
  //  % Compute the Homogeneous transformation matrix between consecutive frames
  //  according to DH convention input:
  //    mDH_row     dim 1x4     i-th row of the modified Denavit-Hartenberg
  //    table
  //  output:
  //    T           dim 4x4     Homogeneous transformation matrix
  //  ----------------------------------------------------------------------------
  //  %
  //  --------------------------------- Variables
  //  -------------------------------- %
  ct = std::cos(mDH_row[3]);
  st = std::sin(mDH_row[3]);
  ca = std::cos(mDH_row[1]);
  sa = std::sin(mDH_row[1]);
  //  ----------------------------------- Core
  //  ----------------------------------- %
  T[0] = ct;
  T[4] = -st;
  T[8] = 0.0;
  T[12] = mDH_row[0];
  T[1] = st * ca;
  T[5] = ct * ca;
  T[9] = -sa;
  T[13] = -mDH_row[2] * sa;
  T[2] = st * sa;
  T[6] = ct * sa;
  T[10] = ca;
  T[14] = mDH_row[2] * ca;
  T[3] = 0.0;
  T[7] = 0.0;
  T[11] = 0.0;
  T[15] = 1.0;
}

} // namespace robot_kinematics

// End of code generation (Homogeneous_mDH.cpp)
