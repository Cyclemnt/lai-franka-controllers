//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// DirectKinematics_mDH.cpp
//
// Code generation for function 'DirectKinematics_mDH'
//

// Include files
#include <robot_kinematics/DirectKinematics_mDH.h>
#include <robot_kinematics/Homogeneous_mDH.h>
#include <cstring>

namespace robot_kinematics {

// Function Definitions
void DirectKinematics_mDH(const double mDH[28], const double A7e[16],
                          double T0[112])
{
  double T[112];
  double b_T0[16];
  int T0_tmp;
  int b_T0_tmp;
  //  --------------------------------- Variables
  //  -------------------------------- %
  //  --- Number of DoFs
  std::memset(&T0[0], 0, 112U * sizeof(double));
  //  ----------------------------------- Core
  //  ----------------------------------- %
  //  --- Homogeneous Transformation Matrix between two consecutive frames
  //  according to the modified DH convention
  for (int i{0}; i < 7; i++) {
    double b_mDH[4];
    b_mDH[0] = mDH[i];
    b_mDH[1] = mDH[i + 7];
    b_mDH[2] = mDH[i + 14];
    b_mDH[3] = mDH[i + 21];
    Homogeneous_mDH(b_mDH, &T[i << 4]);
  }
  //  --- Homogeneous Transformation Matrices wrt the arm base frame.
  //  --- T0(:,:,n) contains the Homogeneous Transformation Matrix between the
  //  end-effector frame and the arm base frame
  for (int b_i{0}; b_i < 4; b_i++) {
    T0_tmp = b_i << 2;
    T0[T0_tmp] = T[T0_tmp];
    T0[T0_tmp + 1] = T[T0_tmp + 1];
    T0[T0_tmp + 2] = T[T0_tmp + 2];
    T0[T0_tmp + 3] = T[T0_tmp + 3];
  }
  for (int i{0}; i < 6; i++) {
    for (int b_i{0}; b_i < 4; b_i++) {
      T0_tmp = b_i + (i << 4);
      for (b_T0_tmp = 0; b_T0_tmp < 4; b_T0_tmp++) {
        int i1;
        int i2;
        i1 = b_T0_tmp << 2;
        i2 = i1 + ((i + 1) << 4);
        b_T0[b_i + i1] = ((T0[T0_tmp] * T[i2] + T0[T0_tmp + 4] * T[i2 + 1]) +
                          T0[T0_tmp + 8] * T[i2 + 2]) +
                         T0[T0_tmp + 12] * T[i2 + 3];
      }
    }
    for (int b_i{0}; b_i < 4; b_i++) {
      T0_tmp = b_i << 2;
      b_T0_tmp = T0_tmp + ((i + 1) << 4);
      T0[b_T0_tmp] = b_T0[T0_tmp];
      T0[b_T0_tmp + 1] = b_T0[T0_tmp + 1];
      T0[b_T0_tmp + 2] = b_T0[T0_tmp + 2];
      T0[b_T0_tmp + 3] = b_T0[T0_tmp + 3];
    }
  }
  for (int b_i{0}; b_i < 4; b_i++) {
    double d;
    double d1;
    double d2;
    double d3;
    d = T0[b_i + 96];
    d1 = T0[b_i + 100];
    d2 = T0[b_i + 104];
    d3 = T0[b_i + 108];
    for (T0_tmp = 0; T0_tmp < 4; T0_tmp++) {
      b_T0_tmp = T0_tmp << 2;
      b_T0[b_i + b_T0_tmp] = ((d * A7e[b_T0_tmp] + d1 * A7e[b_T0_tmp + 1]) +
                              d2 * A7e[b_T0_tmp + 2]) +
                             d3 * A7e[b_T0_tmp + 3];
    }
  }
  for (int b_i{0}; b_i < 4; b_i++) {
    T0_tmp = b_i << 2;
    T0[T0_tmp + 96] = b_T0[T0_tmp];
    T0[T0_tmp + 97] = b_T0[T0_tmp + 1];
    T0[T0_tmp + 98] = b_T0[T0_tmp + 2];
    T0[T0_tmp + 99] = b_T0[T0_tmp + 3];
  }
}

} // namespace robot_kinematics

// End of code generation (DirectKinematics_mDH.cpp)
