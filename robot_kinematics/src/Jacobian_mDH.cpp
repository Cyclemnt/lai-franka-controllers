//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Jacobian_mDH.cpp
//
// Code generation for function 'Jacobian_mDH'
//

// Include files
#include <robot_kinematics/Jacobian_mDH.h>
#include <robot_kinematics/DirectKinematics_mDH.h>
#include <emmintrin.h>

namespace robot_kinematics {

// Function Definitions
void Jacobian_mDH(const double mDH[28], const double A7e[16], double J[42])
{
  __m128d r;
  double T0[112];
  double Jw[21];
  double p[21];
  double z[21];
  double b[3];
  int b_p_tmp;
  int p_tmp;
  //  --------------------------------- Variables
  //  -------------------------------- %
  //  --- Number of DoFs
  //  ----------------------------------- Core
  //  ----------------------------------- %
  //  --- DirectKinematics
  DirectKinematics_mDH(mDH, A7e, T0);
  //  --- Position and z-axis Vectors
  for (int i{0}; i < 7; i++) {
    p_tmp = i << 4;
    p[3 * i] = T0[p_tmp + 12];
    z[3 * i] = T0[p_tmp + 8];
    b_p_tmp = 3 * i + 1;
    p[b_p_tmp] = T0[p_tmp + 13];
    z[b_p_tmp] = T0[p_tmp + 9];
    b_p_tmp = 3 * i + 2;
    p[b_p_tmp] = T0[p_tmp + 14];
    z[b_p_tmp] = T0[p_tmp + 10];
  }
  //  --- End-Effector Position Vector
  //  --- Jacobian Matrix
  r = _mm_loadu_pd(&p[18]);
  for (int i{0}; i < 7; i++) {
    __m128d r1;
    double Jw_tmp;
    double d;
    double d1;
    r1 = _mm_loadu_pd(&p[3 * i]);
    _mm_storeu_pd(&b[0], _mm_sub_pd(r, r1));
    r1 = _mm_loadu_pd(&z[3 * i]);
    _mm_storeu_pd(&Jw[3 * i], r1);
    p_tmp = 3 * i + 2;
    b[2] = p[20] - p[p_tmp];
    Jw_tmp = z[p_tmp];
    Jw[p_tmp] = Jw_tmp;
    b_p_tmp = 3 * i + 1;
    d = z[b_p_tmp];
    d1 = z[3 * i];
    J[6 * i] = d * b[2] - b[1] * Jw_tmp;
    J[6 * i + 3] = Jw[3 * i];
    J[6 * i + 1] = b[0] * Jw_tmp - d1 * b[2];
    J[6 * i + 4] = Jw[b_p_tmp];
    J[6 * i + 2] = d1 * b[1] - b[0] * d;
    J[6 * i + 5] = Jw[p_tmp];
  }
}

} // namespace robot_kinematics

// End of code generation (Jacobian_mDH.cpp)
