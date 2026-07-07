//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// DirectKinematics_mDH.h
//
// Code generation for function 'DirectKinematics_mDH'
//

#ifndef DIRECTKINEMATICS_MDH_H
#define DIRECTKINEMATICS_MDH_H

// Include files
#include <cstddef>
#include <cstdlib>

namespace robot_kinematics {

// Function Declarations
extern void DirectKinematics_mDH(const double mDH[28], const double A7e[16],
                                 double T0[112]);

} // namespace robot_kinematics

#endif
// End of code generation (DirectKinematics_mDH.h)
