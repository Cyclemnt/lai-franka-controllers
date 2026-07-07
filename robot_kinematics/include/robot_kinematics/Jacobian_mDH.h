//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Jacobian_mDH.h
//
// Code generation for function 'Jacobian_mDH'
//

#ifndef JACOBIAN_MDH_H
#define JACOBIAN_MDH_H

// Include files
#include <cstddef>
#include <cstdlib>

namespace robot_kinematics {

// Function Declarations
extern void Jacobian_mDH(const double mDH[28], const double A7e[16],
                         double J[42]);

} // namespace robot_kinematics

#endif
// End of code generation (Jacobian_mDH.h)
