/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 25-Oct-2017 16:07:02
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LeastSquare_NJ.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * Arguments    : const double A[9]
 *                const double B[3]
 *                double Y[3]
 * Return Type  : void
 */
void mldivide(const double A[9], const double B[3], double Y[3])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabs(b_A[3 + r3]) > fabs(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[3 + r3];
  Y[2] /= b_A[6 + r3];
  Y[0] = B[r1] - Y[2] * b_A[6 + r1];
  Y[1] -= Y[2] * b_A[6 + r2];
  Y[1] /= b_A[3 + r2];
  Y[0] -= Y[1] * b_A[3 + r1];
  Y[0] /= b_A[r1];
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
