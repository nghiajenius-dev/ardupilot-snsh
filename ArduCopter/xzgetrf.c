/*
 * File: xzgetrf.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 08-Nov-2017 15:09:06
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v2.h"
#include "xzgetrf.h"

/* Function Definitions */

/*
 * Arguments    : double A[36]
 *                int ipiv[6]
 *                int *info
 * Return Type  : void
 */
void xzgetrf(double A[36], int ipiv[6], int *info)
{
  int i0;
  int j;
  int c;
  int iy;
  int ix;
  double smax;
  int jy;
  double s;
  int b_j;
  int ijA;
  for (i0 = 0; i0 < 6; i0++) {
    ipiv[i0] = 1 + i0;
  }

  *info = 0;
  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 2; jy <= 6 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy - 1;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (j + iy) + 1;
        ix = j;
        iy += j;
        for (jy = 0; jy < 6; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i0 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i0; iy++) {
        A[iy] /= A[c];
      }
    } else {
      *info = j + 1;
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i0 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i0; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  if ((*info == 0) && (!(A[35] != 0.0))) {
    *info = 6;
  }
}

/*
 * File trailer for xzgetrf.c
 *
 * [EOF]
 */
