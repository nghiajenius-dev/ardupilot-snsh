/*
 * File: LeastSquare_NJ.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 06-Nov-2017 15:15:42
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LeastSquare_NJ.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * NOR: max number of receivers = 5
 *  max_nls_NOR = NOR for LLS/NLS = 4
 *  RCM: Receiver Coordinate Matrix [3,NOR]
 *  MR: Measured Range [1,NOR]
 *  method: 1-lls, 2-nls
 *  persistent orderTemp max_nls_NOR
 * Arguments    : short NOR
 *                double updateMR[5]
 *                double updateRCM[15]
 *                short method
 *                double R_OP[3]
 * Return Type  : void
 */
void LeastSquare_NJ(short NOR, double updateMR[5], double updateRCM[15], short
                    method, double R_OP[3])
{
  int i;
  double lsRCM[12];
  double lsMR[4];
  int i0;
  short b_i;
  short j;
  double r1square;
  int k;
  double a;
  double A[9];
  double b_A[9];
  double Rold[3];
  double b[3];
  double scale;
  double b_a;
  double R_est[3];
  int i_count_nls;
  double x;
  double y;
  double z;
  double dv0[3];

  /*  last node */
  for (i = 0; i < 4; i++) {
    lsMR[i] = 0.0;
  }

  memset(&lsRCM[0], 0, 12U * sizeof(double));
  for (i0 = 0; i0 < 3; i0++) {
    R_OP[i0] = 0.0;
  }

  /*  orderTemp = 0; */
  /*  sort distance */
  if (NOR > 4) {
    for (b_i = 1; b_i <= NOR - 2; b_i++) {
      for (j = (short)(NOR - 1); j >= b_i + 1; j--) {
        if (updateMR[j - 1] < updateMR[j - 2]) {
          r1square = updateMR[j - 1];
          updateMR[j - 1] = updateMR[j - 2];
          updateMR[j - 2] = r1square;
          for (k = 0; k < 3; k++) {
            r1square = updateRCM[(j + 5 * k) - 1];
            updateRCM[(j + 5 * k) - 1] = updateRCM[(j + 5 * k) - 2];
            updateRCM[(j + 5 * k) - 2] = r1square;
          }
        }
      }
    }

    /*  change NOR -> 4 */
  }

  /*  Save to lsMR, lsRCM */
  /* NOR = 4 */
  /*  Update center node */
  lsMR[3] = updateMR[NOR - 1];
  for (i = 0; i < 3; i++) {
    lsMR[i] = updateMR[i];
    for (k = 0; k < 3; k++) {
      lsRCM[i + (k << 2)] = updateRCM[i + 5 * k];
    }

    lsRCM[3 + (i << 2)] = updateRCM[(NOR + 5 * i) - 1];
  }

  /*  lsMR */
  /*  lsRCM */
  /*  Run LLS(max_nls_NOR=4, lsMR,lsRCM) */
  r1square = lsMR[0] * lsMR[0];
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      A[i + 3 * k] = lsRCM[(i + (k << 2)) + 1] - lsRCM[k << 2];
    }

    a = lsRCM[i + 1] - lsRCM[0];
    scale = lsRCM[i + 5] - lsRCM[4];
    b_a = lsRCM[i + 9] - lsRCM[8];
    b[i] = 0.5 * ((r1square - lsMR[i + 1] * lsMR[i + 1]) + ((a * a + scale *
      scale) + b_a * b_a));
  }

  /*  LLS solution */
  for (i0 = 0; i0 < 3; i0++) {
    Rold[i0] = 0.0;
    for (k = 0; k < 3; k++) {
      b_A[i0 + 3 * k] = 0.0;
      for (i = 0; i < 3; i++) {
        b_A[i0 + 3 * k] += A[i + 3 * i0] * A[i + 3 * k];
      }

      Rold[i0] += A[k + 3 * i0] * b[k];
    }

    R_est[i0] = 0.0;
  }

  mldivide(b_A, Rold, b);

  /*  Estimated position coordinate */
  for (i = 0; i < 2; i++) {
    R_est[i] = b[i] + lsRCM[i << 2];
  }

  a = R_est[0] - lsRCM[0];
  scale = R_est[1] - lsRCM[4];
  r1square = (lsMR[0] * lsMR[0] - a * a) - scale * scale;
  if (r1square >= 0.0) {
    R_est[2] = lsRCM[8] - sqrt(r1square);
  }

  if (method == 1) {
    for (i0 = 0; i0 < 3; i0++) {
      R_OP[i0] = R_est[i0];
    }
  } else {
    if (method == 2) {
      /*     %% NLS(max_nls_NOR=4, lsMR,lsRCM) */
      i_count_nls = 0;
      r1square = 0.001;
      memset(&A[0], 0, 9U * sizeof(double));
      for (i = 0; i < 3; i++) {
        b[i] = 0.0;
      }

      while ((r1square > 1.0E-6) && (i_count_nls <= 15)) {
        i_count_nls++;
        if (i_count_nls == 1) {
          for (i = 0; i < 3; i++) {
            Rold[i] = R_est[i];
          }
        } else {
          for (i0 = 0; i0 < 3; i0++) {
            Rold[i0] = R_OP[i0];
          }
        }

        x = Rold[0];
        y = Rold[1];
        z = Rold[2];
        for (i = 0; i < 4; i++) {
          a = x - lsRCM[i];
          scale = y - lsRCM[4 + i];
          b_a = z - lsRCM[8 + i];
          r1square = sqrt((a * a + scale * scale) + b_a * b_a) - lsMR[i];

          /* ----------------------------JtJ---------------------------- */
          a = r1square + lsMR[i];
          A[0] += (x - lsRCM[i]) * (x - lsRCM[i]) / (a * a);
          a = r1square + lsMR[i];
          A[3] += (x - lsRCM[i]) * (y - lsRCM[4 + i]) / (a * a);
          a = r1square + lsMR[i];
          A[6] += (x - lsRCM[i]) * (z - lsRCM[8 + i]) / (a * a);
          a = r1square + lsMR[i];
          A[4] += (y - lsRCM[4 + i]) * (y - lsRCM[4 + i]) / (a * a);
          a = r1square + lsMR[i];
          A[7] += (y - lsRCM[4 + i]) * (z - lsRCM[8 + i]) / (a * a);
          a = r1square + lsMR[i];
          A[8] += (z - lsRCM[8 + i]) * (z - lsRCM[8 + i]) / (a * a);

          /* ----------------------------Jtf---------------------------- */
          b[0] += (x - lsRCM[i]) * r1square / (r1square + lsMR[i]);
          b[1] += (y - lsRCM[4 + i]) * r1square / (r1square + lsMR[i]);
          b[2] += (z - lsRCM[8 + i]) * r1square / (r1square + lsMR[i]);
        }

        /* ----------------------------JtJ---------------------------- */
        A[1] = A[3];
        A[2] = A[6];
        A[5] = A[7];
        mldivide(A, b, dv0);
        r1square = 0.0;
        scale = 2.2250738585072014E-308;
        for (k = 0; k < 3; k++) {
          x = Rold[k] - dv0[k];
          y = fabs(x - Rold[k]);
          if (y > scale) {
            z = scale / y;
            r1square = 1.0 + r1square * z * z;
            scale = y;
          } else {
            z = y / scale;
            r1square += z * z;
          }

          R_OP[k] = x;
        }

        r1square = scale * sqrt(r1square);
      }

      /*      R_est = Rnew; */
    }
  }
}

/*
 * File trailer for LeastSquare_NJ.c
 *
 * [EOF]
 */
