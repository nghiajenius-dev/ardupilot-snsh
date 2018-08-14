/*
 * File: LeastSquare2.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 14-Aug-2018 18:35:09
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LeastSquare2.h"
#include "sqrt.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * NOR: max number of receivers
 *  max_nls_NOR: NOR for LLS/NLS
 *  RCM: Receiver Coordinate Matrix [3,NOR]
 *  MR: Measured Range [1,NOR]
 *  method: 1-lls, 2-nls
 * Arguments    : short NOR
 *                short use_nls_NOR
 *                double updateMR[7]
 *                double updateRCM[21]
 *                short method
 *                double R_OP[3]
 * Return Type  : void
 */
void LeastSquare2(short NOR, short use_nls_NOR, double updateMR[7], double
                  updateRCM[21], short method, double R_OP[3])
{
  int i;
  double lsRCM[21];
  double lsMR[7];
  int i0;
  double JtJ[9];
  double Rnew[3];
  double Jtf[3];
  double lamda_k;
  double Ri[7];
  double xi[7];
  double Fmin;
  double yi[7];
  double zi[7];
  double fi[7];
  short b_i;
  short j;
  double r1square;
  double A[18];
  double b[6];
  double a;
  double b_a;
  double c_a;
  double b_A[9];
  double c_A[3];
  double X[3];
  int i_count_nls;
  double R_est[3];
  double x;
  double y;
  double z;
  double Fxyz;

  /* 5 */
  /*  last node */
  for (i = 0; i < 7; i++) {
    lsMR[i] = 0.0;
  }

  memset(&lsRCM[0], 0, 21U * sizeof(double));
  for (i0 = 0; i0 < 3; i0++) {
    Rnew[i0] = 0.0;
    R_OP[i0] = 0.0;
  }

  memset(&JtJ[0], 0, 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    Jtf[i] = 0.0;
  }

  for (i = 0; i < 7; i++) {
    Ri[i] = 0.0;
    xi[i] = 0.0;
    yi[i] = 0.0;
    zi[i] = 0.0;
    fi[i] = 0.0;
  }

  lamda_k = 0.01;

  /*  Fxyz = 0;     %sum of square error */
  Fmin = 999.0;

  /*  sort distance */
  if (NOR > 4) {
    for (b_i = 1; b_i <= NOR - 2; b_i++) {
      for (j = (short)(NOR - 1); j >= b_i + 1; j--) {
        if (updateMR[j - 1] < updateMR[j - 2]) {
          r1square = updateMR[j - 1];
          updateMR[j - 1] = updateMR[j - 2];
          updateMR[j - 2] = r1square;
          for (i = 0; i < 3; i++) {
            r1square = updateRCM[(j + 7 * i) - 1];
            updateRCM[(j + 7 * i) - 1] = updateRCM[(j + 7 * i) - 2];
            updateRCM[(j + 7 * i) - 2] = r1square;
          }
        }
      }
    }

    /*      NOR = max_nls_NOR;      % change NOR -> 4 */
  }

  /*  Save to lsMR, lsRCM */
  i0 = use_nls_NOR - 1;
  if (i0 < -32768) {
    i0 = -32768;
  }

  j = (short)i0;
  for (b_i = 1; b_i <= j; b_i++) {
    lsMR[b_i - 1] = updateMR[b_i - 1];
    for (i = 0; i < 3; i++) {
      lsRCM[(b_i + 7 * i) - 1] = updateRCM[(b_i + 7 * i) - 1];
    }
  }

  /*  Update center node */
  lsMR[use_nls_NOR - 1] = updateMR[NOR - 1];
  for (i = 0; i < 3; i++) {
    lsRCM[(use_nls_NOR + 7 * i) - 1] = updateRCM[(NOR + 7 * i) - 1];
  }

  /*  lsMR */
  /*  lsRCM */
  /*  Run LLS(max_nls_NOR, lsMR,lsRCM) */
  memset(&A[0], 0, 18U * sizeof(double));
  for (i = 0; i < 6; i++) {
    b[i] = 0.0;
  }

  r1square = lsMR[0] * lsMR[0];
  for (b_i = 1; b_i <= (short)(use_nls_NOR - 1); b_i++) {
    for (i = 0; i < 3; i++) {
      A[(b_i + 6 * i) - 1] = lsRCM[b_i + 7 * i] - lsRCM[7 * i];
    }

    a = lsRCM[b_i] - lsRCM[0];
    b_a = lsRCM[7 + b_i] - lsRCM[7];
    c_a = lsRCM[14 + b_i] - lsRCM[14];
    b[b_i - 1] = 0.5 * ((r1square - lsMR[b_i] * lsMR[b_i]) + ((a * a + b_a * b_a)
      + c_a * c_a));
  }

  /*  LLS solution */
  /*  A */
  /*  b */
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      b_A[i0 + 3 * i] = 0.0;
      for (i_count_nls = 0; i_count_nls < 6; i_count_nls++) {
        b_A[i0 + 3 * i] += A[i_count_nls + 6 * i0] * A[i_count_nls + 6 * i];
      }
    }

    c_A[i0] = 0.0;
    for (i = 0; i < 6; i++) {
      c_A[i0] += A[i + 6 * i0] * b[i];
    }

    R_est[i0] = 0.0;
  }

  mldivide(b_A, c_A, X);

  /*  Estimated position coordinate */
  for (i = 0; i < 2; i++) {
    R_est[i] = X[i] + lsRCM[7 * i];
  }

  /*  Bug z */
  a = R_est[0] - lsRCM[0];
  b_a = R_est[1] - lsRCM[7];
  r1square = (lsMR[0] * lsMR[0] - a * a) - b_a * b_a;
  if (r1square >= 0.0) {
    b_sqrt(&r1square);
    R_est[2] = lsRCM[14] - r1square;
  }

  /*  fprintf('%f\t%f\t%f\n',R_est(1),R_est(2),R_est(3)); */
  if (method == 1) {
    for (i0 = 0; i0 < 3; i0++) {
      R_OP[i0] = R_est[i0];
    }
  } else {
    /*     %% NLS(max_nls_NOR=4, lsMR,lsRCM) */
    /*  unchange var ---> calc once */
    for (b_i = 1; b_i <= use_nls_NOR; b_i++) {
      Ri[b_i - 1] = lsMR[b_i - 1];
      xi[b_i - 1] = lsRCM[b_i - 1];
      yi[b_i - 1] = lsRCM[b_i + 6];
      zi[b_i - 1] = lsRCM[b_i + 13];
    }

    for (i_count_nls = 0; i_count_nls < 3; i_count_nls++) {
      if (1 + i_count_nls == 1) {
        /*  start at 1 */
        for (i = 0; i < 3; i++) {
          Rnew[i] = R_est[i];

          /*  use LLS result as init value */
        }
      }

      x = Rnew[0];
      y = Rnew[1];
      z = Rnew[2];
      Fxyz = 0.0;
      for (b_i = 1; b_i <= use_nls_NOR; b_i++) {
        /* ---------------------------fi,Fxyz------------------------- */
        a = x - xi[b_i - 1];
        b_a = y - yi[b_i - 1];
        c_a = z - zi[b_i - 1];
        r1square = (a * a + b_a * b_a) + c_a * c_a;
        b_sqrt(&r1square);
        fi[b_i - 1] = r1square - Ri[b_i - 1];
        Fxyz += fi[b_i - 1] * fi[b_i - 1];

        /* ----------------------------JtJ---------------------------- */
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[0] += (x - xi[b_i - 1]) * (x - xi[b_i - 1]) / (a * a);
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[3] += (x - xi[b_i - 1]) * (y - yi[b_i - 1]) / (a * a);
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[6] += (x - xi[b_i - 1]) * (z - zi[b_i - 1]) / (a * a);
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[4] += (y - yi[b_i - 1]) * (y - yi[b_i - 1]) / (a * a);
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[7] += (y - yi[b_i - 1]) * (z - zi[b_i - 1]) / (a * a);
        a = fi[b_i - 1] + Ri[b_i - 1];
        JtJ[8] += (z - zi[b_i - 1]) * (z - zi[b_i - 1]) / (a * a);

        /* ----------------------------Jtf---------------------------- */
        Jtf[0] += (x - xi[b_i - 1]) * fi[b_i - 1] / (fi[b_i - 1] + Ri[b_i - 1]);
        Jtf[1] += (y - yi[b_i - 1]) * fi[b_i - 1] / (fi[b_i - 1] + Ri[b_i - 1]);
        Jtf[2] += (z - zi[b_i - 1]) * fi[b_i - 1] / (fi[b_i - 1] + Ri[b_i - 1]);
      }

      /* ----------------------------JtJ---------------------------- */
      JtJ[1] = JtJ[3];
      JtJ[2] = JtJ[6];
      JtJ[5] = JtJ[7];
      if (Fxyz < Fmin) {
        /* success */
        lamda_k /= 10.0;
      } else {
        /*  unsuccess */
        lamda_k *= 10.0;
      }

      for (i = 0; i < 3; i++) {
        X[i] = JtJ[i << 2];
      }

      memset(&b_A[0], 0, 9U * sizeof(double));
      for (i = 0; i < 3; i++) {
        b_A[i + 3 * i] = X[i];
      }

      for (i0 = 0; i0 < 9; i0++) {
        JtJ[i0] += lamda_k * b_A[i0];
      }

      mldivide(JtJ, Jtf, X);
      for (i0 = 0; i0 < 3; i0++) {
        Rnew[i0] -= X[i0];
      }

      if (Fxyz < Fmin) {
        /* success */
        Fmin = Fxyz;
        for (i0 = 0; i0 < 3; i0++) {
          R_OP[i0] = Rnew[i0];
        }
      }

      /*          fprintf('%f\t%f\t%f\t%f\t%f\n',Fxyz,lamda_k,Rnew(1),Rnew(2),Rnew(3)); */
    }
  }
}

/*
 * File trailer for LeastSquare2.c
 *
 * [EOF]
 */
