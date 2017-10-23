/*
 * File: LeastSquare.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 21-Oct-2017 13:51:56
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LeastSquare.h"
#include "norm.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * function  [R_OP,i_count]  =   LeastSquare(NOR,RCM,MR)
 *  inputs    :   NOR (number of receivers)
 *            :   RCM (reciever coordinate matrix, NOR by 3 matrix)
 *            :   MR (measured range, NOR by 1 matrix)
 *            :   method -> 1 for lls, 2 for nls
 *  outputs   :   R_OP (calculated x,y,z position based on 'method' input)
 *            :   i_count (no. of iterations required to arrive at position)
 * Arguments    : double NOR
 *                const double RCM[15]
 *                const double MR[5]
 *                double method
 *                double R_OP[3]
 * Return Type  : void
 */
void LeastSquare(double NOR, double RCM[15], int16_t MR[5], double
                 method, double R_OP[3])
{
  int i0;
  double Rnew[3];
  double A[12];
  int i;
  double b[4];
  double JtJ[9];
  double b_A[3];
  double X[3];
  double tol;
  int i_count_nls;
  double R_OP_LLS[3];
  double b_i;
  double Rold[3];
  double a;
  double b_a;
  double dv0[3];
  double f1;
  double f2;
  double f3;
  double f4;
  double c_a;
  double f5;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  for (i0 = 0; i0 < 3; i0++) {
    Rnew[i0] = 0.0;
    R_OP[i0] = 0.0;
  }

  /* -- linear least square method for initial guess */
  memset(&A[0], 0, 12U * sizeof(double));
  for (i = 0; i < 4; i++) {
    b[i] = 0.0;
  }

  for (i = 0; i < (int)(NOR - 1.0); i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A[i + (i0 << 2)] = RCM[((int)((1.0 + (double)i) + 1.0) + 5 * i0) - 1] -
        RCM[5 * i0];
      b_A[i0] = RCM[((int)((1.0 + (double)i) + 1.0) + 5 * i0) - 1] - RCM[5 * i0];
    }

    tol = norm(b_A);
    b[i] = 0.5 * ((MR[0] * MR[0] - MR[(int)((1.0 + (double)i) + 1.0) - 1] * MR
                   [(int)((1.0 + (double)i) + 1.0) - 1]) + tol * tol);
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      JtJ[i0 + 3 * i] = 0.0;
      for (i_count_nls = 0; i_count_nls < 4; i_count_nls++) {
        JtJ[i0 + 3 * i] += A[i_count_nls + (i0 << 2)] * A[i_count_nls + (i << 2)];
      }
    }

    b_A[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      b_A[i0] += A[i + (i0 << 2)] * b[i];
    }
  }

  mldivide(JtJ, b_A, X);

  /*  linear least square solution */
  for (i0 = 0; i0 < 3; i0++) {
    R_OP_LLS[i0] = X[i0] + RCM[5 * i0];
  }

  /*  estimated position coordinate */
  /*  always 1 for linear least squares */
  /* -- nonlinear least square method */
  i_count_nls = 0;

  /*  iterations before convergence */
  b_i = 0.0;
  tol = 0.001;
  while ((tol > 1.0E-6) && (i_count_nls <= 15)) {
    i_count_nls++;
    if (b_i == 0.0) {
      for (i0 = 0; i0 < 3; i0++) {
        Rold[i0] = R_OP_LLS[i0];
      }
    } else {
      for (i0 = 0; i0 < 3; i0++) {
        Rold[i0] = Rnew[i0];
      }
    }

    memset(&JtJ[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      X[i] = 0.0;
    }

    if (NOR == 4.0) {
      tol = Rold[0] - RCM[0];
      a = Rold[1] - RCM[5];
      b_a = Rold[2] - RCM[10];
      f1 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[0];
      tol = Rold[0] - RCM[1];
      a = Rold[1] - RCM[6];
      b_a = Rold[2] - RCM[11];
      f2 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[1];
      tol = Rold[0] - RCM[2];
      a = Rold[1] - RCM[7];
      b_a = Rold[2] - RCM[12];
      f3 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[2];
      tol = Rold[0] - RCM[3];
      a = Rold[1] - RCM[8];
      b_a = Rold[2] - RCM[13];
      f4 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[3];
      tol = Rold[0] - RCM[0];
      a = f1 + MR[0];
      b_a = Rold[0] - RCM[1];
      c_a = f2 + MR[1];
      d_a = Rold[0] - RCM[2];
      e_a = f3 + MR[2];
      f_a = Rold[0] - RCM[3];
      g_a = f4 + MR[3];
      JtJ[0] = ((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                (e_a * e_a)) + f_a * f_a / (g_a * g_a);
      tol = f1 + MR[0];
      a = f2 + MR[1];
      b_a = f3 + MR[2];
      c_a = f4 + MR[3];
      JtJ[3] = (((Rold[0] - RCM[0]) * (Rold[1] - RCM[5]) / (tol * tol) + (Rold[0]
                  - RCM[1]) * (Rold[1] - RCM[6]) / (a * a)) + (Rold[0] - RCM[2])
                * (Rold[1] - RCM[7]) / (b_a * b_a)) + (Rold[0] - RCM[3]) *
        (Rold[1] - RCM[8]) / (c_a * c_a);
      tol = f1 + MR[0];
      a = f2 + MR[1];
      b_a = f3 + MR[2];
      c_a = f4 + MR[3];
      JtJ[6] = (((Rold[0] - RCM[0]) * (Rold[2] - RCM[10]) / (tol * tol) + (Rold
                  [0] - RCM[1]) * (Rold[2] - RCM[11]) / (a * a)) + (Rold[0] -
                 RCM[2]) * (Rold[2] - RCM[12]) / (b_a * b_a)) + (Rold[0] - RCM[3])
        * (Rold[2] - RCM[13]) / (c_a * c_a);
      JtJ[1] = JtJ[3];
      tol = Rold[1] - RCM[5];
      a = f1 + MR[0];
      b_a = Rold[1] - RCM[6];
      c_a = f2 + MR[1];
      d_a = Rold[1] - RCM[7];
      e_a = f3 + MR[2];
      f_a = Rold[1] - RCM[8];
      g_a = f4 + MR[3];
      JtJ[4] = ((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                (e_a * e_a)) + f_a * f_a / (g_a * g_a);
      tol = f1 + MR[0];
      a = f2 + MR[1];
      b_a = f3 + MR[2];
      c_a = f4 + MR[3];
      JtJ[7] = (((Rold[1] - RCM[5]) * (Rold[2] - RCM[10]) / (tol * tol) + (Rold
                  [1] - RCM[6]) * (Rold[2] - RCM[11]) / (a * a)) + (Rold[1] -
                 RCM[7]) * (Rold[2] - RCM[12]) / (b_a * b_a)) + (Rold[1] - RCM[8])
        * (Rold[2] - RCM[13]) / (c_a * c_a);
      JtJ[2] = JtJ[6];
      JtJ[5] = JtJ[7];
      tol = Rold[2] - RCM[10];
      a = f1 + MR[0];
      b_a = Rold[2] - RCM[11];
      c_a = f2 + MR[1];
      d_a = Rold[2] - RCM[12];
      e_a = f3 + MR[2];
      f_a = Rold[2] - RCM[13];
      g_a = f4 + MR[3];
      JtJ[8] = ((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                (e_a * e_a)) + f_a * f_a / (g_a * g_a);
      X[0] = (((Rold[0] - RCM[0]) * f1 / (f1 + MR[0]) + (Rold[0] - RCM[1]) * f2 /
               (f2 + MR[1])) + (Rold[0] - RCM[2]) * f3 / (f3 + MR[2])) + (Rold[0]
        - RCM[3]) * f4 / (f4 + MR[3]);
      X[1] = (((Rold[1] - RCM[5]) * f1 / (f1 + MR[0]) + (Rold[1] - RCM[6]) * f2 /
               (f2 + MR[1])) + (Rold[1] - RCM[7]) * f3 / (f3 + MR[2])) + (Rold[1]
        - RCM[8]) * f4 / (f4 + MR[3]);
      X[2] = (((Rold[2] - RCM[10]) * f1 / (f1 + MR[0]) + (Rold[2] - RCM[11]) *
               f2 / (f2 + MR[1])) + (Rold[2] - RCM[12]) * f3 / (f3 + MR[2])) +
        (Rold[2] - RCM[13]) * f4 / (f4 + MR[3]);
    } else {
      if (NOR == 5.0) {
        tol = Rold[0] - RCM[0];
        a = Rold[1] - RCM[5];
        b_a = Rold[2] - RCM[10];
        f1 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[0];
        tol = Rold[0] - RCM[1];
        a = Rold[1] - RCM[6];
        b_a = Rold[2] - RCM[11];
        f2 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[1];
        tol = Rold[0] - RCM[2];
        a = Rold[1] - RCM[7];
        b_a = Rold[2] - RCM[12];
        f3 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[2];
        tol = Rold[0] - RCM[3];
        a = Rold[1] - RCM[8];
        b_a = Rold[2] - RCM[13];
        f4 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[3];
        tol = Rold[0] - RCM[4];
        a = Rold[1] - RCM[9];
        b_a = Rold[2] - RCM[14];
        f5 = sqrt((tol * tol + a * a) + b_a * b_a) - MR[4];
        tol = Rold[0] - RCM[0];
        a = f1 + MR[0];
        b_a = Rold[0] - RCM[1];
        c_a = f2 + MR[1];
        d_a = Rold[0] - RCM[2];
        e_a = f3 + MR[2];
        f_a = Rold[0] - RCM[3];
        g_a = f4 + MR[3];
        h_a = Rold[0] - RCM[4];
        i_a = f5 + MR[4];
        JtJ[0] = (((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                   (e_a * e_a)) + f_a * f_a / (g_a * g_a)) + h_a * h_a / (i_a *
          i_a);
        tol = f1 + MR[0];
        a = f2 + MR[1];
        b_a = f3 + MR[2];
        c_a = f4 + MR[3];
        d_a = f5 + MR[4];
        JtJ[3] = ((((Rold[0] - RCM[0]) * (Rold[1] - RCM[5]) / (tol * tol) +
                    (Rold[0] - RCM[1]) * (Rold[1] - RCM[6]) / (a * a)) + (Rold[0]
                    - RCM[2]) * (Rold[1] - RCM[7]) / (b_a * b_a)) + (Rold[0] -
                   RCM[3]) * (Rold[1] - RCM[8]) / (c_a * c_a)) + (Rold[0] - RCM
          [4]) * (Rold[1] - RCM[9]) / (d_a * d_a);
        tol = f1 + MR[0];
        a = f2 + MR[1];
        b_a = f3 + MR[2];
        c_a = f4 + MR[3];
        d_a = f5 + MR[4];
        JtJ[6] = ((((Rold[0] - RCM[0]) * (Rold[2] - RCM[10]) / (tol * tol) +
                    (Rold[0] - RCM[1]) * (Rold[2] - RCM[11]) / (a * a)) + (Rold
                    [0] - RCM[2]) * (Rold[2] - RCM[12]) / (b_a * b_a)) + (Rold[0]
                   - RCM[3]) * (Rold[2] - RCM[13]) / (c_a * c_a)) + (Rold[0] -
          RCM[4]) * (Rold[2] - RCM[14]) / (d_a * d_a);
        JtJ[1] = JtJ[3];
        tol = Rold[1] - RCM[5];
        a = f1 + MR[0];
        b_a = Rold[1] - RCM[6];
        c_a = f2 + MR[1];
        d_a = Rold[1] - RCM[7];
        e_a = f3 + MR[2];
        f_a = Rold[1] - RCM[8];
        g_a = f4 + MR[3];
        h_a = Rold[1] - RCM[9];
        i_a = f5 + MR[4];
        JtJ[4] = (((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                   (e_a * e_a)) + f_a * f_a / (g_a * g_a)) + h_a * h_a / (i_a *
          i_a);
        tol = f1 + MR[0];
        a = f2 + MR[1];
        b_a = f3 + MR[2];
        c_a = f4 + MR[3];
        d_a = f5 + MR[4];
        JtJ[7] = ((((Rold[1] - RCM[5]) * (Rold[2] - RCM[10]) / (tol * tol) +
                    (Rold[1] - RCM[6]) * (Rold[2] - RCM[11]) / (a * a)) + (Rold
                    [1] - RCM[7]) * (Rold[2] - RCM[12]) / (b_a * b_a)) + (Rold[1]
                   - RCM[8]) * (Rold[2] - RCM[13]) / (c_a * c_a)) + (Rold[1] -
          RCM[9]) * (Rold[2] - RCM[14]) / (d_a * d_a);
        JtJ[2] = JtJ[6];
        JtJ[5] = JtJ[7];
        tol = Rold[2] - RCM[10];
        a = f1 + MR[0];
        b_a = Rold[2] - RCM[11];
        c_a = f2 + MR[1];
        d_a = Rold[2] - RCM[12];
        e_a = f3 + MR[2];
        f_a = Rold[2] - RCM[13];
        g_a = f4 + MR[3];
        h_a = Rold[2] - RCM[14];
        i_a = f5 + MR[4];
        JtJ[8] = (((tol * tol / (a * a) + b_a * b_a / (c_a * c_a)) + d_a * d_a /
                   (e_a * e_a)) + f_a * f_a / (g_a * g_a)) + h_a * h_a / (i_a *
          i_a);
        X[0] = ((((Rold[0] - RCM[0]) * f1 / (f1 + MR[0]) + (Rold[0] - RCM[1]) *
                  f2 / (f2 + MR[1])) + (Rold[0] - RCM[2]) * f3 / (f3 + MR[2])) +
                (Rold[0] - RCM[3]) * f4 / (f4 + MR[3])) + (Rold[0] - RCM[4]) *
          f5 / (f5 + MR[4]);
        X[1] = ((((Rold[1] - RCM[5]) * f1 / (f1 + MR[0]) + (Rold[1] - RCM[6]) *
                  f2 / (f2 + MR[1])) + (Rold[1] - RCM[7]) * f3 / (f3 + MR[2])) +
                (Rold[1] - RCM[8]) * f4 / (f4 + MR[3])) + (Rold[1] - RCM[9]) *
          f5 / (f5 + MR[4]);
        X[2] = ((((Rold[2] - RCM[10]) * f1 / (f1 + MR[0]) + (Rold[2] - RCM[11]) *
                  f2 / (f2 + MR[1])) + (Rold[2] - RCM[12]) * f3 / (f3 + MR[2]))
                + (Rold[2] - RCM[13]) * f4 / (f4 + MR[3])) + (Rold[2] - RCM[14])
          * f5 / (f5 + MR[4]);
      }
    }

    mldivide(JtJ, X, dv0);
    for (i0 = 0; i0 < 3; i0++) {
      tol = Rold[i0] - dv0[i0];
      b_A[i0] = tol - Rold[i0];
      Rnew[i0] = tol;
    }

    tol = fabs(norm(b_A));
    b_i++;
  }

  if (method == 1.0) {
    for (i0 = 0; i0 < 3; i0++) {
      R_OP[i0] = R_OP_LLS[i0];
    }

    /*      i_count =   i_count_ls; */
  } else {
    if (method == 2.0) {
      for (i0 = 0; i0 < 3; i0++) {
        R_OP[i0] = Rnew[i0];
      }

      /*      i_count =   i_count_nls; */
    }
  }
}

/*
 * File trailer for LeastSquare.c
 *
 * [EOF]
 */
