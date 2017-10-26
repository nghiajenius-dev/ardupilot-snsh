/*
 * File: LeastSquare_NJ.c
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
 * NOR =5
 *  RCM: Receiver Coordinate Matrix [3,5]
 *  MR: Measured Range [1,5]
 *  method: 1-lls, 2-nls
 * Arguments    : const double RCM[15]
 *                const double MR[5]
 *                double method
 *                double R_OP[3]
 * Return Type  : void
 */
void LeastSquare_NJ( double RCM[15],  uint16_t MR[5], double method,
                    double R_OP[3])
{
  int i0;
  double r1square;
  int i;
  double Rnew[3];
  double A[9];
  double Rold[3];
  double X[3];
  double a;
  double b_A[12];
  double b_a;
  double c_a;
  int i1;
  double b[4];
  double b_i;
  double f2;
  double f3;
  double f4;
  double f5;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double JtJ[9];
  double Jtf[3];
  int16_t nls_loop;

  for (i0 = 0; i0 < 3; i0++) {
    R_OP[i0] = 0.0;
    Rnew[i0] = 0.0;
  }

  r1square = MR[0] * MR[0];
  for (i = 0; i < 4; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A[i + (i0 << 2)] = RCM[(i + 5 * i0) + 1] - RCM[5 * i0];
    }

    a = RCM[i + 1] - RCM[0];
    b_a = RCM[i + 6] - RCM[5];
    c_a = RCM[i + 11] - RCM[10];
    b[i] = 0.5 * ((r1square - MR[i + 1] * MR[i + 1]) + ((a * a + b_a * b_a) +
      c_a * c_a));
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      A[i0 + 3 * i] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        A[i0 + 3 * i] += b_A[i1 + (i0 << 2)] * b_A[i1 + (i << 2)];
      }
    }

    Rold[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      Rold[i0] += b_A[i + (i0 << 2)] * b[i];
    }
  }

  mldivide(A, Rold, X);

  /*  linear least square solution */
  for (i0 = 0; i0 < 2; i0++) {
    R_OP[i0] = X[i0] + RCM[5 * i0];
  }

  /*   tempz = (z_Transmitter - z_NearestReceiver)^2 */
  a = R_OP[0] - RCM[0];
  b_a = R_OP[1] - RCM[5];
  R_OP[2] = RCM[10] - sqrt((MR[0] * MR[0] - a * a) - b_a * b_a);
  b_i = 0.0;
  r1square = 0.001;
  nls_loop = 0;

  while ((nls_loop <5)&&(r1square > 1.0E-4)) {
    nls_loop = nls_loop +1;
    if (b_i == 0.0) {
      for (i0 = 0; i0 < 3; i0++) {
        Rold[i0] = R_OP[i0];
      }
    } else {
      for (i0 = 0; i0 < 3; i0++) {
        Rold[i0] = Rnew[i0];
      }
    }

    a = Rold[0] - RCM[0];
    b_a = Rold[1] - RCM[5];
    c_a = Rold[2] - RCM[10];
    r1square = sqrt((a * a + b_a * b_a) + c_a * c_a) - MR[0];
    a = Rold[0] - RCM[1];
    b_a = Rold[1] - RCM[6];
    c_a = Rold[2] - RCM[11];
    f2 = sqrt((a * a + b_a * b_a) + c_a * c_a) - MR[1];
    a = Rold[0] - RCM[2];
    b_a = Rold[1] - RCM[7];
    c_a = Rold[2] - RCM[12];
    f3 = sqrt((a * a + b_a * b_a) + c_a * c_a) - MR[2];
    a = Rold[0] - RCM[3];
    b_a = Rold[1] - RCM[8];
    c_a = Rold[2] - RCM[13];
    f4 = sqrt((a * a + b_a * b_a) + c_a * c_a) - MR[3];
    a = Rold[0] - RCM[4];
    b_a = Rold[1] - RCM[9];
    c_a = Rold[2] - RCM[14];
    f5 = sqrt((a * a + b_a * b_a) + c_a * c_a) - MR[4];
    a = Rold[0] - RCM[0];
    b_a = r1square + MR[0];
    c_a = Rold[0] - RCM[1];
    d_a = f2 + MR[1];
    e_a = Rold[0] - RCM[2];
    f_a = f3 + MR[2];
    g_a = Rold[0] - RCM[3];
    h_a = f4 + MR[3];
    i_a = Rold[0] - RCM[4];
    j_a = f5 + MR[4];
    JtJ[0] = (((a * a / (b_a * b_a) + c_a * c_a / (d_a * d_a)) + e_a * e_a /
               (f_a * f_a)) + g_a * g_a / (h_a * h_a)) + i_a * i_a / (j_a * j_a);
    a = r1square + MR[0];
    b_a = f2 + MR[1];
    c_a = f3 + MR[2];
    d_a = f4 + MR[3];
    e_a = f5 + MR[4];
    JtJ[3] = ((((Rold[0] - RCM[0]) * (Rold[1] - RCM[5]) / (a * a) + (Rold[0] -
      RCM[1]) * (Rold[1] - RCM[6]) / (b_a * b_a)) + (Rold[0] - RCM[2]) * (Rold[1]
                - RCM[7]) / (c_a * c_a)) + (Rold[0] - RCM[3]) * (Rold[1] - RCM[8])
              / (d_a * d_a)) + (Rold[0] - RCM[4]) * (Rold[1] - RCM[9]) / (e_a *
      e_a);
    a = r1square + MR[0];
    b_a = f2 + MR[1];
    c_a = f3 + MR[2];
    d_a = f4 + MR[3];
    e_a = f5 + MR[4];
    JtJ[6] = ((((Rold[0] - RCM[0]) * (Rold[2] - RCM[10]) / (a * a) + (Rold[0] -
      RCM[1]) * (Rold[2] - RCM[11]) / (b_a * b_a)) + (Rold[0] - RCM[2]) * (Rold
                [2] - RCM[12]) / (c_a * c_a)) + (Rold[0] - RCM[3]) * (Rold[2] -
               RCM[13]) / (d_a * d_a)) + (Rold[0] - RCM[4]) * (Rold[2] - RCM[14])
      / (e_a * e_a);
    JtJ[1] = JtJ[3];
    a = Rold[1] - RCM[5];
    b_a = r1square + MR[0];
    c_a = Rold[1] - RCM[6];
    d_a = f2 + MR[1];
    e_a = Rold[1] - RCM[7];
    f_a = f3 + MR[2];
    g_a = Rold[1] - RCM[8];
    h_a = f4 + MR[3];
    i_a = Rold[1] - RCM[9];
    j_a = f5 + MR[4];
    JtJ[4] = (((a * a / (b_a * b_a) + c_a * c_a / (d_a * d_a)) + e_a * e_a /
               (f_a * f_a)) + g_a * g_a / (h_a * h_a)) + i_a * i_a / (j_a * j_a);
    a = r1square + MR[0];
    b_a = f2 + MR[1];
    c_a = f3 + MR[2];
    d_a = f4 + MR[3];
    e_a = f5 + MR[4];
    JtJ[7] = ((((Rold[1] - RCM[5]) * (Rold[2] - RCM[10]) / (a * a) + (Rold[1] -
      RCM[6]) * (Rold[2] - RCM[11]) / (b_a * b_a)) + (Rold[1] - RCM[7]) * (Rold
                [2] - RCM[12]) / (c_a * c_a)) + (Rold[1] - RCM[8]) * (Rold[2] -
               RCM[13]) / (d_a * d_a)) + (Rold[1] - RCM[9]) * (Rold[2] - RCM[14])
      / (e_a * e_a);
    JtJ[2] = JtJ[6];
    JtJ[5] = JtJ[7];
    a = Rold[2] - RCM[10];
    b_a = r1square + MR[0];
    c_a = Rold[2] - RCM[11];
    d_a = f2 + MR[1];
    e_a = Rold[2] - RCM[12];
    f_a = f3 + MR[2];
    g_a = Rold[2] - RCM[13];
    h_a = f4 + MR[3];
    i_a = Rold[2] - RCM[14];
    j_a = f5 + MR[4];
    JtJ[8] = (((a * a / (b_a * b_a) + c_a * c_a / (d_a * d_a)) + e_a * e_a /
               (f_a * f_a)) + g_a * g_a / (h_a * h_a)) + i_a * i_a / (j_a * j_a);
    Jtf[0] = ((((Rold[0] - RCM[0]) * r1square / (r1square + MR[0]) + (Rold[0] -
      RCM[1]) * f2 / (f2 + MR[1])) + (Rold[0] - RCM[2]) * f3 / (f3 + MR[2])) +
              (Rold[0] - RCM[3]) * f4 / (f4 + MR[3])) + (Rold[0] - RCM[4]) * f5 /
      (f5 + MR[4]);
    Jtf[1] = ((((Rold[1] - RCM[5]) * r1square / (r1square + MR[0]) + (Rold[1] -
      RCM[6]) * f2 / (f2 + MR[1])) + (Rold[1] - RCM[7]) * f3 / (f3 + MR[2])) +
              (Rold[1] - RCM[8]) * f4 / (f4 + MR[3])) + (Rold[1] - RCM[9]) * f5 /
      (f5 + MR[4]);
    Jtf[2] = ((((Rold[2] - RCM[10]) * r1square / (r1square + MR[0]) + (Rold[2] -
      RCM[11]) * f2 / (f2 + MR[1])) + (Rold[2] - RCM[12]) * f3 / (f3 + MR[2])) +
              (Rold[2] - RCM[13]) * f4 / (f4 + MR[3])) + (Rold[2] - RCM[14]) *
      f5 / (f5 + MR[4]);
    mldivide(JtJ, Jtf, X);
    r1square = 0.0;
    f2 = 2.2250738585072014E-308;
    for (i = 0; i < 3; i++) {
      f3 = Rold[i] - X[i];
      f4 = fabs(f3 - Rold[i]);
      if (f4 > f2) {
        f5 = f2 / f4;
        r1square = 1.0 + r1square * f5 * f5;
        f2 = f4;
      } else {
        f5 = f4 / f2;
        r1square += f5 * f5;
      }

      Rnew[i] = f3;
    }

    r1square = f2 * sqrt(r1square);
    b_i++;
  }

  if ((!(method == 1.0)) && (method == 2.0)) {
    for (i0 = 0; i0 < 3; i0++) {
      R_OP[i0] = Rnew[i0];
    }

    /*      i_count =   i_count_nls; */
  } else {
    /*      i_count =   i_count_ls; */
  }
}

/*
 * File trailer for LeastSquare_NJ.c
 *
 * [EOF]
 */
