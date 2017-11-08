/*
 * File: multirate_kalman_v2.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 08-Nov-2017 15:09:06
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v2.h"
#include "xzgetrf.h"

/* Variable Definitions */
static double x_est[6];
static boolean_T x_est_not_empty;
static double p_est[36];

/* Function Definitions */

/*
 * MULTIRATE_KALMAN Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : const double ips_pos[3]
 *                double ips_flag
 *                const double opt_flow[2]
 *                const double opt_gyro[2]
 *                double yaw_angle
 *                double k_pos[3]
 * Return Type  : void
 */
void multirate_kalman_v2(double ips_pos[3], double ips_flag, 
  double opt_flow[2], double opt_gyro[2], double yaw_angle, double k_pos[3])
{
  int i;
  int jp;
  double v_optical_idx_0;
  double x_pred[6];
  static const double Q1[36] = { 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 25.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8 };

  int kBcol;
  double v_optical_idx_1;
  double a[36];
  int jBcol;
  double temp;
  static const double b_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 1.0 };

  double p_pred[36];
  static const double Q2[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6 };

  static const double b[36] = { 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int ipiv[6];
  double y[36];
  double S[36];
  int j;
  int k;
  signed char p[6];
  static const signed char c_a[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const double R1[36] = { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6 };

  double K[36];
  double b_x_pred[6];
  double b_p_pred[36];
  double d_a[6];
  double c_x_pred[6];

  /*  function [ k_pos,Vx,Vy] = multirate_kalman_v2(ips_pos,ips_flag,opt_flow,opt_gyro,yaw_angle) */
  /*  KF1: x=[Sx,Sy,Sz,Vx,Vy,Vz] */
  /*  x = Ax + Bu + w (B=0) */
  /*  z = Hx + v */
  /*  100Hz -> 10ms */
  /*  T2 = 1/20;              % 20Hz  -> 50ms */
  /*  Predict: Time update */
  /*  x(k+1|k) */
  /*  P(k+1|k) */
  /*  uk = zeros(3,range);            % vx,vy,vz */
  /*  Update: Measurement update */
  /*  Global var */
  /*  Init value */
  if (!x_est_not_empty) {
    for (i = 0; i < 6; i++) {
      x_est[i] = 0.0;
    }

    x_est_not_empty = true;

    /*  x_est=[Sx,Sy,Sz,Vx,Vy,Vz]' */
    for (i = 0; i < 3; i++) {
      x_est[i] = ips_pos[i];
    }

    memcpy(&p_est[0], &Q1[0], 36U * sizeof(double));
  }

  /*  KALMAN */
  if (ips_flag == 1.0) {
    /*  at 20Hz */
    for (jp = 0; jp < 6; jp++) {
      x_pred[jp] = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += b_a[jp + 6 * jBcol] * p_est[jBcol + 6 * kBcol];
        }

        x_pred[jp] += b_a[jp + 6 * kBcol] * x_est[kBcol];
      }

      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * b[jBcol + 6 * kBcol];
        }

        p_pred[jp + 6 * kBcol] = temp + Q2[jp + 6 * kBcol];
      }
    }

    v_optical_idx_0 = (opt_flow[1] - opt_gyro[1]) * x_pred[2];

    /* [m/s] */
    v_optical_idx_1 = (opt_flow[0] - opt_gyro[0]) * x_pred[2];

    /* [m/s] */
    /*      Vx = (x_pred(1)-x_est(1))/T1; */
    /*      Vy = (x_pred(2)-x_est(2))/T1; */
    for (jp = 0; jp < 6; jp++) {
      for (kBcol = 0; kBcol < 6; kBcol++) {
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += (double)c_a[jp + 6 * jBcol] * p_pred[jBcol + 6 *
            kBcol];
        }
      }

      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * (double)c_a[jBcol + 6 * kBcol];
        }

        S[jp + 6 * kBcol] = temp + R1[jp + 6 * kBcol];
      }
    }

    for (jp = 0; jp < 36; jp++) {
      y[jp] = 0.0;
      a[jp] = S[jp];
    }

    xzgetrf(a, ipiv, &jBcol);
    for (jp = 0; jp < 6; jp++) {
      p[jp] = (signed char)(1 + jp);
    }

    for (k = 0; k < 5; k++) {
      if (ipiv[k] > 1 + k) {
        jBcol = p[ipiv[k] - 1];
        p[ipiv[k] - 1] = p[k];
        p[k] = (signed char)jBcol;
      }
    }

    for (k = 0; k < 6; k++) {
      jBcol = p[k] - 1;
      y[k + 6 * (p[k] - 1)] = 1.0;
      for (j = k; j + 1 < 7; j++) {
        if (y[j + 6 * jBcol] != 0.0) {
          for (i = j + 1; i + 1 < 7; i++) {
            y[i + 6 * jBcol] -= y[j + 6 * jBcol] * a[i + 6 * j];
          }
        }
      }
    }

    for (j = 0; j < 6; j++) {
      jBcol = 6 * j;
      for (k = 5; k >= 0; k += -1) {
        jp = 6 * k;
        if (y[k + jBcol] != 0.0) {
          y[k + jBcol] /= a[k + jp];
          for (i = 0; i + 1 <= k; i++) {
            y[i + jBcol] -= y[k + jBcol] * a[i + jp];
          }
        }
      }

      for (jp = 0; jp < 6; jp++) {
        b_p_pred[j + 6 * jp] = 0.0;
        for (kBcol = 0; kBcol < 6; kBcol++) {
          b_p_pred[j + 6 * jp] += p_pred[j + 6 * kBcol] * (double)c_a[kBcol + 6 *
            jp];
        }
      }
    }

    for (jp = 0; jp < 6; jp++) {
      for (kBcol = 0; kBcol < 6; kBcol++) {
        K[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          K[jp + 6 * kBcol] += b_p_pred[jp + 6 * jBcol] * y[jBcol + 6 * kBcol];
        }
      }
    }

    /*  Kalman gain */
    for (jp = 0; jp < 3; jp++) {
      b_x_pred[jp] = ips_pos[jp];
    }

    b_x_pred[3] = (v_optical_idx_0 * cos(yaw_angle) - v_optical_idx_1 * sin
                   (yaw_angle)) * 1.7;
    b_x_pred[4] = (v_optical_idx_0 * sin(yaw_angle) + v_optical_idx_1 * cos
                   (yaw_angle)) * 1.3;
    b_x_pred[5] = (x_pred[2] - x_est[2]) / 0.01;
    for (jp = 0; jp < 6; jp++) {
      d_a[jp] = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        d_a[jp] += (double)c_a[jp + 6 * kBcol] * x_pred[kBcol];
      }

      c_x_pred[jp] = b_x_pred[jp] - d_a[jp];
    }

    /*  new state */
    for (jp = 0; jp < 6; jp++) {
      temp = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp += K[jp + 6 * kBcol] * c_x_pred[kBcol];
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += K[jp + 6 * jBcol] * S[jBcol + 6 * kBcol];
        }
      }

      x_est[jp] = x_pred[jp] + temp;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * K[kBcol + 6 * jBcol];
        }

        p_est[jp + 6 * kBcol] = p_pred[jp + 6 * kBcol] - temp;
      }
    }

    /*  new covariance */
    for (i = 0; i < 3; i++) {
      k_pos[i] = x_est[i];
    }
  } else {
    /*  Measurement update 1 @100Hz */
    /*    Time update / Predict @ time k */
    /*  x = Ax + Bu + w */
    /*  z = Hx + v */
    for (jp = 0; jp < 6; jp++) {
      x_pred[jp] = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += b_a[jp + 6 * jBcol] * p_est[jBcol + 6 * kBcol];
        }

        x_pred[jp] += b_a[jp + 6 * kBcol] * x_est[kBcol];
      }

      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * b[jBcol + 6 * kBcol];
        }

        p_pred[jp + 6 * kBcol] = temp + Q1[jp + 6 * kBcol];
      }
    }

    v_optical_idx_0 = (opt_flow[1] - opt_gyro[1]) * x_pred[2];

    /* [m/s] */
    v_optical_idx_1 = (opt_flow[0] - opt_gyro[0]) * x_pred[2];

    /* [m/s] */
    /*      Vx = (x_pred(1)-x_est(1))/T1; */
    /*      Vy = (x_pred(2)-x_est(2))/T1; */
    for (jp = 0; jp < 6; jp++) {
      for (kBcol = 0; kBcol < 6; kBcol++) {
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += (double)c_a[jp + 6 * jBcol] * p_pred[jBcol + 6 *
            kBcol];
        }
      }

      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * (double)c_a[jBcol + 6 * kBcol];
        }

        S[jp + 6 * kBcol] = temp + R1[jp + 6 * kBcol];
        K[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          K[jp + 6 * kBcol] += p_pred[jp + 6 * jBcol] * (double)c_a[jBcol + 6 *
            kBcol];
        }
      }
    }

    memcpy(&a[0], &S[0], 36U * sizeof(double));
    xzgetrf(a, ipiv, &jBcol);
    for (j = 0; j < 6; j++) {
      jBcol = 6 * j;
      jp = 6 * j;
      for (k = 1; k <= j; k++) {
        kBcol = 6 * (k - 1);
        if (a[(k + jp) - 1] != 0.0) {
          for (i = 0; i < 6; i++) {
            K[i + jBcol] -= a[(k + jp) - 1] * K[i + kBcol];
          }
        }
      }

      temp = 1.0 / a[j + jp];
      for (i = 0; i < 6; i++) {
        K[i + jBcol] *= temp;
      }
    }

    for (j = 5; j >= 0; j += -1) {
      jBcol = 6 * j;
      jp = 6 * j - 1;
      for (k = j + 2; k < 7; k++) {
        kBcol = 6 * (k - 1);
        if (a[k + jp] != 0.0) {
          for (i = 0; i < 6; i++) {
            K[i + jBcol] -= a[k + jp] * K[i + kBcol];
          }
        }
      }
    }

    for (jBcol = 4; jBcol >= 0; jBcol += -1) {
      if (ipiv[jBcol] != jBcol + 1) {
        jp = ipiv[jBcol] - 1;
        for (kBcol = 0; kBcol < 6; kBcol++) {
          temp = K[kBcol + 6 * jBcol];
          K[kBcol + 6 * jBcol] = K[kBcol + 6 * jp];
          K[kBcol + 6 * jp] = temp;
        }
      }
    }

    /*  Kalman gain */
    for (jp = 0; jp < 3; jp++) {
      b_x_pred[jp] = x_pred[jp];
    }

    b_x_pred[3] = (v_optical_idx_0 * cos(yaw_angle) - v_optical_idx_1 * sin
                   (yaw_angle)) * 1.7;
    b_x_pred[4] = (v_optical_idx_0 * sin(yaw_angle) + v_optical_idx_1 * cos
                   (yaw_angle)) * 1.3;
    b_x_pred[5] = (x_pred[2] - x_est[2]) / 0.01;
    for (jp = 0; jp < 6; jp++) {
      d_a[jp] = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        d_a[jp] += (double)c_a[jp + 6 * kBcol] * x_pred[kBcol];
      }

      c_x_pred[jp] = b_x_pred[jp] - d_a[jp];
    }

    /*  new state */
    for (jp = 0; jp < 6; jp++) {
      temp = 0.0;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp += K[jp + 6 * kBcol] * c_x_pred[kBcol];
        a[jp + 6 * kBcol] = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          a[jp + 6 * kBcol] += K[jp + 6 * jBcol] * S[jBcol + 6 * kBcol];
        }
      }

      x_est[jp] = x_pred[jp] + temp;
      for (kBcol = 0; kBcol < 6; kBcol++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += a[jp + 6 * jBcol] * K[kBcol + 6 * jBcol];
        }

        p_est[jp + 6 * kBcol] = p_pred[jp + 6 * kBcol] - temp;
      }
    }

    /*  new covariance */
    for (i = 0; i < 3; i++) {
      k_pos[i] = x_est[i];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void x_est_not_empty_init(void)
{
  x_est_not_empty = false;
}

/*
 * File trailer for multirate_kalman_v2.c
 *
 * [EOF]
 */
