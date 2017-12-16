/*
 * File: multirate_kalman_v4.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-Dec-2017 17:56:48
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v4.h"
#include "mrdivide.h"

/* Variable Definitions */
static double x_est[6];
static boolean_T x_est_not_empty;
static double p_est[36];
static double Vx_;
static double Vy_;
static double Vz_;

/* Function Definitions */

/*
 * Parameters
 * Arguments    : const double ips_pos[3]
 *                short ips_flag
 *                const double opt_flow[2]
 *                const double opt_gyro[2]
 *                double yaw_angle
 *                double k_pos[3]
 *                double k_vel[3]
 * Return Type  : void
 */
void multirate_kalman_v4( double ips_pos[3], short ips_flag,  double
  opt_flow[2],  double opt_gyro[2], double yaw_angle, double k_pos[3],
  double k_vel[3])
{
  int i;
  double v_optical_idx_0;
  double x_pred[6];
  int i0;
  double v_optical_idx_1;
  static const double Q1[36] = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    9.9999999999999991E-6 };

  double a[36];
  int i1;
  static const double b_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 1.0 };

  double p_pred[36];
  static const double Q2[36] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    9.9999999999999991E-6 };

  static const double b[36] = { 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  double K[36];
  double S[36];
  double b_x_pred[6];
  static const signed char c_a[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const double R1[36] = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    9.9999999999999991E-6 };

  double d_a[6];
  double c_x_pred[6];

  /*  100Hz -> 10ms */
  /*  20Hz  -> 50ms */
  /*  x */
  /*  v */
  /*  [second] */
  /*  offset z */
  /* 10hz */
  /*  KALMAN */
  /*  Init value */
  if (!x_est_not_empty) {
    for (i = 0; i < 6; i++) {
      x_est[i] = 0.0;
    }

    x_est_not_empty = true;

    /*  x_est=[Sx,Sy,Sz,Vx,Vy,Vz]' */
    /*      x_est_ = zeros(6, 1);        % x_est=[Sx,Sy,Sz,Vx,Vy,Vz]' */
    for (i = 0; i < 3; i++) {
      x_est[i] = ips_pos[i];
    }

    memcpy(&p_est[0], &Q1[0], 36U * sizeof(double));
  }

  /*  KALMAN */
  if (ips_flag == 1) {
    for (i = 0; i < 6; i++) {
      x_pred[i] = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += b_a[i + 6 * i1] * p_est[i1 + 6 * i0];
        }

        x_pred[i] += b_a[i + 6 * i0] * x_est[i0];
      }

      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * b[i1 + 6 * i0];
        }

        p_pred[i + 6 * i0] = v_optical_idx_0 + Q2[i + 6 * i0];
      }
    }

    v_optical_idx_0 = (opt_flow[1] * 1.5 - opt_gyro[1]) * x_pred[2];

    /* [m/s] */
    v_optical_idx_1 = (opt_flow[0] * 1.5 - opt_gyro[0]) * x_pred[2];

    /* [m/s] */
    /*  Apply LPF for raw vel */
    Vx_ += 0.09516 * ((v_optical_idx_0 * cos(yaw_angle) - v_optical_idx_1 * sin
                       (yaw_angle)) - Vx_);
    Vy_ += 0.09516 * ((v_optical_idx_0 * sin(yaw_angle) + v_optical_idx_1 * cos
                       (yaw_angle)) - Vy_);
    Vz_ += 0.09516 * ((x_pred[2] - x_est[2]) / 0.01 - Vz_);
    for (i = 0; i < 6; i++) {
      for (i0 = 0; i0 < 6; i0++) {
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += (double)c_a[i + 6 * i1] * p_pred[i1 + 6 * i0];
        }
      }

      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * (double)c_a[i1 + 6 * i0];
        }

        S[i + 6 * i0] = v_optical_idx_0 + R1[i + 6 * i0];
        K[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          K[i + 6 * i0] += p_pred[i + 6 * i1] * (double)c_a[i1 + 6 * i0];
        }
      }
    }

    mrdivide(K, S);

    /*  Kalman gain */
    for (i = 0; i < 3; i++) {
      b_x_pred[i] = ips_pos[i];
    }

    b_x_pred[3] = Vx_;
    b_x_pred[4] = Vy_;
    b_x_pred[5] = Vz_;
    for (i = 0; i < 6; i++) {
      d_a[i] = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        d_a[i] += (double)c_a[i + 6 * i0] * x_pred[i0];
      }

      c_x_pred[i] = b_x_pred[i] - d_a[i];
    }

    /*  new state */
    for (i = 0; i < 6; i++) {
      v_optical_idx_0 = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 += K[i + 6 * i0] * c_x_pred[i0];
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += K[i + 6 * i1] * S[i1 + 6 * i0];
        }
      }

      x_est[i] = x_pred[i] + v_optical_idx_0;
      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * K[i0 + 6 * i1];
        }

        p_est[i + 6 * i0] = p_pred[i + 6 * i0] - v_optical_idx_0;
      }
    }

    /*  new covariance */
  } else {
    /*  Measurement update 1 @100Hz */
    /*    Time update / Predict @ time k */
    /*  x = Ax + Bu + w */
    /*  z = Hx + v */
    for (i = 0; i < 6; i++) {
      x_pred[i] = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += b_a[i + 6 * i1] * p_est[i1 + 6 * i0];
        }

        x_pred[i] += b_a[i + 6 * i0] * x_est[i0];
      }

      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * b[i1 + 6 * i0];
        }

        p_pred[i + 6 * i0] = v_optical_idx_0 + Q1[i + 6 * i0];
      }
    }

    v_optical_idx_0 = (opt_flow[1] - opt_gyro[1]) * x_pred[2];

    /* [m/s] */
    v_optical_idx_1 = (opt_flow[0] - opt_gyro[0]) * x_pred[2];

    /* [m/s] */
    /*  Apply LPF for raw vel */
    Vx_ += 0.09516 * ((v_optical_idx_0 * cos(yaw_angle) - v_optical_idx_1 * sin
                       (yaw_angle)) - Vx_);
    Vy_ += 0.09516 * ((v_optical_idx_0 * sin(yaw_angle) + v_optical_idx_1 * cos
                       (yaw_angle)) - Vy_);
    Vz_ += 0.09516 * ((x_pred[2] - x_est[2]) / 0.01 - Vz_);
    for (i = 0; i < 6; i++) {
      for (i0 = 0; i0 < 6; i0++) {
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += (double)c_a[i + 6 * i1] * p_pred[i1 + 6 * i0];
        }
      }

      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * (double)c_a[i1 + 6 * i0];
        }

        S[i + 6 * i0] = v_optical_idx_0 + R1[i + 6 * i0];
        K[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          K[i + 6 * i0] += p_pred[i + 6 * i1] * (double)c_a[i1 + 6 * i0];
        }
      }
    }

    mrdivide(K, S);

    /*  Kalman gain */
    for (i = 0; i < 3; i++) {
      b_x_pred[i] = x_pred[i];
    }

    b_x_pred[3] = Vx_;
    b_x_pred[4] = Vy_;
    b_x_pred[5] = Vz_;
    for (i = 0; i < 6; i++) {
      d_a[i] = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        d_a[i] += (double)c_a[i + 6 * i0] * x_pred[i0];
      }

      c_x_pred[i] = b_x_pred[i] - d_a[i];
    }

    /*  new state */
    for (i = 0; i < 6; i++) {
      v_optical_idx_0 = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 += K[i + 6 * i0] * c_x_pred[i0];
        a[i + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a[i + 6 * i0] += K[i + 6 * i1] * S[i1 + 6 * i0];
        }
      }

      x_est[i] = x_pred[i] + v_optical_idx_0;
      for (i0 = 0; i0 < 6; i0++) {
        v_optical_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          v_optical_idx_0 += a[i + 6 * i1] * K[i0 + 6 * i1];
        }

        p_est[i + 6 * i0] = p_pred[i + 6 * i0] - v_optical_idx_0;
      }
    }

    /*  new covariance    */
  }

  for (i = 0; i < 3; i++) {
    k_pos[i] = x_est[i];
    k_vel[i] = x_est[i + 3];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void multirate_kalman_v4_init(void)
{
  Vx_ = 0.0;
  Vy_ = 0.0;
  Vz_ = 0.0;
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
 * File trailer for multirate_kalman_v4.c
 *
 * [EOF]
 */
