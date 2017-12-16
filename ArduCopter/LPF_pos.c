/*
 * File: LPF_pos.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-Dec-2017 17:57:04
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LPF_pos.h"

/* Variable Definitions */
static boolean_T x_pred_not_empty;
static double v_pred[3];
static double x_post[3];
static double v_post[3];

/* Function Definitions */

/*
 * Init parameter
 * Arguments    : const double ips_pos[3]
 *                short ips_flag
 *                double delay_ms
 *                const double max_inno[3]
 *                double last_timeout
 *                double k_pos[3]
 *                double k_vel[3]
 * Return Type  : void
 */
void LPF_pos( double ips_pos[3], short ips_flag, double delay_ms, 
             double max_inno[3], double last_timeout, double k_pos[3], double
             k_vel[3])
{
  int i;
  double x_pred;
  double inno;

  /*  function [k_pos, inno] = LPF_pos(ips_pos,ips_flag,delay_ms,max_inno,last_timeout) */
  /* [second] */
  /*  LPF_vel_k = 0.09516;    %10hz */
  /*  max_inno = 0.1; %m */
  /*  Init value */
  if (!x_pred_not_empty) {
    /*  x_est=[Sx,Sy,Sz,Vx,Vy,Vz]' */
    for (i = 0; i < 3; i++) {
      x_post[i] = ips_pos[i];
    }

    /*      v_post_ = zeros(3, 1); */
  }

  /*  LFP */
  if (ips_flag == 1) {
    x_pred_not_empty = true;

    /*      % Apply LPF for raw vel */
    /*      v_post = v_post_ +  LPF_vel_k * (v_post - v_post_);   */
    /*      v_post_ = v_post; */
    for (i = 0; i < 3; i++) {
      x_pred = x_post[i] + 0.01 * v_pred[i];
      inno = v_pred[i] * delay_ms / 1000.0;
      v_pred[i] = v_post[i];
      inno = (ips_pos[i] + inno) - x_pred;
      if (fabs(inno) < max_inno[i]) {
        x_post[i] = x_pred + 0.5 * inno;
        v_post[i] = v_pred[i] + 4.0 * inno;
      } else {
        x_post[i] = x_pred;
        v_post[i] = 0.0;
      }

      k_pos[i] = x_post[i];
      k_vel[i] = v_post[i];
    }
  } else {
    if (last_timeout > 0.2) {
      for (i = 0; i < 3; i++) {
        v_post[i] = 0.0;
      }
    }

    x_pred_not_empty = true;

    /*      % Apply LPF for raw vel */
    /*      v_post = v_post_ +  LPF_vel_k * (v_post - v_post_);   */
    /*      v_post_ = v_post; */
    for (i = 0; i < 3; i++) {
      x_pred = 0.01 * v_pred[i];
      v_pred[i] = v_post[i];
      x_post[i] += x_pred;
      k_pos[i] = x_post[i];
      k_vel[i] = v_post[i];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void LPF_pos_init(void)
{
  int i;
  for (i = 0; i < 3; i++) {
    v_post[i] = 0.0;
    v_pred[i] = 0.0;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void x_pred_not_empty_init(void)
{
  x_pred_not_empty = false;
}

/*
 * File trailer for LPF_pos.c
 *
 * [EOF]
 */
