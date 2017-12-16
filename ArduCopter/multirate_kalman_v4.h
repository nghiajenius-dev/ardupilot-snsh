/*
 * File: multirate_kalman_v4.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-Dec-2017 17:56:48
 */

#ifndef MULTIRATE_KALMAN_V4_H
#define MULTIRATE_KALMAN_V4_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "multirate_kalman_v4_types.h"

/* Function Declarations */
extern void multirate_kalman_v4( double ips_pos[3], short ips_flag, 
  double opt_flow[2],  double opt_gyro[2], double yaw_angle, double k_pos[3],
  double k_vel[3]);
extern void multirate_kalman_v4_init(void);
extern void x_est_not_empty_init(void);

#endif

/*
 * File trailer for multirate_kalman_v4.h
 *
 * [EOF]
 */
