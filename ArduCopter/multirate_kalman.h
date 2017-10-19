/*
 * File: multirate_kalman.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 18-Oct-2017 16:34:24
 */

#ifndef MULTIRATE_KALMAN_H
#define MULTIRATE_KALMAN_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "multirate_kalman_types.h"

/* Function Declarations */
extern void multirate_kalman( double ips_pos[3], double ips_flag, 
  double opt_flow[2],  double opt_gyro[3], double lidar_h, double k_pos[3]);
extern void multirate_kalman_init(void);
extern void x_est_not_empty_init(void);

#endif

/*
 * File trailer for multirate_kalman.h
 *
 * [EOF]
 */
