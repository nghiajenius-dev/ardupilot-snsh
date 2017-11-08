/*
 * File: multirate_kalman_v2.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 08-Nov-2017 15:09:06
 */

#ifndef MULTIRATE_KALMAN_V2_H
#define MULTIRATE_KALMAN_V2_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "multirate_kalman_v2_types.h"

/* Function Declarations */
extern void multirate_kalman_v2(double ips_pos[3], double ips_flag, double opt_flow[2], 
	double opt_gyro[2], double yaw_angle, double k_pos[3]);
extern void x_est_not_empty_init(void);

#endif

/*
 * File trailer for multirate_kalman_v2.h
 *
 * [EOF]
 */
