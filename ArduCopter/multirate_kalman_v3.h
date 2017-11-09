/*
 * File: multirate_kalman_v3.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 09-Nov-2017 13:40:52
 */

#ifndef MULTIRATE_KALMAN_V3_H
#define MULTIRATE_KALMAN_V3_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "multirate_kalman_v3_types.h"

/* Function Declarations */
extern void multirate_kalman_v3(double ips_pos[3], double ips_flag, double
  k_pos[3]);
extern void multirate_kalman_v3_init(void);
extern void x_est_not_empty_init(void);

#endif

/*
 * File trailer for multirate_kalman_v3.h
 *
 * [EOF]
 */
