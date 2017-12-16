/*
 * File: multirate_kalman_v4_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-Dec-2017 17:56:48
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v4.h"
#include "multirate_kalman_v4_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void multirate_kalman_v4_initialize(void)
{
  rt_InitInfAndNaN(8U);
  x_est_not_empty_init();
  multirate_kalman_v4_init();
}

/*
 * File trailer for multirate_kalman_v4_initialize.c
 *
 * [EOF]
 */
