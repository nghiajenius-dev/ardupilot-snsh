/*
 * File: multirate_kalman_v2_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 08-Nov-2017 15:09:06
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v2.h"
#include "multirate_kalman_v2_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void multirate_kalman_v2_initialize(void)
{
  rt_InitInfAndNaN(8U);
  x_est_not_empty_init();
}

/*
 * File trailer for multirate_kalman_v2_initialize.c
 *
 * [EOF]
 */
