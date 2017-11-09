/*
 * File: multirate_kalman_v3_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 09-Nov-2017 13:40:52
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman_v3.h"
#include "multirate_kalman_v3_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void multirate_kalman_v3_initialize(void)
{
  rt_InitInfAndNaN(8U);
  x_est_not_empty_init();
  multirate_kalman_v3_init();
}

/*
 * File trailer for multirate_kalman_v3_initialize.c
 *
 * [EOF]
 */
