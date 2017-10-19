/*
 * File: multirate_kalman_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 18-Oct-2017 16:34:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "multirate_kalman.h"
#include "multirate_kalman_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void multirate_kalman_initialize(void)
{
  rt_InitInfAndNaN(8U);
  x_est_not_empty_init();
  multirate_kalman_init();
}

/*
 * File trailer for multirate_kalman_initialize.c
 *
 * [EOF]
 */
