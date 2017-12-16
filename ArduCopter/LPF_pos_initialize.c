/*
 * File: LPF_pos_initialize.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-Dec-2017 17:57:04
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "LPF_pos.h"
#include "LPF_pos_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void LPF_pos_initialize(void)
{
  rt_InitInfAndNaN(8U);
  x_pred_not_empty_init();
  LPF_pos_init();
}

/*
 * File trailer for LPF_pos_initialize.c
 *
 * [EOF]
 */
