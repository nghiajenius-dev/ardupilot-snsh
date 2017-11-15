/*
 * File: LPF_pos.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 14-Nov-2017 13:54:11
 */

#ifndef LPF_POS_H
#define LPF_POS_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "LPF_pos_types.h"

/* Function Declarations */
extern void LPF_pos( double ips_pos[3], short ips_flag, double delay_ms,
                     double max_inno[3], double last_timeout, double k_pos[3]);
extern void LPF_pos_init(void);
extern void x_pred_not_empty_init(void);

#endif

/*
 * File trailer for LPF_pos.h
 *
 * [EOF]
 */
