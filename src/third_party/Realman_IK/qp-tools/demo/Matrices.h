#ifndef __MATRICES_H__
#define __MATRICES_H__
#include "GlobalOptions.h"

    /* Using the default value of 0.0 */
    qp_real sigma_d = 0.0;

    /* Cost Function P Matrix in CCS format */
    qp_real Ppr[7] = {5.0, 1.0, 1.0, 2.0, 1.0, 1.0, 4.0};
    qp_int Pir[7] = {0, 1, 0, 1, 2, 1, 2};
    qp_int Pjc[4] = {0, 2, 5, 7};

    /* Cost Function c vector */
    qp_real c[3] = {1.0, 2.0, 1.0};

    /* Equality Constraint A Matrix in CCS format */
    qp_real Apr[3] = {1.0, -2.0, 1.0};
    qp_int Air[3] = {0, 0, 0};
    qp_int Ajc[4] = {0, 1, 2, 3};

    /* Equality Constraints b vector */
    qp_real b[1] = {3.0};

    /* Inequality Constraint G Matrix in CCS format */
    qp_real Gpr[3] = {-4.0, -4.0, -1.0};
    qp_int Gir[3] = {0, 0, 1};
    qp_int Gjc[4] = {0, 1, 2, 3};

    /* Inequality Constraint h vector */
    qp_real h[2] = {-1.0, -1.0};

    /* Data for QP*/
    qp_int n = 3; /* Number of decision Variables */
    qp_int m = 2; /* Number of inequality constraints */
    qp_int p = 1; /* Number of equality constraints */

    /* Permutation Vector optional */
    qp_int Permut[6] = {5, 2, 3, 1, 4, 0};

    qp_int Permutineq[5] = {4, 2, 0, 3, 1};

#endif

/*! @file */