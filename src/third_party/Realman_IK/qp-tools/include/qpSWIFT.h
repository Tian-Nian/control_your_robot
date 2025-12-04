#ifndef __QP_SWIFT_H__
#define __QP_SWIFT_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "Auxilary.h"

	
/* Main Solver Functions */

/* QP Setup Function sparse version */
QP *QP_SETUP(qp_int n, qp_int m, qp_int p, qp_int *Pjc, qp_int *Pir, qp_real *Ppr, qp_int *Ajc, qp_int *Air, qp_real *Apr, qp_int *Gjc, qp_int *Gir, qp_real *Gpr, qp_real *c, qp_real *h, qp_real *b, qp_real sigma_d, qp_int *Permut);

/* QP Setup Function dense version */
QP *QP_SETUP_dense(qp_int n, qp_int m, qp_int p, qp_real *Ppr, qp_real *Apr, qp_real *Gpr, qp_real *c, qp_real *h, qp_real *b, qp_int *Permut, int ordering);

/* QP Solve Function */
qp_int QP_SOLVE(QP *myQP);

/* QP Clean Function sparse version */
void QP_CLEANUP(QP *myQP);

/* QP Clean Function dense version */
void QP_CLEANUP_dense(QP *myQP);


#ifdef __cplusplus
}
#endif

#endif

/*! @file */