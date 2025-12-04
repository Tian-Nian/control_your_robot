#ifndef __QP_AUXILARY_H__
#define __QP_AUXILARY_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "timer.h"
#include "amd.h"
#include "amd_internal.h"
#include "ldl.h"
/*!< Written in Auxilary.c */

/*! \struct */
/*! Sparse Matrix Storage Format */
/*! Use Sparse Matrix Setup to Initialise a Sparse Matrix */
typedef struct smat
{
	qp_int *jc;	 /*!< Vector to store column count ; Dim [n+1] */
	qp_int *ir;	 /*!< Vector to store row indices in column major format ; Dim[nnz] */
	qp_real *pr; /*!< Vector to store matrix values in column major format ; Dim[nnz] */
	qp_int n;	 /*!< Number of Rows of the Sparse Matrix */
	qp_int m;	 /*!< Number of Columns of the Sparse Matrix */
	qp_int nnz;	 /*!< Number of nonzeros entries ; nnz = jc [n] */
} smat;

/*! \struct */
/*! KKT Structure */
/*! Used to store everything related to KKT matrix and its factorization */
typedef struct kkt
{
	smat *kktmatrix;  /*!< Sparse kkt matrix */
	qp_real *b;		  /*!< b vector */
	qp_int *Parent;	  /*!< LDL - workspace Vectors */
	qp_int *Flag;	  /*!< LDL - workspace Vectors */
	qp_int *Lnz;	  /*!< LDL - workspace Vectors */
	qp_int *Li;		  /*!< ir vector of LDL Sparse Matrix in column compressed format */
	qp_int *Lp;		  /*!< jc vector of LDL Sparse Matrix in column compressed format */
	qp_int *Lti;	  /*!< ir vector of the transpose of LDL Sparse Matrix in column compressed format */
	qp_int *Ltp;	  /*!< jc vector of the transpose of LDL Sparse Matrix in column compressed format */
	qp_int *Pattern;  /*!< LDL - workspace Vectors */
	qp_int *UPattern; /*!< Nodes to be updated during every iteration */
	qp_real *Y;		  /*!< LDL - workspace Vectors */
	qp_real *Lx;	  /*!< pr vector of LDL Sparse Matrix in column compressed format */
	qp_real *D;		  /*!< LDL - workspace Vectors */
	qp_int *P;		  /*!< Permutation Vector ; Input */
	qp_int *Pinv;	  /*!< Permutation Vector Inverse */

} kkt;

/*! \struct */
/*! Statistics Structure*/
/*! Used to store everything related to qpSWIFT statistics */
typedef struct stats
{

	/*!< Time Statistics */
	qp_real tsetup;		    /*!< Setup Time ; Includes Initialisation Problem as well */
	qp_real tsolve;		    /*!< Solve Time */
	qp_real kkt_time;	    /*!< kkt Solve Time */
	qp_real ldl_numeric;    /*!< ldl_numeric time */
	/*!< Time Statsitics */

	/*!< Algorithmic Statistics */
	qp_int IterationCount; /*!< Iteration Count */
	qp_real n_rx;		   /*!< Norm of residual vector rx */
	qp_real n_ry;		   /*!< Norm of residual vector ry */
	qp_real n_rz;		   /*!< Norm of residual vector rz */
	qp_real n_mu;		   /*!< Complementary Slackness (s'z/m) */

	qp_real alpha_p;        /*!< Primal Step Size */
	qp_real alpha_d;        /*!< Dual Step Size  */
	/*!< Algorithmic Statistics */

    qp_real fval;           /*!< Function Value */
    qp_int Flag;            /*!< Solver FLAG */
    qp_int AMD_RESULT;      /*!< AMD Compilation Result ; Non-negative means Successfull ; Negative means unsuccesfull ; -3 is unused */


	/*!< Internal Statistics */
	qp_int resolve_kkt;     /*!< Used tp indicate if kkt matrix has to be refactored; 0 for no; any other value yes */

} stats;

/*! \struct */
/*! Settings Structure*/
/*! Used to store everything related to qpSWIFT settings */
typedef struct settings
{

	qp_int maxit;	/*!< Maximum Number of Iterations */
	qp_real reltol; /*!< Residual Tolerances */
	qp_real abstol; /*!< s and z Tolerances */
	qp_real sigma;	/*!< Sigma Desired */
	qp_int verbose; /*!< Verbose Levels || 0 :: No Print
					 *                  || 1 :: Print Residuals and Step Sizes
                     * 					|| 2 :: Print Everything */

} settings;

/*! \struct */
/*! QP Structure*/
/*! The main qpSWIFT structure; contains references to all other structures */

	typedef struct QP
	{

	qp_int n; /*!< Dimension of P matrix */
	qp_int m; /*!< First Dimension of G matrix */
	qp_int p; /*!< First Dimension of A matrix */

	qp_real sigma_d; /*!< Parameter */
	qp_real mu;		 /*!< Barrier Function Parameter */
	qp_real rho;	 /*!< Some Parameter */

	qp_real *x; /*!<	Primal Variables ;  Dimensions [n,1] */
	qp_real *y; /*!<    Dual   Variables ;  Dimensions [p,1] */
	qp_real *z; /*!<	Dual Variables	 ;	Dimensions [m,1] */
	qp_real *s; /*!<	Primal Variables ;	Dimensions [m,1] */

	qp_real *rx; /*!<	Residuals	;	Dimensions [n,1] */
	qp_real *ry; /*!<    Residuals	;	Dimensions [p,1] */
	qp_real *rz; /*!<	Residuals	;	Dimensions [m,1] */

	qp_real *delta;	  /*!< [delta_x;delta_y;delta_z]	;	Dimensions [n + p + m,1] */
	qp_real *delta_x; /*!< delta_x	;	Dimensions [n,1] */
	qp_real *delta_y; /*!< delta_y	;	Dimensions [p,1] */
	qp_real *delta_z; /*!< delta_z	;	Dimensions [m,1] */
	qp_real *delta_s; /*!< delta_s	;	Dimensions [m,1] */

	qp_real *ds;	 /*!< ds	;	Dimensions [m,1] */
	qp_real *lambda; /*!< lambda	;	Dimensions[m,1] */

	qp_real *temp; /*!< Temporary Variables to Calculate Objective Function Value */

	smat *P;	/*!< Cost Function	:	Quadratic Part	:	Dimensions	[n,n] */
	qp_real *c; /*!< Cost Function	:	linear term	:	Dimensions	[n,1] */
	smat *G;	/*!< Inequality Matrix	:	Gx<=h	:	Dimension	[m,n] */
	qp_real *h; /*!< Inequality Matrix	:	Gx<=h	:	Dimension	[m,1] */
	smat *A;	/*!< Equality Matrix		:	Ax=b	:	Dimension	[p,n] */
	qp_real *b; /*!< Equality Matrix		:	Ax=b	:	Dimension	[p,1] */

	smat *At; /*!< Transpose of Equality Matrix */
	smat *Gt; /*!< Transpose of InEquality Matrix */

	kkt *kkt;		   /*!< kkt Matrix */
	settings *options; /*!< Solver Settings */
	stats *stats;	   /*!< Solver Stats */

} QP;


qp_int kkt_initialize(QP *myQP);

qp_int kktsolve_1(QP *myQP);

void kktsolve_2(QP *myQP);

void SparseMatrixSetup(qp_int m, qp_int n, qp_int nnz, qp_int *jc, qp_int *ir, qp_real *pr, smat *sparse);

void Transpose_Row_Count(qp_int m, qp_int n, qp_int *Li, qp_int *Lp, qp_int *Lti, qp_int *Ltp);

void computeresiduals(QP *myQP);

void SparseMatrixMultiply(smat *A, qp_real *x, qp_real *y, qp_int start);

void SparseMatrixTransMultiply(smat *A, qp_real *x, qp_real *y, qp_int start);

void form_ds(qp_real *ds, qp_real *lambda, qp_real *delta_s, qp_real *delta_z, qp_real sigma, qp_real mu, qp_int m, qp_int selector);

void formkktmatrix_U(smat *P, smat *G, smat *Gt, smat *kkt);

void formkktmatrix_full(smat *P, smat *G, smat *A, smat *Gt, smat *At, smat *kktmatrix);

void SparseMatrixTranspose(smat *A, smat *At);

void updatekktmatrix(smat *kkt, qp_real *s, qp_real *z, qp_real *delta_s, qp_real *delta_z, qp_real alpha_p, qp_real alpha_d, qp_int m, qp_int n, qp_int p, qp_int indicator);

void updatekktmatrix_b(qp_real *b, qp_real *rx, qp_real *ry, qp_real *rz, qp_real *ds, qp_real *z, qp_int n, qp_int m, qp_int p);

qp_int checksign(qp_real *s, qp_real *delta_s, qp_real alpha, qp_int count);

void updatevariables(qp_real *x, qp_real *delta_x, qp_real alpha, qp_int count);

void formlambda(qp_real *lambda, qp_real *s, qp_real *z, qp_int n);

qp_real formrho(qp_real *s, qp_real *delta_s, qp_real *z, qp_real *delta_z, qp_real alpha_p, qp_real alpha_d, qp_int n);

qp_int ldlinitialsolve(kkt *mykkt, qp_real *delta);

void test_reach(qp_int *Parent, qp_int *Pinv, qp_int *UPattern, qp_int n, qp_int m, qp_int p);

void findsteplength(qp_real *s, qp_real *delta_s, qp_real *z, qp_real *delta_z, qp_int m, qp_real *alpha_p, qp_real *alpha_d);

qp_real obj_value(smat *P, qp_real *c, qp_real *x, qp_real *temp);

qp_real innerproduct(qp_real *x, qp_real *y, qp_int n);

void densetosparse(qp_int m, qp_int n, qp_real *pr, smat *A);

void densetosparse_ROWMAJOR(qp_int m, qp_int n, qp_real *pr, smat *A);


#ifdef __cplusplus
}
#endif

#endif

/*! @file */
