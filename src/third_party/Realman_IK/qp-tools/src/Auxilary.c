/*********************************************************************************************************
* Auxilary Program Module
*
* Contains function implementations decalred in qpSWIFT.h
*
*
*********************************************************************************************************/
#include "Auxilary.h"

/*! Forms the Upper triangular part of the kkt matrix
*
* Status: Inactive
*/
void formkktmatrix_U(smat *P, smat *G, smat *Gt, smat *kkt)
{

	qp_int i, j, k, kkt_nnz;

	kkt_nnz = 0;
	kkt->jc[0] = 0;

	for (i = 0; i < P->n; i++)
	{
		for (j = P->jc[i]; j < P->jc[i + 1]; j++)
		{
			k = P->ir[j];
			if (k <= i)
			{
				kkt->ir[kkt_nnz] = k;
				kkt->pr[kkt_nnz] = P->pr[j];
				kkt_nnz++;
			}
		}
		kkt->jc[i + 1] = kkt_nnz;
	}

	for (i = 0; i < Gt->n; i++)
	{
		for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++)
		{
			kkt->ir[kkt_nnz] = Gt->ir[j];
			kkt->pr[kkt_nnz] = Gt->pr[j];
			kkt_nnz++;
			if (j == Gt->jc[i + 1] - 1)
			{
				kkt->ir[kkt_nnz] = P->n + i;
				kkt->pr[kkt_nnz] = -1.0;
				kkt_nnz++;
			}
		}
		kkt->jc[i + 1 + P->n] = kkt_nnz;
	}
}

/*!
 * @brief Assembles the KKT matrix
 *
 * @param[out] kkt   KKT Matrix
 * @param[in]  P     Cost Function Matrix
 * @param[in]  A     Equality Constraint Matrix
 * @param[in]  A'    Transpose of A
 * @param[in]  G     Inequality Constraint Matrix
 * @param[in]  G'    Transpose of G
 *
 *
 * 	KKT = [P	A'	 G']
 * 		  [A	0	 0]
 * 		  [G	0	-I]
 */

void formkktmatrix_full(smat *P, smat *G, smat *A, smat *Gt, smat *At, smat *kkt)
{

	if (A && At)
	{

		qp_int kkt_nnz = 0;
		qp_int i, j;
		kkt->jc[0] = 0;

		/* Concatenating Matrices P, A, G vertically */
		for (i = 0; i < P->n; i++)
		{
			for (j = P->jc[i]; j < P->jc[i + 1]; j++)
			{
				kkt->pr[kkt_nnz] = P->pr[j];
				kkt->ir[kkt_nnz] = P->ir[j];
				kkt_nnz++;
			}

			for (j = A->jc[i]; j < A->jc[i + 1]; j++)
			{
				kkt->pr[kkt_nnz] = A->pr[j];
				kkt->ir[kkt_nnz] = P->m + A->ir[j];
				kkt_nnz++;
			}

			for (j = G->jc[i]; j < G->jc[i + 1]; j++)
			{
				kkt->pr[kkt_nnz] = G->pr[j];
				kkt->ir[kkt_nnz] = P->m + A->m + G->ir[j];
				kkt_nnz++;
			}
			kkt->jc[i + 1] = P->jc[i + 1] + G->jc[i + 1] + A->jc[i + 1];
		}

		/* Concatenating Matrices A', G' horizontally and G' and -I vertically */
		for (i = 0; i < At->n; i++)
		{
			for (j = At->jc[i]; j < At->jc[i + 1]; j++)
			{
				kkt->ir[kkt_nnz] = At->ir[j];
				kkt->pr[kkt_nnz] = At->pr[j];
				kkt_nnz++;
			}
			kkt->jc[i + 1 + P->n] = kkt_nnz;
		}

		for (i = 0; i < Gt->n; i++)
		{
			for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++)
			{
				kkt->ir[kkt_nnz] = Gt->ir[j];
				kkt->pr[kkt_nnz] = Gt->pr[j];
				kkt_nnz++;
				if (j == Gt->jc[i + 1] - 1)
				{
					kkt->ir[kkt_nnz] = P->m + A->m + i;
					kkt->pr[kkt_nnz] = -1.0;
					kkt_nnz++;
				}
			}
			kkt->jc[i + 1 + P->n + At->n] = kkt_nnz;
		}
	}
	else
	{

		qp_int kkt_nnz = 0;
		qp_int i, j;

		kkt->jc[0] = 0;

		/* Concatenating Matrices P and G vertically */
		for (i = 0; i < P->n; i++)
		{
			for (j = P->jc[i]; j < P->jc[i + 1]; j++)
			{
				kkt->pr[kkt_nnz] = P->pr[j];
				kkt->ir[kkt_nnz] = P->ir[j];
				kkt_nnz++;
			}

			for (j = G->jc[i]; j < G->jc[i + 1]; j++)
			{
				kkt->pr[kkt_nnz] = G->pr[j];
				kkt->ir[kkt_nnz] = P->m + G->ir[j];
				kkt_nnz++;
			}
			kkt->jc[i + 1] = P->jc[i + 1] + G->jc[i + 1];
		}

		/* Concatenating Matrices G' and -I horizontally */
		for (i = 0; i < Gt->n; i++)
		{
			for (j = Gt->jc[i]; j < Gt->jc[i + 1]; j++)
			{
				kkt->ir[kkt_nnz] = Gt->ir[j];
				kkt->pr[kkt_nnz] = Gt->pr[j];
				kkt_nnz++;
				if (j == Gt->jc[i + 1] - 1)
				{
					kkt->ir[kkt_nnz] = P->m + i;
					kkt->pr[kkt_nnz] = -1.0;
					kkt_nnz++;
				}
			}
			kkt->jc[i + 1 + P->n] = kkt_nnz;
		}
	}
}

/*!
 * @brief Updates the lower diagonal part of the kkt Matrix,
 *
 *
 *
 * @param[out] kkt          KKT Matrix
 * @param[in]  s     		Primal Slack Variables
 * @param[in]  z     		Dual Variables
 * @param[in]  delta_s    	Primal slack direction
 * @param[in]  delta_z     	Dual slack direction
 * @param[in]  alpha_p    	Primal step Length
 * @param[in]  alpha_d      Dual step length
 * @param[in]  m			# of ineqaulity constraints
 * @param[in]  n			# of decision variables
 * @param[in]  p			# of equality constraints
 * @param[in]  indicator    selection vector
 *
 *
 *		indicator = 1 : Pure Affine Direction
 *		indicator = 2 : Pure Centering Direction
 *		indicator = 3 : Pure Newton Direction
 */
void updatekktmatrix(smat *kkt, qp_real *s, qp_real *z, qp_real *delta_s, qp_real *delta_z, qp_real alpha_p, qp_real alpha_d, qp_int m, qp_int n, qp_int p, qp_int indicator)
{

	qp_int index, i;
	if (indicator == 0)
	{
		for (i = n + p; i < n + m + p; i++)
		{
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -s[i - n - p] / z[i - n - p];
		}
	}
	if (indicator == 1)
	{
		for (i = n + p; i < n + m + p; i++)
		{
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -(s[i - n - p] / z[i - n - p] - 1);
		}
	}
	if (indicator == 2)
	{
		for (i = n + p; i < n + m + p; i++)
		{
			index = kkt->jc[i + 1] - 1;
			kkt->pr[index] = -(s[i - n - p] / z[i - n - p] - (s[i - n - p] - alpha_p * delta_s[i - n - p]) / (z[i - n - p] - alpha_d * delta_z[i - n - p]));
		}
	}
}

/*!
 * @brief Performs scalar vector addition
 *
 * @param[out] x            vector
 * @param[in]  x     		vector
 * @param[in]  alpha     	scalar value
 * @param[in]  delta_x    	search direction
 *
 *
 *		x = x + alpha*delta_x
 */
void updatevariables(qp_real *x, qp_real *delta_x, qp_real alpha, qp_int count)
{
	qp_int i;
	for (i = 0; i < count; i++)
	{
		x[i] += delta_x[i] * alpha;
	}
		
}

/*!
 * @brief Updates the right hand side of the KKT linear system of equations
 *
 * @param[out] b            KKT Matrix right-hand side vector
 * @param[in]  rx     		Residual
 * @param[in]  ry     		Residual
 * @param[in]  rz     		Residual
 * @param[in]  ds     		vector ds
 * @param[in]  z     		Residual
 * @param[in]  m     		# of inequality constraints
 * @param[in]  n     		# of decision variables
 * @param[in]  p            # of equality constraints
 *
 *
 *		b = [rx]
 *			[ry]
 *			[rz] - [ds]/[z]
 */
void updatekktmatrix_b(qp_real *b, qp_real *rx, qp_real *ry, qp_real *rz, qp_real *ds, qp_real *z, qp_int n, qp_int m, qp_int p)
{

	qp_int i;
	for (i = 0; i < n; i++)
	{
		b[i] = rx[i];
	}
		

	for (i = n; i < n + p; i++)
	{
		b[i] = ry[i - n];
	}
		

	for (i = n + p; i < n + m + p; i++)
	{
		b[i] = rz[i - n - p] - (ds[i - n - p] / z[i - n - p]);
	}
		
}

/*!
 * @brief  Updates the ds vector based on the selector
 *
 * @param[out] ds           vector
 * @param[in]  lambda     	Scaling Point
 * @param[in]  delta_s      Search Direction
 * @param[in]  delta_z      Search Direction
 * @param[in]  sigma        centering parameter
 * @param[in]  mu     		complementary condition
 * @param[in]  m     		# of inequality constraints
 * @param[in]  selector     selector
 *
 *
 * 	selector = 1 => ds = -lambda*lambda; \n
 * 	selector = 2 => ds = -lambda*lambda - (delta_s*delta_z) + (sigma*mu); \n
 * 	selector = 3 => ds = -lambda*lambda + (sigma*mu); \n
 *
 */
void form_ds(qp_real *ds, qp_real *lambda, qp_real *delta_s, qp_real *delta_z, qp_real sigma, qp_real mu, qp_int m, qp_int selector)
{
	qp_int i;
	/* Pure Newton Step */
	if (selector == QP_PURE_NEWTON_STEP)
	{
		for (i = 0; i < m; i++)
		{
			ds[i] = -lambda[i] * lambda[i];
		}
			
	}
	/* Centering + Corrector Step */
	if (selector == QP_CENTERING_CORRECTOR_STEP)
	{
		for (i = 0; i < m; i++)
		{
			ds[i] = -(lambda[i] * lambda[i]) - (delta_s[i] * delta_z[i]) + (sigma * mu);
		}
			
	}
	/* Centering Step */
	if (selector == QP_CENTERING_STEP)
	{
		for (i = 0; i < m; i++)
		{
			ds[i] = -(lambda[i] * lambda[i]) + (sigma * mu);
		}
			
	}
}

/*!
 * @brief  Calculates the step length
 *
 * @param[out] alpha_p      Primal step length
 * @param[out] alpha_d     	Dual step length
 * @param[in]  s      		Primal slack variable
 * @param[in]  delta_s      Search Direction
 * @param[in]  z      		Dual slack variable
 * @param[in]  delta_z      Search Direction
 * @param[in]  m     		# of inequality constraints
 *
 */
void findsteplength(qp_real *s, qp_real *delta_s, qp_real *z, qp_real *delta_z, qp_int m, qp_real *alpha_p, qp_real *alpha_d)
{

	alpha_p[0] = 1e10;
	alpha_d[0] = 1e10;
	qp_int Flag1 = 0;
	qp_int Flag2 = 0;
	qp_int i;

	for (i = 0; i < m; i++)
	{
		if (delta_s[i] < 0 && (-s[i] / delta_s[i]) < alpha_p[0])
		{
			alpha_p[0] = -(s[i] / delta_s[i]);
			Flag1 = 1;
		}
		if (delta_z[i] < 0 && (-z[i] / delta_z[i]) < alpha_d[0])
		{
			alpha_d[0] = -(z[i] / delta_z[i]);
			Flag2 = 1;
		}
	}

	if (!Flag1)
	{
		alpha_p[0] = 1;
	}
		

	if (!Flag2)
	{
		alpha_d[0] = 1;
	}
		
}

/*!
 * @brief  Checks if x + alpha*delta_x < 0
 *
 * @param[out] Flag         0 : Failure   1 : Success
 * @param[in]  x     	    primal variable
 * @param[in]  delta_x      search direction
 * @param[in]  alpha        scalar value
 *
 */
qp_int checksign(qp_real *x, qp_real *delta_x, qp_real alpha, qp_int count)
{

	qp_int Flag = 0;
	qp_int i;
	for (i = 0; i < count; i++)
	{
		if (x[i] < -alpha * delta_x[i])
		{
			Flag = 1;
			break;
		}
	}
		

	return Flag;
}

/*!
 * @brief  Calculates the Eucledian two norm of vector
 *
 * @param[out] value        Norm of the vector
 * @param[in]  p     	    Vector
 * @param[in]  n            Length of the vector
 *
 */
qp_real norm(qp_real *p, qp_int n)
{
	qp_real k = 0;
	qp_int i;
	for (i = 0; i < n; i++)
	{
		k += p[i] * p[i];
	}
		
	return sqrt(k);
}

/*!
 * @brief  Calculates the inner product of two vectors
 *
 * @param[out] value        Inner Product of the vectors x and y i.e, <x,y>
 * @param[in]  x     	    vector x
 * @param[in]  y			vector y
 * @param[in]  n            length of vectors x and y
 *
 */
qp_real innerproduct(qp_real *x, qp_real *y, qp_int n)
{
	qp_real sum = 0;
	qp_int i;
	for (i = 0; i < n; i++)
	{
		sum += x[i] * y[i];
	}
		

	return sum;
}

/*!
 * @brief  Solves the KKT linear systems and updates delta_z and delta_s
 *
 * @param[out] Flag         Flag indicating status of the function; 0 : Failure 1 : Successful
 * @param[in]  myQP			QP Structure
 *
 */
qp_int kktsolve_1(QP *myQP)
{

	qp_int i, Flag, d;
	qp_int n = myQP->kkt->kktmatrix->n;
	qp_timer t;
	tic(&t);
	d = LDL_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Lp, myQP->kkt->Parent, myQP->kkt->Lnz, myQP->kkt->Li, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->Pattern, myQP->kkt->Flag, myQP->kkt->P, myQP->kkt->Pinv);

	/*  d = LDL_cache_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Ltp, myQP->kkt->Lti, myQP->kkt->Li, myQP->kkt->Lp, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->P, myQP->kkt->Pinv,myQP->kkt->UPattern,myQP->kkt->work); */

	/* d = LDL_row_cache_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Ltp, myQP->kkt->Lti, myQP->kkt->Li, myQP->kkt->Lp, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->P, myQP->kkt->Pinv, myQP->kkt->UPattern); */

	myQP->stats->ldl_numeric += toc(&t);
	if (d == n)
	{
		/* solve Ax=b, overwriting b with the solution x */
		LDL_perm(n, myQP->delta, myQP->kkt->b, myQP->kkt->P);
		LDL_lsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
		LDL_dsolve(n, myQP->delta, myQP->kkt->D);
		LDL_ltsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
		LDL_permt(n, myQP->kkt->b, myQP->delta, myQP->kkt->P);
		Flag = 1;
	}
	else
	{
		Flag = 0;
	}

	if (Flag)
	{
		for (i = myQP->n + myQP->p; i < myQP->n + myQP->p + myQP->m; i++)
		{
			myQP->delta_z[i - myQP->n - myQP->p] = myQP->kkt->b[i];
		}
			

		for (i = 0; i < myQP->m; i++)
		{
			myQP->delta_s[i] = (myQP->ds[i] - (myQP->s[i] * myQP->delta_z[i])) / myQP->z[i];
		}
			
	}
	return Flag;
}

/*!
 * @brief  Solves the kktlinear system from results of kktsolve_1 and updates delta_x, delta_y, delta_z and delta_s
 *
 *
 * @param[in]  myQP    	    QP Structure
 *
 */
void kktsolve_2(QP *myQP)
{

	qp_int i;
	qp_int n = myQP->kkt->kktmatrix->n;

	if(myQP->stats->resolve_kkt)
	{
		qp_int d = LDL_numeric(n, myQP->kkt->kktmatrix->jc, myQP->kkt->kktmatrix->ir, myQP->kkt->kktmatrix->pr, myQP->kkt->Lp, myQP->kkt->Parent, myQP->kkt->Lnz, myQP->kkt->Li, myQP->kkt->Lx, myQP->kkt->D, myQP->kkt->Y, myQP->kkt->Pattern, myQP->kkt->Flag, myQP->kkt->P, myQP->kkt->Pinv);
	}
    
	LDL_perm(n, myQP->delta, myQP->kkt->b, myQP->kkt->P);
	LDL_lsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
	LDL_dsolve(n, myQP->delta, myQP->kkt->D);
	LDL_ltsolve(n, myQP->delta, myQP->kkt->Lp, myQP->kkt->Li, myQP->kkt->Lx);
	LDL_permt(n, myQP->kkt->b, myQP->delta, myQP->kkt->P);

	for (i = 0; i < myQP->n; i++)
	{
		myQP->delta_x[i] = myQP->kkt->b[i];
	}
		

	for (i = myQP->n; i < myQP->n + myQP->p; i++)
	{
		myQP->delta_y[i - myQP->n] = myQP->kkt->b[i];
	}
		

	for (i = myQP->n + myQP->p; i < myQP->n + myQP->p + myQP->m; i++)
	{
		myQP->delta_z[i - myQP->n - myQP->p] = myQP->kkt->b[i];
	}
		

	for (i = 0; i < myQP->m; i++)
	{
		myQP->delta_s[i] = (myQP->ds[i] - (myQP->s[i] * myQP->delta_z[i])) / myQP->z[i];
	}
		
}

/*!
 * @brief  Solves the kkt linear system to find initial conditions
 *
 * @param[in]  mykkt    	    KKT Structure
 * @param[in]  delta			delta varible
 *
 *
 * 	Invoked by kkt_initialize
 * 	Creates and updates the LDL workspace variables
 * 	Performs ldl_symbolic and stores the results
 * 	Also Performs LDL_numeric ; LDL_perm; LDL_lsolve; LDL_dsolve; LDL_ltsolve; LDL_permt in the same order
 */
qp_int ldlinitialsolve(kkt *mykkt, qp_real *delta)
{

	qp_int lnz, d;
	qp_int n;
	n = mykkt->kktmatrix->n;

	/* Allocate Memory */
	mykkt->Y = (qp_real *)MALLOC(n * sizeof(qp_real));
	mykkt->D = (qp_real *)MALLOC(n * sizeof(qp_real));

	mykkt->Lp = (qp_int *)MALLOC((n + 1) * sizeof(qp_int));
	mykkt->Parent = (qp_int *)MALLOC(n * sizeof(qp_int));
	mykkt->Pattern = (qp_int *)MALLOC(n * sizeof(qp_int));
	mykkt->Flag = (qp_int *)MALLOC(n * sizeof(qp_int));
	mykkt->Lnz = (qp_int *)MALLOC(n * sizeof(qp_int));

	/* factorize A into LDL' (P and Pinv used) */

	LDL_symbolic(n, mykkt->kktmatrix->jc, mykkt->kktmatrix->ir, mykkt->Lp, mykkt->Parent, mykkt->Lnz, mykkt->Flag, mykkt->P, mykkt->Pinv);

	lnz = mykkt->Lp[n];

	mykkt->Li = (qp_int *)MALLOC((lnz + 1) * sizeof(qp_int));
	mykkt->Lx = (qp_real *)MALLOC((lnz + 1) * sizeof(qp_real));

	d = LDL_numeric(mykkt->kktmatrix->n, mykkt->kktmatrix->jc, mykkt->kktmatrix->ir, mykkt->kktmatrix->pr, mykkt->Lp, mykkt->Parent, mykkt->Lnz, mykkt->Li, mykkt->Lx, mykkt->D, mykkt->Y, mykkt->Pattern, mykkt->Flag, mykkt->P, mykkt->Pinv);

	mykkt->Lti = (qp_int *)MALLOC(lnz * sizeof(qp_int));
	mykkt->Ltp = (qp_int *)MALLOC((n + 1) * sizeof(qp_int));

	Transpose_Row_Count(n, n, mykkt->Li, mykkt->Lp, mykkt->Lti, mykkt->Ltp);

	if (d == n)
	{
		/* solve Ax=b, overwriting b with the solution x */
		LDL_perm(n, delta, mykkt->b, mykkt->P);
		LDL_lsolve(n, delta, mykkt->Lp, mykkt->Li, mykkt->Lx);
		LDL_dsolve(n, delta, mykkt->D);
		LDL_ltsolve(n, delta, mykkt->Lp, mykkt->Li, mykkt->Lx);
		LDL_permt(n, mykkt->b, delta, mykkt->P);
		return (1);
	}
	else
	{
		return (0);
	}
}

/*!
 * @brief  Computes the scaling point lambda
 *
 * @param[out]  lambda    	    Scaling Point
 * @param[in]   s				Primal slack variable
 * @param[in]   z				Dual variable
 * @param[in]   n				length of the vector
 *
 *
 * 	lambda = sqrt(s/z)
 */
void formlambda(qp_real *lambda, qp_real *s, qp_real *z, qp_int n)
{
	qp_int i;
	for (i = 0; i < n; i++)
	{
		lambda[i] = sqrt(s[i] * z[i]);
	}
		
}

/*!
 * @brief  Sets up the Sparse Matrix in Column Compressed Storage Format based on inputs
 *
 * @param[out]  sparse    	    Sparse Matrix structure
 * @param[in]   m				Number of rows of the matrix
 * @param[in]   n				Number of Columns of the matrix
 * @param[in]   nnz				Number of Non zeros of the matrix
 * @param[in]   jc				Vector to store column count ; Dim [n+1]
 * @param[in]   ir				Vector to store row indices in column major format ; Dim[nnz]
 * @param[in]   pr				Vector to store matrix values in column major format ; Dim[nnz]
 *
 */
void SparseMatrixSetup(qp_int m, qp_int n, qp_int nnz, qp_int *jc, qp_int *ir, qp_real *pr, smat *sparse)
{

	sparse->ir = ir;
	sparse->jc = jc;
	sparse->pr = pr;
	sparse->m = m;
	sparse->n = n;
	sparse->nnz = nnz;
}

/*!
 * @brief  Computes the ir and jc of transpose of a Matrix
 *
 * @param[out]  Lti    	    	ir vector of the transpose of sparse choelsky matrix L (Lt->ir)
 * @param[out]  Ltp				jc vector of the transpose of sparse choelsky matrix L (Lt->jc)
 * @param[in]   m				Number of rows the matrix L
 * @param[in]   n				Number of columns of the matrix L
 * @param[in]   Li				ir vector of the sparse choelsky matrix L (L->ir)
 * @param[in]   Lp				jc vector of the sparse choelsky matrix L (L->jc)
 *
 *
 */
void Transpose_Row_Count(qp_int m, qp_int n, qp_int *Li, qp_int *Lp, qp_int *Lti, qp_int *Ltp)
{

	qp_int i, j, k, index;

	qp_int *count;
	count = (qp_int *)MALLOC(m * sizeof(qp_int));

	for (j = 0; j < m; j++)
	{
		count[j] = 0;
	}
		

	for (i = 0; i < n; i++)
	{
		for (j = Lp[i]; j < Lp[i + 1]; j++)
		{
			k = Li[j];
			count[k]++;
		}
	}

	Ltp[0] = 0;
	for (j = 0; j < m; j++)
	{
		Ltp[j + 1] = Ltp[j] + count[j];
	}
		

	for (j = 0; j < m; j++)
	{
		count[j] = 0;
	}
		

	for (i = 0; i < n; i++)
	{
		for (j = Lp[i]; j < Lp[i + 1]; j++)
		{
			k = Li[j];
			index = Ltp[k] + count[k];
			Lti[index] = i;
			count[k]++;
		}
	}
		

	FREE(count);
}

/*!
 * @brief  Computes the residuals rx, ry and rz
 *
 * @param[out] myQP				QP Structure
 *
 *
 *		rx = Px + G'z +c
 *		ry = Ax - b
 *		rz = -s - Gx + h
 *		mu = s'z/m
 */
void computeresiduals(QP *myQP)
{

	/* Compute rx = Px + G'z +c in three steps */
	/* Compute rx = -Px - G'z - c in three steps */
	/* Compute rx = -Px - A'y -G'z - c */

	qp_int i;
	SparseMatrixMultiply(myQP->P, myQP->x, myQP->rx, 1);
	SparseMatrixTransMultiply(myQP->G, myQP->z, myQP->rx, 0);
	if (myQP->p)
	{
		SparseMatrixTransMultiply(myQP->A, myQP->y, myQP->rx, 0);
	}
		

	updatevariables(myQP->rx, myQP->c, -1, myQP->n);
	myQP->stats->n_rx = norm(myQP->rx, myQP->n);

	/* Compute ry = Ax - b in two steps */
	/* Compute ry = -Ax + b in two steps */

	if (myQP->p)
	{
		SparseMatrixMultiply(myQP->A, myQP->x, myQP->ry, 1);
		updatevariables(myQP->ry, myQP->b, 1, myQP->p);
		myQP->stats->n_ry = norm(myQP->ry, myQP->p);
	}

	/* Compute rz = s + G*x - h in two steps */
	/* Compute rz = -s - Gx + h in two steps */
	SparseMatrixMultiply(myQP->G, myQP->x, myQP->rz, 1);
	for (i = 0; i < myQP->m; i++)
	{
		myQP->rz[i] += myQP->h[i] - myQP->s[i];
	}
		

	myQP->stats->n_rz = norm(myQP->rz, myQP->m);

	myQP->stats->n_mu = innerproduct(myQP->s, myQP->z, myQP->m) / myQP->m;
}

/*!
 * @brief  Performs Sparse Matrix Transpose Vector Multiplication as
 *
 * @param[out] y				Outut vector y
 * @param[in]  A 		   	    Sparse Matrix structure
 * @param[in]  x				Number of rows of the matrix
 * @param[in]  start			Selection index
 *
 *
 * 	Computes y = y - A'x
 *		start = 0 ; do nothing
 *		start !=0 ; intialize y=0
 *
 */
void SparseMatrixTransMultiply(smat *A, qp_real *x, qp_real *y, qp_int start)
{

	qp_int i, j, k;

	if (start)
	{
		for (i = 0; i < A->n; i++)
		{
			y[i] = 0;
		}
			
	}
		
	for (j = 0; j < A->n; j++)
	{
		for (k = A->jc[j]; k < A->jc[j + 1]; k++)
		{
			y[j] -= A->pr[k] * x[A->ir[k]];
		}
	}
}

/*!
 * @brief  Performs Sparse Matrix Vector Multiplication as
 *
 * @param[out] y				Outut vector y
 * @param[in]  A 		   	    Sparse Matrix structure
 * @param[in]  x				Number of rows of the matrix
 * @param[in]  start			Selection index
 *
 *
 * 	Computes y = y - Ax
 *		start = 0 ; do nothing
 *		start !=0 ; intialize y=0
 *
 */
void SparseMatrixMultiply(smat *A, qp_real *x, qp_real *y, qp_int start)
{
	qp_int i, j;

	if (start)
	{
		for (i = 0; i < A->m; i++)
		{
			y[i] = 0;
		}
			
	}
		

	for (i = 0; i < A->n; i++)
	{
		for (j = A->jc[i]; j < A->jc[i + 1]; j++)
		{
			y[A->ir[j]] -= x[i] * A->pr[j];
		}
	}
}

/*!
 * @brief  Computes the scalar rho as
 *
 * @param[out]  rho    	        scalar value rho
 * @param[in]   s				Primal slack variable
 * @param[in]   delta_s		    delta_s
 * @param[in]   z				Dual Variable
 * @param[in]   delta_z			delta_z
 * @param[in]   alpha_p			Primal Step length
 * @param[in]   alpha_d         Dual Step length
 * @param[in]   n               length of vectors s,z,delta_s and delta_z (# of ineq. constraints)
 *
 *
 *
 * 	Computes the scalar rho as
 * 	rho = (s+alpha_p*delta_s)*(z+alpha_d*delta_z)/s'z;
 */
qp_real formrho(qp_real *s, qp_real *delta_s, qp_real *z, qp_real *delta_z, qp_real alpha_p, qp_real alpha_d, qp_int n)
{

	qp_real sum = 0.0;
	qp_int i;
	for (i = 0; i < n; i++)
	{
		sum += (s[i] + (alpha_p * delta_s[i])) * (z[i] + (alpha_d * delta_z[i]));
	}

	sum = sum / innerproduct(s, z, n);

	return sum;
}

/*!
 * @brief  Computes the sparse matrix transpose
 *
 * @param[out]  A   	 	    Sparse Matrix
 * @param[in]   At				Transpose of the Sparse Matrix
 *
 */
void SparseMatrixTranspose(smat *A, smat *At)
{
	qp_int i, j, k, index;

	qp_int *count;

	count = (qp_int *)MALLOC(A->m * sizeof(qp_int));

	for (j = 0; j < A->m; j++)
	{
		count[j] = 0;
	}
		

	for (i = 0; i < A->n; i++)
	{
		for (j = A->jc[i]; j < A->jc[i + 1]; j++)
		{
			k = A->ir[j];
			count[k]++;
		}
	}

	At->jc[0] = 0;
	for (j = 0; j < A->m; j++)
	{
		At->jc[j + 1] = At->jc[j] + count[j];
	}
		

	for (j = 0; j < A->m; j++)
	{
		count[j] = 0;
	}
		

	for (i = 0; i < A->n; i++)
	{
		for (j = A->jc[i]; j < A->jc[i + 1]; j++)
		{
			k = A->ir[j];
			index = At->jc[k] + count[k];
			At->ir[index] = i;
			At->pr[index] = A->pr[j];
			count[k]++;
		}
	}
		

	FREE(count);
}

/*!
 * @brief  Computes the minimum and maximum of the vector
 *
 * @param[out]  min    	    	Minimum of the vector z
 * @param[in]   max				Maximum of the vector z
 * @param[in]   n				Vector z
 * @param[in]   nnz				Length of the vector z
 *
 */
void findminmax(qp_real *z, long n, qp_real *min, qp_real *max)
{

	min[0] = z[0];
	max[0] = z[0];
	qp_int i;
	for (i = 1; i < n; i++)
	{
		if (z[i] < min[0])
		{
			min[0] = z[i];
		}
		if (z[i] > max[0])
		{
			max[0] = z[i];
		}
	}
}

/*!
 * @brief  Computes the initial condition for the QP problem
 *
 * @param[in]  myQP    	    QP Matrix Structure
 *
 *
 *		Solves a closely related QP of the form
 *		min. 0.5*x'Px + c'x + s's
 *		s.t		Ax = b
 *				Gx + s = h
 */
qp_int kkt_initialize(QP *myQP)
{

	qp_real *z_inter;
	qp_real alpha_p;
	qp_real alpha_d;
	qp_int i;
	qp_int n, m, p;
	n = myQP->n;
	m = myQP->m;
	p = myQP->p;

	z_inter = (qp_real *)MALLOC(myQP->m * sizeof(qp_real));

	for (i = 0; i < n; i++)
	{
		myQP->kkt->b[i] = -myQP->c[i];
	}
		

	for (i = n; i < n + p; i++)
	{
		myQP->kkt->b[i] = myQP->b[i - n];
	}
		

	for (i = n + p; i < n + p + m; i++)
	{
		myQP->kkt->b[i] = myQP->h[i - n - p];
	}
		

	qp_int Flag = ldlinitialsolve(myQP->kkt, myQP->delta);

	/* test_reach(myQP->kkt->Parent, myQP->kkt->Pinv, myQP->kkt->UPattern, n, m, p); */

	if (Flag)
	{
		for (i = 0; i < n; i++)
		{
			myQP->x[i] = myQP->kkt->b[i];
		}
			

		for (i = n; i < n + p; i++)
		{
			myQP->y[i - n] = myQP->kkt->b[i];
		}
			

		/* Calculate z_inter = Gx - h in two steps */
		/* Calculate z_inter = -Gx */
		SparseMatrixMultiply(myQP->G, myQP->x, z_inter, 1);
		/* Add h to z */
		updatevariables(z_inter, myQP->h, 1, m);

		/* find alpha_p and alpha_d */
		findminmax(z_inter, myQP->m, &alpha_p, &alpha_d);
		alpha_p = -alpha_p;
		if (alpha_p < 0)
		{
			for (i = 0; i < m; i++)
			{
				myQP->s[i] = z_inter[i];
			}
				
		}
		else
		{
			for (i = 0; i < m; i++)
			{
				myQP->s[i] = z_inter[i] + (1 + alpha_p);
			}
				
		}

		if (alpha_d < 0)
		{
			for (i = 0; i < m; i++)
			{
				myQP->z[i] = -z_inter[i];
			}
				
		}
		else
		{
			for (i = 0; i < m; i++)
			{
				myQP->z[i] = -z_inter[i] + (1 + alpha_d);
			}
				
		}
	}

	FREE(z_inter);

	return Flag;
}

/*!
 * @brief  Computes the nodes to be updated at each iteration
 *
 * @param[out]  UPattern    	Reach of the nodes of the lower diagonal part of KKT Matrix
 * @param[in]   n				# of decision variables
 * @param[in]   m				# of inequality constraints
 * @param[in]   p				# of equality constraints
 * @param[in]   Parent			Parent tree of the KKT Matrix
 * @param[in]   Pinv			Inverse of Permutation vector
 *
 *
 */

void test_reach(qp_int *Parent, qp_int *Pinv, qp_int *UPattern, qp_int n, qp_int m, qp_int p)
{
	qp_int i, j;
	qp_int top = n + m + p;
	UPattern[n + m + p - 1] = top;

	for (j = n + p; j < n + p + m; j++)
	{
		i = Pinv[j];

		for (; UPattern[i] != top && i != -1; i = Parent[i])
		{
			UPattern[i] = top;
		}
	}
}

/*!
 * @brief  Computes the objective function value f = x'Px + c'x
 *
 * @param[out]  fval         	Objective Value of QP
 * @param[in]   P				cost Function : quadratic part
 * @param[in]   c				cost function : linear term
 * @param[in]   x				primal solution
 * @param[in]   temp			temporary workspace variable
 *
 *
 */

qp_real obj_value(smat *P, qp_real *c, qp_real *x, qp_real *temp)
{

	SparseMatrixMultiply(P, x, temp, 1);

	/* Negative sign because sparsematrixmutilpy performs temp = temp - Px; */

	return -0.5 * innerproduct(temp, x, P->n) + innerproduct(c, x, P->n);
}

/*!
 * @brief  Converts dense matrix in column major format to CCS format
 *
 * @param[out]  A         	    Sparse Matrix A
 * @param[in]   m				number of rows of A
 * @param[in]   n				number of columns of A
 * @param[in]   pr				pointer to matrix in column major format
 *
 *
 */

void densetosparse(qp_int m, qp_int n, qp_real *pr, smat *A)
{

	qp_int *Ap, *Ai;
	qp_real *Ax;

	qp_int count = 0;

	Ap = (qp_int *)MALLOC((n + 1) * sizeof(qp_int));

	qp_int i, j;
	for (i = 0; i < n + 1; i++)
	{
		Ap[i] = 0;
	}
		

	for (j = 0; j < n; j++)
	{
		for (i = 0; i < m; i++)
		{
			if (pr[j * m + i] != 0.0)
			{
				count++;
			}
		}
		Ap[j + 1] = count;
	}

	Ai = (qp_int *)MALLOC((count) * sizeof(qp_int));
	Ax = (qp_real *)MALLOC((count) * sizeof(qp_real));

	count = 0;
	for (j = 0; j < n; j++)
	{
		for (i = 0; i < m; i++)
		{
			if (pr[j * m + i] != 0)
			{
				Ai[count] = i;
				Ax[count] = pr[j * m + i];
				count++;
			}
		}
	}

	A->ir = Ai;
	A->jc = Ap;
	A->pr = Ax;
	A->m = m;
	A->n = n;
	A->nnz = count;
}

/*!
 * @brief  Converts dense matrix in row major format to CCS format
 *
 * @param[out]  A         	    Sparse Matrix A
 * @param[in]   m				number of rows of A
 * @param[in]   n				number of columns of A
 * @param[in]   pr				pointer to matrix in row major format
 *
 *
 */

void densetosparse_ROWMAJOR(qp_int m, qp_int n, qp_real *pr, smat *A)
{

	qp_int *Ap, *Ai;
	qp_real *Ax;

	qp_int count = 0;

	Ap = (qp_int *)MALLOC((n + 1) * sizeof(qp_int));

	qp_int i,j;

	for (i = 0; i < n + 1; i++)
	{
		Ap[i] = 0;
	}

	for (i = 0; i < m; ++i)
	{
		for (j = 0; j < n; ++j)
		{
			if (pr[i * n + j] != 0.0)
			{
				count++;
				Ap[j + 1]++;
			}
		}
	}

	Ai = (qp_int *)MALLOC((count) * sizeof(qp_int));
	Ax = (qp_real *)MALLOC((count) * sizeof(qp_real));

	count = 0;
	qp_real temp;
	for (j = 0; j < n; ++j)
	{
		for (i = 0; i < m; ++i)
		{
			temp = pr[i * n + j];
			if (temp != 0)
			{
				Ai[count] = i;
				Ax[count++] = temp;
			}
		}
		Ap[j + 1] += Ap[j];
	}

	A->ir = Ai;
	A->jc = Ap;
	A->pr = Ax;
	A->m = m;
	A->n = n;
	A->nnz = count;
}

/*! @file */
