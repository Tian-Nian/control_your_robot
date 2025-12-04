#include "qpSWIFT.h"
#include "iostream"
/*Can be used with Eigen 
* #include "Eigen/Dense"
*/
int main(int argv, char **argc)
{
	/* To use Eigen use the following code
	Eigen::Matrix<qp_real, 3, 3> P;
	Eigen::Matrix<qp_real, 3, 1> c;
	Eigen::Matrix<qp_real, 2, 3> G;
	Eigen::Matrix<qp_real, 2, 1> h;
	Eigen::Matrix<qp_real, 1, 3> A;
	Eigen::Matrix<qp_real, 1, 1> b;
    
	P << 5.0, 1.0, 0.0,
		1.0, 2.0, 1.0,
		0.0, 1.0, 4.0;

	c << 1.0, 2.0, 1.0;

	G << -4.0, -4.0, 0.0,
		0.0, 0.0, -1.0;

	h << -1.0, -1.0;

	A << 1.0, -2.0, 1.0;

	b << 3.0;

	*/

	qp_real P[9] = {5.0, 1.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0, 4.0};
	qp_real c[3] = {1.0, 2.0, 1.0};
	qp_real G[6] = {-4.0, 0.0, -4.0, 0.0, 0.0, -1.0};
	qp_real h[2] = {-1.0, -1.0};
	qp_real A[3] = {1.0, -2.0, 1.0};
	qp_real b[1] = {3.0};

	QP *myQP;

	qp_int n = 3; /*! Number of Decision Variables */
	qp_int m = 2; /*! Number of Inequality Constraints */
	qp_int p = 1; /*! Number of equality Constraints */

	/*! Setup Function Dense Format */
	/*! To use with Eigen */
	/*! myQP = QP_SETUP_dense(n, m, p, P.data(), A.data(), G.data(), c.data(), h.data(), b.data(), NULL, COLUMN_MAJOR_ORDERING); */
	myQP = QP_SETUP_dense(n, m, p, P, A, G, c, h, b, NULL, COLUMN_MAJOR_ORDERING);
	/*! For only inequality constrained QP set the pointers of A matrix and b vectro to zero and p = 0 */
	/*! myQP = QP_SETUP_dense(n, m, 0, P.data(), NULL, G.data(), c.data(), h.data(), NULL, NULL, COLUMN_MAJOR_ORDERING); */
	/*!***************************************
	* 
	*	After this, you can change the solver settings like this
	*	myQP->options->maxit  = 30   (to change the maximum number of iterations to 30; default is 100)
	*	myQP->options->reltol = 1e-3 (to change the Relative tolerance to 1e-3; default is 1e-6)
	*	myQP->options->abstol  = 1e-3 (to change the Absolute tolerance to 1e-3; default is 1e-6)
	*	myQP->options->SIGMA  = 50 (to change the SIGMA to 50; default is 100; recommended not to change this)
	*	myQP->options->VERBOSE  = 0 (displays no output when set to 0; default is 1 which corresponds to complete verbose mode)
	*
	******************************************/

	qp_int ExitCode = QP_SOLVE(myQP);

	if (myQP != NULL)
		printf("Setup Time     : %f ms\n", myQP->stats->tsetup * 1000.0);
	if (ExitCode == QP_OPTIMAL)
	{
		printf("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup) * 1000.0);
		printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
		printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
		printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
		printf("Iterations     : %ld\n", myQP->stats->IterationCount);
		printf("Optimal Solution Found\n");
	}
	if (ExitCode == QP_MAXIT)
	{
		printf("Solve Time     : %f ms\n", myQP->stats->tsolve * 1000.0);
		printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
		printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
		printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
		printf("Iterations     : %ld\n", myQP->stats->IterationCount);
		printf("Maximum Iterations reached\n");
	}

	if (ExitCode == QP_FATAL)
	{
		printf("Unknown Error Detected\n");
	}

	if (ExitCode == QP_KKTFAIL)
	{
		printf("LDL Factorization fail\n");
	}

	/*! The Solution can be found as real pointer in myQP->x;It is an array of Dimension n*/
	std::cout << "Solution" << std::endl;

	for (int i = 0; i < 3; ++i)
	{
		std::cout << "x[" << i << "]: " << myQP->x[i] << std::endl;
	}

	QP_CLEANUP_dense(myQP);

	return 0;
}

/*! @file */