/*********************************************************************************************************
* qpSWIFT Matlab Interface
*
* For help, refer to qpSWIFT.m
*
*
*
*********************************************************************************************************/
#include "mex.h"
#include "qpSWIFT.h"
#include "string.h"
#define BASIC_NO_FIELDS 4
#define ADV_NO_FIELDS 6

char helpstr[] = {
   " qpSWIFT - Light Weight Interior Point QP Solver \n"
" --------------------------------------------------- \n" 
" -------------- Installation ----------------------- \n"
"   \n"
"   Swift_make('qpSWIFT_mex.c'); \n"
" \n"
" --------------------------------------------------- \n"
" --------------------------------------------------- \n"
"	General Quadratic Program \n"
" \n"
"   [sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h) \n"
"			or \n"
"   [sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts) \n"
" \n"
"        minimize    0.5*x'Px + c'x \n"
"        subject to  Ax = b \n"
"                    Gx <= h \n"
" \n"
" --------------------------------------------------- \n"
"   Only Inequality Constrained Quadratic Program \n"
" \n"
"  	[sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(G),h) \n"
"			or \n"
"  	[sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(G),h,opts) \n"
" \n"
"       minimize    0.5*x'Px + c'x \n"
"       subject to  Gx <= h \n"
" \n"
" --------------------------------------------------- \n"
"  \n"
"    INPUT arguments: \n"
" \n"
"       P is a sparse matrix of dimension (n,n) \n"
" \n"
"       c is a dense column vector of size n \n"
" \n"
"       A is a sparse matrix of size (p,n); p is number of equality constraints \n"
" \n"
"       b is a dense column vector of size p \n"
" \n"
"       G is a sparse matrix of size (m,n); m is the number of inequality constraints \n"
" \n"
"       h is a dense column vector of size m \n"
" \n"
"       Opts is a structure with the following fields \n"
"  \n"
"           -> MAXITER : maximum number of iterations needed \n"
"           -> ABSTOL  : absolute tolerance \n"
"           -> RELTOL  : relative tolerance \n"
"           -> SIGMA   : maximum centering allowed \n"
"           -> VERBOSE : PRINT LEVELS ||  0 -- No Print \n"
"                                     || >0 -- Print everything \n"
"           -> Permut  : permutation vector obtained as \n"
"  \n"
"           	KKT = [P A' G'; \n"
"                    A 0   0; \n"
"                    G 0 -I]; \n"
"             Permut = amd(KKT); \n"
" \n"
"   Note: Options are not mandatory \n"
"   Note: All the fileds of Options are also not mandatory \n"
"   Note: All the input Matrices should be sparse \n"
" -------------------------------------------------- \n"
"  \n"
"	OUTPUT arguments: \n"
"  \n"
"   sol represents the primal solution of the QP \n"
"  \n"
"   basic_info has four fields \n"
"       -> Exit Flag : 0 : Optimal Solution Found \n"
"                    : 1 : Failure in factorising KKT matrix \n"
"                    : 2 : Maximum Number of Iterations Reached \n"
"                    : 3 : Unknown Problem in Solver \n"
"      -> Iterations : Number of Iterations \n"
"      -> Setup Time : Invloves setting up QP; solving for initial guess \n"
"      -> Solve Time : Solution Time \n"
" \n"
"   adv_info  has five fields \n"
"      -> Fval       : Objective Value of the QP \n"
"      -> KKT_Time   : Time needed to solve the KKT system of equations \n"
"      -> LDL_Time   : Time needed to perform LDL' factorization \n"
"      -> y          : Dual Variables \n"
"      -> z          : Dual Variables \n"
"      -> s          : Primal Slack Variables \n"
};

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if (!nlhs)
    {
        PRINT(helpstr);
        return;
    }
    // Arguments above 7 and below 4 are not accepted
    if (nrhs < 4 || nrhs > 7)
        mexErrMsgTxt("Please enter the correct number of input arguments");
    
    // Output Arguments must be less than 3
    if (nlhs<0 || nlhs>3)
        mexErrMsgTxt("The number of outputs is restricted to 3");
    
    /* Initialize Output Structure Fields */

    // Initialize Basic Information Structure Fields
    const char *basic_field_names[] = {"Iterations", "ExitFlag", "Setup_Time", "Solve_Time"};
    // Initialize Advanced Information Structure Fields
    const char *adv_field_names[] = {"Fval", "KKT_Time", "LDL_Time", "y", "z", "s"};

    /* Declare Variables for size of Input arguments*/
    const mwSize *size_P;
    const mwSize *size_c;
    const mwSize *size_A = NULL;
    const mwSize *size_b = NULL;
    const mwSize *size_G;
    const mwSize *size_h;
    const mwSize *size_Permut = NULL;
    const mwSize dims[2] = {1, 1};
    const mwSize ZERO[2] = {0, 0};

    /* Declare Variables for Input Arguments*/
    const mxArray *P;
    const mxArray *c;
    const mxArray *A = NULL;
    const mxArray *b = NULL;
    const mxArray *G;
    const mxArray *h;

    /* Output Variables */

    // Basic Output Variables

    mxArray *tset = NULL;
    mxArray *tsol = NULL;
    mxArray *exitfg = NULL;
    mxArray *iter = NULL;

    // Advanced Output Variables

    mxArray *tkkt = NULL;
    mxArray *tldl = NULL;
    mxArray *fval = NULL;
    mxArray *sol_y = NULL;
    mxArray *sol_z = NULL;
    mxArray *sol_s = NULL;

    /* Output Variables */

    /* Declare Input Arguments for qpSWIFT Functions*/
    qp_real *Ppr;
    qp_real *cpr;
    qp_real *Apr = NULL;
    qp_real *bpr = NULL;
    qp_real *Gpr;
    qp_real *hpr;
    qp_int n;
    qp_int m;
    qp_int p;

    qp_int *Pir;
    qp_int *Pjc;
    qp_int *Air = NULL;
    qp_int *Ajc = NULL;
    qp_int *Gir;
    qp_int *Gjc;
    qp_int *Permutpr = NULL;
    qp_int *Flag = NULL;

    /* Declare Input Arguments for Settings */
    const mxArray *sttgs = NULL;
    const mxArray *sttgs_maxiter = NULL;
    const mxArray *sttgs_abstol = NULL;
    const mxArray *sttgs_reltol = NULL;
    const mxArray *sttgs_sigma = NULL;
    const mxArray *sttgs_verbose = NULL;
    const mxArray *sttgs_permut = NULL;

    /* Variable Assignment */
    if (nrhs == 4)
    {
        P = prhs[0];
        size_P = P ? mxGetDimensions(P) : (const mwSize *)&ZERO;
        c = prhs[1];
        size_c = c ? mxGetDimensions(c) : (const mwSize *)&ZERO;
        G = prhs[2];
        size_G = G ? mxGetDimensions(G) : (const mwSize *)&ZERO;
        h = prhs[3];
        size_h = h ? mxGetDimensions(h) : (const mwSize *)&ZERO;
    }
    else if (nrhs == 5)
    {
        P = prhs[0];
        size_P = P ? mxGetDimensions(P) : (const mwSize *)&ZERO;
        c = prhs[1];
        size_c = c ? mxGetDimensions(c) : (const mwSize *)&ZERO;
        G = prhs[2];
        size_G = G ? mxGetDimensions(G) : (const mwSize *)&ZERO;
        h = prhs[3];
        size_h = h ? mxGetDimensions(h) : (const mwSize *)&ZERO;
        sttgs = prhs[4];
    }
    else if (nrhs == 6)
    {
        P = prhs[0];
        size_P = P ? mxGetDimensions(P) : (const mwSize *)&ZERO;
        c = prhs[1];
        size_c = c ? mxGetDimensions(c) : (const mwSize *)&ZERO;
        A = prhs[2];
        size_A = A ? mxGetDimensions(A) : (const mwSize *)&ZERO;
        b = prhs[3];
        size_b = b ? mxGetDimensions(b) : (const mwSize *)&ZERO;
        G = prhs[4];
        size_G = G ? mxGetDimensions(G) : (const mwSize *)&ZERO;
        h = prhs[5];
        size_h = h ? mxGetDimensions(h) : (const mwSize *)&ZERO;
    }
    else if (nrhs == 7)
    {
        P = prhs[0];
        size_P = P ? mxGetDimensions(P) : (const mwSize *)&ZERO;
        c = prhs[1];
        size_c = c ? mxGetDimensions(c) : (const mwSize *)&ZERO;
        A = prhs[2];
        size_A = A ? mxGetDimensions(A) : (const mwSize *)&ZERO;
        b = prhs[3];
        size_b = b ? mxGetDimensions(b) : (const mwSize *)&ZERO;
        G = prhs[4];
        size_G = G ? mxGetDimensions(G) : (const mwSize *)&ZERO;
        h = prhs[5];
        size_h = h ? mxGetDimensions(h) : (const mwSize *)&ZERO;
        sttgs = prhs[6];
    }

    // Get the number of decision variables, equality and inequality constraints
    n = (qp_int)MAX(size_c[0], size_c[1]);
    m = (qp_int)MAX(size_h[0], size_h[1]);

    if (b)
    {
        p = (qp_int)MAX(size_b[0], size_b[1]);
    }
    else
    {
        p = 0;
    }

    // Error Checking

    if (!mxIsDouble(P) || !mxIsSparse(P) || mxIsComplex(P))
        mexErrMsgTxt("P should be a real sparse matrix");

    if (size_P[0] != size_P[1])
        mexErrMsgTxt("P should be a square matrix");

    if (size_P[0] != n)
        mexErrMsgTxt("Dimensions of P and c do not match");

    if (!mxIsDouble(G) || !mxIsSparse(G) || mxIsComplex(G))
        mexErrMsgTxt("G should be a real sparse matrix");

    if (size_G[0] != m)
        mexErrMsgTxt("Dimensions of G and h do not match");

    if (size_G[1] != n)
        mexErrMsgTxt("Dimensions of G and c do not match");

    if (A)
    {
        if (!mxIsDouble(A) || !mxIsSparse(A) || mxIsComplex(A))
            mexErrMsgTxt("A should be a real sparse matrix");

        if (size_A[0] != p)
            mexErrMsgTxt("Dimensions of A and b do not match");

        if (size_A[1] != n)
            mexErrMsgTxt("Dimensions of A and c do not match");
    }

    if (!mxIsDouble(c) || mxIsSparse(c) || mxIsComplex(c))
        mexErrMsgTxt("c should be a real dense vector");

    if (MIN(size_c[0], size_c[1]) != 1)
        mexErrMsgTxt("c should be a column vector");

    if (!mxIsDouble(h) || mxIsSparse(h) || mxIsComplex(h))
        mexErrMsgTxt("h should be a real dense vector");

    if (MIN(size_h[0], size_h[1]) != 1)
        mexErrMsgTxt("h should be a column vector");

    if (A)
    {
        if (!mxIsDouble(b) || mxIsSparse(b) || mxIsComplex(b))
            mexErrMsgTxt("b should be a real dense vector");

        if (MIN(size_b[1], size_b[0]) != 1)
            mexErrMsgTxt("b should be a column vector");
    }

    sttgs_maxiter = sttgs ? mxGetField(sttgs, 0, "MAXITER") : 0;
    if (!sttgs_maxiter)
    {
        sttgs_maxiter = sttgs ? mxGetField(sttgs, 0, "maxiter") : 0;
    }

    sttgs_abstol = sttgs ? mxGetField(sttgs, 0, "ABSTOL") : 0;
    if (!sttgs_abstol)
    {
        sttgs_abstol = sttgs ? mxGetField(sttgs, 0, "abstol") : 0;
    }

    sttgs_reltol = sttgs ? mxGetField(sttgs, 0, "RELTOL") : 0;
    if (!sttgs_reltol)
    {
        sttgs_reltol = sttgs ? mxGetField(sttgs, 0, "reltol") : 0;
    }

    sttgs_verbose = sttgs ? mxGetField(sttgs, 0, "VERBOSE") : 0;
    if (!sttgs_verbose)
    {
        sttgs_verbose = sttgs ? mxGetField(sttgs, 0, "verbose") : 0;
    }

    sttgs_sigma = sttgs ? mxGetField(sttgs, 0, "SIGMA") : 0;
    if (!sttgs_sigma)
    {
        sttgs_sigma = sttgs ? mxGetField(sttgs, 0, "sigma") : 0;
    }

    sttgs_permut = sttgs ? mxGetField(sttgs, 0, "PERMUT") : 0;
    if (!sttgs_permut)
    {
        sttgs_permut = sttgs ? mxGetField(sttgs, 0, "permut") : 0;
    }

    if (sttgs_permut)
    {

        size_Permut = sttgs_permut ? mxGetDimensions(sttgs_permut) : (const mwSize *)&ZERO;
        if (!mxIsDouble(sttgs_permut) || mxIsSparse(sttgs_permut) || mxIsComplex(sttgs_permut))
            mexErrMsgTxt("Permut should be a real dense vector");

        if (MIN(size_Permut[1], size_Permut[0]) != 1)
            mexErrMsgTxt("Permut should be a column vector");

        if (MAX(size_Permut[1], size_Permut[0]) != n + m + p)
            mexErrMsgTxt("The dimensions of Permutation vector are inconsistent");
    }

    // Copy Data from Matlab input arguments to local variables

    if (P)
    {
        Ppr = (qp_real *)mxGetPr(P);
        Pir = (qp_int *)mxGetIr(P);
        Pjc = (qp_int *)mxGetJc(P);
    }

    if (c)
    {
        cpr = (qp_real *)mxGetPr(c);
    }

    if (A)
    {
        Apr = (qp_real *)mxGetPr(A);
        Air = (qp_int *)mxGetIr(A);
        Ajc = (qp_int *)mxGetJc(A);
    }

    if (b)
    {
        bpr = (qp_real *)mxGetPr(b);
    }

    if (G)
    {
        Gpr = (qp_real *)mxGetPr(G);
        Gir = (qp_int *)mxGetIr(G);
        Gjc = (qp_int *)mxGetJc(G);
    }

    if (h)
    {
        hpr = (qp_real *)mxGetPr(h);
    }

    if (sttgs_permut)
    {

        Permutpr = (qp_int *)MALLOC((m + n + p) * sizeof(qp_int));

        double *pr;
        pr = mxGetPr(sttgs_permut);

        // Change Matlab Indexing to C-based Indexing
        for (qp_int i = 0; i < n + m + p; i++)
        {
            Permutpr[i] = pr[i] - 1;
        }

        // Check if the permutation Matrix is Valid

        Flag = (qp_int *)MALLOC((m + n + p) * sizeof(qp_int));
        if (!LDL_valid_perm(m + n + p, Permutpr, Flag))
            mexErrMsgTxt("Not a Valid Permutation Vector");
    }

    // Initialize the QP structure

    QP *myQP = NULL;

    if (nrhs == 6 || nrhs == 7)
    {
        myQP = QP_SETUP(n, m, p, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, cpr, hpr, bpr, 0.0, Permutpr);
    }
    if (nrhs == 4 || nrhs == 5)
    {
        myQP = QP_SETUP(n, m, 0, Pjc, Pir, Ppr, NULL, NULL, NULL, Gjc, Gir, Gpr, cpr, hpr, NULL, 0.0, Permutpr);
    }

    qp_int EXITCODE;

    // Update User-defined Settings

    if (sttgs_maxiter)
    {

        if (*mxGetPr(sttgs_maxiter) < 0 || *mxGetPr(sttgs_maxiter) > 200)
            mexErrMsgTxt("Iteration Number must be within 0 and 200");

        myQP->options->maxit = (qp_int)(*mxGetPr(sttgs_maxiter));
    }

    if (sttgs_abstol)
    {

        if (*mxGetPr(sttgs_abstol) < 0 || *mxGetPr(sttgs_abstol) > 1)
            mexErrMsgTxt("Absolute Tolerance must be within 0 and 1");

        myQP->options->abstol = (qp_real)(*mxGetPr(sttgs_abstol));
    }
    if (sttgs_reltol)
    {

        if (*mxGetPr(sttgs_reltol) < 0 || *mxGetPr(sttgs_reltol) > 1)
            mexErrMsgTxt("Relative Tolerance must be within 0 and 1");

        myQP->options->reltol = (qp_real)(*mxGetPr(sttgs_reltol));
    }

    if (sttgs_sigma)
    {
        myQP->options->sigma = (qp_real)(*mxGetPr(sttgs_sigma));
        if (*mxGetPr(sttgs_sigma) < 0)
            mexErrMsgTxt("Sigma must be positive");
    }

    if (sttgs_verbose)
    {

        if (*mxGetPr(sttgs_verbose) < 0)
            mexErrMsgTxt("Verbose must be positive");

        myQP->options->verbose = (qp_real)(*mxGetPr(sttgs_verbose));
    }

    if (myQP->options->verbose > 0)
    {
        PRINT("****qpSWIFT : Sparse Quadratic Programming Solver****\n\n");
        PRINT("================Settings Applied======================\n");
        PRINT("Maximum Iterations : %d \n", myQP->options->maxit);
        PRINT("ABSTOL             : %e \n", myQP->options->abstol);
        PRINT("RELTOL             : %e \n", myQP->options->reltol);
        PRINT("SIGMA              : %e \n", myQP->options->sigma);
        PRINT("VERBOSE            : %d \n", myQP->options->verbose);
        if (sttgs_permut)
        {
            PRINT("Permutation vector : User-defined\n\n");
        }
        else
        {
            PRINT("Permutation vector : AMD Solver\n\n");
        }

        PRINT("================Data Statistics======================\n");
    }

    // Solve the QP
    if (myQP != NULL)
    {
        EXITCODE = QP_SOLVE(myQP);
    }

    // Copy the required outputs as necessary

    // Copy just the primal solution
    if (nlhs == 1)
    {
        plhs[0] = mxCreateNumericMatrix(n, 1, mxDOUBLE_CLASS, mxREAL);
        memcpy(mxGetPr(plhs[0]), myQP->x, n * sizeof(double));
    }
    // Copy the basic information
    else if (nlhs == 2)
    {
        plhs[0] = mxCreateNumericMatrix(n, 1, mxDOUBLE_CLASS, mxREAL);
        memcpy(mxGetPr(plhs[0]), myQP->x, n * sizeof(double));

        plhs[1] = mxCreateStructArray(2, dims, BASIC_NO_FIELDS, basic_field_names);
        tset = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        tsol = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        exitfg = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        iter = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);

        *mxGetPr(tset) = (double)myQP->stats->tsetup;
        *mxGetPr(tsol) = (double)myQP->stats->tsolve;
        *mxGetPr(exitfg) = (double)EXITCODE;
        *mxGetPr(iter) = (double)myQP->stats->IterationCount;

        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Setup_Time"), tset);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Solve_Time"), tsol);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "ExitFlag"), exitfg);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Iterations"), iter);
    }
    // Copy the advanced information
    else if (nlhs == 3)
    {

        plhs[0] = mxCreateNumericMatrix(n, 1, mxDOUBLE_CLASS, mxREAL);
        memcpy(mxGetPr(plhs[0]), myQP->x, n * sizeof(double));

        plhs[1] = mxCreateStructArray(2, dims, BASIC_NO_FIELDS, basic_field_names);
        tset = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        tsol = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        exitfg = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        iter = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);

        *mxGetPr(tset) = (double)myQP->stats->tsetup;
        *mxGetPr(tsol) = (double)myQP->stats->tsolve;
        *mxGetPr(exitfg) = (double)EXITCODE;
        *mxGetPr(iter) = (double)myQP->stats->IterationCount;

        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Setup_Time"), tset);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Solve_Time"), tsol);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "ExitFlag"), exitfg);
        mxSetFieldByNumber(plhs[1], 0, mxGetFieldNumber(plhs[1], "Iterations"), iter);

        plhs[2] = mxCreateStructArray(2, dims, ADV_NO_FIELDS, adv_field_names);
        tkkt = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        tldl = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
        fval = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);

        if (p != 0)
        {
            sol_y = mxCreateNumericMatrix(p, 1, mxDOUBLE_CLASS, mxREAL);
            memcpy(mxGetPr(sol_y), myQP->y, p * sizeof(double));
        }

        sol_z = mxCreateNumericMatrix(m, 1, mxDOUBLE_CLASS, mxREAL);
        sol_s = mxCreateNumericMatrix(m, 1, mxDOUBLE_CLASS, mxREAL);

        *mxGetPr(tkkt) = (double)myQP->stats->kkt_time;
        *mxGetPr(tldl) = (double)myQP->stats->ldl_numeric;
        *mxGetPr(fval) = (double)myQP->stats->fval;

        memcpy(mxGetPr(sol_z), myQP->z, m * sizeof(double));
        memcpy(mxGetPr(sol_s), myQP->s, m * sizeof(double));

        mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "KKT_Time"), tkkt);
        mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "LDL_Time"), tldl);
        if (p != 0)
        {
            mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "y"), sol_y);
        }
        mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "z"), sol_z);
        mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "s"), sol_s);
        mxSetFieldByNumber(plhs[2], 0, mxGetFieldNumber(plhs[2], "Fval"), fval);
    }
    else
    {
        mexErrMsgTxt("Number of outputs is restricted to 3");
    }

    // Clean up all the variables
    QP_CLEANUP(myQP);

    if (Permutpr)
        FREE(Permutpr);
    if (Flag)
        FREE(Flag);
}

/*! @file */
