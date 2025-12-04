/* This file creates simulink interface for qpSWIFT solver
*/

#include "qpSWIFT.h"

#define S_FUNCTION_NAME qpSWIFT_sfunc /**< Name of the S function. */
#define S_FUNCTION_LEVEL 2          /**< S function level. */

#define SAMPLINGTIME -1 /**< Sampling time. */
#include "simstruc.h"


#define NV 3 /*Output Needed */

static void mdlInitializeSizes(SimStruct *S) /* Init sizes array */
{

    /* Specify the number of continuous and discrete states */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Specify the number of parameters */
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }

    /* Specify the number of input ports */
    if (!ssSetNumInputPorts(S, 4))
    {
        return;
    }

    /* Specify the number of output ports */
    if (!ssSetNumOutputPorts(S, 5))
    {
        return;
    }
     

    /* Specify dimension information for the input ports */

    ssSetInputPortVectorDimension(S, 0, DYNAMICALLY_SIZED); /* P - Matrix */
    ssSetInputPortVectorDimension(S, 1, DYNAMICALLY_SIZED); /* c - Vector */
    ssSetInputPortVectorDimension(S, 2, DYNAMICALLY_SIZED); /* G - Matrix */
    ssSetInputPortVectorDimension(S, 3, DYNAMICALLY_SIZED); /* h - Vector */

    /* Specify dimension information for the output ports */

    ssSetOutputPortVectorDimension(S, 0, NV); /* x */
    ssSetOutputPortVectorDimension(S, 1, 1);  /* exitflag */
    ssSetOutputPortVectorDimension(S, 2, 1);  /* iter */
    ssSetOutputPortVectorDimension(S, 3, 1);  /* tsetup */
    ssSetOutputPortVectorDimension(S, 4, 1);  /* tsolve */

    /* Specify the direct feedthrough status */

    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);

    /* One sample time */

    ssSetNumSampleTimes(S, 1);
}

#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if (!ssSetInputPortDimensionInfo(S, port, dimsInfo))
    {
        return;
    }
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo))
    {
        return;
    }
}

#endif

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{

    QP *myQP;
    qp_int m, n;
    qp_int idx;
    qp_int size_P, size_c, size_G, size_h, size_sigma_d;

    real_T *out_x, *out_exitflag, *out_iter, *out_tsetup, *out_tsolve;

    out_x = ssGetOutputPortRealSignal(S, 0);        /* x */
    out_exitflag = ssGetOutputPortRealSignal(S, 1); /* exitflag */
    out_iter = ssGetOutputPortRealSignal(S, 2);     /* iter */
    out_tsetup = ssGetOutputPortRealSignal(S, 3);   /* tsetup */
    out_tsolve = ssGetOutputPortRealSignal(S, 4);   /* tsolve */

    InputRealPtrsType in_P, in_c, in_G, in_h, in_sigma_d;

    in_P = ssGetInputPortRealSignalPtrs(S, 0);
    in_c = ssGetInputPortRealSignalPtrs(S, 1);
    in_G = ssGetInputPortRealSignalPtrs(S, 2);
    in_h = ssGetInputPortRealSignalPtrs(S, 3);

 
    size_P = ssGetInputPortWidth(S, 0); // P
    size_c = ssGetInputPortWidth(S, 1); // c
    size_G = ssGetInputPortWidth(S, 2); // G
    size_h = ssGetInputPortWidth(S, 3); // h

    
    n = (qp_int)size_c;
    m = (qp_int)size_h;


    
    
    
    
    
    
    
    
    myQP = QP_SETUP_dense(n, m, 0, *in_P , NULL, *in_G, *in_c, *in_h, NULL, NULL, COLUMN_MAJOR_ORDERING);
      
    /* ---------------------- Change the following settings as desired ---------------------- */
    /* -------------------------------------------------------------------------------------- */
    /* myQP->settings->maxit   = <desired_value> ;// Maximum number of Iterations of QP       */ 
    /* myQP->settings->reltol  = <desired_value> ;// Relative Tolerance                       */
    /* myQP->settings->abstol  = <desired_value> ;// Absolute Tolerance                       */
    /* myQP->settings->sigma   = <desired_value> ;// sigma desired                            */
    /* myQP->settings->verbose = <desired_value> ;// Verbose Levels || 0 :: No Print          */
                                                  // >0 :: Print Everything */
    /* -------------------------------------------------------------------------------------- */
    /* ---------------------- Change the following settings as desired ---------------------- */
    
    if (myQP != NULL)
    {

        out_exitflag[0] = (real_T)QP_SOLVE(myQP);
        out_iter[0] = (real_T)(myQP->stats->IterationCount);
        for (idx = 0; idx < NV; ++idx)
        {
            out_x[idx] = (real_T)(myQP->x[idx]);
        }
        out_tsetup[0] = (real_T)myQP->stats->tsetup;
        out_tsolve[0] = (real_T)myQP->stats->tsolve;
        QP_CLEANUP_dense(myQP);
    }
    else
    {
        for (idx = 0; idx < NV; ++idx)
        {
            out_x[idx] = (real_T)(0.0);
        }
        out_exitflag[0] = (real_T)(0.0);
        out_iter[0] = (real_T)(0.0);
        out_tsetup[0] = (real_T)(0.0);
        out_tsolve[0] = (real_T)(0.0);
        QP_CLEANUP_dense(myQP);
    }

}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
