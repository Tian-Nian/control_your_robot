#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"
#include "qpSWIFT.h"

#if PY_MAJOR_VERSION >= 3
#define qpLong_check PyLong_Check
#define qp_getlong PyLong_AsLong
#else
#define qpLong_check PyInt_Check
#define qp_getlong PyInt_AsLong
#endif

qp_real *getptr(PyArrayObject *ar)
{
    // qp_real *m = (qp_real *)PyArray_DATA(PyArray_GETCONTIGUOUS(ar));
    //  return m;

    // PyArrayObject *tmp_arr;

    // tmp_arr = PyArray_GETCONTIGUOUS(ar);
    // qp_real *m = (qp_real *)PyArray_DATA(tmp_arr);
    // //Py_DECREF(tmp_arr);
    // return m;

    PyArrayObject *tmp_arr;
    PyArrayObject *new_owner;
    tmp_arr = PyArray_GETCONTIGUOUS(ar);
    new_owner = (PyArrayObject *)PyArray_Cast(tmp_arr, NPY_DOUBLE);
    Py_DECREF(tmp_arr);
    // qp_real *ptr = PyArray_DATA(new_owner);
    //   Py_DECREF(new_owner);
    return new_owner;
}

static PyObject *method_qpSWIFT(PyObject *self, PyObject *args, PyObject *kwargs)
{
    PyArrayObject *c, *h, *b = NULL;
    PyArrayObject *P;
    PyArrayObject *A = NULL;
    PyArrayObject *G;

    PyObject *opts = NULL;
    PyObject *opts_maxiter = NULL, *opts_abstol = NULL, *opts_reltol = NULL, *opts_sigma = NULL, *opts_verbose = NULL, *opts_output = NULL;

    qp_int opts_output_level = 10;

    qp_real *Ppr, *Apr = NULL, *Gpr;
    qp_real *cpr, *hpr, *bpr = NULL;

    npy_intp *dims;

    PyObject *basic_info = NULL, *adv_info = NULL, *result;

    /* Temporary Pointer */
    qp_real *temptr;

    PyArrayObject *ctemp, *htemp, *btemp = NULL;
    PyArrayObject *Ptemp, *Gtemp, *Atemp = NULL;

    /* Results Pointer */
    PyArrayObject *sol_x, *sol_y = NULL, *sol_z = NULL, *sol_s = NULL;
    /* Results Pointer */

    /* Results Dimensions */
    npy_intp sol_xdim[1], sol_ydim[1], sol_zdim[1], sol_sdim[1];
    /* Results Dimensions */

    static char *kwlist[] = {"c", "h", "P", "G", "A", "b", "opts", NULL};

    static char *argparse_string = "O!O!O!O!|O!O!O!";

    if (!PyArg_ParseTupleAndKeywords(args, kwargs, argparse_string, kwlist,
                                     &PyArray_Type, &c,
                                     &PyArray_Type, &h,
                                     &PyArray_Type, &P,
                                     &PyArray_Type, &G,
                                     &PyArray_Type, &A,
                                     &PyArray_Type, &b,
                                     &PyDict_Type, &opts))

    {
        return NULL;
    }

    qp_int n, m, p;
    dims = PyArray_DIMS(c);
    n = (qp_int)dims[0];
    dims = PyArray_DIMS(h);
    m = (qp_int)dims[0];
    if (b && A)
    {
        dims = PyArray_DIMS(b);
        p = (qp_int)dims[0];
    }
    else
    {
        p = 0;
    }

    /*Check Input Data here*/
    /*---- c vector ------*/
    if (!PyArray_ISFLOAT(c) || (qp_int)(PyArray_NDIM(c) != 1))
    {
        PyErr_SetString(PyExc_TypeError, "c must be a floating array with one dimension");
        return NULL;
    }
    /*---- c vector ------*/

    /*---- h vector ------*/
    if (!PyArray_ISFLOAT(h) || (qp_int)(PyArray_NDIM(h) != 1))
    {
        PyErr_SetString(PyExc_TypeError, "h must be a floating array with one dimension");
        return NULL;
    }
    /*---- h vector ------*/

    /*---- b vector and A Matrix ------*/
    if (b && A)
    {
        /*---- b vector ------*/
        if (!PyArray_ISFLOAT(b) || (qp_int)(PyArray_NDIM(b) != 1))
        {
            PyErr_SetString(PyExc_TypeError, "b must be a floating array with one dimension");
            return NULL;
        }
        /*---- b vector ------*/

        /*---- A Matrix ------*/
        if (!PyArray_ISFLOAT(A) || (qp_int)(PyArray_NDIM(A) != 2))
        {
            PyErr_SetString(PyExc_TypeError, "A must be a floating matrix with two dimensions");
            return NULL;
        }

        if (((qp_int)PyArray_DIM(A, 0) != p) || (qp_int)(PyArray_DIM(A, 1) != n))
        {
            PyErr_SetString(PyExc_TypeError, "b and A do not have compatible dimensions");
            return NULL;
        }
        /*---- A Matrix ------*/
    }

    /*---- G Matrix ------*/
    if (!PyArray_ISFLOAT(G) || (qp_int)(PyArray_NDIM(G) != 2))
    {
        PyErr_SetString(PyExc_TypeError, "G must be a floating matrix with two dimensions");
        return NULL;
    }

    if ((qp_int)(PyArray_DIM(G, 0) != m) || (qp_int)(PyArray_DIM(G, 1) != n))
    {
        PyErr_SetString(PyExc_TypeError, "h and G do not have compatible dimensions");
        return NULL;
    }
    /*---- G Matrix ------*/

    /*---- P Matrix ------*/
    if (!PyArray_ISFLOAT(P) || (qp_int)(PyArray_NDIM(P) != 2))
    {
        PyErr_SetString(PyExc_TypeError, "P must be a floating matrix with two dimensions");
        return NULL;
    }

    if ((qp_int)(PyArray_DIM(P, 0) != n) || (qp_int)(PyArray_DIM(P, 1) != n))
    {
        PyErr_SetString(PyExc_TypeError, "c and P do not have compatible dimensions");
        return NULL;
    }
    /*---- P Matrix ------*/

    /*----- options ------*/

    /*---Initialize default options ----*/
    settings inopts;
    inopts.abstol = ABSTOL;
    inopts.reltol = RELTOL;
    inopts.maxit = MAXIT;
    inopts.sigma = SIGMA;
    inopts.verbose = VERBOSE;
    /*---Initialize default options ----*/

    if (opts)
    {
        opts_maxiter = PyDict_GetItemString(opts, "MAXITER");

        if (opts_maxiter)
        {
            Py_INCREF(opts_maxiter);
            if (qpLong_check(opts_maxiter) && qp_getlong(opts_maxiter) < 200 && qp_getlong(opts_maxiter) > 0)
            {
                inopts.maxit = (qp_int)qp_getlong(opts_maxiter);
            }
            else
            {
                PyErr_SetString(PyExc_TypeError, "max iterations must be between 0 and 200, using the default value of 100");
                return NULL;
            }
            Py_DECREF(opts_maxiter);
        }

        opts_abstol = PyDict_GetItemString(opts, "ABSTOL");

        if (opts_abstol)
        {
            Py_INCREF(opts_abstol);
            if (PyFloat_Check(opts_abstol) && PyFloat_AsDouble(opts_abstol) < 1.0 && PyFloat_AsDouble(opts_abstol) > 0.0)
            {
                inopts.abstol = (qp_real)PyFloat_AsDouble(opts_abstol);
            }
            else
            {
                PyErr_SetString(PyExc_TypeError, "absolute tolerance must be between 0 and 1, using the default value");
                return NULL;
            }
            Py_DECREF(opts_abstol);
        }

        opts_reltol = PyDict_GetItemString(opts, "RELTOL");

        if (opts_reltol)
        {
            Py_INCREF(opts_reltol);
            if (PyFloat_Check(opts_reltol) && PyFloat_AsDouble(opts_reltol) < 1.0 && PyFloat_AsDouble(opts_reltol) > 0.0)
            {
                inopts.reltol = (qp_real)PyFloat_AsDouble(opts_reltol);
            }
            else
            {
                PyErr_SetString(PyExc_TypeError, "realtive tolerance must be between 0 and 1, using the default value");
                return NULL;
            }
            Py_DECREF(opts_reltol);
        }

        opts_sigma = PyDict_GetItemString(opts, "SIGMA");

        if (opts_sigma)
        {
            Py_INCREF(opts_sigma);
            if (PyFloat_Check(opts_sigma) && PyFloat_AsDouble(opts_sigma) > 0.0)
            {
                inopts.sigma = (qp_real)PyFloat_AsDouble(opts_sigma);
            }
            else
            {
                PyErr_SetString(PyExc_TypeError, "sigma must be positive, using the default value");
                return NULL;
            }
            Py_DECREF(opts_sigma);
        }

        opts_verbose = PyDict_GetItemString(opts, "VERBOSE");

        if (opts_verbose)
        {
            Py_INCREF(opts_verbose);
            if (qpLong_check(opts_verbose) && qp_getlong(opts_verbose) >= 0)
            {
                inopts.verbose = (qp_int)qp_getlong(opts_verbose);
            }
            else
            {
                //  PyErr_WarnEx(PyExc_UserWarning, "verbose must be non-negative, using the default value", 1);
                PyErr_SetString(PyExc_TypeError, "verbose must be non-negative integer, using the default value");
                return NULL;
            }
            Py_DECREF(opts_verbose);
        }

        opts_output = PyDict_GetItemString(opts, "OUTPUT");

        if (opts_output)
        {
            Py_INCREF(opts_output);
            if (qpLong_check(opts_output) && qp_getlong(opts_output) >= 0)
            {
                opts_output_level = (qp_int)qp_getlong(opts_output);
            }
            else
            {
                PyErr_WarnEx(PyExc_UserWarning, "output must be non-negative, using the default value", 1);
                // PyErr_SetString(PyExc_TypeError, "verbose must be non-negative integer, using the default value");
                // return NULL;
            }
            Py_DECREF(opts_output);
        }
    }

    /*----- options ------*/

    /*Check Input Data here*/

    /*** Get Data Pointers ***/
    ctemp = getptr(c);
    cpr = (qp_real *)PyArray_DATA(ctemp);

    htemp = getptr(h);
    hpr = (qp_real *)PyArray_DATA(htemp);
    if (b && A)
    {
        btemp = getptr(b);
        bpr = (qp_real *)PyArray_DATA(btemp);
        Atemp = getptr(A);
        Apr = (qp_real *)PyArray_DATA(Atemp);
    }
    Ptemp = getptr(P);
    Ppr = (qp_real *)PyArray_DATA(Ptemp);
    Gtemp = getptr(G);
    Gpr = (qp_real *)PyArray_DATA(Gtemp);
    /*** Get Data Pointers ***/

    QP *myQP;

    myQP = QP_SETUP_dense(n, m, p, Ppr, Apr, Gpr, cpr, hpr, bpr, NULL, ROW_MAJOR_ORDERING);

    /*---- Copy Settings if any---*/
    myQP->options->abstol = inopts.abstol;
    myQP->options->reltol = inopts.reltol;
    myQP->options->maxit = inopts.maxit;
    myQP->options->sigma = inopts.sigma;
    myQP->options->verbose = inopts.verbose;

    /*---- Copy Settings if any---*/

    if (myQP->options->verbose > 0)
    {
        PRINT("\n****qpSWIFT : Sparse Quadratic Programming Solver****\n\n");
        PRINT("================Settings Applied======================\n");
        PRINT("Maximum Iterations : %ld \n", myQP->options->maxit);
        PRINT("ABSTOL             : %e \n", myQP->options->abstol);
        PRINT("RELTOL             : %e \n", myQP->options->reltol);
        PRINT("SIGMA              : %e \n", myQP->options->sigma);
        PRINT("VERBOSE            : %ld \n", myQP->options->verbose);
        PRINT("Permutation vector : AMD Solver\n\n");
    }

    qp_int ExitCode;
    Py_BEGIN_ALLOW_THREADS;
    ExitCode = QP_SOLVE(myQP);
    Py_END_ALLOW_THREADS;

    sol_xdim[0] = myQP->n;
    sol_x = (PyArrayObject *)PyArray_SimpleNew(1, sol_xdim, NPY_DOUBLE);
    temptr = PyArray_DATA(sol_x);
    for (qp_int i = 0; i < myQP->n; ++i)
    {
        temptr[i] = myQP->x[i];
    }

    switch (opts_output_level)
    {
    case 1:
        basic_info = Py_BuildValue("{s:l,s:l,s:d,s:d}",
                                   "ExitFlag", ExitCode,
                                   "Iterations", myQP->stats->IterationCount,
                                   "Setup_Time", myQP->stats->tsetup,
                                   "Solve_Time", myQP->stats->tsolve);

        result = Py_BuildValue("{s:O,s:O}",
                               "sol", sol_x,
                               "basicInfo", basic_info);

        Py_DECREF(basic_info);
        break;
    case 2:
        basic_info = Py_BuildValue("{s:l,s:l,s:d,s:d}",
                                   "ExitFlag", ExitCode,
                                   "Iterations", myQP->stats->IterationCount,
                                   "Setup_Time", myQP->stats->tsetup,
                                   "Solve_Time", myQP->stats->tsolve);

        if (b && A)
        {
            sol_ydim[0] = myQP->p;
            sol_y = (PyArrayObject *)PyArray_SimpleNew(1, sol_ydim, NPY_DOUBLE);
            temptr = PyArray_DATA(sol_y);
            for (qp_int i = 0; i < myQP->p; ++i)
            {
                temptr[i] = myQP->y[i];
            }
        }

        sol_zdim[0] = myQP->m;
        sol_z = (PyArrayObject *)PyArray_SimpleNew(1, sol_zdim, NPY_DOUBLE);
        temptr = PyArray_DATA(sol_z);
        for (qp_int i = 0; i < myQP->m; ++i)
        {
            temptr[i] = myQP->z[i];
        }

        sol_sdim[0] = myQP->m;
        sol_s = (PyArrayObject *)PyArray_SimpleNew(1, sol_sdim, NPY_DOUBLE);
        temptr = PyArray_DATA(sol_s);
        for (qp_int i = 0; i < myQP->m; ++i)
        {
            temptr[i] = myQP->s[i];
        }

        if (b && A)
        {
            adv_info = Py_BuildValue("{s:d,s:d,s:d,s:O,s:O,s:O}",
                                     "fval", myQP->stats->fval,
                                     "kktTime", myQP->stats->kkt_time,
                                     "ldlTime", myQP->stats->ldl_numeric,
                                     "y", sol_y,
                                     "z", sol_z,
                                     "s", sol_s);
        }
        else
        {
            adv_info = Py_BuildValue("{s:d,s:d,s:d,s:O,s:O}",
                                     "fval", myQP->stats->fval,
                                     "kktTime", myQP->stats->kkt_time,
                                     "ldlTime", myQP->stats->ldl_numeric,
                                     "z", sol_z,
                                     "s", sol_s);
        }
        result = Py_BuildValue("{s:O,s:O,s:O}",
                               "sol", sol_x,
                               "basicInfo", basic_info,
                               "advInfo", adv_info);

        Py_DECREF(basic_info);

        Py_DECREF(adv_info);

        if (sol_y)
        {
            Py_DECREF(sol_y);
        }
        Py_DECREF(sol_z);
        Py_DECREF(sol_s);

        break;
    default:
        result = Py_BuildValue("{s:O}",
                               "sol", sol_x);

        break;
    }

    Py_DECREF(sol_x);

    Py_DECREF(Ptemp);
    Py_DECREF(ctemp);
    Py_DECREF(Gtemp);
    Py_DECREF(htemp);
    if (Atemp)
    {
        Py_DECREF(Atemp);
    }

    if (btemp)
    {
        Py_DECREF(btemp);
    }

    QP_CLEANUP_dense(myQP);

    return result;
}

static PyMethodDef qpSWIFTMethods[] = {
    {"run", method_qpSWIFT, METH_VARARGS | METH_KEYWORDS, "res = qpSWIFT.run(c,h,P,G,A,b,opts) \n"
                                                          "Please refer to the Python Documentaion or qpSWIFT_help.py file \n"},
    {NULL, 0, NULL, NULL}};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef qpSWIFTModule = {
    PyModuleDef_HEAD_INIT,
    "qpSWIFT",
    "A Sparse Quadratic Programming Solver",
    -1,
    qpSWIFTMethods};
#endif

#if PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC PyInit_qpSWIFT(void)
{
    import_array();
    return PyModule_Create(&qpSWIFTModule);
}
#else
PyMODINIT_FUNC initqpSWIFT(void)
{
    import_array();
    Py_InitModule("qpSWIFT", qpSWIFTMethods);
}
#endif
