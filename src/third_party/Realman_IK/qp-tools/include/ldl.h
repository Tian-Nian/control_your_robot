/* ========================================================================== */
/* === ldl.h:  include file for the LDL package ============================= */
/* ========================================================================== */

/* Copyright (c) Timothy A Davis, http://www.suitesparse.com.
* All Rights Reserved.  See LDL/Doc/License.txt for the License.
*
* Modified by Abhishek Pandala
*	Changes Made : 1) Removed int and long versions of the code and created a single
*                   unified version
*								 2) Added two more functions LDL_cache_numeric and LDL_row_cache_numeric
*/

#ifndef __LDL_H__
#define __LDL_H__

#include "GlobalOptions.h"

void LDL_symbolic(qp_int n, qp_int Ap[],
	qp_int Ai[], qp_int Lp[],
	qp_int Parent[], qp_int Lnz[],
	qp_int Flag[], qp_int P[],
	qp_int Pinv[]);

qp_int LDL_numeric(qp_int n, qp_int Ap[],
	qp_int Ai[], qp_real Ax[], qp_int Lp[],
	qp_int Parent[], qp_int Lnz[],
	qp_int Li[], qp_real Lx[], qp_real D[], qp_real Y[],
	qp_int Pattern[], qp_int Flag[],
	qp_int P[], qp_int Pinv[]);

qp_int LDL_cache_numeric(qp_int n, qp_int Ap[],
	qp_int Ai[], qp_real Ax[], qp_int Ltp[], qp_int Lti[],
	qp_int Li[], qp_int Lp[], qp_real Lx[], qp_real D[], qp_real Y[],
	qp_int P[], qp_int Pinv[], qp_int UPattern[]);


qp_int LDL_row_cache_numeric(qp_int n, qp_int Ap[],
	qp_int Ai[], qp_real Ax[], qp_int Ltp[], qp_int Lti[],
	qp_int Li[], qp_int Lp[], qp_real Lx[], qp_real D[], qp_real Y[],
	qp_int P[], qp_int Pinv[], qp_int UPattern[]);

void LDL_lsolve(qp_int n, qp_real X[], qp_int Lp[],
	qp_int Li[], qp_real Lx[]);

void LDL_dsolve(qp_int n, qp_real X[], qp_real D[]);

void LDL_ltsolve(qp_int n, qp_real X[], qp_int Lp[],
	qp_int Li[], qp_real Lx[]);

void LDL_perm(qp_int n, qp_real X[], qp_real B[],
	qp_int P[]);
void LDL_permt(qp_int n, qp_real X[], qp_real B[],
	qp_int P[]);

qp_int LDL_valid_perm(qp_int n, qp_int P[],
	qp_int Flag[]);
qp_int LDL_valid_matrix(qp_int n,
	qp_int Ap[], qp_int Ai[]);

/* ========================================================================== */
/* === LDL version ========================================================== */
/* ========================================================================== */

#define LDL_DATE "May 4, 2016"
#define LDL_VERSION_CODE(main,sub) ((main) * 1000 + (sub))
#define LDL_MAIN_VERSION 2
#define LDL_SUB_VERSION 2
#define LDL_SUBSUB_VERSION 6
#define LDL_VERSION LDL_VERSION_CODE(LDL_MAIN_VERSION,LDL_SUB_VERSION)

#endif
