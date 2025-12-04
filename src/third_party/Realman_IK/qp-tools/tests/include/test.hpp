#ifndef __TEST_H
#define __TEST_H

#include <gtest/gtest.h>
#include "qpSWIFT.h"

qp_int A_m = 2;
qp_int A_n = 2;
qp_int A_nnz = 4;
qp_int A_jc[3] = {0, 2, 4};
qp_int A_ir[4] = {0, 1, 0, 1};
qp_real A_pr[4] = {1.0, 3.0, 2.0, 4.0};

qp_int P_m = 2;
qp_int P_n = 2;
qp_int P_nnz = 2;
qp_int P_ir[2] = {0, 1};
qp_int P_jc[3] = {0, 1, 2};
qp_real P_pr[2] = {1.0, 1.0};

qp_int G_m = 3;
qp_int G_n = 2;
qp_int G_nnz = 5;
qp_int G_ir[5] = {0, 1, 2, 1, 2};
qp_int G_jc[3] = {0, 3, 5};
qp_real G_pr[5] = {1.0, 2.0, 3.0, 1.0, 4.0};

// qp_real c[];
// qp_real b[];
// qp_real h[];

class standAloneTests : public ::testing::Test
{
private:
public:
    char diag_msg[50];

    smat P, A, G;
    smat Pt, At, Gt;

    static int margc;
    static char *margv[];

    //void copy_data(standalone_data *new_data) { data = new_data; };

protected:
    // void static init(standalone_data *new_data) { data = new_data; };
    standAloneTests(/* args */);
    ~standAloneTests();

    //  standalone_data *data;
    // void SetUp()
    // {
    //     parser psr;

    //     if (!psr.readfile("some.txt"))
    //     {
    //         printf("Data cannot be read\n");
    //         // return 0;
    //     }
    //     data = psr.getdateptr();
    //     SparseMatrixSetup(data->n, data->n, data->P_nnz, data->Pjc, data->Pir, data->Ppr, &P);
    //     SparseMatrixSetup(data->p, data->n, data->A_nnz, data->Ajc, data->Air, data->Apr, &A);
    //     SparseMatrixSetup(data->m, data->n, data->G_nnz, data->Gjc, data->Gir, data->Gpr, &G);
    // };

    // void TearDown(){
    //     // add teardown code here
    // };
};

bool validate_smat(smat *A, char *msg)
{
    bool result = false;

    /* Size Check */
    if (A->nnz > (A->m * A->n) || A->nnz <= 0 || A->m <= 0 || A->n <= 0)
    {
        strcpy(msg, "Size Check failed\n");
        return result;
    }

    /* For the ir row pointer*/
    for (qp_int i = 0; i < A->nnz; ++i)
    {
        if (A->ir[i] < 0 || A->ir[i] >= A->m)
        {
            strcpy(msg, "Data integrity failed in ir\n");
            return result;
        }
    }

    /* For the jc column pointer*/
    for (qp_int i = 0; i < A->n; ++i)
    {
        if (A->jc[i + 1] < A->jc[i] || A->jc[i] > A->nnz)
        {
            strcpy(msg, "Data integrity failed in jc\n");
            return result;
        }
    }

    if (A->jc[A->n] != A->nnz)
    {
        strcpy(msg, "Data integrity failed in jc;jc[n]~=nnz\n");
        return result;
    }
    /* For the data pointer */
    qp_real temp;
    for (qp_int i = 0; i < A->nnz; ++i)
    {
        temp = A->pr[i];
        if (!std::isfinite(temp) || std::isnan(temp) || std::isinf(temp))
        {
            strcpy(msg, "Data integrity failed in pr\n");
            return result;
        }
    }
    /* All tests passed */

    result = true;
    return result;
}

bool compare_dmat_C_smat(smat *A, qp_real *pr, qp_int m, qp_int n)
{
    bool result = false;
    for (qp_int j = 0, idx = 0; j < n; ++j)
    {
        for (qp_int i = 0; i < m; ++i)
        {
            if (pr[j * m + i] != 0.0)
            {
                if (pr[j * m + i] != A->pr[idx++])
                {
                    return result;
                }
            }
        }
    }

    result = true;
    return result;
}

bool compare_dmat_R_smat(smat *A, qp_real *pr, qp_int m, qp_int n)
{
    bool result = false;
    for (qp_int j = 0, idx = 0; j < n; ++j)
    {
        for (qp_int i = 0; i < m; ++i)
        {
            if (pr[i * n + j] != 0.0)
            {
                if (pr[i * n + j] != A->pr[idx++])
                {
                    return result;
                }
            }
        }
    }

    result = true;
    return result;
}

bool validate_smat_smatT(smat *A, smat *At, char *msg)
{
    bool result = false;

    /* Workspace Variable*/
    qp_int *count = (qp_int *)MALLOC(At->m * sizeof(qp_int));
    for (qp_int i = 0; i < At->m; ++i)
    {
        count[i] = 0;
    }
    /* Dimensions Check */

    if (A->nnz != At->nnz)
    {
        strcpy(msg, "Number of NonZeros do not match\n");
        FREE(count);
        return result;
    }

    if (A->m != At->n || A->n != At->m)
    {
        strcpy(msg, "Number of rows and cols do not match\n");
        FREE(count);
        return result;
    }

    /* jc pointer check */
    for (qp_int i = 0, k; i < At->n; ++i)
    {
        for (qp_int j = At->jc[i]; j < At->jc[i + 1]; ++j)
        {
            k = At->ir[j];
            count[k]++;
        }
    }

    for (qp_int i = 0; i < A->n; ++i)
    {
        if (A->jc[i + 1] - A->jc[i] != count[i])
        {
            FREE(count);
            strcpy(msg, "Data integrity failed in jc pointer \n");
            return result;
        }
    }

    for (qp_int j = 0; j < At->m; j++)
    {
        count[j] = 0;
    }

    /* ir and pr check */
    for (qp_int i = 0, k, idx; i < At->n; ++i)
    {
        for (qp_int j = At->jc[i]; j < At->jc[i + 1]; ++j)
        {
            k = At->ir[j];
            idx = A->jc[k] + count[k];
            if (A->ir[idx] != i)
            {
                FREE(count);
                strcpy(msg, "Data integrity failed in ir pointer\n");
                return result;
            }
            if (A->pr[idx] != At->pr[j])
            {
                FREE(count);
                strcpy(msg, "Data integrity failed in pr pointer\n");
                return result;
            }
            count[k]++;
        }
    }

    result = true;
    FREE(count);
    return result;
}

bool check_vector(qp_int a_start, qp_int a_end, qp_real *a, qp_int b_start, qp_int b_end, qp_real *b, char *msg)
{
    bool result = false;
    qp_int a_length = a_end - a_start;
    qp_int b_length = b_end - b_start;

    if (a_length != b_length)
    {
        strcpy(msg, "Length of vectors compared are not equal");
    }

    qp_int idx_a, idx_b;

    for (idx_a = a_start, idx_b = b_start; idx_a < a_end; idx_a++, idx_b++)
    {
        if (a[idx_a] != b[idx_b])
        {
            strcpy(msg, "Values of the vector are not the same");
            return result;
        }
    }

    result = true;
    return result;
}

bool check_updatekktmatrix_b(qp_real *b, qp_real *rx, qp_real *ry, qp_real *rz, qp_real *ds, qp_real *z, qp_int n, qp_int m, qp_int p, char *msg)
{
    bool result = false;

    if (check_vector(0, n, b, 0, n, rx, msg))
    {
        strcpy(msg, "rx vector values and b vector values are not comparable");
        return result;
    }

    if (!check_vector(n, n + p, b, 0, p, ry, msg))
    {
        strcpy(msg, "ry vector values and b vector values are not comparable");
        return result;
    }

    qp_real *temp;
    temp = (qp_real *)MALLOC(m * sizeof(qp_real));

    qp_int idx;
    for (idx = 0; idx = m; idx++)
    {
        temp[idx] = rz[idx] - (ds[idx] / z[idx]);
    }

    if (check_vector(n + p, n + m + p, b, 0, m, temp, msg))
    {
        FREE(temp);
        strcpy(msg, "rz vectors values and b values are not comparable");
        return result;
    }

    FREE(temp);
    result = true;

    return result;
}

bool check_updatekktmatrix(qp_int m, smat *kkt, qp_real *s, qp_real *z)
{
    // bool result = false;
    // qp_int idx, kkt_idx;

    // for (idx = 0; idx < m; idx++)
    // {
    //     if (kkt->pr[index = kkt->jc[i + 1] - 1;] != s[idx] / z[idx])
    //     {
    //     }
    // }
    bool result = false;
    qp_int indicator;
    // updatekktmatrix(kkt, s, z, delta_s, delta_z, alpha_p, alpha_D, m, n, p, indicator);
    return result;
}

bool check_matrix_symmetric(smat *kkt, char *diag_msg)
{
    bool result = false;
    if (kkt->m != kkt->n)
    {
        strcpy(diag_msg, "matrix dimensions are not symmetric");
        return result;
    }

    qp_int j_tr;
    qp_int *row_ptr;
    row_ptr = (qp_int *)MALLOC(kkt->m * sizeof(qp_int));

    for (qp_int i = 0; i < kkt->m; ++i)
    {
        row_ptr[i] = 0.0;
    }

    for (qp_int i = 0; i < kkt->n; ++i)
    {
        for (qp_int j = kkt->jc[i]; j < kkt->jc[i + 1]; ++j)
        {
            j_tr = kkt->jc[kkt->ir[j]] + (row_ptr[kkt->ir[j]]++);
            std::cout << kkt->pr[j] << "\t" << kkt->pr[j_tr] << "\n";
            if (kkt->pr[j] != kkt->pr[j_tr])
            {
                strcpy(diag_msg, "matrix and matrix transpose values do not match");
                FREE(row_ptr);
                return result;
            }
        }
    }

    result = true;
    FREE(row_ptr);
    return result;
}

bool validate_kktmatrix(smat *kkt, smat *P, smat *A, smat *G, char *diag_msg)
{
    bool result = false;
    if (A)
    {
        if (!check_matrix_symmetric(kkt, diag_msg))
        {
            return result;
        }
        for (qp_int i = 0, kkt_nnz = 0; i < P->n; i++)
        {
            for (qp_int j = P->jc[i]; j < P->jc[i + 1]; j++)
            {

                if (kkt->pr[kkt_nnz] != P->pr[j])
                {
                    strcpy(diag_msg, "kkt matrix and P matrix do not match");
                    return result;
                }
                kkt_nnz++;
            }

            for (qp_int j = A->jc[i]; j < A->jc[i + 1]; j++)
            {
                if (kkt->pr[kkt_nnz] != A->pr[j])
                {
                    strcpy(diag_msg, "kkt matrix and A matrix do not match");
                    return result;
                }
                kkt_nnz++;
            }

            for (qp_int j = G->jc[i]; j < G->jc[i + 1]; j++)
            {
                if (kkt->pr[kkt_nnz] != G->pr[j])
                {
                    strcpy(diag_msg, "kkt matrix and G matrix do not match");
                    return result;
                }
                kkt_nnz++;
            }
        }
    }
    else
    {
        if (!check_matrix_symmetric(kkt, diag_msg))
        {
            return result;
        }

        for (qp_int i = 0, kkt_nnz = 0; i < P->n; i++)
        {
            for (qp_int j = P->jc[i]; j < P->jc[i + 1]; j++)
            {

                if (kkt->pr[kkt_nnz] != P->pr[j])
                {
                    strcpy(diag_msg, "kkt matrix and P matrix do not match");
                    return result;
                }
                kkt_nnz++;
            }

            for (qp_int j = G->jc[i]; j < G->jc[i + 1]; j++)
            {
                if (kkt->pr[kkt_nnz] != G->pr[j])
                {
                    strcpy(diag_msg, "kkt matrix and G matrix do not match");
                    return result;
                }
                kkt_nnz++;
            }
        }
    }

    result = true;
    return true;
}

void print_smat(smat *A)
{
    printf("jc = {");
    for (qp_int i = 0; i <= A->n; ++i)
    {
        printf("%ld,", A->jc[i]);
    }
    printf("}\n");

    printf("ir = {");
    for (qp_int i = 0; i < A->n; ++i)
    {
        for (qp_int j = A->jc[i]; j < A->jc[i + 1]; ++j)
        {
            printf("%ld,", A->ir[j]);
        }
    }
    printf("}\n");

    printf("pr = {");
    for (qp_int i = 0; i < A->n; ++i)
    {
        for (qp_int j = A->jc[i]; j < A->jc[i + 1]; ++j)
        {
            printf("%lf,", A->pr[j]);
        }
    }
    printf("}\n");
}

#endif