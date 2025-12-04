#include "test.hpp"
#define GTEST_COUT std::cerr << "[          ] [ INFO ]"

TEST(standAloneTests, func_SparseMatrixMultiply)
{
    smat A;
    SparseMatrixSetup(A_m, A_n, A_nnz, A_jc, A_ir, A_pr, &A);

    double x[2], y[2];

    x[0] = 1;
    x[1] = 1;

    SparseMatrixMultiply(&A, x, y, 1);

    ASSERT_EQ(-3, y[0]);
    ASSERT_EQ(-7, y[1]);

    y[0] = -y[0];
    y[1] = -y[1];

    SparseMatrixMultiply(&A, x, y, 0);

    ASSERT_EQ(0, y[0]);
    ASSERT_EQ(0, y[1]);
}

TEST(standAloneTests, func_SparseMatrixTransMultiply)
{
    smat A;
    SparseMatrixSetup(A_m, A_n, A_nnz, A_jc, A_ir, A_pr, &A);

    double x[2], y[2];

    x[0] = 1;
    x[1] = 1;

    SparseMatrixTransMultiply(&A, x, y, 1);

    ASSERT_EQ(-4, y[0]);
    ASSERT_EQ(-6, y[1]);

    y[0] = -y[0];
    y[1] = -y[1];

    SparseMatrixTransMultiply(&A, x, y, 0);

    ASSERT_EQ(0, y[0]);
    ASSERT_EQ(0, y[1]);
}

TEST(standAloneTests, func_obj_value)
{
    smat P;
    SparseMatrixSetup(P_m, P_n, P_nnz, P_jc, P_ir, P_pr, &P);

    qp_real c[2] = {1.0, 1.0};

    qp_real temp[2];

    qp_real x[2] = {1.0, 1.0};

    qp_real val = obj_value(&P, c, x, temp);
    x[0] = -1.0;
    x[1] = -1.0;

    qp_real val1 = obj_value(&P, c, x, temp);

    ASSERT_EQ(3.0, val);
    ASSERT_EQ(-1.0, val1);
}

TEST(standAloneTests, func_densetosparse)
{
    char diag_msg[50];

    /* Sparse Matrices; square and rectangular */
    smat sq, rec;

    /* Square Matrix Pointer */
    qp_int sq_m = 3, sq_n = 3;
    qp_real sq_pr[9] = {1.0, 0.0, 2.0, 3.0, 0.0, 1.1, 0.1, 0.0, 3.0};

    /* Rectangular Matrix Pointer */
    qp_int rec_m = 3, rec_n = 1;
    qp_real rec_pr[3] = {0.0, 1.0, 0.0};

    /*  Perform computations */
    densetosparse(sq_m, sq_n, sq_pr, &sq);
    densetosparse(rec_m, rec_n, rec_pr, &rec);

    /* Validate the Matrix */
    ASSERT_TRUE(validate_smat(&sq, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat(&rec, diag_msg)) << diag_msg;
    //std::cout << msg << std::endl;

    /* Data Integrity Checks */
    ASSERT_TRUE(compare_dmat_C_smat(&sq, sq_pr, sq_m, sq_n));
    ASSERT_TRUE(compare_dmat_C_smat(&rec, rec_pr, rec_m, rec_n));
}

TEST(standAloneTests, func_densetosparse_ROWMAJOR)
{
    char diag_msg[50];

    /* Sparse Matrices; square and rectangular */
    smat sq, rec;

    /* Square Matrix Pointer */
    qp_int sq_m = 3, sq_n = 3;
    qp_real sq_pr[9] = {1.0, 0.0, 2.0, 3.0, 0.0, 1.1, 0.1, 0.0, 3.0};

    /* Rectangular Matrix Pointer */
    qp_int rec_m = 3, rec_n = 1;
    qp_real rec_pr[3] = {0.0, 1.0, 0.0};

    /*  Perform computations */
    densetosparse_ROWMAJOR(sq_m, sq_n, sq_pr, &sq);
    densetosparse_ROWMAJOR(rec_m, rec_n, rec_pr, &rec);

    /* Validate the Matrix */
    ASSERT_TRUE(validate_smat(&sq, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat(&rec, diag_msg)) << diag_msg;
    //std::cout << msg << std::endl;

    /* Data Integrity Checks */
    ASSERT_TRUE(compare_dmat_R_smat(&sq, sq_pr, sq_m, sq_n));
    ASSERT_TRUE(compare_dmat_R_smat(&rec, rec_pr, rec_m, rec_n));
}

TEST(standAloneTests, func_SparseMatrixTranspose)
{
    char diag_msg[50];

    /* Square Matrix Test */
    smat A;
    SparseMatrixSetup(A_m, A_n, A_nnz, A_jc, A_ir, A_pr, &A);
    smat At;
    qp_int Atjc[3], Atir[4];
    qp_real Atpr[4];
    SparseMatrixSetup(A_n, A_m, A_nnz, Atjc, Atir, Atpr, &At);

    SparseMatrixTranspose(&A, &At);
    ASSERT_TRUE(validate_smat(&At, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat_smatT(&A, &At, diag_msg)) << diag_msg;

    /* Square Matrix Test */

    /* Rectangular Matrix Test*/

    qp_int rec_n = 3;
    qp_int rec_m = 2;
    qp_int rec_nnz = 4;
    smat rec, recT;
    qp_int rec_ir[4] = {0, 0, 1, 1}, recT_ir[4], rec_jc[4] = {0, 1, 3, 4}, recT_jc[3];
    qp_real rec_pr[4] = {1.0, 2.0, 3.0, 4.0}, recT_pr[4];

    SparseMatrixSetup(rec_m, rec_n, rec_nnz, rec_jc, rec_ir, rec_pr, &rec);
    SparseMatrixSetup(rec_n, rec_m, rec_nnz, recT_jc, recT_ir, recT_pr, &recT);
    SparseMatrixTranspose(&rec, &recT);
    ASSERT_TRUE(validate_smat(&recT, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat_smatT(&rec, &recT, diag_msg)) << diag_msg;

    /* Rectangular Matrix Test*/
}

TEST(standAloneTests, func_SparseMatrixSetup)
{
    smat A, P;
    char diag_msg[50];
    SparseMatrixSetup(A_m, A_n, A_nnz, A_jc, A_ir, A_pr, &A);
    SparseMatrixSetup(P_m, P_n, P_nnz, P_jc, P_ir, P_pr, &P);

    smat rec;
    qp_int rec_n = 3;
    qp_int rec_m = 2;
    qp_int rec_nnz = 4;
    qp_int rec_ir[4] = {0, 0, 1, 1}, rec_jc[4] = {0, 1, 3, 4};
    qp_real rec_pr[4] = {1.0, 2.0, 3.0, 4.0};

    SparseMatrixSetup(rec_m, rec_n, rec_nnz, rec_jc, rec_ir, rec_pr, &rec);

    ASSERT_TRUE(validate_smat(&A, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat(&rec, diag_msg)) << diag_msg;
    ASSERT_TRUE(validate_smat(&P, diag_msg)) << diag_msg;
}

// TEST(standAloneTests, func_formkktmatrix_full)
// {
//     smat P, G;
//     char diag_msg[50];
//     /* P and G Matrices only */
//     SparseMatrixSetup(P_m, P_n, P_nnz, P_jc, P_ir, P_pr, &P);
//     SparseMatrixSetup(G_m, G_n, G_nnz, G_jc, G_ir, G_pr, &G);
//     smat kkt;
//     qp_int kkt_m = P_n + G_m;
//     qp_int kkt_n = P_n + G_m;
//     qp_int kkt_nnz = P_nnz + 2 * G_nnz + G_m;

//     qp_int *kkt_ir, *kkt_jc;
//     qp_real *kkt_pr;

//     kkt_ir = (qp_int *)MALLOC(kkt_nnz * sizeof(qp_int));
//     kkt_jc = (qp_int *)MALLOC((kkt_n + 1) * sizeof(qp_int));
//     kkt_pr = (qp_real *)MALLOC(kkt_nnz * sizeof(qp_real));

//     smat Gt;
//     qp_int Gtjc[4], Gtir[5];
//     qp_real Gtpr[5];
//     SparseMatrixSetup(G_n, G_m, G_nnz, Gtjc, Gtir, Gtpr, &Gt);
//     SparseMatrixTranspose(&G, &Gt);
//     SparseMatrixSetup(kkt_m, kkt_n, kkt_nnz, kkt_ir, kkt_jc, kkt_pr, &kkt);
//     formkktmatrix_full(&P, &G, NULL, &Gt, NULL, &kkt);
//     print_smat(&kkt);

//     ASSERT_TRUE(validate_smat(&kkt, diag_msg)) << diag_msg;
//     ASSERT_TRUE(validate_kktmatrix(&kkt, &P, NULL, &G, diag_msg)) << diag_msg;

//     FREE(kkt_ir);
//     FREE(kkt_jc);
//     FREE(kkt_pr);
// }

TEST(standAloneTests, func_formkktmatrix_fullS)
{
    // smat P, A, G;
    // char diag_msg[50];
    // /* P and G Matrices only */
    // SparseMatrixSetup(P_m, P_n, P_nnz, P_jc, P_ir, P_pr, &P);
    // SparseMatrixSetup(A_m, A_n, A_nnz, A_jc, A_ir, A_pr, &A);
    // SparseMatrixSetup(G_m, G_n, G_nnz, G_jc, G_ir, G_pr, &G);
    // smat kkt;
    // qp_int kkt_m = P_n + A_m + G_m;
    // qp_int kkt_n = P_n + A_m + G_m;
    // qp_int kkt_nnz = P_nnz + 2 * A_nnz + 2 * G_nnz + G_m;

    // qp_int *kkt_ir, *kkt_jc;
    // qp_real *kkt_pr;

    // kkt_ir = (qp_int *)MALLOC(kkt_nnz * sizeof(qp_int));
    // kkt_jc = (qp_int *)MALLOC((kkt_n + 1) * sizeof(qp_int));
    // kkt_pr = (qp_real *)MALLOC(kkt_nnz * sizeof(qp_real));

    // smat At;
    // qp_int Atjc[3], Atir[4];
    // qp_real Atpr[4];
    // SparseMatrixSetup(A_n, A_m, A_nnz, Atjc, Atir, Atpr, &At);
    // SparseMatrixTranspose(&A, &At);

    // smat Gt;
    // qp_int Gtjc[4], Gtir[5];
    // qp_real Gtpr[5];
    // SparseMatrixSetup(G_n, G_m, G_nnz, Gtjc, Gtir, Gtpr, &Gt);
    // SparseMatrixTranspose(&G, &Gt);

    // SparseMatrixSetup(kkt_m, kkt_n, kkt_nnz, kkt_ir, kkt_jc, kkt_pr, &kkt);
    // formkktmatrix_full(&P, &G, &A, &Gt, &At, &kkt);
    // ASSERT_TRUE(validate_smat(&kkt, diag_msg)) << diag_msg;

    // //ASSERT_TRUE(validate_kktmatrix(&kkt, &P, &A, &G, diag_msg)) << diag_msg;

    // FREE(kkt_ir);
    // FREE(kkt_jc);
    // FREE(kkt_pr);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return ::testing::UnitTest::GetInstance()->Run();
    return RUN_ALL_TESTS();
}