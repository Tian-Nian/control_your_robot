#include "test.hpp"

TEST(standAloneTesting, func_innerproduct)
{
    qp_real x[3] = {1.0, 2.0, 3.0};
    qp_real y[3] = {-1.0, -2.0, -3.0};
    ASSERT_NEAR(14.0, innerproduct(x, x, 3), 1e-10);
    ASSERT_NEAR(-14.0, innerproduct(x, y, 3), 1e-10);
    ASSERT_NEAR(-14.0, innerproduct(y, x, 3), 1e-10);
    ASSERT_NEAR(14.0, innerproduct(y, y, 3), 1e-10);
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
    ASSERT_NEAR(0.0, innerproduct(y, y, 3), 1e-10);
}

TEST(standAloneTesting, func_updatevariables)
{
    qp_real x[4] = {1.0, 2.0, 3.0, 4.0};
    qp_real delta_x[4] = {-1.0, -2.0, -3.0, -4.0};

    updatevariables(x, delta_x, 1, 4);
    ASSERT_NEAR(0, innerproduct(x, delta_x, 3), 1e-10);

    updatevariables(x, delta_x, 1, 4);
    ASSERT_NEAR(14, innerproduct(x, delta_x, 3), 1e-10);
}

TEST(standAloneTesting, func_formrho)
{
    qp_real s[3] = {1.0, 2.0, 3.0};
    qp_real delta_s[3] = {-1.0, -2.0, -3.0};

    qp_real z[3] = {-1.0, -2.0, -3.0};
    qp_real delta_z[3] = {1.0, 2.0, 3.0};

    ASSERT_NEAR(1.0, formrho(s, delta_s, z, delta_z, 0.0, 0.0, 4), 1e-10);
    ASSERT_NEAR(0.0, formrho(s, delta_s, z, delta_z, 1.0, 0.0, 4), 1e-10);
    ASSERT_NEAR(0.0, formrho(s, delta_s, z, delta_z, 0.0, 1.0, 4), 1e-10);
    ASSERT_NEAR(0.0, formrho(s, delta_s, z, delta_z, 1.0, 1.0, 4), 1e-10);
    ASSERT_NEAR(4.0, formrho(s, delta_s, z, delta_z, -1.0, -1.0, 4), 1e-10);
    ASSERT_NEAR(9.0, formrho(s, delta_s, z, delta_z, -2.0, -2.0, 4), 1e-10);
}

TEST(standAloneTesting, func_formlambda)
{
    qp_real s[3] = {1.0, 2.0, 3.0};
    qp_real z[3] = {3.0, 2.0, 1.0};
    qp_real lambda[3];

    formlambda(lambda, s, s, 3);
    ASSERT_NEAR(14.0, innerproduct(lambda, lambda, 3), 1e-10);
    formlambda(lambda, z, z, 3);
    ASSERT_NEAR(14.0, innerproduct(lambda, lambda, 3), 1e-10);
    formlambda(lambda, s, z, 3);
    ASSERT_NEAR(10.0, innerproduct(lambda, lambda, 3), 1e-10);
    z[0] = 0.0;
    z[1] = 0.0;
    z[2] = 0.0;
    formlambda(lambda, s, z, 3);
    ASSERT_NEAR(0.0, lambda[0], 1e-10);
    ASSERT_NEAR(0.0, lambda[1], 1e-10);
    ASSERT_NEAR(0.0, lambda[2], 1e-10);
}

TEST(standAloneTesting, func_form_ds)
{
    qp_real delta_s[3] = {-1.0, -2.0, -3.0};
    qp_real delta_z[3] = {1.0, 2.0, 3.0};
    qp_real lambda[3] = {1.0, 2.0, 3.0};

    qp_real ds[3];

    qp_real sigma = 100;
    qp_real mu = 0;

    /* Selector - 1*/
    qp_int selector = 1;
    form_ds(ds, lambda, delta_s, delta_z, sigma, mu, 3, selector);
    ASSERT_NEAR(14.0, innerproduct(lambda, lambda, 3), 1e-10);

    /* Selector - 2*/
    selector = 2;
    form_ds(ds, lambda, delta_s, delta_z, sigma, mu, 3, selector);
    ASSERT_NEAR(14.0, innerproduct(lambda, lambda, 3), 1e-10);

    /* Selector - 3*/
    selector = 3;
    form_ds(ds, lambda, delta_s, delta_z, sigma, mu, 3, selector);
    ASSERT_NEAR(14.0, innerproduct(lambda, lambda, 3), 1e-10);
}

TEST(standAloneTesting, func_findsteplength)
{
    qp_real s[3] = {1.0, 2.0, 3.0};
    qp_real delta_s[3] = {-1.0, -2.0, -3.0};

    qp_real z[3] = {-1.0, -2.0, -3.0};
    qp_real delta_z[3] = {1.0, 2.0, 3.0};

    qp_real alpha_p, alpha_d;
    findsteplength(s, delta_s, z, delta_z, 3, &alpha_p, &alpha_d);
    ASSERT_NEAR(1, alpha_p, 1e-10);
    ASSERT_NEAR(1, alpha_d, 1e-10);

    findsteplength(s, delta_s, z, delta_z, 2, &alpha_p, &alpha_d);
    ASSERT_NEAR(1, alpha_p, 1e-10);
    ASSERT_NEAR(1, alpha_d, 1e-10);

    findsteplength(s, delta_s, z, delta_z, 1, &alpha_p, &alpha_d);
    ASSERT_NEAR(1, alpha_p, 1e-10);
    ASSERT_NEAR(1, alpha_d, 1e-10);
}

TEST(standAloneTesting, func_checksign)
{
    double x[2] = {1.0, 2.0};

    ASSERT_EQ(1, checksign(x, x, -2, 2));
    ASSERT_EQ(0, checksign(x, x, 1, 2));
}

TEST(standAloneTesting, func_updatekktmatrix_b)
{

    // updatekktmatrix_b(b, rx, ry, rz, ds, z, n, m, p);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}