// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html

#include <math.h>
#include <zephyr/kernel.h>

#include "mymathlib_matrix.h"

#include "magneto1_4.h"

double *C, *S11, *S12, *S12t, *S22, *S22a, *S22b, *SS, *E, *U, *SSS;
double *eigen_real, *eigen_imag, *v1, *v2, *v, *Q, *Q_1, *B, *QB, *SSSS;
double *eigen_real3, *eigen_imag3, *Dz, *vdz, *SQ, *A_1;

// checked allocation, unwinds through magneto_free_all on failure
#define MAG_ALLOC(p, n)                                    \
    do                                                     \
    {                                                      \
        (p) = (double *)k_malloc((n) * sizeof(double));    \
        if ((p) == NULL)                                   \
            goto alloc_failed;                             \
    } while (0)

#define MAG_FREE(p)   \
    do                \
    {                 \
        k_free(p);    \
        (p) = NULL;   \
    } while (0)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

static void magneto_free_all(void)
{
    double **held[] = {
        &C, &S11, &S12, &S12t, &S22, &S22a, &S22b, &SS, &E, &U, &SSS,
        &eigen_real, &eigen_imag, &v1, &v2, &v, &Q, &Q_1, &B, &QB, &SSSS,
        &eigen_real3, &eigen_imag3, &Dz, &vdz, &SQ, &A_1,
    };
    for (unsigned int i = 0; i < ARRAY_SIZE(held); i++)
    {
        k_free(*held[i]);
        *held[i] = NULL;
    }
}

void magneto_sample(double x, double y, double z, double *ata, double *norm_sum, double *sample_count)
{
    *sample_count += 1.0;
    *norm_sum += sqrt(x * x + y * y + z * z);

    double D[10] = {
        x * x,
        y * y,
        z * z,
        2.0 * y * z,
        2.0 * x * z,
        2.0 * x * y,
        2.0 * x,
        2.0 * y,
        2.0 * z,
        1.0};

    Multiply_Self_Transpose(ata, D, 10, 1);
}

int magneto_current_calibration(float BAinv[4][3], double *ata, double norm_sum, double sample_count)
{
    MAG_ALLOC(S11, 6 * 6);
    Get_Submatrix(S11, 6, 6, ata, 10, 0, 0);
    MAG_ALLOC(S12, 6 * 4);
    Get_Submatrix(S12, 6, 4, ata, 10, 0, 6);
    MAG_ALLOC(S12t, 4 * 6);
    Get_Submatrix(S12t, 4, 6, ata, 10, 6, 0);
    MAG_ALLOC(S22, 4 * 4);
    Get_Submatrix(S22, 4, 4, ata, 10, 6, 6);

    double hm = norm_sum / sample_count;

    // this is where we'd deallocate ata or the entire calibration class
    // if we decided to compute the calibration destructively

    Choleski_LU_Decomposition(S22, 4);
    Choleski_LU_Inverse(S22, 4);

    // Calculate S22a = S22 * S12t   4*6 = 4x4 * 4x6   C = AB
    MAG_ALLOC(S22a, 4 * 6);
    Multiply_Matrices(S22a, S22, 4, 4, S12t, 6);
    MAG_FREE(S22);
    MAG_FREE(S12t);

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    MAG_ALLOC(S22b, 6 * 6);
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);
    MAG_FREE(S12);

    // Calculate SS = S11 - S22b
    MAG_ALLOC(SS, 6 * 6);
    for (int i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    MAG_FREE(S11);
    MAG_FREE(S22b);

    // Create pre-inverted constraint matrix C
    MAG_ALLOC(C, 6 * 6);
    C[0] = 0.0;
    C[1] = 0.5;
    C[2] = 0.5;
    C[3] = 0.0;
    C[4] = 0.0;
    C[5] = 0.0;
    C[6] = 0.5;
    C[7] = 0.0;
    C[8] = 0.5;
    C[9] = 0.0;
    C[10] = 0.0;
    C[11] = 0.0;
    C[12] = 0.5;
    C[13] = 0.5;
    C[14] = 0.0;
    C[15] = 0.0;
    C[16] = 0.0;
    C[17] = 0.0;
    C[18] = 0.0;
    C[19] = 0.0;
    C[20] = 0.0;
    C[21] = -0.25;
    C[22] = 0.0;
    C[23] = 0.0;
    C[24] = 0.0;
    C[25] = 0.0;
    C[26] = 0.0;
    C[27] = 0.0;
    C[28] = -0.25;
    C[29] = 0.0;
    C[30] = 0.0;
    C[31] = 0.0;
    C[32] = 0.0;
    C[33] = 0.0;
    C[34] = 0.0;
    C[35] = -0.25;
    MAG_ALLOC(E, 6 * 6);
    Multiply_Matrices(E, C, 6, 6, SS, 6);
    MAG_FREE(C);
    MAG_FREE(SS);

    MAG_ALLOC(SSS, 6 * 6);
    if (Hessenberg_Form_Elementary(E, SSS, 6) < 0)
        goto alloc_failed;

    int index = 0;
    {
        MAG_ALLOC(eigen_real, 6);
        MAG_ALLOC(eigen_imag, 6);

        QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);
        MAG_FREE(E);

        double maxval = eigen_real[0];
        for (int i = 1; i < 6; i++)
        {
            if (eigen_real[i] > maxval)
            {
                maxval = eigen_real[i];
                index = i;
            }
        }
        MAG_FREE(eigen_real);
        MAG_FREE(eigen_imag);
    }

    MAG_ALLOC(v1, 6);
    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];
    MAG_FREE(SSS);

    // normalize v1
    {
        double norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
        v1[0] /= norm;
        v1[1] /= norm;
        v1[2] /= norm;
        v1[3] /= norm;
        v1[4] /= norm;
        v1[5] /= norm;
    }

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    MAG_ALLOC(v2, 4);
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);
    MAG_FREE(S22a);

    MAG_ALLOC(U, 3);
    MAG_ALLOC(Q, 3 * 3);
    double J;
    {
        MAG_ALLOC(v, 10);
        v[0] = v1[0];
        v[1] = v1[1];
        v[2] = v1[2];
        v[3] = v1[3];
        v[4] = v1[4];
        v[5] = v1[5];
        MAG_FREE(v1);
        v[6] = -v2[0];
        v[7] = -v2[1];
        v[8] = -v2[2];
        v[9] = -v2[3];
        MAG_FREE(v2);

        Q[0] = v[0];
        Q[1] = v[5];
        Q[2] = v[4];
        Q[3] = v[5];
        Q[4] = v[1];
        Q[5] = v[3];
        Q[6] = v[4];
        Q[7] = v[3];
        Q[8] = v[2];

        U[0] = v[6];
        U[1] = v[7];
        U[2] = v[8];

        J = v[9];
        MAG_FREE(v);
    }

    MAG_ALLOC(B, 3);
    {
        MAG_ALLOC(Q_1, 3 * 3);
        for (int i = 0; i < 9; i++)
            Q_1[i] = Q[i];
        Choleski_LU_Decomposition(Q_1, 3);
        Choleski_LU_Inverse(Q_1, 3);

        // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)
        Multiply_Matrices(B, Q_1, 3, 3, U, 1);
        MAG_FREE(U);
        MAG_FREE(Q_1);
        B[0] = -B[0]; // x-axis combined bias
        B[1] = -B[1]; // y-axis combined bias
        B[2] = -B[2]; // z-axis combined bias
    }

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    double btqb;
    {
        MAG_ALLOC(QB, 3);
        Multiply_Matrices(QB, Q, 3, 3, B, 1);

        // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
        Multiply_Matrices(&btqb, B, 1, 3, QB, 1);
        MAG_FREE(QB);
    }

    // Calculate SQ, the square root of matrix Q
    MAG_ALLOC(SSSS, 3 * 3);
    if (Hessenberg_Form_Elementary(Q, SSSS, 3) < 0)
        goto alloc_failed;

    MAG_ALLOC(Dz, 3 * 3);
    for (int i = 0; i < 9; i++)
    {
        Dz[i] = 0;
    }
    {
        MAG_ALLOC(eigen_real3, 3);
        MAG_ALLOC(eigen_imag3, 3);
        QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);
        MAG_FREE(Q);

        Dz[0] = sqrt(eigen_real3[0]);
        Dz[4] = sqrt(eigen_real3[1]);
        Dz[8] = sqrt(eigen_real3[2]);
        MAG_FREE(eigen_real3);
        MAG_FREE(eigen_imag3);
    }

    {
        // normalize eigenvectors
        double norm = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
        SSSS[0] /= norm;
        SSSS[3] /= norm;
        SSSS[6] /= norm;
        norm = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
        SSSS[1] /= norm;
        SSSS[4] /= norm;
        SSSS[7] /= norm;
        norm = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
        SSSS[2] /= norm;
        SSSS[5] /= norm;
        SSSS[8] /= norm;
    }

    MAG_ALLOC(SQ, 3 * 3);
    {
        MAG_ALLOC(vdz, 3 * 3);
        ;
        Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);
        MAG_FREE(Dz);
        Transpose_Square_Matrix(SSSS, 3);
        Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);
        MAG_FREE(SSSS);
        MAG_FREE(vdz);
    }

    MAG_ALLOC(A_1, 3 * 3);
    // Calculate hmb = sqrt(btqb - J).
    double hmb = sqrt(btqb - J);

    for (int i = 0; i < 9; i++)
        A_1[i] = SQ[i] * hm / hmb;
    MAG_FREE(SQ);

    for (int i = 0; i < 3; i++)
        BAinv[0][i] = B[i];
    MAG_FREE(B);

    for (int i = 0; i < 3; i++)
    {
        BAinv[i + 1][0] = A_1[i * 3];
        BAinv[i + 1][1] = A_1[i * 3 + 1];
        BAinv[i + 1][2] = A_1[i * 3 + 2];
    }
    MAG_FREE(A_1);
    return 0;

alloc_failed:
    magneto_free_all();
    return -1;
}
