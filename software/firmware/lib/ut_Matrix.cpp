/**
 * @file   ut_matrix.cpp
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  Matrix unit test.
*/

#include <gtest/gtest.h>

#include "Matrix.h"

TEST(CMatrixTest, SetArray)
{
    CMatrix<float32_t, 2U, 2U> oA;
    
    const float32_t arf32A[] = {1.0F, 2.0F, 3.0F, 4.0F};
    oA.SetArray(arf32A);

    for(uint8_t u8I = 0U; u8I < (oA.GetRows() * oA.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oA.m_arfMatrix[u8I], arf32A[u8I]);
    }
}

TEST(CMatrixTest, SetDiagonal)
{
    CMatrix<float32_t, 2U, 2U> oA;

    const float32_t arf32D[] = {1.0F, 2.0F};
    const float32_t arf32A[] = {1.0F, 0.0F, 0.0F, 2.0F};

    oA.SetDiagonal(&arf32D[0]);

    for(uint8_t u8I = 0U; u8I < (oA.GetRows() * oA.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oA.m_arfMatrix[u8I], arf32A[u8I]);
    }

}

TEST(CMatrixTest, SetValues)
{
    CMatrix<float32_t, 4U, 3U> oA;

    oA.Zero();

    for(uint8_t u8I = 0U; u8I < (oA.GetRows() * oA.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oA.m_arfMatrix[u8I], 0.0F);
    }

    oA.Identity();
    for(uint8_t u8I = 0U; u8I < oA.GetRows(); u8I++)
    {
        for(uint8_t u8J = 0U; u8J < oA.GetCols(); u8J++)
        {
            if(u8I == u8J)
            {
                EXPECT_FLOAT_EQ(oA.m_arfMatrix[(u8I * oA.GetCols()) + u8J], 1.0F);
            }
            else
            {
                EXPECT_FLOAT_EQ(oA.m_arfMatrix[(u8I * oA.GetCols()) + u8J], 0.0F);
            }
        }
    }
}

TEST(CMatrixTest, operator_plus_minus)
{
    const float32_t arf32A[] = {2.0F, 2.0F, 2.0F, 2.0F};
    const float32_t arf32B[] = {1.0F, 1.0F, 1.0F, 1.0F};

    CMatrix<float32_t, 2U, 2U> oA(arf32A);
    CMatrix<float32_t, 2U, 2U> oB(arf32B);
    CMatrix<float32_t, 2U, 2U> oC;
    CMatrix<float32_t, 2U, 2U> oD;

    oC = (oA + oB) + oA;
    oD = oA - oB;

    for(uint8_t u8I = 0U; u8I < (oC.GetCols()*oC.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arfMatrix[u8I], arf32A[u8I] + arf32A[u8I] + arf32B[u8I]);
        EXPECT_FLOAT_EQ(oD.m_arfMatrix[u8I], arf32A[u8I] - arf32B[u8I]);
    }
}

TEST(CMatrixTest, operator_mul_scalar)
{
    const float32_t arf32A[] = {2.0F, 2.0F, 2.0F, 2.0F};
    const float32_t arf32B[] = {1.0F, 1.0F, 1.0F, 1.0F};
    const float32_t f32S = 10.0F;

    CMatrix<float32_t, 2U, 2U> oA(arf32A);
    CMatrix<float32_t, 2U, 2U> oB(arf32B);
    CMatrix<float32_t, 2U, 2U> oC;

    oC = oA * f32S;
    for(uint8_t u8I = 0U; u8I < (oC.GetRows() * oC.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arfMatrix[u8I], arf32A[u8I] * f32S);
    }

    oC = f32S * oA;
    for(uint8_t u8I = 0U; u8I < (oC.GetRows() * oC.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arfMatrix[u8I], arf32A[u8I] * f32S);
    }
}

TEST(CMatrixTest, operator_mul)
{
    const float32_t arf32A[] = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
    const float32_t arf32B[] = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
    const float32_t arf32C[] = {22.0F, 28.0F, 49.0F, 64.0F};

    CMatrix<float32_t, 2U, 3U> oA(arf32A);
    CMatrix<float32_t, 3U, 2U> oB(arf32B);
    CMatrix<float32_t, 2U, 2U> oC;

    oC = oA * oB;
    
    for(uint8_t u8I = 0U; u8I < (oC.GetRows() * oC.GetCols()); u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arfMatrix[u8I], arf32C[u8I]);
    }
}

TEST(CMatrixTest, operator_paren)
{
    CMatrix<float32_t, 3, 3> oA;

    oA(0U) = 10.0F;
    oA(1U, 1U) = 20.0F;

    EXPECT_FLOAT_EQ(oA.m_arfMatrix[0], 10.0F);
    EXPECT_FLOAT_EQ(oA.m_arfMatrix[(1U * oA.GetCols()) + 1U], 20.0F);

    EXPECT_FLOAT_EQ(oA(0U), 10.0F);
    EXPECT_FLOAT_EQ(oA(1U, 1U), 20.0F);
}

TEST(CMatrixTest, transpose)
{
    const float32_t arf32A[] = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
    const float32_t arf32B[] = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
    
    CMatrix<float32_t, 2U, 3U> oA(arf32A);
    CMatrix<float32_t, 3U, 2U> oB(arf32B);    

    oB = oA.Transpose();
    
    for(uint8_t u8I = 0U; u8I < oB.GetRows(); u8I++)
    {
        for(uint8_t u8J = 0U; u8J < oB.GetCols(); u8J++)
        {
            EXPECT_FLOAT_EQ(oB.m_arfMatrix[(u8I * oB.GetCols()) + u8J], oA.m_arfMatrix[(u8J * oA.GetCols()) + u8I]);
        }        
    }
}

TEST(CMatrixTest, lup)
{
    const float32_t arf32A[] = {10.0F, 20.0F, 0.0F, 
                                20.0F, 0.0F, 10.0F, 
                                0.0F, 20.0F, 30.0F};

    const float32_t arf32LU_exp[] = {20.0F,  0.0F, 10.0F,
                                      0.5F, 20.0F, -5.0F,
                                      0.0F,  1.0F, 35.0F};
    const uint8_t aru8P_exp[] = {1U, 0U, 2U};
    
    CMatrix<float32_t, 3U, 3U> oA(arf32A);

    uint8_t pu8P[3U];
    float32_t arf32LU[9U];
    
    try
    {
        oA.DecomposeLUP(pu8P, arf32LU);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    for(uint8_t u8I = 0U; u8I < oA.GetRows(); u8I++)
    {
        EXPECT_EQ(pu8P[u8I], aru8P_exp[u8I]);
    }

    for(uint16_t u16I = 0U; u16I < (oA.GetCols() * oA.GetRows()); u16I++)
    {
        EXPECT_FLOAT_EQ(arf32LU[u16I], arf32LU_exp[u16I]);
    }

}

TEST(CMatrixTest, solve)
{
    const float32_t arf32A[] = {1.0F, 2.0F, 4.0F, 1.0F};
    const float32_t arf32B[] = {1.0F, 2.0F};
    const float32_t arf32X_exp[] = {0.42857F, 0.28571F};

    CMatrix<float32_t, 2, 2> oA(arf32A);
    CMatrix<float32_t, 2, 1> oB(arf32B);
    CMatrix<float32_t, 2, 1> oX;
    CMatrix<float32_t, 2, 1> oC;

    try
    {
        oX = oA.Solve(oB);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
        
    oC = oA * oX;

    for(uint8_t u8I = 0U; u8I < oX.GetRows(); u8I++)
    {
        EXPECT_NEAR(oX.m_arfMatrix[u8I], arf32X_exp[u8I], 1e-5F);
        EXPECT_FLOAT_EQ(oC.m_arfMatrix[u8I], arf32B[u8I]);
    }
}


TEST(CMatrixTest, inverse)
{
    const float32_t arf32A[] = { 1.0F,  2.0F,  4.0F, 
                                 1.0F, -1.0F, -2.0F,
                                -1.0F, 10.0F,  3.0F};
    CMatrix<float32_t, 3, 3> oA(arf32A);
    CMatrix<float32_t, 3, 3> oB;
    CMatrix<float32_t, 3, 3> oI;

    try
    {
        oB = oA.Invert();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    oI = oA * oB;

    for(uint8_t u8I = 0U; u8I < oI.GetRows(); u8I++)
    {
        for(uint8_t u8J = 0U; u8J < oI.GetRows(); u8J++)
        {
            if(u8I == u8J)
            {
                EXPECT_NEAR(oI(u8I, u8J), 1.0F, 1e-6F);
            }
            else
            {
                EXPECT_NEAR(oI(u8I, u8J), 0.0F, 1e-6F);
            }
                    
        }
    }
}

TEST(CMatrixTest, solveSymmetric)
{
    const float32_t arf32A[] = { 2.0F, 10.0F, 
                                10.0F,  1.0F};
    const float32_t arf32B[] = {1.0F, 
                                2.0F};

    CMatrix<float32_t, 2U, 2U> oA(arf32A);
    CMatrix<float32_t, 2U, 1U> oB(arf32B);
    CMatrix<float32_t, 2U, 1U> oC(arf32B);
    CMatrix<float32_t, 2U, 1U> oX;

    try
    {
        oX = oA.SolveSymmetric(oB);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }    

    // oC = oA * oX;

    // for(uint8_t u8I = 0U; u8I < oC.GetRows(); u8I++)
    // {
    //     EXPECT_NEAR(oC(u8I), arf32B[u8I], 1e-5F);
    // }

}

TEST(CMatrixTest, invertSymmetric)
{
    // Lower triangular matrix.
    const float32_t arf32L[] = {2.0F,  0.0F, 0.0F,
                                10.0F, 1.0F, 0.0F,
                                20.0f, 0.0F, 1.0F};

    const float32_t arf32A[] = {2.0F, 10.0F, 20.0F,
                                10.0F, 1.0F, 0.0F,
                                20.0f, 0.0F, 1.0F};                                

    CMatrix<float32_t, 3U, 3U> oA(arf32L);
    CMatrix<float32_t, 3U, 3U> oB;
    CMatrix<float32_t, 3U, 3U> oI;

    try
    {
        oB = oA.InvertSymmetric();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    oA.SetArray(arf32A);

    oI = oA * oB;

    for(uint8_t u8I = 0U; u8I < oI.GetRows(); u8I++)
    {
        for(uint8_t u8J = 0U; u8J < oI.GetRows(); u8J++)
        {
            if(u8I == u8J)
            {
                EXPECT_NEAR(oI(u8I, u8J), 1.0F, 1e-5F);
            }
            else
            {
                EXPECT_NEAR(oI(u8I, u8J), 0.0F, 1e-5F);
            }
                    
        }
    }   

}