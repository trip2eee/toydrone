/**
 * @file   ut_quaternion.cpp
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  Quaternion unit test.
*/

#include <gtest/gtest.h>
#include "Quaternion.h"

TEST(QuaternionTest, add)
{
    Quaternion oA(0.0F, 1.0F, 0.0F, 0.0F);
    Quaternion oB(0.0F, 0.0F, 1.0F, 1.0F);

    Quaternion oC;

    oC = oA + oB;

    for(uint8_t u8I = 0U; u8I < 4U; u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arf32Q[u8I], oA.m_arf32Q[u8I] + oB.m_arf32Q[u8I]);
    }    
}

TEST(QuaternionTest, sub)
{
    Quaternion oA(0.0F, 1.0F, 0.0F, 0.0F);
    Quaternion oB(0.0F, 0.0F, 1.0F, 1.0F);

    Quaternion oC;

    oC = oA - oB;

    for(uint8_t u8I = 0U; u8I < 4U; u8I++)
    {
        EXPECT_FLOAT_EQ(oC.m_arf32Q[u8I], oA.m_arf32Q[u8I] - oB.m_arf32Q[u8I]);
    }    
}

TEST(QuaternionTest, norm)
{
    Quaternion oA(1.0F, 2.0F, 3.0F, 4.0F);

    const float32_t f32NromA = oA.Norm();
    
    EXPECT_FLOAT_EQ(f32NromA, sqrtf(30.0F));

}

TEST(QuaternionTest, mul)
{
    Quaternion oA(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion oB;

    oB = oA * oA.Invert();
    const float32_t f32NromB = oB.Norm();

    EXPECT_FLOAT_EQ(f32NromB, 1.0F);

}

TEST(QuaternionTest, rotation)
{
    Quaternion oQr;
    Quaternion oQa(0.0F, 1.0F, 0.0F, 0.0F);
    Quaternion oQb;

    float32_t arf32Angles[] = {3.14F / 4.0F, 3.14F / 4.0F, 3.14 / 4.0F};
    
    // qb = qr*qa*conj(qr)
    oQr.SetAngles(arf32Angles);
    oQb = oQa.Rotate(oQr);
    
    CMatrix<float32_t, 3U, 3U> oR = oQr.RotationMatrix();
    CMatrix<float32_t, 3U, 1U> oX;
    CMatrix<float32_t, 3U, 1U> oY;

    // y = R*x
    oX(0) = oQa(1);
    oX(1) = oQa(2);
    oX(2) = oQa(3);
    oY = oR * oX;

    // compare the rotation results.
    for(uint8_t u8I = 0U; u8I < 3U; u8I++)
    {
        EXPECT_FLOAT_EQ(oY(u8I), oQb(u8I + 1U));
    }
}

TEST(QuaternionTest, solve)
{
    Quaternion oQa(0.0F, 1.0F, 0.0F, 0.0F);
    Quaternion oQb(0.0F, 1.0F, 1.0F, 1.0F);
    Quaternion oQr0(1.0F, 0.0F, 0.0F, 0.0F);

    oQb = oQb * (1.0F / oQb.Norm());

    oQb.Print();

    Quaternion oQr = oQa.SolveRotation(oQb, oQr0);

    Quaternion oQb2 = oQa.Rotate(oQr);

    for(uint8_t u8I = 0U; u8I < 3U; u8I++)
    {
        EXPECT_FLOAT_EQ(oQb(u8I), oQb2(u8I));
    }
}




