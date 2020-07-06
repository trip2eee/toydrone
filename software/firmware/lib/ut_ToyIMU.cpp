#include <gtest/gtest.h>
#include "ToyIMU.h"
#include "Quaternion.h"

TEST(ToyIMUTest, ComputeQ)
{
    const float32_t arf32Acc[] = {0.0F, 0.0F, -0.99F};
    const float32_t arf32Mag[] = {-12.68F,  -4.57F,  69.58F};
    float32_t arf32Q[] = {1.0F, 0.0F, 0.0F, 0.0F};

    CToyIMU oIMU;
    oIMU.ComputeQuaternion(arf32Acc, arf32Mag, arf32Q, arf32Q);

    const float32_t arf32Q_expected[] = {0.17209823F,
                                         0.0F,
                                         0.0F,
                                         0.98507979F};

    for(uint16_t u16I = 0U; u16I < 4U; u16I++)
    {
        EXPECT_NEAR(arf32Q[u16I], arf32Q_expected[u16I], 1e-6F);
    }

    Quaternion oQ(arf32Q);
    float32_t arf32Angles[3U];
    oQ.ConvertToAngles(arf32Angles);

    const float32_t arf32Angle_expected[] = {0.0F, 0.0F, 2.79567408F};
    for(uint16_t u16I = 0U; u16I < 3U; u16I++)
    {
        EXPECT_NEAR(arf32Angles[u16I], arf32Angle_expected[u16I], 1e-6F);
    }

}

TEST(ToyIMUTest, Initialize)
{
    CToyIMU oIMU;
    
    const float32_t f32SigmaQ = 0.1F;
    const float32_t f32SigmaBW = 0.01F;

    const float32_t arf32X[] = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    const float32_t arf32Std[] = {f32SigmaQ, f32SigmaQ, f32SigmaQ, f32SigmaQ,
                                f32SigmaBW, f32SigmaBW, f32SigmaBW};

    oIMU.Initialize();

    const uint16_t u16DimState = static_cast<uint16_t>(CToyIMU::eDIM_STATE);

    for(uint16_t u16I = 0U; u16I < u16DimState; u16I++)
    {
        EXPECT_FLOAT_EQ(oIMU.m_oX(u16I), arf32X[u16I]);
    }

    for(uint16_t u16I = 0U; u16I < u16DimState; u16I++)
    {
        for(uint16_t u16J = 0U; u16J < u16DimState; u16J++)
        {
            if(u16I == u16J)
            {
                EXPECT_FLOAT_EQ(oIMU.m_oP(u16I, u16J), arf32Std[u16I] * arf32Std[u16I]);
            }
            else
            {
                EXPECT_FLOAT_EQ(oIMU.m_oP(u16I, u16J), 0.0F);
            }
        }
    }    
        
}


TEST(ToyIMUTest, Update)
{
    CToyIMU oIMU;
    oIMU.Initialize();

    int32_t t = 0;

    FILE* pstFileInput = fopen("imu_test.txt", "r");
    FILE* pstFileOutput = fopen("imu_result_cpp.txt", "w");

    ASSERT_NE(reinterpret_cast<int>(pstFileInput), NULL) << "Failed to open test sequence data." << std::endl;

    while(1)
    {
        printf("iteration: %d\n", t);

        float32_t arf32A[3U];       // accelerometer
        float32_t arf32W[3U];       // gyroscope
        float32_t arf32M[3U];       // magnetometer
        float32_t f32T;

        float32_t arf32X_expected[7U];
        float32_t arf32P_expected[49U];

        int32_t s32Ret = 0U;
        // read aceelerometer
        for(uint16_t u16I = 0U; u16I < 3U; u16I++)
        {
            s32Ret = fscanf(pstFileInput, "%f ", &arf32A[u16I]);
        }
        if(s32Ret == 0 || s32Ret == EOF)
        {
            break;
        }

        // read gyroscope
        for(uint16_t u16I = 0U; u16I < 3U; u16I++)
        {
            fscanf(pstFileInput, "%f ", &arf32W[u16I]);
        }

        // read magnetometer
        for(uint16_t u16I = 0U; u16I < 3U; u16I++)
        {
            fscanf(pstFileInput, "%f ", &arf32M[u16I]);
        }

        // read T
        fscanf(pstFileInput, "%f ", &f32T);

        // read expected x        
        for(uint16_t u16I = 0U; u16I < 7U; u16I++)
        {
            fscanf(pstFileInput, "%f ", &arf32X_expected[u16I]);
        }

        // read expected P        
        for(uint16_t u16I = 0U; u16I < 49U; u16I++)
        {
            fscanf(pstFileInput, "%f ", &arf32P_expected[u16I]);
        }
        
        
        float32_t arf32X[7U];
        float32_t arf32SigmaP[7U];
        
        oIMU.Update(arf32A, arf32W, arf32M, f32T);
#if 0
        for(uint16_t u16I = 0U; u16I < 7U; u16I++)
        {
            EXPECT_NEAR(oIMU.m_oX(u16I), arf32X_expected[u16I], 1e-1F);
#if 1
            for(uint16_t u16J = 0U; u16J < 7U; u16J++)
            {
                EXPECT_NEAR(oIMU.m_oP(u16I, u16J), arf32P_expected[(u16I * 7U) + u16J], 1e-1F);
            }
#endif
        }                        
#endif

        for(uint8_t u8I = 0U; u8I < 7U; u8I++)
        {
            fprintf(pstFileOutput, "%f ", oIMU.m_oXp(u8I));
        }

        for(uint8_t u8I = 0U; u8I < 7U; u8I++)
        {
            fprintf(pstFileOutput, "%f ", oIMU.m_oX(u8I));
        }

        float32_t arf32Angles[3U];
        oIMU.GetAngles(arf32Angles);

        for(uint8_t u8I = 0U; u8I < 3U; u8I++)
        {
            fprintf(pstFileOutput, "%f ", arf32Angles[u8I]);
        }

        fprintf(pstFileOutput, "\n");
        t++;
    }

    fclose(pstFileInput);
    fclose(pstFileOutput);

    oIMU.m_oX.Print();
}

