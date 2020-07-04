/**
 * @file   ToyIMU.cpp
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  CToyIMU class library.
 *         This class estimates orientation using 9-axis IMU sensor in Arduino nano 33 BLE.
*/

#include "ToyIMU.h"
#include "Quaternion.h"
#include <string.h>

CToyIMU::CToyIMU()
{    
    Initialize();
}

CToyIMU::~CToyIMU()
{
    
}

uint8_t CToyIMU::Initialize()
{
    const float32_t f32SigmaQ = 0.1F;
    const float32_t f32SigmaW = 0.3F;
    const float32_t f32SigmaBW = 0.01F;

    m_oQf.Zero();
    m_oQf(0,0) = f32SigmaQ * f32SigmaQ;
    m_oQf(1,1) = f32SigmaQ * f32SigmaQ;
    m_oQf(2,2) = f32SigmaQ * f32SigmaQ;
    m_oQf(3,3) = f32SigmaQ * f32SigmaQ;

    m_oQf(4,4) = f32SigmaBW * f32SigmaBW;
    m_oQf(5,5) = f32SigmaBW * f32SigmaBW;
    m_oQf(6,6) = f32SigmaBW * f32SigmaBW;

    m_oQe.Zero();
    m_oQe(0,0) = f32SigmaW * f32SigmaW;
    m_oQe(1,1) = f32SigmaW * f32SigmaW;
    m_oQe(2,2) = f32SigmaW * f32SigmaW;

    const float32_t f32SigmaQ_R = 0.15F;

    m_oR.Zero();
    m_oR(0,0) = f32SigmaQ_R * f32SigmaQ_R;
    m_oR(1,1) = f32SigmaQ_R * f32SigmaQ_R;
    m_oR(2,2) = f32SigmaQ_R * f32SigmaQ_R;
    m_oR(3,3) = f32SigmaQ_R * f32SigmaQ_R;


    m_oH.Zero();
    m_oH(0,0) = 1.0F;
    m_oH(1,1) = 1.0F;
    m_oH(2,2) = 1.0F;
    m_oH(3,3) = 1.0F;

    m_oF.Identity();
    m_oG.Zero();
    m_oJ.Identity();

    m_oX.Zero();
    m_oX(0) = 1.0F;     // (1, 0, 0, 0, 0, 0, 0)

    m_oXp.Zero();

    m_oP.Zero();
    m_oP(0,0) = (f32SigmaQ * f32SigmaQ);
    m_oP(1,1) = (f32SigmaQ * f32SigmaQ);
    m_oP(2,2) = (f32SigmaQ * f32SigmaQ);
    m_oP(3,3) = (f32SigmaQ * f32SigmaQ);

    m_oP(4,4) = (f32SigmaBW * f32SigmaBW);
    m_oP(5,5) = (f32SigmaBW * f32SigmaBW);
    m_oP(6,6) = (f32SigmaBW * f32SigmaBW);

    return 0U;
}


uint8_t CToyIMU::Update(const float32_t (&arf32A)[3U], const float32_t (&arf32W)[3U], const float32_t (&arf32M)[m_u16NumAxis], const float32_t f32T)
{
    // Predict
    PredictKalman(arf32W, f32T);

    // Update
    UpdateKalman(arf32A, arf32M);
    
    return 0U;
}

void CToyIMU::PredictKalman(const float32_t (&arf32W)[3U], const float32_t f32T)
{
    const float32_t f32w0 = arf32W[0];
    const float32_t f32w1 = arf32W[1];
    const float32_t f32w2 = arf32W[2];

    const float32_t f32q0 = m_oX(0);
    const float32_t f32q1 = m_oX(1);
    const float32_t f32q2 = m_oX(2);
    const float32_t f32q3 = m_oX(3);

    const float32_t f32bw0 = m_oX(4);
    const float32_t f32bw1 = m_oX(5);
    const float32_t f32bw2 = m_oX(6);

    m_oXp(0) = -f32T*f32q1*(-f32bw0 + f32w0)*0.5F - f32T*f32q2*(-f32bw1 + f32w1)*0.5F - f32T*f32q3*(-f32bw2 + f32w2)*0.5F + f32q0;
    m_oXp(1) = f32T*f32q0*(-f32bw0 + f32w0)*0.5F + f32T*f32q2*(-f32bw2 + f32w2)*0.5F - f32T*f32q3*(-f32bw1 + f32w1)*0.5F + f32q1;
    m_oXp(2) = f32T*f32q0*(-f32bw1 + f32w1)*0.5F - f32T*f32q1*(-f32bw2 + f32w2)*0.5F + f32T*f32q3*(-f32bw0 + f32w0)*0.5F + f32q2;
    m_oXp(3) = f32T*f32q0*(-f32bw2 + f32w2)*0.5F + f32T*f32q1*(-f32bw1 + f32w1)*0.5F - f32T*f32q2*(-f32bw0 + f32w0)*0.5F + f32q3;
    m_oXp(4) = f32bw0;
    m_oXp(5) = f32bw1;
    m_oXp(6) = f32bw2;

    m_oF(0, 0) =  1.0F;
    m_oF(0, 1) = -f32T*(f32w0 - f32bw0)*0.5F;
    m_oF(0, 2) = -f32T*(f32w1 - f32bw1)*0.5F;
    m_oF(0, 3) = -f32T*(f32w2 - f32bw2)*0.5F;

    m_oF(0, 4) =  f32T*f32q1*0.5F;
    m_oF(0, 5) =  f32T*f32q2*0.5F;
    m_oF(0, 6) =  f32T*f32q3*0.5F;

    m_oF(1, 0) =  f32T*(f32w0 - f32bw0)*0.5F;
    m_oF(1, 1) =  1.0F;
    m_oF(1, 2) =  f32T*(f32w2 - f32bw2)*0.5F;
    m_oF(1, 3) = -f32T*(f32w1 - f32bw1)*0.5F;

    m_oF(1, 4) = -f32T*f32q0*0.5F;
    m_oF(1, 5) =  f32T*f32q3*0.5F;
    m_oF(1, 6) = -f32T*f32q2*0.5F;

    m_oF(2, 0) =  f32T*(f32w1 - f32bw1)*0.5F;
    m_oF(2, 1) = -f32T*(f32w2 - f32bw2)*0.5F;
    m_oF(2, 2) =  1.0F;
    m_oF(2, 3) =  f32T*(f32w0 - f32bw0)*0.5F;

    m_oF(2, 4) = -f32T*f32q3*0.5F;
    m_oF(2, 5) = -f32T*f32q0*0.5F;
    m_oF(2, 6) =  f32T*f32q1*0.5F;


    m_oF(3, 0) =  f32T*(f32w2 - f32bw2)*0.5F;
    m_oF(3, 1) =  f32T*(f32w1 - f32bw1)*0.5F;
    m_oF(3, 2) = -f32T*(f32w0 - f32bw0)*0.5F;
    m_oF(3, 3) =  1.0F;
    
    m_oF(3, 4) =  f32T*f32q2*0.5F;
    m_oF(3, 5) = -f32T*f32q1*0.5F;
    m_oF(3, 6) = -f32T*f32q0*0.5F;
        
    m_oG(0, 0) =  f32T*f32q1*0.5F;
    m_oG(0, 1) =  f32T*f32q2*0.5F;
    m_oG(0, 2) =  f32T*f32q3*0.5F;

    m_oG(1, 0) = -f32T*f32q0*0.5F;
    m_oG(1, 1) =  f32T*f32q3*0.5F;
    m_oG(1, 2) = -f32T*f32q2*0.5F;

    m_oG(2, 0) = -f32T*f32q3*0.5F;
    m_oG(2, 1) = -f32T*f32q0*0.5F;
    m_oG(2, 2) =  f32T*f32q1*0.5F;

    m_oG(3, 0) =  f32T*f32q2*0.5F;
    m_oG(3, 1) = -f32T*f32q1*0.5F;
    m_oG(3, 2) = -f32T*f32q0*0.5F;

    m_oPp = (m_oF*m_oP*m_oF.Transpose()) + m_oQf + (m_oG*m_oQe*m_oG.Transpose());
}

void CToyIMU::UpdateKalman(const float32_t (&arf32A)[3U], const float32_t (&arf32M)[m_u16NumAxis])
{
    float32_t arf32Q[4U];
#if 0
    arf32Q[0] = m_oXp(0);
    arf32Q[1] = m_oXp(1);
    arf32Q[2] = m_oXp(2);
    arf32Q[3] = m_oXp(3);
#else
    arf32Q[0] = 1.0F;
    arf32Q[1] = 0.0F;
    arf32Q[2] = 0.0F;
    arf32Q[3] = 0.0F;    
#endif

    Quaternion oQ_pred(m_oXp(0), m_oXp(1), m_oXp(2), m_oXp(3));

    ComputeQuaternion(arf32A, arf32M, arf32Q, arf32Q);

    Quaternion oQ_cur(arf32Q);

    const float32_t f32NormDiff0 = (oQ_pred - oQ_cur).Norm();
    const float32_t f32NormDiff1 = (oQ_pred + oQ_cur).Norm();

    // if quaternion is flipped.
    if(f32NormDiff0 > f32NormDiff1)
    {
        printf("Flip\n");
        arf32Q[0] = -arf32Q[0];
        arf32Q[1] = -arf32Q[1];
        arf32Q[2] = -arf32Q[2];
        arf32Q[3] = -arf32Q[3];
    }

    // res = z - H*x-
    CMatrix<float32_t, eDIM_MEASUREMENT, 1U> oRes;
    CMatrix<float32_t, eDIM_MEASUREMENT, 1U> oZ(arf32Q);
    oRes = oZ - m_oH*m_oXp;

    m_oS = (m_oH*m_oPp*m_oH.Transpose()) + m_oR;
    m_oK = m_oPp*m_oH.Transpose()*m_oS.InvertSymmetric();

    m_oX = m_oXp + (m_oK * oRes);

    m_oP = m_oPp - (m_oK*m_oH*m_oPp);

    // Normalize.
    const float32_t f32SumSqrQ = (m_oX(0)*m_oX(0)) + (m_oX(1)*m_oX(1)) + (m_oX(2)*m_oX(2)) + (m_oX(3)*m_oX(3));
    const float32_t f32NormQ = sqrtf(f32SumSqrQ);
    
    for(uint16_t u16I = 0U; u16I < 4U; u16I++)
    {
        for(uint16_t u16J = 0U; u16J < 4U; u16J++)
        {
            m_oJ(u16I, u16J) = (m_oX(u16I) * m_oX(u16J)) / (f32NormQ * f32NormQ * f32NormQ);
        }
    }

    m_oP = (m_oJ*m_oP*m_oJ.Transpose());

    // Normalize quaternions only.
    m_oX(0) = m_oX(0) / f32NormQ;
    m_oX(1) = m_oX(1) / f32NormQ;
    m_oX(2) = m_oX(2) / f32NormQ;
    m_oX(3) = m_oX(3) / f32NormQ;
    
}

void CToyIMU::GetQuaternion(float32_t (&arf32Q)[4U])
{

}


void CToyIMU::GetAngles(float32_t (&arf32Angles)[m_u16NumAxis])
{
    Quaternion oQ(m_oX(0), m_oX(1), m_oX(2), m_oX(3));
    oQ.ConvertToAngles(arf32Angles);
}

uint8_t CToyIMU::ComputeQuaternion(const float32_t (&arf32A)[3U], const float32_t (&arf32M)[3U], const float32_t (&arf32InitQ)[4U], float32_t (&arf32Q)[4U])
{
    const uint8_t u8NumConst = 7U;      // The number of constraints.
    const uint8_t u8DimSolution = 4U;   // The dimension of the solution (quaternion)
    const uint16_t u16MaxIter = 20U;    // Maximum number of iteration.

    CMatrix<float32_t, u8DimSolution, 1U> oQ_lm(arf32InitQ);    // optimal solution vector.

    // Normalized acceleration.
    const float32_t f32SumSqrAcc = (arf32A[0]*arf32A[0]) + (arf32A[1]*arf32A[1]) + (arf32A[2]*arf32A[2]);
    const float32_t f32NormAcc = sqrtf(f32SumSqrAcc);

    const float32_t f32a0 = arf32A[0] / f32NormAcc;
    const float32_t f32a1 = arf32A[1] / f32NormAcc;
    const float32_t f32a2 = arf32A[2] / f32NormAcc;

    // compute m <- m / norm(m), cross product a x (m x a)
    const float32_t f32SumSqrMag = (arf32M[0]*arf32M[0]) + (arf32M[1]*arf32M[1]) + (arf32M[2]*arf32M[2]);
    const float32_t f32NormMag = sqrtf(f32SumSqrMag);
    
    const float32_t f32m0_un = ( f32a1*(-f32a0*arf32M[1] + f32a1*arf32M[0]) - f32a2*( f32a0*arf32M[2] - f32a2*arf32M[0])) / f32NormMag;
    const float32_t f32m1_un = (-f32a0*(-f32a0*arf32M[1] + f32a1*arf32M[0]) + f32a2*(-f32a1*arf32M[2] + f32a2*arf32M[1])) / f32NormMag;
    const float32_t f32m2_un = ( f32a0*( f32a0*arf32M[2] - f32a2*arf32M[0]) - f32a1*(-f32a1*arf32M[2] + f32a2*arf32M[1])) / f32NormMag;

    // normalize again.
    const float32_t f32NormX = sqrtf((f32m0_un*f32m0_un) + (f32m1_un*f32m1_un) + (f32m2_un*f32m2_un));
    const float32_t f32m0 = f32m0_un / f32NormX;
    const float32_t f32m1 = f32m1_un / f32NormX;
    const float32_t f32m2 = f32m2_un / f32NormX;

    float32_t f32q0 = oQ_lm(0);
    float32_t f32q1 = oQ_lm(1);
    float32_t f32q2 = oQ_lm(2);
    float32_t f32q3 = oQ_lm(3);

    float32_t arf32B[] = {1.0F, 0.0F, 0.0F, -1.0F, 1.0F, 0.0F, 0.0F};
    CMatrix<float32_t, u8NumConst, 1U> oB(arf32B);
   
    float32_t arf32F_lm[] = {
        (f32q0*f32q0) + (f32q1*f32q1) + (f32q2*f32q2) + (f32q3*f32q3),

        f32a0*((f32q0*f32q0) + (f32q1*f32q1) - (f32q2*f32q2) - (f32q3*f32q3)) + f32a1*(-2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32a2*(2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3),
        f32a0*(2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32a1*((f32q0*f32q0) - (f32q1*f32q1) + (f32q2*f32q2) - (f32q3*f32q3)) + f32a2*(-2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3),
        f32a0*(-2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3) + f32a1*(2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3) + f32a2*((f32q0*f32q0) - (f32q1*f32q1) - (f32q2*f32q2) + (f32q3*f32q3)),

        f32m0*((f32q0*f32q0) + (f32q1*f32q1) - (f32q2*f32q2) - (f32q3*f32q3)) + f32m1*(-2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32m2*(2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3),
        f32m0*(2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32m1*((f32q0*f32q0) - (f32q1*f32q1) + (f32q2*f32q2) - (f32q3*f32q3)) + f32m2*(-2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3),
        f32m0*(-2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3) + f32m1*(2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3) + f32m2*((f32q0*f32q0) - (f32q1*f32q1) - (f32q2*f32q2) + (f32q3*f32q3))
    };
    CMatrix<float32_t, u8NumConst, 1U> oF_lm(arf32F_lm);

    // Jacobian    
    float32_t arf32Jq_lm[] = {
        2.0F*f32q0, 2.0F*f32q1, 2.0F*f32q2, 2.0F*f32q3,

        2.0F*f32q0*f32a0 + 2.0F*f32q2*f32a2 - 2.0F*f32q3*f32a1, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2, 2.0F*f32q0*f32a2 + 2.0F*f32q1*f32a1 - 2.0F*f32q2*f32a0, -2.0F*f32q0*f32a1 + 2.0F*f32q1*f32a2 - 2.0F*f32q3*f32a0,
        2.0F*f32q0*f32a1 - 2.0F*f32q1*f32a2 + 2.0F*f32q3*f32a0, -2.0F*f32q0*f32a2 - 2.0F*f32q1*f32a1 + 2.0F*f32q2*f32a0, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2, 2.0F*f32q0*f32a0 + 2.0F*f32q2*f32a2 - 2.0F*f32q3*f32a1,
        2.0F*f32q0*f32a2 + 2.0F*f32q1*f32a1 - 2.0F*f32q2*f32a0, 2.0F*f32q0*f32a1 - 2.0F*f32q1*f32a2 + 2.0F*f32q3*f32a0, -2.0F*f32q0*f32a0 - 2.0F*f32q2*f32a2 + 2.0F*f32q3*f32a1, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2,
        
        2.0F*f32q0*f32m0 + 2.0F*f32q2*f32m2 - 2.0F*f32q3*f32m1, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2, 2.0F*f32q0*f32m2 + 2.0F*f32q1*f32m1 - 2.0F*f32q2*f32m0, -2.0F*f32q0*f32m1 + 2.0F*f32q1*f32m2 - 2.0F*f32q3*f32m0,
        2.0F*f32q0*f32m1 - 2.0F*f32q1*f32m2 + 2.0F*f32q3*f32m0, -2.0F*f32q0*f32m2 - 2.0F*f32q1*f32m1 + 2.0F*f32q2*f32m0, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2, 2.0F*f32q0*f32m0 + 2.0F*f32q2*f32m2 - 2.0F*f32q3*f32m1,
        2.0F*f32q0*f32m2 + 2.0F*f32q1*f32m1 - 2.0F*f32q2*f32m0, 2.0F*f32q0*f32m1 - 2.0F*f32q1*f32m2 + 2.0F*f32q3*f32m0, -2.0F*f32q0*f32m0 - 2.0F*f32q2*f32m2 + 2.0F*f32q3*f32m1, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2
    };
    CMatrix<float32_t, u8NumConst, u8DimSolution> oJq(arf32Jq_lm);
    CMatrix<float32_t, u8DimSolution, u8NumConst> oJqt = oJq.Transpose();

    // Residual
    CMatrix<float32_t, u8NumConst, 1U> oRes;
    oRes = oF_lm - oB;
    float32_t f32NormRes_lm = oRes.Norm();

    // Steepest Descent
    CMatrix<float32_t, u8DimSolution, 1U> oSD;
    oSD = oJqt * oRes;

    float32_t f32Mu = 1e-4F;
    
    // Damping vector.
    CMatrix<float32_t, u8DimSolution, u8DimSolution> oMu;
    oMu.Identity();
    oMu = oMu * 1e-4F;      // mu * I


    // for each iteration
    for(uint16_t u16Iter = 0U; u16Iter < u16MaxIter; u16Iter++)
    {
        // Compute Hessian.
        CMatrix<float32_t, u8DimSolution, u8DimSolution> oH;
        oH = (oJqt * oJq) + oMu;

        // Compute delta q
        CMatrix<float32_t, u8DimSolution, 1U> oDeltaQ;
        oDeltaQ = (oH.InvertSymmetric() * oSD);

        CMatrix<float32_t, u8DimSolution, 1U> oQ;
        oQ = oQ_lm - oDeltaQ;

        f32q0 = oQ(0);
        f32q1 = oQ(1);
        f32q2 = oQ(2);
        f32q3 = oQ(3);

        // Compute f
        float32_t arf32F[] = {
            (f32q0*f32q0) + (f32q1*f32q1) + (f32q2*f32q2) + (f32q3*f32q3),

            f32a0*((f32q0*f32q0) + (f32q1*f32q1) - (f32q2*f32q2) - (f32q3*f32q3)) + f32a1*(-2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32a2*(2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3),
            f32a0*(2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32a1*((f32q0*f32q0) - (f32q1*f32q1) + (f32q2*f32q2) - (f32q3*f32q3)) + f32a2*(-2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3),
            f32a0*(-2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3) + f32a1*(2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3) + f32a2*((f32q0*f32q0) - (f32q1*f32q1) - (f32q2*f32q2) + (f32q3*f32q3)),

            f32m0*((f32q0*f32q0) + (f32q1*f32q1) - (f32q2*f32q2) - (f32q3*f32q3)) + f32m1*(-2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32m2*(2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3),
            f32m0*(2.0F*f32q0*f32q3 + 2.0F*f32q1*f32q2) + f32m1*((f32q0*f32q0) - (f32q1*f32q1) + (f32q2*f32q2) - (f32q3*f32q3)) + f32m2*(-2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3),
            f32m0*(-2.0F*f32q0*f32q2 + 2.0F*f32q1*f32q3) + f32m1*(2.0F*f32q0*f32q1 + 2.0F*f32q2*f32q3) + f32m2*((f32q0*f32q0) - (f32q1*f32q1) - (f32q2*f32q2) + (f32q3*f32q3))
        };
        CMatrix<float32_t, u8NumConst, 1U> oF(arf32F);

        // Residual
        CMatrix<float32_t, u8NumConst, 1U> oRes;
        oRes = oF - oB;

        const float32_t f32NormRes = oRes.Norm();

        if(f32NormRes < f32NormRes_lm)
        {
            oQ_lm = oQ;
            f32NormRes_lm = f32NormRes;
            oF_lm = oF;

            // Update Jacobian.
            float32_t arf32Jq[] = {
                2.0F*f32q0, 2.0F*f32q1, 2.0F*f32q2, 2.0F*f32q3,

                2.0F*f32q0*f32a0 + 2.0F*f32q2*f32a2 - 2.0F*f32q3*f32a1, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2, 2.0F*f32q0*f32a2 + 2.0F*f32q1*f32a1 - 2.0F*f32q2*f32a0, -2.0F*f32q0*f32a1 + 2.0F*f32q1*f32a2 - 2.0F*f32q3*f32a0,
                2.0F*f32q0*f32a1 - 2.0F*f32q1*f32a2 + 2.0F*f32q3*f32a0, -2.0F*f32q0*f32a2 - 2.0F*f32q1*f32a1 + 2.0F*f32q2*f32a0, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2, 2.0F*f32q0*f32a0 + 2.0F*f32q2*f32a2 - 2.0F*f32q3*f32a1,
                2.0F*f32q0*f32a2 + 2.0F*f32q1*f32a1 - 2.0F*f32q2*f32a0, 2.0F*f32q0*f32a1 - 2.0F*f32q1*f32a2 + 2.0F*f32q3*f32a0, -2.0F*f32q0*f32a0 - 2.0F*f32q2*f32a2 + 2.0F*f32q3*f32a1, 2.0F*f32q1*f32a0 + 2.0F*f32q2*f32a1 + 2.0F*f32q3*f32a2,
                
                2.0F*f32q0*f32m0 + 2.0F*f32q2*f32m2 - 2.0F*f32q3*f32m1, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2, 2.0F*f32q0*f32m2 + 2.0F*f32q1*f32m1 - 2.0F*f32q2*f32m0, -2.0F*f32q0*f32m1 + 2.0F*f32q1*f32m2 - 2.0F*f32q3*f32m0,
                2.0F*f32q0*f32m1 - 2.0F*f32q1*f32m2 + 2.0F*f32q3*f32m0, -2.0F*f32q0*f32m2 - 2.0F*f32q1*f32m1 + 2.0F*f32q2*f32m0, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2, 2.0F*f32q0*f32m0 + 2.0F*f32q2*f32m2 - 2.0F*f32q3*f32m1,
                2.0F*f32q0*f32m2 + 2.0F*f32q1*f32m1 - 2.0F*f32q2*f32m0, 2.0F*f32q0*f32m1 - 2.0F*f32q1*f32m2 + 2.0F*f32q3*f32m0, -2.0F*f32q0*f32m0 - 2.0F*f32q2*f32m2 + 2.0F*f32q3*f32m1, 2.0F*f32q1*f32m0 + 2.0F*f32q2*f32m1 + 2.0F*f32q3*f32m2
            };

            oJq.SetArray(arf32Jq);
            oJqt = oJq.Transpose();
            oSD = oJqt * oRes;

            oMu = oMu * 0.1F;
        }
        else
        {
            oMu = oMu * 10.0F;
        }

        if(f32NormRes_lm < 1e-10F)
        {
            break;
        }
    }

    oQ_lm.Copy(arf32Q);

    // Normalize.
    const float32_t f32SumSqrQ = (arf32Q[0]*arf32Q[0]) + (arf32Q[1]*arf32Q[1]) + (arf32Q[2]*arf32Q[2]) + (arf32Q[3]*arf32Q[3]);
    const float32_t f32NormQ = sqrtf(f32SumSqrQ);
    arf32Q[0] /= f32NormQ;
    arf32Q[1] /= f32NormQ;
    arf32Q[2] /= f32NormQ;
    arf32Q[3] /= f32NormQ;

    return 0U;
}
