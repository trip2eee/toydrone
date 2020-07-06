/**
  @file   quaternion.c
  @date   20 June 2020
  @brief  Quaternion library.
  @author Jongmin park (trip2eee@gmail.com)
  @remark Copyright (C) 2020, Jongmin Park (trip2eee@gmail.com)
          Alternatively, the contents of this file may be used under the terms 
          of the GNU General Public License Version 3.0 as described below:

          This program is free software: you can redistribute it and/or modify
          it under the terms of the GNU General Public License as published by
          the Free Software Foundation, either version 3 of the License, or
          (at your option) any later version.

          This program is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
          GNU General Public License for more details.

          You should have received a copy of the GNU General Public License
          along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Quaternion.h"
#include <cmath>

Quaternion::Quaternion()
{

}

Quaternion::Quaternion(const float32_t (&arf32Q)[4U])
{
    (void)memcpy(&m_arf32Q[0], &arf32Q[0], sizeof(m_arf32Q));
}

Quaternion::Quaternion(const float32_t f32Q0, const float32_t f32Q1, const float32_t f32Q2, const float32_t f32Q3)
{
    m_arf32Q[0] = f32Q0;
    m_arf32Q[1] = f32Q1;
    m_arf32Q[2] = f32Q2;
    m_arf32Q[3] = f32Q3;
}

Quaternion::Quaternion(const Quaternion& other)
{
    (void)memcpy(&m_arf32Q[0], &other.m_arf32Q[0], sizeof(m_arf32Q));
}

Quaternion::Quaternion(const Quaternion&& other)
{
    (void)memcpy(&m_arf32Q[0], &other.m_arf32Q[0], sizeof(m_arf32Q));
}

void Quaternion::Print()
{
    printf("%f + %fi + %fj + %fk\n", m_arf32Q[0], m_arf32Q[1], m_arf32Q[2], m_arf32Q[3]);
}

Quaternion Quaternion::Conj()
{
    Quaternion oConj;

    oConj.m_arf32Q[0] =  m_arf32Q[0];
    oConj.m_arf32Q[1] = -m_arf32Q[1];
    oConj.m_arf32Q[2] = -m_arf32Q[2];
    oConj.m_arf32Q[3] = -m_arf32Q[3];

    return oConj;
}

float32_t Quaternion::Norm()
{
    float32_t f32SumSqr = 0.0F;

    f32SumSqr += m_arf32Q[0] * m_arf32Q[0];
    f32SumSqr += m_arf32Q[1] * m_arf32Q[1];
    f32SumSqr += m_arf32Q[2] * m_arf32Q[2];
    f32SumSqr += m_arf32Q[3] * m_arf32Q[3];

    const float32_t f32Norm = sqrtf(f32SumSqr);
    
    return f32Norm;
}

Quaternion Quaternion::Invert()
{
    Quaternion oResult;

    const float32_t f32NormSqr = (m_arf32Q[0]*m_arf32Q[0]) + 
                                 (m_arf32Q[1]*m_arf32Q[1]) + 
                                 (m_arf32Q[2]*m_arf32Q[2]) +
                                 (m_arf32Q[3]*m_arf32Q[3]);

    oResult.m_arf32Q[0] = m_arf32Q[0] / f32NormSqr;
    oResult.m_arf32Q[1] = -m_arf32Q[1] / f32NormSqr;
    oResult.m_arf32Q[2] = -m_arf32Q[2] / f32NormSqr;
    oResult.m_arf32Q[3] = -m_arf32Q[3] / f32NormSqr;
    
    return oResult;
}

Quaternion Quaternion::operator+(const Quaternion& oQ)
{
    Quaternion oResult;

    oResult.m_arf32Q[0] = m_arf32Q[0] + oQ.m_arf32Q[0];
    oResult.m_arf32Q[1] = m_arf32Q[1] + oQ.m_arf32Q[1];
    oResult.m_arf32Q[2] = m_arf32Q[2] + oQ.m_arf32Q[2];
    oResult.m_arf32Q[3] = m_arf32Q[3] + oQ.m_arf32Q[3];

    return oResult;
}

Quaternion Quaternion::operator-(const Quaternion& oQ)
{
    Quaternion oResult;

    oResult.m_arf32Q[0] = m_arf32Q[0] - oQ.m_arf32Q[0];
    oResult.m_arf32Q[1] = m_arf32Q[1] - oQ.m_arf32Q[1];
    oResult.m_arf32Q[2] = m_arf32Q[2] - oQ.m_arf32Q[2];
    oResult.m_arf32Q[3] = m_arf32Q[3] - oQ.m_arf32Q[3];

    return oResult;
}

Quaternion Quaternion::operator*(const Quaternion& oQ)
{
    Quaternion oResult;

    const float32_t f32q0 = m_arf32Q[0];
    const float32_t f32q1 = m_arf32Q[1];
    const float32_t f32q2 = m_arf32Q[2];
    const float32_t f32q3 = m_arf32Q[3];

    const float32_t f32p0 = oQ.m_arf32Q[0];
    const float32_t f32p1 = oQ.m_arf32Q[1];
    const float32_t f32p2 = oQ.m_arf32Q[2];
    const float32_t f32p3 = oQ.m_arf32Q[3];

    oResult.m_arf32Q[0] = f32q0*f32p0 - f32q1*f32p1 - f32q2*f32p2 - f32q3*f32p3;  // real
    oResult.m_arf32Q[1] = f32q0*f32p1 + f32q1*f32p0 + f32q2*f32p3 - f32q3*f32p2;  // i
    oResult.m_arf32Q[2] = f32q0*f32p2 + f32q2*f32p0 - f32q1*f32p3 + f32q3*f32p1;  // j
    oResult.m_arf32Q[3] = f32q0*f32p3 + f32q3*f32p0 + f32q1*f32p2 - f32q2*f32p1;  // k

    return oResult;
}

Quaternion Quaternion::operator*(const float32_t f32S)
{
    Quaternion oResult;

    oResult.m_arf32Q[0] = m_arf32Q[0] * f32S;
    oResult.m_arf32Q[1] = m_arf32Q[1] * f32S;
    oResult.m_arf32Q[2] = m_arf32Q[2] * f32S;
    oResult.m_arf32Q[3] = m_arf32Q[3] * f32S;

    return oResult;
}

Quaternion Quaternion::operator=(const Quaternion& oQ)
{
    (void)memcpy(&m_arf32Q[0], &oQ.m_arf32Q[0], sizeof(m_arf32Q));

    return *this;
}

Quaternion Quaternion::operator=(const Quaternion&& oQ)
{
    (void)memcpy(&m_arf32Q[0], &oQ.m_arf32Q[0], sizeof(m_arf32Q));

    return *this;
}

float32_t& Quaternion::operator()(const uint8_t u8I)
{
    return m_arf32Q[u8I];
}

const float32_t& Quaternion::operator()(const uint8_t u8I) const
{
    return m_arf32Q[u8I];
}

Quaternion Quaternion::Rotate(const Quaternion& oQr)
{
    Quaternion oResult;

    const float32_t f32x = m_arf32Q[1];
    const float32_t f32y = m_arf32Q[2];
    const float32_t f32z = m_arf32Q[3];

    const float32_t f32q0 = oQr.m_arf32Q[0];
    const float32_t f32q1 = oQr.m_arf32Q[1];
    const float32_t f32q2 = oQr.m_arf32Q[2];
    const float32_t f32q3 = oQr.m_arf32Q[3];
    
    const float32_t f32r00 = 1.0F - 2.0F*((f32q2*f32q2) + (f32q3*f32q3));
    const float32_t f32r01 = 2.0F*((f32q1*f32q2) - (f32q0*f32q3));
    const float32_t f32r02 = 2.0F*((f32q0*f32q2) + (f32q1*f32q3));

    const float32_t f32r10 = 2.0F*((f32q1*f32q2) + (f32q0*f32q3));
    const float32_t f32r11 = 1.0F - 2.0F*((f32q1*f32q1) + (f32q3*f32q3));
    const float32_t f32r12 = 2.0F*((f32q2*f32q3) - (f32q0*f32q1));

    const float32_t f32r20 = 2.0F*((f32q1*f32q3) - (f32q0*f32q2));
    const float32_t f32r21 = 2.0F*((f32q0*f32q1) + (f32q2*f32q3));
    const float32_t f32r22 = 1.0F - 2.0F*((f32q1*f32q1) + (f32q2*f32q2));
    
    oResult.m_arf32Q[0] = 0.0F;
    oResult.m_arf32Q[1] = (f32r00*f32x) + (f32r01*f32y) + (f32r02*f32z);
    oResult.m_arf32Q[2] = (f32r10*f32x) + (f32r11*f32y) + (f32r12*f32z);
    oResult.m_arf32Q[3] = (f32r20*f32x) + (f32r21*f32y) + (f32r22*f32z);

    return oResult;
}

void Quaternion::ConvertToAngles(float32_t (&arf32Angle)[3U])
{
    const float32_t f32q0 = m_arf32Q[0];
    const float32_t f32q1 = m_arf32Q[1];
    const float32_t f32q2 = m_arf32Q[2];
    const float32_t f32q3 = m_arf32Q[3];

    const float32_t f32Roll = atan2f(2.0F*((f32q0*f32q1) + (f32q2*f32q3)), 1.0F - 2.0F*((f32q1*f32q1) + (f32q2*f32q2)));
    const float32_t f32Pitch = asinf(2.0F*((f32q0*f32q2) - (f32q3*f32q1)));
    const float32_t f32Yaw = atan2f(2.0F*((f32q0*f32q3) + (f32q1*f32q2)), 1.0F - 2.0F*((f32q2*f32q2) + (f32q3*f32q3)));

    arf32Angle[0] = f32Roll;
    arf32Angle[1] = f32Pitch;
    arf32Angle[2] = f32Yaw;
}

CMatrix<float32_t, 3U, 3U> Quaternion::RotationMatrix()
{
    CMatrix<float32_t, 3U, 3U> oR;

    const float32_t f32q0 = m_arf32Q[0];
    const float32_t f32q1 = m_arf32Q[1];
    const float32_t f32q2 = m_arf32Q[2];
    const float32_t f32q3 = m_arf32Q[3];

    float32_t arf32R[9U];

    arf32R[0] = 1.0F - 2.0F*(f32q2*f32q2 + f32q3*f32q3);
    arf32R[1] = 2.0F*(f32q1*f32q2 - f32q0*f32q3);
    arf32R[2] = 2.0F*(f32q0*f32q2 + f32q1*f32q3);

    arf32R[3] = 2.0F*(f32q1*f32q2 + f32q0*f32q3);
    arf32R[4] = 1.0F - 2.0F*(f32q1*f32q1 + f32q3*f32q3);
    arf32R[5] = 2.0F*(f32q2*f32q3 - f32q0*f32q1);

    arf32R[6] = 2.0F*(f32q1*f32q3 - f32q0*f32q2);
    arf32R[7] = 2.0F*(f32q0*f32q1 + f32q2*f32q3);
    arf32R[8] = 1.0F - 2.0F*(f32q1*f32q1 + f32q2*f32q2);

    oR.SetArray(arf32R);

    return oR;
}

void Quaternion::SetAngles(const float32_t (&arf32Angles)[3U])
{
    const float32_t f32Roll = arf32Angles[0];
    const float32_t f32Pitch = arf32Angles[1];
    const float32_t f32Yaw = arf32Angles[2];

    const float32_t f32C_roll  = cosf(f32Roll * 0.5);
    const float32_t f32C_pitch = cosf(f32Pitch * 0.5);
    const float32_t f32C_yaw   = cosf(f32Yaw * 0.5);

    const float32_t f32S_roll  = sinf(f32Roll * 0.5);
    const float32_t f32S_pitch = sinf(f32Pitch * 0.5);
    const float32_t f32S_yaw   = sinf(f32Yaw * 0.5);

    m_arf32Q[0] = ((f32C_roll*f32C_pitch)*f32C_yaw) + ((f32S_roll*f32S_pitch)*f32S_yaw);
    m_arf32Q[1] = ((f32S_roll*f32C_pitch)*f32C_yaw) - ((f32C_roll*f32S_pitch)*f32S_yaw);
    m_arf32Q[2] = ((f32C_roll*f32S_pitch)*f32C_yaw) + ((f32S_roll*f32C_pitch)*f32S_yaw);
    m_arf32Q[3] = ((f32C_roll*f32C_pitch)*f32S_yaw) - ((f32S_roll*f32S_pitch)*f32C_yaw);
}

Quaternion Quaternion::SolveRotation(const Quaternion& qb, const Quaternion& qr0)
{
    const float32_t q0 = qr0(0);
    const float32_t q1 = qr0(1);
    const float32_t q2 = qr0(2);
    const float32_t q3 = qr0(3);
    CMatrix<float32_t, 4U, 1U> oQr_lm;
    oQr_lm(0) = q0;
    oQr_lm(1) = q1;
    oQr_lm(2) = q2;
    oQr_lm(3) = q3;
    
    const float32_t xa = m_arf32Q[1];
    const float32_t ya = m_arf32Q[2];
    const float32_t za = m_arf32Q[3];

    CMatrix<float32_t, 4U, 1U> oB;
    oB(0) = 1.0F;               // constraints to make norm 1.
    oB(1) = qb.m_arf32Q[1];
    oB(2) = qb.m_arf32Q[2];
    oB(3) = qb.m_arf32Q[3];

    float32_t f32mu = 1e-4F;

    // compute rotation.
    CMatrix<float32_t, 4U, 1U> oF_lm;
    oF_lm(0) = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    oF_lm(1) = xa*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + ya*(-2.0F*q0*q3 + 2.0F*q1*q2) + za*(2.0F*q0*q2 + 2.0F*q1*q3);
    oF_lm(2) = xa*(2.0F*q0*q3 + 2.0F*q1*q2) + ya*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + za*(-2.0F*q0*q1 + 2.0F*q2*q3);
    oF_lm(3) = xa*(-2.0F*q0*q2 + 2.0F*q1*q3) + ya*(2.0F*q0*q1 + 2.0F*q2*q3) + za*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    CMatrix<float32_t, 4U, 4U> oJq;
    CMatrix<float32_t, 4U, 4U> oJqT;
    oJq(0,0) = 2.0F*q0;
    oJq(0,1) = 2.0F*q1;
    oJq(0,2) = 2.0F*q2;
    oJq(0,3) = 2.0F*q3; 

    oJq(1,0) = 2.0F*( (q0*xa) + (q2*za) - (q3*ya));
    oJq(1,1) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));
    oJq(1,2) = 2.0F*( (q0*za) + (q1*ya) - (q2*xa));
    oJq(1,3) = 2.0F*(-(q0*ya) + (q1*za) - (q3*xa));

    oJq(2,0) = 2.0F*( (q0*ya) - (q1*za) + (q3*xa));
    oJq(2,1) = 2.0F*(-(q0*za) - (q1*ya) + (q2*xa));
    oJq(2,2) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));
    oJq(2,3) = 2.0F*( (q0*xa) + (q2*za) - (q3*ya));
    
    oJq(3,0) = 2.0F*( (q0*za) + (q1*ya) - (q2*xa));
    oJq(3,1) = 2.0F*( (q0*ya) - (q1*za) + (q3*xa));
    oJq(3,2) = 2.0F*(-(q0*xa) - (q2*za) + (q3*ya));
    oJq(3,3) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));

    oJqT = oJq.Transpose();

    CMatrix<float32_t, 4U, 1U> oRes_lm;     // optimum residual vector.
    oRes_lm = oF_lm - oB;
    float32_t f32NormRes_lm = oRes_lm.Norm();

    // Steepest descent.
    CMatrix<float32_t, 4U, 1U> oSD;
    oSD = oJqT * oRes_lm;

    int32_t s32Iter = 0;

    while(s32Iter < 20)
    {
        // Compute Hessian.        
        // Hessian.
        CMatrix<float32_t, 4U, 4U> oH;
        CMatrix<float32_t, 4U, 4U> oI;
        oI.Identity();
        oH = (oJqT * oJq) + (f32mu * oI);

        // Compute delta qr.
        CMatrix<float32_t, 4U, 1U> oDeltaQr;
        oDeltaQr = -1.0F * oH.SolveSymmetric(oSD);

        // Compute qr.
        CMatrix<float32_t, 4U, 1U> oQr;
        oQr = oQr_lm + oDeltaQr;        

        // compute F.
        const float32_t q0 = oQr(0);
        const float32_t q1 = oQr(1);
        const float32_t q2 = oQr(2);
        const float32_t q3 = oQr(3);
        
        CMatrix<float32_t, 4U, 1U> oF;
        oF(0) = q0*q0 + q1*q1 + q2*q2 + q3*q3;
        oF(1) = xa*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + ya*(-2.0F*q0*q3 + 2.0F*q1*q2) + za*(2.0F*q0*q2 + 2.0F*q1*q3);
        oF(2) = xa*(2.0F*q0*q3 + 2.0F*q1*q2) + ya*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + za*(-2.0F*q0*q1 + 2.0F*q2*q3);
        oF(3) = xa*(-2.0F*q0*q2 + 2*q1*q3) + ya*(2.0F*q0*q1 + 2.0F*q2*q3) + za*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

        CMatrix<float32_t, 4U, 1U> oRes;
        oRes = oF - oB;
        const float32_t f32NormRes = oRes.Norm();

        // If error is minimized.
        if(f32NormRes < f32NormRes_lm)
        {
            oQr_lm = oQr;
            f32NormRes_lm = f32NormRes;
            oRes_lm = oRes;
            oF_lm = oF;

            // Update Jacobian.
            oJq(0,0) = 2.0F*q0;
            oJq(0,1) = 2.0F*q1;
            oJq(0,2) = 2.0F*q2;
            oJq(0,3) = 2.0F*q3; 

            oJq(1,0) = 2.0F*( (q0*xa) + (q2*za) - (q3*ya));
            oJq(1,1) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));
            oJq(1,2) = 2.0F*( (q0*za) + (q1*ya) - (q2*xa));
            oJq(1,3) = 2.0F*(-(q0*ya) + (q1*za) - (q3*xa));

            oJq(2,0) = 2.0F*( (q0*ya) - (q1*za) + (q3*xa));
            oJq(2,1) = 2.0F*(-(q0*za) - (q1*ya) + (q2*xa));
            oJq(2,2) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));
            oJq(2,3) = 2.0F*( (q0*xa) + (q2*za) - (q3*ya));
            
            oJq(3,0) = 2.0F*( (q0*za) + (q1*ya) - (q2*xa));
            oJq(3,1) = 2.0F*( (q0*ya) - (q1*za) + (q3*xa));
            oJq(3,2) = 2.0F*(-(q0*xa) - (q2*za) + (q3*ya));
            oJq(3,3) = 2.0F*( (q1*xa) + (q2*ya) + (q3*za));

            oJqT = oJq.Transpose();

            // Update steepest descent.
            oSD = oJqT * oRes_lm;

            // Reduce damping factor.
            f32mu *= 0.1F;
        }
        // If error is not minimized.
        else
        {
            // Increase damping factor.
            f32mu *= 10.0F;
        }

        if(f32NormRes_lm < 1e-10F)
        {
            break;
        }

        s32Iter++;
    }

    Quaternion qr;
    qr(0) = oQr_lm(0);
    qr(1) = oQr_lm(1);
    qr(2) = oQr_lm(2);
    qr(3) = oQr_lm(3);

    return qr;
}
