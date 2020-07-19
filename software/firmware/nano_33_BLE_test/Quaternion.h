/**
  @file   quaternion.h
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

#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "typedef.h"
#include "Matrix.h"

class Quaternion
{
public:
    Quaternion();
    Quaternion(const float32_t f32Q0, const float32_t f32Q1, const float32_t f32Q2, const float32_t f32Q3);
    Quaternion(const float32_t (&arf32Q)[4U]);
    Quaternion(const Quaternion& other);
    Quaternion(const Quaternion&& other);

    void Print();
    Quaternion Conj();
    float32_t Norm();
    Quaternion Invert();

    Quaternion operator+(const Quaternion& oQ);
    Quaternion operator-(const Quaternion& oQ);
    Quaternion operator*(const Quaternion& oQ);
    Quaternion operator*(const float32_t f32S);
    Quaternion operator=(const Quaternion& oQ);
    Quaternion operator=(const Quaternion&& oQ);
    float32_t& operator()(const uint8_t u8I);
    const float32_t& operator()(const uint8_t u8I) const;
    
    /**
     * @fn     Quaternion Rotate(const Quaternion& oQr)
     * @brief  This method rotates quaternion by Qr.
     * @param  oQr Quaternion defining rotation.
     * @return Quaternion of rotated point.
    */
    Quaternion Rotate(const Quaternion& oQr);

    /**
     * @fn    void ConvertToAngles(float32_t (&arf32Angle)[3U])
     * @brief This method computes the angles (roll, pitch and yaw) of the rotation by quaternion.
     *        Caution: Angles are not the orientation of a 3D point represented by quaternion.
     * @param arf32q Input quaternion.
     * @param arf32Angle: Computed angles (roll, pitch, yaw).
    */
    void ConvertToAngles(float32_t (&arf32Angle)[3U]);

    /**
     * @fn    CMatrix<float32_t, 3U, 3U> RotationMatrix()
     * @brief This method converts quaternion defining rotation into 3x3 Euler rotation matrix.
     * @return 3x3 Rotation matrix.
    */
    CMatrix<float32_t, 3U, 3U> RotationMatrix();

    /**
     * @fn    void SetAngles(const float32_t (&arf32Angle)[3U])
     * @brief This method computes quaternion defining rotation by input angles.
     * @param arf32Angle Input array of angles (roll, pitch, yaw) in radian.
    */
    void SetAngles(const float32_t (&arf32Angles)[3U]);

    /**
     * @fn    Quaternion SolveRotation(const Quaternion& oQb, const Quaternion& oQr0)
     * @brief This method computes quaternion rotating this quaternion to qb.
     * @param oQb Target quaternion qb
     * @param oQr0 Initial quaternion qr.
     * @return The quaternion defining rotation from this to qb.
    */
    Quaternion SolveRotation(const Quaternion& oQb, const Quaternion& oQr0);

#ifndef _UNIT_TEST
protected:
#endif
    float32_t m_arf32Q[4U];  /**< Quaternion q. */
};


#endif


