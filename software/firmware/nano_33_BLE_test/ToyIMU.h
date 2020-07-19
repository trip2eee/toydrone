/**
  @file   ToyIMU.h
  @date   20 June 2020
  @brief  CToyIMU class library.
          This class estimates orientation using 9-axis IMU sensor in Arduino nano 33 BLE.
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
#include "typedef.h"
#include "Matrix.h"

#define PI 3.141592F
#define DEG2RAD(x) ((x) * PI / 180.0F)
#define RAD2DEG(x) ((x) * 180.0F / PI)

class CToyIMU
{
public:
    enum State_e
    {
        eSTATE_Q0,
        eSTATE_Q1,
        eSTATE_Q2,
        eSTATE_Q3,
        eSTATE_BIAS_X,
        eSTATE_BIAS_Y,
        eSTATE_BIAS_Z,
        eDIM_STATE
    };

    enum Measurement_e
    {
        eMEASUREMENT_Q0,
        eMEASUREMENT_Q1,
        eMEASUREMENT_Q2,
        eMEASUREMENT_Q3,
        eDIM_MEASUREMENT
    };

    enum Axis_e
    {
        eAXIS_X,
        eAXIS_Y,
        eAXIS_Z,
        eNUM_AXIS
    };

    CToyIMU();
    ~CToyIMU();

    /**
     * @fn    uint8_t Initialize()
     * @brief This method initializes internal states.
    */
    uint8_t Initialize();

    /**
     * @fn    uint8_t Update(const float32_t (&arf32A)[eNUM_AXIS], const float32_t (&arf32W)[eNUM_AXIS], const float32_t (&arf32M)[eNUM_AXIS], const float32_t f32T)
     * @brief This method updates internal states.
     * @param arf32A [in] Array of normalized accelerometer value.
     * @param arf32W [in] Array of normalized magnetometer value.
     * @param arf32M [in] Array of angular velocity.
     * @param f32T   [in] Delta T.
    */
    uint8_t Update(const float32_t (&arf32A)[eNUM_AXIS], const float32_t (&arf32W)[eNUM_AXIS], const float32_t (&arf32M)[eNUM_AXIS], const float32_t f32T);

    /**
     * @fn    uint8_t ComputeQuaternion(const float32_t (&arf32Acc)[eNUM_AXIS], const float32_t (&arf32Mag)[eNUM_AXIS], const float32_t (&arf32InitQ)[4U], float32_t (&arf32Q)[4U])
     * @brief This method computes quaternion describing rotation from current position to upright position.
     * @param arf32Acc   [in] Array of normalized accelerometer value.
     * @param arf32Mag   [in] Array of normalized magnetometer value.
     * @param arf32InitQ [in] Initial value.
     * @param arf32Q     [out] Solution vector.
    */
    uint8_t ComputeQuaternion(const float32_t (&arf32A)[eNUM_AXIS], const float32_t (&arf32M)[eNUM_AXIS], const float32_t (&arf32InitQ)[4U], float32_t (&arf32Q)[4U]);

    void GetQuaternion(float32_t (&arf32Q)[4U]);
    void GetAngles(float32_t (&arf32Angles)[eNUM_AXIS]);

#ifndef _UNIT_TEST
protected:
#endif
    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_STATE> m_oH;
    
    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oQf;
    CMatrix<float32_t, eNUM_AXIS, eNUM_AXIS> m_oQe;
    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_MEASUREMENT> m_oR;

    CMatrix<float32_t, eDIM_STATE, 1U> m_oX;               // State X
    CMatrix<float32_t, eDIM_STATE, 1U> m_oXp;              // Predicted state X

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oP;       // Error covariance P.
    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oPp;      // Predicted error covariance P.

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oF;
    CMatrix<float32_t, eDIM_STATE, eNUM_AXIS> m_oG;

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oJ;

    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_MEASUREMENT> m_oS;
    CMatrix<float32_t, eDIM_STATE, eDIM_MEASUREMENT> m_oK;

    float32_t m_arf32Angles[eNUM_AXIS];

    void PredictKalman(const float32_t (&arf32W)[eNUM_AXIS], const float32_t f32T);
    void UpdateKalman(const float32_t (&arf32A)[eNUM_AXIS], const float32_t (&arf32M)[eNUM_AXIS]);

};
