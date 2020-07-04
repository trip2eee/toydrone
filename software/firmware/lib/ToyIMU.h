/**
 * @file   ToyIMU.h
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  CToyIMU class library.
 *         This class estimates orientation using 9-axis IMU sensor in Arduino nano 33 BLE.
*/
#include "typedef.h"
#include "Matrix.h"

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

    CToyIMU();
    ~CToyIMU();
    static const uint16_t m_u16NumAxis = 3U;

    /**
     * @fn    uint8_t Initialize()
     * @brief This method initializes internal states.
    */
    uint8_t Initialize();

    /**
     * @fn    uint8_t Update(const float32_t (&arf32Z)[eDIM_MEASUREMENT], const float32_t (&arf32W)[m_u16NumAxis])
     * @brief This method updates internal states.
     * @param [in] Array of normalized accelerometer value.
     * @param [in] Array of normalized magnetometer value.
     * @param [in] Array of angular velocity.
    */
    uint8_t Update(const float32_t (&arf32A)[3U], const float32_t (&arf32W)[3U], const float32_t (&arf32M)[m_u16NumAxis], const float32_t f32T);

    /**
     * @fn    uint8_t ComputeQuaternion(const float32_t (&arf32Acc)[3U], const float32_t (&arf32Mag)[3U], const float32_t (&arf32InitQ)[4U], float32_t (&arf32Q)[4U])
     * @brief This method computes quaternion describing rotation from current position to upright position.
     * @param arf32Acc   [in] Array of normalized accelerometer value.
     * @param arf32Mag   [in] Array of normalized magnetometer value.
     * @param arf32InitQ [in] Initial value.
     * @param arf32Q     [out] Solution vector.
    */
    uint8_t ComputeQuaternion(const float32_t (&arf32A)[3U], const float32_t (&arf32M)[3U], const float32_t (&arf32InitQ)[4U], float32_t (&arf32Q)[4U]);

    void GetQuaternion(float32_t (&arf32Q)[4U]);
    void GetAngles(float32_t (&arf32Angles)[m_u16NumAxis]);

#ifndef _UNIT_TEST
protected:
#endif
    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_STATE> m_oH;
    
    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oQf;
    CMatrix<float32_t, m_u16NumAxis, m_u16NumAxis> m_oQe;
    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_MEASUREMENT> m_oR;

    CMatrix<float32_t, eDIM_STATE, 1U> m_oX;
    CMatrix<float32_t, eDIM_STATE, 1U> m_oXp;

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oP;       // P.
    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oPp;       // Predicted P.

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oF;
    CMatrix<float32_t, eDIM_STATE, m_u16NumAxis> m_oG;

    CMatrix<float32_t, eDIM_STATE, eDIM_STATE> m_oJ;

    CMatrix<float32_t, eDIM_MEASUREMENT, eDIM_MEASUREMENT> m_oS;
    CMatrix<float32_t, eDIM_STATE, eDIM_MEASUREMENT> m_oK;

    void PredictKalman(const float32_t (&arf32W)[3U], const float32_t f32T);
    void UpdateKalman(const float32_t (&arf32A)[3U], const float32_t (&arf32M)[m_u16NumAxis]);

};
