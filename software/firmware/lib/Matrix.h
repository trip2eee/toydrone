/**
 * @file   Matrix.h
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  Matrix class library.
*/

#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "typedef.h"
#include "stdio.h"
#include <string.h>
#include <cfloat>
#include <cmath>

template <typename _T, uint8_t Rows, uint8_t Cols>
class CMatrix
{
public:
    CMatrix()
    {

    }

    CMatrix(const _T* pfArray)
    {
        SetArray(pfArray);
    }
    
    uint8_t GetRows()
    {
        return Rows;
    }

    uint8_t GetCols()
    {
        return Cols;
    }

    /**
     * @fn    void SetArray(const _T* pfArray)
     * @brief This method copies the input array to the matrix.
     * @param pfArray: [in] Input array.
    */
    void SetArray(const _T* pfArray)
    {
        (void)memcpy(&m_arfMatrix[0], pfArray, sizeof(m_arfMatrix));
    }

    /**
     * @fn      void SetDiagonal(const _T* pfDiag)
     * @brief   This methoid assign input array to the diagonal elements of the matrix.
     * @param   pfDiag: [in] input array.
    */
    void SetDiagonal(const _T* pfDiag)
    {
        (void)memset(&m_arfMatrix[0], 0, sizeof(m_arfMatrix));

        // minimum dimension between rows and cols.
        const uint8_t u8M = (Rows < Cols) ? Rows : Cols;
        for(uint8_t u8I = 0U; u8I < u8M; u8I++)
        {
            m_arfMatrix[(u8I * Cols) + u8I] = pfDiag[u8I];
        }
    }

    void Identity()
    {
        (void)memset(&m_arfMatrix[0], 0, sizeof(m_arfMatrix));
        const uint8_t u8M = (Rows < Cols) ? Rows : Cols;

        for(uint8_t u8I = 0U; u8I < u8M; u8I++)
        {
            m_arfMatrix[(u8I * Cols) + u8I] = 1.0F;
        }
    }

    void Zero()
    {
        (void)memset(&m_arfMatrix[0], 0, sizeof(m_arfMatrix));
    }
    
    void Print()
    {
        for(uint8_t u8I = 0U; u8I < Rows; u8I++)
        {
            for(uint8_t u8J = 0U; u8J < Cols; u8J++)
            {
                printf("%f ", m_arfMatrix[(u8I * Cols) + u8J]);
            }
            printf("\n");
        }
    }

    CMatrix<_T, Rows, Cols> operator=(const CMatrix<_T, Rows, Cols>& oA)
    {
        (void)memcpy(m_arfMatrix, oA.m_arfMatrix, sizeof(m_arfMatrix));

        return *this;
    }

    /**
     * @fn    CMatrix<_T, Rows, Cols> operator+(const CMatrix<_T, Rows, Cols>& oA)
     * @brief + operator
     * @param oA: [in] operand 1
     * @return This method returns this + A
    */
    CMatrix<_T, Rows, Cols> operator+(const CMatrix<_T, Rows, Cols>& oA)
    {
        CMatrix<_T, Rows, Cols> oC;
        for(uint16_t u16I = 0U; u16I < (Rows*Cols); u16I++)
        {
            oC.m_arfMatrix[u16I] = m_arfMatrix[u16I] + oA.m_arfMatrix[u16I];
        }

        return oC;
    }

    CMatrix<_T, Rows, Cols> operator-(const CMatrix<_T, Rows, Cols>& oA)
    {
        CMatrix<_T, Rows, Cols> oC;
        for(uint16_t u16I = 0U; u16I < (Rows*Cols); u16I++)
        {
            oC.m_arfMatrix[u16I] = m_arfMatrix[u16I] - oA.m_arfMatrix[u16I];
        }

        return oC;
    }

    CMatrix<_T, Rows, Cols> operator*(const _T fScalar)
    {
        CMatrix<_T, Rows, Cols> oC;
        for(uint16_t u16I = 0U; u16I < (Rows*Cols); u16I++)
        {
            oC.m_arfMatrix[u16I] = fScalar * m_arfMatrix[u16I];
        }

        return oC;
    }
    
    _T& operator()(const uint8_t u8I)
    {
        return m_arfMatrix[u8I];
    }

    _T& operator()(const uint8_t u8I, const uint8_t u8J)
    {
        return m_arfMatrix[(u8I * Cols) + u8J];
    }

    CMatrix<_T, Cols, Rows> Transpose()
    {
        CMatrix<_T, Cols, Rows> oC;
        for(uint8_t u8I = 0U; u8I < Rows; u8I++)
        {
            for(uint8_t u8J = 0U; u8J < Cols; u8J++)
            {
                oC.m_arfMatrix[(u8J * Rows) + u8I] = m_arfMatrix[(u8I * Cols) + u8J];
            }            
        }

        return oC;
    }

    CMatrix<_T, Cols, Rows> Invert()
    {
        CMatrix<_T, Cols, Rows> oB;

        return oB;
    }

    CMatrix<_T, Cols, Rows> Solve(CMatrix<_T, Cols, Rows> oB)
    {
        CMatrix<_T, Cols, Rows> oX;

        uint8_t aru8P[Rows];
        float32_t arfLU[Rows * Cols];

        
        return oX;
    }

    CMatrix<_T, Cols, Rows> InvertSymmetric()
    {
        CMatrix<_T, Cols, Rows> oB;
        
        return oB;
    }

    CMatrix<_T, Cols, Rows> SolveSymmetric(CMatrix<_T, Cols, Rows> oB)
    {
        CMatrix<_T, Cols, Rows> oX;

        return oX;
    }

    template <typename _T1, uint8_t Rows1, uint8_t Cols1>
    friend CMatrix<_T1, Rows1, Cols1> operator*(const _T1 fScalar, CMatrix<_T1, Rows1, Cols1> oA);

    template <typename _T1, uint8_t Rows1, uint8_t Cols1, uint8_t Rows2, uint8_t Cols2>
    friend CMatrix<_T1, Rows1, Cols2> operator*(CMatrix<_T1, Rows1, Cols1> oA, CMatrix<_T1, Rows2, Cols2> oB);
    

#ifndef _UNIT_TEST
protected:
#endif

    uint8_t DecomposeLUP(uint8_t* pu8P, _T* pfLU)
    {
        (void)memcpy(pfLU, m_arfMatrix, sizeof(m_arfMatrix));

        uint8_t u8Valid = 1U;

        for(uint8_t u8I = 0U; u8I < Rows; u8I++)
        {
            pu8P[u8I] = u8I;
        }

        // for column k
        for(uint8_t u8K = 0U; u8K < (Rows - 1U); u8K++)
        {
            _T fPivot = 0.0F;
            uint8_t u8IdxPivot = u8K;

            // for row i
            for(uint8_t u8I = u8K; u8I < Rows; u8I++)
            {
                if(fabsf(pfLU[(u8I * Cols) + u8K]) > fPivot)
                {
                    fPivot = fabsf(pfLU[(u8I * Cols) + u8K]);
                    u8IdxPivot = u8I;
                }
            }

            if(fabsf(fPivot) < FLT_EPSILON)
            {
                // singular matrix.
                u8Valid = 0U;
                break;
            }

            // Pivoting.
            // Permutation.
            const uint8_t u8Temp = pu8P[u8K];
            pu8P[u8K] = pu8P[u8IdxPivot];
            pu8P[u8IdxPivot] = u8Temp;

            // exchange rows.
            for(uint8_t u8J = 0U; u8J < Cols; u8J++)
            {
                const _T fTemp = pfLU[(u8K * Cols) + u8J];
                pfLU[(u8K * Cols) + u8J] = pfLU[(u8IdxPivot * Cols) + u8J];
                pfLU[(u8IdxPivot * Cols) + u8J] = fTemp;
            }

            for(uint8_t u8I = (u8K + 1U); u8I < Rows; u8I++)
            {
                pfLU[(u8I * Cols) + u8K] /= pfLU[(u8K * Cols) + u8K];
                for(uint8_t u8J = (u8K + 1U); u8J < Rows; u8J++)
                {
                    pfLU[(u8I * Cols) + u8J] = pfLU[(u8I * Cols) + u8J] - (pfLU[(u8I * Cols) + u8K] * pfLU[(u8K * Cols) + u8J]);
                }
            }
        }

        return u8Valid;
    }

    _T m_arfMatrix[Rows * Cols];
};

template <typename _T1, uint8_t Rows1, uint8_t Cols1>
CMatrix<_T1, Rows1, Cols1> operator*(const _T1 fScalar, CMatrix<_T1, Rows1, Cols1> oA)
{
    CMatrix<_T1, Rows1, Cols1> oC;
    for(uint16_t u16I = 0U; u16I < (Rows1*Cols1); u16I++)
    {
        oC.m_arfMatrix[u16I] = fScalar * oA.m_arfMatrix[u16I];
    }

    return oC;
}

template <typename _T1, uint8_t Rows1, uint8_t Cols1, uint8_t Rows2, uint8_t Cols2>
CMatrix<_T1, Rows1, Cols2> operator*(CMatrix<_T1, Rows1, Cols1> oA, CMatrix<_T1, Rows2, Cols2> oB)
{
    CMatrix<_T1, Rows1, Cols2> oC;

    for(uint8_t u8I = 0U; u8I < Rows1; u8I++)
    {
        for(uint8_t u8J = 0U; u8J < Cols2; u8J++)
        {
            _T1 fSum = 0.0F;
            for(uint8_t u8K = 0U; u8K < Rows2; u8K++)
            {
                fSum += oA.m_arfMatrix[(u8I * Cols1) + u8K] * oA.m_arfMatrix[(u8K * Cols2) + u8J];
            }
            oC.m_arfMatrix[(u8I * Cols2) + u8J] = fSum;
        }
    }

    return oC;
}

#endif





