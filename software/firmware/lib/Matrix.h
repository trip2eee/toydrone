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

template <typename _T, uint8_t ROWS, uint8_t COLS>
class CMatrix
{
public:
    CMatrix()
    {

    }

    CMatrix(CMatrix<_T, ROWS, COLS>& other)
    {
        (void)memcpy(&m_arfMatrix[0], &other.m_arfMatrix[0], sizeof(m_arfMatrix));
    }

    CMatrix(CMatrix<_T, ROWS, COLS>&& other)
    {
        (void)memcpy(&m_arfMatrix[0], &other.m_arfMatrix[0], sizeof(m_arfMatrix));
    }

    CMatrix(const _T (&arfArray)[ROWS * COLS])
    {
        SetArray(arfArray);
    }
    

    uint8_t GetRows()
    {
        return ROWS;
    }

    uint8_t GetCols()
    {
        return COLS;
    }

    /**
     * @fn    void SetArray(const _T* pfArray)
     * @brief This method copies the input array to the matrix.
     * @param pfArray: [in] Input array.
    */
    void SetArray(const _T (&arfArray)[ROWS * COLS])
    {
        (void)memcpy(&m_arfMatrix[0], &arfArray[0], sizeof(m_arfMatrix));
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
        const uint8_t u8M = (ROWS < COLS) ? ROWS : COLS;
        for(uint8_t u8I = 0U; u8I < u8M; u8I++)
        {
            m_arfMatrix[(u8I * COLS) + u8I] = pfDiag[u8I];
        }
    }

    void Identity()
    {
        (void)memset(&m_arfMatrix[0], 0, sizeof(m_arfMatrix));
        const uint8_t u8M = (ROWS < COLS) ? ROWS : COLS;

        for(uint8_t u8I = 0U; u8I < u8M; u8I++)
        {
            m_arfMatrix[(u8I * COLS) + u8I] = 1.0F;
        }
    }

    void Zero()
    {
        (void)memset(&m_arfMatrix[0], 0, sizeof(m_arfMatrix));
    }
    
    void Print()
    {
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            for(uint8_t u8J = 0U; u8J < COLS; u8J++)
            {
                printf("%f ", m_arfMatrix[(u8I * COLS) + u8J]);
            }
            printf("\n");
        }
    }

    CMatrix<_T, ROWS, COLS> operator=(const CMatrix<_T, ROWS, COLS>& oA)
    {
        (void)memcpy(&m_arfMatrix[0], &oA.m_arfMatrix[0], sizeof(m_arfMatrix));

        return *this;
    }

    CMatrix<_T, ROWS, COLS> operator=(const CMatrix<_T, ROWS, COLS>&& other)
    {
        (void)memcpy(&m_arfMatrix[0], &other.m_arfMatrix[0], sizeof(m_arfMatrix));

        return *this;
    }

    /**
     * @fn    CMatrix<_T, ROWS, COLS> operator+(const CMatrix<_T, ROWS, COLS>& oA)
     * @brief + operator
     * @param oA: [in] operand
     * @return This method returns this + A
    */
    CMatrix<_T, ROWS, COLS> operator+(const CMatrix<_T, ROWS, COLS>& oA)
    {
        CMatrix<_T, ROWS, COLS> oC;
        for(uint16_t u16I = 0U; u16I < (ROWS*COLS); u16I++)
        {
            oC.m_arfMatrix[u16I] = m_arfMatrix[u16I] + oA.m_arfMatrix[u16I];
        }

        return oC;
    }

    /**
     * @fn    CMatrix<_T, ROWS, COLS> operator-(const CMatrix<_T, ROWS, COLS>& oA)
     * @brief - operator
     * @param oA: [in] operand
     * @return This method returns this - A
    */
    CMatrix<_T, ROWS, COLS> operator-(const CMatrix<_T, ROWS, COLS>& oA)
    {
        CMatrix<_T, ROWS, COLS> oC;
        for(uint16_t u16I = 0U; u16I < (ROWS*COLS); u16I++)
        {
            oC.m_arfMatrix[u16I] = m_arfMatrix[u16I] - oA.m_arfMatrix[u16I];
        }

        return oC;
    }

    /**
     * @fn    CMatrix<_T, ROWS, COLS> operator*(const _T fScalar)
     * @brief Multiplication A * s. A: matrix, s: scalar.
     * @param oA: [in] operand
     * @return This method returns this * s
    */
    CMatrix<_T, ROWS, COLS> operator*(const _T fScalar)
    {
        CMatrix<_T, ROWS, COLS> oC;
        for(uint16_t u16I = 0U; u16I < (ROWS*COLS); u16I++)
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
        return m_arfMatrix[(u8I * COLS) + u8J];
    }

    CMatrix<_T, COLS, ROWS> Transpose()
    {
        CMatrix<_T, COLS, ROWS> oC;
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            for(uint8_t u8J = 0U; u8J < COLS; u8J++)
            {
                oC.m_arfMatrix[(u8J * ROWS) + u8I] = m_arfMatrix[(u8I * COLS) + u8J];
            }            
        }

        return oC;
    }

    CMatrix<_T, ROWS, COLS> Invert()
    {
        CMatrix<_T, ROWS, COLS> oB;

        uint8_t aru8P[ROWS];
        _T arfLU[ROWS * COLS];
        _T arfY[ROWS];

        DecomposeLUP(aru8P, arfLU);

        // Test for singularity.
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            if(fabsf(arfLU[(u8I * COLS) + u8I]) < FLT_EPSILON)
            {
                // singular matrix.
                throw std::out_of_range("The matrix is singular.");                
            }
        }

        // for each column of identity matrix.
        for(uint8_t u8C = 0U; u8C < COLS; u8C++)
        {
            // Compute forward substitution.
            for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
            {
                // Determine values of the c-th column vector of identity matrix.
                if(u8C == aru8P[u8I])
                {
                    arfY[u8I] = 1.0F;
                }
                else
                {
                    arfY[u8I] = 0.0F;
                }

                for(uint8_t u8J = 0U; u8J < u8I; u8J++)
                {
                    arfY[u8I] -= (arfLU[(u8I * COLS) + u8J] * arfY[u8J]);
                }
            }

            // Compute backward substitution.
            for(int16_t s16I = (static_cast<int16_t>(ROWS)-1); s16I >= 0; s16I--)
            {
                const uint8_t u8I = static_cast<uint8_t>(s16I);

                oB.m_arfMatrix[(u8I * COLS) + u8C] = arfY[u8I];
                for(uint8_t u8J = (u8I + 1U); u8J < COLS; u8J++)
                {
                    oB.m_arfMatrix[(u8I * COLS) + u8C] -= (arfLU[(u8I * COLS) + u8J] * oB.m_arfMatrix[(u8J * COLS) + u8C]);
                }
                oB.m_arfMatrix[(u8I * COLS) + u8C] /= (arfLU[(u8I * COLS) + u8I]);
            }
        }

        return oB;
    }

    CMatrix<_T, ROWS, 1U> Solve(const CMatrix<_T, ROWS, 1U>& oB)
    {
        CMatrix<_T, ROWS, 1U> oX;

        uint8_t aru8P[ROWS];
        _T arfLU[ROWS * COLS];
        _T arfY[ROWS];

        DecomposeLUP(aru8P, arfLU);

        // Test for singularity.
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            if(fabsf(arfLU[(u8I * COLS) + u8I]) < FLT_EPSILON)
            {
                // singular matrix.
                throw std::out_of_range("The matrix is singular.");                
            }
        }

        // Compute forward substitution.
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            arfY[u8I] = oB.m_arfMatrix[aru8P[u8I]];
            
            for(uint8_t u8J = 0U; u8J < u8I; u8J++)
            {
                arfY[u8I] -= (arfLU[(u8I * COLS) + u8J] * arfY[u8J]);
            }
        }

        // Compute backward substitution.
        for(int16_t s16I = (static_cast<int16_t>(ROWS)-1); s16I >= 0; s16I--)
        {
            const uint8_t u8I = static_cast<uint8_t>(s16I);

            oX.m_arfMatrix[u8I] = arfY[u8I];
            for(uint8_t u8J = (u8I + 1U); u8J < COLS; u8J++)
            {
                oX.m_arfMatrix[u8I] -= (arfLU[(u8I * COLS) + u8J] * oX.m_arfMatrix[u8J]);
            }
            oX.m_arfMatrix[u8I] /= (arfLU[(u8I * COLS) + u8I]);
        }

        return oX;
    }

    CMatrix<_T, ROWS, COLS> InvertSymmetric()
    {
        CMatrix<_T, ROWS, COLS> oB;

        _T arfL[ROWS * COLS];
        _T arfD[ROWS];
        _T arfY[ROWS];
        
        DecomposeLDL(arfL, arfD);
        
        // for each column of identity matrix.
        for(uint8_t u8C = 0U; u8C < COLS; u8C++)
        {
            // Compute forward substitution.
            for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
            {
                if(u8C == u8I)
                {
                    arfY[u8I] = 1.0F;
                }
                else
                {
                    arfY[u8I] = 0.0F;
                }                
                
                for(uint8_t u8J = 0U; u8J < u8I; u8J++)
                {
                    arfY[u8I] -= (arfD[u8J] * arfL[(u8I * COLS) + u8J]) * arfY[u8J];
                }

                arfY[u8I] /= arfD[u8I];
            }

            // Compute backward substitution.
            for(int16_t s16I = (static_cast<int16_t>(ROWS)-1); s16I >= 0; s16I--)
            {
                const uint8_t u8I = static_cast<uint8_t>(s16I);

                oB.m_arfMatrix[(u8I * COLS) + u8C] = arfY[u8I];
                for(uint8_t u8J = (u8I + 1U); u8J < COLS; u8J++)
                {
                    oB.m_arfMatrix[(u8I * COLS) + u8C] -= (arfL[(u8J * COLS) + u8I] * oB.m_arfMatrix[(u8J * COLS) + u8C]);
                }
            }
        }

        return oB;
    }

    /**
     * @fn    CMatrix<_T, ROWS, COLS> SolveSymmetric(CMatrix<_T, ROWS, 1U> oB)
     * @brief This method solves lineary equation Ax = b, where A is symmetric. Lower triangular part has to have valid values.
     * @param oB b vector.
     * @return Solution vector x.
    */
    CMatrix<_T, ROWS, 1U> SolveSymmetric(CMatrix<_T, ROWS, 1U>& oB)
    {
        CMatrix<_T, ROWS, 1U> oX;

        _T arfL[ROWS * COLS];
        _T arfD[ROWS];
        _T arfY[ROWS];
        
        DecomposeLDL(arfL, arfD);
        
        // Compute forward substitution.
        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            arfY[u8I] = oB.m_arfMatrix[u8I];
            
            for(uint8_t u8J = 0U; u8J < u8I; u8J++)
            {
                arfY[u8I] -= (arfD[u8J] * arfL[(u8I * COLS) + u8J]) * arfY[u8J];
            }

            arfY[u8I] /= arfD[u8I];
        }

        // Compute backward substitution.
        for(int16_t s16I = (static_cast<int16_t>(ROWS)-1); s16I >= 0; s16I--)
        {
            const uint8_t u8I = static_cast<uint8_t>(s16I);

            oX.m_arfMatrix[u8I] = arfY[u8I];
            for(uint8_t u8J = (u8I + 1U); u8J < COLS; u8J++)
            {
                oX.m_arfMatrix[u8I] -= (arfL[(u8J * COLS) + u8I] * oX.m_arfMatrix[u8J]);
            }
        }

        return oX;
    }

    /**
     * @fn    CMatrix<_T1, ROWS1, COLS1> operator*(const _T1 fScalar, CMatrix<_T1, ROWS1, COLS1> oA)
     * @brief Multiplication s * A. A: matrix, s: scalar.
     * @param fScalar: [in] scalar s.
     * @param oA: [in] matrix A.
     * @return This method returns s * A.
    */
    template <typename _T1, uint8_t ROWS1, uint8_t COLS1>
    friend CMatrix<_T1, ROWS1, COLS1> operator*(const _T1 fScalar, CMatrix<_T1, ROWS1, COLS1> oA);

    /**
     * @fn    CMatrix<_T1, ROWS1, COLS2> operator*(CMatrix<_T1, ROWS1, COLS1> oA, CMatrix<_T1, ROWS2, COLS2> oB);
     * @brief Multiplication A * B.
     * @param oA: [in] matrix A.
     * @param oB: [in] matrix B.
     * @return This method returns A * B.
    */
    template <typename _T1, uint8_t ROWS1, uint8_t COLS1, uint8_t ROWS2, uint8_t COLS2>
    friend CMatrix<_T1, ROWS1, COLS2> operator*(CMatrix<_T1, ROWS1, COLS1> oA, CMatrix<_T1, ROWS2, COLS2> oB);
    

#ifndef _UNIT_TEST
protected:
#endif
    /**
     * @fn  void DecomposeLUP(_T (&arfL)[ROWS * COLS], _T (&arfD)[ROWS])
     * @brief This method decomposes matrix into PA = LU
     * @param aru8P Permutation
     * @param arfLU Decomposed matrix. Lower triangular part: L, Upper triangular part: P.
    */
    void DecomposeLUP(uint8_t (&aru8P)[ROWS], _T (&arfLU)[ROWS * COLS])
    {
        (void)memcpy(arfLU, m_arfMatrix, sizeof(m_arfMatrix));

        for(uint8_t u8I = 0U; u8I < ROWS; u8I++)
        {
            aru8P[u8I] = u8I;
        }

        // for column k
        for(uint8_t u8K = 0U; u8K < (ROWS - 1U); u8K++)
        {
            _T fPivot = 0.0F;
            uint8_t u8IdxPivot = u8K;

            // for row i
            for(uint8_t u8I = u8K; u8I < ROWS; u8I++)
            {
                if(fabsf(arfLU[(u8I * COLS) + u8K]) > fPivot)
                {
                    fPivot = fabsf(arfLU[(u8I * COLS) + u8K]);
                    u8IdxPivot = u8I;
                }
            }

            if(fabsf(fPivot) < FLT_EPSILON)
            {
                // singular matrix.
                throw std::out_of_range("The matrix is singular.");                
            }

            // Pivoting.
            // Permutation.
            const uint8_t u8Temp = aru8P[u8K];
            aru8P[u8K] = aru8P[u8IdxPivot];
            aru8P[u8IdxPivot] = u8Temp;

            // exchange rows.
            for(uint8_t u8J = 0U; u8J < COLS; u8J++)
            {
                const _T fTemp = arfLU[(u8K * COLS) + u8J];
                arfLU[(u8K * COLS) + u8J] = arfLU[(u8IdxPivot * COLS) + u8J];
                arfLU[(u8IdxPivot * COLS) + u8J] = fTemp;
            }

            for(uint8_t u8I = (u8K + 1U); u8I < ROWS; u8I++)
            {
                arfLU[(u8I * COLS) + u8K] /= arfLU[(u8K * COLS) + u8K];
                for(uint8_t u8J = (u8K + 1U); u8J < ROWS; u8J++)
                {
                    arfLU[(u8I * COLS) + u8J] = arfLU[(u8I * COLS) + u8J] - (arfLU[(u8I * COLS) + u8K] * arfLU[(u8K * COLS) + u8J]);
                }
            }
        }
    }
    
    /**
     * @fn  void DecomposeLDL(_T (&arfL)[ROWS * COLS], _T (&arfD)[ROWS])
     * @brief This method decomposes matrix into LDL^T (Cholesky decomposition).
     * @param arfL Lower triangular matrix.
     * @param arfD Diagonal matrix.
    */
    void DecomposeLDL(_T (&arfL)[ROWS * COLS], _T (&arfD)[ROWS])
    {   
        (void)memset(&arfL[0], 0, sizeof(arfL));

        for(uint8_t u8J = 0U; u8J < COLS; u8J++)
        {
            arfD[u8J] = m_arfMatrix[(u8J * COLS) + u8J];
            for(uint8_t u8K = 0U; u8K < u8J; u8K++)
            {
                arfD[u8J] -= arfL[(u8J * COLS) + u8K]*arfL[(u8J * COLS) + u8K] * arfD[u8K];
            }

            if(fabsf(arfD[u8J]) < FLT_EPSILON)
            {
                // singular matrix.
                throw std::out_of_range("The matrix is singular.");                
            }

            for(uint8_t u8I = u8J; u8I < ROWS; u8I++)
            {
                arfL[(u8I * COLS) + u8J] = m_arfMatrix[(u8I * COLS) + u8J];
                for(uint8_t u8K = 0U; u8K < u8J; u8K++)
                {
                    arfL[(u8I * COLS) + u8J] -= (arfL[(u8I * COLS) + u8K] * arfL[(u8J * COLS) + u8K]) * arfD[u8K];
                }
                arfL[(u8I * COLS) + u8J] /= arfD[u8J];
            }
        }
    }
    _T m_arfMatrix[ROWS * COLS];
};

template <typename _T1, uint8_t ROWS1, uint8_t COLS1>
CMatrix<_T1, ROWS1, COLS1> operator*(const _T1 fScalar, CMatrix<_T1, ROWS1, COLS1> oA)
{
    CMatrix<_T1, ROWS1, COLS1> oC;
    for(uint16_t u16I = 0U; u16I < (ROWS1*COLS1); u16I++)
    {
        oC.m_arfMatrix[u16I] = fScalar * oA.m_arfMatrix[u16I];
    }

    return oC;
}

template <typename _T1, uint8_t ROWS1, uint8_t COLS1, uint8_t ROWS2, uint8_t COLS2>
CMatrix<_T1, ROWS1, COLS2> operator*(CMatrix<_T1, ROWS1, COLS1> oA, CMatrix<_T1, ROWS2, COLS2> oB)
{
    CMatrix<_T1, ROWS1, COLS2> oC;

    for(uint8_t u8I = 0U; u8I < ROWS1; u8I++)
    {
        for(uint8_t u8J = 0U; u8J < COLS2; u8J++)
        {
            _T1 fSum = 0.0F;
            for(uint8_t u8K = 0U; u8K < COLS1; u8K++)
            {
                fSum += oA.m_arfMatrix[(u8I * COLS1) + u8K] * oB.m_arfMatrix[(u8K * COLS2) + u8J];
            }
            oC.m_arfMatrix[(u8I * COLS2) + u8J] = fSum;
        }
    }

    return oC;
}

#endif





