/* Copyright 2022-2024 The MathWorks, Inc. */

/****************************************************
*                                                   *   
* Wrapper functions for CMSIS Custom functions             *
*                                                   *  
****************************************************/

#ifndef MW_CMSIS_IMPL_H
#define MW_CMSIS_IMPL_H

#include "arm_math.h"
#include "rtwtypes.h"
#include "dsp/matrix_functions.h"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define mw_arm_add_q15_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits) \
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q15_t* pSrcAptr = pSrcA; \
  q15_t* pSrcBptr = pSrcB; \
  q15_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) + (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
        *pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) + (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) + (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) + (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++)  + (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) + (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) + (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) + (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) + (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) + (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	blkCnt--; \
  	} \
  } \
} while (0)

#define mw_arm_sub_q15_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits)\
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q15_t* pSrcAptr = pSrcA; \
  q15_t* pSrcBptr = pSrcB; \
  q15_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) - (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) - (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) - (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) - (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)(*pSrcAptr++) - (q31_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),16); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) - (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) - (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) - (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) - (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q15_t) __SSAT((((q31_t)((*pSrcAptr++) << shiftBitsF) - (q31_t)(*pSrcBptr++)) >> shiftBitsF),16); \
    	blkCnt--; \
  	} \
  } \
} while (0)

#if defined (ARM_MATH_DSP)

#define mw_arm_add_q7_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits) \
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q7_t* pSrcAptr = pSrcA; \
  q7_t* pSrcBptr = pSrcB; \
  q7_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  q31_t out1,out2; \
  q15_t srcA, srcB; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out1 = (__PKHBT(srcA + (srcB << shiftBitsF), (q15_t)(*pSrcAptr++)+(q15_t)(*pSrcBptr++ << shiftBitsF), 16)); \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out2 = (__PKHBT(srcA + (srcB << shiftBitsF), (q15_t)(*pSrcAptr++)+(q15_t)(*pSrcBptr++ << shiftBitsF), 16)); \
    	*__SIMD32(pDstptr)++ = __PACKq7(__SSAT(((q15_t)out1 >> shiftBitsF), 8), __SSAT((out1 >> (16 + shiftBitsF)), 8) , __SSAT(((q15_t)out2 >> shiftBitsF), 8), __SSAT((out2 >> (16 + shiftBitsF)), 8)); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
		*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++)  + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out1 = (__PKHBT((srcA << shiftBitsF)+ srcB, (q15_t)(*pSrcAptr++ << shiftBitsF)+(q15_t)(*pSrcBptr++), 16)); \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
		out2 = (__PKHBT((srcA << shiftBitsF)+ srcB, (q15_t)(*pSrcAptr++ << shiftBitsF)+(q15_t)(*pSrcBptr++), 16)); \
		*__SIMD32(pDstptr)++ = __PACKq7(__SSAT(((q15_t)out1 >> shiftBitsF), 8), __SSAT((out1 >> (16 + shiftBitsF)), 8) , __SSAT(((q15_t)out2 >> shiftBitsF), 8), __SSAT((out2 >> (16 + shiftBitsF)), 8)); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
} while (0)

#define mw_arm_sub_q7_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits)\
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q7_t* pSrcAptr = pSrcA; \
  q7_t* pSrcBptr = pSrcB; \
  q7_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  q31_t out1,out2; \
  q15_t srcA, srcB; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
	    srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out1 = (__PKHBT(srcA - (srcB << shiftBitsF), (q15_t)(*pSrcAptr++)-(q15_t)(*pSrcBptr++ << shiftBitsF), 16)); \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out2 = (__PKHBT(srcA - (srcB << shiftBitsF), (q15_t)(*pSrcAptr++)-(q15_t)(*pSrcBptr++ << shiftBitsF), 16)); \
    	*__SIMD32(pDstptr)++ = __PACKq7(__SSAT(((q15_t)out1 >> shiftBitsF), 8), __SSAT((out1 >> (16 + shiftBitsF)), 8) , __SSAT(((q15_t)out2 >> shiftBitsF), 8), __SSAT((out2 >> (16 + shiftBitsF)), 8)); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
		*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++)  - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
    	out1 = (__PKHBT((srcA << shiftBitsF)- srcB, (q15_t)(*pSrcAptr++ << shiftBitsF)-(q15_t)(*pSrcBptr++), 16)); \
    	srcA = (q15_t)(*pSrcAptr++); \
    	srcB = (q15_t)(*pSrcBptr++); \
		out2 = (__PKHBT((srcA << shiftBitsF)- srcB, (q15_t)(*pSrcAptr++ << shiftBitsF)-(q15_t)(*pSrcBptr++), 16)); \
		*__SIMD32(pDstptr)++ = __PACKq7(__SSAT(((q15_t)out1 >> shiftBitsF), 8), __SSAT((out1 >> (16 + shiftBitsF)), 8) , __SSAT(((q15_t)out2 >> shiftBitsF), 8), __SSAT((out2 >> (16 + shiftBitsF)), 8)); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
} while (0)

#else

#define mw_arm_add_q7_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits) \
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q7_t* pSrcAptr = pSrcA; \
  q7_t* pSrcBptr = pSrcB; \
  q7_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++)  + (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) + (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
} while (0)

#define mw_arm_sub_q7_fractional(pSrcA, pSrcB, pDst, blockSize, shiftBits)\
do { \
  uint32_t blkCnt; \
  blkCnt = blockSize >> 2u; \
  q7_t* pSrcAptr = pSrcA; \
  q7_t* pSrcBptr = pSrcB; \
  q7_t* pDstptr = pDst; \
  int8_t shiftBitsF = shiftBits; \
  if(shiftBitsF > 0){ \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++) - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)(*pSrcAptr++)  - (q15_t)((*pSrcBptr++) << shiftBitsF)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
  else { \
    shiftBitsF = -shiftBits; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
     	blkCnt--; \
  	} \
  	blkCnt = blockSize % 0x4u; \
  	while(blkCnt > 0u){ \
    	*pDstptr++ = (q7_t) __SSAT((((q15_t)((*pSrcAptr++) << shiftBitsF) - (q15_t)(*pSrcBptr++)) >> shiftBitsF),8); \
    	blkCnt--; \
  	} \
  } \
} while (0)
#endif

inline static void mw_arm_mat_mult_f32(float32_t *pIn1, float32_t * pIn2, float32_t * pOut, uint16_t M, uint16_t N, uint16_t K) {
  arm_matrix_instance_f32 srcA = {M, N, (float32_t *)pIn1};
  arm_matrix_instance_f32 srcB = {N, K, (float32_t *)pIn2};
  arm_matrix_instance_f32 dstC = {M, K, (float32_t *)pOut};
  arm_mat_mult_f32(&srcA, &srcB, &dstC);
}


inline static void apply_shift_positive_q7(q7_t *dst, q31_t val, int8_t sh) {
    *dst = __SSAT(val >> sh, 8);
}

inline static void apply_shift_negative_q7(q7_t *dst, q31_t val, int8_t sh) {
    *dst = __SSAT(val << -sh, 8);
}

inline static void mw_arm_mat_mult_q7_impl(const arm_matrix_instance_q7 *matrixA, const arm_matrix_instance_q7 *matrixB, arm_matrix_instance_q7 *outputMatrix, int8_t shiftValue) {
    q31_t productSum; // Holds the intermediate sum of the product
    q7_t *matrixARowPtr = matrixA->pData; // Pointer to the current row of matrix A
    q7_t *matrixBColPtr =  matrixB->pData; // Pointer to the current column of matrix B
    q7_t *matrixADataPtr = matrixA->pData; // Pointer to the data of matrix A
    q7_t *matrixBDataPtr = matrixB->pData; // Pointer to the data of matrix B
    q7_t *outputDataPtr = outputMatrix->pData;  // Pointer to the output data matrix
    q7_t *outputTempPtr;                  // Temporary output data pointer for the current row
    uint16_t matrixBCols = matrixB->numCols; // Number of columns in matrix B
    uint16_t matrixACols = matrixA->numCols; // Number of columns in matrix A
    uint16_t matrixARows = matrixA->numRows; // Number of rows in matrix A
    uint16_t currentColumn, index = 0U, currentRow = matrixARows, columnCount; // Loop counters
    void (*apply_shift)(q7_t *, q31_t, int8_t);

    /* Assign the appropriate function to the function pointer */
    if (shiftValue >= 0) {
        apply_shift = apply_shift_positive_q7;
    } else {
        apply_shift = apply_shift_negative_q7;
    }

    // Loop through each row of matrix A using a do-while loop
        do {
            // Set the output pointer to the start of the current row
            outputTempPtr = outputDataPtr + index;

            // Initialize column counter for matrix B
            currentColumn = matrixBCols;
            // Reset matrix B pointer to the start for each new row of matrix A
            matrixBColPtr = matrixB->pData;

            // Loop through each column of matrix B using a do-while loop
            if (currentColumn > 0U) { // Ensure that there is at least one column to process
                do {
                    // Initialize sum to zero for each element in the result matrix
                    productSum = 0;
                    // Reset matrix A pointer to the start of the current row
                    matrixARowPtr = matrixADataPtr;

                    // Perform multiply-accumulate for each element
                    columnCount = matrixACols;
                    while (columnCount > 0U) {
                        // Multiply and accumulate
                        productSum += (q31_t)*matrixARowPtr++ * *matrixBColPtr;
                        // Move to the next element in the current column of matrix B
                        matrixBColPtr += matrixBCols;
                        // Decrement counter for matrix A columns
                        columnCount--;
                    }
                    // Apply the predetermined shift operation and saturate and store the result
                     apply_shift(outputTempPtr++, productSum, shiftValue);

                    // Move to the next column
                    currentColumn--;
                    // Update matrix B pointer to the start of the next column
                    matrixBColPtr = matrixBDataPtr + (matrixBCols - currentColumn);
                } while (currentColumn > 0U); // Check the condition at the end of the loop
            }

            // Move to the next row by updating the row pointers and counters
            index += matrixBCols;
            matrixADataPtr += matrixACols;
            // Decrement the row counter
            currentRow--;
        } while (currentRow > 0U); // Check the condition at the end of the loop
}

inline static void mw_arm_mat_mult_q7(q7_t *pIn1, q7_t *pIn2, q7_t *pOut, uint16_t M, uint16_t N, uint16_t K, int8_t shiftValue) {
  arm_matrix_instance_q7 srcA = {M, N, (q7_t *)pIn1};
  arm_matrix_instance_q7 srcB = {N, K, (q7_t *)pIn2};
  arm_matrix_instance_q7 dstC = {M, K, (q7_t *)pOut};
  mw_arm_mat_mult_q7_impl(&srcA, &srcB, &dstC, shiftValue);
}

inline static void apply_shift_positive_q15(q15_t *dst, q63_t val, int8_t sh) {
    *dst = __SSAT(val >> sh, 16);
}

inline static void apply_shift_negative_q15(q15_t *dst, q63_t val, int8_t sh) {
    *dst = __SSAT(val << -sh, 16);
}


inline static void mw_arm_mat_mult_q15_impl(
  const arm_matrix_instance_q15 *pMatrixA,
  const arm_matrix_instance_q15 *pMatrixB,
        arm_matrix_instance_q15 *pMatrixResult,
        q15_t *pTransposeBuffer,
        int8_t shiftValue)
{
    void (*apply_shift)(q15_t *, q63_t, int8_t);

    /* Assign the appropriate function to the function pointer */
    if (shiftValue >= 0) {
        apply_shift = apply_shift_positive_q15;
    } else {
        apply_shift = apply_shift_negative_q15;
    }

    q63_t productAccumulator;                      /* Product accumulator for matrix multiplication */

    #if defined (ARM_MATH_DSP)                     /* Only for cores that support DSP instructions */

    q15_t *pTransposedB = pTransposeBuffer;        /* Buffer to store the transposed matrix B */
    q15_t *pInputA = pMatrixA->pData;              /* Pointer to the data of matrix A */
    q15_t *pInputB = pMatrixB->pData;              /* Pointer to the data of matrix B */
    q15_t *pOutput;                                /* Pointer to the result data */
    uint16_t rowCountA = pMatrixA->numRows;        /* Row count of matrix A */
    uint16_t colCountB = pMatrixB->numCols;        /* Column count of matrix B */
    uint16_t colCountA = pMatrixA->numCols;        /* Column count of matrix A */
    uint16_t rowCountB = pMatrixB->numRows;        /* Row count of matrix B */
    uint32_t currentCol, index = 0U, currentRow = rowCountB, colIterator; /* Iteration counters */

    q31_t inputAVal1, inputBVal1, inputAVal2, inputBVal2;
    arm_matrix_instance_q15 transposedB;

    transposedB.numRows = colCountB;
    transposedB.numCols = rowCountB;
    transposedB.pData = pTransposedB;

    // Transpose matrix B to align for efficient memory access
    arm_mat_trans_q15(pMatrixB, &transposedB);

    // Initialize for the multiplication process
    currentRow = rowCountA;
    index = 0U;
    pOutput = pMatrixResult->pData;

    /* Perform matrix multiplication for each row of matrix A and each column of matrix B */
    /* Iterate over the rows of matrix A */
    do
    {
      // Initialize column counter for every row
      currentCol = colCountB;

      // Reset the pointer to the beginning of the transposed matrix B data
      pInputB = pTransposedB;

      /* Iterate over the columns of matrix B */
      do
      {
        // Reset the product accumulator for each element in the result matrix
        productAccumulator = 0;

        // Set the pointer to the current element of matrix A
        pInputA = pMatrixA->pData + index;

        // Compute 4 multiplications at a time for efficiency
        colIterator = colCountA >> 2U;

        /* Matrix multiplication core computation */
        while (colIterator > 0U)
        {
          // Load values from matrices A and B
          inputAVal1 = read_q15x2_ia (&pInputA);
          inputBVal1 = read_q15x2_ia (&pInputB);

          inputAVal2 = read_q15x2_ia (&pInputA);
          inputBVal2 = read_q15x2_ia (&pInputB);

          // Perform multiply-accumulate operations
          productAccumulator = __SMLALD(inputAVal1, inputBVal1, productAccumulator);
          productAccumulator = __SMLALD(inputAVal2, inputBVal2, productAccumulator);

          // Decrement the loop counter
          colIterator--;
        }

        // Process remaining columns if any
        colIterator = colCountA & 0x3U;

        while (colIterator > 0U)
        {
          // Perform multiply-accumulate operations for the remaining elements
          productAccumulator += *pInputA++ * *pInputB++;

          // Decrement the loop counter

          colIterator--;
        }
        // Apply the shift function to the product accumulator and store the result
        apply_shift(pOutput++, productAccumulator, shiftValue);

        // Decrement the column counter
        currentCol--;

      } while (currentCol > 0U);

      // Move to the next row in matrix A
      index = index + colCountA;

      // Decrement the row counter
      currentRow--;

    } while (currentRow > 0U);

#else /* #if defined (ARM_MATH_DSP) */

    q15_t *pInputA = pMatrixA->pData;              /* Pointer to the data of matrix A */
    q15_t *pInputB = pMatrixB->pData;              /* Pointer to the data of matrix B */
    q15_t *pCurrentInputA = pMatrixA->pData;       /* Current pointer to matrix A data */
    q15_t *pCurrentInputB = pMatrixB->pData;       /* Current pointer to matrix B data */
    q15_t *pResultData = pMatrixResult->pData;     /* Pointer to the result data */
    q15_t *pCurrentOutput;                         /* Current pointer to the output data */
    uint16_t colCountB = pMatrixB->numCols;        /* Column count of matrix B */
    uint16_t colCountA = pMatrixA->numCols;        /* Column count of matrix A */
    uint16_t rowCountA = pMatrixA->numRows;        /* Row count of matrix A */
    uint32_t currentCol, index = 0U, currentRow = rowCountA, colIterator; /* Iteration counters */
    (void)pTransposeBuffer;                        /* Unused in this context */

    /* Perform matrix multiplication for each row of matrix A and each column of matrix B */
    /* Iterate over the rows of matrix A */
    do
    {
      // Set the output pointer to the start of the current row
      pCurrentOutput = pResultData + index;

      // Initialize column counter for every row
      currentCol = colCountB;

      // Reset the pointer to the beginning of matrix B data
      pInputB = pMatrixB->pData;

      /* Iterate over the columns of matrix B */
      do
      {
        // Reset the product accumulator for each element in the result matrix
        productAccumulator = 0;

        // Set the pointer to the current element of matrix A
        pInputA = pCurrentInputA;

        // Perform the dot-product for the current row and column
        colIterator = colCountA;

        /* Matrix multiplication core computation */
        while (colIterator > 0U)
        {
          // Perform multiply-accumulate operations
          productAccumulator += (q31_t) * pInputA++ * *pInputB;
          pInputB += colCountB;

          // Decrement the loop counter
          colIterator--;
        }

        // Apply the shift function to the product accumulator and store the result
        apply_shift(pCurrentOutput++, productAccumulator, shiftValue);

        // Decrement the column counter
        currentCol--;

        // Update the pointer to the start of the next column in matrix B
        pInputB = pCurrentInputB + (colCountB - currentCol);

      } while (currentCol > 0U);

      // Move to the next row in matrix A
      index = index + colCountB;
      pCurrentInputA = pCurrentInputA + colCountA;

      // Decrement the row counter
      currentRow--;

    } while (currentRow > 0U);

#endif /* #if defined (ARM_MATH_DSP) */

}

inline static void mw_arm_mat_mult_q15(q15_t *pIn1, q15_t *pIn2, q15_t *pOut, uint16_t M, uint16_t N, uint16_t K, int8_t shiftValue) {
  arm_matrix_instance_q15 srcA = {M, N, (q15_t *)pIn1};
  arm_matrix_instance_q15 srcB = {N, K, (q15_t *)pIn2};
  arm_matrix_instance_q15 dstC = {M, K, (q15_t *)pOut};
  q15_t tmp[N * K]; 
  mw_arm_mat_mult_q15_impl(&srcA, &srcB, &dstC, tmp, shiftValue);
}
    
#define mw_arm_mat_trans_q7(pIn, pOut, M, N) {\
  arm_matrix_instance_q7 src = {M, N, (q7_t *)pIn};\
  arm_matrix_instance_q7 dst = {N, M, (q7_t *)pOut};\
  arm_mat_trans_q7(&src, &dst);\
}

#define mw_arm_mat_trans_q15(pIn, pOut, M, N) {\
  arm_matrix_instance_q15 src = {M, N, (q15_t *)pIn};\
  arm_matrix_instance_q15 dst = {N, M, (q15_t *)pOut};\
  arm_mat_trans_q15(&src, &dst);\
}

#define mw_arm_mat_trans_q31(pIn, pOut, M, N) {\
  arm_matrix_instance_q31 src = {M, N, (q31_t *)pIn};\
  arm_matrix_instance_q31 dst = {N, M, (q31_t *)pOut};\
  arm_mat_trans_q31(&src, &dst);\
}

#define mw_arm_mat_trans_f32(pIn, pOut, M, N) {\
  arm_matrix_instance_f32 src = {M, N, (float32_t *)pIn};\
  arm_matrix_instance_f32 dst = {N, M, (float32_t *)pOut};\
  arm_mat_trans_f32(&src, &dst);\
}

#define mw_arm_mat_cmplx_trans_q15(pIn, pOut, M, N) {\
  arm_matrix_instance_q15 src = {M, N, (q15_t *)pIn};\
  arm_matrix_instance_q15 dst = {N, M, (q15_t *)pOut};\
  arm_mat_cmplx_trans_q15(&src, &dst);\
}

#define mw_arm_mat_cmplx_trans_q31(pIn, pOut, M, N) {\
  arm_matrix_instance_q31 src = {M, N, (q31_t *)pIn};\
  arm_matrix_instance_q31 dst = {N, M, (q31_t *)pOut};\
  arm_mat_cmplx_trans_q31(&src, &dst);\
}

#define mw_arm_mat_cmplx_trans_f32(pIn, pOut, M, N) {\
  arm_matrix_instance_f32 src = {M, N, (float32_t *)pIn};\
  arm_matrix_instance_f32 dst = {N, M, (float32_t *)pOut};\
  arm_mat_cmplx_trans_f32(&src, &dst);\
}

#define mw_arm_mat_cmplx_mult_f32(pIn1, pIn2, pOut, M, N, K) {\
  arm_matrix_instance_f32 srcA = {M, N, (float32_t *)pIn1};\
  arm_matrix_instance_f32 srcB = {N, K, (float32_t *)pIn2};\
  arm_matrix_instance_f32 dstC = {M, K, (float32_t *)pOut};\
  arm_mat_cmplx_mult_f32(&srcA, &srcB, &dstC);\
}

inline static void  arm_mat_cmplx_mult_q15_impl(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
        arm_matrix_instance_q15 * pDst,
        q15_t                   * pScratch,
        int8_t                    shiftValue)
{
        void (*apply_shift)(q15_t *, q63_t, int8_t);

        /* Assign the appropriate function to the function pointer */
        if (shiftValue >= 0) {
            apply_shift = apply_shift_positive_q15;
        } else {
            apply_shift = apply_shift_negative_q15;
        }

        q15_t *pSrcBT = pScratch;                      /* input data matrix pointer for transpose */
        q15_t *pInA = pSrcA->pData;                    /* input data matrix pointer A of Q15 type */
        q15_t *pInB = pSrcB->pData;                    /* input data matrix pointer B of Q15 type */
        q15_t *px;                                     /* Temporary output data matrix pointer */
        uint16_t numRowsA = pSrcA->numRows;            /* number of rows of input matrix A */
        uint16_t numColsB = pSrcB->numCols;            /* number of columns of input matrix B */
        uint16_t numColsA = pSrcA->numCols;            /* number of columns of input matrix A */
        uint16_t numRowsB = pSrcB->numRows;            /* number of rows of input matrix A */
        q63_t sumReal, sumImag;                        /* accumulator */
        uint32_t col, i = 0U, row = numRowsB, colCnt;  /* Loop counters */

#if defined (ARM_MATH_DSP)
        q31_t prod1, prod2;
        q31_t pSourceA, pSourceB;
#else
        q15_t a, b, c, d;
#endif /* #if defined (ARM_MATH_DSP) */

    /* Matrix transpose */
    do
    {
      /* The pointer px is set to starting address of column being processed */
      px = pSrcBT + i;

      /* Apply loop unrolling and exchange the columns with row elements */
      col = numColsB >> 2;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
         a second loop below computes the remaining 1 to 3 samples. */
      while (col > 0U)
      {
        /* Read two elements from row */
        write_q15x2 (px, read_q15x2_ia (&pInB));

        /* Update pointer px to point to next row of transposed matrix */
        px += numRowsB * 2;

        /* Read two elements from row */
        write_q15x2 (px, read_q15x2_ia (&pInB));

        /* Update pointer px to point to next row of transposed matrix */
        px += numRowsB * 2;

        /* Read two elements from row */
        write_q15x2 (px, read_q15x2_ia (&pInB));

        /* Update pointer px to point to next row of transposed matrix */
        px += numRowsB * 2;

        /* Read two elements from row */
        write_q15x2 (px, read_q15x2_ia (&pInB));

        /* Update pointer px to point to next row of transposed matrix */
        px += numRowsB * 2;

        /* Decrement column loop counter */
        col--;
      }

      /* If the columns of pSrcB is not a multiple of 4, compute any remaining output samples here.
       ** No loop unrolling is used. */
      col = numColsB % 0x4U;


      while (col > 0U)
      {
        /* Read two elements from row */
        write_q15x2 (px, read_q15x2_ia (&pInB));

        /* Update pointer px to point to next row of transposed matrix */
        px += numRowsB * 2;

        /* Decrement column loop counter */
        col--;
      }

      i = i + 2U;

      /* Decrement row loop counter */
      row--;

    } while (row > 0U);

    /* Reset variables for usage in following multiplication process */
    row = numRowsA;
    i = 0U;
    px = pDst->pData;

    /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
    /* row loop */
    do
    {
      /* For every row wise process, column loop counter is to be initiated */
      col = numColsB;

      /* For every row wise process, pIn2 pointer is set to starting address of transposed pSrcB data */
      pInB = pSrcBT;

      /* column loop */
      do
      {
        /* Set variable sum, that acts as accumulator, to zero */
        sumReal = 0;
        sumImag = 0;

        /* Initiate pointer pInA to point to starting address of column being processed */
        pInA = pSrcA->pData + i * 2;

        /* Apply loop unrolling and compute 2 MACs simultaneously. */
        colCnt = numColsA >> 1U;

        /* matrix multiplication */
        while (colCnt > 0U)
        {
          /* c(m,n) = a(1,1) * b(1,1) + a(1,2) * b(2,1) + .... + a(m,p) * b(p,n) */

#if defined (ARM_MATH_DSP)

          /* read real and imag values from pSrcA and pSrcB buffer */
          pSourceA = read_q15x2_ia (&pInA);
          pSourceB = read_q15x2_ia (&pInB);

          /* Multiply and Accumlates */
#ifdef ARM_MATH_BIG_ENDIAN
          prod1 = -__SMUSD(pSourceA, pSourceB);
#else
          prod1 = __SMUSD(pSourceA, pSourceB);
#endif
          prod2 = __SMUADX(pSourceA, pSourceB);
          sumReal += (q63_t) prod1;
          sumImag += (q63_t) prod2;

          /* read real and imag values from pSrcA and pSrcB buffer */
          pSourceA = read_q15x2_ia (&pInA);
          pSourceB = read_q15x2_ia (&pInB);

          /* Multiply and Accumlates */
#ifdef ARM_MATH_BIG_ENDIAN
          prod1 = -__SMUSD(pSourceA, pSourceB);
#else
          prod1 = __SMUSD(pSourceA, pSourceB);
#endif
          prod2 = __SMUADX(pSourceA, pSourceB);
          sumReal += (q63_t) prod1;
          sumImag += (q63_t) prod2;

#else /* #if defined (ARM_MATH_DSP) */

          /* read real and imag values from pSrcA buffer */
          a = *pInA;
          b = *(pInA + 1U);
          /* read real and imag values from pSrcB buffer */
          c = *pInB;
          d = *(pInB + 1U);

          /* Multiply and Accumlates */
          sumReal += (q31_t) a *c;
          sumImag += (q31_t) a *d;
          sumReal -= (q31_t) b *d;
          sumImag += (q31_t) b *c;

          /* read next real and imag values from pSrcA buffer */
          a = *(pInA + 2U);
          b = *(pInA + 3U);
          /* read next real and imag values from pSrcB buffer */
          c = *(pInB + 2U);
          d = *(pInB + 3U);

          /* update pointer */
          pInA += 4U;

          /* Multiply and Accumlates */
          sumReal += (q31_t) a * c;
          sumImag += (q31_t) a * d;
          sumReal -= (q31_t) b * d;
          sumImag += (q31_t) b * c;
          /* update pointer */
          pInB += 4U;

#endif /* #if defined (ARM_MATH_DSP) */

          /* Decrement loop counter */
          colCnt--;
        }

        /* process odd column samples */
        if ((numColsA & 0x1U) > 0U)
        {
          /* c(m,n) = a(1,1) * b(1,1) + a(1,2) * b(2,1) + .... + a(m,p) * b(p,n) */

#if defined (ARM_MATH_DSP)
          /* read real and imag values from pSrcA and pSrcB buffer */
          pSourceA = read_q15x2_ia (&pInA);
          pSourceB = read_q15x2_ia (&pInB);

          /* Multiply and Accumlates */
#ifdef ARM_MATH_BIG_ENDIAN
          prod1 = -__SMUSD(pSourceA, pSourceB);
#else
          prod1 = __SMUSD(pSourceA, pSourceB);
#endif
          prod2 = __SMUADX(pSourceA, pSourceB);
          sumReal += (q63_t) prod1;
          sumImag += (q63_t) prod2;

#else /* #if defined (ARM_MATH_DSP) */

          /* read real and imag values from pSrcA and pSrcB buffer */
          a = *pInA++;
          b = *pInA++;
          c = *pInB++;
          d = *pInB++;

          /* Multiply and Accumlates */
          sumReal += (q31_t) a * c;
          sumImag += (q31_t) a * d;
          sumReal -= (q31_t) b * d;
          sumImag += (q31_t) b * c;

#endif /* #if defined (ARM_MATH_DSP) */

        }

        /* Saturate and store result in destination buffer */
            apply_shift(px++, sumReal, shiftValue);
            apply_shift(px++, sumImag, shiftValue);
        // *px++ = (q15_t) (__SSAT(sumReal >> 15, 16));
        // *px++ = (q15_t) (__SSAT(sumImag >> 15, 16));

        /* Decrement column loop counter */
        col--;

      } while (col > 0U);

      i = i + numColsA;

      /* Decrement row loop counter */
      row--;

    } while (row > 0U);

}

#define mw_arm_mat_cmplx_mult_q15(pIn1, pIn2, pOut, M, N, K, shiftValue) {\
  arm_matrix_instance_q15 srcA = {M, N, (q15_t *)pIn1};\
  arm_matrix_instance_q15 srcB = {N, K, (q15_t *)pIn2};\
  arm_matrix_instance_q15 dstC = {N, M, (q15_t *)pOut};\
  q15_t tmp[N * K * 2]; \
  arm_mat_cmplx_mult_q15_impl(&srcA, &srcB, &dstC, tmp, shiftValue);\
}
#endif
