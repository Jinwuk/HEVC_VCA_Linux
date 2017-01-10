/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
	m_piYuvExt = NULL;

#if ETRI_MODIFICATION_V02
	em_DbgInfo = nullptr;
	em_uiHADLuma 	= 0;
	em_uiHADCb		= 0;
	em_uiHADCr  		= 0;
#endif
}

TComPrediction::~TComPrediction()
{
  
  delete[] m_piYuvExt;

  m_acYuvPred[0].destroy();
  m_acYuvPred[1].destroy();

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
  }
  
  Int i, j;
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff()
{
  if( m_piYuvExt == NULL )
  {
    Int extWidth  = MAX_CU_SIZE + 16; 
    Int extHeight = MAX_CU_SIZE + 1;
    Int i, j;
    for (i = 0; i < 4; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7);
      for (j = 0; j < 4; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight);
      }
    }
    m_iYuvExtHeight  = ((MAX_CU_SIZE + 2) << 4);
    m_iYuvExtStride = ((MAX_CU_SIZE  + 8) << 4);
    m_piYuvExt = new Int[ m_iYuvExtStride * m_iYuvExtHeight ];

    // new structure
    m_acYuvPred[0] .create( MAX_CU_SIZE, MAX_CU_SIZE );
    m_acYuvPred[1] .create( MAX_CU_SIZE, MAX_CU_SIZE );

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE );
  }

  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// ETRI SIMD Intra functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
#if ETRI_SIMD_INTRA
#if ETRI_SIMD_FIX_INTRA_DC
Pel TComPrediction::predIntraGetPredValDC(Int* pSrc, Int* pLSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft)
{
    assert(iWidth > 0 && iHeight > 0);
    Int iInd, iSum = 0;
    Pel pDcVal;

    __m128i xmmSum = _mm_setzero_si128();
    ALIGNED(32) Int temp[8];
    if (bAbove)
    {
        Int* pTSrc = pSrc - iSrcStride;
        if(iWidth == 4)
            xmmSum = _mm_loadu_si128((__m128i const *)(pTSrc));
        else if(iWidth == 8)
        {
            xmmSum = _mm_loadu_si128((__m128i const *)(pTSrc));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
        }
        else if(iWidth == 16)
        {
            xmmSum = _mm_loadu_si128((__m128i const *)(pTSrc));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
        }
        else if(iWidth == 32)
        {
            xmmSum = _mm_loadu_si128((__m128i const *)(pTSrc));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
        }
        else if(iWidth == 64)
        {
            xmmSum = _mm_loadu_si128((__m128i const *)(pTSrc));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pTSrc+=4)));
        }
    }
    if (bLeft)
    {

        xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc)));
        if (iHeight >= 8)
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
        if (iHeight >= 16)
        {
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
        }
        if (iHeight >= 32)
        {
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
        }
        if (iHeight >= 64)
        {
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
            xmmSum = _mm_add_epi32(xmmSum, _mm_loadu_si128((__m128i const *)(pLSrc+=4)));
        }
    }
    _mm_storeu_si128((__m128i *)temp, xmmSum);
    for (iInd = 0; iInd < 4; iInd++)
        iSum += temp[iInd];

    if (bAbove && bLeft)
    {
        pDcVal = (iSum + iWidth) / (iWidth + iHeight);
    }
    else if (bAbove)
    {
        pDcVal = (iSum + iWidth/2) / iWidth;
    }
    else if (bLeft)
    {
        pDcVal = (iSum + iHeight / 2) / iHeight;
    }
    else
    {
        pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
    }

    return pDcVal;
}
#else 
Pel TComPrediction::predIntraGetPredValDC( Int* pSrc, Int* pColPixs, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft )
	{
	  Int iSum = 0;
	  Pel pDcVal;

	 ALIGNED(16) short eSum[8]={0};
	  __m128i xmm0 = _mm_setzero_si128();
	  if (bAbove)
	  {
		  pSrc = pSrc-iSrcStride;
		  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)(pSrc+4)))); //iWidth = 4 or 8
	
		  if(iWidth==16) // iInd = 0, 8
		  {
			  //	 xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc+0-iSrcStride)),_mm_loadu_si128((__m128i *)(pSrc+0+4-iSrcStride))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
		  }
		  else if(iWidth==32) // iInd = 0, 8, 16, 24
		  {
			  //	  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc+0-iSrcStride)),_mm_loadu_si128((__m128i *)(pSrc+0+4-iSrcStride))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
		  }
		  else if(iWidth==64) // iInd = 0, 8, 16, 24, 32, 40, 48, 56
		  {
			  //	  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc+0-iSrcStride)),_mm_loadu_si128((__m128i *)(pSrc+0+4-iSrcStride))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pSrc)),_mm_loadu_si128((__m128i *)((pSrc+=8)+4))));
		  }
	
	  }
	  if (bLeft)
	  {
		  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)(pColPixs+4)))); 
	
		  if(iWidth==16) // iInd = 0, 8, 
		  {
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
		  }
		  else if(iWidth==32) // iInd = 0, 8, 16, 24, 
		  {
			  //	  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs+0)),_mm_loadu_si128((__m128i *)(pColPixs+0+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
	
		  }
		  else if(iWidth==64) // iInd = 0, 8, 16, 24, 32, 40, 48, 56
		  {
			  //	  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs+0)),_mm_loadu_si128((__m128i *)(pColPixs+0+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
			  xmm0 = _mm_add_epi16(xmm0,_mm_packs_epi32(_mm_loadu_si128((__m128i *)(pColPixs)),_mm_loadu_si128((__m128i *)((pColPixs+=8)+4)))); 
		  }
	  }
	  if(iWidth==4)
	  {
		  _mm_store_si128((__m128i *)eSum,xmm0);
		  iSum = eSum[0] +	eSum[1] +  eSum[2] +  eSum[3];
	  }
	  else
	  {
		  _mm_store_si128((__m128i *)eSum,xmm0);
		  iSum = eSum[0] +	eSum[1] +  eSum[2] +  eSum[3] + eSum[4] +  eSum[5] +  eSum[6] +  eSum[7];
	  }
	
	  //if(iSum > 65535) printf("Overflow!!\n");
	
	  if (bAbove && bLeft)
	  {
		pDcVal = (iSum + iWidth) / (iWidth + iHeight);
	  }
	  else if (bAbove)
	  {
		pDcVal = (iSum + iWidth/2) / iWidth;
	  }
	  else if (bLeft)
	  {
		pDcVal = (iSum + iHeight/2) / iHeight;
	  }
	  else
	  {
		pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
	  }
	  
	  return pDcVal;
	}
#endif 
// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 *
 * This functions are implemented on ETRI's SIMD code.
 */
Void TComPrediction::predIntraLumaAng(TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
	Pel *pDst = piPred;
	Int *ptrSrc;

	assert(g_aucConvertToBit[iWidth] >= 0); //   4x  4
	assert(g_aucConvertToBit[iWidth] <= 5); // 128x128
	assert(iWidth == iHeight);

	ptrSrc = pcTComPattern->getPredictorPtr(uiDirMode, g_aucConvertToBit[iWidth] + 2, m_piYuvExt);

	// get starting pixel in block
	Int sw = 2 * iWidth + 1;

#if ETRI_SIMD_INTRA_FIX_COMPATIBILITY
    Int aiLSrc[MAX_CU_SIZE << 1] = { 0 };
    for( Int k = 0 ;k < iWidth << 1 ; k++ )
    {
        aiLSrc[k] = ptrSrc[(sw+1)+k*sw-1];
    }
#else 
	// ETRI Code : Set eColPixs 
	Int eColPixs[MAX_CU_SIZE*2];
	for(Int k=0;k<iWidth*2;k++)
	{
		eColPixs[k] = ptrSrc[(sw+1)+k*sw-1];
	}
#endif 
	// Create the prediction
	if (uiDirMode == PLANAR_IDX)
	{
		switch(iWidth)
		{
#if ETRI_SIMD_FIX_INTRA_PLANAR_PREDICTION
            case  4: xPredIntraPlanar4x4(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case  8: xPredIntraPlanar8x8(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case 16: xPredIntraPlanar16x16(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case 32: xPredIntraPlanar32x32(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case 64: xPredIntraPlanar64x64(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
#else 
			case 4: xPredIntraPlanar4( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 8: xPredIntraPlanar8( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 16: xPredIntraPlanar16( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 32: xPredIntraPlanar32( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 64: xPredIntraPlanar64( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
#endif 
		}
	}
	else
	{
		switch(iWidth)
		{
#if ETRI_SIMD_FIX_INTRA_ANGULAR_PREDICTION
             case 4 : xPredIntraAng4x4  (g_bitDepthY, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true ); break;
             case 8 : xPredIntraAng8x8  (g_bitDepthY, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true ); break;
             case 16: xPredIntraAng16x16(g_bitDepthY, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true ); break;
             case 32: xPredIntraAng32x32(g_bitDepthY, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false ); break;
             case 64: xPredIntraAng64x64(g_bitDepthY, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false ); break;
#else 
			case 4: xPredIntraAng4(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , true); break;
			case 8: xPredIntraAng8(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , true); break;
			case 16: xPredIntraAng16(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , true); break;
			case 32: xPredIntraAng32(g_bitDepthY, ptrSrc+sw+1, eColPixs, sw, pDst, uiStride, uiDirMode, bAbove, bLeft  ); break;
			case 64: xPredIntraAng64(g_bitDepthY, ptrSrc+sw+1, eColPixs, sw, pDst, uiStride, uiDirMode, bAbove, bLeft  ); break;
#endif 
		}
		if( (uiDirMode == DC_IDX ) && bAbove && bLeft && !( (iWidth > 16) || (iHeight > 16) ))
		{
			xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
		}
	}
}

/**
* Angular chroma
*
* This functions are implemented on ETRI's SIMD code.
*/
Void TComPrediction::predIntraChromaAng(Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
	Pel *pDst = piPred;
	Int *ptrSrc = piSrc;

	// get starting pixel in block
	Int sw = 2 * iWidth + 1;

#if ETRI_SIMD_INTRA_FIX_COMPATIBILITY
    Int aiLSrc[MAX_CU_SIZE];
    for( Int k = 0 ;k < iWidth << 1 ; k++ )
    {
        aiLSrc[k] = ptrSrc[(sw+1)+k*sw-1];
    }
#else 
	// ETRI code for SIMD Prediction
	Int eColPixs[MAX_CU_SIZE]={0};

	for(Int k=0;k<iWidth*2;k++)
	{
		eColPixs[k] = ptrSrc[(sw+1)+k*sw-1];
	}
#endif 
	if ( uiDirMode == PLANAR_IDX )
	{
		switch(iWidth)
		{
#if ETRI_SIMD_FIX_INTRA_PLANAR_PREDICTION
            case  4: xPredIntraPlanar4x4(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case  8: xPredIntraPlanar8x8(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
            case 16: xPredIntraPlanar16x16(ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride); break;
#else 
			case 4: xPredIntraPlanar4( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 8: xPredIntraPlanar8( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
			case 16:xPredIntraPlanar16( ptrSrc+sw+1, eColPixs, sw, pDst, uiStride ); break;
#endif 
		}
	}
	else
	{
		// Create the prediction
		switch(iWidth)
		{
#if ETRI_SIMD_FIX_INTRA_ANGULAR_PREDICTION
            case 4: xPredIntraAng4x4(g_bitDepthC, ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false); break;
            case 8: xPredIntraAng8x8(g_bitDepthC, ptrSrc + sw + 1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false); break;
            case 16: xPredIntraAng16x16(g_bitDepthC, ptrSrc+sw+1, aiLSrc, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false ); break;
#else 
			case 4: xPredIntraAng4(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , false); break;
			case 8: xPredIntraAng8(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , false); break;
			case 16: xPredIntraAng16(g_bitDepthY, ptrSrc+sw+1, eColPixs,  sw, pDst, uiStride, uiDirMode, bAbove, bLeft , false); break;
#endif 
		}
	}
}

#else
/**
-------------------------------------------------------------------------------------------------------------------------------------------------
	@brief 	HM Original function
-------------------------------------------------------------------------------------------------------------------------------------------------
*/
Pel TComPrediction::predIntraGetPredValDC( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft )
{
	assert(iWidth > 0 && iHeight > 0);
	Int iInd, iSum = 0;
	Pel pDcVal;

	if (bAbove)
	{
		for (iInd = 0;iInd < iWidth;iInd++)
		{
			iSum += pSrc[iInd-iSrcStride];
		}
	}
	if (bLeft)
	{
		for (iInd = 0;iInd < iHeight;iInd++)
		{
			iSum += pSrc[iInd*iSrcStride-1];
		}
	}

	if (bAbove && bLeft)
	{
		pDcVal = (iSum + iWidth) / (iWidth + iHeight);
	}
	else if (bAbove)
	{
		pDcVal = (iSum + iWidth/2) / iWidth;
	}
	else if (bLeft)
	{
		pDcVal = (iSum + iHeight/2) / iHeight;
	}
	else
	{
		pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
	}

	return pDcVal;
}

/**
-------------------------------------------------------------------------------------------------------------------------------------------------
	@brief 	HM Original function. It is called by the predIntraLumaAng and predIntraChromaAng
-------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TComPrediction::xPredIntraAng(Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
  Int k,l;
  Int blkSize        = width;
  Pel* pDst          = rpDst;

  // Map the mode index to main prediction direction and angle
  assert( dirMode > 0 ); //no planar
  Bool modeDC        = dirMode < 2;
  Bool modeHor       = !modeDC && (dirMode < 18);
  Bool modeVer       = !modeDC && !modeHor;
  Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
  Int absAng         = abs(intraPredAngle);
  Int signAng        = intraPredAngle < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
  Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
  Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
  Int invAngle       = invAngTable[absAng];
  absAng             = angTable[absAng];
  intraPredAngle     = signAng * absAng;

  // Do the DC prediction
  if (modeDC)
  {
    Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

    for (k=0;k<blkSize;k++)
    {
      for (l=0;l<blkSize;l++)
      {
        pDst[k*dstStride+l] = dcval;
      }
    }
  }

  // Do angular predictions
  else
  {
    Pel* refMain;
    Pel* refSide;
    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      for (k=0;k<blkSize+1;k++)
      {
        refAbove[k+blkSize-1] = pSrc[k-srcStride-1];
      }
      for (k=0;k<blkSize+1;k++)
      {
        refLeft[k+blkSize-1] = pSrc[(k-1)*srcStride-1];
      }
      refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
      refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (k=-1; k>blkSize*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (k=0;k<2*blkSize+1;k++)
      {
        refAbove[k] = pSrc[k-srcStride-1];
      }
      for (k=0;k<2*blkSize+1;k++)
      {
        refLeft[k] = pSrc[(k-1)*srcStride-1];
      }
      refMain = modeVer ? refAbove : refLeft;
      refSide = modeVer ? refLeft  : refAbove;
    }

    if (intraPredAngle == 0)
    {
      for (k=0;k<blkSize;k++)
      {
        for (l=0;l<blkSize;l++)
        {
          pDst[k*dstStride+l] = refMain[l+1];
        }
      }

      if ( bFilter )
      {
        for (k=0;k<blkSize;k++)
        {
          pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Int deltaPos=0;
      Int deltaInt;
      Int deltaFract;
      Int refMainIndex;

      for (k=0;k<blkSize;k++)
      {
        deltaPos += intraPredAngle;
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          for (l=0;l<blkSize;l++)
          {
            refMainIndex        = l+deltaInt+1;
            pDst[k*dstStride+l] = (Pel) ( ((32-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+16) >> 5 );
          }
        }
        else
        {
          // Just copy the integer samples
          for (l=0;l<blkSize;l++)
          {
            pDst[k*dstStride+l] = refMain[l+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (modeHor)
    {
      Pel  tmp;
      for (k=0;k<blkSize-1;k++)
      {
        for (l=k+1;l<blkSize;l++)
        {
          tmp                 = pDst[k*dstStride+l];
          pDst[k*dstStride+l] = pDst[l*dstStride+k];
          pDst[l*dstStride+k] = tmp;
        }
      }
    }
  }
}

/**
-------------------------------------------------------------------------------------------------------------------------------------------------
	@brief 	HM Original function : This function is called by TEncSearch functions
-------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TComPrediction::predIntraLumaAng(TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
	Pel *pDst = piPred;
	Int *ptrSrc;

	assert(g_aucConvertToBit[iWidth] >= 0); //   4x  4
	assert(g_aucConvertToBit[iWidth] <= 5); // 128x128
	assert(iWidth == iHeight);

	ptrSrc = pcTComPattern->getPredictorPtr(uiDirMode, g_aucConvertToBit[iWidth] + 2, m_piYuvExt);

	// get starting pixel in block
	Int sw = 2 * iWidth + 1;

//	EDPRINTF(stderr, "Actrive ???? \n");


	// Create the prediction
	if (uiDirMode == PLANAR_IDX)
	{
		xPredIntraPlanar(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
	}
	else
	{
		if ((iWidth > 16) || (iHeight > 16))
		{
			xPredIntraAng(g_bitDepthY, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false);
		}
		else
		{
			xPredIntraAng(g_bitDepthY, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true);

			if ((uiDirMode == DC_IDX) && bAbove && bLeft)
			{
				xDCPredFiltering(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
			}
		}
	}
}

/**
-------------------------------------------------------------------------------------------------------------------------------------------------
	@brief 	HM Original function : Angular chroma : : This function is called by TEncSearch functions
-------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TComPrediction::predIntraChromaAng(Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
	Pel *pDst = piPred;
	Int *ptrSrc = piSrc;

	// get starting pixel in block
	Int sw = 2 * iWidth + 1;

	if ( uiDirMode == PLANAR_IDX )
	{
		xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
	}
	else
	{
		// Create the prediction
		xPredIntraAng(g_bitDepthC, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false);
	}
}

#endif	/// ETRI_SIMD_INTRA_KJH @ 2015 8 12 by Seok

// ====================================================================================================================
// Finish of ETRI SIMD Intra  functions
// Beginning of Inter functions used in TEncSearch 
// ====================================================================================================================
// ====================================================================================================================
// ETRI FAST ALGORITHM 2015 by ETRI_MODIFICATION_V02
// ====================================================================================================================
#if ETRI_MODIFICATION_V02
/// 2014 6 25 by Seok

#define	ETRI_FastMC_inSKIPMerge_DEVCode0702	0	/// Original : 1 @ 2014 7 3 by Seok : xCheckIdenticalMotion( pcCU, uiPartAddr ) = TRUE 의 경우 Interpolation 분리를 테스트하기 위함 : 안정임
#define	ETRI_FastMC_inSKIPMerge_DEVCode0703	0	/// Original : 0 @ 2014 7 3 by Seok : B-Slice에서 Interpolation 분리를 테스트하기 위함. 앞에서와는 달리 분리 보다는 원래 코드가 Cache에서 유리하여 더 빠름

/** 
-----------------------------------------------------------------------------------------
	@fn    	Void TComPrediction::ETRI_HAD_getLevel
	@param 	UInt* puiLevelInfo   :  [0] : Width, [1]: Slice Depth, [2] : Cu Depth [3]: Level Luma, [4] :Level Cb [5]: Level Cr
	@param 	UInt  _rem   		: 
	@param 	Int _iPer  			: 
	@brief	uiLog2TrSize :   g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, x=32 -> 3 \
			Luma 32x32 의 경우 Chroma 16x16 이 되어 uiLog2Size는 \
			Luma의 경우 Log2(32/4)+2 = 5 Chroma의 경우 Log2(16/4)+2 =4 가 된다. \
			그러므로 14에 대하여 Chroma는 1을 빼주어야 한다.
-----------------------------------------------------------------------------------------
*/
/// 2015 3 29 by Seok
UInt TComPrediction::ETRI_HAD_getLevel (UInt*& uiInfo, UInt _rem, Int _iPer, UInt _remC, Int _iPerC )
{
 	UInt		LevelLuma, LevelCb, LevelCr, LevelEval;
	UInt	 	uiWidth = uiInfo[0];
	UInt	 	uiLog2TrSize = g_aucConvertToBit[ uiWidth ] + 2;
	Int 	 	uiQ  = g_quantScales[_rem];
	Int 	 	uiQc = g_quantScales[_remC];

	LevelLuma 	= (em_uiHADLuma * uiQ) >>(14 + _iPer  + uiLog2TrSize);
	LevelCb   	 	= (em_uiHADCb  * uiQc)>>(13 + _iPerC + uiLog2TrSize);	
	LevelCr   	 	= (em_uiHADCr  * uiQc)>>(13 + _iPerC + uiLog2TrSize);

	uiInfo[ETRI_IdLevelLuma] 	= LevelLuma;
	uiInfo[ETRI_IdLevelCb]   	= LevelCb;
	uiInfo[ETRI_IdLevelCr]   	= LevelCr;

#if ETRI_BUGFIX_ForcedSKIP
	if (uiInfo[ETRI_IdHADUpdate] > 0){
	uiInfo[ETRI_IdHADLuma]  	= em_uiHADLuma;
	uiInfo[ETRI_IdHADCb]  		= em_uiHADCb;
	uiInfo[ETRI_IdHADCr]   		= em_uiHADCr;
	}
#endif	
	/*------------------------------------------------------------
		What is the best Return Value for encoding speed and Performance ??
		1. return (LevelLuma + LevelCb + LevelCr):  somewhat slower and shows Low performance
		2. return LevelLuma : faster  
		3. return LevelEval  : faster 
	--------------------------------------------------------------*/
	LevelEval = (((LevelLuma<<3) - LevelLuma) + LevelCb + LevelCr + 4)>>3;
	
	return LevelEval;		

}

UInt TComPrediction::ETRI_HAD_getControlLevel (UInt*& uiInfo, UInt _rem, Int _iPer, UInt _remC, Int _iPerC )
{
#if ETRI_FASTPUProcessing 
 	UInt		LevelLuma, LevelCb, LevelCr, LevelEval;
	UInt	 	uiWidth = uiInfo[0];
	UInt	 	uiLog2TrSize = g_aucConvertToBit[ uiWidth ] + 2;
	Int 	 	uiQ  = g_quantScales[_rem];
	Int 	 	uiQc = g_quantScales[_remC];

	LevelLuma = (uiInfo[ETRI_IdHADLuma] * uiQ) >>(14 + _iPer  + uiLog2TrSize);
	LevelCb   	= (uiInfo[ETRI_IdHADCb]   * uiQc)>>(13 + _iPerC + uiLog2TrSize);	
	LevelCr   	= (uiInfo[ETRI_IdLevelCr]   * uiQc)>>(13 + _iPerC + uiLog2TrSize);

	uiInfo[ETRI_IdLevelLuma] 	= LevelLuma;
	uiInfo[ETRI_IdLevelCb]   	= LevelCb;
	uiInfo[ETRI_IdLevelCr]   	= LevelCr;

	/*------------------------------------------------------------
		What is the best Return Value for encoding speed and Performance ??
		1. return (LevelLuma + LevelCb + LevelCr):  somewhat slower and shows Low performance
		2. return LevelLuma : faster  
		3. return LevelEval  : faster 
	--------------------------------------------------------------*/
	LevelEval = (((LevelLuma<<3) - LevelLuma) + LevelCb + LevelCr + 4)>>3;
	
	return LevelEval;		
#else
	return 0;
#endif
}


/** 
-----------------------------------------------------------------------------------------
	@fn    	Void TComPrediction::ETRI_SM_motionCompensation 
	@param 	TComDataCU* pcCU    	: 
	@param 	TComYuv* pcOrigYuv   	: 
	@param 	TComYuv* pcOrigYuv   	: 
	@param 	TComRdCost* pcRdCost	:
	@param  	UInt& uiDistortion   		:
	@param  	RefPicList eRefPicList  	:
	@param  	Int iPartIdx    	    	  	:
	@brief	motionCompensation only in SkipMerge Process.	\n
	       		SkipMerge를 빠르게 수행하기 위해 사용되는 MotionCompensation으로서 다른 곳에서는 사용되지 않는다.
-----------------------------------------------------------------------------------------
*/
Void TComPrediction::ETRI_SM_motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, TComYuv* pcOrigYuv, TComRdCost* pcRdCost, UInt& uiDistortion, RefPicList eRefPicList, Int iPartIdx )
{
	TComMv 	cMv, ZeroMV,  FracMV, pMV[2];

	Int  		iRefIdx[3] = {-1, -1, -1};
	Int 		iWidth;
	Int 		iHeight;
	UInt		uiPartAddr;
#if ETRI_FastMC_inSKIPMerge_DEVCode0702
	UInt		uiLumaChromaIdx, uiChromaOnlyIdx;
#endif
	RefPicList	LRefPicList;
	Int			iRefIdxID = 0;
	bool		_Condition[6];

	memset(_Condition, false, 6 * sizeof(bool));
	ZeroMV.setZero();	

	assert( (iPartIdx < 0) && ("Unvalid iPartIdx Condition\n"));
	assert((eRefPicList == REF_PIC_LIST_X) && ("Unvalid eRefPicList Condition\n"));
//	assert((pcCU->getNumPartInter() == 1) && ("Unvalid pcCU->getNumPartInter() Condition\n"));

	uiPartAddr = 0;
	iWidth = pcCU->getWidth(0);	 
	iHeight = pcCU->getHeight(0); 	
	
//	iWidth = pcCU->getWidth(0) >>(pcCU->getPartitionSize(0) & 0x01); 	 
//	iHeight = pcCU->getHeight(0) >>((pcCU->getPartitionSize(0) & 0x10)>>1);		

	/// 2014 7 12 by Seok : For Debug
//	em_DbgInfo[dbgZeroPredMV] = 0;
//	em_DbgInfo[dbgTotalPredMV] = 0;

	if (xCheckIdenticalMotion( pcCU, uiPartAddr ) )
	{
		cMv   	= pcCU->getCUMvField(REF_PIC_LIST_0 )->getMv( uiPartAddr );
		iRefIdx[2] = pcCU->getCUMvField(REF_PIC_LIST_0 )->getRefIdx( uiPartAddr );			assert (iRefIdx[2] >= 0);
		pcCU->ETRI_clipMv(cMv);

#if ETRI_FastMC_inSKIPMerge_DEVCode0702
		/// 2014 6 27 by Seok : Test Code
		pMV[0] = cMv;
		LRefPicList = REF_PIC_LIST_0;
		iRefIdxID = iRefIdx[2];

		uiLumaChromaIdx 	= 0; 
		uiChromaOnlyIdx 	= uiLumaChromaIdx + 1;

		/// 2014 7 3 by Seok : 강제로 2의 배수로 Merge Candidate을 만든다. Luma/Chroma Interpolation은 없음
		pMV[0] >>= 3;
		pMV[0] <<= 3;

		FracMV.set((pMV[0].getHor() & 7),(pMV[0].getVer() & 7)); 	/// Chroma의 경우를 생각했을 때 적어도 이 정도는 되어야 단순 Copy가 됨
		_Condition[uiLumaChromaIdx] = (FracMV == ZeroMV);   /// Zero MV 경우가 포함됨

		FracMV.set((pMV[0].getHor() & 3),(pMV[0].getVer() & 3));	/// Only Luma의 경우
		_Condition[uiChromaOnlyIdx] = (FracMV == ZeroMV);   /// Zero MV 경우가 포함됨

		/// 2014 7 12 by Seok 
//		em_DbgInfo[dbgZeroPredMV] = (_Condition[uiLumaChromaIdx]  &  _Condition[uiChromaOnlyIdx]);
//		em_DbgInfo[dbgTotalPredMV] = 1;

		/// 2014 6 28 by Seok : If _Condition[0] = false, then the following original code is executed.
		if (! _Condition[uiLumaChromaIdx])
		{	
			xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, iRefIdx[2] )->getPicYuvRec(), uiPartAddr, &pMV[0], iWidth, iHeight, pcYuvPred, false);
			if (! _Condition[uiChromaOnlyIdx])
			{
				xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, iRefIdx[2] )->getPicYuvRec(), uiPartAddr, &pMV[0], iWidth, iHeight, pcYuvPred, false);
			}
		}
#else
		//Original 
		xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, iRefIdx[2] )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, false);
		xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, iRefIdx[2] )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, false );
#endif

	}
	else
	{
		TComYuv* pcMbYuv;

		for ( Int iRefList = 0; iRefList < 2; iRefList++ )
		{
			RefPicList		e_RefPicList 	= (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

			cMv = pcCU->getCUMvField(e_RefPicList)->getMv( uiPartAddr );		
			pcCU->ETRI_clipMv(cMv);

			iRefIdx[iRefList] = pcCU->getCUMvField( e_RefPicList )->getRefIdx( uiPartAddr );
			if ( iRefIdx[iRefList] < 0 )	{continue;}
			assert( iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(e_RefPicList) );

			_Condition[4 - iRefList] = (pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 );

			LRefPicList = e_RefPicList;
			iRefIdxID = iRefIdx[iRefList];

#if ETRI_FastMC_inSKIPMerge_DEVCode0703 
			pMV[iRefList] = cMv;

			uiLumaChromaIdx 	= (iRefList<<1); 

			//pMV[iRefList] >>= 3;
			//pMV[iRefList] <<= 3;


			FracMV.set((pMV[iRefList].getHor() & 7),(pMV[iRefList].getVer() & 7));	/// Chroma의 경우를 생각했을 때 적어도 이 정도는 되어야 단순 Copy가 됨
			_Condition[uiLumaChromaIdx] = (FracMV == ZeroMV);	/// Zero MV 경우가 포함됨

			if (! _Condition[uiLumaChromaIdx]){ 
				pcMbYuv = &m_acYuvPred[iRefList];
				xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic(e_RefPicList, iRefIdx[iRefList] )->getPicYuvRec(), uiPartAddr, &pMV[iRefList], iWidth, iHeight, pcMbYuv, _Condition[4 - iRefList]);
				xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic(e_RefPicList, iRefIdx[iRefList] )->getPicYuvRec(), uiPartAddr, &pMV[iRefList], iWidth, iHeight, pcMbYuv, _Condition[4 - iRefList]);
			}
#else
			pcMbYuv = &m_acYuvPred[iRefList];
			xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic(e_RefPicList, iRefIdx[iRefList])->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcMbYuv, _Condition[4 - iRefList]);
			xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic(e_RefPicList, iRefIdx[iRefList] )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcMbYuv, _Condition[4 - iRefList]);

#endif			
		}

		TComYuv* pcYuvSrc0 = &m_acYuvPred[0];
		TComYuv* pcYuvSrc1 = &m_acYuvPred[1];

#if ETRI_FastMC_inSKIPMerge_DEVCode0703
		if ( iRefIdx[0] >= 0 && iRefIdx[1] >= 0 )
		{
			if (_Condition[0]){
				xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, iRefIdx[0] )->getPicYuvRec(), uiPartAddr, &pMV[0], iWidth, iHeight, pcYuvSrc0, _Condition[4]);
				xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, iRefIdx[0] )->getPicYuvRec(), uiPartAddr, &pMV[0], iWidth, iHeight, pcYuvSrc0, _Condition[4]);
			}

			if (_Condition[2]){			
				xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, iRefIdx[1] )->getPicYuvRec(), uiPartAddr, &pMV[1], iWidth, iHeight, pcYuvSrc1, _Condition[3]);
				xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, iRefIdx[1] )->getPicYuvRec(), uiPartAddr, &pMV[1], iWidth, iHeight, pcYuvSrc1, _Condition[3]);
			}

			pcYuvPred->addAvg( &m_acYuvPred[0], &m_acYuvPred[1], uiPartAddr, iWidth, iHeight );
			_Condition[0] = _Condition[1] = false;
		}
		else if ( iRefIdx[0] >= 0 && iRefIdx[1] <  0 )
		{	
			if (_Condition[0])
			{
				LRefPicList = REF_PIC_LIST_0;
				iRefIdxID = iRefIdx[0];
				_Condition[1] = true;
			}
			else
			{
				pcYuvSrc0->copyPartToPartYuv( pcYuvPred, uiPartAddr, iWidth, iHeight );
				_Condition[0] = _Condition[1] = false;
			}
		}
		else if ( iRefIdx[0] <  0 && iRefIdx[1] >= 0 )
		{	  
			if (_Condition[2])
			{
				pMV[0] = pMV[1];
				LRefPicList = REF_PIC_LIST_1;
				iRefIdxID = iRefIdx[1];
				_Condition[0] = _Condition[1] = true;	
			}
			else
			{
				pcYuvSrc1->copyPartToPartYuv( pcYuvPred, uiPartAddr, iWidth, iHeight );
				_Condition[0] = _Condition[1] = false;
			}
		}
#else
		/// 2014 7 2 by Seok : Original 
		if 	   ( iRefIdx[0] >= 0 && iRefIdx[1] >= 0 )	  pcYuvPred->addAvg( &m_acYuvPred[0], &m_acYuvPred[1], uiPartAddr, iWidth, iHeight );
		else if ( iRefIdx[0] >= 0 && iRefIdx[1] <  0 )	  pcYuvSrc0->copyPartToPartYuv( pcYuvPred, uiPartAddr, iWidth, iHeight );
		else if ( iRefIdx[0] <  0 && iRefIdx[1] >= 0 )	  pcYuvSrc1->copyPartToPartYuv( pcYuvPred, uiPartAddr, iWidth, iHeight );
#endif		
	}

	// choose best cand using RDOOff		
	UInt	uiStrideOrg	 = pcOrigYuv->getStride();
	UInt	uiStridePred = pcYuvPred->getStride();
	UInt	uiWidth		 = pcOrigYuv->getWidth();
	UInt	uiHeight 	 = pcOrigYuv->getHeight();
	Pel*	piOrg		 = pcOrigYuv->getLumaAddr( 0, uiWidth );
	Pel*	piPred		 = pcYuvPred->getLumaAddr( 0, uiWidth );

	/// 2014 6 27 by Seok
	Pel*  piTargetY;
	Pel*  piTargetCb;
	Pel*  piTargetCr;
	Int   targetStride;   
	Int   targetCStride;  
	Int   targetOffset;
	Int   targetCOffset;

	if(_Condition[1])
	{
		TComPicYuv*	targetPic = pcCU->getSlice()->getRefPic(LRefPicList, iRefIdxID)->getPicYuvRec();

		targetStride 	= targetPic->getStride();	
		//  targetOffset 	= ( cMv.getHor() >> 2 ) + ( cMv.getVer() >> 2 ) * targetStride;
		targetOffset  = ( pMV[0].getHor() >> 2 ) + ( pMV[0].getVer() >> 2 ) * targetStride;
		piTargetY 	= targetPic->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr) + targetOffset;

		if (_Condition[0])
		{
			targetCStride = targetPic->getCStride();	
			//targetCOffset = ( cMv.getHor() >> 3 ) + ( cMv.getVer() >> 3 ) * targetCStride;
			targetCOffset = ( pMV[0].getHor() >> 3 ) + ( pMV[0].getVer() >> 3 ) * targetCStride;
			piTargetCb	= targetPic->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr) + targetCOffset;
			piTargetCr	= targetPic->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr) + targetCOffset;
		}
		else
		{
			targetCStride = pcYuvPred->getCStride();
			piTargetCb	= pcYuvPred->getCbAddr();
			piTargetCr	= pcYuvPred->getCrAddr();
		}
	}
	else
	{
		targetStride 	= uiStridePred;
		piTargetY 	= piPred;

		targetCStride = pcYuvPred->getCStride();
		piTargetCb	= pcYuvPred->getCbAddr();
		piTargetCr	= pcYuvPred->getCrAddr();
	}

	//ETRI_RDOOFF_MERGE_YCbCrCost == 0 so that ....
	em_uiHADLuma 	= pcRdCost->getDistPart(g_bitDepthY,  piTargetY, targetStride, piOrg, uiStrideOrg,  uiWidth, uiHeight , TEXT_LUMA, DF_HADS );
	em_uiHADCb    	= pcRdCost->getDistPart(g_bitDepthC, piTargetCb, targetCStride, pcOrigYuv->getCbAddr(), pcOrigYuv->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_U, DF_HADS );
	em_uiHADCr    	= pcRdCost->getDistPart(g_bitDepthC, piTargetCr, targetCStride, pcOrigYuv->getCrAddr(),  pcOrigYuv->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_V, DF_HADS );

	//Very Important. Default value of FULL_NBIT = 0. However, when it is 1, for fair comparison between each mode, HAD value is scaled with bit difference.
#if FULL_NBIT
	em_uiHADLuma  	= em_uiHADLuma >> (g_bitDepthY - 8);
	em_uiHADCb   	= em_uiHADCb     >> (g_bitDepthC - 8);
	em_uiHADCr   	= em_uiHADCr     >> (g_bitDepthC - 8);
#endif

	uiDistortion = em_uiHADLuma + em_uiHADCb + em_uiHADCr;

	return;
}


#endif

/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}


Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->ETRI_clipMv(cMv);

  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
  xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvPred )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[2] = {-1, -1};

  for ( Int iRefList = 0; iRefList < 2; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[iRefList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[iRefList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
           ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }  
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred ); 
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }
}

/**
 * \brief Generate motion-compensated luma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterLumaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi )
{
  Int refStride = refPic->getStride();  
  Int refOffset = ( mv->getHor() >> 2 ) + ( mv->getVer() >> 2 ) * refStride;
  Pel *ref      = refPic->getLumaAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Int dstStride = dstPic->getStride();
  Pel *dst      = dstPic->getLumaAddr( partAddr );
  
  Int xFrac = mv->getHor() & 0x3;
  Int yFrac = mv->getVer() & 0x3;

  if ( yFrac == 0 )
  {
    m_if.filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac,       !bi );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true, !bi );
  }
  else
  {
    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();

    Int filterSize = NTAPS_LUMA;
    Int halfFilterSize = ( filterSize >> 1 );

    m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false     );
    m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false, !bi);    
  }
}

/**
 * \brief Generate motion-compensated chroma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi )
{
  Int     refStride  = refPic->getCStride();
  Int     dstStride  = dstPic->getCStride();
  
  Int     refOffset  = (mv->getHor() >> 3) + (mv->getVer() >> 3) * refStride;
  
  Pel*    refCb     = refPic->getCbAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  Pel*    refCr     = refPic->getCrAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Pel* dstCb = dstPic->getCbAddr( partAddr );
  Pel* dstCr = dstPic->getCrAddr( partAddr );
  
  Int     xFrac  = mv->getHor() & 0x7;
  Int     yFrac  = mv->getVer() & 0x7;
  UInt    cxWidth  = width  >> 1;
  UInt    cxHeight = height >> 1;
  
  Int     extStride = m_filteredBlockTmp[0].getStride();
  Short*  extY      = m_filteredBlockTmp[0].getLumaAddr();
  
  Int filterSize = NTAPS_CHROMA;
  
  Int halfFilterSize = (filterSize>>1);
  
  if ( yFrac == 0 )
  {
    m_if.filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
    m_if.filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
    m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
  }
  else
  {
    m_if.filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);
    
    m_if.filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);    
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}



// ====================================================================================================================
// Public member functions
// ====================================================================================================================
#if ETRI_SIMD_INTRA

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering(Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight)
	{
	  Pel* pDst = rpDst;
	  Int y, iDstStride2, iSrcStride2;
	
	  // boundary pixels processing
	  short pDst0  = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);
	  __m128i eMul = _mm_set1_epi16(3);
	  __m128i eAdd = _mm_set1_epi16(2);
	
	  if(iWidth==4)
	  {
		  _mm_storel_epi64((__m128i*)(pDst),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst)),eMul),eAdd)),2));
	  }
	  else if(iWidth==8) // x = 0
	  {
		  _mm_store_si128((__m128i*)(pDst+0),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+0)),eMul),eAdd)),2));
	
	  }
	  else if(iWidth==16) // x = 0 , 8
	  {
		  _mm_store_si128((__m128i*)(pDst+0),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+0)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+8),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+8)),eMul),eAdd)),2));
	
	  }
	  else if(iWidth==32) // x = 0 , 8, 16, 24
	  {
	
		  _mm_store_si128((__m128i*)(pDst+0),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+0)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+8),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+8)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+16),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+16-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+20-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+16)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+24),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+24-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+28-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+24)),eMul),eAdd)),2));
	
	  }
	  else if(iWidth==64) // x = 0 , 8, 16, 24, 32, 40, 48, 56
	  {
		  _mm_store_si128((__m128i*)(pDst+0),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+0)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+8),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+8)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+16),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+16-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+20-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+16)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+24),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+24-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+28-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+24)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+32),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+32-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+36-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+32)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+40),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+40-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+44-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+40)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+48),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+48-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+52-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+48)),eMul),eAdd)),2));
		  _mm_store_si128((__m128i*)(pDst+56),_mm_srai_epi16(_mm_adds_epi16(_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+56-iSrcStride)),_mm_loadu_si128((__m128i*)(pSrc+50-iSrcStride))),_mm_adds_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)(pDst+56)),eMul),eAdd)),2));
	
	  }
	
	  pDst[0] = pDst0;
	
	  for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
	  {
		  pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
	  }
	  return;

}

Void TComPrediction::ETRI_clearYuvPred()
{
	m_acYuvPred[0].clear();
	m_acYuvPred[1].clear();
	m_cYuvPredTemp.clear();

}

Void TComPrediction::ETRI_Transpose4x4( Pel*pDst, Int dstStride)
{
	__m128i s0, s1;
	__m128i t0, t1;

	ALIGNED(16) Pel R0[8]={0};
	ALIGNED(16) Pel R1[8]={0};

	Int edstStride = dstStride;

	*(UInt64 *)R0 = *(UInt64 *)pDst; 
	*(UInt64 *)(R0+4) = *(UInt64 *)(pDst+ edstStride); edstStride+=dstStride;
	*(UInt64 *)R1 = *(UInt64 *)(pDst+ edstStride); edstStride+=dstStride;
	*(UInt64 *)(R1+4) = *(UInt64 *)(pDst+ edstStride); 

	__m128i xmm0 = _mm_loadl_epi64((__m128i*)(pDst)); 
	__m128i xmm1 = _mm_loadl_epi64((__m128i*)(pDst+dstStride));
	__m128i xmm2 = _mm_loadl_epi64((__m128i*)(pDst+3*dstStride));

	s0= _mm_packs_epi16(xmm0, xmm1);
	s1= _mm_packs_epi16(xmm0, xmm2);

	s0 = _mm_load_si128((__m128i*)R0);
	s1 = _mm_load_si128((__m128i*)R1);

	t0 = _mm_unpacklo_epi16(s0, s1);
	t1 = _mm_unpackhi_epi16(s0, s1);

	s0 = _mm_unpacklo_epi16(t0, t1);
	s1 = _mm_unpackhi_epi16(t0, t1);

	_mm_store_si128((__m128i*)R0, s0);
	_mm_store_si128((__m128i*)R1, s1);

	_mm_storel_epi64((__m128i*)(pDst), _mm_loadl_epi64((__m128i*)(R0)));  pDst+= dstStride;
	_mm_storel_epi64((__m128i*)(pDst), _mm_loadl_epi64((__m128i*)(R0+4))); pDst+= dstStride;
	_mm_storel_epi64((__m128i*)(pDst), _mm_loadl_epi64((__m128i*)(R1))); pDst+= dstStride;
	_mm_storel_epi64((__m128i*)(pDst), _mm_loadl_epi64((__m128i*)(R1+4)));
}

Void TComPrediction::ETRI_Transpose8x8( Pel* pDst, Int dstStride)
{
	__m128i s0, s1, s2, s3, s4, s5, s6, s7;
	__m128i t0, t1, t2, t3, t4, t5, t6, t7;

	Pel*epDst = pDst;

	s0 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s1 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s2 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s3 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s4 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s5 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s6 = _mm_load_si128((__m128i*)(epDst)); epDst+=dstStride;
	s7 = _mm_load_si128((__m128i*)(epDst));

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);

	s0 = _mm_unpacklo_epi16(t0, t4);
	s1 = _mm_unpackhi_epi16(t0, t4);
	s2 = _mm_unpacklo_epi16(t1, t5);
	s3 = _mm_unpackhi_epi16(t1, t5);
	s4 = _mm_unpacklo_epi16(t2, t6);
	s5 = _mm_unpackhi_epi16(t2, t6);
	s6 = _mm_unpacklo_epi16(t3, t7);
	s7 = _mm_unpackhi_epi16(t3, t7);

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);

	_mm_store_si128((__m128i*)(pDst), t0); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t1); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t2); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t3); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t4); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t5); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t6); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t7); pDst+= dstStride;
}

Void TComPrediction::ETRI_Transpose16x16( Pel* pDst, Int dstStride)
{
	__m128i s0, s1, s2, s3, s4, s5, s6, s7;
	__m128i t0, t1, t2, t3, t4, t5, t6, t7;

	const Int m_pPosOffset16[2][4] =
	{
		{0, 8, dstStride*8, dstStride*8+8},
		{0, dstStride*8, 8, dstStride*8+8}      
	};

	Int i;
	Pel *epDSt = pDst;

	s0 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;  
	s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s7 = _mm_load_si128((__m128i*)(epDSt));

	for(i=0; i< 3; ++i)
	{
		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);

		s0 = _mm_unpacklo_epi16(t0, t4);
		s1 = _mm_unpackhi_epi16(t0, t4);
		s2 = _mm_unpacklo_epi16(t1, t5);
		s3 = _mm_unpackhi_epi16(t1, t5);
		s4 = _mm_unpacklo_epi16(t2, t6);
		s5 = _mm_unpackhi_epi16(t2, t6);
		s6 = _mm_unpacklo_epi16(t3, t7);
		s7 = _mm_unpackhi_epi16(t3, t7);

		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);    

		epDSt = pDst+m_pPosOffset16[0][i+1];

		s0 = _mm_load_si128((__m128i*)(epDSt )); epDSt+= dstStride;
		s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt+= dstStride;
		s7 = _mm_load_si128((__m128i*)(epDSt));

		Int edstStride = m_pPosOffset16[1][i];
		_mm_store_si128((__m128i*)(pDst+ edstStride), t0); edstStride+=dstStride; 
		_mm_store_si128((__m128i*)(pDst+ edstStride), t1); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t2); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t3); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t4); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t5); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t6); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t7);
	}

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);

	s0 = _mm_unpacklo_epi16(t0, t4);
	s1 = _mm_unpackhi_epi16(t0, t4);
	s2 = _mm_unpacklo_epi16(t1, t5);
	s3 = _mm_unpackhi_epi16(t1, t5);
	s4 = _mm_unpacklo_epi16(t2, t6);
	s5 = _mm_unpackhi_epi16(t2, t6);
	s6 = _mm_unpacklo_epi16(t3, t7);
	s7 = _mm_unpackhi_epi16(t3, t7);

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);    

	pDst = pDst+  m_pPosOffset16[1][3];
	_mm_store_si128((__m128i*)(pDst), t0); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t1); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t2); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t3); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t4); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t5); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t6); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t7);
}

Void TComPrediction::ETRI_Transpose32x32( Pel* pDst, Int dstStride)
{
	__m128i s0, s1, s2, s3, s4, s5, s6, s7;
	__m128i t0, t1, t2, t3, t4, t5, t6, t7;

	const Int m_pPosOffset32[2][16] =
	{
		{0, dstStride*8+8, 8,  dstStride*8, 16,  dstStride*16, 24, dstStride*24, dstStride*8+16, dstStride*16+8, dstStride*8+24, dstStride*24+8, dstStride*16+16, dstStride*24+24, dstStride*16+24, dstStride*24+16},
		{0,  dstStride*8+8,  dstStride*8, 8,  dstStride*16, 16, dstStride*24, 24, dstStride*16+8,  dstStride*8+16, dstStride*24+8, dstStride*8+24, dstStride*16+16,  dstStride*24+24,  dstStride*24+16, dstStride*16+24}      
	};

	Int i;

	Pel *epDSt = pDst;

	s0 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt+=dstStride;
	s7 = _mm_load_si128((__m128i*)(epDSt));

	for(i=0; i< 15; ++i)
	{
		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);

		s0 = _mm_unpacklo_epi16(t0, t4);
		s1 = _mm_unpackhi_epi16(t0, t4);
		s2 = _mm_unpacklo_epi16(t1, t5);
		s3 = _mm_unpackhi_epi16(t1, t5);
		s4 = _mm_unpacklo_epi16(t2, t6);
		s5 = _mm_unpackhi_epi16(t2, t6);
		s6 = _mm_unpacklo_epi16(t3, t7);
		s7 = _mm_unpackhi_epi16(t3, t7);

		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);    

		epDSt = pDst+ m_pPosOffset32[0][i+1];
		s0 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s7 = _mm_load_si128((__m128i*)(epDSt));

		Int edstStride =  m_pPosOffset32[1][i];
		_mm_store_si128((__m128i*)(pDst+ edstStride), t0); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t1); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t2); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t3); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t4); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t5); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t6); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t7);
	}

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);

	s0 = _mm_unpacklo_epi16(t0, t4);
	s1 = _mm_unpackhi_epi16(t0, t4);
	s2 = _mm_unpacklo_epi16(t1, t5);
	s3 = _mm_unpackhi_epi16(t1, t5);
	s4 = _mm_unpacklo_epi16(t2, t6);
	s5 = _mm_unpackhi_epi16(t2, t6);
	s6 = _mm_unpacklo_epi16(t3, t7);
	s7 = _mm_unpackhi_epi16(t3, t7);

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);    

	pDst=pDst+m_pPosOffset32[1][15];
	_mm_store_si128((__m128i*)(pDst ), t0); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t1); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t2); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t3); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t4); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t5); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t6); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t7);
}
Void TComPrediction::ETRI_Transpose64x64( Pel* pDst, Int dstStride)
{
	__m128i s0, s1, s2, s3, s4, s5, s6, s7;
	__m128i t0, t1, t2, t3, t4, t5, t6, t7;

	const Int m_pPosOffset64[2][64] =
	{
		{0, dstStride*8+8, 8, dstStride*8, 16, dstStride*16, 24, dstStride*24, 32, dstStride*32, 40, dstStride*40, 48, dstStride*48, 56, dstStride*56, dstStride*8+16, dstStride*16+8, dstStride*8+24, dstStride*24+8, 
		dstStride*8+32, dstStride*32+8, dstStride*8+40, dstStride*40+8, dstStride*8+48, dstStride*48+8, dstStride*8+56, dstStride*56+8, dstStride*16+16, dstStride*24+24, dstStride*16+24, dstStride*24+16, dstStride*16+32, dstStride*32+16, 
		dstStride*16+40, dstStride*40+16,  dstStride*16+48,  dstStride*48+16,  dstStride*16+56,  dstStride*56+16,  dstStride*24+32,  dstStride*32+24,  dstStride*24+40,  dstStride*40+24,  dstStride*24+48,  dstStride*48+24,  dstStride*24+56, dstStride*56+24, 
		dstStride*32+32, dstStride*40+40, dstStride*32+40, dstStride*40+32, dstStride*32+48, dstStride*48+32, dstStride*32+56, dstStride*56+32, dstStride*40+48, dstStride*48+40, dstStride*40+56, dstStride*56+40,  dstStride*48+48,  dstStride*56+56, dstStride*48+56, dstStride*56+48},	
		{0, dstStride*8+8, dstStride*8, 8,  dstStride*16, 16,  dstStride*24, 24, dstStride*32, 32,  dstStride*40, 40, dstStride*48, 48, dstStride*56, 56,  dstStride*16+8, dstStride*8+16, dstStride*24+8, dstStride*8+24,
		dstStride*32+8, dstStride*8+32, dstStride*40+8, dstStride*8+40, dstStride*48+8,   dstStride*8+48, dstStride*56+8, dstStride*8+56, dstStride*16+16, dstStride*24+24,  dstStride*24+16, dstStride*16+24, dstStride*32+16, dstStride*16+32, 
		dstStride*40+16, dstStride*16+40, dstStride*48+16,  dstStride*16+48, dstStride*56+16, dstStride*16+56, dstStride*32+24, dstStride*24+32, dstStride*40+24, dstStride*24+40, dstStride*48+24, dstStride*24+48,  dstStride*56+24,  dstStride*24+56,
		dstStride*32+32, dstStride*40+40,  dstStride*40+32,  dstStride*32+40, dstStride*48+32,	dstStride*32+48, dstStride*56+32,  dstStride*32+56,	dstStride*48+40,  dstStride*40+48,  dstStride*56+40,  dstStride*40+56, dstStride*48+48,  dstStride*56+56, dstStride*56+48, dstStride*48+56 	}
	};

	Int i;
	Pel *epDSt = pDst;

	s0 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
	s7 = _mm_load_si128((__m128i*)(epDSt));

	for(i=0; i< 63; ++i)
	{
		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);

		s0 = _mm_unpacklo_epi16(t0, t4);
		s1 = _mm_unpackhi_epi16(t0, t4);
		s2 = _mm_unpacklo_epi16(t1, t5);
		s3 = _mm_unpackhi_epi16(t1, t5);
		s4 = _mm_unpacklo_epi16(t2, t6);
		s5 = _mm_unpackhi_epi16(t2, t6);
		s6 = _mm_unpacklo_epi16(t3, t7);
		s7 = _mm_unpackhi_epi16(t3, t7);

		t0 = _mm_unpacklo_epi16(s0, s4);
		t1 = _mm_unpackhi_epi16(s0, s4);
		t2 = _mm_unpacklo_epi16(s1, s5);
		t3 = _mm_unpackhi_epi16(s1, s5);
		t4 = _mm_unpacklo_epi16(s2, s6);
		t5 = _mm_unpackhi_epi16(s2, s6);
		t6 = _mm_unpacklo_epi16(s3, s7);
		t7 = _mm_unpackhi_epi16(s3, s7);    

		epDSt = pDst+ m_pPosOffset64[0][i+1];
		s0 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s1 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s2 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s3 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s4 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s5 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s6 = _mm_load_si128((__m128i*)(epDSt)); epDSt += dstStride;
		s7 = _mm_load_si128((__m128i*)(epDSt));

		Int edstStride =  m_pPosOffset64[1][i];
		_mm_store_si128((__m128i*)(pDst+ edstStride), t0); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t1); edstStride += dstStride; 
		_mm_store_si128((__m128i*)(pDst+ edstStride), t2); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t3); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t4); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t5); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t6); edstStride += dstStride;
		_mm_store_si128((__m128i*)(pDst+ edstStride), t7);

	}

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);

	s0 = _mm_unpacklo_epi16(t0, t4);
	s1 = _mm_unpackhi_epi16(t0, t4);
	s2 = _mm_unpacklo_epi16(t1, t5);
	s3 = _mm_unpackhi_epi16(t1, t5);
	s4 = _mm_unpacklo_epi16(t2, t6);
	s5 = _mm_unpackhi_epi16(t2, t6);
	s6 = _mm_unpacklo_epi16(t3, t7);
	s7 = _mm_unpackhi_epi16(t3, t7);

	t0 = _mm_unpacklo_epi16(s0, s4);
	t1 = _mm_unpackhi_epi16(s0, s4);
	t2 = _mm_unpacklo_epi16(s1, s5);
	t3 = _mm_unpackhi_epi16(s1, s5);
	t4 = _mm_unpacklo_epi16(s2, s6);
	t5 = _mm_unpackhi_epi16(s2, s6);
	t6 = _mm_unpacklo_epi16(s3, s7);
	t7 = _mm_unpackhi_epi16(s3, s7);    

	pDst = pDst + m_pPosOffset64[1][63];
	_mm_store_si128((__m128i*)(pDst), t0); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t1); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t2); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t3); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t4); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t5); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t6); pDst+= dstStride;
	_mm_store_si128((__m128i*)(pDst), t7);
}
#if ETRI_SIMD_FIX_INTRA_ANGULAR_PREDICTION
Void TComPrediction::xPredIntraAng4x4(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
    Int k;
    Int blkSize        = width;
    Pel* pDst          = rpDst;

    // Map the mode index to main prediction direction and angle
    assert( dirMode > 0 ); //no planar
    Bool modeDC        = dirMode < 2;
    Bool modeHor       = !modeDC && (dirMode < 18);
    Bool modeVer       = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng         = abs(intraPredAngle);
    Int signAng        = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle       = invAngTable[absAng];
    absAng             = angTable[absAng];
    intraPredAngle     = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {

        Pel dcval = predIntraGetPredValDC(pSrc, pLSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        __m128i xmm_dcval = _mm_set1_epi16(dcval);

        for (Int y=height; y>0; y--, pDst+=dstStride)
            _mm_storel_epi64((__m128i*)&pDst[0], xmm_dcval);

    }

    // Do angular predictions
    else
    {
        Pel* refMain;
        Pel* refSide;
        Pel  refAbove[2*MAX_CU_SIZE+1];
        Pel  refLeft[2*MAX_CU_SIZE+1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {

            _mm_storel_epi64((__m128i *)(refAbove+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));
            _mm_storel_epi64((__m128i *)(refLeft +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)),_mm_loadu_si128((__m128i*)(pLSrc+4))));	

            refAbove[blkSize-1] = pSrc[0-srcStride-1];
            refLeft[blkSize-1] = pSrc[0-srcStride-1];

            refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

            // Extend the Main reference to the left.
            Int invAngleSum    = 128;       // rounding for (shift by 8)
            for (k=-1; k>blkSize*intraPredAngle>>5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum>>8];
            }
        }
        else
        {
            _mm_storeu_si128((__m128i *)(refAbove+1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));	
            _mm_storeu_si128((__m128i *)(refLeft +1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)),_mm_loadu_si128((__m128i*)(pLSrc+4))));	 


            refAbove[0] = pSrc[-srcStride-1];
            refLeft[0]  = pSrc[-srcStride-1];

            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft  : refAbove;
        }

        if (intraPredAngle == 0)
        {

            Int tmp_dstStride = 0;
            for (Int y=0;y<height;y++)
            {
                _mm_storel_epi64((__m128i *)&pDst[tmp_dstStride] ,_mm_loadu_si128((__m128i *)&refMain[1]));
                tmp_dstStride+=dstStride;
            }

        }
        else
        {
            Int deltaPos=0;
            Int deltaInt;
            Int deltaFract;
            __m128i xmm_add16 = _mm_set1_epi16(16);
            __m128i xmm0, xmm1;
            Int tmp_dstStride = 0;
            for (k=0;k<blkSize;k++)
            {

                deltaPos += intraPredAngle;
                deltaInt   = (deltaPos >> 5) + 1;
                deltaFract = deltaPos & (32 - 1);
                __m128i xmm_deltaFract = _mm_set1_epi16(deltaFract);
                __m128i xmm_deltaFract_minus32 = _mm_set1_epi16(32-deltaFract);

                xmm0 = _mm_loadu_si128((__m128i*)(refMain+deltaInt)); 
                xmm1 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1)); 

                if (deltaFract)
                    _mm_storel_epi64((__m128i*)(pDst+tmp_dstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32,xmm0), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract,xmm1),xmm_add16)),5));
                else
                    _mm_storel_epi64((__m128i *)(pDst+tmp_dstStride),xmm0);

                tmp_dstStride += dstStride;
            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            ETRI_Transpose4x4(pDst, dstStride);
        }
    }
}
Void TComPrediction::xPredIntraAng8x8(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
    Int k;
    Int blkSize        = width;
    Pel* pDst          = rpDst;

    // Map the mode index to main prediction direction and angle
    assert( dirMode > 0 ); //no planar
    Bool modeDC        = dirMode < 2;
    Bool modeHor       = !modeDC && (dirMode < 18);
    Bool modeVer       = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng         = abs(intraPredAngle);
    Int signAng        = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle       = invAngTable[absAng];
    absAng             = angTable[absAng];
    intraPredAngle     = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDC(pSrc, pLSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        __m128i xmm_dcval = _mm_set1_epi16(dcval);

        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval); pDst+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst),xmm_dcval);

    }

    // Do angular predictions
    else
    {
        Pel* refMain;
        Pel* refSide;
        Pel  refAbove[2*MAX_CU_SIZE+1];
        Pel  refLeft[2*MAX_CU_SIZE+1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {

            _mm_storeu_si128((__m128i *)(refAbove+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refLeft +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)),_mm_loadu_si128((__m128i*)(pLSrc+4))));	

            refAbove[blkSize-1] = pSrc[-srcStride-1];
            refLeft[blkSize-1] = pSrc[-srcStride-1];

            refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

            // Extend the Main reference to the left.
            Int invAngleSum    = 128;       // rounding for (shift by 8)
            for (k=-1; k>blkSize*intraPredAngle>>5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum>>8];
            }

        }
        else
        {
            _mm_storeu_si128((__m128i *)(refAbove+1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+9),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-srcStride))));  

            _mm_storeu_si128((__m128i *)(refLeft+1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc  )),_mm_loadu_si128((__m128i*)(pLSrc+4)))); 
            _mm_storeu_si128((__m128i *)(refLeft+9),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+8)),_mm_loadu_si128((__m128i*)(pLSrc+12))));	

            refAbove[0] = pSrc[-srcStride-1];
            refLeft[0] = pSrc[-srcStride-1];

            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft  : refAbove;
        }

        if (intraPredAngle == 0)
        {

            __m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+1));
            Int tmp_dstStride = dstStride; 
            _mm_storeu_si128((__m128i *)(pDst)              ,xmm0); 
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0); tmp_dstStride += dstStride;
            _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0);


            if ( bFilter )
            {
                for (k=0;k<blkSize;k++)
                {
                    pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
                }
            }
        }
        else
        {
            Int deltaPos=0;
            Int deltaInt;
            Int deltaFract;
            __m128i xmm_add16 = _mm_set1_epi16(16);
            __m128i xmm0, xmm1; 
            Int tmp_dstStride = 0; 

            for (k=0;k<blkSize;k++)
            {

                deltaPos += intraPredAngle;
                deltaInt = (deltaPos >> 5) + 1;
                deltaFract = deltaPos & 31;

                __m128i xmm_deltaFract = _mm_set1_epi16(deltaFract); 
                __m128i xmm_deltaFract_minus32 = _mm_set1_epi16(32-deltaFract); 

                xmm0 = _mm_loadu_si128((__m128i *)(refMain+deltaInt));
                xmm1 = _mm_loadu_si128((__m128i *)(refMain+deltaInt+1));

                if (deltaFract)
                {
                    _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32,xmm0), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract,xmm1),xmm_add16)),5));
                }
                else
                {
                    _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0);
                }
                tmp_dstStride += dstStride;
            }

        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            ETRI_Transpose8x8(pDst, dstStride);
        }
    }
}
Void TComPrediction::xPredIntraAng16x16(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
    Int k;
    Int blkSize        = width;
    Pel* pDst          = rpDst;

    // Map the mode index to main prediction direction and angle
    assert( dirMode > 0 ); //no planar
    Bool modeDC        = dirMode < 2;
    Bool modeHor       = !modeDC && (dirMode < 18);
    Bool modeVer       = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng         = abs(intraPredAngle);
    Int signAng        = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle       = invAngTable[absAng];
    absAng             = angTable[absAng];
    intraPredAngle     = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDC(pSrc, pLSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        __m128i xmm_dcval = _mm_set1_epi16(dcval);

        Int tmp_dstStride = dstStride; 

        _mm_storeu_si128((__m128i*)(pDst),              xmm_dcval); 
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride),xmm_dcval); 

        Pel* pDstPlus8 = pDst + 8;
        tmp_dstStride = dstStride; 

        _mm_storeu_si128((__m128i*)(pDstPlus8),              xmm_dcval); 
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); tmp_dstStride+=dstStride;
        _mm_storeu_si128((__m128i*)(pDstPlus8+tmp_dstStride),xmm_dcval); 

    }

    // Do angular predictions
    else
    {
        Pel* refMain;
        Pel* refSide;
        Pel  refAbove[2*MAX_CU_SIZE+1];
        Pel  refLeft[2*MAX_CU_SIZE+1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {

            _mm_storeu_si128((__m128i *)(refAbove  +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+8+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refLeft   +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)),_mm_loadu_si128((__m128i*)(pLSrc+4))));	
            _mm_storeu_si128((__m128i *)(refLeft +8+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+8)),_mm_loadu_si128((__m128i*)(pLSrc+12))));	

            refAbove[blkSize-1] = pSrc[-srcStride-1];
            refLeft[blkSize-1] = pSrc[-srcStride-1];

            refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

            // Extend the Main reference to the left.
            Int invAngleSum    = 128;       // rounding for (shift by 8)
            for (k=-1; k>blkSize*intraPredAngle>>5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum>>8];
            }
        }
        else
        {
            _mm_storeu_si128((__m128i *)(refAbove+1 ),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride))   ,_mm_loadu_si128((__m128i*)(pSrc+4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+9 ),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-srcStride)) ,_mm_loadu_si128((__m128i*)(pSrc+12-srcStride))));  
            _mm_storeu_si128((__m128i *)(refAbove+17),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+16-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+20-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+25),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+24-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+28-srcStride))));  

            _mm_storeu_si128((__m128i *)(refLeft+1 ),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc  )) ,_mm_loadu_si128((__m128i*)(pLSrc+4)))); 
            _mm_storeu_si128((__m128i *)(refLeft+9 ),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+8)) ,_mm_loadu_si128((__m128i*)(pLSrc+12))));	
            _mm_storeu_si128((__m128i *)(refLeft+17),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+16)),_mm_loadu_si128((__m128i*)(pLSrc+20)))); 
            _mm_storeu_si128((__m128i *)(refLeft+25),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+24)),_mm_loadu_si128((__m128i*)(pLSrc+28))));	

            refAbove[0] = pSrc[-srcStride-1];
            refLeft[0] = pSrc[-srcStride-1];

            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft  : refAbove;
        }

        if (intraPredAngle == 0)
        {
            __m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+1));
            __m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain+9));

            Int tmp_dstStride = 0; 
            for(int i=0; i<blkSize; i++)
            {
                _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride  ),xmm0);
                _mm_storeu_si128((__m128i *)(pDst+8+tmp_dstStride),xmm1);
                tmp_dstStride+=dstStride;
            }


            if ( bFilter )
            {
                for (k=0;k<blkSize;k++)
                {
                    pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
                }
            }
        }
        else
        {
            Int deltaPos=0;
            Int deltaInt;
            Int deltaFract;
            __m128i xmm_add16 = _mm_set1_epi16(16);
            __m128i xmm0, xmm1, xmm2, xmm3; 
            Int tmp_dstStride = 0; 

            for (k=0;k<blkSize;k++)
            {
                deltaPos += intraPredAngle;
                deltaInt   = (deltaPos >> 5) + 1;
                deltaFract = deltaPos & (32 - 1);
                __m128i xmm_deltaFract = _mm_set1_epi16(deltaFract);
                __m128i xmm_deltaFract_minus32 = _mm_set1_epi16(32-deltaFract);


                xmm0 = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
                xmm1 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));
                xmm2 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8));
                xmm3 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+9));


                if (deltaFract)
                {
                    _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride  ),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32,xmm0),_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract,xmm1),xmm_add16)),5));
                    _mm_storeu_si128((__m128i*)(pDst+tmp_dstStride+8),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32,xmm2),_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract,xmm3),xmm_add16)),5));
                }
                else
                {
                    _mm_storeu_si128((__m128i *)(pDst+tmp_dstStride),xmm0);
                    _mm_storeu_si128((__m128i *)(pDst+8+tmp_dstStride),xmm2);
                }
                tmp_dstStride+=dstStride;

            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            ETRI_Transpose16x16(pDst, dstStride);
        }
    }
}
Void TComPrediction::xPredIntraAng32x32(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
    Int k;
    Int blkSize        = width;
    Pel* pDst          = rpDst;

    // Map the mode index to main prediction direction and angle
    assert( dirMode > 0 ); //no planar
    Bool modeDC        = dirMode < 2;
    Bool modeHor       = !modeDC && (dirMode < 18);
    Bool modeVer       = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng         = abs(intraPredAngle);
    Int signAng        = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle       = invAngTable[absAng];
    absAng             = angTable[absAng];
    intraPredAngle     = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDC(pSrc, pLSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        __m128i xmm_dcval = _mm_set1_epi16(dcval);

        Int tmp_dstStride = 0;

        for (k = 0; k < blkSize; k++)
        {
            _mm_storeu_si128((__m128i*)(pDst   +tmp_dstStride),xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst +8+tmp_dstStride),xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst+16+tmp_dstStride),xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst+24+tmp_dstStride),xmm_dcval);
            tmp_dstStride+=dstStride;
        }
    }

    // Do angular predictions
    else
    {
        Pel* refMain;
        Pel* refSide;
        Pel  refAbove[2*MAX_CU_SIZE+1];
        Pel  refLeft[2*MAX_CU_SIZE+1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {

            _mm_storeu_si128((__m128i *)(refAbove   +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc   -srcStride)),_mm_loadu_si128((__m128i*)(pSrc +4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove +8+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc +8-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+16+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+16-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+20-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+24+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+24-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+28-srcStride))));	 

            _mm_storeu_si128((__m128i *)(refLeft   +blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc   )),_mm_loadu_si128((__m128i*)(pLSrc +4))));	
            _mm_storeu_si128((__m128i *)(refLeft +8+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc +8)),_mm_loadu_si128((__m128i*)(pLSrc+12))));	
            _mm_storeu_si128((__m128i *)(refLeft+16+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+16)),_mm_loadu_si128((__m128i*)(pLSrc+20))));	
            _mm_storeu_si128((__m128i *)(refLeft+24+blkSize),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+24)),_mm_loadu_si128((__m128i*)(pLSrc+28))));	

            refAbove[blkSize-1] = pSrc[-srcStride-1];
            refLeft[blkSize-1] = pSrc[-srcStride-1];

            refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

            // Extend the Main reference to the left.
            Int invAngleSum    = 128;       // rounding for (shift by 8)
            for (k=-1; k>blkSize*intraPredAngle>>5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum>>8];
            }
        }
        else
        {

            _mm_storeu_si128((__m128i *)(refAbove +1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc   -srcStride)),_mm_loadu_si128((__m128i*)(pSrc +4-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove +9),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc +8-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+12-srcStride))));  
            _mm_storeu_si128((__m128i *)(refAbove+17),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+16-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+20-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+25),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+24-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+28-srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove+33),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+32-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+36-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+41),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+40-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+44-srcStride))));  
            _mm_storeu_si128((__m128i *)(refAbove+49),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+48-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+52-srcStride))));	 
            _mm_storeu_si128((__m128i *)(refAbove+57),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+56-srcStride)),_mm_loadu_si128((__m128i*)(pSrc+60-srcStride))));  

            _mm_storeu_si128((__m128i *)(refLeft +1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc   )),_mm_loadu_si128((__m128i*)(pLSrc +4)))); 
            _mm_storeu_si128((__m128i *)(refLeft +9),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc +8)),_mm_loadu_si128((__m128i*)(pLSrc+12))));	
            _mm_storeu_si128((__m128i *)(refLeft+17),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+16)),_mm_loadu_si128((__m128i*)(pLSrc+20)))); 
            _mm_storeu_si128((__m128i *)(refLeft+25),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+24)),_mm_loadu_si128((__m128i*)(pLSrc+28))));	
            _mm_storeu_si128((__m128i *)(refLeft+33),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+32)),_mm_loadu_si128((__m128i*)(pLSrc+36)))); 
            _mm_storeu_si128((__m128i *)(refLeft+41),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+40)),_mm_loadu_si128((__m128i*)(pLSrc+44))));	
            _mm_storeu_si128((__m128i *)(refLeft+49),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+48)),_mm_loadu_si128((__m128i*)(pLSrc+52)))); 
            _mm_storeu_si128((__m128i *)(refLeft+57),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc+56)),_mm_loadu_si128((__m128i*)(pLSrc+60))));	

            refAbove[0] = pSrc[-srcStride-1];
            refLeft[0] = pSrc[-srcStride-1];

            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft : refAbove;
    }

        if (intraPredAngle == 0)
        {

            __m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain + 1));
            __m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain + 9));
            __m128i xmm2 = _mm_loadu_si128((__m128i*)(refMain + 17));
            __m128i xmm3 = _mm_loadu_si128((__m128i*)(refMain + 25));

            Int tmp_dstStride = 0;
            for (int i = 0; i < blkSize; i++)
            {
                _mm_storeu_si128((__m128i *)(pDst + tmp_dstStride), xmm0);
                _mm_storeu_si128((__m128i *)(pDst + 8 + tmp_dstStride), xmm1);
                _mm_storeu_si128((__m128i *)(pDst + 16 + tmp_dstStride), xmm2);
                _mm_storeu_si128((__m128i *)(pDst + 24 + tmp_dstStride), xmm3);
                tmp_dstStride += dstStride;
            }


            if (bFilter)
            {
                for (k = 0; k < blkSize; k++)
                {
                    pDst[k*dstStride] = Clip3(0, (1 << bitDepth) - 1, pDst[k*dstStride] + ((refSide[k + 1] - refSide[0]) >> 1));
                }
            }
        }
        else
        {
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            __m128i xmm_add16 = _mm_set1_epi16(16);
            __m128i xmm[8];
            Int tmp_dstStride = 0;

            for (k = 0; k < blkSize; k++)
            {
                deltaPos += intraPredAngle;
                deltaInt = (deltaPos >> 5) + 1;
                deltaFract = deltaPos & (32 - 1);
                __m128i xmm_deltaFract = _mm_set1_epi16(deltaFract);
                __m128i xmm_deltaFract_minus32 = _mm_set1_epi16(32 - deltaFract);


                xmm[0] = _mm_loadu_si128((__m128i*)(refMain + deltaInt));
                xmm[1] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 1));
                xmm[2] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 8));
                xmm[3] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 9));
                xmm[4] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 16));
                xmm[5] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 17));
                xmm[6] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 24));
                xmm[7] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 25));


                if (deltaFract)
                {
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[0]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[1]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 8), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[2]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[3]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 16), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[4]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[5]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 24), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[6]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[7]), xmm_add16)), 5));
                }
                else
                {
                    _mm_storeu_si128((__m128i *)(pDst + tmp_dstStride), xmm[0]);
                    _mm_storeu_si128((__m128i *)(pDst + 8 + tmp_dstStride), xmm[2]);
                    _mm_storeu_si128((__m128i *)(pDst + 16 + tmp_dstStride), xmm[4]);
                    _mm_storeu_si128((__m128i *)(pDst + 24 + tmp_dstStride), xmm[6]);
                }
                tmp_dstStride += dstStride;

            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            ETRI_Transpose32x32(pDst, dstStride);
        }
  }
}
Void TComPrediction::xPredIntraAng64x64(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter)
{
    Int k;
    Int blkSize = width;
    Pel* pDst = rpDst;

    // Map the mode index to main prediction direction and angle
    assert(dirMode > 0); //no planar
    Bool modeDC = dirMode < 2;
    Bool modeHor = !modeDC && (dirMode < 18);
    Bool modeVer = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng = abs(intraPredAngle);
    Int signAng = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9] = { 0, 2, 5, 9, 13, 17, 21, 26, 32 };
    Int invAngTable[9] = { 0, 4096, 1638, 910, 630, 482, 390, 315, 256 }; // (256 * 32) / Angle
    Int invAngle = invAngTable[absAng];
    absAng = angTable[absAng];
    intraPredAngle = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDC(pSrc, pLSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        __m128i xmm_dcval = _mm_set1_epi16(dcval);

        Int tmp_dstStride = 0;

        for (k = 0; k < blkSize; k++)
        {
            _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 8 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 16 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 24 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 32 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 40 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 48 + tmp_dstStride), xmm_dcval);
            _mm_storeu_si128((__m128i*)(pDst + 56 + tmp_dstStride), xmm_dcval);
            tmp_dstStride += dstStride;
        }
    }

    // Do angular predictions
    else
    {
        Pel* refMain;
        Pel* refSide;
        Pel  refAbove[2 * MAX_CU_SIZE + 1];
        Pel  refLeft[2 * MAX_CU_SIZE + 1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {

            _mm_storeu_si128((__m128i *)(refAbove + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 4 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 8 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 8 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 12 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 16 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 16 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 20 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 24 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 24 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 28 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 32 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 32 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 36 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 40 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 40 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 44 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 48 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 48 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 52 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 56 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 56 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 60 - srcStride))));

            _mm_storeu_si128((__m128i *)(refLeft + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)), _mm_loadu_si128((__m128i*)(pLSrc + 4))));
            _mm_storeu_si128((__m128i *)(refLeft + 8 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 8)), _mm_loadu_si128((__m128i*)(pLSrc + 12))));
            _mm_storeu_si128((__m128i *)(refLeft + 16 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 16)), _mm_loadu_si128((__m128i*)(pLSrc + 20))));
            _mm_storeu_si128((__m128i *)(refLeft + 24 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 24)), _mm_loadu_si128((__m128i*)(pLSrc + 28))));
            _mm_storeu_si128((__m128i *)(refLeft + 32 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 32)), _mm_loadu_si128((__m128i*)(pLSrc + 36))));
            _mm_storeu_si128((__m128i *)(refLeft + 40 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 40)), _mm_loadu_si128((__m128i*)(pLSrc + 44))));
            _mm_storeu_si128((__m128i *)(refLeft + 48 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 48)), _mm_loadu_si128((__m128i*)(pLSrc + 52))));
            _mm_storeu_si128((__m128i *)(refLeft + 56 + blkSize), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 56)), _mm_loadu_si128((__m128i*)(pLSrc + 60))));

            refAbove[blkSize - 1] = pSrc[-srcStride - 1];
            refLeft[blkSize - 1] = pSrc[-srcStride - 1];

            refMain = (modeVer ? refAbove : refLeft) + (blkSize - 1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize - 1);

            // Extend the Main reference to the left.
            Int invAngleSum = 128;       // rounding for (shift by 8)
            for (k = -1; k > blkSize*intraPredAngle >> 5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum >> 8];
            }

        }
        else
        {

            _mm_storeu_si128((__m128i *)(refAbove + 1), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 4 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 9), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 8 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 12 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 17), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 16 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 20 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 25), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 24 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 28 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 33), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 32 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 36 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 41), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 40 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 44 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 49), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 48 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 52 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 57), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 56 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 60 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 65), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 64 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 68 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 73), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 72 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 76 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 81), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 80 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 84 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 89), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 88 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 92 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 97), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 96 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 100 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 105), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 104 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 108 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 113), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 112 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 116 - srcStride))));
            _mm_storeu_si128((__m128i *)(refAbove + 121), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc + 120 - srcStride)), _mm_loadu_si128((__m128i*)(pSrc + 124 - srcStride))));

            _mm_storeu_si128((__m128i *)(refLeft + 1), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc)), _mm_loadu_si128((__m128i*)(pLSrc + 4))));
            _mm_storeu_si128((__m128i *)(refLeft + 9), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 8)), _mm_loadu_si128((__m128i*)(pLSrc + 12))));
            _mm_storeu_si128((__m128i *)(refLeft + 17), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 16)), _mm_loadu_si128((__m128i*)(pLSrc + 20))));
            _mm_storeu_si128((__m128i *)(refLeft + 25), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 24)), _mm_loadu_si128((__m128i*)(pLSrc + 28))));
            _mm_storeu_si128((__m128i *)(refLeft + 33), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 32)), _mm_loadu_si128((__m128i*)(pLSrc + 36))));
            _mm_storeu_si128((__m128i *)(refLeft + 41), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 40)), _mm_loadu_si128((__m128i*)(pLSrc + 44))));
            _mm_storeu_si128((__m128i *)(refLeft + 49), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 48)), _mm_loadu_si128((__m128i*)(pLSrc + 52))));
            _mm_storeu_si128((__m128i *)(refLeft + 57), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 56)), _mm_loadu_si128((__m128i*)(pLSrc + 60))));
            _mm_storeu_si128((__m128i *)(refLeft + 65), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 64)), _mm_loadu_si128((__m128i*)(pLSrc + 68))));
            _mm_storeu_si128((__m128i *)(refLeft + 73), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 72)), _mm_loadu_si128((__m128i*)(pLSrc + 76))));
            _mm_storeu_si128((__m128i *)(refLeft + 81), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 80)), _mm_loadu_si128((__m128i*)(pLSrc + 84))));
            _mm_storeu_si128((__m128i *)(refLeft + 89), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 88)), _mm_loadu_si128((__m128i*)(pLSrc + 92))));
            _mm_storeu_si128((__m128i *)(refLeft + 97), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 96)), _mm_loadu_si128((__m128i*)(pLSrc + 100))));
            _mm_storeu_si128((__m128i *)(refLeft + 105), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 104)), _mm_loadu_si128((__m128i*)(pLSrc + 108))));
            _mm_storeu_si128((__m128i *)(refLeft + 113), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 112)), _mm_loadu_si128((__m128i*)(pLSrc + 116))));
            _mm_storeu_si128((__m128i *)(refLeft + 121), _mm_packs_epi32(_mm_loadu_si128((__m128i*)(pLSrc + 120)), _mm_loadu_si128((__m128i*)(pLSrc + 124))));

            refAbove[0] = pSrc[-srcStride - 1];
            refLeft[0] = pSrc[-srcStride - 1];

            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft : refAbove;
        }

        if (intraPredAngle == 0)
        {
            __m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain + 1));
            __m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain + 9));
            __m128i xmm2 = _mm_loadu_si128((__m128i*)(refMain + 17));
            __m128i xmm3 = _mm_loadu_si128((__m128i*)(refMain + 25));
            __m128i xmm4 = _mm_loadu_si128((__m128i*)(refMain + 33));
            __m128i xmm5 = _mm_loadu_si128((__m128i*)(refMain + 41));
            __m128i xmm6 = _mm_loadu_si128((__m128i*)(refMain + 49));
            __m128i xmm7 = _mm_loadu_si128((__m128i*)(refMain + 57));

            Int tmp_dstStride = 0;
            for (int i = 0; i < blkSize; i++)
            {
                _mm_storeu_si128((__m128i *)(pDst + tmp_dstStride), xmm0);
                _mm_storeu_si128((__m128i *)(pDst + 8 + tmp_dstStride), xmm1);
                _mm_storeu_si128((__m128i *)(pDst + 16 + tmp_dstStride), xmm2);
                _mm_storeu_si128((__m128i *)(pDst + 24 + tmp_dstStride), xmm3);
                _mm_storeu_si128((__m128i *)(pDst + 32 + tmp_dstStride), xmm4);
                _mm_storeu_si128((__m128i *)(pDst + 40 + tmp_dstStride), xmm5);
                _mm_storeu_si128((__m128i *)(pDst + 48 + tmp_dstStride), xmm6);
                _mm_storeu_si128((__m128i *)(pDst + 56 + tmp_dstStride), xmm7);
                tmp_dstStride += dstStride;
            }


            if (bFilter)
            {
                for (k = 0; k < blkSize; k++)
                {
                    pDst[k*dstStride] = Clip3(0, (1 << bitDepth) - 1, pDst[k*dstStride] + ((refSide[k + 1] - refSide[0]) >> 1));
                }
            }
        }
        else
        {
 
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            __m128i xmm_add16 = _mm_set1_epi16(16);
            __m128i xmm[16];
            Int tmp_dstStride = 0;

            for (k = 0; k < blkSize; k++)
            {
                deltaPos += intraPredAngle;
                deltaInt = (deltaPos >> 5) + 1;
                deltaFract = deltaPos & (32 - 1);
                __m128i xmm_deltaFract = _mm_set1_epi16(deltaFract);
                __m128i xmm_deltaFract_minus32 = _mm_set1_epi16(32 - deltaFract);


                xmm[0] = _mm_loadu_si128((__m128i*)(refMain + deltaInt));
                xmm[1] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 1));
                xmm[2] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 8));
                xmm[3] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 9));
                xmm[4] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 16));
                xmm[5] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 17));
                xmm[6] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 24));
                xmm[7] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 25));
                xmm[8] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 32));
                xmm[9] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 33));
                xmm[10] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 40));
                xmm[11] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 41));
                xmm[12] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 48));
                xmm[13] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 49));
                xmm[14] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 56));
                xmm[15] = _mm_loadu_si128((__m128i*)(refMain + deltaInt + 57));


                if (deltaFract)
                {
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[0]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[1]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 8), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[2]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[3]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 16), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[4]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[5]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 24), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[6]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[7]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 32), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[8]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[9]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 40), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[10]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[11]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 48), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[12]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[13]), xmm_add16)), 5));
                    _mm_storeu_si128((__m128i*)(pDst + tmp_dstStride + 56), _mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract_minus32, xmm[13]), _mm_add_epi16(_mm_mullo_epi16(xmm_deltaFract, xmm[15]), xmm_add16)), 5));
                }
                else
                {
                    _mm_storeu_si128((__m128i *)(pDst + tmp_dstStride), xmm[0]);
                    _mm_storeu_si128((__m128i *)(pDst + 8 + tmp_dstStride), xmm[2]);
                    _mm_storeu_si128((__m128i *)(pDst + 16 + tmp_dstStride), xmm[4]);
                    _mm_storeu_si128((__m128i *)(pDst + 24 + tmp_dstStride), xmm[6]);
                    _mm_storeu_si128((__m128i *)(pDst + 32 + tmp_dstStride), xmm[8]);
                    _mm_storeu_si128((__m128i *)(pDst + 40 + tmp_dstStride), xmm[10]);
                    _mm_storeu_si128((__m128i *)(pDst + 48 + tmp_dstStride), xmm[12]);
                    _mm_storeu_si128((__m128i *)(pDst + 56 + tmp_dstStride), xmm[14]);
                }
                tmp_dstStride += dstStride;

            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
             ETRI_Transpose64x64(pDst, dstStride);
        }
    }
}
#else 
Void TComPrediction::xPredIntraAng4(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
	UInt width = 4;
	UInt height = 4;

	Int k;
	Int blkSize        = width;
	Pel* pDst          = rpDst;

	// Map the mode index to main prediction direction and angle
	assert( dirMode > 0 ); //no planar
	Bool modeDC        = dirMode < 2;
	Bool modeHor       = !modeDC && (dirMode < 18);
	Bool modeVer       = !modeDC && !modeHor;
	Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
	Int absAng         = abs(intraPredAngle);
	Int signAng        = intraPredAngle < 0 ? -1 : 1;

	// Set bitshifts and scale the angle parameter to block size
	Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
	Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
	Int invAngle       = invAngTable[absAng];
	absAng             = angTable[absAng];
	intraPredAngle     = signAng * absAng;

	// Do the DC prediction
	if (modeDC)
	{

		Pel dcval = predIntraGetPredValDC(pSrc, pColPixs, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

		Short tttemp[4] = {dcval, dcval, dcval, dcval};

		//Short *pttDst = pDst;
		*(UInt64 *)pDst = *(UInt64 *)tttemp; pDst += dstStride;
		*(UInt64 *)pDst = *(UInt64 *)tttemp; pDst += dstStride;
		*(UInt64 *)pDst = *(UInt64 *)tttemp; pDst += dstStride;
		*(UInt64 *)pDst = *(UInt64 *)tttemp; 
	}
	else
	{
		Pel* refMain;
		Pel* refSide;
		Pel  refAbove[2*MAX_CU_SIZE+1];
		Pel  refLeft[2*MAX_CU_SIZE+1];

		// Initialise the Main and Left reference array.
		if (intraPredAngle < 0) // // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]   //OK
		{

			_mm_storel_epi64((__m128i *)(refAbove+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride-1)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride-1))));
			_mm_storel_epi64((__m128i *)(refLeft+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pColPixs-1)),_mm_loadu_si128((__m128i*)(pColPixs-1+4))));	
			refLeft[blkSize-1] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[blkSize+blkSize-1] = pSrc[blkSize-srcStride-1]; // blkSize+1
			refLeft[blkSize+blkSize-1] = pSrc[(blkSize-1)*srcStride-1];

			refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
			refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

			// Extend the Main reference to the left.
			Int invAngleSum    = 128;       // rounding for (shift by 8)
			for (k=-1; k>blkSize*intraPredAngle>>5; k--)
			{
				invAngleSum += invAngle;
				refMain[k] = refSide[invAngleSum>>8];
			}
		}
		else
		{	
			_mm_storeu_si128((__m128i *)(refAbove),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride-1)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride-1))));	
			_mm_storeu_si128((__m128i *)(refLeft),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pColPixs-1)),_mm_loadu_si128((__m128i*)(pColPixs-1+4))));	

			refLeft[0] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[2*blkSize] = pSrc[2*blkSize-srcStride-1];
			refLeft[2*blkSize] = pSrc[(2*blkSize-1)*srcStride-1];

			refMain = modeVer ? refAbove : refLeft;
			refSide = modeVer ? refLeft  : refAbove;
		}

		if (intraPredAngle == 0)
		{

			*(UInt64 *)(pDst) = *(UInt64 *)(refMain+1); //pDst += dstStride;
			*(UInt64 *)(pDst+1*dstStride) = *(UInt64 *)(refMain+1); //pDst += dstStride;
			*(UInt64 *)(pDst+2*dstStride) = *(UInt64 *)(refMain+1);// pDst += dstStride;
			*(UInt64 *)(pDst+3*dstStride) = *(UInt64 *)(refMain+1);

			if ( bFilter )
			{
				for (k=0;k<blkSize;k++)  
				{
					pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
				}
			}

		}
		else
		{
			Int deltaPos=0;
			Int deltaInt;
			Int deltaFract;
			__m128i eAdd16 = _mm_set1_epi16(16);
			__m128i xmm0, xmm1;
			Int edstStride = 0;
			for (k=0;k<blkSize;k++)
			{

				deltaPos += intraPredAngle;
				deltaInt   = (deltaPos >> 5) + 1;
				deltaFract = deltaPos & (32 - 1);
				__m128i edeltaFract = _mm_set1_epi16(deltaFract);
				__m128i edeltaFract_Minus32 = _mm_set1_epi16(32-deltaFract);

				xmm0 = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
				xmm1 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));

				if (deltaFract)
					_mm_storel_epi64((__m128i*)(pDst+edstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm0), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm1),eAdd16)),5));
				else
					_mm_storel_epi64((__m128i *)(pDst+edstStride),xmm0);

				edstStride += dstStride;
			}
		}

		// Flip the block if this is the horizontal mode
		if (modeHor)
		{
			ETRI_Transpose4x4(pDst,dstStride);
		}
	}
}

Void TComPrediction::xPredIntraAng8(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
	UInt width = 8;
	UInt height = 8;
	Int k;
	Int blkSize        = width;
	Pel* pDst          = rpDst;

	// Map the mode index to main prediction direction and angle
	assert( dirMode > 0 ); //no planar
	Bool modeDC        = dirMode < 2;
	Bool modeHor       = !modeDC && (dirMode < 18);
	Bool modeVer       = !modeDC && !modeHor;
	Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
	Int absAng         = abs(intraPredAngle);
	Int signAng        = intraPredAngle < 0 ? -1 : 1;

	// Set bitshifts and scale the angle parameter to block size
	Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
	Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
	Int invAngle       = invAngTable[absAng];
	absAng             = angTable[absAng];
	intraPredAngle     = signAng * absAng;

	// Do the DC prediction
	if (modeDC)
	{

		Pel dcval = predIntraGetPredValDC(pSrc, pColPixs, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);
		__m128i edcval = _mm_set1_epi16(dcval);

		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); pDst+=dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval);
	}

	// Do angular predictions
	else
	{
		Pel* refMain;
		Pel* refSide;
		Pel  refAbove[2*MAX_CU_SIZE+1];
		Pel  refLeft[2*MAX_CU_SIZE+1];

		// Initialise the Main and Left reference array.
		if (intraPredAngle < 0) // // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]   //OK
		{
			_mm_storeu_si128((__m128i *)(refAbove+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc-srcStride-1)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride-1))));	//blkSize-1이 버퍼의 [0]
			_mm_storeu_si128((__m128i *)(refLeft+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pColPixs-1)),_mm_loadu_si128((__m128i*)(pColPixs-1+4))));	

			refLeft[blkSize-1] = pSrc[(-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[blkSize+blkSize-1] = pSrc[blkSize-srcStride-1]; // blkSize+1 // 버퍼의 마지막 
			refLeft[blkSize+blkSize-1] = pSrc[(blkSize-1)*srcStride-1]; // blkSize+1

			refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
			refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

			// Extend the Main reference to the left.
			Int invAngleSum    = 128;       // rounding for (shift by 8)
			for (k=-1; k>blkSize*intraPredAngle>>5; k--)
			{   
				invAngleSum += invAngle;
				refMain[k] = refSide[invAngleSum>>8];
			}

		}
		else
		{    
			_mm_storeu_si128((__m128i *)(refAbove),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+0-srcStride-1)),_mm_loadu_si128((__m128i*)(pSrc+4-srcStride-1))));	
			_mm_storeu_si128((__m128i *)(refAbove+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pSrc+8-srcStride-1)),_mm_loadu_si128((__m128i*)(pSrc+8+4-srcStride-1))));

			_mm_storeu_si128((__m128i *)(refLeft+0),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pColPixs-1+0)),_mm_loadu_si128((__m128i*)(pColPixs-1+4))));
			_mm_storeu_si128((__m128i *)(refLeft+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(pColPixs-1+8)),_mm_loadu_si128((__m128i*)(pColPixs-1+8+4))));	


			refLeft[0] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[2*blkSize] = pSrc[2*blkSize-srcStride-1];
			refLeft[2*blkSize] = pSrc[(2*blkSize-1)*srcStride-1];

			refMain = modeVer ? refAbove : refLeft;
			refSide = modeVer ? refLeft  : refAbove;
		}

		if (intraPredAngle == 0)
		{
			__m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+1));
			Int edstStride = dstStride;
			_mm_storeu_si128((__m128i *)(pDst),xmm0); 
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); edstStride += dstStride;
			_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0);

			if ( bFilter )
			{
				for (k=0;k<blkSize;k++)
				{
					pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
				}
			}

		}
		else
		{
			Int deltaPos=0;
			Int deltaInt;
			Int deltaFract;
			__m128i eAdd16 = _mm_set1_epi16(16);
			__m128i xmm0, xmm1;
			Int edstStride = 0;

			for (k=0;k<blkSize;k++)
			{
				deltaPos += intraPredAngle;
				deltaInt   = (deltaPos >> 5) + 1;
				deltaFract = deltaPos & (32 - 1);
				__m128i edeltaFract = _mm_set1_epi16(deltaFract);
				__m128i edeltaFract_Minus32 = _mm_set1_epi16(32-deltaFract);


				xmm0 = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
				xmm1 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));


				if (deltaFract)
				{
					_mm_storeu_si128((__m128i*)(pDst+edstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm0), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm1),eAdd16)),5));
				}
				else
				{
					_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0);
				}
				edstStride += dstStride;
			}
		}

		// Flip the block if this is the horizontal mode
		if (modeHor)
		{
			ETRI_Transpose8x8(pDst,dstStride);

		}
	}
}

Void TComPrediction::xPredIntraAng16(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable , Bool bFilter)
{
	UInt width = 16;
	UInt height = 16;
	Int k;
	Int blkSize        = width;
	Pel* pDst          = rpDst;

	// Map the mode index to main prediction direction and angle
	assert( dirMode > 0 ); //no planar
	Bool modeDC        = dirMode < 2;
	Bool modeHor       = !modeDC && (dirMode < 18);
	Bool modeVer       = !modeDC && !modeHor;
	Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
	Int absAng         = abs(intraPredAngle);
	Int signAng        = intraPredAngle < 0 ? -1 : 1;

	// Set bitshifts and scale the angle parameter to block size
	Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
	Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
	Int invAngle       = invAngTable[absAng];
	absAng             = angTable[absAng];
	intraPredAngle     = signAng * absAng;

	// Do the DC prediction
	if (modeDC)
	{
		Pel dcval = predIntraGetPredValDC(pSrc, pColPixs, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

		__m128i edcval = _mm_set1_epi16(dcval);

		//l = 0 일때

		Int edstStride = dstStride;
		_mm_store_si128((__m128i*)(pDst),edcval); 
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); 

		//l = 8 일때
		edstStride = dstStride + 8;
		_mm_store_si128((__m128i*)(pDst+8),edcval); 
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval); edstStride+=dstStride;
		_mm_store_si128((__m128i*)(pDst+edstStride),edcval);  

	}

	// Do angular predictions
	else
	{
		Pel* refMain;
		Pel* refSide;
		Pel  refAbove[2*MAX_CU_SIZE+1];
		Pel  refLeft[2*MAX_CU_SIZE+1];

		Int* epSrc = pSrc-srcStride-1;
		Int* epColPixs = pColPixs-1;
		// Initialise the Main and Left reference array.
		if (intraPredAngle < 0) // // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]   //OK
		{

			_mm_storeu_si128((__m128i *)(refAbove+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));	

			_mm_storeu_si128((__m128i *)(refLeft+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	

			refLeft[blkSize-1] = pSrc[(-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[blkSize+blkSize-1] = pSrc[blkSize-srcStride-1]; // blkSize+1
			refLeft[blkSize+blkSize-1] = pSrc[(blkSize-1)*srcStride-1];

			refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
			refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

			// Extend the Main reference to the left.
			Int invAngleSum    = 128;       // rounding for (shift by 8)
			for (k=-1; k>blkSize*intraPredAngle>>5; k--)
			{
				invAngleSum += invAngle;
				refMain[k] = refSide[invAngleSum>>8];
			}
		}
		else
		{
			_mm_storeu_si128((__m128i *)(refAbove),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));


			_mm_storeu_si128((__m128i *)(refLeft+0),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));
			_mm_storeu_si128((__m128i *)(refLeft+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));

			refLeft[0] = pSrc[(-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[2*blkSize] = pSrc[2*blkSize-srcStride-1];
			refLeft[2*blkSize] = pSrc[(2*blkSize-1)*srcStride-1];

			refMain = modeVer ? refAbove : refLeft;
			refSide = modeVer ? refLeft  : refAbove;
		}

		if (intraPredAngle == 0)
		{
			__m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+1));
			__m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain+8+1));
			Int edstStride = 0;
			for(int i=0; i<16; i++)
			{
				_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm1);
				edstStride+=dstStride;
			}

			if ( bFilter )
			{
				for (k=0;k<blkSize;k++)
				{
					pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
				}
			}

		}
		else
		{
			Int deltaPos=0;
			Int deltaInt;
			Int deltaFract;
			Int edstStride = 0;
			__m128i eAdd16 = _mm_set1_epi16(16);
			__m128i xmm0, xmm1, xmm2, xmm3;

			for (k=0;k<blkSize;k++)
			{
				deltaPos += intraPredAngle;
				deltaInt   = (deltaPos >> 5) + 1;
				deltaFract = deltaPos & (32 - 1);
				__m128i edeltaFract = _mm_set1_epi16(deltaFract);
				__m128i edeltaFract_Minus32 = _mm_set1_epi16(32-deltaFract);


				xmm0 = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
				xmm1 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));
				xmm2 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8));
				xmm3 = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8+1));


				if (deltaFract)
				{
					_mm_storeu_si128((__m128i*)(pDst+edstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm0), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm1),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+8),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm2), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm3),eAdd16)),5));
				}
				else
				{
					_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm2);
				}
				edstStride+=dstStride;

			}
		}

		// Flip the block if this is the horizontal mode
		if (modeHor)
		{
			ETRI_Transpose16x16(pDst,dstStride);
		}
	}
}

Void TComPrediction::xPredIntraAng32(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride,  UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable)
{
	UInt width = 32;
	UInt height = 32;
	Bool bFilter = false;
	Int k;
	Int blkSize        = width;
	Pel* pDst          = rpDst;

	// Map the mode index to main prediction direction and angle
	assert( dirMode > 0 ); //no planar
	Bool modeDC        = dirMode < 2;
	Bool modeHor       = !modeDC && (dirMode < 18);
	Bool modeVer       = !modeDC && !modeHor;
	Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
	Int absAng         = abs(intraPredAngle);
	Int signAng        = intraPredAngle < 0 ? -1 : 1;

	// Set bitshifts and scale the angle parameter to block size
	Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
	Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
	Int invAngle       = invAngTable[absAng];
	absAng             = angTable[absAng];
	intraPredAngle     = signAng * absAng;

	// Do the DC prediction
	if (modeDC)
	{
		Pel dcval = predIntraGetPredValDC(pSrc, pColPixs, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

		__m128i edcval = _mm_set1_epi16(dcval);
		Int edstStride = 0;
		for (k=0;k<blkSize;k++)
		{
			_mm_store_si128((__m128i*)(pDst+edstStride),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+8),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+16),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+24),edcval);
			edstStride+=dstStride;
		}


	}

	// Do angular predictions
	else
	{
		Pel* refMain;
		Pel* refSide;
		Pel  refAbove[2*MAX_CU_SIZE+1];
		Pel  refLeft[2*MAX_CU_SIZE+1];
		Int* epSrc = pSrc-srcStride-1;
		Int* epColPixs = pColPixs-1;
		// Initialise the Main and Left reference array.
		if (intraPredAngle < 0) // // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]   //OK
		{
			_mm_storeu_si128((__m128i *)(refAbove+0+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+16+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+24+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));

			_mm_storeu_si128((__m128i *)(refLeft+0+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+16+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+24+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	

			refLeft[blkSize-1] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[blkSize+blkSize-1] = pSrc[blkSize-srcStride-1]; // blkSize+1
			refLeft[blkSize+blkSize-1] = pSrc[(blkSize-1)*srcStride-1];

			refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
			refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

			// Extend the Main reference to the left.
			Int invAngleSum    = 128;       // rounding for (shift by 8)
			for (k=-1; k>blkSize*intraPredAngle>>5; k--)
			{
				invAngleSum += invAngle;
				refMain[k] = refSide[invAngleSum>>8];
			}
		}
		else
		{
			_mm_storeu_si128((__m128i *)(refAbove),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+32),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+40),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+48),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+56),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));


			_mm_storeu_si128((__m128i *)(refLeft),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));
			_mm_storeu_si128((__m128i *)(refLeft+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+32),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+40),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+48),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+56),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));


			refLeft[0] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[2*blkSize] = pSrc[2*blkSize-srcStride-1];
			refLeft[2*blkSize] = pSrc[(2*blkSize-1)*srcStride-1];

			refMain = modeVer ? refAbove : refLeft;
			refSide = modeVer ? refLeft  : refAbove;
		}

		if (intraPredAngle == 0)
		{
			__m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+0+1));
			__m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain+8+1));
			__m128i xmm2 = _mm_loadu_si128((__m128i*)(refMain+16+1));
			__m128i xmm3 = _mm_loadu_si128((__m128i*)(refMain+24+1));
			Int edstStride=0;
			for (k=0;k<blkSize;k++)
			{
				_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm0); 
				_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm1);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+16),xmm2);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+24),xmm3);
				edstStride+=dstStride;
			}

			if ( bFilter )
			{
				for (k=0;k<blkSize;k++)
				{
					pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
				}
			}
		}
		else
		{
			Int deltaPos=0;
			Int deltaInt;
			Int deltaFract;
			__m128i eAdd16 = _mm_set1_epi16(16);
			__m128i xmm[8];	
			Int edstStride=0;
			for (k=0;k<blkSize;k++)
			{
				deltaPos += intraPredAngle;
				deltaInt   = (deltaPos >> 5) + 1;
				deltaFract = deltaPos & (32 - 1);
				__m128i edeltaFract = _mm_set1_epi16(deltaFract);
				__m128i edeltaFract_Minus32 = _mm_set1_epi16(32-deltaFract);

				xmm[0] = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
				xmm[1] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));
				xmm[2] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8));
				xmm[3] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8+1));
				xmm[4] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+16));
				xmm[5] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+16+1));
				xmm[6] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+24));
				xmm[7] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+24+1));


				if (deltaFract)
				{
					_mm_storeu_si128((__m128i*)(pDst+edstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm[0]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm[1]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+8),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm[2]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm[3]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+16),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32,xmm[4]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm[5]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+24),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[6]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract,xmm[7]),eAdd16)),5));
				}
				else
				{
					_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm[0]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm[2]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+16),xmm[4]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+24),xmm[6]);
				}
				edstStride += dstStride;
			}
		}

		// Flip the block if this is the horizontal mode
		if (modeHor)
		{
			ETRI_Transpose32x32(pDst,dstStride);
		}
	}
}

Void TComPrediction::xPredIntraAng64(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable )
{
	UInt width = 64;
	UInt height = 64;
	Bool bFilter = false;
	Int k;
	Int blkSize        = width;
	Pel* pDst          = rpDst;

	// Map the mode index to main prediction direction and angle
	assert( dirMode > 0 ); //no planar
	Bool modeDC        = dirMode < 2;
	Bool modeHor       = !modeDC && (dirMode < 18);
	Bool modeVer       = !modeDC && !modeHor;
	Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
	Int absAng         = abs(intraPredAngle);
	Int signAng        = intraPredAngle < 0 ? -1 : 1;

	// Set bitshifts and scale the angle parameter to block size
	Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
	Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
	Int invAngle       = invAngTable[absAng];
	absAng             = angTable[absAng];
	intraPredAngle     = signAng * absAng;

	// Do the DC prediction
	if (modeDC)
	{
		Pel dcval = predIntraGetPredValDC(pSrc, pColPixs, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

		__m128i edcval = _mm_set1_epi16(dcval);
		Int edstStride = 0;

		for (k=0;k<blkSize;k++)
		{
			_mm_store_si128((__m128i*)(pDst+edstStride),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+8),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+16),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+24),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+32),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+40),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+48),edcval);
			_mm_store_si128((__m128i*)(pDst+edstStride+56),edcval);
			edstStride+=dstStride;
		}


	}

	// Do angular predictions
	else
	{
		Pel* refMain;
		Pel* refSide;
		Pel  refAbove[2*MAX_CU_SIZE+1];
		Pel  refLeft[2*MAX_CU_SIZE+1];
		Int* epSrc = pSrc-srcStride-1;
		Int* epColPixs = pColPixs-1;
		// Initialise the Main and Left reference array.
		if (intraPredAngle < 0) // // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]   //OK
		{
			_mm_storeu_si128((__m128i *)(refAbove+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+16+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+24+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+32+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+40+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+48+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+56+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));

			_mm_storeu_si128((__m128i *)(refLeft+0+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+8+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+16+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+24+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+32+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+40+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+48+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+56+blkSize-1),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	


			refLeft[blkSize-1] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[blkSize+blkSize-1] = pSrc[blkSize-srcStride-1]; // blkSize+1
			refLeft[blkSize+blkSize-1] = pSrc[(blkSize-1)*srcStride-1];

			refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
			refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

			// Extend the Main reference to the left.
			Int invAngleSum    = 128;       // rounding for (shift by 8)
			for (k=-1; k>blkSize*intraPredAngle>>5; k--)
			{
				invAngleSum += invAngle;
				refMain[k] = refSide[invAngleSum>>8];
			}
		}
		else
		{
			_mm_storeu_si128((__m128i *)(refAbove),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)(epSrc+4))));	
			_mm_storeu_si128((__m128i *)(refAbove+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+32),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+40),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+48),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+56),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+64),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+72),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+80),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+88),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+96),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+104),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+112),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));
			_mm_storeu_si128((__m128i *)(refAbove+120),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epSrc)),_mm_loadu_si128((__m128i*)((epSrc+=8)+4))));


			_mm_storeu_si128((__m128i *)(refLeft),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)(epColPixs+4))));
			_mm_storeu_si128((__m128i *)(refLeft+8),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));	
			_mm_storeu_si128((__m128i *)(refLeft+16),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+24),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+32),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+40),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+48),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+56),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+64),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+72),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+80),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+88),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+96),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+104),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+112),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));
			_mm_storeu_si128((__m128i *)(refLeft+120),_mm_packs_epi32(_mm_loadu_si128((__m128i*)(epColPixs)),_mm_loadu_si128((__m128i*)((epColPixs+=8)+4))));


			refLeft[0] = pSrc[(0-1)*srcStride-1];  // pSrc[k*iSrcStride-1] --> pColPixs[k], pSrc[(k-1)*srcStride-1] --> pColPixs[k-1]로 치환 시, pColPixs[-1]의 위치의 값이 없음

			refAbove[2*blkSize] = pSrc[2*blkSize-srcStride-1];
			refLeft[2*blkSize] = pSrc[(2*blkSize-1)*srcStride-1];

			refMain = modeVer ? refAbove : refLeft;
			refSide = modeVer ? refLeft  : refAbove;
		}

		if (intraPredAngle == 0)
		{
			__m128i xmm0 = _mm_loadu_si128((__m128i*)(refMain+0+1));
			__m128i xmm1 = _mm_loadu_si128((__m128i*)(refMain+8+1));
			__m128i xmm2 = _mm_loadu_si128((__m128i*)(refMain+16+1));
			__m128i xmm3 = _mm_loadu_si128((__m128i*)(refMain+24+1));
			__m128i xmm4 = _mm_loadu_si128((__m128i*)(refMain+32+1));
			__m128i xmm5 = _mm_loadu_si128((__m128i*)(refMain+40+1));
			__m128i xmm6 = _mm_loadu_si128((__m128i*)(refMain+48+1));
			__m128i xmm7 = _mm_loadu_si128((__m128i*)(refMain+56+1));
			Int edstStride = 0;
			for (k=0;k<blkSize;k++)
			{    

				_mm_storeu_si128((__m128i *)(pDst+edstStride+0),xmm0);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm1);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+16),xmm2);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+24),xmm3);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+32),xmm4);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+40),xmm5);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+48),xmm6);
				_mm_storeu_si128((__m128i *)(pDst+edstStride+56),xmm7);
				edstStride+=dstStride;
			}

			if ( bFilter )
			{
				for (k=0;k<blkSize;k++)
				{
					pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
				}
			}
		}
		else
		{
			Int deltaPos=0;
			Int deltaInt;
			Int deltaFract;
			Int edstStride = 0;
			//			Int refMainIndex;
			__m128i eAdd16 = _mm_set1_epi16(16);
			__m128i xmm[16];
			for (k=0;k<blkSize;k++)
			{
				deltaPos += intraPredAngle;
				deltaInt   = (deltaPos >> 5) + 1;
				deltaFract = deltaPos & (32 - 1);
				__m128i edeltaFract = _mm_set1_epi16(deltaFract);
				__m128i edeltaFract_Minus32 = _mm_set1_epi16(32-deltaFract);

				xmm[0] = _mm_loadu_si128((__m128i*)(refMain+deltaInt));
				xmm[1] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+1));
				xmm[2] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8));
				xmm[3] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+8+1));
				xmm[4] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+16));
				xmm[5] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+16+1));
				xmm[6] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+24));
				xmm[7] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+24+1));
				xmm[8] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+32));
				xmm[9] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+32+1));
				xmm[10] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+40));
				xmm[11] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+40+1));
				xmm[12] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+48));
				xmm[13] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+48+1));
				xmm[14] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+56));
				xmm[15] = _mm_loadu_si128((__m128i*)(refMain+deltaInt+56+1));



				if (deltaFract)
				{
					//refMainIndex        = 0+deltaInt+1;
					_mm_storeu_si128((__m128i*)(pDst+edstStride),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[0]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[1]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+8),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[2]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[3]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+16),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[4]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[5]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+24),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[6]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[7]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+32),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[8]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[9]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+40),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[10]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[11]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+48),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[12]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[13]),eAdd16)),5));
					_mm_storeu_si128((__m128i*)(pDst+edstStride+56),_mm_srai_epi16(_mm_add_epi16(_mm_mullo_epi16(edeltaFract_Minus32, xmm[14]), _mm_add_epi16(_mm_mullo_epi16(edeltaFract, xmm[15]),eAdd16)),5));
				}
				else
				{
					_mm_storeu_si128((__m128i *)(pDst+edstStride),xmm[0]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+8),xmm[2]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+16),xmm[4]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+24),xmm[6]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+32),xmm[8]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+40),xmm[10]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+48),xmm[12]);
					_mm_storeu_si128((__m128i *)(pDst+edstStride+56),xmm[14]);
				}
				edstStride+=dstStride;
			}
		}

		// Flip the block if this is the horizontal mode
		if (modeHor)
		{
			ETRI_Transpose64x64(pDst,dstStride);
		}
	}
}

#endif 
#if ETRI_SIMD_FIX_INTRA_PLANAR_PREDICTION
Void TComPrediction::xPredIntraPlanar4x4(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride)
{
    UInt width= 4;
    UInt height= 4;
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    Int blkSize = width; // UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[ width ] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    // Prepare intermediate variables used in interpolation
    bottomLeft = pSrc[blkSize*srcStride-1];
    topRight   = pSrc[blkSize-srcStride];

    __m128i xmm_bottomLeft = _mm_set1_epi32(bottomLeft);
    __m128i xmm_topRight = _mm_set1_epi32(topRight);

    __m128i xmm_topRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    __m128i xmm_leftColumn = _mm_setzero_si128();


    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc));
    _mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc),_mm_slli_epi32(xmm_leftColumn,shift1D));

    // Generate prediction signal
    for (k=0;k<blkSize;k++)
    {
        horPred = pLSrc[k] + offset2D;
        for (l=0;l<blkSize;l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
        }
    }
}
Void TComPrediction::xPredIntraPlanar8x8( Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride )
{
    UInt width= 8;
    UInt height= 8;
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    Int blkSize = width; // UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[ width ] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    // Prepare intermediate variables used in interpolation
    bottomLeft = pSrc[blkSize*srcStride-1];
    topRight   = pSrc[blkSize-srcStride];

    __m128i xmm_bottomLeft = _mm_set1_epi32(bottomLeft);
    __m128i xmm_topRight = _mm_set1_epi32(topRight);

    __m128i xmm_topRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    __m128i xmm_leftColumn = _mm_setzero_si128();

    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc));
    _mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc),_mm_slli_epi32(xmm_leftColumn,shift1D));

    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+4));
    _mm_storeu_si128((__m128i *)(bottomRow+4),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn+4),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow+4),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc+4),_mm_slli_epi32(xmm_leftColumn,shift1D));

    // Generate prediction signal
    for (k=0;k<blkSize;k++)
    {
        horPred = pLSrc[k] + offset2D;
        for (l=0;l<blkSize;l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
        }
    }
}
Void TComPrediction::xPredIntraPlanar16x16( Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride )
{
    UInt width= 16;
    UInt height= 16;
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    Int blkSize = width; // UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[ width ] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    // Prepare intermediate variables used in interpolation
    bottomLeft = pSrc[blkSize*srcStride-1];
    topRight   = pSrc[blkSize-srcStride];

    __m128i xmm_bottomLeft = _mm_set1_epi32(bottomLeft);
    __m128i xmm_topRight = _mm_set1_epi32(topRight);

    __m128i xmm_topRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    __m128i xmm_leftColumn = _mm_setzero_si128();

    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc));
    _mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc),_mm_slli_epi32(xmm_leftColumn,shift1D));

    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+4));
    _mm_storeu_si128((__m128i *)(bottomRow+4),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn+4),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow+4),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc+4),_mm_slli_epi32(xmm_leftColumn,shift1D));


    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+8));
    _mm_storeu_si128((__m128i *)(bottomRow+8),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn+8),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow+8),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc+8),_mm_slli_epi32(xmm_leftColumn,shift1D));


    xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+12));
    _mm_storeu_si128((__m128i *)(bottomRow+12),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
    _mm_storeu_si128((__m128i *)(rightColumn+12),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
    _mm_storeu_si128((__m128i *)(topRow+12),_mm_slli_epi32(xmm_topRow,shift1D));
    _mm_storeu_si128((__m128i *)(pLSrc+12),_mm_slli_epi32(xmm_leftColumn,shift1D));

    // Generate prediction signal
    for (k=0;k<blkSize;k++)
    {
        horPred = pLSrc[k] + offset2D;
        for (l=0;l<blkSize;l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
        }
    }
}
Void TComPrediction::xPredIntraPlanar32x32( Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride )
{
    UInt width= 32;
    UInt height= 32;
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    Int blkSize = width; // UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[ width ] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    // Prepare intermediate variables used in interpolation
    bottomLeft = pSrc[blkSize*srcStride-1];
    topRight   = pSrc[blkSize-srcStride];

    __m128i xmm_bottomLeft = _mm_set1_epi32(bottomLeft);
    __m128i xmm_topRight = _mm_set1_epi32(topRight);

    __m128i xmm_topRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    __m128i xmm_leftColumn = _mm_setzero_si128();

    for(k=0;k<blkSize;k+=16)
    {
        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k));
        _mm_storeu_si128((__m128i *)(bottomRow+k),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k),_mm_slli_epi32(xmm_leftColumn,shift1D));

        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+4));
        _mm_storeu_si128((__m128i *)(bottomRow+k+4),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+4),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+4),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+4),_mm_slli_epi32(xmm_leftColumn,shift1D));


        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+8));
        _mm_storeu_si128((__m128i *)(bottomRow+k+8),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+8),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+8),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+8),_mm_slli_epi32(xmm_leftColumn,shift1D));


        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+12));
        _mm_storeu_si128((__m128i *)(bottomRow+k+12),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+12),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+12),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+12),_mm_slli_epi32(xmm_leftColumn,shift1D));
    }
    // Generate prediction signal
    for (k=0;k<blkSize;k++)
    {
        horPred = pLSrc[k] + offset2D;
        for (l=0;l<blkSize;l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
        }
    }
}
Void TComPrediction::xPredIntraPlanar64x64( Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride )
{
    UInt width= 64;
    UInt height= 64;

    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int  topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    Int blkSize = width; // UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[ width ] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    // Prepare intermediate variables used in interpolation
    bottomLeft = pSrc[blkSize*srcStride-1];
    topRight   = pSrc[blkSize-srcStride];

    __m128i xmm_bottomLeft = _mm_set1_epi32(bottomLeft);
    __m128i xmm_topRight = _mm_set1_epi32(topRight);

    __m128i xmm_topRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
    __m128i xmm_leftColumn = _mm_setzero_si128();

    for(k=0;k<blkSize;k+=16)
    {
        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k));
        _mm_storeu_si128((__m128i *)(bottomRow+k),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k),_mm_slli_epi32(xmm_leftColumn,shift1D));

        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+4));
        _mm_storeu_si128((__m128i *)(bottomRow+k+4),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+4),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+4),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+4),_mm_slli_epi32(xmm_leftColumn,shift1D));


        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+8));
        _mm_storeu_si128((__m128i *)(bottomRow+k+8),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+8),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+8),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+8),_mm_slli_epi32(xmm_leftColumn,shift1D));


        xmm_topRow = _mm_loadu_si128((__m128i *)(pSrc+k+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
        xmm_leftColumn = _mm_loadu_si128((__m128i *)(pLSrc+k+12));
        _mm_storeu_si128((__m128i *)(bottomRow+k+12),_mm_sub_epi32(xmm_bottomLeft,xmm_topRow));
        _mm_storeu_si128((__m128i *)(rightColumn+k+12),_mm_sub_epi32(xmm_topRight,xmm_leftColumn));
        _mm_storeu_si128((__m128i *)(topRow+k+12),_mm_slli_epi32(xmm_topRow,shift1D));
        _mm_storeu_si128((__m128i *)(pLSrc+k+12),_mm_slli_epi32(xmm_leftColumn,shift1D));
    }


    // Generate prediction signal
    for (k=0;k<blkSize;k++)
    {
        horPred = pLSrc[k] + offset2D;
        for (l=0;l<blkSize;l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
        }
    }
}
#else 
Void TComPrediction::xPredIntraPlanar4( Int* pSrc, Int*pColPixs, Int srcStride, Pel* rpDst, Int dstStride )
{
	UInt width= 4;
	UInt height= 4;
	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	Int blkSize = width; // UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[ width ] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	// Prepare intermediate variables used in interpolation
	bottomLeft = pSrc[blkSize*srcStride-1];
	topRight   = pSrc[blkSize-srcStride];

	__m128i ebottomLeft = _mm_set1_epi32(bottomLeft);
	__m128i etopRight = _mm_set1_epi32(topRight);

	__m128i etopRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	__m128i eleftColumn = _mm_setzero_si128();


	etopRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs));
	_mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs),_mm_slli_epi32(eleftColumn,shift1D));

	// Generate prediction signal
	for (k=0;k<blkSize;k++)
	{
		horPred = pColPixs[k] + offset2D;
		for (l=0;l<blkSize;l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
		}
	}
}
Void TComPrediction::xPredIntraPlanar8( Int* pSrc, Int*pColPixs, Int srcStride, Pel* rpDst, Int dstStride)
{
	UInt width= 8;
	UInt height= 8;
	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	Int blkSize = width; // UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[ width ] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	// Prepare intermediate variables used in interpolation
	bottomLeft = pSrc[blkSize*srcStride-1];
	topRight   = pSrc[blkSize-srcStride];

	__m128i ebottomLeft = _mm_set1_epi32(bottomLeft);
	__m128i etopRight = _mm_set1_epi32(topRight);

	__m128i etopRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	__m128i eleftColumn = _mm_setzero_si128();

	etopRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs));
	_mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs),_mm_slli_epi32(eleftColumn,shift1D));

	etopRow = _mm_loadu_si128((__m128i *)(pSrc+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+4));
	_mm_storeu_si128((__m128i *)(bottomRow+4),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn+4),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow+4),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs+4),_mm_slli_epi32(eleftColumn,shift1D));

	// Generate prediction signal
	for (k=0;k<blkSize;k++)
	{
		horPred = pColPixs[k] + offset2D;
		for (l=0;l<blkSize;l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
		}
	}
}
Void TComPrediction::xPredIntraPlanar16( Int* pSrc, Int*pColPixs, Int srcStride, Pel* rpDst, Int dstStride )
{
	UInt width= 16;
	UInt height= 16;
	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	Int blkSize = width; // UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[ width ] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	// Prepare intermediate variables used in interpolation
	bottomLeft = pSrc[blkSize*srcStride-1];
	topRight   = pSrc[blkSize-srcStride];

	__m128i ebottomLeft = _mm_set1_epi32(bottomLeft);
	__m128i etopRight = _mm_set1_epi32(topRight);

	__m128i etopRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	__m128i eleftColumn = _mm_setzero_si128();

	etopRow = _mm_loadu_si128((__m128i *)(pSrc-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs));
	_mm_storeu_si128((__m128i *)(bottomRow),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs),_mm_slli_epi32(eleftColumn,shift1D));

	etopRow = _mm_loadu_si128((__m128i *)(pSrc+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+4));
	_mm_storeu_si128((__m128i *)(bottomRow+4),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn+4),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow+4),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs+4),_mm_slli_epi32(eleftColumn,shift1D));


	etopRow = _mm_loadu_si128((__m128i *)(pSrc+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+8));
	_mm_storeu_si128((__m128i *)(bottomRow+8),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn+8),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow+8),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs+8),_mm_slli_epi32(eleftColumn,shift1D));


	etopRow = _mm_loadu_si128((__m128i *)(pSrc+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+12));
	_mm_storeu_si128((__m128i *)(bottomRow+12),_mm_sub_epi32(ebottomLeft,etopRow));
	_mm_storeu_si128((__m128i *)(rightColumn+12),_mm_sub_epi32(etopRight,eleftColumn));
	_mm_storeu_si128((__m128i *)(topRow+12),_mm_slli_epi32(etopRow,shift1D));
	_mm_storeu_si128((__m128i *)(pColPixs+12),_mm_slli_epi32(eleftColumn,shift1D));

	// Generate prediction signal
	for (k=0;k<blkSize;k++)
	{
		horPred = pColPixs[k] + offset2D;
		for (l=0;l<blkSize;l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
		}
	}
}
Void TComPrediction::xPredIntraPlanar32( Int* pSrc, Int*pColPixs, Int srcStride, Pel* rpDst, Int dstStride )
{
	UInt width= 32;
	UInt height= 32;
	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	Int blkSize = width; // UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[ width ] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	// Prepare intermediate variables used in interpolation
	bottomLeft = pSrc[blkSize*srcStride-1];
	topRight   = pSrc[blkSize-srcStride];

	__m128i ebottomLeft = _mm_set1_epi32(bottomLeft);
	__m128i etopRight = _mm_set1_epi32(topRight);

	__m128i etopRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	__m128i eleftColumn = _mm_setzero_si128();

	for(k=0;k<blkSize;k+=16)
	{
		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k));
		_mm_storeu_si128((__m128i *)(bottomRow+k),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k),_mm_slli_epi32(eleftColumn,shift1D));

		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+4));
		_mm_storeu_si128((__m128i *)(bottomRow+k+4),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+4),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+4),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+4),_mm_slli_epi32(eleftColumn,shift1D));


		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+8));
		_mm_storeu_si128((__m128i *)(bottomRow+k+8),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+8),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+8),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+8),_mm_slli_epi32(eleftColumn,shift1D));


		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+12));
		_mm_storeu_si128((__m128i *)(bottomRow+k+12),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+12),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+12),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+12),_mm_slli_epi32(eleftColumn,shift1D));
	}
	// Generate prediction signal
	for (k=0;k<blkSize;k++)
	{
		horPred = pColPixs[k] + offset2D;
		for (l=0;l<blkSize;l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
		}
	}
}
Void TComPrediction::xPredIntraPlanar64( Int* pSrc, Int*pColPixs, Int srcStride, Pel* rpDst, Int dstStride )
{
	UInt width= 64;
	UInt height= 64;

	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int  topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	Int blkSize = width; // UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[ width ] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	// Prepare intermediate variables used in interpolation
	bottomLeft = pSrc[blkSize*srcStride-1];
	topRight   = pSrc[blkSize-srcStride];

	__m128i ebottomLeft = _mm_set1_epi32(bottomLeft);
	__m128i etopRight = _mm_set1_epi32(topRight);

	__m128i etopRow = _mm_setzero_si128(); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
	__m128i eleftColumn = _mm_setzero_si128();

	for(k=0;k<blkSize;k+=16)
	{
		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k));
		_mm_storeu_si128((__m128i *)(bottomRow+k),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k),_mm_slli_epi32(eleftColumn,shift1D));

		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+4-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+4));
		_mm_storeu_si128((__m128i *)(bottomRow+k+4),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+4),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+4),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+4),_mm_slli_epi32(eleftColumn,shift1D));


		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+8-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+8));
		_mm_storeu_si128((__m128i *)(bottomRow+k+8),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+8),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+8),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+8),_mm_slli_epi32(eleftColumn,shift1D));


		etopRow = _mm_loadu_si128((__m128i *)(pSrc+k+12-srcStride)); //(pSrc+4*k-srcStride) = (topRow+4*k-srcStride)
		eleftColumn = _mm_loadu_si128((__m128i *)(pColPixs+k+12));
		_mm_storeu_si128((__m128i *)(bottomRow+k+12),_mm_sub_epi32(ebottomLeft,etopRow));
		_mm_storeu_si128((__m128i *)(rightColumn+k+12),_mm_sub_epi32(etopRight,eleftColumn));
		_mm_storeu_si128((__m128i *)(topRow+k+12),_mm_slli_epi32(etopRow,shift1D));
		_mm_storeu_si128((__m128i *)(pColPixs+k+12),_mm_slli_epi32(eleftColumn,shift1D));
	}


	// Generate prediction signal
	for (k=0;k<blkSize;k++)
	{
		horPred = pColPixs[k] + offset2D;
		for (l=0;l<blkSize;l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
		}
	}
}
#endif 
#else
// ====================================================================================================================
// HM Original Public member functions
// ====================================================================================================================
/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 * This function is called by the original Intra Prediction functions in TComPrediction.
 */
Void TComPrediction::xPredIntraPlanar(Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height)
{
	assert(width == height);

	Int k, l, bottomLeft, topRight;
	Int horPred;
	Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
	UInt blkSize = width;
	UInt offset2D = width;
	UInt shift1D = g_aucConvertToBit[width] + 2;
	UInt shift2D = shift1D + 1;

	// Get left and above reference column and row
	for (k = 0; k<blkSize + 1; k++)
	{
		topRow[k] = pSrc[k - srcStride];
		leftColumn[k] = pSrc[k*srcStride - 1];
	}

	// Prepare intermediate variables used in interpolation
	bottomLeft = leftColumn[blkSize];
	topRight = topRow[blkSize];
	for (k = 0; k<blkSize; k++)
	{
		bottomRow[k] = bottomLeft - topRow[k];
		rightColumn[k] = topRight - leftColumn[k];
		topRow[k] <<= shift1D;
		leftColumn[k] <<= shift1D;
	}

	// Generate prediction signal
	for (k = 0; k<blkSize; k++)
	{
		horPred = leftColumn[k] + offset2D;
		for (l = 0; l<blkSize; l++)
		{
			horPred += rightColumn[k];
			topRow[l] += bottomRow[l];
			rpDst[k*dstStride + l] = ((horPred + topRow[l]) >> shift2D);
		}
	}
}

/**
* HM Original Function
* This function is called by the original Intra Prediction functions in TComPrediction.
*/
Void TComPrediction::xDCPredFiltering(Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight)
{
	Pel* pDst = rpDst;
	Int x, y, iDstStride2, iSrcStride2;

	// boundary pixels processing
	pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

	for ( x = 1; x < iWidth; x++ )
	{
		pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
	}

	for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
	{
		pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
	}

	return;
}


#endif

















//! \}
