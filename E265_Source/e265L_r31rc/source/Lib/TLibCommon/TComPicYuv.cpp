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

/** \file     TComPicYuv.cpp
    \brief    picture YUV buffer class
*/

#include <cstdlib>
#include <assert.h>
#include <memory.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

#include "TComPicYuv.h"

//! \ingroup TLibCommon
//! \{

TComPicYuv::TComPicYuv()
{
  m_apiPicBufY      = NULL;   // Buffer (including margin)
  m_apiPicBufU      = NULL;
  m_apiPicBufV      = NULL;
  
  m_piPicOrgY       = NULL;    // m_apiPicBufY + m_iMarginLuma*getStride() + m_iMarginLuma
  m_piPicOrgU       = NULL;
  m_piPicOrgV       = NULL;
  
  m_bIsBorderExtended = false;
#if ETRI_MULTITHREAD_2
	// gplusplus
	em_nPoc			= -1;
	em_bUsed			= false;
#endif
}

TComPicYuv::~TComPicYuv()
{
}

#if ETRI_EM_OPERATION_OPTIMIZATION
Void TComPicYuv::create(Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth)
{
	m_iPicWidth = iPicWidth;
	m_iPicHeight = iPicHeight;

	// --> After config finished!
	m_iCuWidth = uiMaxCUWidth;
	m_iCuHeight = uiMaxCUHeight;

	Int numCuInWidth = (m_iPicWidth >> 6) + ((m_iPicWidth & 0x3F) != 0);
	Int numCuInHeight = (m_iPicHeight >> 6) + ((m_iPicHeight & 0x3F) != 0);

	m_iLumaMarginX = g_uiMaxCUWidth + 16; // for 16-byte alignment
	m_iLumaMarginY = g_uiMaxCUHeight + 16;  // margin for 8-tap filter and infinite padding

	m_iChromaMarginX = m_iLumaMarginX >> 1;
	m_iChromaMarginY = m_iLumaMarginY >> 1;

	m_apiPicBufY = (Pel*)xMalloc(Pel, (m_iPicWidth + (m_iLumaMarginX << 1)) * (m_iPicHeight + (m_iLumaMarginY << 1)));
	m_apiPicBufU = (Pel*)xMalloc(Pel, ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)));
	m_apiPicBufV = (Pel*)xMalloc(Pel, ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)));

	m_piPicOrgY = m_apiPicBufY + m_iLumaMarginY   * getStride() + m_iLumaMarginX;
	m_piPicOrgU = m_apiPicBufU + m_iChromaMarginY * getCStride() + m_iChromaMarginX;
	m_piPicOrgV = m_apiPicBufV + m_iChromaMarginY * getCStride() + m_iChromaMarginX;

	m_bIsBorderExtended = false;

	m_cuOffsetY = new Int[numCuInWidth * numCuInHeight];
	m_cuOffsetC = new Int[numCuInWidth * numCuInHeight];
	for (Int cuRow = 0; cuRow < numCuInHeight; cuRow++)
	{
		for (Int cuCol = 0; cuCol < numCuInWidth; cuCol++)
		{
			m_cuOffsetY[cuRow * numCuInWidth + cuCol] = ((getStride() * cuRow) << 6) + (cuCol << 6);
			m_cuOffsetC[cuRow * numCuInWidth + cuCol] = ((getCStride() * cuRow) << 5) + (cuCol << 5);
		}
	}

	m_buOffsetY = new Int[(size_t)1 << (2 << 2)];
	m_buOffsetC = new Int[(size_t)1 << (2 << 2)];
	for (Int buRow = 0; buRow < (1 << uiMaxCUDepth); buRow++)
	{
		for (Int buCol = 0; buCol < (1 << uiMaxCUDepth); buCol++)
		{
			m_buOffsetY[(buRow << uiMaxCUDepth) + buCol] = ((getStride() * buRow) << 2) + (buCol << 2);
			m_buOffsetC[(buRow << uiMaxCUDepth) + buCol] = ((getCStride() * buRow) << 1) + (buCol << 1);
		}
	}
	return;
}
#else
Void TComPicYuv::create( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth )
{
  m_iPicWidth       = iPicWidth;
  m_iPicHeight      = iPicHeight;
  
  // --> After config finished!
  m_iCuWidth        = uiMaxCUWidth;
  m_iCuHeight       = uiMaxCUHeight;

  Int numCuInWidth  = m_iPicWidth  / m_iCuWidth  + (m_iPicWidth  % m_iCuWidth  != 0);
  Int numCuInHeight = m_iPicHeight / m_iCuHeight + (m_iPicHeight % m_iCuHeight != 0);
  
  m_iLumaMarginX    = g_uiMaxCUWidth  + 16; // for 16-byte alignment
  m_iLumaMarginY    = g_uiMaxCUHeight + 16;  // margin for 8-tap filter and infinite padding
  
  m_iChromaMarginX  = m_iLumaMarginX>>1;
  m_iChromaMarginY  = m_iLumaMarginY>>1;
  
  m_apiPicBufY      = (Pel*)xMalloc( Pel, ( m_iPicWidth       + (m_iLumaMarginX  <<1)) * ( m_iPicHeight       + (m_iLumaMarginY  <<1)));
  m_apiPicBufU      = (Pel*)xMalloc( Pel, ((m_iPicWidth >> 1) + (m_iChromaMarginX<<1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY<<1)));
  m_apiPicBufV      = (Pel*)xMalloc( Pel, ((m_iPicWidth >> 1) + (m_iChromaMarginX<<1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY<<1)));
  
  m_piPicOrgY       = m_apiPicBufY + m_iLumaMarginY   * getStride()  + m_iLumaMarginX;
  m_piPicOrgU       = m_apiPicBufU + m_iChromaMarginY * getCStride() + m_iChromaMarginX;
  m_piPicOrgV       = m_apiPicBufV + m_iChromaMarginY * getCStride() + m_iChromaMarginX;
  
  m_bIsBorderExtended = false;
  
  m_cuOffsetY = new Int[numCuInWidth * numCuInHeight];
  m_cuOffsetC = new Int[numCuInWidth * numCuInHeight];
  for (Int cuRow = 0; cuRow < numCuInHeight; cuRow++)
  {
    for (Int cuCol = 0; cuCol < numCuInWidth; cuCol++)
    {
      m_cuOffsetY[cuRow * numCuInWidth + cuCol] = getStride() * cuRow * m_iCuHeight + cuCol * m_iCuWidth;
      m_cuOffsetC[cuRow * numCuInWidth + cuCol] = getCStride() * cuRow * (m_iCuHeight / 2) + cuCol * (m_iCuWidth / 2);
    }
  }
  
  m_buOffsetY = new Int[(size_t)1 << (2 * uiMaxCUDepth)];
  m_buOffsetC = new Int[(size_t)1 << (2 * uiMaxCUDepth)];
  for (Int buRow = 0; buRow < (1 << uiMaxCUDepth); buRow++)
  {
    for (Int buCol = 0; buCol < (1 << uiMaxCUDepth); buCol++)
    {
      m_buOffsetY[(buRow << uiMaxCUDepth) + buCol] = getStride() * buRow * (uiMaxCUHeight >> uiMaxCUDepth) + buCol * (uiMaxCUWidth  >> uiMaxCUDepth);
      m_buOffsetC[(buRow << uiMaxCUDepth) + buCol] = getCStride() * buRow * (uiMaxCUHeight / 2 >> uiMaxCUDepth) + buCol * (uiMaxCUWidth / 2 >> uiMaxCUDepth);
    }
  }
  return;
}
#endif

Void TComPicYuv::destroy()
{
  m_piPicOrgY       = NULL;
  m_piPicOrgU       = NULL;
  m_piPicOrgV       = NULL;
  
  if( m_apiPicBufY ){ xFree( m_apiPicBufY );    m_apiPicBufY = NULL; }
  if( m_apiPicBufU ){ xFree( m_apiPicBufU );    m_apiPicBufU = NULL; }
  if( m_apiPicBufV ){ xFree( m_apiPicBufV );    m_apiPicBufV = NULL; }

  delete[] m_cuOffsetY;
  delete[] m_cuOffsetC;
  delete[] m_buOffsetY;
  delete[] m_buOffsetC;
}

Void TComPicYuv::createLuma( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth )
{
  m_iPicWidth       = iPicWidth;
  m_iPicHeight      = iPicHeight;
  
  // --> After config finished!
  m_iCuWidth        = uiMaxCUWidth;
  m_iCuHeight       = uiMaxCUHeight;
  
  Int numCuInWidth  = m_iPicWidth  / m_iCuWidth  + (m_iPicWidth  % m_iCuWidth  != 0);
  Int numCuInHeight = m_iPicHeight / m_iCuHeight + (m_iPicHeight % m_iCuHeight != 0);
  
  m_iLumaMarginX    = g_uiMaxCUWidth  + 16; // for 16-byte alignment
  m_iLumaMarginY    = g_uiMaxCUHeight + 16;  // margin for 8-tap filter and infinite padding
  
  m_apiPicBufY      = (Pel*)xMalloc( Pel, ( m_iPicWidth       + (m_iLumaMarginX  <<1)) * ( m_iPicHeight       + (m_iLumaMarginY  <<1)));
  m_piPicOrgY       = m_apiPicBufY + m_iLumaMarginY   * getStride()  + m_iLumaMarginX;
  
  m_cuOffsetY = new Int[numCuInWidth * numCuInHeight];
  m_cuOffsetC = NULL;
  for (Int cuRow = 0; cuRow < numCuInHeight; cuRow++)
  {
    for (Int cuCol = 0; cuCol < numCuInWidth; cuCol++)
    {
      m_cuOffsetY[cuRow * numCuInWidth + cuCol] = getStride() * cuRow * m_iCuHeight + cuCol * m_iCuWidth;
    }
  }
  
  m_buOffsetY = new Int[(size_t)1 << (2 * uiMaxCUDepth)];
  m_buOffsetC = NULL;
  for (Int buRow = 0; buRow < (1 << uiMaxCUDepth); buRow++)
  {
    for (Int buCol = 0; buCol < (1 << uiMaxCUDepth); buCol++)
    {
      m_buOffsetY[(buRow << uiMaxCUDepth) + buCol] = getStride() * buRow * (uiMaxCUHeight >> uiMaxCUDepth) + buCol * (uiMaxCUWidth  >> uiMaxCUDepth);
    }
  }
  return;
}

Void TComPicYuv::destroyLuma()
{
  m_piPicOrgY       = NULL;
  
  if( m_apiPicBufY ){ xFree( m_apiPicBufY );    m_apiPicBufY = NULL; }
  
  delete[] m_cuOffsetY;
  delete[] m_buOffsetY;
}

Void  TComPicYuv::copyToPic (TComPicYuv*  pcPicYuvDst)
{
  assert( m_iPicWidth  == pcPicYuvDst->getWidth()  );
  assert( m_iPicHeight == pcPicYuvDst->getHeight() );
#if ETRI_SIMD_COPY_TO_PIC
  Pel* DstY = pcPicYuvDst->getBufY();
  Pel* SrcY = m_apiPicBufY;
  Pel* DstU = pcPicYuvDst->getBufU();
  Pel* SrcU = m_apiPicBufU;
  Pel* DstV = pcPicYuvDst->getBufV();
  Pel* SrcV = m_apiPicBufV;
  UInt StrideY = m_iPicWidth + (m_iLumaMarginX << 1);
  UInt iHeight = m_iPicHeight + (m_iLumaMarginY << 1);
  UInt iWidth = m_iPicWidth + (m_iLumaMarginX << 1);
  UInt iHeightUV = (m_iPicHeight >> 1) + (m_iChromaMarginY << 1);
  UInt iWidthUV = (m_iPicWidth >> 1) + (m_iChromaMarginX << 1);
  UInt StrideUV = (m_iPicWidth >> 1) + (m_iChromaMarginX << 1);
  for (int j = 0; j < iHeight; j++)
  {
      for (int i = 0; i < iWidth; i += 8)
          _mm_storeu_si128((__m128i *)&DstY[i], _mm_loadu_si128((__m128i *)&SrcY[i]));
      DstY += StrideY;
      SrcY += StrideY;
  }
  for (int j = 0; j < iHeightUV; j++)
  {
      for (int i = 0; i < iWidthUV; i += 8)
      {
          _mm_storeu_si128((__m128i *)&DstU[i], _mm_loadu_si128((__m128i *)&SrcU[i]));
          _mm_storeu_si128((__m128i *)&DstV[i], _mm_loadu_si128((__m128i *)&SrcV[i]));
      }
      DstU += StrideUV;
      SrcU += StrideUV;
      DstV += StrideUV;
      SrcV += StrideUV;
  }
#else 
  ::memcpy ( pcPicYuvDst->getBufY(), m_apiPicBufY, sizeof (Pel) * ( m_iPicWidth       + (m_iLumaMarginX   << 1)) * ( m_iPicHeight       + (m_iLumaMarginY   << 1)) );
  ::memcpy ( pcPicYuvDst->getBufU(), m_apiPicBufU, sizeof (Pel) * ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)) );
  ::memcpy ( pcPicYuvDst->getBufV(), m_apiPicBufV, sizeof (Pel) * ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)) );
#endif 
  return;
}

#if ETRI_COPYTOPIC_MULTITHREAD

Void TComPicYuv::qrCopyToPic(TComPicYuv*  pcPicYuvDst, int nThread, int index)
{
	assert(m_iPicWidth == pcPicYuvDst->getWidth());
	assert(m_iPicHeight == pcPicYuvDst->getHeight());
#if ETRI_SIMD_COPY_TO_PIC
	Pel* DstY = pcPicYuvDst->getBufY();
	Pel* SrcY = m_apiPicBufY;
	Pel* DstU = pcPicYuvDst->getBufU();
	Pel* SrcU = m_apiPicBufU;
	Pel* DstV = pcPicYuvDst->getBufV();
	Pel* SrcV = m_apiPicBufV;
	UInt StrideY = m_iPicWidth + (m_iLumaMarginX << 1);
	UInt iHeight = m_iPicHeight + (m_iLumaMarginY << 1);
	UInt iWidth = m_iPicWidth + (m_iLumaMarginX << 1);
	UInt iHeightUV = (m_iPicHeight >> 1) + (m_iChromaMarginY << 1);
	UInt iWidthUV = (m_iPicWidth >> 1) + (m_iChromaMarginX << 1);
	UInt StrideUV = (m_iPicWidth >> 1) + (m_iChromaMarginX << 1);


	UInt startHeight = iHeight / nThread * index;
	UInt endHeight = iHeight / nThread * (index + 1);
	UInt startHeightUV = iHeightUV / nThread * index;
	UInt endHeightUV = iHeightUV / nThread * (index + 1);
	
	DstY += startHeight * StrideY;
	SrcY += startHeight * StrideY;

	DstU += startHeightUV * StrideUV;
	SrcU += startHeightUV * StrideUV;
	DstV += startHeightUV * StrideUV;
	SrcV += startHeightUV * StrideUV;

	for (int j = startHeight; j < endHeight; j++)
	{
		for (int i = 0; i < iWidth; i += 8)
			_mm_storeu_si128((__m128i *)&DstY[i], _mm_loadu_si128((__m128i *)&SrcY[i]));
		DstY += StrideY;
		SrcY += StrideY;
	}
	for (int j = startHeightUV; j < endHeightUV; j++)
	{
		for (int i = 0; i < iWidthUV; i += 8)
		{
			_mm_storeu_si128((__m128i *)&DstU[i], _mm_loadu_si128((__m128i *)&SrcU[i]));
			_mm_storeu_si128((__m128i *)&DstV[i], _mm_loadu_si128((__m128i *)&SrcV[i]));
		}
		DstU += StrideUV;
		SrcU += StrideUV;
		DstV += StrideUV;
		SrcV += StrideUV;
	}
#else 
	::memcpy(pcPicYuvDst->getBufY(), m_apiPicBufY, sizeof (Pel)* (m_iPicWidth + (m_iLumaMarginX << 1)) * (m_iPicHeight + (m_iLumaMarginY << 1)));
	::memcpy(pcPicYuvDst->getBufU(), m_apiPicBufU, sizeof (Pel)* ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)));
	::memcpy(pcPicYuvDst->getBufV(), m_apiPicBufV, sizeof (Pel)* ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)));
#endif 
	return;
}
#endif

Void  TComPicYuv::copyToPicLuma (TComPicYuv*  pcPicYuvDst)
{
  assert( m_iPicWidth  == pcPicYuvDst->getWidth()  );
  assert( m_iPicHeight == pcPicYuvDst->getHeight() );
  
  ::memcpy ( pcPicYuvDst->getBufY(), m_apiPicBufY, sizeof (Pel) * ( m_iPicWidth       + (m_iLumaMarginX   << 1)) * ( m_iPicHeight       + (m_iLumaMarginY   << 1)) );
  return;
}

Void  TComPicYuv::copyToPicCb (TComPicYuv*  pcPicYuvDst)
{
  assert( m_iPicWidth  == pcPicYuvDst->getWidth()  );
  assert( m_iPicHeight == pcPicYuvDst->getHeight() );
  
  ::memcpy ( pcPicYuvDst->getBufU(), m_apiPicBufU, sizeof (Pel) * ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)) );
  return;
}

Void  TComPicYuv::copyToPicCr (TComPicYuv*  pcPicYuvDst)
{
  assert( m_iPicWidth  == pcPicYuvDst->getWidth()  );
  assert( m_iPicHeight == pcPicYuvDst->getHeight() );
  
  ::memcpy ( pcPicYuvDst->getBufV(), m_apiPicBufV, sizeof (Pel) * ((m_iPicWidth >> 1) + (m_iChromaMarginX << 1)) * ((m_iPicHeight >> 1) + (m_iChromaMarginY << 1)) );
  return;
}

Void TComPicYuv::extendPicBorder ()
{
  if ( m_bIsBorderExtended ) return;
  
  xExtendPicCompBorder( getLumaAddr(), getStride(),  getWidth(),      getHeight(),      m_iLumaMarginX,   m_iLumaMarginY   );
  xExtendPicCompBorder( getCbAddr()  , getCStride(), getWidth() >> 1, getHeight() >> 1, m_iChromaMarginX, m_iChromaMarginY );
  xExtendPicCompBorder( getCrAddr()  , getCStride(), getWidth() >> 1, getHeight() >> 1, m_iChromaMarginX, m_iChromaMarginY );
  
  m_bIsBorderExtended = true;
}

Void TComPicYuv::xExtendPicCompBorder  (Pel* piTxt, Int iStride, Int iWidth, Int iHeight, Int iMarginX, Int iMarginY)
{
#if ETRI_SIMD_EXTENEDED_PIC_BORDER
    Int   x, y;
    Pel*  pi;
    Pel*  dst;

    __m128i xmm_pi_left[4];
    __m128i xmm_pi_right[4];
    __m128i xmm_bottom;



    pi = piTxt;
    for ( y = 0; y < iHeight; y += 4)
    {
        for (x = 0; x < iMarginX; x += 8)
        {
            //left
            xmm_pi_left[0] = _mm_loadu_si128((__m128i *) (pi - iMarginX + 0 * iStride + x));
            xmm_pi_left[1] = _mm_loadu_si128((__m128i *) (pi - iMarginX + 1 * iStride + x));
            xmm_pi_left[2] = _mm_loadu_si128((__m128i *) (pi - iMarginX + 2 * iStride + x));
            xmm_pi_left[3] = _mm_loadu_si128((__m128i *) (pi - iMarginX + 3 * iStride + x));


            xmm_pi_left[0] = _mm_set1_epi16(pi[0 + 0 * iStride]);
            xmm_pi_left[1] = _mm_set1_epi16(pi[0 + 1 * iStride]);
            xmm_pi_left[2] = _mm_set1_epi16(pi[0 + 2 * iStride]);
            xmm_pi_left[3] = _mm_set1_epi16(pi[0 + 3 * iStride]);


            //right		
            xmm_pi_right[0] = _mm_loadu_si128((__m128i *) (pi + iWidth + 0 * iStride + x));
            xmm_pi_right[1] = _mm_loadu_si128((__m128i *) (pi + iWidth + 1 * iStride + x));
            xmm_pi_right[2] = _mm_loadu_si128((__m128i *) (pi + iWidth + 2 * iStride + x));
            xmm_pi_right[3] = _mm_loadu_si128((__m128i *) (pi + iWidth + 3 * iStride + x));


            xmm_pi_right[0] = _mm_set1_epi16(pi[iWidth + 0 * iStride - 1]);
            xmm_pi_right[1] = _mm_set1_epi16(pi[iWidth + 1 * iStride - 1]);
            xmm_pi_right[2] = _mm_set1_epi16(pi[iWidth + 2 * iStride - 1]);
            xmm_pi_right[3] = _mm_set1_epi16(pi[iWidth + 3 * iStride - 1]);


            //left
            _mm_storeu_si128((__m128i *) (pi - iMarginX + 0 * iStride + x), xmm_pi_left[0]);
            _mm_storeu_si128((__m128i *) (pi - iMarginX + 1 * iStride + x), xmm_pi_left[1]);
            _mm_storeu_si128((__m128i *) (pi - iMarginX + 2 * iStride + x), xmm_pi_left[2]);
            _mm_storeu_si128((__m128i *) (pi - iMarginX + 3 * iStride + x), xmm_pi_left[3]);


            //right
            _mm_storeu_si128((__m128i *) (pi + iWidth + 0 * iStride + x), xmm_pi_right[0]);
            _mm_storeu_si128((__m128i *) (pi + iWidth + 1 * iStride + x), xmm_pi_right[1]);
            _mm_storeu_si128((__m128i *) (pi + iWidth + 2 * iStride + x), xmm_pi_right[2]);
            _mm_storeu_si128((__m128i *) (pi + iWidth + 3 * iStride + x), xmm_pi_right[3]);


}
        pi += 4 * iStride;
    }

    pi -= (iStride + iMarginX);

    for (y = 0; y < iMarginY; y += 4)
    {
        dst = pi + (y + 1) * iStride;

        for (x = 0; x < (iWidth + (iMarginX << 1)); x += 8)
        {
            xmm_bottom = _mm_loadu_si128((__m128i *) (pi + x));

            _mm_storeu_si128((__m128i *)(dst + 0 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst + 1 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst + 2 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst + 3 * (iStride)+x), xmm_bottom);

        }
    }


    pi -= ((iHeight - 1) * iStride);

    for (y = 0; y < iMarginY; y += 4)
    {
        dst = pi - (y + 1)*iStride;
        for (x = 0; x < (iWidth + (iMarginX << 1)); x += 8)
        {
            xmm_bottom = _mm_loadu_si128((__m128i *) (pi + x));

            _mm_storeu_si128((__m128i *)(dst - 0 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst - 1 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst - 2 * (iStride)+x), xmm_bottom);
            _mm_storeu_si128((__m128i *)(dst - 3 * (iStride)+x), xmm_bottom);
        }
    }
#else 
  Int   x, y;
  Pel*  pi;
  
  pi = piTxt;
  for ( y = 0; y < iHeight; y++)
  {
    for ( x = 0; x < iMarginX; x++ )
    {
      pi[ -iMarginX + x ] = pi[0];
      pi[    iWidth + x ] = pi[iWidth-1];
    }
    pi += iStride;
  }
  
  pi -= (iStride + iMarginX);
  for ( y = 0; y < iMarginY; y++ )
  {
    ::memcpy( pi + (y+1)*iStride, pi, sizeof(Pel)*(iWidth + (iMarginX<<1)) );
  }
  
  pi -= ((iHeight-1) * iStride);
  for ( y = 0; y < iMarginY; y++ )
  {
    ::memcpy( pi - (y+1)*iStride, pi, sizeof(Pel)*(iWidth + (iMarginX<<1)) );
  }
#endif 
}


Void TComPicYuv::dump (Char* pFileName, Bool bAdd)
{
  FILE* pFile;
  if (!bAdd)
  {
    pFile = fopen (pFileName, "wb");
  }
  else
  {
    pFile = fopen (pFileName, "ab");
  }
  
  Int     shift = g_bitDepthY-8;
  Int     offset = (shift>0)?(1<<(shift-1)):0;
  
  Int   x, y;
  UChar uc;
  
  Pel*  piY   = getLumaAddr();
  Pel*  piCb  = getCbAddr();
  Pel*  piCr  = getCrAddr();
  
  for ( y = 0; y < m_iPicHeight; y++ )
  {
    for ( x = 0; x < m_iPicWidth; x++ )
    {
      uc = (UChar)Clip3<Pel>(0, 255, (piY[x]+offset)>>shift);
      
      fwrite( &uc, sizeof(UChar), 1, pFile );
    }
    piY += getStride();
  }
  
  shift = g_bitDepthC-8;
  offset = (shift>0)?(1<<(shift-1)):0;

  for ( y = 0; y < m_iPicHeight >> 1; y++ )
  {
    for ( x = 0; x < m_iPicWidth >> 1; x++ )
    {
      uc = (UChar)Clip3<Pel>(0, 255, (piCb[x]+offset)>>shift);
      fwrite( &uc, sizeof(UChar), 1, pFile );
    }
    piCb += getCStride();
  }
  
  for ( y = 0; y < m_iPicHeight >> 1; y++ )
  {
    for ( x = 0; x < m_iPicWidth >> 1; x++ )
    {
      uc = (UChar)Clip3<Pel>(0, 255, (piCr[x]+offset)>>shift);
      fwrite( &uc, sizeof(UChar), 1, pFile );
    }
    piCr += getCStride();
  }
  
  fclose(pFile);
}


//! \}
