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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/

#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComInterpolationFilter.h"

//! \ingroup TLibCommon
//! \{

TComYuv::TComYuv()
{
  m_apiBufY = NULL;
  m_apiBufU = NULL;
  m_apiBufV = NULL;
}

TComYuv::~TComYuv()
{
}

Void TComYuv::create( UInt iWidth, UInt iHeight )
{
  // memory allocation
  m_apiBufY  = (Pel*)xMalloc( Pel, iWidth*iHeight    );
  m_apiBufU  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );
  m_apiBufV  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );
  
  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_iCWidth  = iWidth  >> 1;
  m_iCHeight = iHeight >> 1;
}

Void TComYuv::destroy()
{
  // memory free
  xFree( m_apiBufY ); m_apiBufY = NULL;
  xFree( m_apiBufU ); m_apiBufU = NULL;
  xFree( m_apiBufV ); m_apiBufV = NULL;
}

Void TComYuv::clear()
{
#if ETRI_CLEAR_FUNC_SIMD
    __m128i xmm_zero = _mm_setzero_si128();
    for (int i = 0; i < m_iWidth*m_iHeight; i += 8)
        _mm_storeu_si128((__m128i*)&m_apiBufY[i], xmm_zero);
    for (int i = 0; i < m_iCWidth*m_iCHeight; i += 8)
    {
        _mm_storeu_si128((__m128i*)&m_apiBufU[i], xmm_zero);
        _mm_storeu_si128((__m128i*)&m_apiBufV[i], xmm_zero);
    }
#else 
  ::memset( m_apiBufY, 0, ( m_iWidth  * m_iHeight  )*sizeof(Pel) );
  ::memset( m_apiBufU, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
  ::memset( m_apiBufV, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) ); 
#endif 
}

Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  copyToPicLuma  ( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
  copyToPicChroma( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
}

Void TComYuv::copyToPicLuma  ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iWidth >>uiPartDepth;
  iHeight = m_iHeight>>uiPartDepth;
  
  Pel* pSrc     = getLumaAddr(uiPartIdx, iWidth);
  Pel* pDst     = pcPicYuvDst->getLumaAddr ( iCuAddr, uiAbsZorderIdx );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcPicYuvDst->getStride();
  
#if ETRI_SIMD_COPY_TO_PIC_YUV
  if ( iWidth >= 8 )
  {
      for ( y = iHeight; y !=0; y-- )
      {
          for ( Int i = 0; i < iWidth; i += 8 ) 
              _mm_storeu_si128((__m128i*)&pDst[i],_mm_loadu_si128((__m128i*)&pSrc[i]));
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
  else
  {
      for (y = iHeight; y != 0; y--)
      {
          ::memcpy(pDst, pSrc, sizeof(Pel)*iWidth);
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
#else 
  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
#endif 
}

Void TComYuv::copyToPicChroma( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iCWidth >>uiPartDepth;
  iHeight = m_iCHeight>>uiPartDepth;
  
  Pel* pSrcU      = getCbAddr(uiPartIdx, iWidth);
  Pel* pSrcV      = getCrAddr(uiPartIdx, iWidth);
  Pel* pDstU      = pcPicYuvDst->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pDstV      = pcPicYuvDst->getCrAddr( iCuAddr, uiAbsZorderIdx );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcPicYuvDst->getCStride();
#if ETRI_SIMD_COPY_TO_PIC_YUV
  if ( iWidth >=8 )
  {
      for ( y = iHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < iWidth; i += 8 ) 
          {
              _mm_storeu_si128((__m128i*)&pDstU[i],_mm_loadu_si128((__m128i*)&pSrcU[i]));
              _mm_storeu_si128((__m128i*)&pDstV[i],_mm_loadu_si128((__m128i*)&pSrcV[i]));
          }
          pSrcU += iSrcStride;
          pSrcV += iSrcStride;
          pDstU += iDstStride;
          pDstV += iDstStride;
      }
  }
  else
  {
      for (y = iHeight; y != 0; y--)
      {
          ::memcpy(pDstU, pSrcU, sizeof(Pel)*(iWidth));
          ::memcpy(pDstV, pSrcV, sizeof(Pel)*(iWidth));
          pSrcU += iSrcStride;
          pSrcV += iSrcStride;
          pDstU += iDstStride;
          pDstV += iDstStride;
      }
  }
#else 
  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(iWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(iWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
#endif 
}

Void TComYuv::copyFromPicYuv   ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  copyFromPicLuma  ( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
  copyFromPicChroma( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
}

Void TComYuv::copyFromPicLuma  ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;
  
  Pel* pDst     = m_apiBufY;
  Pel* pSrc     = pcPicYuvSrc->getLumaAddr ( iCuAddr, uiAbsZorderIdx );
  
  UInt  iDstStride  = getStride();
  UInt  iSrcStride  = pcPicYuvSrc->getStride();
#if ETRI_SIMD_COPY_FROM_PIC_YUV
  if ( m_iWidth >= 8 )
  {
      for ( y = m_iHeight; y !=0; y-- )
      {
          for ( Int i = 0; i < m_iWidth; i += 8 ) 
              _mm_storeu_si128((__m128i*)&pDst[i],_mm_loadu_si128((__m128i*)&pSrc[i]));
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
  else
  {
      for (y = m_iHeight; y != 0; y--)
      {
          ::memcpy(pDst, pSrc, sizeof(Pel)*m_iWidth);
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
#else 
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
#endif 
}

Void TComYuv::copyFromPicChroma( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;
  
  Pel* pDstU      = m_apiBufU;
  Pel* pDstV      = m_apiBufV;
  Pel* pSrcU      = pcPicYuvSrc->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pSrcV      = pcPicYuvSrc->getCrAddr( iCuAddr, uiAbsZorderIdx );
  
  UInt  iDstStride = getCStride();
  UInt  iSrcStride = pcPicYuvSrc->getCStride();
#if ETRI_SIMD_COPY_FROM_PIC_YUV
  if ( m_iCWidth >=8 )
  {
      for ( y = m_iCHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < m_iCWidth; i += 8 ) 
          {
              _mm_storeu_si128((__m128i*)&pDstU[i],_mm_loadu_si128((__m128i*)&pSrcU[i]));
              _mm_storeu_si128((__m128i*)&pDstV[i],_mm_loadu_si128((__m128i*)&pSrcV[i]));
          }
          pSrcU += iSrcStride;
          pSrcV += iSrcStride;
          pDstU += iDstStride;
          pDstV += iDstStride;
      }
  }
  else
  {
      for (y = m_iCHeight; y != 0; y--)
      {
          ::memcpy(pDstU, pSrcU, sizeof(Pel)*(m_iCWidth));
          ::memcpy(pDstV, pSrcV, sizeof(Pel)*(m_iCWidth));
          pSrcU += iSrcStride;
          pSrcV += iSrcStride;
          pDstU += iDstStride;
          pDstV += iDstStride;
      }
  }
#else 
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
#endif 
}

Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  copyToPartLuma  ( pcYuvDst, uiDstPartIdx );
  copyToPartChroma( pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToPartLuma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
#if !KAIST_RC
  Int  y;
#endif

  Pel* pSrc     = m_apiBufY;
  Pel* pDst     = pcYuvDst->getLumaAddr( uiDstPartIdx );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();
#if ETRI_SIMD_COPY_PART_YUV
  if(m_iWidth >= 8)
  {
      for ( Int y = m_iHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < m_iWidth; i += 8 ) _mm_storeu_si128((__m128i*)&pDst[i],_mm_loadu_si128((__m128i*)&pSrc[i]));
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
  else
  {
      for (Int y = m_iHeight; y != 0; y--)
      {
          ::memcpy(pDst, pSrc, sizeof(Pel)*m_iWidth);
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
#else 
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
#endif 
}

Void TComYuv::copyToPartChroma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
#if !KAIST_RC
	Int  y;
#endif
  
  Pel* pSrcU      = m_apiBufU;
  Pel* pSrcV      = m_apiBufV;
  Pel* pDstU      = pcYuvDst->getCbAddr( uiDstPartIdx );
  Pel* pDstV      = pcYuvDst->getCrAddr( uiDstPartIdx );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();
#if ETRI_SIMD_COPY_PART_YUV
  if(m_iCWidth >= 8)
  {
      for (Int y = m_iCHeight; y != 0; y--)
      {
          for (Int i = 0; i < m_iCWidth; i += 8)
          {
              _mm_storeu_si128((__m128i*)&pDstU[i], _mm_loadu_si128((__m128i*)&pSrcU[i]));
              _mm_storeu_si128((__m128i*)&pDstV[i], _mm_loadu_si128((__m128i*)&pSrcV[i]));
          }
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
  else
  {
      for (Int y = m_iCHeight; y != 0; y--)
      {
          ::memcpy(pDstU, pSrcU, sizeof(Pel)*m_iCWidth);
          ::memcpy(pDstV, pSrcV, sizeof(Pel)*m_iCWidth);
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
#else 
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
#endif 
}

Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  copyPartToLuma  ( pcYuvDst, uiSrcPartIdx );
  copyPartToChroma( pcYuvDst, uiSrcPartIdx );
}

Void TComYuv::copyPartToLuma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
#if !KAIST_RC
	Int  y;
#endif
  
  Pel* pSrc     = getLumaAddr(uiSrcPartIdx);
  Pel* pDst     = pcYuvDst->getLumaAddr( 0 );
  
  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();
  
  UInt uiHeight = pcYuvDst->getHeight();
  UInt uiWidth = pcYuvDst->getWidth();
#if ETRI_SIMD_COPY_PART_YUV
  if(uiWidth >= 8)
  {
      for ( Int y = uiHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < uiWidth; i += 8 ) _mm_storeu_si128((__m128i*)&pDst[i],_mm_loadu_si128((__m128i*)&pSrc[i]));
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
  else
  {
      for (Int y = uiHeight; y != 0; y--)
      {
          ::memcpy(pDst, pSrc, sizeof(Pel)*uiWidth);
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
#else 
  for ( y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
#endif 
}

Void TComYuv::copyPartToChroma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
#if !KAIST_RC
	Int  y;
#endif
  
  Pel* pSrcU      = getCbAddr( uiSrcPartIdx );
  Pel* pSrcV      = getCrAddr( uiSrcPartIdx );
  Pel* pDstU      = pcYuvDst->getCbAddr( 0 );
  Pel* pDstV      = pcYuvDst->getCrAddr( 0 );
  
  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();
  
  UInt uiCHeight = pcYuvDst->getCHeight();
  UInt uiCWidth = pcYuvDst->getCWidth();
#if ETRI_SIMD_COPY_PART_YUV
  if(uiCWidth >= 8)
  {
      for ( Int y = uiCHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < uiCWidth; i += 8 ) 
          {
              _mm_storeu_si128((__m128i*)&pDstU[i],_mm_loadu_si128((__m128i*)&pSrcU[i]));
              _mm_storeu_si128((__m128i*)&pDstV[i],_mm_loadu_si128((__m128i*)&pSrcV[i]));
          }
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
  else
  {
      for (Int y = uiCHeight; y != 0; y--)
      {
          ::memcpy(pDstU, pSrcU, sizeof(Pel)*uiCWidth);
          ::memcpy(pDstV, pSrcV, sizeof(Pel)*uiCWidth);
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
#else 
  for ( y = uiCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(uiCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(uiCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
#endif 
}

Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  copyPartToPartLuma   (pcYuvDst, uiPartIdx, iWidth, iHeight );
  copyPartToPartChroma (pcYuvDst, uiPartIdx, iWidth>>1, iHeight>>1 );
}

Void TComYuv::copyPartToPartLuma  ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel* pSrc =           getLumaAddr(uiPartIdx);
  Pel* pDst = pcYuvDst->getLumaAddr(uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }
  
  UInt  iSrcStride = getStride();
  UInt  iDstStride = pcYuvDst->getStride();
#if ETRI_SIMD_COPY_PART_YUV
  if(iWidth >= 8)
  {
      for (Int y = iHeight; y != 0; y--)
      {
          for (Int i = 0; i < iWidth; i += 8) _mm_storeu_si128((__m128i*)&pDst[i], _mm_loadu_si128((__m128i*)&pSrc[i]));
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
  else
  {
      for (Int y = iHeight; y != 0; y--)
      {
          ::memcpy(pDst, pSrc, sizeof(Pel)*iWidth);
          pDst += iDstStride;
          pSrc += iSrcStride;
      }
  }
#else 
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidth * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
#endif 
}

Void TComYuv::copyPartToPartChroma( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel*  pSrcU =           getCbAddr(uiPartIdx);
  Pel*  pSrcV =           getCrAddr(uiPartIdx);
  Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
  Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
  
  if( pSrcU == pDstU && pSrcV == pDstV)
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }
  
  UInt   iSrcStride = getCStride();
  UInt   iDstStride = pcYuvDst->getCStride();
#if ETRI_SIMD_COPY_PART_YUV
  if(iWidth >= 8)
  {
      for ( Int y = iHeight; y != 0; y-- )
      {
          for ( Int i = 0; i < iWidth; i += 8 ) 
          {
              _mm_storeu_si128((__m128i*)&pDstU[i],_mm_loadu_si128((__m128i*)&pSrcU[i]));
              _mm_storeu_si128((__m128i*)&pDstV[i],_mm_loadu_si128((__m128i*)&pSrcV[i]));
          }
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
  else
  {
      for (Int y = iHeight; y != 0; y--)
      {
          ::memcpy(pDstU, pSrcU, sizeof(Pel)*iWidth);
          ::memcpy(pDstV, pSrcV, sizeof(Pel)*iWidth);
          pDstU += iDstStride;
          pSrcU += iSrcStride;
          pDstV += iDstStride;
          pSrcV += iSrcStride;
      }
  }
#else 
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
    ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
#endif 
}

Void TComYuv::copyPartToPartChroma( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight, UInt chromaId)
{
  if(chromaId == 0)
  {
    Pel*  pSrcU =           getCbAddr(uiPartIdx);
    Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
    if( pSrcU == pDstU)
    {
      return ;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    {
      ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
      pSrcU += iSrcStride;
      pDstU += iDstStride;
    }
  }
  else if (chromaId == 1)
  {
    Pel*  pSrcV =           getCrAddr(uiPartIdx);
    Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
    if( pSrcV == pDstV)
    {
      return;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    { 
      ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
      pSrcV += iSrcStride;
      pDstV += iDstStride;
    }
  }
  else
  {
    Pel*  pSrcU =           getCbAddr(uiPartIdx);
    Pel*  pSrcV =           getCrAddr(uiPartIdx);
    Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
    Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);
    
    if( pSrcU == pDstU && pSrcV == pDstV)
    {
      //th not a good idea
      //th best would be to fix the caller 
      return ;
    }
    UInt   iSrcStride = getCStride();
    UInt   iDstStride = pcYuvDst->getCStride();
    for ( UInt y = iHeight; y != 0; y-- )
    {
      ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
      ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
      pSrcU += iSrcStride;
      pSrcV += iSrcStride;
      pDstU += iDstStride;
      pDstV += iDstStride;
    }
  }
}

#if ETRI_REDUNDANCY_OPTIMIZATION
Void TComYuv::ETRI_addClip(TComYuv* pcYuvSrc0, UInt uiTrUnitIdx, UInt uiPartSize)
{
	ETRI_addClipLuma(pcYuvSrc0, uiTrUnitIdx, uiPartSize);
	ETRI_addClipChroma(pcYuvSrc0, uiTrUnitIdx, uiPartSize >> 1);
}


Void TComYuv::ETRI_addClipLuma(TComYuv* pcYuvSrc0, UInt uiTrUnitIdx, UInt uiPartSize)
{
	Int x, y;
	Pel* pSrc0 = pcYuvSrc0->getLumaAddr(uiTrUnitIdx, uiPartSize);
	Pel* pDst = getLumaAddr(uiTrUnitIdx, uiPartSize);

	UInt iSrc0Stride = pcYuvSrc0->getStride();
	UInt iDstStride = getStride();

	Int     RemainX = uiPartSize & 0x07;
	__m128i	xmm0;

	for (y = uiPartSize - 1; y >= 0; y--)
	{
		for (x = 0; x < uiPartSize - RemainX; x += 8)
		{
			xmm0 = _mm_load_si128((__m128i const *)&pSrc0[x]);
			_mm_store_si128((__m128i *)&pDst[x], xmm0);
		}

		for (x = uiPartSize - RemainX; x < uiPartSize; x++)
		{
			pDst[x] = ClipY(pSrc0[x]);
		}

		pSrc0 += iSrc0Stride;
		pDst += iDstStride;
	}
}

Void TComYuv::ETRI_addClipChroma(TComYuv* pcYuvSrc0, UInt uiTrUnitIdx, UInt uiPartSize)
{
	Int x, y;
	Pel* pSrcU0 = pcYuvSrc0->getCbAddr(uiTrUnitIdx, uiPartSize);
	Pel* pSrcV0 = pcYuvSrc0->getCrAddr(uiTrUnitIdx, uiPartSize);

	Pel* pDstU = getCbAddr(uiTrUnitIdx, uiPartSize);
	Pel* pDstV = getCrAddr(uiTrUnitIdx, uiPartSize);

	UInt  iSrc0Stride = pcYuvSrc0->getCStride();
	UInt  iDstStride = getCStride();

	Int  	   RemainX = uiPartSize & 0x07;
	__m128i  xmm0, xmm1;

	for (y = uiPartSize - 1; y >= 0; y--)
	{
		for (x = 0; x < uiPartSize - RemainX; x += 8)
		{
			xmm0 = _mm_load_si128((__m128i const *)&pSrcU0[x]);
			xmm1 = _mm_load_si128((__m128i const *)&pSrcV0[x]);

			_mm_store_si128((__m128i *)&pDstU[x], xmm0);
			_mm_store_si128((__m128i *)&pDstV[x], xmm1);
		}

		for (x = uiPartSize - RemainX; x < uiPartSize; x++)
		{
			pDstU[x] = ClipC(pSrcU0[x]);
			pDstV[x] = ClipC(pSrcV0[x]);
		}

		pSrcU0 += iSrc0Stride;
		pSrcV0 += iSrc0Stride;

		pDstU += iDstStride;
		pDstV += iDstStride;
	}
}
#endif // #if ETRI_REDUNDANCY_OPTIMIZATION

Void TComYuv::addClip( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  addClipLuma   ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize     );
  addClipChroma ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize>>1  );
}

Void TComYuv::addClipLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
  
  UInt iSrc0Stride = pcYuvSrc0->getStride();
  UInt iSrc1Stride = pcYuvSrc1->getStride();
  UInt iDstStride  = getStride();

#if ETRI_SIMD_YUV
  Int     RemainX = uiPartSize & 0x07;
  __m128i  xmm0, xmm1, xmmz, xmax;
  short   Tmax = (1 << g_bitDepthY) - 1;

  xmax = _mm_set1_epi16(Tmax);
  xmmz = _mm_setzero_si128();

  for (y = uiPartSize - 1; y >= 0; y--)
  {
	  for (x = 0; x < uiPartSize - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrc0[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrc1[x]);
		  xmm0 = _mm_add_epi16(xmm0, xmm1);
		  xmm0 = _mm_min_epi16(xmm0, xmax);
		  xmm0 = _mm_max_epi16(xmm0, xmmz);

		  _mm_store_si128((__m128i *)&pDst[x], xmm0);
	  }

	  for (x = uiPartSize - RemainX; x < uiPartSize; x++)
	  {
		  pDst[x] = ClipY(pSrc0[x] + pSrc1[x]);
	  }

	  pSrc0 += iSrc0Stride;
	  pSrc1 += iSrc1Stride;
	  pDst += iDstStride;
  }
#else
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDst[x] = ClipY( pSrc0[x] + pSrc1[x] );
    }
    pSrc0 += iSrc0Stride;
    pSrc1 += iSrc1Stride;
    pDst  += iDstStride;
  }
#endif
}

Void TComYuv::addClipChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV = getCrAddr( uiTrUnitIdx, uiPartSize );
  
  UInt  iSrc0Stride = pcYuvSrc0->getCStride();
  UInt  iSrc1Stride = pcYuvSrc1->getCStride();
  UInt  iDstStride  = getCStride();

#if ETRI_SIMD_YUV
  Int      RemainX = uiPartSize & 0x07;
  __m128i xmm0, xmm1, xmm2, xmm3, xmmz, xmax;
  short    Tmax = (1 << g_bitDepthC) - 1;

  xmax = _mm_set1_epi16(Tmax);
  xmmz = _mm_setzero_si128();

  for (y = uiPartSize - 1; y >= 0; y--)
  {
	  for (x = 0; x < uiPartSize - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrcU0[x]);
		  xmm2 = _mm_load_si128((__m128i const *)&pSrcU1[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrcV0[x]);
		  xmm3 = _mm_load_si128((__m128i const *)&pSrcV1[x]);
		  xmm0 = _mm_add_epi16(xmm0, xmm2);
		  xmm1 = _mm_add_epi16(xmm1, xmm3);
		  xmm0 = _mm_min_epi16(xmm0, xmax);
		  xmm0 = _mm_max_epi16(xmm0, xmmz);
		  xmm1 = _mm_min_epi16(xmm1, xmax);
		  xmm1 = _mm_max_epi16(xmm1, xmmz);

		  _mm_store_si128((__m128i *)&pDstU[x], xmm0);
		  _mm_store_si128((__m128i *)&pDstV[x], xmm1);
	  }

	  for (x = uiPartSize - RemainX; x < uiPartSize; x++)
	  {
		  pDstU[x] = ClipC(pSrcU0[x] + pSrcU1[x]);
		  pDstV[x] = ClipC(pSrcV0[x] + pSrcV1[x]);
	  }

	  pSrcU0 += iSrc0Stride;
	  pSrcU1 += iSrc1Stride;
	  pSrcV0 += iSrc0Stride;
	  pSrcV1 += iSrc1Stride;
	  pDstU += iDstStride;
	  pDstV += iDstStride;
  }
#else
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = ClipC( pSrcU0[x] + pSrcU1[x] );
      pDstV[x] = ClipC( pSrcV0[x] + pSrcV1[x] );
    }
    
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
#endif
}

Void TComYuv::subtract( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  subtractLuma  ( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize    );
  subtractChroma( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize>>1 );
}

Void TComYuv::subtractLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );
  
  Int  iSrc0Stride = pcYuvSrc0->getStride();
  Int  iSrc1Stride = pcYuvSrc1->getStride();
  Int  iDstStride  = getStride();

#if ETRI_SIMD_YUV
  Int  RemainX = uiPartSize & 0x07;
  __m128i   xmm0, xmm1;

  for (y = uiPartSize - 1; y >= 0; y--)
  {
	  for (x = 0; x < uiPartSize - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrc0[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrc1[x]);
		  xmm0 = _mm_sub_epi16(xmm0, xmm1);

		  _mm_store_si128((__m128i *)&pDst[x], xmm0);
	  }

	  for (x = uiPartSize - RemainX; x < uiPartSize; x++)
	  {
		  pDst[x] = pSrc0[x] - pSrc1[x];
	  }

	  pSrc0 += iSrc0Stride;
	  pSrc1 += iSrc1Stride;
	  pDst += iDstStride;
  }
#else
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDst[x] = pSrc0[x] - pSrc1[x];
    }
    pSrc0 += iSrc0Stride;
    pSrc1 += iSrc1Stride;
    pDst  += iDstStride;
  }
#endif
}

Void TComYuv::subtractChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;
  
  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU  = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV  = getCrAddr( uiTrUnitIdx, uiPartSize );
  
  Int  iSrc0Stride = pcYuvSrc0->getCStride();
  Int  iSrc1Stride = pcYuvSrc1->getCStride();
  Int  iDstStride  = getCStride();

#if ETRI_SIMD_YUV
  Int     RemainX = uiPartSize & 0x07;
  __m128i xmm0, xmm1, xmm2, xmm3;

  for (y = uiPartSize - 1; y >= 0; y--)
  {
	  for (x = 0; x < uiPartSize - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrcU0[x]);
		  xmm2 = _mm_load_si128((__m128i const *)&pSrcU1[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrcV0[x]);
		  xmm3 = _mm_load_si128((__m128i const *)&pSrcV1[x]);

		  xmm0 = _mm_sub_epi16(xmm0, xmm2);
		  xmm1 = _mm_sub_epi16(xmm1, xmm3);

		  _mm_store_si128((__m128i *)&pDstU[x], xmm0);
		  _mm_store_si128((__m128i *)&pDstV[x], xmm1);
	  }

	  for (x = uiPartSize - RemainX; x < uiPartSize; x++)
	  {
		  pDstU[x] = pSrcU0[x] - pSrcU1[x];
		  pDstV[x] = pSrcV0[x] - pSrcV1[x];
	  }

	  pSrcU0 += iSrc0Stride;
	  pSrcU1 += iSrc1Stride;
	  pSrcV0 += iSrc0Stride;
	  pSrcV1 += iSrc1Stride;
	  pDstU += iDstStride;
	  pDstV += iDstStride;
  }
#else
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = pSrcU0[x] - pSrcU1[x];
      pDstV[x] = pSrcV0[x] - pSrcV1[x];
    }
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
#endif
}

Void TComYuv::addAvg( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt iPartUnitIdx, UInt iWidth, UInt iHeight )
{
  Int x, y;
  
  Pel* pSrcY0  = pcYuvSrc0->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU0  = pcYuvSrc0->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV0  = pcYuvSrc0->getCrAddr  ( iPartUnitIdx );
  
  Pel* pSrcY1  = pcYuvSrc1->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU1  = pcYuvSrc1->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV1  = pcYuvSrc1->getCrAddr  ( iPartUnitIdx );
  
  Pel* pDstY   = getLumaAddr( iPartUnitIdx );
  Pel* pDstU   = getCbAddr  ( iPartUnitIdx );
  Pel* pDstV   = getCrAddr  ( iPartUnitIdx );
  
  UInt  iSrc0Stride = pcYuvSrc0->getStride();
  UInt  iSrc1Stride = pcYuvSrc1->getStride();
  UInt  iDstStride  = getStride();
  Int shiftNum = IF_INTERNAL_PREC + 1 - g_bitDepthY;
  Int offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

#if ETRI_SIMD_YUV
  Int        RemainX = iWidth & 0x07;
  __m128i  xmm0, xmm1, xmm2, xmm3, xmm4, xmmz, xmax;
  short     Tmax = (1 << g_bitDepthY) - 1;

  xmax = _mm_set1_epi16(Tmax);
  xmmz = _mm_setzero_si128();
  xmm4 = _mm_set1_epi16((short)offset);

  for (y = 0; y < iHeight; y++)
  {
	  for (x = 0; x < iWidth - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrcY0[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrcY1[x]);
		  xmm0 = _mm_adds_epi16(xmm0, xmm1);
		  xmm0 = _mm_adds_epi16(xmm0, xmm4);
		  xmm0 = _mm_srai_epi16(xmm0, (short)shiftNum);
		  xmm0 = _mm_min_epi16(xmm0, xmax);
		  xmm0 = _mm_max_epi16(xmm0, xmmz);

		  _mm_store_si128((__m128i *)&pDstY[x], xmm0);
	  }

	  for (x = iWidth - RemainX; x <iWidth; x += 4)
	  {
		  pDstY[x + 0] = ClipY((pSrcY0[x + 0] + pSrcY1[x + 0] + offset) >> shiftNum);
		  pDstY[x + 1] = ClipY((pSrcY0[x + 1] + pSrcY1[x + 1] + offset) >> shiftNum);
		  pDstY[x + 2] = ClipY((pSrcY0[x + 2] + pSrcY1[x + 2] + offset) >> shiftNum);
		  pDstY[x + 3] = ClipY((pSrcY0[x + 3] + pSrcY1[x + 3] + offset) >> shiftNum);
	  }

	  pSrcY0 += iSrc0Stride;
	  pSrcY1 += iSrc1Stride;
	  pDstY += iDstStride;
  }
#else
  for ( y = 0; y < iHeight; y++ )
  {
    for ( x = 0; x < iWidth; x += 4 )
    {
      pDstY[ x + 0 ] = ClipY( ( pSrcY0[ x + 0 ] + pSrcY1[ x + 0 ] + offset ) >> shiftNum );
      pDstY[ x + 1 ] = ClipY( ( pSrcY0[ x + 1 ] + pSrcY1[ x + 1 ] + offset ) >> shiftNum );
      pDstY[ x + 2 ] = ClipY( ( pSrcY0[ x + 2 ] + pSrcY1[ x + 2 ] + offset ) >> shiftNum );
      pDstY[ x + 3 ] = ClipY( ( pSrcY0[ x + 3 ] + pSrcY1[ x + 3 ] + offset ) >> shiftNum );
    }
    pSrcY0 += iSrc0Stride;
    pSrcY1 += iSrc1Stride;
    pDstY  += iDstStride;
  }
#endif 

  shiftNum = IF_INTERNAL_PREC + 1 - g_bitDepthC;
  offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

  iSrc0Stride = pcYuvSrc0->getCStride();
  iSrc1Stride = pcYuvSrc1->getCStride();
  iDstStride  = getCStride();
  
  iWidth  >>=1;
  iHeight >>=1;

#if ETRI_SIMD_YUV
  RemainX = iWidth & 0x07;
  xmax = _mm_set1_epi16((1 << g_bitDepthC) - 1);
  xmm4 = _mm_set1_epi16((short)offset);

  for (y = 0; y < iHeight; y++)
  {
	  for (x = 0; x < iWidth - RemainX; x += 8)
	  {
		  xmm0 = _mm_load_si128((__m128i const *)&pSrcU0[x]);
		  xmm2 = _mm_load_si128((__m128i const *)&pSrcU1[x]);
		  xmm1 = _mm_load_si128((__m128i const *)&pSrcV0[x]);
		  xmm3 = _mm_load_si128((__m128i const *)&pSrcV1[x]);

		  xmm0 = _mm_adds_epi16(xmm0, xmm2);
		  xmm0 = _mm_adds_epi16(xmm0, xmm4);
		  xmm0 = _mm_srai_epi16(xmm0, (short)shiftNum);

		  xmm1 = _mm_add_epi16(xmm1, xmm3);
		  xmm1 = _mm_add_epi16(xmm1, xmm4);
		  xmm1 = _mm_srai_epi16(xmm1, (short)shiftNum);

		  xmm0 = _mm_min_epi16(xmm0, xmax);
		  xmm0 = _mm_max_epi16(xmm0, xmmz);
		  xmm1 = _mm_min_epi16(xmm1, xmax);
		  xmm1 = _mm_max_epi16(xmm1, xmmz);

		  _mm_store_si128((__m128i *)&pDstU[x], xmm0);
		  _mm_store_si128((__m128i *)&pDstV[x], xmm1);
	  }

	  for (x = iWidth - RemainX; x <iWidth;)
	  {
		  pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
		  pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x++;
		  pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
		  pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x++;
	  }

	  pSrcU0 += iSrc0Stride;
	  pSrcU1 += iSrc1Stride;
	  pSrcV0 += iSrc0Stride;
	  pSrcV1 += iSrc1Stride;
	  pDstU += iDstStride;
	  pDstV += iDstStride;
  }
#else  
  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: chroma min width is 2
      pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
      pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x--;
      pDstU[x] = ClipC((pSrcU0[x] + pSrcU1[x] + offset) >> shiftNum);
      pDstV[x] = ClipC((pSrcV0[x] + pSrcV1[x] + offset) >> shiftNum); x--;
    }
    
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
#endif
}

#if ETRI_SIMD_REMOVE_HIGH_FREQ
Void TComYuv::removeHighFreq(TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight)		// by org
{
	Int x, y;

	Pel* pSrc = pcYuvSrc->getLumaAddr(uiPartIdx);
	Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
	Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

	Pel* pDst = getLumaAddr(uiPartIdx);
	Pel* pDstU = getCbAddr(uiPartIdx);
	Pel* pDstV = getCrAddr(uiPartIdx);

	Int  iSrcStride = pcYuvSrc->getStride();
	Int  iDstStride = getStride();
	__m128i xmm_2 = _mm_set1_epi16(2);
	for (y = 0; y < uiHeight; y++)
	{
		for (x = 0; x < uiWidht; x += 8)
		{
			//	pDst[x ] = 2 * pDst[x] - pSrc[x];
			_mm_storeu_si128((__m128i*)&pDst[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDst[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrc[x])));
		}
		pSrc += iSrcStride;
		pDst += iDstStride;
	}

	iSrcStride = pcYuvSrc->getCStride();
	iDstStride = getCStride();

	uiHeight >>= 1;
	uiWidht >>= 1;

	for (y = 0; y < uiHeight; y++)
	{
		for (x = 0; x < uiWidht; x += 8)
		{
			//pDstU[x ] = 2 * pDstU[x] - pSrcU[x];
			//pDstV[x ] = 2 * pDstV[x] - pSrcV[x];
			_mm_storeu_si128((__m128i*)&pDstU[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDstU[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrcU[x])));
			_mm_storeu_si128((__m128i*)&pDstV[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDstV[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrcV[x])));
		}
		pSrcU += iSrcStride;
		pSrcV += iSrcStride;
		pDstU += iDstStride;
		pDstV += iDstStride;
	}
}
#else
Void TComYuv::removeHighFreq(TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight)		// by org
{
	Int x, y;

	Pel* pSrc = pcYuvSrc->getLumaAddr(uiPartIdx);
	Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
	Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

	Pel* pDst = getLumaAddr(uiPartIdx);
	Pel* pDstU = getCbAddr(uiPartIdx);
	Pel* pDstV = getCrAddr(uiPartIdx);

	Int  iSrcStride = pcYuvSrc->getStride();
	Int  iDstStride = getStride();

	for (y = uiHeight - 1; y >= 0; y--)
	{
		for (x = uiWidht - 1; x >= 0; x--)
		{
#if DISABLING_CLIP_FOR_BIPREDME
			pDst[x] = 2 * pDst[x] - pSrc[x];
#else
			pDst[x] = ClipY(2 * pDst[x] - pSrc[x]);
#endif
		}
		pSrc += iSrcStride;
		pDst += iDstStride;
	}

	iSrcStride = pcYuvSrc->getCStride();
	iDstStride = getCStride();

	uiHeight >>= 1;
	uiWidht >>= 1;

	for (y = uiHeight - 1; y >= 0; y--)
	{
		for (x = uiWidht - 1; x >= 0; x--)
		{
#if DISABLING_CLIP_FOR_BIPREDME
			pDstU[x] = 2 * pDstU[x] - pSrcU[x];
			pDstV[x] = 2 * pDstV[x] - pSrcV[x];
#else
			pDstU[x] = ClipC(2 * pDstU[x] - pSrcU[x]);
			pDstV[x] = ClipC(2 * pDstV[x] - pSrcV[x]);
#endif
		}
		pSrcU += iSrcStride;
		pSrcV += iSrcStride;
		pDstU += iDstStride;
		pDstV += iDstStride;
	}
}
#endif

#if ETRI_MODIFICATION_V02
// ================================================================================================================
// ETRI Debug Functions 
// ================================================================================================================
Void TComYuv::ETRI_PrintLUMA  (FILE *_style)
{
	Int  x, y;

	Pel* 	pSrc     		= m_apiBufY;
	UInt 	iSrcStride  	= getStride();

	for ( y = 0; y < m_iHeight; y++ )
	{
		for( x = 0; x < m_iWidth; x++)
			fprintf(_style, "%4d ", *(pSrc + x));
		fprintf(_style, "\n");
		pSrc += iSrcStride;
	}
}

Void TComYuv::ETRI_PrintChroma 	(FILE *_style, UInt uiChromaID)
{
	Int  x, y;

	Pel* 	pSrc = (uiChromaID == 0)? m_apiBufU : m_apiBufV;
	UInt 	iSrcStride = getCStride();

	for ( y = 0; y < m_iCHeight; y++ )
	{
		for( x = 0; x < m_iCWidth; x++)
			fprintf(_style, "%4d ", *(pSrc + x));
		fprintf(_style, "\n");
		pSrc += iSrcStride;
	}
}

bool TComYuv::ETRI_CompareLuma 	( TComYuv *Dst)
{
	Int  x, y;

	Pel* 	pSrc = m_apiBufY;
	Pel* 	pDst = Dst->getLumaAddr();

	UInt 	iSrcStride = getStride();
	UInt 	iDstStride = Dst->getStride();

	for ( y = 0; y < m_iHeight; y++ )
	{
		for( x = 0; x < m_iWidth; x++)
		{
			if (pSrc[x] != pDst[x])
			{
				EDPRINTF(stderr, "Error Ocurr x: %d  y: %d \n", x, y);
				return false;
			}
		}
		pSrc += iSrcStride;
		pDst += iDstStride;
	}

	return true;

}

bool TComYuv::ETRI_CompareChroma 	( TComYuv *Dst, UInt uiChromaID)
{
	Int  x, y;

	Pel* 	pSrc = (uiChromaID == 0)? m_apiBufU : m_apiBufV;
	Pel* 	pDst = (uiChromaID == 0)? Dst->getCbAddr(): Dst->getCrAddr();

	UInt 	iSrcStride = getCStride();
	UInt 	iDstStride = Dst->getCStride();

	for ( y = 0; y < m_iCHeight; y++ )
	{
		for( x = 0; x < m_iCWidth; x++)
		{
			if (pSrc[x] != pDst[x])
			{
				EDPRINTF(stderr, "Error Ocurr x: %d  y: %d \n", x, y);
				return false;
			}
		}
		pSrc += iSrcStride;
		pDst += iDstStride;
	}

	return true;

}
#endif	//ETRI_MODIFICATION_V02


//! \}
/*Void TComYuv::removeHighFreq(TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight)		//	by serdong
{
	Int x, y;


	EDPRINTF(stderr, "============ Debug Info : %d ===========\n", em_dbgInfo);

	short R[8] = { 0 };
	short RV[8] = { 0 };
	short RSrc[8] = { 0 };
	short RDst[8] = { 0 };

	short SrcUR[8] = { 0 };
	short SrcVR[8] = { 0 };
//	short DR[8] = { 0 };
//	short DRV[8] = { 0 };
//	short DSrcUR[8] = { 0 };
//	short DSrcVR[8] = { 0 };

	__m128i xmmR, xmmRV, xmmSrcU, xmmSrcV, xmmSrc, xmmDst;
//	FILE *fp = fopen("output.txt", "w");
	FILE *fp = fopen("output_SIMD.txt", "w");

	Pel* pSrc = pcYuvSrc->getLumaAddr(uiPartIdx);
	Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
	Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

	Pel* pDst = getLumaAddr(uiPartIdx);
	Pel* pDstU = getCbAddr(uiPartIdx);
	Pel* pDstV = getCrAddr(uiPartIdx);

	Int  iSrcStride = pcYuvSrc->getStride();		//org
	Int  iDstStride = getStride();					//org

#if SIMD_DEBUG
	Pel* pDSrc = pcYuvSrc->getLumaAddr(uiPartIdx);
	Pel* pDSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
	Pel* pDSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

	Pel* pDDst = getLumaAddr(uiPartIdx);
	Pel* pDDstU = getCbAddr(uiPartIdx);
	Pel* pDDstV = getCrAddr(uiPartIdx);

	Int  iDSrcStride = pcYuvSrc->getStride();
	Int  iDDstStride = getStride();
//	UInt _nSize = uiWidht * uiHeight;
//	UInt _nCSize = (uiWidht * uiHeight)>>2;
	
//	pDDst = (Pel *)calloc(_nSize, sizeof(Pel));
//	pDDstU= (Pel *)calloc(_nCSize, sizeof(Pel));
//	pDDstV= (Pel *)calloc(_nCSize, sizeof(Pel));
#endif

	__m128i xmm_2 = _mm_set1_epi16(2);
	for (y = 0; y < uiHeight; y++)
	{
		fprintf(fp, " uiHeight [%d]\n", y);
		for (x = 0; x < uiHeight; x += 8)
		{
			//	pDst[x ] = 2 * pDst[x] - pSrc[x];
			_mm_storeu_si128((__m128i*)&pDst[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDst[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrc[x])));

			xmmSrc = _mm_loadu_si128((__m128i*)&pSrc[x]);
			xmmDst = _mm_loadu_si128((__m128i*)&pDst[x]);

			_mm_storeu_si128((__m128i*)RSrc, xmmSrc);
			_mm_storeu_si128((__m128i*)RDst, xmmDst);
//			fprintf(fp, "pSrc[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, RSrc[0], RSrc[1], RSrc[2], RSrc[3], RSrc[4], RSrc[5], RSrc[6], RSrc[7]);
//			fprintf(fp, "pDst[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, RDst[0], RDst[1], RDst[2], RDst[3], RDst[4], RDst[5], RDst[6], RDst[7]);
		}
		pSrc += iSrcStride;
		pDst += iDstStride;
//		fprintf(fp, "Sum of pSrc, pDst : %d, %d\n", pSrc, pDst);
	}
//	printf("SIMD_HIGH : %d	%d\n", pSrc, pDst);


#if SIMD_DEBUG
	for (y = uiHeight - 1; y >= 0; y--)
	{
		for (x = uiWidht - 1; x >= 0; x--)
		{
#if DISABLING_CLIP_FOR_BIPREDME
			pDDst[x] = 2 * pDDst[x] - pDSrc[x];
#else
			pDDst[x] = ClipY(2 * pDDst[x] - pDSrc[x]);
#endif
		}
		pDSrc += iDSrcStride;
		pDDst += iDDstStride;
	}
#endif
//	printf("NO_SIMD : %d	%d\n", pDSrc, pDDst);

	iSrcStride = pcYuvSrc->getCStride();
	iDstStride = getCStride();

#if SIMD_DEBUG
	//	iDSrcStride = pcYuvSrc->getCStride();
	//	iDDstStride = getCStride();
	#endif

  uiHeight >>= 1;
  uiWidht >>= 1;

  for (y = 0; y < uiHeight; y++)
  {
	  fprintf(fp, "Result y[%d]\n", y);
//	  fprintf(fp2, "Result y[%d]\n", y);
	  for (x = 0; x < uiHeight; x += 8)
	  {
		  xmmR = _mm_loadu_si128((__m128i*)&pDstU[x]);
		  xmmRV = _mm_loadu_si128((__m128i*)&pDstV[x]);

		  _mm_storeu_si128((__m128i*)R, xmmR);
		  _mm_storeu_si128((__m128i*)RV, xmmRV);
		  fprintf(fp, "Origin SIMD pDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7]);
		  fprintf(fp, "Origin SIMD pDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, RV[0], RV[1], RV[2], RV[3], RV[4], RV[5], RV[6], RV[7]);

					DR[0] = pDDstU[x];
					DRV[0] = pDDstV[x];

					DR[1] = pDDstU[x + 1];
					DRV[1] = pDDstV[x + 1];

					DR[2] = pDDstU[x + 2];
					DRV[2] = pDDstV[x + 2];

					DR[3] = pDDstU[x + 3];
					DRV[3] = pDDstV[x + 3];

					DR[4] = pDDstU[x + 4];
					DRV[4] = pDDstV[x + 4];

					DR[5] = pDDstU[x + 5];
					DRV[5] = pDDstV[x + 5];

					DR[6] = pDDstU[x + 6];
					DRV[6] = pDDstV[x + 6];

					DR[7] = pDDstU[x + 7];
					DRV[7] = pDDstV[x + 7];

					//		  printf("Input     pDDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DR[0], DR[1], DR[2], DR[3], DR[4], DR[5], DR[6], DR[7]);
					//		  printf("Input     pDDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRV[0], DRV[1], DRV[2], DRV[3], DRV[4], DRV[5], DRV[6], DRV[7]);
#if 0


		  _mm_storeu_si128((__m128i*)&pDstU[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDstU[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrcU[x])));
		  _mm_storeu_si128((__m128i*)&pDstV[x], _mm_sub_epi16(_mm_mullo_epi16(_mm_loadu_si128((__m128i*)&pDstV[x]), xmm_2), _mm_loadu_si128((__m128i*)&pSrcV[x])));
		  xmmR = _mm_loadu_si128((__m128i*)&pDstU[x]);
		  xmmRV = _mm_loadu_si128((__m128i*)&pDstV[x]);
		  xmmSrcU = _mm_loadu_si128((__m128i*)&pSrcU[x]);
		  xmmSrcV = _mm_loadu_si128((__m128i*)&pSrcV[x]);
#else
		  Short e_sProbe[8];
		  
		  __m128i xmm0, xmm1, xmm3, xmm4, xmm5;

		  xmm0 = _mm_loadu_si128((__m128i *)&pSrcU[x]);
		  _mm_storeu_si128((__m128i*)e_sProbe, xmm0);
		  fprintf(fp, "pSrcU : %d, %d, %d, %d, %d, %d, %d, %d\n", e_sProbe[0], e_sProbe[1], e_sProbe[2], e_sProbe[3], e_sProbe[4], e_sProbe[5], e_sProbe[6], e_sProbe[7]);
		  xmm1 = _mm_loadu_si128((__m128i *)&pDstU[x]);
		  _mm_storeu_si128((__m128i*)e_sProbe, xmm1);
		  fprintf(fp, "pDstU : %d, %d, %d, %d, %d, %d, %d, %d\n", e_sProbe[0], e_sProbe[1], e_sProbe[2], e_sProbe[3], e_sProbe[4], e_sProbe[5], e_sProbe[6], e_sProbe[7]);
		  xmm3 = _mm_mullo_epi16(xmm1, xmm_2);
		  _mm_storeu_si128((__m128i*)e_sProbe, xmm3);
		  fprintf(fp, "mullo : %d, %d, %d, %d, %d, %d, %d, %d\n", e_sProbe[0], e_sProbe[1], e_sProbe[2], e_sProbe[3], e_sProbe[4], e_sProbe[5], e_sProbe[6], e_sProbe[7]);
		  xmm4 = _mm_sub_epi16(xmm3, xmm0);
		  _mm_storeu_si128((__m128i*)e_sProbe, xmm4);
		  fprintf(fp, "sub : %d, %d, %d, %d, %d, %d, %d, %d\n", e_sProbe[0], e_sProbe[1], e_sProbe[2], e_sProbe[3], e_sProbe[4], e_sProbe[5], e_sProbe[6], e_sProbe[7]);
		  _mm_storeu_si128((__m128i*)&pDstU[x], xmm4);
		  xmm5 = _mm_loadu_si128((__m128i*)&pDstU[x]);
		  _mm_storeu_si128((__m128i*)e_sProbe, xmm5);
		  fprintf(fp, "Result pDstU : %d, %d, %d, %d, %d, %d, %d, %d\n", e_sProbe[0], e_sProbe[1], e_sProbe[2], e_sProbe[3], e_sProbe[4], e_sProbe[5], e_sProbe[6], e_sProbe[7]);

		  xmm0 = _mm_loadu_si128((__m128i *)&pSrcV[x]);
		  xmm1 = _mm_loadu_si128((__m128i *)&pDstV[x]);
		  xmm3 = _mm_mullo_epi16(xmm1, xmm_2);
		  xmm4 = _mm_sub_epi16(xmm3, xmm0);
		  _mm_storeu_si128((__m128i*)&pDstV[x], xmm4);


#endif
		  
		  _mm_storeu_si128((__m128i*)R, xmmR);
		  _mm_storeu_si128((__m128i*)RV, xmmRV);
		  _mm_storeu_si128((__m128i*)SrcUR, xmmSrcU);
		  _mm_storeu_si128((__m128i*)SrcVR, xmmSrcV);

		  fprintf(fp, "SIMD pSrcU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, SrcUR[0], SrcUR[1], SrcUR[2], SrcUR[3], SrcUR[4], SrcUR[5], SrcUR[6], SrcUR[7]);
		  fprintf(fp, "SIMD pSrcV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, SrcVR[0], SrcVR[1], SrcVR[2], SrcVR[3], SrcVR[4], SrcVR[5], SrcVR[6], SrcVR[7]);
		  		  
		  pDDstU[x] = 2 * pDDstU[x] - pDSrcU[x];
		  pDDstV[x] = 2 * pDDstV[x] - pDSrcV[x];
		  DR[0] = pDDstU[x];
		  DRV[0] = pDDstV[x];
		  DSrcUR[0] = pDSrcU[x];
		  DSrcVR[0] = pDSrcV[x]; 

		  pDDstU[x + 1] = 2 * pDDstU[x + 1] - pDSrcU[x + 1];
		  pDDstV[x + 1] = 2 * pDDstV[x + 1] - pDSrcV[x + 1];
		  DR[1] = pDDstU[x + 1];
		  DRV[1] = pDDstV[x + 1];
		  DSrcUR[1] = pDSrcU[x+1];
		  DSrcVR[1] = pDSrcV[x+1];

		  pDDstU[x + 2] = 2 * pDDstU[x + 2] - pDSrcU[x + 2];
		  pDDstV[x + 2] = 2 * pDDstV[x + 2] - pDSrcV[x + 2];
		  DR[2] = pDDstU[x + 2];
		  DRV[2] = pDDstV[x + 2];
		  DSrcUR[2] = pDSrcU[x + 2];
		  DSrcVR[2] = pDSrcV[x + 2];

		  pDDstU[x + 3] = 2 * pDDstU[x + 3] - pDSrcU[x + 3];
		  pDDstV[x + 3] = 2 * pDDstV[x + 3] - pDSrcV[x + 3];
		  DR[3] = pDDstU[x + 3];
		  DRV[3] = pDDstV[x + 3];
		  DSrcUR[3] = pDSrcU[x + 3];
		  DSrcVR[3] = pDSrcV[x + 3];

		  pDDstU[x + 4] = 2 * pDDstU[x + 4] - pDSrcU[x + 4];
		  pDDstV[x + 4] = 2 * pDDstV[x + 4] - pDSrcV[x + 4];
		  DR[4] = pDDstU[x + 4];
		  DRV[4] = pDDstV[x + 4];
		  DSrcUR[4] = pDSrcU[x + 4];
		  DSrcVR[4] = pDSrcV[x + 4];

		  pDDstU[x + 5] = 2 * pDDstU[x + 5] - pDSrcU[x + 5];
		  pDDstV[x + 5] = 2 * pDDstV[x + 5] - pDSrcV[x + 5];
		  DR[5] = pDDstU[x + 5];
		  DRV[5] = pDDstV[x + 5];
		  DSrcUR[5] = pDSrcU[x + 5];
		  DSrcVR[5] = pDSrcV[x + 5];

		  pDDstU[x + 6] = 2 * pDDstU[x + 6] - pDSrcU[x + 6];
		  pDDstV[x + 6] = 2 * pDDstV[x + 6] - pDSrcV[x + 6];
		  DR[6] = pDDstU[x + 6];
		  DRV[6] = pDDstV[x + 6];
		  DSrcUR[6] = pDSrcU[x + 6];
		  DSrcVR[6] = pDSrcV[x + 6];

		  pDDstU[x + 7] = 2 * pDDstU[x + 7] - pDSrcU[x + 7];
		  pDDstV[x + 7] = 2 * pDDstV[x + 7] - pDSrcV[x + 7];
		  DR[7] = pDDstU[x + 7];
		  DRV[7] = pDDstV[x + 7];
		  DSrcUR[7] = pDSrcU[x + 7];
		  DSrcVR[7] = pDSrcV[x + 7];

//		  printf("     pSrcU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DSrcUR[0], DSrcUR[1], DSrcUR[2], DSrcUR[3], DSrcUR[4], DSrcUR[5], DSrcUR[6], DSrcUR[7]);
//		  printf("     pSrcV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DSrcVR[0], DSrcVR[1], DSrcVR[2], DSrcVR[3], DSrcVR[4], DSrcVR[5], DSrcVR[6], DSrcVR[7]);

//		  fprintf(fp, "Result SIMD pDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7]);
//		  fprintf(fp, "Result SIMD pDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, RV[0], RV[1], RV[2], RV[3], RV[4], RV[5], RV[6], RV[7]);

//		  printf("Result     pDDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DR[0], DR[1], DR[2], DR[3], DR[4], DR[5], DR[6], DR[7]);
//		  printf("Result     pDDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRV[0], DRV[1], DRV[2], DRV[3], DRV[4], DRV[5], DRV[6], DRV[7]);

	  }
	  pSrcU += iSrcStride;
	  pSrcV += iSrcStride;
	  pDstU += iDstStride;
	  pDstV += iDstStride;
//	  pDSrcU += iDSrcStride;
//	  pDSrcV += iDSrcStride;
//	  pDDstU += iDDstStride;
//	  pDDstV += iDDstStride;
	  fprintf(fp, "-- Result [%d] : %d, %d, %d, %d\n", y, pSrcU, pSrcV, pDstU, pDstV);
  }

//  printf("SIMD_HIGH2 : %d	%d	%d	%d\n", pSrcU, pSrcV, pDstU, pDstV);

	// for NO SIMD
  for (y = uiHeight - 1; y >= 0; y--)
  {
	  for (x = uiWidht - 1; x >= 0; x--)
	  {
#if DISABLING_CLIP_FOR_BIPREDME
		  pDDstU[x] = 2 * pDDstU[x] - pDSrcU[x];
		  pDDstV[x] = 2 * pDDstV[x] - pDSrcV[x];
#else
		  pDDstU[x] = ClipC(2 * pDDstU[x] - pDSrcU[x]);
		  pDDstV[x] = ClipC(2 * pDDstV[x] - pDSrcV[x]);
#endif
	  }
	  pDSrcU += iDSrcStride;
	  pDSrcV += iDSrcStride;
	  pDDstU += iDDstStride;
	  pDDstV += iDDstStride;
  }

//  printf("NO_SIMD2 : %d	%d	%d	%d\n", pDSrcU, pDSrcV, pDDstU, pDDstV);

  fclose(fp);
//  fclose(fp2);
}
#else
Void TComYuv::removeHighFreq(TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight)		// by org
{
	Int x, y;

	short DR[8] = { 0 };
	short DRV[8] = { 0 };
	short DRSrc[8] = {0};
	short DRDst[8] = {0};
	short DSrcUR[8] = { 0 };
	short DSrcVR[8] = { 0 };
	FILE *fp = fopen("output_NOSIMD.txt", "w");
	Pel* pSrc = pcYuvSrc->getLumaAddr(uiPartIdx);
	Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
	Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

	Pel* pDst = getLumaAddr(uiPartIdx);
	Pel* pDstU = getCbAddr(uiPartIdx);
	Pel* pDstV = getCrAddr(uiPartIdx);

	Int  iSrcStride = pcYuvSrc->getStride();
	Int  iDstStride = getStride();


	for (y = 0; y < uiHeight; y++)
	{
		fprintf(fp, " uiHeight [%d]\n", y);
		for (x = 0; x < uiHeight; x += 8)
		{
#if DISABLING_CLIP_FOR_BIPREDME
			
			pDst[x] = 2 * pDst[x] - pSrc[x];
			DRSrc[0] = pSrc[x];
			DRDst[0] = pDst[x];
			pDst[x + 1] = 2 * pDst[x + 1] - pSrc[x + 1];
			DRSrc[1] = pSrc[x+1];
			DRDst[1] = pDst[x+1];
			pDst[x + 2] = 2 * pDst[x + 2] - pSrc[x + 2];
			DRSrc[2] = pSrc[x+2];
			DRDst[2] = pDst[x+2];
			pDst[x + 3] = 2 * pDst[x + 3] - pSrc[x + 3];
			DRSrc[3] = pSrc[x+3];
			DRDst[3] = pDst[x+3];
			pDst[x + 4] = 2 * pDst[x + 4] - pSrc[x + 4];
			DRSrc[4] = pSrc[x+4];
			DRDst[4] = pDst[x+4];
			pDst[x + 5] = 2 * pDst[x + 5] - pSrc[x + 5];
			DRSrc[5] = pSrc[x+5];
			DRDst[5] = pDst[x+5];
			pDst[x + 6] = 2 * pDst[x + 6] - pSrc[x + 6];
			DRSrc[6] = pSrc[x+6];
			DRDst[6] = pDst[x+6];
			pDst[x + 7] = 2 * pDst[x + 7] - pSrc[x + 7];
			DRSrc[7] = pSrc[x+7];
			DRDst[7] = pDst[x+7];
#else
			pDst[x] = ClipY(2 * pDst[x] - pSrc[x]);
#endif
//			fprintf(fp, "pSrc[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRSrc[0], DRSrc[1], DRSrc[2], DRSrc[3], DRSrc[4], DRSrc[5], DRSrc[6], DRSrc[7]);
//			fprintf(fp, "pDDst[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRDst[0], DRDst[1], DRDst[2], DRDst[3], DRDst[4], DRDst[5], DRDst[6], DRDst[7]);
		}
			pSrc += iSrcStride;
			pDst += iDstStride;
//			fprintf(fp, "Sum of pSrc, pDst : %d, %d\n", pSrc, pDst);
	}

	iSrcStride = pcYuvSrc->getCStride();
	iDstStride = getCStride();

	uiHeight >>= 1;
	uiWidht >>= 1;

	for (y = 0; y < uiHeight; y++)
	{
		fprintf(fp, "Result y[%d]\n", y);
		for (x = 0; x < uiHeight; x += 8)
		{
#if DISABLING_CLIP_FOR_BIPREDME

			DR[0] = pDstU[x];
			DRV[0] = pDstV[x];

			DR[1] = pDstU[x + 1];
			DRV[1] = pDstV[x + 1];

			DR[2] = pDstU[x + 2];
			DRV[2] = pDstV[x + 2];

			DR[3] = pDstU[x + 3];
			DRV[3] = pDstV[x + 3];

			DR[4] = pDstU[x + 4];
			DRV[4] = pDstV[x + 4];

			DR[5] = pDstU[x + 5];
			DRV[5] = pDstV[x + 5];

			DR[6] = pDstU[x + 6];
			DRV[6] = pDstV[x + 6];

			DR[7] = pDstU[x + 7];
			DRV[7] = pDstV[x + 7];

			pDstU[x] = 2 * pDstU[x] - pSrcU[x];
			pDstV[x] = 2 * pDstV[x] - pSrcV[x];
			DR[0] = pDstU[x];
			DRV[0] = pDstV[x];
			DSrcUR[0] = pSrcU[x];
			DSrcVR[0] = pSrcV[x]; 

			fprintf(fp, "Origin    pDDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DR[0], DR[1], DR[2], DR[3], DR[4], DR[5], DR[6], DR[7]);
			fprintf(fp, "Origin     pDDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRV[0], DRV[1], DRV[2], DRV[3], DRV[4], DRV[5], DRV[6], DRV[7]);

			pDstU[x + 1] = 2 * pDstU[x + 1] - pSrcU[x + 1];
			pDstV[x + 1] = 2 * pDstV[x + 1] - pSrcV[x + 1];
			DR[1] = pDstU[x + 1];
			DRV[1] = pDstV[x + 1];
			DSrcUR[1] = pSrcU[x+1];
			DSrcVR[1] = pSrcV[x+1];

			pDstU[x + 2] = 2 * pDstU[x + 2] - pSrcU[x + 2];
			pDstV[x + 2] = 2 * pDstV[x + 2] - pSrcV[x + 2];
			DR[2] = pDstU[x + 2];
			DRV[2] = pDstV[x + 2];
			DSrcUR[2] = pSrcU[x + 2];
			DSrcVR[2] = pSrcV[x + 2];

			pDstU[x + 3] = 2 * pDstU[x + 3] - pSrcU[x + 3];
			pDstV[x + 3] = 2 * pDstV[x + 3] - pSrcV[x + 3];
			DR[3] = pDstU[x + 3];
			DRV[3] = pDstV[x + 3];
			DSrcUR[3] = pSrcU[x + 3];
			DSrcVR[3] = pSrcV[x + 3];

			pDstU[x + 4] = 2 * pDstU[x + 4] - pSrcU[x + 4];
			pDstV[x + 4] = 2 * pDstV[x + 4] - pSrcV[x + 4];
			DR[4] = pDstU[x + 4];
			DRV[4] = pDstV[x + 4];
			DSrcUR[4] = pSrcU[x + 4];
			DSrcVR[4] = pSrcV[x + 4];

			pDstU[x + 5] = 2 * pDstU[x + 5] - pSrcU[x + 5];
			pDstV[x + 5] = 2 * pDstV[x + 5] - pSrcV[x + 5];
			DR[5] = pDstU[x + 5];
			DRV[5] = pDstV[x + 5];
			DSrcUR[5] = pSrcU[x + 5];
			DSrcVR[5] = pSrcV[x + 5];

			pDstU[x + 6] = 2 * pDstU[x + 6] - pSrcU[x + 6];
			pDstV[x + 6] = 2 * pDstV[x + 6] - pSrcV[x + 6];
			DR[6] = pDstU[x + 6];
			DRV[6] = pDstV[x + 6];
			DSrcUR[6] = pSrcU[x + 6];
			DSrcVR[6] = pSrcV[x + 6];

			pDstU[x + 7] = 2 * pDstU[x + 7] - pSrcU[x + 7];
			pDstV[x + 7] = 2 * pDstV[x + 7] - pSrcV[x + 7];
			DR[7] = pDstU[x + 7];
			DRV[7] = pDstV[x + 7];
			DSrcUR[7] = pSrcU[x + 7];
			DSrcVR[7] = pSrcV[x + 7];
			fprintf(fp, "     pSrcU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DSrcUR[0], DSrcUR[1], DSrcUR[2], DSrcUR[3], DSrcUR[4], DSrcUR[5], DSrcUR[6], DSrcUR[7]);
//			fprintf(fp, "     pSrcV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DSrcVR[0], DSrcVR[1], DSrcVR[2], DSrcVR[3], DSrcVR[4], DSrcVR[5], DSrcVR[6], DSrcVR[7]);
			fprintf(fp, "Result     pDDstU[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DR[0], DR[1], DR[2], DR[3], DR[4], DR[5], DR[6], DR[7]);
			fprintf(fp, "Result     pDDstV[%d] : %d, %d, %d, %d, %d, %d, %d, %d\n", x, DRV[0], DRV[1], DRV[2], DRV[3], DRV[4], DRV[5], DRV[6], DRV[7]);

#else
			pDstU[x] = ClipC(2 * pDstU[x] - pSrcU[x]);
			pDstV[x] = ClipC(2 * pDstV[x] - pSrcV[x]);
#endif
		}
		pSrcU += iSrcStride;
		pSrcV += iSrcStride;
		pDstU += iDstStride;
		pDstV += iDstStride;
		fprintf(fp, "--Result [%d] : %d, %d, %d, %d\n", y, pSrcU, pSrcV, pDstU, pDstV);
	}
	fclose(fp);
}
#endif*/
//! \}
