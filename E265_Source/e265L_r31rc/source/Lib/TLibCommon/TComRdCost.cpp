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

/** \file     TComRdCost.cpp
    \brief    RD cost computation class
*/

#include <math.h>
#include <assert.h>
#include "TComRom.h"
#include "TComRdCost.h"

//! \ingroup TLibCommon
//! \{

TComRdCost::TComRdCost()
{
  init();

  //---------------------------------------------------------------------------
  //  ETRI Modification  Revision
  //---------------------------------------------------------------------------
  //ESPRINTF( ETRI_MODIFICATION_V01, stderr, " Compiled @%s  [%s] \n", __DATE__, __TIME__);

}

TComRdCost::~TComRdCost()
{
#if !FIX203
  xUninit();
#endif

	ETRI_FreeMEdata ();
}


Void TComRdCost::ETRI_AllocMEdata ()
{
#if ETRI_FAST_INTEGERME
	em_bAllocMEData = false;
	em_tmm = nullptr;
	em_smm = nullptr;

	if (!em_bAllocMEData)
	{
		Int j;
		em_tmm    = (UInt **)_aligned_malloc(ETRI_VER_nSAD * sizeof(Int *), 32);
		if (em_tmm == nullptr)    {EDPRINTF(stderr, "Aligned Memory Allocation Fail \n"); ETRI_EXIT(0);}

		em_tmm[0] = (UInt *) _aligned_malloc(ETRI_VER_nSAD * ETRI_HOR_nSAD* sizeof(Int), 32);
		if (em_tmm[0] == nullptr) {EDPRINTF(stderr, "Aligned Memory Allocation Fail \n"); ETRI_EXIT(0);}

		for (j=1; j<ETRI_VER_nSAD; j++)
			em_tmm[j]=  em_tmm[j-1] + ETRI_HOR_nSAD; 

		em_smm    = (UInt **)_aligned_malloc(ETRI_AUXMEM_SIZE * sizeof(Int *), 32);
		if (em_smm == nullptr)    {EDPRINTF(stderr, "Aligned Memory Allocation Fail \n"); ETRI_EXIT(0);}

		em_smm[0] = (UInt *) _aligned_malloc(ETRI_AUXMEM_SIZE * ETRI_HOR_nSAD * sizeof(Int), 32);
		if (em_smm[0] == nullptr) {EDPRINTF(stderr, "Aligned Memory Allocation Fail \n"); ETRI_EXIT(0);}
		
		for (j=1; j<ETRI_AUXMEM_SIZE; j++)
			em_smm[j]=  em_smm[j-1] + ETRI_HOR_nSAD; 

		em_bAllocMEData = true;
	}

#endif

}

Void TComRdCost::ETRI_FreeMEdata ()
{
#if ETRI_FAST_INTEGERME
	if (!em_bAllocMEData)	return;	// 2013 5 27 by Seok : For System Stability

	ETRI_AlignedMallocFree(em_tmm[0]);
	ETRI_AlignedMallocFree(em_tmm);
	ETRI_AlignedMallocFree(em_smm[0]);
	ETRI_AlignedMallocFree(em_smm);

	em_bAllocMEData = false;
#endif	
}



// Calculate RD functions
Double TComRdCost::calcRdCost( UInt uiBits, UInt uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;
  
  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
      dLambda = (Double)m_uiLambdaMotionSAD;
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }
  
  if (bFlag)
  {
    // Intra8x8, Intra4x4 Block only...
#if SEQUENCE_LEVEL_LOSSLESS
    dRdCost = (Double)(uiBits);
#else
    dRdCost = (((Double)uiDistortion) + ((Double)uiBits * dLambda));
#endif
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      dRdCost = ((Double)uiDistortion + (Double)((Int)(uiBits * dLambda+.5)>>16));
      dRdCost = (Double)(UInt)floor(dRdCost);
    }
    else
    {
#if SEQUENCE_LEVEL_LOSSLESS
      dRdCost = (Double)(uiBits);
#else
      dRdCost = ((Double)uiDistortion + (Double)((Int)(uiBits * dLambda+.5)));
      dRdCost = (Double)(UInt)floor(dRdCost);
#endif
    }
  }
  
  return dRdCost;
}

Double TComRdCost::calcRdCost64( UInt64 uiBits, UInt64 uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;
  
  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
      dLambda = (Double)m_uiLambdaMotionSAD;
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }
  
  if (bFlag)
  {
    // Intra8x8, Intra4x4 Block only...
#if SEQUENCE_LEVEL_LOSSLESS
    dRdCost = (Double)(uiBits);
#else
    dRdCost = (((Double)(Int64)uiDistortion) + ((Double)(Int64)uiBits * dLambda));
#endif
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      dRdCost = ((Double)(Int64)uiDistortion + (Double)((Int)((Int64)uiBits * dLambda+.5)>>16));
      dRdCost = (Double)(UInt)floor(dRdCost);
    }
    else
    {
#if SEQUENCE_LEVEL_LOSSLESS
      dRdCost = (Double)(uiBits);
#else
      dRdCost = ((Double)(Int64)uiDistortion + (Double)((Int)((Int64)uiBits * dLambda+.5)));
      dRdCost = (Double)(UInt)floor(dRdCost);
#endif
    }
  }
  
  return dRdCost;
}

Void TComRdCost::setLambda( Double dLambda )
{
  m_dLambda           = dLambda;
  m_sqrtLambda        = sqrt(m_dLambda);
  m_uiLambdaMotionSAD = (UInt)floor(65536.0 * m_sqrtLambda);
  m_uiLambdaMotionSSE = (UInt)floor(65536.0 * m_dLambda   );
}


// Initalize Function Pointer by [eDFunc]
Void TComRdCost::init()
{
  m_afpDistortFunc[0]  = NULL;                  // for DF_DEFAULT
  
  m_afpDistortFunc[1]  = TComRdCost::xGetSSE;
  m_afpDistortFunc[2]  = TComRdCost::xGetSSE4;
  m_afpDistortFunc[3]  = TComRdCost::xGetSSE8;
  m_afpDistortFunc[4]  = TComRdCost::xGetSSE16;
  m_afpDistortFunc[5]  = TComRdCost::xGetSSE32;
  m_afpDistortFunc[6]  = TComRdCost::xGetSSE64;
  m_afpDistortFunc[7]  = TComRdCost::xGetSSE16N;
  
  m_afpDistortFunc[8]  = TComRdCost::xGetSAD;
  m_afpDistortFunc[9]  = TComRdCost::xGetSAD4;
  m_afpDistortFunc[10] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[11] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[12] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[13] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[14] = TComRdCost::xGetSAD16N;
  
  m_afpDistortFunc[15] = TComRdCost::xGetSAD;
  m_afpDistortFunc[16] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[17] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[18] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[19] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[20] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[21] = TComRdCost::xGetSAD16N;
  
#if AMP_SAD
  m_afpDistortFunc[43] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[44] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[45] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[46] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[47] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[48] = TComRdCost::xGetSAD48;
#endif
  m_afpDistortFunc[22] = TComRdCost::xGetHADs;
  m_afpDistortFunc[23] = TComRdCost::xGetHADs;
  m_afpDistortFunc[24] = TComRdCost::xGetHADs;
  m_afpDistortFunc[25] = TComRdCost::xGetHADs;
  m_afpDistortFunc[26] = TComRdCost::xGetHADs;
  m_afpDistortFunc[27] = TComRdCost::xGetHADs;
  m_afpDistortFunc[28] = TComRdCost::xGetHADs;

#if (ETRI_FAST_INTEGERME)
  m_afpDistortFunc[52] = TComRdCost::ETRI_GetSAD;
  m_afpDistortFunc[53] = TComRdCost::ETRI_GetSAD4;
  m_afpDistortFunc[54] = TComRdCost::ETRI_GetSAD8;
  m_afpDistortFunc[55] = TComRdCost::ETRI_GetSAD16;
  m_afpDistortFunc[56] = TComRdCost::ETRI_GetSAD32;
  m_afpDistortFunc[57] = TComRdCost::ETRI_GetSAD64;

  ETRI_AllocMEdata ();
#endif	//#if (ETRI_FAST_INTEGERME)
  
#if !FIX203
  m_puiComponentCostOriginP = NULL;
  m_puiComponentCost        = NULL;
  m_puiVerCost              = NULL;
  m_puiHorCost              = NULL;
#endif
  m_uiCost                  = 0;
  m_iCostScale              = 0;
#if !FIX203
  m_iSearchLimit            = 0xdeaddead;
#endif
}

#if !FIX203
Void TComRdCost::initRateDistortionModel( Int iSubPelSearchLimit )
{
  // make it larger
  iSubPelSearchLimit += 4;
  iSubPelSearchLimit *= 8;
  
  if( m_iSearchLimit != iSubPelSearchLimit )
  {
    xUninit();
    
    m_iSearchLimit = iSubPelSearchLimit;
    
    m_puiComponentCostOriginP = new UInt[ 4 * iSubPelSearchLimit ];
    iSubPelSearchLimit *= 2;
    
    m_puiComponentCost = m_puiComponentCostOriginP + iSubPelSearchLimit;
    
    for( Int n = -iSubPelSearchLimit; n < iSubPelSearchLimit; n++)
    {
      m_puiComponentCost[n] = xGetComponentBits( n );
    }
  }
}

Void TComRdCost::xUninit()
{
  if( NULL != m_puiComponentCostOriginP )
  {
    delete [] m_puiComponentCostOriginP;
    m_puiComponentCostOriginP = NULL;
  }
}
#endif

UInt TComRdCost::xGetComponentBits( Int iVal )
{
#if ETRI_COMPONENT_BIT_OPTIMIZATION
	return g_pComponentBits[iVal];
#else
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);
  
  assert ( uiTemp );
  
  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  
  return uiLength;
#endif
}

Void TComRdCost::setDistParam( UInt uiBlkWidth, UInt uiBlkHeight, DFunc eDFunc, DistParam& rcDistParam )
{
  // set Block Width / Height
  rcDistParam.iCols    = uiBlkWidth;
  rcDistParam.iRows    = uiBlkHeight;
  rcDistParam.DistFunc = m_afpDistortFunc[eDFunc + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  
  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (ME)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, DistParam& rcDistParam )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;

  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();

#if ETRI_FAST_INTEGERME
	// Storethe Index of  Default SAD Function Pointer to backup_data 	
	rcDistParam.BackupDistFuncIdx = DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + 1; 
	rcDistParam.DistFunc = m_afpDistortFunc[rcDistParam.BackupDistFuncIdx];
#else
  rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
#endif

#if AMP_SAD
  if (rcDistParam.iCols == 12)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[43 ];
  }
  else if (rcDistParam.iCols == 24)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[44 ];
  }
  else if (rcDistParam.iCols == 48)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[45 ];
  }
#endif

  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (subpel ME with step)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, Int iStep, DistParam& rcDistParam, Bool bHADME )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;
  
  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride * iStep;
  
  // set Step for interpolated buffer
  rcDistParam.iStep = iStep;
  
  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();
  
  // set distortion function
  if ( !bHADME )
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
#if AMP_SAD
    if (rcDistParam.iCols == 12)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[46 ];
    }
    else if (rcDistParam.iCols == 24)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[47 ];
    }
    else if (rcDistParam.iCols == 48)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[48 ];
    }
#endif
  }
  else
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_HADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  }
  
  // initialize
  rcDistParam.iSubShift  = 0;
}

Void
TComRdCost::setDistParam( DistParam& rcDP, Int bitDepth, Pel* p1, Int iStride1, Pel* p2, Int iStride2, Int iWidth, Int iHeight, Bool bHadamard )
{
  rcDP.pOrg       = p1;
  rcDP.pCur       = p2;
  rcDP.iStrideOrg = iStride1;
  rcDP.iStrideCur = iStride2;
  rcDP.iCols      = iWidth;
  rcDP.iRows      = iHeight;
  rcDP.iStep      = 1;
  rcDP.iSubShift  = 0;
  rcDP.bitDepth   = bitDepth;
  rcDP.DistFunc   = m_afpDistortFunc[ ( bHadamard ? DF_HADS : DF_SADS ) + g_aucConvertToBit[ iWidth ] + 1 ];
}

UInt TComRdCost::calcHAD(Int bitDepth, Pel* pi0, Int iStride0, Pel* pi1, Int iStride1, Int iWidth, Int iHeight )
{
  UInt uiSum = 0, uiHADDC = 0, uiSumHADDC = 0;
  Int x, y;

#if ETRI_EM_OPERATION_OPTIMIZATION
  if (((iWidth & 0x07) == 0) && ((iHeight & 0x07) == 0))
  {
	  for (y = 0; y<iHeight; y += 8)
	  {
		  for (x = 0; x<iWidth; x += 8)
		  {
			  uiSum += xCalcHADs8x8(&pi0[x], &pi1[x], iStride0, iStride1, 1, uiHADDC); uiSumHADDC +=  uiHADDC;
		  }
		  pi0 += iStride0 << 3;
		  pi1 += iStride1 << 3;
	  }
  }
  else
  {
	  assert((iWidth & 0x03) == 0 && (iHeight & 0x03) == 0);

	  for (y = 0; y<iHeight; y += 4)
	  {
		  for (x = 0; x<iWidth; x += 4)
		  {
			  uiSum += xCalcHADs4x4(&pi0[x], &pi1[x], iStride0, iStride1, 1, uiHADDC); uiSumHADDC +=  uiHADDC;
		  }
		  pi0 += iStride0 << 2;
		  pi1 += iStride1 << 2;
	  }
  }
#else
  if ( ( (iWidth % 8) == 0 ) && ( (iHeight % 8) == 0 ) )
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &pi0[x], &pi1[x], iStride0, iStride1, 1, uiHADDC);
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else
  {
    assert(iWidth % 4 == 0 && iHeight % 4 == 0);
    
    for ( y=0; y<iHeight; y+= 4 )
    {
      for ( x=0; x<iWidth; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &pi0[x], &pi1[x], iStride0, iStride1, 1, uiHADDC);
      }
      pi0 += iStride0*4;
      pi1 += iStride1*4;
    }
  }
#endif
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8);

}

UInt TComRdCost::getDistPart(Int bitDepth, Pel* piCur, Int iCurStride,  Pel* piOrg, Int iOrgStride, UInt uiBlkWidth, UInt uiBlkHeight, TextType eText, DFunc eDFunc)
{
  DistParam cDtParam;
  setDistParam( uiBlkWidth, uiBlkHeight, eDFunc, cDtParam );
  cDtParam.pOrg       = piOrg;
  cDtParam.pCur       = piCur;
  cDtParam.iStrideOrg = iOrgStride;
  cDtParam.iStrideCur = iCurStride;
  cDtParam.iStep      = 1;

  cDtParam.bApplyWeight = false;
  cDtParam.uiComp       = 255;    // just for assert: to be sure it was set before use, since only values 0,1 or 2 are allowed.
  cDtParam.bitDepth = bitDepth;

  if (eText == TEXT_CHROMA_U)
  {
   return ((Int) (m_cbDistortionWeight * cDtParam.DistFunc( &cDtParam )));
  }
  else if (eText == TEXT_CHROMA_V)
  {
   return ((Int) (m_crDistortionWeight * cDtParam.DistFunc( &cDtParam )));
  }
  else
  {
    return cDtParam.DistFunc( &cDtParam );
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xGetSAD( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD4( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[3];
	xmm[0] = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep << 1) // iRows는 8, 16 iSubStep은 1, 2 값을 가짐, 416x240 콘텐츠로 인코딩할 경우에 대해서 확인한 것이며, 다른 함수에서도 동일한 환경에서 확인한 값을 적어둠
	{
		xmm[1] = _mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg)));
		xmm[2] = _mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)piCur), _mm_loadu_si128((__m128i const *)(piCur + iStrideCur)));

		xmm[0] = _mm_adds_epu16(xmm[0], _mm_abs_epi16(_mm_sub_epi16(xmm[1], xmm[2]))); // abs(piOrg-piCur) 값을 누적해서 더함, xmm[0]에 16bit 단위로 더해진 값들이 들어있음, 이것을 다 더해야지 최종 SAD값이 나옴

		piOrg += iStrideOrg << 1;
		piCur += iStrideCur << 1;
	}

	xmm[1] = _mm_xor_si128(xmm[1], xmm[1]); // xmm[1] = 0 0 0 0 0 0 0 0
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi16(xmm[0], xmm[1]), _mm_unpackhi_epi16(xmm[0], xmm[1])); //xmm[0] = x3+x7 x2+x6 x1+x5 x0+x4
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 x1+x3+x5+x7 0 x0+x2+x4+x6
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 0 0 x1+x2+x3+x4+x5+x6+x7

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	UInt uiSum = est[0] <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  
	if ( pcDtParam->bApplyWeight ) 
	{
		return xGetSADw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
#endif	// ETRI_SIMD_MD
}

UInt TComRdCost::xGetSAD8( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[2];
	xmm[0] = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep) // iRows 4, 8, 16, 32 iSubStep 1, 2
	{
		xmm[0] = _mm_adds_epu16(xmm[0], _mm_abs_epi16(_mm_sub_epi16(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)piCur))));
		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	xmm[1] = _mm_setzero_si128();
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi16(xmm[0], xmm[1]), _mm_unpackhi_epi16(xmm[0], xmm[1]));
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 x1+x3+x5+x7 0 x0+x2+x4+x6
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 0 0 x1+x2+x3+x4+x5+x6+x7

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	UInt uiSum = est[0] <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

//	return (est[0] <<= iSubShift);
#else  
	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}
	Pel* piOrg      = pcDtParam->pOrg;
	Pel* piCur      = pcDtParam->pCur;
	Int  iRows      = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
#endif // ETRI_SIMD_MD 
}

UInt TComRdCost::xGetSAD16( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[5];
	xmm[0] = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep) // iRows 4, 8, 12, 16, 32 iSubStep 1, 2
	{
		xmm[1] = _mm_loadu_si128((__m128i const *)piOrg); // a7 a6 a5 a4 a3 a2 a1 a0
		xmm[2] = _mm_loadu_si128((__m128i const *)(piOrg + 8)); // a15 a14 a13 a12 a11 a10 a9 a8
		xmm[3] = _mm_loadu_si128((__m128i const *)piCur); // b7 b6 b5 b4 b3 b2 b1 b0 // 8 bit
		xmm[4] = _mm_loadu_si128((__m128i const *)(piCur + 8)); // b15 b14 b13 b12 b11 b10 b9 b8 // 8 bit

		xmm[0] = _mm_adds_epu16(xmm[0], _mm_adds_epu16(_mm_abs_epi16(_mm_sub_epi16(xmm[1], xmm[3])), _mm_abs_epi16(_mm_sub_epi16(xmm[2], xmm[4]))));

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	xmm[1] = _mm_xor_si128(xmm[1], xmm[1]);
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi16(xmm[0], xmm[1]), _mm_unpackhi_epi16(xmm[0], xmm[1]));
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 x1+x3+x5+x7 0 x0+x2+x4+x6
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 0 0 x1+x2+x3+x4+x5+x6+x7

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	UInt uiSum = est[0] <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  
	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );
		uiSum += abs( piOrg[8] - piCur[8] );
		uiSum += abs( piOrg[9] - piCur[9] );
		uiSum += abs( piOrg[10] - piCur[10] );
		uiSum += abs( piOrg[11] - piCur[11] );
		uiSum += abs( piOrg[12] - piCur[12] );
		uiSum += abs( piOrg[13] - piCur[13] );
		uiSum += abs( piOrg[14] - piCur[14] );
		uiSum += abs( piOrg[15] - piCur[15] );

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth - 8);
#endif	// ETRI_SIMD_MD 
}

#if AMP_SAD
UInt TComRdCost::xGetSAD12( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[8];
	xmm[0] = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep << 1) // iRows 16 iSubStep 2
	{
		xmm[1] = _mm_loadu_si128((__m128i const *)piOrg);
		xmm[2] = _mm_loadu_si128((__m128i const *)(piOrg + 8));
		xmm[3] = _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg));
		xmm[4] = _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg + 8));
		xmm[2] = _mm_unpacklo_epi64(xmm[2], xmm[4]);
		xmm[4] = _mm_loadu_si128((__m128i const *)piCur);
		xmm[5] = _mm_loadu_si128((__m128i const *)(piCur + 8));
		xmm[6] = _mm_loadu_si128((__m128i const *)(piCur + iStrideCur));
		xmm[7] = _mm_loadu_si128((__m128i const *)(piCur + iStrideCur + 8));
		xmm[5] = _mm_unpacklo_epi64(xmm[5], xmm[7]);

		xmm[0] = _mm_adds_epu16(xmm[0], _mm_adds_epu16(_mm_abs_epi16(_mm_sub_epi16(xmm[3], xmm[6])), _mm_adds_epu16(_mm_abs_epi16(_mm_sub_epi16(xmm[1], xmm[4])), _mm_abs_epi16(_mm_sub_epi16(xmm[2], xmm[5])))));

		piOrg += iStrideOrg << 1;
		piCur += iStrideCur << 1;
	}

	xmm[1] = _mm_xor_si128(xmm[1], xmm[1]);
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi16(xmm[0], xmm[1]), _mm_unpackhi_epi16(xmm[0], xmm[1]));
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 x1+x3+x5+x7 0 x0+x2+x4+x6
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 0 0 x1+x2+x3+x4+x5+x6+x7

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	UInt uiSum = est[0] <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  
	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );
		uiSum += abs( piOrg[8] - piCur[8] );
		uiSum += abs( piOrg[9] - piCur[9] );
		uiSum += abs( piOrg[10] - piCur[10] );
		uiSum += abs( piOrg[11] - piCur[11] );

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
#endif // ETRI_SIMD_MD 
}
#endif

/**
	@brief : This function is considered as a useless. EDPRINTF is not activated. 
*/
UInt TComRdCost::xGetSAD16N( DistParam* pcDtParam )
{
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;

  EDPRINTF(stderr, "Active ????\n");
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (Int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD32( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD

	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[8], xmmz, smm;

	xmmz = _mm_set1_epi16(1);
	smm = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep) // iRows 4, 8, 12, 16, 32 iSubStep 1, 2
	{
		xmm[0] = _mm_load_si128((__m128i const *) piOrg);		/// O[7:0]
		xmm[1] = _mm_load_si128((__m128i const *)(piOrg+8)); 	/// O[7:0]
		xmm[2] = _mm_load_si128((__m128i const *)(piOrg+16));	/// O[7:0]
		xmm[3] = _mm_load_si128((__m128i const *)(piOrg+24));	/// O[7:0]
		xmm[4] = _mm_loadu_si128((__m128i const *) piCur);		/// O[7:0]
		xmm[5] = _mm_loadu_si128((__m128i const *)(piCur+8));	/// O[7:0]
		xmm[6] = _mm_loadu_si128((__m128i const *)(piCur+16));	/// O[7:0]
		xmm[7] = _mm_loadu_si128((__m128i const *)(piCur+24));	/// O[7:0]

		xmm[0] = _mm_sub_epi16(xmm[0], xmm[4]);
		xmm[1] = _mm_sub_epi16(xmm[1], xmm[5]);
		xmm[2] = _mm_sub_epi16(xmm[2], xmm[6]);
		xmm[3] = _mm_sub_epi16(xmm[3], xmm[7]);

		xmm[0] = _mm_abs_epi16(xmm[0]);
		xmm[1] = _mm_abs_epi16(xmm[1]);		
		xmm[2] = _mm_abs_epi16(xmm[2]);
		xmm[3] = _mm_abs_epi16(xmm[3]);		

		xmm[0] = _mm_add_epi16(xmm[0], xmm[1]);	/// 12 bit
		xmm[0] = _mm_add_epi16(xmm[0], xmm[2]);	/// 13 bit
		xmm[0] = _mm_add_epi16(xmm[0], xmm[3]);	/// 14 bit

		xmm[0] = _mm_madd_epi16(xmm[0], xmmz);	/// 7+6 5+4 3+2 1+0 
		smm	 = _mm_add_epi32(smm, xmm[0]);	

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}	

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	UInt uiSumSAD = _mm_extract_epi32(smm, 0);
	uiSumSAD <<= iSubShift;

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	return uiSumSAD >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  

	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}

	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );
		uiSum += abs( piOrg[8] - piCur[8] );
		uiSum += abs( piOrg[9] - piCur[9] );
		uiSum += abs( piOrg[10] - piCur[10] );
		uiSum += abs( piOrg[11] - piCur[11] );
		uiSum += abs( piOrg[12] - piCur[12] );
		uiSum += abs( piOrg[13] - piCur[13] );
		uiSum += abs( piOrg[14] - piCur[14] );
		uiSum += abs( piOrg[15] - piCur[15] );
		uiSum += abs( piOrg[16] - piCur[16] );
		uiSum += abs( piOrg[17] - piCur[17] );
		uiSum += abs( piOrg[18] - piCur[18] );
		uiSum += abs( piOrg[19] - piCur[19] );
		uiSum += abs( piOrg[20] - piCur[20] );
		uiSum += abs( piOrg[21] - piCur[21] );
		uiSum += abs( piOrg[22] - piCur[22] );
		uiSum += abs( piOrg[23] - piCur[23] );
		uiSum += abs( piOrg[24] - piCur[24] );
		uiSum += abs( piOrg[25] - piCur[25] );
		uiSum += abs( piOrg[26] - piCur[26] );
		uiSum += abs( piOrg[27] - piCur[27] );
		uiSum += abs( piOrg[28] - piCur[28] );
		uiSum += abs(piOrg[29] - piCur[29]);
		uiSum += abs(piOrg[30] - piCur[30]);
		uiSum += abs(piOrg[31] - piCur[31]);

		piOrg += iStrideOrg;
		piCur += iStrideCur;
}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth - 8);

#endif // ETRI_SIMD_MD 
}

#if AMP_SAD
UInt TComRdCost::xGetSAD24( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[7];
	xmm[0] = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep) // iRows 4, 8, 12, 16, 32 iSubStep 1, 2
	{
		xmm[1] = _mm_loadu_si128((__m128i const *)piOrg); // a7 a6 a5 a4 a3 a2 a1 a0
		xmm[2] = _mm_loadu_si128((__m128i const *)(piOrg + 8)); // a15 a14 a13 a12 a11 a10 a9 a8
		xmm[3] = _mm_loadu_si128((__m128i const *)(piOrg + 16)); // a7 a6 a5 a4 a3 a2 a1 a0
		xmm[4] = _mm_loadu_si128((__m128i const *)piCur); // b7 b6 b5 b4 b3 b2 b1 n0 // 8 bit
		xmm[5] = _mm_loadu_si128((__m128i const *)(piCur + 8)); // b15 b14 b13 b12 b11 b10 b9 b8 // 8 bit  
		xmm[6] = _mm_loadu_si128((__m128i const *)(piCur + 16)); // b7 b6 b5 b4 b3 b2 b1 n0 // 8 bit

		xmm[0] = _mm_adds_epu16(xmm[0], _mm_adds_epu16(_mm_abs_epi16(_mm_sub_epi16(xmm[3], xmm[6])), _mm_adds_epu16(_mm_abs_epi16(_mm_sub_epi16(xmm[1], xmm[4])), _mm_abs_epi16(_mm_sub_epi16(xmm[2], xmm[5])))));

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	xmm[1] = _mm_xor_si128(xmm[1], xmm[1]);
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi16(xmm[0], xmm[1]), _mm_unpackhi_epi16(xmm[0], xmm[1]));
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 x1+x3+x5+x7 0 x0+x2+x4+x6
	xmm[0] = _mm_add_epi32(_mm_unpacklo_epi32(xmm[0], xmm[1]), _mm_unpackhi_epi32(xmm[0], xmm[1])); //xmm[0] = 0 0 0 x1+x2+x3+x4+x5+x6+x7

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	UInt uiSum = est[0] <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  
	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );
		uiSum += abs( piOrg[8] - piCur[8] );
		uiSum += abs( piOrg[9] - piCur[9] );
		uiSum += abs( piOrg[10] - piCur[10] );
		uiSum += abs( piOrg[11] - piCur[11] );
		uiSum += abs( piOrg[12] - piCur[12] );
		uiSum += abs( piOrg[13] - piCur[13] );
		uiSum += abs( piOrg[14] - piCur[14] );
		uiSum += abs( piOrg[15] - piCur[15] );
		uiSum += abs( piOrg[16] - piCur[16] );
		uiSum += abs( piOrg[17] - piCur[17] );
		uiSum += abs( piOrg[18] - piCur[18] );
		uiSum += abs( piOrg[19] - piCur[19] );
		uiSum += abs( piOrg[20] - piCur[20] );
		uiSum += abs( piOrg[21] - piCur[21] );
		uiSum += abs( piOrg[22] - piCur[22] );
		uiSum += abs( piOrg[23] - piCur[23] );

		piOrg += iStrideOrg;
		piCur += iStrideCur;
}

	uiSum <<= iSubShift;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth - 8);
#endif // ETRI_SIMD_MD 
}

#endif

UInt TComRdCost::xGetSAD64( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		return xGetSADw(pcDtParam);
	}

	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;
	Int  iRows = pcDtParam->iRows;
	Int  iSubShift = pcDtParam->iSubShift;
	Int  iSubStep = (1 << iSubShift);
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

	__m128i xmm[8], cmm[8], xmmz, smm;
		
	xmmz = _mm_set1_epi16(1);
	smm = _mm_setzero_si128();

	for (; iRows != 0; iRows -= iSubStep) // iRows 4, 8, 12, 16, 32 iSubStep 1, 2
	{
		xmm[0] = _mm_load_si128((__m128i const *) piOrg);		/// O[7:0]
		xmm[1] = _mm_load_si128((__m128i const *)(piOrg+8));  	/// O[15:8]
		xmm[2] = _mm_load_si128((__m128i const *)(piOrg+16));	/// O[23:16]
		xmm[3] = _mm_load_si128((__m128i const *)(piOrg+24));	/// O[31:24]
		xmm[4] = _mm_load_si128((__m128i const *)(piOrg+32));	/// O[39:32]
		xmm[5] = _mm_load_si128((__m128i const *)(piOrg+40));  	/// O[47:40]
		xmm[6] = _mm_load_si128((__m128i const *)(piOrg+48));	/// O[55:48]
		xmm[7] = _mm_load_si128((__m128i const *)(piOrg+56));	/// O[63:56]

		cmm[0] = _mm_loadu_si128((__m128i const *) piCur);		/// C[7:0]
		cmm[1] = _mm_loadu_si128((__m128i const *)(piCur+8));	/// C[15:8]
		cmm[2] = _mm_loadu_si128((__m128i const *)(piCur+16));	/// C[23:16]
		cmm[3] = _mm_loadu_si128((__m128i const *)(piCur+24));	/// C[31:24]
		cmm[4] = _mm_loadu_si128((__m128i const *)(piCur+32));	/// C[39:32]
		cmm[5] = _mm_loadu_si128((__m128i const *)(piCur+40));  	/// C[47:40]
		cmm[6] = _mm_loadu_si128((__m128i const *)(piCur+48));	/// C[55:48]
		cmm[7] = _mm_loadu_si128((__m128i const *)(piCur+56));	/// C[63:56]

		xmm[0] = _mm_sub_epi16(xmm[0], cmm[0]);
		xmm[1] = _mm_sub_epi16(xmm[1], cmm[1]);
		xmm[2] = _mm_sub_epi16(xmm[2], cmm[2]);
		xmm[3] = _mm_sub_epi16(xmm[3], cmm[3]);
		xmm[4] = _mm_sub_epi16(xmm[4], cmm[4]);
		xmm[5] = _mm_sub_epi16(xmm[5], cmm[5]);
		xmm[6] = _mm_sub_epi16(xmm[6], cmm[6]);
		xmm[7] = _mm_sub_epi16(xmm[7], cmm[7]);

		xmm[0] = _mm_abs_epi16(xmm[0]);
		xmm[1] = _mm_abs_epi16(xmm[1]); 	
		xmm[2] = _mm_abs_epi16(xmm[2]);
		xmm[3] = _mm_abs_epi16(xmm[3]); 	
		xmm[4] = _mm_abs_epi16(xmm[4]);
		xmm[5] = _mm_abs_epi16(xmm[5]); 	
		xmm[6] = _mm_abs_epi16(xmm[6]);
		xmm[7] = _mm_abs_epi16(xmm[7]); 	

		xmm[0] = _mm_add_epi16(xmm[0], xmm[1]); /// 12 bit
		xmm[0] = _mm_add_epi16(xmm[0], xmm[2]); /// 13 bit
		xmm[0] = _mm_add_epi16(xmm[0], xmm[3]); /// 14 bit

		xmm[4] = _mm_add_epi16(xmm[4], xmm[5]); /// 12 bit
		xmm[4] = _mm_add_epi16(xmm[4], xmm[6]); /// 13 bit
		xmm[4] = _mm_add_epi16(xmm[4], xmm[7]); /// 14 bit

		xmm[0] = _mm_madd_epi16(xmm[0], xmmz);	/// 7+6 5+4 3+2 1+0 
		xmm[4] = _mm_madd_epi16(xmm[4], xmmz);	/// 7+6 5+4 3+2 1+0 
		xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);

		smm = _mm_add_epi32(smm, xmm[0]);

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}	

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	UInt uiSumSAD = _mm_extract_epi32(smm, 0);
	uiSumSAD <<= iSubShift;

	///------------- 8/10 bit Compatible @ 2015 8 9 by Seok -----------------------
	return uiSumSAD >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

#else  
	if ( pcDtParam->bApplyWeight )
	{
		return xGetSADw( pcDtParam );
	}

	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iSubShift  = pcDtParam->iSubShift;
	Int  iSubStep   = ( 1 << iSubShift );
	Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
	Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
	UInt uiSum = 0;

	for( ; iRows != 0; iRows-=iSubStep )
	{
		uiSum += abs( piOrg[0] - piCur[0] );
		uiSum += abs( piOrg[1] - piCur[1] );
		uiSum += abs( piOrg[2] - piCur[2] );
		uiSum += abs( piOrg[3] - piCur[3] );
		uiSum += abs( piOrg[4] - piCur[4] );
		uiSum += abs( piOrg[5] - piCur[5] );
		uiSum += abs( piOrg[6] - piCur[6] );
		uiSum += abs( piOrg[7] - piCur[7] );
		uiSum += abs( piOrg[8] - piCur[8] );
		uiSum += abs( piOrg[9] - piCur[9] );
		uiSum += abs( piOrg[10] - piCur[10] );
		uiSum += abs( piOrg[11] - piCur[11] );
		uiSum += abs( piOrg[12] - piCur[12] );
		uiSum += abs( piOrg[13] - piCur[13] );
		uiSum += abs( piOrg[14] - piCur[14] );
		uiSum += abs( piOrg[15] - piCur[15] );
		uiSum += abs( piOrg[16] - piCur[16] );
		uiSum += abs( piOrg[17] - piCur[17] );
		uiSum += abs( piOrg[18] - piCur[18] );
		uiSum += abs( piOrg[19] - piCur[19] );
		uiSum += abs( piOrg[20] - piCur[20] );
		uiSum += abs( piOrg[21] - piCur[21] );
		uiSum += abs( piOrg[22] - piCur[22] );
		uiSum += abs( piOrg[23] - piCur[23] );
		uiSum += abs( piOrg[24] - piCur[24] );
		uiSum += abs( piOrg[25] - piCur[25] );
		uiSum += abs( piOrg[26] - piCur[26] );
		uiSum += abs( piOrg[27] - piCur[27] );
		uiSum += abs( piOrg[28] - piCur[28] );
		uiSum += abs( piOrg[29] - piCur[29] );
		uiSum += abs( piOrg[30] - piCur[30] );
		uiSum += abs( piOrg[31] - piCur[31] );
		uiSum += abs( piOrg[32] - piCur[32] );
		uiSum += abs( piOrg[33] - piCur[33] );
		uiSum += abs( piOrg[34] - piCur[34] );
		uiSum += abs( piOrg[35] - piCur[35] );
		uiSum += abs( piOrg[36] - piCur[36] );
		uiSum += abs( piOrg[37] - piCur[37] );
		uiSum += abs( piOrg[38] - piCur[38] );
		uiSum += abs( piOrg[39] - piCur[39] );
		uiSum += abs( piOrg[40] - piCur[40] );
		uiSum += abs( piOrg[41] - piCur[41] );
		uiSum += abs( piOrg[42] - piCur[42] );
		uiSum += abs( piOrg[43] - piCur[43] );
		uiSum += abs( piOrg[44] - piCur[44] );
		uiSum += abs( piOrg[45] - piCur[45] );
		uiSum += abs( piOrg[46] - piCur[46] );
		uiSum += abs( piOrg[47] - piCur[47] );
		uiSum += abs(piOrg[48] - piCur[48]);
		uiSum += abs(piOrg[49] - piCur[49]);
		uiSum += abs(piOrg[50] - piCur[50]);
		uiSum += abs(piOrg[51] - piCur[51]);
		uiSum += abs(piOrg[52] - piCur[52]);
		uiSum += abs(piOrg[53] - piCur[53]);
		uiSum += abs(piOrg[54] - piCur[54]);
		uiSum += abs(piOrg[55] - piCur[55]);
		uiSum += abs(piOrg[56] - piCur[56]);
		uiSum += abs(piOrg[57] - piCur[57]);
		uiSum += abs(piOrg[58] - piCur[58]);
		uiSum += abs(piOrg[59] - piCur[59]);
		uiSum += abs(piOrg[60] - piCur[60]);
		uiSum += abs(piOrg[61] - piCur[61]);
		uiSum += abs(piOrg[62] - piCur[62]);
		uiSum += abs(piOrg[63] - piCur[63]);

		piOrg += iStrideOrg;
		piCur += iStrideCur;
}

	uiSum <<= iSubShift;

	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth - 8);

#endif // ETRI_SIMD_MD 
}

#if AMP_SAD
UInt TComRdCost::xGetSAD48( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}
#endif

// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xGetSSE( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  
  Int iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ )
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += ( iTemp * iTemp ) >> uiShift;
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

/**
====================================================================================================================
	@brief  	As to 8 bit, it is possible to omit shift operation between calculation. so that SIMD operation can be simple. \
			However, As to 10 bit with FULL_NBIT = 0 case, since uishift is not 0, SIMD operation is somewhat complicated.
====================================================================================================================
*/
UInt TComRdCost::xGetSSE4( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
#if ETRI_SSE_SIMD_OPT
    const Pel* piOrg   = pcDtParam->pOrg;
    const Pel* piCur   = pcDtParam->pCur;
    Int  iRows   = pcDtParam->iRows;
    Int  iStrideOrg = pcDtParam->iStrideOrg;
    Int  iStrideCur = pcDtParam->iStrideCur;
    UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

    __m128i xmm[4], xmm_iTemp[8];
    __m128i xmm_zero = _mm_setzero_si128();
    __m128i xmm_uiSum;

    UInt aSum[4];

    if(iRows!=4)
    {
        xmm[0] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)(piOrg+iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piCur), _mm_loadu_si128((__m128i const *)(piCur+iStrideCur))))); //x
        xmm[1] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piOrg+2*iStrideOrg)), _mm_loadu_si128((__m128i const *)(piOrg+3*iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piCur+2*iStrideCur)), _mm_loadu_si128((__m128i const *)(piCur+3*iStrideCur))))); //x
        xmm[2] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piOrg+4*iStrideOrg)), _mm_loadu_si128((__m128i const *)(piOrg+5*iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piCur+4*iStrideCur)), _mm_loadu_si128((__m128i const *)(piCur+5*iStrideCur))))); //x
        xmm[3] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piOrg+6*iStrideOrg)), _mm_loadu_si128((__m128i const *)(piOrg+7*iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piCur+6*iStrideCur)), _mm_loadu_si128((__m128i const *)(piCur+7*iStrideCur))))); //x


        //10bit Extension
        xmm_iTemp[0] =  _mm_unpacklo_epi16(xmm[0],xmm_zero);
        xmm_iTemp[1] =  _mm_unpackhi_epi16(xmm[0],xmm_zero);
        xmm_iTemp[2] =  _mm_unpacklo_epi16(xmm[1],xmm_zero);
        xmm_iTemp[3] =  _mm_unpackhi_epi16(xmm[1],xmm_zero);
        xmm_iTemp[4] =  _mm_unpacklo_epi16(xmm[2],xmm_zero);
        xmm_iTemp[5] =  _mm_unpackhi_epi16(xmm[2],xmm_zero);
        xmm_iTemp[6] =  _mm_unpacklo_epi16(xmm[3],xmm_zero);
        xmm_iTemp[7] =  _mm_unpackhi_epi16(xmm[3],xmm_zero);


        xmm_iTemp[0] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[0], xmm_iTemp[0]), uiShift);
        xmm_iTemp[1] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[1], xmm_iTemp[1]), uiShift);
        xmm_iTemp[2] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[2], xmm_iTemp[2]), uiShift);
        xmm_iTemp[3] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[3], xmm_iTemp[3]), uiShift);
        xmm_iTemp[4] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[4], xmm_iTemp[4]), uiShift);
        xmm_iTemp[5] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[5], xmm_iTemp[5]), uiShift);
        xmm_iTemp[6] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[6], xmm_iTemp[6]), uiShift);
        xmm_iTemp[7] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[7], xmm_iTemp[7]), uiShift);


        xmm_uiSum = _mm_add_epi32(xmm_iTemp[0], xmm_iTemp[1]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[2]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[3]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[4]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[5]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[6]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[7]);

        xmm_uiSum = _mm_hadd_epi32(xmm_uiSum, xmm_zero);
        xmm_uiSum = _mm_hadd_epi32(xmm_uiSum, xmm_zero);
        _mm_storeu_si128((__m128i*)aSum, xmm_uiSum);

        return (aSum[0]);
    }
    else
    {

        xmm[0] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piCur), _mm_loadu_si128((__m128i const *)(piCur + iStrideCur))))); //x
        xmm[1] = _mm_abs_epi16(_mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piOrg + 2 * iStrideOrg)), _mm_loadu_si128((__m128i const *)(piOrg + 3 * iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piCur + 2 * iStrideCur)), _mm_loadu_si128((__m128i const *)(piCur + 3 * iStrideCur))))); //x

        //10bit Extension
        xmm_iTemp[0] = _mm_unpacklo_epi16(xmm[0], xmm_zero);
        xmm_iTemp[1] = _mm_unpackhi_epi16(xmm[0], xmm_zero);
        xmm_iTemp[2] = _mm_unpacklo_epi16(xmm[1], xmm_zero);
        xmm_iTemp[3] = _mm_unpackhi_epi16(xmm[1], xmm_zero);

        xmm_iTemp[0] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[0], xmm_iTemp[0]), uiShift);
        xmm_iTemp[1] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[1], xmm_iTemp[1]), uiShift);
        xmm_iTemp[2] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[2], xmm_iTemp[2]), uiShift);
        xmm_iTemp[3] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[3], xmm_iTemp[3]), uiShift);


        xmm_uiSum = _mm_add_epi32(xmm_iTemp[0], xmm_iTemp[1]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[2]);
        xmm_uiSum = _mm_add_epi32(xmm_uiSum, xmm_iTemp[3]);

        xmm_uiSum = _mm_hadd_epi32(xmm_uiSum, xmm_zero);
        xmm_uiSum = _mm_hadd_epi32(xmm_uiSum, xmm_zero);

        _mm_storeu_si128((__m128i*)aSum, xmm_uiSum);

        return (aSum[0]);
    }
#else 
	if (pcDtParam->bApplyWeight)
	{
		assert(pcDtParam->iCols == 4);
		return xGetSSEw(pcDtParam);
	}

	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;

	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);	///< 2015 8 9 by Seok : Fix 8/10 bit

	__m128i xmm[4];

	xmm[0] = _mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)piCur), _mm_loadu_si128((__m128i const *)(piCur + iStrideCur))));
	xmm[1] = _mm_sub_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piOrg + 2 * iStrideOrg)), _mm_loadu_si128((__m128i const *)(piOrg + 3 * iStrideOrg))), _mm_unpacklo_epi16(_mm_loadu_si128((__m128i const *)(piCur + 2 * iStrideCur)), _mm_loadu_si128((__m128i const *)(piCur + 3 * iStrideCur))));

	xmm[2] = _mm_mullo_epi16(xmm[0], xmm[0]);		/// L7 L6 L5 L4 L3 L2 L1 L0
	xmm[3] = _mm_mulhi_epi16(xmm[0], xmm[0]);		/// H7 H6 H5 H4 H3 H2 H1 H0
	xmm[0] = _mm_unpacklo_epi16(xmm[2], xmm[3]);	/// H3 L3 H2 L2 H1 L1 H0 L0
	xmm[2] = _mm_unpackhi_epi16(xmm[2], xmm[3]);	/// H7 L7 H6 L6 H5 L5 H4 L4
	xmm[0] = _mm_srai_epi32(xmm[0], uiShift);
	xmm[2] = _mm_srai_epi32(xmm[2], uiShift);
	xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);		/// x7+x3 x6+x2 x5+x1 x4+x0

	xmm[2] = _mm_mullo_epi16(xmm[1], xmm[1]);		/// L15 L14 L13 L12 L11 L10 L9 L8
	xmm[3] = _mm_mulhi_epi16(xmm[1], xmm[1]);		/// H15 H14 H13 H12 H11 H10 H9 H8
	xmm[1] = _mm_unpacklo_epi16(xmm[2], xmm[3]);	/// H11 L11 H10 L10 H9 L9 H8 L8
	xmm[2] = _mm_unpackhi_epi16(xmm[2], xmm[3]);	/// H15 L15 H14 L14 H13 L13 H12 L12
	xmm[1] = _mm_srai_epi32(xmm[1], uiShift);
	xmm[2] = _mm_srai_epi32(xmm[2], uiShift);
	xmm[1] = _mm_add_epi32(xmm[1], xmm[2]);		/// x15+x11 x14+x10 x13+x9 x12+x8

	xmm[0] = _mm_add_epi32(xmm[0], xmm[1]);		/// (15 11 7 3)(14 10 6 2)(13 9 5 1)(12 8 4 0)
	xmm[0] = _mm_hadd_epi32(xmm[0], xmm[0]);		/// (15 11 7 3 14 10 6 2)(13 9 5 1 12 8 4 0)
	xmm[0] = _mm_hadd_epi32(xmm[0], xmm[0]);		/// (15 11 7 3 14 10 6 2 13 9 5 1 12 8 4 0)

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);
	return (est[0]);
#endif 
#else
	if ( pcDtParam->bApplyWeight )
	{
		assert( pcDtParam->iCols == 4 );
		return xGetSSEw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;

	UInt uiSum = 0;
	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

	Int  iTemp;

	for( ; iRows != 0; iRows-- )
	{

		iTemp = piOrg[0] - piCur[0]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[1] - piCur[1]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[2] - piCur[2]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[3] - piCur[3]; uiSum += ( iTemp * iTemp ) >> uiShift;

		piOrg += iStrideOrg;
		piCur += iStrideCur;
}

	return (uiSum);
#endif // ETRI_SIMD_MD 
}

// ====================================================================================================================
//   ETRI  Modification for E265 Version 1 :: Macros to ETRI_SIMD_MD for SSE static functions
//
// ====================================================================================================================

#if ETRI_SIMD_MD
#define	ETRI_SSE_BITPRECISION(k, uiShift, xmm, tmm) \
	tmm[0] = _mm_mullo_epi16(xmm[k], xmm[k]); \
	tmm[1] = _mm_mulhi_epi16(xmm[k], xmm[k]); \
	xmm[k] = _mm_unpacklo_epi16(tmm[0], tmm[1]); \
	tmm[0] = _mm_unpackhi_epi16(tmm[0], tmm[1]); \
	tmm[0] = _mm_srai_epi32(tmm[0], uiShift); \
	xmm[k] = _mm_srai_epi32(xmm[k], uiShift); \
	xmm[k] = _mm_add_epi32(xmm[k], tmm[0]); 


#define 	ETRI_SIMD_SUB8_EPI16(xmm, tmm)\
	xmm[0] = _mm_sub_epi16(xmm[0], tmm[0]); \
	xmm[1] = _mm_sub_epi16(xmm[1], tmm[1]); \
	xmm[2] = _mm_sub_epi16(xmm[2], tmm[2]); \
	xmm[3] = _mm_sub_epi16(xmm[3], tmm[3]); \
	xmm[4] = _mm_sub_epi16(xmm[4], tmm[4]); \
	xmm[5] = _mm_sub_epi16(xmm[5], tmm[5]); \
	xmm[6] = _mm_sub_epi16(xmm[6], tmm[6]); \
	xmm[7] = _mm_sub_epi16(xmm[7], tmm[7]);


#define	ETRI_SSE_LOAD64(piOrg, iStrideOrg, xmm)\
	xmm[0] = _mm_load_si128((__m128i const *)piOrg); \
	xmm[1] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[2] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[3] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[4] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[5] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[6] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[7] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg));

#define	ETRI_SSE_LOADU64(piOrg, iStrideOrg, xmm)\
	xmm[0] = _mm_loadu_si128((__m128i const *)piOrg); \
	xmm[1] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[2] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[3] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[4] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[5] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[6] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[7] = _mm_loadu_si128((__m128i const *)(piOrg += iStrideOrg));


#define	ETRI_SSE8_LOAD64(piOrg, iStrideOrg, iStartoffset, xmm)\
	xmm[0] = _mm_load_si128((__m128i const *)(piOrg += iStartoffset)); \
	xmm[1] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[2] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[3] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[4] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[5] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[6] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[7] = _mm_load_si128((__m128i const *)(piOrg + 8));


#define	ETRI_SSE32_LOAD64(piOrg, iStrideOrg, iStartoffset, xmm)\
	xmm[0] = _mm_load_si128((__m128i const *)(piOrg += iStartoffset)); \
	xmm[1] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[2] = _mm_load_si128((__m128i const *)(piOrg + 16)); \
	xmm[3] = _mm_load_si128((__m128i const *)(piOrg + 24)); \
	xmm[4] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[5] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[6] = _mm_load_si128((__m128i const *)(piOrg + 16)); \
	xmm[7] = _mm_load_si128((__m128i const *)(piOrg + 24));


#define	ETRI_SSE64_LOAD64(piOrg, iStrideOrg, xmm)\
	xmm[0] = _mm_load_si128((__m128i const *)(piOrg += iStrideOrg)); \
	xmm[1] = _mm_load_si128((__m128i const *)(piOrg + 8)); \
	xmm[2] = _mm_load_si128((__m128i const *)(piOrg + 16)); \
	xmm[3] = _mm_load_si128((__m128i const *)(piOrg + 24)); \
	xmm[4] = _mm_load_si128((__m128i const *)(piOrg + 32)); \
	xmm[5] = _mm_load_si128((__m128i const *)(piOrg + 40)); \
	xmm[6] = _mm_load_si128((__m128i const *)(piOrg + 48)); \
	xmm[7] = _mm_load_si128((__m128i const *)(piOrg + 56));


#define	ETRI_SIMD_SUM8(smm, xmm)\
	smm = _mm_add_epi32(smm, xmm[0]); \
	smm = _mm_add_epi32(smm, xmm[1]); \
	smm = _mm_add_epi32(smm, xmm[2]); \
	smm = _mm_add_epi32(smm, xmm[3]); \
	smm = _mm_add_epi32(smm, xmm[4]); \
	smm = _mm_add_epi32(smm, xmm[5]); \
	smm = _mm_add_epi32(smm, xmm[6]); \
	smm = _mm_add_epi32(smm, xmm[7]); 

#endif

UInt TComRdCost::xGetSSE8( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
#if ETRI_SSE_SIMD_OPT
    const Pel* piOrg   = pcDtParam->pOrg;
    const Pel* piCur   = pcDtParam->pCur;
    Int  iRows   = pcDtParam->iRows;
    Int  iStrideOrg = pcDtParam->iStrideOrg;
    Int  iStrideCur = pcDtParam->iStrideCur;

    UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);


    __m128i sum  = _mm_setzero_si128();
    __m128i tmp1;

    __m128i xmm_org, xmm_cur;
    __m128i xmm_iTemp[2];
    __m128i xmm_zero = _mm_setzero_si128();

    for (; iRows != 0; iRows--)
    {
        xmm_org = _mm_loadu_si128((__m128i const*)(piOrg )); 
        xmm_cur = _mm_loadu_si128((__m128i const*)(piCur )); 
        tmp1 = _mm_abs_epi16(_mm_sub_epi16(xmm_org, xmm_cur));

        xmm_iTemp[0] =  _mm_unpacklo_epi16(tmp1,xmm_zero);
        xmm_iTemp[1] =  _mm_unpackhi_epi16(tmp1,xmm_zero);
        xmm_iTemp[0] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[0], xmm_iTemp[0]), uiShift);
        xmm_iTemp[1] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[1], xmm_iTemp[1]), uiShift);

        sum = _mm_add_epi32(sum,xmm_iTemp[0]);
        sum = _mm_add_epi32(sum,xmm_iTemp[1]);

        piOrg += iStrideOrg;
        piCur += iStrideCur;
    }

    __m128i sum1 = _mm_hadd_epi32(sum, sum);

    return _mm_cvtsi128_si32(_mm_hadd_epi32(sum1, sum1));
#else 
	if (pcDtParam->bApplyWeight)
	{
		assert(pcDtParam->iCols == 8);
		return xGetSSEw(pcDtParam);
	}

	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;

	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);	///< 2015 8 9 by Seok : Fix 8/10 bit

	__m128i xmm[8], tmm[8];

	ETRI_SSE_LOAD64(piOrg, iStrideOrg, xmm);
	ETRI_SSE_LOAD64(piCur,  iStrideCur, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm, tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);
	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);

	xmm[0] = _mm_add_epi32(xmm[0], xmm[1]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[2]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[3]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[4]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[5]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[6]); 	
	xmm[0] = _mm_add_epi32(xmm[0], xmm[7]); 	
	xmm[0] = _mm_hadd_epi32(xmm[0], xmm[0]);
	xmm[0] = _mm_hadd_epi32(xmm[0], xmm[0]);

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, xmm[0]);
	return (est[0]);
#endif 
#else  
	if ( pcDtParam->bApplyWeight )
	{
		assert( pcDtParam->iCols == 8 );
		return xGetSSEw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;

	UInt uiSum = 0;
	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

	Int  iTemp;

	for( ; iRows != 0; iRows-- )
	{
		iTemp = piOrg[0] - piCur[0]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[1] - piCur[1]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[2] - piCur[2]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[3] - piCur[3]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[4] - piCur[4]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[5] - piCur[5]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[6] - piCur[6]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[7] - piCur[7]; uiSum += ( iTemp * iTemp ) >> uiShift;

		piOrg += iStrideOrg;
		piCur += iStrideCur;
}

	return (uiSum);
#endif // ETRI_SIMD_MD 
}

UInt TComRdCost::xGetSSE16( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
#if ETRI_SSE_SIMD_OPT
    Int  iStrideOrg = pcDtParam->iStrideOrg;
    Int  iStrideCur = pcDtParam->iStrideCur;
    Pel* piOrg    = pcDtParam->pOrg;
    Pel* piCur    = pcDtParam->pCur; 
    Int  iRows   = pcDtParam->iRows;  // row 16, 32
    UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);


    __m128i sum  = _mm_setzero_si128();

    __m128i tmp1;

    __m128i xmm_org, xmm_cur;
    __m128i xmm_iTemp[2];
    __m128i xmm_zero = _mm_setzero_si128();


    for (; iRows != 0; iRows--)
    {
        for (int i = 0; i < 16; i += 8)
        {
            xmm_org = _mm_loadu_si128((__m128i const*)(piOrg + i)); 
            xmm_cur = _mm_loadu_si128((__m128i const*)(piCur + i)); 
            tmp1 = _mm_abs_epi16(_mm_sub_epi16(xmm_org, xmm_cur));

            xmm_iTemp[0] =  _mm_unpacklo_epi16(tmp1,xmm_zero);
            xmm_iTemp[1] =  _mm_unpackhi_epi16(tmp1,xmm_zero);
            xmm_iTemp[0] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[0], xmm_iTemp[0]), uiShift);
            xmm_iTemp[1] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[1], xmm_iTemp[1]), uiShift);

            sum = _mm_add_epi32(sum,xmm_iTemp[0]);
            sum = _mm_add_epi32(sum,xmm_iTemp[1]);
        }
        piOrg += iStrideOrg;
        piCur += iStrideCur;
    }

    __m128i sum1  = _mm_hadd_epi32(sum, sum);

    return _mm_cvtsi128_si32(_mm_hadd_epi32(sum1, sum1));
#else 
	if (pcDtParam->bApplyWeight)
	{
		assert(pcDtParam->iCols == 16);
		return xGetSSEw(pcDtParam);
	}

	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;

	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);	///< 2015 8 9 by Seok : Fix 8/10 bit

	__m128i xmm[8], tmm[8], smm;

	smm = _mm_setzero_si128();
	
	///< 0: 3 @ 2015 8 10 by Seok
	ETRI_SSE8_LOAD64(piOrg, iStrideOrg, 0, xmm);
	ETRI_SSE8_LOAD64(piCur, iStrideCur, 0, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	

	ETRI_SIMD_SUM8(smm,xmm);

	///< 4: 7 @ 2015 8 10 by Seok
	ETRI_SSE8_LOAD64(piOrg, iStrideOrg, iStrideOrg, xmm);
	ETRI_SSE8_LOAD64(piCur, iStrideCur, iStrideCur, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	

	ETRI_SIMD_SUM8(smm,xmm);

	///< 8: 11 @ 2015 8 10 by Seok
	ETRI_SSE8_LOAD64(piOrg, iStrideOrg, iStrideOrg, xmm);
	ETRI_SSE8_LOAD64(piCur, iStrideCur, iStrideCur, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	

	ETRI_SIMD_SUM8(smm,xmm);

	///< 12: 15 @ 2015 8 10 by Seok
	ETRI_SSE8_LOAD64(piOrg, iStrideOrg, iStrideOrg, xmm);
	ETRI_SSE8_LOAD64(piCur, iStrideCur, iStrideCur, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	

	ETRI_SIMD_SUM8(smm,xmm);

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, smm);
	return (est[0]);
#endif 
#else  
	if ( pcDtParam->bApplyWeight )
	{
		assert( pcDtParam->iCols == 16 );
		return xGetSSEw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;

	UInt uiSum = 0;
	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

	Int  iTemp;

	for( ; iRows != 0; iRows-- )
	{

		iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	return ( uiSum );
#endif // ETRI_SIMD_MD 
}

UInt TComRdCost::xGetSSE16N( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n+=16 )
    {
      
      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += ( iTemp * iTemp ) >> uiShift;
      
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}


UInt TComRdCost::xGetSSE32( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
#if ETRI_SSE_SIMD_OPT
    const Pel* piOrg   = pcDtParam->pOrg;
    const Pel* piCur   = pcDtParam->pCur;

    Int  iStrideOrg = pcDtParam->iStrideOrg;
    Int  iStrideCur = pcDtParam->iStrideCur;


    UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);


    __m128i sum  = _mm_setzero_si128();

    __m128i tmp1;

    __m128i xmm_org, xmm_cur;
    __m128i xmm_iTemp[2];
    __m128i xmm_zero = _mm_setzero_si128();
    Int  iRows   = pcDtParam->iRows;

    for (; iRows != 0; iRows--)
    {
        for (int i = 0; i < 32; i += 8)
        {
            xmm_org = _mm_loadu_si128((__m128i const*)(piOrg + i)); 
            xmm_cur = _mm_loadu_si128((__m128i const*)(piCur + i)); 
            tmp1 = _mm_abs_epi16(_mm_sub_epi16(xmm_org, xmm_cur));

            xmm_iTemp[0] =  _mm_unpacklo_epi16(tmp1,xmm_zero);
            xmm_iTemp[1] =  _mm_unpackhi_epi16(tmp1,xmm_zero);
            xmm_iTemp[0] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[0], xmm_iTemp[0]), uiShift);
            xmm_iTemp[1] = _mm_srai_epi32(_mm_mullo_epi32(xmm_iTemp[1], xmm_iTemp[1]), uiShift);

            sum = _mm_add_epi32(sum,xmm_iTemp[0]);
            sum = _mm_add_epi32(sum,xmm_iTemp[1]);
        }
        piOrg += iStrideOrg;
        piCur += iStrideCur;
    }

    __m128i sum1 = _mm_hadd_epi32(sum, sum);

    return _mm_cvtsi128_si32(_mm_hadd_epi32(sum1, sum1));
#else 
	if (pcDtParam->bApplyWeight)
	{
		assert(pcDtParam->iCols == 32);
		return xGetSSEw(pcDtParam);
	}

	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;

	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);	///< 2015 8 9 by Seok : Fix 8/10 bit

	__m128i xmm[8], tmm[8], smm;

	smm = _mm_setzero_si128();

	///< 0: 1 @ 2015 8 10 by Seok
	ETRI_SSE32_LOAD64(piOrg, iStrideOrg, 0, xmm);
	ETRI_SSE32_LOAD64(piCur, iStrideCur, 0, tmm);
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);

	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	

	ETRI_SIMD_SUM8(smm,xmm);

	for(UInt k=2; k<32; k+=2)
	{
		ETRI_SSE32_LOAD64(piOrg, iStrideOrg, iStrideOrg, xmm);
		ETRI_SSE32_LOAD64(piCur, iStrideCur, iStrideCur, tmm);
		ETRI_SIMD_SUB8_EPI16(xmm,tmm);
		
		ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	
		
		ETRI_SIMD_SUM8(smm,xmm);
	}

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, smm);
	return (est[0]);
#endif 
#else
	if ( pcDtParam->bApplyWeight )
	{
		assert( pcDtParam->iCols == 32 );
		return xGetSSEw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;

	UInt uiSum = 0;
	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
	Int  iTemp;

	for( ; iRows != 0; iRows-- )
	{

		iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[16] - piCur[16]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[17] - piCur[17]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[18] - piCur[18]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[19] - piCur[19]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[20] - piCur[20]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[21] - piCur[21]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[22] - piCur[22]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[23] - piCur[23]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[24] - piCur[24]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[25] - piCur[25]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[26] - piCur[26]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[27] - piCur[27]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[28] - piCur[28]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[29] - piCur[29]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[30] - piCur[30]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[31] - piCur[31]; uiSum += ( iTemp * iTemp ) >> uiShift;

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	return ( uiSum );
#endif // ETRI_SIMD_MD 
}

UInt TComRdCost::xGetSSE64( DistParam* pcDtParam )
{
#if ETRI_SIMD_MD
	if (pcDtParam->bApplyWeight)
	{
		assert(pcDtParam->iCols == 32);
		return xGetSSEw(pcDtParam);
	}

	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Pel* piOrg = pcDtParam->pOrg;
	Pel* piCur = pcDtParam->pCur;

	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);	///< 2015 8 9 by Seok : Fix 8/10 bit

	__m128i xmm[8], tmm[8], smm;

	smm = _mm_setzero_si128();

	ETRI_SSE64_LOAD64(piOrg, 0, xmm);
	ETRI_SSE64_LOAD64(piCur, 0, tmm);
	
	ETRI_SIMD_SUB8_EPI16(xmm,tmm);
	
	ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
	ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	
	
	ETRI_SIMD_SUM8(smm,xmm);

	for(UInt k=1; k<64; k++)
	{
		ETRI_SSE64_LOAD64(piOrg, iStrideOrg, xmm);
		ETRI_SSE64_LOAD64(piCur, iStrideCur, tmm);

		ETRI_SIMD_SUB8_EPI16(xmm,tmm);
		
		ETRI_SSE_BITPRECISION(0, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(1, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(2, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(3, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(4, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(5, uiShift, xmm, tmm);	
		ETRI_SSE_BITPRECISION(6, uiShift, xmm, tmm);	ETRI_SSE_BITPRECISION(7, uiShift, xmm, tmm);	
		
		ETRI_SIMD_SUM8(smm,xmm);
	}

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	ALIGNED(16) UInt est[4];
	_mm_store_si128((__m128i*)est, smm);
	return (est[0]);

#else

	if ( pcDtParam->bApplyWeight )
	{
		assert( pcDtParam->iCols == 64 );
		return xGetSSEw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStrideCur = pcDtParam->iStrideCur;

	UInt uiSum = 0;
	UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
	Int  iTemp;

	for( ; iRows != 0; iRows-- )
	{
		iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[16] - piCur[16]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[17] - piCur[17]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[18] - piCur[18]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[19] - piCur[19]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[20] - piCur[20]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[21] - piCur[21]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[22] - piCur[22]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[23] - piCur[23]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[24] - piCur[24]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[25] - piCur[25]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[26] - piCur[26]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[27] - piCur[27]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[28] - piCur[28]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[29] - piCur[29]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[30] - piCur[30]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[31] - piCur[31]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[32] - piCur[32]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[33] - piCur[33]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[34] - piCur[34]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[35] - piCur[35]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[36] - piCur[36]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[37] - piCur[37]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[38] - piCur[38]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[39] - piCur[39]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[40] - piCur[40]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[41] - piCur[41]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[42] - piCur[42]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[43] - piCur[43]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[44] - piCur[44]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[45] - piCur[45]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[46] - piCur[46]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[47] - piCur[47]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[48] - piCur[48]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[49] - piCur[49]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[50] - piCur[50]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[51] - piCur[51]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[52] - piCur[52]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[53] - piCur[53]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[54] - piCur[54]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[55] - piCur[55]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[56] - piCur[56]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[57] - piCur[57]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[58] - piCur[58]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[59] - piCur[59]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[60] - piCur[60]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[61] - piCur[61]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[62] - piCur[62]; uiSum += ( iTemp * iTemp ) >> uiShift;
		iTemp = piOrg[63] - piCur[63]; uiSum += ( iTemp * iTemp ) >> uiShift;

		piOrg += iStrideOrg;
		piCur += iStrideCur;
	}

	return ( uiSum );
#endif // ETRI_SIMD_MD 
}

// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xCalcHADs2x2( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep, UInt& uiDCValue )
{
	Int satd = 0, diff[4], m[4];
	assert( iStep == 1 );

	diff[0] = piOrg[0             ] - piCur[0];
	diff[1] = piOrg[1             ] - piCur[1];
	diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
	diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
	m[0] = diff[0] + diff[2];
	m[1] = diff[1] + diff[3];
	m[2] = diff[0] - diff[2];
	m[3] = diff[1] - diff[3];

	satd += (uiDCValue = abs(m[0] + m[1]));
	satd += abs(m[0] - m[1]);
	satd += abs(m[2] + m[3]);
	satd += abs(m[2] - m[3]);

	return satd;
}

UInt TComRdCost::xCalcHADs4x4( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep, UInt& uiDCValue )
{
#if ETRI_SIMD_MD
	__m128i xmm[3]; Int satd = 0;

	xmm[0] = _mm_sub_epi16(_mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)piOrg), _mm_loadu_si128((__m128i const *)(piOrg + iStrideOrg))), _mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)piCur), _mm_loadu_si128((__m128i const *)(piCur + iStrideCur))));
	xmm[1] = _mm_shuffle_epi32(_mm_sub_epi16(_mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)(piOrg + (iStrideOrg << 1))), _mm_loadu_si128((__m128i const *)(piOrg + (iStrideOrg << 1) + iStrideOrg))), _mm_unpacklo_epi64(_mm_loadu_si128((__m128i const *)(piCur + (iStrideCur << 1))), _mm_loadu_si128((__m128i const *)(piCur + (iStrideCur << 1) + iStrideCur)))), 78); // shuffle 01001110b xmm[2] = d[11] d[10] d[9] d[8] d[15] d[14] d[13] d[12]
	xmm[2] = _mm_add_epi16(xmm[0], xmm[1]); //xmm[1] = d7 d6 d5 d4 d3 d2 d1 d0
	xmm[0] = _mm_shuffle_epi32(_mm_sub_epi16(xmm[0], xmm[1]), 78); // shuffle 01001110b xmm[2] = d15 d14 d13 d12 d11 d10 d9 d8
	xmm[1] = _mm_add_epi16(_mm_unpacklo_epi16(xmm[2], xmm[0]), _mm_unpackhi_epi16(xmm[2], xmm[0])); // xmm[0] =  d7  d3   d6  d2   d5  d1  d4  d0 // 11 bits
	xmm[0] = _mm_sub_epi16(_mm_unpacklo_epi16(xmm[2], xmm[0]), _mm_unpackhi_epi16(xmm[2], xmm[0])); // xmm[1] = -d15 d11 -d14 d10 -d13 d9 -d12 d8 뒤에 4개 부호 바뀌어도 최종 결과는 동일
	xmm[2] = _mm_add_epi16(_mm_unpacklo_epi16(xmm[1], xmm[0]), _mm_shuffle_epi32(_mm_unpackhi_epi16(xmm[1], xmm[0]), 78)); // xmm[0] = -m13 m5 m9  m1 -m12 m4 m8  m0
	xmm[0] = _mm_sub_epi16(_mm_unpacklo_epi16(xmm[1], xmm[0]), _mm_shuffle_epi32(_mm_unpackhi_epi16(xmm[1], xmm[0]), 78)); // xmm[1] = -m14 m6 m10 m2 -m15 m7 m11 m3 // 12 bits
	xmm[1] = _mm_add_epi16(_mm_unpacklo_epi32(xmm[2], xmm[0]), _mm_unpackhi_epi32(xmm[2], xmm[0])); //xmm[0] = -d14 d6 -d12 d4 d10 d2 d8 d0
	xmm[0] = _mm_sub_epi16(_mm_unpacklo_epi32(xmm[2], xmm[0]), _mm_unpackhi_epi32(xmm[2], xmm[0])); //xmm[1] = -d15 d7 -d13 d5 d11 d3 d9 d1 // 13 bits

	//SAD 구하는 부분
	xmm[1] = _mm_abs_epi16(xmm[1]);
	xmm[0] = _mm_abs_epi16(xmm[0]);
	xmm[0] = _mm_hadd_epi16(xmm[0], xmm[1]); // xmm[0] = xmm[5]+xmm[6] a7 a6 a5 a4 a3 a2 a1 a0 // 14 bits
	xmm[0] = _mm_hadd_epi16(xmm[0], xmm[0]);
	xmm[0] = _mm_hadd_epi16(xmm[0], xmm[0]);
	xmm[0] = _mm_hadd_epi16(xmm[0], xmm[0]);
	
	uiDCValue = (_mm_extract_epi16(xmm[1], 0) + 1) >> 1;
	satd = (_mm_extract_epi16(xmm[0], 0) + 1) >> 1;
	return satd;

#else
	Int k, satd = 0, diff[16], m[16], d[16];

	assert( iStep == 1 );
	for( k = 0; k < 16; k+=4 )
	{
		diff[k+0] = piOrg[0] - piCur[0];
		diff[k+1] = piOrg[1] - piCur[1];
		diff[k+2] = piOrg[2] - piCur[2];
		diff[k+3] = piOrg[3] - piCur[3];

		piCur += iStrideCur;
		piOrg += iStrideOrg;
	}

	/*===== hadamard transform =====*/
	m[ 0] = diff[ 0] + diff[12];
	m[ 1] = diff[ 1] + diff[13];
	m[ 2] = diff[ 2] + diff[14];
	m[ 3] = diff[ 3] + diff[15];
	m[4] = diff[4] + diff[8];
	m[5] = diff[5] + diff[9];
	m[6] = diff[6] + diff[10];
	m[7] = diff[7] + diff[11];
	m[8] = diff[4] - diff[8];
	m[9] = diff[5] - diff[9];
	m[10] = diff[6] - diff[10];
	m[11] = diff[7] - diff[11];
	m[12] = diff[0] - diff[12];
	m[13] = diff[1] - diff[13];
	m[14] = diff[2] - diff[14];
	m[15] = diff[3] - diff[15];

	d[0] = m[0] + m[4];
	d[1] = m[1] + m[5];
	d[2] = m[2] + m[6];
	d[3] = m[3] + m[7];
	d[4] = m[8] + m[12];
	d[5] = m[9] + m[13];
	d[6] = m[10] + m[14];
	d[7] = m[11] + m[15];
	d[8] = m[0] - m[4];
	d[9] = m[1] - m[5];
	d[10] = m[2] - m[6];
	d[11] = m[3] - m[7];
	d[12] = m[12] - m[8];
	d[13] = m[13] - m[9];
	d[14] = m[14] - m[10];
	d[15] = m[15] - m[11];

	m[0] = d[0] + d[3];
	m[1] = d[1] + d[2];
	m[2] = d[1] - d[2];
	m[3] = d[0] - d[3];
	m[4] = d[4] + d[7];
	m[5] = d[5] + d[6];
	m[6] = d[5] - d[6];
	m[7] = d[4] - d[7];
	m[8] = d[8] + d[11];
	m[9] = d[9] + d[10];
	m[10] = d[9] - d[10];
	m[11] = d[8] - d[11];
	m[12] = d[12] + d[15];
	m[13] = d[13] + d[14];
	m[14] = d[13] - d[14];
	m[15] = d[12] - d[15];

	d[0] = m[0] + m[1];
	d[1] = m[0] - m[1];
	d[2] = m[2] + m[3];
	d[3] = m[3] - m[2];
	d[4] = m[4] + m[5];
	d[5] = m[4] - m[5];
	d[6] = m[6] + m[7];
	d[7] = m[7] - m[6];
	d[8] = m[8] + m[9];
	d[9] = m[8] - m[9];
	d[10] = m[10] + m[11];
	d[11] = m[11] - m[10];
	d[12] = m[12] + m[13];
	d[13] = m[12] - m[13];
	d[14] = m[14] + m[15];
	d[15] = m[15] - m[14];

	for (k = 0; k<16; ++k)
	{
		satd += abs(d[k]);
	}
	satd = ((satd + 1) >> 1);

	return satd;
#endif // #if ETRI_SIMD_MD
}




UInt TComRdCost::xCalcHADs8x8( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep, UInt& uiDCValue )
{
#if ETRI_SIMD_MD

	__m128i xmm[8], cmm[8];
	__m128i	pmm, Mmm, smm, dmm;
	ALIGNED(16) Short _sParam[8] = {1, -1, 1, -1, 1, -1, 1, -1}; 

	Mmm = _mm_load_si128((__m128i const *)_sParam);

	ETRI_SSE_LOAD64(piOrg, iStrideOrg, xmm);
	ETRI_SSE_LOADU64(piCur, iStrideCur, cmm);
	ETRI_SIMD_SUB8_EPI16(xmm, cmm);			/// diff(xmm) : 7 6 5 4 3 2 1 0

	cmm[0] = _mm_unpacklo_epi64(xmm[0], xmm[1]);	/// 11 10  9  8  3 2 1 0
	cmm[1] = _mm_unpackhi_epi64(xmm[0], xmm[1]);	/// 15 14 13 12 7 6 5 4
	cmm[2] = _mm_unpacklo_epi64(xmm[2], xmm[3]);	/// 11 10  9  8  3 2 1 0   [Base 16]
	cmm[3] = _mm_unpackhi_epi64(xmm[2], xmm[3]);	/// 15 14 13 12 7 6 5 4   [Base 16]
	cmm[4] = _mm_unpacklo_epi64(xmm[4], xmm[5]);	/// 11 10  9  8  3 2 1 0   [Base 32]
	cmm[5] = _mm_unpackhi_epi64(xmm[4], xmm[5]);	/// 15 14 13 12 7 6 5 4   [Base 32]
	cmm[6] = _mm_unpacklo_epi64(xmm[6], xmm[7]);	/// 11 10  9  8  3 2 1 0   [Base 48]
	cmm[7] = _mm_unpackhi_epi64(xmm[6], xmm[7]);	/// 11 10  9  8  3 2 1 0   [Base 48]

	//------------- Horizaontal M2 m2[j][0] = diff[jj  ] + diff[jj+4]; ----------

	xmm[0] = _mm_add_epi16(cmm[0], cmm[1]);	/// m2[1][3:0]  m2[0][3:0] 
	xmm[1] = _mm_add_epi16(cmm[2], cmm[3]);	/// m2[3][3:0]  m2[2][3:0]     
	xmm[2] = _mm_add_epi16(cmm[4], cmm[5]);	/// m2[5][3:0]  m2[4][3:0]     
	xmm[3] = _mm_add_epi16(cmm[6], cmm[7]);	/// m2[7][3:0]  m2[6][3:0]     
	xmm[4] = _mm_sub_epi16(cmm[0], cmm[1]);	/// m2[1][7:4]  m2[0][7:4]
	xmm[5] = _mm_sub_epi16(cmm[2], cmm[3]);	/// m2[3][7:4]  m2[2][7:4]
	xmm[6] = _mm_sub_epi16(cmm[4], cmm[5]);	/// m2[5][7:4]  m2[4][7:4]
	xmm[7] = _mm_sub_epi16(cmm[6], cmm[7]);	/// m2[7][7:4]  m2[6][7:4]

	cmm[0] = _mm_unpacklo_epi32(xmm[0], xmm[4]);	///  m2[0]:[7:6] m2[0][3:2] m2[0]:[5:4] m2[0][1:0]
	cmm[1] = _mm_unpackhi_epi32(xmm[0], xmm[4]);	///  m2[1]:[7:6] m2[1][3:2] m2[1]:[5:4] m2[1][1:0]
	cmm[2] = _mm_unpacklo_epi32(xmm[1], xmm[5]);	///	m2[2]:[7:6] m2[2][3:2] m2[2]:[5:4] m2[2][1:0]	
	cmm[3] = _mm_unpackhi_epi32(xmm[1], xmm[5]);	///	m2[3]:[7:6] m2[3][3:2] m2[3]:[5:4] m2[3][1:0]	
	cmm[4] = _mm_unpacklo_epi32(xmm[2], xmm[6]);	///	m2[4]:[7:6] m2[4][3:2] m2[4]:[5:4] m2[4][1:0]	
	cmm[5] = _mm_unpackhi_epi32(xmm[2], xmm[6]);	///	m2[5]:[7:6] m2[5][3:2] m2[5]:[5:4] m2[5][1:0]	
	cmm[6] = _mm_unpacklo_epi32(xmm[3], xmm[7]);	///	m2[6]:[7:6] m2[6][3:2] m2[6]:[5:4] m2[6][1:0]	
	cmm[7] = _mm_unpackhi_epi32(xmm[3], xmm[7]);	///	m2[7]:[7:6] m2[7][3:2] m2[7]:[5:4] m2[7][1:0]

	xmm[0] = _mm_unpacklo_epi64(cmm[0], cmm[1]);	///  m2[1]:[5:4] m2[1][1:0] m2[0]:[5:4] m2[0][1:0]
	xmm[1] = _mm_unpackhi_epi64(cmm[0], cmm[1]);	///  m2[1]:[7:6] m2[1][3:2] m2[0]:[7:6] m2[0][3:2]
	xmm[2] = _mm_unpacklo_epi64(cmm[2], cmm[3]);	///  m2[3]:[5:4] m2[3][1:0] m2[2]:[5:4] m2[2][1:0]
	xmm[3] = _mm_unpackhi_epi64(cmm[2], cmm[3]);	///  m2[3]:[7:6] m2[3][3:2] m2[2]:[7:6] m2[2][3:2]
	xmm[4] = _mm_unpacklo_epi64(cmm[4], cmm[5]);	///  m2[5]:[5:4] m2[5][1:0] m2[4]:[5:4] m2[4][1:0]
	xmm[5] = _mm_unpackhi_epi64(cmm[4], cmm[5]);	///  m2[5]:[7:6] m2[5][3:2] m2[4]:[7:6] m2[4][3:2]
	xmm[6] = _mm_unpacklo_epi64(cmm[6], cmm[7]);	///  m2[7]:[5:4] m2[7][1:0] m2[6]:[5:4] m2[6][1:0]
	xmm[7] = _mm_unpackhi_epi64(cmm[6], cmm[7]);	///  m2[7]:[7:6] m2[7][3:2] m2[6]:[7:6] m2[6][3:2]


	//------------- Horizaontal M1 m1[j][0] = m2[j][0] + m2[j][2]; ----------

	cmm[0] = _mm_add_epi16(xmm[0], xmm[1]);	///m1[1][5 4 1 0] m1[0][5 4 1 0]	
	cmm[1] = _mm_add_epi16(xmm[2], xmm[3]);	///m1[3][5 4 1 0] m1[2][5 4 1 0]	
	cmm[2] = _mm_add_epi16(xmm[4], xmm[5]);	///m1[5][5 4 1 0] m1[4][5 4 1 0]	
	cmm[3] = _mm_add_epi16(xmm[6], xmm[7]);	///m1[7][5 4 1 0] m1[6][5 4 1 0]	
	cmm[4] = _mm_sub_epi16(xmm[0], xmm[1]);	///m1[1][7 6 3 2] m1[0][7 6 3 2]	
	cmm[5] = _mm_sub_epi16(xmm[2], xmm[3]);	///m1[3][7 6 3 2] m1[2][7 6 3 2]	
	cmm[6] = _mm_sub_epi16(xmm[4], xmm[5]);	///m1[5][7 6 3 2] m1[4][7 6 3 2]	
	cmm[7] = _mm_sub_epi16(xmm[6], xmm[7]);	///m1[7][7 6 3 2] m1[6][7 6 3 2]	

	xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);	///m1[0][7 5 6 4 3 1 2 0]
	xmm[1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);	///m1[1][7 5 6 4 3 1 2 0]
	xmm[2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);	///m1[2][7 5 6 4 3 1 2 0]
	xmm[3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);	///m1[3][7 5 6 4 3 1 2 0]
	xmm[4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);	///m1[4][7 5 6 4 3 1 2 0]
	xmm[5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);	///m1[5][7 5 6 4 3 1 2 0]
	xmm[6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);	///m1[6][7 5 6 4 3 1 2 0]
	xmm[7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);	///m1[7][7 5 6 4 3 1 2 0]

	cmm[0] = _mm_unpacklo_epi32(xmm[0], xmm[1]);	///m1[1][3 1] m1[0][3 1] m1[1][2 0] m1[0][2 0]
	cmm[1] = _mm_unpackhi_epi32(xmm[0], xmm[1]);	///m1[1][7 5] m1[0][7 5] m1[1][6 4] m1[0][6 4]
	cmm[2] = _mm_unpacklo_epi32(xmm[2], xmm[3]);	///m1[3][3 1] m1[2][3 1] m1[3][2 0] m1[2][2 0]
	cmm[3] = _mm_unpackhi_epi32(xmm[2], xmm[3]);	///m1[3][7 5] m1[2][7 5] m1[3][6 4] m1[2][6 4]
	cmm[4] = _mm_unpacklo_epi32(xmm[4], xmm[5]);	///m1[5][3 1] m1[4][3 1] m1[5][2 0] m1[4][2 0]
	cmm[5] = _mm_unpackhi_epi32(xmm[4], xmm[5]);	///m1[5][7 5] m1[4][7 5] m1[5][6 4] m1[4][6 4]
	cmm[6] = _mm_unpacklo_epi32(xmm[6], xmm[7]);	///m1[7][3 1] m1[6][3 1] m1[7][2 0] m1[6][2 0]
	cmm[7] = _mm_unpackhi_epi32(xmm[6], xmm[7]);	///m1[7][7 5] m1[6][7 5] m1[7][6 4] m1[6][6 4]

	xmm[0] = _mm_unpacklo_epi64(cmm[0], cmm[1]);	///m1[1][6 4] m1[0][6 4] m1[1][2 0] m1[0][2 0]
	xmm[1] = _mm_unpackhi_epi64(cmm[0], cmm[1]);	///m1[1][7 5] m1[0][7 5] m1[1][3 1] m1[0][3 1] 
	xmm[2] = _mm_unpacklo_epi64(cmm[2], cmm[3]);	///m1[3][6 4] m1[2][6 4] m1[3][2 0] m1[2][2 0]  
	xmm[3] = _mm_unpackhi_epi64(cmm[2], cmm[3]);	///m1[3][7 5] m1[2][7 5] m1[3][3 1] m1[2][3 1] 
	xmm[4] = _mm_unpacklo_epi64(cmm[4], cmm[5]);	///m1[5][6 4] m1[4][6 4] m1[5][2 0] m1[4][2 0]    
	xmm[5] = _mm_unpackhi_epi64(cmm[4], cmm[5]);	///m1[5][7 5] m1[4][7 5] m1[5][3 1] m1[4][3 1] 
	xmm[6] = _mm_unpacklo_epi64(cmm[6], cmm[7]);	///m1[7][6 4] m1[6][6 4] m1[7][2 0] m1[6][2 0]      
	xmm[7] = _mm_unpackhi_epi64(cmm[6], cmm[7]);	///m1[7][7 5] m1[6][7 5] m1[7][3 1] m1[6][3 1]   

	//------------- Horizaontal M2 m2[j][0] = m1[j][0] + m1[j][1]; ----------

	cmm[0] = _mm_add_epi16(xmm[0], xmm[1]); ///m2[1][6 4] m2[0][6 4] m2[1][2 0] m2[0][2 0]
	cmm[1] = _mm_add_epi16(xmm[2], xmm[3]); ///m2[3][6 4] m2[2][6 4] m2[3][2 0] m2[2][2 0]
	cmm[2] = _mm_add_epi16(xmm[4], xmm[5]); ///m2[5][6 4] m2[4][6 4] m2[5][2 0] m2[4][2 0]
	cmm[3] = _mm_add_epi16(xmm[6], xmm[7]); ///m2[7][6 4] m2[6][6 4] m2[7][2 0] m2[6][2 0]
	cmm[4] = _mm_sub_epi16(xmm[0], xmm[1]); ///m2[1][7 5] m2[0][7 5] m2[1][3 1] m2[0][3 1]
	cmm[5] = _mm_sub_epi16(xmm[2], xmm[3]); ///m2[3][7 5] m2[2][7 5] m2[3][3 1] m2[2][3 1]
	cmm[6] = _mm_sub_epi16(xmm[4], xmm[5]); ///m2[5][7 5] m2[4][7 5] m2[5][3 1] m2[4][3 1]
	cmm[7] = _mm_sub_epi16(xmm[6], xmm[7]); ///m2[7][7 5] m2[6][7 5] m2[7][3 1] m2[6][3 1]

	xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);	///m2[1][3 2 1 0] m2[0][3 2 1 0]
	xmm[1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);	///m2[1][7 6 5 4] m2[0][7 6 5 4]
	xmm[2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);	///m2[3][3 2 1 0] m2[2][3 2 1 0]
	xmm[3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);	///m2[3][7 6 5 4] m2[2][7 6 5 4]
	xmm[4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);	///m2[5][3 2 1 0] m2[4][3 2 1 0]
	xmm[5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);	///m2[5][7 6 5 4] m2[4][7 6 5 4]
	xmm[6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);	///m2[7][3 2 1 0] m2[6][3 2 1 0]
	xmm[7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);	///m2[7][7 6 5 4] m2[6][7 6 5 4]

	cmm[0] = _mm_unpacklo_epi64(xmm[0], xmm[1]);	///m2[0][7 6 5 4 3 2 1 0]
	cmm[1] = _mm_unpackhi_epi64(xmm[0], xmm[1]);	///m2[1][7 6 5 4 3 2 1 0]
	cmm[2] = _mm_unpacklo_epi64(xmm[2], xmm[3]);	///m2[2][7 6 5 4 3 2 1 0]
	cmm[3] = _mm_unpackhi_epi64(xmm[2], xmm[3]);	///m2[3][7 6 5 4 3 2 1 0]
	cmm[4] = _mm_unpacklo_epi64(xmm[4], xmm[5]);	///m2[4][7 6 5 4 3 2 1 0]
	cmm[5] = _mm_unpackhi_epi64(xmm[4], xmm[5]);	///m2[5][7 6 5 4 3 2 1 0]
	cmm[6] = _mm_unpacklo_epi64(xmm[6], xmm[7]);	///m2[6][7 6 5 4 3 2 1 0]
	cmm[7] = _mm_unpackhi_epi64(xmm[6], xmm[7]);	///m2[7][7 6 5 4 3 2 1 0]

	//------------- Vertical Processing ----------

	xmm[0] = _mm_add_epi16(cmm[0], cmm[4]);	///	m3[0][i] = m2[0][i] + m2[4][i];
	xmm[1] = _mm_add_epi16(cmm[1], cmm[5]);	///	m3[1][i] = m2[1][i] + m2[5][i];
	xmm[2] = _mm_add_epi16(cmm[2], cmm[6]);	///	m3[2][i] = m2[2][i] + m2[6][i];
	xmm[3] = _mm_add_epi16(cmm[3], cmm[7]);	///	m3[3][i] = m2[3][i] + m2[7][i];
	xmm[4] = _mm_sub_epi16(cmm[0], cmm[4]);	///	m3[4][i] = m2[0][i] - m2[4][i];
	xmm[5] = _mm_sub_epi16(cmm[1], cmm[5]);	///	m3[5][i] = m2[1][i] - m2[5][i];
	xmm[6] = _mm_sub_epi16(cmm[2], cmm[6]);	///	m3[6][i] = m2[2][i] - m2[6][i];
	xmm[7] = _mm_sub_epi16(cmm[3], cmm[7]);	///	m3[7][i] = m2[3][i] - m2[7][i];

	cmm[0] = _mm_add_epi16(xmm[0], xmm[2]);	///	m1[0][i] = m3[0][i] + m3[2][i];
	cmm[1] = _mm_add_epi16(xmm[1], xmm[3]);	///	m1[1][i] = m3[1][i] + m3[3][i];
	cmm[2] = _mm_sub_epi16(xmm[0], xmm[2]);	///	m1[2][i] = m3[0][i] - m3[2][i];
	cmm[3] = _mm_sub_epi16(xmm[1], xmm[3]);	///	m1[3][i] = m3[1][i] - m3[3][i];
	cmm[4] = _mm_add_epi16(xmm[4], xmm[6]);	///	m1[4][i] = m3[4][i] + m3[6][i];
	cmm[5] = _mm_add_epi16(xmm[5], xmm[7]);	///	m1[5][i] = m3[5][i] + m3[7][i];
	cmm[6] = _mm_sub_epi16(xmm[4], xmm[6]);	///	m1[6][i] = m3[4][i] - m3[6][i];
	cmm[7] = _mm_sub_epi16(xmm[5], xmm[7]);	///	m1[7][i] = m3[5][i] - m3[7][i];

	/** -------------------------------------------------------------------------
	Vertical Final Processing
	It causes the overflow of 16 bit processing in SIMD, when the 10 bit processing is conducted. 
	At the Final Processing in calHAD 8x8, since it requires 16bits, the final processing is conducted 
	on the 32bits SIMD.
	---------------------------------------------------------------------------*/
	smm = _mm_setzero_si128();
	pmm = _mm_set1_epi16(1);

	xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[1]);	/// m1[1][k] m1[0][k] k:[3:0]
	xmm[1] = _mm_unpackhi_epi16(cmm[0], cmm[1]);	/// m1[1][k] m1[0][k] k:[7:4]
	xmm[2] = _mm_unpacklo_epi16(cmm[2], cmm[3]);	/// m1[3][k] m1[2][k] k:[3:0]
	xmm[3] = _mm_unpackhi_epi16(cmm[2], cmm[3]);	/// m1[3][k] m1[2][k] k:[7:4]
	xmm[4] = _mm_unpacklo_epi16(cmm[4], cmm[5]);	/// m1[5][k] m1[4][k] k:[3:0]
	xmm[5] = _mm_unpackhi_epi16(cmm[4], cmm[5]);	/// m1[5][k] m1[4][k] k:[7:4]
	xmm[6] = _mm_unpacklo_epi16(cmm[6], cmm[7]);	/// m1[7][k] m1[6][k] k:[3:0]
	xmm[7] = _mm_unpackhi_epi16(cmm[6], cmm[7]);	/// m1[7][k] m1[6][k] k:[7:4]

	cmm[0] = _mm_madd_epi16(xmm[0], pmm);	///	m2[0][i] = m1[0][i] + m1[1][i]; i:[3:0]
	cmm[1] = _mm_madd_epi16(xmm[1], pmm);	///	m2[0][i] = m1[0][i] + m1[1][i]; i:[7:4]
	cmm[2] = _mm_madd_epi16(xmm[0], Mmm);	///	m2[1][i] = m1[0][i] - m1[1][i]; i:[3:0]
	cmm[3] = _mm_madd_epi16(xmm[1], Mmm);	///	m2[1][i] = m1[0][i] - m1[1][i]; i:[7:4]
	cmm[4] = _mm_madd_epi16(xmm[2], pmm);	///	m2[2][i] = m1[2][i] + m1[3][i]; i:[3:0]
	cmm[5] = _mm_madd_epi16(xmm[3], pmm);	///	m2[2][i] = m1[2][i] + m1[3][i]; i:[7:4]
	cmm[6] = _mm_madd_epi16(xmm[2], Mmm);	///	m2[3][i] = m1[2][i] - m1[3][i]; i:[3:0]
	cmm[7] = _mm_madd_epi16(xmm[3], Mmm);	///	m2[3][i] = m1[2][i] - m1[3][i]; i:[7:4]
	
	dmm = _mm_abs_epi32(cmm[0]);			/// abs(m2[0][i]) i:[3:0]

	smm = _mm_add_epi32(smm, dmm);
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[1]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[2]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[3]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[4]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[5]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[6]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[7]));

	cmm[0] = _mm_madd_epi16(xmm[4], pmm);	/// m2[4][i] = m1[4][i] + m1[5][i]; i:[3:0]
	cmm[1] = _mm_madd_epi16(xmm[5], pmm);	/// m2[4][i] = m1[4][i] + m1[5][i]; i:[7:4]
	cmm[2] = _mm_madd_epi16(xmm[4], Mmm);	/// m2[5][i] = m1[4][i] - m1[5][i]; i:[3:0]
	cmm[3] = _mm_madd_epi16(xmm[5], Mmm);	/// m2[5][i] = m1[4][i] - m1[5][i]; i:[7:4]
	cmm[4] = _mm_madd_epi16(xmm[6], pmm);	/// m2[6][i] = m1[6][i] + m1[7][i]; i:[3:0]
	cmm[5] = _mm_madd_epi16(xmm[7], pmm);	/// m2[6][i] = m1[6][i] + m1[7][i]; i:[7:4]
	cmm[6] = _mm_madd_epi16(xmm[6], Mmm);	/// m2[7][i] = m1[6][i] - m1[7][i]; i:[3:0]
	cmm[7] = _mm_madd_epi16(xmm[7], Mmm);	/// m2[7][i] = m1[6][i] - m1[7][i]; i:[7:4]

	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[0]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[1]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[2]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[3]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[4]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[5]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[6]));
	smm = _mm_add_epi32(smm, _mm_abs_epi32(cmm[7]));

	smm = _mm_hadd_epi32(smm, smm);
	smm = _mm_hadd_epi32(smm, smm);

	int satd = (_mm_extract_epi32(smm, 0) + 2) >> 2;
	uiDCValue = (_mm_extract_epi32(dmm, 0) + 2) >> 2;

	return satd;

#else

// [JDS]: Original HM Code
	Int k, i, j, jj, sad=0;
	Int diff[64], m1[8][8], m2[8][8], m3[8][8];
	assert( iStep == 1 );
	for( k = 0; k < 64; k += 8 )
	{
		diff[k+0] = piOrg[0] - piCur[0];
		diff[k+1] = piOrg[1] - piCur[1];
		diff[k+2] = piOrg[2] - piCur[2];
		diff[k+3] = piOrg[3] - piCur[3];
		diff[k+4] = piOrg[4] - piCur[4];
		diff[k+5] = piOrg[5] - piCur[5];
		diff[k+6] = piOrg[6] - piCur[6];
		diff[k+7] = piOrg[7] - piCur[7];

		piCur += iStrideCur;
		piOrg += iStrideOrg;
	}

	//horizontal
	for (j=0; j < 8; j++)
	{
		jj = j << 3;
		m2[j][0] = diff[jj  ] + diff[jj+4];
		m2[j][1] = diff[jj+1] + diff[jj+5];
		m2[j][2] = diff[jj+2] + diff[jj+6];
		m2[j][3] = diff[jj+3] + diff[jj+7];
		m2[j][4] = diff[jj  ] - diff[jj+4];
		m2[j][5] = diff[jj+1] - diff[jj+5];
		m2[j][6] = diff[jj+2] - diff[jj+6];
		m2[j][7] = diff[jj+3] - diff[jj+7];

		m1[j][0] = m2[j][0] + m2[j][2];
		m1[j][1] = m2[j][1] + m2[j][3];
		m1[j][2] = m2[j][0] - m2[j][2];
		m1[j][3] = m2[j][1] - m2[j][3];
		m1[j][4] = m2[j][4] + m2[j][6];
		m1[j][5] = m2[j][5] + m2[j][7];
		m1[j][6] = m2[j][4] - m2[j][6];
		m1[j][7] = m2[j][5] - m2[j][7];

		m2[j][0] = m1[j][0] + m1[j][1];
		m2[j][1] = m1[j][0] - m1[j][1];
		m2[j][2] = m1[j][2] + m1[j][3];
		m2[j][3] = m1[j][2] - m1[j][3];
		m2[j][4] = m1[j][4] + m1[j][5];
		m2[j][5] = m1[j][4] - m1[j][5];
		m2[j][6] = m1[j][6] + m1[j][7];
		m2[j][7] = m1[j][6] - m1[j][7];
	}

	//vertical
	for (i=0; i < 8; i++)
	{
		m3[0][i] = m2[0][i] + m2[4][i];
		m3[1][i] = m2[1][i] + m2[5][i];
		m3[2][i] = m2[2][i] + m2[6][i];
		m3[3][i] = m2[3][i] + m2[7][i];
		m3[4][i] = m2[0][i] - m2[4][i];
		m3[5][i] = m2[1][i] - m2[5][i];
		m3[6][i] = m2[2][i] - m2[6][i];
		m3[7][i] = m2[3][i] - m2[7][i];

		m1[0][i] = m3[0][i] + m3[2][i];
		m1[1][i] = m3[1][i] + m3[3][i];
		m1[2][i] = m3[0][i] - m3[2][i];
		m1[3][i] = m3[1][i] - m3[3][i];
		m1[4][i] = m3[4][i] + m3[6][i];
		m1[5][i] = m3[5][i] + m3[7][i];
		m1[6][i] = m3[4][i] - m3[6][i];
		m1[7][i] = m3[5][i] - m3[7][i];

		m2[0][i] = m1[0][i] + m1[1][i];
		m2[1][i] = m1[0][i] - m1[1][i];
		m2[2][i] = m1[2][i] + m1[3][i];
		m2[3][i] = m1[2][i] - m1[3][i];
		m2[4][i] = m1[4][i] + m1[5][i];
		m2[5][i] = m1[4][i] - m1[5][i];
		m2[6][i] = m1[6][i] + m1[7][i];
		m2[7][i] = m1[6][i] - m1[7][i];
	}

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 8; j++)
		{
			sad += abs(m2[i][j]);
		}
	}

	sad=((sad+2)>>2);

	return sad;
#endif // #if ETRI_SIMD_MD
}

UInt TComRdCost::xGetHADs4( DistParam* pcDtParam )
{
	if ( pcDtParam->bApplyWeight )
	{
		return xGetHADs4w( pcDtParam );
	}

	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStep  = pcDtParam->iStep;
	Int  y;
	Int  iOffsetOrg = iStrideOrg<<2;
	Int  iOffsetCur = iStrideCur<<2;

	UInt uiHADDC = 0, uiSumHADDC = 0; 
	UInt uiSum = 0;

	for ( y=0; y<iRows; y+= 4 )
	{
		uiSum += xCalcHADs4x4( piOrg, piCur, iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC;
		piOrg += iOffsetOrg;
		piCur += iOffsetCur;
	}

	pcDtParam->e_uiHADDC = uiSumHADDC;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetHADs8( DistParam* pcDtParam )
{
	if ( pcDtParam->bApplyWeight )
	{
		return xGetHADs8w( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStep  = pcDtParam->iStep;
	Int  y;

	UInt uiHADDC = 0, uiSumHADDC = 0; 
	UInt uiSum = 0;

	if ( iRows == 4 )
	{
		uiSum += xCalcHADs4x4( piOrg+0, piCur        , iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC; 
		uiSum += xCalcHADs4x4( piOrg+4, piCur+4*iStep, iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC; 
	}
	else
	{
		Int  iOffsetOrg = iStrideOrg<<3;
		Int  iOffsetCur = iStrideCur<<3;
		for ( y=0; y<iRows; y+= 8 )
		{
			uiSum += xCalcHADs8x8( piOrg, piCur, iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC;
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}

	pcDtParam->e_uiHADDC = uiSumHADDC;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetHADs( DistParam* pcDtParam )
{
	if ( pcDtParam->bApplyWeight )
	{
		return xGetHADsw( pcDtParam );
	}
	Pel* piOrg   = pcDtParam->pOrg;
	Pel* piCur   = pcDtParam->pCur;
	Int  iRows   = pcDtParam->iRows;
	Int  iCols   = pcDtParam->iCols;
	Int  iStrideCur = pcDtParam->iStrideCur;
	Int  iStrideOrg = pcDtParam->iStrideOrg;
	Int  iStep  = pcDtParam->iStep;

	Int  x, y;

	UInt uiHADDC = 0, uiSumHADDC = 0; 
	UInt uiSum = 0;

#if ETRI_EM_OPERATION_OPTIMIZATION
	if (((iRows & 0x07) == 0) && ((iCols & 0x07) == 0))
	{
		Int  iOffsetOrg = iStrideOrg << 3;
		Int  iOffsetCur = iStrideCur << 3;
		for (y = 0; y<iRows; y += 8)
		{
			for (x = 0; x<iCols; x += 8)
			{
				uiSum += xCalcHADs8x8(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC; 
			}
		piOrg += iOffsetOrg;
		piCur += iOffsetCur;
		}
	}
	else if (((iRows & 0x03) == 0) && ((iCols & 0x03) == 0))
	{
		Int  iOffsetOrg = iStrideOrg << 2;
		Int  iOffsetCur = iStrideCur << 2;

		for (y = 0; y<iRows; y += 4)
		{
			for (x = 0; x<iCols; x += 4)
			{
				uiSum += xCalcHADs4x4(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC; 
			}
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}
	else if (((iRows & 1) == 0) && ((iCols & 1) == 0))
	{
		Int  iOffsetOrg = iStrideOrg << 1;
		Int  iOffsetCur = iStrideCur << 1;
		for (y = 0; y<iRows; y += 2)
		{
			for (x = 0; x<iCols; x += 2)
			{
				uiSum += xCalcHADs2x2(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);	uiSumHADDC += uiHADDC; 
			}
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}
	else
	{
		assert(false);
	}

	pcDtParam->e_uiHADDC = uiSumHADDC;
	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
#else
	if( ( iRows % 8 == 0) && (iCols % 8 == 0) )
	{
		Int  iOffsetOrg = iStrideOrg<<3;
		Int  iOffsetCur = iStrideCur<<3;
		for ( y=0; y<iRows; y+= 8 )
		{
			for ( x=0; x<iCols; x+= 8 )
			{
				uiSum += xCalcHADs8x8(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);
			}
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}
	else if( ( iRows % 4 == 0) && (iCols % 4 == 0) )
	{
		Int  iOffsetOrg = iStrideOrg<<2;
		Int  iOffsetCur = iStrideCur<<2;

		for ( y=0; y<iRows; y+= 4 )
		{
			for ( x=0; x<iCols; x+= 4 )
			{
				uiSum += xCalcHADs4x4(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);
			}
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}
	else if( ( iRows % 2 == 0) && (iCols % 2 == 0) )
	{
		Int  iOffsetOrg = iStrideOrg<<1;
		Int  iOffsetCur = iStrideCur<<1;
		for ( y=0; y<iRows; y+=2 )
		{
			for ( x=0; x<iCols; x+=2 )
			{
				uiSum += xCalcHADs2x2(&piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep, uiHADDC);
			}
			piOrg += iOffsetOrg;
			piCur += iOffsetCur;
		}
	}
	else
	{
		assert(false);
	}

	return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
#endif

}


#if ETRI_FAST_INTEGERME
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief	Working Function for InterSearch. Default PreDefinitions. Look at the V12 Ver 143
	@param	pcDtParam
------------------------------------------------------------------------------------------------------------------------------------------------
*/
UInt TComRdCost::ETRI_GetSAD( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int 	BaseUnit, iLimit = (pcDtParam->iCols>>ETRI_SIMD_LogUnit), jLimit = (pcDtParam->iRows>>ETRI_SIMD_LogUnit);
	UInt* tmm = pcDtParam->tmm;

	Short  _BitShift = pcDtParam->bitDepth - 8;

	Int   k, j, i;

	__m128i	zmm[8][8], Tmm[2]; 
	__m128i rmm[3], omm, xmmz, xmmS;

#if 1
//	if (pcDtParam->iCols == 4)	
	printf("ETRI_GetSAD   Cols , Rows  : %d , %d \n", pcDtParam->iCols, pcDtParam->iRows);
#endif

	xmmz    = _mm_setzero_si128();
	Tmm[0] = _mm_setzero_si128();
	Tmm[1] = _mm_setzero_si128();

	// SAD for 64x64 Block
	for(k=0; k<jLimit; k++)
	{
		memset(zmm[k], 0, sizeof(__m128i) * ETRI_SIMD_BaseUnit_16bit);

		// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
		for(j=0; j<ETRI_SIMD_BaseUnit_16bit; j++)
		{
			//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
			omm     = _mm_load_si128((__m128i const *)pcpyOrg);
			rmm[0] = _mm_loadu_si128((__m128i const *)piRefSrch);
			rmm[1] = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

			omm  = _mm_srai_epi16(omm, _BitShift);		  // 10bit를 8비트로 
			rmm[0] = _mm_srai_epi16(rmm[0], _BitShift); 	   // 10bit를 8비트로 
			rmm[1] = _mm_srai_epi16(rmm[1], _BitShift); 	   // 10bit를 8비트로 

			rmm[2] = _mm_packus_epi16(rmm[0], rmm[1]);
			omm     = _mm_packus_epi16(omm, xmmz);
			
			//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 | Ref:14-4	: Org :7-4	Mask 1 0 1>
			xmmS       = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm[2], omm, 0), _mm_mpsadbw_epu8(rmm[2], omm, 5));	
			zmm[k][0] = _mm_adds_epu16(zmm[k][0], xmmS);

			for(i=1; i<iLimit; i++)	//i : 8 Cols Line
			{
				omm        = _mm_load_si128((__m128i const *)(pcpyOrg + (BaseUnit = (i<<ETRI_SIMD_LogUnit))));
				rmm[(i+1)& 0x01]	= _mm_loadu_si128((__m128i const *)(piRefSrch + BaseUnit + ETRI_SIMD_BaseUnit_16bit));

				omm  = _mm_srai_epi16(omm, _BitShift);		  // 10bit를 8비트로 
				rmm[(i+1)& 0x01] = _mm_srai_epi16(rmm[(i+1)& 0x01], _BitShift); 	   // 10bit를 8비트로 

				rmm[2]    = _mm_packus_epi16(rmm[i & 0x01], rmm[(i+1) & 0x01]);
				omm        = _mm_packus_epi16(omm, xmmz);
				xmmS      = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm[2], omm, 0), _mm_mpsadbw_epu8(rmm[2], omm, 5));	
				zmm[k][i] = _mm_adds_epu16(zmm[k][i], xmmS);
			}

			pcpyOrg	+= 	 iStrideOrg;
			piRefSrch	+= 	iRefStride;
		}

		omm     = _mm_adds_epu16(_mm_adds_epu16(zmm[k][0], zmm[k][1]), _mm_adds_epu16(zmm[k][2], zmm[k][3]));
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));
		omm     = _mm_adds_epu16(_mm_adds_epu16(zmm[k][4], zmm[k][5]), _mm_adds_epu16(zmm[k][6], zmm[k][7]));
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));
	}

	_mm_store_si128((__m128i *)tmm,        Tmm[0]);
	_mm_store_si128((__m128i *)(tmm+4), Tmm[1]);
	
	return 0;
}

UInt TComRdCost::ETRI_GetSAD4( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int   j, jLimit  = (pcDtParam->iRows > 4)? ETRI_SIMD_BaseUnit_16bit : pcDtParam->iRows; 
	Short  _BitShift = pcDtParam->bitDepth - 8;
	UInt* tmm = pcDtParam->tmm;

	__m128i zmm, rmm0, rmm1, rmm2, omm, xmmz;

	xmmz   = _mm_setzero_si128();
	zmm     = _mm_setzero_si128();

	// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
	for(j=0; j<jLimit; j++)
	{
		//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
		omm  = _mm_loadu_si128((__m128i const *)pcpyOrg);
		rmm0 = _mm_loadu_si128((__m128i const *)piRefSrch);
		rmm1 = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

		// 10bit를 8비트로 
		omm  = _mm_srai_epi16(omm, _BitShift);  		
		rmm0 = _mm_srai_epi16(rmm0, _BitShift);			
		rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   

		// Data Pack for _mm_mpsadbw_epu8
		rmm2 = _mm_packus_epi16(rmm0, rmm1);
		omm  = _mm_packus_epi16(omm, xmmz);
			
		//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 >
		zmm = _mm_adds_epu16(zmm, _mm_mpsadbw_epu8(rmm2, omm, 0));

		pcpyOrg	+= 	 iStrideOrg;
		piRefSrch	+= 	iRefStride;
	}
	_mm_store_si128((__m128i *)tmm,       _mm_unpacklo_epi16(zmm, xmmz));
	_mm_store_si128((__m128i *)(tmm+4), _mm_unpackhi_epi16(zmm, xmmz));

	return 0;
}



UInt TComRdCost::ETRI_GetSAD8( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int 	kLimit = (pcDtParam->iRows > 4)? (pcDtParam->iRows>>ETRI_SIMD_LogUnit) : 1;
	Int   jLimit  = (pcDtParam->iRows > 4)? ETRI_SIMD_BaseUnit_16bit : pcDtParam->iRows; 
	Short  _BitShift = pcDtParam->bitDepth - 8;
	UInt* tmm = pcDtParam->tmm;

	Int   k, j;

	__m128i	zmm[8], Tmm[2]; 
	__m128i rmm0, rmm1, rmm2, omm, xmmz, xmmS;

	xmmz    = _mm_setzero_si128();
	Tmm[0] = _mm_setzero_si128();
	Tmm[1] = _mm_setzero_si128();

	for(k=0; k<kLimit; k++)
	{
		zmm[k] = _mm_setzero_si128();

		// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
		for(j=0; j<jLimit; j++)
		{
			//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
			omm  = _mm_load_si128((__m128i const *)pcpyOrg);
			rmm0 = _mm_loadu_si128((__m128i const *)piRefSrch);
			rmm1 = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2 = _mm_packus_epi16(rmm0, rmm1);
			omm  = _mm_packus_epi16(omm, xmmz);
			
			//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 | Ref:14-4	: Org :7-4	Mask 1 0 1>
			xmmS       = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5));	
			zmm[k]     = _mm_adds_epu16(zmm[k], xmmS);

			pcpyOrg	+= 	 iStrideOrg;
			piRefSrch	+= 	iRefStride;
		}

		Tmm[0] = _mm_adds_epi16(Tmm[0], zmm[k]);
	}
	_mm_store_si128((__m128i *)tmm,       _mm_unpacklo_epi16(Tmm[0], xmmz));
	_mm_store_si128((__m128i *)(tmm+4), _mm_unpackhi_epi16(Tmm[0], xmmz));

	return 0;
}


UInt TComRdCost::ETRI_GetSAD16( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int 	jLimit = (pcDtParam->iRows>>ETRI_SIMD_LogUnit);
	Short  _BitShift = pcDtParam->bitDepth - 8;
	UInt* tmm = pcDtParam->tmm;

	Int   k, j;

	__m128i	zmm[8][2], Tmm[2]; 
	__m128i rmm0, rmm1, rmm2, omm, xmmz, xmmS;

	xmmz    = _mm_setzero_si128();
	Tmm[0] = _mm_setzero_si128();
	Tmm[1] = _mm_setzero_si128();

	for(k=0; k<jLimit; k++)
	{
		zmm[k][0] = _mm_setzero_si128();
		zmm[k][1] = _mm_setzero_si128();

		// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
		for(j=0; j<ETRI_SIMD_BaseUnit_16bit; j++)
		{
			//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
			omm  = _mm_load_si128((__m128i const *)pcpyOrg);
			rmm0 = _mm_loadu_si128((__m128i const *)piRefSrch);
			rmm1 = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2 = _mm_packus_epi16(rmm0, rmm1);
			omm  = _mm_packus_epi16(omm, xmmz);
			
			//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 | Ref:14-4	: Org :7-4	Mask 1 0 1>
			xmmS       = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5));	
			zmm[k][0] = _mm_adds_epu16(zmm[k][0], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 8));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 16));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][1] = _mm_adds_epu16(zmm[k][1], xmmS);

			pcpyOrg	+= 	 iStrideOrg;
			piRefSrch	+= 	iRefStride;
		}

		omm     = _mm_adds_epu16(zmm[k][0], zmm[k][1]); 
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));
	}

	_mm_store_si128((__m128i *)tmm,        Tmm[0]);
	_mm_store_si128((__m128i *)(tmm+4), Tmm[1]);
	
	return 0;

}

UInt TComRdCost::ETRI_GetSAD32( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int 	jLimit = (pcDtParam->iRows>>ETRI_SIMD_LogUnit);
	Short  _BitShift = pcDtParam->bitDepth - 8;
	UInt* tmm = pcDtParam->tmm;

	Int   k, j;

	__m128i	zmm[8][4], Tmm[2]; 
	__m128i rmm0, rmm1, rmm2, omm, xmmz, xmmS;

	xmmz    = _mm_setzero_si128();
	Tmm[0] = _mm_setzero_si128();
	Tmm[1] = _mm_setzero_si128();

	// SAD for 64x64 Block
	for(k=0; k<jLimit; k++)
	{
		zmm[k][0] = _mm_setzero_si128();
		zmm[k][1] = _mm_setzero_si128();
		zmm[k][2] = _mm_setzero_si128();
		zmm[k][3] = _mm_setzero_si128();

		// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
		for(j=0; j<ETRI_SIMD_BaseUnit_16bit; j++)
		{

			//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
			omm  = _mm_load_si128((__m128i const *)pcpyOrg);
			rmm0 = _mm_loadu_si128((__m128i const *)piRefSrch);
			rmm1 = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2 = _mm_packus_epi16(rmm0, rmm1);
			omm  = _mm_packus_epi16(omm, xmmz);
		
			//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 | Ref:14-4	: Org :7-4	Mask 1 0 1>
			xmmS       = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5));	
			zmm[k][0] = _mm_adds_epu16(zmm[k][0], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 8));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 16));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][1] = _mm_adds_epu16(zmm[k][1], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 16));
			rmm1  = _mm_loadu_si128((__m128i const *)(piRefSrch + 24));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm0, rmm1);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][2] = _mm_adds_epu16(zmm[k][2], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 24));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 32));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][3] = _mm_adds_epu16(zmm[k][3], xmmS);

			pcpyOrg	+= 	 iStrideOrg;
			piRefSrch	+= 	iRefStride;
		}

		omm     = _mm_adds_epu16(_mm_adds_epu16(zmm[k][0], zmm[k][1]), _mm_adds_epu16(zmm[k][2], zmm[k][3]));
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));
	}

	_mm_store_si128((__m128i *)tmm,        Tmm[0]);
	_mm_store_si128((__m128i *)(tmm+4), Tmm[1]);
	
	return 0;

}

UInt TComRdCost::ETRI_GetSAD64( DistParam* pcDtParam )
{
	Pel* pcpyOrg   = pcDtParam->pOrg; 				//pOrg : Realk Current Original DaTA
	Pel* piRefSrch = pcDtParam->pCur;
		
	Int   iStrideOrg = 	pcDtParam->iStrideOrg;
	Int   iRefStride  = pcDtParam->iStrideCur;
	Int 	jLimit = (pcDtParam->iRows>>ETRI_SIMD_LogUnit);
	Short  _BitShift = pcDtParam->bitDepth - 8;
	UInt* tmm = pcDtParam->tmm;

	Int   k, j;

	__m128i	zmm[8][8], Tmm[2]; 
	__m128i rmm0, rmm1, rmm2, omm, xmmz, xmmS;

	xmmz    = _mm_setzero_si128();
	Tmm[0] = _mm_setzero_si128();
	Tmm[1] = _mm_setzero_si128();

	// SAD for 64x64 Block
	for(k=0; k<jLimit; k++)
	{
		zmm[k][0] = _mm_setzero_si128();
		zmm[k][1] = _mm_setzero_si128();
		zmm[k][2] = _mm_setzero_si128();
		zmm[k][3] = _mm_setzero_si128();
		zmm[k][4] = _mm_setzero_si128();
		zmm[k][5] = _mm_setzero_si128();
		zmm[k][6] = _mm_setzero_si128();
		zmm[k][7] = _mm_setzero_si128();

		// SAD for k-th 8x8 Block Line k/i 8x8 Block Idx : up to 14 bit
		for(j=0; j<ETRI_SIMD_BaseUnit_16bit; j++)
		{
			//Read Org 16 bit x 16 =256 = 128bit x 2 : ETRI_SIMD_BaseUnit_16bit == 8 : Read Ref and make 16 components of Ref
			omm  = _mm_load_si128((__m128i const *)pcpyOrg);
			rmm0 = _mm_loadu_si128((__m128i const *)piRefSrch);
			rmm1 = _mm_loadu_si128((__m128i const *)(piRefSrch+ETRI_SIMD_BaseUnit_16bit));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2 = _mm_packus_epi16(rmm0, rmm1);
			omm  = _mm_packus_epi16(omm, xmmz);
			
			//Evaluate SAD on 1 Line with 8 Components : <Ref:10-0: Org :3-0 Mask 0 0 0 | Ref:14-4	: Org :7-4	Mask 1 0 1>
			xmmS       = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5));	
			zmm[k][0] = _mm_adds_epu16(zmm[k][0], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 8));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 16));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][1] = _mm_adds_epu16(zmm[k][1], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 16));
			rmm1  = _mm_loadu_si128((__m128i const *)(piRefSrch + 24));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm0, rmm1);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][2] = _mm_adds_epu16(zmm[k][2], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 24));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 32));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][3] = _mm_adds_epu16(zmm[k][3], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 32));
			rmm1  = _mm_loadu_si128((__m128i const *)(piRefSrch + 40));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm0, rmm1);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][4] = _mm_adds_epu16(zmm[k][4], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 40));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 48));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
		
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][5] = _mm_adds_epu16(zmm[k][5], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 48));
			rmm1  = _mm_loadu_si128((__m128i const *)(piRefSrch + 56));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm1 = _mm_srai_epi16(rmm1, _BitShift); 	   
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm0, rmm1);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][6] = _mm_adds_epu16(zmm[k][6], xmmS);

			omm   = _mm_load_si128((__m128i const *)(pcpyOrg + 56));
			rmm0  = _mm_loadu_si128((__m128i const *)(piRefSrch + 64));

			// 10bit를 8비트로 
			omm  = _mm_srai_epi16(omm, _BitShift);			
			rmm0 = _mm_srai_epi16(rmm0, _BitShift); 		
			
			// Data Pack for _mm_mpsadbw_epu8
			rmm2  = _mm_packus_epi16(rmm1, rmm0);
			omm   = _mm_packus_epi16(omm, xmmz);
			xmmS = _mm_adds_epu16(_mm_mpsadbw_epu8(rmm2, omm, 0), _mm_mpsadbw_epu8(rmm2, omm, 5)); 
			zmm[k][7] = _mm_adds_epu16(zmm[k][7], xmmS);

			pcpyOrg	+= 	 iStrideOrg;
			piRefSrch	+= 	iRefStride;
		}

		omm     = _mm_adds_epu16(_mm_adds_epu16(zmm[k][0], zmm[k][1]), _mm_adds_epu16(zmm[k][2], zmm[k][3]));
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));
		omm     = _mm_adds_epu16(_mm_adds_epu16(zmm[k][4], zmm[k][5]), _mm_adds_epu16(zmm[k][6], zmm[k][7]));
		Tmm[0] = _mm_add_epi32(Tmm[0], _mm_unpacklo_epi16(omm, xmmz));
		Tmm[1] = _mm_add_epi32(Tmm[1], _mm_unpackhi_epi16(omm, xmmz));

	}

	_mm_store_si128((__m128i *)tmm,        Tmm[0]);
	_mm_store_si128((__m128i *)(tmm+4), Tmm[1]);
	
	return 0;
}

#endif


//! \}
