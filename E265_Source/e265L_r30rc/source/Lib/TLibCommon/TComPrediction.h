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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComPic.h"
#include "TComMotionInfo.h"
#include "TComPattern.h"
#include "TComTrQuant.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
class TComPrediction : public TComWeightPrediction
{
protected:
	Int*      m_piYuvExt;
	Int       m_iYuvExtStride;
	Int       m_iYuvExtHeight;

	TComYuv   m_acYuvPred[2];
	TComYuv   m_cYuvPredTemp;
	TComYuv m_filteredBlock[4][4];
	TComYuv m_filteredBlockTmp[4];

	TComInterpolationFilter m_if;

	Pel*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample 
	Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

	// ====================================================================================================================
	//ETRI FAST ALGORITHM in Protected Function
	// ====================================================================================================================
#if ETRI_SIMD_INTRA
#if ETRI_SIMD_FIX_INTRA_PLANAR_PREDICTION
    Void xPredIntraPlanar4x4(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride);
    Void xPredIntraPlanar8x8(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride);
    Void xPredIntraPlanar16x16(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride);
    Void xPredIntraPlanar32x32(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride);
    Void xPredIntraPlanar64x64(Int* pSrc, Int*pLSrc, Int srcStride, Pel* rpDst, Int dstStride);
#else 
	Void xPredIntraPlanar4  		(Int* pSrc, Int* pColPixs, Int srcStride, Pel* rpDst, Int dstStride );
	Void xPredIntraPlanar8  		(Int* pSrc, Int* pColPixs, Int srcStride, Pel* rpDst, Int dstStride );
	Void xPredIntraPlanar16 		(Int* pSrc, Int* pColPixs, Int srcStride, Pel* rpDst, Int dstStride );
	Void xPredIntraPlanar32 		(Int* pSrc, Int* pColPixs, Int srcStride, Pel* rpDst, Int dstStride );
	Void xPredIntraPlanar64 		(Int* pSrc, Int* pColPixs, Int srcStride, Pel* rpDst, Int dstStride );
#endif 
#if ETRI_SIMD_FIX_INTRA_ANGULAR_PREDICTION
    Void xPredIntraAng4x4         (Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter );
    Void xPredIntraAng8x8(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter);
    Void xPredIntraAng16x16(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter);
    Void xPredIntraAng32x32(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter);
    Void xPredIntraAng64x64(Int bitDepth, Int* pSrc, Int* pLSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter);
#else 
	Void xPredIntraAng4  			(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride,	UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable , Bool bFilter );
	Void xPredIntraAng8  			(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride,	UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter );
	Void xPredIntraAng16			(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter );
	Void xPredIntraAng32			(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable );
	Void xPredIntraAng64			(Int bitDepth, Int* pSrc, Int* pColPixs, Int srcStride, Pel*& rpDst, Int dstStride, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable );
#endif 
#else
	// HM Original INtra Prediction 
	Void xPredIntraAng  			(Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter );
	Void xPredIntraPlanar  			(Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );
#endif

	// ====================================================================================================================
	//HM Search Function in Protected Function
	// ====================================================================================================================
	// motion compensation functions
	Void xPredInterUni  			( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Bool bi=false);
	Void xPredInterBi  				( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight,                         TComYuv*& rpcYuvPred );
	Void xPredInterLumaBlk 			( TComDataCU* cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi );
	Void xPredInterChromaBlk 		( TComDataCU* cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi );
	Void xWeightedAverage 			( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst );

	Void xGetLLSPrediction			( TComPattern* pcPattern, Int* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0 );

	Void xDCPredFiltering 			( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight );
	Bool xCheckIdenticalMotion  	( TComDataCU* pcCU, UInt PartAddr);

	// ====================================================================================================================
	//HM Search Function in public Function
	// ====================================================================================================================
public:
	TComPrediction();
	virtual ~TComPrediction();

	Void    initTempBuff();

	// inter
	Void motionCompensation 	( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );

	// motion vector prediction
	Void getMvPredAMVP 			( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );

	// Angular Intra
	Void predIntraLumaAng 		( TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft );
	Void predIntraChromaAng  	( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft );

	Int* getPredicBuf()  			{ return m_piYuvExt;      }
	Int  getPredicBufWidth()  		{ return m_iYuvExtStride; }
	Int  getPredicBufHeight() 		{ return m_iYuvExtHeight; }

	// ====================================================================================================================
	//ETRI FAST ALGORITHM in Public Function
	// ====================================================================================================================
#if ETRI_SIMD_INTRA
	Pel  predIntraGetPredValDC 		( Int* pSrc, Int* pColPixs, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft );
	Void ETRI_Transpose4x4 			( Pel* pDst, Int dstStride);
	Void ETRI_Transpose8x8 			( Pel* pDst, Int dstStride);
	Void ETRI_Transpose16x16 		( Pel* pDst, Int dstStride);
	Void ETRI_Transpose32x32 		( Pel* pDst, Int dstStride);
	Void ETRI_Transpose64x64 		( Pel* pDst, Int dstStride);

	//Clear Predition samples
	Void ETRI_clearYuvPred();
#else
	Pel  predIntraGetPredValDC 		( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft );
#endif 

#if ETRI_MODIFICATION_V02
	Void ETRI_HAD_SetParamforGetLevel    		(UInt uiLuma,UInt uiCb, UInt uiCr ){em_uiHADLuma = uiLuma; em_uiHADCb = uiCb; em_uiHADCr = uiCr;}
	UInt ETRI_HAD_getLevel    	   	   	   	   	(UInt*& uiInfo, UInt _rem, Int _iPer, UInt _remC, Int _iPerC );		/// 2015 3 30 by Seok : : Debug for QP Selection between Luma and Chroma
	UInt ETRI_HAD_getControlLevel   			(UInt*& uiInfo, UInt _rem, Int _iPer, UInt _remC, Int _iPerC );
	Void ETRI_SM_motionCompensation         	(TComDataCU*  pcCU, TComYuv* pcYuvPred, TComYuv* pcOrigYuv, TComRdCost* pcRdCost, UInt& uiDistortion, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );


	UInt  ETRI_getHADLuma 			() {return  em_uiHADLuma;}
	UInt  ETRI_getHADCb   			() {return  em_uiHADCb;}
	UInt  ETRI_getHADCr   			() {return  em_uiHADCr;}

	// ====================================================================================================================
	//ETRI  FAST ALGORITHM in Private Variable
	// ====================================================================================================================
private:
	UInt*  em_DbgInfo;

	UInt	  em_uiHADLuma;
	UInt	  em_uiHADCb;
	UInt	  em_uiHADCr;

#endif


};

//! \}

#endif // __TCOMPREDICTION__
