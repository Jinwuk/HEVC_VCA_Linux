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

/** \file     TEncRateCtrl.h
    \brief    Rate control manager class
*/

#ifndef _HM_TENCRATECTRL_H_
#define _HM_TENCRATECTRL_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "../TLibCommon/CommonDef.h"
#include "../TLibCommon/TComDataCU.h"

#include <vector>
#include <algorithm>

using namespace std;

//! \ingroup TLibEncoder
//! \{

#include "../TLibEncoder/TEncCfg.h"
#include <list>
#include <cassert>

#if KAIST_RC
const Int g_RCInvalidQPValue = -999;
const Int g_RCIterationNum = 20;
const Double g_RCWeightHistoryLambda = 0.5;
const Double g_RCWeightCurrentLambda = 1.0 - g_RCWeightHistoryLambda;
const Int g_RCLCUSmoothWindowSize = 4;
const Double g_RCAlphaMinValue = 0.05;
const Double g_RCAlphaMaxValue = 500.0;
const Double g_RCBetaMinValue  = -3.0;
const Double g_RCBetaMaxValue  = -0.1;

#define ALPHA     6.7542;
#define BETA1     1.2517
#define BETA2     1.7860

struct TRCLCU
{
  Int m_actualBits;
  Int m_QP;     // QP of skip mode is set to g_RCInvalidQPValue
  Int m_targetBits;
  Double m_lambda;
  Double m_bitWeight;
  Int m_numberOfPixel;
  Double m_costIntra;
  Int m_targetBitsLeft;
};

struct TRCParameter
{
  Double m_alpha;
  Double m_beta;
  Int m_fromWhichPOC;
  Int m_NofUpdate;
};
class TEncRateCtrl;
class TRCPic
{
public:
	TRCPic();
	~TRCPic();

	Int calcFrameTargetBit(Int iPOC);
	Int getRefineBitsForIntra(Int orgBits);
	Void getLCUInitTargetBits(Int bits);
	Void setTotalIntraCost(Double cost)                     { m_totalCostIntra = cost; }
	Double estimatePicLambda(SliceType eSliceType);
	Int    estimatePicQP(Double lambda, TComSlice* pcSlice);
	Double calculateLambdaIntra(double alpha, double beta, double MADPerPixel, double bitsPerPixel);
	Double getLCUTargetBpp(Int   LCUIdx, SliceType eSliceType);
	Double getLCUEstLambdaAndQP(Int   LCUIdx, Double bpp, Int clipPicQP, Int *estQP);
	Double getLCUEstLambda(Int   LCUIdx, Double bpp);
	Int    getLCUEstQP(Int LCUIdx, Double lambda, Int clipPicQP);
	Void updateAfterLCU(Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter = true);

	Double calAverageQP();
	Double calAverageLambda();

	Void updateAfterPicture(Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType);
	Void   updateAlphaBetaIntra(double *alpha, double *beta);


	Int  getOutputBit()                 { return m_outputBit; }
	Void  setTargetBit(Int targetBit)                 { m_targetBit = targetBit; }

	TRCLCU* getLCU()                                        { return m_LCUs; }
	TRCLCU& getLCU(Int LCUIdx)                            { return m_LCUs[LCUIdx]; }

	TRCParameter  getPicPara()                                   { return m_picPara; }
	Void           setPicPara(TRCParameter para)     { m_picPara = para; }
	TRCParameter* getLCUPara()                                   { return m_LCUPara; }
	TRCParameter   getLCUPara(Int LCUIdx)            { assert(LCUIdx < m_numberOfLCU); return getLCUPara()[LCUIdx]; }
	Void           setLCUPara(Int LCUIdx, TRCParameter para) {assert(LCUIdx < m_numberOfLCU); m_LCUPara[LCUIdx] = para; }

	Int m_frameLevel;
	Int m_POC;
	Double m_totalCostIntra;
	TRCLCU * m_LCUs;
	Double* m_remainingCostIntra;
	Int* m_LCULeft;
	Int* m_bitsLeft;
	Int* m_width_tile;
	Int* m_height_tile;	
	Int* m_numberOfLCUinTile;
	Int* m_numberOfPixel;
	Int* m_pixelsLeft;
	Int m_numberOfLCU;
	Int m_picQP;                  // in integer form
	Double m_picLambda;
	Int m_outputBit;
	Int m_targetBit;

	Int m_intraSize;
	Int m_numTile;

	TEncRateCtrl* m_pcRateCtrl;

	TRCParameter  m_picPara;
	TRCParameter* m_LCUPara;
};

class TEncRateCtrl
{
public:
  TEncRateCtrl();
  ~TEncRateCtrl();

public:
	Void init(Int numTileCol, Int numTileRow, Int* tileColWidth, Int* tileRowHeight, Int totalFrames, Int targetBitrate, Int frameRate, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Bool useLCUSeparateModel, Int intraSize, Int GOPSize, GOPEntry  GOPList[MAX_GOP]);
  Void destroy();
  Void initRCPic( Int frameLevel );

public:
	Int  getTotalFrames()                 { return m_totalFrames; }
	Int  getTargetRate()                  { return m_targetRate; }
	Int  getFrameRate()                   { return m_frameRate; }
	Int  getIntraSize()                   { return m_intraSize; }
	Int  getPicWidth()                    { return m_picWidth; }
	Int  getPicHeight()                   { return m_picHeight; }
	Int  getLCUWidth()                    { return m_LCUWidth; }
	Int  getLCUHeight()                   { return m_LCUHeight; }
	Bool getUseLCUSeparateModel()         { return m_useLCUSeparateModel; }
	Int* getBitRatio()                    { return m_weight4bpp; }
	Int  getBitRatio(Int idx)           { assert(idx<8); return m_weight4bpp[idx]; }
	Int* getGOPID2Level()                 { return m_GOPID2Level; }
	Int  getGOPID2Level( Int ID )         { assert( ID < 8 ); return m_GOPID2Level[ID]; }

	Int  getTargetBitsForIDR()                   { return m_TargetBitsForIDR; }

	TRCParameter*  getPicPara()                                   { return m_picPara; }
	TRCParameter   getPicPara(Int level)                        { assert(level < 5); return m_picPara[level]; }
	Void           setPicPara(Int level, TRCParameter para)     { assert(level < 5); m_picPara[level] = para; }
	TRCParameter** getLCUPara()                                   { return m_LCUPara; }
	TRCParameter*  getLCUPara(Int level)                        { assert(level < 5); return m_LCUPara[level]; }
	TRCParameter   getLCUPara(Int level, Int LCUIdx)            { assert(level < 5); assert(LCUIdx < m_numberOfLCU); return m_LCUPara[level][LCUIdx]; }
	Void           setLCUPara(Int level, Int LCUIdx, TRCParameter para) { assert(level < 5); assert(LCUIdx < m_numberOfLCU); m_LCUPara[level][LCUIdx] = para; }

	Double getAlphaUpdate()               { return m_alphaUpdate; }
	Double getBetaUpdate()                { return m_betaUpdate; }

// 	TRCPic** getTRCPic()                                        { return m_Pics; }
// 	TRCPic* getTRCPic(Int Idridx)                                        { return m_Pics[Idridx]; }
 	TRCPic* getTRCPic(Int POCIdx)                            { return &m_Pics[POCIdx]; }


	Int			xEstimateVirtualBuffer(Int iIDRModulus);

#if (ETRI_DLL_INTERFACE)
#if !KAIST_RC
#if 0
  bool 		ETRI_getRCRestart()	 {return em_bRCRestart;}
  Void		ETRI_setRCRestart(bool bValue, int iTotalFrame); 
#endif
#endif
#endif

public:
  Int IDRnum;

	Int m_numTile;
	Int m_totalFrames;
	Int m_targetRate;
	Int m_frameRate;
	Int m_intraSize;
	Int m_picWidth;
	Int m_picHeight;
	Int m_LCUWidth;
	Int m_LCUHeight;

	Int m_numFrameLevel;

	Int m_numberOfPixel;
	Int64 m_targetBits;
	Int m_numberOfLCU;
	Double m_seqTargetBpp;
	Double m_alphaUpdate;
	Double m_betaUpdate;
	Bool m_useLCUSeparateModel;

	TRCParameter*  m_picPara;
	TRCParameter** m_LCUPara;


  Double m_bpp;
  Int m_weight4bpp[5];
  Int m_GOPID2Level[8];
  Int m_TargetBitsForIDR;

  Int KAIST_NUM_IDR_ENC;

  TRCPic* m_Pics;

  Int* m_tileIdxMap;

public:
	Int* m_sliceActualBits;//모든 타일의 출력 비트를 더한 값
	Int* m_sliceTotalTargetBits;
	Int* m_cpbState;
	Int* m_cpbStateFlag;
	UInt m_cpbSize;
	UInt m_bufferingRate;

#if KAIST_USEPREPS
	Double *m_costPOC;// [KAIST_NUM_FRAME_IDR];
	Double m_CostIDR;
	Double *m_CostGOP;
	//Int ****m_mv;// [KAIST_NUM_IDR_ENC][KAIST_NUM_FRAME_IDR][CTU][x/y];
#if KAIST_SCENECHANGE
	Int m_iSceneChange;
#endif
#endif

#if (ETRI_DLL_INTERFACE)
  bool 	em_bRCRestart;
#endif
};

#endif


#endif
