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

/** \file     TEncSearch.h
    \brief    encoder search class (header)
*/

#ifndef __TENCSEARCH__
#define __TENCSEARCH__

// Include files
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TLibCommon/TComPattern.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComPic.h"
#include "TEncEntropy.h"
#include "TEncSbac.h"
#include "TEncCfg.h"

//! \ingroup TLibEncoder
//! \{

class TEncCu;

// ====================================================================================================================
// ETRI definition
// ====================================================================================================================
#if ETRI_FAST_INTEGERME
#define	IME_SETDEBUG			0
#define	IME_FIRST  				(1 + IME_SETDEBUG)
#define	IME_OTHOGONAL_CCW 	(1 + IME_FIRST)
#define	IME_OTHOGONAL_CW		(1 + IME_OTHOGONAL_CCW)
#define	IME_MEANVALUE_BEGIN	(1 + IME_OTHOGONAL_CW)
#define	IME_MEANVALUE_PROC	(1 + IME_MEANVALUE_BEGIN)
#define	IME_MEANVALUE_FINAL	(1 + IME_MEANVALUE_PROC)
#define	IME_RASTER				(1 + IME_MEANVALUE_FINAL)
#define	IME_RASTER_FINAL		(1 + IME_RASTER)

#define	IME_FINAL				(1 + IME_RASTER_FINAL)

#define	IME_FAST_BOUNDARY		0
#define	IME_FAST_INTERNEL		1
#define	IME_SPEC_MEANVALUE	2

#define	IDX 						0
#define	IDY 						1
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder search class
class TEncSearch : public TComPrediction
{
private:
  TCoeff**        m_ppcQTTempCoeffY;
  TCoeff**        m_ppcQTTempCoeffCb;
  TCoeff**        m_ppcQTTempCoeffCr;
  TCoeff*         m_pcQTTempCoeffY;
  TCoeff*         m_pcQTTempCoeffCb;
  TCoeff*         m_pcQTTempCoeffCr;
#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
  Int**           m_ppcQTTempArlCoeffY;
  Int**           m_ppcQTTempArlCoeffCb;
  Int**           m_ppcQTTempArlCoeffCr;
  Int*            m_pcQTTempArlCoeffY;
  Int*            m_pcQTTempArlCoeffCb;
  Int*            m_pcQTTempArlCoeffCr;
#endif
#endif
  UChar*          m_puhQTTempTrIdx;
  UChar*          m_puhQTTempCbf[3];
  
  TComYuv*        m_pcQTTempTComYuv;
  TComYuv         m_tmpYuvPred; // To be used in xGetInterPredictionError() to avoid constant memory allocation/deallocation
  Pel*            m_pSharedPredTransformSkip[3];
  TCoeff*         m_pcQTTempTUCoeffY;
  TCoeff*         m_pcQTTempTUCoeffCb;
  TCoeff*         m_pcQTTempTUCoeffCr;
  UChar*          m_puhQTTempTransformSkipFlag[3];
  TComYuv         m_pcQTTempTransformSkipTComYuv;
#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
  Int*            m_ppcQTTempTUArlCoeffY;
  Int*            m_ppcQTTempTUArlCoeffCb;
  Int*            m_ppcQTTempTUArlCoeffCr;
#endif
#endif

// ====================================================================================================================
//	ETRI Variables as Private
// ====================================================================================================================
#if ETRI_MODIFICATION_V02
  TComYuv*** em_pppcAxPredYuvTemp; 		///< 2014 7 17 by Seok : Temporary Prediction Yuv for for each Mode [Mode][depth] : here mode is MAX Chroma Mode
  UInt  	em_uiNumComponent;  		/// = mode for em_pppcAxPredYuvTemp : NUM_CHROMA_MODE 2015 9 12 by Seok 
  UInt  	em_uiTotalDepth; 			/// = Depth for em_pppcAxPredYuvTemp : g_uiMaxCUDepth;

  UInt*  	em_puiHADCost;  			///< INTRA : LUMA, CHromaU, ChromaV + 1 temp
  UInt*  	em_puiRdModeList;  			///< INTRA : [FAST_UDI_MAX_RDMODE_NUM];
  UInt*  	em_puiChromaModeList; 		///< INTRA : [FAST_UDI_MAX_RDMODE_NUM];
  UInt  	em_numModesForFullRD;
  UInt  	em_uiDMChromaModePred; 		///< INTRA : DM Mode in Prediction Stage. Accoring to em_puiChromaModeList

  UInt*  	em_puiDBGInfo;  			///< Debug Signal from outside of this class

#if 0
  TComMv* 	em_apSkipMergeBestMv;		///< Fast AMVP selection 
  Int*  	em_piRefIndex; 	      		///< Fasy AMVP selection 
#endif
    
#endif
// -----------------------------------------------------------------------------------------------------------------------------------------------------

protected:
  // interface to option
  TEncCfg*        m_pcEncCfg;
  
  // interface to classes
  TComTrQuant*    m_pcTrQuant;
  TComRdCost*     m_pcRdCost;
  TEncEntropy*    m_pcEntropyCoder;
  
  // ME parameters
  Int             m_iSearchRange;
  Int             m_bipredSearchRange; // Search range for bi-prediction
  Int             m_iFastSearch;
  Int             m_aaiAdaptSR[2][33];
  TComMv          m_cSrchRngLT;
  TComMv          m_cSrchRngRB;
  TComMv          m_acMvPredictors[3];
  
  // RD computation
  TEncSbac***     m_pppcRDSbacCoder;
  TEncSbac*       m_pcRDGoOnSbacCoder;
  DistParam       m_cDistParam;
  
  // Misc.
  Pel*            m_pTempPel;
  const UInt*     m_puiDFilter;
  Int             m_iMaxDeltaQP;
  
  // AMVP cost computation
  // UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS];
  UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds
  
public:
  TEncSearch();
  virtual ~TEncSearch();
  
  Void init(  TEncCfg*      pcEncCfg,
            TComTrQuant*  pcTrQuant,
            Int           iSearchRange,
            Int           bipredSearchRange,
            Int           iFastSearch,
            Int           iMaxDeltaQP,
            TEncEntropy*  pcEntropyCoder,
            TComRdCost*   pcRdCost,
            TEncSbac***   pppcRDSbacCoder,
            TEncSbac*     pcRDGoOnSbacCoder );
  
 // ====================================================================================================================
 // ETRI Public Functions: 
 // ====================================================================================================================
 Void ETRI_Init(UInt uiMaxWidth, UInt uiMaxHeight);
 
 /// This Function is very Important to prevent Memory Leak when Multiple Threads for Tile Application @ 2015 5 19 by Seok
 Void ETRI_destroy();	

 Void ETRI_IntraLumaPred 		( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, UInt*& uiRdModeList);
 Void ETRI_IntraLumaPred_V2		( TComDataCU*& pcCU, TComYuv* pcOrgYuv, TComYuv*& pcPredYuv, UInt uiAbsPartIdx);

 Void ETRI_IntraChromaPred  		( TComDataCU* pcCU, TComYuv*    pcOrgYuv, TComYuv* pcPredYuv, bool bPurePred);
 
 Void ETRI_IntraPrediction 		( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv);
 
 /// Test Function For Intra Division
 Void ETRI_estIntraPredQT 		( TComDataCU* pcCU, 
								TComYuv*	pcOrgYuv, 
								TComYuv*	pcPredYuv, 
								TComYuv*	pcResiYuv, 
								TComYuv*	pcRecoYuv,
								UInt&		ruiDistC,
								Bool		bLumaOnly );

Void ETRI_xIntraCodingLumaBlk( TComDataCU* pcCU,
								 UInt		 uiTrDepth,
								 UInt		 uiAbsPartIdx,
								 TComYuv*	 pcOrgYuv, 
								 TComYuv*	 pcPredYuv, 
								 TComYuv*	 pcResiYuv, 
								 UInt&		 ruiDist,
								 Int		default0Save1Load2 );


 Void ETRI_xRecurIntraCodingQT( TComDataCU*	pcCU, 
								 UInt		  uiTrDepth,
								 UInt		  uiAbsPartIdx, 
								 Bool		  bLumaOnly,
								 TComYuv*	  pcOrgYuv, 
								 TComYuv*	  pcPredYuv, 
								 TComYuv*	  pcResiYuv, 
								 UInt&		  ruiDistY,
								 UInt&		  ruiDistC,
								 Bool		  bCheckFirst,
								 Double&	  dRDCost );


 Void ETRI_IntraLumaPred_AX( TComDataCU* pcCU,
								   UInt 	   uiTrDepth,
								   UInt 	   uiAbsPartIdx,
								   TComYuv*    pcOrgYuv, 
								   TComYuv*&   pcPredYuv);

 Void ETRI_IntraChromaPred_AX 		( TComDataCU* pcCU,
								  UInt		  uiTrDepth,
								  UInt		  uiAbsPartIdx,
								  TComYuv*	  pcOrgYuv, 
								  TComYuv*&	  pcPredYuv, 
								  UInt		  uiChromaId);
 
///< This function is for analysis @2015 8 29 by Seok
 Void ETRI_xIntraCodingChromaBlk( TComDataCU* pcCU,
								UInt 	   uiTrDepth,
								UInt 	   uiAbsPartIdx,
								TComYuv*    pcOrgYuv, 
								TComYuv*    pcPredYuv, 
								TComYuv*    pcResiYuv, 
								UInt&	   ruiDist,
								UInt 	   uiChromaId,
								Int	   default0Save1Load2 = 0 );


 Void ETRI_estIntraPredChromaQT	( TComDataCU* pcCU, 
								TComYuv*	  pcOrgYuv, 
								TComYuv*	  pcPredYuv, 
								TComYuv*	  pcResiYuv, 
								TComYuv*	  pcRecoYuv,
								UInt		  uiPreCalcDistC );

 /// ETRI_MODIFICATION_V02 Functions  
 Void ETRI_predInterSearch 		( TComDataCU* pcCU,
								TComYuv*    pcOrgYuv,
								TComYuv*&   rpcPredYuv,
								Bool        bUseRes = false,
								Bool        bUseSliceEncoderMVClip = false
								,Bool        bUseMRG = false
                             				  );

/// ETRI_MODIFICATION_V03 Functions  
Void ETRI_xEstimateMvPredAMVP 	( TComDataCU* pcCU, 
									TComYuv* pcOrgYuv, 
									UInt uiPartIdx, 
									RefPicList eRefPicList, 
									Int iRefIdx, 
									TComMv& rcMvPred, 
									Bool bFilled, 
									UInt* puiDistBiP,
									Bool* bNoMVP);


Void ETRI_xMotionEstimation  	( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, 
									RefPicList eRefPicList, TComMv* pcMvPred, Int iRefIdxPred, TComMv& rcMv, UInt& ruiBits, UInt& ruiCost, Bool bBi = false);
Void ETRI_xTZSearch  			( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, 
									TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD );
Void ETRI_xPatternSearch 		(TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD );

#if ETRI_MODIFICATION_V02
/// Service Function when ETRI_MODIFICATION_V02 is active @ 2015 8 29 by Seok
#if ETRI_NO_RATE_CALC_IN_RMD
UInt ETRI_xUpdateCandList(UInt uiMode, Double uiCost, UInt uiHADCost, UInt uiFastCandNum, UInt * CandModeList,
    Double * CandCostList, UInt * ModeBitList, UInt * HADCostList);
#else 
UInt ETRI_xUpdateCandList 	( UInt uiMode, Double uiCost, UInt uiHADCost,  Int iModeBits, UInt uiFastCandNum, UInt * CandModeList, 
								Double * CandCostList,  UInt * ModeBitList, UInt * HADCostList  );
#endif 
Void ETRI_SliceEncoder_CheckMVP(AMVPInfo* pcAMVPInfo, TComDataCU* pcCU, Int iHeight, Bool* bcliped);

///< For INTRA Prediction :: INTRA Best LUMA, Chroma HAD value 
UInt* ETRI_getHADCost		()	{return em_puiHADCost;}	

/// ETRI Service Function For Debug
Void ETRI_setDBGInfo  		(UInt* puiValue){em_puiDBGInfo = puiValue;}
UInt ETRI_getDBGInfo  		() 	{return (em_puiDBGInfo? *em_puiDBGInfo : 0);}

#endif
// ====================================================================================================================


protected:
  
  /// sub-function for motion vector refinement used in fractional-pel accuracy
  UInt  xPatternRefinement( TComPattern* pcPatternKey,
                           TComMv baseRefMv,
                           Int iFrac, TComMv& rcMvFrac );
#if ETRI_MODIFICATION_V02
  /// ETRI_SliceEncoder_MVClip adopted motion vector refinement used in fractional-pel accuracy
  UInt  ETRI_SliceEncoderMVClip_PatternRefinementBoundary(TComPattern* pcPatternKey,
	  TComMv baseRefMv, Int iFrac, TComMv& rcMvFrac, bool bVerMax, bool bVerMin);
#endif
  
  Void xGetInterPredictionError(TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, UInt& ruiSAD, Bool Hadamard);

  typedef struct
{
	Pel*  piRefY;
	Int   iYStride;
	Int   iBestX;
	Int   iBestY;
	UInt  uiBestRound;
	UInt  uiBestDistance;
	UInt  uiBestSad;
	UChar ucPointNr;

#if ETRI_FAST_INTEGERME
	Char	e_ucLimit[4];
	Bool	bCenterOffset;
	Bool	bFMEEnable;
#endif
} IntTZSearchStruct;
  
  // sub-functions for ME
  __inline Void xTZSearchHelp         ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance );
  __inline Void xTZ2PointSearch       ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB );
  __inline Void xTZ8PointSquareSearch ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist );
  __inline Void xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist );

#if ETRI_FAST_INTEGERME
  // ====================================================================================================================
  /// ETRI_MODIFICATION_V03 Functions  
  // ====================================================================================================================
  __inline Void ETRI_DEBUG_IME  	( TComDataCU* pcCU, IntTZSearchStruct cDbgStruct, 
										TComMv PredMv, Int e_iIteration, Int e_iStartX, Int e_iStartY, UInt uiRevisedSAD, Int iDeltaSAD,
										Bool bDbgCond, UInt ProcID);
  __inline Void ETRI_CopySearchStruct 		( IntTZSearchStruct& rDstStruct, IntTZSearchStruct rSrcStruct, Bool e_bReset);
  __inline Void ETRI_8x9ClipMv  			( IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, Int*& iSrchRng);
  __inline Void ETRI_UpdateStartPoint 		( IntTZSearchStruct& rcStruct, Int& e_iStartX, Int& e_iStartY, Int e_iSearchOffset, Int iOPStage = IME_FAST_BOUNDARY);
  __inline Void ETRI_8x9Search  			( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, Int* iSrchRng, const Int iStartX, const Int iStartY, Int& uiIteration );
  
  // ====================================================================================================================
  /// ETRI_MODIFICATION_V03 Private Parameters
  // ====================================================================================================================
private:
	IntTZSearchStruct	em_SearchStruct; 	///< For FME and IME Debug 2015 10 9 by Seok

#endif
// ====================================================================================================================

public:
  Void  preestChromaPredMode    ( TComDataCU* pcCU, 
                                  TComYuv*    pcOrgYuv, 
                                  TComYuv*    pcPredYuv );
  Void  estIntraPredQT          ( TComDataCU* pcCU, 
                                  TComYuv*    pcOrgYuv, 
                                  TComYuv*    pcPredYuv, 
                                  TComYuv*    pcResiYuv, 
                                  TComYuv*    pcRecoYuv,
                                  UInt&       ruiDistC,
                                  Bool        bLumaOnly );
  Void  estIntraPredChromaQT    ( TComDataCU* pcCU, 
                                  TComYuv*    pcOrgYuv, 
                                  TComYuv*    pcPredYuv, 
                                  TComYuv*    pcResiYuv, 
                                  TComYuv*    pcRecoYuv,
                                  UInt        uiPreCalcDistC );
  
  
  /// encoder estimation - inter prediction (non-skip)
  Void predInterSearch          ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*&   rpcPredYuv,
                                  TComYuv*&   rpcResiYuv,
                                  TComYuv*&   rpcRecoYuv,
                                  Bool        bUseRes = false
#if AMP_MRG
                                 ,Bool        bUseMRG = false
#endif
                                );
  
  /// encode residual and compute rd-cost for inter mode
  Void encodeResAndCalcRdInterCU( TComDataCU* pcCU,
                                  TComYuv*    pcYuvOrg,
                                  TComYuv*    pcYuvPred,
                                  TComYuv*&   rpcYuvResi,
                                  TComYuv*&   rpcYuvResiBest,
                                  TComYuv*&   rpcYuvRec,
                                  Bool        bSkipRes );
  
  /// set ME search range
  Void setAdaptiveSearchRange   ( Int iDir, Int iRefIdx, Int iSearchRange) { m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }
  
  Void xEncPCM    (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPCM, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, TextType eText);
  Void IPCMSearch (TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv );
protected:
  
  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------
  
  Void  xEncSubdivCbfQT           ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLuma,
                                    Bool         bChroma );

  Void  xEncCoeffQT               ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    TextType     eTextType,
                                    Bool         bRealCoeff );
  Void  xEncIntraHeader           ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLuma,
                                    Bool         bChroma );
  UInt  xGetIntraBitsQT           ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLuma,
                                    Bool         bChroma,
                                    Bool         bRealCoeff );
  UInt  xGetIntraBitsQTChroma    ( TComDataCU*   pcCU,
                                   UInt          uiTrDepth,
                                   UInt          uiAbsPartIdx,
                                   UInt          uiChromaId,
                                   Bool          bRealCoeff );
  
  Void  xIntraCodingLumaBlk       ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    TComYuv*     pcOrgYuv, 
                                    TComYuv*     pcPredYuv, 
                                    TComYuv*     pcResiYuv, 
                                    UInt&        ruiDist,
                                    Int         default0Save1Load2 = 0);
  Void  xIntraCodingChromaBlk     ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    TComYuv*     pcOrgYuv, 
                                    TComYuv*     pcPredYuv, 
                                    TComYuv*     pcResiYuv, 
                                    UInt&        ruiDist,
                                    UInt         uiChromaId,
                                    Int          default0Save1Load2 = 0 );

  Void  xRecurIntraCodingQT       ( TComDataCU*  pcCU, 
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx, 
                                    Bool         bLumaOnly,
                                    TComYuv*     pcOrgYuv, 
                                    TComYuv*     pcPredYuv, 
                                    TComYuv*     pcResiYuv, 
                                    UInt&        ruiDistY,
                                    UInt&        ruiDistC,
#if HHI_RQT_INTRA_SPEEDUP
                                   Bool         bCheckFirst,
#endif
                                   Double&      dRDCost );
  
  Void  xSetIntraResultQT         ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLumaOnly,
                                    TComYuv*     pcRecoYuv );
  
  Void  xRecurIntraChromaCodingQT ( TComDataCU*  pcCU, 
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx, 
                                    TComYuv*     pcOrgYuv, 
                                    TComYuv*     pcPredYuv, 
                                    TComYuv*     pcResiYuv, 
                                    UInt&        ruiDist );
  Void  xSetIntraResultChromaQT   ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    TComYuv*     pcRecoYuv );
  
  Void  xStoreIntraResultQT       ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLumaOnly );
  Void  xLoadIntraResultQT        ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLumaOnly );
  Void xStoreIntraResultChromaQT  ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    UInt         stateU0V1Both2 );
  Void xLoadIntraResultChromaQT   ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    UInt         stateU0V1Both2 );

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------
  
  Void xEstimateMvPredAMVP        ( TComDataCU* pcCU,
                                    TComYuv*    pcOrgYuv,
                                    UInt        uiPartIdx,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    TComMv&     rcMvPred,
                                    Bool        bFilled = false
                                  , UInt*       puiDistBiP = NULL
                                  #if ZERO_MVD_EST
                                  , UInt*       puiDist = NULL
                                  #endif
                                     );
  
  Void xCheckBestMVP              ( TComDataCU* pcCU,
                                    RefPicList  eRefPicList,
                                    TComMv      cMv,
                                    TComMv&     rcMvPred,
                                    Int&        riMVPIdx,
                                    UInt&       ruiBits,
                                    UInt&       ruiCost );
  
  UInt xGetTemplateCost           ( TComDataCU* pcCU,
                                    UInt        uiPartIdx,
                                    UInt        uiPartAddr,
                                    TComYuv*    pcOrgYuv,
                                    TComYuv*    pcTemplateCand,
                                    TComMv      cMvCand,
                                    Int         iMVPIdx,
                                    Int         iMVPNum,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    Int         iSizeX,
                                    Int         iSizeY
                                  #if ZERO_MVD_EST
                                  , UInt&       ruiDist
                                  #endif
                                   );
  
  
  Void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  UInt xGetMvpIdxBits             ( Int iIdx, Int iNum );
  Void xGetBlkBits                ( PartSize  eCUMode, Bool bPSlice, Int iPartIdx,  UInt uiLastMode, UInt uiBlkBit[3]);
  
  Void xMergeEstimation           ( TComDataCU*     pcCU,
                                    TComYuv*        pcYuvOrg,
                                    Int             iPartIdx,
                                    UInt&           uiInterDir,
                                    TComMvField*    pacMvField,
                                    UInt&           uiMergeIndex,
                                    UInt&           ruiCost
                                  , TComMvField* cMvFieldNeighbours,  
                                    UChar* uhInterDirNeighbours,
                                    Int& numValidMergeCand
                                   );

  Void xRestrictBipredMergeCand   ( TComDataCU*     pcCU,
                                    UInt            puIdx,
                                    TComMvField*    mvFieldNeighbours, 
                                    UChar*          interDirNeighbours, 
                                    Int             numValidMergeCand );

  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------
  
  Void xMotionEstimation          ( TComDataCU*   pcCU,
                                    TComYuv*      pcYuvOrg,
                                    Int           iPartIdx,
                                    RefPicList    eRefPicList,
                                    TComMv*       pcMvPred,
                                    Int           iRefIdxPred,
                                    TComMv&       rcMv,
                                    UInt&         ruiBits,
                                    UInt&         ruiCost,
                                    Bool          bBi = false  );
  
  Void xTZSearch                  ( TComDataCU*   pcCU,
                                    TComPattern*  pcPatternKey,
                                    Pel*          piRefY,
                                    Int           iRefStride,
                                    TComMv*       pcMvSrchRngLT,
                                    TComMv*       pcMvSrchRngRB,
                                    TComMv&       rcMv,
                                    UInt&         ruiSAD );
  
  Void xSetSearchRange            ( TComDataCU*   pcCU,
                                    TComMv&       cMvPred,
                                    Int           iSrchRng,
                                    TComMv&       rcMvSrchRngLT,
                                    TComMv&       rcMvSrchRngRB );
  
  Void xPatternSearchFast         ( TComDataCU*   pcCU,
                                    TComPattern*  pcPatternKey,
                                    Pel*          piRefY,
                                    Int           iRefStride,
                                    TComMv*       pcMvSrchRngLT,
                                    TComMv*       pcMvSrchRngRB,
                                    TComMv&       rcMv,
                                    UInt&         ruiSAD );
  
  Void xPatternSearch             ( TComPattern*  pcPatternKey,
                                    Pel*          piRefY,
                                    Int           iRefStride,
                                    TComMv*       pcMvSrchRngLT,
                                    TComMv*       pcMvSrchRngRB,
                                    TComMv&       rcMv,
                                    UInt&         ruiSAD );
  
  Void xPatternSearchFracDIF      ( TComDataCU*   pcCU,
									TComPattern*  pcPatternKey,
									Pel*          piRefY,
									Int           iRefStride,
									TComMv*       pcMvInt,
									TComMv&       rcMvHalf,
#if !ETRI_NOT_QUARTERPEL_ME
									TComMv&       rcMvQter,
#endif 
                                    UInt&         ruiCost 
                                   ,Bool biPred
									);

  Void xExtDIFUpSamplingH( TComPattern* pcPattern, Bool biPred  );
  Void xExtDIFUpSamplingQ( TComPattern* pcPatternKey, TComMv halfPelRef, Bool biPred );


  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------
  
  Void xEncodeResidualQT( TComDataCU* pcCU, UInt uiAbsPartIdx, const UInt uiDepth, Bool bSubdivAndCbf, TextType eType );
  Void xEstimateResidualQT( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, UInt absTUPartIdx,TComYuv* pcResi, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist, UInt *puiZeroDist );
  Void xSetResidualQTData( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx,UInt absTUPartIdx, TComYuv* pcResi, UInt uiDepth, Bool bSpatial );
  
  UInt  xModeBitsIntra ( TComDataCU* pcCU, UInt uiMode, UInt uiPU, UInt uiPartOffset, UInt uiDepth, UInt uiInitTrDepth );
  UInt  xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList );
  
  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------
  
  Void xAddSymbolBitsInter        ( TComDataCU*   pcCU,
                                   UInt          uiQp,
                                   UInt          uiTrMode,
                                   UInt&         ruiBits,
                                   TComYuv*&     rpcYuvRec,
                                   TComYuv*      pcYuvPred,
                                   TComYuv*&     rpcYuvResi );
  
  Void  setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur );
  inline  Void  setDistParamComp( UInt uiComp )  { m_cDistParam.uiComp = uiComp; }
  
};// END CLASS DEFINITION TEncSearch

//! \}

#endif // __TENCSEARCH__
