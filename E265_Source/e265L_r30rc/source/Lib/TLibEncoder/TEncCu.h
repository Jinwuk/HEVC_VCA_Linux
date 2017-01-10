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

/** \file     TEncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __TENCCU__
#define __TENCCU__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComDataCU.h"

#include "TEncEntropy.h"
#include "TEncSearch.h"
#include "TEncRateCtrl.h"
//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncSbac;
class TEncCavlc;
class TEncSlice;
#if ETRI_MULTITHREAD_2
class TEncFrame;	// gplusplus
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================


/// CU encoder class
class TEncCu
{
private:

  TComDataCU**  			m_ppcBestCU;  		///< Best CUs in each depth
  TComDataCU**  			m_ppcTempCU; 		///< Temporary CUs in each depth
  UChar  					m_uhTotalDepth;

  TComYuv**     			m_ppcPredYuvBest; 	///< Best Prediction Yuv for each depth
  TComYuv**     			m_ppcResiYuvBest; 	///< Best Residual Yuv for each depth
  TComYuv**     			m_ppcRecoYuvBest; 	///< Best Reconstruction Yuv for each depth
  TComYuv**     			m_ppcPredYuvTemp; 	///< Temporary Prediction Yuv for each depth
  TComYuv**     			m_ppcResiYuvTemp; 	///< Temporary Residual Yuv for each depth
  TComYuv**     			m_ppcRecoYuvTemp; 	///< Temporary Reconstruction Yuv for each depth
  TComYuv**     			m_ppcOrigYuv;  		///< Original Yuv for each depth

  //  Data : encoder control
  Bool  					m_bEncodeDQP;

  //  Access channel
  TEncCfg*  				m_pcEncCfg;
  TEncSearch*				m_pcPredSearch;
  TComTrQuant*  			m_pcTrQuant;
  TComBitCounter*		  	m_pcBitCounter;
  TComRdCost*  			m_pcRdCost;

  TEncEntropy*  			m_pcEntropyCoder;
  TEncCavlc* 				m_pcCavlcCoder;
  TEncSbac*  				m_pcSbacCoder;
  TEncBinCABAC*  			m_pcBinCABAC;

  // SBAC RD
  TEncSbac***				m_pppcRDSbacCoder;
  TEncSbac*  				m_pcRDGoOnSbacCoder;
#if KAIST_RC
  TEncRateCtrl*  			m_pcRateCtrl;
#endif

  // ====================================================================================================================
  // ETRI Private Data : All Data is operated under ETRI_MODIFICATION_V02
  // ====================================================================================================================
  TComDataCU***		  	m_pppcAxTempCU;  			///< 2014 7 17 by Seok : Auxiliary TempCU for each Mode [Mode][depth] 	
  TComYuv***			  	m_pppcAxPredYuvTemp;		///< 2014 7 17 by Seok : Temporary Prediction Yuv for for each Mode [Mode][depth]
  TComYuv***			  	m_pppcAxRecoYuvTemp;		///< Temporary Reconstruction Yuv for each depth [Mode][depth]

  Bool  					em_bLCUSkipFlag;			///< Indicate whether skip 64x64 or not.
  Bool*  					em_bESD;					///< Global Variable of ESD for ETRI_xCheckEarlySkipDecision @ 2015 9 3 by Seok

  Bool*  					em_bControlParam;			///< 2014 8 4 by Seok : Look at ETRI_HEVC_define.h
  Bool** 					em_bDepthSkipMode;			///< Main Control Parameter for PU Processing (Prediction & RDOQ). [Depth]{Mode]
  Bool*  					em_bSkipMode;				///< MAIN CONTROL PARAMETER for PU Processing (Prediction & RDOQ). Indicates which Mode is skipped 
  Bool*					em_pbdoNotBlockPU;			///< Global Sensing of doNotBlockPU 

  Bool  					em_bActiveCheckBestCU;		///< For ETRI_xCheckBestMode : TRUE : Exchange TempCU to Best CU, False : No Exchange 2014 7 3 by Seok 
  Bool  					em_bActivexCheckETRIBestMode;

  UInt*				  	em_uiLevelInfo;		  		///< 2015 3 23 by Seok : Monitoring of HAD Level per each color component : 
  ///< [0] : Width, [1]: Slice Depth, [2] : Depth [3]: Level Luma [4] Level Cb [5] Level Cr [6] HadLuma [7] HadCb [8] HadCr
  UInt**					em_uiHADInfo; 				///< 2015 3 23 by Seok : Monitoring of HAD value from em_uiLevelInfo for Fast Prediction [MODE][HAD LUMA/HAD Cb/HAD Cr]
  UChar					em_ucFPUInterSkipLevel[2];	///< Inter Skip Level From g_FPUInterSkipLevel :  2015 11 10 by Seok														

  //---------------------- SKIP/Merge ------------------------
  Int    					em_inumValidMergeCand;		///< For Lossless SKIP/MERGE Prediction/RDOQ
  Int    					em_iRDOOffBestMergeCand;	///< Index of Best Merge Cand : In SliceEncoder_MVClip's Algorithm : Index of Merg/Skip Cand, In case of -1, no Valid Candidate: In HM Algorithm If not, it is 0 amd negligible
  Int*  					em_pimergeCandBuffer;		///< Size : [MRG_MAX_NUM_CANDS]; For Lossless SKIP/MERGE Prediction/RDOQ
  UChar*					em_puhInterDirNeighbours;	///< Size : [MRG_MAX_NUM_CANDS];
  TComMvField*  			em_pcMvFieldNeighbours;		///< Size : [MRG_MAX_NUM_CANDS << 1]


  UInt  					em_uiSMHADDistortion;		///< HAD value  2015 9 2 by Seok		
  UInt						em_uiITHADDistortion;
  UInt						em_uiINHADDistortion;
  UInt*					em_uiMINHADDistortion;		///< Minimum HAD Distortion for Prediction Stage [Depth] @ 2015 11 19 by Seok

  UInt*				  	em_uiSKLevel;   				///< Level For SKIP	2015 9 20 by Seok				
  UInt*				  	em_uiMGLevel;   				///< Level For Merge 2015 9 20 by Seok
  UInt*				  	em_uiINLevel;   				///< Level For Inter 2015 9 20 by Seok
  UInt*				  	em_uiITLevel;  				///< Level For INTRA 2015 9 20 by Seok
  UInt*				  	em_uiMINLevel; 				///< Level For INTRA 2015 9 20 by Seok

  //------------CU Depth Processing/ CU Depth Prunning -----------
  UInt*  					em_uiPartUnitIdx;  			///< Indicate PartUnit 2015 3 24 by Seok : 0 ~ 3 Range 
  UInt* 					em_uiACCPartDistortion;		///< 2015 11 23 by Seok	
  Double*					em_dACCPartRDCost;			///< 2015 3 24 by Seok : DP Prunning based on RD-Cost : Accumulation of RD-cost as to Partitions in CU [Depth]
  Double*					em_pdACCEstRDCost;	  		///< 2015 3 19 by Seok : Accumulate Estimated RD Cost per each Depth	  [Depth]  

  QpParam*				em_qp;					///< For Adaptive Fast Encoding 2015 9 17 by Seok
  TComDataCU**			em_pcBestUpCU;				///< For Fast Prediction Mode SKIP [Depth](pcCu Pointer) @ 2015 11 20 by Seok

  //-------------------------Misc  -------------------------


  //-------------------------Debug  -------------------------
#if ETRI_DEBUG_CODE_CLEANUP
  UInt*  					em_DbgInfo;					///<  Size: [ETRI_nDbgInfo] : Main Debug Data (UInt Type) for TEncCU @2015 3 25 by Seok 
#endif 
#if ETRI_CU_MODE_INHERITANCE
  Bool					em_bDoInterModeFlag;
#endif 
  // ====================================================================================================================

public:
  /// copy parameters from encoder class
#if ETRI_MULTITHREAD_2
  Void  init(TEncTop* pcEncTop, TEncFrame* pcEncFrame);	// gplusplus
#else
  Void  init                ( TEncTop* pcEncTop );
#endif

  /// create internal buffers
  Void  create(UChar uhTotalDepth, UInt iMaxWidth, UInt iMaxHeight);

  /// destroy internal buffers
  Void  destroy();

  /// CU analysis function
  Void  compressCU(TComDataCU*&  rpcCU);

  /// CU encoding function
  Void  encodeCU(TComDataCU*    pcCU);

  Void setBitCounter(TComBitCounter* pcBitCounter) { m_pcBitCounter = pcBitCounter; }
  Int   updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height);

  // ====================================================================================================================
  // ETRI Public Functions: 
  // ====================================================================================================================
  void ETRI_createCU(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight);
  void ETRI_destroyCU();

  Void ETRI_initForTiles(TEncTop* pcEncTop,
    TEncSearch*   	pcPredSearch,
    TComTrQuant*  	pcTrQuant,
    TComBitCounter*	pcBitCounter,
    TEncEntropy*  	pcEntropyCoder,
    TComRdCost*   	pcRdCost,
    TEncSbac***   	pppcRDSbacCoder,
    TEncSbac* 		pcRDGoOnSbacCoder);	///< For Multiple Thread Tile @ 2015 5 19 by Seok

  Void ETRI_initForTiles(TEncCavlc*  	pcCavlcCoder,
    TEncSbac*  		pcSbacCoder,
    TEncBinCABAC* 	pcBinCABAC
#if KAIST_RC
    , TEncRateCtrl* 	pcRateCtrl	///< For Rate Control @ 2015 5 19 by Seok
#endif
    );

	Void ETRI_Init_TileLevel			();  	///< Tile Level Initilization 
	Void ETRI_Release_TileLevel	 		();
	Void ETRI_Init_onPictureSliceTile 	(TComSlice* pcSlice);
	Void ETRI_Final_onPictureSliceTile	(TComSlice* pcSlice, UInt uiTileIdx);

	__inline 	Void ETRI_Init_CUTopLevel  			(TComDataCU*& rpcCU);	///< ETRI's Initilization on Top Level/Depth 
	__inline 	Void ETRI_Init_SubCULevel  			(TComDataCU*& rpcBestCU, UInt uhNextDepth);
	__inline 	Void ETRI_Init_SubCUPartitionLevel	(TComDataCU*& rpcBestCU, UInt uiPartUnitIdx, Int iQP, UInt uhNextDepth, UInt uiDepth);
	__inline	Void ETRI_Init_CUPartitionLevel   	(TComDataCU*& rpcCU, UInt uiDepth);

#if ETRI_DEBUG_CODE_CLEANUP
	UInt* ETRI_getCUDebugData 	  		()			{return em_DbgInfo;}
#endif 
// ====================================================================================================================

#if KAIST_RC
	Void setRateCtrl(TEncRateCtrl* cRateCtrl) { m_pcRateCtrl = cRateCtrl; }
#endif

protected:
	Void  finishCU            ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );
#if AMP_ENC_SPEEDUP
	Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize = SIZE_NONE );
#else
	Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
	Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );

	Int   xComputeQP          ( TComDataCU* pcCU, UInt uiDepth );
	Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );

	Void  xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode);

#if AMP_MRG
	Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG = false  );
#else
	Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif
	Void  xCheckRDCostIntra   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
	Void  xCheckDQP           ( TComDataCU*  pcCU );

	Void  xCheckIntraPCM      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU                      );
	Void  xCopyAMVPInfo       ( AMVPInfo* pSrc, AMVPInfo* pDst );
	Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
	Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiDepth );

	Bool getdQPFlag           ()                        { return m_bEncodeDQP;        }
	Void setdQPFlag           ( Bool b )                { m_bEncodeDQP = b;           }

#if ADAPTIVE_QP_SELECTION
	// Adaptive reconstruction level (ARL) statistics collection functions
	Void xLcuCollectARLStats 	(TComDataCU* rpcCU);
	Int  xTuCollectARLStats  	(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples );
#endif

#if AMP_ENC_SPEEDUP 
#if AMP_MRG
	Void deriveTestModeAMP   	(TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver);
#else
	Void deriveTestModeAMP  	(TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver);
#endif
#endif

	Void  xFillPCMBuffer      	( TComDataCU*& pCU, TComYuv* pOrgYuv ); 

// ====================================================================================================================
// ETRI Protected Functions: 
// ====================================================================================================================
	
	Void	ETRI_xCompressCU					( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize = SIZE_NONE);

#if	ETRI_MODIFICATION_V02
	//----------------------------------------------------------------------------
	//	Removable Function 
	//----------------------------------------------------------------------------
	__inline Int 	ETRI_SetCUQP				( TComDataCU*& rpcTempCU, Int& iMinQP, Int& iMaxQP, Bool& isAddLowestQP, Int iBaseQP, UInt uiDepth, Bool e_bPreSet);
	__inline Void 	ETRI_InterPUBlockProcessing	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize, Bool& doNotBlockPu, Int iQP, Bool bIsLosslessMode);
	__inline void 	ETRI_TestPCMProcess    		( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComPic* pcPic, UInt uiDepth, Int iQP, Bool bIsLosslessMode);

	//----------------------------------------------------------------------------
	//	Developing and Changable Function 
	//----------------------------------------------------------------------------
	__inline void 	ETRI_Normal_INTERProcess 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP, Bool bIsLosslessMode, Bool&   earlyDetectionSkipMode, Bool& doNotBlockPu);
	__inline void 	ETRI_INTRAPrediction   	   	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP);
	__inline void 	ETRI_Normal_INTRAProcess 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP, Bool bIsLosslessMode);

	__inline void 	ETRI_SubCUControl  			( TComDataCU*& rpcBestCU, Bool& bSubBranch, Bool bInsidePicture, UInt uiDepth, Bool bBoundary);
	__inline Void 	ETRI_xControlPUProcessing 	( TComDataCU*& pcCU, UChar uhDepth, UInt PhaseIdx, UInt SubStage);
	__inline Void 	ETRI_xCheckSkipMerge_PRED 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth );
	__inline Void 	ETRI_xCheckOnlySKIP_RDOQ 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth);
	__inline Void 	ETRI_xCheckSkipMerge_RDOQ 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth);
	__inline void 	ETRI_xEarlyCheckSkipMerge_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth, Int iQP, Bool bIsLosslessMode, Bool bOP);
	__inline void 	ETRI_xEarlyCheckInter_RDOQ	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth, Int iQP, Bool bIsLosslessMode, Bool bOP);
	__inline void 	ETRI_xCheckInter_PRED  		( TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth, Int iQP);
	__inline void  	ETRI_xCheckInter_RDOQ  		( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth, Bool& doNotBlockPu);
	__inline void 	ETRI_xCheckIntra_PRED  		( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth,  Int iQP, bool earlyDetectionSkipMode, Bool SubStage);
	__inline void 	ETRI_xCheckIntra_RDOQ  		( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth);
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
    __inline void 	ETRI_xCheckInter2NxNNx2N_PRED(TComDataCU*& rpcTempCU, Int iQP, UInt uiDepth);
	__inline void 	ETRI_xCheckInter2NxNNx2N_RDOQ(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Int iQP, Bool bIsLosslessMode, UInt uiDepth, Bool& doNotBlockPu);
	__inline void 	ETRI_PseudoInter2NxNNx2N 	(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Int iQP, Bool bIsLosslessMode, UInt uiDepth, Bool& doNotBlockPu);
#endif 
    __inline Bool 	ETRI_CheckMergeCandidateonBoundary(TComDataCU* rpcTempCU, UInt uiMergeCand, bool bOperation);

	Void 	ETRI_xCheckRDCostMerge2Nx2N   		( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode );

	//----------------------------------------------------------------------------
	//	Service Function (From V12)
	//----------------------------------------------------------------------------
	__inline UInt 	ETRI_getPartIdx   			( TComDataCU* pcCU, UInt uiDepth)	{return (pcCU->getZorderIdxInCU() >> (8 - (uiDepth<<1)));}
#if ETRI_DEBUG_CODE_CLEANUP
    __inline Void 	ETRI_PrintDebugnfo  		( int WhereLine, const char *WhereARG, TComDataCU* pcCU, TComDataCU* pcBestCU, UInt	 uiDepth, Int DebugPhase, Int SubDbgPhase, Int iAnalysisType);
#endif 
    __inline UInt 	ETRI_EvalLevel 				( TComDataCU* pcCU, UInt uiStage, UInt*& puiLevelInfo);
	__inline UInt 	ETRI_EvalHAD  				( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, UInt*& puiLevelInfo);
	__inline UInt 	ETRI_EvalControlLevel  		(QpParam e_QPpram, UInt HADofStage, UInt uiDepth, UInt InitLevel, UInt*& puiLevelInfo, Bool bOP);
	__inline Void 	ETRI_xCheckEarlySkipDecision   (TComDataCU*& rpcBestCU, Bool* earlyDetectionSkipMode, UInt bOP);
	__inline Void   ETRI_xCheckBestModeMV		(TComDataCU*& pcCU, UChar uhDepth);		//For V3

	Void 	ETRI_xCheckBestMode 				(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth );
	Void 	ETRI_xStoreModeData 				(TComDataCU*& rpcDstCU, TComDataCU* rpcSrcCU, UInt uiDepth, Int BufferIdx, PartSize ePartSize, Bool StoreTrue );

	//----------------------------------------------------------------------------
	//	Fixed Function 
	//----------------------------------------------------------------------------
	__inline void	ETRI_CUPruningOnSameDepth 	( TComDataCU*& rpcTempCU,  UInt uiPartUnitIdx, Bool& bOperation);
	__inline void 	ETRI_RDcostEvaluationforCU 	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComSlice * pcSlice, TComPic* pcPic, UInt uiDepth, Bool bOperation);
	__inline void 	ETRI_CUBestModeDecision  	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhNextDepth, Int iQP, UInt uiDepth, Bool bOperation);


	void 	ETRI_SubCUProcessing_inPicture  	( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComSlice * pcSlice, Bool bBoundary, UInt uhNextDepth, Int iQP, UInt uiDepth);
#endif	///<  ETRI_MODIFICATION_V02 : 2015 7 31 by Seok

// ===========================================================
//  ETRI Removable Function : The internal codes is not operated any time.
// ===========================================================
	Void ETRI_TryAMPProcess(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize, Bool& doNotBlockPu, Int iQP, Bool bIsLosslessMode);

};

//! \}

#endif // __TENCMB__
