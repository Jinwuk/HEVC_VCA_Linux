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

/** \file     TEncSlice.h
	\brief    slice encoder class (header)
*/

#ifndef __TENCSLICE__
#define __TENCSLICE__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicYuv.h"
#include "TEncCu.h"
#include "WeightPredAnalysis.h"
#include "TEncRateCtrl.h"
#include "TEncTile.h"				/// 2015 5 17 by Seok
#include "TLibCommon/TComThreadPool.h"  // gplusplus

#if (_ETRI_WINDOWS_APPLICATION)
#include <windows.h>
#else
#include <pthread.h>
#include <semaphore.h>
#endif

#if ETRI_THREADPOOL_OPT
#include "Threadpool/Quram.h"
#endif

//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncGOP;
#if ETRI_MULTITHREAD_2
class TEncFrame;	// gplusplus
#endif

#if ETRI_THREADPOOL_OPT
class EncTileJob;
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// slice encoder class
class TEncSlice
  : public WeightPredAnalysis
{
private:
  // encoder configuration
  TEncCfg*                m_pcCfg;                              ///< encoder configuration class
  
  // pictures
  TComList<TComPic*>*     m_pcListPic;                          ///< list of pictures
  TComPicYuv*             m_apcPicYuvPred;                      ///< prediction picture buffer
  TComPicYuv*             m_apcPicYuvResi;                      ///< residual picture buffer
  
  // processing units
  TEncGOP*                m_pcGOPEncoder;                       ///< GOP encoder
  TEncCu*                 m_pcCuEncoder;                        ///< CU encoder
  
  // encoder search
  TEncSearch*             m_pcPredSearch;                       ///< encoder search class
  
  // coding tools
  TEncEntropy*            m_pcEntropyCoder;                     ///< entropy encoder
  TEncCavlc*              m_pcCavlcCoder;                       ///< CAVLC encoder
  TEncSbac*               m_pcSbacCoder;                        ///< SBAC encoder  : [Dpeth][State :CI_IDX]
  TEncBinCABAC*           m_pcBinCABAC;                         ///< Bin encoder CABAC
  TComTrQuant*            m_pcTrQuant;                          ///< transform & quantization
  
  // RD optimization
  TComBitCounter*         m_pcBitCounter;                       ///< bit counter
  TComRdCost*             m_pcRdCost;                           ///< RD cost computation
  TEncSbac***             m_pppcRDSbacCoder;                    ///< storage for SBAC-based RD optimization
  TEncSbac*               m_pcRDGoOnSbacCoder;                  ///< go-on SBAC encoder
  UInt64                  m_uiPicTotalBits;                     ///< total bits for the picture
  UInt64                  m_uiPicDist;                          ///< total distortion for the picture
  Double                  m_dPicRdCost;                         ///< picture-level RD cost
  Double*                 m_pdRdPicLambda;                      ///< array of lambda candidates
  Double*                 m_pdRdPicQp;                          ///< array of picture QP candidates (double-type for lambda)
  Int*                    m_piRdPicQp;                          ///< array of picture QP candidates (Int-type)
  TEncBinCABAC*           m_pcBufferBinCoderCABACs;       		///< line of bin coder CABAC
  TEncSbac*               m_pcBufferSbacCoders;                 ///< line to store temporary contexts
  TEncBinCABAC*           m_pcBufferLowLatBinCoderCABACs;       ///< dependent tiles: line of bin coder CABAC
  TEncSbac*               m_pcBufferLowLatSbacCoders;           ///< dependent tiles: line to store temporary contexts
#if KAIST_RC
  TEncRateCtrl*           m_pcRateCtrl;                         ///< Rate control manager
#endif
  UInt                    m_uiSliceIdx;
  std::vector<TEncSbac*> CTXMem;

//======================================================================================
//	ETRI member Parameters
//======================================================================================
  UInt  				em_uiTileIdx;

#if ETRI_MULTITHREAD_2
	TEncFrame*			em_pcEncFrame;	// gplusplus
#endif

#if !(_ETRI_WINDOWS_APPLICATION)
  //Tile pthread var. by yhee 2016.04.19
  pthread_t em_TileThreadID[ETRI_MAX_TILES];       
  sem_t em_TileSemaphore;
#endif


public:
  TEncSlice();
  virtual ~TEncSlice();

  TEncTile*				em_pcTileEncoder;						///< Tile Encoder;	// quram : ysjeong 160804
  
  Void    create              ( Int iWidth, Int iHeight, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth );
  Void    destroy             ();
#if ETRI_MULTITHREAD_2
	Void    init(TEncTop* pcEncTop, TEncFrame* pcEncFrame);	// gplusplus
#else
  Void    init                ( TEncTop* pcEncTop );
#endif
  
  /// preparation of slice encoding (reference marking, QP and lambda)
  Void    initEncSlice        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd,
								Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, bool isField );
  Void    resetQP             ( TComPic* pic, Int sliceQP, Double lambda );
  // compress and encode slice
  Void    precompressSlice    ( TComPic*& rpcPic                                );      ///< precompress slice for multi-loop opt.
  Void    compressSlice       ( TComPic*& rpcPic                                );      ///< analysis stage of slice
  Void    calCostSliceI       ( TComPic*& rpcPic );
  Void    encodeSlice         ( TComPic*& rpcPic, TComOutputBitstream* pcSubstreams  );
  
  // misc. functions
  Void    setSearchRange      ( TComSlice* pcSlice  );                                  ///< set ME range adaptively
  UInt64  getTotalBits        ()  { return m_uiPicTotalBits; }
  
  TEncCu*        getCUEncoder() { return m_pcCuEncoder; }                        ///< CU encoder
  Void    xDetermineStartAndBoundingCUAddr  ( UInt& uiStartCUAddr, UInt& uiBoundingCUAddr, TComPic*& rpcPic, Bool bEncodeSlice );
  UInt    getSliceIdx()         { return m_uiSliceIdx;                    }
  Void    setSliceIdx(UInt i)   { m_uiSliceIdx = i;                       }
  Void      initCtxMem( UInt i );
  Void      setCtxMem( TEncSbac* sb, Int b )   { CTXMem[b] = sb; }

//========================================================================================================
//	ETRI Public Functions 
//========================================================================================================
#if ETRI_MODIFICATION_V00 
Void ETRI_InitWeightedPrediction 	(TComSlice* pcSlice);																							///< Initialization of Weighted Prediction and Adaptive QP Selection @ 2015 5 15 by Seok
Void ETRI_GetUseSABACRD 			(TEncSbac****& ppppcRDSbacCoders, CI_IDX  e_CI_IDX, TComSlice* pcSlice, TComPic*& rpcPic, Int& iNumSubstreams);	///< Get SABACRD coder. However it can be optimized almost codes using ETRI_Remove_BufferSbacCoders @  2015 5 15 by Seok
void ETRI_InitDependentSlices 		(TComPic*& rpcPic, TComSlice* pcSlice,  TEncSbac**** ppppcRDSbacCoders, ETRI_InfoofCU& pETRI_InfoofCU);			///< This Function will be almost SKipped @2015 5 15 by Seok
Void ETRI_UpdateSliceSegemnt    	(TComSlice* pcSlice, ETRI_InfoofCU& pETRI_InfoofCU);															///< Update Parameters of Slice Segment after comress CU @ 2015 5 15 by Seok


void ETRI_InitRDCoderForSubStream	(TEncSbac****& ppppcRDSbacCoders, TComPic*& rpcPic, TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU); ///< Initialization of RD Coders for SubStream.  Set ppppcRDSbacCoders & m_pppcRDSbacCoder[0][CI_CURR_BEST] 2015 5 15 by Seok
void ETRI_InitRateControlSBACRD  	(TComPic*& rpcPic, TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters);
Void ETRI_RestoreEntropyCoder   	(TEncSbac****& ppppcRDSbacCoders, TEncBinCABAC*& pppcRDSbacCoder, TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU); ///< restore entropy coder to an initial stage after comress CU @ 2015 5 15 by Seok	
#if KAIST_RC
Void ETRI_RestoreRateControl     	(TComPic*& rpcPic, TComDataCU*& pcCU, ETRI_InfoofCU& pETRI_InfoofCU);											///< Update Parameters of Rate Control after comress CU @ 2015 5 15 by Seok						
#endif
//---------------------------------------------------------------------------------------------------------
// 	Macro and Service Function for ETRI_compressSlice
//---------------------------------------------------------------------------------------------------------
#if (_ETRI_WINDOWS_APPLICATION)
static  	Void ETRI_CompressTile 				(TEncTile* e_pcTileEncoder); 
#else
static		Void* ETRI_CompressTile			    (void* e_pcTileEncoder);
#endif
TEncTile* 		 ETRI_getTileEncoder			()	{return em_pcTileEncoder;}
__inline 	Void ETRI_InitcompressSlice 		(TEncBinCABAC*& pppcRDSbacCoder, CI_IDX  e_CI_IDX, TComSlice* pcSlice);								///< Initilization of TEncBinCABAC Coder (pppcRDSbacCoder) for  Slice Encoding @ 2015 5 15 by Seok 
__inline 	Void ETRI_SetSliceRDOResult  	 	(UInt uiNumTiles);
#if 0 //yhee 2015.10.29
__inline 	Void ETRI_SetSliceRDCodertoTileCoder(UInt uiNumTiles);
#endif
__inline 	Void ETRI_SliceDebugInfo			(TComSlice* pcSlice, UInt uiNumTiles);  															///< Slice Debug Information @ 2015 11 17 by Seok

//---------------------------------------------------------------------------------------------------------
//	For Developing Parallel Class such that TEncTile , TEncWPP 
//---------------------------------------------------------------------------------------------------------
Void ETRI_CompressUnit(TComPic*& rpcPic, TComSlice* pcSlice, TEncSbac****& ppppcRDSbacCoders, TEncBinCABAC*& pppcRDSbacCoder, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU);
Void	ETRI_compressSlice		( TComPic*& rpcPic, Bool bOperation);	  ///< Main Slice Compression : IF bOperation is False or "0", then it combined with compressSlice. @ 2015 5 14 by Seok
#else
Void	ETRI_compressSlice		( TComPic*& rpcPic, Bool bOperation) 	{TEncSlice::compressSlice(rpcPic);}	  ///< Main Slice Compression : IF bOperation is False or "0", then it combined with compressSlice. @ 2015 5 14 by Seok
#endif
//========================================================================================================

#if KAIST_RC
Void setRateCtrl(TEncRateCtrl* cRateCtrl) { m_pcRateCtrl = cRateCtrl; }
#endif
private:
	Double  xGetQPValueAccordingToLambda(Double lambda);
#if ETRI_MULTITHREAD_2
public:
	TComThreadPoolMgr		em_hThreadPoolTile;

public:
	static void threadProcessingTile(void *param, int num);
#endif
};

//! \}

#endif // __TENCSLICE__
