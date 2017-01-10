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

/** \file     TEncTop.h
    \brief    encoder class (header)
*/

#ifndef __TENCTOP__
#define __TENCTOP__

// Include files
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/AccessUnit.h"

#include "TLibVideoIO/TVideoIOYuv.h"

#include "TEncCfg.h"
#include "TEncGOP.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "TEncSearch.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncPreanalyzer.h"
#include "TEncRateCtrl.h"

#include "TEncTile.h"
#include "TEncFrame.h"

#if KAIST_RC
#include <list>
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class TEncTop : public TEncCfg
{
private:
  // picture
  Int                     m_iPOCLast;                     ///< time index (POC)
  Int                     m_iNumPicRcvd;                  ///< number of received pictures
  UInt                    m_uiNumAllPicCoded;             ///< number of coded pictures
  TComList<TComPic*>      m_cListPic;                     ///< dynamic list of pictures
#if ETRI_DLL_INTERFACE	
  TComList<TComPic*>	  em_cListPic; 					  ///< to refresh encoder, copy of dynamic list of pictures
  Bool					  em_bAnalyserClear;			  ///< signal the refreshing encoder
#endif

 #if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move  
  // encoder search
  TEncSearch              m_cSearch;                      ///< encoder search class
  //TEncEntropy*            m_pcEntropyCoder;                     ///< entropy encoder
  TEncCavlc*              m_pcCavlcCoder;                       ///< CAVLC encoder  
  // coding tool
  TComTrQuant             m_cTrQuant;                     ///< transform & quantization class
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
  TComLoopFilter          m_cLoopFilter[MAX_NUM_THREAD];                  ///< deblocking filter class
#else 
  TComLoopFilter          m_cLoopFilter;                  ///< deblocking filter class
#endif 
  TEncSampleAdaptiveOffset m_cEncSAO;                     ///< sample adaptive offset class
  TEncEntropy             m_cEntropyCoder;                ///< entropy encoder
  TEncCavlc               m_cCavlcCoder;                  ///< CAVLC encoder
  TEncSbac                m_cSbacCoder;                   ///< SBAC encoder
  TEncBinCABAC            m_cBinCoderCABAC;               ///< bin coder CABAC
  TEncSbac*               m_pcSbacCoders;                 ///< SBAC encoders (to encode substreams )
  TEncBinCABAC*           m_pcBinCoderCABACs;             ///< bin coders CABAC (one per substream)
#endif  

  // processing unit
  TEncGOP   				m_cGOPEncoder;                  ///< GOP encoder
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  TEncSlice   				m_cSliceEncoder;                ///< slice encoder 
  TEncCu     				m_cCuEncoder;                   ///< CU encoder
#endif
  // SPS
  TComSPS   				m_cSPS;                         ///< SPS
  TComPPS   				m_cPPS;                         ///< PPS
#if ETRI_MultiplePPS 
  TComPPS					em_cPPS_id1;                         ///< PPS
  TComPPS					em_cPPS_id2;                         ///< PPS
#endif
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  // RD cost computation
  TComBitCounter    		m_cBitCounter;                  ///< bit counter for RD optimization
  TComRdCost    			m_cRdCost;                      ///< RD cost computation class
  TEncSbac***    			m_pppcRDSbacCoder;              ///< temporal storage for RD computation
  TEncSbac   				m_cRDGoOnSbacCoder;             ///< going on SBAC model for RD stage
#if FAST_BIT_EST
  TEncBinCABACCounter***	m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
  TEncBinCABACCounter     	m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#else
  TEncBinCABAC***         	m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
  TEncBinCABAC    			m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#endif

//--------------------------------------------------------------------------------------------------------
//	For WPP Coders
//--------------------------------------------------------------------------------------------------------

  Int     					m_iNumSubstreams;                ///< # of top-level elements allocated.
  TComBitCounter*         	m_pcBitCounters;                 ///< bit counters for RD optimization per substream
  TComRdCost*   			m_pcRdCosts;                     ///< RD cost computation class per substream
  TEncSbac****   			m_ppppcRDSbacCoders;             ///< temporal storage for RD computation per substream
  TEncSbac*   				m_pcRDGoOnSbacCoders;            ///< going on SBAC model for RD stage per substream
  TEncBinCABAC****         	m_ppppcBinCodersCABAC;           ///< temporal CABAC state storage for RD computation per substream
  TEncBinCABAC*           	m_pcRDGoOnBinCodersCABAC;        ///< going on bin coder CABAC for RD stage per substream

  //--------------------------------------------------------------------------------------------------------
#endif //if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move

  // quality control
  TEncPreanalyzer  			m_cPreanalyzer;                 ///< image characteristics analyzer for TM5-step3-like adaptive QP

  TComScalingList  			m_scalingList;                 ///< quantization matrix information

#if KAIST_RC
  std::list<TEncRateCtrl>    			m_lcRateCtrl;                    ///< Rate control class
#endif
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
#if KAIST_RC
  TEncRateCtrl    			m_cRateCtrl;                    ///< Rate control class
#endif

//======================================================================================
//	ETRI member Parameters
//======================================================================================
  TEncFrame*  				em_pcFrameEncoder;				///< Frame Encoder @ 2015 5 23 by Seok
  TEncSlice*   				em_pcSliceEncoder;				///< Slice Encoder @ 2015 5 23 by Seok
  TEncTile*    				em_pcTileEncoder;				///< Tile Encoder @ 2015 5 17 by Seok
#endif

#if ETRI_DLL_INTERFACE	// 2013 10 23 by Seok
  UInt*	em_pFrameTypeInGOP;
  UInt*	em_pFramePOC;
  UInt*	em_pFrameEncodingOrder;
  short* em_pFrameSliceIndex;
#endif
  
protected:
  Void  xGetNewPicBuffer  ( TComPic*& rpcPic );           ///< get picture buffer which will be processed
  Void  xInitSPS          ();                             ///< initialize SPS from encoder options
  Void  xInitPPS          ();                             ///< initialize PPS from encoder options
  
  Void  xInitPPSforTiles  ();
#if ETRI_MultiplePPS  
  Void  ETRI_xInitPPS(TComPPS* pPPS, Int id);
  Void  ETRI_xInitPPSforTiles(TComPPS* pPPS, ETRI_PPSTile_t* pMultipleTile);
#endif

  Void  xInitRPS          (Bool isFieldCoding);           ///< initialize PPS from encoder options

public:
  TEncTop();
  virtual ~TEncTop();
  
  Void      create          ();
  Void      destroy         ();
  Void      init            (Bool isFieldCoding);
  Void      deletePicBuffer ();

  Void      createWPPCoders(Int iNumSubstreams);
  
  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------
  
#if KAIST_RC
  Bool getRCEnableRateControl() { return m_RCEnableRateControl; }

  Int getframesToBeEncoded() { return m_framesToBeEncoded; }
  Int getRCTargetBitrate() { return m_RCTargetBitrate; }
  Int getFrameRate() { return m_iFrameRate; }
  Int getGOPSize() { return m_iGOPSize; }
  Int getSourceWidth() { return m_iSourceWidth; }
  Int getSourceHeight() { return m_iSourceHeight; }
  Bool getRCUseLCUSeparateModel() { return m_RCUseLCUSeparateModel; }
  std::list<TEncRateCtrl>* getRateCtrlLst() { return &m_lcRateCtrl; }
#endif
  TComList<TComPic*>*     getListPic            () { return  &m_cListPic;             }
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  TEncSearch*             getPredSearch         () { return  &m_cSearch;              }
  
  TComTrQuant*            getTrQuant            () { return  &m_cTrQuant;             }
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
  TComLoopFilter*          getLoopFilter         (int i) { return  &m_cLoopFilter[i];          }
#else 
  TComLoopFilter*         getLoopFilter         () { return  &m_cLoopFilter;          }
#endif 
  TEncSampleAdaptiveOffset* getSAO              () { return  &m_cEncSAO;              }
#endif
  TEncGOP*                getGOPEncoder         () { return  &m_cGOPEncoder;          }
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  TEncSlice*              getSliceEncoder       () { return  &m_cSliceEncoder;        }
  TEncCu*                 getCuEncoder          () { return  &m_cCuEncoder;           }
  TEncEntropy*            getEntropyCoder       () { return  &m_cEntropyCoder;        }
  TEncCavlc*              getCavlcCoder         () { return  &m_cCavlcCoder;          }
  TEncSbac*               getSbacCoder          () { return  &m_cSbacCoder;           }
  TEncBinCABAC*           getBinCABAC           () { return  &m_cBinCoderCABAC;       }
  TEncSbac*               getSbacCoders     	() { return  m_pcSbacCoders;      }
  TEncBinCABAC*           getBinCABACs          () { return  m_pcBinCoderCABACs;      }
  
  TComBitCounter*         getBitCounter         () { return  &m_cBitCounter;          }
  TComRdCost*             getRdCost             () { return  &m_cRdCost;              }
  TEncSbac***             getRDSbacCoder        () { return  m_pppcRDSbacCoder;       }
  TEncSbac*               getRDGoOnSbacCoder    () { return  &m_cRDGoOnSbacCoder;     }
  TComBitCounter*         getBitCounters        () { return  m_pcBitCounters;         }
  TComRdCost*             getRdCosts            () { return  m_pcRdCosts;             }
  TEncSbac****            getRDSbacCoders       () { return  m_ppppcRDSbacCoders;     }
  TEncSbac*               getRDGoOnSbacCoders   () { return  m_pcRDGoOnSbacCoders;   }
#if KAIST_RC
  TEncRateCtrl*           getRateCtrl           () { return &m_cRateCtrl;             }
#endif
#endif
  TComSPS*                getSPS                () { return  &m_cSPS;                 }
  TComPPS*                getPPS() { return  &m_cPPS; }
#if ETRI_MultiplePPS
    TComPPS*                getPPS(Int ippsid); 
#endif
  Void selectReferencePictureSet(TComSlice* slice, Int POCCurr, Int GOPid );
  Int getReferencePictureSetIdxForSOP(TComSlice* slice, Int POCCurr, Int GOPid );
  TComScalingList*        getScalingList        () { return  &m_scalingList;         }
  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut,
              std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );  

  /// encode several number of pictures until end-of-sequence
  Void encode( bool bEos, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut,
              std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, bool isTff);
#endif
  Void printSummary(bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField); }
  
  //======================================================================================
  //  ETRI Public Functions 
  //======================================================================================
  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move
  TEncTile*   			ETRI_getTileEncoder    	()	{return em_pcTileEncoder;  		}
  TEncFrame*  			ETRI_getFrameEncoder 	()	{return em_pcFrameEncoder;		}
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // Initial and Destroy function
  // -------------------------------------------------------------------------------------------------------------------
  Void	ETRI_TEncTopCreate 	(Bool bOperation); 	
  Void 	ETRI_TEncTopDestroy	(Bool bOperation); 
  Void 	ETRI_TEncTopInit 	(Bool bOperation);
  // -------------------------------------------------------------------------------------------------------------------
  // ETRI_MULTITHREAD_2 Frame/GOP Parallel
  // -------------------------------------------------------------------------------------------------------------------
  ///////////////////////////////////////////////////////////////////////////////////////////
  // gplusplus
#if ETRI_MULTITHREAD_2
private:
	TComThreadPoolMgr m_hThreadPoolTop;
	TComPic**      em_ppcPic;                     ///< dynamic list of pictures
	//int	em_bThread[MAX_THREADPOOL_TOP];
	//int   em_nThreadCnt;
	bool  em_bCreate;
	bool em_bReconFileOk;  

public:
	static void threadTopProcessing(void *param, int num);
	Void ETRI_xSetNewPicBuffer(TComList<TComPic*>* pcListPic, int nCount);
	Void ETRI_deletePicBuffer(TComList<TComPic*>* pcListPic);
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	Void ETRI_encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPic*>& rcListPic, AccessUnit_t* accessUnitsOut, Int& iNumEncoded );
#else
	Void ETRI_encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, AccessUnit_t* accessUnitsOut, Int& iNumEncoded);
#endif

#if ETRI_COPYTOPIC_MULTITHREAD
	static Void *copyToPicProc(void *Param);
	struct copyToPicInfo
	{
		TComPicYuv* pcComPicYuv;
		TComPic* pcPicCurr;
		int index;
		int nThread;
	};
#endif

	Void ETRI_xPocReArrayPicBuffer(TComList<TComPic*>* pcListPic, int nOffset);
	Void ETRI_xGetNewPicBuffer(TComPic*& rpcPic, TComList<TComPic*>* pcListPic);
	// gplusplus_151102 memory optimizer
	void ETRI_setReconFileOk(bool bSet) { em_bReconFileOk = bSet; }
	bool ETRI_getReconFileOk() { return em_bReconFileOk; }

#endif

  // -------------------------------------------------------------------------------------------------------------------
  // DLLInterface function
  // -------------------------------------------------------------------------------------------------------------------
#if	ETRI_DLL_INTERFACE 	
  //Bool ETRI_CopyToPicFrame (TComPicYuv* pcPicYuvOrg, Int& iNumEncoded, Bool flush); ///< Copy to Pic
	Void ETRI_setFrameInfoforDLL(Int FrameIdx, UInt FrameEncodingOrder, UInt FrameTypeInGOP, UInt FramePOC, short FrameSliceIdx)
  {
	  em_pFrameEncodingOrder[FrameIdx] 		= FrameEncodingOrder;
	  em_pFrameTypeInGOP[FrameIdx]  		= FrameTypeInGOP;
	  em_pFramePOC[FrameIdx]     			= FramePOC;
	  em_pFrameSliceIndex[FrameIdx]			= FrameSliceIdx;
  }
  Void ETRI_getFrameInfoforDLL (Int FrameIdx, UInt& FrameEncodingOrder, UInt& FrameTypeInGOP, UInt& FramePOC, short& FrameSliceIdx)
  {
	  FrameEncodingOrder 	= em_pFrameEncodingOrder[FrameIdx];
	  FrameTypeInGOP 	 	= em_pFrameTypeInGOP[FrameIdx];
	  FramePOC    	  		= em_pFramePOC[FrameIdx];
	  FrameSliceIdx			= em_pFrameSliceIndex[FrameIdx];
  }
  ///<Refresh Encoder every IDR
  __inline	Void	ETRI_Clear_AnalyzeParameter()			{m_gcAnalyzeAll.clear(); m_gcAnalyzeI.clear(); m_gcAnalyzeP.clear(); m_gcAnalyzeB.clear();}
  __inline	Void	ETRI_Clear_ListPic(Bool bAalyserClear)	{m_cListPic.swap(em_cListPic);	em_bAnalyserClear = bAalyserClear;}
  ///<Set CTRParam
  __inline	Int*   	ETRI_getpiPOCLast() 					{return &m_iPOCLast;}
  __inline	Int*   	ETRI_getpiNumPicRcvd() 					{return &m_iNumPicRcvd;}
  __inline	UInt* 	ETRI_getpiNumAllPicCoded()				{return &m_uiNumAllPicCoded;}
#endif

};

//! \}

#endif // __TENCTOP__

