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

/** \file     TEncGOP.h
    \brief    GOP encoder class (header)
*/

#ifndef __TENCGOP__
#define __TENCGOP__

#include <list>

#include <stdlib.h>

#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "SEIwrite.h"

#include "TEncAnalyze.h"
#include "TEncRateCtrl.h"
#include <vector>

#include "TLibCommon/TComThreadPool.h"	// gplusplus

#include "TEncFrame.h"

#if ETRI_THREADPOOL_OPT
#include "Threadpool/Quram.h"
class EncGopJob;
#endif

//! \ingroup TLibEncoder
//! \{

class TEncTop;

#if ETRI_MULTITHREAD_2
// gplusplus [[
typedef struct{
	list<AccessUnit> outputAccessUnits;
	int pos;
	int poc;
} AccessUnit_t;


// ====================================================================================================================
// Class definition
// ====================================================================================================================
class TEncThreadGOP {
public:

	TComList<TComPic*>*		em_rpcListPic;
	TComList<TComPicYuv*>*	em_rpcListPicYuvRecOut;
	list<AccessUnit>*		em_paccessUnitsInGOP;
	TComPic*				em_pcPic;
	TComPicYuv*				em_pcPicYuvRecOut;
	int						em_iPos;
	int						em_pocCurr;
	bool					em_bFirst;
	bool					em_isField;
	bool					em_isTff;

public:
	TEncThreadGOP();
	virtual ~TEncThreadGOP();
};
#endif

/// GOP encoder class
class TEncGOP
{
private:
  //  Data
  Bool                    m_bLongtermTestPictureHasBeenCoded;
  Bool                    m_bLongtermTestPictureHasBeenCoded2;
  UInt                    m_numLongTermRefPicSPS;
  UInt                    m_ltRefPicPocLsbSps[33];
  Bool                    m_ltRefPicUsedByCurrPicFlag[33];
  Int                     m_iLastIDR;
  Int                     m_iGopSize;
  Int                     m_iNumPicCoded;
  Bool                    m_bFirst;
#if ALLOW_RECOVERY_POINT_AS_RAP
  Int                     m_iLastRecoveryPicPOC;
#endif

#if !ETRI_MULTITHREAD_2
  TEncSlice*              m_pcSliceEncoder;
  TEncEntropy*            m_pcEntropyCoder;
  TEncCavlc*              m_pcCavlcCoder;
  TEncSbac*               m_pcSbacCoder;
  TEncBinCABAC*           m_pcBinCABAC;
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
  TComLoopFilter*          m_pcLoopFilter[MAX_NUM_THREAD];
#else 
  TComLoopFilter*         m_pcLoopFilter;
#endif 
  SEIWriter               m_seiWriter;
  
  //--Adaptive Loop filter
  TEncSampleAdaptiveOffset*  m_pcSAO;
  TComBitCounter*         m_pcBitCounter;
#if KAIST_RC
  TEncRateCtrl*           m_pcRateCtrl;
#endif
  Bool                    m_activeParameterSetSEIPresentInAU;
  Bool                    m_bufferingPeriodSEIPresentInAU;
  Bool                    m_pictureTimingSEIPresentInAU;
  Bool                    m_nestedBufferingPeriodSEIPresentInAU;
  Bool                    m_nestedPictureTimingSEIPresentInAU;
#endif
  
  //  Access channel
  TEncTop*                m_pcEncTop;
#if !ETRI_THREADPOOL_OPT
  TEncCfg*                m_pcCfg;
#endif
  TComList<TComPic*>*     m_pcListPic;
  
  // indicate sequence first
  Bool                    m_bSeqFirst;
  
  // clean decoding refresh
  Bool                    m_bRefreshPending;
  Int                     m_pocCRA;
  std::vector<Int>        m_storedStartCUAddrForEncodingSlice;
  std::vector<Int>        m_storedStartCUAddrForEncodingSliceSegment;
#if FIX1172
  NalUnitType             m_associatedIRAPType;
  Int                     m_associatedIRAPPOC;
#endif

  std::vector<Int> m_vRVM_RP;
  UInt                    m_lastBPSEI;
  UInt                    m_totalCoded;
  UInt                    m_cpbRemovalDelay;
  UInt                    m_tl0Idx;
  UInt                    m_rapIdx;
 
//======================================================================================
//	ETRI member Variables 
//======================================================================================

public:
  TEncGOP();
  virtual ~TEncGOP();
  
  TEncFrame*		em_pcFrameEncoder;            // member variable to public. Quram  yjnam (16.08.12)
#if ETRI_THREADPOOL_OPT
  TEncCfg*                m_pcCfg;
#endif
#if ETRI_MULTITHREAD_2
  Void  create      (TEncTop* pcEncTop);
#else
  Void  create      ();
  
  Void  xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream*& rpcBitstreamRedirect);
 
  Void  preLoopFilterPicAll  ( TComPic* pcPic, UInt64& ruiDist, UInt64& ruiBits );  
  TEncSlice*  getSliceEncoder()   { return m_pcSliceEncoder; }
  
//======================================================================================
//	ETRI Public Functions 
//======================================================================================
//------------------------ For Data Interface -------------------------
TEncEntropy*	ETRI_getEntropyCoder	()	{return m_pcEntropyCoder;}
TEncCavlc*   	ETRI_getCavlcCoder		()	{return m_pcCavlcCoder;}
TEncSbac*  		ETRI_getSbacCoder		()	{return	m_pcSbacCoder;}
TEncBinCABAC* 	ETRI_getBinCABAC		()	{return	m_pcBinCABAC;}
TComBitCounter*	ETRI_getBitCounter		()	{return	m_pcBitCounter;}
SEIWriter*   	ETRI_getSEIWriter		()	{return &m_seiWriter;}
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
TComLoopFilter*  ETRI_getLoopFilter(int i)   {return m_pcLoopFilter[i];}
#else 
TComLoopFilter* ETRI_getLoopFilter  	()	{return m_pcLoopFilter;}
#endif 
TEncSampleAdaptiveOffset*  ETRI_getSAO	()	{return m_pcSAO;}


Bool 	ETRI_getactiveParameterSetSEIPresentInAU	()	{return m_activeParameterSetSEIPresentInAU;}	///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
Bool& 	ETRI_getpictureTimingSEIPresentInAU 		()	{return m_pictureTimingSEIPresentInAU;}   		///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
Bool&	ETRI_getbufferingPeriodSEIPresentInAU 		()	{return m_bufferingPeriodSEIPresentInAU;}  		///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
Bool& 	ETRI_getnestedBufferingPeriodSEIPresentInAU	()	{return	m_nestedBufferingPeriodSEIPresentInAU;}	///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
UInt& 	ETRI_getcpbRemovalDelay  		()  {return m_cpbRemovalDelay;}   	///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
#endif
  Void  destroy     ();
  
  Void  init        ( TEncTop* pcTEncTop );
 Int   getGOPSize()          { return  m_iGopSize;  }
  
  TComList<TComPic*>*   getListPic()      { return m_pcListPic; }
  
  Void  printOutSummary      ( UInt uiNumAllPicCoded, bool isField);
  NalUnitType getNalUnitType( Int pocCurr, Int lastIdr, Bool isField );
  Void arrangeLongtermPicturesInRPS(TComSlice *, TComList<TComPic*>& );
Int  	ETRI_getiLastIDR				()	{return m_iLastIDR;}
Int  	ETRI_getassociatedIRAPPOC   	()	{return m_associatedIRAPPOC ;}	///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok
Int& 	ETRI_getiLastRecoveryPicPOC 	()	{return m_iLastRecoveryPicPOC;}	///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok 2015 5 24 by Seok
Int  	ETRI_getpocCRA					()	{return m_pocCRA;}				///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok 2015 5 24 by Seok

UInt&  	ETRI_getlastBPSEI				()	{return m_lastBPSEI;}			///< For in ETRI_setPictureTimingSEI @2015 5 24 by Seok
UInt   	ETRI_gettotalCoded				()	{return m_totalCoded;}  		///< For in ETRI_setPictureTimingSEI @2015 5 24 by Seok
#if ETRI_DLL_INTERFACE
Void   	ETRI_init_totalCoded()	{ m_totalCoded = 0; }  		///< For DLL refresh w/Multithread_2 GOP_Parallel
#endif

UInt  	ETRI_getnumLongTermRefPicSPS	()	{return m_numLongTermRefPicSPS;}
UInt*	ETRI_getltRefPicPocLsbSps		()	{return	m_ltRefPicPocLsbSps;}	///< Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok
Bool*	ETRI_getltRefPicUsedByCurrPicFlag()	{return	m_ltRefPicUsedByCurrPicFlag;}///< Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok
Bool 	ETRI_getbRefreshPending			()	{return m_bRefreshPending;}		///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok 2015 5 24 by Seok
Bool& 	ETRI_getbFirst  				()	{return m_bFirst;}
Void    ETRI_setbFirst					(bool isFirst){ m_bFirst = isFirst; }//< For ETRI_refresh DLL

Void 	ETRI_setpocCRA					(Int ipocCRA) 			{m_pocCRA = ipocCRA;}	///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok 
Void 	ETRI_setbRefreshPending			(Bool bRefreshPending)	{m_bRefreshPending = bRefreshPending;}	///< For ETRI_SetReferencePictureSetforSlice : Is it necessary to a member variable ??	@ 2015 5 24 by Seok

NalUnitType ETRI_getassociatedIRAPType	()	{return m_associatedIRAPType;}	///Is it necessary to a member variable ??	@ 2015 5 24 by Seok

std::vector<Int>	*ETRI_getstoredStartCUAddrForEncodingSlice  		()	{return	&m_storedStartCUAddrForEncodingSlice;}	   		///< For Test @ 2015 5 24 by Seok		
std::vector<Int>	*ETRI_getstoredStartCUAddrForEncodingSliceSegment	()	{return &m_storedStartCUAddrForEncodingSliceSegment;}	///< For Test @ 2015 5 24 by Seok

//------------------------ For GOP Compression -------------------------
Void ETRI_EfficientFieldIRAPProc(Int iPOCLast, Int iNumPicRcvd, bool isField, Int& IRAPGOPid, Bool& IRAPtoReorder, Bool& swapIRAPForward, Bool bOperation);
void ETRI_xCalculateAddPSNR(TComPic* pcPic, TComList<TComPic*>& rcListPic, AccessUnit& accessUnit, Double dEncTime, Bool isField, Bool isTff); ///< 2015 5 13 by Seok:This Function is move to TEncFrame Class


Void ETRI_SetSEITemporalLevel0Index(TComSlice *pcSlice, SEITemporalLevel0Index& sei_temporal_level0_index);		///< TO Write Out SEI data,  Set the SEI data of TemporalLevel0Index @ 2015 5 25 by Seok

__inline Int ETRI_iGOPIRAPtoReorder_v1(Int iGOPid, Int IRAPGOPid, Bool IRAPtoReorder, Bool swapIRAPForward, Bool bOperation);
__inline Int ETRI_iGOPIRAPtoReorder_v2(Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, Bool bOperation);
__inline Int ETRI_iGOPIRAPtoReorder_v3(Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, Bool bOperation);
__inline Bool ETRI_FIrstPOCForFieldFrame(Int& pocCurr, Int& iTimeOffset, Int iPOCLast, Int iNumPicRcvd, Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, bool isField);	///< To TEncFrame

#if ETRI_MODIFICATION_V00
#if ETRI_MULTITHREAD_2
Void  ETRI_compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, AccessUnit_t* accessUnitsInGOP, bool isField, bool isTff);
#else
Void  ETRI_compressGOP ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec, std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff );
#endif
#else
Void  compressGOP ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec, std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff );
#endif
//======================================================================================
  
  Void dblMetric( TComPic* pcPic, UInt uiNumSlices );			///< Original dblMetric is a protected function. WHy ??? @ 2015 5 24 by Seok

  Int xGetFirstSeiLocation (AccessUnit &accessUnit);
  Void  xCalculateAddPSNR ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime );
  Void  xCalculateInterlacedAddPSNR( TComPic* pcPicOrgTop, TComPic* pcPicOrgBottom, TComPicYuv* pcPicRecTop, TComPicYuv* pcPicRecBottom, const AccessUnit& accessUnit, Double dEncTime );

#if !ETRI_MULTITHREAD_2
#if KAIST_RC
TEncRateCtrl* getRateCtrl()       { return m_pcRateCtrl;  }		///< Original getRateCtrl() is a protected function. WHy ??? @ 2015 5 24 by Seok
#endif
Void xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps); ///< Original Function is a protected function. WHy ??? @ 2015 5 24 by Seok

Void xResetNonNestedSEIPresentFlags()
{
  m_activeParameterSetSEIPresentInAU = false;
  m_bufferingPeriodSEIPresentInAU	 = false;
  m_pictureTimingSEIPresentInAU 	 = false;
}
Void xResetNestedSEIPresentFlags()
{
  m_nestedBufferingPeriodSEIPresentInAU    = false;
  m_nestedPictureTimingSEIPresentInAU	   = false;
}
#endif
Void  xGetBuffer		( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, bool isField );


protected:
  
  Void  xInitGOP          ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, bool isField );
//  Void  xGetBuffer        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, bool isField );


#if !ETRI_MULTITHREAD_2
  UInt64 xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1);
    SEIActiveParameterSets* xCreateSEIActiveParameterSets (TComSPS *sps);
  SEIFramePacking*        xCreateSEIFramePacking();
  SEIDisplayOrientation*  xCreateSEIDisplayOrientation();

  SEIToneMappingInfo*     xCreateSEIToneMappingInfo();

#endif
  Double xCalculateRVM();


//--------------------------------------------------------------------------------------------------------
//	Originaly the foloowing Functions defined as protected function. However, we chaged the chrateristics of the functions as public
//	2015 5 25 by Seok ETRI
//--------------------------------------------------------------------------------------------------------
#if !ETRI_MODIFICATION_V00
#if 0
  Void xResetNonNestedSEIPresentFlags()
  {
    m_activeParameterSetSEIPresentInAU = false;
    m_bufferingPeriodSEIPresentInAU    = false;
    m_pictureTimingSEIPresentInAU      = false;
  }
  Void xResetNestedSEIPresentFlags()
  {
    m_nestedBufferingPeriodSEIPresentInAU    = false;
    m_nestedPictureTimingSEIPresentInAU      = false;
  }

  TEncRateCtrl* getRateCtrl() 	  { return m_pcRateCtrl;  } 	  
  Int xGetFirstSeiLocation (AccessUnit &accessUnit);
  Void xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps);
  Void dblMetric( TComPic* pcPic, UInt uiNumSlices );
#endif
#endif
#if ETRI_MULTITHREAD_2
////////////////////////////////////////////
// gplusplus
public:
  TComThreadPoolMgr			em_hThreadPoolGOP;
  TEncThreadGOP				em_cThreadGOP[MAX_THREAD_GOP];
  int						em_IDRpos;
  ETRI_RefPic_t				em_refPic[MAX_THREAD_SIZE];  // GOP size
  int						em_iprevLastIDR;
  int					    em_iMaxFrameNum;
  bool					    em_bThreadUsed[MAX_THREAD_SIZE];

#if ETRI_THREADPOOL_OPT
  QphotoThreadPool			*m_qrThreadpool;
  bool						m_bThreadRunning[MAX_THREAD_GOP];
#endif
public:
  Int     ETRI_getnumPicCoded				()  {return m_iNumPicCoded;}
  UInt64 xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1);
 
  TEncTop*   	ETRI_getEncTop		()	{return m_pcEncTop;}
  TEncFrame*		ETRI_getFrameEncoder()	{return em_pcFrameEncoder;}

  static void threadProcessingGOP(void *param, int num);
  bool ETRI_RefPicCheck(int curPOC, TComList<TComPic*>* rpcListPic, int RefSize);
  Void ETRI_xGetBuffer( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, int num, bool isField);

#endif 
};// END CLASS DEFINITION TEncGOP

// ====================================================================================================================
// Enumeration
// ====================================================================================================================
enum PROCESSING_STATE
{
  EXECUTE_INLOOPFILTER,
  ENCODE_SLICE
};

enum SCALING_LIST_PARAMETER
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

//! \}

#endif // __TENCGOP__

