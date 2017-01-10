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

/** \file     TEncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "TEncTop.h"
#include "TEncGOP.h"
#include "TEncAnalyze.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/NAL.h"
#include "NALwrite.h"
#include <time.h>
#include <math.h>
#if !(_ETRI_WINDOWS_APPLICATION)
#include <pthread.h>
#include <unistd.h>
#endif

using namespace std;

//#if ETRI_THREADPOOL_OPT && ETRI_MULTITHREAD_2
//QphotoThreadPool *gop_QphotoPool;
//bool bThreadRunning[MAX_THREAD_GOP] = { 0, };
//#endif

//! \ingroup TLibEncoder
//! \{

#if PSNR_DISPLAY	
#if (_ETRI_WINDOWS_APPLICATION)
CRITICAL_SECTION GOP_cs;  // gplusplus_151116
#else
pthread_mutex_t GOP_cs;
#endif
#endif

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
Int getLSB(Int poc, Int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}
#if ETRI_MULTITHREAD_2
// gplusplus_151124 Initialize
TEncThreadGOP::TEncThreadGOP()
{
	em_rpcListPic = NULL;
	em_rpcListPicYuvRecOut = NULL;
	em_pcPic = NULL;
	em_pcPicYuvRecOut = NULL;
	em_iPos = 0;
	em_pocCurr = 0;
	em_bFirst = true;
	em_isField = false;
	em_isTff = false;
}

TEncThreadGOP::~TEncThreadGOP()
{
}
#endif

TEncGOP::TEncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
#if ETRI_THREADPOOL_OPT
	m_qrThreadpool = NULL;
#endif

#if ALLOW_RECOVERY_POINT_AS_RAP
  m_iLastRecoveryPicPOC = 0;
#endif

#if !ETRI_MULTITHREAD_2
  m_pcSliceEncoder = NULL;
  m_pcEntropyCoder = NULL;
  m_pcCavlcCoder = NULL;
  m_pcSbacCoder = NULL;
  m_pcBinCABAC = NULL;
  xResetNonNestedSEIPresentFlags();
  xResetNestedSEIPresentFlags();
#endif
  
  m_pcCfg               = NULL;

  m_pcListPic           = NULL;
  
 
  
  m_bSeqFirst           = true;
  
  m_bRefreshPending     = 0;
  m_pocCRA            = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_cpbRemovalDelay   = 0;
  m_lastBPSEI         = 0;
  
#if FIX1172
  m_associatedIRAPType = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC  = 0;
#endif
#if ETRI_MULTITHREAD_2
  em_IDRpos = 0;
  em_iprevLastIDR = 0;
  em_iMaxFrameNum = 0;
#endif
  return;
}

TEncGOP::~TEncGOP()
{
}

/** Create list to contain pointers to LCU start addresses of slice.
 */
# if ETRI_MULTITHREAD_2
Void TEncGOP::create(TEncTop* pcEncTop)
#else 
Void  TEncGOP::create()
#endif
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;

# if ETRI_MULTITHREAD_2 //decr. em_iMaxFrameNum to 4 or 5 ??
  // yhee added
#if (ETRI_PARALLEL_SEL == ETRI_TILE_ONLY)
  em_iMaxFrameNum = 1;
#elif (ETRI_PARALLEL_SEL == ETRI_FRAME_PARALLEL)
  em_iMaxFrameNum = pcEncTop->getGOPSize();
#else
  em_iMaxFrameNum = pcEncTop->getIntraPeriod();
#endif

#if KAIST_RC
//   if (pcEncTop->getRCEnableRateControl())
//   {
// 	  Int* tileColumnWidth = new Int[pcEncTop->getNumColumnsMinus1() + 1];
// 	  Int* tileRowHeight = new Int[pcEncTop->getNumRowsMinus1() + 1];
// 	  if (!pcEncTop->getTileUniformSpacingFlag())
// 	  {
// 		  for (Int col = 0; col < pcEncTop->getNumColumnsMinus1(); col++)
// 			  tileColumnWidth[col] = pcEncTop->getColumnWidth(col);
// 		  for (Int row = 0; row < pcEncTop->getNumRowsMinus1(); row++)
// 			  tileRowHeight[row] = pcEncTop->getRowHeight(row);
// 	  }
// 	  else
// 	  {
// 		  Int picWidth = pcEncTop->getSourceWidth();
// 		  Int picHeight = pcEncTop->getSourceHeight();
// 		  Int picWidthInBU = (picWidth  % g_uiMaxCUWidth) == 0 ? picWidth / g_uiMaxCUWidth : picWidth / g_uiMaxCUWidth + 1;
// 		  Int picHeightInBU = (picHeight % g_uiMaxCUHeight) == 0 ? picHeight / g_uiMaxCUHeight : picHeight / g_uiMaxCUHeight + 1;
// 		  Int numCols = pcEncTop->getNumColumnsMinus1() + 1;
// 		  Int numRows = pcEncTop->getNumRowsMinus1() + 1;
// 		  for (Int col = 0; col < pcEncTop->getNumColumnsMinus1(); col++)
// 			  tileColumnWidth[col] = (col + 1)*picWidthInBU / numCols - (col*picWidthInBU) / numCols;
// 		  for (Int row = 0; row < pcEncTop->getNumRowsMinus1(); row++)
// 			  tileRowHeight[row] = (row + 1)*picHeightInBU / numRows - (row*picHeightInBU) / numRows;
// 	  }
// 
// 	  pcEncTop->getRateCtrl()->init(pcEncTop->getNumColumnsMinus1() + 1, pcEncTop->getNumRowsMinus1() + 1, tileColumnWidth, tileRowHeight, pcEncTop->getframesToBeEncoded(), pcEncTop->getRCTargetBitrate(), pcEncTop->getFrameRate(), pcEncTop->getSourceWidth(), pcEncTop->getSourceHeight(),
// 		  g_uiMaxCUWidth, g_uiMaxCUHeight, pcEncTop->getRCUseLCUSeparateModel(), pcEncTop->getIntraPeriod(), pcEncTop->getGOPSize(), pcEncTop->ETRI_getGOPEntry());
// 
// 	  delete[] tileColumnWidth;
// 	  delete[] tileRowHeight;
//   }
#endif
  em_pcFrameEncoder = new TEncFrame[em_iMaxFrameNum];
  for (int i = 0; i < em_iMaxFrameNum; i++)
  {
	  em_pcFrameEncoder[i].create(pcEncTop);
  }

#if ETRI_THREADPOOL_OPT
	if (!m_qrThreadpool)
		m_qrThreadpool = QphotoThreadPool::createQphotoThreadPool(ETRI_THREADPOOL_OPT_MAX_THREAD);
	memset(m_bThreadRunning, 0, sizeof(m_bThreadRunning));
#else
  em_hThreadPoolGOP.Create(threadProcessingGOP, MAX_THREAD_GOP);
#endif

#endif
}

Void  TEncGOP::destroy()
{
# if ETRI_MULTITHREAD_2 

#if ETRI_THREADPOOL_OPT
	if (m_qrThreadpool != NULL)
		QphotoThreadPool::destroyQphotoThreadPool(m_qrThreadpool);
	m_qrThreadpool = NULL;
#else
	// gplusplus
	em_hThreadPoolGOP.FinishThread();
#endif

	for (int i = 0; i < em_iMaxFrameNum; i++)
	{
		ESPRINTF(ETRI_MODV2_DEBUG, stderr, "Frame Encoder Release : %d of %d \n", i, em_iMaxFrameNum);
		em_pcFrameEncoder[i].destroy();
	}

	if (em_pcFrameEncoder)
		delete[] em_pcFrameEncoder;
#if PSNR_DISPLAY	
#if (_ETRI_WINDOWS_APPLICATION)
	DeleteCriticalSection(&GOP_cs); // gplusplus_151116
#else
	pthread_mutex_destroy(&GOP_cs);
#endif
#endif

#endif
}

Void TEncGOP::init ( TEncTop* pcTEncTop )
{
	m_pcEncTop  			= pcTEncTop;
	m_pcCfg    	    	  	= pcTEncTop;
	m_pcListPic = pcTEncTop->getListPic();
#if !ETRI_MULTITHREAD_2
	m_pcSliceEncoder		= pcTEncTop->getSliceEncoder();

	m_pcEntropyCoder		= pcTEncTop->getEntropyCoder();
	m_pcCavlcCoder  		= pcTEncTop->getCavlcCoder();
	m_pcSbacCoder   		= pcTEncTop->getSbacCoder();
	m_pcBinCABAC    		= pcTEncTop->getBinCABAC();
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
	for (int i=0; i < MAX_NUM_THREAD; i++)
		m_pcLoopFilter[i]  		= pcTEncTop->getLoopFilter(i);
#else 
	m_pcLoopFilter  		= pcTEncTop->getLoopFilter();
#endif 
	m_pcBitCounter  		= pcTEncTop->getBitCounter();

	//--Adaptive Loop filter
	m_pcSAO 	     		= pcTEncTop->getSAO();
#if KAIST_RC
	m_pcRateCtrl 			= pcTEncTop->getRateCtrl();
#endif
#endif
	m_lastBPSEI  			= 0;
	m_totalCoded			= 0;

#if ETRI_MODIFICATION_V00

#if ETRI_MULTITHREAD_2
	// gplusplus
	for (int i = 0; i < em_iMaxFrameNum; i++)
	{
		em_pcFrameEncoder[i].init(this);
	}

#if ETRI_THREADPOOL_OPT
#else
	em_hThreadPoolGOP.Init();
#endif


#if PSNR_DISPLAY	
#if (_ETRI_WINDOWS_APPLICATION)
	InitializeCriticalSection(&GOP_cs); // gplusplus_151116
#else
	pthread_mutex_init(&GOP_cs, NULL);
#endif
#endif
#else
	em_pcFrameEncoder	= pcTEncTop->ETRI_getFrameEncoder();
#endif

	EDPRINTF(stderr, "------------------------------------------ \n");

#if ETRI_MODIFICATION_V02 
	EDPRINTF(stderr, "         Status of Fast Algorithms  \n");
	EDPRINTF(stderr, " ETRI_DEV_0731            :%s \n", GetBoolVal(ETRI_DEV_0731));
	EDPRINTF(stderr, " ETRI_SKIP_64x64LCU       :%s \n", GetBoolVal(ETRI_SKIP_64x64LCU));
	EDPRINTF(stderr, " ETRI_ENABLE_2NxNNx2NProc :%s \n", GetBoolVal(ETRI_ENABLE_2NxNNx2NProc));	
	EDPRINTF(stderr, " ETRI_FIXED_ESDOFF        :%s \n", GetBoolVal(ETRI_FIXED_ESDOFF));	
	EDPRINTF(stderr, " ETRI_RDOOffBestMergeCand :%s \n", GetBoolVal(ETRI_RDOOffBestMergeCand));	
	EDPRINTF(stderr, " ETRI_FRDOOffBestMergeCand:%s \n", GetBoolVal(ETRI_FRDOOffBestMergeCand));
	EDPRINTF(stderr, " ETRI_POSTESD             :%s \n", GetBoolVal(ETRI_POSTESD));
	EDPRINTF(stderr, " ETRI_FPOSTESD            :%s \n", GetBoolVal(ETRI_FPOSTESD));
#if ETRI_STATUS_FAST_ME_INFORM
	EDPRINTF(stderr, " ETRI_FAST_INTEGER_ME     :%s [Method: %s Range: %d] ETRI_INC_RASTER : %s\n", 
				GetBoolVal(ETRI_FAST_MOTION_ESTIMATION), 
				((pcTEncTop->getFastSearch() == 2)? "ETRI_Fast_IME" : "HM TZSearch"), 
				pcTEncTop->getSearchRange(), 
				GetBoolVal(ETRI_INC_RASTER));
	EDPRINTF(stderr, "    ETRI_NOT_QPEL_ME      :%s \n", GetBoolVal(ETRI_NOT_QUARTERPEL_ME));
	EDPRINTF(stderr, "    ETRI_HALF_DIAMOND_ME  :%s \n", GetBoolVal(ETRI_FME_HALFPEL_DIAMOND_ON));
	EDPRINTF(stderr, " ETRI_SliceEncoder_MVClip :%s \n", GetBoolVal(ETRI_SliceEncoder_MVClip));
#endif 
#if ETRI_FAST_CU_METHODS
	EDPRINTF(stderr, " ETRI_ADPTIVE_MAXCTU_SIZE :%s \n", GetBoolVal(ETRI_ADAPTIVE_MAXCTU_SIZE));
	EDPRINTF(stderr, " ETRI_ADPTIVE_MINCTU_SIZE :%s \n", GetBoolVal(ETRI_ADAPTIVE_MINCTU_SIZE));
#endif 
#if ETRI_FAST_PU_METHODS
	EDPRINTF(stderr, " ETRI_INTRA_LUMA_MODE_DEC :%s \n", GetBoolVal(ETRI_INTRA_LUMA_MODE_DECS));
	EDPRINTF(stderr, " ETRI_DISABLE_INTRA4x4GPB :%s \n", GetBoolVal(ETRI_DISABLE_INTRA4x4_GPB));
	EDPRINTF(stderr, " ETRI_SAD_SUBSAMPLING     :%s \n", GetBoolVal(ETRI_COARSE_SAD_SUBSAMPLING));
#endif 
#endif
#if ETRI_MODIFICATION_V03 
	EDPRINTF(stderr, " ETRI_AMVP_ACCELERATION   :%s [%d] \n", GetBoolVal(ETRI_AMVP_ACCELERATION), ETRI_AMVP_ACCELERATION);
	#if ETRI_FAST_INTEGERME
	EDPRINTF(stderr, " ETRI_FAST_INTEGERME      :%s [Method: %s Range: %d] FAST BI-PRED: %s\n", 
				GetBoolVal(ETRI_FAST_INTEGERME), 
				((pcTEncTop->getFastSearch() == 2)? "ETRI_Fast_IME" : "HM TZSearch"), 
				pcTEncTop->getSearchRange(), 
				GetBoolVal((ETRI_FAST_BIPRED && pcTEncTop->getFastSearch() >= 2)));
	#endif

	EDPRINTF(stderr, " ETRI_FASTPUProcessing    :%s \n", GetBoolVal(ETRI_FASTPUProcessing));
#if ETRI_FASTPUProcessing
	EDPRINTF(stderr, "    ETRI_TEST_FASTINPRED  :%s \n", GetBoolVal(ETRI_TEST_FASTINPRED));
	EDPRINTF(stderr, "    ETRI_TEST_FASTITPRED  :%s \n", GetBoolVal(ETRI_TEST_FASTITPRED));
	EDPRINTF(stderr, "    ETRI_FAST_MGITRDOQ    :%s \n", GetBoolVal(ETRI_FAST_MGITRDOQ));
	EDPRINTF(stderr, "    ETRI_FAST_MGINRDOQ    :%s \n", GetBoolVal(ETRI_FAST_MGINRDOQ));	
	EDPRINTF(stderr, "    ETRI_FAST_INITRDOQ    :%s \n", GetBoolVal(ETRI_FAST_INITRDOQ));
	#if ETRI_FFESD
	EDPRINTF(stderr, "    ETRI_FFESD            :%s (Level: %d) \n", GetBoolVal(ETRI_FFESD), ETRI_FFESDLEVEL);
	#endif
	EDPRINTF(stderr, "    ETRI_FastIntraSKIP    :%s \n", GetBoolVal(ETRI_FastIntraSKIP));
	ESPRINTF(ETRI_TEST_1201, stderr, "    ETRI_TEST_1201        :%s \n", GetBoolVal(ETRI_TEST_1201));
	ESPRINTF(ETRI_TEST_1202, stderr, "    ETRI_TEST_1202        :%s \n", GetBoolVal(ETRI_TEST_1202));
#endif
#endif 	/// ETRI_MODIFICATION_V03 

/// ETRI_HDR_WCG_ENCODER
	EDPRINTF(stderr, " ETRI_HDR_WCG_ENCODER    :%s \n", GetBoolVal(ETRI_HDR_WCG_ENCODER));
	EDPRINTF(stderr, " FULL_NBIT               :%s \n", GetBoolVal(FULL_NBIT));

	EDPRINTF(stderr, "------------------------------------------ \n");
	EDPRINTF(stderr, " Compiled @%s  [%s] \n\n", __DATE__, __TIME__);
#endif	

}
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move 
SEIActiveParameterSets* TEncGOP::xCreateSEIActiveParameterSets (TComSPS *sps)
{
  SEIActiveParameterSets *seiActiveParameterSets = new SEIActiveParameterSets(); 
  seiActiveParameterSets->activeVPSId = m_pcCfg->getVPS()->getVPSId(); 
  seiActiveParameterSets->m_selfContainedCvsFlag = false;
  seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
  seiActiveParameterSets->numSpsIdsMinus1 = 0;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1); 
  seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
  return seiActiveParameterSets;
}

SEIFramePacking* TEncGOP::xCreateSEIFramePacking()
{
  SEIFramePacking *seiFramePacking = new SEIFramePacking();
  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  assert((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) );
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && m_iNumPicCoded&1);
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
  return seiFramePacking;
}

SEIDisplayOrientation* TEncGOP::xCreateSEIDisplayOrientation()
{
  SEIDisplayOrientation *seiDisplayOrientation = new SEIDisplayOrientation();
  seiDisplayOrientation->cancelFlag = false;
  seiDisplayOrientation->horFlip = false;
  seiDisplayOrientation->verFlip = false;
  seiDisplayOrientation->anticlockwiseRotation = m_pcCfg->getDisplayOrientationSEIAngle();
  return seiDisplayOrientation;
}

SEIToneMappingInfo*  TEncGOP::xCreateSEIToneMappingInfo()
{
  SEIToneMappingInfo *seiToneMappingInfo = new SEIToneMappingInfo();
  seiToneMappingInfo->m_toneMapId = m_pcCfg->getTMISEIToneMapId();
  seiToneMappingInfo->m_toneMapCancelFlag = m_pcCfg->getTMISEIToneMapCancelFlag();
  seiToneMappingInfo->m_toneMapPersistenceFlag = m_pcCfg->getTMISEIToneMapPersistenceFlag();

  seiToneMappingInfo->m_codedDataBitDepth = m_pcCfg->getTMISEICodedDataBitDepth();
  assert(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14);
  seiToneMappingInfo->m_targetBitDepth = m_pcCfg->getTMISEITargetBitDepth();
  assert( seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17 );
  seiToneMappingInfo->m_modelId = m_pcCfg->getTMISEIModelID();
  assert(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4);

  switch( seiToneMappingInfo->m_modelId)
  {
  case 0:
    {
      seiToneMappingInfo->m_minValue = m_pcCfg->getTMISEIMinValue();
      seiToneMappingInfo->m_maxValue = m_pcCfg->getTMISEIMaxValue();
      break;
    }
  case 1:
    {
      seiToneMappingInfo->m_sigmoidMidpoint = m_pcCfg->getTMISEISigmoidMidpoint();
      seiToneMappingInfo->m_sigmoidWidth = m_pcCfg->getTMISEISigmoidWidth();
      break;
    }
  case 2:
    {
      UInt num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
      seiToneMappingInfo->m_startOfCodedInterval.resize(num);
      Int* ptmp = m_pcCfg->getTMISEIStartOfCodedInterva();
      if(ptmp)
      {
        for(int i=0; i<num;i++)
        {
          seiToneMappingInfo->m_startOfCodedInterval[i] = ptmp[i];
        }
      }
      break;
    }
  case 3:
    {
      seiToneMappingInfo->m_numPivots = m_pcCfg->getTMISEINumPivots();
      seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
      seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
      Int* ptmpcoded = m_pcCfg->getTMISEICodedPivotValue();
      Int* ptmptarget = m_pcCfg->getTMISEITargetPivotValue();
      if(ptmpcoded&&ptmptarget)
      {
        for(int i=0; i<(seiToneMappingInfo->m_numPivots);i++)
        {
          seiToneMappingInfo->m_codedPivotValue[i]=ptmpcoded[i];
          seiToneMappingInfo->m_targetPivotValue[i]=ptmptarget[i];
         }
       }
       break;
     }
  case 4:
     {
       seiToneMappingInfo->m_cameraIsoSpeedIdc = m_pcCfg->getTMISEICameraIsoSpeedIdc();
       seiToneMappingInfo->m_cameraIsoSpeedValue = m_pcCfg->getTMISEICameraIsoSpeedValue();
       assert( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 );
       seiToneMappingInfo->m_exposureIndexIdc = m_pcCfg->getTMISEIExposurIndexIdc();
       seiToneMappingInfo->m_exposureIndexValue = m_pcCfg->getTMISEIExposurIndexValue();
       assert( seiToneMappingInfo->m_exposureIndexValue !=0 );
       seiToneMappingInfo->m_exposureCompensationValueSignFlag = m_pcCfg->getTMISEIExposureCompensationValueSignFlag();
       seiToneMappingInfo->m_exposureCompensationValueNumerator = m_pcCfg->getTMISEIExposureCompensationValueNumerator();
       seiToneMappingInfo->m_exposureCompensationValueDenomIdc = m_pcCfg->getTMISEIExposureCompensationValueDenomIdc();
       seiToneMappingInfo->m_refScreenLuminanceWhite = m_pcCfg->getTMISEIRefScreenLuminanceWhite();
       seiToneMappingInfo->m_extendedRangeWhiteLevel = m_pcCfg->getTMISEIExtendedRangeWhiteLevel();
       assert( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 );
       seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = m_pcCfg->getTMISEINominalBlackLevelLumaCodeValue();
       seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = m_pcCfg->getTMISEINominalWhiteLevelLumaCodeValue();
       assert( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue );
       seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = m_pcCfg->getTMISEIExtendedWhiteLevelLumaCodeValue();
       assert( seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue >= seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue );
       break;
    }
  default:
    {
      assert(!"Undefined SEIToneMapModelId");
      break;
    }
  }
  return seiToneMappingInfo;
}

Void TEncGOP::xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled())
  {
    SEIActiveParameterSets *sei = xCreateSEIActiveParameterSets (sps);

    //nalu = NALUnit(NAL_UNIT_SEI); 
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
    m_activeParameterSetSEIPresentInAU = true;
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = xCreateSEIFramePacking ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = xCreateSEIDisplayOrientation();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI); 
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = xCreateSEIToneMappingInfo ();
      
    nalu = NALUnit(NAL_UNIT_PREFIX_SEI); 
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
}
#endif
// ====================================================================================================================
// Public member functions
// ====================================================================================================================
// ====================================================================================================================
// ETRI Compress GOP Functions @ 2015 5 11 by Seok
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: For Field Processing to corresponding to iGOPid. The Following Functions are defined under EFFICIENT_FIELD_IRAP = 1 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncGOP::ETRI_EfficientFieldIRAPProc(Int iPOCLast, Int iNumPicRcvd, bool isField, Int& IRAPGOPid, Bool& IRAPtoReorder, Bool& swapIRAPForward, Bool bOperation)
{
	if (!bOperation || !isField)	{return;}	///2015 5 11 by Seok : This Function is defined when EFFICIENT_FIELD_IRAP = 1     

	Int pocCurr;
	for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
	{
		// determine actual POC
		if(iPOCLast == 0) //case first frame or first top field
		{
			pocCurr=0;
		}
		else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
		{
			pocCurr = 1;
		}
		else
		{
			pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
		}

		// check if POC corresponds to IRAP
		NalUnitType tmpUnitType = getNalUnitType(pocCurr, m_iLastIDR, isField);
		if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
		{
			if((pocCurr & 1) == 0 && iGOPid < m_iGopSize-1 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid+1).m_POC-1)
			{ // if top field and following picture in enc order is associated bottom field
				IRAPGOPid = iGOPid;
				IRAPtoReorder = true;
				swapIRAPForward = true; 
				break;
			}
			if((pocCurr & 1) != 0 && iGOPid > 0 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid-1).m_POC+1)
			{// if picture is an IRAP remember to process it first
				IRAPGOPid = iGOPid;
				IRAPtoReorder = true;
				swapIRAPForward = false; 
				break;
			}
		}
	}
}

__inline	Int TEncGOP::ETRI_iGOPIRAPtoReorder_v1(Int iGOPid, Int IRAPGOPid, Bool IRAPtoReorder, Bool swapIRAPForward, Bool bOperation)
{
	if (!bOperation){return iGOPid;}
	if(IRAPtoReorder){
	  if(swapIRAPForward){
		if(iGOPid == IRAPGOPid)  		{iGOPid = IRAPGOPid +1;}
		else if(iGOPid == IRAPGOPid +1)	{iGOPid = IRAPGOPid;}
	  }
	  else{
		if(iGOPid == IRAPGOPid -1)  	{iGOPid = IRAPGOPid;}
		else if(iGOPid == IRAPGOPid)	{iGOPid = IRAPGOPid -1;}
	  }
	}

	return iGOPid;
}

__inline	Int TEncGOP::ETRI_iGOPIRAPtoReorder_v2(Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, Bool bOperation)
{
	if (!bOperation){return iGOPid;}
	if(IRAPtoReorder){
		if(swapIRAPForward){
			if 	(iGOPid == IRAPGOPid)    	{iGOPid = IRAPGOPid +1; IRAPtoReorder = false;}
			else if(iGOPid == IRAPGOPid +1)	{iGOPid --;}
		}
		else{
			if 	(iGOPid == IRAPGOPid)    	{iGOPid = IRAPGOPid -1;}
			else if(iGOPid == IRAPGOPid -1)	{iGOPid = IRAPGOPid;	IRAPtoReorder = false;}
		}
	}
	return iGOPid;
}

__inline	Int TEncGOP::ETRI_iGOPIRAPtoReorder_v3(Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, Bool bOperation)
{
	if (!bOperation){return iGOPid;}
	if(IRAPtoReorder)
	{
		if(swapIRAPForward)
		{
			if  	(iGOPid == IRAPGOPid) 	{iGOPid = IRAPGOPid +1;	IRAPtoReorder = false;	}
			else if(iGOPid == IRAPGOPid +1)	{iGOPid --;}
		}
		else
		{
			if  	(iGOPid == IRAPGOPid)  	{iGOPid = IRAPGOPid -1;}
			else if(iGOPid == IRAPGOPid -1)	{iGOPid = IRAPGOPid; IRAPtoReorder = false;}
		}
	}
	return iGOPid;
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Amendment of First POC to a Frame or Field. This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Bool TEncGOP::ETRI_FIrstPOCForFieldFrame(Int& pocCurr, Int& iTimeOffset, Int iPOCLast, Int iNumPicRcvd, Int iGOPid, Int IRAPGOPid, Bool& IRAPtoReorder, Bool swapIRAPForward, bool isField)
{

	if(iPOCLast == 0){ //case first frame or first top field
		pocCurr=0;	iTimeOffset = 1;
	}
	else if(iPOCLast == 1 && isField){ //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
		pocCurr = 1;	iTimeOffset = 1;
	}
	else{
		pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
		iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
	}

	if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
	{
		iGOPid = ETRI_iGOPIRAPtoReorder_v2(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);	///iGOPid Reordering and IRAPtoReordering when Field Coding
		return true;  ///In for Routine in the Called Function, it must be continued when this condition is active 
	}

	if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
	{
		m_iLastIDR = pocCurr;
	}
	return false;
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Calculate PSNR. Including PSNR calculation for Interlaced Picture, we make other version of xcalculateAddPSNR
			It is a Service Function. However, this Function should be move to the TencFrame Member Functions, owing to Frame Parallelism
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/

void TEncGOP::ETRI_xCalculateAddPSNR(TComPic* pcPic, TComList<TComPic*>& rcListPic, AccessUnit& accessUnit, Double dEncTime, Bool isField, Bool isTff)
{
	xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime );

	//In case of field coding, compute the interlaced PSNR for both fields
	if (isField && ((!pcPic->isTopField() && isTff) || (pcPic->isTopField() && !isTff)) && (pcPic->getPOC()%m_iGopSize != 1))
	{
	  //get complementary top field
	  TComPic* pcPicTop;
	  TComList<TComPic*>::iterator	 iterPic = rcListPic.begin();
	  while ((*iterPic)->getPOC() != pcPic->getPOC()-1)
	  {
		iterPic ++;
	  }
	  pcPicTop = *(iterPic);
	  xCalculateInterlacedAddPSNR(pcPicTop, pcPic, pcPicTop->getPicYuvRec(), pcPic->getPicYuvRec(), accessUnit, dEncTime );
	}
	else if (isField && pcPic->getPOC()!= 0 && (pcPic->getPOC()%m_iGopSize == 0))
	{
	  //get complementary bottom field
	  TComPic* pcPicBottom;
	  TComList<TComPic*>::iterator	 iterPic = rcListPic.begin();
	  while ((*iterPic)->getPOC() != pcPic->getPOC()+1)
	  {
		iterPic ++;
	  }
	  pcPicBottom = *(iterPic);
	  xCalculateInterlacedAddPSNR(pcPic, pcPicBottom, pcPic->getPicYuvRec(), pcPicBottom->getPicYuvRec(), accessUnit, dEncTime );
	}
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: TO Write Out SEI data,  Set the SEI data of TemporalLevel0Index
			This Function can be move to the TencFrame Member Functions, owing to Frame Parallelism
			However, a new member data such as m_tl0Idx, m_rapIdx are needed in TEncFrame.
			Thus, this function is defined in the TEncGOP and called in TEncFrame.
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------	
*/
Void TEncGOP::ETRI_SetSEITemporalLevel0Index(TComSlice *pcSlice, SEITemporalLevel0Index& sei_temporal_level0_index)
{
	if (pcSlice->getRapPicFlag())
	{
		m_tl0Idx = 0;
		m_rapIdx = (m_rapIdx + 1) & 0xFF;
	}
	else
	{
		m_tl0Idx = (m_tl0Idx + (pcSlice->getTLayer() ? 0 : 1)) & 0xFF;
	}
	sei_temporal_level0_index.tl0Idx = m_tl0Idx;
	sei_temporal_level0_index.rapIdx = m_rapIdx;
}


#define	ETRI_SET_SISLICEINFO(e_sISliceInfo) \
	e_sISliceInfo.actualHeadBits    			= &actualHeadBits; \
	e_sISliceInfo.actualTotalBits    			= &actualTotalBits; \
	e_sISliceInfo.estimatedBits      			= &estimatedBits; \
	e_sISliceInfo.lambda    					= &lambda; \
	e_sISliceInfo.uiNumSlices   				= &uiNumSlices; \
	e_sISliceInfo.uiRealEndAddress   			= &uiRealEndAddress; \
	e_sISliceInfo.uiInternalAddress   			= &uiInternalAddress; \
	e_sISliceInfo.uiExternalAddress   			= &uiExternalAddress; \
	e_sISliceInfo.uiPosX    					= &uiPosX; \
	e_sISliceInfo.uiPosY    					= &uiPosY; \
	e_sISliceInfo.uiWidth    					= &uiWidth; \
	e_sISliceInfo.uiHeight    					= &uiHeight; \
	e_sISliceInfo.iNumSubstreams    			= &iNumSubstreams; \
	e_sISliceInfo.startCUAddrSlice   			= &startCUAddrSlice;\
	e_sISliceInfo.startCUAddrSliceIdx    		= &startCUAddrSliceIdx; \
	e_sISliceInfo.startCUAddrSliceSegment 		= &startCUAddrSliceSegment; \
	e_sISliceInfo.startCUAddrSliceSegmentIdx 	= &startCUAddrSliceSegmentIdx; \
	e_sISliceInfo.nextCUAddr    				= &nextCUAddr; \
	e_sISliceInfo.picSptDpbOutputDuDelay 		= &picSptDpbOutputDuDelay; \
	e_sISliceInfo.accumBitsDU   				= accumBitsDU; \
	e_sISliceInfo.accumNalsDU   				= accumNalsDU; \
	e_sISliceInfo.tmpBitsBeforeWriting   		= &tmpBitsBeforeWriting; \
	e_sISliceInfo.uiOneBitstreamPerSliceLength 	= &uiOneBitstreamPerSliceLength; \
	e_sISliceInfo.picSptDpbOutputDuDelay 		= &picSptDpbOutputDuDelay;
	
#if ETRI_MODIFICATION_V00
/**
=======================================================================================================================
	@brief :  The main function of GOP (not between IDR) encoding. it is included almost basic structure of HEVC encoder
	@param: Int iPOCLast,
	@param: Int iNumPicRcvd, 
	@param: TComList<TComPic*>& rcListPic, 
	@param: TComList<TComPicYuv*>& rcListPicYuvRecOut, 
	@param: std::list<AccessUnit>& accessUnitsInGOP, 
	@param: bool isField, 
	@param: bool isTff
	@author: Jinwuk Seok : 2015 5 14 
=======================================================================================================================
*/
#if ETRI_MULTITHREAD_2
Void TEncGOP::ETRI_compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, AccessUnit_t* accessUnitsInGOP, bool isField, bool isTff)
#else
Void TEncGOP::ETRI_compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP, bool isField, bool isTff)
#endif
{
	if (!ETRI_MODIFICATION_V00)	{return;}

	TComPic*        pcPic;							///Remove  2015 5 23 by Seok
	TComPicYuv*     pcPicYuvRecOut;					///Remove  2015 5 23 by Seok : need parallelism 
	
	TComOutputBitstream  *pcBitstreamRedirect;		/// need parallelism @ 2015 5 23 by Seok : 
	pcBitstreamRedirect = new TComOutputBitstream;	/// Is it Good?? Is there any alternative method?? @ 2015 5 25 by Seok 
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee	
	UInt	eEncOrder = 0;	
	short   eSliceIndex = m_pcEncTop->ETRI_getETRI_SliceIndex();
#endif
	xInitGOP( iPOCLast, iNumPicRcvd, rcListPic, rcListPicYuvRecOut, isField );

	UInt *accumBitsDU = NULL;
	UInt *accumNalsDU = NULL;
	Int  	iTimeOffset, pocCurr;

	// #if EFFICIENT_FIELD_IRAP
	Int    	IRAPGOPid 			= -1;   	///Originally This params are defined under EFFICIENT_FIELD_IRAP 
	Bool 	IRAPtoReorder 		= false; 	///However, For simple Code work, we defined this params to work in general case.
	Bool 	swapIRAPForward 	= false; 	///2015 5 11 by Seok
	// #endif 
#if ETRI_MULTITHREAD_2
	int nRefCnt		= 0;	
	int	nThreadID	= 0;
	int iGOPid = 0;
	Int nCount = 0;
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	int iOffset = 0;
#endif
#endif

	m_iNumPicCoded = 0;
	ETRI_EfficientFieldIRAPProc(iPOCLast, iNumPicRcvd, isField, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);

#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	int Size;
	if (m_iGopSize == 1) Size = 1;
	else Size = m_pcEncTop->getIntraPeriod();
	iOffset = (m_totalCoded / m_iGopSize)*m_iGopSize;
#endif

#if (_ETRI_WINDOWS_APPLICATION)
	long iBeforeTime;
#else
	timespec iBeforeTime;
#endif
#if _YHEEDEBUG
	Double dEncTime;
#endif

#if (ETRI_THREAD_SEL == ETRI_SEQUENCE)  // 순차적 처리
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	for (Int iPos = 0; iPos < Size; iPos++)
	{
#if (_ETRI_WINDOWS_APPLICATION)
		iBeforeTime = clock();
#else
		clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
		iTimeOffset = pocCurr = 0;

		if (Size != 1)
		{
			iNumPicRcvd = m_iGopSize;
			iPOCLast = ((iPos / m_iGopSize) + 1) * m_iGopSize + iOffset;
			iGOPid = iPos % m_iGopSize;
		}

		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
		{
			accessUnitsInGOP[iPos].pos = -1;
			continue;
		}

		// Read One Picture for Frame Compression from INput Picture Buffer
		ETRI_xGetBuffer(rcListPic, rcListPicYuvRecOut, pcPic, pcPicYuvRecOut, pocCurr, iPos, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[0].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);
		accessUnitsInGOP[iPos].poc = pocCurr;
		accessUnitsInGOP[iPos].pos = iPos;
		em_pcFrameEncoder[0].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iPos].outputAccessUnits, m_bFirst);

		// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder->ETRI_getAccessUnitFrame(), em_pcFrameEncoder->ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
		printf("\n");	fflush(stdout);
#if _YHEEDEBUG	
		dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
		printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

	}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )

#else // (ETRI_PARALLEL_SEL ==  ETRI_FRAME_PARALLEL  || ETRI_PARALLEL_SEL ==  TILE_ONLY)
	for (iGOPid = 0; iGOPid < m_iGopSize; iGOPid++)
	{
#if (_ETRI_WINDOWS_APPLICATION)
		long lBefore = clock();
#else
		timespec lBefore;
		clock_gettime(CLOCK_MONOTONIC, &lBefore); // Works on Linux by yhee 2016.04.19
#endif
		iTimeOffset = pocCurr = 0;
		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
		{
			accessUnitsInGOP[iGOPid].poc = pocCurr;
			accessUnitsInGOP[iGOPid].pos = -1;
			continue;
		}

		// Read One Picture for Frame Compression from INput Picture Buffer
		xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[0].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

		accessUnitsInGOP[iGOPid].poc = pocCurr;
		accessUnitsInGOP[iGOPid].pos = iGOPid;
		em_pcFrameEncoder[0].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst);

		//num++;
		//num %= MAX_THREAD_GOP;

		// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder->ETRI_getAccessUnitFrame(), em_pcFrameEncoder->ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
		printf("\n");	fflush(stdout);
#if _YHEEDEBUG
		dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
		printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);		
		//printf("\n");	fflush(stdout);
#endif
#endif
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

	}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
#endif // (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)

#elif (ETRI_THREAD_SEL == ETRI_SEQ_ARRAY)
#if (ETRI_PARALLEL_SEL == ETRI_FRAME_PARALLEL) //Serial_Frame
	if (m_iGopSize == 1)
	{
	
#if (_ETRI_WINDOWS_APPLICATION)
        long lBefore = clock();
#else
        timespec lBefore;
        clock_gettime(CLOCK_MONOTONIC, &lBefore); // Works on Linux by yhee 2016.04.19
#endif
		iTimeOffset = pocCurr = 0;
		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField);

		// Read One Picture for Frame Compression from INput Picture Buffer
		xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[iGOPid].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

		accessUnitsInGOP[iGOPid].poc = pocCurr;
		accessUnitsInGOP[iGOPid].pos = iGOPid;


#if ETRI_THREADPOOL_OPT
    em_pcFrameEncoder[iGOPid].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst,
		m_prevBits[1], m_initialRemainingBits[0], m_initialRemainingFrames, m_remainingBits, m_remainingFrames, targetbitsTemp, usedbittemp2, compressflagtemp2, frameqptemp2, -1, -1, weightInIDR_temp, iSCIndex, iSATD, m_cpbState[0][0], m_bufferingRate, m_cpbSize, m_cpbStateFlag[0][0], NULL, NULL, NULL, m_qrThreadpool);
#else
	em_pcFrameEncoder[iGOPid].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst,
		m_prevBits[1], m_initialRemainingBits[0], m_initialRemainingFrames, m_remainingBits, m_remainingFrames, targetbitsTemp, usedbittemp2, compressflagtemp2, frameqptemp2, -1, -1, weightInIDR_temp, iSCIndex, iSATD, m_cpbState[0][0], m_bufferingRate, m_cpbSize, m_cpbStateFlag[0][0]);
#endif

		// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder[iGOPid].ETRI_getAccessUnitFrame(), em_pcFrameEncoder[iGOPid].ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
		printf("\n");	fflush(stdout);
#if _YHEEDEBUG
		dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
		printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
		//printf("\n");	fflush(stdout);
#endif
#endif
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding
	}
	else 
	{
		for (Int iPos = 0; iPos < m_iGopSize; iPos++)
		{
			iTimeOffset = pocCurr = 0;

			iGOPid = ETRI_iGOPIRAPtoReorder_v1(iPos, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
			if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
			{
				accessUnitsInGOP[iGOPid].poc = pocCurr;
				accessUnitsInGOP[iGOPid].pos = -1;
				continue;
			}

			// Read One Picture for Frame Compression from INput Picture Buffer
			xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

			// Set Parameter for Frame Compression 
			em_pcFrameEncoder[iGOPid].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);
			em_pcFrameEncoder[iGOPid].ETRI_GetRefPic(&em_refPic[nRefCnt++], pocCurr, pcPic, rcListPic, iGOPid, isField, iPOCLast, iNumPicRcvd, m_iLastIDR);
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
			UInt eFrameType = pcPic->getSlice(0)->getSliceType();
			m_pcEncTop->ETRI_setFrameInfoforDLL(eEncOrder, iGOPid, eFrameType, pocCurr, eSliceIndex);
			eEncOrder++;
#endif
		}

		for (iGOPid = 0; iGOPid < m_iGopSize; iGOPid++)
		{
#if (_ETRI_WINDOWS_APPLICATION)
			iBeforeTime = clock();
#else
			clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
			iTimeOffset = pocCurr = 0;
			iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
			if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField)){ continue; }

			// Read One Picture for Frame Compression from INput Picture Buffer
			xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

			// Set Parameter for Frame Compression 
			em_pcFrameEncoder[iGOPid].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

			accessUnitsInGOP[iGOPid].poc = pocCurr;
			accessUnitsInGOP[iGOPid].pos = iGOPid;


#if ETRI_THREADPOOL_OPT
      em_pcFrameEncoder[iGOPid].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst,
		  m_prevBits[1], m_initialRemainingBits[0], m_initialRemainingFrames, m_remainingBits, m_remainingFrames, targetbitsTemp, usedbittemp2, compressflagtemp2, frameqptemp2, -1, -1, weightInIDR_temp, iSCIndex, iSATD, m_cpbState[0][0], m_bufferingRate, m_cpbSize, m_cpbStateFlag[0][0], NULL, NULL, NULL, m_qrThreadpool, false);
#else
	  em_pcFrameEncoder[iGOPid].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst,
		  m_prevBits[1], m_initialRemainingBits[0], m_initialRemainingFrames, m_remainingBits, m_remainingFrames, targetbitsTemp, usedbittemp2, compressflagtemp2, frameqptemp2, -1, -1, weightInIDR_temp, iSCIndex, iSATD, m_cpbState[0][0], m_bufferingRate, m_cpbSize, m_cpbStateFlag[0][0], false);
#endif

			// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
			ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder[iGOPid].ETRI_getAccessUnitFrame(), em_pcFrameEncoder[iGOPid].ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
			printf("\n");	fflush(stdout);
#if _YHEEDEBUG
			dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
			printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
			m_bFirst = false;
			m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

			iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

		}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
	}
#else //Serial_GOP
	for (Int iPos = 0; iPos < Size; iPos++)
	{
		iTimeOffset = pocCurr = 0;

		if (Size != 1)
		{
			iNumPicRcvd = m_iGopSize;
			iPOCLast = ((iPos / m_iGopSize) + 1) * m_iGopSize + iOffset;
			iGOPid = iPos % m_iGopSize;
		}

		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
		{
			accessUnitsInGOP[iPos].poc = pocCurr;
			accessUnitsInGOP[iPos].pos = -1;
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee
			eEncOrder++;
#endif
			continue;
		}

		// Read One Picture for Frame Compression from INput Picture Buffer
		ETRI_xGetBuffer(rcListPic, rcListPicYuvRecOut, pcPic, pcPicYuvRecOut, pocCurr, iPos, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);
		em_pcFrameEncoder[iPos].ETRI_GetRefPic(&em_refPic[nRefCnt++], pocCurr, pcPic, rcListPic, iGOPid, isField, iPOCLast, iNumPicRcvd, m_iLastIDR);
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
		UInt eFrameType = pcPic->getSlice(0)->getSliceType();
		m_pcEncTop->ETRI_setFrameInfoforDLL(eEncOrder, iGOPid, eFrameType, pocCurr, eSliceIndex);
		eEncOrder++;
#endif
	}

	for (Int iPos = 0; iPos < Size; iPos++)
	{
#if (_ETRI_WINDOWS_APPLICATION)
		iBeforeTime = clock();
#else
		clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
		iTimeOffset = pocCurr = 0;

		if (Size != 1)
		{
			iNumPicRcvd = m_iGopSize;
			iPOCLast = ((iPos / m_iGopSize) + 1) * m_iGopSize + iOffset;
			iGOPid = iPos % m_iGopSize;
		}

		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField)){ continue; }

		// Read One Picture for Frame Compression from INput Picture Buffer
		ETRI_xGetBuffer(rcListPic, rcListPicYuvRecOut, pcPic, pcPicYuvRecOut, pocCurr, iPos, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

		accessUnitsInGOP[iPos].poc = pocCurr;
		accessUnitsInGOP[iPos].pos = iPos;
		em_pcFrameEncoder[iPos].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iPos].outputAccessUnits, m_bFirst, false);

		// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder[iPos].ETRI_getAccessUnitFrame(), em_pcFrameEncoder[iPos].ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok

		printf("\n");	fflush(stdout);
#if _YHEEDEBUG
		dEncTime = (Double)(clock() - iBeforeTime) / CLOCKS_PER_SEC;
		printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

	}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
#endif
#elif (ETRI_THREAD_SEL == ETRI_MULTI_ARRAY)
#if (ETRI_PARALLEL_SEL == ETRI_FRAME_PARALLEL) //FrameParallel
	if (m_iGopSize == 1)
	{
#if (_ETRI_WINDOWS_APPLICATION)
		iBeforeTime = clock();
#else
		clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
		iTimeOffset = pocCurr = 0;
		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField);

		// Read One Picture for Frame Compression from INput Picture Buffer
		xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[iGOPid].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

		accessUnitsInGOP[iGOPid].poc = pocCurr;
		accessUnitsInGOP[iGOPid].pos = iGOPid;
		em_pcFrameEncoder[iGOPid].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iGOPid].outputAccessUnits, m_bFirst);
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
		m_pcEncTop->ETRI_setFrameInfoforDLL(eEncOrder, iGOPid, em_pcFrameEncoder[iGOPid].ETRI_getFrameTypeInGOP(), pocCurr, eSliceIndex);
		eEncOrder++;
#endif
		// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder->ETRI_getAccessUnitFrame(), em_pcFrameEncoder->ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
		printf("\n");	fflush(stdout);
#if _YHEEDEBUG
		dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
		printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
		//printf("\n");	fflush(stdout);
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding
	}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
	else
	{
		//bool *bThreadUsed = new bool[m_iGopSize];  //gplusplus_151124 local variable -> class member variable
		int nThreadCnt = 0;
		memset(em_bThreadUsed, 0x00, sizeof(bool)*MAX_THREAD_SIZE);

		for (Int iPos = 0; iPos < m_iGopSize; iPos++)
		{
			iTimeOffset = pocCurr = 0;

			iGOPid = ETRI_iGOPIRAPtoReorder_v1(iPos, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
			if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
			{
				continue;
			}

			// Read One Picture for Frame Compression from INput Picture Buffer
			xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

			em_pcFrameEncoder[iGOPid].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);
			em_pcFrameEncoder[iGOPid].ETRI_GetRefPic(&em_refPic[nRefCnt++], pocCurr, pcPic, rcListPic, iGOPid, isField, iPOCLast, iNumPicRcvd, m_iLastIDR);
#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
			UInt eFrameType = pcPic->getSlice(0)->getSliceType();
			m_pcEncTop->ETRI_setFrameInfoforDLL(eEncOrder, iGOPid, eFrameType, pocCurr, eSliceIndex);
			eEncOrder++;
#endif
		}

		while (nCount < m_iGopSize)
		{
			for (Int iPos = 0; iPos < m_iGopSize; iPos++)
			{
				iTimeOffset = pocCurr = 0;
				if(em_bThreadUsed[iPos])
					continue;

				iGOPid = ETRI_iGOPIRAPtoReorder_v1(iPos, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
				if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
				{
					accessUnitsInGOP[iGOPid].poc = pocCurr;
					accessUnitsInGOP[iGOPid].pos = -1;
					em_bThreadUsed[iPos] = true;
					nCount++;
					continue;
				}

				if (!ETRI_RefPicCheck(pocCurr, &rcListPic, nRefCnt))
				{
					//Sleep(1);
					continue;
				}

				// Read One Picture for Frame Compression from INput Picture Buffer
				xGetBuffer(rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

				// 여유 있는 스레드를 찾는다.
				nThreadID = em_hThreadPoolGOP.GetFreeThreadID();
				//gplusplus_151124 Thread Function -> Local Function
				em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

				em_bThreadUsed[iPos] = true;
				accessUnitsInGOP[iPos].poc = pocCurr;
				accessUnitsInGOP[iPos].pos = iGOPid;

				em_cThreadGOP[nThreadID].em_rpcListPic			= &rcListPic;
				em_cThreadGOP[nThreadID].em_rpcListPicYuvRecOut= &rcListPicYuvRecOut;
				em_cThreadGOP[nThreadID].em_paccessUnitsInGOP	= &accessUnitsInGOP[iPos].outputAccessUnits;
				em_cThreadGOP[nThreadID].em_pcPic				= pcPic;
				em_cThreadGOP[nThreadID].em_pcPicYuvRecOut		= pcPicYuvRecOut;
				em_cThreadGOP[nThreadID].em_iPos				= iPos;
				em_cThreadGOP[nThreadID].em_pocCurr				= pocCurr;
				em_cThreadGOP[nThreadID].em_bFirst				= m_bFirst;
				em_cThreadGOP[nThreadID].em_isField				= isField;
				em_cThreadGOP[nThreadID].em_isTff				= isTff;

				em_hThreadPoolGOP.AddThread(this, nThreadID);
				nCount++;
				nThreadCnt++;


			}
		}

		// 모든 스레드가 끝날때까지 기다린다.
		em_hThreadPoolGOP.GetAllFreeThreadWaiting();
		m_iNumPicCoded += nThreadCnt; m_totalCoded += nThreadCnt;
		//printf("//////////////// Thread Join here /////////////\n");	fflush(stdout);
		//delete[] bThreadUsed;
	}
#else //GOPParallel
	//bool *bThreadUsed = new bool[Size]; //gplusplus_151124 local variable -> class member variable
	int nThreadCnt = 0;
	memset(em_bThreadUsed, 0x00, sizeof(bool)*MAX_THREAD_SIZE);

	for (Int iPos = 0; iPos < Size; iPos++)
	{
		iTimeOffset = pocCurr = 0;

		if (Size != 1)
		{
			iNumPicRcvd = m_iGopSize;
			iPOCLast = ((iPos / m_iGopSize) + 1) * m_iGopSize + iOffset;
			iGOPid = iPos % m_iGopSize;
		}

		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
		{
			continue;
		}
		
		// Read One Picture for Frame Compression from INput Picture Buffer
		ETRI_xGetBuffer(rcListPic, rcListPicYuvRecOut, pcPic, pcPicYuvRecOut, pocCurr, iPos, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);
		em_pcFrameEncoder[iPos].ETRI_GetRefPic(&em_refPic[nRefCnt++], pocCurr, pcPic, rcListPic, iGOPid, isField, iPOCLast, iNumPicRcvd, m_iLastIDR);

#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
		UInt eFrameType = pcPic->getSlice(0)->getSliceType();
		m_pcEncTop->ETRI_setFrameInfoforDLL(iPos, iGOPid, eFrameType, pocCurr, eSliceIndex);
#endif
	}


	//-------------------------------------------------------------------------------------------------

#if ETRI_THREADPOOL_OPT
	const int nJob = 32;
	int jobIndex = 0;

	EncGopJob* job[nJob];
	bool bStart[nJob] = { 0, };
	bool bEnd[nJob] = { 0, };

	pthread_mutex_t quramMutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t  quramCond  = PTHREAD_COND_INITIALIZER;
	int bRefPicAvailable = 0;

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 50; // 10 us

#else
#endif

	while (nCount < Size)
	{
#if ETRI_THREADPOOL_OPT
		bRefPicAvailable = 0;
#endif
		for (Int iPos = 0; iPos < Size; iPos++)
		{
			iTimeOffset = pocCurr = 0;
			if(em_bThreadUsed[iPos])
				continue;

			if (Size != 1)
			{
				iNumPicRcvd = m_iGopSize;
				iPOCLast = ((iPos / m_iGopSize) + 1) * m_iGopSize + iOffset;
				iGOPid = iPos % m_iGopSize;
			}

			iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
			if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField))
			{
				accessUnitsInGOP[iPos].poc = pocCurr;
				accessUnitsInGOP[iPos].pos = -1;
				em_bThreadUsed[iPos] = true;
				nCount++;
				continue;
			}

			if (!ETRI_RefPicCheck(pocCurr, &rcListPic, nRefCnt))
			{
				//Sleep(1); //yhee, 2015116
				continue;
			}

			// Read One Picture for Frame Compression from INput Picture Buffer
			ETRI_xGetBuffer(rcListPic, rcListPicYuvRecOut, pcPic, pcPicYuvRecOut, pocCurr, iPos, isTff);

			if (Size == 1)
			{
#if (_ETRI_WINDOWS_APPLICATION)
				iBeforeTime = clock();
#else
				clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
				// Set Parameter for Frame Compression 
				em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

				accessUnitsInGOP[iPos].poc = pocCurr;
				accessUnitsInGOP[iPos].pos = iPos;

#if ETRI_THREADPOOL_OPT
				em_pcFrameEncoder[iPos].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iPos].outputAccessUnits, m_bFirst, NULL, NULL, NULL, m_qrThreadpool, false);
#else
				em_pcFrameEncoder[iPos].ETRI_CompressFrame(pocCurr, pcPic, pcPicYuvRecOut, rcListPic, accessUnitsInGOP[iPos].outputAccessUnits, m_bFirst, false);
#endif

				// Final Parameter Setting After Frame Compression 
#if PSNR_DISPLAY	
				ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder[iPos].ETRI_getAccessUnitFrame(), em_pcFrameEncoder[iPos].ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
				printf("\n");	fflush(stdout);
#if _YHEEDEBUG
				dEncTime = (Double)(clock() - iBeforeTime) / CLOCKS_PER_SEC;
				printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
				m_bFirst = false;
				//m_iNumPicCoded++; //m_totalCoded++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

				iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

				em_bThreadUsed[iPos] = true;
			}

#if ETRI_THREADPOOL_OPT
			else
			{
				bool b = false;
				int index = 0;

				while (1)
				{
					for (int i = 0; i < MAX_THREAD_GOP; i++)
					{
						if (m_bThreadRunning[i] == false)
						{
							b = true;
							index = i;
							break;
						}
					}
					if (b)
					{
						break;
					}
					else
					{
#if !(_ETRI_WINDOWS_APPLICATION)
						usleep(50);
#else
						//select(0, NULL, NULL, NULL, &tv); // sleep 10 us
						Sleep(1);
#endif
					}
				}

				//gplusplus_151124 Thread Function -> Local Function
				em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

				em_bThreadUsed[iPos] = true;
				accessUnitsInGOP[iPos].poc = pocCurr;
				accessUnitsInGOP[iPos].pos = iGOPid;

				em_cThreadGOP[index].em_rpcListPic = &rcListPic;
				em_cThreadGOP[index].em_rpcListPicYuvRecOut = &rcListPicYuvRecOut;
				em_cThreadGOP[index].em_paccessUnitsInGOP = &accessUnitsInGOP[iPos].outputAccessUnits;
				em_cThreadGOP[index].em_pcPic = pcPic;
				em_cThreadGOP[index].em_pcPicYuvRecOut = pcPicYuvRecOut;
				em_cThreadGOP[index].em_iPos = iPos;
				em_cThreadGOP[index].em_pocCurr = pocCurr;
				em_cThreadGOP[index].em_bFirst = m_bFirst;
				em_cThreadGOP[index].em_isField = isField;
				em_cThreadGOP[index].em_isTff = isTff;

				job[jobIndex] = EncGopJob::createJob(this, index, &quramMutex, &quramCond, &bRefPicAvailable, m_qrThreadpool, m_bThreadRunning);
				m_qrThreadpool->run(job[jobIndex]);

				m_bThreadRunning[index] = true;
				jobIndex++;
			}
#else
			else
			{
				// 여유 있는 스레드를 찾는다.
				nThreadID = em_hThreadPoolGOP.GetFreeThreadID();

				//gplusplus_151124 Thread Function -> Local Function
				em_pcFrameEncoder[iPos].ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, isField, isTff);

				em_bThreadUsed[iPos] = true;
				accessUnitsInGOP[iPos].poc = pocCurr;
				accessUnitsInGOP[iPos].pos = iGOPid;

				em_cThreadGOP[nThreadID].em_rpcListPic			= &rcListPic;
				em_cThreadGOP[nThreadID].em_rpcListPicYuvRecOut= &rcListPicYuvRecOut;
				em_cThreadGOP[nThreadID].em_paccessUnitsInGOP	= &accessUnitsInGOP[iPos].outputAccessUnits;
				em_cThreadGOP[nThreadID].em_pcPic				= pcPic;
				em_cThreadGOP[nThreadID].em_pcPicYuvRecOut		= pcPicYuvRecOut;
				em_cThreadGOP[nThreadID].em_iPos				= iPos;
				em_cThreadGOP[nThreadID].em_pocCurr				= pocCurr;
				em_cThreadGOP[nThreadID].em_bFirst				= m_bFirst;
				em_cThreadGOP[nThreadID].em_isField				= isField;
				em_cThreadGOP[nThreadID].em_isTff				= isTff;

				em_hThreadPoolGOP.AddThread(this, nThreadID);
			}
#endif
			nCount++;
			nThreadCnt++;
		}	// for()
#if ETRI_FRAME_THEAD_OPT
		if (Size != 1)
		{
			pthread_mutex_lock(&quramMutex);
			if (bRefPicAvailable == 0)
			{
				pthread_cond_wait(&quramCond, &quramMutex);
			}
			pthread_mutex_unlock(&quramMutex);
		}
#endif
	}
#if ETRI_THREADPOOL_OPT
	int cnt = 0;
	while (1)
	{
		cnt = 0;
		for (int i = 0; i < MAX_THREAD_GOP; i++)
		{
			if (m_bThreadRunning[i] == false)
			{
				cnt++;
			}
		}
		if (cnt == MAX_THREAD_GOP)
		{
			break;
		}
		else
		{
#if !(_ETRI_WINDOWS_APPLICATION)
			usleep(50);
#else
			//select(0, NULL, NULL, NULL, &tv); // sleep 10 us
			Sleep(1);
#endif
		}
	}

#else
	// 모든 스레드가 끝날때까지 기다린다.
	em_hThreadPoolGOP.GetAllFreeThreadWaiting();
#endif

#if KAIST_RC
  if (m_pcEncTop->getUseRateCtrl())
  {
#if ETRI_MULTITHREAD_2
#if ETRI_DLL_INTERFACE
    /*Int EndofIDRp = 0;

    Int intra_size = m_pcEncTop->getIntraPeriod();
    Int iIDRIndex = pocCurr / intra_size;
    Int iIDRModulus = pocCurr % intra_size;

    std::list<TEncRateCtrl>::iterator		tempIter;

    int idx = 0;
    for (tempIter = m_pcEncTop->getRateCtrlLst()->begin(); tempIter != m_pcEncTop->getRateCtrlLst()->end(); tempIter++)
    {
      //em_pcRateCtrl = &(*tempIter);
      idx++;
    }
    printf("idx : %d\n", idx);

    printf("Actual Bits : ");
    for (Int i = 0; i < intra_size; i++)
    {
      EndofIDRp += (((m_pcEncTop->getRateCtrlLst()->begin())->m_sliceActualBits[i]) ? 1 : 0);

      printf("%d ", m_pcEncTop->getRateCtrlLst()->begin()->m_sliceActualBits[i]);
    }
    printf("NumCodedPics:%d\n", EndofIDRp);
    if (EndofIDRp == intra_size)
    {
      printf("IDR done!!!\n");
      m_pcEncTop->getRateCtrlLst()->begin()->destroy();
      m_pcEncTop->getRateCtrlLst()->pop_front();
      printf("RC instance removed!!!\n");
    }*/
    Int EndofIDRp = 0;

    Int intra_size = m_pcEncTop->getIntraPeriod();
    Int iIDRIndex = pocCurr / intra_size;
    Int iIDRModulus = pocCurr % intra_size;

    std::list<TEncRateCtrl>::iterator		tempIter;

    int idx = 0;
    TEncRateCtrl * pRateCtrl;

    //printf("Current POC : %d\n", pocCurr);

    tempIter = m_pcEncTop->getRateCtrlLst()->begin();
    pRateCtrl = &(*tempIter);

    for (Int i = 0; i < intra_size; i++)
    {
      EndofIDRp += ((pRateCtrl->m_sliceActualBits[i]) ? 1 : 0);

      //printf("%d ", pRateCtrl->m_sliceActualBits[i]);
    }
    //printf("NumCodedPics:%d\n", EndofIDRp);
    if (EndofIDRp == intra_size)
    {
      //printf("IDR done!!!\n");
      pRateCtrl->destroy();
      m_pcEncTop->getRateCtrlLst()->erase(tempIter);
      //printf("RC instance removed!!!\n");
    }
#else
    Int EndofIDRp = 0;

    Int intra_size = m_pcEncTop->getIntraPeriod();
    Int iIDRIndex = pocCurr / intra_size;
    Int iIDRModulus = pocCurr % intra_size;

    std::list<TEncRateCtrl>::iterator		tempIter;

    int idx = 0;
    TEncRateCtrl * pRateCtrl;

    //printf("Current POC : %d\n", pocCurr);

    tempIter = m_pcEncTop->getRateCtrlLst()->begin();
    pRateCtrl = &(*tempIter);

    for (Int i = 0; i < intra_size; i++)
    {
      EndofIDRp += ((pRateCtrl->m_sliceActualBits[i]) ? 1 : 0);

      //printf("%d ", pRateCtrl->m_sliceActualBits[i]);
    }
    //printf("NumCodedPics:%d\n", EndofIDRp);
    if (EndofIDRp == intra_size)
    {
      //printf("IDR done!!!\n");
      pRateCtrl->destroy();
      m_pcEncTop->getRateCtrlLst()->erase(tempIter);
      //printf("RC instance removed!!!\n");
    }

    /*
    for (idx = 0 ; idx < m_pcEncTop->getRateCtrlLst()->size(); idx++)
    {
      pRateCtrl = &(*tempIter);

      printf("idx : %d\n", idx);

      printf("Actual Bits : ");
      for (Int i = 0; i < intra_size; i++)
      {
        EndofIDRp += ((pRateCtrl->m_sliceActualBits[i]) ? 1 : 0);

        printf("%d ", pRateCtrl->m_sliceActualBits[i]);
      }
      printf("NumCodedPics:%d\n", EndofIDRp);
      if (EndofIDRp == intra_size)
      {
        printf("IDR done!!!\n");
        pRateCtrl->destroy();
        m_pcEncTop->getRateCtrlLst()->erase(tempIter);
        printf("RC instance removed!!!\n");
      }
      tempIter++;
    }
    */
#endif
#endif
  }
#endif

	m_iNumPicCoded += nThreadCnt; m_totalCoded += nThreadCnt;
	//delete[] bThreadUsed;

#endif
#endif
	for (int i = 0; i < nRefCnt; i++)
	{
		delete em_refPic[i].pRefPOC;
	}	

#else //!ETRI_MULTITHREAD_2
	for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
	{
		iTimeOffset = pocCurr=0;
		iGOPid = ETRI_iGOPIRAPtoReorder_v1(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP);   		///iGOPid Reordering when Field Coding
		if (ETRI_FIrstPOCForFieldFrame(pocCurr, iTimeOffset, iPOCLast, iNumPicRcvd, iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, isField)){continue;}

		// Read One Picture for Frame Compression from INput Picture Buffer
		xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isTff);

		// Set Parameter for Frame Compression 
		em_pcFrameEncoder->ETRI_setFrameParameter(iGOPid, iPOCLast, iNumPicRcvd, IRAPGOPid, m_iLastIDR, accumBitsDU, accumNalsDU, pcBitstreamRedirect, isField, isTff);

		em_pcFrameEncoder->ETRI_CompressFrame(iTimeOffset, pocCurr, pcPic, pcPicYuvRecOut, rcListPic, rcListPicYuvRecOut, accessUnitsInGOP);

		// Final Parameter Setting After Frame Compression 
		ETRI_xCalculateAddPSNR(pcPic, rcListPic, *em_pcFrameEncoder->ETRI_getAccessUnitFrame(), em_pcFrameEncoder->ETRI_getdEncTime(), isField, isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
		printf("\n");	fflush(stdout);
		m_bFirst = false;
		m_iNumPicCoded++; m_totalCoded ++;		///update Encoding Parameter such as the number of Coded Pic and total coded Pic @ 2015 5 14 by Seok

		iGOPid = ETRI_iGOPIRAPtoReorder_v3(iGOPid, IRAPGOPid, IRAPtoReorder, swapIRAPForward, EFFICIENT_FIELD_IRAP); ///iGOP Reordering at the end of Frame Encoding

#if ETRI_DLL_INTERFACE //write DLL TS data, 2015 06 15 by yhee		
		m_pcEncTop->ETRI_setFrameInfoforDLL(eEncOrder, iGOPid, em_pcFrameEncoder->ETRI_getFrameTypeInGOP(), pocCurr, eSliceIndex);	
		eEncOrder++;
#endif
	}/// for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
#endif //ETRI_MULTITHREAD_2
	delete pcBitstreamRedirect;

	if( accumBitsDU != NULL) delete accumBitsDU;
	if( accumNalsDU != NULL) delete accumNalsDU;

//# if !ETRI_MULTITHREAD_2 //removed by yhee
//	assert ( (m_iNumPicCoded == iNumPicRcvd) || (isField && iPOCLast == 1) );
//#endif
}

#endif /// 2015 5 11 by Seok : #if ETRI_MODIFICATION_V00
// ====================================================================================================================


Void TEncGOP::printOutSummary(UInt uiNumAllPicCoded, bool isField)
{
#if PSNR_DISPLAY	
	assert(uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic());
#endif
  
  
  //--CFG_KDY
  if(isField)
  {
    m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate() * 2);
    m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate() * 2);
    m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate() * 2);
    m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate() * 2);
  }
  else
  {
    m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate() );
    m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate() );
    m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate() );
    m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate() );
  }
  
  //-- all
  printf( "\n\nSUMMARY --------------------------------------------------------\n" );
  m_gcAnalyzeAll.printOut('a');
  
  printf( "\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut('i');
  
  printf( "\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut('p');
  
  printf( "\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut('b');
  
#if _SUMMARY_OUT_
  m_gcAnalyzeAll.printSummaryOut();
#endif
#if _SUMMARY_PIC_
  m_gcAnalyzeI.printSummary('I');
  m_gcAnalyzeP.printSummary('P');
  m_gcAnalyzeB.printSummary('B');
#endif

  if(isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate());
    printf( "\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
    m_gcAnalyzeAll_in.printOutInterlaced('a',  m_gcAnalyzeAll.getBits());
    
#if _SUMMARY_OUT_
    m_gcAnalyzeAll_in.printSummaryOutInterlaced();
#endif
  }

  printf("\nRVM: %.3lf\n" , xCalculateRVM());
}

#if !ETRI_MULTITHREAD_2
Void TEncGOP::preLoopFilterPicAll( TComPic* pcPic, UInt64& ruiDist, UInt64& ruiBits )
{
  TComSlice* pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
  Bool bCalcDist = false;
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
  for (int i=0; i<MAX_NUM_THREAD; i++)
  {
	  m_pcLoopFilter[i]->setCfg(m_pcCfg->getLFCrossTileBoundaryFlag());
  }
  m_pcLoopFilter[0]->loopFilterPic(pcPic, m_pcLoopFilter);
#else 
  m_pcLoopFilter->setCfg(m_pcCfg->getLFCrossTileBoundaryFlag());
  m_pcLoopFilter->loopFilterPic( pcPic );
#endif 
  m_pcEntropyCoder->setEntropyCoder ( m_pcEncTop->getRDGoOnSbacCoder(), pcSlice );
  m_pcEntropyCoder->resetEntropy    ();
  m_pcEntropyCoder->setBitstream    ( m_pcBitCounter );
  m_pcEntropyCoder->resetEntropy    ();
  ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  
  if (!bCalcDist)
    ruiDist = xFindDistortionFrame(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
}
#endif
// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


Void TEncGOP::xInitGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, bool isField )
{
  assert( iNumPicRcvd > 0 );
  //  Exception for the first frames
  if ( ( isField && (iPOCLast == 0 || iPOCLast == 1) ) || (!isField  && (iPOCLast == 0))  )
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  assert (m_iGopSize > 0);
  
  return;
}

Void TEncGOP::xGetBuffer( TComList<TComPic*>&      rcListPic,
                         TComList<TComPicYuv*>&    rcListPicYuvRecOut,
                         Int                       iNumPicRcvd,
                         Int                       iTimeOffset,
                         TComPic*&                 rpcPic,
                         TComPicYuv*&              rpcPicYuvRecOut,
                         Int                       pocCurr,
                         bool                      isField)
{
  Int i;
#if ETRI_MULTITHREAD_2  
  if (m_pcEncTop->ETRI_getReconFileOk())
#endif
  {

	  //  Rec. output
	  TComList<TComPicYuv*>::iterator iterPicYuvRec = rcListPicYuvRecOut.end();

	  if (isField)
	  {
		  for (i = 0; i < ((pocCurr == 0) || (pocCurr == 1) ? (iNumPicRcvd - iTimeOffset + 1) : (iNumPicRcvd - iTimeOffset + 2)); i++)
		  {
			  iterPicYuvRec--;
		  }
	  }
	  else
	  {
		  for (i = 0; i < (iNumPicRcvd - iTimeOffset + 1); i++)
		  {
			  iterPicYuvRec--;
		  }

	  }

	  if (isField)
	  {
		  if (pocCurr == 1)
		  {
			  iterPicYuvRec++;
		  }
	  }
	  rpcPicYuvRecOut = *(iterPicYuvRec);
  }
  
  //  Current pic.
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic);
    rpcPic->setCurrSliceIdx(0);
    if (rpcPic->getPOC() == pocCurr)
    {
      break;
    }
    iterPic++;
  }
  
  assert( rpcPic != NULL );
  assert( rpcPic->getPOC() == pocCurr );
  
  return;
}

UInt64 TEncGOP::xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1)
{
  Int     x, y;
  Pel*  pSrc0   = pcPic0 ->getLumaAddr();
  Pel*  pSrc1   = pcPic1 ->getLumaAddr();
  UInt  uiShift = 2 * DISTORTION_PRECISION_ADJUSTMENT(g_bitDepthY-8);
  Int   iTemp;
  
  Int   iStride = pcPic0->getStride();
  Int   iWidth  = pcPic0->getWidth();
  Int   iHeight = pcPic0->getHeight();
  
  UInt64  uiTotalDiff = 0;
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pSrc0[x] - pSrc1[x]; uiTotalDiff += (iTemp*iTemp) >> uiShift;
    }
    pSrc0 += iStride;
    pSrc1 += iStride;
  }
  
  uiShift = 2 * DISTORTION_PRECISION_ADJUSTMENT(g_bitDepthC-8);
  iHeight >>= 1;
  iWidth  >>= 1;
  iStride >>= 1;
  
  pSrc0  = pcPic0->getCbAddr();
  pSrc1  = pcPic1->getCbAddr();
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pSrc0[x] - pSrc1[x]; uiTotalDiff += (iTemp*iTemp) >> uiShift;
    }
    pSrc0 += iStride;
    pSrc1 += iStride;
  }
  
  pSrc0  = pcPic0->getCrAddr();
  pSrc1  = pcPic1->getCrAddr();
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pSrc0[x] - pSrc1[x]; uiTotalDiff += (iTemp*iTemp) >> uiShift;
    }
    pSrc0 += iStride;
    pSrc1 += iStride;
  }
  
  return uiTotalDiff;
}

#if VERBOSE_RATE
static const Char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
    case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
    case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
    case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
    case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
    case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
    case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
    case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
    case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
    case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
    case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
    case NAL_UNIT_VPS:                    return "VPS";
    case NAL_UNIT_SPS:                    return "SPS";
    case NAL_UNIT_PPS:                    return "PPS";
    case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
    case NAL_UNIT_EOS:                    return "EOS";
    case NAL_UNIT_EOB:                    return "EOB";
    case NAL_UNIT_FILLER_DATA:            return "FILLER";
    case NAL_UNIT_PREFIX_SEI:             return "SEI";
    case NAL_UNIT_SUFFIX_SEI:             return "SEI";
    default:                              return "UNK";
  }
}
#endif

Void TEncGOP::xCalculateAddPSNR( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit& accessUnit, Double dEncTime )
{
  Int     x, y;
  UInt64 uiSSDY  = 0;
  UInt64 uiSSDU  = 0;
  UInt64 uiSSDV  = 0;
  
  Double  dYPSNR  = 0.0;
  Double  dUPSNR  = 0.0;
  Double  dVPSNR  = 0.0;
  
  //===== calculate PSNR =====
  Pel*  pOrg    = pcPic ->getPicYuvOrg()->getLumaAddr();
  Pel*  pRec    = pcPicD->getLumaAddr();
  Int   iStride = pcPicD->getStride();
  
  Int   iWidth;
  Int   iHeight;
  
  iWidth  = pcPicD->getWidth () - m_pcEncTop->getPad(0);
  iHeight = pcPicD->getHeight() - m_pcEncTop->getPad(1);
  
  Int   iSize   = iWidth*iHeight;
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrg[x] - pRec[x] );
      uiSSDY   += iDiff * iDiff;
    }
    pOrg += iStride;
    pRec += iStride;
  }
  
  iHeight >>= 1;
  iWidth  >>= 1;
  iStride >>= 1;
  pOrg  = pcPic ->getPicYuvOrg()->getCbAddr();
  pRec  = pcPicD->getCbAddr();
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrg[x] - pRec[x] );
      uiSSDU   += iDiff * iDiff;
    }
    pOrg += iStride;
    pRec += iStride;
  }
  
  pOrg  = pcPic ->getPicYuvOrg()->getCrAddr();
  pRec  = pcPicD->getCrAddr();
  
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrg[x] - pRec[x] );
      uiSSDV   += iDiff * iDiff;
    }
    pOrg += iStride;
    pRec += iStride;
  }
  
  Int maxvalY = 255 << (g_bitDepthY-8);
  Int maxvalC = 255 << (g_bitDepthC-8);
  Double fRefValueY = (Double) maxvalY * maxvalY * iSize;
  Double fRefValueC = (Double) maxvalC * maxvalC * iSize / 4.0;
  dYPSNR            = ( uiSSDY ? 10.0 * log10( fRefValueY / (Double)uiSSDY ) : 99.99 );
  dUPSNR            = ( uiSSDU ? 10.0 * log10( fRefValueC / (Double)uiSSDU ) : 99.99 );
  dVPSNR            = ( uiSSDV ? 10.0 * log10( fRefValueC / (Double)uiSSDV ) : 99.99 );

  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
#if VERBOSE_RATE
    printf("*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
#endif
    if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
    {
      numRBSPBytes += numRBSPBytes_nal;
    }
  }

  UInt uibits = numRBSPBytes * 8;
  TComSlice*  pcSlice = pcPic->getSlice(0);
#if PSNR_DISPLAY //ETRI_MULTITHREAD_2  
#if (_ETRI_WINDOWS_APPLICATION)
  ::EnterCriticalSection(&GOP_cs); // gplusplus_151116
#else
  pthread_mutex_lock(&GOP_cs);
#endif
#endif
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
  m_gcAnalyzeAll.addResult (dYPSNR, dUPSNR, dVPSNR, (Double)uibits);
  //TComSlice*  pcSlice = pcPic->getSlice(0);
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult (dYPSNR, dUPSNR, dVPSNR, (Double)uibits);
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult (dYPSNR, dUPSNR, dVPSNR, (Double)uibits);
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult (dYPSNR, dUPSNR, dVPSNR, (Double)uibits);
  }

  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pcSlice->isReferenced()) c += 32;

#if ADAPTIVE_QP_SELECTION
  printf("POC %4d TId: %1d ( %c-SLICE, nQP %d QP %d ) %10d bits",
         pcSlice->getPOC(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQpBase(),
         pcSlice->getSliceQp(),
         uibits );
#else
  printf("POC %4d TId: %1d ( %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC()-pcSlice->getLastIDR(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp(),
         uibits );
#endif

  printf(" [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dYPSNR, dUPSNR, dVPSNR );
  printf(" [ET %5.3f ]", dEncTime );
  
  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf(" [L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex)-pcSlice->getLastIDR());
    }
    printf("]");
  }
#if PSNR_DISPLAY //ETRI_MULTITHREAD_2
#if (_ETRI_WINDOWS_APPLICATION)
  ::LeaveCriticalSection(&GOP_cs);
#else
  pthread_mutex_unlock(&GOP_cs);
#endif
#endif
}

Void reinterlace(Pel* top, Pel* bottom, Pel* dst, UInt stride, UInt width, UInt height, bool isTff)
{
  
  for (Int y = 0; y < height; y++)
  {
    for (Int x = 0; x < width; x++)
    {
      dst[x] = isTff ? top[x] : bottom[x];
      dst[stride+x] = isTff ? bottom[x] : top[x];
    }
    top += stride;
    bottom += stride;
    dst += stride*2;
  }
}

Void TEncGOP::xCalculateInterlacedAddPSNR( TComPic* pcPicOrgTop, TComPic* pcPicOrgBottom, TComPicYuv* pcPicRecTop, TComPicYuv* pcPicRecBottom, const AccessUnit& accessUnit, Double dEncTime )
{
  Int     x, y;
  
  UInt64 uiSSDY_in  = 0;
  UInt64 uiSSDU_in  = 0;
  UInt64 uiSSDV_in  = 0;
  
  Double  dYPSNR_in  = 0.0;
  Double  dUPSNR_in  = 0.0;
  Double  dVPSNR_in  = 0.0;
  
  /*------ INTERLACED PSNR -----------*/
  
  /* Luma */
  
  Pel*  pOrgTop = pcPicOrgTop->getPicYuvOrg()->getLumaAddr();
  Pel*  pOrgBottom = pcPicOrgBottom->getPicYuvOrg()->getLumaAddr();
  Pel*  pRecTop = pcPicRecTop->getLumaAddr();
  Pel*  pRecBottom = pcPicRecBottom->getLumaAddr();
  
  Int   iWidth;
  Int   iHeight;
  Int iStride;
  
  iWidth  = pcPicOrgTop->getPicYuvOrg()->getWidth () - m_pcEncTop->getPad(0);
  iHeight = pcPicOrgTop->getPicYuvOrg()->getHeight() - m_pcEncTop->getPad(1);
  iStride = pcPicOrgTop->getPicYuvOrg()->getStride();
  Int   iSize   = iWidth*iHeight;
  bool isTff = pcPicOrgTop->isTopField();
  
  TComPicYuv* pcOrgInterlaced = new TComPicYuv;
  pcOrgInterlaced->create( iWidth, iHeight << 1, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
  
  TComPicYuv* pcRecInterlaced = new TComPicYuv;
  pcRecInterlaced->create( iWidth, iHeight << 1, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
  
  Pel* pOrgInterlaced = pcOrgInterlaced->getLumaAddr();
  Pel* pRecInterlaced = pcRecInterlaced->getLumaAddr();
  
  //=== Interlace fields ====
  reinterlace(pOrgTop, pOrgBottom, pOrgInterlaced, iStride, iWidth, iHeight, isTff);
  reinterlace(pRecTop, pRecBottom, pRecInterlaced, iStride, iWidth, iHeight, isTff);
  
  //===== calculate PSNR =====
  for( y = 0; y < iHeight << 1; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrgInterlaced[x] - pRecInterlaced[x] );
      uiSSDY_in   += iDiff * iDiff;
    }
    pOrgInterlaced += iStride;
    pRecInterlaced += iStride;
  }
  
  /*Chroma*/
  
  iHeight >>= 1;
  iWidth  >>= 1;
  iStride >>= 1;
  
  pOrgTop = pcPicOrgTop->getPicYuvOrg()->getCbAddr();
  pOrgBottom = pcPicOrgBottom->getPicYuvOrg()->getCbAddr();
  pRecTop = pcPicRecTop->getCbAddr();
  pRecBottom = pcPicRecBottom->getCbAddr();
  pOrgInterlaced = pcOrgInterlaced->getCbAddr();
  pRecInterlaced = pcRecInterlaced->getCbAddr();
  
  //=== Interlace fields ====
  reinterlace(pOrgTop, pOrgBottom, pOrgInterlaced, iStride, iWidth, iHeight, isTff);
  reinterlace(pRecTop, pRecBottom, pRecInterlaced, iStride, iWidth, iHeight, isTff);
  
  //===== calculate PSNR =====
  for( y = 0; y < iHeight << 1; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrgInterlaced[x] - pRecInterlaced[x] );
      uiSSDU_in   += iDiff * iDiff;
    }
    pOrgInterlaced += iStride;
    pRecInterlaced += iStride;
  }
  
  pOrgTop = pcPicOrgTop->getPicYuvOrg()->getCrAddr();
  pOrgBottom = pcPicOrgBottom->getPicYuvOrg()->getCrAddr();
  pRecTop = pcPicRecTop->getCrAddr();
  pRecBottom = pcPicRecBottom->getCrAddr();
  pOrgInterlaced = pcOrgInterlaced->getCrAddr();
  pRecInterlaced = pcRecInterlaced->getCrAddr();
  
  //=== Interlace fields ====
  reinterlace(pOrgTop, pOrgBottom, pOrgInterlaced, iStride, iWidth, iHeight, isTff);
  reinterlace(pRecTop, pRecBottom, pRecInterlaced, iStride, iWidth, iHeight, isTff);
  
  //===== calculate PSNR =====
  for( y = 0; y < iHeight << 1; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      Int iDiff = (Int)( pOrgInterlaced[x] - pRecInterlaced[x] );
      uiSSDV_in   += iDiff * iDiff;
    }
    pOrgInterlaced += iStride;
    pRecInterlaced += iStride;
  }
  
  Int maxvalY = 255 << (g_bitDepthY-8);
  Int maxvalC = 255 << (g_bitDepthC-8);
  Double fRefValueY = (Double) maxvalY * maxvalY * iSize*2;
  Double fRefValueC = (Double) maxvalC * maxvalC * iSize*2 / 4.0;
  dYPSNR_in            = ( uiSSDY_in ? 10.0 * log10( fRefValueY / (Double)uiSSDY_in ) : 99.99 );
  dUPSNR_in            = ( uiSSDU_in ? 10.0 * log10( fRefValueC / (Double)uiSSDU_in ) : 99.99 );
  dVPSNR_in            = ( uiSSDV_in ? 10.0 * log10( fRefValueC / (Double)uiSSDV_in ) : 99.99 );
  
  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
    
    if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
      numRBSPBytes += numRBSPBytes_nal;
  }
  
  UInt uibits = numRBSPBytes * 8 ;
  
  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dYPSNR_in, dUPSNR_in, dVPSNR_in, (Double)uibits);
  
  printf("\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgBottom->getPOC()/2 , dYPSNR_in, dUPSNR_in, dVPSNR_in );
  
  pcOrgInterlaced->destroy();
  delete pcOrgInterlaced;
  pcRecInterlaced->destroy();
  delete pcRecInterlaced;
}



/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \returns the nal unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType TEncGOP::getNalUnitType(Int pocCurr, Int lastIDR, Bool isField)
{
  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }
#if EFFICIENT_FIELD_IRAP
  if(isField && pocCurr == 1)
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL_R;
  }
#endif

#if ALLOW_RECOVERY_POINT_AS_RAP
  if(m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)
#else
  if ((pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)
#endif
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)
    {
      return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all 
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading 
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by 
      // controlling the reference pictures used for encoding that leading picture. Such a leading 
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL_R;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL_R;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL_R;
}

Double TEncGOP::xCalculateRVM()
{
  Double dRVM = 0;
  
  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<Double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );
    
    Int i;
    Double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( Int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
        vRL[i] += m_vRVM_RP[j];
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }
    
    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );
    
    Double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      Double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );
    
    Double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );
    
    dRVM = dSigamB / dRavg * f;
  }
  
  return( dRVM );
}
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 TEncFrame move 
/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
Void TEncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream*& codedSliceData)
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }

  m_pcEntropyCoder->setBitstream(&rNalu.m_Bitstream);

  codedSliceData->clear();
}
#endif
// Function will arrange the long-term pictures in the decreasing order of poc_lsb_lt, 
// and among the pictures with the same lsb, it arranges them in increasing delta_poc_msb_cycle_lt value
Void TEncGOP::arrangeLongtermPicturesInRPS(TComSlice *pcSlice, TComList<TComPic*>& rcListPic)
{
  TComReferencePictureSet *rps = pcSlice->getRPS();
  if(!rps->getNumberOfLongtermPictures())
  {
    return;
  }

  // Arrange long-term reference pictures in the correct order of LSB and MSB,
  // and assign values for pocLSBLT and MSB present flag
  Int longtermPicsPoc[MAX_NUM_REF_PICS], longtermPicsLSB[MAX_NUM_REF_PICS], indices[MAX_NUM_REF_PICS];
  Int longtermPicsMSB[MAX_NUM_REF_PICS];
  Bool mSBPresentFlag[MAX_NUM_REF_PICS];
  ::memset(longtermPicsPoc, 0, sizeof(longtermPicsPoc));    // Store POC values of LTRP
  ::memset(longtermPicsLSB, 0, sizeof(longtermPicsLSB));    // Store POC LSB values of LTRP
  ::memset(longtermPicsMSB, 0, sizeof(longtermPicsMSB));    // Store POC LSB values of LTRP
  ::memset(indices        , 0, sizeof(indices));            // Indices to aid in tracking sorted LTRPs
  ::memset(mSBPresentFlag , 0, sizeof(mSBPresentFlag));     // Indicate if MSB needs to be present

  // Get the long-term reference pictures 
  Int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
  Int i, ctr = 0;
  Int maxPicOrderCntLSB = 1 << pcSlice->getSPS()->getBitsForPOC();
  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    longtermPicsPoc[ctr] = rps->getPOC(i);                                  // LTRP POC
    longtermPicsLSB[ctr] = getLSB(longtermPicsPoc[ctr], maxPicOrderCntLSB); // LTRP POC LSB
    indices[ctr]      = i; 
    longtermPicsMSB[ctr] = longtermPicsPoc[ctr] - longtermPicsLSB[ctr];
  }
  Int numLongPics = rps->getNumberOfLongtermPictures();
  assert(ctr == numLongPics);

  // Arrange pictures in decreasing order of MSB; 
  for(i = 0; i < numLongPics; i++)
  {
    for(Int j = 0; j < numLongPics - 1; j++)
    {
      if(longtermPicsMSB[j] < longtermPicsMSB[j+1])
      {
        std::swap(longtermPicsPoc[j], longtermPicsPoc[j+1]);
        std::swap(longtermPicsLSB[j], longtermPicsLSB[j+1]);
        std::swap(longtermPicsMSB[j], longtermPicsMSB[j+1]);
        std::swap(indices[j]        , indices[j+1]        );
      }
    }
  }

  for(i = 0; i < numLongPics; i++)
  {
    // Check if MSB present flag should be enabled.
    // Check if the buffer contains any pictures that have the same LSB.
    TComList<TComPic*>::iterator  iterPic = rcListPic.begin();  
    TComPic*                      pcPic;
    while ( iterPic != rcListPic.end() )
    {
      pcPic = *iterPic;
      if( (getLSB(pcPic->getPOC(), maxPicOrderCntLSB) == longtermPicsLSB[i])   &&     // Same LSB
                                      (pcPic->getSlice(0)->isReferenced())     &&    // Reference picture
                                        (pcPic->getPOC() != longtermPicsPoc[i])    )  // Not the LTRP itself
      {
        mSBPresentFlag[i] = true;
        break;
      }
      iterPic++;      
    }
  }

  // tempArray for usedByCurr flag
  Bool tempArray[MAX_NUM_REF_PICS]; ::memset(tempArray, 0, sizeof(tempArray));
  for(i = 0; i < numLongPics; i++)
  {
    tempArray[i] = rps->getUsed(indices[i]);
  }
  // Now write the final values;
  ctr = 0;
  Int currMSB = 0, currLSB = 0;
  // currPicPoc = currMSB + currLSB
  currLSB = getLSB(pcSlice->getPOC(), maxPicOrderCntLSB);  
  currMSB = pcSlice->getPOC() - currLSB;

  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    rps->setPOC                   (i, longtermPicsPoc[ctr]);
    rps->setDeltaPOC              (i, - pcSlice->getPOC() + longtermPicsPoc[ctr]);
    rps->setUsed                  (i, tempArray[ctr]);
    rps->setPocLSBLT              (i, longtermPicsLSB[ctr]);
    rps->setDeltaPocMSBCycleLT    (i, (currMSB - (longtermPicsPoc[ctr] - longtermPicsLSB[ctr])) / maxPicOrderCntLSB);
    rps->setDeltaPocMSBPresentFlag(i, mSBPresentFlag[ctr]);     

    assert(rps->getDeltaPocMSBCycleLT(i) >= 0);   // Non-negative value
  }
  for(i = rps->getNumberOfPictures() - 1, ctr = 1; i >= offset; i--, ctr++)
  {
    for(Int j = rps->getNumberOfPictures() - 1 - ctr; j >= offset; j--)
    {
      // Here at the encoder we know that we have set the full POC value for the LTRPs, hence we 
      // don't have to check the MSB present flag values for this constraint.
      assert( rps->getPOC(i) != rps->getPOC(j) ); // If assert fails, LTRP entry repeated in RPS!!!
    }
  }
}

/** Function for finding the position to insert the first of APS and non-nested BP, PT, DU info SEI messages.
 * \param accessUnit Access Unit of the current picture
 * This function finds the position to insert the first of APS and non-nested BP, PT, DU info SEI messages.
 */
Int TEncGOP::xGetFirstSeiLocation(AccessUnit &accessUnit)
{
  // Find the location of the first SEI message
  AccessUnit::iterator it;
  Int seiStartPos = 0;
  for(it = accessUnit.begin(); it != accessUnit.end(); it++, seiStartPos++)
  {
     if ((*it)->isSei() || (*it)->isVcl())
     {
       break;
     }               
  }
//  assert(it != accessUnit.end());  // Triggers with some legit configurations
  return seiStartPos;
}

Void TEncGOP::dblMetric( TComPic* pcPic, UInt uiNumSlices )
{
  TComPicYuv* pcPicYuvRec = pcPic->getPicYuvRec();
  Pel* Rec    = pcPicYuvRec->getLumaAddr( 0 );
  Pel* tempRec = Rec;
  Int  stride = pcPicYuvRec->getStride();
  UInt log2maxTB = pcPic->getSlice(0)->getSPS()->getQuadtreeTULog2MaxSize();
  UInt maxTBsize = (1<<log2maxTB);
  const UInt minBlockArtSize = 8;
  const UInt picWidth = pcPicYuvRec->getWidth();
  const UInt picHeight = pcPicYuvRec->getHeight();
  const UInt noCol = (picWidth>>log2maxTB);
  const UInt noRows = (picHeight>>log2maxTB);
  assert(noCol > 1);
  assert(noRows > 1);
  UInt64 *colSAD = (UInt64*)malloc(noCol*sizeof(UInt64));
  UInt64 *rowSAD = (UInt64*)malloc(noRows*sizeof(UInt64));
  UInt colIdx = 0;
  UInt rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;
  
  Int qp = pcPic->getSlice(0)->getSliceQp();
  Int bitdepthScale = 1 << (g_bitDepthY-8);
  Int beta = TComLoopFilter::getBeta( qp ) * bitdepthScale;
  const Int thr2 = (beta>>2);
  const Int thr1 = 2*bitdepthScale;
  UInt a = 0;
  
  memset(colSAD, 0, noCol*sizeof(UInt64));
  memset(rowSAD, 0, noRows*sizeof(UInt64));
  
  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(Int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for(Int r = 0; r < picHeight; r++)
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if ( thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        Rec += stride;
      }
      colIdx++;
      Rec = tempRec;
    }
    
    // Analyze horizontal artifact edges
    for(Int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for(Int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }
  
  UInt64 colSADsum = 0;
  UInt64 rowSADsum = 0;
  for(Int c = 0; c < noCol-1; c++)
  {
    colSADsum += colSAD[c];
  }
  for(Int r = 0; r < noRows-1; r++)
  {
    rowSADsum += rowSAD[r];
  }
  
  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;
  
  UInt64 avgSAD = ((colSADsum + rowSADsum)>>1);
  avgSAD >>= (g_bitDepthY-8);
  
  if ( avgSAD > 2048 )
  {
    avgSAD >>= 9;
    Int offset = Clip3(2,6,(Int)avgSAD);
    for (Int i=0; i<uiNumSlices; i++)
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(true);
      pcPic->getSlice(i)->setDeblockingFilterDisable(false);
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( offset );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2( offset );
    }
  }
  else
  {
    for (Int i=0; i<uiNumSlices; i++)
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(false);
      pcPic->getSlice(i)->setDeblockingFilterDisable(        pcPic->getSlice(i)->getPPS()->getPicDisableDeblockingFilterFlag() );
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( pcPic->getSlice(i)->getPPS()->getDeblockingFilterBetaOffsetDiv2() );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2(   pcPic->getSlice(i)->getPPS()->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
  
  free(colSAD);
  free(rowSAD);
}

#if !ETRI_MODIFICATION_V00
// ====================================================================================================================
// ETRI Compress GOP Functions @ 2015 5 11 by Seok
// ====================================================================================================================
/**
	@brief:	Original CompressGOP in HM.
	@param: Int iPOCLast,
	@param: Int iNumPicRcvd, 
	@param: TComList<TComPic*>& rcListPic, 
	@param: TComList<TComPicYuv*>& rcListPicYuvRecOut, 
	@param: std::list<AccessUnit>& accessUnitsInGOP, 
	@param: bool isField, 
	@param: bool isTff
	@date:	2015 5 26 by Seok
*/
Void TEncGOP::compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP, bool isField, bool isTff)
{
  TComPic*        pcPic;
  TComPicYuv*     pcPicYuvRecOut;
  TComSlice*      pcSlice;
  TComOutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new TComOutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted
  UInt                  uiOneBitstreamPerSliceLength = 0;
  TEncSbac* pcSbacCoders = NULL;
  TComOutputBitstream* pcSubstreamsOut = NULL;
  
  xInitGOP( iPOCLast, iNumPicRcvd, rcListPic, rcListPicYuvRecOut, isField );
  
  m_iNumPicCoded = 0;
  SEIPictureTiming pictureTimingSEI;
  Bool writeSOP = m_pcCfg->getSOPDescriptionSEIEnabled();
  // Initialize Scalable Nesting SEI with single layer values
  SEIScalableNesting scalableNestingSEI;
  scalableNestingSEI.m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffereing SEI mesages, picure timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  scalableNestingSEI.m_nestingOpFlag                 = 0;
  scalableNestingSEI.m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
  scalableNestingSEI.m_allLayersFlag                 = 0;
  scalableNestingSEI.m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  scalableNestingSEI.m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  scalableNestingSEI.m_nestingLayerId[0]             = 0;
  scalableNestingSEI.m_callerOwnsSEIs                = true;
  Int picSptDpbOutputDuDelay = 0;
  UInt *accumBitsDU = NULL;
  UInt *accumNalsDU = NULL;
  SEIDecodingUnitInfo decodingUnitInfoSEI;
#if EFFICIENT_FIELD_IRAP
  Int IRAPGOPid = -1;
  Bool IRAPtoReorder = false;
  Bool swapIRAPForward = false;
  if(isField)
  {
    Int pocCurr;
    for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
    {
      // determine actual POC
      if(iPOCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = getNalUnitType(pocCurr, m_iLastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
#if ETRI_EM_OPERATION_OPTIMIZATION
		  if ((pocCurr & 1) == 0 && iGOPid < m_iGopSize - 1 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid + 1).m_POC - 1)
		  { // if top field and following picture in enc order is associated bottom field
			  IRAPGOPid = iGOPid;
			  IRAPtoReorder = true;
			  swapIRAPForward = true;
			  break;
		  }
		  if ((pocCurr & 1) != 0 && iGOPid > 0 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid - 1).m_POC + 1)
		  {
			  // if picture is an IRAP remember to process it first
			  IRAPGOPid = iGOPid;
			  IRAPtoReorder = true;
			  swapIRAPForward = false;
			  break;
		  }
#else
        if(pocCurr%2 == 0 && iGOPid < m_iGopSize-1 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true; 
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false; 
          break;
        }
#endif
      }
    }
  }
#endif
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
#if EFFICIENT_FIELD_IRAP
    if(IRAPtoReorder)
    {
      if(swapIRAPForward)
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid +1;
        }
        else if(iGOPid == IRAPGOPid +1)
        {
          iGOPid = IRAPGOPid;
        }
      }
      else
      {
        if(iGOPid == IRAPGOPid -1)
        {
          iGOPid = IRAPGOPid;
        }
        else if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid -1;
        }
      }
    }
#endif
    UInt uiColDir = 1;
    //-- For time output for each slice
#if (_ETRI_WINDOWS_APPLICATION)
	long iBeforeTime = clock();
#else
	timespec iBeforeTime;
	clock_gettime(CLOCK_MONOTONIC, &iBeforeTime);
#endif
    
    //select uiColDir
    Int iCloseLeft=1, iCloseRight=-1;
    for(Int i = 0; i<m_pcCfg->getGOPEntry(iGOPid).m_numRefPics; i++)
    {
      Int iRef = m_pcCfg->getGOPEntry(iGOPid).m_referencePics[i];
      if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))
      {
        iCloseRight=iRef;
      }
      else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))
      {
        iCloseLeft=iRef;
      }
    }
    if(iCloseRight>-1)
    {
      iCloseRight=iCloseRight+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
    }
    if(iCloseLeft<1)
    {
      iCloseLeft=iCloseLeft+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
      while(iCloseLeft<0)
      {
        iCloseLeft+=m_iGopSize;
      }
    }
    Int iLeftQP=0, iRightQP=0;
    for(Int i=0; i<m_iGopSize; i++)
    {
      if(m_pcCfg->getGOPEntry(i).m_POC==(iCloseLeft%m_iGopSize)+1)
      {
        iLeftQP= m_pcCfg->getGOPEntry(i).m_QPOffset;
      }
      if (m_pcCfg->getGOPEntry(i).m_POC==(iCloseRight%m_iGopSize)+1)
      {
        iRightQP=m_pcCfg->getGOPEntry(i).m_QPOffset;
      }
    }
    if(iCloseRight>-1&&iRightQP<iLeftQP)
    {
      uiColDir=0;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    Int iTimeOffset;
    Int pocCurr;
    
    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = 1;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }
    
    if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
    {
#if EFFICIENT_FIELD_IRAP
      if(IRAPtoReorder)
      {
        if(swapIRAPForward)
        {
          if(iGOPid == IRAPGOPid)
          {
            iGOPid = IRAPGOPid +1;
            IRAPtoReorder = false;
          }
          else if(iGOPid == IRAPGOPid +1)
          {
            iGOPid --;
          }
        }
        else
        {
          if(iGOPid == IRAPGOPid)
          {
            iGOPid = IRAPGOPid -1;
          }
          else if(iGOPid == IRAPGOPid -1)
          {
            iGOPid = IRAPGOPid;
            IRAPtoReorder = false;
          }
        }
      }
#endif
      continue;
    }
    
    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }
    // start a new access unit: create an entry in the list of output access units
    accessUnitsInGOP.push_back(AccessUnit());
    AccessUnit& accessUnit = accessUnitsInGOP.back();
    xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isField);
    
    //  Slice data initialization
    pcPic->clearSliceBuffer();
    assert(pcPic->getNumAllocatedSlice() == 1);
    m_pcSliceEncoder->setSliceIdx(0);
    pcPic->setCurrSliceIdx(0);
    
    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField );
    
    //Set Frame/Field coding
    pcSlice->getPic()->setField(isField);
    
    pcSlice->setLastIDR(m_iLastIDR);
    pcSlice->setSliceIdx(0);
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );
    pcSlice->setScalingList ( m_pcEncTop->getScalingList()  );
    if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_OFF)
    {
      m_pcEncTop->getTrQuant()->setFlatScalingList();
      m_pcEncTop->getTrQuant()->setUseScalingList(false);
      m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
    }
    else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_DEFAULT)
    {
      pcSlice->setDefaultScalingList ();
      m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList());
      m_pcEncTop->getTrQuant()->setUseScalingList(true);
    }
    else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_FILE_READ)
    {
      if(pcSlice->getScalingList()->xParseScalingList(m_pcCfg->getScalingListFile()))
      {
        pcSlice->setDefaultScalingList ();
      }
      pcSlice->getScalingList()->checkDcOfMatrix();
      m_pcEncTop->getSPS()->setScalingListPresentFlag(pcSlice->checkDefaultScalingList());
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList());
      m_pcEncTop->getTrQuant()->setUseScalingList(true);
    }
    else
    {
      printf("error : ScalingList == %d no support\n",m_pcEncTop->getUseScalingListId());
      assert(0);
    }
    
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }
    
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
    if(pcSlice->getTemporalLayerNonReferenceFlag())
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }
    
#if EFFICIENT_FIELD_IRAP
#if FIX1172
    if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
    {
      m_associatedIRAPType = pcSlice->getNalUnitType();
      m_associatedIRAPPOC = pocCurr;
    }
    pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
    pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
#endif
#endif
    // Do decoding refresh marking if any
    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic);
    m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
    pcSlice->getRPS()->setNumberOfLongtermPictures(0);
#if EFFICIENT_FIELD_IRAP
#else
#if FIX1172
    if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
    {
      m_associatedIRAPType = pcSlice->getNalUnitType();
      m_associatedIRAPPOC = pocCurr;
    }
    pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
    pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
#endif
#endif
    
#if ALLOW_RECOVERY_POINT_AS_RAP
    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP()) 
#if EFFICIENT_FIELD_IRAP
      || (isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
#endif
      )
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3);
    }
#else
    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false) != 0) || (pcSlice->isIRAP()))
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP());
    }
#endif
    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());
    
    if(pcSlice->getTLayer() > 0 
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
      {
        if(pcSlice->getTemporalLayerNonReferenceFlag())
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
      }
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        Bool isSTSA=true;
        for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer())
          {
            TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                Int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                    break;
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA=false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)
        {
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }
      }
    }
    arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);
    pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    
#if ADAPTIVE_QP_SELECTION
    pcSlice->setTrQuant( m_pcEncTop->getTrQuant() );
#endif
    
    //  Set reference list
    pcSlice->setRefPicList ( rcListPic );
    
    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }
    
    if (pcSlice->getSliceType() == B_SLICE)
    {
      pcSlice->setColFromL0Flag(1-uiColDir);
      Bool bLowDelay = true;
      Int  iCurrPOC  = pcSlice->getPOC();
      Int iRefIdx = 0;
      
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      
      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }
    
    uiColDir = 1-uiColDir;
    
    //-------------------------------------------------------------
    pcSlice->setRefPOCList();
    
    pcSlice->setList1IdxToList0Idx();
    
    if (m_pcEncTop->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        pcSlice->setEnableTMVPFlag(1);
      }
      pcSlice->getSPS()->setTMVPFlagsPresent(1);
    }
    else if (m_pcEncTop->getTMVPModeId() == 1)
    {
      pcSlice->getSPS()->setTMVPFlagsPresent(1);
      pcSlice->setEnableTMVPFlag(1);
    }
    else
    {
      pcSlice->getSPS()->setTMVPFlagsPresent(0);
      pcSlice->setEnableTMVPFlag(0);
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////// Compress a slice
    //  Slice compression
    if (m_pcCfg->getUseASR())
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }
    
    Bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        Int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      pcSlice->setMvdL1ZeroFlag(true);
    }
    else
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }
    pcPic->getSlice(pcSlice->getSliceIdx())->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());
    
    Double lambda            = 0.0;
    Int actualHeadBits       = 0;
    Int actualTotalBits      = 0;
    Int estimatedBits        = 0;
    Int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->getSlice(0)->getSliceType() == I_SLICE )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();
      
      Int sliceQP = m_pcCfg->getInitialQP();
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
      {
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
        Double dQPFactor     = 0.57*dLambda_scale;
        Int    SHIFT_QP      = 12;
        Int    bitdepth_luma_qp_scale = 0;
        Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic);
        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );
          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }
        
        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      
      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffsetY(), MAX_QP, sliceQP );
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );
      
      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }
    
    UInt uiNumSlices = 1;
    
    UInt uiInternalAddress = pcPic->getNumPartInCU()-4;
    UInt uiExternalAddress = pcPic->getPicSym()->getNumberOfCUsInFrame()-1;
    UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
    UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
    UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
    UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
    while(uiPosX>=uiWidth||uiPosY>=uiHeight)
    {
      uiInternalAddress--;
      uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
      uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
    }
    uiInternalAddress++;
    if(uiInternalAddress==pcPic->getNumPartInCU())
    {
      uiInternalAddress = 0;
      uiExternalAddress++;
    }
    UInt uiRealEndAddress = uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress;
    
    Int  p, j;
    UInt uiEncCUAddr;
    
    pcPic->getPicSym()->initTiles(pcSlice->getPPS());
    
    // Allocate some coders, now we know how many tiles there are.
    Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    
    //generate the Coding Order Map and Inverse Coding Order Map
    for(p=0, uiEncCUAddr=0; p<pcPic->getPicSym()->getNumberOfCUsInFrame(); p++, uiEncCUAddr = pcPic->getPicSym()->xCalculateNxtCUAddr(uiEncCUAddr))
    {
      pcPic->getPicSym()->setCUOrderMap(p, uiEncCUAddr);
      pcPic->getPicSym()->setInverseCUOrderMap(uiEncCUAddr, p);
    }
    pcPic->getPicSym()->setCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
    pcPic->getPicSym()->setInverseCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
    
    // Allocate some coders, now we know how many tiles there are.
    m_pcEncTop->createWPPCoders(iNumSubstreams);
    pcSbacCoders = m_pcEncTop->getSbacCoders();
    pcSubstreamsOut = new TComOutputBitstream[iNumSubstreams];
    
    UInt startCUAddrSliceIdx = 0; // used to index "m_uiStoredStartCUAddrForEncodingSlice" containing locations of slice boundaries
    UInt startCUAddrSlice    = 0; // used to keep track of current slice's starting CU addr.
    pcSlice->setSliceCurStartCUAddr( startCUAddrSlice ); // Setting "start CU addr" for current slice
    m_storedStartCUAddrForEncodingSlice.clear();
    
    UInt startCUAddrSliceSegmentIdx = 0; // used to index "m_uiStoredStartCUAddrForEntropyEncodingSlice" containing locations of slice boundaries
    UInt startCUAddrSliceSegment    = 0; // used to keep track of current Dependent slice's starting CU addr.
    pcSlice->setSliceSegmentCurStartCUAddr( startCUAddrSliceSegment ); // Setting "start CU addr" for current Dependent slice
    
    m_storedStartCUAddrForEncodingSliceSegment.clear();
    UInt nextCUAddr = 0;
    m_storedStartCUAddrForEncodingSlice.push_back (nextCUAddr);
    startCUAddrSliceIdx++;
    m_storedStartCUAddrForEncodingSliceSegment.push_back(nextCUAddr);
    startCUAddrSliceSegmentIdx++;
    
    while(nextCUAddr<uiRealEndAddress) // determine slice boundaries
    {
      pcSlice->setNextSlice       ( false );
      pcSlice->setNextSliceSegment( false );
      assert(pcPic->getNumAllocatedSlice() == startCUAddrSliceIdx);
      m_pcSliceEncoder->precompressSlice( pcPic );
      m_pcSliceEncoder->compressSlice   ( pcPic );
      
      Bool bNoBinBitConstraintViolated = (!pcSlice->isNextSlice() && !pcSlice->isNextSliceSegment());
      if (pcSlice->isNextSlice() || (bNoBinBitConstraintViolated && m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU))
      {
        startCUAddrSlice = pcSlice->getSliceCurEndCUAddr();
        // Reconstruction slice
        m_storedStartCUAddrForEncodingSlice.push_back(startCUAddrSlice);
        startCUAddrSliceIdx++;
        // Dependent slice
        if (startCUAddrSliceSegmentIdx>0 && m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx-1] != startCUAddrSlice)
        {
          m_storedStartCUAddrForEncodingSliceSegment.push_back(startCUAddrSlice);
          startCUAddrSliceSegmentIdx++;
        }
        
        if (startCUAddrSlice < uiRealEndAddress)
        {
          pcPic->allocateNewSlice();
          pcPic->setCurrSliceIdx                  ( startCUAddrSliceIdx-1 );
          m_pcSliceEncoder->setSliceIdx           ( startCUAddrSliceIdx-1 );
          pcSlice = pcPic->getSlice               ( startCUAddrSliceIdx-1 );
          pcSlice->copySliceInfo                  ( pcPic->getSlice(0)      );
          pcSlice->setSliceIdx                    ( startCUAddrSliceIdx-1 );
          pcSlice->setSliceCurStartCUAddr         ( startCUAddrSlice      );
          pcSlice->setSliceSegmentCurStartCUAddr  ( startCUAddrSlice      );
          pcSlice->setSliceBits(0);
          uiNumSlices ++;
        }
      }
      else if (pcSlice->isNextSliceSegment() || (bNoBinBitConstraintViolated && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU))
      {
        startCUAddrSliceSegment                                                     = pcSlice->getSliceSegmentCurEndCUAddr();
        m_storedStartCUAddrForEncodingSliceSegment.push_back(startCUAddrSliceSegment);
        startCUAddrSliceSegmentIdx++;
        pcSlice->setSliceSegmentCurStartCUAddr( startCUAddrSliceSegment );
      }
      else
      {
        startCUAddrSlice                                                            = pcSlice->getSliceCurEndCUAddr();
        startCUAddrSliceSegment                                                     = pcSlice->getSliceSegmentCurEndCUAddr();
      }
      
      nextCUAddr = (startCUAddrSlice > startCUAddrSliceSegment) ? startCUAddrSlice : startCUAddrSliceSegment;
    }
    m_storedStartCUAddrForEncodingSlice.push_back( pcSlice->getSliceCurEndCUAddr());
    startCUAddrSliceIdx++;
    m_storedStartCUAddrForEncodingSliceSegment.push_back(pcSlice->getSliceCurEndCUAddr());
    startCUAddrSliceSegmentIdx++;
    
    pcSlice = pcPic->getSlice(0);
    
    // SAO parameter estimation using non-deblocked pixels for LCU bottom and right boundary areas
    if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoLcuBoundary() )
    {
      m_pcSAO->getPreDBFStatistics(pcPic);
    }
    //-- Loop filter
    Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
    m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
      dblMetric(pcPic, uiNumSlices);
    }
    m_pcLoopFilter->loopFilterPic( pcPic );

    /////////////////////////////////////////////////////////////////////////////////////////////////// File writing
    // Set entropy coder
    m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
    
    /* write various header sets. */
    if ( m_bSeqFirst )
    {
      OutputNALUnit nalu(NAL_UNIT_VPS);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      m_pcEntropyCoder->encodeVPS(m_pcEncTop->getVPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
      
      nalu = NALUnit(NAL_UNIT_SPS);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      if (m_bSeqFirst)
      {
        pcSlice->getSPS()->setNumLongTermRefPicSPS(m_numLongTermRefPicSPS);
        for (Int k = 0; k < m_numLongTermRefPicSPS; k++)
        {
          pcSlice->getSPS()->setLtRefPicPocLsbSps(k, m_ltRefPicPocLsbSps[k]);
          pcSlice->getSPS()->setUsedByCurrPicLtSPSFlag(k, m_ltRefPicUsedByCurrPicFlag[k]);
        }
      }
      if( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        UInt maxCU = m_pcCfg->getSliceArgument() >> ( pcSlice->getSPS()->getMaxCUDepth() << 1);
        UInt numDU = ( m_pcCfg->getSliceMode() == 1 ) ? ( pcPic->getNumCUsInFrame() / maxCU ) : ( 0 );
        if( pcPic->getNumCUsInFrame() % maxCU != 0 || numDU == 0 )
        {
          numDU ++;
        }
        pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->setNumDU( numDU );
        pcSlice->getSPS()->setHrdParameters( m_pcCfg->getFrameRate(), numDU, m_pcCfg->getTargetBitrate(), ( m_pcCfg->getIntraPeriod() > 0 ) );
      }
      if( m_pcCfg->getBufferingPeriodSEIEnabled() || m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
      }
      m_pcEntropyCoder->encodeSPS(pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
      
      nalu = NALUnit(NAL_UNIT_PPS);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      m_pcEntropyCoder->encodePPS(pcSlice->getPPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
      
      xCreateLeadingSEIMessages(accessUnit, pcSlice->getSPS());
      
      m_bSeqFirst = false;
    }
    
    if (writeSOP) // write SOP description SEI (if enabled) at the beginning of GOP
    {
      Int SOPcurrPOC = pocCurr;
      
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      
      SEISOPDescription SOPDescriptionSEI;
      SOPDescriptionSEI.m_sopSeqParameterSetId = pcSlice->getSPS()->getSPSId();
      
      UInt i = 0;
      UInt prevEntryId = iGOPid;
      for (j = iGOPid; j < m_iGopSize; j++)
      {
        Int deltaPOC = m_pcCfg->getGOPEntry(j).m_POC - m_pcCfg->getGOPEntry(prevEntryId).m_POC;
        if ((SOPcurrPOC + deltaPOC) < m_pcCfg->getFramesToBeEncoded())
        {
          SOPcurrPOC += deltaPOC;
          SOPDescriptionSEI.m_sopDescVclNaluType[i] = getNalUnitType(SOPcurrPOC, m_iLastIDR, isField);
          SOPDescriptionSEI.m_sopDescTemporalId[i] = m_pcCfg->getGOPEntry(j).m_temporalId;
          SOPDescriptionSEI.m_sopDescStRpsIdx[i] = m_pcEncTop->getReferencePictureSetIdxForSOP(pcSlice, SOPcurrPOC, j);
          SOPDescriptionSEI.m_sopDescPocDelta[i] = deltaPOC;
          
          prevEntryId = j;
          i++;
        }
      }
      
      SOPDescriptionSEI.m_numPicsInSopMinus1 = i - 1;
      
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, SOPDescriptionSEI, pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      
      writeSOP = false;
    }
    
    if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
       ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
       ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
        || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      if( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() )
      {
        UInt numDU = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNumDU();
        pictureTimingSEI.m_numDecodingUnitsMinus1     = ( numDU - 1 );
        pictureTimingSEI.m_duCommonCpbRemovalDelayFlag = false;
        
        if( pictureTimingSEI.m_numNalusInDuMinus1 == NULL )
        {
          pictureTimingSEI.m_numNalusInDuMinus1       = new UInt[ numDU ];
        }
        if( pictureTimingSEI.m_duCpbRemovalDelayMinus1  == NULL )
        {
          pictureTimingSEI.m_duCpbRemovalDelayMinus1  = new UInt[ numDU ];
        }
        if( accumBitsDU == NULL )
        {
          accumBitsDU                                  = new UInt[ numDU ];
        }
        if( accumNalsDU == NULL )
        {
          accumNalsDU                                  = new UInt[ numDU ];
        }
      }
      pictureTimingSEI.m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<double>(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
      pictureTimingSEI.m_picDpbOutputDelay = pcSlice->getSPS()->getNumReorderPics(pcSlice->getSPS()->getMaxTLayers()-1) + pcSlice->getPOC() - m_totalCoded;
#if EFFICIENT_FIELD_IRAP
      if(IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
      {
        // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
        pictureTimingSEI.m_picDpbOutputDelay ++;
      }
#endif
      Int factor = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2;
      pictureTimingSEI.m_picDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
      if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        picSptDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
      }
    }
    
    if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) &&
       ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
       ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
        || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      
      SEIBufferingPeriod sei_buffering_period;
      
      UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
      sei_buffering_period.m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;
      
      Double dTmp = (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();
      
      UInt uiTmp = (UInt)( dTmp * 90000.0 );
      uiInitialCpbRemovalDelay -= uiTmp;
      uiInitialCpbRemovalDelay -= uiTmp / ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
      sei_buffering_period.m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;
      
      sei_buffering_period.m_rapCpbParamsPresentFlag              = 0;
      //for the concatenation, it can be set to one during splicing.
      sei_buffering_period.m_concatenationFlag = 0;
      //since the temporal layer HRD is not ready, we assumed it is fixed
      sei_buffering_period.m_auCpbRemovalDelayDelta = 1;
      sei_buffering_period.m_cpbDelayOffset = 0;
      sei_buffering_period.m_dpbDelayOffset = 0;
      
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_buffering_period, pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      {
        UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
        UInt offsetPosition = m_activeParameterSetSEIPresentInAU;   // Insert BP SEI after APS SEI
        AccessUnit::iterator it;
        for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
        {
          it++;
        }
        accessUnit.insert(it, new NALUnitEBSP(nalu));
        m_bufferingPeriodSEIPresentInAU = true;
      }
      
      if (m_pcCfg->getScalableNestingSEIEnabled())
      {
        OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        m_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
        scalableNestingSEI.m_nestedSEIs.clear();
        scalableNestingSEI.m_nestedSEIs.push_back(&sei_buffering_period);
        m_seiWriter.writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
        writeRBSPTrailingBits(naluTmp.m_Bitstream);
        UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
        UInt offsetPosition = m_activeParameterSetSEIPresentInAU + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU;   // Insert BP SEI after non-nested APS, BP and PT SEIs
        AccessUnit::iterator it;
        for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
        {
          it++;
        }
        accessUnit.insert(it, new NALUnitEBSP(naluTmp));
        m_nestedBufferingPeriodSEIPresentInAU = true;
      }
      
      m_lastBPSEI = m_totalCoded;
      m_cpbRemovalDelay = 0;
    }
    m_cpbRemovalDelay ++;
    if( ( m_pcEncTop->getRecoveryPointSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) )
    {
      if( m_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !pcSlice->getRapPicFlag() )
      {
        // Gradual decoding refresh SEI
        OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
        
        SEIGradualDecodingRefreshInfo seiGradualDecodingRefreshInfo;
        seiGradualDecodingRefreshInfo.m_gdrForegroundFlag = true; // Indicating all "foreground"
        
        m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, pcSlice->getSPS() );
        writeRBSPTrailingBits(nalu.m_Bitstream);
        accessUnit.push_back(new NALUnitEBSP(nalu));
      }
      // Recovery point SEI
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      
      SEIRecoveryPoint sei_recovery_point;
      sei_recovery_point.m_recoveryPocCnt    = 0;
      sei_recovery_point.m_exactMatchingFlag = ( pcSlice->getPOC() == 0 ) ? (true) : (false);
      sei_recovery_point.m_brokenLinkFlag    = false;
#if ALLOW_RECOVERY_POINT_AS_RAP
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        m_iLastRecoveryPicPOC = pocCurr;
      }
#endif
      
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_recovery_point, pcSlice->getSPS() );
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
    }
    
    /* use the main bitstream buffer for storing the marshalled picture */
    m_pcEntropyCoder->setBitstream(NULL);
    
    startCUAddrSliceIdx = 0;
    startCUAddrSlice    = 0;
    
    startCUAddrSliceSegmentIdx = 0;
    startCUAddrSliceSegment    = 0;
    nextCUAddr                 = 0;
    pcSlice = pcPic->getSlice(startCUAddrSliceIdx);
    
    Int processingState = (pcSlice->getSPS()->getUseSAO())?(EXECUTE_INLOOPFILTER):(ENCODE_SLICE);
    Bool skippedSlice=false;
    while (nextCUAddr < uiRealEndAddress) // Iterate over all slices
    {
      switch(processingState)
      {
        case ENCODE_SLICE:
        {
          pcSlice->setNextSlice       ( false );
          pcSlice->setNextSliceSegment( false );
          if (nextCUAddr == m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx])
          {
            pcSlice = pcPic->getSlice(startCUAddrSliceIdx);
            if(startCUAddrSliceIdx > 0 && pcSlice->getSliceType()!= I_SLICE)
            {
              pcSlice->checkColRefIdx(startCUAddrSliceIdx, pcPic);
            }
            pcPic->setCurrSliceIdx(startCUAddrSliceIdx);
            m_pcSliceEncoder->setSliceIdx(startCUAddrSliceIdx);
            assert(startCUAddrSliceIdx == pcSlice->getSliceIdx());
            // Reconstruction slice
            pcSlice->setSliceCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
            pcSlice->setSliceCurEndCUAddr  ( m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx+1 ] );
            // Dependent slice
            pcSlice->setSliceSegmentCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
            pcSlice->setSliceSegmentCurEndCUAddr  ( m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx+1 ] );
            
            pcSlice->setNextSlice       ( true );
            
            startCUAddrSliceIdx++;
            startCUAddrSliceSegmentIdx++;
          }
          else if (nextCUAddr == m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx])
          {
            // Dependent slice
            pcSlice->setSliceSegmentCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
            pcSlice->setSliceSegmentCurEndCUAddr  ( m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx+1 ] );
            
            pcSlice->setNextSliceSegment( true );
            
            startCUAddrSliceSegmentIdx++;
          }
          
          pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
          pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());
          UInt uiDummyStartCUAddr;
          UInt uiDummyBoundingCUAddr;
          m_pcSliceEncoder->xDetermineStartAndBoundingCUAddr(uiDummyStartCUAddr,uiDummyBoundingCUAddr,pcPic,true);
          
          uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
          uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
          uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
          uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
          uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
          uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
          while(uiPosX>=uiWidth||uiPosY>=uiHeight)
          {
            uiInternalAddress--;
            uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
            uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
          }
          uiInternalAddress++;
          if(uiInternalAddress==pcPic->getNumPartInCU())
          {
            uiInternalAddress = 0;
            uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
          }
          UInt endAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);
          if(endAddress<=pcSlice->getSliceSegmentCurStartCUAddr())
          {
            UInt boundingAddrSlice, boundingAddrSliceSegment;
            boundingAddrSlice          = m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx];
            boundingAddrSliceSegment = m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx];
            nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
            if(pcSlice->isNextSlice())
            {
              skippedSlice=true;
            }
            continue;
          }
          if(skippedSlice)
          {
            pcSlice->setNextSlice       ( true );
            pcSlice->setNextSliceSegment( false );
          }
          skippedSlice=false;
          pcSlice->allocSubstreamSizes( iNumSubstreams );
          for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
          {
            pcSubstreamsOut[ui].clear();
          }
          
          m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
          m_pcEntropyCoder->resetEntropy      ();
          /* start slice NALunit */
          OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
          Bool sliceSegment = (!pcSlice->isNextSlice());
          if (!sliceSegment)
          {
            uiOneBitstreamPerSliceLength = 0; // start of a new slice
          }
          m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

#if SETTING_NO_OUT_PIC_PRIOR
          pcSlice->setNoRaslOutputFlag(false);
          if (pcSlice->isIRAP())
          {
            if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
            {
              pcSlice->setNoRaslOutputFlag(true);
            }
            //the inference for NoOutputPriorPicsFlag
            // KJS: This cannot happen at the encoder
            if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
            {
              if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
              {
                pcSlice->setNoOutputPriorPicsFlag(true);
              }
            }
          }
#endif

          tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();
          m_pcEntropyCoder->encodeSliceHeader(pcSlice);
          actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );
          
          // is it needed?
          {
            if (!sliceSegment)
            {
              pcBitstreamRedirect->writeAlignOne();
            }
            else
            {
              // We've not completed our slice header info yet, do the alignment later.
            }
            m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
            m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
            m_pcEntropyCoder->resetEntropy    ();
            for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
            {
              m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
              m_pcEntropyCoder->resetEntropy    ();
            }
          }
          
          if(pcSlice->isNextSlice())
          {
            // set entropy coder for writing
            m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
            {
              for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
              {
                m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
                m_pcEntropyCoder->resetEntropy    ();
              }
              pcSbacCoders[0].load(m_pcSbacCoder);
              m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[0], pcSlice );  //ALF is written in substream #0 with CABAC coder #0 (see ALF param encoding below)
            }
            m_pcEntropyCoder->resetEntropy    ();
            // File writing
            if (!sliceSegment)
            {
              m_pcEntropyCoder->setBitstream(pcBitstreamRedirect);
            }
            else
            {
              m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
            }
            // for now, override the TILES_DECODER setting in order to write substreams.
            m_pcEntropyCoder->setBitstream    ( &pcSubstreamsOut[0] );
            
          }
          pcSlice->setFinalized(true);
          
          m_pcSbacCoder->load( &pcSbacCoders[0] );
          
          pcSlice->setTileOffstForMultES( uiOneBitstreamPerSliceLength );
          pcSlice->setTileLocationCount ( 0 );
          m_pcSliceEncoder->encodeSlice(pcPic, pcSubstreamsOut);
          
          {
            // Construct the final bitstream by flushing and concatenating substreams.
            // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
            UInt* puiSubstreamSizes = pcSlice->getSubstreamSizes();
            UInt uiTotalCodedSize = 0; // for padding calcs.
            UInt uiNumSubstreamsPerTile = iNumSubstreams;
            if (iNumSubstreams > 1)
            {
              uiNumSubstreamsPerTile /= pcPic->getPicSym()->getNumTiles();
            }
            for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
            {
              // Flush all substreams -- this includes empty ones.
              // Terminating bit and flush.
              m_pcEntropyCoder->setEntropyCoder   ( &pcSbacCoders[ui], pcSlice );
              m_pcEntropyCoder->setBitstream      (  &pcSubstreamsOut[ui] );
              m_pcEntropyCoder->encodeTerminatingBit( 1 );
              m_pcEntropyCoder->encodeSliceFinish();
              
              pcSubstreamsOut[ui].writeByteAlignment();   // Byte-alignment in slice_data() at end of sub-stream
              // Byte alignment is necessary between tiles when tiles are independent.
              uiTotalCodedSize += pcSubstreamsOut[ui].getNumberOfWrittenBits();
              
              Bool bNextSubstreamInNewTile = ((ui+1) < iNumSubstreams)&& ((ui+1)%uiNumSubstreamsPerTile == 0);
              if (bNextSubstreamInNewTile)
              {
                pcSlice->setTileLocation(ui/uiNumSubstreamsPerTile, pcSlice->getTileOffstForMultES()+(uiTotalCodedSize>>3));
              }
              if (ui+1 < pcSlice->getPPS()->getNumSubstreams())
              {
                puiSubstreamSizes[ui] = pcSubstreamsOut[ui].getNumberOfWrittenBits() + (pcSubstreamsOut[ui].countStartCodeEmulations()<<3);
              }
            }
            
            // Complete the slice header info.
            m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
            m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );
            
            // Substreams...
            TComOutputBitstream *pcOut = pcBitstreamRedirect;
            Int offs = 0;
            Int nss = pcSlice->getPPS()->getNumSubstreams();
            if (pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
            {
              // 1st line present for WPP.
              offs = pcSlice->getSliceSegmentCurStartCUAddr()/pcSlice->getPic()->getNumPartInCU()/pcSlice->getPic()->getFrameWidthInCU();
              nss  = pcSlice->getNumEntryPointOffsets()+1;
            }
            for ( UInt ui = 0 ; ui < nss; ui++ )
            {
              pcOut->addSubstream(&pcSubstreamsOut[ui+offs]);
            }
          }
          
          UInt boundingAddrSlice, boundingAddrSliceSegment;
          boundingAddrSlice        = m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx];
          boundingAddrSliceSegment = m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx];
          nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
          // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
          // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
          Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
          xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
          accessUnit.push_back(new NALUnitEBSP(nalu));
          actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
          bNALUAlignedWrittenToList = true;
          uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits(); // length of bitstream after byte-alignment
          
          if (!bNALUAlignedWrittenToList)
          {
            {
              nalu.m_Bitstream.writeAlignZero();
            }
            accessUnit.push_back(new NALUnitEBSP(nalu));
            uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits() + 24; // length of bitstream after byte-alignment + 3 byte startcode 0x000001
          }
          
          if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
             ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
             ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
              || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
             ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
          {
            UInt numNalus = 0;
            UInt numRBSPBytes = 0;
            for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
            {
              UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
              if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
              {
                numRBSPBytes += numRBSPBytes_nal;
                numNalus ++;
              }
            }
            accumBitsDU[ pcSlice->getSliceIdx() ] = ( numRBSPBytes << 3 );
            accumNalsDU[ pcSlice->getSliceIdx() ] = numNalus;   // SEI not counted for bit count; hence shouldn't be counted for # of NALUs - only for consistency
          }
          processingState = ENCODE_SLICE;
        }
          break;
        case EXECUTE_INLOOPFILTER:
        {
          // set entropy coder for RD
          m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
          if ( pcSlice->getSPS()->getUseSAO() )
          {
            m_pcEntropyCoder->resetEntropy();
            m_pcEntropyCoder->setBitstream( m_pcBitCounter );
            Bool sliceEnabled[NUM_SAO_COMPONENTS];
            m_pcSAO->initRDOCabacCoder(m_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
            m_pcSAO->SAOProcess(pcPic
              , sliceEnabled
              , pcPic->getSlice(0)->getLambdas()
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
              , m_pcCfg->getSaoLcuBoundary()
#endif
              );
            m_pcSAO->PCMLFDisableProcess(pcPic);   

            //assign SAO slice header
            for(Int s=0; s< uiNumSlices; s++)
            {
              pcPic->getSlice(s)->setSaoEnabledFlag(sliceEnabled[SAO_Y]);
              assert(sliceEnabled[SAO_Cb] == sliceEnabled[SAO_Cr]);
              pcPic->getSlice(s)->setSaoEnabledFlagChroma(sliceEnabled[SAO_Cb]);
            }
          }
          processingState = ENCODE_SLICE;
        }
          break;
        default:
        {
          printf("Not a supported encoding state\n");
          assert(0);
          exit(-1);
        }
      }
    } // end iteration over slices
#if ETRI_COMPRESSMOTION_OPTIMIZATION
	if (pcPic->getTLayer())
		pcPic->compressMotion(); 
#else
    pcPic->compressMotion();
#endif    
    //-- For time output for each slice
#if (_ETRI_WINDOWS_APPLICATION)
    Double dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
#else
	timespec iCurrTime;
	clock_gettime(CLOCK_MONOTONIC, &iCurrTime); // Works on Linux by yhee 2016.04.19
	Double dEncTime = (iCurrTime.tv_sec - iBeforeTime.tv_sec);
	dEncTime += (iCurrTime.tv_nsec - iBeforeTime.tv_nsec) / 1000000000.0;
#endif
    
    const Char* digestStr = NULL;
    if (m_pcCfg->getDecodedPictureHashSEIEnabled())
    {
      /* calculate MD5sum for entire reconstructed picture */
      SEIDecodedPictureHash sei_recon_picture_digest;
      if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::MD5;
        calcMD5(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
        digestStr = digestToString(sei_recon_picture_digest.digest, 16);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::CRC;
        calcCRC(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
        digestStr = digestToString(sei_recon_picture_digest.digest, 2);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::CHECKSUM;
        calcChecksum(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
        digestStr = digestToString(sei_recon_picture_digest.digest, 4);
      }
      OutputNALUnit nalu(NAL_UNIT_SUFFIX_SEI, pcSlice->getTLayer());
      
      /* write the SEI messages */
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      
      accessUnit.insert(accessUnit.end(), new NALUnitEBSP(nalu));
    }
    if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
    {
      SEITemporalLevel0Index sei_temporal_level0_index;
      if (pcSlice->getRapPicFlag())
      {
        m_tl0Idx = 0;
        m_rapIdx = (m_rapIdx + 1) & 0xFF;
      }
      else
      {
        m_tl0Idx = (m_tl0Idx + (pcSlice->getTLayer() ? 0 : 1)) & 0xFF;
      }
      sei_temporal_level0_index.tl0Idx = m_tl0Idx;
      sei_temporal_level0_index.rapIdx = m_rapIdx;
      
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      
      /* write the SEI messages */
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      
      /* insert the SEI message NALUnit before any Slice NALUnits */
      AccessUnit::iterator it = find_if(accessUnit.begin(), accessUnit.end(), mem_fun(&NALUnit::isSlice));
      accessUnit.insert(it, new NALUnitEBSP(nalu));
    }
    
    xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime );
    
    //In case of field coding, compute the interlaced PSNR for both fields
    if (isField && ((!pcPic->isTopField() && isTff) || (pcPic->isTopField() && !isTff)) && (pcPic->getPOC()%m_iGopSize != 1))
    {
      //get complementary top field
      TComPic* pcPicTop;
      TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != pcPic->getPOC()-1)
      {
        iterPic ++;
      }
      pcPicTop = *(iterPic);
      xCalculateInterlacedAddPSNR(pcPicTop, pcPic, pcPicTop->getPicYuvRec(), pcPic->getPicYuvRec(), accessUnit, dEncTime );
    }
    else if (isField && pcPic->getPOC()!= 0 && (pcPic->getPOC()%m_iGopSize == 0))
    {
      //get complementary bottom field
      TComPic* pcPicBottom;
      TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != pcPic->getPOC()+1)
      {
        iterPic ++;
      }
      pcPicBottom = *(iterPic);
      xCalculateInterlacedAddPSNR(pcPic, pcPicBottom, pcPic->getPicYuvRec(), pcPicBottom->getPicYuvRec(), accessUnit, dEncTime );
    }
    
    if (digestStr)
    {
      if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
      {
        printf(" [MD5:%s]", digestStr);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
      {
        printf(" [CRC:%s]", digestStr);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
      {
        printf(" [Checksum:%s]", digestStr);
      }
    }
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
      Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
      if ( avgLambda < 0.0 )
      {
        avgLambda = lambda;
      }
      m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
      m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );
      
      m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
      if ( pcSlice->getSliceType() != I_SLICE )
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
      }
      else    // for intra picture, the estimated bits are used to update the current status in the GOP
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
      }
    }
    
    if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
       ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
       ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
        || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      TComVUI *vui = pcSlice->getSPS()->getVuiParameters();
      TComHRD *hrd = vui->getHrdParameters();
      
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        Int i;
        UInt64 ui64Tmp;
        UInt uiPrev = 0;
        UInt numDU = ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 );
        UInt *pCRD = &pictureTimingSEI.m_duCpbRemovalDelayMinus1[0];
        UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;
        
        for( i = 0; i < numDU; i ++ )
        {
          pictureTimingSEI.m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( accumNalsDU[ i ] - 1 ) : ( accumNalsDU[ i ] - accumNalsDU[ i - 1] - 1 );
        }
        
        if( numDU == 1 )
        {
          pCRD[ 0 ] = 0; /* don't care */
        }
        else
        {
          pCRD[ numDU - 1 ] = 0;/* by definition */
          UInt tmp = 0;
          UInt accum = 0;
          
          for( i = ( numDU - 2 ); i >= 0; i -- )
          {
            ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
            if( (UInt)ui64Tmp > maxDiff )
            {
              tmp ++;
            }
          }
          uiPrev = 0;
          
          UInt flag = 0;
          for( i = ( numDU - 2 ); i >= 0; i -- )
          {
            flag = 0;
            ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
            
            if( (UInt)ui64Tmp > maxDiff )
            {
              if(uiPrev >= maxDiff - tmp)
              {
                ui64Tmp = uiPrev + 1;
                flag = 1;
              }
              else                            ui64Tmp = maxDiff - tmp + 1;
            }
            pCRD[ i ] = (UInt)ui64Tmp - uiPrev - 1;
            if( (Int)pCRD[ i ] < 0 )
            {
              pCRD[ i ] = 0;
            }
            else if (tmp > 0 && flag == 1)
            {
              tmp --;
            }
            accum += pCRD[ i ] + 1;
            uiPrev = accum;
          }
        }
      }
      if( m_pcCfg->getPictureTimingSEIEnabled() )
      {
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
          m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
          pictureTimingSEI.m_picStruct = (isField && pcSlice->getPic()->isTopField())? 1 : isField? 2 : 0;
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, pcSlice->getSPS());
          writeRBSPTrailingBits(nalu.m_Bitstream);
          UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
          UInt offsetPosition = m_activeParameterSetSEIPresentInAU
          + m_bufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
          AccessUnit::iterator it;
          for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
          {
            it++;
          }
          accessUnit.insert(it, new NALUnitEBSP(nalu));
          m_pictureTimingSEIPresentInAU = true;
        }
        if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
          m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
          scalableNestingSEI.m_nestedSEIs.clear();
          scalableNestingSEI.m_nestedSEIs.push_back(&pictureTimingSEI);
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
          writeRBSPTrailingBits(nalu.m_Bitstream);
          UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
          UInt offsetPosition = m_activeParameterSetSEIPresentInAU
          + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU + m_nestedBufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
          AccessUnit::iterator it;
          for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
          {
            it++;
          }
          accessUnit.insert(it, new NALUnitEBSP(nalu));
          m_nestedPictureTimingSEIPresentInAU = true;
        }
      }
      if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
      {
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        for( Int i = 0; i < ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 ); i ++ )
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
          
          SEIDecodingUnitInfo tempSEI;
          tempSEI.m_decodingUnitIdx = i;
          tempSEI.m_duSptCpbRemovalDelay = pictureTimingSEI.m_duCpbRemovalDelayMinus1[i] + 1;
          tempSEI.m_dpbOutputDuDelayPresentFlag = false;
          tempSEI.m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;
          
          AccessUnit::iterator it;
          // Insert the first one in the right location, before the first slice
          if(i == 0)
          {
            // Insert before the first slice.
            m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
            writeRBSPTrailingBits(nalu.m_Bitstream);
            
            UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
            UInt offsetPosition = m_activeParameterSetSEIPresentInAU
            + m_bufferingPeriodSEIPresentInAU
            + m_pictureTimingSEIPresentInAU;  // Insert DU info SEI after APS, BP and PT SEI
            for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
            {
              it++;
            }
            accessUnit.insert(it, new NALUnitEBSP(nalu));
          }
          else
          {
            Int ctr;
            // For the second decoding unit onwards we know how many NALUs are present
            for (ctr = 0, it = accessUnit.begin(); it != accessUnit.end(); it++)
            {
              if(ctr == accumNalsDU[ i - 1 ])
              {
                // Insert before the first slice.
                m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
                writeRBSPTrailingBits(nalu.m_Bitstream);
                
                accessUnit.insert(it, new NALUnitEBSP(nalu));
                break;
              }
              if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
              {
                ctr++;
              }
            }
          }
        }
      }
    }
    xResetNonNestedSEIPresentFlags();
    xResetNestedSEIPresentFlags();
    pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
    
    pcPic->setReconMark   ( true );
    m_bFirst = false;
    m_iNumPicCoded++;
    m_totalCoded ++;
    /* logging: insert a newline at end of picture period */
    printf("\n");
    fflush(stdout);
    
    delete[] pcSubstreamsOut;

#if EFFICIENT_FIELD_IRAP
    if(IRAPtoReorder)
    {
      if(swapIRAPForward)
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid +1;
          IRAPtoReorder = false;
        }
        else if(iGOPid == IRAPGOPid +1)
        {
          iGOPid --;
        }
      }
      else
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid -1;
        }
        else if(iGOPid == IRAPGOPid -1)
        {
          iGOPid = IRAPGOPid;
          IRAPtoReorder = false;
        }
      }
    }
#endif
  }
  delete pcBitstreamRedirect;
  
  if( accumBitsDU != NULL) delete accumBitsDU;
  if( accumNalsDU != NULL) delete accumNalsDU;
  
  assert ( (m_iNumPicCoded == iNumPicRcvd) || (isField && iPOCLast == 1) );
}


#endif


#if ETRI_MULTITHREAD_2
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// gplusplus
void TEncGOP::threadProcessingGOP(void *param, int num)
{
	TEncGOP *pcGOP = (TEncGOP *)param;
	TEncThreadGOP *pcThreadGOP = &pcGOP->em_cThreadGOP[num];
	UInt *accumBitsDU = NULL;
	UInt *accumNalsDU = NULL;

#if	_YHEEDEBUG
	long iBeforeTime = clock();
	Double dEncTime;
	printf("Start:\tPOC = %d\n", pcThreadGOP->em_pocCurr); fflush(stdout);
#endif
	num = pcThreadGOP->em_iPos;

#if 0 //gplusplus_151124 Thread Function -> Local Function
	pcGOP->em_pcFrameEncoder[num].ETRI_setFrameParameter(pcThreadGOP->em_iGOPid, pcThreadGOP->em_iPOCLast, pcThreadGOP->em_iNumPicRcvd, pcThreadGOP->em_IRAPGOPid,
		pcThreadGOP->em_iLastIDR, accumBitsDU, accumNalsDU, pcThreadGOP->em_isField, pcThreadGOP->em_isTff);
#endif

#if ETRI_THREADPOOL_OPT
	// original code starts_yjc
	pcGOP->em_pcFrameEncoder[num].ETRI_CompressFrame(pcThreadGOP->em_pocCurr, pcThreadGOP->em_pcPic, pcThreadGOP->em_pcPicYuvRecOut,
		*pcThreadGOP->em_rpcListPic, *pcThreadGOP->em_paccessUnitsInGOP, pcThreadGOP->em_bFirst, NULL, NULL, NULL, NULL, false);
#else

	pcGOP->em_pcFrameEncoder[num].ETRI_CompressFrame(pcThreadGOP->em_pocCurr, pcThreadGOP->em_pcPic, pcThreadGOP->em_pcPicYuvRecOut,
		*pcThreadGOP->em_rpcListPic, *pcThreadGOP->em_paccessUnitsInGOP, pcThreadGOP->em_bFirst, false);
#endif

#if PSNR_DISPLAY	
	pcGOP->ETRI_xCalculateAddPSNR(pcThreadGOP->em_pcPic, *pcThreadGOP->em_rpcListPic, *pcGOP->em_pcFrameEncoder[num].ETRI_getAccessUnitFrame(), pcGOP->em_pcFrameEncoder[num].ETRI_getdEncTime(), pcThreadGOP->em_isField, pcThreadGOP->em_isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
	printf("\n");	fflush(stdout);
#if _YHEEDEBUG
	dEncTime = (Double)(clock() - iBeforeTime) / CLOCKS_PER_SEC;
	printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif
	
//#if	_YHEEDEBUG
//	printf("POC:%d Frame EncTime : %12.3f sec.\n", pcThreadGOP->em_pocCurr, dEncTime);	fflush(stdout);
//#endif
	//pcGOP->m_bFirst = false; //need Critical Section?? yhee 2015.11.20.
	
	if( accumBitsDU != NULL) delete accumBitsDU;
	if( accumNalsDU != NULL) delete accumNalsDU;
//#if	_YHEEDEBUG
//	printf("==================End:\tPOC = %d\n", pcThreadGOP->em_pocCurr); fflush(stdout);
//#endif
}

bool TEncGOP::ETRI_RefPicCheck(int curPOC, TComList<TComPic*>* rpcListPic, int RefSize)
{
	int refPoc;
	TComList<TComPic*>::iterator iterPic;
	TComPic* rpcPic;
	int i;

	for(i = 0; i < RefSize; i++)
	{
		if(curPOC == em_refPic[i].nPOC)
		{
			//assert(em_refPic[i].nRefNum!=0);

			for(int j = 0; j < em_refPic[i].nRefNum; j++)
			{
				refPoc = em_refPic[i].pRefPOC[j];

				iterPic = rpcListPic->begin();
				while(iterPic != rpcListPic->end())
				{
					rpcPic = *(iterPic);
					rpcPic->setCurrSliceIdx(0);

					if (rpcPic->getPOC() == refPoc)
					{
					  break;
					}
					iterPic++;
				}

				if(rpcPic->getReconMark() == false)
					return false;
			}
			break;
		}
	}

	assert(i!=RefSize);
	return true;
}

Void TEncGOP::ETRI_xGetBuffer( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, int num, bool isField)
{
  //  Rec. output
  TComList<TComPicYuv*>::iterator iterPicYuvRec = rcListPicYuvRecOut.begin();   
  //  Current pic.
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();


#if ETRI_MULTITHREAD_2
  if (m_pcEncTop->ETRI_getReconFileOk())
#endif
	  iterPicYuvRec = rcListPicYuvRecOut.begin();

  while (iterPic != rcListPic.end())
  {
#if ETRI_MULTITHREAD_2	 
	  if (m_pcEncTop->ETRI_getReconFileOk())
#endif
		  rpcPicYuvRecOut = *(iterPicYuvRec);


	  rpcPic = *(iterPic);
	  rpcPic->setCurrSliceIdx(0);
	  if (rpcPic->getPOC() == pocCurr)
	  {
#if ETRI_MULTITHREAD_2
		  
		  if (m_pcEncTop->ETRI_getReconFileOk())
#endif
		  {
			  rpcPicYuvRecOut->setPoc(pocCurr);
			  rpcPicYuvRecOut->setbUsed(true);
		  }
		  break;
	  }
#if ETRI_MULTITHREAD_2
	  
	  if (m_pcEncTop->ETRI_getReconFileOk())
#endif
		  iterPicYuvRec++;
	  iterPic++;
  }
  
  assert( rpcPic != NULL );
  assert( rpcPic->getPOC() == pocCurr );
  
  return;
}

////! \}

#endif
////! \}


#if ETRI_THREADPOOL_OPT && ETRI_MULTITHREAD_2

EncGopJob* EncGopJob::createJob(void *param, int id, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, bool *bThreadRunning)
{
	EncGopJob *job = new EncGopJob(param, id, mutex, cond, bRefPicAvailable, threadpool, bThreadRunning);
	return job;
}

EncGopJob::EncGopJob(void *param, int id, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, bool *bThreadRunning)
{
	m_encGOP = (TEncGOP *)param;

	m_threadpool = threadpool;
	m_bThreadRunning = bThreadRunning;
	m_bRefPicAvailable = bRefPicAvailable;
	m_mutex = mutex;
	m_cond = cond;
	m_id = id;
}

EncGopJob::~EncGopJob()
{
	m_bThreadRunning[m_id] = false;
}

void EncGopJob::run(void *)
{
	void *param = m_encGOP;
	int num = m_id;

	//TEncGOP *pcGOP = (TEncGOP *)param;
	//TEncThreadGOP *pcThreadGOP = &pcGOP->em_cThreadGOP[num];
	//UInt *accumBitsDU = NULL;
	//UInt *accumNalsDU = NULL;



	////////////////////////////////////
	////////////////////////////////////
	TEncGOP *pcGOP = (TEncGOP *)param;
	TEncThreadGOP *pcThreadGOP = &pcGOP->em_cThreadGOP[num];
	UInt *accumBitsDU = NULL;
	UInt *accumNalsDU = NULL;

#if	_YHEEDEBUG
	long iBeforeTime = clock();
	Double dEncTime;
	printf("Start:\tPOC = %d\n", pcThreadGOP->em_pocCurr); fflush(stdout);
#endif
	num = pcThreadGOP->em_iPos;

#if 0 //gplusplus_151124 Thread Function -> Local Function
	pcGOP->em_pcFrameEncoder[num].ETRI_setFrameParameter(pcThreadGOP->em_iGOPid, pcThreadGOP->em_iPOCLast, pcThreadGOP->em_iNumPicRcvd, pcThreadGOP->em_IRAPGOPid,
		pcThreadGOP->em_iLastIDR, accumBitsDU, accumNalsDU, pcThreadGOP->em_isField, pcThreadGOP->em_isTff);
#endif


#if ETRI_THREADPOOL_OPT
	// original code starts_yjc
pcGOP->em_pcFrameEncoder[num].ETRI_CompressFrame(pcThreadGOP->em_pocCurr, pcThreadGOP->em_pcPic, pcThreadGOP->em_pcPicYuvRecOut,
	*pcThreadGOP->em_rpcListPic, *pcThreadGOP->em_paccessUnitsInGOP, pcThreadGOP->em_bFirst, m_mutex, m_cond, m_bRefPicAvailable, m_threadpool, false);
#else

	pcGOP->em_pcFrameEncoder[num].ETRI_CompressFrame(pcThreadGOP->em_pocCurr, pcThreadGOP->em_pcPic, pcThreadGOP->em_pcPicYuvRecOut,
		*pcThreadGOP->em_rpcListPic, *pcThreadGOP->em_paccessUnitsInGOP, pcThreadGOP->em_bFirst, false);
#endif
#if PSNR_DISPLAY	
	pcGOP->ETRI_xCalculateAddPSNR(pcThreadGOP->em_pcPic, *pcThreadGOP->em_rpcListPic, *pcGOP->em_pcFrameEncoder[num].ETRI_getAccessUnitFrame(), pcGOP->em_pcFrameEncoder[num].ETRI_getdEncTime(), pcThreadGOP->em_isField, pcThreadGOP->em_isTff); 	///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
	printf("\n");	fflush(stdout);
#if _YHEEDEBUG
	dEncTime = (Double)(clock() - iBeforeTime) / CLOCKS_PER_SEC;
	printf(", Frame Time : %12.3f sec.\n", dEncTime);	fflush(stdout);
#endif
#endif

	//#if	_YHEEDEBUG
	//	printf("POC:%d Frame EncTime : %12.3f sec.\n", pcThreadGOP->em_pocCurr, dEncTime);	fflush(stdout);
	//#endif
	//pcGOP->m_bFirst = false; //need Critical Section?? yhee 2015.11.20.

	if (accumBitsDU != NULL) delete accumBitsDU;
	if (accumNalsDU != NULL) delete accumNalsDU;
	//#if	_YHEEDEBUG
	//	printf("==================End:\tPOC = %d\n", pcThreadGOP->em_pocCurr); fflush(stdout);
	//#endif
}

#endif