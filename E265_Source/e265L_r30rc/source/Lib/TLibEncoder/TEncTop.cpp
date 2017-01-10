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

/** \file     TEncTop.cpp
    \brief    encoder class
*/

#include "TLibCommon/CommonDef.h"
#include "TEncTop.h"
#include "TEncPic.h"
#if FAST_BIT_EST
#include "TLibCommon/ContextModel.h"
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncTop::TEncTop()
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;
#if !ETRI_MULTITHREAD_2
  m_pppcRDSbacCoder   =  NULL;
  m_pppcBinCoderCABAC =  NULL;
  m_cRDGoOnSbacCoder.init( &m_cRDGoOnBinCoderCABAC );
#endif
#if ENC_DEC_TRACE
  g_hTrace = fopen( "TraceEnc.txt", "wb" );
  g_bJustDoIt = g_bEncDecTraceDisable;
  g_nSymbolCounter = 0;
#endif

  m_iMaxRefPicNum     = 0;

#if FAST_BIT_EST
  ContextModel::buildNextStateTable();
#endif
#if !ETRI_MULTITHREAD_2
  m_pcSbacCoders           = NULL;
  m_pcBinCoderCABACs       = NULL;
  m_ppppcRDSbacCoders      = NULL;
  m_ppppcBinCodersCABAC    = NULL;
  m_pcRDGoOnSbacCoders     = NULL;
  m_pcRDGoOnBinCodersCABAC = NULL;
  m_pcBitCounters          = NULL;
  m_pcRdCosts              = NULL;
#else
  em_bReconFileOk = false;  // gplusplus_151102 memory optimizer
#endif
#if ETRI_DLL_INTERFACE
  em_bAnalyserClear		   = false;
#endif
}

TEncTop::~TEncTop()
{
#if ENC_DEC_TRACE
  fclose( g_hTrace );
#endif
}

Void TEncTop::create ()
{
  // initialize global variables
  initROM();
#if ETRI_COMPONENT_BIT_OPTIMIZATION
  initComponentBits();
#endif

  // create processing unit classes
#if ETRI_MULTITHREAD_2
  m_cGOPEncoder.        create(this);
#else
  m_cGOPEncoder.        create();
  m_cSliceEncoder.      create( getSourceWidth(), getSourceHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#if !ETRI_UNNECESSARY_CODE_REMOVAL
  m_cCuEncoder.         create( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight );
#endif

  if (m_bUseSAO)
  {
    m_cEncSAO.create( getSourceWidth(), getSourceHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
    m_cEncSAO.createEncData(getSaoLcuBoundary());
#else
    m_cEncSAO.createEncData();
#endif
  }
#if ADAPTIVE_QP_SELECTION
  if (m_bUseAdaptQpSelect)
  {
    m_cTrQuant.initSliceQpDelta();
  }
#endif
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
  for (Int i=0; i<MAX_NUM_THREAD; i++)
  {
	  m_cLoopFilter[i].create(g_uiMaxCUDepth);
  }
#else 
  m_cLoopFilter.        create( g_uiMaxCUDepth );
#endif 
  if ( m_RCEnableRateControl )
  {
//     m_cRateCtrl.init( m_framesToBeEncoded, m_RCTargetBitrate, m_iFrameRate, m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
//                       g_uiMaxCUWidth, g_uiMaxCUHeight, m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList );
  }

  m_pppcRDSbacCoder = new TEncSbac** [g_uiMaxCUDepth+1];
#if FAST_BIT_EST
  m_pppcBinCoderCABAC = new TEncBinCABACCounter** [g_uiMaxCUDepth+1];
#else
  m_pppcBinCoderCABAC = new TEncBinCABAC** [g_uiMaxCUDepth+1];
#endif

  for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
  {
    m_pppcRDSbacCoder[iDepth] = new TEncSbac* [CI_NUM];
#if FAST_BIT_EST
    m_pppcBinCoderCABAC[iDepth] = new TEncBinCABACCounter* [CI_NUM];
#else
    m_pppcBinCoderCABAC[iDepth] = new TEncBinCABAC* [CI_NUM];
#endif

    for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
    {
      m_pppcRDSbacCoder[iDepth][iCIIdx] = new TEncSbac;
#if FAST_BIT_EST
      m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABACCounter;
#else
      m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABAC;
#endif
      m_pppcRDSbacCoder   [iDepth][iCIIdx]->init( m_pppcBinCoderCABAC [iDepth][iCIIdx] );
    }
  }
#endif

//------------------------------------------------------------------------
// ETRI Amendment @ 2015 5 18 by Seok
//------------------------------------------------------------------------
	ETRI_TEncTopCreate(ETRI_MODIFICATION_V00);	/// 2015 5 18 by Seok

}
#if !ETRI_MULTITHREAD_2
/**
 - Allocate coders required for wavefront for the nominated number of substreams.
 .
 \param iNumSubstreams Determines how much information to allocate.
 */
Void TEncTop::createWPPCoders(Int iNumSubstreams)
{
  if (m_pcSbacCoders != NULL)
  {
    return; // already generated.
  }

  m_iNumSubstreams         = iNumSubstreams;
  m_pcSbacCoders           = new TEncSbac       [iNumSubstreams];
  m_pcBinCoderCABACs       = new TEncBinCABAC   [iNumSubstreams];
  m_pcRDGoOnSbacCoders     = new TEncSbac       [iNumSubstreams];
  m_pcRDGoOnBinCodersCABAC = new TEncBinCABAC   [iNumSubstreams];
  m_pcBitCounters          = new TComBitCounter [iNumSubstreams];
  m_pcRdCosts              = new TComRdCost     [iNumSubstreams];

  for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
  {
    m_pcRDGoOnSbacCoders[ui].init( &m_pcRDGoOnBinCodersCABAC[ui] );
    m_pcSbacCoders[ui].init( &m_pcBinCoderCABACs[ui] );
  }

  m_ppppcRDSbacCoders      = new TEncSbac***    [iNumSubstreams];
  m_ppppcBinCodersCABAC    = new TEncBinCABAC***[iNumSubstreams];
  for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ )
  {
    m_ppppcRDSbacCoders[ui]  = new TEncSbac** [g_uiMaxCUDepth+1];
    m_ppppcBinCodersCABAC[ui]= new TEncBinCABAC** [g_uiMaxCUDepth+1];

    for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
    {
      m_ppppcRDSbacCoders[ui][iDepth]  = new TEncSbac*     [CI_NUM];
      m_ppppcBinCodersCABAC[ui][iDepth]= new TEncBinCABAC* [CI_NUM];

      for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
      {
        m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx] = new TEncSbac;
        m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] = new TEncBinCABAC;
        m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx]->init( m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] );
      }
    }
  }
}
#endif
// ====================================================================================================================
// ETRI TEncTop Functions @ 2015 5 11 by Seok
// ====================================================================================================================
/**
-----------------------------------------------------------------------------------------------------------------------
	@brief: Creation of ETRI defined Classes 
	@Author: Jinwuk Seok  2015 5 11 
-----------------------------------------------------------------------------------------------------------------------
*/
Void TEncTop::ETRI_TEncTopCreate(Bool bOperation)
{
	if (!bOperation){return;}

#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
//#if MAX_THREADPOOL_TOP
//	m_hThreadPoolTop.Create(threadTopProcessing, MAX_THREADPOOL_TOP);	// gplusplus
//#endif
#endif
#else
	//-------------------------------------------------------------
	//	Create Frame Encoder
	//-------------------------------------------------------------
	em_pcFrameEncoder = new TEncFrame;

	//-------------------------------------------------------------
	//	Re-Create Slice Encoder
	//-------------------------------------------------------------

	em_pcSliceEncoder = new TEncSlice;

	//-------------------------------------------------------------
	//	Create Tile Encoder
	//-------------------------------------------------------------
	UInt	uiNumTile = (m_iNumColumnsMinus1 + 1) * (m_iNumRowsMinus1 + 1);
	
	EDPRINTF(stderr, "m_iNumColumnsMinus1 : %d \n", m_iNumColumnsMinus1);
	EDPRINTF(stderr, "m_iNumRowsMinus1    : %d \n", m_iNumRowsMinus1);
	EDPRINTF(stderr, "uiNumTile           : %d \n", uiNumTile);

	em_pcTileEncoder 	= new TEncTile[uiNumTile];
	for(UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].ETRI_getTotalNumbetOfTile() = uiNumTile;
		em_pcTileEncoder[uiTileIdx].create();
	}
#endif
}

Void TEncTop::ETRI_TEncTopDestroy(Bool bOperation)
{
	if (!bOperation){return;}
	EDPRINTF(stderr, "Function CALL \n");
	
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
//#if MAX_THREADPOOL_TOP
//	m_hThreadPoolTop.FinishThread();	// gplusplus
//#endif
#endif
#else

	//-------------------------------------------------------------
	//	Destory Tile Encoder
	//-------------------------------------------------------------
	UInt uiNumTile = em_pcTileEncoder[0].ETRI_getTotalNumbetOfTile();
	for(UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].destroy(bOperation);
		ESPRINTF(ETRI_MODV2_DEBUG, stderr, "em_pcTileEncoder[%d].destroy	OK \n", uiTileIdx);
	}

	if(em_pcTileEncoder)	{delete[] em_pcTileEncoder;}
	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "em_pcTileEncoder Release OK \n");

	//-------------------------------------------------------------
	//	Destory Slice Encoder
	//-------------------------------------------------------------

	if (em_pcSliceEncoder)	{delete em_pcSliceEncoder;}
	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "em_pcSliceEncoder Release OK \n");


	//-------------------------------------------------------------
	//	Destory Frame Encoder
	//-------------------------------------------------------------

	if (em_pcFrameEncoder)	{delete em_pcFrameEncoder;}
	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "em_pcFrameEncoder Release OK \n");
#endif
	//-------------------------------------------------------------
	//	Destory TS Data
	//-------------------------------------------------------------
#if ETRI_DLL_INTERFACE	// 2013 10 24 by Seok 
	if (em_pFrameEncodingOrder)	{delete[] em_pFrameEncodingOrder; em_pFrameEncodingOrder = NULL;}
	if (em_pFrameTypeInGOP)	   	{delete[] em_pFrameTypeInGOP; em_pFrameTypeInGOP = NULL;}
	if (em_pFramePOC)			{delete[] em_pFramePOC; em_pFramePOC = NULL;}
	if (em_pFrameSliceIndex)			{ delete[] em_pFrameSliceIndex; em_pFrameSliceIndex = NULL; }
#endif

}

Void TEncTop::ETRI_TEncTopInit(Bool bOperation)
{
	if (bOperation)
	{
		EDPRINTF(stderr, "Initialization ETRI's Class and Functionals \n");
	}
	else 	{ return; }

#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
//#if MAX_THREADPOOL_TOP
//	m_hThreadPoolTop.Init();	// gplusplus
//#endif
#endif
#else

	//-------------------------------------------------------------
	//	Init Frame Encoder
	//-------------------------------------------------------------

	em_pcFrameEncoder->init(this, &m_cGOPEncoder, 0);

	//-------------------------------------------------------------
	//	Re-Init Slice Encoder
	//-------------------------------------------------------------



	//-------------------------------------------------------------
	//	Init Tile Encoder
	//-------------------------------------------------------------
	UInt	uiNumTile = (m_iNumColumnsMinus1 + 1) * (m_iNumRowsMinus1 + 1);

	for(UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].init(this, uiTileIdx, bOperation);
	}

#endif
	//-------------------------------------------------------------
	//	Init DLL TS data
	//-------------------------------------------------------------
#if ETRI_DLL_INTERFACE	
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	// create encoder search class after m_iGOPSize is set
	em_pFrameEncodingOrder = new UInt[m_uiIntraPeriod];
	em_pFrameTypeInGOP     = new UInt[m_uiIntraPeriod];
	em_pFramePOC           = new UInt[m_uiIntraPeriod];
	em_pFrameSliceIndex    = new short[m_uiIntraPeriod];
#else
	// create encoder search class after m_iGOPSize is set
	em_pFrameEncodingOrder 	= new UInt [m_iGOPSize];
	em_pFrameTypeInGOP		= new UInt [m_iGOPSize];
	em_pFramePOC			= new UInt [m_iGOPSize];
	em_pFrameSliceIndex 	= new short[m_iGOPSize];
#endif
#endif
}
// --------------------------------------------------------------------------------------------------------------------
// End of ETRI TEncTop Functions @ 2015 5 11 by Seok
// --------------------------------------------------------------------------------------------------------------------

Void TEncTop::destroy ()
{

	//------------------------------------------------------------------------
	// ETRI Amendment @ 2015 5 18 by Seok
	//------------------------------------------------------------------------
	ETRI_TEncTopDestroy(ETRI_MODIFICATION_V00);	  /// 2015 5 18 by Seok
	//------------------------------------------------------------------------	

	// destroy processing unit classes
	m_cGOPEncoder.        destroy();	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "m_cGOPEncoder.destroy() : OK \n");
#if KAIST_RC
// 	for (Int i = 0; i < m_vcRateCtrl.capacity(); i++)
// 		m_vcRateCtrl[i].destroy();
	if (m_RCEnableRateControl)
		m_lcRateCtrl.clear();
#endif
#if !ETRI_MULTITHREAD_2
	m_cSliceEncoder.      destroy();	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "m_cSliceEncoder.destroy() : OK \n");
#if !ETRI_UNNECESSARY_CODE_REMOVAL
	m_cCuEncoder.destroy();	ESPRINTF(ETRI_MODV2_DEBUG, stderr, "m_cCuEncoder.destroy() : OK \n");
#endif
	if (m_cSPS.getUseSAO())
	{
		m_cEncSAO.destroyEncData();
		m_cEncSAO.destroy();
		ESPRINTF(ETRI_MODV2_DEBUG, stderr, "m_cEncSAO.destroy() : OK \n");
	}
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
	for (int i=0; i<MAX_NUM_THREAD; i++)
		m_cLoopFilter[i].destroy();
#else 
	m_cLoopFilter.        destroy();
#endif 
#if KAIST_RC
	if (m_RCEnableRateControl)
		m_cRateCtrl.          destroy();
#endif

	Int iDepth;
	for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			delete m_pppcRDSbacCoder[iDepth][iCIIdx];
			delete m_pppcBinCoderCABAC[iDepth][iCIIdx];
		}
	}

	for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		delete [] m_pppcRDSbacCoder[iDepth];
		delete [] m_pppcBinCoderCABAC[iDepth];
	}

	delete [] m_pppcRDSbacCoder;
	delete [] m_pppcBinCoderCABAC;
	// gplusplus [[ //Why yhee, 20151028	
	/*if(!m_pcSbacCoders)
	{
		// destroy ROM
		destroyROM();

		return;
	}*/
	// ]]

	for ( UInt ui = 0; ui < m_iNumSubstreams; ui++ )
	{
		for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
		{
			for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
			{
				delete m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx];
				delete m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx];
			}
		}

		for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
		{
			delete [] m_ppppcRDSbacCoders  [ui][iDepth];
			delete [] m_ppppcBinCodersCABAC[ui][iDepth];
		}
		delete[] m_ppppcRDSbacCoders  [ui];
		delete[] m_ppppcBinCodersCABAC[ui];
	}

	delete[] m_ppppcRDSbacCoders;
	delete[] m_ppppcBinCodersCABAC;
	delete[] m_pcSbacCoders;
	delete[] m_pcBinCoderCABACs;
	delete[] m_pcRDGoOnSbacCoders;  
	delete[] m_pcRDGoOnBinCodersCABAC;
	delete[] m_pcBitCounters;
	delete[] m_pcRdCosts;
#endif
	// destroy ROM
	destroyROM();

	return;
}

Void TEncTop::init(Bool isFieldCoding)
{
  // initialize SPS
  xInitSPS();
  
  /* set the VPS profile information */
  *m_cVPS.getPTL() = *m_cSPS.getPTL();
  m_cVPS.getTimingInfo()->setTimingInfoPresentFlag       ( false );
  // initialize PPS
  m_cPPS.setSPS(&m_cSPS);
  xInitPPS();
  xInitRPS(isFieldCoding);

  xInitPPSforTiles();
  
#if ETRI_MultiplePPS
  if (em_NumAdditionalPPS > 0){
	  ETRI_xInitPPS(&em_cPPS_id1, 1);
	  ETRI_xInitPPSforTiles(&em_cPPS_id1, &em_pMultipleTile[0]);
  }
  if (em_NumAdditionalPPS > 1){
	  ETRI_xInitPPS(&em_cPPS_id2, 2);
	  ETRI_xInitPPSforTiles(&em_cPPS_id2, &em_pMultipleTile[1]); 
  }
#endif

  // initialize processing unit classes
  m_cGOPEncoder.  init( this );
#if !ETRI_MULTITHREAD_2
  m_cSliceEncoder.init( this );
  m_cCuEncoder.   init( this );
  
  // initialize transform & quantization class
  m_pcCavlcCoder = getCavlcCoder();
  
  m_cTrQuant.init( 1 << m_uiQuadtreeTULog2MaxSize,
                  m_useRDOQ, 
                  m_useRDOQTS,
                  true 
                  ,m_useTransformSkipFast
#if ADAPTIVE_QP_SELECTION                  
                  , m_bUseAdaptQpSelect
#endif
                  );

  // initialize encoder search class
  m_cSearch.init( this, &m_cTrQuant, m_iSearchRange, m_bipredSearchRange, m_iFastSearch, 0, &m_cEntropyCoder, &m_cRdCost, getRDSbacCoder(), getRDGoOnSbacCoder() );
#endif

  m_iMaxRefPicNum = 0;

//==========================================================================
//	ETRI Class/Functions Initilization (Multithread) 
//	@Author : Jinwuk Seok  @ 2015 5 19 
//==========================================================================

  ETRI_TEncTopInit(ETRI_MODIFICATION_V00);

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncTop::deletePicBuffer()
{
  TComList<TComPic*>::iterator iterPic = m_cListPic.begin();
  Int iSize = Int( m_cListPic.size() );
  
  for ( Int i = 0; i < iSize; i++ )
  {
    TComPic* pcPic = *(iterPic++);
    
    pcPic->destroy();
    delete pcPic;
    pcPic = NULL;
  }
}

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  rcListBitstreamOut  list of output bitstreams
 \retval  iNumEncoded         number of encoded pictures
 */
# if !ETRI_MULTITHREAD_2
Void TEncTop::encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
  if (pcPicYuvOrg) {
    // get original YUV
    TComPic* pcPicCurr = NULL;
    xGetNewPicBuffer( pcPicCurr );
    pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );

    // compute image characteristics
    if ( getUseAdaptiveQP() )
    {
      m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
    }
  }
  
  if (!m_iNumPicRcvd || (!flush && m_iPOCLast != 0 && m_iNumPicRcvd != m_iGOPSize && m_iGOPSize))
  {
    iNumEncoded = 0;
    return;
  }
  
  if ( m_RCEnableRateControl )
  {
#if ETRI_DLL_INTERFACE
#if !KAIST_RC
#if 0
	  if (m_iPOCLast > 0)
		  m_cRateCtrl.ETRI_setRCRestart(false, 0);
	  else
		  m_cRateCtrl.ETRI_setRCRestart(true, m_framesToBeEncoded);			
#endif
#endif
#endif
//    m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
  }

#if ETRI_MODIFICATION_V00
  // compress GOP
	m_cGOPEncoder.ETRI_compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false);
#else
	m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false);
#endif

//   if ( m_RCEnableRateControl )
//   {
//     m_cRateCtrl.destroyRCGOP();
//   }
  
  iNumEncoded         = m_iNumPicRcvd;
  m_iNumPicRcvd       = 0;
  m_uiNumAllPicCoded += iNumEncoded;
}
#endif
/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
void separateFields(Pel* org, Pel* dstField, UInt stride, UInt width, UInt height, bool isTop)
{
  if (!isTop)
  {
    org += stride;
  }
  for (Int y = 0; y < height>>1; y++)
  {
    for (Int x = 0; x < width; x++)
    {
      dstField[x] = org[x];
    }
    
    dstField += stride;
    org += stride*2;
  }
  
}
#if !ETRI_MULTITHREAD_2 // gplusplus
Void TEncTop::encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, bool isTff)
{
  /* -- TOP FIELD -- */
  
  if (pcPicYuvOrg)
  {
    /* -- Top field initialization -- */
    TComPic *pcTopField;
    xGetNewPicBuffer( pcTopField );
    pcTopField->setReconMark (false);
    
    pcTopField->getSlice(0)->setPOC( m_iPOCLast );
    pcTopField->getPicYuvRec()->setBorderExtension(false);
    pcTopField->setTopField(isTff);
    
    int nHeight 	= pcPicYuvOrg->getHeight();
    int nWidth 		= pcPicYuvOrg->getWidth();
    int nStride 	= pcPicYuvOrg->getStride();
    int nPadLuma 	= pcPicYuvOrg->getLumaMargin();
    int nPadChroma	= pcPicYuvOrg->getChromaMargin();
    
    // Get pointers
    Pel * PicBufY 	= pcPicYuvOrg->getBufY();
    Pel * PicBufU 	= pcPicYuvOrg->getBufU();
    Pel * PicBufV 	= pcPicYuvOrg->getBufV();
    
    Pel * pcTopFieldY =  pcTopField->getPicYuvOrg()->getLumaAddr();
    Pel * pcTopFieldU =  pcTopField->getPicYuvOrg()->getCbAddr();
    Pel * pcTopFieldV =  pcTopField->getPicYuvOrg()->getCrAddr();
    
    /* -- Defield -- */
    
    bool isTop = isTff;
    
    separateFields(PicBufY + nPadLuma + nStride*nPadLuma, pcTopFieldY, nStride, nWidth, nHeight, isTop);
    separateFields(PicBufU + nPadChroma + (nStride >> 1)*nPadChroma, pcTopFieldU, nStride >> 1, nWidth >> 1, nHeight >> 1, isTop);
    separateFields(PicBufV + nPadChroma + (nStride >> 1)*nPadChroma, pcTopFieldV, nStride >> 1, nWidth >> 1, nHeight >> 1, isTop);
    
    // compute image characteristics
    if ( getUseAdaptiveQP() )
    {
      m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcTopField ) );
    }    
  }
  
  if (m_iPOCLast == 0) // compress field 0
  {
#if ETRI_MODIFICATION_V00
	m_cGOPEncoder.ETRI_compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, true, isTff);
#else
    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, true, isTff);
#endif
  }
  
  /* -- BOTTOM FIELD -- */
  
  if (pcPicYuvOrg)
  {
    
    /* -- Bottom field initialization -- */
    
    TComPic* pcBottomField;
    xGetNewPicBuffer( pcBottomField );
    pcBottomField->setReconMark (false);
    
    TComPicYuv* rpcPicYuvRec;
    if ( rcListPicYuvRecOut.size() == (UInt)m_iGOPSize )
    {
      rpcPicYuvRec = rcListPicYuvRecOut.popFront();
    }
    else
    {
      rpcPicYuvRec = new TComPicYuv;
      rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
    }
    rcListPicYuvRecOut.pushBack( rpcPicYuvRec );
    
    pcBottomField->getSlice(0)->setPOC( m_iPOCLast);
    pcBottomField->getPicYuvRec()->setBorderExtension(false);
    pcBottomField->setTopField(!isTff);
    
    int nHeight 	= pcPicYuvOrg->getHeight();
    int nWidth 		= pcPicYuvOrg->getWidth();
    int nStride 	= pcPicYuvOrg->getStride();
    int nPadLuma 	= pcPicYuvOrg->getLumaMargin();
    int nPadChroma	= pcPicYuvOrg->getChromaMargin();
    
    // Get pointers
    Pel * PicBufY 	= pcPicYuvOrg->getBufY();
    Pel * PicBufU 	= pcPicYuvOrg->getBufU();
    Pel * PicBufV 	= pcPicYuvOrg->getBufV();
    
    Pel * pcBottomFieldY =  pcBottomField->getPicYuvOrg()->getLumaAddr();
    Pel * pcBottomFieldU =  pcBottomField->getPicYuvOrg()->getCbAddr();
    Pel * pcBottomFieldV =  pcBottomField->getPicYuvOrg()->getCrAddr();
    
    /* -- Defield -- */
    
    bool isTop = !isTff;
    
    separateFields(PicBufY + nPadLuma + nStride*nPadLuma, pcBottomFieldY, nStride, nWidth, nHeight, isTop);
    separateFields(PicBufU + nPadChroma + (nStride >> 1)*nPadChroma, pcBottomFieldU, nStride >> 1, nWidth >> 1, nHeight >> 1, isTop);
    separateFields(PicBufV + nPadChroma + (nStride >> 1)*nPadChroma, pcBottomFieldV, nStride >> 1, nWidth >> 1, nHeight >> 1, isTop);
    
    // Compute image characteristics
    if ( getUseAdaptiveQP() )
    {
      m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcBottomField ) );
    }    
  }
  
  if ( ( !(m_iNumPicRcvd) || (!flush && m_iPOCLast != 1 && m_iNumPicRcvd != m_iGOPSize && m_iGOPSize)) )
  {
    iNumEncoded = 0;
    return;
  }

  #if ETRI_MODIFICATION_V00
  // compress GOP
  m_cGOPEncoder.ETRI_compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, true, isTff);
  #else
  m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, true, isTff);
  #endif
  
  iNumEncoded = m_iNumPicRcvd;
  m_iNumPicRcvd = 0;
  m_uiNumAllPicCoded += iNumEncoded;
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
Void TEncTop::xGetNewPicBuffer ( TComPic*& rpcPic )
{
  TComSlice::sortPicList(m_cListPic);
  
  if (m_cListPic.size() >= (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2) )
  {
    TComList<TComPic*>::iterator iterPic  = m_cListPic.begin();
    Int iSize = Int( m_cListPic.size() );
    for ( Int i = 0; i < iSize; i++ )
    {
      rpcPic = *(iterPic++);
      if(rpcPic->getSlice(0)->isReferenced() == false)
      {
        break;
      }
    }
  }
  else
  {
    if ( getUseAdaptiveQP() )
    {
      TEncPic* pcEPic = new TEncPic;
      pcEPic->create( m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_cPPS.getMaxCuDQPDepth()+1 ,
                      m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
      rpcPic = pcEPic;
    }
    else
    {
#if ETRI_DLL_INTERFACE	//zeroone 20140725  2013 11 1 by Seok : TEncTop::xGetNewPicBuffer
		if (em_bAnalyserClear)
		{
			rpcPic = em_cListPic.front();
			em_cListPic.pop_front();
		}
		else
		{
			rpcPic = new TComPic;
			rpcPic->create( m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
				m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
		}
#else
		rpcPic = new TComPic;
		rpcPic->create( m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
			m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
#endif
    }
    m_cListPic.pushBack( rpcPic );
  }
  rpcPic->setReconMark (false);
  
  m_iPOCLast++;
  m_iNumPicRcvd++;
  
  rpcPic->getSlice(0)->setPOC( m_iPOCLast );
  // mark it should be extended
  rpcPic->getPicYuvRec()->setBorderExtension(false);
}

Void TEncTop::xInitSPS()
{
  ProfileTierLevel& profileTierLevel = *m_cSPS.getPTL()->getGeneralPTL();
  profileTierLevel.setLevelIdc(m_level);
  profileTierLevel.setTierFlag(m_levelTier);
  profileTierLevel.setProfileIdc(m_profile);
  profileTierLevel.setProfileCompatibilityFlag(m_profile, 1);
  profileTierLevel.setProgressiveSourceFlag(m_progressiveSourceFlag);
  profileTierLevel.setInterlacedSourceFlag(m_interlacedSourceFlag);
  profileTierLevel.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
  profileTierLevel.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);
  
  if (m_profile == Profile::MAIN10 && g_bitDepthY == 8 && g_bitDepthC == 8)
  {
    /* The above constraint is equal to Profile::MAIN */
    profileTierLevel.setProfileCompatibilityFlag(Profile::MAIN, 1);
  }
  if (m_profile == Profile::MAIN)
  {
    /* A Profile::MAIN10 decoder can always decode Profile::MAIN */
    profileTierLevel.setProfileCompatibilityFlag(Profile::MAIN10, 1);
  }
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */

  m_cSPS.setPicWidthInLumaSamples         ( m_iSourceWidth      );
  m_cSPS.setPicHeightInLumaSamples        ( m_iSourceHeight     );
  m_cSPS.setConformanceWindow             ( m_conformanceWindow );
  m_cSPS.setMaxCUWidth    ( g_uiMaxCUWidth      );
  m_cSPS.setMaxCUHeight   ( g_uiMaxCUHeight     );
  m_cSPS.setMaxCUDepth    ( g_uiMaxCUDepth      );

  Int minCUSize = m_cSPS.getMaxCUWidth() >> ( m_cSPS.getMaxCUDepth()-g_uiAddCUDepth );
  Int log2MinCUSize = 0;
  while(minCUSize > 1)
  {
    minCUSize >>= 1;
    log2MinCUSize++;
  }

  m_cSPS.setLog2MinCodingBlockSize(log2MinCUSize);
  m_cSPS.setLog2DiffMaxMinCodingBlockSize(m_cSPS.getMaxCUDepth()-g_uiAddCUDepth);
  
  m_cSPS.setPCMLog2MinSize (m_uiPCMLog2MinSize);
  m_cSPS.setUsePCM        ( m_usePCM           );
  m_cSPS.setPCMLog2MaxSize( m_pcmLog2MaxSize  );

  m_cSPS.setQuadtreeTULog2MaxSize( m_uiQuadtreeTULog2MaxSize );
  m_cSPS.setQuadtreeTULog2MinSize( m_uiQuadtreeTULog2MinSize );
  m_cSPS.setQuadtreeTUMaxDepthInter( m_uiQuadtreeTUMaxDepthInter    );
  m_cSPS.setQuadtreeTUMaxDepthIntra( m_uiQuadtreeTUMaxDepthIntra    );
  
  m_cSPS.setTMVPFlagsPresent(false);

  m_cSPS.setMaxTrSize   ( 1 << m_uiQuadtreeTULog2MaxSize );
  
  Int i;
  
  for (i = 0; i < g_uiMaxCUDepth-g_uiAddCUDepth; i++ )
  {
    m_cSPS.setAMPAcc( i, m_useAMP );
    //m_cSPS.setAMPAcc( i, 1 );
  }

  m_cSPS.setUseAMP ( m_useAMP );

  for (i = g_uiMaxCUDepth-g_uiAddCUDepth; i < g_uiMaxCUDepth; i++ )
  {
    m_cSPS.setAMPAcc(i, 0);
  }

  m_cSPS.setBitDepthY( g_bitDepthY );
  m_cSPS.setBitDepthC( g_bitDepthC );

  m_cSPS.setQpBDOffsetY ( 6*(g_bitDepthY - 8) );
  m_cSPS.setQpBDOffsetC ( 6*(g_bitDepthC - 8) );

  m_cSPS.setUseSAO( m_bUseSAO );

  m_cSPS.setMaxTLayers( m_maxTempLayer );
  m_cSPS.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );
  for ( i = 0; i < min(m_cSPS.getMaxTLayers(),(UInt) MAX_TLAYER); i++ )
  {
    m_cSPS.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
    m_cSPS.setNumReorderPics(m_numReorderPics[i], i);
  }
  m_cSPS.setPCMBitDepthLuma (g_uiPCMBitDepthLuma);
  m_cSPS.setPCMBitDepthChroma (g_uiPCMBitDepthChroma);
  m_cSPS.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag );

  m_cSPS.setScalingListFlag ( (m_useScalingListId == 0) ? 0 : 1 );

  m_cSPS.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );

  m_cSPS.setVuiParametersPresentFlag(getVuiParametersPresentFlag());
  if (m_cSPS.getVuiParametersPresentFlag())
  {
    TComVUI* pcVUI = m_cSPS.getVuiParameters();
    pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
    pcVUI->setAspectRatioIdc(getAspectRatioIdc());
    pcVUI->setSarWidth(getSarWidth());
    pcVUI->setSarHeight(getSarHeight());
    pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
    pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
    pcVUI->setVideoSignalTypePresentFlag(getVideoSignalTypePresentFlag());
    pcVUI->setVideoFormat(getVideoFormat());
    pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
    pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
    pcVUI->setColourPrimaries(getColourPrimaries());
    pcVUI->setTransferCharacteristics(getTransferCharacteristics());
    pcVUI->setMatrixCoefficients(getMatrixCoefficients());
    pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
    pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
    pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
    pcVUI->setNeutralChromaIndicationFlag(getNeutralChromaIndicationFlag());
    pcVUI->setDefaultDisplayWindow(getDefaultDisplayWindow());
    pcVUI->setFrameFieldInfoPresentFlag(getFrameFieldInfoPresentFlag());
    pcVUI->setFieldSeqFlag(false);
    pcVUI->setHrdParametersPresentFlag(false);
    pcVUI->getTimingInfo()->setPocProportionalToTimingFlag(getPocProportionalToTimingFlag());
    pcVUI->getTimingInfo()->setNumTicksPocDiffOneMinus1   (getNumTicksPocDiffOneMinus1()   );
    pcVUI->setBitstreamRestrictionFlag(getBitstreamRestrictionFlag());
    pcVUI->setTilesFixedStructureFlag(getTilesFixedStructureFlag());
    pcVUI->setMotionVectorsOverPicBoundariesFlag(getMotionVectorsOverPicBoundariesFlag());
    pcVUI->setMinSpatialSegmentationIdc(getMinSpatialSegmentationIdc());
    pcVUI->setMaxBytesPerPicDenom(getMaxBytesPerPicDenom());
    pcVUI->setMaxBitsPerMinCuDenom(getMaxBitsPerMinCuDenom());
    pcVUI->setLog2MaxMvLengthHorizontal(getLog2MaxMvLengthHorizontal());
    pcVUI->setLog2MaxMvLengthVertical(getLog2MaxMvLengthVertical());
  }
}

Void TEncTop::xInitPPS()
{
  m_cPPS.setConstrainedIntraPred( m_bUseConstrainedIntraPred );
  Bool bUseDQP = (getMaxCuDQPDepth() > 0)? true : false;

  if((getMaxDeltaQP() != 0 )|| getUseAdaptiveQP())
  {
    bUseDQP = true;
  }

  if(bUseDQP)
  {
    m_cPPS.setUseDQP(true);
    m_cPPS.setMaxCuDQPDepth( m_iMaxCuDQPDepth );
    m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
  }
  else
  {
    m_cPPS.setUseDQP(false);
    m_cPPS.setMaxCuDQPDepth( 0 );
    m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
  }

  if ( m_RCEnableRateControl )
  {
    m_cPPS.setUseDQP(true);
    m_cPPS.setMaxCuDQPDepth( 0 );
    m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
  } 

  m_cPPS.setChromaCbQpOffset( m_chromaCbQpOffset );
  m_cPPS.setChromaCrQpOffset( m_chromaCrQpOffset );

  m_cPPS.setNumSubstreams(m_iWaveFrontSubstreams);
  m_cPPS.setEntropyCodingSyncEnabledFlag( m_iWaveFrontSynchro > 0 );
  m_cPPS.setTilesEnabledFlag( (m_iNumColumnsMinus1 > 0 || m_iNumRowsMinus1 > 0) );
  m_cPPS.setUseWP( m_useWeightedPred );
  m_cPPS.setWPBiPred( m_useWeightedBiPred );
  m_cPPS.setOutputFlagPresentFlag( false );
  m_cPPS.setSignHideFlag(getSignHideFlag());
  if ( getDeblockingFilterMetric() )
  {
    m_cPPS.setDeblockingFilterControlPresentFlag (true);
    m_cPPS.setDeblockingFilterOverrideEnabledFlag(true);
    m_cPPS.setPicDisableDeblockingFilterFlag(false);
    m_cPPS.setDeblockingFilterBetaOffsetDiv2(0);
    m_cPPS.setDeblockingFilterTcOffsetDiv2(0);
  } 
  else
  {
    m_cPPS.setDeblockingFilterControlPresentFlag (m_DeblockingFilterControlPresent );
  }
  m_cPPS.setLog2ParallelMergeLevelMinus2   (m_log2ParallelMergeLevelMinus2 );
  m_cPPS.setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
  m_cPPS.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );
  Int histogram[MAX_NUM_REF + 1];
  for( Int i = 0; i <= MAX_NUM_REF; i++ )
  {
    histogram[i]=0;
  }
  for( Int i = 0; i < getGOPSize(); i++ )
  {
    assert(getGOPEntry(i).m_numRefPicsActive >= 0 && getGOPEntry(i).m_numRefPicsActive <= MAX_NUM_REF);
    histogram[getGOPEntry(i).m_numRefPicsActive]++;
  }
  Int maxHist=-1;
  Int bestPos=0;
  for( Int i = 0; i <= MAX_NUM_REF; i++ )
  {
    if(histogram[i]>maxHist)
    {
      maxHist=histogram[i];
      bestPos=i;
    }
  }
  assert(bestPos <= 15);
  m_cPPS.setNumRefIdxL0DefaultActive(bestPos);
  m_cPPS.setNumRefIdxL1DefaultActive(bestPos);
  m_cPPS.setTransquantBypassEnableFlag(getTransquantBypassEnableFlag());
  m_cPPS.setUseTransformSkip( m_useTransformSkip );
  if (m_sliceSegmentMode)
  {
    m_cPPS.setDependentSliceSegmentsEnabledFlag( true );
  }
#if !ETRI_MULTITHREAD_2
  if( m_cPPS.getDependentSliceSegmentsEnabledFlag() )
  {
    Int NumCtx = m_cPPS.getEntropyCodingSyncEnabledFlag()?2:1;
    m_cSliceEncoder.initCtxMem( NumCtx );
    for ( UInt st = 0; st < NumCtx; st++ )
    {
      TEncSbac* ctx = NULL;
      ctx = new TEncSbac;
      ctx->init( &m_cBinCoderCABAC );
      m_cSliceEncoder.setCtxMem( ctx, st );
    }
  }
#endif  
}

#if ETRI_MultiplePPS
Void TEncTop::ETRI_xInitPPS(TComPPS* pPPS, Int ppsid)
{
	pPPS->setPPSId(ppsid);
	pPPS->setSPS(&m_cSPS);
	
	//same as to the orgi.
	pPPS->setConstrainedIntraPred(m_bUseConstrainedIntraPred);
	Bool bUseDQP = (getMaxCuDQPDepth() > 0) ? true : false;

	if ((getMaxDeltaQP() != 0) || getUseAdaptiveQP())
	{
		bUseDQP = true;
	}

	if (bUseDQP)
	{
		pPPS->setUseDQP(true);
		pPPS->setMaxCuDQPDepth(m_iMaxCuDQPDepth);
		pPPS->setMinCuDQPSize(pPPS->getSPS()->getMaxCUWidth() >> (pPPS->getMaxCuDQPDepth()));
	}
	else
	{
		pPPS->setUseDQP(false);
		pPPS->setMaxCuDQPDepth(0);
		pPPS->setMinCuDQPSize(pPPS->getSPS()->getMaxCUWidth() >> (pPPS->getMaxCuDQPDepth()));
	}

	if (m_RCEnableRateControl)
	{
		pPPS->setUseDQP(true);
		pPPS->setMaxCuDQPDepth(0);
		pPPS->setMinCuDQPSize(pPPS->getSPS()->getMaxCUWidth() >> (pPPS->getMaxCuDQPDepth()));
	}

	pPPS->setChromaCbQpOffset(m_chromaCbQpOffset);
	pPPS->setChromaCrQpOffset(m_chromaCrQpOffset);

	pPPS->setNumSubstreams(m_iWaveFrontSubstreams);
	pPPS->setEntropyCodingSyncEnabledFlag(m_iWaveFrontSynchro > 0);
	
	//pPPS->setTilesEnabledFlag((m_iNumColumnsMinus1 > 0 || m_iNumRowsMinus1 > 0));
	//tileEnableFlag
	if (ppsid > 0)
		pPPS->setTilesEnabledFlag((em_pMultipleTile[ppsid - 1].numTileColumnsMinus1 || em_pMultipleTile[ppsid - 1].numTileRowsMinus1));

	pPPS->setUseWP(m_useWeightedPred);
	pPPS->setWPBiPred(m_useWeightedBiPred);
	pPPS->setOutputFlagPresentFlag(false);
	pPPS->setSignHideFlag(getSignHideFlag());
	if (getDeblockingFilterMetric())
	{
		pPPS->setDeblockingFilterControlPresentFlag(true);
		pPPS->setDeblockingFilterOverrideEnabledFlag(true);
		pPPS->setPicDisableDeblockingFilterFlag(false);
		pPPS->setDeblockingFilterBetaOffsetDiv2(0);
		pPPS->setDeblockingFilterTcOffsetDiv2(0);
	} 
	else
	{
		pPPS->setDeblockingFilterControlPresentFlag (m_DeblockingFilterControlPresent );
	}
	pPPS->setLog2ParallelMergeLevelMinus2(m_log2ParallelMergeLevelMinus2);
	pPPS->setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
	pPPS->setLoopFilterAcrossSlicesEnabledFlag(m_bLFCrossSliceBoundaryFlag);
	Int histogram[MAX_NUM_REF + 1];
	for (Int i = 0; i <= MAX_NUM_REF; i++)
	{
		histogram[i] = 0;
	}
	for (Int i = 0; i < getGOPSize(); i++)
	{
		assert(getGOPEntry(i).m_numRefPicsActive >= 0 && getGOPEntry(i).m_numRefPicsActive <= MAX_NUM_REF);
		histogram[getGOPEntry(i).m_numRefPicsActive]++;
	}
	Int maxHist=-1;
	Int bestPos=0;
	for( Int i = 0; i <= MAX_NUM_REF; i++ )
	{
		if(histogram[i]>maxHist)
		{
			maxHist=histogram[i];
			bestPos=i;
		}
	}
	assert(bestPos <= 15);
	pPPS->setNumRefIdxL0DefaultActive(bestPos);
	pPPS->setNumRefIdxL1DefaultActive(bestPos);
	pPPS->setTransquantBypassEnableFlag(getTransquantBypassEnableFlag());
	pPPS->setUseTransformSkip(m_useTransformSkip);
	if (m_sliceSegmentMode)
	{
		pPPS->setDependentSliceSegmentsEnabledFlag(true);
	}
#if !ETRI_MULTITHREAD_2
	if (pPPS->getDependentSliceSegmentsEnabledFlag())
	{
		Int NumCtx = pPPS->getEntropyCodingSyncEnabledFlag() ? 2 : 1;
		m_cSliceEncoder.initCtxMem(NumCtx);
		for (UInt st = 0; st < NumCtx; st++)
		{
			TEncSbac* ctx = NULL;
			ctx = new TEncSbac;
			ctx->init(&m_cBinCoderCABAC);
			m_cSliceEncoder.setCtxMem(ctx, st);
		}
	}
#endif  
}
#endif
//Function for initializing m_RPSList, a list of TComReferencePictureSet, based on the GOPEntry objects read from the config file.
Void TEncTop::xInitRPS(Bool isFieldCoding)
{
  TComReferencePictureSet*      rps;
  
  m_cSPS.createRPSList(getGOPSize()+m_extraRPSs+1);
  TComRPSList* rpsList = m_cSPS.getRPSList();

  for( Int i = 0; i < getGOPSize()+m_extraRPSs; i++) 
  {
    GOPEntry ge = getGOPEntry(i);
    rps = rpsList->getReferencePictureSet(i);
    rps->setNumberOfPictures(ge.m_numRefPics);
    rps->setNumRefIdc(ge.m_numRefIdc);
    Int numNeg = 0;
    Int numPos = 0;
    for( Int j = 0; j < ge.m_numRefPics; j++)
    {
      rps->setDeltaPOC(j,ge.m_referencePics[j]);
      rps->setUsed(j,ge.m_usedByCurrPic[j]);
      if(ge.m_referencePics[j]>0)
      {
        numPos++;
      }
      else
      {
        numNeg++;
      }
    }
    rps->setNumberOfNegativePictures(numNeg);
    rps->setNumberOfPositivePictures(numPos);

    // handle inter RPS intialization from the config file.
#if AUTO_INTER_RPS
    rps->setInterRPSPrediction(ge.m_interRPSPrediction > 0);  // not very clean, converting anything > 0 to true.
    rps->setDeltaRIdxMinus1(0);                               // index to the Reference RPS is always the previous one.
    TComReferencePictureSet*     RPSRef = rpsList->getReferencePictureSet(i-1);  // get the reference RPS

    if (ge.m_interRPSPrediction == 2)  // Automatic generation of the inter RPS idc based on the RIdx provided.
    {
      Int deltaRPS = getGOPEntry(i-1).m_POC - ge.m_POC;  // the ref POC - current POC
      Int numRefDeltaPOC = RPSRef->getNumberOfPictures();

      rps->setDeltaRPS(deltaRPS);           // set delta RPS
      rps->setNumRefIdc(numRefDeltaPOC+1);  // set the numRefIdc to the number of pictures in the reference RPS + 1.
      Int count=0;
      for (Int j = 0; j <= numRefDeltaPOC; j++ ) // cycle through pics in reference RPS.
      {
        Int RefDeltaPOC = (j<numRefDeltaPOC)? RPSRef->getDeltaPOC(j): 0;  // if it is the last decoded picture, set RefDeltaPOC = 0
        rps->setRefIdc(j, 0);
        for (Int k = 0; k < rps->getNumberOfPictures(); k++ )  // cycle through pics in current RPS.
        {
          if (rps->getDeltaPOC(k) == ( RefDeltaPOC + deltaRPS))  // if the current RPS has a same picture as the reference RPS. 
          {
              rps->setRefIdc(j, (rps->getUsed(k)?1:2));
              count++;
              break;
          }
        }
      }
      if (count != rps->getNumberOfPictures())
      {
        printf("Warning: Unable fully predict all delta POCs using the reference RPS index given in the config file.  Setting Inter RPS to false for this RPS.\n");
        rps->setInterRPSPrediction(0);
      }
    }
    else if (ge.m_interRPSPrediction == 1)  // inter RPS idc based on the RefIdc values provided in config file.
    {
      rps->setDeltaRPS(ge.m_deltaRPS);
      rps->setNumRefIdc(ge.m_numRefIdc);
      for (Int j = 0; j < ge.m_numRefIdc; j++ )
      {
        rps->setRefIdc(j, ge.m_refIdc[j]);
      }
#if WRITE_BACK
      // the folowing code overwrite the deltaPOC and Used by current values read from the config file with the ones
      // computed from the RefIdc.  A warning is printed if they are not identical.
      numNeg = 0;
      numPos = 0;
      TComReferencePictureSet      RPSTemp;  // temporary variable

      for (Int j = 0; j < ge.m_numRefIdc; j++ )
      {
        if (ge.m_refIdc[j])
        {
          Int deltaPOC = ge.m_deltaRPS + ((j < RPSRef->getNumberOfPictures())? RPSRef->getDeltaPOC(j) : 0);
          RPSTemp.setDeltaPOC((numNeg+numPos),deltaPOC);
          RPSTemp.setUsed((numNeg+numPos),ge.m_refIdc[j]==1?1:0);
          if (deltaPOC<0)
          {
            numNeg++;
          }
          else
          {
            numPos++;
          }
        }
      }
      if (numNeg != rps->getNumberOfNegativePictures())
      {
        printf("Warning: number of negative pictures in RPS is different between intra and inter RPS specified in the config file.\n");
        rps->setNumberOfNegativePictures(numNeg);
        rps->setNumberOfPictures(numNeg+numPos);
      }
      if (numPos != rps->getNumberOfPositivePictures())
      {
        printf("Warning: number of positive pictures in RPS is different between intra and inter RPS specified in the config file.\n");
        rps->setNumberOfPositivePictures(numPos);
        rps->setNumberOfPictures(numNeg+numPos);
      }
      RPSTemp.setNumberOfPictures(numNeg+numPos);
      RPSTemp.setNumberOfNegativePictures(numNeg);
      RPSTemp.sortDeltaPOC();     // sort the created delta POC before comparing
      // check if Delta POC and Used are the same 
      // print warning if they are not.
      for (Int j = 0; j < ge.m_numRefIdc; j++ )
      {
        if (RPSTemp.getDeltaPOC(j) != rps->getDeltaPOC(j))
        {
          printf("Warning: delta POC is different between intra RPS and inter RPS specified in the config file.\n");
          rps->setDeltaPOC(j,RPSTemp.getDeltaPOC(j));
        }
        if (RPSTemp.getUsed(j) != rps->getUsed(j))
        {
          printf("Warning: Used by Current in RPS is different between intra and inter RPS specified in the config file.\n");
          rps->setUsed(j,RPSTemp.getUsed(j));
        }
      }
#endif
    }
#else
    rps->setInterRPSPrediction(ge.m_interRPSPrediction);
    if (ge.m_interRPSPrediction)
    {
      rps->setDeltaRIdxMinus1(0);
      rps->setDeltaRPS(ge.m_deltaRPS);
      rps->setNumRefIdc(ge.m_numRefIdc);
      for (Int j = 0; j < ge.m_numRefIdc; j++ )
      {
        rps->setRefIdc(j, ge.m_refIdc[j]);
      }
#if WRITE_BACK
      // the folowing code overwrite the deltaPOC and Used by current values read from the config file with the ones
      // computed from the RefIdc.  This is not necessary if both are identical. Currently there is no check to see if they are identical.
      numNeg = 0;
      numPos = 0;
      TComReferencePictureSet*     RPSRef = m_RPSList.getReferencePictureSet(i-1);

      for (Int j = 0; j < ge.m_numRefIdc; j++ )
      {
        if (ge.m_refIdc[j])
        {
          Int deltaPOC = ge.m_deltaRPS + ((j < RPSRef->getNumberOfPictures())? RPSRef->getDeltaPOC(j) : 0);
          rps->setDeltaPOC((numNeg+numPos),deltaPOC);
          rps->setUsed((numNeg+numPos),ge.m_refIdc[j]==1?1:0);
          if (deltaPOC<0)
          {
            numNeg++;
          }
          else
          {
            numPos++;
          }
        }
      }
      rps->setNumberOfNegativePictures(numNeg);
      rps->setNumberOfPositivePictures(numPos);
      rps->sortDeltaPOC();
#endif
    }
#endif //INTER_RPS_AUTO
  }
  //In case of field coding, we need to set special parameters for the first bottom field of the sequence, since it is not specified in the cfg file. 
  //The position = GOPSize + extraRPSs which is (a priori) unused is reserved for this field in the RPS. 
  if (isFieldCoding) 
  {
    rps = rpsList->getReferencePictureSet(getGOPSize()+m_extraRPSs);
    rps->setNumberOfPictures(1);
    rps->setNumberOfNegativePictures(1);
    rps->setNumberOfPositivePictures(0);
    rps->setNumberOfLongtermPictures(0);
    rps->setDeltaPOC(0,-1);
    rps->setPOC(0,0);
    rps->setUsed(0,true);
    rps->setInterRPSPrediction(false);
    rps->setDeltaRIdxMinus1(0);
    rps->setDeltaRPS(0);
    rps->setNumRefIdc(0);
}
}

   // This is a function that 
   // determines what Reference Picture Set to use 
   // for a specific slice (with POC = POCCurr)
Void TEncTop::selectReferencePictureSet(TComSlice* slice, Int POCCurr, Int GOPid )
{
  slice->setRPSidx(GOPid);
  for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
  {    
    if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      Int POCIndex = POCCurr%m_uiIntraPeriod;
      if(POCIndex == 0)
      {
        POCIndex = m_uiIntraPeriod;
      }
      if(POCIndex == m_GOPList[extraNum].m_POC)
      {
        slice->setRPSidx(extraNum);
      }
    }
    else
    {
      if(POCCurr==m_GOPList[extraNum].m_POC)
      {
        slice->setRPSidx(extraNum);
      }
    }
  }

  if(POCCurr == 1 && slice->getPic()->isField())
  {
    slice->setRPSidx(m_iGOPSize+m_extraRPSs);
  }

  slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(slice->getRPSidx()));
  slice->getRPS()->setNumberOfPictures(slice->getRPS()->getNumberOfNegativePictures()+slice->getRPS()->getNumberOfPositivePictures());
}

#if ETRI_MultiplePPS
TComPPS*  TEncTop::getPPS(Int ippsid)
{
	switch (ippsid)
	{
	case 0:
		return &m_cPPS;
		break;
	case 1:
		return &em_cPPS_id1;
		break;
	case 2:
		return &em_cPPS_id2;
		break;
	default:
		printf("ERROR: getPPS\n");
		return NULL;
		break;
	}
}
#endif

Int TEncTop::getReferencePictureSetIdxForSOP(TComSlice* slice, Int POCCurr, Int GOPid )
{
  int rpsIdx = GOPid;

  for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
  {    
    if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      Int POCIndex = POCCurr%m_uiIntraPeriod;
      if(POCIndex == 0)
      {
        POCIndex = m_uiIntraPeriod;
      }
      if(POCIndex == m_GOPList[extraNum].m_POC)
      {
        rpsIdx = extraNum;
      }
    }
    else
    {
      if(POCCurr==m_GOPList[extraNum].m_POC)
      {
        rpsIdx = extraNum;
      }
    }
  }

  return rpsIdx;
}

Void  TEncTop::xInitPPSforTiles()
{
  m_cPPS.setTileUniformSpacingFlag( m_tileUniformSpacingFlag );
  m_cPPS.setNumTileColumnsMinus1( m_iNumColumnsMinus1 );
  m_cPPS.setNumTileRowsMinus1( m_iNumRowsMinus1 );
  if( !m_tileUniformSpacingFlag )
  {
    m_cPPS.setTileColumnWidth( m_tileColumnWidth );
    m_cPPS.setTileRowHeight( m_tileRowHeight );
  }
  m_cPPS.setLoopFilterAcrossTilesEnabledFlag( m_loopFilterAcrossTilesEnabledFlag );

  // # substreams is "per tile" when tiles are independent.
  if (m_iWaveFrontSynchro )
  {
    m_cPPS.setNumSubstreams(m_iWaveFrontSubstreams * (m_iNumColumnsMinus1+1));
  }
}
#if ETRI_MultiplePPS
Void  TEncTop::ETRI_xInitPPSforTiles(TComPPS* pPPS, ETRI_PPSTile_t* pMultipleTile)
{
	pPPS->setTileUniformSpacingFlag(pMultipleTile->tileUniformSpacingFlag);
	pPPS->setNumTileColumnsMinus1(pMultipleTile -> numTileColumnsMinus1);
	pPPS->setNumTileRowsMinus1(pMultipleTile->numTileRowsMinus1);
	if (!pMultipleTile->tileUniformSpacingFlag)
	{
		pPPS->setTileColumnWidth(pMultipleTile->tileColumnWidth);
		pPPS->setTileRowHeight(pMultipleTile->tileRowHeight);
	}
	pPPS->setLoopFilterAcrossTilesEnabledFlag(pMultipleTile->loopFilterAcrossTilesEnabledFlag);

	// # substreams is "per tile" when tiles are independent.
	if (m_iWaveFrontSynchro)
	{
		pPPS->setNumSubstreams(m_iWaveFrontSubstreams * (pMultipleTile->numTileColumnsMinus1 + 1));
	}
}
#endif
Void  TEncCfg::xCheckGSParameters()
{
  Int   iWidthInCU = ( m_iSourceWidth%g_uiMaxCUWidth ) ? m_iSourceWidth/g_uiMaxCUWidth + 1 : m_iSourceWidth/g_uiMaxCUWidth;
  Int   iHeightInCU = ( m_iSourceHeight%g_uiMaxCUHeight ) ? m_iSourceHeight/g_uiMaxCUHeight + 1 : m_iSourceHeight/g_uiMaxCUHeight;
  UInt  uiCummulativeColumnWidth = 0;
  UInt  uiCummulativeRowHeight = 0;

  //check the column relative parameters
  if( m_iNumColumnsMinus1 >= (1<<(LOG2_MAX_NUM_COLUMNS_MINUS1+1)) )
  {
    printf( "The number of columns is larger than the maximum allowed number of columns.\n" );
    exit( EXIT_FAILURE );
  }

  if( m_iNumColumnsMinus1 >= iWidthInCU )
  {
    printf( "The current picture can not have so many columns.\n" );
    exit( EXIT_FAILURE );
  }

  if( m_iNumColumnsMinus1 && !m_tileUniformSpacingFlag )
  {
    for(Int i=0; i<m_iNumColumnsMinus1; i++)
    {
      uiCummulativeColumnWidth += m_tileColumnWidth[i];
    }

    if( uiCummulativeColumnWidth >= iWidthInCU )
    {
      printf( "The width of the column is too large.\n" );
      exit( EXIT_FAILURE );
    }
  }

  //check the row relative parameters
  if( m_iNumRowsMinus1 >= (1<<(LOG2_MAX_NUM_ROWS_MINUS1+1)) )
  {
    printf( "The number of rows is larger than the maximum allowed number of rows.\n" );
    exit( EXIT_FAILURE );
  }

  if( m_iNumRowsMinus1 >= iHeightInCU )
  {
    printf( "The current picture can not have so many rows.\n" );
    exit( EXIT_FAILURE );
  }

  if( m_iNumRowsMinus1 && !m_tileUniformSpacingFlag )
  {
    for(Int i=0; i<m_iNumRowsMinus1; i++)
      uiCummulativeRowHeight += m_tileRowHeight[i];

    if( uiCummulativeRowHeight >= iHeightInCU )
    {
      printf( "The height of the row is too large.\n" );
      exit( EXIT_FAILURE );
    }
  }
}

#if ETRI_MULTITHREAD_2
///////////////////////////////////////////////////////////////////
// gplusplus
Void TEncTop::ETRI_xSetNewPicBuffer ( TComList<TComPic*>* pcListPic, int nCount )
{
	TComPic* rpcPic;

//#if MAX_THREADPOOL_TOP
//	em_ppcPic = new TComPic*[nCount];
//
//	for (int i = 0; i < nCount; i++)
//	{
//		rpcPic = new TComPic;
//
//		em_ppcPic[i] = rpcPic;
//		pcListPic->pushBack(rpcPic);
//
//		//em_bThread[i] = -1;
//	}
//
//	em_bCreate = true;
//	for (int i = 0; i < nCount; i++)
//	{
//		// 여유 있는 스레드를 찾는다.
//		m_hThreadPoolTop.GetFreeThreadID();
//
//		m_hThreadPoolTop.AddThread(this, i, false);
//		//Sleep(1);  // gplusplus_151116
//	}
//
//	// 모든 스레드가 끝날때까지 기다린다.
//	m_hThreadPoolTop.GetAllFreeThreadWaiting();
//
//	delete[] em_ppcPic;
//#else
	for (int i = 0; i < nCount; i++)
	{
		rpcPic = new TComPic;

		rpcPic->create(m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
			m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
		rpcPic->getSlice(0)->setPOC(i);

		pcListPic->pushBack(rpcPic);
	}
//#endif
}

Void TEncTop::ETRI_deletePicBuffer(TComList<TComPic*>* pcListPic)
{
	TComList<TComPic*>::iterator iterPic = pcListPic->begin();
	Int iSize = Int(pcListPic->size());

//#if MAX_THREADPOOL_TOP
//	em_ppcPic = new TComPic*[iSize];
//
//	for (Int i = 0; i < iSize; i++)
//	{
//		em_ppcPic[i] = *(iterPic++);
//		//em_bThread[i] = -1;
//	}
//
//	//em_nThreadCnt = 0;
//	em_bCreate = false;
//
//	for (int i = 0; i < iSize; i++)
//	{
//		// 여유 있는 스레드를 찾는다.
//		m_hThreadPoolTop.GetFreeThreadID();
//
//		m_hThreadPoolTop.AddThread(this, i, false);
//		// Sleep(2); // gplusplus_151116
//	}
//
//	// 모든 스레드가 끝날때까지 기다린다.
//	m_hThreadPoolTop.GetAllFreeThreadWaiting();
//
//	delete[] em_ppcPic;
//#else
	for (Int i = 0; i < iSize; i++)
	{
		TComPic* pcPic = *(iterPic++);

		pcPic->destroy();
		delete pcPic;
		pcPic = NULL;
	}
//#endif
}
void TEncTop::threadTopProcessing(void *param, int num)
{
	TEncTop *pcTop = (TEncTop *)param;

	//pcTop->em_bThread[pcTop->em_nThreadCnt++] = num;

	if (pcTop->em_bCreate)
	{
		pcTop->em_ppcPic[num][0].create(pcTop->m_iSourceWidth, pcTop->m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
			pcTop->m_conformanceWindow, pcTop->m_defaultDisplayWindow, pcTop->m_numReorderPics);
		pcTop->em_ppcPic[num][0].getSlice(0)->setPOC(num);
	}
	else
	{
		if (pcTop->em_ppcPic[num])
		{
			pcTop->em_ppcPic[num][0].destroy();
			delete pcTop->em_ppcPic[num];
			pcTop->em_ppcPic[num] = NULL;
		}
		else
		{
			printf("NULL : num = %d\n", num);
		}
	}
	//pcTop->em
	//	*pcTop->em_ppcPic[num].create(pcTop->m_iSourceWidth, pcTop->m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
	//									pcTop->m_conformanceWindow, pcTop->m_defaultDisplayWindow, pcTop->m_numReorderPics);
}

#if ETRI_COPYTOPIC_MULTITHREAD

Void *TEncTop::copyToPicProc(void* Param)
{
	copyToPicInfo* copyInfo = (copyToPicInfo *)Param;
	copyInfo->pcComPicYuv->qrCopyToPic(copyInfo->pcPicCurr->getPicYuvOrg(), copyInfo->nThread, copyInfo->index);

	return NULL;
}

#endif

#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
Void TEncTop::ETRI_encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPic*>& rcListPic, AccessUnit_t* accessUnitsOut, Int& iNumEncoded )
#else
Void TEncTop::ETRI_encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComList<TComPicYuv*>& rcListPicYuvRecOut, AccessUnit_t* accessUnitsOut, Int& iNumEncoded)
#endif
{
#if ETRI_DLL_INTERFACE
	if (m_iPOCLast == -1){ //refresh DLL		
		m_cGOPEncoder.ETRI_init_totalCoded();
		m_cGOPEncoder.ETRI_setbFirst(true);
		short eSliceIndex = ETRI_getETRI_SliceIndex();
		ETRI_setETRI_SliceIndex(eSliceIndex);
	}
#endif

	if (pcPicYuvOrg) {
		// get original YUV
		TComPic* pcPicCurr = NULL;
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
		ETRI_xGetNewPicBuffer( pcPicCurr, &rcListPic);
#else
		xGetNewPicBuffer(pcPicCurr);
#endif

#if ETRI_COPYTOPIC_MULTITHREAD
		const int nThread = 4;
		copyToPicInfo copyInfo[nThread];
		pthread_t tempThread[nThread];

		for (int i = 0; i < nThread; i++)
		{
			copyInfo[i].pcComPicYuv = pcPicYuvOrg;
			copyInfo[i].pcPicCurr = pcPicCurr;
			copyInfo[i].index = i;
			copyInfo[i].nThread = nThread;

			pthread_create(&tempThread[i], NULL, &TEncTop::copyToPicProc, (void *)&(copyInfo[i]));
		}

		for (int i = 0; i < nThread; i++)
		{
			pthread_join(tempThread[i], NULL);
		}
#else
		pcPicYuvOrg->copyToPic(pcPicCurr->getPicYuvOrg());
#endif

		// compute image characteristics
		if (getUseAdaptiveQP())
		{
			m_cPreanalyzer.xPreanalyze(dynamic_cast<TEncPic*>(pcPicCurr));
		}
	}

#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	if (!m_iNumPicRcvd || (!flush && m_iPOCLast != 0 && m_iNumPicRcvd != m_uiIntraPeriod && m_uiIntraPeriod))
#else
	if (!m_iNumPicRcvd || (!flush && m_iPOCLast != 0 && m_iNumPicRcvd != m_iGOPSize && m_iGOPSize))
#endif
	{
		iNumEncoded = 0;
		return;
	}

#if 0 // gplusplus_151005 TEncFrame move    
	if (m_RCEnableRateControl)
	{
		m_cRateCtrl.initRCGOP(m_iNumPicRcvd);
	}
#endif


#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	m_cGOPEncoder.ETRI_compressGOP(m_iPOCLast, m_iNumPicRcvd, rcListPic, rcListPicYuvRecOut, accessUnitsOut, false, false);
#else
	m_cGOPEncoder.ETRI_compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false);
#endif
#if KAIST_HRD_print
	FILE *fp2 = fopen("hrd_output.txt", "a");
	Int encOrder2POC[32] = { 0, 8, 4, 2, 1, 3, 6, 5, 7, 16, 12, 10, 9, 11, 14, 13, 15, 24, 20, 18, 17, 19, 22, 21, 23, 28, 26, 25, 27, 30, 29, 31 };// 32 between 23 and 28
	Int intraSize = m_cRateCtrl.m_intraSize;
	for (Int iIDRIndex = 0; iIDRIndex < m_cRateCtrl.KAIST_NUM_IDR_ENC; iIDRIndex++)
	{
		for (Int iDecOrder = 0; iDecOrder < intraSize; iDecOrder++)
		{
			if (iDecOrder == 0)
			{
				if (iIDRIndex == 0)
					m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] = Int(m_cRateCtrl.m_cpbSize*getInitialCpbFullness()) - m_cRateCtrl.m_sliceActualBits[iIDRIndex][encOrder2POC[iDecOrder]];
				else
					m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] = m_cRateCtrl.m_cpbState[iIDRIndex - 1][encOrder2POC[intraSize - 1]] - m_cRateCtrl.m_sliceActualBits[iIDRIndex][encOrder2POC[iDecOrder]];
			}
			else
				m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] = m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder - 1]] - m_cRateCtrl.m_sliceActualBits[iIDRIndex][encOrder2POC[iDecOrder]];
			if (m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] < 0)
			{
				m_cRateCtrl.m_cpbStateFlag[iIDRIndex][encOrder2POC[iDecOrder]] = -1;
			}
			m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] += m_cRateCtrl.m_bufferingRate;
			if (m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]] > m_cRateCtrl.m_cpbSize)
			{
				m_cRateCtrl.m_cpbStateFlag[iIDRIndex][encOrder2POC[iDecOrder]] = 0;
			}
			fprintf(fp2, "%6d\t%6d\t%6d\n", iIDRIndex*intraSize + encOrder2POC[iDecOrder], m_cRateCtrl.m_cpbState[iIDRIndex][encOrder2POC[iDecOrder]], m_cRateCtrl.m_cpbStateFlag[iIDRIndex][encOrder2POC[iDecOrder]]);
		}
	}
	fclose(fp2);	
#endif

#if 0 // gplusplus_151005 TEncFrame move  
	if (m_RCEnableRateControl)
	{
		m_cRateCtrl.destroyRCGOP();
	}
#endif

	iNumEncoded = m_iNumPicRcvd;
	m_iNumPicRcvd = 0;
	m_uiNumAllPicCoded += iNumEncoded;
}

Void TEncTop::ETRI_xPocReArrayPicBuffer(TComList<TComPic*>* pcListPic, int nOffset)
{
	TComPic* pcPic = NULL;
	TComPic* pcPicCopy = NULL;
	TComList<TComPic*>::iterator iterPicExtract;
	TComList<TComPic*>::iterator iterPic; //  = pcListPic->begin();
	int i;

	iterPicExtract = pcListPic->begin();
	if (nOffset != 0)
	{
		for (i = 0; i < pcListPic->size(); i++)
		{
			pcPicCopy = *(iterPicExtract);

			if (nOffset == pcPicCopy->getPOC())
				break;

			iterPicExtract++;
		}
	}

	iterPic = pcListPic->begin();
	for (i = 0; i < pcListPic->size(); i++)
	{
		pcPic = *(iterPic);
		if (i == 0 && pcPicCopy)
		{
			pcListPic->insert(iterPic, pcPicCopy);
			pcListPic->erase(iterPicExtract);
			continue;
		}
		iterPic++;

		pcPic->getSlice(0)->setPOC(nOffset + i);
	}
}

Void TEncTop::ETRI_xGetNewPicBuffer(TComPic*& rpcPic, TComList<TComPic*>* pcListPic)
{
	m_iPOCLast++;

	TComList<TComPic*>::iterator iterPic = pcListPic->begin();
	while (iterPic != pcListPic->end())
	{
		rpcPic = *(iterPic);
		if (rpcPic->getPOC() == m_iPOCLast)
			break;
		iterPic++;
	}

	rpcPic->setReconMark(false);

	m_iNumPicRcvd++;

	rpcPic->getSlice(0)->setPOC(m_iPOCLast);
	// mark it should be extended
	rpcPic->getPicYuvRec()->setBorderExtension(false);
}



#endif
//! \}
