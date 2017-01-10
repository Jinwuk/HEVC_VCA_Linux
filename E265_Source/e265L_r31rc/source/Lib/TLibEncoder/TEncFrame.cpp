/*
*********************************************************************************************

   Copyright (c) 2006 Electronics and Telecommunications Research Institute (ETRI) All Rights Reserved.

   Following acts are STRICTLY PROHIBITED except when a specific prior written permission is obtained from 
   ETRI or a separate written agreement with ETRI stipulates such permission specifically:

      a) Selling, distributing, sublicensing, renting, leasing, transmitting, redistributing or otherwise transferring 
          this software to a third party;
      b) Copying, transforming, modifying, creating any derivatives of, reverse engineering, decompiling, 
          disassembling, translating, making any attempt to discover the source code of, the whole or part of 
          this software in source or binary form; 
      c) Making any copy of the whole or part of this software other than one copy for backup purposes only; and 
      d) Using the name, trademark or logo of ETRI or the names of contributors in order to endorse or promote 
          products derived from this software.

   This software is provided "AS IS," without a warranty of any kind. ALL EXPRESS OR IMPLIED CONDITIONS, 
   REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY, FITNESS 
   FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT, ARE HEREBY EXCLUDED. IN NO EVENT WILL ETRI 
   (OR ITS LICENSORS, IF ANY) BE LIABLE FOR ANY LOST REVENUE, PROFIT OR DATA, OR FOR DIRECT, 
   INDIRECT, SPECIAL, CONSEQUENTIAL, INCIDENTAL OR PUNITIVE DAMAGES, HOWEVER CAUSED AND 
   REGARDLESS OF THE THEORY OF LIABILITY, ARISING FROM, OUT OF OR IN CONNECTION WITH THE USE 
   OF OR INABILITY TO USE THIS SOFTWARE, EVEN IF ETRI HAS BEEN ADVISED OF THE POSSIBILITY OF 
   SUCH DAMAGES.

   Any permitted redistribution of this software must retain the copyright notice, conditions, and disclaimer 
   as specified above.

*********************************************************************************************
*/
/** 
	\file   	TEncFrame.cpp
   	\brief    	Frame encoder class (Source)
*/

#include "TEncTop.h"
#include "TEncFrame.h"
#include "libmd5/MD5.h"
#include "NALwrite.h"

#include <time.h>

//! \ingroup TLibEncoder
//! \{


// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================
TEncFrame::TEncFrame()
{
#if !ETRI_MULTITHREAD_2
	em_pcEncTop = nullptr;
	em_pcSliceEncoder = nullptr;

	em_pcGOPEncoder  	= nullptr;
#if KAIST_RC
	em_pcRateCtrl   	= nullptr;
#endif
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
#else 
	em_pcLoopFilter  	= nullptr;
#endif 
	em_pcSAO     		= nullptr;

	em_pcEntropyCoder	= nullptr;		
	em_pcSbacCoder   	= nullptr;
	em_pcBinCABAC   	= nullptr;
	em_pcBitCounter  	= nullptr;

	em_pcBitstreamRedirect = nullptr;
	em_iMTFrameIdx = 0;
#endif
	em_pcAU = nullptr;


	em_iGOPid   		= 0;
	em_iPOCLast   		= 0;
	em_iNumPicRcvd   	= 0;
	em_iNumSubstreams   = 0;
	em_IRAPGOPid    	= 0;		///How about -1 to keep consistency to TEncGOP ? : 2015 5 23 by Seok

	em_bLastGOP 		= false;
	em_bisField  		= false;
	em_bisTff   		= false;

	em_iGopSize   		= 0;
	em_iLastIDR   		= 0;

	accumBitsDU  		= nullptr;
	accumNalsDU 		= nullptr;

	em_dEncTime  		= 0.0;
#if ETRI_DLL_INTERFACE	// 2013 10 23 by Seok
	FrameTypeInGOP		= 0;
	FramePOC			= 0;
	FrameEncodingOrder	= 0;
#endif

#if ETRI_MULTITHREAD_2
	/////////////////////////////////////////
	// gplusplus
	em_pcEncTop      	= nullptr;			
	em_pcGOPEncoder  	= nullptr;	

	em_pppcRDSbacCoder   =  NULL;
	em_pppcBinCoderCABAC =  NULL;
	em_cRDGoOnSbacCoder.init( &em_cRDGoOnBinCoderCABAC );

	em_pcSbacCoders           = NULL;
	em_pcBinCoderCABACs       = NULL;
	em_ppppcRDSbacCoders      = NULL;
	em_ppppcBinCodersCABAC    = NULL;
	em_pcRDGoOnSbacCoders     = NULL;
	em_pcRDGoOnBinCodersCABAC = NULL;
	em_pcBitCounters          = NULL;
	em_pcRdCosts              = NULL;
	em_pcTileEncoder		  = NULL;
#endif
}

TEncFrame::~TEncFrame()
{
#if ETRI_MULTITHREAD_2
   if(em_pcTileEncoder)
		delete[] em_pcTileEncoder;
#endif
}

#if ETRI_MULTITHREAD_2
Void TEncFrame::create(TEncTop* pcEncTop)
{
	TEncTop* e_pcEncTop = pcEncTop;

	em_cSliceEncoder.create( e_pcEncTop->getSourceWidth(), e_pcEncTop->getSourceHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
	em_cCuEncoder.create( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight );

	if(e_pcEncTop->getUseSAO())
	{
		em_cEncSAO.create( e_pcEncTop->getSourceWidth(), e_pcEncTop->getSourceHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
		em_cEncSAO.createEncData(e_pcEncTop->getSaoLcuBoundary());
#else
		em_cEncSAO.createEncData();
#endif
	}

#if ADAPTIVE_QP_SELECTION
	if (e_pcEncTop->getUseAdaptQpSelect())
	{
		em_cTrQuant.initSliceQpDelta();
	}
#endif
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	for (int i = 0; i < MAX_NUM_THREAD; i++)
		em_cLoopFilter[i].create(g_uiMaxCUDepth);
#else 
	em_cLoopFilter.create( g_uiMaxCUDepth );
#endif 
	// Rate Control 
	if(e_pcEncTop->getUseRateCtrl())
	{
#if KAIST_RC

		//em_cRateCtrl = e_pcEncTop->getRateCtrl();
#endif
	}

	em_pppcRDSbacCoder = new TEncSbac** [g_uiMaxCUDepth+1];
#if FAST_BIT_EST
	em_pppcBinCoderCABAC = new TEncBinCABACCounter** [g_uiMaxCUDepth+1];
#else
	em_pppcBinCoderCABAC = new TEncBinCABAC** [g_uiMaxCUDepth+1];
#endif

	for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		em_pppcRDSbacCoder[iDepth] = new TEncSbac* [CI_NUM];
#if FAST_BIT_EST
		em_pppcBinCoderCABAC[iDepth] = new TEncBinCABACCounter* [CI_NUM];
#else
		em_pppcBinCoderCABAC[iDepth] = new TEncBinCABAC* [CI_NUM];
#endif

		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			em_pppcRDSbacCoder[iDepth][iCIIdx] = new TEncSbac;
#if FAST_BIT_EST
			em_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABACCounter;
#else
			em_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABAC;
#endif
			em_pppcRDSbacCoder   [iDepth][iCIIdx]->init( em_pppcBinCoderCABAC [iDepth][iCIIdx] );
		}
	}

	//-------------------------------------------------------------
	//	Create Tile Encoder
	//-------------------------------------------------------------
	UInt	uiNumTile = (e_pcEncTop->getNumColumnsMinus1() + 1) * (e_pcEncTop->getNumRowsMinus1() + 1);

#if 0
	EDPRINTF(stderr, "m_iNumColumnsMinus1 : %d \n", e_pcEncTop->getNumColumnsMinus1());
	EDPRINTF(stderr, "m_iNumRowsMinus1    : %d \n", e_pcEncTop->getNumRowsMinus1());
	EDPRINTF(stderr, "uiNumTile           : %d \n", uiNumTile);
#endif

	em_pcTileEncoder 	= new TEncTile[uiNumTile];
	for(UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].ETRI_getTotalNumbetOfTile() = uiNumTile;
		em_pcTileEncoder[uiTileIdx].create();
	}
}
#else
Void 	TEncFrame::create()
{

}
#endif

Void TEncFrame::destroy()
{
#if ETRI_MULTITHREAD_2
	// gplusplus [[
	// gplusplus_151005 memory leak bugfix
	UInt uiNumTile = em_pcTileEncoder[0].ETRI_getTotalNumbetOfTile();
	for (UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].destroy(ETRI_MODIFICATION_V00);
		//EDPRINTF(stderr, "em_pcTileEncoder[%d].destroy	OK \n", uiTileIdx);
	}

	em_cSliceEncoder.destroy();
	em_cCuEncoder.destroy();

	if (em_pcEncTop->getSPS()->getUseSAO())
	{
		em_cEncSAO.destroyEncData();
		em_cEncSAO.destroy();
	}
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	for (int i = 0; i < MAX_NUM_THREAD; i++)
		em_cLoopFilter[i].destroy();
#else 
	em_cLoopFilter.destroy();
#endif 
#if KAIST_RC

#endif

	Int iDepth;
	for (iDepth = 0; iDepth < g_uiMaxCUDepth + 1; iDepth++)
	{
		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx++)
		{
			delete em_pppcRDSbacCoder[iDepth][iCIIdx];
			delete em_pppcBinCoderCABAC[iDepth][iCIIdx];
		}
	}

	for (iDepth = 0; iDepth < g_uiMaxCUDepth + 1; iDepth++)
	{
		delete[] em_pppcRDSbacCoder[iDepth];
		delete[] em_pppcBinCoderCABAC[iDepth];
	}

	delete[] em_pppcRDSbacCoder;
	delete[] em_pppcBinCoderCABAC;

	for (UInt ui = 0; ui < em_iNumSubstreams; ui++)
	{
		for (iDepth = 0; iDepth < g_uiMaxCUDepth + 1; iDepth++)
		{
			for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx++)
			{
				delete em_ppppcRDSbacCoders[ui][iDepth][iCIIdx];
				delete em_ppppcBinCodersCABAC[ui][iDepth][iCIIdx];
			}
		}

		for (iDepth = 0; iDepth < g_uiMaxCUDepth + 1; iDepth++)
		{
			delete[] em_ppppcRDSbacCoders[ui][iDepth];
			delete[] em_ppppcBinCodersCABAC[ui][iDepth];
		}
		delete[] em_ppppcRDSbacCoders[ui];
		delete[] em_ppppcBinCodersCABAC[ui];
	}

	delete[] em_ppppcRDSbacCoders;
	delete[] em_ppppcBinCodersCABAC;
	delete[] em_pcSbacCoders;
	delete[] em_pcBinCoderCABACs;
	delete[] em_pcRDGoOnSbacCoders;
	delete[] em_pcRDGoOnBinCodersCABAC;
	delete[] em_pcBitCounters;
	delete[] em_pcRdCosts;
	// ]]	
#endif
}
// ====================================================================================================================
// Data and Coder Setting
// ====================================================================================================================
#if ETRI_MULTITHREAD_2
Void TEncFrame::init(TEncGOP* pcGOPEncoder)
{
	em_pcGOPEncoder = pcGOPEncoder;
	em_pcEncTop = pcGOPEncoder->ETRI_getEncTop();
	// gplusplus_151005 TEncTop
	TComPPS* pcPPS = em_pcEncTop->getPPS();
	if (pcPPS->getDependentSliceSegmentsEnabledFlag())
	{
		Int NumCtx = pcPPS->getEntropyCodingSyncEnabledFlag() ? 2 : 1;
		em_cSliceEncoder.initCtxMem(NumCtx);
		for (UInt st = 0; st < NumCtx; st++)
		{
			TEncSbac* ctx = NULL;
			ctx = new TEncSbac;
			ctx->init(&em_cBinCoderCABAC);
			em_cSliceEncoder.setCtxMem(ctx, st);
		}
	}

	em_cSliceEncoder.init(em_pcEncTop, this);
	em_cCuEncoder.init(em_pcEncTop, this);

	em_cTrQuant.init(1 << em_pcEncTop->getQuadtreeTULog2MaxSize(),
		em_pcEncTop->getUseRDOQ(),
		em_pcEncTop->getUseRDOQTS(),
		true,
		em_pcEncTop->getUseTransformSkipFast(),
#if ADAPTIVE_QP_SELECTION   
		em_pcEncTop->getUseAdaptQpSelect()
#endif
		);

	// Initialize encoder search class
	em_cSearch.init(em_pcEncTop, &em_cTrQuant, em_pcEncTop->getSearchRange(), em_pcEncTop->ETRI_getBipredSearchRange(), em_pcEncTop->getFastSearch(),
		0, &em_cEntropyCoder, &em_cRdCost, em_pppcRDSbacCoder, &em_cRDGoOnSbacCoder);

	em_cpbRemovalDelay = 0;										///Important Variable :  2015 5 26 by Seok

	//-------------------------------------------------------------
	//	Init Tile Encoder
	//-------------------------------------------------------------
	UInt	uiNumTile = (em_pcEncTop->getNumColumnsMinus1() + 1) * (em_pcEncTop->getNumRowsMinus1() + 1);

	for (UInt uiTileIdx = 0; uiTileIdx < uiNumTile; uiTileIdx++)
	{
		em_pcTileEncoder[uiTileIdx].init(em_pcEncTop, uiTileIdx, ETRI_MODIFICATION_V00);
	}

#if 0 //ETRI_E265_PH01
	EDPRINTF(stderr, "------------------------------------------ \n");
	EDPRINTF(stderr, " Compiled @%s  [%s] \n\n", __DATE__, __TIME__);
#endif
}
#else
Void TEncFrame::init(TEncTop* pcCfg, TEncGOP* pcGOPEncoder, Int iFrameIdx)
{
	em_pcEncTop     	= pcCfg;
	em_pcGOPEncoder 	= pcGOPEncoder;
	em_pcSliceEncoder 	= pcGOPEncoder->getSliceEncoder();

	//-------- Encoder filter -------
	em_pcSbacCoder		= pcGOPEncoder->ETRI_getSbacCoder();
	em_pcBinCABAC		= pcGOPEncoder->ETRI_getBinCABAC();
	em_pcEntropyCoder 	= pcGOPEncoder->ETRI_getEntropyCoder();
	em_pcCavlcCoder		= pcGOPEncoder->ETRI_getCavlcCoder();
	em_pcBitCounter		= pcGOPEncoder->ETRI_getBitCounter();		///For SAO Process in Frame Class 2015 5 25 by Seok

	//----- Adaptive Loop filter ----
	em_pcSAO 			= pcGOPEncoder->ETRI_getSAO();
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
	for (int i=0; i < MAX_NUM_THREAD; i++)
		em_pcLoopFilter[i] = pcGOPEncoder->ETRI_getLoopFilter(i);
#else 
	em_pcLoopFilter		= pcGOPEncoder->ETRI_getLoopFilter();
#endif 
#if KAIST_RC
	em_pcRateCtrl  		= pcGOPEncoder->getRateCtrl();
#endif
	em_pcseiWriter 		= pcGOPEncoder->ETRI_getSEIWriter();

//	em_storedStartCUAddrForEncodingSlice = new std::vector<Int>; 		///<P : 2015 5 25 by Seok
//	em_storedStartCUAddrForEncodingSliceSegment = new std::vector<Int>; ///<P : 2015 5 25 by Seok

	em_iMTFrameIdx 	= iFrameIdx;
	em_bFirst			= pcGOPEncoder->ETRI_getbFirst();				///Important Variable :  2015 5 26 by Seok
	em_cpbRemovalDelay = 0;										///Important Variable :  2015 5 26 by Seok

}
#endif
// ====================================================================================================================
// ETRI Compress Frame Functions @ 2015 5 11 by Seok
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Select uiColDir. This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
UInt	TEncFrame::ETRI_Select_UiColDirection(Int iGOPid, UInt	uiColDir)
{
	//select uiColDir
	Int iRef = 0;
	Int iCloseLeft=1, iCloseRight=-1;
	for(Int i = 0; i<em_pcEncTop->getGOPEntry(iGOPid).m_numRefPics; i++)
	{
		iRef = em_pcEncTop->getGOPEntry(iGOPid).m_referencePics[i];
		if  	(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))	{iCloseRight=iRef;}
		else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1)) 	{iCloseLeft=iRef;}
	}

	if(iCloseRight>-1){iCloseRight=iCloseRight+em_pcEncTop->getGOPEntry(iGOPid).m_POC-1;}
	if(iCloseLeft<1){
		iCloseLeft=iCloseLeft+em_pcEncTop->getGOPEntry(iGOPid).m_POC-1;
		while(iCloseLeft<0)	{iCloseLeft+=em_iGopSize;}
	}

	Int iLeftQP=0, iRightQP=0;
	for(Int i=0; i<em_iGopSize; i++)
	{
		if(em_pcEncTop->getGOPEntry(i).m_POC==(iCloseLeft%em_iGopSize)+1)
		iLeftQP= em_pcEncTop->getGOPEntry(i).m_QPOffset;

		if (em_pcEncTop->getGOPEntry(i).m_POC==(iCloseRight%em_iGopSize)+1)
		iRightQP=em_pcEncTop->getGOPEntry(i).m_QPOffset;
	}

	return ((iCloseRight>-1&&iRightQP<iLeftQP)? 0 : uiColDir);  	/// Return Value is uiColDir

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set Default Slice Level data to the same as SPS level  flag. 
	           It is not so important, so as it is possible to remove the codes with ETRI_SCALING_LIST_OPTIMIZATION
	           This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_SliceDataInitialization(TComPic* pcPic, TComSlice* pcSlice)
{
	//set default slice level flag to the same as SPS level flag
	pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );
#if !ETRI_SCALING_LIST_OPTIMIZATION
	pcSlice->setScalingList ( em_pcEncTop->getScalingList()	);
	if(em_pcEncTop->getUseScalingListId() == SCALING_LIST_OFF)
	{
		em_pcEncTop->getTrQuant()->setFlatScalingList();
		em_pcEncTop->getTrQuant()->setUseScalingList(false);
		em_pcEncTop->getSPS()->setScalingListPresentFlag(false);
		em_pcEncTop->getPPS()->setScalingListPresentFlag(false);
	}
	else if(em_pcEncTop->getUseScalingListId() == SCALING_LIST_DEFAULT)
	{
		pcSlice->setDefaultScalingList ();
		em_pcEncTop->getSPS()->setScalingListPresentFlag(false);
		em_pcEncTop->getPPS()->setScalingListPresentFlag(false);
		em_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList());
		em_pcEncTop->getTrQuant()->setUseScalingList(true);
	}
	else if(em_pcEncTop->getUseScalingListId() == SCALING_LIST_FILE_READ)
	{
		if(pcSlice->getScalingList()->xParseScalingList(em_pcEncTop->getScalingListFile()))
		{
			pcSlice->setDefaultScalingList ();
		}
		pcSlice->getScalingList()->checkDcOfMatrix();
		em_pcEncTop->getSPS()->setScalingListPresentFlag(pcSlice->checkDefaultScalingList());
		em_pcEncTop->getPPS()->setScalingListPresentFlag(false);
		em_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList());
		em_pcEncTop->getTrQuant()->setUseScalingList(true);
	}
	else
	{
		printf("error : ScalingList == %d no support\n",em_pcEncTop->getUseScalingListId());
		assert(0);
	}
#endif

}



/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set RPS data for Slice Here.  
			It is so important that, when we make Frame Paralleization, we should fix and add the codes for RPS providing Frame Parallelization
			I mark the location for RPS amenment in this function
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_SetReferencePictureSetforSlice(TComPic* pcPic, TComSlice* pcSlice, Int iGOPid, Int pocCurr, Bool isField, TComList<TComPic*>& rcListPic)
{

	if(pcSlice->getSliceType()==B_SLICE&&em_pcEncTop->getGOPEntry(iGOPid).m_sliceType=='P')
	{
		pcSlice->setSliceType(P_SLICE);
	}
	if(pcSlice->getSliceType()==B_SLICE&&em_pcEncTop->getGOPEntry(iGOPid).m_sliceType=='I')
	{
		pcSlice->setSliceType(I_SLICE);
	}

	// Set the nal unit type
	pcSlice->setNalUnitType(em_pcGOPEncoder->getNalUnitType(pocCurr, em_iLastIDR, isField));
	if(pcSlice->getTemporalLayerNonReferenceFlag())
	{
		if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
		!(em_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
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
		em_associatedIRAPType = pcSlice->getNalUnitType();
		em_associatedIRAPPOC = pocCurr;
	}
	pcSlice->setAssociatedIRAPType(em_associatedIRAPType);
	pcSlice->setAssociatedIRAPPOC(em_associatedIRAPPOC);
#endif
#endif

	// Do decoding refresh marking if any
	pcSlice->decodingRefreshMarking(em_pocCRA, em_bRefreshPending, rcListPic);
	em_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
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
	em_associatedIRAPType = pcSlice->getNalUnitType();
	em_associatedIRAPPOC = pocCurr;
	}
	pcSlice->setAssociatedIRAPType(em_associatedIRAPType);
	pcSlice->setAssociatedIRAPPOC(em_associatedIRAPPOC);
#endif
#endif

#if ALLOW_RECOVERY_POINT_AS_RAP
	if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, em_iLastRecoveryPicPOC, em_pcEncTop->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP()) 
#if EFFICIENT_FIELD_IRAP
	|| (isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
#endif
	)
	{
		/*-------------------------------------------------------------------------
			If you want RPS amendment, Please add the code here	@ 2015 5 14 by Seok 
			For example : 
				//ETRI_V10 change, no need by yhee 2013.10.16
				#if (!ETRI_MTFrameRPS && ETRI_MTLevel3Frame)
				if(em_iGOPDepth !=3 || em_bLastGOP){
					pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS());
				}
				#endif
		---------------------------------------------------------------------------*/
		pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), em_iLastRecoveryPicPOC, em_pcEncTop->getDecodingRefreshType() == 3);
	}
#else
	if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false) != 0) || (pcSlice->isIRAP()))
	{
	pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP());
	}
#endif
	pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());

	if(pcSlice->getTLayer() > 0 
	&&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N	  // Check if not a leading picture
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
			for(Int ii=iGOPid+1;(ii<em_pcEncTop->getGOPSize() && isSTSA==true);ii++)
			{
				Int lTid= em_pcEncTop->getGOPEntry(ii).m_temporalId;
				if(lTid==pcSlice->getTLayer())
				{
					TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
					for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
					{
						if(nRPS->getUsed(jj))
						{
							Int tPoc=em_pcEncTop->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
							Int kk=0;
							for(kk=0;kk<em_pcEncTop->getGOPSize();kk++)
							{
								if(em_pcEncTop->getGOPEntry(kk).m_POC==tPoc)
								break;
							}
							Int tTid=em_pcEncTop->getGOPEntry(kk).m_temporalId;
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

	em_pcGOPEncoder->arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set RefPicList based on RPS generated by ETRI_SetReferencePictureSetforSlice
			Remenber the RefList for each Picture is Evaluated HERE !!!
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_refPicListModification(TComSlice* pcSlice, TComRefPicListModification* refPicListModification, TComList<TComPic*>& rcListPic, Int iGOPid, UInt& uiColDir)
{

	refPicListModification->setRefPicListModificationFlagL0(0);
	refPicListModification->setRefPicListModificationFlagL1(0);
	pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(em_pcEncTop->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
	pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(em_pcEncTop->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
	
#if ADAPTIVE_QP_SELECTION
#if ETRI_MULTITHREAD_2
	pcSlice->setTrQuant(&em_cTrQuant);
#else
	pcSlice->setTrQuant( em_pcEncTop->getTrQuant() );
#endif
#endif
	
	//	Set reference list
	pcSlice->setRefPicList ( rcListPic );
	
	//	Slice info. refinement
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

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set TMVPset based on Evaluated POC for Slice
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_NoBackPred_TMVPset(TComSlice* pcSlice, Int iGOPid)
{
	pcSlice->setRefPOCList();
	pcSlice->setList1IdxToList0Idx();		///L0034_COMBINED_LIST_CLEANUP is Auto Active (Default) @ 2015 5 14 by Seok

	if (em_pcEncTop->getTMVPModeId() == 2)
	{
		if (iGOPid == 0) // first picture in SOP (i.e. forward B)
		{
			pcSlice->setEnableTMVPFlag(0);
		}
		else
		{	// Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
			pcSlice->setEnableTMVPFlag(1);
		}
		pcSlice->getSPS()->setTMVPFlagsPresent(1);
	}
	else if (em_pcEncTop->getTMVPModeId() == 1)
	{
		pcSlice->getSPS()->setTMVPFlagsPresent(1);
		pcSlice->setEnableTMVPFlag(1);
	}
	else
	{
		pcSlice->getSPS()->setTMVPFlagsPresent(0);
		pcSlice->setEnableTMVPFlag(0);
	}
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set MVdL1Zero Flag 
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_setMvdL1ZeroFlag(TComPic* pcPic, TComSlice* pcSlice)
{
	Bool bGPBcheck=false;
	if ( pcSlice->getSliceType() == B_SLICE)
	{
		if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
		{
			bGPBcheck=true;
			for (Int i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
			{
				if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
				{
					bGPBcheck=false;
					break;
				}
			}
		}
	}
 
	pcSlice->setMvdL1ZeroFlag(bGPBcheck);
 	pcPic->getSlice(pcSlice->getSliceIdx())->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());

}



/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Rate Control For Slice : Alternative Name is ETRI_RateControlLambdaDomain in the Previous Version
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if ETRI_MULTITHREAD_2
#if KAIST_RC
Void TEncFrame::ETRI_RateControlSlice(Int pocCurr, TComPic* pcPic, TComSlice* pcSlice, int iGOPid, ETRI_SliceInfo& ReturnValue)
{
	Double* lambda = ReturnValue.lambda;
	Int* actualHeadBits = ReturnValue.actualHeadBits;
	Int* actualTotalBits = ReturnValue.actualTotalBits;
	Int* estimatedBits = ReturnValue.estimatedBits;

	*lambda = 0.0; *actualHeadBits = *actualTotalBits = *estimatedBits = 0;

	Int intra_size = em_pcEncTop->getIntraPeriod();
	Int iIDRIndex = pocCurr / intra_size;
	Int iIDRModulus = pocCurr % intra_size;

  Int frameLevel = em_pcRateCtrl->getGOPID2Level(iGOPid);
	if (pcPic->getSlice(0)->getSliceType() == I_SLICE)
	{
		frameLevel = 0;
	}

  TRCPic* tRCPic = em_pcRateCtrl->getTRCPic(iIDRModulus);

	tRCPic->m_POC = pocCurr;
	tRCPic->m_frameLevel = frameLevel;

#if !KAIST_USEPREPS
	Int targetBits = tRCPic->calcFrameTargetBit(pocCurr);
	if (targetBits < 200)
		targetBits = 200;   // at least allocate 200 bits for picture data

	Int sliceQP = em_pcEncTop->getInitialQP();
	Int bits = targetBits;

	if (frameLevel == 0)   // intra case
	{
		em_cSliceEncoder.calCostSliceI(pcPic);
		if (em_pcEncTop->getIntraPeriod() != 1)   // do not refine allocated bits for all intra case
		{
			bits = tRCPic->getRefineBitsForIntra(bits);
			if (bits < 200){ bits = 200; }
		}
		tRCPic->getLCUInitTargetBits(bits);
	}
#else
	Int targetBits = tRCPic->calcFrameTargetBit(pocCurr);
	if (targetBits < 200)
		targetBits = 200;   // at least allocate 200 bits for picture data
	if (frameLevel == 0)   // intra case
	{
		targetBits *= 3.0 / 4.0;
		em_cSliceEncoder.calCostSliceI(pcPic);
		tRCPic->getLCUInitTargetBits(targetBits);
	}

	Int sliceQP = em_pcEncTop->getInitialQP();
	Int bits = targetBits;
#endif
#if KAIST_HARDCODING_TARGETBIT
	char ch[200];
	Int BitsIn;
	sprintf(ch, "Targetbits_%d_%s.txt", pocCurr, inputFile);
	FILE *fp = fopen(ch, "r");
	if (!fp)
	{
		printf("\tError-ReadTargetbits-nofile\n");
	}
	else
	{
		fscanf(fp, "%d", &BitsIn);
		fclose(fp);
		bits = BitsIn;
	}
	
#endif
	tRCPic->m_targetBit = bits;
  em_pcRateCtrl->m_sliceTotalTargetBits[iIDRModulus] = tRCPic->m_targetBit;

#if KAIST_HRD
  tRCPic->m_targetBit = em_pcRateCtrl->xEstimateVirtualBuffer(iIDRModulus);
#endif

	
	*lambda = tRCPic->estimatePicLambda(pcSlice->getSliceType());
	sliceQP = tRCPic->estimatePicQP(*lambda, pcSlice);

#if KAIST_SCENECHANGE
	if (em_cRateCtrl->m_iSceneChange == iIDRModulus)
	{
		switch (frameLevel)
		{
		case 1:
			sliceQP = 30;
			break;
		case 2:
		case 3:
			sliceQP = 35;
			break;
		case 4:
			sliceQP = 40;
			break;
		}
		*lambda = exp((sliceQP - 13.7122) / 4.2005);
	}
#endif

  Int* tileMap = em_pcRateCtrl->m_tileIdxMap;
  for (Int tileIdx = 0; tileIdx < em_pcRateCtrl->m_numTile; tileIdx++)
		tRCPic->m_bitsLeft[tileIdx] = 0;
	for (Int i = 0; i < tRCPic->m_numberOfLCU; i++)
		tRCPic->m_bitsLeft[tileMap[i]] += Int( tRCPic->m_LCUs[i].m_bitWeight);

	if (pocCurr == 0 && em_pcEncTop->getInitialQP()>0)
		sliceQP = em_pcEncTop->getInitialQP();

#if KAIST_HARDCODING_QP
	char ch[200];
	Int frameQP = sliceQP;
	Double temp_lambda = *lambda;
	sprintf(ch, "QP_%d_%s.txt", pocCurr, inputFile);
	FILE *fp = fopen(ch, "r");
	if (!fp)
	{
		frameQP = sliceQP;
		printf("\tError-ReadQP-nofile\n");
	}
	else
	{
		fscanf(fp, "%d", &frameQP);
		temp_lambda = exp((frameQP - 13.7122) / 4.2005);
		fclose(fp);
	}
	sliceQP = frameQP;
	*lambda = temp_lambda;
#endif

	em_cSliceEncoder.resetQP(pcPic, sliceQP, *lambda);
}
#endif
#else
#if KAIST_RC
Void TEncFrame::ETRI_RateControlSlice(Int pocCurr, TComPic* pcPic, TComSlice* pcSlice, int iGOPid, ETRI_SliceInfo& ReturnValue)
{
	Double* lambda = ReturnValue.lambda;
	Int* actualHeadBits = ReturnValue.actualHeadBits;
	Int* actualTotalBits = ReturnValue.actualTotalBits;
	Int* estimatedBits = ReturnValue.estimatedBits;

	*lambda = 0.0; *actualHeadBits = *actualTotalBits = *estimatedBits = 0;

	Int frameLevel = em_pcRateCtrl->getGOPID2Level(iGOPid);
	if (pcPic->getSlice(0)->getSliceType() == I_SLICE)
	{
		frameLevel = 0;
	}

	Int intra_size = em_pcRateCtrl->getIntraSize();
	Int iIDRIndex = pocCurr / intra_size;
	Int iIDRModulus = pocCurr % intra_size;
	TRCPic* tRCPic = em_pcRateCtrl->getTRCPic(iIDRModulus);

	tRCPic->m_POC = pocCurr;
	tRCPic->m_frameLevel = frameLevel;

#if !KAIST_USEPREPS
	Int targetBits = tRCPic->calcFrameTargetBit(pocCurr);
	if (targetBits < 200)
		targetBits = 200;   // at least allocate 200 bits for picture data

	Int sliceQP = em_pcEncTop->getInitialQP();
	Int bits = targetBits;

	if (frameLevel == 0)   // intra case
	{
		em_pcSliceEncoder->calCostSliceI(pcPic);
		if (em_pcEncTop->getIntraPeriod() != 1)   // do not refine allocated bits for all intra case
		{
			bits = tRCPic->getRefineBitsForIntra(bits);
			if (bits < 200){ bits = 200; }
		}
		tRCPic->getLCUInitTargetBits(bits);
	}
#else
	Int targetBits = tRCPic->calcFrameTargetBit(pocCurr);
	if (targetBits < 200)
		targetBits = 200;   // at least allocate 200 bits for picture data
	if (frameLevel == 0)   // intra case
	{
		targetBits *= 3.0 / 4.0;
#if ETRI_MULTITHREAD_2
		em_cSliceEncoder.calCostSliceI(pcPic);
#else
		em_pcSliceEncoder->calCostSliceI(pcPic);
#endif
		tRCPic->getLCUInitTargetBits(targetBits);
	}

	Int sliceQP = em_pcEncTop->getInitialQP();
	Int bits = targetBits;
#endif
#if KAIST_HARDCODING_TARGETBIT
	char ch[200];
	Int BitsIn;
	sprintf(ch, "Targetbits_%d_%s.txt", pocCurr, inputFile);
	FILE *fp = fopen(ch, "r");
	if (!fp)
	{
		printf("\tError-ReadTargetbits-nofile\n");
	}
	else
	{
		fscanf(fp, "%d", &BitsIn);
		fclose(fp);
		bits = BitsIn;
	}

#endif
	tRCPic->m_targetBit = bits;
	em_pcRateCtrl->m_sliceTotalTargetBits[iIDRModulus] = tRCPic->m_targetBit;

#if KAIST_HRD
	tRCPic->m_targetBit = em_pcRateCtrl->xEstimateVirtualBuffer(iIDRModulus);
#endif


	*lambda = tRCPic->estimatePicLambda(pcSlice->getSliceType());
	sliceQP = tRCPic->estimatePicQP(*lambda, pcSlice);

#if KAIST_SCENECHANGE
	if (em_pcRateCtrl->m_iSceneChange == iIDRModulus)
	{
		switch (frameLevel)
		{
		case 1:
			sliceQP = 30;
			break;
		case 2:
		case 3:
			sliceQP = 35;
			break;
		case 4:
			sliceQP = 40;
			break;
		}
		*lambda = exp((sliceQP - 13.7122) / 4.2005);
	}
#endif

	Int* tileMap = em_pcRateCtrl->m_tileIdxMap;
	for (Int tileIdx = 0; tileIdx < em_pcRateCtrl->m_numTile; tileIdx++)
		tRCPic->m_bitsLeft[tileIdx] = 0;
	for (Int i = 0; i < tRCPic->m_numberOfLCU; i++)
		tRCPic->m_bitsLeft[tileMap[i]] += Int(tRCPic->m_LCUs[i].m_bitWeight);

	if (pocCurr == 0 && em_pcEncTop->getInitialQP()>0)
		sliceQP = em_pcEncTop->getInitialQP();

#if KAIST_HARDCODING_QP
	char ch[200];
	Int frameQP = sliceQP;
	Double temp_lambda = *lambda;
	sprintf(ch, "QP_%d_%s.txt", pocCurr, inputFile);
	FILE *fp = fopen(ch, "r");
	if (!fp)
	{
		frameQP = sliceQP;
		printf("\tError-ReadQP-nofile\n");
	}
	else
	{
		fscanf(fp, "%d", &frameQP);
		temp_lambda = exp((frameQP - 13.7122) / 4.2005);
		fclose(fp);
	}
	sliceQP = frameQP;
	*lambda = temp_lambda;
#endif

	em_pcSliceEncoder->resetQP(pcPic, sliceQP, *lambda);
}
#endif
#endif

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set CU Address in Slice or Frame (Picture) This Function is compbined with ETRI_setStartCUAddr
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_setCUAddressinFrame(TComPic* pcPic, TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue)
{
	UInt* uiNumSlices   		= ReturnValue.uiNumSlices;
	UInt* uiInternalAddress 	= ReturnValue.uiInternalAddress;
	UInt* uiExternalAddress 	= ReturnValue.uiExternalAddress;
	UInt* uiPosX  				= ReturnValue.uiPosX;
	UInt* uiPosY  				= ReturnValue.uiPosY;
	UInt* uiWidth   			= ReturnValue.uiWidth;
	UInt* uiHeight   			= ReturnValue.uiHeight;
	UInt* uiRealEndAddress   	= ReturnValue.uiRealEndAddress;

	*uiNumSlices = 1;
	*uiInternalAddress = pcPic->getNumPartInCU()-4;
	*uiExternalAddress = pcPic->getPicSym()->getNumberOfCUsInFrame()-1;
	*uiPosX = ( *uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[*uiInternalAddress] ];
	*uiPosY = ( *uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[*uiInternalAddress] ];
	*uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
	*uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();

	while(*uiPosX>=*uiWidth||*uiPosY>=*uiHeight)
	{
		(*uiInternalAddress)--;
		*uiPosX = ( *uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[*uiInternalAddress] ];
		*uiPosY = ( *uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[*uiInternalAddress] ];
	}
	(*uiInternalAddress)++;
	if(*uiInternalAddress==pcPic->getNumPartInCU())
	{
		*uiInternalAddress = 0;
		(*uiExternalAddress)++;
	}

	*uiRealEndAddress = *uiExternalAddress * pcPic->getNumPartInCU() + *uiInternalAddress;

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Generating Codig Order Map and Inverse Coding Order Map :
			It is very Important Function for Parallel Processing in HEVC Encoding using WPP and Tiles
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_EvalCodingOrderMAPandInverseCOMAP(TComPic* pcPic)
{
    //generate the Coding Order Map and Inverse Coding Order Map
    UInt uiEncCUAddr, p;
    for(p=0, uiEncCUAddr=0; p<pcPic->getPicSym()->getNumberOfCUsInFrame(); p++, uiEncCUAddr = pcPic->getPicSym()->xCalculateNxtCUAddr(uiEncCUAddr))
    {
      pcPic->getPicSym()->setCUOrderMap(p, uiEncCUAddr);
      pcPic->getPicSym()->setInverseCUOrderMap(uiEncCUAddr, p);
    }
    pcPic->getPicSym()->setCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
    pcPic->getPicSym()->setInverseCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
}
 

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Generating Start CU Address for Slice 
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_setStartCUAddr(TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue)
{
	UInt* startCUAddrSliceIdx   		= ReturnValue.startCUAddrSliceIdx; 			
	UInt* startCUAddrSlice	 			= ReturnValue.startCUAddrSlice;   			
	UInt* startCUAddrSliceSegmentIdx	= ReturnValue.startCUAddrSliceSegmentIdx; 	
	UInt* startCUAddrSliceSegment	 	= ReturnValue.startCUAddrSliceSegment;    	
	UInt* nextCUAddr					= ReturnValue.nextCUAddr;

	*startCUAddrSliceIdx 		= 0; 	///used to index "m_uiStoredStartCUAddrForEncodingSlice" containing locations of slice boundaries
	*startCUAddrSlice	 		= 0; 	///used to keep track of current slice's starting CU addr.
	*startCUAddrSliceSegmentIdx = 0; ///used to index "m_uiStoredStartCUAddrForEntropyEncodingSlice" containing locations of slice boundaries
	*startCUAddrSliceSegment	= 0;	///used to keep track of current Dependent slice's starting CU addr.
	*nextCUAddr 				= 0;
#if ETRI_MULTITHREAD_2
	pcSlice->setSliceCurStartCUAddr(*startCUAddrSlice); 				///Setting "start CU addr" for current slice
	em_cstoredStartCUAddrForEncodingSlice.clear();

	pcSlice->setSliceSegmentCurStartCUAddr( *startCUAddrSliceSegment ); 	///Setting "start CU addr" for current Dependent slice
	em_cstoredStartCUAddrForEncodingSliceSegment.clear();

	em_cstoredStartCUAddrForEncodingSlice.push_back (*nextCUAddr);
	(*startCUAddrSliceIdx)++;
	em_cstoredStartCUAddrForEncodingSliceSegment.push_back(*nextCUAddr);
	(*startCUAddrSliceSegmentIdx)++;
#else
	pcSlice->setSliceCurStartCUAddr(*startCUAddrSlice); 				///Setting "start CU addr" for current slice
	em_storedStartCUAddrForEncodingSlice->clear();

	pcSlice->setSliceSegmentCurStartCUAddr( *startCUAddrSliceSegment ); 	///Setting "start CU addr" for current Dependent slice
	em_storedStartCUAddrForEncodingSliceSegment->clear();

	em_storedStartCUAddrForEncodingSlice->push_back (*nextCUAddr);
	(*startCUAddrSliceIdx)++;
	em_storedStartCUAddrForEncodingSliceSegment->push_back(*nextCUAddr);
	(*startCUAddrSliceSegmentIdx)++;
#endif

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set Data such as CU Address for compression of Next Slice 
	           	This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_SetNextSlice_with_IF(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue)
{
	Bool bNoBinBitConstraintViolated = (!pcSlice->isNextSlice() && !pcSlice->isNextSliceSegment());

	UInt* startCUAddrSlice  			= ReturnValue.startCUAddrSlice;
	UInt* startCUAddrSliceIdx 			= ReturnValue.startCUAddrSliceIdx;
	UInt* startCUAddrSliceSegmentIdx 	= ReturnValue.startCUAddrSliceSegmentIdx;
	UInt* startCUAddrSliceSegment 		= ReturnValue.startCUAddrSliceSegment;
	UInt* uiRealEndAddress  			= ReturnValue.uiRealEndAddress;
	UInt* uiNumSlices 					= ReturnValue.uiNumSlices;
	UInt* nextCUAddr 					= ReturnValue.nextCUAddr;
#if ETRI_MULTITHREAD_2
	if (pcSlice->isNextSlice() || (bNoBinBitConstraintViolated && em_pcEncTop->getSliceMode() == FIXED_NUMBER_OF_LCU))
	{
		*startCUAddrSlice = pcSlice->getSliceCurEndCUAddr();
		// Reconstruction slice
		em_cstoredStartCUAddrForEncodingSlice.push_back(*startCUAddrSlice);
		(*startCUAddrSliceIdx)++;
		// Dependent slice
		if (*startCUAddrSliceSegmentIdx>0 && em_cstoredStartCUAddrForEncodingSliceSegment[(*startCUAddrSliceSegmentIdx) - 1] != *startCUAddrSlice)
		{ 
			em_cstoredStartCUAddrForEncodingSliceSegment.push_back(*startCUAddrSlice);
			(*startCUAddrSliceSegmentIdx)++;
		}
		if (*startCUAddrSlice < *uiRealEndAddress)
		{
			pcPic->allocateNewSlice();
			pcPic->setCurrSliceIdx					( (*startCUAddrSliceIdx) -1 );
			em_cSliceEncoder.setSliceIdx			( (*startCUAddrSliceIdx) -1 );
			pcSlice = pcPic->getSlice				( (*startCUAddrSliceIdx) -1 );
			pcSlice->copySliceInfo					( pcPic->getSlice(0)	);
			pcSlice->setSliceIdx					( (*startCUAddrSliceIdx) -1 );
			pcSlice->setSliceCurStartCUAddr			( *startCUAddrSlice	);
			pcSlice->setSliceSegmentCurStartCUAddr	( *startCUAddrSlice	);
			pcSlice->setSliceBits(0);
			(*uiNumSlices) ++;
		}
	}
	else if (pcSlice->isNextSliceSegment() || (bNoBinBitConstraintViolated && em_pcEncTop->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU))
	{
		*startCUAddrSliceSegment	  = pcSlice->getSliceSegmentCurEndCUAddr();
		em_cstoredStartCUAddrForEncodingSliceSegment.push_back(*startCUAddrSliceSegment);
		(*startCUAddrSliceSegmentIdx)++;
		pcSlice->setSliceSegmentCurStartCUAddr( *startCUAddrSliceSegment );
	}
	else
	{
		*startCUAddrSlice  		= pcSlice->getSliceCurEndCUAddr();
		*startCUAddrSliceSegment 	= pcSlice->getSliceSegmentCurEndCUAddr();
	}
#else
	if (pcSlice->isNextSlice() || (bNoBinBitConstraintViolated && em_pcEncTop->getSliceMode()==FIXED_NUMBER_OF_LCU))
	{
	  *startCUAddrSlice = pcSlice->getSliceCurEndCUAddr();
	  // Reconstruction slice
	  em_storedStartCUAddrForEncodingSlice->push_back(*startCUAddrSlice);
	  (*startCUAddrSliceIdx)++;
	  // Dependent slice
	  if (*startCUAddrSliceSegmentIdx>0 && (*em_storedStartCUAddrForEncodingSliceSegment)[(*startCUAddrSliceSegmentIdx) - 1] != *startCUAddrSlice)
	  { 
		em_storedStartCUAddrForEncodingSliceSegment->push_back(*startCUAddrSlice);
		(*startCUAddrSliceSegmentIdx)++;
	  }
	  
	  if (*startCUAddrSlice < *uiRealEndAddress)
	  {
		pcPic->allocateNewSlice();
		pcPic->setCurrSliceIdx					( (*startCUAddrSliceIdx) -1 );
		em_pcSliceEncoder->setSliceIdx			( (*startCUAddrSliceIdx) -1 );
		pcSlice = pcPic->getSlice				( (*startCUAddrSliceIdx) -1 );
		pcSlice->copySliceInfo					( pcPic->getSlice(0)	);
		pcSlice->setSliceIdx					( (*startCUAddrSliceIdx) -1 );
		pcSlice->setSliceCurStartCUAddr			( *startCUAddrSlice	);
		pcSlice->setSliceSegmentCurStartCUAddr	( *startCUAddrSlice	);
		pcSlice->setSliceBits(0);
		(*uiNumSlices) ++;
	  }
	}
	else if (pcSlice->isNextSliceSegment() || (bNoBinBitConstraintViolated && em_pcEncTop->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU))
	{
	  *startCUAddrSliceSegment	  = pcSlice->getSliceSegmentCurEndCUAddr();
	  em_storedStartCUAddrForEncodingSliceSegment->push_back(*startCUAddrSliceSegment);
	  (*startCUAddrSliceSegmentIdx)++;
	  pcSlice->setSliceSegmentCurStartCUAddr( *startCUAddrSliceSegment );
	}
	else
	{
	  *startCUAddrSlice  		= pcSlice->getSliceCurEndCUAddr();
	  *startCUAddrSliceSegment 	= pcSlice->getSliceSegmentCurEndCUAddr();
	}

#endif

	*nextCUAddr = (*startCUAddrSlice > *startCUAddrSliceSegment) ? *startCUAddrSlice : *startCUAddrSliceSegment;

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Loop Filter and Gather the statistical Information of SAO Process
			In Comparison to the previous Version, this routine looks very simple.
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_LoopFilter(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue)
{
#if ETRI_MULTITHREAD_2
	// SAO parameter estimation using non-deblocked pixels for LCU bottom and right boundary areas
	if( pcSlice->getSPS()->getUseSAO() && em_pcEncTop->getSaoLcuBoundary()){
		em_cEncSAO.getPreDBFStatistics(pcPic);
	}

	//-- Loop filter
	Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	for (int i = 0; i < MAX_NUM_THREAD; i++)
		em_cLoopFilter[i].setCfg(bLFCrossTileBoundary);
#else 
	em_cLoopFilter.setCfg(bLFCrossTileBoundary);
#endif 
	if ( em_pcEncTop->getDeblockingFilterMetric() )
	{
		em_pcGOPEncoder->dblMetric(pcPic, *ReturnValue.uiNumSlices);
	}
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	em_cLoopFilter[0].loopFilterPic(pcPic, em_cLoopFilter);
#else 
	em_cLoopFilter.loopFilterPic( pcPic );
#endif 
#else
	// SAO parameter estimation using non-deblocked pixels for LCU bottom and right boundary areas
	if (pcSlice->getSPS()->getUseSAO() && em_pcEncTop->getSaoLcuBoundary()){
		em_pcSAO->getPreDBFStatistics(pcPic);
	}

	//-- Loop filter
	Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
	for (int i = 0; i < MAX_NUM_THREAD; i++)
		em_pcLoopFilter[i]->setCfg(bLFCrossTileBoundary);
#else 
	em_pcLoopFilter->setCfg(bLFCrossTileBoundary);
#endif 
	if (em_pcEncTop->getDeblockingFilterMetric())
	{
		em_pcGOPEncoder->dblMetric(pcPic, *ReturnValue.uiNumSlices);
	}
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
	em_pcLoopFilter[0]->loopFilterPic(pcPic, em_pcLoopFilter);
#else 
	em_pcLoopFilter->loopFilterPic(pcPic);
#endif 
#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Writeout VPS, SPS, PPS at the First Frame of Sequence. If you want IDR Parallelization, set m_bSeqFirst at the other side of encoder.
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if ETRI_MULTITHREAD_2
void TEncFrame::ETRI_WriteSeqHeader(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit, Int& actualTotalBits, Bool bFirst)
#else
void TEncFrame::ETRI_WriteSeqHeader(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit, Int& actualTotalBits)
#endif
{
#if ETRI_MULTITHREAD_2

#if ETRI_SliceEncoderHeader	
	if (bFirst)
	{
		//short eSliceEncoder_SliceIdx = em_pcEncTop->ETRI_getETRI_SliceIndex();
		short eSliceEncoder_SliceIdx = pcSlice->ETRI_getSliceIndex();
		if (eSliceEncoder_SliceIdx != 0){
			bFirst = false;
			return;
		}
	}
#endif

	em_cEntropyCoder.setEntropyCoder	( &em_cCavlcCoder, pcSlice );

	/* write various header sets. */
	if(bFirst)
	{
		OutputNALUnit nalu(NAL_UNIT_VPS);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cEntropyCoder.encodeVPS(em_pcEncTop->getVPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

		nalu = NALUnit(NAL_UNIT_SPS);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		//		if (em_bFirst)
		//		{
		UInt	e_numLongTermRefPicSPS   	= em_pcGOPEncoder->ETRI_getnumLongTermRefPicSPS();
		UInt*	e_ltRefPicPocLsbSps  		= em_pcGOPEncoder->ETRI_getltRefPicPocLsbSps();			///Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok
		Bool*	e_ltRefPicUsedByCurrPicFlag	= em_pcGOPEncoder->ETRI_getltRefPicUsedByCurrPicFlag();	///Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok

		pcSlice->getSPS()->setNumLongTermRefPicSPS(e_numLongTermRefPicSPS);
		for (Int k = 0; k < e_numLongTermRefPicSPS; k++)
		{
			pcSlice->getSPS()->setLtRefPicPocLsbSps(k, e_ltRefPicPocLsbSps[k]);
			pcSlice->getSPS()->setUsedByCurrPicLtSPSFlag(k, e_ltRefPicUsedByCurrPicFlag[k]);
		}
		//		}

		if( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() )
		{
			UInt maxCU = em_pcEncTop->getSliceArgument() >> ( pcSlice->getSPS()->getMaxCUDepth() << 1);
			UInt numDU = ( em_pcEncTop->getSliceMode() == 1 ) ? ( pcPic->getNumCUsInFrame() / maxCU ) : ( 0 );
			if( pcPic->getNumCUsInFrame() % maxCU != 0 || numDU == 0 )
			{
				numDU ++;
			}
			pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->setNumDU( numDU );
			pcSlice->getSPS()->setHrdParameters( em_pcEncTop->getFrameRate(), numDU, em_pcEncTop->getTargetBitrate(), ( em_pcEncTop->getIntraPeriod() > 0 ) );
		}

		if( em_pcEncTop->getBufferingPeriodSEIEnabled() || em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() )
		{
			pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
		}

#if (ETRI_WriteVUIHeader && SG_EnableFractionalFrameRate) 
		pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
		pcSlice->getSPS()->ETRI_setTimingInfo(em_pcEncTop->getFrameRateF()); // wsseo@2015-08-24. fix fps
#endif

#if ETRI_SliceEncoderHeader  ///< SPS for etri slice encoding	
		UInt uiOrgPicWidthInLumaSamples = pcSlice->getSPS()->getPicWidthInLumaSamples();
		UInt uiOrgPicHeightInLumaSamples = pcSlice->getSPS()->getPicHeightInLumaSamples();
		UInt uiPicWidthInLumaSamples = ETRI_Header_ImgYWidth;
		UInt uiPicHeightInLumaSamples = ETRI_Header_ImgYHeight;
		pcSlice->getSPS()->setPicWidthInLumaSamples(uiPicWidthInLumaSamples);
		pcSlice->getSPS()->setPicHeightInLumaSamples(uiPicHeightInLumaSamples);		
#endif

		em_cEntropyCoder.encodeSPS(pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

#if (ETRI_SliceEncoderHeader && !ETRI_Header_NoTile )///< First Frame-merged-PPS 	
		//Copy the org configuration
		TComPPS* pcPPS = pcSlice->getPPS();
		Int uiOrgNumTileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
		Int uiOrgNumTileRowsMinus1 = pcPPS->getTileNumRowsMinus1();
		Bool bOrgTileUniformSpacingFlag = pcPPS->getTileUniformSpacingFlag();
		std::vector<Int> OrgcolumnWidth(uiOrgNumTileColumnsMinus1);
		std::vector<Int> OrgrowHeight(uiOrgNumTileRowsMinus1);
		if (!bOrgTileUniformSpacingFlag)
		{
			for (UInt i = 0; i < uiOrgNumTileColumnsMinus1; i++)
			{
				OrgcolumnWidth[i] = pcPPS->getTileColumnWidth(i);
			}

			for (UInt i = 0; i < uiOrgNumTileRowsMinus1; i++)
			{
				OrgrowHeight[i] = pcPPS->getTileRowHeight(i);
			}
		}
		//set SliceEncodingHeader TileUniformSpacingFlag to none-uniform
		pcPPS->setTileUniformSpacingFlag(false);

		//set the SliceEncodingHeader configuration
		//Int eSliceEncodingNumSlice = pcSlice->ETRI_getNumSlices();
		Int eSliceEncodingNumSlice = ETRI_Header_NumSlices;
		Int uiNumTileColumnsMinus1 = 0; Int uiNumTileRowsMinus1 = 0;
		if (eSliceEncodingNumSlice == 6)
		{
			uiNumTileColumnsMinus1 = 7;
#if ETRI_MultiplePPS
			uiNumTileRowsMinus1 = 11;
#else
			uiNumTileRowsMinus1 = 5;
#endif
		}		
		else{
			EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
		}
		pcPPS->setNumTileColumnsMinus1(uiNumTileColumnsMinus1);
		pcPPS->setNumTileRowsMinus1(uiNumTileRowsMinus1);


		int TileColumnWidthArray[7] = { 8,7,8,7,8,7,8 };
		std::vector<Int> columnWidth(uiNumTileColumnsMinus1);
		for (Int i = 0; i< uiNumTileColumnsMinus1; i++)
		{
			columnWidth[i] = TileColumnWidthArray[i];
		}
		pcPPS->setTileColumnWidth(columnWidth);		
		if (eSliceEncodingNumSlice == 6)
		{
#if ETRI_MultiplePPS
			int TileRowHeightArray[11] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
#else
			int TileRowHeightArray[5] = {6, 6, 6, 6, 6};
#endif
			std::vector<Int> rowHeight(uiNumTileRowsMinus1);
			for (Int i = 0; i< uiNumTileRowsMinus1; i++)
			{
				rowHeight[i] = TileRowHeightArray[i];
			}
			pcPPS->setTileRowHeight(rowHeight);
		}	
		else{
			EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
		}
#endif

		nalu = NALUnit(NAL_UNIT_PPS);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cEntropyCoder.encodePPS(pcSlice->getPPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

#if ETRI_SliceEncoderHeader //set back org SPS & PPS	
		pcSlice->getSPS()->setPicWidthInLumaSamples(uiOrgPicWidthInLumaSamples);
		pcSlice->getSPS()->setPicHeightInLumaSamples(uiOrgPicHeightInLumaSamples);
#if (!ETRI_Header_NoTile )
		//set back org Tile info		
		pcPPS->setNumTileColumnsMinus1(uiOrgNumTileColumnsMinus1);
		pcPPS->setNumTileRowsMinus1(uiOrgNumTileRowsMinus1);
		pcPPS->setTileUniformSpacingFlag(bOrgTileUniformSpacingFlag);
		if (!bOrgTileUniformSpacingFlag)
		{
			pcPPS->setTileColumnWidth(OrgcolumnWidth);
			pcPPS->setTileRowHeight(OrgrowHeight);
		}
#endif
#endif
		//write PPS id 1, 2
#if ETRI_MultiplePPS		
		Int numAdditionalPPS = em_pcEncTop->ETRI_getNumAdditionalPPS();
#if (ETRI_SliceEncoderHeader && !ETRI_Header_NoTile )///< 2nd & 3rd PPS		
		for (int ppsid = 1; ppsid < numAdditionalPPS + 1; ppsid++)
		{
			TComPPS* pcPPS = em_pcEncTop->getPPS(ppsid);
			Int uiOrgNumTileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
			Int uiOrgNumTileRowsMinus1 = pcPPS->getTileNumRowsMinus1();
			Bool bOrgTileUniformSpacingFlag = pcPPS->getTileUniformSpacingFlag();
			std::vector<Int> OrgcolumnWidth(uiOrgNumTileColumnsMinus1);
			std::vector<Int> OrgrowHeight(uiOrgNumTileRowsMinus1);
			if (!bOrgTileUniformSpacingFlag)
			{
				for (UInt i = 0; i < uiOrgNumTileColumnsMinus1; i++)
				{
					OrgcolumnWidth[i] = pcPPS->getTileColumnWidth(i);
				}

				for (UInt i = 0; i < uiOrgNumTileRowsMinus1; i++)
				{
					OrgrowHeight[i] = pcPPS->getTileRowHeight(i);
				}
			}
			//set SliceEncodingHeader TileUniformSpacingFlag to none-uniform
			pcPPS->setTileUniformSpacingFlag(false);

			//set the SliceEncodingHeader configuration			
			Int eSliceEncodingNumSlice = ETRI_Header_NumSlices;
			Int uiNumTileColumnsMinus1 = 0; Int uiNumTileRowsMinus1 = 0;
			if (eSliceEncodingNumSlice == 6)
			{
				if (ppsid == 1)
				{
					uiNumTileColumnsMinus1 = 7;
					uiNumTileRowsMinus1 = 5;
				}
				else if (ppsid == 2)
				{
					uiNumTileColumnsMinus1 = 3;
					uiNumTileRowsMinus1 = 5;
				}
			}
			else{
				EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
			}
			pcPPS->setNumTileColumnsMinus1(uiNumTileColumnsMinus1);
			pcPPS->setNumTileRowsMinus1(uiNumTileRowsMinus1);

			int TileColumnWidthArray_1[7] = { 8, 7, 8, 7, 8, 7, 8 };
			int TileColumnWidthArray_2[7] = { 15, 15, 15 };

			std::vector<Int> columnWidth(uiNumTileColumnsMinus1);
			for (Int i = 0; i < uiNumTileColumnsMinus1; i++)
			{
				if (ppsid == 1)
					columnWidth[i] = TileColumnWidthArray_1[i];
				else if (ppsid == 2)
					columnWidth[i] = TileColumnWidthArray_2[i];
			}
			pcPPS->setTileColumnWidth(columnWidth);

			if (eSliceEncodingNumSlice == 6)
			{

				int TileRowHeightArray2[5] = { 6, 6, 6, 6, 6 };

				std::vector<Int> rowHeight(uiNumTileRowsMinus1);
				for (Int i = 0; i < uiNumTileRowsMinus1; i++)
				{
					rowHeight[i] = TileRowHeightArray2[i];
				}
				pcPPS->setTileRowHeight(rowHeight);
			}
			else{
				EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
			}

			//write pps
			nalu = NALUnit(NAL_UNIT_PPS);
			em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
			em_cEntropyCoder.encodePPS(em_pcEncTop->getPPS(ppsid));
			writeRBSPTrailingBits(nalu.m_Bitstream);
			accessUnit.push_back(new NALUnitEBSP(nalu));
			actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

#if (!ETRI_Header_NoTile )
			//set back org Tile info		
			pcPPS->setNumTileColumnsMinus1(uiOrgNumTileColumnsMinus1);
			pcPPS->setNumTileRowsMinus1(uiOrgNumTileRowsMinus1);
			pcPPS->setTileUniformSpacingFlag(bOrgTileUniformSpacingFlag);
			if (!bOrgTileUniformSpacingFlag)
			{
				pcPPS->setTileColumnWidth(OrgcolumnWidth);
				pcPPS->setTileRowHeight(OrgrowHeight);
			}
#endif
		}

#else
		for (int ppsid = 1; ppsid < numAdditionalPPS +1; ppsid++)
		{
			nalu = NALUnit(NAL_UNIT_PPS);
			em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
			em_cEntropyCoder.encodePPS(em_pcEncTop->getPPS(ppsid));			
			writeRBSPTrailingBits(nalu.m_Bitstream);
			accessUnit.push_back(new NALUnitEBSP(nalu));
			actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
		}
#endif
#endif // end of ETRI_MultiplePPS
		ETRI_xCreateLeadingSEIMessages(accessUnit, pcSlice->getSPS());
	}
#else // else of ETRI_MULTITHREAD_2 

	// Set entropy coder
	em_pcEntropyCoder->setEntropyCoder	( em_pcCavlcCoder, pcSlice );

#if ETRI_SliceEncoderHeader
	if (em_bFirst)
	{
		short eSliceEncoder_SliceIdx = pcSlice->ETRI_getSliceIndex();
		if(eSliceEncoder_SliceIdx != 0)		
			em_bFirst = false;
	}
#endif

	/* write various header sets. */
	if (em_bFirst)
	{
		OutputNALUnit nalu(NAL_UNIT_VPS);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
		em_pcEntropyCoder->encodeVPS(em_pcEncTop->getVPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

		nalu = NALUnit(NAL_UNIT_SPS);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

//		if (em_bFirst)
//		{
			UInt	e_numLongTermRefPicSPS   	= em_pcGOPEncoder->ETRI_getnumLongTermRefPicSPS();
			UInt*	e_ltRefPicPocLsbSps  		= em_pcGOPEncoder->ETRI_getltRefPicPocLsbSps();			///Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok
			Bool*	e_ltRefPicUsedByCurrPicFlag	= em_pcGOPEncoder->ETRI_getltRefPicUsedByCurrPicFlag();	///Total Number of Ref Pic Order is 33. Look at the TEncGOP.h  @2015 5 24 by Seok

			pcSlice->getSPS()->setNumLongTermRefPicSPS(e_numLongTermRefPicSPS);
			for (Int k = 0; k < e_numLongTermRefPicSPS; k++)
			{
				pcSlice->getSPS()->setLtRefPicPocLsbSps(k, e_ltRefPicPocLsbSps[k]);
				pcSlice->getSPS()->setUsedByCurrPicLtSPSFlag(k, e_ltRefPicUsedByCurrPicFlag[k]);
			}
//		}

		if( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() )
		{
			UInt maxCU = em_pcEncTop->getSliceArgument() >> ( pcSlice->getSPS()->getMaxCUDepth() << 1);
			UInt numDU = ( em_pcEncTop->getSliceMode() == 1 ) ? ( pcPic->getNumCUsInFrame() / maxCU ) : ( 0 );
			if( pcPic->getNumCUsInFrame() % maxCU != 0 || numDU == 0 )
			{
				numDU ++;
			}
			pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->setNumDU( numDU );
			pcSlice->getSPS()->setHrdParameters( em_pcEncTop->getFrameRate(), numDU, em_pcEncTop->getTargetBitrate(), ( em_pcEncTop->getIntraPeriod() > 0 ) );
		}

		if( em_pcEncTop->getBufferingPeriodSEIEnabled() || em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() )
		{
			pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
		}

#if ETRI_Header_FirstSlice  ///< SPS for etri slice encoding
		UInt uiOrgPicWidthInLumaSamples = pcSlice->getSPS()->getPicWidthInLumaSamples();
		UInt uiOrgPicHeightInLumaSamples = pcSlice->getSPS()->getPicHeightInLumaSamples();
		UInt uiPicWidthInLumaSamples = ETRI_Header_ImgYWidth;
		UInt uiPicHeightInLumaSamples = ETRI_Header_ImgYHeight;
		pcSlice->getSPS()->setPicWidthInLumaSamples(uiPicWidthInLumaSamples);
		pcSlice->getSPS()->setPicHeightInLumaSamples(uiPicHeightInLumaSamples);
#endif

		em_pcEntropyCoder->encodeSPS(pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

#if (ETRI_SliceEncoderHeader && !ETRI_Header_NoTile )///< PPS		
		//Copy the org configuration
		TComPPS* pcPPS = pcSlice->getPPS();
		Int uiOrgNumTileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
		Int uiOrgNumTileRowsMinus1 = pcPPS->getTileNumRowsMinus1();
		Bool bOrgTileUniformSpacingFlag = pcPPS->getTileUniformSpacingFlag();
		std::vector<Int> OrgcolumnWidth(uiOrgNumTileColumnsMinus1);
		std::vector<Int> OrgrowHeight(uiOrgNumTileRowsMinus1);
		if (!bOrgTileUniformSpacingFlag)
		{			
			for (UInt i = 0; i < uiOrgNumTileColumnsMinus1; i++)
			{
				OrgcolumnWidth[i] = pcPPS->getTileColumnWidth(i);
			}
			
			for (UInt i = 0; i < uiOrgNumTileRowsMinus1; i++)
			{
				OrgrowHeight[i] = pcPPS->getTileRowHeight(i);
			}
		}
		//set SliceEncodingHeader TileUniformSpacingFlag to none-uniform
		pcPPS->setTileUniformSpacingFlag(false);

		//set the SliceEncodingHeader configuration
		Int eSliceEncodingNumSlice = pcSlice->ETRI_getNumSlices();
		Int uiNumTileColumnsMinus1 = 0; Int uiNumTileRowsMinus1 = 0;
		if (eSliceEncodingNumSlice == 8)
		{
			uiNumTileColumnsMinus1 = 9;
			uiNumTileRowsMinus1 = 7;
		}
		else if (eSliceEncodingNumSlice == 2)
		{
			uiNumTileColumnsMinus1 = 9;
			uiNumTileRowsMinus1 = 1;
		}
		else{
			EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
		}		
		pcPPS->setNumTileColumnsMinus1(uiNumTileColumnsMinus1);
		pcPPS->setNumTileRowsMinus1(uiNumTileRowsMinus1);

		//TileColumnWidthArray: case of UniformSpacingFlag ==false [6 6 6 6 6 6 6 6 6  ] 
		int TileColumnWidthArray[9] = { 6, 6, 6, 6, 6, 6, 6, 6, 6 };
		std::vector<Int> columnWidth(uiNumTileColumnsMinus1);
		for (Int i = 0; i< uiNumTileColumnsMinus1; i++)
		{
			columnWidth[i] = TileColumnWidthArray[i];
		}
		pcPPS->setTileColumnWidth(columnWidth);
		//TileRowHeight: case of UniformSpacingFlag ==false [5 4 4 4 4 4 4  ] 		
		if (eSliceEncodingNumSlice == 8)
		{
			int TileRowHeightArray[7] = { 5, 4, 4, 4, 4, 4, 4 };
			std::vector<Int> rowHeight(uiNumTileRowsMinus1);
			for (Int i = 0; i< uiNumTileRowsMinus1; i++)
			{
				rowHeight[i] = TileRowHeightArray[i];
			}
			pcPPS->setTileRowHeight(rowHeight);
		}
		else if (eSliceEncodingNumSlice == 2)
		{
			int TileRowHeightArray[1] = { 17 };
			std::vector<Int> rowHeight(uiNumTileRowsMinus1);
			for (Int i = 0; i< uiNumTileRowsMinus1; i++)
			{
				rowHeight[i] = TileRowHeightArray[i];
			}
			pcPPS->setTileRowHeight(rowHeight);
		}
		else{
			EDPRINTF(stderr, "Not valid numbers of slice from PPS writing\n");
		}
#endif

		nalu = NALUnit(NAL_UNIT_PPS);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
		em_pcEntropyCoder->encodePPS(pcSlice->getPPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

		em_pcGOPEncoder->xCreateLeadingSEIMessages(accessUnit, pcSlice->getSPS());

#if ETRI_Header_FirstSlice //set back org SPS & PPS
		pcSlice->getSPS()->setPicWidthInLumaSamples(uiOrgPicWidthInLumaSamples);
		pcSlice->getSPS()->setPicHeightInLumaSamples(uiOrgPicHeightInLumaSamples);
#if (!ETRI_Header_NoTile )
		//set back org Tile info		
		pcSlice->getPPS()->setNumTileColumnsMinus1(uiOrgNumTileColumnsMinus1);
		pcSlice->getPPS()->setNumTileRowsMinus1(uiOrgNumTileRowsMinus1);
		pcSlice->getPPS()->setTileUniformSpacingFlag(bOrgTileUniformSpacingFlag);
		if (!bOrgTileUniformSpacingFlag)		
		{
			pcSlice->getPPS()->setTileColumnWidth(OrgcolumnWidth);
			pcSlice->getPPS()->setTileRowHeight(OrgrowHeight);
		}
#endif
#endif 

//		em_pcGOPEncoder->ETRI_getbFirst() = false;
	}
#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Writeout SOP in SEI at the First part of GOP.  
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_WriteSOPDescriptionInSEI(Int iGOPid, Int pocCurr, TComSlice*& pcSlice, AccessUnit& accessUnit, Bool& writeSOP, Bool isField)
{

#if ETRI_MULTITHREAD_2
	if (writeSOP) // write SOP description SEI (if enabled) at the beginning of GOP
	{
		Int SOPcurrPOC = pocCurr;

		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		SEISOPDescription SOPDescriptionSEI;
		SOPDescriptionSEI.m_sopSeqParameterSetId = pcSlice->getSPS()->getSPSId();

		UInt i = 0;
		UInt prevEntryId = iGOPid;
		for (UInt j = iGOPid; j < em_iGopSize; j++)
		{
			Int deltaPOC = em_pcEncTop->getGOPEntry(j).m_POC - em_pcEncTop->getGOPEntry(prevEntryId).m_POC;
			if ((SOPcurrPOC + deltaPOC) < em_pcEncTop->getFramesToBeEncoded())
			{
				SOPcurrPOC += deltaPOC;
				SOPDescriptionSEI.m_sopDescVclNaluType[i] = em_pcGOPEncoder->getNalUnitType(SOPcurrPOC, em_iLastIDR, isField);
				SOPDescriptionSEI.m_sopDescTemporalId[i] = em_pcEncTop->getGOPEntry(j).m_temporalId;
				SOPDescriptionSEI.m_sopDescStRpsIdx[i] = em_pcEncTop->getReferencePictureSetIdxForSOP(pcSlice, SOPcurrPOC, j);
				SOPDescriptionSEI.m_sopDescPocDelta[i] = deltaPOC;

				prevEntryId = j;
				i++;
			}
		}

		SOPDescriptionSEI.m_numPicsInSopMinus1 = i - 1;

		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, SOPDescriptionSEI, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));

		writeSOP = false;
	}
#else
	if (writeSOP) // write SOP description SEI (if enabled) at the beginning of GOP
	{
		Int SOPcurrPOC = pocCurr;

		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

		SEISOPDescription SOPDescriptionSEI;
		SOPDescriptionSEI.m_sopSeqParameterSetId = pcSlice->getSPS()->getSPSId();

		UInt i = 0;
		UInt prevEntryId = iGOPid;
		for (UInt j = iGOPid; j < em_iGopSize; j++)
		{
			Int deltaPOC = em_pcEncTop->getGOPEntry(j).m_POC - em_pcEncTop->getGOPEntry(prevEntryId).m_POC;
			if ((SOPcurrPOC + deltaPOC) < em_pcEncTop->getFramesToBeEncoded())
			{
				SOPcurrPOC += deltaPOC;
				SOPDescriptionSEI.m_sopDescVclNaluType[i] = em_pcGOPEncoder->getNalUnitType(SOPcurrPOC, em_iLastIDR, isField);
				SOPDescriptionSEI.m_sopDescTemporalId[i] = em_pcEncTop->getGOPEntry(j).m_temporalId;
				SOPDescriptionSEI.m_sopDescStRpsIdx[i] = em_pcEncTop->getReferencePictureSetIdxForSOP(pcSlice, SOPcurrPOC, j);
				SOPDescriptionSEI.m_sopDescPocDelta[i] = deltaPOC;

				prevEntryId = j;
				i++;
			}
		}

		SOPDescriptionSEI.m_numPicsInSopMinus1 = i - 1;

		em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, SOPDescriptionSEI, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));

		writeSOP = false;
	}
#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set Picture Timing Information in SEI 
			CpbRemovalDelay is Evaluate Here Correctly. Look at the code about pictureTimingSEI
			This code can hinder the Frame Parallelism, if this information is write out to all header of pictures.
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_setPictureTimingSEI(TComSlice*& pcSlice, SEIPictureTiming& pictureTimingSEI, Int IRAPGOPid, ETRI_SliceInfo& SliceInfo)
{
	if( ( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() ) &&
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
			if( SliceInfo.accumBitsDU == NULL )
			{
				SliceInfo.accumBitsDU                                  = new UInt[ numDU ];
			}
			if( SliceInfo.accumNalsDU == NULL )
			{
				SliceInfo.accumNalsDU                                  = new UInt[ numDU ];
			}
		}

		UInt 	e_totalCoded	= em_pcGOPEncoder->ETRI_gettotalCoded();
		UInt  	e_lastBPSEI  	= em_pcGOPEncoder->ETRI_getlastBPSEI();

		pictureTimingSEI.m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, e_totalCoded - e_lastBPSEI), static_cast<Int>(pow(2, static_cast<double>(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
		pictureTimingSEI.m_picDpbOutputDelay = pcSlice->getSPS()->getNumReorderPics(pcSlice->getSPS()->getMaxTLayers()-1) + pcSlice->getPOC() - e_totalCoded;
#if EFFICIENT_FIELD_IRAP
		// if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
		if(IRAPGOPid > 0 && IRAPGOPid < em_iGopSize)
		{
			pictureTimingSEI.m_picDpbOutputDelay ++;
		}
#endif
		Int factor = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2;
		pictureTimingSEI.m_picDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
		if( em_pcEncTop->getDecodingUnitInfoSEIEnabled() )
		{
			*SliceInfo.picSptDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
		}
	}

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set HRD Information in VUI
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_writeHRDInfo(TComSlice*& pcSlice, AccessUnit& accessUnit, SEIScalableNesting& scalableNestingSEI)
{
	UInt j;

#if ETRI_MULTITHREAD_2
	if( ( em_pcEncTop->getBufferingPeriodSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) &&
		( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
		( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
		|| ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
	{
		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
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
		em_cseiWriter.writeSEImessage( nalu.m_Bitstream, sei_buffering_period, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);

		Bool e_activeParameterSetSEIPresentInAU = ETRI_getactiveParameterSetSEIPresentInAU();
		Bool e_pictureTimingSEIPresentInAU = ETRI_getpictureTimingSEIPresentInAU();
		{
			UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
			UInt offsetPosition = e_activeParameterSetSEIPresentInAU;   // Insert BP SEI after APS SEI
			AccessUnit::iterator it;
			for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
			{
				it++;
			}
			accessUnit.insert(it, new NALUnitEBSP(nalu));
			ETRI_getbufferingPeriodSEIPresentInAU() = true;
		}

		if (em_pcEncTop->getScalableNestingSEIEnabled())
		{
			OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
			em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
			em_cEntropyCoder.setBitstream(&naluTmp.m_Bitstream);
			scalableNestingSEI.m_nestedSEIs.clear();
			scalableNestingSEI.m_nestedSEIs.push_back(&sei_buffering_period);
			em_cseiWriter.writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
			writeRBSPTrailingBits(naluTmp.m_Bitstream);
			UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
			UInt offsetPosition = e_activeParameterSetSEIPresentInAU + ETRI_getbufferingPeriodSEIPresentInAU() + e_pictureTimingSEIPresentInAU;   // Insert BP SEI after non-nested APS, BP and PT SEIs
			AccessUnit::iterator it;
			for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
			{
				it++;
			}
			accessUnit.insert(it, new NALUnitEBSP(naluTmp));
			ETRI_getnestedBufferingPeriodSEIPresentInAU() = true;
		}
		em_pcGOPEncoder->ETRI_getlastBPSEI() = em_pcGOPEncoder->ETRI_gettotalCoded();	///In TEncGOP : m_lastBPSEI = m_totalCoded @ 2015 5 24 by Seok
		em_cpbRemovalDelay = 0;
	}
	em_cpbRemovalDelay += 1;
#else

	if( ( em_pcEncTop->getBufferingPeriodSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) &&
	( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
	( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
	|| ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
	{
		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

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

		em_pcseiWriter->writeSEImessage( nalu.m_Bitstream, sei_buffering_period, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);

		Bool e_activeParameterSetSEIPresentInAU = em_pcGOPEncoder->ETRI_getactiveParameterSetSEIPresentInAU();
		Bool e_pictureTimingSEIPresentInAU = em_pcGOPEncoder->ETRI_getpictureTimingSEIPresentInAU();
		{
			UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
			UInt offsetPosition = e_activeParameterSetSEIPresentInAU;   // Insert BP SEI after APS SEI
			AccessUnit::iterator it;
			for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
			{
				it++;
			}
			accessUnit.insert(it, new NALUnitEBSP(nalu));
			em_pcGOPEncoder->ETRI_getbufferingPeriodSEIPresentInAU() = true;
		}

		if (em_pcEncTop->getScalableNestingSEIEnabled())
		{
			OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
			em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
			em_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
			scalableNestingSEI.m_nestedSEIs.clear();
			scalableNestingSEI.m_nestedSEIs.push_back(&sei_buffering_period);
			em_pcseiWriter->writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
			writeRBSPTrailingBits(naluTmp.m_Bitstream);
			UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
			UInt offsetPosition = e_activeParameterSetSEIPresentInAU + em_pcGOPEncoder->ETRI_getbufferingPeriodSEIPresentInAU() + e_pictureTimingSEIPresentInAU;   // Insert BP SEI after non-nested APS, BP and PT SEIs
			AccessUnit::iterator it;
			for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
			{
				it++;
			}
			accessUnit.insert(it, new NALUnitEBSP(naluTmp));
			em_pcGOPEncoder->ETRI_getnestedBufferingPeriodSEIPresentInAU() = true;
		}
		em_pcGOPEncoder->ETRI_getlastBPSEI() = em_pcGOPEncoder->ETRI_gettotalCoded();	///In TEncGOP : m_lastBPSEI = m_totalCoded @ 2015 5 24 by Seok
		em_cpbRemovalDelay = 0;
	}
	em_cpbRemovalDelay += 1;

#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Gradual decoding refresh SEI  and Ready for Write out Slice
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_Ready4WriteSlice(TComPic* pcPic,  Int pocCurr, TComSlice*& pcSlice, AccessUnit& accessUnit, ETRI_SliceInfo& SliceInfo)
{

#if ETRI_MULTITHREAD_2

	if( ( em_pcEncTop->getRecoveryPointSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) )
	{
		if( em_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !pcSlice->getRapPicFlag() )
		{
			// Gradual decoding refresh SEI
			OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
			em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
			em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
			SEIGradualDecodingRefreshInfo seiGradualDecodingRefreshInfo;
			seiGradualDecodingRefreshInfo.m_gdrForegroundFlag = true; // Indicating all "foreground"
			em_cseiWriter.writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, pcSlice->getSPS() );
			writeRBSPTrailingBits(nalu.m_Bitstream);
			accessUnit.push_back(new NALUnitEBSP(nalu));
		}
		// Recovery point SEI
		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		SEIRecoveryPoint sei_recovery_point;
		sei_recovery_point.m_recoveryPocCnt    = 0;
		sei_recovery_point.m_exactMatchingFlag = ( pcSlice->getPOC() == 0 ) ? (true) : (false);
		sei_recovery_point.m_brokenLinkFlag    = false;
#if ALLOW_RECOVERY_POINT_AS_RAP
		if(em_pcEncTop->getDecodingRefreshType() == 3)
		{
			em_pcGOPEncoder->ETRI_getiLastRecoveryPicPOC() = pocCurr;
		}
#endif
		em_cseiWriter.writeSEImessage( nalu.m_Bitstream, sei_recovery_point, pcSlice->getSPS() );
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
	}

	/* use the main bitstream buffer for storing the marshalled picture */
	em_cEntropyCoder.setBitstream(NULL);

	*SliceInfo.startCUAddrSliceIdx = 0;
	*SliceInfo.startCUAddrSlice    = 0;

	*SliceInfo.startCUAddrSliceSegmentIdx = 0;
	*SliceInfo.startCUAddrSliceSegment    = 0;
	*SliceInfo.nextCUAddr                 = 0;
	pcSlice = pcPic->getSlice(*SliceInfo.startCUAddrSliceIdx);
#else
	if ((em_pcEncTop->getRecoveryPointSEIEnabled()) && (pcSlice->getSliceType() == I_SLICE))
	{
		if( em_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !pcSlice->getRapPicFlag() )
		{
			// Gradual decoding refresh SEI
			OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
			em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
			em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

			SEIGradualDecodingRefreshInfo seiGradualDecodingRefreshInfo;
			seiGradualDecodingRefreshInfo.m_gdrForegroundFlag = true; // Indicating all "foreground"

			em_pcseiWriter->writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, pcSlice->getSPS() );
			writeRBSPTrailingBits(nalu.m_Bitstream);
			accessUnit.push_back(new NALUnitEBSP(nalu));
		}
		// Recovery point SEI
		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

		SEIRecoveryPoint sei_recovery_point;
		sei_recovery_point.m_recoveryPocCnt    = 0;
		sei_recovery_point.m_exactMatchingFlag = ( pcSlice->getPOC() == 0 ) ? (true) : (false);
		sei_recovery_point.m_brokenLinkFlag    = false;
#if ALLOW_RECOVERY_POINT_AS_RAP
		if(em_pcEncTop->getDecodingRefreshType() == 3)
		{
			em_pcGOPEncoder->ETRI_getiLastRecoveryPicPOC() = pocCurr;
		}
#endif
		em_pcseiWriter->writeSEImessage( nalu.m_Bitstream, sei_recovery_point, pcSlice->getSPS() );
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
	}

	/* use the main bitstream buffer for storing the marshalled picture */
	em_pcEntropyCoder->setBitstream(NULL);

	*SliceInfo.startCUAddrSliceIdx = 0;
	*SliceInfo.startCUAddrSlice    = 0;

	*SliceInfo.startCUAddrSliceSegmentIdx = 0;
	*SliceInfo.startCUAddrSliceSegment    = 0;
	*SliceInfo.nextCUAddr                 = 0;
	pcSlice = pcPic->getSlice(*SliceInfo.startCUAddrSliceIdx);
#endif
}



/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Slice Data Re-Initilization for Encode Slice, Especially, SetRPS is conducted in this function. 
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_ReInitSliceData(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue)
{
	UInt* 	nextCUAddr 			= ReturnValue.nextCUAddr;
	UInt*	startCUAddrSliceIdx	= ReturnValue.startCUAddrSliceIdx;
	UInt*	startCUAddrSliceSegmentIdx = ReturnValue.startCUAddrSliceSegmentIdx;

	pcSlice->setNextSlice(false);
	pcSlice->setNextSliceSegment(false);

#if ETRI_MULTITHREAD_2
	if (*nextCUAddr == em_cstoredStartCUAddrForEncodingSlice[*startCUAddrSliceIdx])
	{
		pcSlice = pcPic->getSlice(*startCUAddrSliceIdx);
		if(*startCUAddrSliceIdx > 0 && pcSlice->getSliceType()!= I_SLICE)
		{
			pcSlice->checkColRefIdx(*startCUAddrSliceIdx, pcPic);
		}
		pcPic->setCurrSliceIdx(*startCUAddrSliceIdx);
		em_cSliceEncoder.setSliceIdx(*startCUAddrSliceIdx);
		assert(*startCUAddrSliceIdx == pcSlice->getSliceIdx());
		// Reconstruction slice
		pcSlice->setSliceCurStartCUAddr( *nextCUAddr );	// to be used in encodeSlice() + context restriction
		pcSlice->setSliceCurEndCUAddr  ( em_cstoredStartCUAddrForEncodingSlice[*startCUAddrSliceIdx + 1 ] );
		// Dependent slice
		pcSlice->setSliceSegmentCurStartCUAddr( *nextCUAddr );  // to be used in encodeSlice() + context restriction
		pcSlice->setSliceSegmentCurEndCUAddr  ( em_cstoredStartCUAddrForEncodingSliceSegment[*startCUAddrSliceSegmentIdx + 1 ] );
		pcSlice->setNextSlice		( true );

		(*startCUAddrSliceIdx)++;
		(*startCUAddrSliceSegmentIdx)++;
	}
	else if (*nextCUAddr == em_cstoredStartCUAddrForEncodingSliceSegment[*startCUAddrSliceSegmentIdx])
	{
		// Dependent slice
		pcSlice->setSliceSegmentCurStartCUAddr( *nextCUAddr );  // to be used in encodeSlice() + context restriction
		pcSlice->setSliceSegmentCurEndCUAddr  ( em_cstoredStartCUAddrForEncodingSliceSegment[*startCUAddrSliceSegmentIdx+1 ] );
		pcSlice->setNextSliceSegment( true );

		(*startCUAddrSliceSegmentIdx)++;
	}

	pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
	pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());
#else

	if (*nextCUAddr == (*em_storedStartCUAddrForEncodingSlice)[*startCUAddrSliceIdx])
	{
		pcSlice = pcPic->getSlice(*startCUAddrSliceIdx);
		if(*startCUAddrSliceIdx > 0 && pcSlice->getSliceType()!= I_SLICE)
		{
		  pcSlice->checkColRefIdx(*startCUAddrSliceIdx, pcPic);
		}
		pcPic->setCurrSliceIdx(*startCUAddrSliceIdx);
		em_pcSliceEncoder->setSliceIdx(*startCUAddrSliceIdx);
		assert(*startCUAddrSliceIdx == pcSlice->getSliceIdx());
		// Reconstruction slice
		pcSlice->setSliceCurStartCUAddr( *nextCUAddr );	// to be used in encodeSlice() + context restriction
		pcSlice->setSliceCurEndCUAddr  ( (*em_storedStartCUAddrForEncodingSlice)[*startCUAddrSliceIdx + 1 ] );
		// Dependent slice
		pcSlice->setSliceSegmentCurStartCUAddr( *nextCUAddr );  // to be used in encodeSlice() + context restriction
		pcSlice->setSliceSegmentCurEndCUAddr  ( (*em_storedStartCUAddrForEncodingSliceSegment)[*startCUAddrSliceSegmentIdx + 1 ] );
		
		pcSlice->setNextSlice		( true );
		
		(*startCUAddrSliceIdx)++;
		(*startCUAddrSliceSegmentIdx)++;
	  }
	  else if (*nextCUAddr == (*em_storedStartCUAddrForEncodingSliceSegment)[*startCUAddrSliceSegmentIdx])
	  {
		// Dependent slice
		pcSlice->setSliceSegmentCurStartCUAddr( *nextCUAddr );  // to be used in encodeSlice() + context restriction
		pcSlice->setSliceSegmentCurEndCUAddr  ( (*em_storedStartCUAddrForEncodingSliceSegment)[*startCUAddrSliceSegmentIdx+1 ] );
		
		pcSlice->setNextSliceSegment( true );
		
		(*startCUAddrSliceSegmentIdx)++;
	}

	pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
	pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());
#endif

}




/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Reset Slice Boundary Data. 
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_ResetSliceBoundaryData(TComPic* pcPic, TComSlice*& pcSlice, Bool& skippedSlice, Bool& bRtValue, ETRI_SliceInfo& ReturnValue)
{
	  UInt uiDummyStartCUAddr, uiDummyBoundingCUAddr;

	  UInt* uiInternalAddress		  	= ReturnValue.uiInternalAddress;
	  UInt* uiExternalAddress  			= ReturnValue.uiExternalAddress;
	  UInt* uiPosX					  	= ReturnValue.uiPosX;
	  UInt* uiPosY					 	= ReturnValue.uiPosY;
	  UInt* uiWidth 			  		= ReturnValue.uiWidth;
	  UInt* uiHeight			  		= ReturnValue.uiHeight;
	  UInt* nextCUAddr  				= ReturnValue.nextCUAddr;
	  UInt* startCUAddrSliceIdx 	  	= ReturnValue.startCUAddrSliceIdx;
	  UInt* startCUAddrSliceSegmentIdx 	= ReturnValue.startCUAddrSliceSegmentIdx;

#if ETRI_MULTITHREAD_2
	em_cSliceEncoder.xDetermineStartAndBoundingCUAddr(uiDummyStartCUAddr,uiDummyBoundingCUAddr,pcPic,true);
#else
	em_pcSliceEncoder->xDetermineStartAndBoundingCUAddr(uiDummyStartCUAddr, uiDummyBoundingCUAddr, pcPic, true);
#endif

	  *uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
	  *uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
	  *uiPosX = ( *uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[*uiInternalAddress] ];
	  *uiPosY = ( *uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[*uiInternalAddress] ];
	  *uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
	  *uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();

	  while(*uiPosX >= *uiWidth||*uiPosY >= *uiHeight)
	  {
		(*uiInternalAddress)--;
		*uiPosX = ( *uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[*uiInternalAddress] ];
		*uiPosY = ( *uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[*uiInternalAddress] ];
	  }

	  (*uiInternalAddress)++;
	  if(*uiInternalAddress==pcPic->getNumPartInCU())
	  {
		*uiInternalAddress = 0;
		*uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(*uiExternalAddress)+1);
	  }

	  UInt endAddress = pcPic->getPicSym()->getPicSCUEncOrder(*uiExternalAddress * pcPic->getNumPartInCU()+ *uiInternalAddress);
	  if(endAddress<=pcSlice->getSliceSegmentCurStartCUAddr())
	  {
		UInt boundingAddrSlice, boundingAddrSliceSegment;
#if ETRI_MULTITHREAD_2
		boundingAddrSlice		   	= em_cstoredStartCUAddrForEncodingSlice[*startCUAddrSliceIdx];
		boundingAddrSliceSegment 	= em_cstoredStartCUAddrForEncodingSliceSegment[*startCUAddrSliceSegmentIdx];
#else
		boundingAddrSlice = (*em_storedStartCUAddrForEncodingSlice)[*startCUAddrSliceIdx];
		boundingAddrSliceSegment = (*em_storedStartCUAddrForEncodingSliceSegment)[*startCUAddrSliceSegmentIdx];
#endif
		*nextCUAddr = min(boundingAddrSlice, boundingAddrSliceSegment);
		if (pcSlice->isNextSlice())
		{
		  skippedSlice=true;
		}
		bRtValue = true;	return;
	  }

	  if(skippedSlice)
	  {
		pcSlice->setNextSlice		( true );
		pcSlice->setNextSliceSegment( false );
	  }
	  skippedSlice=false;

	bRtValue = false; 	/// return Value
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set Slice Encoder. Set Substream, CAVLC, CABAC Coder and Bitstream for Slice
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_SetSliceEncoder (TComPic* pcPic, TComSlice*& pcSlice, TComOutputBitstream* pcSubstreamsOut, TComOutputBitstream*& pcBitstreamRedirect, 
									TEncSbac* pcSbacCoders,  OutputNALUnit& nalu, ETRI_SliceInfo& ReturnValue)
{
	Int*  	iNumSubstreams  			= ReturnValue.iNumSubstreams;
	Int*  	tmpBitsBeforeWriting		= ReturnValue.tmpBitsBeforeWriting;
	Int*  	actualHeadBits  			= ReturnValue.actualHeadBits;
	UInt* 	uiOneBitstreamPerSliceLength= ReturnValue.uiOneBitstreamPerSliceLength;


	pcSlice->allocSubstreamSizes( *iNumSubstreams );
	for ( UInt ui = 0 ; ui < *iNumSubstreams; ui++ )
	{
		pcSubstreamsOut[ui].clear();
	}
#if ETRI_MULTITHREAD_2
	em_cEntropyCoder.setEntropyCoder   ( &em_cCavlcCoder, pcSlice );
	em_cEntropyCoder.resetEntropy	  ();
	/* start slice NALunit */
	//			OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
	Bool sliceSegment = (!pcSlice->isNextSlice());
	if (!sliceSegment)
	{
		*uiOneBitstreamPerSliceLength = 0; // start of a new slice
	}
	em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);

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
		if (! em_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
		{
			if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
				pcSlice->setNoOutputPriorPicsFlag(true);
		}
	}
#endif

	*tmpBitsBeforeWriting = em_cEntropyCoder.getNumberOfWrittenBits();
	em_cEntropyCoder.encodeSliceHeader(pcSlice);
	*actualHeadBits += ( em_cEntropyCoder.getNumberOfWrittenBits() - *tmpBitsBeforeWriting );

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
#if !ETRI_Remove_Redundant_EntoropyLoader	
		em_cSbacCoder.init( &em_cBinCoderCABAC );
		em_cEntropyCoder.setEntropyCoder ( &em_cSbacCoder, pcSlice );
		em_cEntropyCoder.resetEntropy	  ();
		for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
		{
			em_cEntropyCoder.setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
			em_cEntropyCoder.resetEntropy	();
		}
#endif
	}

	if(pcSlice->isNextSlice())
	{
		// set entropy coder for writing
		em_cSbacCoder.init( &em_cBinCoderCABAC );
		{
			for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
			{
				em_cEntropyCoder.setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
				em_cEntropyCoder.resetEntropy	  ();
			}
			pcSbacCoders[0].load(&em_cSbacCoder);
			em_cEntropyCoder.setEntropyCoder ( &pcSbacCoders[0], pcSlice );	//ALF is written in substream #0 with CABAC coder #0 (see ALF param encoding below)
		}
		em_cEntropyCoder.resetEntropy	  ();

		// File writing
#if !ETRI_Remove_Redundant_EntoropyLoader	
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
#endif
		// for now, override the TILES_DECODER setting in order to write substreams.
		em_cEntropyCoder.setBitstream	  ( &pcSubstreamsOut[0] );
	}
#else
	em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
	em_pcEntropyCoder->resetEntropy();
	/* start slice NALunit */
	//			OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
	Bool sliceSegment = (!pcSlice->isNextSlice());
	if (!sliceSegment)
	{
		*uiOneBitstreamPerSliceLength = 0; // start of a new slice
	}
	em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

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
		if (! em_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
		{
			if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
			pcSlice->setNoOutputPriorPicsFlag(true);
		}
	}
#endif

	*tmpBitsBeforeWriting = em_pcEntropyCoder->getNumberOfWrittenBits();
	em_pcEntropyCoder->encodeSliceHeader(pcSlice);
	*actualHeadBits += ( em_pcEntropyCoder->getNumberOfWrittenBits() - *tmpBitsBeforeWriting );

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
#if !ETRI_Remove_Redundant_EntoropyLoader	///<만일 Slice 병렬화에서도 이 부분이 필요 없다면  ! ETRI_Remove_Redundant_EntoropyLoader 하여 해당 코드를 삭제한다. @2015 5 12 by Seok
		em_pcSbacCoder->init( (TEncBinIf*)em_pcBinCABAC );
		em_pcEntropyCoder->setEntropyCoder ( em_pcSbacCoder, pcSlice );
		em_pcEntropyCoder->resetEntropy	  ();
		for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
		{
			em_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
			em_pcEntropyCoder->resetEntropy	();
		}
#endif
	}

	if(pcSlice->isNextSlice())
	{
		// set entropy coder for writing
		em_pcSbacCoder->init( (TEncBinIf*)em_pcBinCABAC );
		{
		for ( UInt ui = 0 ; ui < pcSlice->getPPS()->getNumSubstreams() ; ui++ )
		{
			em_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
			em_pcEntropyCoder->resetEntropy	  ();
		}
		pcSbacCoders[0].load(em_pcSbacCoder);
		em_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[0], pcSlice );	//ALF is written in substream #0 with CABAC coder #0 (see ALF param encoding below)
		}
		em_pcEntropyCoder->resetEntropy	  ();

		// File writing
#if !ETRI_Remove_Redundant_EntoropyLoader	///만일 Slice 병렬화에서도 이 부분이 필요 없다면  ! ETRI_Remove_Redundant_EntoropyLoader 하여 해당 코드를 삭제한다. @2015 5 12 by Seok
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#endif
 		// for now, override the TILES_DECODER setting in order to write substreams.
		em_pcEntropyCoder->setBitstream	  ( &pcSubstreamsOut[0] );

	}
#endif
	pcSlice->setFinalized(true);


}



/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Writeout Slice Data Including SLice NALU. Espesically, WPP params here.
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_WriteOutSlice(TComPic* pcPic, TComSlice*& pcSlice, TComOutputBitstream* pcSubstreamsOut, TComOutputBitstream*& pcBitstreamRedirect, 
									TEncSbac* pcSbacCoders, AccessUnit& accessUnit, OutputNALUnit& nalu, ETRI_SliceInfo& ReturnValue)
{
	Int*		iNumSubstreams 	= ReturnValue.iNumSubstreams;
	UInt*	uiOneBitstreamPerSliceLength = ReturnValue.uiOneBitstreamPerSliceLength;
#if ETRI_MULTITHREAD_2

	em_cSbacCoder.load( &pcSbacCoders[0] );

	pcSlice->setTileOffstForMultES( *uiOneBitstreamPerSliceLength );
	pcSlice->setTileLocationCount ( 0 );
	em_cSliceEncoder.encodeSlice(pcPic, pcSubstreamsOut);
	{
		// Construct the final bitstream by flushing and concatenating substreams.
		// The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
		UInt* puiSubstreamSizes = pcSlice->getSubstreamSizes();
		UInt uiTotalCodedSize = 0; // for padding calcs.
		UInt uiNumSubstreamsPerTile = *iNumSubstreams;

		if (*iNumSubstreams > 1)
			uiNumSubstreamsPerTile /= pcPic->getPicSym()->getNumTiles();

		for ( UInt ui = 0 ; ui < *iNumSubstreams; ui++ )
		{
			// Flush all substreams -- this includes empty ones.
			// Terminating bit and flush.
			em_cEntropyCoder.setEntropyCoder   ( &pcSbacCoders[ui], pcSlice );
			em_cEntropyCoder.setBitstream      (  &pcSubstreamsOut[ui] );
			em_cEntropyCoder.encodeTerminatingBit( 1 );
			em_cEntropyCoder.encodeSliceFinish();

		pcSubstreamsOut[ui].writeByteAlignment();   // Byte-alignment in slice_data() at end of sub-stream
		// Byte alignment is necessary between tiles when tiles are independent.
		uiTotalCodedSize += pcSubstreamsOut[ui].getNumberOfWrittenBits();

		Bool bNextSubstreamInNewTile = ((ui+1) < *iNumSubstreams)&& ((ui+1)%uiNumSubstreamsPerTile == 0);
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
		em_cEntropyCoder.setEntropyCoder   ( &em_cCavlcCoder, pcSlice );
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cEntropyCoder.encodeTilesWPPEntryPoint( pcSlice );

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
			pcOut->addSubstream(&pcSubstreamsOut[ui+offs]);

	}

	UInt boundingAddrSlice, boundingAddrSliceSegment;
	boundingAddrSlice        = em_cstoredStartCUAddrForEncodingSlice[*ReturnValue.startCUAddrSliceIdx];
	boundingAddrSliceSegment = em_cstoredStartCUAddrForEncodingSliceSegment[*ReturnValue.startCUAddrSliceSegmentIdx];
	*ReturnValue.nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
	// If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
	// If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
	Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
	ETRI_xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
	accessUnit.push_back(new NALUnitEBSP(nalu));
	*ReturnValue.actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
	bNALUAlignedWrittenToList = true;
	*uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits(); // length of bitstream after byte-alignment

	if (!bNALUAlignedWrittenToList)
	{
		nalu.m_Bitstream.writeAlignZero();
		accessUnit.push_back(new NALUnitEBSP(nalu));
		*uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits() + 24; // length of bitstream after byte-alignment + 3 byte startcode 0x000001
	}

	if( ( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() ) &&
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
		ReturnValue.accumBitsDU[ pcSlice->getSliceIdx() ] = ( numRBSPBytes << 3 );
		ReturnValue.accumNalsDU[ pcSlice->getSliceIdx() ] = numNalus;   // SEI not counted for bit count; hence shouldn't be counted for # of NALUs - only for consistency
	}

#else
	em_pcSbacCoder->load(&pcSbacCoders[0]);

	pcSlice->setTileOffstForMultES(*uiOneBitstreamPerSliceLength);
	pcSlice->setTileLocationCount(0);
	em_pcSliceEncoder->encodeSlice(pcPic, pcSubstreamsOut);

	{
		// Construct the final bitstream by flushing and concatenating substreams.
		// The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
		UInt* puiSubstreamSizes = pcSlice->getSubstreamSizes();
		UInt uiTotalCodedSize = 0; // for padding calcs.
		UInt uiNumSubstreamsPerTile = *iNumSubstreams;

		if (*iNumSubstreams > 1)
			uiNumSubstreamsPerTile /= pcPic->getPicSym()->getNumTiles();

		for (UInt ui = 0; ui < *iNumSubstreams; ui++)
		{
			// Flush all substreams -- this includes empty ones.
			// Terminating bit and flush.
			em_pcEntropyCoder->setEntropyCoder(&pcSbacCoders[ui], pcSlice);
			em_pcEntropyCoder->setBitstream(&pcSubstreamsOut[ui]);
			em_pcEntropyCoder->encodeTerminatingBit(1);
			em_pcEntropyCoder->encodeSliceFinish();

			pcSubstreamsOut[ui].writeByteAlignment();   // Byte-alignment in slice_data() at end of sub-stream
			// Byte alignment is necessary between tiles when tiles are independent.
			uiTotalCodedSize += pcSubstreamsOut[ui].getNumberOfWrittenBits();

			Bool bNextSubstreamInNewTile = ((ui + 1) < *iNumSubstreams) && ((ui + 1) % uiNumSubstreamsPerTile == 0);
			if (bNextSubstreamInNewTile)
			{
				pcSlice->setTileLocation(ui / uiNumSubstreamsPerTile, pcSlice->getTileOffstForMultES() + (uiTotalCodedSize >> 3));
			}
			if (ui + 1 < pcSlice->getPPS()->getNumSubstreams())
			{
				puiSubstreamSizes[ui] = pcSubstreamsOut[ui].getNumberOfWrittenBits() + (pcSubstreamsOut[ui].countStartCodeEmulations() << 3);
			}
		}

		// Complete the slice header info.
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
		em_pcEntropyCoder->encodeTilesWPPEntryPoint(pcSlice);

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
	pcOut->addSubstream(&pcSubstreamsOut[ui+offs]);

	}

	UInt boundingAddrSlice, boundingAddrSliceSegment;
	boundingAddrSlice        = (*em_storedStartCUAddrForEncodingSlice)[*ReturnValue.startCUAddrSliceIdx];
	boundingAddrSliceSegment = (*em_storedStartCUAddrForEncodingSliceSegment)[*ReturnValue.startCUAddrSliceSegmentIdx];
	*ReturnValue.nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
	// If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
	// If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
	Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
	em_pcGOPEncoder->xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
	accessUnit.push_back(new NALUnitEBSP(nalu));
	*ReturnValue.actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
	bNALUAlignedWrittenToList = true;
	*uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits(); // length of bitstream after byte-alignment

	if (!bNALUAlignedWrittenToList)
	{
		nalu.m_Bitstream.writeAlignZero();
		accessUnit.push_back(new NALUnitEBSP(nalu));
		*uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits() + 24; // length of bitstream after byte-alignment + 3 byte startcode 0x000001
	}

	if( ( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() ) &&
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
		ReturnValue.accumBitsDU[ pcSlice->getSliceIdx() ] = ( numRBSPBytes << 3 );
		ReturnValue.accumNalsDU[ pcSlice->getSliceIdx() ] = numNalus;   // SEI not counted for bit count; hence shouldn't be counted for # of NALUs - only for consistency
	}
#endif

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Writeout Slice Data Including SLice NALU. Espesically, WPP params here.
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_SAO_Process(TComPic* pcPic, TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue)
{
#if ETRI_MULTITHREAD_2
	// set entropy coder for RD
	em_cEntropyCoder.setEntropyCoder ( &em_cSbacCoder, pcSlice );
	if ( pcSlice->getSPS()->getUseSAO() )
	{
		em_cEntropyCoder.resetEntropy();
		em_cEntropyCoder.setBitstream( &em_cBitCounter );
		Bool sliceEnabled[NUM_SAO_COMPONENTS];
		em_cEncSAO.initRDOCabacCoder(&em_cRDGoOnSbacCoder, pcSlice);
		em_cEncSAO.SAOProcess(pcPic
			, sliceEnabled
			, pcPic->getSlice(0)->getLambdas()
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
			, em_pcEncTop->getSaoLcuBoundary()
#endif
			);
		em_cEncSAO.PCMLFDisableProcess(pcPic);   

		//assign SAO slice header
		for(Int s=0; s< *ReturnValue.uiNumSlices; s++)
		{
			pcPic->getSlice(s)->setSaoEnabledFlag(sliceEnabled[SAO_Y]);
			assert(sliceEnabled[SAO_Cb] == sliceEnabled[SAO_Cr]);
			pcPic->getSlice(s)->setSaoEnabledFlagChroma(sliceEnabled[SAO_Cb]);
		}
	}

#else

	// set entropy coder for RD
	em_pcEntropyCoder->setEntropyCoder ( em_pcSbacCoder, pcSlice );
	if ( pcSlice->getSPS()->getUseSAO() )
	{
		em_pcEntropyCoder->resetEntropy();
		em_pcEntropyCoder->setBitstream( em_pcBitCounter );
		Bool sliceEnabled[NUM_SAO_COMPONENTS];
		em_pcSAO->initRDOCabacCoder(em_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
		em_pcSAO->SAOProcess(pcPic
		, sliceEnabled
		, pcPic->getSlice(0)->getLambdas()
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
		, em_pcEncTop->getSaoLcuBoundary()
#endif
		);
		em_pcSAO->PCMLFDisableProcess(pcPic);   

		//assign SAO slice header
		for(Int s=0; s< *ReturnValue.uiNumSlices; s++)
		{
			pcPic->getSlice(s)->setSaoEnabledFlag(sliceEnabled[SAO_Y]);
			assert(sliceEnabled[SAO_Cb] == sliceEnabled[SAO_Cr]);
			pcPic->getSlice(s)->setSaoEnabledFlagChroma(sliceEnabled[SAO_Cb]);
		}
	}
#endif

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Writeout SEI Data and MD5 Check. Specially, DecodedPictureHash, TemporalLevel0Index
	         : This Function is move to the TencFrame Member Functions 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_WriteOutSEI(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit)
{

	const Char* digestStr = NULL;
	if (em_pcEncTop->getDecodedPictureHashSEIEnabled())
	{
		/* calculate MD5sum for entire reconstructed picture */
		SEIDecodedPictureHash sei_recon_picture_digest;
		if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 1)
		{
		sei_recon_picture_digest.method = SEIDecodedPictureHash::MD5;
		calcMD5(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
		digestStr = digestToString(sei_recon_picture_digest.digest, 16);
		}
		else if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 2)
		{
		sei_recon_picture_digest.method = SEIDecodedPictureHash::CRC;
		calcCRC(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
		digestStr = digestToString(sei_recon_picture_digest.digest, 2);
		}
		else if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 3)
		{
		sei_recon_picture_digest.method = SEIDecodedPictureHash::CHECKSUM;
		calcChecksum(*pcPic->getPicYuvRec(), sei_recon_picture_digest.digest);
		digestStr = digestToString(sei_recon_picture_digest.digest, 4);
		}
		OutputNALUnit nalu(NAL_UNIT_SUFFIX_SEI, pcSlice->getTLayer());

#if ETRI_MULTITHREAD_2
		/* write the SEI messages */
		em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);

#else

		/* write the SEI messages */
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
#endif

		accessUnit.insert(accessUnit.end(), new NALUnitEBSP(nalu));
	}

	if (em_pcEncTop->getTemporalLevel0IndexSEIEnabled())
	{
		SEITemporalLevel0Index sei_temporal_level0_index;
		em_pcGOPEncoder->ETRI_SetSEITemporalLevel0Index(pcSlice, sei_temporal_level0_index);

		OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

#if ETRI_MULTITHREAD_2
		/* write the SEI messages */
		em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
#else

		/* write the SEI messages */
		em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
		em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, pcSlice->getSPS());
		writeRBSPTrailingBits(nalu.m_Bitstream);
#endif

		/* insert the SEI message NALUnit before any Slice NALUnits */
		AccessUnit::iterator it = find_if(accessUnit.begin(), accessUnit.end(), mem_fun(&NALUnit::isSlice));
		accessUnit.insert(it, new NALUnitEBSP(nalu));
	}


	if (digestStr)
	{
		if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 1)	{printf(" [MD5:%s]", digestStr);}
		else if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 2){printf(" [CRC:%s]", digestStr);}
		else if(em_pcEncTop->getDecodedPictureHashSEIEnabled() == 3){printf(" [Checksum:%s]", digestStr);}
	}

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Rate Control For GOP
			This Function should be move to the TencFrame Member Functions, owing to Frame Parallelism
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if KAIST_RC
void TEncFrame::ETRI_RateControlForGOP(TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue)
{
	Int* actualHeadBits	= ReturnValue.actualHeadBits;
	Int* actualTotalBits	= ReturnValue.actualTotalBits;
	Int* estimatedBits	 	= ReturnValue.estimatedBits;
	Double*	lambda		= ReturnValue.lambda;

	if ( em_pcEncTop->getUseRateCtrl() )
	{
		Int intra_size = em_pcRateCtrl->getIntraSize();
		Int iIDRModulus = pcSlice->getPOC() % intra_size;
		em_pcRateCtrl->m_sliceActualBits[iIDRModulus] = 0;
		TRCPic* tRCPic = em_pcRateCtrl->getTRCPic(iIDRModulus);

		Double avgQP = tRCPic->calAverageQP();
		Double avgLambda = tRCPic->calAverageLambda();
		if (avgLambda < 0.0){ avgLambda = *lambda; }

		tRCPic->updateAfterPicture(*actualHeadBits, *actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());

		em_pcRateCtrl->m_sliceActualBits[iIDRModulus] = *actualTotalBits;

		//printf("POC\t%d\ttargetbit\t%d\tactualbit\t%d\n", pcSlice->getPOC(), tRCPic->m_targetBit, tRCPic->m_outputBit);
	}

}
#endif

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Write HRD Model in VUI and SEI. Inaddition, Reset SEI Flags function if needed.
			This Function should be move to the TencFrame Member Functions, owing to Frame Parallelism
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_WriteOutHRDModel(TComSlice*& pcSlice, SEIPictureTiming& pictureTimingSEI, AccessUnit& accessUnit, ETRI_SliceInfo& ReturnValue)
{
	UInt  j;	
	UInt* paccumBitsDU = ReturnValue.accumBitsDU;
	UInt* paccumNalsDU = ReturnValue.accumNalsDU;

	/*--------------------------------------------------------------------------------------
		HRD condition corresponding to SEI and VUI Parameters 
	--------------------------------------------------------------------------------------*/
	if( ( em_pcEncTop->getPictureTimingSEIEnabled() || em_pcEncTop->getDecodingUnitInfoSEIEnabled() ) &&
	( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
	( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
	|| ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
	{
		TComVUI *vui = pcSlice->getSPS()->getVuiParameters();
		TComHRD *hrd = vui->getHrdParameters();

		/*--------------------------------------------------------------------------------------
			HRD CPB Parameters to SEI 
		--------------------------------------------------------------------------------------*/
		if( hrd->getSubPicCpbParamsPresentFlag() )
		{
			Int i;
			UInt64 ui64Tmp;
			UInt uiPrev = 0;
			UInt numDU = ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 );
			UInt *pCRD = &pictureTimingSEI.m_duCpbRemovalDelayMinus1[0];
			UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

			for( i = 0; i < numDU; i ++ )
			pictureTimingSEI.m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( paccumNalsDU[ i ] - 1 ) : ( paccumNalsDU[ i ] - paccumNalsDU[ i - 1] - 1 );

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
					ui64Tmp = ( ( ( paccumBitsDU[ numDU - 1 ]  - paccumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( em_pcEncTop->getTargetBitrate() ) );
					if( (UInt)ui64Tmp > maxDiff )	{
						tmp ++;
					}
				}
				uiPrev = 0;

				UInt flag = 0;
				for( i = ( numDU - 2 ); i >= 0; i -- )
				{
					flag = 0;
					ui64Tmp = ( ( ( paccumBitsDU[ numDU - 1 ]  - paccumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( em_pcEncTop->getTargetBitrate() ) );

					if( (UInt)ui64Tmp > maxDiff )
					{
						if(uiPrev >= maxDiff - tmp)
						{
							ui64Tmp = uiPrev + 1;
							flag = 1;
						}
						else ui64Tmp = maxDiff - tmp + 1;
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


		/*--------------------------------------------------------------------------------------
			Writing HRD Parameters to SEI
			--------------------------------------------------------------------------------------*/
#if ETRI_MULTITHREAD_2
		Bool  e_activeParameterSetSEIPresentInAU	= ETRI_getactiveParameterSetSEIPresentInAU();
		Bool  e_bufferingPeriodSEIPresentInAU    	= ETRI_getbufferingPeriodSEIPresentInAU();
#else
		Bool  e_activeParameterSetSEIPresentInAU = em_pcGOPEncoder->ETRI_getactiveParameterSetSEIPresentInAU();
		Bool  e_bufferingPeriodSEIPresentInAU = em_pcGOPEncoder->ETRI_getbufferingPeriodSEIPresentInAU();
#endif
		if (em_pcEncTop->getPictureTimingSEIEnabled())
		{
			{
				OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
#if ETRI_MULTITHREAD_2
				em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
				pictureTimingSEI.m_picStruct = (em_bisField && pcSlice->getPic()->isTopField())? 1 : em_bisField? 2 : 0;
				em_cseiWriter.writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, pcSlice->getSPS());
#else
				em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
				pictureTimingSEI.m_picStruct = (em_bisField && pcSlice->getPic()->isTopField()) ? 1 : em_bisField ? 2 : 0;
				em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, pcSlice->getSPS());
#endif
				writeRBSPTrailingBits(nalu.m_Bitstream);
				UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
				UInt offsetPosition = e_activeParameterSetSEIPresentInAU + e_bufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
				AccessUnit::iterator it;
				for (j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
				{
					it++;
				}
				accessUnit.insert(it, new NALUnitEBSP(nalu));
#if ETRI_MULTITHREAD_2
				ETRI_getpictureTimingSEIPresentInAU() = true;
#else
				em_pcGOPEncoder->ETRI_getpictureTimingSEIPresentInAU() = true;
#endif
			}
			if ( em_pcEncTop->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
			{
#if ETRI_MULTITHREAD_2
				OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
				em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
				em_scalableNestingSEI.m_nestedSEIs.clear();
				em_scalableNestingSEI.m_nestedSEIs.push_back(&pictureTimingSEI);
				em_cseiWriter.writeSEImessage(nalu.m_Bitstream, em_scalableNestingSEI, pcSlice->getSPS());
				writeRBSPTrailingBits(nalu.m_Bitstream);
				UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
				UInt offsetPosition = e_activeParameterSetSEIPresentInAU	+ e_bufferingPeriodSEIPresentInAU 
					+ ETRI_getpictureTimingSEIPresentInAU() 
					+ ETRI_getnestedBufferingPeriodSEIPresentInAU();    // Insert PT SEI after APS and BP SEI
				AccessUnit::iterator it;
				for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
				{
					it++;
				}
				accessUnit.insert(it, new NALUnitEBSP(nalu));
				ETRI_getnestedBufferingPeriodSEIPresentInAU() = true;
#else
				OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
				em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
				em_scalableNestingSEI.m_nestedSEIs.clear();
				em_scalableNestingSEI.m_nestedSEIs.push_back(&pictureTimingSEI);
				em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, em_scalableNestingSEI, pcSlice->getSPS());
				writeRBSPTrailingBits(nalu.m_Bitstream);
				UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
				UInt offsetPosition = e_activeParameterSetSEIPresentInAU	+ e_bufferingPeriodSEIPresentInAU 
					+ em_pcGOPEncoder->ETRI_getpictureTimingSEIPresentInAU() 
					+ em_pcGOPEncoder->ETRI_getnestedBufferingPeriodSEIPresentInAU();    // Insert PT SEI after APS and BP SEI
				AccessUnit::iterator it;
				for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
				{
					it++;
				}
				accessUnit.insert(it, new NALUnitEBSP(nalu));
				em_pcGOPEncoder->ETRI_getnestedBufferingPeriodSEIPresentInAU() = true;
#endif
			}
		}


		if( em_pcEncTop->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
		{
#if ETRI_MULTITHREAD_2
			em_cEntropyCoder.setEntropyCoder(&em_cCavlcCoder, pcSlice);
#else
			em_pcEntropyCoder->setEntropyCoder(em_pcCavlcCoder, pcSlice);
#endif
			for (Int i = 0; i < (pictureTimingSEI.m_numDecodingUnitsMinus1 + 1); i++)
			{
				OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());

				SEIDecodingUnitInfo tempSEI;
				tempSEI.m_decodingUnitIdx = i;
				tempSEI.m_duSptCpbRemovalDelay = pictureTimingSEI.m_duCpbRemovalDelayMinus1[i] + 1;
				tempSEI.m_dpbOutputDuDelayPresentFlag = false;
				tempSEI.m_picSptDpbOutputDuDelay = *ReturnValue.picSptDpbOutputDuDelay;

				AccessUnit::iterator it;
				// Insert the first one in the right location, before the first slice
				if(i == 0)
				{
#if ETRI_MULTITHREAD_2
					// Insert before the first slice.
					em_cseiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
					writeRBSPTrailingBits(nalu.m_Bitstream);

					UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
					UInt offsetPosition = e_activeParameterSetSEIPresentInAU	+ e_bufferingPeriodSEIPresentInAU 
						+ ETRI_getpictureTimingSEIPresentInAU();  // Insert DU info SEI after APS, BP and PT SEI
#else
					// Insert before the first slice.
					em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
					writeRBSPTrailingBits(nalu.m_Bitstream);

					UInt seiPositionInAu = em_pcGOPEncoder->xGetFirstSeiLocation(accessUnit);
					UInt offsetPosition = e_activeParameterSetSEIPresentInAU	+ e_bufferingPeriodSEIPresentInAU 
						+ em_pcGOPEncoder->ETRI_getpictureTimingSEIPresentInAU();  // Insert DU info SEI after APS, BP and PT SEI
#endif
					for (j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
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
						if(ctr == paccumNalsDU[ i - 1 ])
						{
							// Insert before the first slice.
#if ETRI_MULTITHREAD_2
							em_cseiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
#else
							em_pcseiWriter->writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
#endif
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

#if ETRI_MULTITHREAD_2
	ETRI_xResetNonNestedSEIPresentFlags();
	ETRI_xResetNestedSEIPresentFlags();
#else
	em_pcGOPEncoder->xResetNonNestedSEIPresentFlags();
	em_pcGOPEncoder->xResetNestedSEIPresentFlags();
#endif

}


// ====================================================================================================================
// Frame Compression Auxlilary Functions : inline, Macro, and Data Set 
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Before ETRI_CompressFrame, using the local and member variable in TEncGOP and TEncGOP::TEncCompressGOP
	@author: Jinwuk Seok : 2015 5 23 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if ETRI_MULTITHREAD_2
Void TEncFrame::ETRI_setFrameParameter(Int iGOPid,  Int pocLast, Int iNumPicRcvd, Int IRAPGOPid, int iLastIDR, UInt*	uiaccumBitsDU, UInt* uiaccumNalsDU, Bool isField, Bool isTff)
#else
Void TEncFrame::ETRI_setFrameParameter(Int iGOPid, Int pocLast, Int iNumPicRcvd, Int IRAPGOPid, int iLastIDR, UInt*	uiaccumBitsDU, UInt* uiaccumNalsDU, TComOutputBitstream*& pcBitstreamRedirect, Bool isField, Bool isTff)
#endif
{
	em_iGOPid  		= iGOPid;
	em_bisField		= isField;
	em_bisTff 		= isTff;
	em_iPOCLast		= pocLast;
	em_iNumPicRcvd 	= iNumPicRcvd;
	em_IRAPGOPid  	= IRAPGOPid;
	em_iLastIDR		= iLastIDR;		/// set with m_iLastIDR in TEncGOP : 2015 5 23 by Seok 

	//----------------------------------------------------------------------------------------
	em_iGopSize  		= em_pcGOPEncoder->getGOPSize();		
#if ETRI_MULTITHREAD_2
	em_iLastIDR			= iLastIDR;
#else
	em_iLastIDR = em_pcGOPEncoder->ETRI_getiLastIDR();
#endif

	em_associatedIRAPType	= em_pcGOPEncoder->ETRI_getassociatedIRAPType();	///Is it necessary to a member variable ?? and Value to TEncGOP ??	@ 2015 5 24 by Seok
	em_associatedIRAPPOC 	= em_pcGOPEncoder->ETRI_getassociatedIRAPPOC();		///Is it necessary to a member variable ?? and Value to TEncGOP ??	@ 2015 5 24 by Seok
	em_iLastRecoveryPicPOC	= em_pcGOPEncoder->ETRI_getiLastRecoveryPicPOC();

	em_pocCRA   		= em_pcGOPEncoder->ETRI_getpocCRA();
	em_bRefreshPending  = em_pcGOPEncoder->ETRI_getbRefreshPending();			/// Value to TEncGOP ??	@2015 5 24 by Seok

#if !ETRI_MULTITHREAD_2
	/// 2015 5 24 by Seok : it's Test. Not Implemeted as this.
	em_storedStartCUAddrForEncodingSlice = em_pcGOPEncoder->ETRI_getstoredStartCUAddrForEncodingSlice();
	em_storedStartCUAddrForEncodingSliceSegment = em_pcGOPEncoder->ETRI_getstoredStartCUAddrForEncodingSliceSegment();

	//----------------------------------------------------------------------------------------
	em_pcBitstreamRedirect = pcBitstreamRedirect;
#endif

	em_scalableNestingSEI.m_bitStreamSubsetFlag 			= 1;	 			///If the nested SEI messages are picture buffereing SEI mesages, picure timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
	em_scalableNestingSEI.m_nestingOpFlag   				= 0;
	em_scalableNestingSEI.m_nestingNumOpsMinus1  			= 0;	 			///nesting_num_ops_minus1
	em_scalableNestingSEI.m_allLayersFlag					= 0;
	em_scalableNestingSEI.m_nestingNoOpMaxTemporalIdPlus1	= 6 + 1;  			///nesting_no_op_max_temporal_id_plus1
	em_scalableNestingSEI.m_nestingNumLayersMinus1   		= 1 - 1;  			///nesting_num_layers_minus1
	em_scalableNestingSEI.m_nestingLayerId[0] 				= 0;
	em_scalableNestingSEI.m_callerOwnsSEIs   				= true;

	accumBitsDU = uiaccumBitsDU;
	accumNalsDU = uiaccumNalsDU;

	//------------------------------ After V0 Revision --------------------------------------
	// These values are updated in compressFrame. 
	// Thus, always get from GOP before compression Frame and  return the updated value to GOP 
	// @ 2015 5 27 by Seok
	//----------------------------------------------------------------------------------------
	em_bFirst = em_pcGOPEncoder->ETRI_getbFirst();				///Most Serious variable  2015 5 27 by Seok
#if !ETRI_MULTITHREAD_2
	em_cpbRemovalDelay = em_pcGOPEncoder->ETRI_getcpbRemovalDelay();		///So So ... 2015 5 27 by Seok
#endif

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Some Parameters have to be copied to the GOP member variable
	@author: Jinwuk Seok : 2015 5 23 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncFrame::ETRI_ResetFrametoGOPParameter()
{
#if !ETRI_MULTITHREAD_2
	em_pcGOPEncoder->ETRI_getcpbRemovalDelay() = em_cpbRemovalDelay;
#endif
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Macro and Service Function for ETRI_compressGOP
			This Function should be move to the TencFrame Member Functions, owing to Frame Parallelism
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
//__inline Void TEncFrame::ETRI_UpdateSliceAfterCompression(TComPic*& pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue) 
__inline Void TEncFrame::ETRI_UpdateSliceAfterCompression(TComPic*& pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue) 
{
#if ETRI_MULTITHREAD_2
	em_cstoredStartCUAddrForEncodingSlice.push_back( pcSlice->getSliceCurEndCUAddr()); 
	(*ReturnValue.startCUAddrSliceIdx)++; 
	em_cstoredStartCUAddrForEncodingSliceSegment.push_back(pcSlice->getSliceCurEndCUAddr()); 
#else
	em_storedStartCUAddrForEncodingSlice->push_back(pcSlice->getSliceCurEndCUAddr());
	(*ReturnValue.startCUAddrSliceIdx)++;
	em_storedStartCUAddrForEncodingSliceSegment->push_back(pcSlice->getSliceCurEndCUAddr());
#endif
	(*ReturnValue.startCUAddrSliceSegmentIdx)++;

	pcSlice = pcPic->getSlice(0);
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
	e_sISliceInfo.uiWidth   					= &uiWidth; \
	e_sISliceInfo.uiHeight  					= &uiHeight; \
	e_sISliceInfo.iNumSubstreams    			= &iNumSubstreams; \
	e_sISliceInfo.startCUAddrSlice   			= &startCUAddrSlice;\
	e_sISliceInfo.startCUAddrSliceIdx    		= &startCUAddrSliceIdx; \
	e_sISliceInfo.startCUAddrSliceSegment    	= &startCUAddrSliceSegment; \
	e_sISliceInfo.startCUAddrSliceSegmentIdx 	= &startCUAddrSliceSegmentIdx; \
	e_sISliceInfo.nextCUAddr    				= &nextCUAddr; \
	e_sISliceInfo.picSptDpbOutputDuDelay    	= &picSptDpbOutputDuDelay; \
	e_sISliceInfo.accumBitsDU   				= accumBitsDU; \
	e_sISliceInfo.accumNalsDU   				= accumNalsDU; \
	e_sISliceInfo.tmpBitsBeforeWriting   		= &tmpBitsBeforeWriting; \
	e_sISliceInfo.uiOneBitstreamPerSliceLength 	= &uiOneBitstreamPerSliceLength; \
	e_sISliceInfo.picSptDpbOutputDuDelay 		= &picSptDpbOutputDuDelay;


// ====================================================================================================================
// Frame Compression Functions 
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Main 1 Frame Compression Function 
			This Function consists with the Set accessUnit (bin out), Slice Encoder, Slice Compression, Slice Filtering, Write out the Header of Frame (SPS/VPS/PPS/SEI/VUI) and Slice.
	@param: 	Int iTimeOffset 	: Look atthe GOP Compression 
	@param:  Int pocCurr		
	@param:  TComPic* pcPic	: Input Picture (One)
	@param:  TComPicYuv* pcPicYuvRecOut : Recon Picture (One)
	@param:  TComList<TComPic*>& rcListPic : Input Picture Buffer List 
	@param:  TComList<TComPicYuv*>& rcListPicYuvRecOut : Reconstruction Picture Buffer List (For DPB)
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if ETRI_MULTITHREAD_2
#if ETRI_THREADPOOL_OPT
Void TEncFrame::ETRI_CompressFrame(Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, std::list<AccessUnit>& accessUnitsInGOP, Bool bFirst, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, Bool bDefault)
#else
Void TEncFrame::ETRI_CompressFrame(Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, std::list<AccessUnit>& accessUnitsInGOP, Bool bFirst, Bool bDefault)
#endif

#else
Void TEncFrame::ETRI_CompressFrame(Int iTimeOffset, Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP)
#endif
{
	TComSlice*	pcSlice			= nullptr;
	TEncSbac* 	pcSbacCoders 	= nullptr;
	TComOutputBitstream* pcSubstreamsOut = nullptr;

	Bool  	writeSOP = em_pcEncTop->getSOPDescriptionSEIEnabled();
	Int    	picSptDpbOutputDuDelay = 0;
	SEIPictureTiming pictureTimingSEI;

	/// In ETRI_SetSliceEncoder
	UInt   	uiOneBitstreamPerSliceLength = 0;

	/// Parameter for Rate Control
	Double	lambda		   = 0.0;
	Int    	actualHeadBits, actualTotalBits, estimatedBits, tmpBitsBeforeWriting;

	/// Set CU Address for Slice compression 2015 5 14 by Seok
	UInt   	uiNumSlices, uiInternalAddress, uiExternalAddress, uiRealEndAddress; 
	UInt   	uiPosX, uiPosY, uiWidth, uiHeight;

	/// Set CU Address for Slice Processing ... @2 015 5 14 by Seok 
	UInt   	startCUAddrSliceIdx, startCUAddrSlice, startCUAddrSliceSegmentIdx, startCUAddrSliceSegment, nextCUAddr;

	/// Number of Unit to Parallel Processing such as WPP and ... @2 015 5 14 by Seok : Local Variable Used in ETRI_ComressFrame : Remove 
	Int    	iNumSubstreams;

	TComRefPicListModification* refPicListModification = nullptr;

	/// 2015 5 14 by Seok : Remove
	ETRI_SliceInfo	e_sISliceInfo;
	ETRI_SET_SISLICEINFO(e_sISliceInfo);	///Data Set for CompressionGOP using  ETRI_SliceInfo	e_sISliceInfo @ 2015 5 14 by Seok

	//=================================================================================
	// Initial to start encoding
	//=================================================================================
	UInt uiColDir = 1;
	//-- For time output for each slice
#if ETRI_GNUC_TIME_FUNCTION
    timespec lBefore;
    clock_gettime(CLOCK_MONOTONIC, &lBefore);
#else 
	long iBeforeTime = clock();
#endif 

#if ETRI_MULTITHREAD_2
	TComOutputBitstream  *e_pcBitstreamRedirect;
	e_pcBitstreamRedirect = new TComOutputBitstream;

	// Rate Control
	if(em_pcEncTop->getUseRateCtrl())
	{
#if KAIST_RC
		//em_cRateCtrl.initRCGOP(1);
#endif
	}

	if(bDefault)
	{
		uiColDir=ETRI_Select_UiColDirection(em_iGOPid, uiColDir);					/// select uiColDir : 2015 5 11 by Seok
	}
	else
	{
		pcSlice = pcPic->getSlice(0);
	}
#else
	uiColDir = ETRI_Select_UiColDirection(em_iGOPid, uiColDir);					/// select uiColDir : 2015 5 11 by Seok
#endif

	// start a new access unit: create an entry in the list of output access units
	accessUnitsInGOP.push_back(AccessUnit());
	AccessUnit& accessUnit = accessUnitsInGOP.back();
	em_pcAU = &accessUnit;														///For Interface @2015 5 25 by Seok

#if ETRI_MULTITHREAD_2
	if(bDefault)
	{
		//	Slice data initialization
		pcPic->clearSliceBuffer();
		assert(pcPic->getNumAllocatedSlice() == 1);
		em_cSliceEncoder.setSliceIdx(0);
		em_cSliceEncoder.initEncSlice ( pcPic, em_iPOCLast, pocCurr, em_iNumPicRcvd, em_iGOPid, pcSlice, em_pcEncTop->getSPS(), em_pcEncTop->getPPS(), em_bisField );

		//Set Frame/Field coding
		pcSlice->getPic()->setField(em_bisField);
		pcSlice->setLastIDR(em_iLastIDR);
		pcSlice->setSliceIdx(0);

		refPicListModification = pcSlice->getRefPicListModification();				///Get RefPicList for each Slice (ot Picture) If you want to Slice Paralleization, This Parameter should be multiple allocated. @2015 5 14 by Seok 

		ETRI_SliceDataInitialization(pcPic, pcSlice);
		ETRI_SetReferencePictureSetforSlice(pcPic, pcSlice, em_iGOPid, pocCurr, em_bisField, rcListPic);
		ETRI_refPicListModification(pcSlice, refPicListModification, rcListPic, em_iGOPid, uiColDir);
		ETRI_NoBackPred_TMVPset(pcSlice, em_iGOPid);
	}

	pcSlice->setRefPicBorder(rcListPic);  // gplusplus
#else
	//	Slice data initialization
	pcPic->clearSliceBuffer();
	assert(pcPic->getNumAllocatedSlice() == 1);
	em_pcSliceEncoder->setSliceIdx(0);
	pcPic->setCurrSliceIdx(0);
	em_pcSliceEncoder->initEncSlice ( pcPic, em_iPOCLast, pocCurr, em_iNumPicRcvd, em_iGOPid, pcSlice, em_pcEncTop->getSPS(), em_pcEncTop->getPPS(), em_bisField );

	//Set Frame/Field coding
	pcSlice->getPic()->setField(em_bisField);
	pcSlice->setLastIDR(em_iLastIDR);
	pcSlice->setSliceIdx(0);

	refPicListModification = pcSlice->getRefPicListModification();				///Get RefPicList for each Slice (ot Picture) If you want to Slice Paralleization, This Parameter should be multiple allocated. @2015 5 14 by Seok 

	ETRI_SliceDataInitialization(pcPic, pcSlice);
	ETRI_SetReferencePictureSetforSlice(pcPic, pcSlice, em_iGOPid, pocCurr, em_bisField, rcListPic);
	ETRI_refPicListModification(pcSlice, refPicListModification, rcListPic, em_iGOPid, uiColDir);
	ETRI_NoBackPred_TMVPset(pcSlice, em_iGOPid);
#endif

	//=================================================================================
	// Slice compression
	//=================================================================================
#if ETRI_MULTITHREAD_2
	if (em_pcEncTop->getUseASR()){em_cSliceEncoder.setSearchRange(pcSlice);} 	///When you use a Adaptive Search Ramge, Serach Range is set HERE @ 2015 5 14 by Seok
#else
	if (em_pcEncTop->getUseASR()){ em_pcSliceEncoder->setSearchRange(pcSlice); } 	///When you use a Adaptive Search Ramge, Serach Range is set HERE @ 2015 5 14 by Seok
#endif
	ETRI_setMvdL1ZeroFlag(pcPic, pcSlice);   	    							///Set MVDL1Zero Flag according to RefPicList @ 2015 5 14 by Seok
#if KAIST_RC
	if (em_pcEncTop->getUseRateCtrl())
	{
    if (pocCurr%em_pcEncTop->getIntraPeriod() == 0)
    {
      if (em_pcEncTop->getRCEnableRateControl())
      {
        Int* tileColumnWidth = new Int[em_pcEncTop->getNumColumnsMinus1() + 1];
        Int* tileRowHeight = new Int[em_pcEncTop->getNumRowsMinus1() + 1];
        if (!em_pcEncTop->getTileUniformSpacingFlag())
        {
          for (Int col = 0; col < em_pcEncTop->getNumColumnsMinus1(); col++)
            tileColumnWidth[col] = em_pcEncTop->getColumnWidth(col);
          for (Int row = 0; row < em_pcEncTop->getNumRowsMinus1(); row++)
            tileRowHeight[row] = em_pcEncTop->getRowHeight(row);
        }
        else
        {
          Int picWidth = em_pcEncTop->getSourceWidth();
          Int picHeight = em_pcEncTop->getSourceHeight();
          Int picWidthInBU = (picWidth  % g_uiMaxCUWidth) == 0 ? picWidth / g_uiMaxCUWidth : picWidth / g_uiMaxCUWidth + 1;
          Int picHeightInBU = (picHeight % g_uiMaxCUHeight) == 0 ? picHeight / g_uiMaxCUHeight : picHeight / g_uiMaxCUHeight + 1;
          Int numCols = em_pcEncTop->getNumColumnsMinus1() + 1;
          Int numRows = em_pcEncTop->getNumRowsMinus1() + 1;
          for (Int col = 0; col < em_pcEncTop->getNumColumnsMinus1(); col++)
            tileColumnWidth[col] = (col + 1)*picWidthInBU / numCols - (col*picWidthInBU) / numCols;
          for (Int row = 0; row < em_pcEncTop->getNumRowsMinus1(); row++)
            tileRowHeight[row] = (row + 1)*picHeightInBU / numRows - (row*picHeightInBU) / numRows;
        }
        
#if ETRI_MULTITHREAD_2
        TEncRateCtrl tempRC;
        std::list<TEncRateCtrl>::iterator		tempIter;
        std::list<TEncRateCtrl>::reverse_iterator		tempRIter;

        em_pcEncTop->getRateCtrlLst()->push_back(tempRC);
        tempRIter = em_pcEncTop->getRateCtrlLst()->rbegin();
        em_pcRateCtrl = &(*tempRIter);

        /*
        Int intra_size = em_pcEncTop->getIntraPeriod();
        
        for (tempIter = em_pcEncTop->getRateCtrlLst()->begin(); tempIter != em_pcEncTop->getRateCtrlLst()->end(); tempIter++)
        {
          em_pcRateCtrl = &(*tempIter);
        }
        */
        //printf("POC for Intra picture : %d\n\n", pocCurr);
        em_pcRateCtrl->IDRnum = (Int)(pocCurr / em_pcEncTop->getIntraPeriod());
#endif
        em_pcRateCtrl->init(em_pcEncTop->getNumColumnsMinus1() + 1, em_pcEncTop->getNumRowsMinus1() + 1, tileColumnWidth, tileRowHeight, em_pcEncTop->getframesToBeEncoded(), em_pcEncTop->getRCTargetBitrate(), em_pcEncTop->getFrameRate(), em_pcEncTop->getSourceWidth(), em_pcEncTop->getSourceHeight(),
          g_uiMaxCUWidth, g_uiMaxCUHeight, em_pcEncTop->getRCUseLCUSeparateModel(), em_pcEncTop->getIntraPeriod(), em_pcEncTop->getGOPSize(), em_pcEncTop->ETRI_getGOPEntry());

        delete[] tileColumnWidth;
        delete[] tileRowHeight;
      }

    }
#if ETRI_MULTITHREAD_2
		else
		{
			std::list<TEncRateCtrl>::iterator		tempIter;
			Int intra_size = em_pcEncTop->getIntraPeriod();
			Int iIDRIndex = pocCurr / intra_size;
      for (tempIter = em_pcEncTop->getRateCtrlLst()->begin(); tempIter != em_pcEncTop->getRateCtrlLst()->end(); tempIter++)
      {
        if ((em_pcRateCtrl = &(*tempIter))->IDRnum == iIDRIndex)
        {
          //printf("RCIDRnum : %d\n", em_pcRateCtrl->IDRnum);
          break;
        }
      }
      //em_pcRateCtrl = &(*em_pcEncTop->getRateCtrlLst()->end());
		}
    //printf("RC instance selected. POC : %d IDR number : %d RC list size : %d\n", pocCurr, (Int)(pocCurr / em_pcEncTop->getIntraPeriod()), em_pcEncTop->getRateCtrlLst()->size());
		em_cSliceEncoder.setRateCtrl(em_pcRateCtrl);
		em_cCuEncoder.setRateCtrl(em_pcRateCtrl);
#endif
		ETRI_RateControlSlice(pocCurr, pcPic, pcSlice, em_iGOPid, e_sISliceInfo);
	}
#endif
	ETRI_setCUAddressinFrame(pcPic, pcSlice, e_sISliceInfo);   					///Set Fundamental CU Address for Slice Compression @ 2015 5 14 by Seok

	pcPic->getPicSym()->initTiles(pcSlice->getPPS());   						///Tile Initilization @ 2015 5 14 by Seok
	iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();    			    	///Allocate some coders, now we know how many tiles there are.

	ETRI_EvalCodingOrderMAPandInverseCOMAP(pcPic);    	     					///<Generating Coding Order MAP and Inverse MAP; Check Location of FUnction @ 2015 5 14 by Seok

#if ETRI_MULTITHREAD_2
	// Allocate some coders, now we know how many tiles there are.
	ETRI_createWPPCoders(iNumSubstreams);	
	pcSbacCoders = em_pcSbacCoders;
#else

	// Allocate some coders, now we know how many tiles there are.
	em_pcEncTop->createWPPCoders(iNumSubstreams);
	pcSbacCoders = em_pcEncTop->getSbacCoders();
#endif

	pcSubstreamsOut = new TComOutputBitstream[iNumSubstreams];

	ETRI_setStartCUAddr(pcSlice, e_sISliceInfo);								/// Ready to Compress Slice @ 2015 5 14 by Seok
	//=================================================================================
	//Set DLL info
	//=================================================================================
#if ETRI_DLL_INTERFACE	// 2015 06 15 by yhee
	FrameTypeInGOP = pcSlice->getSliceType();
	FramePOC = pocCurr;
	FrameEncodingOrder = em_iGOPid;
#endif

	while(nextCUAddr<uiRealEndAddress)   	    					     		///determine slice boundaries : Multiple Slice Encoding is somewhat difficult @ 2015 5 14 by Seok
	{
		pcSlice->setNextSlice		( false );
		pcSlice->setNextSliceSegment( false );
		assert(pcPic->getNumAllocatedSlice() == startCUAddrSliceIdx);
#if ETRI_MULTITHREAD_2
		em_cSliceEncoder.precompressSlice( pcPic );   						///If  eem_pcEncTop->getDeltaQpRD() = 0, then it is not working : Default Value is 0 @ 2015 5 14 by Seok
		em_cSliceEncoder.ETRI_compressSlice   (pcPic, threadpool, ETRI_MODIFICATION_V00);	///Main Compression Function 
#else

		em_pcSliceEncoder->precompressSlice(pcPic);   						///If  eem_pcEncTop->getDeltaQpRD() = 0, then it is not working : Default Value is 0 @ 2015 5 14 by Seok
		em_pcSliceEncoder->ETRI_compressSlice(pcPic, ETRI_MODIFICATION_V00);	///Main Compression Function 
#endif
		ETRI_SetNextSlice_with_IF(pcPic, pcSlice, e_sISliceInfo);				///Data Update for Next Slice Compression @ 2015 5 14 by Seok
	}
	ETRI_UpdateSliceAfterCompression(pcPic, pcSlice, e_sISliceInfo); 			///startCUAddrSliceIdx and startCUAddrSliceSegmentrIdx are Update @ 2015 5 14 by Seok	
	ETRI_LoopFilter(pcPic, pcSlice, e_sISliceInfo);   							///Loop Filter and Gather the statistics of SAO @ 2015 5 14 by Seok

//#if QURAM_ES_FILE_WRITING
//#if ETRI_MULTITHREAD_2
//
//	if (em_pcEncTop->ETRI_getReconFileOk())
//	{
//		pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
//	}
//#else
//	pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
//#endif
//	pcPic->setReconMark(true);
//	
//#if ETRI_FRAME_THEAD_OPT
//	if (mutex != NULL && cond != NULL)
//	{
//		pthread_mutex_lock(mutex);
//
//		*bRefPicAvailable = 1;
//		pthread_cond_signal(cond);
//			
//		pthread_mutex_unlock(mutex);
//	}
//#endif
//		
//#endif

	//=================================================================================
	// File Writing 
	//=================================================================================
#if ETRI_MULTITHREAD_2
	ETRI_WriteSeqHeader( pcPic, pcSlice, accessUnit, actualTotalBits, bFirst);			///Writeout VPS, PPS, SPS when m_bSeqFirst= TRUE @ 2015 5 11 by Seok
#else
	ETRI_WriteSeqHeader(pcPic, pcSlice, accessUnit, actualTotalBits);			///Writeout VPS, PPS, SPS when m_bSeqFirst= TRUE @ 2015 5 11 by Seok
#endif
	ETRI_WriteSOPDescriptionInSEI(em_iGOPid, pocCurr, pcSlice, accessUnit, writeSOP, em_bisField); 	///write SOP description SEI (if enabled) at the beginning of GOP @2015 5 11 by Seok
	ETRI_setPictureTimingSEI(pcSlice, pictureTimingSEI, em_IRAPGOPid, e_sISliceInfo);	///Write out Picture Timing Information in SEI @ 2015 5 11 by Seok
	ETRI_writeHRDInfo(pcSlice, accessUnit, em_scalableNestingSEI);				///WriteOut HRD Information @ 2015 5 11 by Seok
	ETRI_Ready4WriteSlice(pcPic, pocCurr, pcSlice, accessUnit, e_sISliceInfo);	///Ready for Write Out Slice Information through Encode Slice @ 2015 5 12 by Seok

	Int processingState = (pcSlice->getSPS()->getUseSAO())?(EXECUTE_INLOOPFILTER):(ENCODE_SLICE);
	Bool skippedSlice=false, bStopEncodeSlice = false;
	while (nextCUAddr < uiRealEndAddress) // Iterate over all slices
	{
		switch(processingState)
		{
			case ENCODE_SLICE:
			{
				ETRI_ReInitSliceData(pcPic, pcSlice, e_sISliceInfo); 			///Slice Data Re-Initilization for Encode Slice, Especially, SetRPS is conducted in this function. 
				ETRI_ResetSliceBoundaryData(pcPic, pcSlice, skippedSlice, bStopEncodeSlice, e_sISliceInfo); ///Set Slice Boundary Data 
				if (bStopEncodeSlice) {continue;}								/// If bStopEncodeSlice is true, escape the out of Encode Slice
				OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer());		///Get New NALU for Slice Information @ 2015 5 12 by Seok
#if ETRI_MULTITHREAD_2
							 ETRI_SetSliceEncoder(pcPic, pcSlice, pcSubstreamsOut, e_pcBitstreamRedirect, pcSbacCoders, nalu, e_sISliceInfo);///Set Slice Data to be written to HEVC ES, including Slice NALU @ 2015 5 12 by Seok
							 ETRI_WriteOutSlice(pcPic, pcSlice, pcSubstreamsOut, e_pcBitstreamRedirect, pcSbacCoders, accessUnit, nalu, e_sISliceInfo);///Main Encoding Slice Data set in the ETRI_SetSliceEncoder with CAVLC and CABAC @ 2015 5 12 by Seok

#else

							 ETRI_SetSliceEncoder(pcPic, pcSlice, pcSubstreamsOut, em_pcBitstreamRedirect, pcSbacCoders, nalu, e_sISliceInfo);///Set Slice Data to be written to HEVC ES, including Slice NALU @ 2015 5 12 by Seok
							 ETRI_WriteOutSlice(pcPic, pcSlice, pcSubstreamsOut, em_pcBitstreamRedirect, pcSbacCoders, accessUnit, nalu, e_sISliceInfo);///Main Encoding Slice Data set in the ETRI_SetSliceEncoder with CAVLC and CABAC @ 2015 5 12 by Seok
#endif
				processingState = ENCODE_SLICE;
			}
			break;

			case EXECUTE_INLOOPFILTER:
			{
				ETRI_SAO_Process(pcPic, pcSlice, e_sISliceInfo); 				///HERE, SAO Process isconducted Not Inloop Filter At First SAO and Encode SLice Finally, InLoopFilter @	2015 5 12 by Seok
				processingState = ENCODE_SLICE;
			}
			break;

			default:{printf("Not a supported encoding state\n"); assert(0); exit(-1);}
		}
	} // end iteration over slices

#if ETRI_MOTION_COMORESSION
    if (pcSlice->getSliceType() != I_SLICE && pcPic->getSlice(0)->getDepth() != 3)
        pcPic->compressMotion();
#else 
	pcPic->compressMotion();
#endif 
	//=================================================================================
	// Final Processing (Rate Control/Services/ Recon Pic Copy/PSNR/ HDR Information and others)
	//=================================================================================
#if ETRI_GNUC_TIME_FUNCTION
    timespec iCurrTime;
    clock_gettime(CLOCK_MONOTONIC, &iCurrTime); // Works on Linux by yhee 2016.04.19
    em_dEncTime = (iCurrTime.tv_sec - lBefore.tv_sec);
    em_dEncTime += (iCurrTime.tv_nsec - lBefore.tv_nsec) / 1000000000.0;
#else 
	em_dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;   			///For time output for each slice
#endif 
	ETRI_WriteOutSEI(pcPic, pcSlice, accessUnit);     								///Write Out SEI Information and MD5 Hash Check @ 2015 5 13 by Seok
#if KAIST_RC
	if (em_pcEncTop->getUseRateCtrl())
	{
		ETRI_RateControlForGOP(pcSlice, e_sISliceInfo);     							///Rate Cobtrol for GOP !!! @ 2015 5 14 by Seok
	}
#endif
	ETRI_WriteOutHRDModel(pcSlice, pictureTimingSEI, accessUnit, e_sISliceInfo);   	///HRD Model in VUI and SEI @ 2015 5 14 by Seok
	ETRI_ResetFrametoGOPParameter();									/// Some Important parameters are updated to GOP @ 2015 5 26 by Seok

//#if !QURAM_ES_FILE_WRITING
#if ETRI_MULTITHREAD_2
	
	if (em_pcEncTop->ETRI_getReconFileOk())
	{
		pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
	}
#else
	pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
#endif
	pcPic->setReconMark   ( true );

#if ETRI_FRAME_THEAD_OPT
	if (mutex != NULL && cond != NULL)
	{
		pthread_mutex_lock(mutex);

		*bRefPicAvailable = 1;
		pthread_cond_signal(cond);

		pthread_mutex_unlock(mutex);
	}
#endif

//#endif

#if ETRI_MULTITHREAD_2
#if KAIST_RC
	// Rate Control
//	if(em_pcEncTop->getUseRateCtrl())
//	{
//		em_cRateCtrl.destroyRCGOP();
//	}
#endif
#endif
	delete[] pcSubstreamsOut;
#if ETRI_MULTITHREAD_2
	delete e_pcBitstreamRedirect;
#else

	/* logging: insert a newline at end of picture period */
//	ETRI_xCalculateAddPSNR(pcPic, rcListPic, accessUnit, dEncTime, em_bisField, em_bisTff);		///Calculate PSNR as the result of Encoding @ 2015 5 14 by Seok
//	printf("\n");	fflush(stdout);
#endif

}


//=================================================================================
// Reserved Code 
//=================================================================================
#if 0
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Calculate PSNR. Including PSNR calculation for Interlaced Picture, we make other version of xcalculateAddPSNR
			For Parallelism, the Function through em_pcGOPEncoder may be changed and defined in TEncFrame.
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncFrame::ETRI_xCalculateAddPSNR(TComPic* pcPic, TComList<TComPic*>& rcListPic, AccessUnit& accessUnit, Double dEncTime, Bool isField, Bool isTff)
{
	em_pcGOPEncoder->xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime );
	
	//In case of field coding, compute the interlaced PSNR for both fields
	if (isField && ((!pcPic->isTopField() && isTff) || (pcPic->isTopField() && !isTff)) && (pcPic->getPOC()%em_iGopSize != 1))
	{
	  //get complementary top field
	  TComPic* pcPicTop;
	  TComList<TComPic*>::iterator	 iterPic = rcListPic.begin();
	  while ((*iterPic)->getPOC() != pcPic->getPOC()-1)
	  {
		iterPic ++;
	  }
	  pcPicTop = *(iterPic);
	  em_pcGOPEncoder->xCalculateInterlacedAddPSNR(pcPicTop, pcPic, pcPicTop->getPicYuvRec(), pcPic->getPicYuvRec(), accessUnit, dEncTime );
	}
	else if (isField && pcPic->getPOC()!= 0 && (pcPic->getPOC()%em_iGopSize == 0))
	{
	  //get complementary bottom field
	  TComPic* pcPicBottom;
	  TComList<TComPic*>::iterator	 iterPic = rcListPic.begin();
	  while ((*iterPic)->getPOC() != pcPic->getPOC()+1)
	  {
		iterPic ++;
	  }
	  pcPicBottom = *(iterPic);
	  em_pcGOPEncoder->xCalculateInterlacedAddPSNR(pcPic, pcPicBottom, pcPic->getPicYuvRec(), pcPicBottom->getPicYuvRec(), accessUnit, dEncTime );
	}

}
#endif

#if ETRI_MULTITHREAD_2
////////////////////////////////////////////////////////////////////////////
// gplusplus

/**
 - Allocate coders required for wavefront for the nominated number of substreams.
 .
 \param iNumSubstreams Determines how much information to allocate.
 */
Void TEncFrame::ETRI_createWPPCoders(Int iNumSubstreams)
{
	if (em_pcSbacCoders != NULL)
	{
		return; // already generated.
	}

	em_iNumSubstreams         = iNumSubstreams;
	em_pcSbacCoders           = new TEncSbac       [iNumSubstreams];
	em_pcBinCoderCABACs       = new TEncBinCABAC   [iNumSubstreams];
	em_pcRDGoOnSbacCoders     = new TEncSbac       [iNumSubstreams];
	em_pcRDGoOnBinCodersCABAC = new TEncBinCABAC   [iNumSubstreams];
	em_pcBitCounters          = new TComBitCounter [iNumSubstreams];
	em_pcRdCosts              = new TComRdCost     [iNumSubstreams];

	for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
	{
		em_pcRDGoOnSbacCoders[ui].init( &em_pcRDGoOnBinCodersCABAC[ui] );
		em_pcSbacCoders[ui].init( &em_pcBinCoderCABACs[ui] );
	}

	em_ppppcRDSbacCoders      = new TEncSbac***    [iNumSubstreams];
	em_ppppcBinCodersCABAC    = new TEncBinCABAC***[iNumSubstreams];
	for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ )
	{
		em_ppppcRDSbacCoders[ui]  = new TEncSbac** [g_uiMaxCUDepth+1];
		em_ppppcBinCodersCABAC[ui]= new TEncBinCABAC** [g_uiMaxCUDepth+1];

		for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
		{
			em_ppppcRDSbacCoders[ui][iDepth]  = new TEncSbac*     [CI_NUM];
			em_ppppcBinCodersCABAC[ui][iDepth]= new TEncBinCABAC* [CI_NUM];

			for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
			{
				em_ppppcRDSbacCoders  [ui][iDepth][iCIIdx] = new TEncSbac;
				em_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] = new TEncBinCABAC;
				em_ppppcRDSbacCoders  [ui][iDepth][iCIIdx]->init( em_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] );
			}
		}
	}
}

Void TEncFrame::ETRI_xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream*& codedSliceData)
{
	// Byte-align
	rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

	// Perform bitstream concatenation
	if (codedSliceData->getNumberOfWrittenBits() > 0)
	{
		rNalu.m_Bitstream.addSubstream(codedSliceData);
	}

	em_cEntropyCoder.setBitstream(&rNalu.m_Bitstream);

	codedSliceData->clear();
}

Void TEncFrame::ETRI_preLoopFilterPicAll( TComPic* pcPic, UInt64& ruiDist, UInt64& ruiBits )
{
	TComSlice* pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
	Bool bCalcDist = false;
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	for (int i = 0; i < MAX_NUM_THREAD; i++)
	{
		em_cLoopFilter[i].setCfg(em_pcEncTop->getLFCrossTileBoundaryFlag());
	}
	em_cLoopFilter[0].loopFilterPic(pcPic, em_cLoopFilter);
#else 
	em_cLoopFilter.setCfg(em_pcEncTop->getLFCrossTileBoundaryFlag());
	em_cLoopFilter.loopFilterPic( pcPic );
#endif 
	em_cEntropyCoder.setEntropyCoder ( &em_cRDGoOnSbacCoder, pcSlice );
	em_cEntropyCoder.resetEntropy    ();
	em_cEntropyCoder.setBitstream    ( &em_cBitCounter );
	em_cEntropyCoder.resetEntropy    ();
	ruiBits += em_cEntropyCoder.getNumberOfWrittenBits();

	if (!bCalcDist)
		ruiDist = em_pcGOPEncoder->xFindDistortionFrame(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
}

///////////////////////////////////////////////////////////////////////////////////////
// SEI Messages
SEIActiveParameterSets* TEncFrame::ETRI_xCreateSEIActiveParameterSets (TComSPS *sps)
{
	SEIActiveParameterSets *seiActiveParameterSets = new SEIActiveParameterSets(); 
	seiActiveParameterSets->activeVPSId = em_pcEncTop->getVPS()->getVPSId(); 
	seiActiveParameterSets->m_selfContainedCvsFlag = false;
	seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
	seiActiveParameterSets->numSpsIdsMinus1 = 0;
	seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1); 
	seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
	return seiActiveParameterSets;
}

SEIFramePacking* TEncFrame::ETRI_xCreateSEIFramePacking()
{
	SEIFramePacking *seiFramePacking = new SEIFramePacking();
	seiFramePacking->m_arrangementId = em_pcEncTop->getFramePackingArrangementSEIId();
	seiFramePacking->m_arrangementCancelFlag = 0;
	seiFramePacking->m_arrangementType = em_pcEncTop->getFramePackingArrangementSEIType();
	assert((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) );
	seiFramePacking->m_quincunxSamplingFlag = em_pcEncTop->getFramePackingArrangementSEIQuincunx();
	seiFramePacking->m_contentInterpretationType = em_pcEncTop->getFramePackingArrangementSEIInterpretation();
	seiFramePacking->m_spatialFlippingFlag = 0;
	seiFramePacking->m_frame0FlippedFlag = 0;
	seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
	seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && em_pcGOPEncoder->ETRI_getnumPicCoded()&1);
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

SEIDisplayOrientation* TEncFrame::ETRI_xCreateSEIDisplayOrientation()
{
	SEIDisplayOrientation *seiDisplayOrientation = new SEIDisplayOrientation();
	seiDisplayOrientation->cancelFlag = false;
	seiDisplayOrientation->horFlip = false;
	seiDisplayOrientation->verFlip = false;
	seiDisplayOrientation->anticlockwiseRotation = em_pcEncTop->getDisplayOrientationSEIAngle();
	return seiDisplayOrientation;
}

SEIToneMappingInfo*  TEncFrame::ETRI_xCreateSEIToneMappingInfo()
{
	SEIToneMappingInfo *seiToneMappingInfo = new SEIToneMappingInfo();
	seiToneMappingInfo->m_toneMapId = em_pcEncTop->getTMISEIToneMapId();
	seiToneMappingInfo->m_toneMapCancelFlag = em_pcEncTop->getTMISEIToneMapCancelFlag();
	seiToneMappingInfo->m_toneMapPersistenceFlag = em_pcEncTop->getTMISEIToneMapPersistenceFlag();

	seiToneMappingInfo->m_codedDataBitDepth = em_pcEncTop->getTMISEICodedDataBitDepth();
	assert(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14);
	seiToneMappingInfo->m_targetBitDepth = em_pcEncTop->getTMISEITargetBitDepth();
	assert( seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17 );
	seiToneMappingInfo->m_modelId = em_pcEncTop->getTMISEIModelID();
	assert(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4);

	switch( seiToneMappingInfo->m_modelId)
	{
	case 0:
	{
			  seiToneMappingInfo->m_minValue = em_pcEncTop->getTMISEIMinValue();
			  seiToneMappingInfo->m_maxValue = em_pcEncTop->getTMISEIMaxValue();
			  break;
	}
	case 1:
	{
			  seiToneMappingInfo->m_sigmoidMidpoint = em_pcEncTop->getTMISEISigmoidMidpoint();
			  seiToneMappingInfo->m_sigmoidWidth = em_pcEncTop->getTMISEISigmoidWidth();
			  break;
	}
	case 2:
	{
			  UInt num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
			  seiToneMappingInfo->m_startOfCodedInterval.resize(num);
			  Int* ptmp = em_pcEncTop->getTMISEIStartOfCodedInterva();
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
			  seiToneMappingInfo->m_numPivots = em_pcEncTop->getTMISEINumPivots();
			  seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
			  seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
			  Int* ptmpcoded = em_pcEncTop->getTMISEICodedPivotValue();
			  Int* ptmptarget = em_pcEncTop->getTMISEITargetPivotValue();
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
			  seiToneMappingInfo->m_cameraIsoSpeedIdc = em_pcEncTop->getTMISEICameraIsoSpeedIdc();
			  seiToneMappingInfo->m_cameraIsoSpeedValue = em_pcEncTop->getTMISEICameraIsoSpeedValue();
			  assert( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 );
			  seiToneMappingInfo->m_exposureIndexIdc = em_pcEncTop->getTMISEIExposurIndexIdc();
			  seiToneMappingInfo->m_exposureIndexValue = em_pcEncTop->getTMISEIExposurIndexValue();
			  assert( seiToneMappingInfo->m_exposureIndexValue !=0 );
			  seiToneMappingInfo->m_exposureCompensationValueSignFlag = em_pcEncTop->getTMISEIExposureCompensationValueSignFlag();
			  seiToneMappingInfo->m_exposureCompensationValueNumerator = em_pcEncTop->getTMISEIExposureCompensationValueNumerator();
			  seiToneMappingInfo->m_exposureCompensationValueDenomIdc = em_pcEncTop->getTMISEIExposureCompensationValueDenomIdc();
			  seiToneMappingInfo->m_refScreenLuminanceWhite = em_pcEncTop->getTMISEIRefScreenLuminanceWhite();
			  seiToneMappingInfo->m_extendedRangeWhiteLevel = em_pcEncTop->getTMISEIExtendedRangeWhiteLevel();
			  assert( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 );
			  seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = em_pcEncTop->getTMISEINominalBlackLevelLumaCodeValue();
			  seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = em_pcEncTop->getTMISEINominalWhiteLevelLumaCodeValue();
			  assert( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue );
			  seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = em_pcEncTop->getTMISEIExtendedWhiteLevelLumaCodeValue();
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

Void TEncFrame::ETRI_xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps)
{
	OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

	if(em_pcEncTop->getActiveParameterSetsSEIEnabled())
	{
		SEIActiveParameterSets *sei = ETRI_xCreateSEIActiveParameterSets (sps);

		//nalu = NALUnit(NAL_UNIT_SEI); 
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		delete sei;
		em_activeParameterSetSEIPresentInAU = true;
	}

	if(em_pcEncTop->getFramePackingArrangementSEIEnabled())
	{
		SEIFramePacking *sei = ETRI_xCreateSEIFramePacking ();

		nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		delete sei;
	}
	if (em_pcEncTop->getDisplayOrientationSEIAngle())
	{
		SEIDisplayOrientation *sei = ETRI_xCreateSEIDisplayOrientation();

		nalu = NALUnit(NAL_UNIT_PREFIX_SEI); 
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		delete sei;
	}
	if(em_pcEncTop->getToneMappingInfoSEIEnabled())
	{
		SEIToneMappingInfo *sei = ETRI_xCreateSEIToneMappingInfo ();

		nalu = NALUnit(NAL_UNIT_PREFIX_SEI); 
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		delete sei;
	}

	// [HDR/WCG Encoding] VUI/SEI parameter setting (by Dongsan Jun, 20160124)
#if ETRI_HDR_WCG_ENCODER
	if (em_pcEncTop->getMasteringDisplaySEI().colourVolumeSEIEnabled)
	{
		const TComSEIMasteringDisplay &seiCfg = em_pcEncTop->getMasteringDisplaySEI();
		SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
		sei->values = seiCfg;

		nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
		em_cEntropyCoder.setBitstream(&nalu.m_Bitstream);
		em_cseiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
		writeRBSPTrailingBits(nalu.m_Bitstream);
		accessUnit.push_back(new NALUnitEBSP(nalu));
		delete sei;
	}
#endif
}

Void TEncFrame::ETRI_GetRefPic(ETRI_RefPic_t *pRefPic, Int pocCurr, TComPic* pcPic, TComList<TComPic*>& rcListPic, int iGOPid, bool bisField, int iPOCLast, int iNumPicRcvd, int iLastIDR)
{
	TComSlice*	pcSlice			= nullptr;

	TComRefPicListModification* refPicListModification = nullptr;

	//=================================================================================
	// Initial to start encoding
	//=================================================================================
	UInt uiColDir = 1;

	uiColDir=ETRI_Select_UiColDirection(iGOPid, uiColDir);					/// select uiColDir : 2015 5 11 by Seok


	//	Slice data initialization
	pcPic->clearSliceBuffer();
	assert(pcPic->getNumAllocatedSlice() == 1);
	em_cSliceEncoder.setSliceIdx(0);
	pcPic->setCurrSliceIdx(0);
	
#if ETRI_MultiplePPS

#if 1

#if 0
	Int ipps_id = 0;
	if (pocCurr == 0 || pocCurr == 8 || pocCurr == 16)
		ipps_id = 0;
	else if (pocCurr == 4 || pocCurr == 12 || pocCurr == 24)
		ipps_id = 1;
	else
		ipps_id = 2;
#else

	Int ipps_id = 0;
	if (pocCurr == 0 || pocCurr == 8)
		ipps_id = 0;
	else if (pocCurr == 1 || pocCurr == 3 || pocCurr == 9)
		ipps_id = 2;
	else
		ipps_id = 1;

#endif

#else
	Int ipps_id = 1;
#endif


	if (em_pcEncTop->ETRI_getNumAdditionalPPS() == 0)
		ipps_id = 0;
	em_cSliceEncoder.initEncSlice(pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, em_pcEncTop->getSPS(), em_pcEncTop->getPPS(ipps_id), bisField);
#else
	em_cSliceEncoder.initEncSlice(pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, em_pcEncTop->getSPS(), em_pcEncTop->getPPS(), bisField);
#endif	

	

	//Set Frame/Field coding
	pcSlice->getPic()->setField(bisField);
	pcSlice->setLastIDR(iLastIDR);
	pcSlice->setSliceIdx(0);

	refPicListModification = pcSlice->getRefPicListModification();				///Get RefPicList for each Slice (ot Picture) If you want to Slice Paralleization, This Parameter should be multiple allocated. @2015 5 14 by Seok 
	ETRI_SliceDataInitialization(pcPic, pcSlice);
	ETRI_SetReferencePictureSetforSlice(pcPic, pcSlice, iGOPid, pocCurr, bisField, rcListPic);
	ETRI_refPicListModification(pcSlice, refPicListModification, rcListPic, iGOPid, uiColDir);
	ETRI_NoBackPred_TMVPset(pcSlice, iGOPid);


	int nCnt = 0;
	int iRefIdx;

	pRefPic->nPOC = pocCurr;
	pRefPic->nRefNum = pcSlice->getNumRefIdx(REF_PIC_LIST_0) + pcSlice->getNumRefIdx(REF_PIC_LIST_1);
	pRefPic->pRefPOC = new int[pRefPic->nRefNum];

	for(iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0); iRefIdx++) 
	{
		pRefPic->pRefPOC[nCnt++] = pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC();		
	}

	for(iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1); iRefIdx++) 
	{
		pRefPic->pRefPOC[nCnt++] = pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC();
	}
}
#endif


//! \}
