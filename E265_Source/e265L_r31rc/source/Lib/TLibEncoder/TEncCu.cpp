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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"

#include <cmath>
#include <algorithm>
using namespace std;


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight)
{
	Int i;

	m_uhTotalDepth   = uhTotalDepth + 1;
	m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
	m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];

	m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
	m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
	m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
	m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
	m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
	m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
	m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];

	UInt uiNumPartitions;
	for( i=0 ; i<m_uhTotalDepth-1 ; i++)
	{
		uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
		UInt uiWidth  = uiMaxWidth  >> i;
		UInt uiHeight = uiMaxHeight >> i;

		m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
		m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );

		m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight);
		m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight);
		m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight);

		m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight);
		m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight);
		m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight);

		m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight);
	}

	m_bEncodeDQP = false;

	// initialize partition order.
	UInt* piTmp = &g_auiZscanToRaster[0];
	initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
	initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );

	// initialize conversion matrix from partition index to pel
	initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );

	//Initialize ETRI's member function
	ETRI_createCU (uhTotalDepth, uiMaxWidth, uiMaxHeight);

}

Void TEncCu::destroy()
{
	Int i;

	//Release ETRI's member data
	ETRI_destroyCU();

	for( i=0 ; i<m_uhTotalDepth-1 ; i++)
	{
		if(m_ppcBestCU[i])
		{
			m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
		}
		if(m_ppcTempCU[i])
		{
			m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
		}
		if(m_ppcPredYuvBest[i])
		{
			m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
		}
		if(m_ppcResiYuvBest[i])
		{
			m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
		}
		if(m_ppcRecoYuvBest[i])
		{
			m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
		}
		if(m_ppcPredYuvTemp[i])
		{
			m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
		}
		if(m_ppcResiYuvTemp[i])
		{
			m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
		}
		if(m_ppcRecoYuvTemp[i])
		{
			m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
		}
		if(m_ppcOrigYuv[i])
		{
			m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
		}
	}

	if(m_ppcBestCU)
	{
		delete [] m_ppcBestCU;	m_ppcBestCU = NULL;
	}
	if(m_ppcTempCU)
	{
		delete [] m_ppcTempCU;	m_ppcTempCU = NULL;
	}
	if(m_ppcPredYuvBest)
	{
		delete [] m_ppcPredYuvBest;	m_ppcPredYuvBest = NULL;
	}
	if(m_ppcResiYuvBest)
	{
		delete [] m_ppcResiYuvBest;	m_ppcResiYuvBest = NULL;
	}
	if(m_ppcRecoYuvBest)
	{
		delete [] m_ppcRecoYuvBest;	m_ppcRecoYuvBest = NULL;
	}
	if(m_ppcPredYuvTemp)
	{
		delete [] m_ppcPredYuvTemp;	m_ppcPredYuvTemp = NULL;
	}
	if(m_ppcResiYuvTemp)
	{
		delete [] m_ppcResiYuvTemp;	m_ppcResiYuvTemp = NULL;
	}
	if(m_ppcRecoYuvTemp)
	{
		delete [] m_ppcRecoYuvTemp;	m_ppcRecoYuvTemp = NULL;
	}
	if(m_ppcOrigYuv)
	{
		delete [] m_ppcOrigYuv;	m_ppcOrigYuv = NULL;
	}

}

/** \param    pcEncTop      pointer of encoder class
 */
#if ETRI_MULTITHREAD_2
Void TEncCu::init( TEncTop* pcEncTop, TEncFrame* pcEncFrame )
#else
Void TEncCu::init( TEncTop* pcEncTop )
#endif
{
	m_pcEncCfg           = pcEncTop;

#if ETRI_MULTITHREAD_2
  m_pcPredSearch       = pcEncFrame->ETRI_getPredSearch();
  m_pcTrQuant          = pcEncFrame->ETRI_getTrQuant();
  m_pcBitCounter       = pcEncFrame->ETRI_getBitCounter();
  m_pcRdCost           = pcEncFrame->ETRI_getRdCost();
  
  m_pcEntropyCoder     = pcEncFrame->ETRI_getEntropyCoder();
  m_pcCavlcCoder       = pcEncFrame->ETRI_getCavlcCoder();
  m_pcSbacCoder		   = pcEncFrame->ETRI_getSbacCoder();
  m_pcBinCABAC         = pcEncFrame->ETRI_getBinCABAC();
  
  m_pppcRDSbacCoder   = pcEncFrame->ETRI_getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncFrame->ETRI_getRDGoOnSbacCoder();

#if KAIST_RC
  m_pcRateCtrl        = pcEncFrame->ETRI_getRateCtrl();
#endif

#else
	m_pcPredSearch       = pcEncTop->getPredSearch();
	m_pcTrQuant          = pcEncTop->getTrQuant();
	m_pcBitCounter       = pcEncTop->getBitCounter();
	m_pcRdCost           = pcEncTop->getRdCost();

	m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
	m_pcCavlcCoder       = pcEncTop->getCavlcCoder();
	m_pcSbacCoder       = pcEncTop->getSbacCoder();
	m_pcBinCABAC         = pcEncTop->getBinCABAC();

	m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
	m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();

#if KAIST_RC
	m_pcRateCtrl        = pcEncTop->getRateCtrl();
#endif
#endif

	//---------------------------------------------------------------------------
	//	ETRI Modification for V02 and V03 
	//---------------------------------------------------------------------------

	//ESPRINTF( ETRI_MODIFICATION_V02, stderr, "   Compiled @%s  [%s] \n", __DATE__, __TIME__);

}

// ====================================================================================================================
// 	ETRI TEncCU Initilization
// ====================================================================================================================
/**
-----------------------------------------------------------------------------------------------------------------------
	@brief: Memory allocation for ETRI's class member data in TEncCu::create
	@param: 
	@Author: Jinwuk Seok @ 2012.09.18
-----------------------------------------------------------------------------------------------------------------------
*/
void TEncCu::ETRI_createCU 	( UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight )
{
	UInt uiNumPartitions;
	UInt uiWidth, uiHeight;
	Int j, i;

	//--------------------------------------------------------------------------------
	//	Initilization for ETRI's TEncCU Class and Array variables
	//--------------------------------------------------------------------------------

	m_pppcAxTempCU					= new TComDataCU**[ETRI_nAXTempCU]; 	
	m_pppcAxPredYuvTemp   			= new TComYuv**[ETRI_nAXTempCU];

	for(j=0; j<ETRI_nAXTempCU; j++)
	{
		m_pppcAxTempCU[j]			= new TComDataCU*[m_uhTotalDepth-1];
		m_pppcAxPredYuvTemp[j]		= new TComYuv*[m_uhTotalDepth-1];

		for( i=0 ; i<m_uhTotalDepth-1 ; i++)
		{
			uiNumPartitions = 1 << ((m_uhTotalDepth - i - 1) << 1);
			uiWidth  = uiMaxWidth  >> i;
			uiHeight = uiMaxHeight >> i;

			m_pppcAxTempCU[j][i]	   	= new TComDataCU; m_pppcAxTempCU[j][i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) ); 
			m_pppcAxPredYuvTemp[j][i]	= new TComYuv; m_pppcAxPredYuvTemp[j][i]->create(uiWidth, uiHeight);
		}
	}

	//--------------------------------------------------------------------------------
	//	TencCu,h 에 선언되어 있으나, TileLevel 에서 사용되지 않는 변수는 여기에서 선언되어야 한다.
	//	특히 포인터를 위한 포인터 변수는 무조건 여기에서 선언되어야 한다.
	// 	안 그러면 cEncoderCU 클래스 때문에 문제가 발생할 수도 있다.
	//--------------------------------------------------------------------------------

	em_pcBestUpCU = new TComDataCU*[m_uhTotalDepth-1];
	for(i=0 ; i < m_uhTotalDepth-1; i++) {
		em_pcBestUpCU[i] = nullptr;
	}

	//--------------------------------------------------------------------------------
	//	Initilization for ETRI's TEncCU member variables
	//--------------------------------------------------------------------------------
	em_inumValidMergeCand   		= 0;
	em_iRDOOffBestMergeCand  		= -1;
	em_uiSMHADDistortion  			= 0;
	em_uiITHADDistortion  			= 0;
	em_uiINHADDistortion  			= 0;

	em_bLCUSkipFlag  				= false;
	em_bActiveCheckBestCU   		= false;	
	em_bActivexCheckETRIBestMode	= false;

	em_bControlParam				= nullptr;	
	em_puhInterDirNeighbours 		= nullptr;
	em_pcMvFieldNeighbours  		= nullptr;
	em_pimergeCandBuffer			= nullptr;
	em_uiPartUnitIdx   				= nullptr;
	em_bDepthSkipMode   			= nullptr;
	em_bSkipMode     				= nullptr;
	em_pbdoNotBlockPU   			= nullptr;	///< Sensing Pointer so that there is not memory allocation and release 
	em_uiSKLevel					= nullptr;
	em_uiMGLevel					= nullptr;
	em_uiINLevel					= nullptr;
	em_uiITLevel					= nullptr;
	em_uiMINLevel					= nullptr;	

	em_uiLevelInfo     				= nullptr;
	em_uiHADInfo     				= nullptr;
	em_bESD     					= nullptr;

	em_qp  						    = nullptr;
	em_uiMINHADDistortion  			= nullptr;
#if ETRI_DEBUG_CODE_CLEANUP
	em_DbgInfo  					= nullptr;
#endif 
	em_dACCPartRDCost 			    = nullptr; 	/// Bug Patch @ 2015 12 15 by Seok
	em_pdACCEstRDCost 			    = nullptr; 	/// Bug Patch @ 2015 12 15 by Seok
	em_uiACCPartDistortion 			= nullptr; 	/// Bug Patch @ 2015 12 15 by Seok

#if ETRI_CU_MODE_INHERITANCE
	em_bDoInterModeFlag = true;
#endif 

}

/**
-----------------------------------------------------------------------------------------------------------------------
	@brief: Memory release for ETRI's class member data in TEncCu::destroy
	@param: 
	@Author: Jinwuk Seok @ 2012.09.18
-----------------------------------------------------------------------------------------------------------------------
*/
void TEncCu::ETRI_destroyCU	()
{
	Int i, j;

	//--------------------------------------------------------------------------------
	//	Destroy for Tile Level
	//--------------------------------------------------------------------------------
	ETRI_Release_TileLevel ();

	//--------------------------------------------------------------------------------
	//	Destroy for Create Level
	//--------------------------------------------------------------------------------	
	if (m_pppcAxTempCU)
	{
		for(j=0; j<ETRI_nAXTempCU; j++) {
			if(m_pppcAxTempCU[j])
			{
				for(i=0; i<m_uhTotalDepth-1; i++) {
					if(m_pppcAxTempCU[j][i]) {m_pppcAxTempCU[j][i]->destroy();	delete m_pppcAxTempCU[j][i];	m_pppcAxTempCU[j][i] = nullptr;}
				}
				delete [] m_pppcAxTempCU[j]; m_pppcAxTempCU[j] = nullptr;
			}
		}
		delete [] m_pppcAxTempCU; m_pppcAxTempCU = nullptr;
	}	

	if(m_pppcAxPredYuvTemp)
	{
		for(j=0; j<ETRI_nAXTempCU; j++) {
			if(m_pppcAxPredYuvTemp[j])
			{
				for(i=0; i<m_uhTotalDepth-1; i++) {
					if(m_pppcAxPredYuvTemp[j][i]) {m_pppcAxPredYuvTemp[j][i]->destroy(); delete m_pppcAxPredYuvTemp[j][i]; m_pppcAxPredYuvTemp[j][i] = nullptr;}
				}
				delete [] m_pppcAxPredYuvTemp[j]; m_pppcAxPredYuvTemp[j] = nullptr;
			}
		}
		delete [] m_pppcAxPredYuvTemp; m_pppcAxPredYuvTemp = nullptr;
	}

	if (em_pcBestUpCU)
	{
		for(i=0 ; i < m_uhTotalDepth-1; i++){
			if (em_pcBestUpCU[i]) { 
				em_pcBestUpCU[i] = nullptr;
			}
		}			
		DeleteDimParam(em_pcBestUpCU);
	}
}


/**
-----------------------------------------------------------------------------------------------------------------------
	@brief: ETRI Multiple Threaded Tile:  
	@param: 
	@Author: yhee @ 2012.09.18
-----------------------------------------------------------------------------------------------------------------------
*/
Void TEncCu::ETRI_initForTiles ( TEncTop* pcEncTop,
							  TEncSearch* pcPredSearch,
							  TComTrQuant*  pcTrQuant,      
							  TComBitCounter* pcBitCounter,
							  TEncEntropy*  pcEntropyCoder,
							  TComRdCost*   pcRdCost,
                              TEncSbac*** pppcRDSbacCoder,
                              TEncSbac*   pcRDGoOnSbacCoder)
{
	m_pcEncCfg 			= pcEncTop;
	m_pcPredSearch 		= pcPredSearch;
	m_pcTrQuant   		= pcTrQuant;
	m_pcBitCounter 		= pcBitCounter;
	m_pcEntropyCoder  	= pcEntropyCoder;
	m_pcRdCost 			= pcRdCost;  
	
	m_pppcRDSbacCoder	= pppcRDSbacCoder;
	m_pcRDGoOnSbacCoder	= pcRDGoOnSbacCoder;

	ETRI_Init_TileLevel ();
}

/**
-----------------------------------------------------------------------------------------------------------------------
	@brief: ETRI Multiple Threaded Tile for LCU rate Control:  
	@param: 
	@Author: yhee @ 2014.03.18
-----------------------------------------------------------------------------------------------------------------------
*/
Void TEncCu::ETRI_initForTiles (TEncCavlc*  	pcCavlcCoder,
							TEncSbac*  		pcSbacCoder,
							TEncBinCABAC* 	pcBinCABAC 
#if KAIST_RC
							,TEncRateCtrl* 	pcRateCtrl
#endif
              )
{
		m_pcCavlcCoder	 = pcCavlcCoder;
		m_pcSbacCoder	 = pcSbacCoder; 
		m_pcBinCABAC	 = pcBinCABAC;	
#if KAIST_RC
		m_pcRateCtrl   	 = pcRateCtrl;
#endif
}

/**
==================================================================
	@Brief :  	This function is called at the Tile Initilization. 
			It is a main initilization function for the fast algorithm in TEncCU \ 
			Same Function : TEncCu::ETRI_FCU_DataAllocation \
			Same Function : TEncCu::ETRI_FCU_Destroy \
			Pair Function : TEncCu::ETRI_Release_TileLevel \
			Initialization Level  : Tile Initilization of CU  \
==================================================================
*/
Void TEncCu::ETRI_Init_TileLevel (void)
{
#if	ETRI_MODIFICATION_V02
#if ETRI_DEBUG_CODE_CLEANUP
	em_DbgInfo  	  	= (UInt	*)calloc(ETRI_nDbgInfo, sizeof(Int));
#endif 
	//---------------------------------------------------------------------------
	//	Algorithm Control Parameter
	//---------------------------------------------------------------------------
	em_bControlParam	= (Bool	*)calloc(ETRI_nControlParam, sizeof(bool));

	em_bDepthSkipMode = (Bool **)calloc(MAX_CU_DEPTH, sizeof(bool *));
	for(Int e_iMode=0; e_iMode < MAX_CU_DEPTH; e_iMode++)
		em_bDepthSkipMode[e_iMode] = (Bool *)calloc(ETRI_nAXTempCU, sizeof(bool));

	em_bESD = new Bool 	[MAX_CU_DEPTH];

	//---------------------------------------------------------------------------
	//	SKIP/MERGE Algorithm
	//---------------------------------------------------------------------------
	em_puhInterDirNeighbours 	= (UChar *)calloc(MRG_MAX_NUM_CANDS, sizeof(char));
	em_pcMvFieldNeighbours  	= (TComMvField *)calloc((MRG_MAX_NUM_CANDS << 1), sizeof(TComMvField));
	em_pimergeCandBuffer		= (Int *)calloc(MRG_MAX_NUM_CANDS, sizeof(Int));


	//---------------------------------------------------------------------------
	//	General Fast Algorithm parameter
	//---------------------------------------------------------------------------
	em_uiSKLevel				= new UInt 	[MAX_CU_DEPTH]; 
	em_uiMGLevel				= new UInt 	[MAX_CU_DEPTH]; 
	em_uiINLevel 				= new UInt 	[MAX_CU_DEPTH]; 
	em_uiITLevel 				= new UInt 	[MAX_CU_DEPTH]; 
	em_uiMINLevel				= new UInt 	[MAX_CU_DEPTH]; 
	em_uiMINHADDistortion 		= new UInt 	[MAX_CU_DEPTH]; 
	em_uiACCPartDistortion 		= new UInt 	[MAX_CU_DEPTH]; 
	em_dACCPartRDCost 		    = new Double [MAX_CU_DEPTH]; 
	em_pdACCEstRDCost 		    = new Double [MAX_CU_DEPTH]; 

	em_uiLevelInfo  			= new UInt 	[ETRI_nLevelInfo];	///< 2015 3 23 by Seok : ETRI_nDBGInfo_CULevelScope= 8. If you want another predefinition, you can change it

	em_qp   					= new QpParam [ETRI_nAXTempCU];
	for(Int e_iMode=0; e_iMode < ETRI_nAXTempCU; e_iMode++){em_qp[e_iMode].clear();}

	/// HAD LUMA/ HAD Cb/ HAD Cr / Reserve :: For Fast Prediction 
	em_uiHADInfo				= new UInt* [ETRI_IdColorComponent];									
	for(Int e_iMode=0; e_iMode < ETRI_IdColorComponent; e_iMode++) {
		em_uiHADInfo[e_iMode] = new UInt [ETRI_IdColorComponent];
		memset(em_uiHADInfo[e_iMode], 0, ETRI_IdColorComponent * sizeof(Int));	
	}	

	//---------------------------------------------------------------------------
	//	Fast CU Depth Decision/Prunning
	//---------------------------------------------------------------------------
	em_uiPartUnitIdx			= new UInt	[MAX_CU_DEPTH]; 		

	memset(em_uiPartUnitIdx, 0, MAX_CU_DEPTH * sizeof(UInt));
#if ETRI_DEBUG_CODE_CLEANUP
	memset(em_DbgInfo, 0, ETRI_nDbgInfo * sizeof(Int));
#endif 
#endif
}

Void TEncCu::ETRI_Release_TileLevel (void)
{
#if	ETRI_MODIFICATION_V02
	if (em_bDepthSkipMode){
		for(Int e_iMode=0; e_iMode < MAX_CU_DEPTH; e_iMode++) {
			ETRI_MallocFree(em_bDepthSkipMode[e_iMode]);
		}
		ETRI_MallocFree(em_bDepthSkipMode);
	}

	if (em_uiHADInfo) {
		for(Int e_iMode=0; e_iMode < ETRI_IdColorComponent; e_iMode++) {
			DeleteDimParam(em_uiHADInfo[e_iMode]);
		}
		DeleteDimParam(em_uiHADInfo);
	}
#if ETRI_DEBUG_CODE_CLEANUP
	ETRI_MallocFree(em_DbgInfo);
#endif 
	ETRI_MallocFree(em_bControlParam);
	ETRI_MallocFree(em_puhInterDirNeighbours);
	ETRI_MallocFree(em_pcMvFieldNeighbours);
	ETRI_MallocFree(em_pimergeCandBuffer);

	DeleteDimParam(em_bESD);
	DeleteDimParam(em_uiSKLevel);
	DeleteDimParam(em_uiMGLevel);
	DeleteDimParam(em_uiINLevel);
	DeleteDimParam(em_uiITLevel);
	DeleteDimParam(em_uiLevelInfo);
	DeleteDimParam(em_uiPartUnitIdx);
	DeleteDimParam(em_qp);
	DeleteDimParam(em_uiMINLevel);
	DeleteDimParam(em_uiACCPartDistortion);
	DeleteDimParam(em_dACCPartRDCost);
	DeleteDimParam(em_uiMINHADDistortion);
	DeleteDimParam(em_pdACCEstRDCost);
#endif
}

/**
==================================================================
	@Brief :  	This function is called out of TEncCU. (ex: TEncTile, or TEncSlice). 
			It is a Debugfunction for the fast algorithm in TEncCU \ 
			Same Function : TEncCu::ETRI_FCU_Init_onPictureSliceTile \
			Same Function : TEncCu::ETRI_FCU_Final_onPictureSliceTile \
			Initialization Level  : Picture (Frame)/Slice /Tile  \
==================================================================
*/
Void TEncCu::ETRI_Init_onPictureSliceTile (TComSlice* pcSlice)
{
#if	ETRI_MODIFICATION_V02
#if ETRI_DEBUG_CODE_CLEANUP
	memset(em_DbgInfo, 0, ETRI_nDBGInfo_ChkPoint * sizeof(UInt));
#endif 
#if _YHEEDEBUG 
	EDPRINTF(stderr, " bSetQpParam %d, Initial QpParam:%d\n", m_pcTrQuant->getQParam().qp());
#endif
#if	(ETRI_MODIFICATION_V03 && ETRI_FASTPUProcessing)
	em_ucFPUInterSkipLevel[0] = g_FPUInterSkipLevel[0][pcSlice->getSliceQp()];
	em_ucFPUInterSkipLevel[1] = g_FPUInterSkipLevel[1][pcSlice->getSliceQp() - pcSlice->getDepth()];
#endif
#endif
}

Void TEncCu::ETRI_Final_onPictureSliceTile (TComSlice* pcSlice, UInt uiTileIdx)
{
#if	ETRI_MODIFICATION_V02
#if ETRI_DEBUG_CODE_CLEANUP
	if (0)
	{
		EDPRINTF(stderr, " ======= Tile ID : %d =======\n", uiTileIdx);
		EDPRINTF(stderr, " Total Case  : %d \n", em_DbgInfo[ETRI_nDBGInfo_TotalInfo]);
		EDPRINTF(stderr, " HADQ Zero   : %d \n", em_DbgInfo[ETRI_nDBGInfo_CASE01]);
		EDPRINTF(stderr, " ESD Active  : %d \n", em_DbgInfo[ETRI_nDBGInfo_CASE02]);
		EDPRINTF(stderr, " Same Case   : %d \n", em_DbgInfo[ETRI_nDBGInfo_CASE03]);
		EDPRINTF(stderr, " HADQ Zero ESD inActive   : %d \n", em_DbgInfo[ETRI_nDBGInfo_CASE04]);
		EDPRINTF(stderr, " HADQ NOZero ESD Active   : %d \n", em_DbgInfo[ETRI_nDBGInfo_CASE05]);
	}
#endif 
#endif
}

/**
==================================================================
	@Brief :  Initilization of Fast Algorithm for TEncCU for the Top Depth level \	
			This function is called just before the ETRI_xcompressCU is called. \ 
			Same Function : TEncCu::ETRI_FCU_Init_onEachCU \
			Initialization Level  : CU (Not Sub CU) \
==================================================================
*/
__inline Void TEncCu::ETRI_Init_CUTopLevel	(TComDataCU*& rpcCU)
{
#if	ETRI_MODIFICATION_V02
	m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
	m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
	m_pppcAxTempCU[ETRI_IdAxTempCU_Inter][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
	m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
	m_pppcAxTempCU[ETRI_IdAxTempCU_Inter2NxN][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
	m_pppcAxTempCU[ETRI_IdAxTempCU_InterNx2N][0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
#endif 
#if ETRI_ADAPTIVE_MAXCTU_SIZE
	em_bLCUSkipFlag |= !(rpcCU->getSlice()->getSliceType() != I_SLICE && rpcCU->getSlice()->getDepth() > ETRI_F64SLiceLevel); 
#else
	em_bLCUSkipFlag = ETRI_SKIP_64x64LCU;  	///< [TRUE] SKIP 64x64 LCU [FALSE:Original] No SKIP
#endif	
#endif
#if	ETRI_MODIFICATION_V03
	em_uiACCPartDistortion[0] = 0;
	em_dACCPartRDCost[0] = 0;
	em_uiPartUnitIdx[0] = 0;

	for(Int i=0; i < m_uhTotalDepth - 1; i++)
	em_pcBestUpCU[i] = nullptr;

#endif
}


/**
==================================================================
	@Brief  : Initilization of Fast Algorithm for TEncCU for a Depth level without Top Depth \
                     This function is called at the early part of ETRI_SubCUProcessing_inPicture \
			same Function : TEncCu::ETRI_FCU_Init_onEachSubCU	
			Initialization Level  : Sub CU : PartId = 0
==================================================================
*/
__inline Void TEncCu::ETRI_Init_SubCULevel(TComDataCU*& rpcBestCU, UInt uhNextDepth)
{
#if	ETRI_MODIFICATION_V03
	em_uiACCPartDistortion[uhNextDepth] = 0;
	em_dACCPartRDCost[uhNextDepth] = 0;

	em_pcBestUpCU[uhNextDepth] = rpcBestCU;
#endif
}

/**
-------------------------------------------------------------------------------------
	@Brief  : Initilization of Fast Algorithm for TEncCU for a Partition level in SubCu Processing \
                     This function is called in the for loop with the uiPartUnitIdx  \
			same Function : None. Look at the ETRI_SubCUProcessing_inPicture	
			Initialization Level  : Sub CU Partition
-------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_Init_SubCUPartitionLevel(TComDataCU*& rpcBestCU, UInt uiPartUnitIdx, Int iQP, UInt uhNextDepth, UInt uiDepth)
{
#if	ETRI_MODIFICATION_V02

	if (rpcBestCU->getSlice()->getSliceType() != I_SLICE){
	m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  
	m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  
	m_pppcAxTempCU[ETRI_IdAxTempCU_Inter][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_Inter][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
    m_pppcAxTempCU[ETRI_IdAxTempCU_Inter2NxN][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_Inter2NxN][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  
	m_pppcAxTempCU[ETRI_IdAxTempCU_InterNx2N][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_InterNx2N][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  
#endif 
    }
	m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][uhNextDepth]->initSubCU(m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][uiDepth], uiPartUnitIdx, uhNextDepth, iQP );  

	em_uiPartUnitIdx[uhNextDepth] = uiPartUnitIdx;

#endif
}

/**
===================================================================
	@Brief  : Initilization of Fast Algorithm for TEncCU for each part  \
                     This function is called at the early part of ETRI_xcomressCU \
                     Main Initialization of Fast Algorithm for TEncCU \
			same Function : TEncCu::ETRI_FCU_SetSimplePartIdx	
	              Initialization Level  : CU partition on same Depth (Not Sub CU) 
===================================================================
*/
__inline	Void TEncCu::ETRI_Init_CUPartitionLevel (TComDataCU*& rpcCU, UInt uiDepth)
{
#if	ETRI_MODIFICATION_V02

	//--------------------------------------------------------------------------------
	//	Initilization em_bControlParam
	//--------------------------------------------------------------------------------
	TComSlice* pcSlice = rpcCU->getSlice();

	memset(em_bControlParam, 	false, ETRI_nControlParam * sizeof(Bool));
	em_bControlParam[ETRI_Id2NxNProcessing] = ETRI_ENABLE_2NxNNx2NProc;	/// [TRUE:Original] Turn ON Turn 2NxN Processing [FALSE] Turn off 2NxN Processing 
#if ETRI_ADAPTIVE_MAXCTU_SIZE
	em_bControlParam[ETRI_IdxMAXCTUSIZE]   = (pcSlice->getSliceType() != I_SLICE && uiDepth == 0 && pcSlice->getDepth() > ETRI_F64SLiceLevel);
#endif
	//--------------------------------------------------------------------------------
	//	Initilization em_bSkipMode
	//--------------------------------------------------------------------------------
	em_bSkipMode = em_bDepthSkipMode[uiDepth];
	memset(em_bSkipMode,  	false, ETRI_nAXTempCU * sizeof(Bool));

	//--------------------------------------------------------------------------------
	//	Initilization of Other Control Parameters per Partition
	//--------------------------------------------------------------------------------

	/// ETRI_POSTESD,  ETRI_FASTPOSTESD
	em_bESD[uiDepth] = false;

	em_pdACCEstRDCost[uiDepth] = 0;

	//--------------------------------------------------------------------------------
	//	Initilization of Parameters for Fast PU Prediction :: HADcost, LevelInfo... 
	//--------------------------------------------------------------------------------
	/// Set QP Parameters 

	Int e_iQP = 0;
	Int qpy = rpcCU->getQP( 0 );
	Int qpBdOffset = rpcCU->getSlice()->getSPS()->getQpBDOffsetY();
	Int qpScaled = Clip3( -qpBdOffset, 57, qpy);

	e_iQP = ((qpScaled < 0) ? qpScaled : g_aucChromaScale[qpScaled]) + qpBdOffset;

	Int e_iQPOffset[4] = {0, 0, 0, 0};

	/// Very Important Idea by seok 
#if 	ETRI_FASTPUProcessing
//	e_iQPOffset[ETRI_IdAxTempCU_Merge] =  (rpcCU->getQP( 0 ) > 29)?  6 : 0;
	e_iQPOffset[ETRI_IdAxTempCU_Merge] =  (UInt)g_FPUMergeSkipLevel[0][(UInt)rpcCU->getQP(0)];
	e_iQPOffset[ETRI_IdAxTempCU_Inter] =  (rpcCU->getQP( 0 ) > 29)?  6 : 0;
	e_iQPOffset[ETRI_IdAxTempCU_Intra] =  (rpcCU->getQP( 0 ) > 29)?  6 : 0;
#endif

	em_qp[ETRI_IdAxTempCU_Skip]  	.setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Skip]);
	em_qp[ETRI_IdAxTempCU_Merge] 	.setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Merge]);
	em_qp[ETRI_IdAxTempCU_Inter] 	.setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Inter]);
	em_qp[ETRI_IdAxTempCU_Intra] 	.setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Intra]);
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
	em_qp[ETRI_IdAxTempCU_Inter2NxN].setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Inter]);
	em_qp[ETRI_IdAxTempCU_InterNx2N].setQpParam(e_iQP - e_iQPOffset[ETRI_IdAxTempCU_Inter]);
#endif 

	/// Initilization of LevelInfo Data 
	em_uiLevelInfo[ETRI_IdWidTh]    	= rpcCU->getWidth(0);
	em_uiLevelInfo[ETRI_IdSliceDepth]	= rpcCU->getSlice()->getDepth();
	em_uiLevelInfo[ETRI_IdCUDepth]  	= uiDepth;
	memset(&em_uiLevelInfo[ETRI_IdCUDepth + 1], 0, (ETRI_nLevelInfo - ETRI_IdCUDepth - 1) * sizeof(Int));

#if ETRI_BUGFIX_Miss_Init_ETRI_Level
	/// Initilization of Temporal HAD value Storage
	memset(em_uiHADInfo[ETRI_IdAxTempCU_Skip], 0, ETRI_IdColorComponent * sizeof(Int));
	memset(em_uiHADInfo[ETRI_IdAxTempCU_Inter], 0, ETRI_IdColorComponent * sizeof(Int));

	/// Initilization of Level
	em_uiSKLevel[uiDepth] = MAX_INT;
	em_uiMGLevel[uiDepth] = MAX_INT;
	em_uiINLevel[uiDepth] = MAX_INT;
	em_uiITLevel[uiDepth] = MAX_INT;
	em_uiMINLevel[uiDepth] = MAX_INT;

	/// Initilization of HAD values @ 2015 11 19 by Seok
	em_uiSMHADDistortion = MAX_INT;
	em_uiITHADDistortion  = MAX_INT;
	em_uiINHADDistortion  = MAX_INT;
	
	em_uiMINHADDistortion[uiDepth] = MAX_INT;
#endif

#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
/** 
	\param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU( TComDataCU*& rpcCU )
{
	// initialize CU data
	m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
	m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

	//================ Debug Point ===================
	/*UInt SubDbgPhase = 0;		///< 2015 2 27 by Seok : For Debug
	ETRI_PrintDebugnfo(WHEREARG, rpcCU, rpcCU, 0, -1, SubDbgPhase, 1);
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
		EDPRINTF(stderr, "BEGIN DEBUG \n");*/
	//================ Debug Point ===================
	//yhee check the m_pcTrQuant
	/*Int e_iQP = m_pcTrQuant->getQParam().qp();
	if (rpcCU->getAddr()==0)
		EDPRINTF(stderr, "e_iQP: %d\n", e_iQP);*/

	// initialize ETRI data
	ETRI_Init_CUTopLevel(rpcCU);

	// analysis of CU
	ETRI_xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 );

#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
	if( m_pcEncCfg->getUseAdaptQpSelect() )
	{
		if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
		{
		xLcuCollectARLStats( rpcCU);
		}
	}
#endif
#endif
}

/** \param  pcCU  pointer of CU data class
 */
Void TEncCu::encodeCU ( TComDataCU* pcCU )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  // Encode CU data
  xEncodeCU( pcCU, 0, 0 );
}

#if ETRI_MODIFICATION_V02
// ====================================================================================================================
//   ETRI  Modification for E265 Version 2 and 3 
//
// ====================================================================================================================
/**
==============================================================================
	@brief : ENcoder 분석을 위한 보조함수 
	ETRI_ANALYSIS_ON
	0	No Analysis
	1 	Only Print Position Info with Addr, Depth, PartIdx, AbsPart and others
	2	Position Info + Final RD 
	3	Full Info including Prediction
==============================================================================
*/
#define	ETRI_ANALYSIS_ON	  	0//_YHEEDEBUG		
#define	ETRI_ANALYSIS_TYPE   	stdout
#define	CurrentDevelop 			0	
#define	FIND_ERROR_LOCATION 	0

#define	ETRI_CUTEST 			0

#define	ETRI_THCUPrun 			1.5
#define	SSEHADTRRatio 			2.37

#if (_ETRI_WINDOWS_APPLICATION)
#define	EDPRINTF2(Type, _fmt, ...) 	fprintf(Type, WHERESTR _fmt, WhereLine, WhereARG, __VA_ARGS__);
#define	ESPRINTF2(Key, Type, _fmt, ...) if (Key){fprintf(Type, WHERESTR _fmt, WhereLine, WhereARG, __VA_ARGS__);}
#else
#define	EDPRINTF2(Type, _fmt, ...) 	fprintf(Type, WHERESTR _fmt, WhereLine, WhereARG, ##__VA_ARGS__);
#define	ESPRINTF2(Key, Type, _fmt, ...) if (Key){fprintf(Type, WHERESTR _fmt, WhereLine, WhereARG, ##__VA_ARGS__);}
#endif
#define	PrintPREDMODE(x) ((x == 0)? "MODE_INTER" : ((x == 1)? "MODE_INTRA" : "MODE_NONE"))
#define	PrintHADMODE(x) ((x == 0)? "SKIP" : ((x == 1)? "MERGE" : ((x == 2)? "INTER": ((x == 3)? "INTER": "UnKnown"))))
/**
===============================================================================
	@brief  : Collection of DebugInfo
	@param: TComDataCU*& pcCU
	@param: UChar uhDepth
	@param: Int DebugPhase : <-1, return, -1 : Initial or Begin
	@param: char *WhereARG

	* When you set Debug Signal to TEncSearch, use following Code
	// Set Debug Info to PredSeracg Class
	m_pcPredSearch->ETRI_setDBGInfo(em_DbgInfo[ETRI_nDBGInfo_ChkPoint]);
===============================================================================
*/
#if ETRI_DEBUG_CODE_CLEANUP
__inline Void TEncCu::ETRI_PrintDebugnfo(int WhereLine, const char *WhereARG, TComDataCU* pcCU, TComDataCU* pcBestCU, 
										UInt	 uiDepth, Int DebugPhase, Int SubDbgPhase, Int iAnalysisType)

{
	if (iAnalysisType == 0 || DebugPhase < -1)   return;

	UInt	   		HADLuma, HADChromaCb, HADChromaCr, uiWidth, uinLumaPixels, uinChromaPixels;
	TComSlice*  	pcSlice = pcCU->getSlice();
	FILE*   	   	ETRI_STDMSG = ETRI_ANALYSIS_TYPE;

	Bool			bTotalDbg 	= (ETRI_ANALYSIS_ON == 3);
	Bool			bRDInfoDbg 	= (ETRI_ANALYSIS_ON <= 3);



	if (DebugPhase == -1){
		//----------------------------------------------------------------------------------------
		//		ETRI_PrintDebugnfo Initialization : Initializa, Set Main Debug Condition
		//		Example : 
		//		Bool	bCond01 = (pcCU->getAddr() == 70) && (uiDepth == 1) && (ETRI_getPartIdx(pcCU, uiDepth) == 41);
		//		Bool	bCond01 = (pcCU->getAddr() == 1) && (uiDepth == 1) && (em_uiPartUnitIdx[uiDepth] == 3);
		//----------------------------------------------------------------------------------------
		em_DbgInfo[ETRI_nDBGInfo_ChkPoint] = false;
		em_DbgInfo[ETRI_nDBGInfo_Control] = false;

		Bool		bCond01 = false, bCond02 = false;
//		bCond01 = (FIND_ERROR_LOCATION)? true : (pcCU->getAddr() == 390) && (uiDepth == 3 ) && (pcCU->getZorderIdxInCU()== 4); 	/// Kit :  && (uiDepth == 1) && (em_uiPartUnitIdx[uiDepth] == 0)
//		bCond01 = (FIND_ERROR_LOCATION)? true : (pcCU->getAddr() == 390); 	/// Kit :  && (uiDepth == 1) && (em_uiPartUnitIdx[uiDepth] == 0)
		bCond01 = (FIND_ERROR_LOCATION)? true : (pcCU->getAddr() == 850)  ; 	/// Kit :  && (uiDepth == 1) && (em_uiPartUnitIdx[uiDepth] == 0)


		em_DbgInfo[ETRI_nDBGInfo_ChkPoint] = (pcSlice->getPOC() == 16) && (bCond01 || bCond02);		/// Main Debug Condition

		if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])	
		{
			int DevelopVersion[2];
			DevelopVersion[0] = CurrentDevelop;
			DevelopVersion[1] = 1;

			em_DbgInfo[ETRI_nDBGInfo_Control] = false;		/// 2015 5 4 by Seok : 특정한 Debug 조건을 Active 시킬 때 사용하는 Flag

			UInt uiTileID = pcCU->getPic()->getPicSym()->getTileIdxMap(pcCU->getAddr());

			ESPRINTF2((iAnalysisType > 1), ETRI_STDMSG, "----------- Debug @ Test Vesrion CurrentDevelop = %d : %d -----------\n", DevelopVersion[0], DevelopVersion[1]);
			ESPRINTF2((iAnalysisType > 0), ETRI_STDMSG, "Developing??  : %s  POC: %d @Addr: %d Depth: %d Part: %d AbsPart : %d ( %d , %d ) @Id: %d Tile: %d SliceDepth: %d \n", 
									GetBoolVal(CurrentDevelop), pcSlice->getPOC(), pcCU->getAddr(), uiDepth, em_uiPartUnitIdx[uiDepth], ETRI_getPartIdx(pcCU, uiDepth),
									pcCU->getCUPelX(), pcCU->getCUPelY(), pcCU->getZorderIdxInCU(), uiTileID, pcSlice->getDepth());

//			EDPRINTF2(ETRI_STDMSG, "QP: %d	Rem: %d  Per(): %d	EstHeaderBits: %d sqrtLambda: %.2f \n", m_pcTrQuant->getQParam().qp(), m_pcTrQuant->getQParam().rem(), m_pcTrQuant->getQParam().per(),	pcCU->ETRI_getEstHeaderBits(), m_pcRdCost->getSqrtLambda());
//			em_DbgInfo[ETRI_nDBGInfo_ChkPoint] = false;
		}
		
		HADLuma = HADChromaCb = HADChromaCr = 0;
		
		//----------------------------------------------------------------------------------------
		//	If you want set Debug Parameter for TEncSearch or other TCom Class, set here
		//----------------------------------------------------------------------------------------

		m_pcPredSearch->ETRI_setDBGInfo(em_DbgInfo);

		//----------------------------------------------------------------------------------------

	}
	else if (DebugPhase == ETRI_IdAxTempCU_Skip && bTotalDbg )
	{
		Bool	bChkPtr = em_DbgInfo[ETRI_nDBGInfo_ChkPoint];

		UInt HeaderBits 	= pcBestCU->ETRI_getTotallHeaderBits();
		UInt ResidualBits	= pcBestCU->getTotalBits() - HeaderBits;

		if (em_bSkipMode[ETRI_IdAxTempCU_Skip]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "------ SKIP ETRI_IdAxTempCU_SKIP ");

			if (SubDbgPhase == ETRI_PRED){
			if(bChkPtr){fprintf(ETRI_STDMSG, "[PRED]------ \n");}}	
			else{
			if(bChkPtr){fprintf(ETRI_STDMSG, "[RDOQ]------ \n");}}
		}

		if (SubDbgPhase == ETRI_PRED)
		{

		}
		else
		{
			if (!em_bSkipMode[ETRI_IdAxTempCU_Skip]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "--------- ETRI_IdAxTempCU_Skip RDOQ Result ---------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost	: %.2f \n", pcBestCU->getTotalCost());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: %d \n", pcBestCU->getTotalDistortion());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bits      : %d  Header/Resi Bits ( %d / %d )\n", pcBestCU->getTotalBits(), HeaderBits, ResidualBits);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bins      : %d \n", pcBestCU->getTotalBins());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag      : %d \n", pcBestCU->getSkipFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag     : %d \n", pcBestCU->getMergeFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode: %s \n", PrintPREDMODE(pcBestCU->getPredictionMode(0)));
			}

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 
		}
		
	}
	else if (DebugPhase == ETRI_IdAxTempCU_Merge && bTotalDbg )
	{
		Bool	bChkPtr = em_DbgInfo[ETRI_nDBGInfo_ChkPoint];

		if (em_bSkipMode[ETRI_IdAxTempCU_Merge]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "----------SKIP ETRI_IdAxTempCU_Merge ");	

			if (SubDbgPhase == ETRI_PRED){
			if(bChkPtr){fprintf(ETRI_STDMSG, "[PRED]------ \n");}}	
			else{
			if(bChkPtr){fprintf(ETRI_STDMSG, "[RDOQ]------ \n");}}

			/// SKIP 되어 더 이상 데이터 표시할 필요가 없으면 @ 2015 11 22 by Seok
			//return;	
		}

		HADLuma = em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdLuma];
		HADChromaCb = em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdChromaU];
		HADChromaCr = em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdChromaV];

//		HADLuma  	= em_uiLevelInfo[ETRI_IdHADLuma]; 
//		HADChromaCb	= em_uiLevelInfo[ETRI_IdHADCb];
//		HADChromaCr	= em_uiLevelInfo[ETRI_IdHADCr]; 

		uiWidth		= em_uiLevelInfo[ETRI_IdWidTh]; 

		uinLumaPixels	= 1 << ((g_aucConvertToBit[ uiWidth ] + 2)<< 1);
		uinChromaPixels= 1 << (((g_aucConvertToBit[ uiWidth ] + 2)<< 1) - 2);

		UInt HeaderBits 	= pcBestCU->ETRI_getTotallHeaderBits();
		UInt ResidualBits	= pcBestCU->getTotalBits() - HeaderBits;

		if (SubDbgPhase == ETRI_PRED)
		{
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "-------------- ETRI_IdAxTempCU_Merge -------------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "HAD LUMA : %6d   HAD Cb: %6d   HAD Cr: %6d \n", HADLuma, HADChromaCb, HADChromaCr);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "uinLogLumaPixels : %2d	uinLogChromaPixels: %2d \n", uinLumaPixels, uinChromaPixels);

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD LUMA : %.2f  LumaLevel  : %2d \n", (1.0 * HADLuma) / (1.0 * uinLumaPixels), em_uiLevelInfo[ETRI_IdLevelLuma]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cb   : %.2f  CbLevel    : %2d \n", (1.0 * HADChromaCb) / (1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCb]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cr   : %.2f  CrLevel    : %2d \n", (1.0 * HADChromaCr) / (1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCr]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "QP               : %2d    ControlQP  : %2d \n", pcCU->getQP(0), em_qp[ETRI_IdAxTempCU_Merge].qp());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Lambda           : %.2f  SqtLambda  : %.2f \n", m_pcRdCost->getLambda(), m_pcRdCost->getSqrtLambda());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiMGLevel[%d]  : %2d    MergeHAD   : %d \n", uiDepth, em_uiMGLevel[uiDepth], em_uiSMHADDistortion);

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "\n");
		}
		else
		{
			if (!em_bSkipMode[ETRI_IdAxTempCU_Merge]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "--------- ETRI_IdAxTempCU_Merge RDOQ Result ---------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost   : %.2f \n", pcBestCU->getTotalCost());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: %d \n", pcBestCU->getTotalDistortion());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bits      : %d  Header/Resi Bits ( %d / %d )\n", pcBestCU->getTotalBits(), HeaderBits, ResidualBits);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bins      : %d \n", pcBestCU->getTotalBins());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag      : %d \n", pcBestCU->getSkipFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag     : %d \n", pcBestCU->getMergeFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode: %s \n", PrintPREDMODE(pcBestCU->getPredictionMode(0)));
			}

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 
		}

	}
	else if (DebugPhase == ETRI_IdAxTempCU_Inter && bTotalDbg )
	{
		Bool	bChkPtr = em_DbgInfo[ETRI_nDBGInfo_ChkPoint];

		if (em_bSkipMode[ETRI_IdAxTempCU_Inter]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "----------SKIP ETRI_IdAxTempCU_Inter ");	

			if (SubDbgPhase == ETRI_PRED){
			if(bChkPtr){fprintf(ETRI_STDMSG, "[PRED]------ \n");}}	
			else{
			if(bChkPtr){fprintf(ETRI_STDMSG, "[RDOQ]------ \n");}}

			/// SKIP 되어 더 이상 데이터 표시할 필요가 없으면 @ 2015 11 22 by Seok
			// if (SubDbgPhase == ETRI_RDOQ) {return;}
		}

		HADLuma = em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdLuma];
		HADChromaCb = em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdChromaU];
		HADChromaCr = em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdChromaV];

//		HADLuma 	= em_uiLevelInfo[ETRI_IdHADLuma]; 
//		HADChromaCb = em_uiLevelInfo[ETRI_IdHADCb];
//		HADChromaCr = em_uiLevelInfo[ETRI_IdHADCr]; 
		
		uiWidth 	= em_uiLevelInfo[ETRI_IdWidTh]; 
		
		uinLumaPixels	= 1 << ((g_aucConvertToBit[ uiWidth ] + 2)<< 1);
		uinChromaPixels= 1 << (((g_aucConvertToBit[ uiWidth ] + 2)<< 1) - 2);

		Double 	dEstPseudoRD = em_uiINHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getMEBits();

		UInt HeaderBits 	= pcBestCU->ETRI_getTotallHeaderBits();
		UInt ResidualBits	= pcBestCU->getTotalBits() - HeaderBits;

		if (SubDbgPhase == ETRI_PRED)
		{
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "-------------- ETRI_IdAxTempCU_Inter -------------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "HAD LUMA : %6d   HAD Cb: %6d	HAD Cr: %6d \n", HADLuma, HADChromaCb, HADChromaCr);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "uinLogLumaPixels : %2d	uinLogChromaPixels: %2d \n", uinLumaPixels, uinChromaPixels);
			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD LUMA : %.2f  LumaLevel  : %2d \n", (1.0 * HADLuma)/(1.0 * uinLumaPixels), em_uiLevelInfo[ETRI_IdLevelLuma]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cb   : %.2f  CbLevel    : %2d \n", (1.0 * HADChromaCb)/(1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCb]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cr   : %.2f  CrLevel    : %2d \n", (1.0 * HADChromaCr)/(1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCr]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "QP               : %2d    ControlQP  : %2d \n", pcCU->getQP(0), em_qp[ETRI_IdAxTempCU_Merge].qp());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Lambda           : %.2f  SqtLambda  : %.2f \n", m_pcRdCost->getLambda(), m_pcRdCost->getSqrtLambda());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiINLevel[%d]  : %2d    InterHAD   : %d \n", uiDepth, em_uiINLevel[uiDepth], em_uiINHADDistortion);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Estimated Bits   : %2d    EstimatedRD: %.2f \n", pcCU->ETRI_getMEBits(), dEstPseudoRD);

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "\n");
		}
		else
		{
			if (!em_bSkipMode[ETRI_IdAxTempCU_Inter]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "--------- ETRI_IdAxTempCU_Inter RDOQ Result ---------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost	: %.2f \n", pcBestCU->getTotalCost());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: %d \n", pcBestCU->getTotalDistortion());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bits		: %d  Header/Resi Bits ( %d / %d )\n", pcBestCU->getTotalBits(), HeaderBits, ResidualBits);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bins		: %d \n", pcBestCU->getTotalBins());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag		: %d \n", pcBestCU->getSkipFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag 	: %d \n", pcBestCU->getMergeFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode: %s \n", PrintPREDMODE(pcBestCU->getPredictionMode(0)));
			}
			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 
		}

	}
	else if (DebugPhase == ETRI_IdAxTempCU_Intra && bTotalDbg )
	{
		Bool	bChkPtr = em_DbgInfo[ETRI_nDBGInfo_ChkPoint];
		if (!bChkPtr) {return;}

		if (em_bSkipMode[ETRI_IdAxTempCU_Intra]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "----------SKIP ETRI_IdAxTempCU_Intra");	
		
			if (SubDbgPhase == ETRI_PRED){
			if(bChkPtr){fprintf(ETRI_STDMSG, "[PRED]------ \n");}}	
			else{
			if(bChkPtr){fprintf(ETRI_STDMSG, "[RDOQ]------ \n");}}

			/// SKIP 되어 더 이상 데이터 표시할 필요가 없으면 @ 2015 11 22 by Seok
			// if (SubDbgPhase == ETRI_RDOQ) {return;}
		}

		HADLuma = em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdLuma];
		HADChromaCb = em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdChromaU];
		HADChromaCr = em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdChromaV];

//		HADLuma 	= em_uiLevelInfo[ETRI_IdHADLuma]; 
//		HADChromaCb = em_uiLevelInfo[ETRI_IdHADCb];
//		HADChromaCr = em_uiLevelInfo[ETRI_IdHADCr]; 
		
		uiWidth 	= em_uiLevelInfo[ETRI_IdWidTh]; 
		
		uinLumaPixels	= 1 << ((g_aucConvertToBit[ uiWidth ] + 2)<< 1);
		uinChromaPixels= 1 << (((g_aucConvertToBit[ uiWidth ] + 2)<< 1) - 2);

		UInt HeaderBits 	= pcBestCU->ETRI_getTotallHeaderBits();
		UInt ResidualBits	= pcBestCU->getTotalBits() - HeaderBits;

		Double 	dEstPseudoRD = em_uiITHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getTotallHeaderBits(); 

		if (SubDbgPhase == ETRI_PRED)
		{
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "-------------- ETRI_IdAxTempCU_Intra -------------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "HAD LUMA : %6d   HAD Cb: %6d	HAD Cr: %6d \n", HADLuma, HADChromaCb, HADChromaCr);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "uinLogLumaPixels : %2d	uinLogChromaPixels: %2d \n", uinLumaPixels, uinChromaPixels);
			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD LUMA : %.2f  LumaLevel  : %2d \n", (1.0 * HADLuma) / (1.0 * uinLumaPixels), em_uiLevelInfo[ETRI_IdLevelLuma]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cb   : %.2f  CbLevel    : %2d \n", (1.0 * HADChromaCb) / (1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCb]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average HAD Cr   : %.2f  CrLevel    : %2d \n", (1.0 * HADChromaCr) / (1.0 * uinChromaPixels), em_uiLevelInfo[ETRI_IdLevelCr]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "QP               : %2d    ControlQP  : %2d \n", pcCU->getQP(0), em_qp[ETRI_IdAxTempCU_Merge].qp());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Lambda           : %.2f  SqtLambda  : %.2f \n", m_pcRdCost->getLambda(), m_pcRdCost->getSqrtLambda());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiITLevel[%d]  : %2d    IntraHAD   : %d \n", uiDepth, em_uiITLevel[uiDepth], em_uiITHADDistortion);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Estimated Bits   : %2d    EstimatedRD: %.2f \n", pcCU->ETRI_getTotallHeaderBits(), dEstPseudoRD);

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bControlParam[ETRI_IdFastIntraSKIP] : %s\n", GetBoolVal(em_bControlParam[ETRI_IdFastIntraSKIP]	)); 			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "\n");
		}
		else
		{
			if (!em_bSkipMode[ETRI_IdAxTempCU_Intra]){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "--------- ETRI_IdAxTempCU_Intra RDOQ Result ---------- \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost	: %.2f \n", pcBestCU->getTotalCost());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: %d \n", pcBestCU->getTotalDistortion());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bits		: %d  Header/Resi Bits ( %d / %d )\n", pcBestCU->getTotalBits(), HeaderBits, ResidualBits);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bins		: %d \n", pcBestCU->getTotalBins());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag		: %d \n", pcBestCU->getSkipFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag 	: %d \n", pcBestCU->getMergeFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode: %s \n", PrintPREDMODE(pcBestCU->getPredictionMode(0)));
			}
			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bControlParam[ETRI_IdFastIntraSKIP] : %s\n", GetBoolVal(em_bControlParam[ETRI_IdFastIntraSKIP]	)); 
		}

	}
	else if (DebugPhase == ETRI_IdAxTempCU_BESTRD && bRDInfoDbg)
	{
		UInt e_uiAveragePartDistortion = 0, e_uiExpectedTotalDistortion = 0; 
		Bool e_bTopDepth = false; 

		e_bTopDepth = (em_pcBestUpCU[uiDepth]) ? (em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_NONE && uiDepth == 1) : (uiDepth == 0);

		if (em_pcBestUpCU[uiDepth] && !e_bTopDepth)
		{
			e_uiAveragePartDistortion 	= (em_pcBestUpCU[uiDepth]->getTotalDistortion() + 2)>>2;
			e_uiExpectedTotalDistortion	= em_uiACCPartDistortion[uiDepth] + ((em_uiPartUnitIdx[uiDepth] < 3)? (e_uiAveragePartDistortion * ( 2 - em_uiPartUnitIdx[uiDepth])) : 0);
		}

		UInt HeaderBits 	= pcBestCU->ETRI_getTotallHeaderBits();
		UInt ResidualBits	= pcBestCU->getTotalBits() - HeaderBits;

		Bool	bChkPtr = em_DbgInfo[ETRI_nDBGInfo_ChkPoint];
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "-------------- Best RD Info -------------- \n");
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Addr: %d Depth: %d Part: %d AbsPart : %d ( %d , %d ) @Id: %d \n", pcCU->getAddr(), uiDepth, em_uiPartUnitIdx[uiDepth], ETRI_getPartIdx(pcCU, uiDepth), pcCU->getCUPelX(), pcCU->getCUPelY(), pcCU->getZorderIdxInCU());

		if (pcBestCU->getPredictionMode(0) == MODE_NONE){
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost	: MAX_DOUBLE \n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: MAX_INT \n");
		}
		else{
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost	: %.2f \n", pcBestCU->getTotalCost());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Distortion: %d \n", pcBestCU->getTotalDistortion());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "ACC Best RDCost: %.2f \n", em_dACCPartRDCost[uiDepth]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "ACC Best Distor: %d \n", em_uiACCPartDistortion[uiDepth]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Exp Total Dist : %d \n", e_uiExpectedTotalDistortion);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "TOP Depth      : %s \n", GetBoolVal(e_bTopDepth));
		}

		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bits		: %d  Header/Resi Bits ( %d / %d )\n", pcBestCU->getTotalBits(), HeaderBits, ResidualBits);
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best Bins		: %d \n", pcBestCU->getTotalBins());
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag		: %d \n", pcBestCU->getSkipFlag(0));
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag		: %d \n", pcBestCU->getMergeFlag(0));
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiMINLevel[%d]       : %d \n", uiDepth, em_uiMINLevel[uiDepth]);			
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiMINHADDistortion[%d]: %d \n", uiDepth, em_uiMINHADDistortion[uiDepth]);			
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode: %s \n", PrintPREDMODE(pcBestCU->getPredictionMode(0)));
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bESD[%2d]: %s\n", uiDepth, GetBoolVal(em_bESD[uiDepth])); 
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Skip]  : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Skip])); 
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Merge] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Merge])); 
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Inter] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Inter])); 
		ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s\n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra])); 

		if (uiDepth > 0)
		{
			UInt uiUpCUMode = ETRI_nAXTempCU;
			if (em_pcBestUpCU[uiDepth]->getPredictionMode(0)== MODE_INTER)
			{
				uiUpCUMode	= (em_pcBestUpCU[uiDepth]->getSkipFlag(0))? (ETRI_IdAxTempCU_Skip):((em_pcBestUpCU[uiDepth]->getMergeFlag(0))? ETRI_IdAxTempCU_Merge : ETRI_IdAxTempCU_Inter);	
			}
			else if (em_pcBestUpCU[uiDepth]->getPredictionMode(0)== MODE_INTRA)
			{
				uiUpCUMode	= ETRI_IdAxTempCU_Intra;
			}
			else
			{
			}

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "[ Information of Upper CU ]\n");
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Addr: %d Depth: %d Part: %d AbsPart : %d ( %d , %d ) @Id: %d \n", em_pcBestUpCU[uiDepth]->getAddr(), (uiDepth -1 ), 
											em_uiPartUnitIdx[uiDepth -1], ETRI_getPartIdx(em_pcBestUpCU[uiDepth], uiDepth-1), 
											em_pcBestUpCU[uiDepth]->getCUPelX(), em_pcBestUpCU[uiDepth]->getCUPelY(), em_pcBestUpCU[uiDepth]->getZorderIdxInCU());
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction Mode     : %s \n", PrintPREDMODE(em_pcBestUpCU[uiDepth]->getPredictionMode(0)));

			if (em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_NONE)
			{
				ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost [UpCU] : MAX_DOUBLE \n");
				ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Dist [UpCU] : MAX_INT \n");
			}
			else
			{
				ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Cost [UpCU] : %.2f \n", em_pcBestUpCU[uiDepth]->getTotalCost());
				ESPRINTF2(bChkPtr, ETRI_STDMSG, "Best RD Dist [UpCU] : %d \n", em_pcBestUpCU[uiDepth]->getTotalDistortion());
				ESPRINTF2(bChkPtr, ETRI_STDMSG, "Average Distotion   : %d \n", e_uiAveragePartDistortion);
			}

			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Skip Flag           : %d \n", em_pcBestUpCU[uiDepth]->getSkipFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Merge Flag          : %d \n", em_pcBestUpCU[uiDepth]->getMergeFlag(0));
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "em_uiMINLevel[%d]   : %d \n", uiDepth-1, em_uiMINLevel[uiDepth-1]);			
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Minimum HAD Cost    : %2d  \n", em_uiMINHADDistortion[uiDepth - 1]);
			ESPRINTF2(bChkPtr, ETRI_STDMSG, "Prediction HAD Mode : %s \n", PrintHADMODE(uiUpCUMode));
		}
	}

}
#endif 

/**
----------------------------------------------------------------------------------------------------------------------------------
@brief:  Check the best mode motion vector for ETRI_SliceEncoder MvClip \
to use printing debug information
@param:  TComDataCU*& pcCU
----------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xCheckBestModeMV(TComDataCU*& pcCU, UChar uhDepth)
{
#if ETRI_SliceEncoder_MVClipCheck	
	{
		if (pcCU->getPartitionSize(0) == SIZE_NONE)
		return;
		//check the mode inter or merge
		UInt iNumPart = pcCU->getNumPartitions();
		UInt uiAbsPartIdx = 0;  Int iRoiWidth = 0;  Int iRoiHeight = 0;  Int iPartIdx = 0;
		Int iVerMax = -1; Int iVerMin = -1; TComMv temp1, temp2;
		Int iSourceHeight = m_pcEncCfg->getSourceHeight();
		iVerMin = (pcCU->getCUPelY()) ? -1 * (Int)(pcCU->getCUPelY()) << 2 : 0;

		for (Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++){
			pcCU->getPartIndexAndSize(iPartIdx, uiAbsPartIdx, iRoiWidth, iRoiHeight);
			Char mode = pcCU->getPredictionMode(uiAbsPartIdx);

			if (mode == MODE_INTER){				
				iVerMax = (iSourceHeight - (Int)pcCU->getCUPelY() - iRoiHeight) << 2;
				bool bMerge = pcCU->getMergeFlag(uiAbsPartIdx);

				/*if (iNumPart > 1)
					EDPRINTF(stderr, "\nSliceEncoder MvClip Debug Information\nNumPart:%d Mode:: CU(%d, %d) PartIdx: %d Mode: %d Merge: %d \t", iNumPart, pcCU->getCUPelX(), pcCU->getCUPelY(), iPartIdx, mode, bMerge);*/

				if (bMerge){
					//check the best mvp		
					temp1 = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiAbsPartIdx);
					temp2 = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiAbsPartIdx);
					if (temp1.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight) || temp2.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight))
					{
						EDPRINTF(stderr, "\nSliceEncoder MvClip Error: Merge/Skip: CU(%d, %d) L0_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp1.getHor() >> 2, temp1.getVer() >> 2);
						EDPRINTF(stderr, "\nSliceEncoder MvClip Error: Merge/Skip: CU(%d, %d) L1_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp2.getHor() >> 2, temp2.getVer() >> 2);						
					}
				}
				else{
					
					if (pcCU->getInterDir(uiAbsPartIdx) == 1){
						temp1 = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiAbsPartIdx);
						if (temp1.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight))
							EDPRINTF(stderr, "\nSliceEncoder MvClip Error: InterUni: CU(%d, %d) L0_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp1.getHor() >> 2, temp1.getVer() >> 2);
					}
					else if (pcCU->getInterDir(uiAbsPartIdx) == 2){
						temp2 = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiAbsPartIdx);
						if (temp2.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight))
							EDPRINTF(stderr, "\nSliceEncoder MvClip Error: InterUni: CU(%d, %d) L1_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp1.getHor() >> 2, temp1.getVer() >> 2);
					}
					else if (pcCU->getInterDir(uiAbsPartIdx) == 3){
						temp1 = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiAbsPartIdx);
						if (temp1.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight))
							EDPRINTF(stderr, "\nSliceEncoder MvClip Error: InterBi: CU(%d, %d) L0_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp1.getHor() >> 2, temp1.getVer() >> 2);
						temp2 = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiAbsPartIdx);
						if (temp2.ETRI_DetectVerMVBoundary(pcCU->getCUPelY(), pcCU->getHeight(0), iSourceHeight))
							EDPRINTF(stderr, "\nSliceEncoder MvClip Error: InterBi: CU(%d, %d) L01_MV:(%d, %d) \n", pcCU->getCUPelX(), pcCU->getCUPelY(), temp1.getHor() >> 2, temp1.getVer() >> 2);
					}
					else{
						EDPRINTF(stderr, "\nSliceEncoder MvClip Error: Inter but no MvSliceEncoder MvClip Debug Information\n");
					}

				}
			}
		}
	}
#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Set QP for Compress CU [REMOVABLE}
			This function can be removable, when optimization is processed 
	@param   e_bPreSet  	When the function is called at the front part of Compress, it is true. However, at the front of SubCUprocessing, it is false 
	@return	e_ilowestQP	When PrePocessing, it is iMinQP. When PreProcessing for Sub-CU, it is 0
	* Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Int TEncCu::ETRI_SetCUQP(TComDataCU*& rpcTempCU, Int& iMinQP, Int& iMaxQP, Bool& isAddLowestQP, Int iBaseQP, UInt uiDepth, Bool e_bPreSet)
{
	Int e_ilowestQP = 0;

#if KAIST_RC
	Int intra_size = 0;
	Int iIDRIndex = 0;
	Int iIDRModulus = 0;
	TRCPic* tRCPic = NULL;
	if (m_pcEncCfg->getUseRateCtrl())
	{
		intra_size = m_pcEncCfg->getIntraPeriod();
		iIDRIndex = rpcTempCU->getPic()->getPOC() / intra_size;
		iIDRModulus = rpcTempCU->getPic()->getPOC() % intra_size;
		tRCPic = m_pcRateCtrl->getTRCPic(iIDRModulus);
	}
#endif

	if (e_bPreSet)
	{
		//--------------------------------------------------------------------------------
		//	Early part of xCompressCU :: processing here : True
		//--------------------------------------------------------------------------------
		if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
		{
			Int idQP = m_pcEncCfg->getMaxDeltaQP();
			iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
			iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
		}
		else
		{
			iMinQP = rpcTempCU->getQP(0);
			iMaxQP = rpcTempCU->getQP(0);
		}

		if ( m_pcEncCfg->getUseRateCtrl() )
		{
#if KAIST_RC
			iMinQP = tRCPic->getLCU(rpcTempCU->getAddr()).m_QP;
			iMaxQP = tRCPic->getLCU(rpcTempCU->getAddr()).m_QP;
#endif
		}

		// transquant-bypass (TQB) processing loop variable initialisation ---

		e_ilowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

		if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
		{
			isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
			iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
			if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
			{
				iMaxQP = iMinQP;
			}
		}

	}
	else
	{
		//--------------------------------------------------------------------------------
		//	the front of Subprocessing :: processing here : False
		//--------------------------------------------------------------------------------
		if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
		{
			Int idQP = m_pcEncCfg->getMaxDeltaQP();
			iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
			iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
		}
		else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
		{
			iMinQP = iBaseQP;
			iMaxQP = iBaseQP;
		}
		else
		{
			Int iStartQP;
#if ETRI_SLICE_SEGMENT_OPTIMIZATION
			iStartQP = rpcTempCU->getQP(0);
#else
#if V01_BugFix_Yhee
			TComPic* pcPic = rpcTempCU->getPic();
			TComSlice* pcSlice = rpcTempCU->getSlice();
#endif
#if ETRI_DQP_FIX
            TComPic* pcPic = rpcTempCU->getPic();
            TComSlice* pcSlice = rpcTempCU->getSlice();
#endif 
			if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
			{
				iStartQP = rpcTempCU->getQP(0);
			}
			else
			{
				UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
				iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
			}
#endif
			iMinQP = iStartQP;
			iMaxQP = iStartQP;
		}

		if ( m_pcEncCfg->getUseRateCtrl() )
		{
#if KAIST_RC
			iMinQP = tRCPic->getLCU(rpcTempCU->getAddr()).m_QP;
			iMaxQP = tRCPic->getLCU(rpcTempCU->getAddr()).m_QP;
#endif
		}

		if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
		{
			iMaxQP = iMinQP; // If all blocks are forced into using transquant bypass, do not loop here.
		}
	}

	return e_ilowestQP;

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief   : For the Fast RD-Optimization . This function has the member variable indicating change of two object by em_bActivexCheckETRIBestMode \
			Instead of rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost()
	@param : TComDataCU*& rpcBestCU
	@param : TComDataCU*& rpcTempCU 
	@param : UInt uiDepth 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncCu::ETRI_xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
{
	/// 2014 7 3 by Seok
	em_bActiveCheckBestCU = false;

	if(em_bActivexCheckETRIBestMode)
	{	 
		TComYuv* pcYuv;

		// Change Information data
		TComDataCU* pcCU = rpcBestCU;
		rpcBestCU = rpcTempCU;
		rpcTempCU = pcCU;

		// Change Prediction data
		pcYuv = m_ppcPredYuvBest[uiDepth];
		m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
		m_ppcPredYuvTemp[uiDepth] = pcYuv;

		// Change Reconstruction data
		pcYuv = m_ppcRecoYuvBest[uiDepth];
		m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
		m_ppcRecoYuvTemp[uiDepth] = pcYuv;

		pcYuv = NULL;
		pcCU  = NULL;

		m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);

		em_bActiveCheckBestCU = true;
	}
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief   : To Copy the data in tempCu and PredYUVtemp to an auxiliary CU and YUV Buffer.
	@param : TComDataCU*& pcDestCU : AuxiliaryCU
	@param : TComDataCU*& rpcSrcCU  : TempCU 
	@param : UInt uiDepth 
	@param : Int BufferIdx : SkipMerge, Inter, Intra Index
	@param : For Inter Process, Indicates SIZE_2Nx2N, SIZE_2NxN, SIZE_Nx2N.
	@param : Bool StoreTrue: True Direction : Store SrcData to Auxiliary Dst Data. False : Restore Auxiliary Dst Data to SrcData
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncCu::ETRI_xStoreModeData (TComDataCU*& rpcDstCU, TComDataCU* rpcSrcCU, UInt uiDepth, Int BufferIdx, PartSize ePartSize, Bool StoreTrue )
{
	UInt 	uiPartAddr = 0;

	Int 	iWidth = rpcSrcCU->getWidth(0);	 
	Int  	iHeight = rpcSrcCU->getHeight(0); 	

	TComYuv*	pcYuvSrc = nullptr; 
	TComYuv*	pcYuvDst = nullptr;

	///< 2015 9 7 by Seok : 해당 코드는 FastRDOMergeOFF 와 무관하개 잘 수정된 코드
	if (BufferIdx == ETRI_IdAxTempCU_Skip)
	{
		if (!StoreTrue) 
		{ 
			m_pppcAxPredYuvTemp[ETRI_IdAxTempCU_Skip][uiDepth]->copyPartToPartYuv(m_ppcPredYuvTemp[uiDepth], uiPartAddr, iWidth, iHeight );
		}
		BufferIdx = ETRI_IdAxTempCU_Merge;
	}
	else
	{
		pcYuvSrc = (StoreTrue)? m_ppcPredYuvTemp[uiDepth]				: m_pppcAxPredYuvTemp[BufferIdx][uiDepth];
		pcYuvDst = (StoreTrue)? m_pppcAxPredYuvTemp[BufferIdx][uiDepth] 	: m_ppcPredYuvTemp[uiDepth];
		pcYuvSrc->copyPartToPartYuv(pcYuvDst, uiPartAddr, iWidth, iHeight );
	}


	if (BufferIdx == ETRI_IdAxTempCU_Merge)
	{
		//yhee, error check
		if (em_iRDOOffBestMergeCand < 0){
			EDPRINTF(stderr, "=====ERROR:: ETRI_SliceEncoder_MVClip  < 0 \n");
		}
		else{
		rpcDstCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth ); 		
		rpcDstCU->setPredModeSubParts( MODE_INTER, 0, uiDepth ); 		
		rpcDstCU->setMergeFlagSubParts( true, 0, 0, uiDepth ); 			 

//		rpcDstCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(),	0, uiDepth );

		rpcDstCU->setMergeIndexSubParts(em_iRDOOffBestMergeCand, 0, 0, uiDepth );  
		rpcDstCU->setInterDirSubParts( em_puhInterDirNeighbours[em_iRDOOffBestMergeCand], 0, 0, uiDepth ); 
		rpcDstCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( em_pcMvFieldNeighbours[0 + 2*em_iRDOOffBestMergeCand], SIZE_2Nx2N, 0, 0 ); // m_acCUMvField
		rpcDstCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( em_pcMvFieldNeighbours[1 + 2*em_iRDOOffBestMergeCand], SIZE_2Nx2N, 0, 0 ); // m_acCUMvField

		rpcDstCU->getTotalCost() = rpcSrcCU->getTotalCost();	//HAD Value 
		}
	}
	else if (BufferIdx == ETRI_IdAxTempCU_Inter || BufferIdx == ETRI_IdAxTempCU_InterNx2N || BufferIdx == ETRI_IdAxTempCU_Inter2NxN)	
	{
		rpcDstCU->setPartSizeSubParts( ePartSize, 0, uiDepth);
		rpcDstCU->setPredModeSubParts( MODE_INTER, 0, uiDepth ); 		
		rpcDstCU->setMergeAMP (true);

//		rpcDstCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 	0, uiDepth );

		if (BufferIdx == ETRI_IdAxTempCU_Inter)
		{
			rpcDstCU->setMergeFlagSubParts(rpcSrcCU->getMergeFlag(0), 0, 0, uiDepth );
		}
		else
		{
			UInt	ruiPartAddr = 0;
			Int 	riWidth=0, riHeight=0;
				
			rpcSrcCU->getPartIndexAndSize( 1, ruiPartAddr, riWidth, riHeight );
			rpcDstCU->setMergeFlagSubParts(rpcSrcCU->getMergeFlag(0),  				0, 0, uiDepth );
			rpcDstCU->setMergeFlagSubParts(rpcSrcCU->getMergeFlag(ruiPartAddr), ruiPartAddr, 1, uiDepth );
		}

		rpcDstCU->setInterDirSubParts(rpcSrcCU->getInterDir(0), 0, 0, uiDepth ); // rpcSrcCU->getInterDir(0) : rpcSrcCU->getInterDir(0) : Removable : Active in ETRI_copyInterPredInfoFrom

		rpcDstCU->ETRI_copyInterPredInfoFrom(rpcSrcCU, 0);

		rpcDstCU->getTotalDistortion() = rpcSrcCU->getTotalDistortion();	/// Instead of : rpcDstCU->getTotalDistortion() = MAX_INT;
		rpcDstCU->getTotalCost() = rpcSrcCU->getTotalCost();			/// Instead of : rpcDstCU->getTotalCost()= MAX_DOUBLE;
		rpcDstCU->getTotalBits()= rpcSrcCU->getTotalBits();			/// Instead of : rpcDstCU->getTotalBits()= 0;

		rpcDstCU->ETRI_SetmvRdCost (rpcSrcCU->ETRI_GetmvRdCost());

	}
	else	/// INTRA
	{
		rpcDstCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
		rpcDstCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
		rpcDstCU->setMergeFlagSubParts(false, 0, 0, uiDepth );			

//		rpcDstCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uiDepth );

		rpcDstCU->setLumaIntraDirSubParts(rpcSrcCU->getLumaIntraDir(0), 0, uiDepth);
		rpcDstCU->setChromIntraDirSubParts(rpcSrcCU->getChromaIntraDir(0), 0, uiDepth);

//		rpcDstCU->getPattern()
	}

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: 	Evaluate Hadanard value and Level \

	@return	UInt Hadamard Level 
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline UInt TEncCu::ETRI_EvalLevel( TComDataCU* pcCU, UInt uiStage, UInt*& puiLevelInfo)
{
	//---------------------------------------------------------------------------------------
	//	원래 생각은 여기에서 qp를 각 Mode 별로 주어서 Level을 결정하려고 하였다.
	//	하지만 그 경우, 각 Mode 별 Level의 통일성이 주어지지 않는다는 문제점이 있다.
	//	따라서 Prediction Level에서 고속 알고리즘은 
	//	별도의 qp 를 통해 따로 계산하는 방식을 사용하는 것이 맞을 듯 하다.
	//---------------------------------------------------------------------------------------	
#if ETRI_BUGFIX_ForcedSKIP 
	QpParam*	e_QPpram = &em_qp[ ETRI_IdAxTempCU_Skip];
#else
	QpParam*	e_QPpram = &em_qp[uiStage];
#endif

	Int LumaRem   	= e_QPpram->rem(); 
	Int LumaPer    	= e_QPpram->per();
	Int ChromaRem 	= e_QPpram->rem(); 
	Int ChromaPer	= e_QPpram->per();

	if (uiStage == ETRI_IdAxTempCU_Intra)
	{
		UInt* puiIntraBestHAD = m_pcPredSearch->ETRI_getHADCost();
		
		puiLevelInfo[ETRI_IdHADLuma]	= puiIntraBestHAD[ETRI_IdLuma];
		puiLevelInfo[ETRI_IdHADCb]    	= puiIntraBestHAD[ETRI_IdChromaU];
		puiLevelInfo[ETRI_IdHADCr]    	= puiIntraBestHAD[ETRI_IdChromaV];

		m_pcPredSearch->ETRI_HAD_SetParamforGetLevel(puiLevelInfo[ETRI_IdHADLuma], puiLevelInfo[ETRI_IdHADCb], puiLevelInfo[ETRI_IdHADCr]);
	}

	/// In Skip Process, HAD value should be updated when m_pcPredSearch->ETRI_HAD_getLevel is operated 
	puiLevelInfo[ETRI_IdHADUpdate] = (uiStage == ETRI_IdAxTempCU_Skip)? 1 : 0;

	/// 2014 7 26 by Seok : Pseudo Quantization Method
	puiLevelInfo[ETRI_IdLevelLuma] = puiLevelInfo[ETRI_IdLevelCb] = puiLevelInfo[ETRI_IdLevelCr] = 0;
	return (m_pcPredSearch->ETRI_HAD_getLevel(puiLevelInfo, LumaRem, LumaPer, ChromaRem, ChromaPer));  
}

__inline UInt TEncCu::ETRI_EvalHAD( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, UInt*& puiLevelInfo)
{
	Pel* 		piTargetY 	= pcPredYuv->getLumaAddr();
	Pel* 		piTargetCb 	= pcPredYuv->getCbAddr();
	Pel* 		piTargetCr 	= pcPredYuv->getCrAddr();

	UInt 		uiWidth   	= pcOrgYuv->getWidth();
	UInt 		uiHeight  	= pcOrgYuv->getHeight();	
	UInt 		TargetStride	= pcPredYuv->getStride();
	UInt 		TargetCStride= pcPredYuv->getCStride();

	UInt HADLuma 		= m_pcRdCost->getDistPart(g_bitDepthY, piTargetY,  TargetStride,  pcOrgYuv->getLumaAddr(), pcOrgYuv->getStride(),  uiWidth, 	 uiHeight , 	TEXT_LUMA,     DF_HADS );
	UInt HADChromaCb 	= m_pcRdCost->getDistPart(g_bitDepthC, piTargetCb, TargetCStride, pcOrgYuv->getCbAddr(),   pcOrgYuv->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_U, DF_HADS );
	UInt HADChromaCr 	= m_pcRdCost->getDistPart(g_bitDepthC, piTargetCr, TargetCStride, pcOrgYuv->getCrAddr(),   pcOrgYuv->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_V, DF_HADS );

	/// 2015 3 31 by Seok : Monitor For Bet Hadmard Value for SKIP/Merge
	puiLevelInfo[ETRI_IdHADLuma] 	= HADLuma;
	puiLevelInfo[ETRI_IdHADCb]  	= HADChromaCb;
#if ETRI_BUGFIX_ETRI_EvalHAD_Cr
	puiLevelInfo[ETRI_IdHADCr]  	= HADChromaCr;
#else
	puiLevelInfo[ETRI_IdHADCr]  	= HADChromaCr;
#endif

	m_pcPredSearch->ETRI_HAD_SetParamforGetLevel(HADLuma, HADChromaCb, HADChromaCr);

	return (HADLuma + HADChromaCb + HADChromaCr);
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: 	Evaluate Control Level on HAD value from Prediction 
	@param	uiStage 	: Current Processing Stage
	@param	HADofStage 	: Current Processing Stage
	@param	uiDepth 	: CU Depth
	@param	puiLevelInfo: IO Data
	@param	bOP		 	: Operation Condition
	@param   InitLevel		: When bOP is False, Return Value must be same to the Initial Value
	@return	UInt Lower Level of Hadamard Level 
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline UInt TEncCu::ETRI_EvalControlLevel(QpParam e_QPpram, UInt HADofStage, UInt uiDepth, UInt InitLevel, UInt*& puiLevelInfo, Bool bOP)
{
	UInt	uiLBLevel = InitLevel;

	if (bOP)
	{
		//QpParam* e_QPpram = &em_qp[uiStage];
		memcpy(puiLevelInfo, em_uiLevelInfo, ETRI_nLevelInfo * sizeof(Int));
		
		puiLevelInfo[ETRI_IdLevelLuma]	= puiLevelInfo[ETRI_IdLevelCb] = puiLevelInfo[ETRI_IdLevelCr] = 0;
		puiLevelInfo[ETRI_IdHADLuma]	= em_uiHADInfo[HADofStage][ETRI_IdLuma];
		puiLevelInfo[ETRI_IdHADCb]  	= em_uiHADInfo[HADofStage][ETRI_IdChromaU];
		puiLevelInfo[ETRI_IdHADCr]  	= em_uiHADInfo[HADofStage][ETRI_IdChromaV];

		/// 2015 11 17 by Seok : Debug	
		uiLBLevel  = m_pcPredSearch->ETRI_HAD_getControlLevel(puiLevelInfo, e_QPpram.rem(), e_QPpram.per(), e_QPpram.rem(), e_QPpram.per());  
	}

	return uiLBLevel;
}

// ====================================================================================================================
//	ETRI Fast PU Processing 
// ====================================================================================================================
#if ETRI_ADAPTIVE_MAXCTU_SIZE
#define	FAST64x64CONDITION(ACTIVE)	(em_bControlParam[ETRI_IdxMAXCTUSIZE] & ACTIVE)
#else
#define	FAST64x64CONDITION(ACTIVE)	(false)
#endif

#if ETRI_FASTPUProcessing
#define	LEFTContInRange(LB, x, UB, ACTIVE) ((LB <= x && x < UB) && ACTIVE)
#define	RIGHTContInRange(LB, x, UB, ACTIVE) ((LB < x && x <= UB) && ACTIVE)
#define	RIGHTContOutRange(LB, x, UB, ACTIVE) ((x < LB  || x >= UB) && ACTIVE)
#define	ETRI_TH_SKIPMODE		1
#else
#define	LEFTContInRange(LB, x, UB, ACTIVE) (false)
#define	RIGHTContInRange(LB, x, UB, ACTIVE) (false)
#define	RIGHTContOutRange(LB, x, UB, ACTIVE) (false)
#define	ETRI_TH_SKIPMODE		3
#endif

#define	ETRI_TH_MERGEMODE   	5
#define	ETRI_INIT_RDTH 			1.15		///Bandwidth for Pseudo Rate-Distortion Optimization

/**
----------------------------------------------------------------------------------------------------------------------------------
	@brief  :  Set Control Parameter (em_bSKipMode) for Fast PU Mode Decision \
			This function is same to ETRI_xCheckInterDistortion in V12
	@param:  TComDataCU*& pcCU 
	@param:  UChar uhDepth 
	@param:  UInt PhaseIdx 
	@param:  UInt SubStage
----------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xControlPUProcessing(TComDataCU*& pcCU, UChar uhDepth, UInt PhaseIdx, UInt SubStage)
{
	Bool  		bAlgorithm = false;
	Bool  		bControl = false, bMGControl = false, bINControl = false, bITControl = false;

	TComSlice* 	pcSlice = pcCU->getSlice();
	UInt  		uiLevelInfo[ETRI_nLevelInfo], *e_puiLevelInfo = &uiLevelInfo[0];
	Bool  		bNoI_Slice = (pcSlice->getSliceType() != I_SLICE);

#if 	ETRI_FASTPUProcessing	
	Double		e_dPseudoRDcost[4] = { 0, 0, 0, 0 };
#endif

	//--------------------------------------------------------------------------------------------------
	//	[1] ETRI_Fast64x64  :: ETRI_MODIFICATION_V03 
	//		FAST64x64CONDITION(FALSE) : ETRI_Fast64x64 적용이 안된다는 의미
	//		FAST64x64CONDITION(TRUE)  : ETRI_Fast64x64 적용이 된다는 의미 :: 실제로 해당 MODE Processing을 하지 않는다는 의미 (SKIP 제외)
	//--------------------------------------------------------------------------------------------------
	if (PhaseIdx == ETRI_IdAxTempCU_Skip)
	{
		//--------------------------------------------------------------------------------------------------
		//	Control parameter Setting : 
		//	Example :  uiThrehold = 1 보다 크면 PSNR 이 좋아지나 비트가 증가한다. 3 보다 크면 하지 않는 것과 거의 차이가 없다. 
		//--------------------------------------------------------------------------------------------------
		UInt uiThrehold = ETRI_TH_SKIPMODE;

		bAlgorithm 	= ((em_iRDOOffBestMergeCand >= 0) |(!ETRI_SliceEncoder_MVClip)) > 0;
		bControl   	= em_uiSKLevel[uhDepth] > uiThrehold;	

		em_bSkipMode[ETRI_IdAxTempCU_Skip] |= bControl &  bAlgorithm;
	}
	if (PhaseIdx == ETRI_IdAxTempCU_Merge)
	{
		//--------------------------------------------------------------------------------------------------
		//	Example :  uiThrehold = 1 보다 크면 PSNR 이 좋아지나 비트가 증가한다. 5 보다 크면 하지 않는 것과 거의 차이가 없다. 
		//--------------------------------------------------------------------------------------------------
		UInt uiThrehold = ETRI_TH_MERGEMODE;

		em_uiMGLevel[uhDepth] = em_uiSKLevel[uhDepth];	///< 2015 9 20 by Seok : Code Test 
 
		bAlgorithm	= ((em_iRDOOffBestMergeCand >= 0) |(!ETRI_SliceEncoder_MVClip)) > 0;
		bMGControl	= (em_uiSKLevel[uhDepth] > uiThrehold); 
		bINControl	= em_bSkipMode[ETRI_IdAxTempCU_Inter];	
		bITControl	= em_bSkipMode[ETRI_IdAxTempCU_Intra];	

		if (SubStage == ETRI_PRED)
		{
			#if ETRI_SliceEncoder_MVClip
			em_bSkipMode[ETRI_IdAxTempCU_Merge] |= (em_iRDOOffBestMergeCand < 0);
			em_bSkipMode[ETRI_IdAxTempCU_Skip] |= (em_iRDOOffBestMergeCand < 0);
			#endif

			/// Initilization of em_uiMINHADDistortion[uhDepth]/em_uiMINLevel[uhDepth] @ 2015 11 20 by Seok
			em_uiMINHADDistortion[uhDepth]	= em_uiSMHADDistortion;
			em_uiMINLevel[uhDepth]  		= em_uiSKLevel[uhDepth];

			/// Condition of FFESD ACTIVATION of Early SKIP/Merge @ 2015 11 22 by Seok
			#if ETRI_FFESD
			/// 단순 FFESD 조건 : 실험 결과 가장 성적이 좋다
			em_bControlParam[ETRI_IdEarlyMerge] = em_uiSKLevel[uhDepth] <= ETRI_FFESDLEVEL;
			#else
			em_bControlParam[ETRI_IdEarlyMerge] = (RIGHTContInRange(0, em_uiSKLevel[uhDepth], ETRI_FFESD, true) && !em_bSkipMode[ETRI_IdAxTempCU_Merge]);
			#endif
		}
		else
		{
			//--------------------------------------------------------------------------------------------------
			//	SKIP RDOQ Phase :: Update Minimum HAD Distortion & Level 
			//--------------------------------------------------------------------------------------------------
			em_uiMINHADDistortion[uhDepth]	= (em_uiMINHADDistortion[uhDepth] > em_uiITHADDistortion && !em_bSkipMode[ETRI_IdAxTempCU_Intra])? 
											em_uiITHADDistortion : em_uiMINHADDistortion[uhDepth];
			
			em_uiMINLevel[uhDepth] 		= (em_uiMINLevel[uhDepth] > em_uiITLevel[uhDepth] && !em_bSkipMode[ETRI_IdAxTempCU_Intra])? 
											em_uiITLevel[uhDepth] : em_uiMINLevel[uhDepth];

			//--------------------------------------------------------------------------------------------------
			//	Fast SKIP/Merge RDOQ Decision 
			//--------------------------------------------------------------------------------------------------
			if (!em_bSkipMode[ETRI_IdAxTempCU_Merge] && !em_bSkipMode[ETRI_IdAxTempCU_Skip])
			{
				// em_DbgInfo[ETRI_nDBGInfo_CASE02]++; 
			}
 
			//--------------------------------------------------------------------------------------------------
			//	Fast Merge/Inter RDOQ Decision @ 2015 11 19 by Seok
			//--------------------------------------------------------------------------------------------------
			if (!em_bSkipMode[ETRI_IdAxTempCU_Merge] && !em_bSkipMode[ETRI_IdAxTempCU_Inter])
			{
				#if ETRI_FAST_MGINRDOQ
				bMGControl 	|= (em_uiSKLevel[uhDepth] > em_uiINLevel[uhDepth]);	
				bINControl 	|= (em_uiSKLevel[uhDepth] < em_uiINLevel[uhDepth]); 
				if (em_uiSKLevel[uhDepth] == em_uiINLevel[uhDepth])
				{
					e_dPseudoRDcost[ETRI_IdAxTempCU_Merge]= 1.0 * em_uiSMHADDistortion;
					e_dPseudoRDcost[ETRI_IdAxTempCU_Inter] 	= 1.0 * em_uiINHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getMEBits();

					bINControl |=  (e_dPseudoRDcost[ETRI_IdAxTempCU_Merge] < e_dPseudoRDcost[ETRI_IdAxTempCU_Inter]);
					bMGControl |= (e_dPseudoRDcost[ETRI_IdAxTempCU_Merge] > (e_dPseudoRDcost[ETRI_IdAxTempCU_Inter] * 1.15));
				}
				#endif
			}

			//--------------------------------------------------------------------------------------------------
			//	Fast Merge/Intra RDOQ Decision : pcCU->ETRI_getTotallHeaderBits() INTRA Header Bits @ 2015 11 15 by Seok
			//--------------------------------------------------------------------------------------------------
			if (!em_bSkipMode[ETRI_IdAxTempCU_Merge] && !em_bSkipMode[ETRI_IdAxTempCU_Intra])
			{
				#if ETRI_FAST_MGITRDOQ
				bMGControl 	|= (em_uiSKLevel[uhDepth] > em_uiITLevel[uhDepth]);	
				bITControl 	|= (em_uiSKLevel[uhDepth] < em_uiITLevel[uhDepth]); 
				if (em_uiSKLevel[uhDepth] == em_uiITLevel[uhDepth])
				{
					e_dPseudoRDcost[ETRI_IdAxTempCU_Merge]= 1.0 * em_uiSMHADDistortion;
					e_dPseudoRDcost[ETRI_IdAxTempCU_Intra] 	= 1.0 * em_uiITHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getTotallHeaderBits();

					bITControl 	|=  (e_dPseudoRDcost[ETRI_IdAxTempCU_Merge] <= e_dPseudoRDcost[ETRI_IdAxTempCU_Intra]);
					bMGControl 	|= !(e_dPseudoRDcost[ETRI_IdAxTempCU_Merge] <= e_dPseudoRDcost[ETRI_IdAxTempCU_Intra]);
				}
				#endif
			}

		}

		em_bSkipMode[ETRI_IdAxTempCU_Merge] |= bMGControl &  bAlgorithm;
		em_bSkipMode[ETRI_IdAxTempCU_Inter] |= bINControl &  bAlgorithm;
		em_bSkipMode[ETRI_IdAxTempCU_Intra] |= bITControl &  bAlgorithm;

	}
	else if (PhaseIdx == ETRI_IdAxTempCU_Inter)
	{
		if (SubStage == ETRI_PRED)
		{
			em_bSkipMode[ETRI_IdAxTempCU_Inter] |= em_bESD[uhDepth];

			//--------------------------------------------------------------------------------------------------
			//	Inter Prediction SKIP  : 
			//		[1] Slice Depth 혹은 QP 에 에 따른 UpperBound를 놓고 이보다 크면 Inter Prediction을 하지 않는다
			//		[2] MVClip의 경우, Inter Prediction중에 Check가 이루어지므로 여기서 할 필요가 없다
			//		[3] ETRI_CU_INTRA_MODE_INHERITANCE 를 여기에 구현한다 
			//	@ Fast PU Test Code 2015 10 27 by Seok
			//--------------------------------------------------------------------------------------------------
			#if ETRI_TEST_FASTINPRED
			UInt	uiUpperBound = em_ucFPUInterSkipLevel[(pcSlice->getDepth() > 0)];			
			em_bSkipMode[ETRI_IdAxTempCU_Inter] |= (em_uiSKLevel[uhDepth] > uiUpperBound);		
			em_uiINHADDistortion	= (em_bSkipMode[ETRI_IdAxTempCU_Inter])? MAX_INT : em_uiINHADDistortion;
			#endif

			#if 0
			if (!em_bSkipMode[ETRI_IdAxTempCU_Inter] && em_bSkipMode[ETRI_IdAxTempCU_Intra])
			{
				#if ETRI_CU_INTRA_MODE_INHERITANCE
				em_bSkipMode[ETRI_IdAxTempCU_Inter] |= !em_bDoInterModeFlag;
				#endif
			}
			#endif
		
		}
		else if (SubStage == ETRI_AdditionalInter)
		{
			EDPRINTF(stderr, "ADDITIONAL INETR :: LOGICAL ERROR \n");
		}
		else if (SubStage == ETRI_RDOQ)
		{
			//--------------------------------------------------------------------------------------------------
			//	Inter/Merge RDOQ 안정성 체크 
			//--------------------------------------------------------------------------------------------------
			if (!em_bSkipMode[ETRI_IdAxTempCU_Inter] && !em_bSkipMode[ETRI_IdAxTempCU_Merge])		
			{
				#if ETRI_FAST_MGINRDOQ && STRONG_CONDITION
				EDPRINTF(stderr, "ADDITIONAL INETR :: LOGICAL ERROR \n");
				#endif	
			}
			
			//--------------------------------------------------------------------------------------------------
			//	Fast INTER INTRA Decision  @ 2015 11 19 by Seok
			//--------------------------------------------------------------------------------------------------
			if (!em_bSkipMode[ETRI_IdAxTempCU_Inter] && !em_bSkipMode[ETRI_IdAxTempCU_Intra])
			{
				#if ETRI_FAST_INITRDOQ	
				bINControl 	|= (em_uiINLevel[uhDepth] > (em_uiITLevel[uhDepth] + 1));	
				bITControl 	|= ((em_uiINLevel[uhDepth] + 1) < em_uiITLevel[uhDepth]); 
				if (!bINControl && !bITControl)
				{
					e_dPseudoRDcost[ETRI_IdAxTempCU_Intra] 	= 1.0 * em_uiITHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getTotallHeaderBits();
					e_dPseudoRDcost[ETRI_IdAxTempCU_Inter] 	= 1.0 * em_uiINHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getMEBits();

					bITControl |=  ((e_dPseudoRDcost[ETRI_IdAxTempCU_Inter] * ETRI_INIT_RDTH) < (e_dPseudoRDcost[ETRI_IdAxTempCU_Intra]));
					bINControl |=  (e_dPseudoRDcost[ETRI_IdAxTempCU_Inter] > ( e_dPseudoRDcost[ETRI_IdAxTempCU_Intra] * ETRI_INIT_RDTH));
				}
				#endif
			}

		}

#if 1
#if ETRI_CU_INTRA_MODE_INHERITANCE
		em_bSkipMode[ETRI_IdAxTempCU_Inter] |= !em_bDoInterModeFlag;
#endif
#endif
		em_bSkipMode[ETRI_IdAxTempCU_Inter] |= (bINControl & bNoI_Slice) > 0; 
		em_bSkipMode[ETRI_IdAxTempCU_Intra] |= (bITControl & bNoI_Slice) > 0; 

	}
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
	else if (PhaseIdx == ETRI_IdAxTempCU_Inter2NxN)
	{
		em_bSkipMode[ETRI_IdAxTempCU_Inter2NxN] |= em_bESD[uhDepth];
		em_bSkipMode[ETRI_IdAxTempCU_Inter2NxN] |= !*em_pbdoNotBlockPU;
	}	
	else if (PhaseIdx == ETRI_IdAxTempCU_InterNx2N)
	{
		em_bSkipMode[ETRI_IdAxTempCU_InterNx2N] |= em_bESD[uhDepth];
		em_bSkipMode[ETRI_IdAxTempCU_InterNx2N] |= !*em_pbdoNotBlockPU;
	}
#endif 
	else if (PhaseIdx == ETRI_IdAxTempCU_Intra)
	{
		// EarlySkipDecision의 영향으로 아무런 효과가 없음
		em_bSkipMode[ETRI_IdAxTempCU_Intra] |= em_bESD[uhDepth];	
		em_bSkipMode[ETRI_IdAxTempCU_Intra] |= FAST64x64CONDITION(true);

		/// For Stability Check	
		bAlgorithm	= (em_bSkipMode[ETRI_IdAxTempCU_Merge] & em_bSkipMode[ETRI_IdAxTempCU_Skip] & em_bSkipMode[ETRI_IdAxTempCU_Inter]) > 0;
		if (!bAlgorithm)
		{
			if (SubStage == ETRI_PRED)
			{
				//--------------------------------------------------------------------------------------------------
				//	Intra Prediction SKIP  : 
				//		[1] QP > 29 에서 판정을 위한 QP를 낮게 하여 1 보다 Level 이 작거나 Inter Level 이 0이면 INTRTA Prediction/RDOQ를 하지 않는다. 
				//	@ Fast PU Test Code 2015 10 27 by Seok
				//--------------------------------------------------------------------------------------------------
				#if ETRI_TEST_FASTITPRED
				UInt	uiLBLevel = em_uiSKLevel[uhDepth];	/// Very Important : Initial Value of uiLBLevel must be  em_uiSKLevel[uhDepth] @ 2015 11 14 by Seok
				uiLBLevel  = ETRI_EvalControlLevel(em_qp[ETRI_IdAxTempCU_Intra], ETRI_IdAxTempCU_Skip, uhDepth, uiLBLevel, e_puiLevelInfo, (pcCU->getQP(0) > 29 && bNoI_Slice));
				bITControl	|= ((uiLBLevel <= 1)||(em_uiINLevel[uhDepth] <= 0));
				#endif

				//--------------------------------------------------------------------------------------------------
				//	Update Minimum HAD Distortion & Level 
				//--------------------------------------------------------------------------------------------------
				em_uiMINHADDistortion[uhDepth] = (em_uiMINHADDistortion[uhDepth] > em_uiINHADDistortion && !em_bSkipMode[ETRI_IdAxTempCU_Inter])? 
												em_uiINHADDistortion : em_uiMINHADDistortion[uhDepth];

				em_uiMINLevel[uhDepth] = (em_uiMINLevel[uhDepth] > em_uiINLevel[uhDepth] && !em_bSkipMode[ETRI_IdAxTempCU_Inter])? 
										em_uiINLevel[uhDepth] : em_uiMINLevel[uhDepth];

				#if CurrentDevelop
				Bool   		e_bTopDepth = false;
				e_bTopDepth = (pcSlice->getDepth() <= ETRI_F64SLiceLevel )? (uhDepth == 1):(uhDepth == 0);

				/// 2015 11 28 by Seok : FastINPRED
				if (!e_bTopDepth)
				{
					if (em_pcBestUpCU[uhDepth]->getPredictionMode(0) == MODE_INTER && !em_pcBestUpCU[uhDepth]->getMergeFlag(0))
					{
						if (em_uiSKLevel[uhDepth] > em_uiINLevel[uhDepth])
						{
							em_bSkipMode[ETRI_IdAxTempCU_Merge] |= true;
							em_bSkipMode[ETRI_IdAxTempCU_Skip] |= true;
						}
					}
				}
				#endif

			}
			else
			{
				em_bSkipMode[ETRI_IdAxTempCU_Intra] |= em_bControlParam[ETRI_IdFastIntraSKIP];
#if ETRI_DEBUG_CODE_CLEANUP
				if (!em_bSkipMode[ETRI_IdAxTempCU_Inter] && !em_bSkipMode[ETRI_IdAxTempCU_Intra])
				{
					em_DbgInfo[ETRI_nDBGInfo_CASE09]++; 
				}
#endif 
			}
		}

		em_bSkipMode[ETRI_IdAxTempCU_Intra] |= (bITControl & bNoI_Slice) > 0; 

#if ETRI_SliceEncoder_MVClip
		//if all modes are skipped, try intra mode
		UInt	e_uiNoSkipIntra =	em_bSkipMode[ETRI_IdAxTempCU_Merge]
							& em_bSkipMode[ETRI_IdAxTempCU_Skip]
							& em_bSkipMode[ETRI_IdAxTempCU_Inter];

		if (uhDepth > 0 && e_uiNoSkipIntra == 1)
		{
#if ETRI_FFESD
			if (!(em_bControlParam[ETRI_IdEarlyMerge] && em_bESD[uhDepth]))
#endif
			em_bSkipMode[ETRI_IdAxTempCU_Intra] = false;
		}
#endif
#if ETRI_DEBUG_CODE_CLEANUP
		if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
		{
			EDPRINTF(stdout, "em_bSkipMode[ETRI_IdAxTempCU_Intra] : %s \n", GetBoolVal(em_bSkipMode[ETRI_IdAxTempCU_Intra] ));
			EDPRINTF(stdout, "bITControl  : %s \n", GetBoolVal(bITControl));
		}
#endif 

	}

	//--------------------------------------------------------------------------------------------------
	//	For Stability :: pcCU->getDepth(0) > 0
	//	Exception : 
	//		[1] MVCLip 의 경우에는  모든 것이 TRUE가 되도록 한다. 조건 : (rpcTempCU->ETRI_getNoMVP() || em_iRDOOffBestMergeCand < 0)
	//		[2] EarlyMerge의 경우에는 TRUE 상태가 되어도 괜찮다. RDOQ가 수행되었기 때문이다.  (불필요 SKIP RDOQ 제거)
	//		[3] RD 값이 너무 커서 강제로 SKIP으로 배당되었으면 BestRD 값은 MAX_INT 이어야 한다.	
	//--------------------------------------------------------------------------------------------------
	Bool	e_bSkipCheck = em_bSkipMode[ETRI_IdAxTempCU_Merge] 
					& em_bSkipMode[ETRI_IdAxTempCU_Inter]
					& em_bSkipMode[ETRI_IdAxTempCU_Intra]
					& em_bSkipMode[ETRI_IdAxTempCU_Skip];

	if (em_bControlParam[ETRI_Id2NxNProcessing])
		e_bSkipCheck &= em_bSkipMode[ETRI_IdAxTempCU_Inter2NxN] & em_bSkipMode[ETRI_IdAxTempCU_InterNx2N];

	// [2] EarlyMerge의 경우 처리 
	e_bSkipCheck &= !(em_bControlParam[ETRI_IdEarlyMerge]);

	// [0][1] if e_bSkipCheck == 0 then em_bSkipMode[ETRI_IdAxTempCU_Skip or ETRI_IdAxTempCU_Intra] must be FALSE
	if (e_bSkipCheck)
	{
		/// When ETRI_SliceEncoder_MVClip Active : When ETRI_SliceEncoder_MVClip = 0, follwoing Condition is always FALSE
		#if ETRI_BUGFIX_ForcedSKIP
		em_bSkipMode[ETRI_IdAxTempCU_Skip] = (pcCU->ETRI_getNoMVP() || em_iRDOOffBestMergeCand < 0);
		#endif
		em_bControlParam[ETRI_IdAllModesSKIPPED] = true;
	}

#if ETRI_TEST_1203
	if (pcSlice->getSliceType() != I_SLICE)
	{
		em_bSkipMode[ETRI_IdAxTempCU_Merge] = true;
		em_bSkipMode[ETRI_IdAxTempCU_Inter] = true;
		em_bSkipMode[ETRI_IdAxTempCU_Intra] = true;
		em_bSkipMode[ETRI_IdAxTempCU_Skip] = false;

		em_bControlParam[ETRI_IdEarlyMerge] = false;
	}
#endif

	//--------------------------------------------------------------------------------------------------
	//	Debug Information 
	//--------------------------------------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		EDPRINTF(stdout, "e_bSkipCheck : %s \n", GetBoolVal(e_bSkipCheck ));
		EDPRINTF(stdout, "MVCLIP  : %s \n", GetBoolVal((pcCU->ETRI_getNoMVP() || em_iRDOOffBestMergeCand < 0)));
	}
#endif 

}

/**
----------------------------------------------------------------------------------------------------------------------------------
	@brief  :  Check Early Skip Decision Function. We use it for similar type of ESD (ETRI_POSTESD) and Fast ESD (ETRI_FASTESD)
	@param:  TComDataCU*& rpcBestCU 
	@param:  Bool* earlyDetectionSkipMode
	* 2015 9 3 by Seok
----------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xCheckEarlySkipDecision(TComDataCU*& rpcBestCU, Bool* earlyDetectionSkipMode, UInt bOP)
{
if (!ETRI_FIXED_ESDOFF || !ETRI_POSTESD || !bOP){return;}

	UInt uiDepth = rpcBestCU->getDepth(0);

	///em_bControlParam[ETRI_IdEarlyMerge] 조건 추가 @ 2015 11 18 by Seok
	if (bOP == 1 || em_bControlParam[ETRI_IdEarlyMerge])
	{
		//Error check, yhee
		if ((rpcBestCU->getPartitionSize(0) == SIZE_NONE) | (rpcBestCU->getPredictionMode(0) == MODE_NONE) | (rpcBestCU->getTotalCost() == MAX_DOUBLE)) {
			return;
		}
#if ETRI_SliceEncoder_MVClip
		for (UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++){
			if (rpcBestCU->getSlice()->getNumRefIdx(RefPicList(uiRefListIdx)) > 0)	{
				TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList(uiRefListIdx));
				TComMv temp = pcCUMvField->getMv(0);
				Int iSourceHeight = m_pcEncCfg->getSourceHeight();
				Bool bBoundary = temp.ETRI_DetectVerMVBoundary(rpcBestCU->getCUPelY(), rpcBestCU->getHeight(0), iSourceHeight);
				if (bBoundary){return;	}
			}
		}
#endif

		//--------------------------------------------------------------------------------------------------
		//	BugFix Information :
		//	다음 Break Condition이 만족되는 경우에는 SKIP Mode가 되어서는 안됨에도 INTRA로 들어가지 못하고 
		//	Early SKIP 판정을 받게 되어 INTRA RDOQ를 수행하지 못한다.
		//	@ 2015 12 2 by Seok
		//--------------------------------------------------------------------------------------------------
#if ETRI_BUGFIX_EarlySkipDecision
 		if (!em_bControlParam[ETRI_IdEarlyMerge])
		{
			Bool	bBreakCondition = false;
			bBreakCondition = !em_bSkipMode[ETRI_IdAxTempCU_Skip] 
							&& em_bSkipMode[ETRI_IdAxTempCU_Inter] 
							&& em_bSkipMode[ETRI_IdAxTempCU_Merge] 
							&& !em_bSkipMode[ETRI_IdAxTempCU_Intra]; 

			if (bBreakCondition)
			{
				uiDepth = rpcBestCU->getDepth(0);
				TComDataCU* pcCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][uiDepth];
				UInt	uiEstPseudoRD	= (UInt)(em_uiITHADDistortion + m_pcRdCost->getSqrtLambda() * pcCU->ETRI_getTotallHeaderBits()); 

				if (uiEstPseudoRD  < em_uiSMHADDistortion)
				{
					if (rpcBestCU->getPredictionMode(0) == MODE_INTER && rpcBestCU->getSkipFlag(0))
					{
						rpcBestCU->setCbf(0, TEXT_LUMA, 1);
					}
				}	
			}
		}
#endif

		if(rpcBestCU->getQtRootCbf( 0 ) == 0){
			if( rpcBestCU->getMergeFlag( 0 ))
			{ 
				*earlyDetectionSkipMode = true;
			}
			else
			{
				Int absoulte_MV=0;
				for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ ){
					if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )	{
						TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
						Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
						Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
						absoulte_MV+=iHor+iVer;
					}
				}

				if(absoulte_MV == 0)
				{ 
					*earlyDetectionSkipMode = true;
				}
			}
		}

		em_bESD[uiDepth] |= *earlyDetectionSkipMode;

	}
	else	/// FAST ERSD @ 2015 9 3 by Seok
	{
		Bool bAlgorithm = ((em_iRDOOffBestMergeCand >= 0) |!ETRI_SliceEncoder_MVClip) > 0; 
		em_bESD [uiDepth]= (em_uiSKLevel[uiDepth] == 0) & bAlgorithm;
	}
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Check MergeCandidate on Slice Boundary
			This fuction is defined and operated by ETRI_SliceEncoder_MVClip
	@return	Bool
	@author: yhee Kim  2015 10 12 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline	Bool TEncCu::ETRI_CheckMergeCandidateonBoundary(TComDataCU* rpcTempCU, UInt uiMergeCand, bool bOperation)
{
	if (!bOperation){return false;}

	TComMvField*	cMvFieldNeighbours	= em_pcMvFieldNeighbours;
	Int* 	mergeCandBuffer = em_pimergeCandBuffer;
	Int  	iSourceHeight = m_pcEncCfg->getSourceHeight();

	//Check the candidate's mv boundary 
	TComMv temp = cMvFieldNeighbours[0 + 2 * uiMergeCand].getMv();
	Bool bBoundary = temp.ETRI_DetectVerMVBoundary(rpcTempCU->getCUPelY(), rpcTempCU->getHeight(0),iSourceHeight);
	if (bBoundary){
		mergeCandBuffer[uiMergeCand] = 1;  return true;//e_uiMvClipValidMergeCand = e_uiMvClipValidMergeCand - 1;			
	}
	else{
		temp = cMvFieldNeighbours[1 + 2 * uiMergeCand].getMv();
		bBoundary = temp.ETRI_DetectVerMVBoundary(rpcTempCU->getCUPelY(), rpcTempCU->getHeight(0), iSourceHeight);
		if (bBoundary){
			mergeCandBuffer[uiMergeCand] = 1; return true; //e_uiMvClipValidMergeCand = e_uiMvClipValidMergeCand - 1;
		}
	}
	return false;

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Working Function of TEncCu::ETRI_ETRI_xCheckSkipMerge_PRED [Instead of xCheckRDCostMerge2Nx2N]
 			ETRI_SET_FUNCTION_OPTIMIZATION = 1 ; ETRI_FIXED_ESDOFF = 1 :
 			rpcTempCU->getCUTransquantBypass(0) = false;
 			bTransquantBypassFlag = false by ETRI_LOSSLESS_OPTIMIZATION;
 			m_pcEncCfg->getUseFastDecisionForMerge() : true
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xCheckSkipMerge_PRED( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth )
{
	assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );

	TComMvField*	cMvFieldNeighbours   	= em_pcMvFieldNeighbours;
	UChar*  		uhInterDirNeighbours	= em_puhInterDirNeighbours;
	Int* 			mergeCandBuffer   		= em_pimergeCandBuffer;

	Int   	numValidMergeCand 	= 0;
	UInt 	uiDistortion  		= MAX_UINT;
	em_iRDOOffBestMergeCand = -1; //To indicate noBestMergCand, set uiRDOOffBestMergeCand to -1

	for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
	uhInterDirNeighbours[ui] = 0;

	rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
	rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );

	for( UInt ui = 0; ui < numValidMergeCand; ++ui )
	mergeCandBuffer[ui] = 0;

#if ETRI_RDOOffBestMergeCand
	//--------------------------------------------------------------------
	//	Check Pseudo SKIP
	//--------------------------------------------------------------------
	UInt  	uiWidth = rpcTempCU->getWidth(0);
	UInt  	uiHeight = rpcTempCU->getHeight(0);

	Int  	uiRDOOffBestMergeCand 	= -1; //To indicate noBestMergCand, set uiRDOOffBestMergeCand to -1
	double 	dRDOOffBestCost  		= MAX_DOUBLE;  				

	UInt*   	puiLevelInfo = em_uiLevelInfo;
	puiLevelInfo[ETRI_IdWidTh]   		= m_ppcOrigYuv[uhDepth]->getWidth();		/// Per each Depth ?? 2015 9 2 by Seok
	puiLevelInfo[ETRI_IdSliceDepth] 	= rpcTempCU->getSlice()->getDepth();
	puiLevelInfo[ETRI_IdCUDepth]  	= uhDepth;
	puiLevelInfo[ETRI_IdLevelLuma] 	= puiLevelInfo[ETRI_IdLevelCb] = puiLevelInfo[ETRI_IdLevelCr] = 0;

	for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
	{
		ETRI_CheckMergeCandidateonBoundary(rpcTempCU, uiMergeCand, ETRI_SliceEncoder_MVClip);

		if(mergeCandBuffer[uiMergeCand]==0)
		{
			// set MC parameters
			rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
			rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

			// do MC
			m_pcPredSearch->ETRI_SM_motionCompensation(rpcTempCU, m_ppcPredYuvTemp[uhDepth], m_ppcOrigYuv[uhDepth], m_pcRdCost, uiDistortion);

			/// 2014 7 12 by Seok Double Version
			if( uiDistortion < dRDOOffBestCost )
			{						 
				dRDOOffBestCost = uiDistortion;
				uiRDOOffBestMergeCand = uiMergeCand;			
				em_uiSMHADDistortion = uiDistortion;
				em_uiSKLevel[uhDepth] = ETRI_EvalLevel(rpcTempCU, ETRI_IdAxTempCU_Skip, puiLevelInfo);	  /// 2015 3 30 by Seok : Debug for QP Selection between Luma and Chroma

				// Store Best Prediction to Temporal Prediction Buffer
				m_ppcPredYuvTemp[uhDepth]->copyPartToPartYuv(m_pppcAxPredYuvTemp[ETRI_IdAxTempCU_Skip][uhDepth], 0, uiWidth, uiHeight);
			}
		}
		
		//setting  mergeCandBuffer[uiMergeCand] for duplicated candidate
		if(uiMergeCand+1 < numValidMergeCand)
		{
			for(UInt ui = 0; ui < uiMergeCand+1;  ui++ )
			{
				UInt uj = uiMergeCand+1;
				if( (!mergeCandBuffer[ui]) && uhInterDirNeighbours[ui] == uhInterDirNeighbours[uj] &&
				cMvFieldNeighbours[0 + 2*ui].getRefIdx()	== cMvFieldNeighbours[0 + 2*uj].getRefIdx()  &&
				cMvFieldNeighbours[0 + 2*ui].getHor()		== cMvFieldNeighbours[0 + 2*uj].getHor() &&
				cMvFieldNeighbours[0 + 2*ui].getVer()		== cMvFieldNeighbours[0 + 2*uj].getVer() &&
				cMvFieldNeighbours[1 + 2*ui].getRefIdx()	== cMvFieldNeighbours[1 + 2*uj].getRefIdx() &&
				cMvFieldNeighbours[1 + 2*ui].getHor()		== cMvFieldNeighbours[1 + 2*uj].getHor() &&
				cMvFieldNeighbours[1 + 2*ui].getVer()		== cMvFieldNeighbours[1 + 2*uj].getVer() )
				{
					mergeCandBuffer[uj]=1;	break;
				}
			}
		}	///  if(uiMergeCand+1 < numValidMergeCand)
	}

	//For Post Processing
	em_inumValidMergeCand = numValidMergeCand;
	em_iRDOOffBestMergeCand = uiRDOOffBestMergeCand; 
	m_pcPredSearch->ETRI_HAD_SetParamforGetLevel(puiLevelInfo[ETRI_IdHADLuma], puiLevelInfo[ETRI_IdHADCb], puiLevelInfo[ETRI_IdHADCr]);		/// 2015 3 31 by Seok : Set Correct HADmard Value for Best Candidate.
	em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdLuma] 	= puiLevelInfo[ETRI_IdHADLuma];
	em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdChromaU] 	= puiLevelInfo[ETRI_IdHADCb];
	em_uiHADInfo[ETRI_IdAxTempCU_Skip][ETRI_IdChromaV] 	= puiLevelInfo[ETRI_IdHADCr];

#if ETRI_DEBUG_CODE_CLEANUP
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		EDPRINTF(stdout, "puiLevelInfo[ETRI_IdHADLuma] : %d \n", puiLevelInfo[ETRI_IdHADLuma]);
		EDPRINTF(stdout, "puiLevelInfo[ETRI_IdHADCb] : %d \n", puiLevelInfo[ETRI_IdHADCb]);
		EDPRINTF(stdout, "puiLevelInfo[ETRI_IdHADCr] : %d \n", puiLevelInfo[ETRI_IdHADCr]);
	}
#endif 

	//if found the BestMergeCand
	if (em_iRDOOffBestMergeCand >= 0){
	//Store Best Estimated Pred YUV to m_ppcPredYuvTemp :: For Merge Processing 
	m_pppcAxPredYuvTemp[ETRI_IdAxTempCU_Skip][uhDepth]->copyPartToPartYuv(m_ppcPredYuvTemp[uhDepth], 0, rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));

	// Prediction Data in Temp/BestCU and m_ppcPredYuvTemp Copy TO Buffer
	TComDataCU* rpcSKTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][uhDepth];
	TComDataCU* rpcSMTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][uhDepth];	

	ETRI_xStoreModeData(rpcSKTempCU, rpcTempCU, uhDepth, ETRI_IdAxTempCU_Skip, SIZE_2Nx2N, true);		
	ETRI_xStoreModeData(rpcSMTempCU, rpcTempCU, uhDepth, ETRI_IdAxTempCU_Merge, SIZE_2Nx2N, true);		
	}

	// Clear MergeFlags For other Processing 
	rpcTempCU->setMergeFlagSubParts( 0, 0, 0, uhDepth ); 

#else
	//--------------------------------------------------------------------
	//	Check Real SKIP
	//--------------------------------------------------------------------	
	
	UInt	e_uiNumBoundaryCand = 0;// number of invalid cand,  yhee
	em_iRDOOffBestMergeCand = 0; //orginal 
	for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
	{
#if ETRI_SliceEncoder_MVClip
		//Check the candidate's mv boundary
		Int iSourceHeight = m_pcEncCfg->getSourceHeight();				
		TComMv temp = cMvFieldNeighbours[0 + 2 * uiMergeCand].getMv();
		Bool bBoundary = temp.ETRI_DetectVerMVBoundary(rpcTempCU->getCUPelY(), rpcTempCU->getHeight(0),iSourceHeight);
		if (bBoundary){
			mergeCandBuffer[uiMergeCand] = 1; e_uiNumBoundaryCand++; continue;
		}			
		else{
			temp = cMvFieldNeighbours[1 + 2 * uiMergeCand].getMv();			
			bBoundary = temp.ETRI_DetectVerMVBoundary(rpcTempCU->getCUPelY(), rpcTempCU->getHeight(0), iSourceHeight);
			if (bBoundary){
				mergeCandBuffer[uiMergeCand] = 1; e_uiNumBoundaryCand++; continue;
			}
		}		
#endif
		// set MC parameters
		rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
		rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
		rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
		rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
		rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
		rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
		rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

		// do MC
		m_pcPredSearch->ETRI_SM_motionCompensation(rpcTempCU, m_ppcPredYuvTemp[uhDepth], m_ppcOrigYuv[uhDepth], m_pcRdCost, uiDistortion);

		// estimate residual and encode everything
		m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
		m_ppcOrigYuv    [uhDepth],
		m_ppcPredYuvTemp[uhDepth],
		m_ppcResiYuvTemp[uhDepth],
		m_ppcResiYuvBest[uhDepth],
		m_ppcRecoYuvTemp[uhDepth],
		false);

		rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
		Int orgQP = rpcTempCU->getQP( 0 );
		xCheckDQP( rpcTempCU );
		xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
		rpcTempCU->initEstData( uhDepth, orgQP, false );

		// If no residual when allowing for one, then set mark to not try case where residual is forced to 0
		if (rpcBestCU->getQtRootCbf(0) == 0 )	{mergeCandBuffer[uiMergeCand] = 1;	em_iRDOOffBestMergeCand = uiMergeCand; break;}

	}

	em_inumValidMergeCand = numValidMergeCand;

#if ETRI_SliceEncoder_MVClip		
	if (e_uiNumBoundaryCand == numValidMergeCand)
		em_iRDOOffBestMergeCand = -1;
	else
#endif
	{
		// Prediction Data in Temp/BestCU and m_ppcPredYuvTemp Copy TO Buffer
		TComDataCU* rpcSKTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][uhDepth];
		TComDataCU* rpcSMTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][uhDepth];

		ETRI_xStoreModeData(rpcSKTempCU, rpcBestCU, uhDepth, ETRI_IdAxTempCU_Skip, SIZE_2Nx2N, true);
		ETRI_xStoreModeData(rpcSMTempCU, rpcBestCU, uhDepth, ETRI_IdAxTempCU_Merge, SIZE_2Nx2N, true);
	}
#endif

}
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief  : 	Auxiliary Function for Only ETRI_RDOOffBestMergeCand :: When Apply Fast RDOOff in V12 to E265, This Function should be changed \
	       		Optimized For Skip First Process. 
	@param: TComDataCU*& rpcBestCU
	@param: TComDataCU*& rpcTempCU
	@param: Bool*&  earlyDetectionSkipMode 	
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xCheckSkipMerge_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth)
{

#if ETRI_RDOOffBestMergeCand
	//------------------------------------------------------------------------------------------------------------
	//	Control Logic : Fast ESD is not perfectly operated owing to the QP range. 
	//	Therefore, in spite of logical inconsistency, we use the em_bSkipMode[ETRI_IdAxTempCU_Merge]= false : always to control this function.
	//------------------------------------------------------------------------------------------------------------
	if ((ETRI_FRDOOffBestMergeCand || ETRI_SliceEncoder_MVClip) && em_bSkipMode[ETRI_IdAxTempCU_Merge]) { return; }

	Int*			mergeCandBuffer 		= em_pimergeCandBuffer;

	//ETRI_SKIP_MergeSkip
	TComDataCU*  	rpcSKTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Merge][uhDepth];
	ETRI_xStoreModeData(rpcTempCU, rpcSKTempCU, uhDepth, ETRI_IdAxTempCU_Merge, SIZE_2Nx2N, false);

	// estimate residual and encode everything
	m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
	m_ppcOrigYuv	[uhDepth],
	m_ppcPredYuvTemp[uhDepth],
	m_ppcResiYuvTemp[uhDepth],
	m_ppcResiYuvBest[uhDepth],
	m_ppcRecoYuvTemp[uhDepth],
	false);

	rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
#if !ETRI_REMOVE_XCHECKDQP
	xCheckDQP( rpcTempCU );
#endif 
 	xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);

	//yhee
	if (em_iRDOOffBestMergeCand < 0){
		EDPRINTF(stdout, "=====ERROR ETRI_SliceEncoder_MVClip  < 0 \n");		
	}
	else{
	// If no residual when allowing for one, then set mark to not try case where residual is forced to 0
	mergeCandBuffer[em_iRDOOffBestMergeCand] = (rpcBestCU->getQtRootCbf(0) == 0 );	
	}

	// For Other Processing 
	rpcTempCU->setSkipFlagSubParts(0, 0, uhDepth );

	//==========================================================================
	//	Debug Information
	//==========================================================================
	/// Analysis and Debug 2015 11 18 by Seok
#if ETRI_DEBUG_CODE_CLEANUP
	em_DbgInfo[ETRI_nDBGInfo_CASE07]++ ; 

	if (em_uiSKLevel[uhDepth] >  ETRI_TH_MERGEMODE && !em_bControlParam[ETRI_IdEarlyMerge])
	EDPRINTF(stderr, "Merge Logical Error SK Level > 5 : %d \n", em_uiSKLevel[uhDepth]); 
#endif 
#endif
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: For Early SKIP Merge : \ 
			[1] Merge RDOQ 조건이 만족되면 em_bESD |= true로 만들어 INTER/INTRA를 전부 뺄 수 있도록 한다. \
			[2] MVClip을 대비하여 MerSKIP이 False 일때만 수행한다. \
			[3] 여기서 Merge RDOQ가 수행되었다면, 다른 곳에서 RDOQ가 수행되지 않도록 한다. \
			[4] 이전 버전과의 호환성을 유지할 수 있도록 bOP를 사용하여 수행 여부가 결정 되도록 한다. \
	@return	Void
	@author: Jinwuk Seok  2015 11 18 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_xEarlyCheckSkipMerge_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth, Int iQP, Bool bIsLosslessMode, Bool bOP)
{
	//--------------------------------------------------------------------
	//	ETRI_SliceEncoder_MVClip 관련 조건을 추가하지 않아도 된다. 
	//	MVClip 에 걸리면 em_bSkipMode[ETRI_IdAxTempCU_Merge]=true 이기 때문이다. 
	//	[1] SKIP Merge 가 TRUE 이면 SKIP Mode 할 필요 없다
	//  [2] Merge 했을 경우 CBF 가 0 이면 SKIP RDOQ 해야 한다.
	//	by  2015 11 20 by Seok	
	//--------------------------------------------------------------------
	em_bControlParam[ETRI_IdEarlyMerge] &= bOP;

	if (em_bControlParam[ETRI_IdEarlyMerge])
	{
		ETRI_xCheckSkipMerge_RDOQ(rpcBestCU, rpcTempCU, uhDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
		rpcTempCU->initEstData(uhDepth, iQP, bIsLosslessMode);
#endif 
		em_bSkipMode[ETRI_IdAxTempCU_Skip] |= (rpcBestCU->getQtRootCbf(0) > 0 && !em_bSkipMode[ETRI_IdAxTempCU_Merge]);

		ETRI_xCheckOnlySKIP_RDOQ(rpcBestCU, rpcTempCU, uhDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
		rpcTempCU->initEstData(uhDepth, iQP, bIsLosslessMode);
#endif 
		//--------------------------------------------------------------------
		//	For Debug Info : Removable
		//--------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
		if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint]){
			EDPRINTF(ETRI_ANALYSIS_TYPE, "--------- Early SKIP_Merge RDOQ Result ---------- \n");
			ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uhDepth, ETRI_IdAxTempCU_Merge, ETRI_RDOQ, ETRI_ANALYSIS_ON);
		}
#endif 

		//--------------------------------------------------------------------
		//	Stability : To avoid repeated Merge RDOQ in the other side @ 2015 11 18 by Seok
		//--------------------------------------------------------------------
		if (!em_bSkipMode[ETRI_IdAxTempCU_Skip])
			em_bSkipMode[ETRI_IdAxTempCU_Skip] = true;

		if (!em_bSkipMode[ETRI_IdAxTempCU_Merge])
			em_bSkipMode[ETRI_IdAxTempCU_Merge] = true;
	}
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: For Early Inter : \ 
			[1] 만일 Inter RDOQ를 수행하여 CBF가 0 이면 INTRA Prediction/RDOQ를 수행하지 않는다. \
			[4] 이전 버전과의 호환성을 유지할 수 있도록 bOP를 사용하여 수행 여부가 결정 되도록 한다. \
	@return	Void
	@author: Jinwuk Seok  2015 11 18 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_xEarlyCheckInter_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth, Int iQP, Bool bIsLosslessMode, Bool bOP)
{
	//--------------------------------------------------------------------
	//	by  2015 11 20 by Seok	
	//--------------------------------------------------------------------
#if 0	
	em_bControlParam[] &= bOP;

	if (em_bControlParam[])
	{




	}
#endif
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief  : 	Auxiliary Function for ETRI_CheckSimpleCostMergeSkip2Nx2N_v3PP  
	       		Optimized For Skip First Process. It is Pair to Another Function : ETRI_CheckSimpleCostMergeSkip2Nx2N_v3PP
	@param: TComDataCU*& rpcBestCU
	@param: TComDataCU*& rpcTempCU
	@param: Bool*&  earlyDetectionSkipMode 	
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_xCheckOnlySKIP_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhDepth)
{
	//--------------------------------------------------------------------
	//	Fast Processing in V12
	//--------------------------------------------------------------------


	//--------------------------------------------------------------------
	//	Standard Processing in HM 
	//--------------------------------------------------------------------
	if ((ETRI_FRDOOffBestMergeCand || ETRI_SliceEncoder_MVClip) && em_bSkipMode[ETRI_IdAxTempCU_Skip]) { return; }

	TComMvField* 	cMvFieldNeighbours		= em_pcMvFieldNeighbours;
	UChar*			uhInterDirNeighbours	= em_puhInterDirNeighbours;
	Int*   			mergeCandBuffer 		= em_pimergeCandBuffer;
	Int    			numValidMergeCand		= em_inumValidMergeCand;

	TComDataCU*  	rpcSMTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Skip][uhDepth];

	ETRI_xStoreModeData(rpcTempCU, rpcSMTempCU, uhDepth, ETRI_IdAxTempCU_Skip, SIZE_2Nx2N, false);

	for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
	{
		if(mergeCandBuffer[uiMergeCand]==0)
		{
			if (ETRI_CheckMergeCandidateonBoundary(rpcTempCU, uiMergeCand, ETRI_SliceEncoder_MVClip)){continue;}

			///< 이렇게 하지 않으면 rpcBestCu 에서 정확한 데이터가 없어서 Parameter 세팅 중에 다운 될 수 있다. @ 2015 9 7 by Seok : Debug
			rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level

			// set MC parameters
			rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
			rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
			rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

			m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );

			// estimate residual and encode everything
			m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
			m_ppcOrigYuv	[uhDepth],
			m_ppcPredYuvTemp[uhDepth],
			m_ppcResiYuvTemp[uhDepth],
			m_ppcResiYuvBest[uhDepth],
			m_ppcRecoYuvTemp[uhDepth],
			true);

			rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
#if !ETRI_REMOVE_XCHECKDQP
			xCheckDQP( rpcTempCU );
#endif 
			xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
		
		}
	}

	//--------------------------------------------------------------------
	//	Debug Information :: 만일 LOGIC 문제가 생기면, 화면에 다음 메시지가 뜬다. 
	//	분석기 돌려 보면 확실히 문제가 있다
	//--------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
	if (em_uiSKLevel[uhDepth] >  ETRI_TH_SKIPMODE && !em_bControlParam[ETRI_IdEarlyMerge])
	{
		//if (rpcBestCU->getSlice()->getPOC() == 28)
		//FINDDBGLOCATION(stdout, rpcBestCU, uhDepth);
		if (!em_bControlParam[ETRI_IdAllModesSKIPPED])
		EDPRINTF(stderr, "SKIP Logical Error SK Level > 1 : %d \n", em_uiSKLevel[uhDepth]);	
	}
#endif 
}
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Auxiliary Inline Function for INTER Processing  [DEVELOPING] : This function will be divided into Prediction and RD part
	@return	Void
	@author: Jinwuk Seok  2015 8 25 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_xCheckInter_PRED( TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth, Int iQP)
{
	//------------------------------------------------------------------------
	//	Check Processing is Operated or Not
	//------------------------------------------------------------------------
	if (	em_bSkipMode[ETRI_IdAxTempCU_Inter] | 
		em_bSkipMode[ETRI_IdAxTempCU_Inter2NxN] | 
		em_bSkipMode[ETRI_IdAxTempCU_InterNx2N] ){return;}

	//------------------------------------------------------------------------
	//	Inter Prediction
	//------------------------------------------------------------------------
	TComDataCU* rpcINTempCU = nullptr;

	rpcTempCU->setPartSizeSubParts	( ePartSize, 0, uiDepth );
	rpcTempCU->setPredModeSubParts	( MODE_INTER, 0, uiDepth );
	rpcTempCU->setMergeAMP (true);
	rpcTempCU->ETRI_setNoMVP(false);

	m_pcPredSearch->ETRI_predInterSearch ( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], false, false);

#if ETRI_SliceEncoder_MVClip	
	//Since this control setting location is critical, consult to author if need to move, yhee
	em_bSkipMode[ETRI_IdAxTempCU_Inter] |= rpcTempCU->ETRI_getNoMVP();
#endif
	UInt	e_iPartSizeIdx = (ePartSize == SIZE_2Nx2N)? ETRI_IdAxTempCU_Inter : ((ePartSize == SIZE_2NxN)? ETRI_IdAxTempCU_Inter2NxN : ETRI_IdAxTempCU_InterNx2N); 

	rpcINTempCU = m_pppcAxTempCU[e_iPartSizeIdx][uiDepth];
	ETRI_xStoreModeData(rpcINTempCU, rpcTempCU, uiDepth, e_iPartSizeIdx, ePartSize, true);			

	//------------------------------------------------------------------------
	//	Post Processing
	//------------------------------------------------------------------------
	em_uiINHADDistortion	= ETRI_EvalHAD(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], em_uiLevelInfo);
	em_uiINLevel[uiDepth]	= ETRI_EvalLevel(rpcTempCU, ETRI_IdAxTempCU_Inter, em_uiLevelInfo);

	em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdLuma] 	= em_uiLevelInfo[ETRI_IdHADLuma];
	em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdChromaU]	= em_uiLevelInfo[ETRI_IdHADCb];
	em_uiHADInfo[ETRI_IdAxTempCU_Inter][ETRI_IdChromaV]	= em_uiLevelInfo[ETRI_IdHADCr];

}

__inline void TEncCu::ETRI_xCheckInter_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth, Bool& doNotBlockPu)
{
	//------------------------------------------------------------------------
	//	Check Processing is Operated or Not
	//------------------------------------------------------------------------
	if (	em_bSkipMode[ETRI_IdAxTempCU_Inter] | 
		em_bSkipMode[ETRI_IdAxTempCU_Inter2NxN] | 
		em_bSkipMode[ETRI_IdAxTempCU_InterNx2N] ){return;}

	//------------------------------------------------------------------------
	//	Inter RDOQ
	//------------------------------------------------------------------------
	UInt	e_iPartSizeIdx = (ePartSize == SIZE_2Nx2N)? ETRI_IdAxTempCU_Inter : ((ePartSize == SIZE_2NxN)? ETRI_IdAxTempCU_Inter2NxN : ETRI_IdAxTempCU_InterNx2N); 
	TComDataCU* rpcINTempCU = m_pppcAxTempCU[e_iPartSizeIdx][uiDepth];

	ETRI_xStoreModeData(rpcTempCU, rpcINTempCU, uiDepth, e_iPartSizeIdx, ePartSize, false);

	m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], 
												m_ppcResiYuvBest[uiDepth], m_ppcRecoYuvTemp[uiDepth], false );
	rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
	
#if !ETRI_REMOVE_XCHECKDQP
	xCheckDQP( rpcTempCU );
#endif 
	xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);

	if(!m_pcEncCfg->getUseEarlySkipDetection() && m_pcEncCfg->getUseCbfFastMode())
	doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
}
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
__inline void TEncCu::ETRI_xCheckInter2NxNNx2N_PRED(TComDataCU*& rpcTempCU, Int iQP, UInt uiDepth)
{
	if (!(em_bControlParam[ETRI_Id2NxNProcessing] && ETRI_MODIFICATION_V03)){return;}

	ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_InterNx2N, ETRI_PRED);
	ETRI_xCheckInter_PRED(rpcTempCU, SIZE_Nx2N, uiDepth, iQP);

	ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter2NxN, ETRI_PRED);
	ETRI_xCheckInter_PRED(rpcTempCU, SIZE_2NxN, uiDepth, iQP);

}

__inline void TEncCu::ETRI_xCheckInter2NxNNx2N_RDOQ(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Int iQP, Bool bIsLosslessMode, UInt uiDepth, Bool& doNotBlockPu)
{
	if (!(em_bControlParam[ETRI_Id2NxNProcessing] && ETRI_MODIFICATION_V03)){return;}

	ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_InterNx2N, ETRI_RDOQ);
	ETRI_xCheckInter_RDOQ(rpcBestCU, rpcTempCU, SIZE_Nx2N, uiDepth, doNotBlockPu);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
	rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
	ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter2NxN, ETRI_RDOQ);
	ETRI_xCheckInter_RDOQ(rpcBestCU, rpcTempCU, SIZE_2NxN, uiDepth, doNotBlockPu);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
}


__inline void TEncCu::ETRI_PseudoInter2NxNNx2N(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Int iQP, Bool bIsLosslessMode, UInt uiDepth, Bool& doNotBlockPu)
{
#if !ETRI_MODIFICATION_V03

	if (em_bControlParam[ETRI_Id2NxNProcessing])	
	{
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_InterNx2N, ETRI_PRED);
		ETRI_xCheckInter_PRED(rpcTempCU, SIZE_Nx2N, uiDepth, iQP);
	
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_InterNx2N, ETRI_RDOQ);
		ETRI_xCheckInter_RDOQ(rpcBestCU, rpcTempCU, SIZE_Nx2N, uiDepth, doNotBlockPu);
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
	
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter2NxN, ETRI_PRED);
		ETRI_xCheckInter_PRED(rpcTempCU, SIZE_2NxN, uiDepth, iQP);
	
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter2NxN, ETRI_RDOQ);
		ETRI_xCheckInter_RDOQ(rpcBestCU, rpcTempCU, SIZE_2NxN, uiDepth, doNotBlockPu);
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
	}

#endif	
}
#endif 
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Normal INTER Processing  [DEVELOPING] : This function will be divided into Prediction and RD part
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_Normal_INTERProcess( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP, 
													Bool bIsLosslessMode, Bool& earlyDetectionSkipMode, Bool& doNotBlockPu)
{
//	em_bInterPrediction  	= false;	/// 2015 5 4 by Seok : Prevent repeat Inter Prediction : Initialization 
//	em_bPrePredictionIntra 	= false;	/// 2014 11 23 by Seok : Prevent repeat Intra Prediction : Initialization 

#if ETRI_FIXED_ESDOFF

	//================ Debug Point ===================
//	if (rpcTempCU->getSlice()->getPOC() == 8 && rpcTempCU->getAddr() == 4 && uiDepth == 2 && rpcTempCU->getZorderIdxInCU() == 64)
#if ETRI_DEBUG_CODE_CLEANUP
    if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	EDPRINTF(stderr, "============= DEBUG BEGIN ===========\n");
#endif 
	//================ Debug Point ===================

	//--------------------------------------------------------------------------------
	//	Prediction Stage 
	//	Jinwuk Seok 2015 0801
	//--------------------------------------------------------------------------------
	UInt 	uiFastESDId = (( ETRI_FPOSTESD ) << 1);	///< SKIP/Merge Prediction 직후 ESD를 할 것인지를 Check 하는 변수 @ 2015 9 3 by Seok

	// do inter modes, SKIP and 2Nx2N
	if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
	{
		//--------------------------------------------------------------------------------
		//	If ESD = 0 and the configuration of ESD is fixed, I recommend the following simple code. 
		//	Jinwuk Seok 2015 0801
		//--------------------------------------------------------------------------------
		// SKIP Prediction  : Instead of xCheckRDCostMerge2Nx2N : by Merge for inter_2Nx2N
		ETRI_xCheckSkipMerge_PRED( rpcBestCU, rpcTempCU, uiDepth); 
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Merge, ETRI_PRED); /// Check no ValidMergeCandidates for ETRI_SliceEncoder_MVClip, yhee
		ETRI_xEarlyCheckSkipMerge_RDOQ(rpcBestCU, rpcTempCU, uiDepth, iQP, bIsLosslessMode, ETRI_FFESD);
		ETRI_xCheckEarlySkipDecision(rpcBestCU, &earlyDetectionSkipMode, uiFastESDId);	/// It requires REVISION @ 2015 9 3 by Seok uiFastESDId

#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Merge, ETRI_PRED, ETRI_ANALYSIS_ON);
#endif 

		// Inter[2Nx2N] Prediction : Instead of xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter, ETRI_PRED);
		ETRI_xCheckInter_PRED(rpcTempCU, SIZE_2Nx2N, uiDepth, iQP);

#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Inter, ETRI_PRED, ETRI_ANALYSIS_ON);
#endif 

		// Inter[2NxN][Nx2N] Prediction : Including ETRI_xControlPUProcessing :: ETRI_MODIFICATION_V03 Only
#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
		ETRI_xCheckInter2NxNNx2N_PRED(rpcTempCU, iQP, uiDepth);
#endif 
	}
	
	// do intra modes Prediction
	ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Intra, ETRI_PRED);
	ETRI_xCheckIntra_PRED(rpcBestCU, rpcTempCU, SIZE_2Nx2N, uiDepth, iQP, earlyDetectionSkipMode, ETRI_PRED);

#if ETRI_DEBUG_CODE_CLEANUP
	ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Intra, ETRI_PRED, ETRI_ANALYSIS_ON);
#endif 

	// Inter[2Nx2N] Additional Prediction : When Inter SKIP and Intra ON, if the Prediction Cost of Intra is not sufficint small, the Additional Inter is active.
	if( rpcBestCU->getSlice()->getSliceType() != I_SLICE && em_bControlParam[ETRI_IdAdditionalInter])
	{
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter, ETRI_AdditionalInter);
		ETRI_xCheckInter_PRED(rpcTempCU, SIZE_2Nx2N, uiDepth, iQP);
	}


	//--------------------------------------------------------------------------------
	//	Transform Stage (Motion Compensation with RDOQ)
	//	Jinwuk Seok 2015 0801
	//--------------------------------------------------------------------------------

	if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
	{
		/// 2015 11 15 by Seok : For DEBUG and ANalysis 
#if ETRI_DEBUG_CODE_CLEANUP
		em_DbgInfo[ETRI_nDBGInfo_TotalInfo	]++; 
#endif 
  		// Merge RDOQ  : Instead of xCheckRDCostMerge2Nx2N : by Merge for inter_2Nx2N
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Merge, ETRI_RDOQ);
		ETRI_xCheckSkipMerge_RDOQ(rpcBestCU, rpcTempCU, uiDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Merge, ETRI_RDOQ, ETRI_ANALYSIS_ON);
#endif 

		// SKIP RDOQ :: When Apply Fast RDOOff in V12 to E265, This Function should be changed
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Skip, ETRI_RDOQ);
		ETRI_xCheckOnlySKIP_RDOQ(rpcBestCU, rpcTempCU, uiDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );	///< unnecessary 2015 9 13 by Seok
#endif 
#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Skip, ETRI_RDOQ, ETRI_ANALYSIS_ON);
#endif 

		// Inter[2Nx2N] RDOQ : Instead of xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Inter, ETRI_RDOQ);
		ETRI_xCheckInter_RDOQ(rpcBestCU, rpcTempCU, SIZE_2Nx2N, uiDepth, doNotBlockPu);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Inter, ETRI_RDOQ, ETRI_ANALYSIS_ON);
#endif 

		// POST ESD :: Fast ESD is operated the above : following the SKIPMerge Prediction (ETRI_xCheckSkipMerge_PRED)
		ETRI_xCheckEarlySkipDecision(rpcBestCU, &earlyDetectionSkipMode, ETRI_POSTESD);

#if !ETRI_PU_2NxN_Nx2N_CODE_CLEANUP
		// Inter[2NxN][Nx2N] RDOQ : Including ETRI_xControlPUProcessing :: ETRI_MODIFICATION_V03 Only
		ETRI_xCheckInter2NxNNx2N_RDOQ(rpcBestCU, rpcTempCU, iQP, bIsLosslessMode, uiDepth, doNotBlockPu);

		// Inter[2Nx2N][Nx2N] Processing : Incluiding Prediction & RDOQ :: ETRI_MODIFICATION_V02 Only
		ETRI_PseudoInter2NxNNx2N(rpcBestCU, rpcTempCU, iQP, bIsLosslessMode, uiDepth, doNotBlockPu);
#endif 
		// Intra RDOQ Decision Control @ 2015 11 19 by Seok
		ETRI_xControlPUProcessing(rpcTempCU, uiDepth, ETRI_IdAxTempCU_Intra, ETRI_RDOQ);

	}

	// check chosen motion vector for debug
#if !ETRI_CODE_FURTHER_CLEANUP
	ETRI_xCheckBestModeMV(rpcBestCU, uiDepth);
#endif 
#else
	// do inter modes, SKIP and 2Nx2N
	if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
	{
		//--------------------------------------------------------------------------------
		//	Original HM Code
		//--------------------------------------------------------------------------------
		// 2Nx2N
		if(m_pcEncCfg->getUseEarlySkipDetection())
		{
			xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
			rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
		}

		// SKIP
		xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU, &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

		if(!m_pcEncCfg->getUseEarlySkipDetection())
		{
			// 2Nx2N, NxN
			xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
			rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
		}

		if(!m_pcEncCfg->getUseEarlySkipDetection() && m_pcEncCfg->getUseCbfFastMode())
		doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
	}


	// If you want Seperated INTRA prediction and transform, use this code.
	if(!earlyDetectionSkipMode)
		ETRI_xCheckIntra_PRED(rpcBestCU, rpcTempCU, SIZE_2Nx2N, uiDepth, iQP, earlyDetectionSkipMode, ETRI_RDOQ); 
#endif

}

/**
-----------------------------------------------------------------------------------------------------------------------------------------------
	@brief: 	Auxiliary Inline Function for INTRA Processing  [DEVELOPING] : This function will be divided into Prediction and RD part \
		   	However, at this time, it operated only for LUMA prediction, not chroma prediction. \
		   	We should amend chroma prediction mode at this function.
	@return	Void
	@author: Jinwuk Seok  2015 8 25 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_xCheckIntra_PRED( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, UInt uiDepth,  Int iQP, bool earlyDetectionSkipMode, Bool SubStage)
{
	//------------------------------------------------------------------------
	//	Check Processing is Operated or Not
	//------------------------------------------------------------------------
	if (earlyDetectionSkipMode || em_bSkipMode[ETRI_IdAxTempCU_Intra]) {return;}

	Bool e_bSpeedUp	= rpcBestCU->getSlice()->getSliceType() != I_SLICE;
	Bool e_bCBFSum 	= (rpcBestCU->getCbf( 0, TEXT_LUMA ) +  rpcBestCU->getCbf( 0, TEXT_CHROMA_U) + rpcBestCU->getCbf( 0, TEXT_CHROMA_V)) == 0;

#if ETRI_SliceEncoder_MVClip
	Bool e_bSkipCheck = (rpcBestCU->getPartitionSize(0) != SIZE_NONE);
	if (e_bSpeedUp && e_bCBFSum && SubStage && e_bSkipCheck)	{ return; }
#else
	if (e_bSpeedUp && e_bCBFSum && SubStage)	{return;}
#endif

	//------------------------------------------------------------------------
	//	INTRA Prediction : LUMA
	//------------------------------------------------------------------------
	TComDataCU* rpcITTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][uiDepth];

	rpcTempCU->setPartSizeSubParts( ePartSize, 0, uiDepth );
	rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );

	// Luma Only : bSeparateLumaChroma = true
	m_pcPredSearch->ETRI_IntraPrediction( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth]);
	m_pcPredSearch->ETRI_IntraLumaPred_V2(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], 0);

#if ETRI_DEBUG_CODE_CLEANUP
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		FILE* _dbgType = ETRI_ANALYSIS_TYPE;
		UInt	  uiDbgVal[3], *puiDbgVal = & uiDbgVal[0];

		puiDbgVal = m_pcPredSearch->ETRI_getHADCost();
		
		EDPRINTF(_dbgType, "--------- INTRA LUMA Prediction --------\n");	
		EDPRINTF(_dbgType, "INTRA Pred HAD LUMA : %d \n", puiDbgVal[ETRI_IdLuma]);
		EDPRINTF(_dbgType, "INTRA Pred HAD CB   : %d \n", puiDbgVal[ETRI_IdChromaU]);
		EDPRINTF(_dbgType, "INTRA Pred HAD CR   : %d \n", puiDbgVal[ETRI_IdChromaV]);
	}
#endif 


	//------------------------------------------------------------------------
	// FFast Intra Prediction : 
	//	HAD value 조건을 만족하면 Chroma Prediction 과 INTRA RDOQ를 모두 SKIP 한다.
	//	[1] Prediction 단계에서 Merge가 TRUE 이고 INTER가 TRUE 이면 SKIP은 RDOQ단계에서 TRUE 이므로 INTRA Prediction을 무조건 해야 한다.
	//  	@ 2015 11 19 by Seok
	//------------------------------------------------------------------------
#if ETRI_FastIntraSKIP
	UInt*	e_uiIntraHAdDistortion	= m_pcPredSearch->ETRI_getHADCost();
	Bool 	e_bFastIntraCondition 	= false;
	e_bFastIntraCondition	|= em_uiMINHADDistortion[uiDepth] < e_uiIntraHAdDistortion[ETRI_IdLuma] && rpcTempCU->getSlice()->getSliceType() != I_SLICE;

	///[1] Debug Code @ 2015 12 11 by Seok
	e_bFastIntraCondition	&= !(em_bSkipMode[ETRI_IdAxTempCU_Merge] && em_bSkipMode[ETRI_IdAxTempCU_Inter]);

	#if ETRI_SliceEncoder_MVClip
	e_bFastIntraCondition &= !(rpcTempCU->ETRI_getNoMVP() || em_iRDOOffBestMergeCand < 0);
	#endif
	
	if (e_bFastIntraCondition)
	{
		///Level Check 로 끝나버리므로 HAD Distortion 을 사용하는 Check 까지 들어가지 않는다. 
		///또한 무조건 INTRA를 해야 하는 문제점도 있기 떄문에 여기서 SKIPMODE를 결정하면 안된다 @ 2015 11 19 by Seok
		em_uiITLevel[uiDepth] 	= MAX_INT;	
		em_uiITHADDistortion	= MAX_INT;
		em_uiLevelInfo[ETRI_IdHADLuma] = e_uiIntraHAdDistortion[ETRI_IdLuma];

		em_bControlParam[ETRI_IdFastIntraSKIP] = true;
		em_bControlParam[ETRI_IdAdditionalInter] = false;
		return;
	}
#endif
	
	//------------------------------------------------------------------------
	//	INTRA Prediction : Chroma 
	//------------------------------------------------------------------------
	m_pcPredSearch->ETRI_IntraChromaPred	(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], true);
#if ETRI_DEBUG_CODE_CLEANUP
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		FILE* _dbgType = ETRI_ANALYSIS_TYPE;
		UInt	  uiDbgVal[3], *puiDbgVal = & uiDbgVal[0];

		puiDbgVal = m_pcPredSearch->ETRI_getHADCost();
		
		EDPRINTF(_dbgType, "--------- INTRA Chroma Prediction --------\n");	
		EDPRINTF(_dbgType, "INTRA Pred HAD LUMA : %d \n", puiDbgVal[ETRI_IdLuma]);
		EDPRINTF(_dbgType, "INTRA Pred HAD CB   : %d \n", puiDbgVal[ETRI_IdChromaU]);
		EDPRINTF(_dbgType, "INTRA Pred HAD CR   : %d \n", puiDbgVal[ETRI_IdChromaV]);
	}
#endif 
	//------------------------------------------------------------------------
	//	Post Processing : Evaluation of Prediction Cost and Additional Inter 
	//------------------------------------------------------------------------
	// Store Prediction Data 
	ETRI_xStoreModeData(rpcITTempCU, rpcTempCU, uiDepth, ETRI_IdAxTempCU_Intra, SIZE_2Nx2N, true);	

	em_uiITLevel[uiDepth]	= ETRI_EvalLevel(rpcTempCU, ETRI_IdAxTempCU_Intra, em_uiLevelInfo);
	em_uiITHADDistortion 	= em_uiLevelInfo[ETRI_IdHADLuma] + em_uiLevelInfo[ETRI_IdHADCb] + em_uiLevelInfo[ETRI_IdHADCr]; 

	em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdLuma] 	= em_uiLevelInfo[ETRI_IdHADLuma];
	em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdChromaU]	= em_uiLevelInfo[ETRI_IdHADCb];
	em_uiHADInfo[ETRI_IdAxTempCU_Intra][ETRI_IdChromaV]	= em_uiLevelInfo[ETRI_IdHADCr];

	/// Set Additional Inter Condition :: [DEVELOPING]
	em_bControlParam[ETRI_IdAdditionalInter] = false;

}


__inline void TEncCu::ETRI_xCheckIntra_RDOQ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth)
{
	TComDataCU* rpcITTempCU = m_pppcAxTempCU[ETRI_IdAxTempCU_Intra][uiDepth];

	//=========  Restore Prediction Data  ========
	ETRI_xStoreModeData(rpcTempCU, rpcITTempCU, uiDepth, ETRI_IdAxTempCU_Intra, SIZE_2Nx2N, false);

	//========== MC and RD Optimization ========	
	UInt 	uiPreCalcDistC = 0;

	m_pcPredSearch->ETRI_estIntraPredQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], 
									m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC, true );	
	m_pcPredSearch->ETRI_IntraChromaPred	(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], false);
	m_ppcRecoYuvTemp[uiDepth]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
	m_pcPredSearch->ETRI_estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], 
									m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC );

	//===========  Evaluate RD cost ===========
	m_pcEntropyCoder->resetBits();
	if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
	{
		m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,			true );
	}
	m_pcEntropyCoder->encodeSkipFlag( rpcTempCU, 0,			true );
	m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,		    true );
	m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth,true );
	m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0,		    true );
	m_pcEntropyCoder->encodeIPCMInfo( rpcTempCU, 0, 		true );

	// Encode Coefficients
	Bool bCodeDQP = getdQPFlag();
	m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, rpcTempCU->getWidth (0), rpcTempCU->getHeight(0), bCodeDQP );
	setdQPFlag( bCodeDQP );

	m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

	rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
	rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
	rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

	//------------------------------------------------------------------------
	//	For Debug and Analysis 
	//------------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		FILE* _dbgType = ETRI_ANALYSIS_TYPE;
		EDPRINTF(_dbgType, "INTRA Total RDCost : %.2f \n", rpcTempCU->getTotalCost());
		EDPRINTF(_dbgType, "INTRA Distortion      : %d \n", rpcTempCU->getTotalDistortion() );
	}
#endif 

	//------------------------------------------------------------------------
	//	Final Stage 
	//------------------------------------------------------------------------
#if !ETRI_REMOVE_XCHECKDQP
	xCheckDQP( rpcTempCU );
#endif 
	xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);


}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: INTRA Prediction  [DEVELOPING] : This function will be divided into Prediction and RD part
	@return	Void
	@author: Jinwuk Seok  2015 9 14 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_INTRAPrediction( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP)
{
	//------------------------------------------------------------------------
	//	Check Processing is Operated or Not
	//------------------------------------------------------------------------
	if (em_bSkipMode[ETRI_IdAxTempCU_Intra]){return;}

	Bool e_bSpeedUp = rpcBestCU->getSlice()->getSliceType() != I_SLICE;
	Bool e_bCBFSum	= (rpcBestCU->getCbf( 0, TEXT_LUMA ) + rpcBestCU->getCbf( 0, TEXT_CHROMA_U) + rpcBestCU->getCbf( 0, TEXT_CHROMA_V)) == 0;

	// avoid very complex intra if it is unlikely
	if ( e_bSpeedUp && e_bCBFSum) {return;}

	//------------------------------------------------------------------------
	//	INTRA Prediction
	//------------------------------------------------------------------------
	m_pcPredSearch->ETRI_IntraLumaPred_V2(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], 0);
	m_pcPredSearch->ETRI_IntraChromaPred	(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], true);
}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Normal INTRA Processing  [DEVELOPING] : This function will be divided into Prediction and RD part
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_Normal_INTRAProcess( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, Int iQP, Bool bIsLosslessMode)
{
	//------------------------------------------------------------------------
	//	Check Processing is Operated or Not
	//------------------------------------------------------------------------
	//Check if there is no best mode, 2015.10.06, yhee
	Bool e_bNoBestCU = ((rpcBestCU->getPartitionSize(0) == SIZE_NONE) | (rpcBestCU->getPredictionMode(0) == MODE_NONE) | (rpcBestCU->getTotalCost() == MAX_DOUBLE));
	if (em_bSkipMode[ETRI_IdAxTempCU_Intra] && !e_bNoBestCU){ return; }

#if ETRI_ADAPTIVE_MAXCTU_SIZE
	if (em_bControlParam[ETRI_IdAllModesSKIPPED]){return;}
#endif

	Bool e_bSpeedUp	= rpcBestCU->getSlice()->getSliceType() != I_SLICE;
	Bool e_bCBFSum 	= (rpcBestCU->getCbf( 0, TEXT_LUMA ) +  rpcBestCU->getCbf( 0, TEXT_CHROMA_U) + rpcBestCU->getCbf( 0, TEXT_CHROMA_V)) == 0;

	// avoid very complex intra if it is unlikely
	if (e_bSpeedUp && e_bCBFSum && !e_bNoBestCU)	{ return; }	

	//------------------------------------------------------------------------
	//	Intra RDOQ
	//------------------------------------------------------------------------
	ETRI_xCheckIntra_RDOQ(rpcBestCU, rpcTempCU, uiDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
	rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
#if ETRI_DISABLE_INTRA4x4_GPB
	if (rpcTempCU->getSlice()->getDepth() > 1 && (uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth))
#else 
	if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
#endif 
	{
		if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
		{
			xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_NxN	);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
			rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
		}
	}

	//------------------------------------------------------------------------
	//	Intra RDOQ Debug Info
	//------------------------------------------------------------------------

}
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: PU Block Procesing for Inter  [DEVELOPING][REMOVABLE] \
			do inter modes, NxN, 2NxN, Nx2N and AMP \
			This function can be removable, when optimization is processed  \
			Now, This function is controlled by em_bControlParam[ETRI_Id2NxNProcessing]	
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_InterPUBlockProcessing(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize, Bool& doNotBlockPu, Int iQP, Bool bIsLosslessMode)
{
	// do inter modes, NxN, 2NxN, and Nx2N
	if( rpcBestCU->getSlice()->getSliceType() != I_SLICE)
	{
		// 2Nx2N, NxN : When uiDepth == 6, this procedure is active, but this case is not occurred. 
		if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
		{
			if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
			{
				if (rpcTempCU->getSlice()->getPOC() == 8)
				EDPRINTF(stderr, "uiDepth: %d  g_uiMaxCUDepth: %d g_uiAddCUDepth : %d \n", uiDepth, g_uiMaxCUDepth, g_uiAddCUDepth);

				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN	);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#endif 
			}
		}

		ETRI_TryAMPProcess(rpcBestCU, rpcTempCU, uiDepth, eParentPartSize, doNotBlockPu, iQP, bIsLosslessMode); /// 2015 7 31 by Seok : Removable : ETRI_REMOVE_AMP_RELATED_CODE = 1
	}

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Test PCM Processing  [DEVELOPING][REMOVABLE] : This Function is not active \
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_TestPCMProcess( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComPic* pcPic, UInt uiDepth, Int iQP, Bool bIsLosslessMode)
{
	// test PCM
	if(pcPic->getSlice(0)->getSPS()->getUsePCM()
	&& rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
	&& rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
	{
		UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * rpcBestCU->getWidth(0) * rpcBestCU->getHeight(0) / 2;
		UInt uiBestBits = rpcBestCU->getTotalBits();
		if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
		{
			xCheckIntraPCM (rpcBestCU, rpcTempCU);
			rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
		}
	}
}

// ====================================================================================================================
//	ETRI Fast PU Processing 
// ====================================================================================================================

#define	ETRI_CUDepthProcessing 
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: To Control the compressing of sub-CU  [DEVELOPING]: This Function is very important \
			Fast CU Depth Decision such as CU Prunning, Top Cut of CU Depth, Bottom Cut of Cu Depth and others \
	@param	Bool& bSubBranch : Real Output Param. [TRUE] Operation Sub CU [FALSE] SKIP Sub CU 
	@return	Void
	@author: Jinwuk Seok  2015 7 31 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_SubCUControl( TComDataCU*& rpcBestCU, Bool& bSubBranch, Bool bInsidePicture, UInt uiDepth, Bool bBoundary)
{
	Bool 	e_bTopDepth = false; 
	Double 	e_dRatio = 0.0;
	TComSlice* pcSlice = rpcBestCU->getSlice();

	/// 2015 11 30 by Seok : For Debug
	FILE* dbgType = ETRI_ANALYSIS_TYPE;
	//------------------------------------------------------------------------------------------
	//	Summarize of Control Parameter
	//------------------------------------------------------------------------------------------
	em_bLCUSkipFlag = false; 	///[FALSE: Original] LCU  No SKIP and Here This value is always false: 

	//==========================================================================
	//	Sub CU Control Operation : No operation when bBoundary = TRUE
	//==========================================================================
	if (bBoundary){return;}

#if ETRI_BUGFIX_ForcedSKIP
	Bool	e_bLocalSubBranch = false; 
	e_bLocalSubBranch = bSubBranch;
#else
	Bool	e_bLocalSubBranch = true;
#endif
	//------------------------------------------------------------------------------------------
	//	Fast 64x64 
	//------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------
	//	Fast Top Cut :: Statistical Method 
	//	[1] 		RD-Cost Based Method : 
	//	[2]		
	//	[2-1] Homogeneous   : 평균과의 오차가 1% 이내 (and No Problem to MVCLip)
	//	[2-2] Heterogeneous  : 평균보다 10% 이상 작은 경우 : e_iDiffDistortion= e_iAvrDistortion-(Int)rpcBestCU->getTotalDistortion() > 0
	//	[3]  		Fast CU Depth at Inter MODE with NoQT
	//------------------------------------------------------------------------------------------
#if ETRI_TEST_1201
	if (uiDepth > 0 && pcSlice->getSliceType() != I_SLICE)
	{


		e_bTopDepth = (pcSlice->getDepth() <= ETRI_F64SLiceLevel)? (uiDepth == 1):(uiDepth == 0);
		if (!e_bTopDepth)		
		{
			Double	e_dCurrentRDCost = rpcBestCU->getTotalCost();
			Double	e_dAverageRDCost = em_pcBestUpCU[uiDepth]->getTotalCost()/4.0;
			Double 	e_dDiffRDCost = e_dAverageRDCost - e_dCurrentRDCost;
			e_dRatio = (100.0 * e_dDiffRDCost)/e_dAverageRDCost;

			//Int	e_iFCutTh = 10;
			Int	e_iHomTh = 4;
			//Int	e_iHetTh = 15;

			Int	e_iDiffRDcost = (Int)e_dDiffRDCost;
			Int 	e_iABSRatio = ETRI_sABS((Int)e_dRatio);
			
			if (e_iDiffRDcost < 0)
			{
#if 0
				if ((em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTER && rpcBestCU->getPredictionMode(0) == MODE_INTER) 
					||(em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTRA && rpcBestCU->getPredictionMode(0) == MODE_INTRA))
				{
					e_bLocalSubBranch &= !(e_iABSRatio <= e_iFCutTh);
				}
				else
				{
					#if 0	
					FINDDBGLOCATION(ETRI_ANALYSIS_TYPE, rpcBestCU, uiDepth);
					EDPRINTF(ETRI_ANALYSIS_TYPE, " ===== DEBUG for SUbbranch ===== \n");
					#endif
				}
#endif
			}
			else
			{
				if ((em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTER && rpcBestCU->getPredictionMode(0) == MODE_INTER) 
					||(em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTRA && rpcBestCU->getPredictionMode(0) == MODE_INTRA))
				{
					/// [2-1] Homogeneous Case @ 2015 11 30 by Seok
					e_bLocalSubBranch &= !(e_iABSRatio <= e_iHomTh);
				}
				else
				{
					/// [2-2]  Heterogeneous Case @ 2015 11 30 by Seok
				//	e_bLocalSubBranch &= !(e_iABSRatio >= e_iHetTh);
				}
			}

			//------------------------------------------------------------------------------------------
			//	Debug Information 
			//------------------------------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
			if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
			{
				FILE*	_dbgType = ETRI_ANALYSIS_TYPE;
				EDPRINTF(_dbgType, "[SubBranch :Top Cutting] \n");
				EDPRINTF(_dbgType, "e_dCurrentRDCost : %.2f \n", e_dCurrentRDCost );
				EDPRINTF(_dbgType, "e_dAverageRDCost : %.2f \n", e_dAverageRDCost );
				EDPRINTF(_dbgType, "e_dDiffRDCost    : %.2f \n", e_dDiffRDCost);
				EDPRINTF(_dbgType, "e_dRatio         : %.2f \n", e_dRatio);
				EDPRINTF(_dbgType, "e_bLocalSubBranch: %s \n", GetBoolVal(e_bLocalSubBranch));

				if ((em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTER && rpcBestCU->getPredictionMode(0) == MODE_INTER) 
					||(em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_INTRA && rpcBestCU->getPredictionMode(0) == MODE_INTRA))
				{
					EDPRINTF(_dbgType, "Homogeneous Case \n");
				}
				else
				{
					EDPRINTF(_dbgType, "Heterogeneous Case \n");
				}
			}
#endif 
			//------------------------------------------------------------------------------------------


		}
	}
#endif


#if ETRI_TEST_1202
	//------------------------------------------------------------------------------------------
	//	[3] Very Good !!!
	//------------------------------------------------------------------------------------------
	if (pcSlice->getSliceType() != I_SLICE)
	{
		if (rpcBestCU->getPredictionMode(0) == MODE_INTER && rpcBestCU->getQtRootCbf(0) == 0)
		{
			e_bLocalSubBranch &= false;
		}
	}
#endif

#if ETRI_TEST_1204
	e_bTopDepth = (pcSlice->getDepth() <= ETRI_F64SLiceLevel)? (uiDepth == 1):(uiDepth == 0);
	if (e_bTopDepth)
	e_bLocalSubBranch &= false;	
#endif

	//------------------------------------------------------------------------------------------
	//	Test 
	//	[1] CU Prunning 
	//------------------------------------------------------------------------------------------
	//UInt e_uiAveragePartDistortion = 0, e_uiExpectedTotalDistortion = 0; 
#if 0
	TComSlice*	pcSlice = rpcBestCU->getSlice();

//	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	if (pcSlice->getSliceType() != I_SLICE)
	{
		//------------------------------------------------------------------------------------------
		//	CU Top Cutting with RD Cost Prediction 
		//------------------------------------------------------------------------------------------
//		e_bTopDepth = (em_pcBestUpCU)? (em_pcBestUpCU[uiDepth]->getPredictionMode(0) == MODE_NONE && uiDepth == 1) : (uiDepth == 0);
		if (pcSlice->getDepth() == 0)
		{
			e_bTopDepth = (uiDepth == 1);
		}
		else
		{
			e_bTopDepth = (uiDepth == 0);
		}

		if (em_pcBestUpCU[uiDepth] && !e_bTopDepth)
		{
			e_uiAveragePartDistortion 	= (em_pcBestUpCU[uiDepth]->getTotalDistortion() + 2)>>2;
			e_uiExpectedTotalDistortion	= em_uiACCPartDistortion[uiDepth] + ((em_uiPartUnitIdx[uiDepth] < 3)? (e_uiAveragePartDistortion * ( 2 - em_uiPartUnitIdx[uiDepth])) : 0);

			//------------------------------------------------------------------------------------------
			//	Top Depth의 경우 Expected Total Distortion 과 UPDepthCU의 Distortion이 같다.
			//	그러므로, 이 경우에는 SubBranch 가 1이 되도록 하는 것이 맞다.
			//------------------------------------------------------------------------------------------		
			e_bLocalSubBranch &= ((UInt)(e_uiExpectedTotalDistortion * 0.9) <= em_pcBestUpCU[uiDepth]->getTotalDistortion());

			//------------------------------------------------------------------------------------------
			//	Strong Prunning : 여기에서 만일 ACC cost 가 Up Depth 의 RD Cost를 넘어서면 Part 연산도 그만하고 Subbranch 도 포기한다.
			//	즉, Subbranch 연산을 통해 RD cost가 더 작아질 수 있는 가능성을 포기한다.
			//	무의미한 Prunning 
			//		[1] Depth = 3, partidx = 3 : prunning/subbranch 모두 무의미 (partidx : em_uiPartUnitIdx[uiDepth] )
			//		[2] Depth < 3, partidx = 3 : prunning 무의미 / subbranch 유의미 
			//	Weak Prunning : 이곳이 아닌 SubCU Processing이 끝난 후에 하는 Prunning 속도 상승 폭이 매우 작다. 그러나 Lossless
			//------------------------------------------------------------------------------------------		
			em_bControlParam[ETRI_IdCUPrunningON] = em_dACCPartRDCost[uiDepth] > em_pcBestUpCU[uiDepth]->getTotalCost();
			e_bLocalSubBranch &= !em_bControlParam[ETRI_IdCUPrunningON];
		}
	}
#endif

	//------------------------------------------------------------------------------------------
	//	Forced Subbranch : Highest Privilage 
	//	[1] ALL CU Modes are SKIPPED 
	//	[2] 
	//------------------------------------------------------------------------------------------
#if ETRI_BUGFIX_ForcedSKIP
	e_bLocalSubBranch |= em_bControlParam[ETRI_IdAllModesSKIPPED];
	rpcBestCU->ETRI_setAllModesSkip(em_bControlParam[ETRI_IdAllModesSKIPPED]);

	//==========================================================================
	//	Result of Sub CU Control (Final Stage)
	//==========================================================================
	bSubBranch = e_bLocalSubBranch;
#else
	//==========================================================================
	//	Result of Sub CU Control (Final Stage)
	//==========================================================================
	bSubBranch &= e_bLocalSubBranch;
#endif

	//==========================================================================
	//	Debug Information
	//==========================================================================
	//------------------------------------------------------------------------------------------
	//	[1] ALL CU Modes are SKIPPED && FFESD (OK : No Conflicts)
	//	[2] ALL CU Modes are SKIPPED && bSubbranch (OK : No Conflicts)
	//------------------------------------------------------------------------------------------
#if !ETRI_CODE_FURTHER_CLEANUP
	if ( e_dRatio < 1.0 && bSubBranch)
	{
		//EDPRINTF(dbgType, "Distortion Distribution & Subbranch Conflicts\n");
	}
#endif 
#if ETRI_DEBUG_CODE_CLEANUP
	em_DbgInfo[ETRI_nDBGInfo_CASE02] += em_bControlParam[ETRI_IdAllModesSKIPPED];
	em_DbgInfo[ETRI_nDBGInfo_CASE03] += em_bControlParam[ETRI_IdEarlyMerge];

	//------------------------------------------------------------------------------------------
	//	[0] General Debug Information 
	//------------------------------------------------------------------------------------------
	if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	{
		EDPRINTF(dbgType, "[SubBranch Information]\n");
		EDPRINTF(dbgType, "bSubBranch : %s \n", GetBoolVal(bSubBranch));
		EDPRINTF(dbgType, "e_dRatio : %.2f \n", e_dRatio);
		EDPRINTF(dbgType, "em_bControlParam[ETRI_IdAllModesSKIPPED] : %s \n", GetBoolVal(em_bControlParam[ETRI_IdAllModesSKIPPED]));
		EDPRINTF(dbgType, "rpcBestCU->ETRI_getAllModesSkip : %s \n", GetBoolVal(rpcBestCU->ETRI_getAllModesSkip()));

		if (em_pcBestUpCU[uiDepth] && !e_bTopDepth)
		{
			EDPRINTF(dbgType, "em_dACCPartRDCost[%d] : %.2f \n", uiDepth, em_dACCPartRDCost[uiDepth]);
			EDPRINTF(dbgType, "em_pcBestUpCU[%d]->getTotalCost(): %.2f \n", uiDepth, em_pcBestUpCU[uiDepth]->getTotalCost());
			
			if (em_bControlParam[ETRI_IdCUPrunningON])
				EDPRINTF(dbgType, "em_bControlParam[ETRI_IdCUPrunningON] : %s \n", GetBoolVal(em_bControlParam[ETRI_IdCUPrunningON]));
		}
	}	
#endif 

}

// ====================================================================================================================
// ETRI Main Compress CU Function
// ====================================================================================================================
/** 
------------------------------------------------------------------------------------------------------------------------------------------------
@brief  	Compress a CU block recursively with enabling sub-LCU-level delta QP \
		Fixed PreProcessoers : 	ETRI_IPCM_OPTIMIZATION = 1 \
							AMP_MRG = 1	\
							AMP_ENC_SPEEDUP = 1 
@param  eParentPartSize : This value is used when AMP_MRG=1.  
@returns Void

 *- for loop of QP value to compress the current CU with all possible QP
 @Author Jinwuk Seok 2015 0731
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncCu::ETRI_xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize)
{
	//if ((ETRI_RETURN_ORIGINAL || ETRI_MODIFICATION_V00 || ETRI_MODIFICATION_V01) && !ETRI_MODIFICATION_V02){xCompressCU(rpcBestCU, rpcTempCU, uiDepth); return;}
	if (ETRI_RETURN_ORIGINAL){ xCompressCU(rpcBestCU, rpcTempCU, uiDepth); return; }

	TComPic* pcPic = rpcBestCU->getPic();

	// get Original YUV data from picture
	m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );

	// variable for Early CU determination
	Bool	bSubBranch = true;

	// variable for Cbf fast mode PU decision
	Bool	doNotBlockPu = true; 	em_pbdoNotBlockPU = &doNotBlockPu;
	Bool	earlyDetectionSkipMode = false;

	Bool	bBoundary = false;
	UInt	uiLPelX   = rpcBestCU->getCUPelX();
	UInt	uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
	UInt	uiTPelY   = rpcBestCU->getCUPelY();
	UInt	uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;

	Int 	iBaseQP = xComputeQP( rpcBestCU, uiDepth );
	Int  	iMinQP;
	Int  	iMaxQP;
	Bool	isAddLowestQP = false;

	const Int lowestQP = ETRI_SetCUQP(rpcTempCU, iMinQP, iMaxQP, isAddLowestQP, iBaseQP, uiDepth, true);

	// If slice start or slice end is within this cu...
	TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
	Bool	bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
	Bool	bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
	Bool	bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );

	// Initilization of parameters tp Fast Algorithms for all CU Depth
	ETRI_Init_CUPartitionLevel(rpcTempCU, uiDepth);
	//================ Debug Point ===================
#if ETRI_DEBUG_CODE_CLEANUP
	UInt SubDbgPhase = 0;		///< 2015 2 27 by Seok : For Debug
	ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, -1, SubDbgPhase, ETRI_ANALYSIS_ON);
#endif 
	//if (em_DbgInfo[ETRI_nDBGInfo_ChkPoint])
	//EDPRINTF(stderr, "BEGIN DEBUG \n");	
	//================ Debug Point ===================

	// We need to split, so don't try these modes.
	if(!bSliceEnd && !bSliceStart && bInsidePicture && !em_bLCUSkipFlag)
	{
		for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)		/// 2015 8 1 by Seok :  But iMinQP == iMaxQP
		{
			const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
			iQP = (bIsLosslessMode)? lowestQP : iQP;	/// Check and Remove : 2015 8 1 by Seok

#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
			rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );	///< Init CU For SKIP/Merge when P/B-Slice and For INTRA 2Nx2N when I-Slice @2015 8 30 by Seok
#endif 
            ETRI_Normal_INTERProcess( rpcBestCU, rpcTempCU, uiDepth, iQP, bIsLosslessMode, earlyDetectionSkipMode, doNotBlockPu);

			iQP = (bIsLosslessMode)? iMinQP : iQP;
		}

		if(!earlyDetectionSkipMode)
		{
			for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
			{
				const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
				iQP = (bIsLosslessMode)? lowestQP : iQP;

				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#if !ETRI_CODE_FURTHER_CLEANUP
				ETRI_InterPUBlockProcessing(rpcBestCU, rpcTempCU, uiDepth, eParentPartSize, doNotBlockPu, iQP, bIsLosslessMode);	/// 2015 8 1 by Seok : DEVELOPING or REMOVABLE
#endif 
                ETRI_Normal_INTRAProcess(rpcBestCU, rpcTempCU, uiDepth, iQP, bIsLosslessMode);
				ETRI_TestPCMProcess(rpcBestCU, rpcTempCU, pcPic, uiDepth, iQP, bIsLosslessMode);	/// 2015 8 1 by Seok : Developing or Removable

				iQP = (bIsLosslessMode)? iMinQP : iQP;
			}
		}

		//--------------------------------------------------------------------------------
		//	Debug Info : For INTRA
		//--------------------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
		ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_Intra, ETRI_RDOQ, ETRI_ANALYSIS_ON);
#endif 
		//--------------------------------------------------------------------------------
		//	Post Processing : Set rpcBestCU for Stability
		//--------------------------------------------------------------------------------
#if ETRI_ADAPTIVE_MAXCTU_SIZE
#if ETRI_ADAPTIVE_CTU_SIZE_BUGFIX
		if (!(uiDepth == 0 && em_bControlParam[ETRI_IdxMAXCTUSIZE] && rpcBestCU->getPredictionMode(0) == (PredMode)SIZE_NONE))
#else 
		if (!(uiDepth == 0 && em_bControlParam[ETRI_IdxMAXCTUSIZE] && rpcBestCU->getPredictionMode(uiDepth) == (PredMode)SIZE_NONE))
#endif 
		{
			m_pcEntropyCoder->resetBits();
			m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
			rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
			rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
			rpcBestCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion());
		}
		else
		{
			m_pcEntropyCoder->resetBits();
			m_pcEntropyCoder->encodeSplitFlag(rpcBestCU, 0, uiDepth, true);
			rpcBestCU->getTotalBits() = 0;
			rpcBestCU->getTotalBins() = 0;
			rpcBestCU->getTotalCost() = MAX_DOUBLE;

		}
#else 
		m_pcEntropyCoder->resetBits();
		m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
		rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
		rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
		rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
#endif 
		// Early CU determination :: ECU : TRUE
#if ETRI_REVISE_ECU
#if ETRI_REVISE_ECU_INTRA_CHECKING
        bSubBranch = !(( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->getCbf( 0, TEXT_LUMA ) == 0 && rpcBestCU->getPredictionMode(0) != (PredMode)SIZE_NONE )||
            (rpcBestCU->getPredictionMode(0) == MODE_INTRA && rpcBestCU->getSlice()->getDepth()==0 && rpcBestCU->getSlice()->getSliceType()!=I_SLICE && uiDepth==2)); 
#else 
		bSubBranch = !( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->getCbf( 0, TEXT_LUMA ) == 0 && rpcBestCU->getPredictionMode(0) != (PredMode)SIZE_NONE ); 
#endif 
#else 
		bSubBranch = !( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) ); 
#endif 
#if ETRI_ADAPTIVE_MINCTU_SIZE
#if ETRI_ADAPTIVE_CTU_SIZE_BUGFIX
		if (rpcBestCU->getSlice()->getDepth() > 1 && (rpcBestCU->getWidth(0) == 16 && uiDepth == 2))
#else 
		if (rpcBestCU->getSlice()->getDepth() > 1 && (rpcBestCU->getWidth(uiDepth) == 16 && uiDepth == 2))
#endif 
			bSubBranch = false;
#endif 
#if ETRI_CU_INTRA_MODE_INHERITANCE
		if (uiDepth == 1 && rpcBestCU->getPredictionMode(0) == MODE_INTRA && bSubBranch == true)
		{
			em_bDoInterModeFlag = false;
		}
#endif 

	}
	else if(!(bSliceEnd && bInsidePicture))
	{
		bBoundary = true;
	}

	//--------------------------------------------------------------------------------
	//	Auxiliary Code : Follwing Code should be inserted in the ETRI_SubCUControl
	//--------------------------------------------------------------------------------
#if !ETRI_CODE_FURTHER_CLEANUP
	em_dACCPartRDCost[uiDepth]  	+= rpcBestCU->getTotalCost();
	em_uiACCPartDistortion[uiDepth] 	+= rpcBestCU->getTotalDistortion();
#endif 
	//--------------------------------------------------------------------------------
	//	Debug Info 
	//--------------------------------------------------------------------------------
#if ETRI_DEBUG_CODE_CLEANUP
	ETRI_PrintDebugnfo(WHEREARG, rpcTempCU, rpcBestCU, uiDepth, ETRI_IdAxTempCU_BESTRD, ETRI_PRED, ETRI_ANALYSIS_ON);

	em_DbgInfo[ETRI_nDBGInfo_CASE01] += (em_bSkipMode[ETRI_IdAxTempCU_Merge] \
										+ em_bSkipMode[ETRI_IdAxTempCU_Inter] \
										+ em_bSkipMode[ETRI_IdAxTempCU_Intra] \
										+ em_bSkipMode[ETRI_IdAxTempCU_Skip]) == 3;
#endif 
	//--------------------------------------------------------------------------------
	//	SubCU Processing 
	//--------------------------------------------------------------------------------
	ETRI_SubCUControl(rpcBestCU, bSubBranch, bInsidePicture, uiDepth, bBoundary);
#if !ETRI_CODE_FURTHER_CLEANUP
	ETRI_SetCUQP(rpcTempCU, iMinQP, iMaxQP, isAddLowestQP, iBaseQP, uiDepth, false);	/// 2015 8 1 by Seok : REMOVABLE 
#endif 
	for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
	{
		const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

		// further split
		if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
		{
			UChar       uhNextDepth         = uiDepth+1;

			ETRI_SubCUProcessing_inPicture(rpcBestCU, rpcTempCU, pcSlice, bBoundary, uhNextDepth, iQP, uiDepth);
			ETRI_RDcostEvaluationforCU(rpcBestCU, rpcTempCU, pcSlice, pcPic, uiDepth, true);
			ETRI_CUBestModeDecision(rpcBestCU, rpcTempCU, uhNextDepth, iQP, uiDepth, true);
		}                                                                                  
	}

	//--------------------------------------------------------------------------------
	//	Final Processing 
	//--------------------------------------------------------------------------------
	rpcBestCU->copyToPic(uiDepth); 	// Copy Best data to Picture for next partition prediction.                                                     

	// Copy Yuv data to picture Yuv
	xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   
	if( bBoundary ||(bSliceEnd && bInsidePicture)){return;}


	//Check Error Location @ 2015 9 6 by Seok
#if !ETRI_CODE_FURTHER_CLEANUP
	if (( rpcBestCU->getPartitionSize ( 0 ) == SIZE_NONE  ) || ( rpcBestCU->getPredictionMode( 0 ) == MODE_NONE  ) || ( rpcBestCU->getTotalCost     (   ) == MAX_DOUBLE ))
	{
		EDPRINTF(stderr, "================ ERROR Location ==================\n");
		EDPRINTF(stderr, "POC: %d  CU Address: %d  Depth: %d ID : %d (x,y): (%4d, %4d) \n", rpcBestCU->getSlice()->getPOC(), 
						(Int)rpcBestCU->getAddr(), (Int)uiDepth, (Int)rpcBestCU->getZorderIdxInCU(), (Int)rpcBestCU->getCUPelX(), (Int)rpcBestCU->getCUPelY());
	}
#endif 
	// Assert if Best prediction mode is NONE
	// Selected mode's RD-cost must be not MAX_DOUBLE.
	assert( rpcBestCU->getPartitionSize ( 0 ) != SIZE_NONE  );
	assert( rpcBestCU->getPredictionMode( 0 ) != MODE_NONE  );
	assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE );
}

// ====================================================================================================================
// ETRI SubCU Processing functions
//	Main Function  		:  	ETRI_SubCUProcessing_inPicture
//	
//	Auxiliary Function 	:	ETRI_CUPruningOnSameDepth
// ====================================================================================================================

#define	ETRI_CUPrunning
/** 
------------------------------------------------------------------------------------------------------------------------------------------------
@brief  	CU Prunning on Same Depth based on RD-Cost or Peudo RD Cost. Mainly this function is operated by Real RD-COst
@param	bOperation	If the structure of compress CU is changed, this function may not be operated. This parameter isready for such case.
@Author 	Jinwuk Seok 2015 0731
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncCu::ETRI_CUPruningOnSameDepth(TComDataCU*& rpcTempCU,  UInt uiPartUnitIdx, Bool& bOperation)
{
	// If Under some Condition, CU Prunning ON 
	if (em_bControlParam[ETRI_IdCUPrunningON])	
	{
		// Prunning ON 
		bOperation = true;

		// Stop Subbranching For Upper CU 
		rpcTempCU->getTotalDistortion()  = MAX_INT;
		
		// Restore Prunning Parameter
		em_bControlParam[ETRI_IdCUPrunningON] = false;
	}

	// When No Operation, No CU Prunning Occur
}

/** 
------------------------------------------------------------------------------------------------------------------------------------------------
@brief  	Compress a sub-CU-Level with delta QP; AMP_ENC_SPEEDUP is fixed 1
@Author Jinwuk Seok 2015 0731
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncCu::ETRI_SubCUProcessing_inPicture( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComSlice * pcSlice, 
												 	Bool bBoundary, UInt uhNextDepth, Int iQP, UInt uiDepth)
{
	TComDataCU* pcSubBestPartCU 	= m_ppcBestCU[uhNextDepth];
	TComDataCU* pcSubTempPartCU 	= m_ppcTempCU[uhNextDepth];

	ETRI_Init_SubCULevel(rpcBestCU, uhNextDepth);

	for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
	{

#if ETRI_CU_MODE_INHERITANCE
		if (uiDepth == 0)
		{
			em_bDoInterModeFlag = true;
		}
#endif
		pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
		pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

		ETRI_Init_SubCUPartitionLevel(rpcBestCU, uiPartUnitIdx, iQP, uhNextDepth, uiDepth);

		Bool bInSlice = (pcSubBestPartCU->getSCUAddr() + pcSubBestPartCU->getTotalNumPart()) > pcSlice->getSliceSegmentCurStartCUAddr()
			 		&& pcSubBestPartCU->getSCUAddr() < pcSlice->getSliceSegmentCurEndCUAddr();

		if(bInSlice \
			&& ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() )  \
			&& ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
		{
			Bool	e_bPartIdx = uiPartUnitIdx > 0;
			m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth + e_bPartIdx][CI_CURR_BEST + e_bPartIdx]);	/// CI_NEXT_BEST = CI_CURR_BEST + 1 @ 2015 7 31 by Seok

			//--------------------------------------------------------------------------------
			//  When Oprimization is going on, compare the followig codes to  
			//  ETRI_xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
			//  Jinwuk Seok 2015 07 31
			//--------------------------------------------------------------------------------
			ETRI_xCompressCU(pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, (PartSize)((rpcBestCU->isIntra(0))? SIZE_NONE : rpcBestCU->getPartitionSize(0)));

			rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
			xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
		}
#if !ETRI_DEV_0731	// In V12 the following code is inactive. plz check
		else if (bInSlice)
		{
			pcSubBestPartCU->copyToPic( uhNextDepth );
			rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
		}
#endif
		//--------------------------------------------------------------------------------
		//	Fast Algorithm : CU Prunning is here
		//--------------------------------------------------------------------------------
		Bool e_bCUPrunning = false;
		ETRI_CUPruningOnSameDepth(rpcTempCU, uiPartUnitIdx, e_bCUPrunning);	
		if (e_bCUPrunning){break;}

	}
	

	if( !bBoundary )
	{
		m_pcEntropyCoder->resetBits();
		m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );

		rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
		rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
	}
	rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

}

/** 
------------------------------------------------------------------------------------------------------------------------------------------------
@brief  	Evaluation of RD-cost for a sub-CU. This Function is almost unchangable.
@param	bOperation	If the structure of compress CU is changed, this function may not be operated. This parameter isready for such case.
@Author Jinwuk Seok 2015 0731
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_RDcostEvaluationforCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, TComSlice * pcSlice, TComPic* pcPic, UInt uiDepth, Bool bOperation)
{
	if (!bOperation){return;}

	if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
	{
		Bool hasResidual = false;
		for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
		{
#if ETRI_SLICE_SEGMENT_OPTIMIZATION
			if ((rpcTempCU->getCbf(uiBlkIdx, TEXT_LUMA) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_U) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_V)))
#else
			if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) && 
			( rpcTempCU->getCbf( uiBlkIdx, TEXT_LUMA ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_U ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_V ) ) )
#endif
			{
				hasResidual = true;
				break;
			}
		}

		UInt uiTargetPartIdx;
#if !ETRI_SLICE_SEGMENT_OPTIMIZATION
		if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
		{
		uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
		}
		else
#endif
		{
			uiTargetPartIdx = 0;
		}

		if ( hasResidual )
		{
#if !RDO_WITHOUT_DQP_BITS
			m_pcEntropyCoder->resetBits();
			m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
			rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
			rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
			rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif

			Bool foundNonZeroCbf = false;
			rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
			assert( foundNonZeroCbf );
		}
		else
		{
			rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
		}
	}

}


/** 
------------------------------------------------------------------------------------------------------------------------------------------------
@brief  	Best Mode Decision for a sub-CU. This Function is almost unchangable.
@param	bOperation	If the structure of compress CU is changed, this function may not be operated. This parameter isready for such case.
@Author Jinwuk Seok 2015 0731
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline void TEncCu::ETRI_CUBestModeDecision(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uhNextDepth, Int iQP, UInt uiDepth, Bool bOperation)
{
	if (!bOperation){return;}

	m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

	Bool isEndOfSlice		 = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
	&& (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
	Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
	&& (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
	if(isEndOfSlice||isEndOfSliceSegment)
	{
		rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
	}
	xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth); 								 // RD compare current larger prediction
}

// ====================================================================================================================
//   END of ETRI  Modification for E265 Version 2 and 3 
// ====================================================================================================================
#else
// ====================================================================================================================
//  When ETRI_MODIFICATION_V02 = 0 or undefined,  ETRI_xCompressCU call the HM Original xCompressCU
// ====================================================================================================================
Void TEncCu::ETRI_xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize)
{
	xCompressCU(rpcBestCU, rpcTempCU, uiDepth); 
}
#endif	/// End of #if ETRI_MODIFICATION_V02


/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  UInt uiCUAddr = pcCU->getSCUAddr()+uiAbsPartIdx;

  UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
  UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
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
  if(uiInternalAddress==pcCU->getPic()->getNumPartInCU())
  {
    uiInternalAddress = 0;
    uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
  }
  UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);

  // Encode slice finish
  Bool bTerminateSlice = false;
  if (uiCUAddr+(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)) == uiRealEndAddress)
  {
    bTerminateSlice = true;
  }
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Bool granularityBoundary=((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight));
  
  if(granularityBoundary)
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    if (!bTerminateSlice)
      m_pcEntropyCoder->encodeTerminatingBit( bTerminateSlice ? 1 : 0 );
  }
  
  Int numberOfWrittenBits = 0;
  if (m_pcBitCounter)
  {
    numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  
  // Calculate slice end IF this CU puts us over slice bit size.
  UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
  Int iGranularityEnd = ((pcCU->getSCUAddr()+uiAbsPartIdx)/iGranularitySize)*iGranularitySize;
  if(iGranularityEnd<=pcSlice->getSliceSegmentCurStartCUAddr()) 
  {
    iGranularityEnd+=max(iGranularitySize,(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)));
  }
  // Set slice end parameter
  if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceBits()+numberOfWrittenBits>pcSlice->getSliceArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
    return;
  }
  // Set dependent slice end parameter
  if(pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceSegmentBits()+numberOfWrittenBits > pcSlice->getSliceSegmentArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    return;
  }
  if(granularityBoundary)
  {
    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);
    if (m_pcBitCounter)
    {
      m_pcEntropyCoder->resetBits();      
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }
  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  // If slice start is within this cu...
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
  // We need to split, so don't try these modes.
  if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else
  {
    bBoundary = true;
  }
  
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
      setdQPFlag(true);
    }
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
      Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );
      }
    }
    return;
  }
  
  if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
  {
    setdQPFlag(true);
  }
#if !ETRI_LOSSLESS_OPTIMIZATION
  if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
  }
#endif
  if( !pcCU->getSlice()->isIntra() )
  {
    m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );
  }
  
  if( pcCU->isSkipped( uiAbsPartIdx ) )
  {
    m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );
    finishCU(pcCU,uiAbsPartIdx,uiDepth);
    return;
  }
  m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );
  
  m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );

#if !ETRI_IPCM_OPTIMIZATION
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      // Encode slice finish
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
      return;
    }
  }
#endif

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );
  
  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, pcCU->getWidth (uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx), bCodeDQP );
  setdQPFlag( bCodeDQP );

  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx,uiDepth);
}

Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg) 
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;
 
    piOrg += iStrideOrg;
  }
  
  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];
    
    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];
    
    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }
  
  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];
    
    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];
    
    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }
  
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height)
{
  Int  xBl, yBl; 
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pcCU->getPic()->getPicYuvOrg()->getLumaAddr(pcCU->getAddr(), 0);
  Int  iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride();
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl; 
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode )
{
	assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );
	TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
	UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
	Int numValidMergeCand = 0;

	const Bool bTransquantBypassFlag = (!ETRI_LOSSLESS_OPTIMIZATION)? rpcTempCU->getCUTransquantBypass(0) : false;

	for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
	{
		uhInterDirNeighbours[ui] = 0;
	}
	UChar uhDepth = rpcTempCU->getDepth( 0 );
	rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
	rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );

	Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
	for( UInt ui = 0; ui < numValidMergeCand; ++ui )
	{
		mergeCandBuffer[ui] = 0;
	}

	Bool bestIsSkip = false;
	UInt iteration;
	iteration = 2 -rpcTempCU->isLosslessCoded(0);

	for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
	{
		for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
		{
			if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
			{
				if( !(bestIsSkip && uiNoResidual == 0) )
				{
					// set MC parameters
					rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
#if !ETRI_SET_FUNCTION_OPTIMIZATION
					rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag,     0, uhDepth );
#endif
					rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
					rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
					rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
					rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
					rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
					rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

					// do MC
					m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );
					// estimate residual and encode everything
					m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
					m_ppcOrigYuv    [uhDepth],
					m_ppcPredYuvTemp[uhDepth],
					m_ppcResiYuvTemp[uhDepth],
					m_ppcResiYuvBest[uhDepth],
					m_ppcRecoYuvTemp[uhDepth],
					(uiNoResidual? true:false));


					if ( uiNoResidual == 0 && rpcTempCU->getQtRootCbf(0) == 0 )
					{
						// If no residual when allowing for one, then set mark to not try case where residual is forced to 0
						mergeCandBuffer[uiMergeCand] = 1;
					}

					rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
					Int orgQP = rpcTempCU->getQP( 0 );
#if !ETRI_REMOVE_XCHECKDQP
					xCheckDQP( rpcTempCU );
#endif 
					xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
#if !ETRI_CU_INIT_FUNCTION_OPTIMIZATION
					rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );
#endif 
					if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
					{
						bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
					}
				}
			}
		}

		if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
		{
			if(rpcBestCU->getQtRootCbf( 0 ) == 0)
			{
				if( rpcBestCU->getMergeFlag( 0 ))
				{
					*earlyDetectionSkipMode = true;
				}
				else
				{
					Int absoulte_MV=0;
					for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
					{
						if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
						{
							TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
							Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
							Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
							absoulte_MV+=iHor+iVer;
						}
					}

					if(absoulte_MV == 0)
					{
						*earlyDetectionSkipMode = true;
					}
				}
			}
		}

	}
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
  UChar uhDepth = rpcTempCU->getDepth( 0 );

#if !ETRI_SET_FUNCTION_OPTIMIZATION
  rpcTempCU->setDepthSubParts( uhDepth, 0 );
  rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );
#endif
  rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
  
#if AMP_MRG
  rpcTempCU->setMergeAMP (true);
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], false, bUseMRG );
#else  
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif

#if AMP_MRG
  if ( !rpcTempCU->getMergeAMP() )
  {
	  EDPRINTF(stderr, "rpcTempCU->getMergeAMP() : %s \n", GetBoolVal(rpcTempCU->getMergeAMP()));
    return;
  }
#endif

  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false );

#if !ETRI_REDUNDANCY_OPTIMIZATION
  rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif

#if !ETRI_REMOVE_XCHECKDQP
  xCheckDQP( rpcTempCU );
#endif 
  xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
}

Void TEncCu::xCheckRDCostIntra( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize eSize )
{
  UInt uiDepth = rpcTempCU->getDepth( 0 );

#if !ETRI_SET_FUNCTION_OPTIMIZATION
  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );
#endif
  rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  
  Bool bSeparateLumaChroma = true; // choose estimation mode
  UInt uiPreCalcDistC      = 0;
  if( !bSeparateLumaChroma )
  {
    m_pcPredSearch->preestChromaPredMode( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth] );
  }
  
  m_pcPredSearch  ->estIntraPredQT      ( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC, bSeparateLumaChroma );

  m_ppcRecoYuvTemp[uiDepth]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
  
  m_pcPredSearch  ->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC );
  
  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
#if ETRI_RESIDUAL_BYPASS_MODE
  m_pcEntropyCoder->encodeCoeff(rpcTempCU, 0, uiDepth, rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), bCodeDQP, true);
#else 
  m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, rpcTempCU->getWidth (0), rpcTempCU->getHeight(0), bCodeDQP );
#endif 
  setdQPFlag( bCodeDQP );
  
  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
  
  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
  
#if !ETRI_REMOVE_XCHECKDQP
  xCheckDQP( rpcTempCU );
#endif 
  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);
}

/** Check R-D costs for a CU with PCM mode. 
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 * 
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU )
{
  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );

  rpcTempCU->setIPCMFlag(0, true);
  rpcTempCU->setIPCMFlagSubParts (true, 0, rpcTempCU->getDepth(0));
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setTrIdxSubParts ( 0, 0, uiDepth );

  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize ( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodeIPCMInfo ( rpcTempCU, 0, true );

  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
  xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth );
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
{
  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = rpcBestCU;
    rpcBestCU = rpcTempCU;
    rpcTempCU = pcCU;

    // Change Prediction data
    pcYuv = m_ppcPredYuvBest[uiDepth];
    m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    m_ppcPredYuvTemp[uiDepth] = pcYuv;

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBest[uiDepth];
    m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    m_ppcRecoYuvTemp[uiDepth] = pcYuv;

    pcYuv = NULL;
    pcCU  = NULL;

    // store temp best CI for next CU coding
    m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
  }
}

Void TEncCu::xCheckDQP( TComDataCU* pcCU )
{
  UInt uiDepth = pcCU->getDepth( 0 );

  if( pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    if ( pcCU->getCbf( 0, TEXT_LUMA, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_U, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_V, 0 ) )
    {
#if !RDO_WITHOUT_DQP_BITS
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQP( pcCU, 0, false );
      pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
      pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      pcCU->getTotalCost() = m_pcRdCost->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );
#endif
    }
    else
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
{
#if !ETRI_XCOPYYUV2PIC_CLEANUP
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
#endif 
  {
    UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
    UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
    UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
    UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;
    m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
  }
#if !ETRI_XCOPYYUV2PIC_CLEANUP
  else
  {
    UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
      UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

      Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && 
        rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
      }
    }
  }
#endif 
}

Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiNextDepth )
{
  UInt uiCurrDepth = uiNextDepth - 1;
  m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv( m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx );
}

/** Function for filling the PCM buffer of a CU using its original sample array 
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv )
{

  UInt   width        = pCU->getWidth(0);
  UInt   height       = pCU->getHeight(0);

  Pel*   pSrcY = pOrgYuv->getLumaAddr(0, width); 
  Pel*   pDstY = pCU->getPCMSampleY();
  UInt   srcStride = pOrgYuv->getStride();

  for(Int y = 0; y < height; y++ )
  {
    for(Int x = 0; x < width; x++ )
    {
      pDstY[x] = pSrcY[x];
    }
    pDstY += width;
    pSrcY += srcStride;
  }

  Pel* pSrcCb       = pOrgYuv->getCbAddr();
  Pel* pSrcCr       = pOrgYuv->getCrAddr();;

  Pel* pDstCb       = pCU->getPCMSampleCb();
  Pel* pDstCr       = pCU->getPCMSampleCr();;

  UInt srcStrideC = pOrgYuv->getCStride();
  UInt heightC   = height >> 1;
  UInt widthC    = width  >> 1;

  for(Int y = 0; y < heightC; y++ )
  {
    for(Int x = 0; x < widthC; x++ )
    {
      pDstCb[x] = pSrcCb[x];
      pDstCr[x] = pSrcCr[x];
    }
    pDstCb += widthC;
    pDstCr += widthC;
    pSrcCb += srcStrideC;
    pSrcCr += srcStrideC;
  }
}

// ====================================================================================================================
//  Collection of No Operated Functions 
//  Two Types of no operated Functions 
//     (1) Constructed in HM, but, by Configuration, this function is not operated 
//     (2) Unnecessary codes by ETRI's Optimization. 
// 
// ====================================================================================================================
// ====================================================================================================================
//  (1) No Operated Functions  in HM 
// ====================================================================================================================

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    Int u = abs( rpcCoeff[ n ] );
    Int absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else 
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU* rpcCU )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to datatype and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to datatype and quantization output

  TCoeff* pCoeffY = rpcCU->getCoeffY();
  Int* pArlCoeffY = rpcCU->getArlCoeffY();

  UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < rpcCU->getTotalNumPart(); i ++ )
  {
    UInt uiTrIdx = rpcCU->getTransformIdx(i);

    if(rpcCU->getPredictionMode(i) == MODE_INTER)
    if( rpcCU->getCbf( i, TEXT_LUMA, uiTrIdx ) )
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.
   
    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void 
*/
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{

  if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
  {
    bTestAMP_Hor = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
  {
    bTestAMP_Ver = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false )
  {
    bTestAMP_Hor = true;          
    bTestAMP_Ver = true;          
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU    
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_NONE ) //! if parent is intra
  {
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
      bTestMergeAMP_Hor = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;
    }
  }

  if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false )
  {
    bTestMergeAMP_Hor = true;          
    bTestMergeAMP_Ver = true;          
  }

  if ( rpcBestCU->getWidth(0) == 64 )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }    
#else
  //! Utilizing the partition size of parent PU        
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }      
#endif
}
#endif

#if (ETRI_RETURN_ORIGINAL || !ETRI_MODIFICATION_V02)
/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
  TComPic* pcPic = rpcBestCU->getPic();

  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );

  // variable for Early CU determination
  Bool    bSubBranch = true;

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool earlyDetectionSkipMode = false;

  Bool bBoundary = false;
  UInt uiLPelX   = rpcBestCU->getCUPelX();
  UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
  UInt uiTPelY   = rpcBestCU->getCUPelY();
  UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;

  if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  // transquant-bypass (TQB) processing loop variable initialisation ---

  const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

  if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
  {
    isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
    iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      iMaxQP = iMinQP;
    }
  }

  // If slice start or slice end is within this cu...
  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
  Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
  Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
  // We need to split, so don't try these modes.
  if(!bSliceEnd && !bSliceStart && bInsidePicture )
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
      const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

      if (bIsLosslessMode)
      {
        iQP = lowestQP;
      }

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU, &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (bIsLosslessMode)
      {
        iQP = iMinQP;
      }
    }

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

        if (bIsLosslessMode)
        {
          iQP = lowestQP;
        }
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN
          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
          {
            if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
            {
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }

          // 2NxN, Nx2N
          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

#if 1
          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
          {
#if AMP_ENC_SPEEDUP        
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor ) 
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
          }    
#endif
        }

        // do normal intra modes
        // speedup for inter frames
        if( rpcBestCU->getSlice()->getSliceType() == I_SLICE || 
          rpcBestCU->getCbf( 0, TEXT_LUMA     ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_U ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_V ) != 0     ) // avoid very complex intra if it is unlikely
        {
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
          {
            if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
            {
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }
        }

        // test PCM
        if(pcPic->getSlice(0)->getSPS()->getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
        {
          UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * rpcBestCU->getWidth(0) * rpcBestCU->getHeight(0) / 2;
          UInt uiBestBits = rpcBestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          }
        }
        if (bIsLosslessMode)
        {
          iQP = iMinQP;
        }
      }
    }

    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
    rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
    rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );

    // Early CU determination
    if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
    {
      bSubBranch = false;
    }
    else
    {
      bSubBranch = true;
    }
  }
  else if(!(bSliceEnd && bInsidePicture))
  {
    bBoundary = true;
  }

#if !ETRI_IPCM_OPTIMIZATION
  // copy orginal YUV samples to PCM buffer
  if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
  }
#endif

  if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
  }
  else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    Int iStartQP;
#if ETRI_SLICE_SEGMENT_OPTIMIZATION
	iStartQP = rpcTempCU->getQP(0);
#else
    if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
    {
      iStartQP = rpcTempCU->getQP(0);
    }
    else
    {
      UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
      iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
    }
#endif
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
  {
    iMaxQP = iMinQP; // If all blocks are forced into using transquant bypass, do not loop here.
  }

  for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
  {
    const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

    // further split
    if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      UChar       uhNextDepth         = uiDepth+1;
      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
        pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
          }

#if AMP_ENC_SPEEDUP
          if ( rpcBestCU->isIntra(0) )
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, SIZE_NONE );
          }
          else
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, rpcBestCU->getPartitionSize(0) );
          }
#else
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
        }
        else if (bInSlice)
        {
          pcSubBestPartCU->copyToPic( uhNextDepth );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
        }
      }

      if( !bBoundary )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );

        rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      }
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
        {
#if ETRI_SLICE_SEGMENT_OPTIMIZATION
			if ((rpcTempCU->getCbf(uiBlkIdx, TEXT_LUMA) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_U) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_V)))
#else
          if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) && 
              ( rpcTempCU->getCbf( uiBlkIdx, TEXT_LUMA ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_U ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_V ) ) )
#endif
          {
            hasResidual = true;
            break;
          }
        }

        UInt uiTargetPartIdx;
#if !ETRI_SLICE_SEGMENT_OPTIMIZATION
        if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
        {
          uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
        }
        else
#endif
        {
          uiTargetPartIdx = 0;
        }
        if ( hasResidual )
        {
#if !RDO_WITHOUT_DQP_BITS
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
          rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
          rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif

          Bool foundNonZeroCbf = false;
          rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else
        {
          rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
        }
      }

      m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth);                                  // RD compare current larger prediction
    }                                                                                  // with sub partitioned prediction.
  }

  rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.

  xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
  if( bBoundary ||(bSliceEnd && bInsidePicture))
  {
    return;
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( rpcBestCU->getPartitionSize ( 0 ) != SIZE_NONE  );
  assert( rpcBestCU->getPredictionMode( 0 ) != MODE_NONE  );
  assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE );
}
#else
/**
	@brief   : Null function when ETRI_RETURN_ORIGINAL is inactive to avoid compile or link error.
	@author: Jinwuk Seok 2015 0731
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{ }
#endif 	///< ETRI_RETURN_ORIGINAL : 2015 7 31 by Seok

// ====================================================================================================================
//  (2) Unnecessary codes by ETRI's Optimization. 
//		It is possible to be erased. 
// ====================================================================================================================

Void TEncCu::ETRI_TryAMPProcess(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize, Bool& doNotBlockPu, Int iQP, Bool bIsLosslessMode)
{
#if !ETRI_REMOVE_AMP_RELATED_CODE
	TComPic* pcPic = rpcBestCU->getPic();

	//! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
	if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
	{

#if AMP_ENC_SPEEDUP        
		Bool bTestAMP_Hor = false, bTestAMP_Ver = false;
#if AMP_MRG
		Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;
		deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
		deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif
		//! Do horizontal AMP
		if ( bTestAMP_Hor )
		{
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
		}
#if AMP_MRG
		else if ( bTestMergeAMP_Hor ) 
		{
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU, true );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD, true );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
		}
#endif

		//! Do horizontal AMP
		if ( bTestAMP_Ver )
		{
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
			}
		}
#if AMP_MRG
		else if ( bTestMergeAMP_Ver )
		{
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N, true );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
				if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
				{
					doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
				}
			}
			if(doNotBlockPu)
			{
				xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N, true );
				rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
			}
		}
#endif

#else
		xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
		xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
		xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

		xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
		rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
	}	 
#endif


}


//! \}
