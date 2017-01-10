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
	\file   	TEncTile.h
   	\brief    	Tile encoder class (header)
*/

#include "TEncTop.h"
#include "TEncTile.h"

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================
TEncTile::TEncTile()
{
	em_pcCfg 	= nullptr;		
	em_pcPic   	= nullptr;
	em_pcSlice 	= nullptr;

	em_pcTileEntropyCoder			= nullptr; 	///EntropyCoder. used.
	em_pcTileBitCounter 			= nullptr; 	///Bit counters per tile. used
	em_pppcTileRDSbacCoders 		= nullptr; 	///temporal storage for RD computation per tile. used : *[iTile][Depth][CI_IDX:Model]
	em_pcTileRDGoOnSbacCoder		= nullptr; 	///going on SBAC model for RD stage per tile. used
	em_pppcTileBinCodersCABAC   	= nullptr; 	///temporal CABAC state storage for RD computation per tile. used : *[iTile][Depth][CI_IDX:Model] : init m_ppppcTileRDSbacCoders wiht em_ppppcTileBinCodersCABAC 
	em_pcTileRDGoOnBinCodersCABAC	= nullptr; 	///going on bin coder CABAC for RD stage per tile 

	em_pcTileRdCost 				= nullptr; 	///RD cost computation class per tile. used
	em_pcTileTrQuant 				= nullptr; 	///transform & quantization class per tile. used
	em_pcTilePredSearch 			= nullptr;
	em_pcTileCuEncoder  			= nullptr;
		
	em_pInfoCU					= nullptr; 	///Data Structure for  Tile Functions @2015 5 17 by Seok

	em_uiTileIdx 					= 0;			///Index of Tile : 0 ~  em_uiNumTiles @ 2015 5 17 by Seok 
	em_uiNumTiles					= 0;			///Total Number of Tiles @ 2015 5 17 by Seok

}

TEncTile::~TEncTile()
{

}

Void TEncTile::create()
{
	em_pInfoCU = new ETRI_InfoofCU	;
	memset(em_pInfoCU, 0, sizeof(ETRI_InfoofCU));
}

Void TEncTile::destroy(Bool bOperation)
{
	if (!bOperation) {return;}

	for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			delete em_pppcTileRDSbacCoders[iDepth][iCIIdx];		em_pppcTileRDSbacCoders[iDepth][iCIIdx] = nullptr;
			delete em_pppcTileBinCodersCABAC[iDepth][iCIIdx];	em_pppcTileBinCodersCABAC[iDepth][iCIIdx] = nullptr;
		}
		delete [] em_pppcTileRDSbacCoders[iDepth];		em_pppcTileRDSbacCoders[iDepth] = nullptr;
		delete [] em_pppcTileBinCodersCABAC[iDepth];	em_pppcTileBinCodersCABAC[iDepth] = nullptr;
	}
	delete [] em_pppcTileRDSbacCoders;		em_pppcTileRDSbacCoders = nullptr;
	delete [] em_pppcTileBinCodersCABAC;	em_pppcTileBinCodersCABAC = nullptr;

	delete em_pcTileEntropyCoder; 			em_pcTileEntropyCoder = nullptr;  		
	delete em_pcTileRDGoOnSbacCoder;		em_pcTileRDGoOnSbacCoder = nullptr;
	delete em_pcTileRDGoOnBinCodersCABAC;	em_pcTileRDGoOnBinCodersCABAC = nullptr;
	delete em_pcTileBitCounter;    			em_pcTileBitCounter = nullptr;

	em_pcTileCuEncoder->destroy();

	if (em_pcTileRdCost)		{delete em_pcTileRdCost;   	em_pcTileRdCost   		= nullptr;} 
	if (em_pcTileTrQuant)		{delete em_pcTileTrQuant;  	em_pcTileTrQuant    	= nullptr;}
	if (em_pcTilePredSearch)	{delete em_pcTilePredSearch;em_pcTilePredSearch		= nullptr;}
	if (em_pcTileCuEncoder) 	{delete em_pcTileCuEncoder;   	em_pcTileCuEncoder  	= nullptr;} 

	if (em_pInfoCU) 	{delete em_pInfoCU;}

}


// ====================================================================================================================
// Definitions 
// ====================================================================================================================

#define	ETRI_REMOVE_CODER_RESET		1		///1 is active (Remove the Reset Coder Routine) for DEBUG 2015 5 21 by Seok
#define	ETRI_DBG_PRINT					0		///0 No-Print Debug Info, 1 Print Debug Info @ 2015 5 21 by Seok

/**
---------------------------------------------------------------------------------------------------------------
	@brief: Initialization of TIle Encoder i.e. memory allocation and Initilization of RD and Data Coders. 
			6 RD coders and 4 Data Coders are allocated andf initilized here.
			However, we don't set the RateControl Class parallely here. If it is required, we should construct it here to support Parallelism.
	@author: First Formulation is designed by yhee and Revised Jinwuk Seok as the member function of TEncTile 
	@date:  2015 5 19 
---------------------------------------------------------------------------------------------------------------
*/
Void	TEncTile::	init(TEncTop* pcEncTop, UInt uiTileIdx, Bool bOperation)
{
	if (!bOperation) {return;}

	//ESPRINTF( ETRI_E265_PH01, stderr, " Compiled @%s  [%s] \n", __DATE__, __TIME__);
	/*-------------------------------------------------------------------------------
		em_pcPic, and 	em_pcSlice can be obtained in the TEncGOP. Since pcPic is a Picture Object and 
		it is obtained in the GOP Level. Thus, these class pointer cannot be get in this function.
		This function is defined and operated in the xInitLib(). (i.e. at the first part of encoding)
	---------------------------------------------------------------------------------*/
	em_pcCfg  	= pcEncTop;

	em_uiTileIdx = uiTileIdx;
	//--------------------------------------------------------------------------------
	//	RD Coder Allocation and Initialization : with TEncBinCABACCounter  
	//--------------------------------------------------------------------------------

	em_pcTileEntropyCoder			= new TEncEntropy;
	em_pcTileRDGoOnSbacCoder		= new TEncSbac;			///: Real SBAC Coder based on TEncBinCABACCounter : RDGoOnSbacCoder @ 2015 5 21 by Seok
	em_pcTileRDGoOnBinCodersCABAC	= new TileEncBinCABAC;	///: TEncBinCABACCounter @ 2015 5 19 by Seok 
	em_pcTileBitCounter    			= new TComBitCounter;

	em_pppcTileRDSbacCoders   		= new TEncSbac**  		[g_uiMaxCUDepth+1];	///SBAC Storage for CU Encoder based on TEncBinCABACCounter : RDGoOnSbacCoder @ 2015 5 21 by Seok
	em_pppcTileBinCodersCABAC		= new TileEncBinCABAC**	[g_uiMaxCUDepth+1];	///TEncBinCABACCounter @ 2015 5 21 by Seok
	for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		em_pppcTileRDSbacCoders[iDepth]		= new TEncSbac* [CI_NUM];
		em_pppcTileBinCodersCABAC[iDepth]	= new TileEncBinCABAC* [CI_NUM];

		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			em_pppcTileRDSbacCoders[iDepth][iCIIdx]   	= new TEncSbac;
			em_pppcTileBinCodersCABAC[iDepth][iCIIdx]	= new TileEncBinCABAC;
			em_pppcTileRDSbacCoders[iDepth][iCIIdx]->init( em_pppcTileBinCodersCABAC[iDepth][iCIIdx] );	///Init with initCabacCounter, Faster @ 2015 5 19 by Seok
		}
	}

	em_pcTileRDGoOnSbacCoder->init(em_pcTileRDGoOnBinCodersCABAC );	///Init with initCabacCounter, Faster : m_pcBinIf = TEncBINCABACCounter @ 2015 5 19 by Seok

	//--------------------------------------------------------------------------------
	//	Compression Coder Allocation and Initialization
	//--------------------------------------------------------------------------------
	em_pcTilePredSearch		= new TEncSearch;
	em_pcTileCuEncoder   		= new TEncCu;
	em_pcTileTrQuant    		= new TComTrQuant;
	em_pcTileRdCost   			= new TComRdCost;

	em_pcTileRdCost->init();

	UInt	e_uiQuadtreeTULog2MaxSize 	= pcEncTop->getQuadtreeTULog2MaxSize();
	Bool	e_bUseAdaptQpSelect   		= pcEncTop->getUseAdaptiveQP();
	Bool	e_useRDOQ  					= pcEncTop->getUseRDOQ(); 
	Bool	e_useRDOQTS  				= pcEncTop->getUseRDOQTS();
	Bool	e_useTransformSkipFast		= pcEncTop->getUseTransformSkipFast();

	em_pcTileTrQuant->init( 1 << e_uiQuadtreeTULog2MaxSize,	e_useRDOQ, e_useRDOQTS, true, e_useTransformSkipFast
					#if ADAPTIVE_QP_SELECTION                  
					 , e_bUseAdaptQpSelect
					#endif
					);

	Int 	e_iSearchRange				= pcEncTop->getSearchRange();
	Int 	e_iFastSearch				= pcEncTop->getFastSearch();
	Bool	e_bipredSearchRange			= pcEncTop->ETRI_getBipredSearchRange();
	
	em_pcTilePredSearch->init( pcEncTop, em_pcTileTrQuant, e_iSearchRange, e_bipredSearchRange, e_iFastSearch, 0, 
						       em_pcTileEntropyCoder, em_pcTileRdCost, em_pppcTileRDSbacCoders, em_pcTileRDGoOnSbacCoder);	

	em_pcTileCuEncoder->create(g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight );
	em_pcTileCuEncoder->ETRI_initForTiles( pcEncTop, em_pcTilePredSearch, em_pcTileTrQuant, em_pcTileBitCounter, em_pcTileEntropyCoder, 
									       em_pcTileRdCost, em_pppcTileRDSbacCoders, em_pcTileRDGoOnSbacCoder);			

}

// ====================================================================================================================
// Data and Coder Setting
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Reset SBAC Coders for Tile Coders using a Dummy TEncBinCABAC Coder (em_pcTileRDGoOnSbacCoder /em_ppppcTileRDSbacCoder)
			All states SBAC Coders sre reset to an initial status.
			
	@author: yhee Revised Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncTile::ETRI_initTileSbacRD()
{
	//init em_pcTileRDGoOnSbacCoder
	em_pcTileEntropyCoder->setEntropyCoder(em_pcTileRDGoOnSbacCoder, em_pcSlice);		///Slice Data to Sbac Coder and IF, and Sbac to IF @ 2015 5 21 by Seok
	em_pcTileEntropyCoder->resetEntropy();     											///initialize: QP, CABAC table and CTX Model to Slice Type @ 2015 5 21 by Seok
	em_pcTileEntropyCoder->setBitstream(em_pcTileBitCounter);	   						///Set BitCounter to IF @ 2015 5 21 by Seok
	((TEncBinCABAC*)em_pcTileRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);	///<Bin Counting Enable (m_binCountIncremet) in Sbac Coder @ 2015 5 21 by Seok
	((TEncBinCABAC*)em_pcTileRDGoOnSbacCoder->getEncBinIf())->setBinsCoded( 0 );		///Set  m_uiBinsCoded = 0 @ 2015 5 21 by Seok
	((TEncBinCABAC*)em_pcTileRDGoOnSbacCoder->getEncBinIf())->ETRI_resetFracBits();		///@ 2015 5 19 by Seok	: by ETRI_CABAC_RESET_YHEE

	//init em_ppppcTileRDSbacCoders
	em_pppcTileRDSbacCoders[0][CI_CURR_BEST]->load(em_pcTileRDGoOnSbacCoder);			///initialize, Copy State, CTX, QP from Source SBAC : Set From TEncBinCABCCounter 
	((TEncBinCABAC *)em_pppcTileRDSbacCoders[0][CI_CURR_BEST]->getEncBinIf())->setBinsCoded( 0 );

	//init em_pcTileBitCount
	em_pcTileBitCounter->resetBits();													///Reset m_uiBitCounter 2015 5 21 by Seok

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of Tile Coders according to the result of previous Slice or Frame Compression.
			The status of QP, CAVLC, CAVAC and Rate Control are changed, 
			so that RDCost (Lambda, Color Weight), TrQuant (QP), CAVLC, CABAC and BinCABAC should be reset. 
			Main Codes are in the initSliceCoders of TencSlice in ETRI_compressGOP 
			Including ETRI_initTileSbacRD, RD coders are reset in this function
	@author: yhee, zeroone and Revised Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if KAIST_RC
Void	TEncTile::ETRI_initTileCoders (TComPic* pcPic, TComSlice* pcSlice, TComRdCost* pcRdCost, TComTrQuant* pcTrQuant, TEncCavlc*& pcCavlcCoder, TEncSbac*& pcSbacCoder, TEncBinCABAC*& pcBinCABAC, TEncRateCtrl*& pcRateCtrl)
#else
Void	TEncTile::ETRI_initTileCoders (TComPic* pcPic, TComSlice* pcSlice, TComRdCost* pcRdCost, TComTrQuant* pcTrQuant, TEncCavlc*& pcCavlcCoder, TEncSbac*& pcSbacCoder, TEncBinCABAC*& pcBinCABAC)
#endif
{
	///--------------------------------------------------------------
	///	Initial setting m_pcTileRdCosts	
	///--------------------------------------------------------------

	Double dLambda  			= pcRdCost->ETRI_getLambda();
	Double dFrameLambda 		= pcRdCost->ETRI_getFrameLambda();
	Double dCbDistortionWeight 	= pcRdCost->ETRI_getCbDistortionWeight();	///only with #if WEIGHTED_CHROMA_DISTORTION 1, see TComRdCost.h
	Double dCrDistortionWeight	= pcRdCost->ETRI_getCrDistortionWeight();
#if RDOQ_CHROMA_LAMBDA 
	Double	dTrLambda[3];
	dTrLambda[0] = pcTrQuant->ETRI_getLambdaLuma();	///LUMA :  2015 5 19 by Seok
	dTrLambda[1] = pcTrQuant->ETRI_getLambdaCb(); 
	dTrLambda[2] = pcTrQuant->ETRI_getLambdaCr(); 
#else
	Double dTrLambda 			= m_pcTrQuant->ETRI_getLambda();
#endif
	em_pcTileCuEncoder->ETRI_initForTiles(pcCavlcCoder, pcSbacCoder, pcBinCABAC
#if KAIST_RC
    , pcRateCtrl
#endif
    );	///Set CU Encoder for Tile Codings set Coders in Slice  
	em_pcTileRdCost->ETRI_InitForTile();
	em_pcTileRdCost->setLambda(dLambda);
	em_pcTileRdCost->setFrameLambda(dFrameLambda);
	em_pcTileRdCost->setCbDistortionWeight(dCbDistortionWeight);
	em_pcTileRdCost->setCrDistortionWeight(dCrDistortionWeight);
#if ETRI_SCALING_LIST_OPTIMIZATION
	em_pcTileTrQuant->setLambdas(dTrLambda);	  				///The code is same whether RDOQ_CHROMA_LAMBDA is active or not @ 2015 5 20 by Seok
#else
	UInt scaling_list_Id = ((TEncTop*)em_pcCfg)->getUseScalingListId();

	//set scaling list
	if(scaling_list_Id == SCALING_LIST_OFF)
	{
		em_pcTileTrQuant->setFlatScalingList();
		em_pcTileTrQuant->setUseScalingList(false);		
	}
	else if(scaling_list_Id== SCALING_LIST_DEFAULT)
	{	printf("error : ScalingList == %d no support\n",scaling_list_Id); assert(0);}
	else if(scaling_list_Id  == SCALING_LIST_FILE_READ)
	{	printf("error : ScalingList == %d no support\n",scaling_list_Id); assert(0);}
	else
	{	printf("error : ScalingList == %d no support\n",scaling_list_Id); assert(0);}

	em_pcTileTrQuant->setLambdas(dTrLambda);					///The code is same whether RDOQ_CHROMA_LAMBDA is active or not @ 2015 5 20 by Seok
#endif	

	///--------------------------------------------------------------
	/// Initilization of Tile Encode itself @ 2015 5 20 by Seok
	///--------------------------------------------------------------
	em_pcPic = pcPic;
	em_pcSlice = pcSlice;
	ETRI_initTileSbacRD();		///Reset RDCoders for Tiles @ 2015 5 19 by Seok

	//---------------------------------------------------------------
#if KAIST_RC
	em_pcRateCtrl = pcRateCtrl; ///At this time, we abandon Tile Rate Control @ 2015 5 21 by Seok
#endif

#if ETRI_GFParallelCompliance
	em_pcTileTrQuant->getQParam()->clear();
#endif
}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Evaluation Tile Address and Start/Final CU Order
			To Tile, CU Address is somewhat complicated than other types of partions in picture.
			Thus, we should the function evaluating CU Address to get into the TIle Compressor.
	@param: 	ETRI_InfoofCU*& rpCurrInfo 		Struct including Current Processing CU Address		
	@param: 	ETRI_InfoofCU*& rpPrevInfo 		Struct including Previous Processing CU Address		
	@param:  ETRI_InfoofCU& pETRI_InfoofCU	Struct get TileIdx from here (to use another Function or Objects)
	@param:  UInt iTileIdx					TileIdx (from outside .. )
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if ETRI_MULTITHREAD_2
Void TEncTile::ETRI_getTileCUAddr(ETRI_InfoofCU*& rpCurrInfo, ETRI_InfoofCU* rpPrevInfo, ETRI_InfoofCU& pETRI_InfoofCU, UInt iTileIdx)
#else
Void TEncTile::ETRI_getTileCUAddr(ETRI_InfoofCU*& rpCurrInfo, ETRI_InfoofCU*& rpPrevInfo, ETRI_InfoofCU& pETRI_InfoofCU, UInt iTileIdx)
#endif
{
	UInt	uiTileWidth =  em_pcPic->getPicSym()->getTComTile(iTileIdx)->getTileWidth();
	UInt	uiTileHeight =  em_pcPic->getPicSym()->getTComTile(iTileIdx)->getTileHeight();

	em_uiTileIdx = iTileIdx;

#if !ETRI_MULTITHREAD_2
	rpPrevInfo = (iTileIdx == 0)? nullptr : rpCurrInfo;
#endif
	rpCurrInfo = em_pInfoCU;
	
	memcpy(rpCurrInfo, &pETRI_InfoofCU, sizeof(ETRI_InfoofCU));		///To avoid Memory Release Problem @ 2015 5 21 by Seok
	
	rpCurrInfo->ETRI_StartCUOrder =  (iTileIdx == 0)? 0 : rpPrevInfo->ETRI_FinalCUOrder; 
	rpCurrInfo->ETRI_FinalCUOrder = uiTileWidth * uiTileHeight ;
	rpCurrInfo->ETRI_FinalCUOrder += rpCurrInfo->ETRI_StartCUOrder;
}

// ====================================================================================================================
// Tile Compression Functions 
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of RD Coders for SubStream.  Set ppppcRDSbacCoders & m_pppcRDSbacCoder[0][CI_CURR_BEST]
			For Two Types of Substream such as Tile and WPP. Remember : 
	 		Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
			// iNumSubstreams has already been multiplied. 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncTile::ETRI_InitRDCoderForSubStream(TComDataCU*& pcCU)
{
	// set go-on entropy coder
	em_pcTileEntropyCoder->setEntropyCoder 	(em_pcTileRDGoOnSbacCoder, em_pcSlice );	/// 2015 5 20 by Seok
	em_pcTileEntropyCoder->setBitstream	  	(em_pcTileBitCounter);	/// 2015 5 20 by Seok
	((TEncBinCABAC*)em_pcTileRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);	/// 2015 5 20 by Seok

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of Rate Control Based on SBAC Coding Tool 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncTile::ETRI_InitRateControlSBACRD(TComDataCU*& pcCU)
{
	if ( em_pcCfg->getUseRateCtrl() )
	{
#if KAIST_RC
		Int intra_size = em_pcCfg->getIntraPeriod();
		Int iIDRIndex = pcCU->getPic()->getPOC() / intra_size;
		Int iIDRModulus = pcCU->getPic()->getPOC() % intra_size;
		TRCPic* tRCPic = em_pcRateCtrl->getTRCPic(iIDRModulus);
#endif
		Int estQP 	   = em_pcSlice->getSliceQp();
		Double estLambda = -1.0;
		Double bpp	   = -1.0;

		if ( ( em_pcPic->getSlice( 0 )->getSliceType() == I_SLICE && em_pcCfg->getForceIntraQP() ) || !em_pcCfg->getLCULevelRC() )
		{
			estQP = em_pcSlice->getSliceQp();
		}
		else
		{
#if KAIST_RC
			bpp = tRCPic->getLCUTargetBpp(pcCU->getAddr(), em_pcSlice->getSliceType());
			if ( em_pcPic->getSlice( 0 )->getSliceType() == I_SLICE)
			{
				estLambda = tRCPic->getLCUEstLambdaAndQP(pcCU->getAddr(), bpp, em_pcSlice->getSliceQp(), &estQP);
			}
			else
			{
				estLambda = tRCPic->getLCUEstLambda(pcCU->getAddr(), bpp);
				estQP = tRCPic->getLCUEstQP(pcCU->getAddr(), estLambda, em_pcSlice->getSliceQp());
			}
#endif
			estQP	  = Clip3( -em_pcSlice->getSPS()->getQpBDOffsetY(), MAX_QP, estQP );

			em_pcTileRdCost->setLambda(estLambda);
#if RDOQ_CHROMA_LAMBDA
			// set lambda for RDOQ
			Double weight=em_pcTileRdCost->getChromaWeight();
			const Double lambdaArray[3] = { estLambda, (estLambda / weight), (estLambda / weight) };
			em_pcTileTrQuant->setLambdas( lambdaArray );
#else
			em_pcTileTrQuant->setLambda( estLambda );
#endif
		}
#if KAIST_RC
		tRCPic->getLCU(pcCU->getAddr()).m_QP = estQP;
#endif
#if ADAPTIVE_QP_SELECTION
		pcCU->getSlice()->setSliceQpBase( estQP );
#endif
	}


}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief:  restore entropy coder to an initial stage after comress CU
	@return:  TRUE : Break for multiple CU process  FALSE : Non Break
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncTile::ETRI_RestoreEntropyCoder(TComDataCU*& pcCU)
{
	// restore entropy coder to an initial stage
	em_pcTileEntropyCoder->setEntropyCoder ( em_pppcTileRDSbacCoders[0][CI_CURR_BEST], em_pcSlice );		/// 2015 5 20 by Seok : DBG Point Slice Coder???
	em_pcTileEntropyCoder->setBitstream(em_pcTileBitCounter);		
	em_pcTileCuEncoder->setBitCounter(em_pcTileBitCounter);		

//	em_pcSliceBitCounter = em_pcTileBitCounter;	/// 2015 5 20 by Seok : not useful.

	((TEncBinCABAC *)em_pppcTileRDSbacCoders[0][CI_CURR_BEST]->getEncBinIf())->setBinCountingEnableFlag( true );
	em_pcTileBitCounter->resetBits();																		///Since m_pcBitCounter = em_pcTileBitCounter @ 2015 5 20 by Seok
	((TEncBinCABAC *)em_pppcTileRDSbacCoders[0][CI_CURR_BEST]->getEncBinIf())->setBinsCoded( 0 );

	em_pcTileCuEncoder->encodeCU( pcCU );

	((TEncBinCABAC *)em_pppcTileRDSbacCoders[0][CI_CURR_BEST]->getEncBinIf())->setBinCountingEnableFlag( false );

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief:  Update Parameters of Rate Control after comress CU
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if KAIST_RC
Void TEncTile::ETRI_RestoreRateControl(TComDataCU*& pcCU)
{
	if ( em_pcCfg->getUseRateCtrl() )
	{
		Int 		actualQP		  	= g_RCInvalidQPValue;
		Double	actualLambda 	= em_pcTileRdCost->getLambda();
		Int 		actualBits	  	= pcCU->getTotalBits();
		Int 		numberOfEffectivePixels  = 0;
		for ( Int idx = 0; idx < em_pcPic->getNumPartInCU(); idx++ )
		{
			if ( pcCU->getPredictionMode( idx ) != MODE_NONE && ( !pcCU->isSkipped( idx ) ) )
			{
				numberOfEffectivePixels = numberOfEffectivePixels + 16; break;
			}
		}

		actualQP = ( numberOfEffectivePixels == 0 )? g_RCInvalidQPValue : pcCU->getQP(0);
		em_pcTileRdCost->setLambda(*em_pInfoCU->oldLambda);

#if KAIST_RC
		Int intra_size = em_pcCfg->getIntraPeriod();
		Int iIDRIndex = pcCU->getPic()->getPOC() / intra_size;
		Int iIDRModulus = pcCU->getPic()->getPOC() % intra_size;
		TRCPic* tRCPic = em_pcRateCtrl->getTRCPic(iIDRModulus);
		tRCPic->updateAfterLCU(pcCU->getAddr(), actualBits, actualQP, actualLambda,
			pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : em_pcCfg->getLCULevelRC() );
#endif
	}

}
#endif


/**
=====================================================================================================================
	@brief: Main Tile Compression Function. If you want Multiprocessing, connect the MultiThread API or Thread Pool to this function. 
	@author: Jinwuk Seok  2015 5 11 
=====================================================================================================================
*/
Void TEncTile::ETRI_CompressUnit()
{
	UInt uiEncCUOrder = 0;
	UInt ETRI_StartCUOrder = em_pInfoCU->ETRI_StartCUOrder;
	UInt ETRI_FinalCUOrder = em_pInfoCU->ETRI_FinalCUOrder;
	UInt uiCUAddr = em_pcPic->getPicSym()->getCUOrderMap(ETRI_StartCUOrder);
//#if !ETRI_MULTITHREAD_2
	em_pcTileCuEncoder->ETRI_Init_onPictureSliceTile(em_pcPic->getSlice(0));
//#endif


	for(uiEncCUOrder = ETRI_StartCUOrder; uiEncCUOrder < ETRI_FinalCUOrder; uiCUAddr = em_pcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
	{
		///Indicate the Cu Address to all Functions : 2015 5 20 by Seok
		*em_pInfoCU->uiCUAddr = uiCUAddr;				

		// initialize CU encoder
		TComDataCU*& pcCU = em_pcPic->getCU( uiCUAddr );
		pcCU->initCU( em_pcPic, uiCUAddr );

		///Initilization Entropy Coder (TEncSBAC for CU Compression,	m_pcRDGoOnSbacCoder) @ 2015 5 15 by Seok
		ETRI_InitRDCoderForSubStream(pcCU);				
		ETRI_InitRateControlSBACRD(pcCU);		///Initilization of Rate Control @ 2015 5 15 by Seok

		// run CU encoder
		em_pcTileCuEncoder->compressCU( pcCU );

		// Restore CU encoder
		ETRI_RestoreEntropyCoder(pcCU);	///Restore Entropy Coder to Initial Stage @ 2015 5 15 by Seok
#if KAIST_RC
		ETRI_RestoreRateControl(pcCU);		///Restore Rate Control State @ 2015 5 15 by Seok 
#endif

		em_pInfoCU->u64PicTotalBits 	+= pcCU->getTotalBits();
		em_pInfoCU->u64PicDist		+= pcCU->getTotalDistortion();
		em_pInfoCU->dPicRdCost		+= pcCU->getTotalCost();

	}
#if !ETRI_MULTITHREAD_2
	em_pcTileCuEncoder->ETRI_Final_onPictureSliceTile(em_pcSlice, em_uiTileIdx);
#endif

}


#if ETRI_THREAD_LOAD_BALANCING

Void TEncTile::ETRI_CompressUnit(TileLoadBalanceInfo lbInfo)
{
	UInt uiEncCUOrder = 0;
	UInt ETRI_StartCUOrder = em_pInfoCU->ETRI_StartCUOrder;
	UInt ETRI_FinalCUOrder = em_pInfoCU->ETRI_FinalCUOrder;
	UInt uiCUAddr = em_pcPic->getPicSym()->getCUOrderMap(ETRI_StartCUOrder);
	//#if !ETRI_MULTITHREAD_2
	em_pcTileCuEncoder->ETRI_Init_onPictureSliceTile(em_pcPic->getSlice(0));
	//#endif



	double myEncRate = 0;
	const double encRateToSleep = 0.6;
	const double encRateToWakeUp = 0.4;
	bool *bBalanced = lbInfo.bBalanced;
	int *nBalancedTile = lbInfo.nBalancedTile;
	int *nWaitingThread = lbInfo.nWaitingThread;
	int nMaxThread = lbInfo.nMaxThread;
	int nTile = lbInfo.nTile;

	pthread_mutex_t *mutex = lbInfo.mutex;
	pthread_cond_t *cond = lbInfo.cond;

	bool bRateCheck = false;

	int id = lbInfo.id;
	//int sliceCnt = lbInfo.sliceCnt;

	const double nCU = ETRI_FinalCUOrder - ETRI_StartCUOrder;
	double cntCU = 0;
		
	for (uiEncCUOrder = ETRI_StartCUOrder; uiEncCUOrder < ETRI_FinalCUOrder; uiCUAddr = em_pcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder))
	{
		if (*bBalanced == false)
		{
			myEncRate = cntCU++ / nCU;

			pthread_mutex_lock(mutex);
			if (myEncRate >= encRateToSleep)
			{
				if (*nBalancedTile >= nTile)
				{
					*bBalanced = true;
					pthread_cond_broadcast(cond);
				}
				else if (*nWaitingThread < nMaxThread && !bRateCheck)
				{
					(*nBalancedTile)++;
					(*nWaitingThread)++;
					pthread_cond_wait(cond, mutex);
				}
				else
				{
					if (!bRateCheck)
					{
						if (myEncRate >= encRateToWakeUp)
						{
							bRateCheck = true;
							(*nBalancedTile)++;
						}
					}
				}
			}
			pthread_mutex_unlock(mutex);
		}
		

		///Indicate the Cu Address to all Functions : 2015 5 20 by Seok
		*em_pInfoCU->uiCUAddr = uiCUAddr;


		// initialize CU encoder
		TComDataCU*& pcCU = em_pcPic->getCU(uiCUAddr);
		pcCU->initCU(em_pcPic, uiCUAddr);


		///Initilization Entropy Coder (TEncSBAC for CU Compression,	m_pcRDGoOnSbacCoder) @ 2015 5 15 by Seok
		ETRI_InitRDCoderForSubStream(pcCU);
		ETRI_InitRateControlSBACRD(pcCU);		///Initilization of Rate Control @ 2015 5 15 by Seok



		// run CU encoder
		em_pcTileCuEncoder->compressCU(pcCU);


		// Restore CU encoder
		ETRI_RestoreEntropyCoder(pcCU);	///Restore Entropy Coder to Initial Stage @ 2015 5 15 by Seok
		ETRI_RestoreRateControl(pcCU);		///Restore Rate Control State @ 2015 5 15 by Seok 


		em_pInfoCU->u64PicTotalBits += pcCU->getTotalBits();
		em_pInfoCU->u64PicDist += pcCU->getTotalDistortion();
		em_pInfoCU->dPicRdCost += pcCU->getTotalCost();

	}

	pthread_mutex_lock(mutex);
	if(*bBalanced == false)
	{
		pthread_cond_signal(cond);
	}
	pthread_mutex_unlock(mutex);

#if !ETRI_MULTITHREAD_2
	em_pcTileCuEncoder->ETRI_Final_onPictureSliceTile(em_pcSlice, em_uiTileIdx);
#endif

}
#endif

//! \}


