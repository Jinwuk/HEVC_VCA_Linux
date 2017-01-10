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
#include "TEncWPP.h"

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncWPP::TEncWPP()
{
	em_pcPic   	= nullptr;
	em_pcSlice 	= nullptr;
	
	em_pcEncTop   			= nullptr;
	em_pcEncSlice   			= nullptr;

	em_pcWPPCuEncoder   		= nullptr;
	em_pcWPPPredSearch 		= nullptr;
	em_pcWPPTrQuant			= nullptr;
	em_pcWPPRdCost 			= nullptr;
#if KAIST_RC
	em_pcWPPRateCtrl			= nullptr;	
#endif
	em_pcSbacCoders 			= nullptr;
	em_pcBinCoderCABACs 		= nullptr;

	em_pcWPPEntropyCoder  	= nullptr;
	em_pcBitCounters 			= nullptr;   	/// @brief		code bit counters for RD optimization per substream endcode
	em_pppcRDSbacCoders   	= nullptr;		/// @brief		temporal storage for RD computation per substream
	em_pcRDGoOnSbacCoders	= nullptr; 	/// @brief		going on SBAC model for RD stage per substream
	em_pppcBinCodersCABAC 	= nullptr;		/// @brief		temporal CABAC state storage for RD computation per substream
	em_pcRDGoOnBinCodersCABAC = nullptr;	/// @brief		going on bin coder CABAC for RD stage per substream

	em_iNumSubstreams 		= 0;			/// @brief		Number of top-level elements allocated.

}

TEncWPP::~TEncWPP()
{

}

// -------------------------------------------------------------------------------------------------------------------
// Creation and Initilization 
// -------------------------------------------------------------------------------------------------------------------
#if FAST_BIT_EST
#define	WPPBINCABAC	TEncBinCABACCounter
#else
#define	WPPBINCABAC	TEncBinCABAC
#endif

/**
 -------------------------------------------------------------------------------------------------------------------
 	@brief	Create WPP Class with the including coders themselves, instead of coders from TEncTop.
 	@author	jinwuk seok
 -------------------------------------------------------------------------------------------------------------------
*/
Void	TEncWPP::create	()
{
	/**  m_iWaveFrontSubstreams = m_iWaveFrontSynchro ? (m_iSourceHeight+m_uiMaxCUHeight-1) / m_uiMaxCUHeight : 1; */
	em_iNumSubstreams = em_pcEncTop->getWaveFrontSubstreams();		

	em_pcWPPCuEncoder   		= new TEncCu;
	em_pcWPPPredSearch 		= new TEncSearch;
	em_pcWPPTrQuant			= new TComTrQuant;
	em_pcWPPRdCost 			= new TComRdCost;
#if KAIST_RC
	em_pcWPPRateCtrl			= new TEncRateCtrl;	
#endif
	em_pcWPPEntropyCoder  	= new TEncEntropy;
	em_pcSbacCoders 			= new TEncSbac;
	em_pcRDGoOnSbacCoders    	= new TEncSbac;
	em_pcBinCoderCABACs 		= new WPPBINCABAC;
	em_pcRDGoOnBinCodersCABAC 	= new WPPBINCABAC;
	em_pcBitCounters 			= new TComBitCounter ;

	em_pcRDGoOnSbacCoders = new TEncSbac;						
	em_pcSbacCoders = new TEncSbac;

	em_pppcRDSbacCoders  = new TEncSbac** [g_uiMaxCUDepth+1];
	em_pppcBinCodersCABAC= new WPPBINCABAC** [g_uiMaxCUDepth+1];

	for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		em_pppcRDSbacCoders[iDepth]  = new TEncSbac*	 [CI_NUM];
		em_pppcBinCodersCABAC[iDepth]= new WPPBINCABAC* [CI_NUM];

		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			em_pppcRDSbacCoders  [iDepth][iCIIdx] = new TEncSbac;
			em_pppcBinCodersCABAC[iDepth][iCIIdx] = new WPPBINCABAC;
			em_pppcRDSbacCoders  [iDepth][iCIIdx]->init( em_pppcBinCodersCABAC[iDepth][iCIIdx] );
		}
	}



}

Void	TEncWPP::destroy (Bool bOperation)
{

	if (em_pcWPPCuEncoder)	{delete em_pcWPPCuEncoder; em_pcWPPCuEncoder = nullptr;}
	if (em_pcWPPPredSearch	)	{delete em_pcWPPPredSearch; em_pcWPPPredSearch = nullptr;}
	if (em_pcWPPTrQuant) 		{delete em_pcWPPPredSearch; em_pcWPPPredSearch = nullptr;}
	if (em_pcWPPRdCost)		{delete em_pcWPPRdCost; em_pcWPPRdCost = nullptr;}
#if KAIST_RC
	if (em_pcWPPRateCtrl)		{delete em_pcWPPRateCtrl; em_pcWPPRateCtrl = nullptr;}
#endif
	if (em_pcWPPEntropyCoder)	{delete em_pcWPPEntropyCoder; em_pcWPPEntropyCoder = nullptr;}
	if (em_pcSbacCoders) 		{delete em_pcSbacCoders; em_pcSbacCoders = nullptr;}
	if (em_pcBinCoderCABACs)	{delete em_pcBinCoderCABACs; em_pcBinCoderCABACs = nullptr;}
	if (em_pcRDGoOnSbacCoders){delete em_pcRDGoOnSbacCoders; em_pcRDGoOnSbacCoders = nullptr;}
	if (em_pcRDGoOnBinCodersCABAC) {delete em_pcRDGoOnBinCodersCABAC; em_pcRDGoOnBinCodersCABAC = nullptr;}
	if (em_pcBitCounters)		{delete em_pcBitCounters; em_pcBitCounters = nullptr;}

	if (em_pcRDGoOnSbacCoders)	{delete em_pcRDGoOnSbacCoders; em_pcRDGoOnSbacCoders = nullptr;}
	if (em_pcSbacCoders)			{delete em_pcSbacCoders; em_pcSbacCoders = nullptr;}

	for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
	{
		for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
		{
			if (em_pppcRDSbacCoders  [iDepth][iCIIdx]) {delete [] em_pppcRDSbacCoders  [iDepth][iCIIdx]; em_pppcRDSbacCoders  [iDepth][iCIIdx] = nullptr;}
			if (em_pppcBinCodersCABAC[iDepth][iCIIdx]){delete [] em_pppcBinCodersCABAC[iDepth][iCIIdx]; em_pppcBinCodersCABAC[iDepth][iCIIdx] = nullptr;}
		}
		if (em_pppcRDSbacCoders[iDepth])   {delete [] em_pppcRDSbacCoders [iDepth]; em_pppcRDSbacCoders [iDepth] = nullptr;}
		if (em_pppcBinCodersCABAC[iDepth]){delete [] em_pppcBinCodersCABAC[iDepth]; em_pppcBinCodersCABAC[iDepth] = nullptr;}
	}

	if (em_pppcRDSbacCoders)	{delete em_pppcRDSbacCoders; em_pppcRDSbacCoders = nullptr;}
	if (em_pppcBinCodersCABAC)	{delete em_pppcBinCodersCABAC; em_pppcBinCodersCABAC = nullptr;}

}

Void	TEncWPP::init  (TEncTop* pcEncTop, UInt uiTileIdx, Bool bOperation)
{

	em_pcRDGoOnSbacCoders->init( em_pcRDGoOnBinCodersCABAC);
	em_pcSbacCoders->init( em_pcBinCoderCABACs);
}







//! \}
