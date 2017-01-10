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

#ifndef __TENCTILE__
#define __TENCTILE__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicYuv.h"
#include "TEncSlice.h"
#include "TEncCu.h"
#include "TEncRateCtrl.h"

#if ETRI_THREADPOOL_OPT
#include <pthread.h>
#endif

//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
/**
	FAST_BIT_EST is much more Fast. So we use TEncBinCABACCounter to "em_ppppcTileBinCodersCABAC" and "em_pcTileRDGoOnBinCodersCABAC" 
	Instead of TEncBinCABAC @ 2015 5 19 by Seok
*/
#if FAST_BIT_EST 
#define	TileEncBinCABAC	TEncBinCABACCounter	
#else
#define	TileEncBinCABAC	TEncBinCABAC
#endif

#if ETRI_THREAD_LOAD_BALANCING
struct TileLoadBalanceInfo
{
	bool *bBalanced;
	int *nBalancedTile;
	int *nWaitingThread;
	int nMaxThread;
	int nTile;
	pthread_mutex_t *mutex;
	pthread_cond_t *cond;

	int id;
	//int sliceCnt;
};
#endif 


class TEncTile
{
private:
	TEncCfg*   		em_pcCfg;		
	TComPic*   		em_pcPic;
	TComSlice*		em_pcSlice;

	// RD Coders for TILE in Multiple Threads (Only 6)
	TEncEntropy*  	em_pcTileEntropyCoder;		///< EntropyCoder IF. used.
	TComBitCounter*	em_pcTileBitCounter;  		///< Bit counters per tile. used
	TEncSbac***		em_pppcTileRDSbacCoders;	///< temporal storage for RD computation per tile. used : *[Depth][CI_IDX:Model]
	TEncSbac*		em_pcTileRDGoOnSbacCoder;	///< going on SBAC model for RD stage per tile. used :init by em_pppcTileBinCodersCABAC
	TileEncBinCABAC*** 	em_pppcTileBinCodersCABAC; 	///< temporal CABAC state storage for RD computation per tile. used : *[Depth][CI_IDX:Model] 
	TileEncBinCABAC*	em_pcTileRDGoOnBinCodersCABAC;	///< going on bin coder CABAC for RD stage per tile 

	// Coders for Compression in Tile using Multiple Threads (only 4)
	TComRdCost*  	em_pcTileRdCost;			///< RD cost computation class per tile. used
	TComTrQuant*	em_pcTileTrQuant;			///< transform & quantization class per tile. used
	TEncSearch*    	em_pcTilePredSearch;
	TEncCu*    		em_pcTileCuEncoder;

//	TEncBinCABAC*	em_ppppcTileBinCabac;		///< It is not defined @ 2015 5 19 by Seok

	// It is necessary to set Tile Rate Control. However, in this application, we does not implementation now.
#if KAIST_RC
	TEncRateCtrl*  	em_pcRateCtrl;   				///< Rate control manager
#endif

	// Pointer Value
	UInt   			em_uiTileIdx;				///< Index of Tile : 0 ~  em_uiNumTiles @ 2015 5 17 by Seok 
	ETRI_InfoofCU*	em_pInfoCU; 				///< Data Structure for  Tile Functions @2015 5 17 by Seok

	//Const Value
	UInt   			em_uiNumTiles; 			///< Total Number of Tiles @ 2015 5 17 by Seok

public:
	TEncTile();
	virtual ~TEncTile();
	// -------------------------------------------------------------------------------------------------------------------
	// Creation and Initilization 
	// -------------------------------------------------------------------------------------------------------------------
	Void	create					();
	Void	destroy 				(Bool bOperation);
	Void	init  					(TEncTop* pcEncTop, UInt uiTileIdx, Bool bOperation);
	Void	ETRI_initTileSbacRD 	();
#if KAIST_RC
	Void	ETRI_initTileCoders 	(TComPic* pcPic, TComSlice* pcSlice, TComRdCost* pcRdCost, TComTrQuant* pcTrQuant, 
								TEncCavlc*& pcCavlcCoder, TEncSbac*& pcSbacCoder, TEncBinCABAC*& pcBinCABAC, TEncRateCtrl*& pcRateCtrl);
#else
  Void	ETRI_initTileCoders(TComPic* pcPic, TComSlice* pcSlice, TComRdCost* pcRdCost, TComTrQuant* pcTrQuant,
    TEncCavlc*& pcCavlcCoder, TEncSbac*& pcSbacCoder, TEncBinCABAC*& pcBinCABAC);
#endif
	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------
	Void 	ETRI_setTComPic 		(TComPic* pcPic)		{em_pcPic = pcPic;}
	Void 	ETRI_setTComSlice 		(TComSlice* pcSlice)	{em_pcSlice = pcSlice;}
#if KAIST_RC
	Void 	ETRI_setTEncRateControl	(TEncRateCtrl* pcRateCtrl) {em_pcRateCtrl = pcRateCtrl;}
#endif

	UInt&	ETRI_getTotalNumbetOfTile		(){return em_uiNumTiles;	}
	UInt&	ETRI_getTileIdx					(){return em_uiTileIdx;	}


	TComRdCost*  	ETRI_getTileRdCost		(){return em_pcTileRdCost;	}
	TComTrQuant*	ETRI_getTileTrQuant		(){return em_pcTileTrQuant;	}
	TEncCu*			ETRI_getTileCuEncoder	(){return em_pcTileCuEncoder;}
	TEncEntropy*  	ETRI_getTileEntropyCoder(){return em_pcTileEntropyCoder;}
	TComBitCounter*	ETRI_getTileBitCounter	(){return em_pcTileBitCounter;}
	TEncSbac*		ETRI_getTileRDGoOnSbacCoder	(){return em_pcTileRDGoOnSbacCoder;}
	TEncSbac***    	ETRI_getTileRDSbacCoders(){return em_pppcTileRDSbacCoders;}
	ETRI_InfoofCU*	ETRI_getInfoofCU ()	{return em_pInfoCU;}		///< Test Function @ 2015 5 18 by Seok
#if ETRI_MULTITHREAD_2
	Void			ETRI_setInfoofCUCopy(ETRI_InfoofCU* pInfoCU) { memcpy(em_pInfoCU, pInfoCU, sizeof(ETRI_InfoofCU)); }
	Void			ETRI_getInfoofCUCopy(ETRI_InfoofCU* pInfoCU) { memcpy(pInfoCU, em_pInfoCU, sizeof(ETRI_InfoofCU)); }
	Void			ETRI_getTileCUAddr(ETRI_InfoofCU*& rpCurrInfo, ETRI_InfoofCU* rpPrevInfo, ETRI_InfoofCU& pETRI_InfoofCU, UInt iTileIdx);
#else
	Void			ETRI_getTileCUAddr(ETRI_InfoofCU*& rpCurrInfo, ETRI_InfoofCU*& rpPrevInfo, ETRI_InfoofCU& pETRI_InfoofCU, UInt iTileIdx);
#endif


	// -------------------------------------------------------------------------------------------------------------------
	// Compression functions
	// -------------------------------------------------------------------------------------------------------------------
	void  	ETRI_InitRDCoderForSubStream	(TComDataCU*& pcCU); 		///< Initialization of RD Coders for SubStream.  Set ppppcRDSbacCoders & m_pppcRDSbacCoder[0][CI_CURR_BEST] 2015 5 15 by Seok
	void  	ETRI_InitRateControlSBACRD 		(TComDataCU*& pcCU);
	Void  	ETRI_RestoreEntropyCoder   		(TComDataCU*& pcCU);		///< restore entropy coder to an initial stage after comress CU @ 2015 5 15 by Seok 
#if KAIST_RC
	Void  	ETRI_RestoreRateControl			(TComDataCU*& pcCU);		///< Update Parameters of Rate Control after comress CU @ 2015 5 15 by Seok 					
#endif

	Void  	ETRI_CompressUnit   			();

#if ETRI_THREAD_LOAD_BALANCING
	Void  	ETRI_CompressUnit				(TileLoadBalanceInfo lbInfo);
#endif

//	Void 	ETRI_TileCompression 	(TEncSlice* pcSliceEncoder);	
};

//! \}
#endif


//Additional Codes after e265v03
/*
#if ETRI_TileLoadBalancing_JSH
  UInt			*em_uiTileBits;
  Int			**em_iTileEncodeOption;
#endif
*/

