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
	\file   	TEncWPP.h
   	\brief    	WPP encoder class (header)
*/

#ifndef __TENCWPP__
#define __TENCWPP__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicYuv.h"
#include "TEncSlice.h"
#include "TEncCu.h"
#include "TEncRateCtrl.h"

//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncSlice;

// ====================================================================================================================
// Class definition  

// TileEncBinCABAC***	em_pppcTileBinCodersCABAC;	///< temporal CABAC state storage for RD computation per tile. used : *[Depth][CI_IDX:Model] 
// TileEncBinCABAC*	em_pcTileRDGoOnBinCodersCABAC;	///< going on bin coder CABAC for RD stage per tile 

// ====================================================================================================================
class TEncWPP
{

private:

	TComPic*     			em_pcPic;
	TComSlice*  			em_pcSlice;
	
	TEncTop*				em_pcEncTop;
	TEncSlice*  			em_pcEncSlice;

	TEncCu* 				em_pcWPPCuEncoder;
	TComRdCost* 			em_pcWPPRdCost;			///< RD cost computation class per substream
	TComTrQuant*     		em_pcWPPTrQuant;			///< transform & quantization class substream
	TEncSearch* 			em_pcWPPPredSearch;
#if KAIST_RC
	TEncRateCtrl*			em_pcWPPRateCtrl;					///< Rate control manager
#endif

	TEncSbac*   			em_pcSbacCoders;			/// 2015 5 26 by Seok 
	TEncBinCABAC*  		em_pcBinCoderCABACs;		/// 2015 5 26 by Seok 

	TEncEntropy*  		em_pcWPPEntropyCoder;		///< EntropyCoder IF. used.
	TComBitCounter* 		em_pcBitCounters;   			///< bit counters for RD optimization per substream
	TEncSbac***			em_pppcRDSbacCoders;		///< temporal storage for RD computation per substream
	TEncSbac*   			em_pcRDGoOnSbacCoders; 		///< going on SBAC model for RD stage per substream

#if FAST_BIT_EST
	TEncBinCABACCounter***		em_pppcBinCodersCABAC;		///< temporal CABAC state storage for RD computation per substream
	TEncBinCABACCounter*   		em_pcRDGoOnBinCodersCABAC;	///< going on bin coder CABAC for RD stage per substream
#else
	TEncBinCABAC***		em_pppcBinCodersCABAC;		///< temporal CABAC state storage for RD computation per substream
	TEncBinCABAC*   		em_pcRDGoOnBinCodersCABAC;	///< going on bin coder CABAC for RD stage per substream
#endif

	Int 					em_iNumSubstreams;			///< # of top-level elements allocated.


public:
	TEncWPP();
	virtual ~TEncWPP();

	// -------------------------------------------------------------------------------------------------------------------
	// Creation and Initilization 
	// -------------------------------------------------------------------------------------------------------------------

	Void	create					();
	Void	destroy 				(Bool bOperation);
	Void	init  					(TEncTop* pcEncTop, UInt uiTileIdx, Bool bOperation);

	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------


	// -------------------------------------------------------------------------------------------------------------------
	// Compression functions
	// -------------------------------------------------------------------------------------------------------------------

	Void  	ETRI_CompressUnit   			();


};
//! \}
#endif
