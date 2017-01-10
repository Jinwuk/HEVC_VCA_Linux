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
	\file   	TEncProcess.cpp
   	\brief    	Process class (Main)
*/

#include "TEncTop.h"
#include "TEncProcess.h"

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================
TEncProcess::TEncProcess()
{
	em_pcPic   			= nullptr;
	em_pcSlice 			= nullptr;
	em_pcSliceEncoder 		= nullptr;

	em_uiCUAddr   		= nullptr;
	em_uiStartCUAddr   	=nullptr;
	em_uiBoundingCUAddr 	=nullptr;

	em_uiWidthInLCUs  	= nullptr;
	em_uiCol  			= nullptr;
	em_uiLin   			= nullptr; 
	em_uiSubStrm   		= nullptr;
	em_uiTileCol  			= nullptr;
	em_uiTileStartLCU  		= nullptr;
	em_uiTileLCUX   		= nullptr;

	ETRI_StartCUOrder  	= 0; /// For Start CU Order in a Tile : 2015 5 18 by Seok 
	ETRI_FinalCUOrder		= 0; /// For Final CU Order in a Tile : 2015 5 18 by Seok 

	em_iNumSubstreams 	= nullptr;
	em_bBreak   			= nullptr;
	em_depSliceSegmentsEnabled=nullptr;
	em_oldLambda    		= nullptr;
}

TEncProcess::~TEncProcess()
{

}

Void TEncProcess::create()
{

}

Void TEncProcess::destroy()
{

}

Void TEncProcess::init(TEncTop * pcEncTop)
{

}

// ====================================================================================================================
// Virtual Function
// ====================================================================================================================




// ====================================================================================================================
// TEncSingle Function [Initial]
// ====================================================================================================================




// ====================================================================================================================
// TEncSingle Function Main Process
// ====================================================================================================================




//! \}
