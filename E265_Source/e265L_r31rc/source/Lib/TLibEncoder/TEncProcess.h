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
	\file   	TEncProcess.h
   	\brief    	Process encoder class (header)
*/

#ifndef __TENCPROCESS__
#define __TENCPROCESS__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicYuv.h"
#include "TEncSlice.h"

//! \ingroup TLibEncoder
//! \{

//class TEncTop;
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class TEncProcess
{
protected:

	TComPic*   	em_pcPic;
	TComSlice*	em_pcSlice;
	TEncSlice*	em_pcSliceEncoder;

	UInt* 		em_uiCUAddr;
	UInt* 		em_uiStartCUAddr;
	UInt* 		em_uiBoundingCUAddr;

	UInt* 		em_uiWidthInLCUs;
	UInt* 		em_uiCol;
	UInt* 		em_uiLin; 
	UInt* 		em_uiSubStrm;
	UInt* 		em_uiTileCol;
	UInt* 		em_uiTileStartLCU;
	UInt* 		em_uiTileLCUX;

	UInt   		ETRI_StartCUOrder; /// For Start CU Order in a Tile : 2015 5 18 by Seok 
	UInt   		ETRI_FinalCUOrder; /// For Final CU Order in a Tile : 2015 5 18 by Seok 
	UInt 			em_uiProcessingType;

	Int*  		em_iNumSubstreams;
	Bool* 		em_bBreak;
	Bool* 		em_depSliceSegmentsEnabled;
	Double*		em_oldLambda;


public:
	TEncProcess();
	virtual ~TEncProcess();

	Void		create				();
	Void		destroy 			();
	Void		init					(TEncTop* pcEncTop );

	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------
	UInt*	ETRI_getuiCUAddr    	()	{return em_uiCUAddr;}
	UInt*	ETRI_getuiStartCUAddr	()	{return em_uiStartCUAddr;}
	UInt*	ETRI_getuiBoundingCUAddr()	{return em_uiBoundingCUAddr;}
	UInt*	ETRI_getuiWidthInLCUs	()	{return em_uiWidthInLCUs;}	
	UInt*	ETRI_getuiCol     		()	{return em_uiCol;}	
	UInt*	ETRI_getuiLin     		()	{return em_uiLin;}	
	UInt* 	ETRI_getuiSubStrm		()	{return em_uiSubStrm;}
	UInt* 	ETRI_getuiTileCol		()	{return	em_uiTileCol;}
	UInt* 	ETRI_getuiTileStartLCU	()	{return	em_uiTileStartLCU;}
	UInt* 	ETRI_getuiTileLCUX		()	{return	em_uiTileLCUX;}
	Int*  	ETRI_getiNumSubstreams	()	{return	em_iNumSubstreams;}
	Bool* 	ETRI_getbBreak			()	{return	em_bBreak;}
	Bool* 	ETRI_getdepSliceSegmentsEnabled(){return em_depSliceSegmentsEnabled;}
	Double*	ETRI_getdOldLambda		()	{return	em_oldLambda;}

	UInt&	ETRI_getuiStartCUOrder	()	{return	ETRI_StartCUOrder;} 
	UInt& 	ETRI_getuiFinalCUOrder	()	{return	ETRI_FinalCUOrder;} 
	UInt&	ETRI_getuiProcessingType()	{return em_uiProcessingType;}

	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------
	Void		ETRI_SetProcessorEncoder();
	Void 	ETRI_CopyToCompressor(Void *Compressor);


};


class TEncSingle : public TEncProcess
{
private:


public:
	TEncSingle();
	virtual ~TEncSingle();

	// -------------------------------------------------------------------------------------------------------------------
	// Initial functions
	// -------------------------------------------------------------------------------------------------------------------
	Void		create				();
	Void		destroy 			();
	Void		init					(TEncTop* pcEncTop );

	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------


};


//! \}
#endif


