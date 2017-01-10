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

/** \file     encmain.cpp
	\brief    Encoder application main
*/

#include <time.h>
#include <fstream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"
#include "TDllEncoder.h"
#if _WIN32_WINNT >= 0x0602
// added. wsseo. for process group control
#include "TLibCommon/process_group.h"
#endif

using namespace std;
namespace po = df::program_options_lite;

#ifndef __FILENAME__
#define __FILENAME__ "TDllEncoder.cpp"
#endif

static void* pETRIEncoder;   		//< Encoder Pointer
unsigned int eg_uiBits;				//< ETRI global variable for count total bits



//===========================================================================================
//	Service Function
//===========================================================================================

#define 	BoolValue(x) ((x)? "true":"false")
__inline void error_dll (const char *text, int code)		{ ::fprintf_s(stderr, "%s\n", text); ETRI_EXIT(code); }

__inline	short 	ETRI_getDLLID  	  	   	  	(ETRI_Interface* EncoderIF)	{ return EncoderIF->CTRParam.iDllIdx; }
__inline	int    	ETRI_getiPOCLast  	  	   	(ETRI_Interface* EncoderIF)	{ return *EncoderIF->CTRParam.piPOCLastIdx; }
__inline	int    	ETRI_getNumPicRcvdIdx 	  	(ETRI_Interface* EncoderIF)	{ return *EncoderIF->CTRParam.piNumPicRcvdIdx; }
__inline	int    	ETRI_getNumAllPicCodedIdx 	(ETRI_Interface* EncoderIF)	{ return *EncoderIF->CTRParam.puiNumAllPicCodedIdx; }
__inline	bool 	ETRI_getAnalyzeClear   	   	(ETRI_Interface* EncoderIF)	{ return EncoderIF->CTRParam.bAnalyzeClear; }


#if (_ETRI_WINDOWS_APPLICATION)
void ETRI_Debug_DLLInterfaceInfo(ETRI_Interface* EncoderIF)
{
	::fprintf_s( stderr, "----- DLLInterface ----\n" );
	::fprintf_s( stderr, "EncoderIF          : %Ix \n", EncoderIF );
	::fprintf_s( stderr, "EncoderID          : %d \n", ETRI_getDLLID(EncoderIF) );
	::fprintf_s( stderr, "iNumEncoded        : %d \n", EncoderIF->iNumEncoded );
	::fprintf_s( stderr, "bEos               : %d \n", EncoderIF->bEos );	
	::fprintf_s( stderr, "m_piFrameRcvd      : %d \n", *EncoderIF->m_piFrameRcvd );	
	::fprintf_s( stderr, "m_pchInputFile     : %s \n", EncoderIF->m_pchInputFile );	
	::fprintf_s( stderr, "m_pchBitstreamFile : %s \n", EncoderIF->m_pchInputFile );	
	::fprintf_s( stderr, "ptrData            : %Ix \n", EncoderIF->ptrData );	
	::fprintf_s( stderr, "AnnexBData         : %Ix \n", EncoderIF->AnnexBData );
	::fprintf_s( stderr, "FrameSize          : %d \n", EncoderIF->FrameSize );
	::fprintf_s( stderr, "AnnexBFrameSize    : %d \n", EncoderIF->AnnexBFrameSize );
	::fprintf_s( stderr, "pcPicYuvOrg        : %Ix \n", EncoderIF->pcPicYuvOrg );
	::fprintf_s( stderr, "pcPicYuvRec        : %Ix \n", EncoderIF->pcPicYuvRec );
	::fprintf_s( stderr, "iPOCLastIdx        : %d \n", ETRI_getiPOCLast(EncoderIF) );
	::fprintf_s( stderr, "iNumPicRcvdIdx	 : %d \n", ETRI_getNumPicRcvdIdx(EncoderIF) );
	::fprintf_s( stderr, "uiNumAllPicCodedIdx: %d \n", ETRI_getNumAllPicCodedIdx(EncoderIF) );
	::fprintf_s( stderr, "bAnalyzeClear      : %s \n", BoolValue(ETRI_getAnalyzeClear(EncoderIF)) );

}
#else	// to remove warnigs for linux
void ETRI_Debug_DLLInterfaceInfo(ETRI_Interface* EncoderIF)
{
	::fprintf_s( stderr, "----- DLLInterface ----\n" );
	::fprintf_s( stderr, "EncoderIF          : %x \n", EncoderIF );
	::fprintf_s( stderr, "EncoderID          : %d \n", ETRI_getDLLID(EncoderIF) );
	::fprintf_s( stderr, "iNumEncoded        : %d \n", EncoderIF->iNumEncoded );
	::fprintf_s( stderr, "bEos               : %d \n", EncoderIF->bEos );	
	::fprintf_s( stderr, "m_piFrameRcvd      : %d \n", *EncoderIF->m_piFrameRcvd );	
	::fprintf_s( stderr, "m_pchInputFile     : %s \n", EncoderIF->m_pchInputFile );	
	::fprintf_s( stderr, "m_pchBitstreamFile : %s \n", EncoderIF->m_pchInputFile );	
	::fprintf_s( stderr, "ptrData            : %x \n", EncoderIF->ptrData );	
	::fprintf_s( stderr, "AnnexBData         : %x \n", EncoderIF->AnnexBData );
	::fprintf_s( stderr, "FrameSize          : %d \n", EncoderIF->FrameSize );
	::fprintf_s( stderr, "AnnexBFrameSize    : %d \n", EncoderIF->AnnexBFrameSize );
	::fprintf_s( stderr, "pcPicYuvOrg        : %x \n", EncoderIF->pcPicYuvOrg );
	::fprintf_s( stderr, "pcPicYuvRec        : %x \n", EncoderIF->pcPicYuvRec );
	::fprintf_s( stderr, "iPOCLastIdx        : %d \n", ETRI_getiPOCLast(EncoderIF) );
	::fprintf_s( stderr, "iNumPicRcvdIdx	 : %d \n", ETRI_getNumPicRcvdIdx(EncoderIF) );
	::fprintf_s( stderr, "uiNumAllPicCodedIdx: %d \n", ETRI_getNumAllPicCodedIdx(EncoderIF) );
	::fprintf_s( stderr, "bAnalyzeClear      : %s \n", BoolValue(ETRI_getAnalyzeClear(EncoderIF)) );
}
#endif
// ====================================================================================================================
// Main function
// ====================================================================================================================
#if ETRI_STATIC_DLL
ENCODERDLL_API void *EncoderMain::EncoderConstruct(int argc, char *argv[])
#else
extern "C" DLL_DECL void *ETRI_EncoderConstruct(int argc, char *argv[])
#endif
{
	if( argc < 2 || argv == NULL ) {
		return NULL;
	}

	TAppEncTop *pcTAppEncTop = new TAppEncTop;

	pcTAppEncTop->e_ETRIInterface.argc      = argc;
	pcTAppEncTop->e_ETRIInterface.argv      = argv;

	::memset( pcTAppEncTop->e_ETRIInterface.ServiceFunctionIndex, false, 16 );

	pcTAppEncTop->e_ETRIInterface.nFrameCount = 0;
	pcTAppEncTop->e_ETRIInterface.nGOPsize    = 0;

	::memset( pcTAppEncTop->e_ETRIInterface.nFrameStartOffset, 0, MAX_FRAME_NUM_IN_GOP * sizeof(unsigned int) );
	::memset( pcTAppEncTop->e_ETRIInterface.nFrameTypeInGop, 0, MAX_FRAME_NUM_IN_GOP * sizeof(unsigned int) );
	::memset( pcTAppEncTop->e_ETRIInterface.nPicPresentationOrder, 0, MAX_FRAME_NUM_IN_GOP * sizeof(unsigned int) );
	::memset( pcTAppEncTop->e_ETRIInterface.nPicDecodingOrder, 0, MAX_FRAME_NUM_IN_GOP * sizeof(unsigned int) );
	::memset( pcTAppEncTop->e_ETRIInterface.nTimestamp, 0, MAX_FRAME_NUM_IN_GOP * sizeof(UInt64) );
	::memset( pcTAppEncTop->e_ETRIInterface.nSliceIndex, 0, MAX_FRAME_NUM_IN_GOP * sizeof(short) );

	::memset( &pcTAppEncTop->e_ETRIInterface.CTRParam, 0, sizeof(pcTAppEncTop->e_ETRIInterface.CTRParam) );

	pETRIEncoder = nullptr;
	eg_uiBits    = 0;

#if ETRI_STATIC_DLL
  hTAppEncTop = (void *)pcTAppEncTop;
#endif

  return (void *)pcTAppEncTop;
}

#if ETRI_STATIC_DLL
ENCODERDLL_API bool EncoderMain::EncoderInitilization()
#else
extern "C" DLL_DECL bool  ETRI_EncoderInitilization	(void *hTAppEncTop)
#endif
{
	if( hTAppEncTop == NULL ) {
		return false;
	}

	TAppEncTop *pcTAppEncTop = (TAppEncTop *)hTAppEncTop;
	pcTAppEncTop->ETRI_DLLEncoderInitialize( pcTAppEncTop->e_ETRIInterface );

	//--------------------  Print DLL Information ---------------------
	::fprintf_s(stdout, "\n");
	::fprintf_s(stdout, "HM software: Encoder Version [%s]", NV_VERSION);
	::fprintf_s(stdout, NVM_ONOS);
	::fprintf_s(stdout, NVM_COMPILEDBY);
	::fprintf_s(stdout, NVM_BITS);
	::fprintf_s(stdout, "\n");

	::fprintf_s(stdout, APP_TYPE);
	::fprintf_s(stdout, RLS_TYPE);
	::fprintf_s(stdout, "\n");

#if _WIN32_WINNT >= 0x0602
	// added. wsseo. for process group control
	GetProcGrpMgr()->m_bUseProcessGroup = pcTAppEncTop->e_ETRIInterface.CTRParam.usUseProcessGroup != 0;
	if ( GetProcGrpMgr()->m_bUseProcessGroup )
	{
		::fprintf_s( stdout, "[PROC_GRP.GRP:%d, AFF:0x%016llX] ",	pcTAppEncTop->e_ETRIInterface.CTRParam.usPorcGroup, 
			pcTAppEncTop->e_ETRIInterface.CTRParam.ullAffinityMask );
		GetProcGrpMgr()->m_nProcessGroup = pcTAppEncTop->e_ETRIInterface.CTRParam.usPorcGroup;
		GetProcGrpMgr()->m_nAffinityMask = pcTAppEncTop->e_ETRIInterface.CTRParam.ullAffinityMask;
	}
	else
	{
		GetProcGrpMgr()->m_nProcessGroup = 0;
		GetProcGrpMgr()->m_nAffinityMask = 0;
	}
	::fprintf_s( stdout, "\n" );
#endif

	int FrameSize = pcTAppEncTop->e_ETRIInterface.m_iSourceWidth * pcTAppEncTop->e_ETRIInterface.m_iSourceHeight * (pcTAppEncTop->e_ETRIInterface.is16bit ? 2 : 1); //Size of Luma 
	FrameSize += (pcTAppEncTop->e_ETRIInterface.m_iSourceWidth >> 1) * (pcTAppEncTop->e_ETRIInterface.m_iSourceHeight >> 1) * (pcTAppEncTop->e_ETRIInterface.is16bit ? 2 : 1) * 2; //Size of U/V 

	//--------------------  HEVC Output Bitstream Setting ---------------------
	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bOutBufferOff )
	{
		pcTAppEncTop->e_ETRIInterface.AnnexBData = new unsigned char[FrameSize];
	}
	else
	{
		if( pcTAppEncTop->e_ETRIInterface.AnnexBData == NULL )
		{
			error_dll("\nfailed to Allocate AnnexBData\n", 0);
			return false;
		}
	}

	//--------------------  HEVC Input YUV Data Setting ---------------------
	/// Provide Full IO Processing @ 2016 2 1 by Seok
	if( pcTAppEncTop->e_ETRIInterface.CTRParam.iFullIORDProcess > 0 &&  pcTAppEncTop->e_ETRIInterface.CTRParam.bInBufferOff ) 
	pcTAppEncTop->e_ETRIInterface.CTRParam.bInBufferOff = false;
	
	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bInBufferOff )
	{
		pcTAppEncTop->e_ETRIInterface.ptrData    = new unsigned char[FrameSize];
	}
	
	//-------------------- HEVC Reconstruction YUV Setting (Not Valid : just Jointng the buffer and Stream) --------------------
	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bRecBufferOff )
	{
		pcTAppEncTop->e_ETRIInterface.ptrRec = new unsigned char[FrameSize];
	}
	else
	{
		if( pcTAppEncTop->e_ETRIInterface.AnnexBData == NULL )
	{
		error_dll("\nfailed to Allocate ptrRec\n", 0);
			return false;
	}
	}

	//-------------------- Data Interface --------------------
	pcTAppEncTop->e_ETRIInterface.FrameSize = FrameSize;

#if !ETRI_DLL_UNIFICATION
	EncoderIF->m_pcRecStr->open(EncoderIF->ptrRec, FrameSize);
#endif


	//--------------------	DLL CTRParam Setting ---------------------
	pcTAppEncTop->e_ETRIInterface.CTRParam.uiMultipleEncoder = 0;

	return true;
}

#if ETRI_STATIC_DLL
ENCODERDLL_API bool EncoderMain::EncoderMainFunc()
#else
extern "C" DLL_DECL bool  ETRI_EncoderMainFunc(void *hTAppEncTop)
#endif
{
	if( hTAppEncTop == NULL ) {
		return false;
	}

	TAppEncTop *pcTAppEncTop = (TAppEncTop *)hTAppEncTop;

	pcTAppEncTop->e_ETRIInterface.m_pcHandle->open( pcTAppEncTop->e_ETRIInterface.ptrData, pcTAppEncTop->e_ETRIInterface.FrameSize ); 	
	pcTAppEncTop->e_ETRIInterface.FStream->open( pcTAppEncTop->e_ETRIInterface.AnnexBData, pcTAppEncTop->e_ETRIInterface.FrameSize );
	
#if ETRI_BUGFIX_DLL_INTERFACE
	//Check End Of File From Encoder System 
	Bool bCheckDLLEos = pcTAppEncTop->e_ETRIInterface.bEos;
#endif

	// starting time
	//double dResult;
	//long lBefore = clock();

	// call encoding function
	pcTAppEncTop->encode( pcTAppEncTop->e_ETRIInterface );

	//--------------------	Store result of Encoding : 2013 10 23 by Seok ----------
	pcTAppEncTop->e_ETRIInterface.AnnexBFrameSize = pcTAppEncTop->e_ETRIInterface.FStream->Ftell();	//Size of Output per Frame

	//// check the TS data
	//ETRI_dbgMsg( ETRI_DBG_DLL, 2, " ========= 1 GOP ========== \n" );
	for( int i=0; i < pcTAppEncTop->e_ETRIInterface.iNumEncoded; i++ )
	{		
		/*ETRI_dbgMsg( ETRI_DBG_DLL, 2, "pcTAppEncTop->e_ETRIInterface.nFrameStartOffset[%d]:%6d \n", i, pcTAppEncTop->e_ETRIInterface.nFrameStartOffset[i] );
		ETRI_dbgMsg( ETRI_DBG_DLL, 2, "pcTAppEncTop->e_ETRIInterface.nFrameTypeInGop[%d]:\t %6d \n", i, pcTAppEncTop->e_ETRIInterface.nFrameTypeInGop[i] );
		ETRI_dbgMsg( ETRI_DBG_DLL, 2, "pcTAppEncTop->e_ETRIInterface.nPicDecodingOrder[%d]:%6d \n", i, pcTAppEncTop->e_ETRIInterface.nPicDecodingOrder[i] );
		ETRI_dbgMsg( ETRI_DBG_DLL, 2, "pcTAppEncTop->e_ETRIInterface.nPicPresentationOrder[%d]:%6d \n", i, pcTAppEncTop->e_ETRIInterface.nPicPresentationOrder[i] );*/

		eg_uiBits += pcTAppEncTop->e_ETRIInterface.nFrameStartOffset[i];
	}

	// encoding time
	//dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;
	//ETRI_dbgMsg( ETRI_DBG_DLL, 2, "\n Total Time: %12.3f sec.\n", dResult );

#if ETRI_BUGFIX_DLL_INTERFACE
	//Check End Of File From Encoder System 
	pcTAppEncTop->e_ETRIInterface.bEos = bCheckDLLEos;
#endif

	return true;
}

#if ETRI_STATIC_DLL
ENCODERDLL_API void EncoderMain::EncoderDestroy()	
#else
extern "C" DLL_DECL void  ETRI_EncoderDestroy(void **hTAppEncTop)
#endif	
{
	if( hTAppEncTop == NULL ) {
		return;
	}

#if ETRI_STATIC_DLL
	TAppEncTop *pcTAppEncTop = (TAppEncTop *)hTAppEncTop;	
#else
	TAppEncTop *pcTAppEncTop = (TAppEncTop *)*hTAppEncTop;	
#endif

	// delete original YUV buffer
	TComPicYuv* 	  pcPicYuvOrg = (TComPicYuv*)pcTAppEncTop->e_ETRIInterface.pcPicYuvOrg;
	pcPicYuvOrg->destroy();
	delete pcPicYuvOrg;
	pcPicYuvOrg = NULL;

#if ETRI_MULTITHREAD_2
	pcTAppEncTop->ETRI_xDeleteAU();	
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)	
	pcTAppEncTop->ETRI_xDeletePicBuffers(); //delet em_cListPicYuvRec, em_cListPic
#endif
#else
	pcTAppEncTop->getTEncTop().deletePicBuffer();
#endif
	// delete buffers & classes
	pcTAppEncTop->xDeleteBuffer();
	pcTAppEncTop->xDestroyLib();

	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bOutBufferOff && pcTAppEncTop->e_ETRIInterface.AnnexBData != NULL ) {
		delete[] pcTAppEncTop->e_ETRIInterface.AnnexBData;
		pcTAppEncTop->e_ETRIInterface.AnnexBData = NULL;
	}

	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bInBufferOff && pcTAppEncTop->e_ETRIInterface.ptrData != NULL ) {
		delete[] pcTAppEncTop->e_ETRIInterface.ptrData;
		pcTAppEncTop->e_ETRIInterface.ptrData = NULL;
	}

	if( pcTAppEncTop->e_ETRIInterface.CTRParam.bRecBufferOff && pcTAppEncTop->e_ETRIInterface.ptrRec != NULL ) {
		delete[] pcTAppEncTop->e_ETRIInterface.ptrRec;
		pcTAppEncTop->e_ETRIInterface.ptrRec = NULL;
	}

	pcTAppEncTop->destroy();
	delete pcTAppEncTop;

#if ETRI_STATIC_DLL
	hTAppEncTop = NULL;
#else
	*hTAppEncTop = NULL;
#endif
}


#if ETRI_STATIC_DLL
ENCODERDLL_API void EncoderMain::EncoderPrintSummary()
#else
extern "C" DLL_DECL void  ETRI_printSummary(void *hTAppEncTop)
#endif
{
	if (hTAppEncTop == NULL) {
		return;
	}

	TAppEncTop *pcTAppEncTop = (TAppEncTop *)hTAppEncTop;

	if( pcTAppEncTop->e_ETRIInterface.ServiceFunctionIndex[0] )
	{
		pcTAppEncTop->getTEncTop().printSummary( pcTAppEncTop->ETRI_getisField() );
		pcTAppEncTop->printRateSummary();

		if( pcTAppEncTop->e_ETRIInterface.CTRParam.iInfiniteProcessing ) {
			printf("eg_uiBits:  %d \n", eg_uiBits);
		}
	}
}

#if ETRI_STATIC_DLL
ETRI_Interface * EncoderMain::GetEncInterface()
#else
extern "C" DLL_DECL ETRI_Interface *ETRI_GetEncInterface(void *hTAppEncTop)
#endif
{
	if (hTAppEncTop == NULL) {
		return NULL;
	}

	TAppEncTop *pcTAppEncTop = (TAppEncTop *)hTAppEncTop;
	return &pcTAppEncTop->e_ETRIInterface;
}
