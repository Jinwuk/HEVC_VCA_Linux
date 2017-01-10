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
    @file    TAppDllEncoderExplite.h
    @date    2015/06/08
    @author  김연희 (kimyounhee@etri.re.kr) ETRI
    @brief   Header File for TAppDllEncoderExplite.cpp
*/

#ifndef	_TAPPDLLENCODER_H_
#define	_TAPPDLLENCODER_H_

#define	MAX_HEADER_BUFFER_SIZE   	9200	// 4(bytes) x (300(SeqHeaderCount) + 20 * 100(NumSlice) = 9200
#define	BASIC_NUM_BUFFER              32	// 2014 4 11 by Seok : Default Value
#define	ERROR_DLL_BUF_SZ            1024	// Debug Msg Buffer


#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <fstream>
#include "DLLInterfaceType.h"

#if (_ETRI_WINDOWS_APPLICATION)
#include <tchar.h>

#else

#include <xmmintrin.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#ifdef _UNICODE
#define _tprintf	wprintf
#define __T(x)		L ## x 
#else
#define _tprintf	printf
#define __T(x)		x 
#endif
#define	_T(x) __T(x)

#endif

#define ETRI_DLL_GOPParallel	1

// ====================================================================================================================
// Definitions (& Macros )
// ====================================================================================================================
#if defined(_USRDLL)
#ifndef ETRI_EXIT
#define	ETRI_EXIT(x)	throw(x)
#endif
#else
#ifndef ETRI_EXIT
#define	ETRI_EXIT(x)	exit(x)
#endif
#endif


// ====================================================================================================================
// ETRI DLL Memory Test
// ====================================================================================================================
typedef struct Test_IO_Info
{
	/// Test Memory
	void **testInput;				///< 2014 5 5 by Seok : For Test of a Multiple ENcoder DLL 
	void **testOutput;				///< 2014 5 5 by Seok : For Test of a Multiple ENcoder DLL 
	unsigned int i_nTestInput;		///< 2014 5 5 by Seok : Number of Pointers for *testInput
	unsigned int i_nTestOutput;		///< 2014 5 5 by Seok : Number of Pointers for *testOutput
	unsigned int ui_OpInputIdx;    	///< 2014 5 6 by Seok : Operating Index
	unsigned int ui_OpOutputIdx;    ///< 2014 5 6 by Seok : Operating Index

	void* GetData_From_TESTIO_INBUF ()
	{
		/// 포인터 _pBuffPtr 에 Buffer testInput 를 할당한다. 
		void* _pBuffPtr = testInput[ui_OpInputIdx];
		ui_OpInputIdx++;
		ui_OpInputIdx = (ui_OpInputIdx >= i_nTestInput)? 0 : ui_OpInputIdx;
		return _pBuffPtr; 
	};

	void* GetData_From_TESTIO_OUTBUF ()
	{
		/// 포인터 _pBuffPtr 에 Buffer testOutput 를 할당한다. 
		void* _pBuffPtr = testOutput[ui_OpOutputIdx];
		ui_OpOutputIdx++;
		ui_OpOutputIdx = (ui_OpOutputIdx >= i_nTestOutput)? 0 : ui_OpOutputIdx;
		return _pBuffPtr; 
	};

	void	Rewind_InputBuf(bool RewindCondition)
	{
		ui_OpInputIdx = (RewindCondition)? 0 : ui_OpInputIdx;
	}

	void	Rewind_OutputBuf(bool RewindCondition)
	{
		ui_OpOutputIdx = (RewindCondition)? 0 : ui_OpOutputIdx;
	}
} ETRI_TESTIO;

// ====================================================================================================================
// Function Pointer for DLL
// ====================================================================================================================
void *(*ETRI_HEVC_Constructor) 	(int argc, char *argv[]);
bool (*ETRI_HEVC_Init_func) 	(void *hTAppEncTop);
bool (*ETRI_HEVC_Encode_func) 	(void *hTAppEncTop);
void (*ETRI_HEVC_Destroyer)    	(void *hTAppEncTop);
void (*ETRI_HEVC_Service_func) 	(void *hTAppEncTop);
ETRI_Interface *(*ETRI_HEVC_GetEncInterface) (void *hTAppEncTop);

ETRI_TESTIO  	e_TestIO;		///< Simulation of Memory PooL


// ====================================================================================================================
// ETRI DLL Service Functions 
// ====================================================================================================================
__inline void error_dll (const char *text, int code)
{
#if (_ETRI_WINDOWS_APPLICATION)
	fprintf(stderr, "%s Error COde:%d \n", text, GetLastError()); 
#else
	// need to be confirmed !!! - shcho 160510
	fprintf(stderr, "%s Error COde:%d \n", text, errno); 
#endif
	ETRI_EXIT(code);
}
void	ReleaeseOutputPointer(void*& Outptr, bool Condition) { Outptr = (Condition)? Outptr : nullptr;}

void ETRI_dbgMsg(bool dbgCondition, int PrtMethod,  const char *fmt, ...)
{
	if (dbgCondition)
	{
		va_list args;
		char buf[ERROR_DLL_BUF_SZ];

		memset(buf, 0, ERROR_DLL_BUF_SZ);
		
		va_start(args, fmt);
#if (_ETRI_WINDOWS_APPLICATION)
		int 	len;
		len = _vscprintf(fmt, args) + 1;
#endif
		vsprintf_s(buf, len, fmt, args);
		va_end(args);
		
		switch(PrtMethod)
		{	
			default: _tprintf(_T("%s"), buf);	break;
			case 1 : printf_s("%s", buf); break;
			case 2 : fprintf(stderr, "%s", buf); break;
		}
	}
}

#define DLLNull_ErrorMSG(DLLFunc, text, code) \
	if (DLLFunc == NULL) {error_dll(text, code);}
#define 	BoolValue(x) ((x)? "true":"false")

// ====================================================================================================================
// ETRI Encoder Functions
// ====================================================================================================================

void Init_Encode (void *hTAppEncTop)
{	
	/// For Encoder Initialization 
	ETRI_HEVC_Init_func(hTAppEncTop);
}

void Encode (void *hTAppEncTop)	//wrapper Function 
{
	ETRI_HEVC_Encode_func(hTAppEncTop);	//Execute Encoding Function 
}

__inline	void 	ETRI_setAnalyzeClear  		(ETRI_Interface* EncoderIF, bool bGoOn)	{ EncoderIF->CTRParam.bAnalyzeClear	= bGoOn; }	// 2014 1 8 by Seok
__inline	int 	ETRI_getInfiniteProcessing 	(ETRI_Interface* EncoderIF)  			{ return EncoderIF->CTRParam.iInfiniteProcessing; }
__inline	bool	ETRI_ProcessStopCondition 	(ETRI_Interface* EncoderIF)
{
	// True : Stop, False: Run
	bool InfiniteProcess = (ETRI_getInfiniteProcessing(EncoderIF))? false : true;
	bool FundamentalCondition = *EncoderIF->m_piFrameRcvd < EncoderIF->CTRParam.iFramestobeEncoded;

#if ETRI_BUGFIX_DLL_INTERFACE
	if (InfiniteProcess && !FundamentalCondition)
	{
		///< Non Infinite Process Case :: 2016 1 14 by Seok
		(EncoderIF->CTRParam.uiNumofEncodedGOPforME)++;
		if (EncoderIF->CTRParam.uiNumofEncodedGOPforME < (unsigned int)EncoderIF->CTRParam.iNumofGOPforME)
		{
			InfiniteProcess = false;
		}
	}
#endif	

	/// 2014 5 10 by Seok : Debug
	/*ETRI_dbgMsg(ETRI_DBG_ProcessStopCondition, 0, " ===== %s in %s @LINE: %d \n", __FUNCTION__, __FILENAME__, __LINE__);
	ETRI_dbgMsg(ETRI_DBG_ProcessStopCondition, 0, "InfiniteProcess : %s \n", BoolValue(InfiniteProcess));
	ETRI_dbgMsg(ETRI_DBG_ProcessStopCondition, 0, "ETRI_getInfiniteProcessing(EncoderIF) : %s \n", BoolValue(ETRI_getInfiniteProcessing(EncoderIF)));
	ETRI_dbgMsg(ETRI_DBG_ProcessStopCondition, 0, "FundamentalCondition : %s \n", BoolValue(FundamentalCondition));	*/

	return ((FundamentalCondition)? FundamentalCondition : InfiniteProcess);
}

__inline 	void 	ETRI_RefreshEncoder (ETRI_Interface* EncoderIF, bool RefreshStop)
{
	ETRI_setAnalyzeClear(EncoderIF, false);

	if (ETRI_ProcessStopCondition(EncoderIF) || RefreshStop)	return;

	//ETRI_dbgMsg(false, 0, "[LINE:%d %s] %s Refresh ON \n", __LINE__, __FILENAME__, __FUNCTION__);	/// 2014 4 30 by Seok

	EncoderIF->iNumEncoded                    = 0;
	*EncoderIF->m_piFrameRcvd                 = 0;	///ETRI_BUGFIX_DLL_INTERFACE
	*EncoderIF->CTRParam.piPOCLastIdx         = -1;
	*EncoderIF->CTRParam.piNumPicRcvdIdx      = 0;
	*EncoderIF->CTRParam.puiNumAllPicCodedIdx = 0;
	EncoderIF->CTRParam.bAnalyzeClear         = true;
	EncoderIF->bEos                           = false;

//	short sSliceIndex                 		  = EncoderIF->CTRParam.sSliceIndex;

}

bool ETRI_dbgTest(int AvailableEncoder, int IDRLength, int*& dbgParam, int LimitCondition, bool dbgTestOn)
{
	/// Return Value 
	///	False : Go Encoding Process 
	///  True : Stop Encoding Process 

	if (!dbgTestOn)	return false;	
	
	int	_iTestParam = (*dbgParam);
	int	_iLimitParam = LimitCondition * IDRLength * AvailableEncoder;		

	if (_iTestParam >= _iLimitParam)	
	{		
		return true;
	}

	return false;	
}

void Memory_Pool_Constructor (ETRI_Interface* EncoderIF)
{
	/// Input/Output Data allocation for Simulation. : 2014 4 3 by Seok
	ETRI_TESTIO 	  	*Le_TestIO    	= (ETRI_TESTIO *)&e_TestIO;

	Le_TestIO->testInput 	= nullptr;
	Le_TestIO->testOutput	= nullptr;
	
	unsigned int Operation_Frames = BASIC_NUM_BUFFER;
	unsigned int Input_Buffer_Size = MAX_INPUT_BUFFER_SIZE;
	unsigned int Output_Buffer_Size = MAX_ANNEXB_BUFFER_SIZE;
	
	Le_TestIO->i_nTestInput 	= Operation_Frames;
	Le_TestIO->i_nTestOutput 	= Operation_Frames;
	Le_TestIO->ui_OpInputIdx 	= 0;
	Le_TestIO->ui_OpOutputIdx 	= 0;

	if (!EncoderIF->CTRParam.bInBufferOff)
	{
		///When Full RD IO Process Active case 2016 2 1 by Seok
		Int iFramesTobeEncoded 	= EncoderIF->CTRParam.iFramestobeEncoded;
		Int inGOPsforME 		= EncoderIF->CTRParam.iNumofGOPforME;

		Operation_Frames = iFramesTobeEncoded * inGOPsforME;
		Input_Buffer_Size = EncoderIF->FrameSize;

		if (Input_Buffer_Size == 0 || Operation_Frames == 0)
		{
			if (Input_Buffer_Size == 0) 
			EDPRINTF(stderr, "Input Buffer Size for Full IO Processing is invalid !!!\n");
			if (Operation_Frames == 0)
			EDPRINTF(stderr, "Operation_Frames for Full IO Processing is invalid !!!\n");
			EDPRINTF(stderr, "(How to Debug?? : Memory_Pool_Constructor is called after Encoder Initilization \n");
			exit(0);
		}

		Le_TestIO->i_nTestInput = Operation_Frames;

		Le_TestIO->testInput = (void **)_aligned_malloc(Le_TestIO->i_nTestInput * sizeof(void *), 32);
				
		for( unsigned int k = 0; k < Le_TestIO->i_nTestInput; k++ )
		{
			unsigned char *InputBuffer;
			InputBuffer = (unsigned char *)_aligned_malloc(Input_Buffer_Size * sizeof(char), 32);
			DLLNull_ErrorMSG (InputBuffer, "InputBuffer Allocation Fail \n", 0);	
			Le_TestIO->testInput[k] = (void *)InputBuffer;
		}
	}

	if (!EncoderIF->CTRParam.bOutBufferOff)
	{
		///When Full RD IO Process Active case 2016 2 1 by Seok
		Int iFramesTobeEncoded 	= EncoderIF->CTRParam.iFramestobeEncoded;
		Int inGOPsforME 			= EncoderIF->CTRParam.iNumofGOPforME;
		Operation_Frames = iFramesTobeEncoded * inGOPsforME;
		Le_TestIO->i_nTestOutput = Operation_Frames;

		if (Operation_Frames == 0)
		{
			EDPRINTF(stderr, "Operation_Frames for Full IO Processing is invalid !!!\n");
			EDPRINTF(stderr, "(How to Debug?? : Memory_Pool_Constructor is called after Encoder Initilization \n");
			exit(0);
		}

		Le_TestIO->testOutput = (void **)_aligned_malloc(Le_TestIO->i_nTestOutput * sizeof(void *), 32);

		for( unsigned int k = 0; k<Le_TestIO->i_nTestInput; k++ )
		{
			unsigned char *OutputBuffer;
			OutputBuffer = (unsigned char *)_aligned_malloc(Output_Buffer_Size * sizeof(char), 32);
			DLLNull_ErrorMSG (OutputBuffer, "OutputBuffer Allocation Fail \n", 0);	
			Le_TestIO->testOutput[k] = (void *)OutputBuffer;
		}
	}
}

void Memory_Pool_Initialization (std::fstream& IYUVFile, std::fstream& bitstreamFile, ETRI_Interface* EncoderIF)
{
	/// File Open 
	IYUVFile.open(EncoderIF->m_pchInputFile, std::fstream::binary | std::fstream::in);	
	if(IYUVFile.fail()){
		fprintf(stderr, "\nfailed to open Input YUV file\n");	ETRI_EXIT(0);
	}
	
	bitstreamFile.open(EncoderIF->m_pchBitstreamFile, std::fstream::binary | std::fstream::out);	
	if(bitstreamFile.fail()){
		fprintf(stderr, "\nfailed to open Output HEVC stream file\n");	ETRI_EXIT(0);
	}
}

void Memory_Pool_Free (ETRI_Interface* EncoderIF)
{
	ETRI_TESTIO 	*Le_TestIO = (ETRI_TESTIO *)&e_TestIO;
	bool	badPtr = true;

	if (!EncoderIF->CTRParam.bOutBufferOff)
	{
		for( unsigned int k = 0; k < Le_TestIO->i_nTestOutput; k++ )
		{
#if (_ETRI_WINDOWS_APPLICATION)
		badPtr = IsBadReadPtr(Le_TestIO->testOutput[k],(MAX_ANNEXB_BUFFER_SIZE * sizeof(char)));
#else
		badPtr = access((const char *)Le_TestIO->testOutput[k],		F_OK);
#endif

		if (!badPtr) _aligned_free(Le_TestIO->testOutput[k]);
			Le_TestIO->testOutput[k] = nullptr;
	}
	_aligned_free(Le_TestIO->testOutput); 
	}

	if (!EncoderIF->CTRParam.bInBufferOff)
	{
		for( unsigned int k = 0; k < Le_TestIO->i_nTestInput; k++ )
		{
#if (_ETRI_WINDOWS_APPLICATION)
			badPtr = IsBadReadPtr(Le_TestIO->testInput[k],(EncoderIF->FrameSize * sizeof(char)));
#else
			badPtr = access((const char *)Le_TestIO->testInput[k],		F_OK);
#endif

		if (!badPtr) _aligned_free(Le_TestIO->testInput[k]);
			Le_TestIO->testInput[k] = nullptr;
	}
	_aligned_free(Le_TestIO->testInput); 
	}
}

void Memory_Pool_PushInputData(std::fstream& IYUVFile, std::fstream& bitstreamFile, ETRI_Interface* EncoderIF)
{
	if (EncoderIF->CTRParam.iFullIORDProcess == 0 || EncoderIF->CTRParam.bInBufferOff)	{return;}

	ETRI_TESTIO 	  	*Le_TestIO  = (ETRI_TESTIO *)&e_TestIO;

	for(Int iIdx = 0 ; iIdx < 	Le_TestIO->i_nTestInput; iIdx++)
	{
		IYUVFile.read(reinterpret_cast<char*>(Le_TestIO->testInput[iIdx]), EncoderIF->FrameSize);		/// 2014 6 18 by Seok : Revision
		if ((IYUVFile.eof() || IYUVFile.fail()))  error_dll("File Read Fail in Memory_Pool_PushInputData \n", 0);
	}
}

void Memory_Pool_GetFrame(std::fstream& IYUVFile, ETRI_Interface* EncoderIF)
{
	if (EncoderIF->CTRParam.bInBufferOff)
	{
	/// read input YUV file
		IYUVFile.read(reinterpret_cast<char*>(EncoderIF->ptrData), EncoderIF->FrameSize);		/// 2014 6 18 by Seok : Revision
		if ((IYUVFile.eof() || IYUVFile.fail())) 
		{
			///Test 즉, eof는 Encoder내부에서 주어지지 않고 밖에서 주어지는 것으로 한다.  2016 2 19 by Seok
			if (IYUVFile.eof())
			{
				EncoderIF->bEos = true;
				// EDPRINTF(stderr, "End of File \n");
			}
			else
			{
				///IYUVFile.fail()) :  2016 2 19 by Seok
				error_dll("File Read Fail in Memory_Pool_GetFrame\n", 0);
			}	
		}	
	}	
	else
	{
		/// Link Memory Pool Data to Encoder Input
		ETRI_TESTIO *Le_TestIO = (ETRI_TESTIO *)&e_TestIO;

		if (Le_TestIO->ui_OpInputIdx <  Le_TestIO->i_nTestInput)
		EncoderIF->ptrData = (unsigned char*)(Le_TestIO->testInput[Le_TestIO->ui_OpInputIdx]);
		else
		error_dll("File Read Fail in Memory_Pool_GetFrame for Full IO Read \n", 0);	

		Le_TestIO->ui_OpInputIdx++;
	}
}

void Memory_Pool_PutFrame(std::fstream& bitstreamFile, ETRI_Interface* EncoderIF)
{
	/// Write data to a File : 
	/* 
	After Encoding, We can get HEVC Stream (EncoderIF->AnnexBData) and \n
	size of HEVC stream per Frame (EncoderIF->AnnexBFrameSize) 
	*/
	if (EncoderIF->CTRParam.iFullIORDProcess > 0)
	{
		noop;
	}
	else
	{
		bitstreamFile.write((const char *)EncoderIF->AnnexBData, EncoderIF->AnnexBFrameSize);
	}
}
#endif
