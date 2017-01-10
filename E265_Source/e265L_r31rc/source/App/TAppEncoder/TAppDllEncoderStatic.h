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
    @file    TAppDLLEncdoer.h
    @date    2015/06/08
    @author  김연희 (kimyounhee@etri.re.kr) ETRI
    @brief   Header File for TAppDllEncoderMain.cpp
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


#define ETRI_DLL_GOPParallel	1 //same to ETRI_GOP_PARALLEL

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


ETRI_TESTIO  	e_TestIO;		///< Simulation of Memory PooL


// ====================================================================================================================
// ETRI DLL Service Functions 
// ====================================================================================================================
__inline void error_dll (char *text, int code)
{
	fprintf(stderr, "%s Error COde:%d \n", text, GetLastError()); 
	ETRI_EXIT(code);
}
void	ReleaeseOutputPointer(void*& Outptr, bool Condition) { Outptr = (Condition)? Outptr : nullptr;}

void ETRI_dbgMsg(bool dbgCondition, int PrtMethod,  const char *fmt, ...)
{
	if (dbgCondition)
	{
		va_list args;
		int 	len;
		char buf[ERROR_DLL_BUF_SZ];

		memset(buf, 0, ERROR_DLL_BUF_SZ);
		
		va_start(args, fmt);
		len = _vscprintf(fmt, args) + 1;
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

__inline	void 	ETRI_setAnalyzeClear  		(ETRI_Interface* EncoderIF, bool bGoOn)	{ EncoderIF->CTRParam.bAnalyzeClear	= bGoOn; }	// 2014 1 8 by Seok
__inline	int 	ETRI_getInfiniteProcessing 	(ETRI_Interface* EncoderIF)  			{ return EncoderIF->CTRParam.iInfiniteProcessing; }
__inline	bool	ETRI_ProcessStopCondition 	(ETRI_Interface* EncoderIF)
{
	// True : Stop, False: Run
	bool InfiniteProcess = (ETRI_getInfiniteProcessing(EncoderIF))? false : true;
	bool FundamentalCondition = *EncoderIF->m_piFrameRcvd < EncoderIF->CTRParam.iFramestobeEncoded;

	return ((FundamentalCondition)? FundamentalCondition : InfiniteProcess);	
}

__inline 	void 	ETRI_RefreshEncoder (ETRI_Interface* EncoderIF, bool RefreshStop)
{
	ETRI_setAnalyzeClear(EncoderIF, false);

	if (ETRI_ProcessStopCondition(EncoderIF) || RefreshStop)	return;

	int* iNumEncoded                  = &EncoderIF->iNumEncoded;
	int* iPOCLast			 	      = EncoderIF->CTRParam.piPOCLastIdx;
	int* iNumPicRcvdIdx      	      = EncoderIF->CTRParam.piNumPicRcvdIdx;
	unsigned int* uiNumAllPicCodedIdx = EncoderIF->CTRParam.puiNumAllPicCodedIdx;
	bool* bAnalyzeClear			      = &EncoderIF->CTRParam.bAnalyzeClear;
	int* iFrameRcvd                   = EncoderIF->CTRParam.piFrameRcvd;
	short sSliceIndex                 = EncoderIF->CTRParam.sSliceIndex;
	

#if ETRI_DLL_GOPParallel
	*iNumEncoded         = 0;
#endif
	*iPOCLast            = -1;
	*iNumPicRcvdIdx      = 0;
	*uiNumAllPicCodedIdx = 0;
	*iFrameRcvd          = 0;	
	*bAnalyzeClear       = true;
	EncoderIF->bEos      = false;
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
		bool 	_dbgkey = true;
		int  	_dbgType = 2;

		/*ETRI_dbgMsg(_dbgkey, _dbgType, "\n");
		ETRI_dbgMsg(_dbgkey, _dbgType, "[LINE:%d %s %s] _iTestParam(%d) >= _iLimitParam(%d) \n", __LINE__, __FUNCTION__, __FILENAME__, _iTestParam, _iLimitParam);
		ETRI_dbgMsg(_dbgkey, _dbgType, "[LINE:%d %s %s] Forced Stop to Infinite Processing \n", __LINE__, __FUNCTION__, __FILENAME__);*/
		return true;
	}

//	fprintf(stderr, "[LINE: %d %s %s] GOP Index : %d @LimitCondition: %d \n", __LINE__, __FUNCTION__, __FILENAME__, *dbgParam, _iLimitParam);
//	(*dbgParam)++;

	return false;	
}

void Memory_Pool_Constructor (ETRI_Interface* EncoderIF)
{

	/// Input/Output Data allocation for Simulation. : 2014 4 3 by Seok
	ETRI_TESTIO 	  	*Le_TestIO    	= (ETRI_TESTIO *)&e_TestIO;
	
	unsigned int Operation_Frames = BASIC_NUM_BUFFER;
	Le_TestIO->i_nTestInput 	  = Operation_Frames;
	Le_TestIO->i_nTestOutput 	  = Operation_Frames;
	Le_TestIO->ui_OpInputIdx 	  = 0;
	Le_TestIO->ui_OpOutputIdx 	  = 0;

	if (!EncoderIF->CTRParam.bInBufferOff)
	{
		Le_TestIO->testInput = (void **)_aligned_malloc(Le_TestIO->i_nTestInput * sizeof(void *), Operation_Frames);
				
		for( unsigned int k = 0; k < Le_TestIO->i_nTestInput; k++ )
		{
			unsigned char *InputBuffer;
			InputBuffer = (unsigned char *)_aligned_malloc(MAX_INPUT_BUFFER_SIZE * sizeof(char), Operation_Frames);
			DLLNull_ErrorMSG (InputBuffer, "InputBuffer Allocation Fail \n", 0);	
			Le_TestIO->testInput[k] = (void *)InputBuffer;
		}
	}

	if (!EncoderIF->CTRParam.bOutBufferOff)
	{
		Le_TestIO->testOutput = (void **)_aligned_malloc(Le_TestIO->i_nTestOutput * sizeof(void *), Operation_Frames);

		for( unsigned int k = 0; k<Le_TestIO->i_nTestInput; k++ )
		{
			unsigned char *OutputBuffer;
			OutputBuffer = (unsigned char *)_aligned_malloc(MAX_ANNEXB_BUFFER_SIZE * sizeof(char), Operation_Frames);
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

	for( unsigned int k = 0; k < Le_TestIO->i_nTestOutput; k++ )
	{
		badPtr = IsBadReadPtr(Le_TestIO->testOutput[k],(MAX_ANNEXB_BUFFER_SIZE * sizeof(char)));

		if (!badPtr) _aligned_free(Le_TestIO->testOutput[k]);
		Le_TestIO->testOutput[k] = NULL;
	}
	_aligned_free(Le_TestIO->testOutput); 

	/// 2014 5 5 by Seok : Test Model
	for( unsigned int k = 0; k < Le_TestIO->i_nTestInput; k++ )
	{
		badPtr = IsBadReadPtr(Le_TestIO->testInput[k],(MAX_INPUT_BUFFER_SIZE * sizeof(char)));

		if (!badPtr) _aligned_free(Le_TestIO->testInput[k]);
		Le_TestIO->testInput[k] = NULL;
	}
	_aligned_free(Le_TestIO->testInput); 
}


void Memory_Pool_GetFrame(std::fstream& IYUVFile, ETRI_Interface* EncoderIF)
{
	/// read input YUV file
	IYUVFile.read(reinterpret_cast<char*>(EncoderIF->ptrData), EncoderIF->FrameSize);		/// 2014 6 18 by Seok : Revision
	if ((IYUVFile.eof() || IYUVFile.fail()))  error_dll("File Read Fail \n", 0);
}


void Memory_Pool_PutFrame(std::fstream& bitstreamFile, ETRI_Interface* EncoderIF)
{
	/// Write data to a File : 
	/* 
	After Encoding, We can get HEVC Stream (EncoderIF->AnnexBData) and \n
	size of HEVC stream per Frame (EncoderIF->AnnexBFrameSize) 
	*/
	bitstreamFile.write((const char *)EncoderIF->AnnexBData, EncoderIF->AnnexBFrameSize);			
}
#endif
