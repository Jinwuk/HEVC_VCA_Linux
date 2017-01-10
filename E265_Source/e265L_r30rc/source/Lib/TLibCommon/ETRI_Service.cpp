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

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <emmintrin.h>
#include <tchar.h>

#ifdef _USRDLL
#include "..\..\App\TAppEncoder\TDllEncoder.h"
#endif
#include "ETRI_HEVC_Define.h"

void 	ETRI_Service_ReportError(LPCSTR UserMessage, DWORD ExitCode, bool PrintErrorMsg);
bool 	PrintStrings (HANDLE hOut, ...);
bool 	PrintMsg (HANDLE hOut, LPCTSTR pMsg);

#if defined(WIN32)
HANDLE			DFHandle;
#endif
int				DFP;		//File Handle for Realtime Debugging
ETRIParameters 	ETRIParam;

void DPRINTF(char *fmt, ...)
{
#if defined(_FILE_DEBUG)
#if defined(_DEBUG) || defined(_REALTIME_DEBUG)
#if (SERIAL_TEST || _REALTIME_DEBUG)
	ETRIParameters *pHEVC_param = &ETRIParam;
	if (pHEVC_param->FirstDebuggingFileOpen){
		pHEVC_param->FirstDebuggingFileOpen = false;
		DCLOSE(DFP);
	}

	DFP = DOPEN(pHEVC_param->DebugFileName, OPENFLAGS_REWRITE , OPEN_PERMISSIONS);
	
	if( DFP == -1 ){
	   	perror( "Open failed on Debug file in DPRINTF" );
	}
#endif
#endif
	va_list args;
	int 	len;
	char buf[ERROR_DLL_BUF_SZ];

	va_start(args, fmt);
	len = _vscprintf(fmt, args) + 1;
	vsprintf_s(buf, len, fmt, args);
	va_end(args);

	DWRITE(DFP, buf, len);

#if defined(_DEBUG) || defined(_REALTIME_DEBUG)
#if (SERIAL_TEST || _REALTIME_DEBUG)
	DCLOSE(DFP);
#endif
#endif
#endif
}


void THDPRINTF(char *fmt, ...)
{

#if defined(_FILE_DEBUG)
#if defined(WIN32)
	va_list args;
	DWORD MsgLen;
	char pMsg[ERROR_DLL_BUF_SZ];

	va_start(args, fmt);
	MsgLen = _vscprintf(fmt, args) + 1;
	vsprintf_s(pMsg, MsgLen, fmt, args);
	va_end(args);

	PrintMsg (GetStdHandle (STD_ERROR_HANDLE), (LPCTSTR)pMsg);


//	WriteFile(DFHandle, (LPCTSTR)pMsg, MsgLen,(LPDWORD)&Count, NULL);
#else

#endif

#endif
}

void SSERegPRINTF(__m128i a, int	 ssetype, char *fmt, ...)
{
	int	i;

	static	ALIGNED(16) Int8		bytedata[16];
	static	ALIGNED(16) Int16		shortdata[8];
	static	ALIGNED(16) Int32		worddata[4];
	static	ALIGNED(16) Int64		lworddata[2];

	switch(ssetype)
	{
		case Int8Align:	_mm_store_si128((__m128i *)&bytedata[0], a); 
						DPRINTF(fmt); 
						for(i=0; i<16; i++)
							DPRINTF("hex : %x dec : %d \n", bytedata[i], bytedata[i]);  
						break;
		case Int16Align:	_mm_store_si128((__m128i *)&shortdata[0], a); 
						DPRINTF(fmt); 
						for(i=0; i<8; i++)
							DPRINTF("hex : %x dec : %d \n", shortdata[i], shortdata[i]);  
						break;
		case Int32Align:	_mm_store_si128((__m128i *)&worddata[0], a); 
						DPRINTF(fmt); 
						for(i=0; i<4; i++)
							DPRINTF("hex : %x dec : %d \n", worddata[i], worddata[i]);  
						break;
		case Int64Align: _mm_store_si128((__m128i *)&lworddata[0], a); 
						DPRINTF(fmt); 
						for(i=0; i<2; i++)
							DPRINTF("hex : %x dec : %d \n", lworddata[i], lworddata[i]);  
						break;
		default: break;
	}
}

void MultiThreadDCLOSE_REOPEN(void)
{
#if defined(_FILE_DEBUG)
#if defined(_DEBUG) || defined(_REALTIME_DEBUG)
#if !SERIAL_TEST
	ETRIParameters *pHEVC_param = &ETRIParam;
	DCLOSE(DFP);

	DFP = DOPEN(pHEVC_param->DebugFileName, OPENFLAGS_REWRITE , OPEN_PERMISSIONS);
	if( DFP == -1 ){
		perror( "Open failed on Debug file in DPRINTF" );
	}

#endif
#endif
#endif
}

double	ETRI_SetHighFrequencyTimer(ETRIParameters* pHEVC_param)
{
	double tot_time = 0.0;
	pHEVC_param->prev_time = pHEVC_param->current_time = 0;
	pHEVC_param->freq = pHEVC_param->start_time = 0;

	if (pHEVC_param->HighResolutionClockExist = (int)QueryPerformanceFrequency((LARGE_INTEGER*)&pHEVC_param->freq)) 
		QueryPerformanceCounter((LARGE_INTEGER*)&pHEVC_param->start_time);

	pHEVC_param->current_time = pHEVC_param->start_time;
	tot_time = (double)pHEVC_param->freq;
	return tot_time;
}

double	ETRI_GetCurrentTimeEx(ETRIParameters* pHEVC_param)
{
	double  	tot_time = 0.0;
	Int64 	freq = pHEVC_param->freq;
	int	HighResolutionClockExist = pHEVC_param->HighResolutionClockExist;

	pHEVC_param->prev_time = pHEVC_param->current_time;
	if (HighResolutionClockExist)
		QueryPerformanceCounter((LARGE_INTEGER*)&pHEVC_param->current_time);
	tot_time = ((double)(pHEVC_param->current_time - pHEVC_param->start_time)/(double)freq);
	return tot_time;
}

double	ETRI_GetCurrentTime(Int64 *Current)
{
	ETRIParameters *pHEVC_param = &ETRIParam;
	double tot_time = 0.0;
	Int64 					freq = pHEVC_param->freq;
	int	HighResolutionClockExist = pHEVC_param->HighResolutionClockExist;

	pHEVC_param->prev_time = pHEVC_param->current_time;
	if (HighResolutionClockExist)
		QueryPerformanceCounter((LARGE_INTEGER*)Current);
	pHEVC_param->current_time = *Current;

	tot_time = ((double)(pHEVC_param->current_time - pHEVC_param->start_time)/(double)freq);
	return tot_time;
}

double	ETRI_GetInsideTime(Int64 *Current)
{
	ETRIParameters *pHEVC_param = &ETRIParam;
	double tot_time = 0.0;
	Int64 					freq = pHEVC_param->freq;
	int	HighResolutionClockExist = pHEVC_param->HighResolutionClockExist;

	pHEVC_param->prev_time = pHEVC_param->current_time;
	if (HighResolutionClockExist)
		QueryPerformanceCounter((LARGE_INTEGER*)Current);
	pHEVC_param->current_time = *Current;

	tot_time = ((double)(pHEVC_param->current_time - pHEVC_param->prev_time)/(double)freq);
	return tot_time;
}

#pragma warning(push)
#pragma warning(disable:4789)
void		ETRI_BreakPoint4ReleaseMode(void)
{
	int	DummyINTData[2];

	DummyINTData[2] = 567;

/*
	What is the best code for hacking to an exe file??
*/
}
#pragma warning(pop)

#ifndef	ETRI_IDX_dbgMsg
/*
=======================================================================================
	@brief: 기존 debug 메시지 출력 방식을 개선하기 위한 기본 함수 WHERESTR, WHEREARG와 혼용하고 더 나은 DEBUG 방식과 결합 가능하다.
	@param: bool dbgCondition : 출력 여부 결정
	@param: int PrtMethod : Print 방식 결정,  
	@param:char *fmt, ...  : Print Format  
=======================================================================================
*/
void 	ETRI_dbgMsg(bool dbgCondition, int PrtMethod,  const char *fmt, ...)
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
#endif

void ETRI_DebugFile_Init(void)
{
#if defined(_FILE_DEBUG)
	ETRIParameters *pHEVC_param = &ETRIParam;

	if (pHEVC_param->FirstDebuggingFileOpen) return;

	sprintf(pHEVC_param->DebugFileName, "Debug.txt");
	DFP = DOPEN(pHEVC_param->DebugFileName, OPENFLAGS_WRITE, OPEN_PERMISSIONS);
	
	if( DFP == -1 ){
		pHEVC_param->FirstDebuggingFileOpen = false;
	   	perror( "Open failed on Debug file" );
	}
	else
	{
		bool _dbgkey = false;
		int _dbgStyle = 1;
				
		ETRI_dbgMsg(_dbgkey, _dbgStyle,   "Open succeeded on Debug file : %s \n", pHEVC_param->DebugFileName);
		pHEVC_param->FirstDebuggingFileOpen = true;
		DPRINTF("Debug @ %s  %s \n", __DATE__, __TIME__);
	}
#endif	//#if defined(_FILE_DEBUG)

}

#define ETRI_DBG_ETRI_Service_Init	(0 & ETRI_TDLLDBGMSG)

void	ETRI_Service_Init(int ApplicationNameIdx)
{
	ETRIParameters *pHEVC_param = &ETRIParam;

	bool _dbgkey = ETRI_DBG_ETRI_Service_Init; 
	int _dbgStyle = 2;
 
	DFP = 0;
	ETRI_SetHighFrequencyTimer(pHEVC_param);

	ETRI_dbgMsg(_dbgkey, _dbgStyle, "\n\n\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t###################################################\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#                                                 #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#      ETRI HEVC %s VER %s #\n", ((ApplicationNameIdx == 0)? "Encoder" : "Decoder"), EHEVC_VERSION);
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#                                                 #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t# Contributors :                                  #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#   Jinwuk seok     (jnwseok@etri.re.kr)          #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#   Dongsan Jeon    (dschun@etri.re.kr)           #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#   Younhee Kim     (kimyounhee@etri.re.kr)       #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#   Soon-heung Jung (zeroone@etri.re.kr)          #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#   Jongho Kim      (pooney@etri.re.kr)           #\n");

#if defined(_DEBUG)
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#                                Debug Mode       #\n");
#else
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#                              Release Mode       #\n");
#endif
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#                           Copyright by ETRI     #\n");
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t#    Compiled @    %s  Time : %s   #\n",  __DATE__, __TIME__);
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "\t###################################################\n\n");

	ETRI_DebugFile_Init();

	if (pHEVC_param->HighResolutionClockExist)
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "High Frequency Timer Set with %f \n",  pHEVC_param->freq);
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "%s \n", TEST_msg);

#ifdef _USRDLL
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "DLL Active \n");
#endif
	//2013 5 29 by Seok
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "ETRI_DLL_INTERFACE : %s   ETRI_INTERFACE_HEADER : %s\n", ((ETRI_DLL_INTERFACE)? "On":"OFF"), ((ETRI_INTERFACE_HEADER)? "On":"Off"));

}

void ETRI_Service_End(void)
{
	ETRIParameters *pHEVC_param = &ETRIParam;
	double tot_time = 0.0;

	bool _dbgkey = false; int _dbgStyle = 2;

	if (pHEVC_param->HighResolutionClockExist)
		QueryPerformanceCounter((LARGE_INTEGER*)&pHEVC_param->finish_time);
	tot_time = ((double)(pHEVC_param->finish_time - pHEVC_param->start_time)/(double)pHEVC_param->freq);
	ETRI_dbgMsg(_dbgkey, _dbgStyle,  "Total Processing Time with HFC : %10.4f \n", tot_time);


#if defined(_FILE_DEBUG)
	if (pHEVC_param->FirstDebuggingFileOpen)
		DCLOSE( DFP );
#endif	

}

bool PrintStrings (HANDLE hOut, ...)
{
	DWORD MsgLen, Count;
	LPCTSTR pMsg;
	va_list pMsgList;	/* Current message string. */
	va_start (pMsgList, hOut);	/* Start processing msgs. */
	while ((pMsg = va_arg (pMsgList, LPCTSTR)) != NULL) {
		MsgLen = lstrlen (pMsg);
		if (!WriteConsole (hOut, pMsg, MsgLen, &Count, NULL)
				&& !WriteFile (hOut, pMsg, MsgLen * sizeof (TCHAR),
				&Count, NULL))
			return FALSE;
	}
	va_end (pMsgList);
	return TRUE;
}

bool PrintMsg (HANDLE hOut, LPCTSTR pMsg)
{
	return PrintStrings (hOut, pMsg, NULL);
}


void ETRI_Service_ReportError(LPCSTR UserMessage, DWORD ExitCode, bool PrintErrorMsg)
{
	DWORD eMsgLen, ErrNum = GetLastError ();
	LPTSTR lpvSysMsg;
	HANDLE hStdErr;

	hStdErr = GetStdHandle (STD_ERROR_HANDLE);
	PrintMsg (hStdErr, (LPCTSTR)UserMessage);
	if (PrintErrorMsg) {
		eMsgLen = FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM, NULL,
		ErrNum, MAKELANGID (LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR) &lpvSysMsg, 0, NULL);
		
		PrintStrings (hStdErr, TEXT ("\n"), lpvSysMsg, TEXT ("\n"), NULL);
		HeapFree (GetProcessHeap (), 0, lpvSysMsg);
	}
	
	if (ExitCode > 0)
		ExitProcess (ExitCode);
	else
		return;
}

bool ETRI_SIMD16_Check(bool Condition, Int16 *dst, Int16 *est, bool FileOrTerminal)
{
	int 	i; 
	bool	FaultOccur;


	for(FaultOccur=false, i=0; i<8; i++)
		FaultOccur = (dst[i] != est[i])? true : FaultOccur;

	if (FaultOccur || Condition)
	{

		if (FileOrTerminal){DPRINTF("Destination \n");} else {printf("Destination \n");}	
		for(i=0; i<8; i++){
			if (FileOrTerminal) {DPRINTF("[%3d]  ", 7-i);}	else{printf("[%3d]  ", 7-i);}
		}
		if (FileOrTerminal) {DPRINTF("\n");} else {printf("\n");}	

		for(i=0; i<8; i++){
			if (FileOrTerminal) {DPRINTF(" %3d   ", dst[7-i]);}	else{printf(" %3d   ", dst[7-i]);}	
		}


		if (FileOrTerminal){DPRINTF("\n Estimation \n");}	else{printf("\n Estimation \n");}	
		for(i=0; i<8; i++){
			if (FileOrTerminal) {DPRINTF("[%3d]  ", 7-i);}	else{printf("[%3d]  ", 7-i);}
		}
		if (FileOrTerminal) {DPRINTF("\n");} else {printf("\n");}	

		for(i=0; i<8; i++){
			if (FileOrTerminal) {DPRINTF(" %3d   ", est[7-i]);}	else{printf(" %3d   ", est[7-i]);} 
		}
		if (FileOrTerminal) {DPRINTF("\n");} else {printf("\n");}	
	}

	return FaultOccur;	

}

// ====================================================================================================================
// ETRI_Hardware Service
// ====================================================================================================================



