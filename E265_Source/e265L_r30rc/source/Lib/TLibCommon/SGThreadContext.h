/**
----------------------------------------------------------------------------------
@file		SGThreadContext.h
@date		2015-04-06
@author		wsseo
@brief		
@endcode		
----------------------------------------------------------------------------------
@section MODIFYINFO 수정내용
- 2015-04-06, wsseo: 
@endcode
*/
#ifndef _SG_THREAD_CONTEXT_H_INCLUDED_
#define _SG_THREAD_CONTEXT_H_INCLUDED_

#include "ETRI_HEVC_define.h"
#include <windows.h>
#if SG_THREAD_POOLING
typedef struct {
	HANDLE	hThread;
	HANDLE	hStartEvent;
	HANDLE	hFinishEvent;
	DWORD	dwThreadId;
	volatile bool bTerminate;

	void*	pMtEncInfoTop;
} sg_thread_ctx_t, *psg_thread_ctx_t;

void		SG_SetProcessOption( bool bUseProcGroup, USHORT usProcessGroup, ULONGLONG ullAffinityMask );
bool		SG_UsingProcGroup();
USHORT		SG_GetProcessGroup();
ULONGLONG	SG_GetAffinityMask();
void		SG_SetThreadAffinity(HANDLE hThread);
#endif // SG_THREAD_POOLING

#endif // _SG_THREAD_CONTEXT_H_INCLUDED_