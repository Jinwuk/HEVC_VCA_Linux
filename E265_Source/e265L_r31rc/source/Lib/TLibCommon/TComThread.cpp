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
	\file   	TComThread.h
   	\brief    	Thread class (header)
*/


#include "TComThread.h"

//! \ingroup TComLibrary
//! \{

TComThread::TComThread(void)
{
	m_hThread = NULL;
	m_bBeing = false;
	m_szThreadName = NULL;
	m_bUseProcessGroup = false;
}

TComThread::~TComThread(void)
{

}

void	TComThread::SetProcessGroupOption( unsigned short nGroup, UInt64 nAffinityMask )
{

}

bool	TComThread::GetAffinity( unsigned short * pusGroup, UInt64* pullAffinityMask )
{
	return true;
}

DWORD __stdcall TComThread::ThreadStartPoint(void* pParam)
{
	DWORD dwRet = 0;
	return dwRet;
}

///< start 시 PreStart() 함수 실패시 시작하지 않는 방식으로 변경.
bool	TComThread::_Start()
{

	return false;
}

bool	TComThread::_Start_Suspended()
{


	return true;
}

void	TComThread::_Stop()
{

}

void	TComThread::_StopWait()
{


}

bool	TComThread::_SetPriority(int nPriority)
{
	return true;
}

bool	TComThread::_Suspend()
{
	return true;
}

bool	TComThread::_Resume()
{
	return true;
}

bool	TComThread::GetExitCode(DWORD* pdwExitCode)
{
	return true;
}

bool	TComThread::IsThreadRunning()
{
	return m_bBeing;
}

bool	TComThread::Terminate()
{
	return true;
}

HANDLE	TComThread::GetSafeHandle()
{
	return m_hThread;
}

const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{


} THREADNAME_INFO;
#pragma pack(pop)


void TComThread::SetThreadName(const char* szThreadName)
{


}

//! \}
