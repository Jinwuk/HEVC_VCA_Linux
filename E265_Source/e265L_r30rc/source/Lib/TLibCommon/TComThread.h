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
   	\brief    	ThreadPool class (header)
   	\date 	2015 5 26 
*/

#ifndef __TCOMTHREAD__
#define __TCOMTHREAD__

// Include files
#include "TLibCommon/CommonDef.h"
#if (_ETRI_WINDOWS_APPLICATION)
#include <windows.h>
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TComThread
{
	
public:
	TComThread();
	virtual ~TComThread();
	// -------------------------------------------------------------------------------------------------------------------
	// Basic Operation
	// -------------------------------------------------------------------------------------------------------------------
	virtual bool	_Start();
	virtual bool	_Start_Suspended();
	virtual void	_Stop();
	virtual void	_StopWait();

	bool	_SetPriority(int);
	bool	_Suspend();
	bool	_Resume();
	bool	GetExitCode(DWORD*);
	bool	IsThreadRunning();

	// -------------------------------------------------------------------------------------------------------------------
	// Pre-Operation
	// -------------------------------------------------------------------------------------------------------------------
	virtual bool	PreStart()	{ return true; };
	virtual void	PostStart() {};
	virtual void	PreStop()	{};
	virtual void	PostStop()	{};

	void	SetProcessGroupOption( unsigned short nGroup, UInt64 nAffinityMask );
	bool	GetAffinity( unsigned short * pusGroup, UInt64* pullAffinityMask );

	bool	Terminate();
	DWORD	GetThreadId()		{ return m_nThreadId; };
	bool	Init_Succeeded()	{ return m_bPrepareStart; }; 

	HANDLE	GetSafeHandle();

protected:
	virtual	DWORD	_Run() = 0;
	static 	DWORD __stdcall ThreadStartPoint( void* pParam );
	void	SetThreadName(const char* szThreadName);

	HANDLE	m_hThread;
	DWORD	m_nThreadId;

	bool	   			m_bUseProcessGroup;
	unsigned short		m_usProcessGroup;
	UInt64				m_ullAffinityMask;

	volatile bool	m_bStop;
	volatile bool	m_bPrepareStart;
	char*			m_szThreadName;

private:
	volatile bool	m_bBeing;


};
//! \}
#endif
