
#ifndef HEVC_HTHREAD_H
#define HEVC_HTHREAD_H

#include <list>
#include "../inc/hevc_base.h"

#ifdef HEVC_WINDOWS

//#include <windows.h>

namespace hevc {
namespace workflow {

#ifndef DWORD
typedef unsigned long DWORD;
#endif // DWORD

class HEVC_BASE_API CThread
{
public:
#ifdef _DEBUG
	CThread( const char* szFunction );
	CThread( const wchar_t* szFunction );
#endif
	CThread(void);
	virtual ~CThread(void);
	virtual bool	_Start();
	virtual bool	_Start_Suspended();
	virtual void	_Stop();
	virtual void	_StopWait();
	bool	_SetPriority(int);
	bool	_Suspend();
	bool	_Resume();
	bool	GetExitCode(DWORD*);
	bool	IsThreadRunning();
	virtual bool	PreStart()	{ return true; };
	virtual void	PostStart() {};
	virtual void	PreStop()	{};
	virtual void	PostStop()	{};

	bool	Terminate();
	u32		GetThreadId()		{ return m_nThreadId; };
	bool	Init_Succeeded()	{ return m_bPrepareStart; }; 

	HANDLE	GetSafeHandle();
protected:
	virtual DWORD	_Run() = 0;
	static	DWORD __stdcall ThreadStartPoint( void* pParam );

	HANDLE	m_hThread;
	DWORD	m_nThreadId;

	volatile bool	m_bStop;
	volatile bool	m_bPrepareStart;
#ifdef _DEBUG
	char*	m_szThreadName;
#endif
private:
	volatile bool	m_bBeing;
};


#ifdef _DEBUG
#define DEBUG_THREAD_CREATOR		hevc::workflow::CThread(__FUNCTION__)
#else
#define DEBUG_THREAD_CREATOR		hevc::workflow::CThread()
#endif

} // namespace workflow
} // namespace hevc

#else // HEVC_WINDOWS 
// code here for linux
#endif // HEVC_WINDOWS

#endif // HEVC_HTHREAD_H
