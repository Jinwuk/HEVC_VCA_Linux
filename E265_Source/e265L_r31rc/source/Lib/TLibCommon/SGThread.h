#pragma once

#include <windows.h>

class SGThread
{
public:
	SGThread(void);
	virtual ~SGThread(void);
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

	void	SetProcessGroupOption( unsigned short nGroup, unsigned __int64 nAffinityMask );
	bool	GetAffinity( unsigned short * pusGroup, unsigned __int64* pullAffinityMask );

	bool	Terminate();
	DWORD	GetThreadId()		{ return m_nThreadId; };
	bool	Init_Succeeded()	{ return m_bPrepareStart; }; 

	HANDLE	GetSafeHandle();
protected:
	virtual DWORD	_Run() = 0;
	static	DWORD __stdcall ThreadStartPoint( void* pParam );
	void	SetThreadName(const char* szThreadName);

	HANDLE	m_hThread;
	DWORD	m_nThreadId;

	bool	m_bUseProcessGroup;
	unsigned short		m_usProcessGroup;
	unsigned __int64	m_ullAffinityMask;

	volatile bool	m_bStop;
	volatile bool	m_bPrepareStart;
	char*	m_szThreadName;
private:
	volatile bool	m_bBeing;
};

#ifdef _DEBUG
#define DEBUG_THREAD_CREATOR		hevc::workflow::CThread(__FUNCTION__)
#else
#define DEBUG_THREAD_CREATOR		hevc::workflow::CThread()
#endif

