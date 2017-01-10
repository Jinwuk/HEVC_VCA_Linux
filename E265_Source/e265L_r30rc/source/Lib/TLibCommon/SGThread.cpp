#include "SGThread.h"

SGThread::SGThread(void)
{
	m_hThread = NULL;
	m_bBeing = false;
	m_szThreadName = NULL;
	m_bUseProcessGroup = false;
}

SGThread::~SGThread(void)
{
	if (WaitForSingleObject(m_hThread,10) == WAIT_TIMEOUT)
	{
		_Stop();
	}

	if (m_szThreadName)
		delete [] m_szThreadName;
}

void	SGThread::SetProcessGroupOption( unsigned short nGroup, unsigned __int64 nAffinityMask )
{
	m_bUseProcessGroup = true;

	m_usProcessGroup = nGroup;
	m_ullAffinityMask = nAffinityMask;
}

bool	SGThread::GetAffinity( unsigned short * pusGroup, unsigned __int64* pullAffinityMask )
{
	if ( ! IsThreadRunning() ) return false;
	if ( !m_bUseProcessGroup ) return false;
	if ( nullptr == pusGroup || nullptr == pullAffinityMask ) return false;

	GROUP_AFFINITY ga;
	if ( ! GetThreadGroupAffinity( m_hThread, &ga) )
	{
		return false;
	}

	*pusGroup = ga.Group;
	*pullAffinityMask = ga.Mask;

	return true;
}

DWORD __stdcall SGThread::ThreadStartPoint(void* pParam)
{
	((SGThread*)pParam)->m_bBeing = true;
	SGThread* pTemp = ((SGThread*)pParam);
#ifdef _DEBUG
	//	char szMsg[512];
	if (((SGThread*)pParam)->m_szThreadName)
	{
		//		sprintf_s(szMsg, "[Thread:id-0x%X]::%s::_Run() in\n", ((CThread*)pParam)->m_nThreadId, ((CThread*)pParam)->m_szThreadName);
		//		OutputDebugStringA(szMsg);
	}
#endif
	DWORD dwRet = ((SGThread*)pParam)->_Run();
	((SGThread*)pParam)->m_bBeing = false;
#ifdef _DEBUG
	{
		//		sprintf_s(szMsg, "[Thread:id-0x%X]::%s::_Run out()\n", ((CThread*)pParam)->m_nThreadId, ((CThread*)pParam)->m_szThreadName);
		//		OutputDebugStringA(szMsg);
	}
#endif
	return dwRet;
}

// wsseo@2013-12-14. start 시 PreStart() 함수 실패시 시작하지 않는 방식으로 변경.
bool	SGThread::_Start()
{
	if (m_hThread)
		_Stop();

	m_bStop = false;

	m_bPrepareStart = PreStart();

	if ( m_bPrepareStart )
	{
		m_hThread = CreateThread(NULL, 0, ThreadStartPoint, this, 0, &m_nThreadId);
		if (m_hThread == NULL){
			m_bStop = true;
			return false;
		}

		if ( m_bUseProcessGroup )
		{
			GROUP_AFFINITY ga;
			memset(&ga, 0, sizeof(ga));
			ga.Group = m_usProcessGroup;
			ga.Mask = m_ullAffinityMask;
			if ( ! SetThreadGroupAffinity( m_hThread, &ga, NULL ) )
			{
			}
		}

		if ( m_szThreadName ) SetThreadName(m_szThreadName);

		PostStart();

		return true;
	}
	else
		return false;
}

bool	SGThread::_Start_Suspended()
{
	if (m_hThread)
		_Stop();

	m_bStop = false;

	m_bPrepareStart = PreStart();

	m_hThread = CreateThread(NULL, 0, ThreadStartPoint, this, CREATE_SUSPENDED, &m_nThreadId);
	if (m_hThread == NULL)
	{
		m_bStop = true;
		return false;
	}

	if ( m_szThreadName ) SetThreadName(m_szThreadName);

	PostStart();

	return true;
}

void	SGThread::_Stop()
{
	PreStop();

	m_bStop = true;
	MSG message;
	while (WaitForSingleObject(m_hThread, 0) == WAIT_TIMEOUT)
	{
		if(::PeekMessage(&message, 0, 0, 0, PM_REMOVE)) {
			::TranslateMessage(&message);
			::DispatchMessage(&message);
		}
	}

	CloseHandle(m_hThread);

	m_hThread = NULL;

#ifdef _DEBUG
	if (m_szThreadName)
	{
// 		char szMsg[512];
// 		sprintf_s(szMsg, "[Thread]::%s::_Stop()\n", m_szThreadName);
// 		OutputDebugStringA(szMsg);
	}
#endif
	PostStop();
}

void	SGThread::_StopWait()
{
	PreStop();

	m_bStop = true;

	WaitForSingleObject(m_hThread, INFINITE);

	CloseHandle(m_hThread);

	m_hThread = NULL;

#ifdef _DEBUG
// 	if (m_szThreadName)
// 	{
// 		char szMsg[512];
// 		sprintf_s(szMsg, "[Thread]::%s::_Stop()\n", m_szThreadName);
// 		OutputDebugStringA(szMsg);
// 	}
#endif
	PostStop();
}

bool	SGThread::_SetPriority(int nPriority)
{
	return (SetThreadPriority(m_hThread, nPriority) == TRUE);
}

bool	SGThread::_Suspend()
{
	return (SuspendThread(m_hThread) == TRUE);
}

bool	SGThread::_Resume()
{
	return (ResumeThread(m_hThread) == TRUE);
}

bool	SGThread::GetExitCode(DWORD* pdwExitCode)
{
	return (GetExitCodeThread(m_hThread, pdwExitCode) == TRUE);
}

bool	SGThread::IsThreadRunning()
{
	return m_bBeing;
}

bool	SGThread::Terminate()
{
	return TerminateThread(m_hThread, 0xFFFFFFFF) == TRUE;
}

HANDLE	SGThread::GetSafeHandle()
{
	if (!IsThreadRunning()) return NULL;
	return m_hThread;
}

const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
	DWORD dwType;   // Must be 0x1000.
	LPCSTR szName;  // Pointer to name (in user addr space).
	DWORD dwThreadID;// Thread ID (-1=caller thread).
	DWORD dwFlags;   // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)


void SGThread::SetThreadName(const char* szThreadName)
{
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = szThreadName;
	info.dwThreadID = m_nThreadId;
	info.dwFlags = 0;

	__try
	{
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
	}
}