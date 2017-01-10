

/****************************************************************************/
#if !defined(HEVC_SYNCHRONIZER_H)
#define HEVC_SYNCHRONIZER_H

#include "./hevc_base.h"

namespace hevc {
namespace util {

#ifdef HEVC_WINDOWS

class HEVC_BASE_API CSyncLock
{
	friend class CSynchronizer;
	friend class CSynchronizerEx;
public:
	CSyncLock()
	{
		InitializeCriticalSection(&m_cs);
	}

	~CSyncLock()
	{
		DeleteCriticalSection(&m_cs);
	}
	operator LPCRITICAL_SECTION() 
	{
		return &m_cs;
	}
	LPCRITICAL_SECTION operator &() 
	{
		return &m_cs;
	}
private:
	CRITICAL_SECTION	m_cs;
};

class HEVC_BASE_API CSynchronizer  
{
public:
	CSynchronizer(LPCRITICAL_SECTION pCs)
	{
		EnterCriticalSection(pCs);
		m_pCs = pCs;
	};
	CSynchronizer(CSyncLock *pSyncLock)
	{
		EnterCriticalSection(&pSyncLock->m_cs);
		m_pCs = &pSyncLock->m_cs;
	}
	~CSynchronizer()
	{
		LeaveCriticalSection(m_pCs);
	};
private:
	LPCRITICAL_SECTION	m_pCs;
};


class HEVC_BASE_API CSynchronizerEx
{
public:
	CSynchronizerEx(LPCRITICAL_SECTION pCs, bool bLock)
	{
		if (bLock)
		{
			EnterCriticalSection(pCs);
		}
		else
		{
			LeaveCriticalSection(pCs);
		}
	}
	CSynchronizerEx(CSyncLock *pSyncLock, bool bLock)
	{
		if (bLock)
		{
			EnterCriticalSection(&pSyncLock->m_cs);
		}
		else
		{
			LeaveCriticalSection(&pSyncLock->m_cs);
		}
	}
};

class HEVC_BASE_API CSignal
{
public:
	CSignal()
	{
		m_hEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
	}
	~CSignal()
	{
		if ( 0 != m_hEvent )
		{
			CloseHandle( m_hEvent );
			m_hEvent = 0;
		}
	}

	void Set()
	{
		SetEvent( m_hEvent );
	}

	DWORD Wait( u32 dwWaitMs = 0xFFFFFFFF )
	{
		return WaitForSingleObject( m_hEvent, dwWaitMs );
	}

	operator const HANDLE() const
	{
		return m_hEvent;
	}

private:
	HANDLE m_hEvent;
};

class HEVC_BASE_API	CSignalArray
{
public:
	CSignalArray( u32 nSignalCount )
	{
		m_pSignals = new CSignal[nSignalCount];
		m_pHandles = new HANDLE[nSignalCount];

		for ( u32 ii = 0; ii < nSignalCount; ii++ )
		{
			m_pHandles[ii] = m_pSignals[ii];
		}
	}
	~CSignalArray()
	{
		if (m_pSignals) delete [] m_pSignals;
		if (m_pHandles) delete [] m_pHandles;
	}
	DWORD Wait( BOOL bWaitAll, u32 dwWaitMs )
	{
		return WaitForMultipleObjects( m_nSignalCounts, m_pHandles, bWaitAll, dwWaitMs );
	}
private:
	CSignal*	m_pSignals;
	u32			m_nSignalCounts;
	HANDLE		*m_pHandles;
};

#else

#endif

#define SYNC(cs)	CSynchronizer __tmpSync(&cs)
#define SYNC1(cs)	CSynchronizer __tmpSync1(&cs)
#define SYNC2(cs)	CSynchronizer __tmpSync2(&cs)
#define SYNC3(cs)	CSynchronizer __tmpSync3(&cs)
#define SYNC4(cs)	CSynchronizer __tmpSync4(&cs)
#define SYNCP(pcs)	CSynchronizer __tmpSync(pcs)
#define SYNCP1(pcs)	CSynchronizer __tmpSync1(pcs)
#define SYNCP2(pcs)	CSynchronizer __tmpSync2(pcs)
#define SYNCP3(pcs)	CSynchronizer __tmpSync3(pcs)
#define SYNCP4(pcs)	CSynchronizer __tmpSync4(pcs)

#define SYNC_JUST_SYNC(cs)			CSynchronizerEx __tmpSync(&cs, true)
#define SYNC_JUST_RELEASE(cs)		CSynchronizerEx __tmpSync(&cs, false)

}; // namespace Util
}; // namespace Framework

#endif // !defined(HEVC_SYNCHRONIZER_H)
