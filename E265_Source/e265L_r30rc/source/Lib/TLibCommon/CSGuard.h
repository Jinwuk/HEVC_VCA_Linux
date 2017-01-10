#pragma once

#include <windows.h>

class CSGuard
{
public:
	CSGuard(void)
	{
		InitializeCriticalSection(&m_cs);
		::EnterCriticalSection(&m_cs);
	}
	~CSGuard(void)
	{
		::LeaveCriticalSection(&m_cs);
		DeleteCriticalSection(&m_cs);
	}

private:
	CRITICAL_SECTION m_cs;
};