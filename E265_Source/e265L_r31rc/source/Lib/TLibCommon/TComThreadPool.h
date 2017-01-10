#ifndef __TCOMTHREADPOOL__
#define __TCOMTHREADPOOL__

// Include files
#include "TLibCommon/CommonDef.h"
#if (_ETRI_WINDOWS_APPLICATION)
#include <windows.h>
#else
#include <pthread.h>
#endif

//! \ingroup TLibEncoder
//! \{

#define MAX_THREAD_NUM		64

#if !(_ETRI_WINDOWS_APPLICATION)
#define WORKEVENT_L			1
#define SHUTDOWNEVENT_L		2

struct event_flag
{
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	unsigned int flag;
};
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class TComThread
{
	
public:
	TComThread();
	virtual ~TComThread();

private:
#if (_ETRI_WINDOWS_APPLICATION)
	DWORD	m_ThreadID;			// ID of thread
	HANDLE	m_hThread;			// Handle to thread
	HANDLE	m_hWorkEvent[2];	// m_hWorkEvent[0]: Work Event, m_hWorkEvent[1]: ShutDown Event
#else
	//pthread porting 2016.04.07 by yhee
	pthread_t			m_ThreadID;          
	struct event_flag	*m_event1, *m_event2;
	unsigned int		m_uiEventType;    //event_type 0: work_event 1: shutdown_event
#endif
	
	bool	m_bIsFree;			// Flag indicates which is free thread	
	void*	m_pParam;
	Int		m_nNum;

public:
	void (*Run)(void *param, Int num);	
	bool IsFree();		// Determine if thread is free or not
	void CreateWorkThread();	
	void SignalWorkEvent();
	void SingalShutDownEvent();
	void SetThreadBusy();
	void ReleaseHandles();
	void SetParam(void *param, Int num) { m_pParam = param; m_nNum = num; }	
#if (_ETRI_WINDOWS_APPLICATION)
	static unsigned __stdcall ThreadProc(void* Param);
	HANDLE GetThreadHandle();
	DWORD GetThreadID();
#else
	static void* ThreadProc(void* Param); //pthread porting 2016.04.07 by yhee
	pthread_t GetThreadID();//pthread porting
	unsigned int GetEventType();
	unsigned int WaitShutDownEvent();
	unsigned int WaitWorkEvent();
	struct event_flag* CreateEvent_L();
	void ResetWorkEvent();
	void WaitEvent_L(struct event_flag* ev);
	void SetEvent_L(struct event_flag* ev);
	void ResetEvent_L(struct event_flag* ev);
#endif
};

class TComThreadPoolMgr
{
	
public:
	TComThreadPoolMgr();
	virtual ~TComThreadPoolMgr();

private:
	TComThread* m_ptrCThread[MAX_THREAD_NUM];
#if (_ETRI_WINDOWS_APPLICATION)
	HANDLE m_hThreadPool[MAX_THREAD_NUM];  // Handle will be used in the end of Pool MGR for waiting on all thread to end
#else
	pthread_t m_hThreadPool[MAX_THREAD_NUM];  // 
#endif
	
	int m_nThreadCount;
	bool m_bThreadPause;

public:
	void Init();
	void Create(void (*Run)(void *param, Int num), Int nThread);
	bool AddThread(void *param, int nID, bool bThreadID = true);
	void FinishThread(void);
	Int GetFreeThread(void);
	Int GetFreeThreadID();
	bool GetAllFreeThread();
	void GetAllFreeThreadWaiting();
	Int GetThreadCount(void);
	Int GetFreeThreadCount(void);
	bool GetFreeThreadStatus(int nID);
	void Suspend();
	void Resume();
	bool IsRunning();
	void ETRI_SleepMS(int milliseconds);
};

#endif
