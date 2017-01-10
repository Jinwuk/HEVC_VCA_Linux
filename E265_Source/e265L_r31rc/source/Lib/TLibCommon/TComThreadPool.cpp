#include "TComThreadPool.h"
#if (_ETRI_WINDOWS_APPLICATION)
#include <process.h>
#if _WIN32_WINNT >= 0x0602
// added. wsseo. for process group control
#include "TLibCommon/process_group.h"
#endif
#endif

/////////////////////////////////////////////////////////////////////////////////////////
// TComThread
/////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************
TComThread Constructor
*************************************************************************/
TComThread::TComThread(void)
{
	// Initialize members
#if (_ETRI_WINDOWS_APPLICATION)
	m_ThreadID	= 0;
	m_hThread = NULL;
	m_hWorkEvent[0] = CreateEvent(NULL, true, false, NULL);
	m_hWorkEvent[1] = CreateEvent(NULL, true, false, NULL);
#else
	m_event1 = CreateEvent_L();
	m_event2 = CreateEvent_L();
	m_uiEventType = 0;
#endif
	m_bIsFree = true;
	
}

TComThread::~TComThread(void)
{

}

/*************************************************************************
CThread ThreadProc
Description : Thread Procedure
*************************************************************************/
#if (_ETRI_WINDOWS_APPLICATION)
unsigned __stdcall TComThread::ThreadProc(void* Param)
{
	// Create object of ThreadPoolMgr
	TComThread* ptrThread = (TComThread *)Param;

	bool bShutDown = false;

	while(!bShutDown)
	{
		DWORD dwWaitResult = WaitForMultipleObjects(2, ptrThread->m_hWorkEvent, false, INFINITE);
		switch(dwWaitResult)
		{
		case WAIT_OBJECT_0:
			// Work event signaled Call the Run function
			ptrThread->Run(ptrThread->m_pParam, ptrThread->m_nNum);

			ResetEvent(ptrThread->m_hWorkEvent[0]);
			ptrThread->m_bIsFree = true;
			break;

		case WAIT_OBJECT_0 + 1:
			bShutDown = true;
			break;

		default:
			break;
		}

	}

	return 0;
}
#else
void* TComThread::ThreadProc(void* Param)
{
	// Create object of ThreadPoolMgr
	TComThread* ptrThread = (TComThread *)Param;
	bool bShutDown = false;	

	while (!bShutDown)
	{
		while (ptrThread->WaitWorkEvent() || ptrThread->WaitShutDownEvent())
		{
			switch (ptrThread->GetEventType())
			{
			case WORKEVENT_L:
				ptrThread->Run(ptrThread->m_pParam, ptrThread->m_nNum);
				ptrThread->ResetWorkEvent();
				ptrThread->m_bIsFree = true;
				break;
			case SHUTDOWNEVENT_L:
				bShutDown = true;
				break;
			default:
				break;
			}
		}
		break;
	}	
	return NULL;
}

#endif

/*************************************************************************
CThread IsFree
Description : returns state of thread.
*************************************************************************/
bool TComThread::IsFree()
{
	return m_bIsFree;
}

void TComThread::CreateWorkThread()
{
#if (_ETRI_WINDOWS_APPLICATION)
#if _WIN32_WINNT >= 0x0602
	if( !GetProcGrpMgr()->m_bUseProcessGroup )
	{
		m_hThread = (HANDLE) _beginthreadex(NULL, 0, ThreadProc, (void *)this, 0, (unsigned *)&m_ThreadID);
	}
	else
	{
		m_hThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)ThreadProc, (LPVOID)this, CREATE_SUSPENDED, (LPDWORD)&m_ThreadID );
		if( m_hThread != NULL ) {
			_SetThreadAffinity( m_hThread );
			ResumeThread( m_hThread );
		}
	}
#else
	m_hThread = (HANDLE) _beginthreadex(NULL, 0, ThreadProc, (void *)this, 0, (unsigned *)&m_ThreadID);
#endif

    if( m_hThread == NULL )
		printf("Thread could not be created: Error = 0x%X\n", GetLastError());
	//     else
	//		 printf("Successfully created thread: ThreadID = 0x%X\n", m_ThreadID);
#else
	int success = pthread_create(&m_ThreadID, NULL, &TComThread::ThreadProc, (void *)this);
	if (success != 0)
		printf("PThread could not be created: Error = %d\n", errno);
	//     else
	//		 printf("Successfully created thread: ThreadID = 0x%X\n", m_ThreadID);
#endif	
}


#if (_ETRI_WINDOWS_APPLICATION)
HANDLE TComThread::GetThreadHandle()
{
	return m_hThread;
}
#endif

#if (_ETRI_WINDOWS_APPLICATION)
DWORD TComThread::GetThreadID()
{
	return m_ThreadID;
}

void TComThread::SignalWorkEvent()
{
	SetEvent(m_hWorkEvent[0]);
}

void TComThread::SingalShutDownEvent()
{
	SetEvent(m_hWorkEvent[1]);
}

void TComThread::ReleaseHandles()
{
	// Close all handles
	CloseHandle(m_hThread);
	CloseHandle(m_hWorkEvent[0]);
	CloseHandle(m_hWorkEvent[1]);
}

#else
pthread_t TComThread::GetThreadID()
{
	return m_ThreadID;
}
unsigned int TComThread::GetEventType()
{
	return m_uiEventType;
}
struct event_flag* TComThread::CreateEvent_L()
{
	struct event_flag* ev;
	ev = (struct event_flag*)malloc(sizeof(struct event_flag));
	pthread_mutex_init(&(ev->mutex), NULL);
	pthread_cond_init(&(ev->condition), NULL);
	ev->flag = 0;
	return ev;
}
void TComThread::WaitEvent_L(struct event_flag* ev)
{
	pthread_mutex_lock(&ev->mutex);
	while (!ev->flag)
		pthread_cond_wait(&ev->condition, &ev->mutex);
	ev->flag = 0;
	pthread_mutex_unlock(&ev->mutex);
}

void TComThread::SetEvent_L(struct event_flag* ev)
{
	pthread_mutex_lock(&ev->mutex);
	ev->flag = 1;
	pthread_mutex_unlock(&ev->mutex);
	pthread_cond_signal(&ev->condition);
}

void TComThread::ResetEvent_L(struct event_flag* ev)
{
	ev->flag = 0;	
}

void TComThread::SignalWorkEvent()
{	
	SetEvent_L(m_event1);
}

void TComThread::SingalShutDownEvent()
{	
	SetEvent_L(m_event2);
}

unsigned int TComThread::WaitShutDownEvent()
{
	//wait shutdown signal
	WaitEvent_L(m_event2);
	m_uiEventType = SHUTDOWNEVENT_L;
	return 2; // SHUTDOWNEVENT;
}

unsigned int TComThread::WaitWorkEvent()
{
	//wait work signal
	WaitEvent_L(m_event1);
	m_uiEventType = WORKEVENT_L;
	return 1; // WORKEVENT;
}

void TComThread::ResetWorkEvent()
{
	//wait work signal	
	ResetEvent_L(m_event1);
	m_uiEventType = 0;
}

void TComThread::ReleaseHandles()
{	
	//distroy event
	pthread_mutex_destroy(&(m_event1->mutex));
	pthread_mutex_destroy(&(m_event2->mutex));
	pthread_cond_destroy(&(m_event1->condition));
	pthread_cond_destroy(&(m_event2->condition));
	free(m_event1);
	free(m_event2);
}
#endif

void TComThread::SetThreadBusy()
{
	m_bIsFree = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
// TComThreadPoolMgr
/////////////////////////////////////////////////////////////////////////////////////////

TComThreadPoolMgr::TComThreadPoolMgr()
{

}

TComThreadPoolMgr::~TComThreadPoolMgr()
{

}

/******************************************************************
TComThreadPoolMgr Initialize
Description : Creates threads. 
******************************************************************/

void TComThreadPoolMgr::Init()
{
	m_bThreadPause = false;
}

/******************************************************************
TComThreadPoolMgr Initialize
Description : Creates threads. 
******************************************************************/
void TComThreadPoolMgr::Create(void (*Run)(void *param, Int num), Int nThread)
{
	m_nThreadCount = nThread;

	Int nCounter = 0;
	Int nThreadCount = m_nThreadCount;

	while(nCounter < nThreadCount)
	{
		// Create objects in heap
		m_ptrCThread[nCounter] = new TComThread();

		m_ptrCThread[nCounter]->CreateWorkThread();
		m_ptrCThread[nCounter]->Run = Run;
#if (_ETRI_WINDOWS_APPLICATION)
		m_hThreadPool[nCounter] = m_ptrCThread[nCounter]->GetThreadHandle();
#else
		m_hThreadPool[nCounter] = m_ptrCThread[nCounter]->GetThreadID();
#endif	

		// Increment the counter
		nCounter++;
	}
}

/******************************************************************
TComThreadPoolMgr FinishThread
Description : Mark shutdown signal and wait for each thread to end
******************************************************************/
void TComThreadPoolMgr::FinishThread()
{
	Int nCounter = 0;

	while(nCounter < m_nThreadCount)
	{
		m_ptrCThread[nCounter]->SingalShutDownEvent();
		nCounter++;
	}

#if (_ETRI_WINDOWS_APPLICATION)
	// Check if all threads ended successfully
	DWORD dwWaitResult = WaitForMultipleObjects(GetThreadCount(), m_hThreadPool, true, INFINITE);

	switch(dwWaitResult)
	{
	case WAIT_OBJECT_0:
		//printf("All threads are ended.\n");
		//Close all handles
		nCounter = 0;
		while(nCounter < m_nThreadCount)
		{
			m_ptrCThread[nCounter]->ReleaseHandles();
			delete m_ptrCThread[nCounter];
			nCounter++;
		}
		break;

	default:
		printf("Wait Error = 0x%X\n", GetLastError());
		break;
	}
#else
	nCounter = 0;
	while (nCounter < m_nThreadCount)
	{
		m_ptrCThread[nCounter]->WaitShutDownEvent();
		m_ptrCThread[nCounter]->ReleaseHandles();
		delete m_ptrCThread[nCounter];
		nCounter++;
	}

#endif
}

/******************************************************************
TComThreadPoolMgr GetFreeThread
Description : Return no. of free thread.
******************************************************************/
Int TComThreadPoolMgr::GetFreeThread()
{
	// Search which thread is free
	Int nCounter = 0;

	while(nCounter < m_nThreadCount)
	{
		if(m_ptrCThread[nCounter]->IsFree() == true)
		{
			return nCounter;
		}
		nCounter++;
	}

	// printf("All thread are busy. Wait for thread to be free!!!\n");
	return -1;  // All busy
}

Int TComThreadPoolMgr::GetFreeThreadID()
{
	int nThreadID = -1;

	while(1)
	{
		if(m_bThreadPause)
		{
			continue;
			ETRI_SleepMS(1);
		}

		nThreadID = GetFreeThread();
		if(nThreadID >= 0)
			break;
		ETRI_SleepMS(1);
	}

	return nThreadID;
}

bool TComThreadPoolMgr::GetAllFreeThread()
{
	Int nCount;
	bool bRet = true;

	for(nCount = 0; nCount < m_nThreadCount; nCount++)
	{
		if(!m_ptrCThread[nCount]->IsFree())
		{
			bRet = false;
			break;
		}
	}

	return bRet;
}

void TComThreadPoolMgr::GetAllFreeThreadWaiting()
{
	while(1)
	{
		if(GetAllFreeThread())
			break;
		ETRI_SleepMS(1);
	}
}

bool TComThreadPoolMgr::GetFreeThreadStatus(int nID)
{
	return m_ptrCThread[nID]->IsFree();
}

/******************************************************************
TComThreadPoolMgr AddThread
Description : Add thread to free thread.
******************************************************************/
bool TComThreadPoolMgr::AddThread(void *param, int nID, bool bThreadID)
{
	int nThreadID;

	if(bThreadID) nThreadID = nID;
	else
	{
		nThreadID = GetFreeThread();
	}

	if(nThreadID != -1)
	{
		m_ptrCThread[nThreadID]->SetThreadBusy();

		// Set information to thread member so that thread can use it
		m_ptrCThread[nThreadID]->SetParam(param, nID);

		// Signal Work event
		m_ptrCThread[nThreadID]->SignalWorkEvent();
	}

	return (nThreadID>=0)? true: false;
}

Int TComThreadPoolMgr::GetThreadCount()
{
	return m_nThreadCount;
}

Int TComThreadPoolMgr::GetFreeThreadCount()
{
	// Search which thread is free
	Int nCounter = 0;

	for(int i = 0; i < m_nThreadCount; i++)
	{
		if(m_ptrCThread[i]->IsFree() == true)
			nCounter++;
	}

	return nCounter;
}

void TComThreadPoolMgr::Suspend()
{
	m_bThreadPause = true;
}

void TComThreadPoolMgr::Resume()
{
	m_bThreadPause = false;
}

bool TComThreadPoolMgr::IsRunning()
{
	return m_bThreadPause? false: true;
}


#if (_ETRI_WINDOWS_APPLICATION)
void TComThreadPoolMgr::ETRI_SleepMS(int milliseconds)
{
	Sleep(milliseconds);
}
#else
void TComThreadPoolMgr::ETRI_SleepMS(int milliseconds)
{
#if _POSIX_C_SOURCE >= 199309L
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds % 1000) * 1000000;
	nanosleep(&ts, NULL);
#else
	usleep(milliseconds * 1000);
#endif
}
#endif
