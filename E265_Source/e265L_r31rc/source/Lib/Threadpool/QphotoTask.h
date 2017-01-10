#ifndef __QPHOTOTASK_HH
#define __QPHOTOTASK_HH
//
//  Project : ThreadPool
//  File    : TThreadPool.hh
//  Author  : Ronald Kriemann
//  Purpose : class for managing a pool of threads
//

#include <iostream>
#include <list>

#include "QphotoMutex.h"

// no specific processor
const int  NO_PROC = -1;

class QphotoTask
{
protected:
    // @cond
        
    // number of processor this job was assigned to
    const int  _job_no;
    bool mVolatile;

    // mutex for synchronisation
    QphotoMutex     _sync_mutex;
        
    // @endcond
        
public:
	QphotoTask(const int n = NO_PROC);

	QphotoTask(bool isvolatile);
	virtual ~QphotoTask();
    virtual void run ( void * ptr ) = 0;
        
    int  job_no () const;

    void lock   ();

    void unlock ();

    bool on_proc ( const int  p ) const;

	void notify_();
	void wait_();
	bool mIsVolatile();

	static void DestroyQphotoTask(QphotoTask* qphotoTask);
};

#endif  // __TTHREADPOOL_HH
