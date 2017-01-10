#include <pthread.h>

#include "QphotoTask.h"

QphotoTask::QphotoTask(const int n) :
		_job_no(n),mVolatile(true){
}

QphotoTask::QphotoTask(bool isvolatile) :
		_job_no(0),mVolatile(isvolatile){
}

QphotoTask::~QphotoTask(){};

int  QphotoTask::job_no () const { return _job_no; }

void QphotoTask::lock   () { _sync_mutex.lock(); }

void QphotoTask::unlock () { _sync_mutex.unlock(); }

bool QphotoTask::on_proc ( const int  p ) const
{
    return ((p == NO_PROC) || (_job_no == NO_PROC) || (p == _job_no));
}

void QphotoTask::notify_() {
	_sync_mutex.lock();
	_sync_mutex.signal();
	_sync_mutex.unlock();
}

void QphotoTask::wait_() {
	_sync_mutex.lock();
	_sync_mutex.wait();
	_sync_mutex.unlock();
}

bool QphotoTask::mIsVolatile() {
	return mVolatile;
}

void QphotoTask::DestroyQphotoTask(QphotoTask* qphotoTask) {
	if(qphotoTask != NULL) {
		 qphotoTask->_sync_mutex.is_locked();
	}
}

//Qphoto END

