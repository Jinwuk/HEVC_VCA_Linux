#define QPHOTO_THREAD_POOL
//#define WIN32	// ysjeong remove for linux

#ifdef QPHOTO_THREAD_POOL
#include <pthread.h>
#include "QphotoThreadPool.h"
#include "QphotoThreadManager.h"
#ifdef WIN32
#include <Windows.h>
#endif


//#define SINGLE_PROCESS


#define Q_IDX(idx)	((idx) < 0) ? (QphotoThreadManager::DECODE_QUQUE_SIZE+(idx)) : ((idx) >= QphotoThreadManager::DECODE_QUQUE_SIZE) ? ((idx)&QUEUE_MOD) : (idx) 
#ifdef WIN32
#else
//#include <android/log.h>	// ysjeong remove for linux
#ifdef DEBUG_LOG
#  define  D(x...)  __android_log_print(ANDROID_LOG_INFO,"Qmage",x)
#else
#  define  D(...)	do {} while (0)
#endif

//#define  E(x...)  __android_log_print(ANDROID_LOG_ERROR,"Qmage",x)
#endif

QmageJob* QmageJob::createQmageJob(void* data) {
	QmageJob* newJob = new QmageJob();
	newJob->mData = data;
	return newJob;
	};

void QmageJob::destoryTQmageJob(QmageJob* job) {
		if(job != NULL)
			delete job;
	};

void QmageJob::run(void *) {
}

//140426 QPHOTO THREADPOOL START
QphotoThreadPoolBaseManager::QphotoThreadPoolBaseManager(int maxp) {
	mQphotoPool = QphotoThreadPool::createQphotoThreadPool(maxp);
	mIsUseFuture = false;
	mCurrentTaskIndex = 0;
}

int QphotoThreadPoolBaseManager::QphotoSetThreadPool(int maxp) {
	if (mQphotoPool == NULL) {
		mQphotoPool = QphotoThreadPool::createQphotoThreadPool(maxp);
		mIsUseFuture = false;
		mCurrentTaskIndex = 0;
	}
	else
		return -1;
	return 0;
}

QphotoThreadPoolBaseManager::~QphotoThreadPoolBaseManager() {
	if(mQphotoPool != NULL) {
		QphotoThreadPool::destroyQphotoThreadPool(mQphotoPool);
	}
}

void QphotoThreadPoolBaseManager::init() {
	setThreadData(mUserData);
}
void QphotoThreadPoolBaseManager::prepare() {
	prepareTask(mUserData);
}
void QphotoThreadPoolBaseManager::startThreadPool() {
	startThreadPool(mUserData);
}
void QphotoThreadPoolBaseManager::future() {
	if (mIsUseFuture) {
		if (mCurrentTaskIndex < 0 || mCurrentTaskIndex >=mTaskStatue.size())
		
		if (mTaskStatue[mCurrentTaskIndex] == TASK_COMPLETE) {
			future(mUserData);
			mCurrentTaskIndex++;
		}
	}
}
//140426 QPHOTO THREADPOOL END
#endif