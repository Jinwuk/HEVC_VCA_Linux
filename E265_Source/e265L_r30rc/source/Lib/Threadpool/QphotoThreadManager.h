#ifndef __QPHOTHREADMANAGER_HH
#define __QPHOTHREADMANAGER_HH

#define QPHOTO_THREAD_POOL

#ifdef QPHOTO_THREAD_POOL
#include <iostream>
#include <list>
#include <vector>
#include "QphotoTask.h"
//#include "QmageDecType.h"
//#include "QmageDecoder.h"
//#include "QmageDecCommon.h"

class QphotoThreadPool;
class QphotoThreadPoolBaseManager;


class classThreadData
{
public:
	const char* mOrgFileName;
	int mFileSize;
	int mImageWidth;
	int mImageHeight;
	int mBitmapPadding;

	
	unsigned char *mEncData;
	unsigned char* mDecData
		;
	unsigned char* mOrgPreViewImage;
	unsigned char* mDecPreViewImage;

	int mEncodedSize;

	
	int mframeNum;
	int mBp;
	int mOffset;
	int mStatus;
	int Queue_index;




private:
	bool isRecycle;
public:


	classThreadData() {
	mOrgFileName = "";

	}
};


class QmageJob: public QphotoTask {
private:
	void* mData;
	QphotoThreadPoolBaseManager* manager;
	QmageJob(){};
	virtual ~QmageJob(){};

public:
	virtual void run(void *);
	static QmageJob* createQmageJob(void* data);
	static void destoryTQmageJob(QmageJob* job);
};

//140426 QPHOTO THREADPOOL START
class QphotoThreadPoolBaseManager
{
public:
	QphotoThreadPool* mQphotoPool;
	////////////////////////////customization for thread pool/////////////////////////////
	void* mUserData;
	bool mIsUseFuture;  //default == false;
	//////////////////////////////////////////////////////////////////////////////////////
	static const int TASK_INIT = 0x0;
	static const int TASK_RUNNING = 0x1;
	static const int TASK_COMPLETE = 0x2;
	std::vector<int> mTaskStatue;
	int mCurrentTaskIndex;
private:
	void init();
	void prepare();
	void requesetThreadPool();
	void startThreadPool();
	void future();
public:
	QphotoThreadPoolBaseManager(int maxp);
	QphotoThreadPoolBaseManager(){mQphotoPool = NULL;};
	int QphotoSetThreadPool(int maxp);
	~QphotoThreadPoolBaseManager();
	int getCompleteTask(); 

	////////////////////////////customization for thread pool/////////////////////////////
	virtual void setThreadData(void* userData) = 0;  
	virtual void prepareTask(void* userData) = 0;
	virtual void startThreadPool(void* userData) = 0;
	virtual void future(void* userData) = 0;
	//////////////////////////////////////////////////////////////////////////////////////
};
//140426 QPHOTO THREADPOOL END
#endif

#endif  // __QPHOTHREADMANAGER_HH