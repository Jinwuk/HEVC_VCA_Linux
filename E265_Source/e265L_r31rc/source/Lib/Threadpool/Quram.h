#ifndef __SAMPLE_H__
#define __SAMPLE_H__


#include "QphotoThreadPool.h"
//#include "QphotoThreadPool/QphotoThreadManager.h"
#include "QphotoThreadManager.h"

#include "../Lib/TLibEncoder/TEncSlice.h"
#include "../Lib/TLibEncoder/TEncGOP.h"
#include "../Lib/TLibEncoder/TEncTile.h"

class TEncGOP;
class TEncThreadGOP;

struct EncTileInfo
{
	int id;
	int nTile;
	int *endCount;
	pthread_mutex_t *mutex;
	pthread_cond_t  *cond;
};

class EncTileJob : public QphotoTask {
	EncTileJob() {
	};

public:
#if ETRI_THREAD_LOAD_BALANCING
	EncTileJob(void *param, EncTileInfo info, TileLoadBalanceInfo lbInfo);
#else
	EncTileJob(void *param, EncTileInfo info);
#endif
	virtual ~EncTileJob() {};

	virtual void run(void *);
#if ETRI_THREAD_LOAD_BALANCING
	static EncTileJob* createJob(void *param, EncTileInfo info, TileLoadBalanceInfo lbInfo);
#else
	static EncTileJob* createJob(void *param, EncTileInfo info);
#endif
	static void destroyJob(EncTileJob* job);

private:

	EncTileInfo m_encInfo;
	TEncSlice *m_encSlice;

#if ETRI_THREAD_LOAD_BALANCING
	TileLoadBalanceInfo m_lbInfo;
#endif
};

class EncGopJob : public QphotoTask {
	EncGopJob() {
	};

public:

	EncGopJob(void *param, int id, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, bool *bThreadRunning);
	//virtual ~EncGopJob() {};
	virtual ~EncGopJob();

	virtual void run(void *);
	static EncGopJob* createJob(void *param, int id, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, bool *bThreadRunning);
	static void destroyJob(EncGopJob* job);

private:

	pthread_mutex_t *m_mutex;
	pthread_cond_t *m_cond;
	int *m_bRefPicAvailable;

	TEncGOP *m_encGOP;
	TEncThreadGOP *m_encThreadGOP;
	QphotoThreadPool *m_threadpool;
	bool *m_bThreadRunning;
	int m_id;
};

#endif