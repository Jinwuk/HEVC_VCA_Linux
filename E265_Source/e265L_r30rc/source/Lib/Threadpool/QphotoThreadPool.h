#ifndef THREADPOOL_H_
#define THREADPOOL_H_

#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>

//#define THREAD_POOL_DEBUG
//#define _UES_GLOBAL_SINGLE_THREAD_POOL
//#define WIN32

#ifdef THREAD_POOL_DEBUG
#define REPORT_ERROR(...) fprintf (stderr,"line %d - ",__LINE__); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n")
#else
#define REPORT_ERROR(...)
#endif /* THREAD_POOL_DEBUG */
	#define THREAD_POOL_QUEUE_SIZE 10000

class QphotoTask;

class threadpool_task
{
public:
QphotoTask* mTask;
};

class threadpool_queue
{
public:
	unsigned int head;
	unsigned int tail;
	unsigned int num_of_cells;
	void *cells[THREAD_POOL_QUEUE_SIZE];
};

class QphotoThreadPool
{
private:
	threadpool_queue tasks_queue;
	threadpool_queue free_tasks_queue;

	threadpool_task tasks[THREAD_POOL_QUEUE_SIZE];

	pthread_t *thr_arr;

	unsigned short num_of_threads;
	volatile unsigned short stop_flag;

	pthread_mutex_t free_tasks_mutex;
	pthread_mutex_t mutex;
	pthread_cond_t free_tasks_cond;
	pthread_cond_t cond;

private:
	void threadpool_queue_init(threadpool_queue *queue);
	int threadpool_queue_enqueue(threadpool_queue *queue, void *data);
	void *threadpool_queue_dequeue(threadpool_queue *queue);
	int threadpool_queue_is_empty(threadpool_queue *queue);
	int threadpool_queue_getsize(threadpool_queue *queue);
	static threadpool_task* threadpool_task_get_task(QphotoThreadPool *pool);
	threadpool_task* threadpool_task_get_task();
	static void* worker_thr_routine(void *data);
	static void* stop_worker_thr_routines_cb(void *ptr);
	void stop_worker_thr_routines_cb();
	void threadpool_task_init(threadpool_task *task);

public:

	/**
 * This function creates a newly allocated thread pool.
 *
 * @param num_of_threads The number of worker thread used in this pool.
 * @return On success returns a newly allocated thread pool, on failure NULL is returned.
 */
	static QphotoThreadPool* createQphotoThreadPool(int num_of_threads);


	/**
 * This function adds a routine to be exexuted by the threadpool at some future time.
 *
 * @param pool The thread pool structure.
 * @param routine The routine to be executed.
 * @param data The data to be passed to the routine.
 * @param blocking The threadpool might be overloaded if blocking != 0 the operation will block until it is possible to add the routine to the thread pool. If blocking is 0 and the thread pool is overloaded, the call to this function will return immediately.
 *
 * @return 0 on success.
 * @return -1 on failure.
 * @return -2 when the threadpool is overloaded and blocking is set to 0 (non-blocking).
 */
	int threadpool_add_task(QphotoThreadPool *pool, void (*routine)(void*), void *data, int blocking);
	int threadpool_add_job(QphotoThreadPool *pool, QphotoTask* task, int blocking);


/**
 * This function stops all the worker threads (stop & exit). And frees all the allocated memory.
 * In case blocking != 0 the call to this function will block until all worker threads have exited.
 *
 * @param pool The thread pool structure.
 * @param blocking If blocking != 0, the call to this function will block until all worker threads are done.
 */
	void threadpool_free(QphotoThreadPool *pool, int blocking);
	static void destroyQphotoThreadPool(QphotoThreadPool* pool);

#ifdef _UES_GLOBAL_SINGLE_THREAD_POOL
#else
void  run       ( QphotoTask * job, void * ptr = NULL, const bool del = false );
void  sync      ( QphotoTask * job );
#endif

	void waitAllThreads();	
};


#ifdef _UES_GLOBAL_SINGLE_THREAD_POOL
namespace QphotoGlobalThreadPool {
QphotoThreadPool* createQphotoThreadPool(const unsigned int  max_p);
void  run       ( QphotoTask *        job,
                  void *               ptr = NULL,
                  const bool           del = false );
void  sync      ( QphotoTask *        job );
void destroyQphotoThreadPool();
}
#endif

#endif /* THREADPOOL_H_ */
