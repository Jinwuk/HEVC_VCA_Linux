#include <pthread.h>

#include "QphotoThreadPool.h"
#include "QphotoTask.h"

#ifdef _UES_GLOBAL_SINGLE_THREAD_POOL
QphotoThreadPool* g_other_thread_pool = NULL;
#endif

#ifdef WIN32
#else
//#include <android/log.h>	// ysjeong remove for linux
#ifdef DEBUG
#  define  D(x...)  __android_log_print(ANDROID_LOG_INFO,"Qmage",x)
#else
#  define  D(...)	do {} while (0)
#endif

//#define  E(x...)  __android_log_print(ANDROID_LOG_ERROR,"Qmage",x)
#endif

/**
 * This function inits a threadpool queue.
 *
 * @param queue The queue structure.
 */

void QphotoThreadPool::threadpool_queue_init(threadpool_queue *queue)
{
	int i;

	for (i = 0; i < THREAD_POOL_QUEUE_SIZE; i++)
	{
		queue->cells[i] = NULL;
	}

	queue->head = 0;
	queue->tail = 0;
	queue->num_of_cells = 0;
}

/**
 * This function adds data to the tail of the queue.
 *
 * @param queue The queue structure.
 * @param data The data to be added to the queue.
 * @return 0 on success (the data was added to the queue) else -1 is returned.
 */
int QphotoThreadPool::threadpool_queue_enqueue(threadpool_queue *queue, void *data)
{
	if (queue->num_of_cells == THREAD_POOL_QUEUE_SIZE) {
#ifdef DEBUG
		E("The queue is full, unable to add data to it.");
#endif
		return -1;
	}

	if (queue->cells[queue->tail] != NULL) {
#ifdef DEBUG
		E("A problem was detected in the queue (expected NULL, but found a different value).");
#endif
		return -1;
	}

	queue->cells[queue->tail] = data;

	queue->num_of_cells++;
	queue->tail++;

	if (queue->tail == THREAD_POOL_QUEUE_SIZE) {
		queue->tail = 0;
	}

	return 0;
}

/**
 * This function removes and returns the head data element in the queue.
 *
 * @param queue The queue structure.
 * @return On success a data element is returned, on failure NULL is returned.
 */
void* QphotoThreadPool::threadpool_queue_dequeue(threadpool_queue *queue)
{
	void *data;

	if (queue->num_of_cells == 0) {
#ifdef DEBUG
		E("Tried to dequeue from an empty queue.");
#endif
			return NULL;
	}

	data = queue->cells[queue->head];

	queue->cells[queue->head] = NULL;
	queue->num_of_cells--;

	if (queue->num_of_cells == 0) {
		queue->head = 0;
		queue->tail = 0;
	}
	else {
		queue->head++;
		if (queue->head == THREAD_POOL_QUEUE_SIZE) {
			queue->head = 0;
		}
	}

	return data;
}

/**
 * This function checks if a given queue is empty.
 *
 * @param queue The queue structure.
 * @return 1 if the queue is empty, else 0.
 */
int QphotoThreadPool::threadpool_queue_is_empty(threadpool_queue *queue)
{
	if (queue->num_of_cells == 0) {
		return 1;
	}

	return 0;
}

/**
 * This function queries for the size of the given queue argument.
 *
 * @param queue The queue structure.
 * @return The size of the queue.
 */
int QphotoThreadPool::threadpool_queue_getsize(threadpool_queue *queue)
{
	return queue->num_of_cells;
}

/**
 * This function initializes the a threadpool_task structure.
 *
 * @param task The threadpool_task to init.
 */
void QphotoThreadPool::threadpool_task_init(threadpool_task *task)
{
	task->mTask = NULL;
}

/**
 * This function obtains a queued task from the pool and returns it.
 * If no such task is available the operation blocks.
 *
 * @param pool The thread pool structure.
 * @return A task or NULL on error (or if thread pool should shut down).
 */
threadpool_task* QphotoThreadPool::threadpool_task_get_task(QphotoThreadPool *pool)
{
	threadpool_task* task;

	if (pool->stop_flag) {
		/* The pool should shut down return NULL. */
		return NULL;
	}

	/* Obtain a task */
	if (pthread_mutex_lock(&(pool->mutex))) {
		perror("pthread_mutex_lock: ");
		return NULL;
	}

	while (pool->threadpool_queue_is_empty(&(pool->tasks_queue)) && !pool->stop_flag) {
		/* Block until a new task arrives. */
		if (pthread_cond_wait(&(pool->cond),&(pool->mutex))) {
			perror("pthread_cond_wait: ");
			if (pthread_mutex_unlock(&(pool->mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return NULL;
		}
	}

	if (pool->stop_flag) {
		/* The pool should shut down return NULL. */
		if (pthread_mutex_unlock(&(pool->mutex))) {
			perror("pthread_mutex_unlock: ");
		}
		return NULL;
	}

	if ((task = (threadpool_task*)pool->threadpool_queue_dequeue(&(pool->tasks_queue))) == NULL) {
		/* Since task is NULL returning task will return NULL as required. */
		REPORT_ERROR("Failed to obtain a task from the jobs queue.");
	}

	if (pthread_mutex_unlock(&(pool->mutex))) {
		perror("pthread_mutex_unlock: ");
		return NULL;
	}

	return task;
}

threadpool_task* QphotoThreadPool::threadpool_task_get_task()
{
	threadpool_task* task;

	if (stop_flag) {
		/* The pool should shut down return NULL. */
		return NULL;
	}

	/* Obtain a task */
	if (pthread_mutex_lock(&(mutex))) {
		perror("pthread_mutex_lock: ");
		return NULL;
	}

	while (threadpool_queue_is_empty(&(tasks_queue)) && !stop_flag) {
		/* Block until a new task arrives. */
		if (pthread_cond_wait(&(cond),&(mutex))) {
			perror("pthread_cond_wait: ");
			if (pthread_mutex_unlock(&(mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return NULL;
		}
	}

	if (stop_flag) {
		/* The pool should shut down return NULL. */
		if (pthread_mutex_unlock(&(mutex))) {
			perror("pthread_mutex_unlock: ");
		}
		return NULL;
	}

	if ((task = (threadpool_task*)threadpool_queue_dequeue(&(tasks_queue))) == NULL) {
		/* Since task is NULL returning task will return NULL as required. */
		REPORT_ERROR("Failed to obtain a task from the jobs queue.");
	}

	if (pthread_mutex_unlock(&(mutex))) {
		perror("pthread_mutex_unlock: ");
		return NULL;
	}

	return task;
}

/**
 * This is the routine the worker threads do during their life.
 *
 * @param data Contains a pointer to the threadpool structure.
 * @return NULL.
 */
void* QphotoThreadPool::worker_thr_routine(void *data)
{
	QphotoThreadPool *pool = (QphotoThreadPool*)data;
	threadpool_task *task;

	while (1) {
		task = pool->threadpool_task_get_task();
		if (task == NULL) {
			if (pool->stop_flag) {
				/* Worker thr needs to exit (thread pool was shutdown). */
				break;
			}
			else {
				/* An error has occurred. */
				REPORT_ERROR("Warning an error has occurred when trying to obtain a worker task.");
				REPORT_ERROR("The worker thread has exited.");
				break;
			}
		}

		/* Execute routine (if any). */
		if (task->mTask) {
			QphotoTask * job = task->mTask;
			job->run(NULL);
//			job->unlock();

			if(job->mIsVolatile() == true) {
				delete job;
				job = NULL;
			}

			/* Release the task by returning it to the free_task_queue. */
			pool->threadpool_task_init(task);
			if (pthread_mutex_lock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_lock: ");
				REPORT_ERROR("The worker thread has exited.");
				break;
			}

			if (pool->threadpool_queue_enqueue(&(pool->free_tasks_queue),task)) {
				REPORT_ERROR("Failed to enqueue a task to free tasks queue.");
				if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
					perror("pthread_mutex_unlock: ");
				}

				REPORT_ERROR("The worker thread has exited.");
				break;
			}

			if (pool->threadpool_queue_getsize(&(pool->free_tasks_queue)) == 1) {
				/* Notify all waiting threads that new tasks can added. */
				if (pthread_cond_broadcast(&(pool->free_tasks_cond))) {
					perror("pthread_cond_broadcast: ");
					if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
						perror("pthread_mutex_unlock: ");
					}

					break;
				}
			}

			if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_unlock: ");
				REPORT_ERROR("The worker thread has exited.");
				break;
			}
		}
	}

	return NULL;
}

/**
 * This callback function does the following steps:
 * 1. It raises a flag that notifies the worker threads to stop working.
 * 2. It waits until all worker threads are done with their execution.
 * 3. It frees all the allocated memory of the threadpool struct.
 *
 * @param ptr The pool to stop its worker threads.

 * @return 0.
 */
void* QphotoThreadPool::stop_worker_thr_routines_cb(void *ptr)
{
	int i;
	QphotoThreadPool *pool = (QphotoThreadPool*)ptr;

	pool->stop_flag = 1;

	/* Wakeup all worker threads (broadcast operation). */
	if (pthread_mutex_lock(&(pool->mutex))) {
#ifdef DEBUG
		E("pthread_mutex_lock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return NULL;
	}

	if (pthread_cond_broadcast(&(pool->cond))) {
#ifdef DEBUG
		E("pthread_cond_broadcast: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif

		return NULL;
	}

	if (pthread_mutex_unlock(&(pool->mutex))) {
#ifdef DEBUG
		E("pthread_mutex_unlock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return NULL;
	}

	/* Wait until all worker threads are done. */
	for (i = 0; i < pool->num_of_threads; i++) {
		if (pthread_join(pool->thr_arr[i],NULL)) {
			perror("pthread_join: ");
		}
	}

	/* Free all allocated memory. */
	free(pool->thr_arr);
	delete (pool);
	//printd("stop_worker_thr_routines_cb() API call...\n");
	return NULL;
}

void QphotoThreadPool::stop_worker_thr_routines_cb()
{
	int i;
	stop_flag = 1;

	/* Wakeup all worker threads (broadcast operation). */
	if (pthread_mutex_lock(&(mutex))) {
#ifdef DEBUG
		E("pthread_mutex_lock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	if (pthread_cond_broadcast(&(cond))) {
#ifdef DEBUG
		E("pthread_cond_broadcast: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	if (pthread_mutex_unlock(&(mutex))) {
#ifdef DEBUG
		E("pthread_mutex_unlock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	/* Wait until all worker threads are done. */
	for (i = 0; i < num_of_threads; i++) {
		if (pthread_join(thr_arr[i],NULL)) {
			perror("pthread_join: ");
		}
	}

	/* Free all allocated memory. */
	free(thr_arr);
}

QphotoThreadPool* QphotoThreadPool::createQphotoThreadPool(int num_of_threads)
{
	QphotoThreadPool *pool;
	int i;

	/* Create the thread pool struct. */
/*	if ((pool = malloc(sizeof(threadpool))) == NULL) {
		perror("malloc: ");
		return NULL;
	}*/

	pool = new QphotoThreadPool();


	pool->stop_flag = 0;

	/* Init the mutex and cond vars. */
	if (pthread_mutex_init(&(pool->free_tasks_mutex),NULL)) {
		perror("pthread_mutex_init: ");
		delete (pool);
		return NULL;
	}
	if (pthread_mutex_init(&(pool->mutex),NULL)) {
		perror("pthread_mutex_init: ");
		delete (pool);
		return NULL;
	}
	if (pthread_cond_init(&(pool->free_tasks_cond),NULL)) {
		perror("pthread_mutex_init: ");
		delete (pool);
		return NULL;
	}
	if (pthread_cond_init(&(pool->cond),NULL)) {
		perror("pthread_mutex_init: ");
		delete (pool);
		return NULL;
	}

	/* Init the jobs queue. */
	pool->threadpool_queue_init(&(pool->tasks_queue));

	/* Init the free tasks queue. */
	pool->threadpool_queue_init(&(pool->free_tasks_queue));
	/* Add all the free tasks to the free tasks queue. */
	for (i = 0; i < THREAD_POOL_QUEUE_SIZE; i++) {
		pool->threadpool_task_init((pool->tasks) + i);
		if (pool->threadpool_queue_enqueue(&(pool->free_tasks_queue),(pool->tasks) + i)) {
			REPORT_ERROR("Failed to a task to the free tasks queue during initialization.");
			delete (pool);
			return NULL;
		}
	}

	/* Create the thr_arr. */
	if ((pool->thr_arr = (pthread_t *)malloc(sizeof(pthread_t) * num_of_threads)) == NULL) {
		perror("malloc: ");
		delete (pool);
		return NULL;
	}

	/* Start the worker threads. */
	for (pool->num_of_threads = 0; pool->num_of_threads < num_of_threads; (pool->num_of_threads)++) {
		if (pthread_create(&(pool->thr_arr[pool->num_of_threads]),NULL,worker_thr_routine,pool)) {
			perror("pthread_create:");

			QphotoThreadPool::destroyQphotoThreadPool(pool);

			return NULL;
		}
	}
	//printd("[Qphoto][ other thread pool  ]threadpool_init create API(num of threads:%d)\n",num_of_threads);
	return pool;
}

void QphotoThreadPool::destroyQphotoThreadPool(QphotoThreadPool* pool) {
	if(pool != NULL) {
		pool->stop_worker_thr_routines_cb();
		delete (pool);
		pool = NULL;
	}
}

int QphotoThreadPool::threadpool_add_task(QphotoThreadPool *pool, void (*routine)(void*), void *data, int blocking)
{
	threadpool_task *task;

	if (pool == NULL) {
		REPORT_ERROR("The threadpool received as argument is NULL.");
		return -1;
	}

	if (pthread_mutex_lock(&(pool->free_tasks_mutex))) {
		perror("pthread_mutex_lock: ");
		return -1;
	}

	/* Check if the free task queue is empty. */
	while (threadpool_queue_is_empty(&(pool->free_tasks_queue))) {
		if (!blocking) {
			/* Return immediately if the command is non blocking. */
			if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_unlock: ");
				return -1;
			}

			return -2;
		}

		/* blocking is set to 1, wait until free_tasks queue has a task to obtain. */
		if (pthread_cond_wait(&(pool->free_tasks_cond),&(pool->free_tasks_mutex))) {
			perror("pthread_cond_wait: ");
			if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return -1;
		}
	}

	/* Obtain an empty task. */
	if ((task = (threadpool_task*)threadpool_queue_dequeue(&(pool->free_tasks_queue))) == NULL) {
		REPORT_ERROR("Failed to obtain an empty task from the free tasks queue.");
		if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
			perror("pthread_mutex_unlock: ");
		}

		return -1;
	}

	if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
		perror("pthread_mutex_unlock: ");
		return -1;
	}

	//task->data = data;
	//task->routine_cb = routine;

	/* Add the task, to the tasks queue. */
	if (pthread_mutex_lock(&(pool->mutex))) {
		perror("pthread_mutex_lock: ");
		return -1;
	}

	if (threadpool_queue_enqueue(&(pool->tasks_queue),task)) {
		REPORT_ERROR("Failed to add a new task to the tasks queue.");
		if (pthread_mutex_unlock(&(pool->mutex))) {
			perror("pthread_mutex_unlock: ");
		}
		return -1;
	}

	if (threadpool_queue_getsize(&(pool->tasks_queue)) == 1) {
		/* Notify all worker threads that there are new jobs. */
		if (pthread_cond_broadcast(&(pool->cond))) {
			perror("pthread_cond_broadcast: ");
			if (pthread_mutex_unlock(&(pool->mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return -1;
		}
	}

	if (pthread_mutex_unlock(&(pool->mutex))) {
		perror("pthread_mutex_unlock: ");
		return -1;
	}

	return 0;
}

int QphotoThreadPool::threadpool_add_job(QphotoThreadPool *pool, QphotoTask* Task, int blocking)
{
	threadpool_task *task;

	if (pool == NULL) {
		REPORT_ERROR("The threadpool received as argument is NULL.");
		return -1;
	}

//	Task->lock();

	if (pthread_mutex_lock(&(pool->free_tasks_mutex))) {
		perror("pthread_mutex_lock: ");
		return -1;
	}

	/* Check if the free task queue is empty. */
	while (threadpool_queue_is_empty(&(pool->free_tasks_queue))) {
		if (!blocking) {
			/* Return immediately if the command is non blocking. */
			if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_unlock: ");
				return -1;
			}

			return -2;
		}

		/* blocking is set to 1, wait until free_tasks queue has a task to obtain. */
		if (pthread_cond_wait(&(pool->free_tasks_cond),&(pool->free_tasks_mutex))) {
			perror("pthread_cond_wait: ");
			if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return -1;
		}
	}

	/* Obtain an empty task. */
	if ((task = (threadpool_task*)threadpool_queue_dequeue(&(pool->free_tasks_queue))) == NULL) {
		REPORT_ERROR("Failed to obtain an empty task from the free tasks queue.");
		if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
			perror("pthread_mutex_unlock: ");
		}

		return -1;
	}

	if (pthread_mutex_unlock(&(pool->free_tasks_mutex))) {
		perror("pthread_mutex_unlock: ");
		return -1;
	}

	task->mTask = Task;
	//task->routine_cb = routine;

	/* Add the task, to the tasks queue. */
	if (pthread_mutex_lock(&(pool->mutex))) {
		perror("pthread_mutex_lock: ");
		return -1;
	}

	if (threadpool_queue_enqueue(&(pool->tasks_queue),task)) {
		REPORT_ERROR("Failed to add a new task to the tasks queue.");
		if (pthread_mutex_unlock(&(pool->mutex))) {
			perror("pthread_mutex_unlock: ");
		}
		return -1;
	}

	if (threadpool_queue_getsize(&(pool->tasks_queue)) == 1) {
		/* Notify all worker threads that there are new jobs. */
		if (pthread_cond_broadcast(&(pool->cond))) {
			perror("pthread_cond_broadcast: ");
			if (pthread_mutex_unlock(&(pool->mutex))) {
				perror("pthread_mutex_unlock: ");
			}

			return -1;
		}
	}

	if (pthread_mutex_unlock(&(pool->mutex))) {
		perror("pthread_mutex_unlock: ");
		return -1;
	}

	return 0;
}

void QphotoThreadPool::threadpool_free(QphotoThreadPool *pool, int blocking)
{
	pthread_t thr;

	if (blocking) {
		pool->stop_worker_thr_routines_cb();
		delete (pool);
		pool = NULL;
	}
	else {
		if (pthread_create(&thr,NULL,stop_worker_thr_routines_cb,pool)) {
			perror("pthread_create: ");
			REPORT_ERROR("Warning! will not be able to free memory allocation. This will cause a memory leak.");
			pool->stop_flag = 1;
		}
	}
}

#ifdef _UES_GLOBAL_SINGLE_THREAD_POOL
namespace QphotoGlobalThreadPool {

QphotoThreadPool* createQphotoThreadPool(const unsigned int  max_p) {
	if(g_other_thread_pool == NULL) {
		g_other_thread_pool = QphotoThreadPool::createQphotoThreadPool(max_p);
	}
	return g_other_thread_pool;
	}

void destroyQphotoThreadPool() {
	if(g_other_thread_pool != NULL) {
		QphotoThreadPool::destroyQphotoThreadPool(g_other_thread_pool);
		g_other_thread_pool = NULL;
	}
}

void  sync      ( QphotoTask *        job ) {
	    if ( job == NULL )
        return;
		job->lock();
		job->unlock();

}

void run ( QphotoTask * job, void * ptr, const bool del )
{
    if ( job == NULL )
        return;

    g_other_thread_pool->threadpool_add_job(g_other_thread_pool,job,0);
}

}
#else
void  QphotoThreadPool::sync      ( QphotoTask *        job ) {
		if ( job == NULL )
        return;

		job->lock();
		job->unlock();
}

void QphotoThreadPool::run ( QphotoTask * job, void * ptr, const bool del )
{
    if ( job == NULL )
        return;

	threadpool_add_job(this,job,0);
}
#endif

void QphotoThreadPool::waitAllThreads()
{
	int i;

	stop_flag = 1;

	/* Wakeup all worker threads (broadcast operation). */
	if (pthread_mutex_lock(&(mutex))) {
#ifdef DEBUG
		E("pthread_mutex_lock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	if (pthread_cond_broadcast(&(cond))) {
#ifdef DEBUG
		E("pthread_cond_broadcast: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	if (pthread_mutex_unlock(&(mutex))) {
#ifdef DEBUG
		E("pthread_mutex_unlock: ");
		E("Warning: Memory was not released.");
		E("Warning: Some of the worker threads may have failed to exit.");
#endif
		return;
	}

	/* Wait until all worker threads are done. */
	for (i = 0; i < num_of_threads; i++) {
		if (pthread_join(thr_arr[i], NULL)) {
			perror("pthread_join: ");
		}
	}

	stop_flag = 0;

	/* Free all allocated memory. */
	//free(thr_arr);
}