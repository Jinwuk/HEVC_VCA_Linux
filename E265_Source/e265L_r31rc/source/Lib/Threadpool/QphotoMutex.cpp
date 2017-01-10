#ifdef _WIN32
#else
#include <unistd.h>
#endif
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <string.h>

#include <iostream>
#include <cmath>

#include "QphotoMutex.h"

QphotoMutex::QphotoMutex ()
{
    pthread_mutexattr_init( & _mutex_attr );
    int val = PTHREAD_MUTEX_RECURSIVE_NP;
    pthread_mutexattr_settype( &_mutex_attr, val);
    pthread_mutex_init( & _mutex, & _mutex_attr );
    pthread_cond_init(    & _cond, NULL );
}

QphotoMutex::~QphotoMutex ()
{
    pthread_mutex_destroy( & _mutex );
    pthread_mutexattr_destroy( & _mutex_attr );
    pthread_cond_destroy( & _cond );
}

void QphotoMutex::lock    () { pthread_mutex_lock(   & _mutex ); }

void QphotoMutex::unlock  () { pthread_mutex_unlock( & _mutex ); }

bool QphotoMutex::is_locked ()
{
    if ( pthread_mutex_trylock( & _mutex ) != 0 )
        return true;
    else
    {
        unlock();
        return false;
    }// else
}

void QphotoMutex::wait      () { pthread_cond_wait( & _cond, & _mutex ); }

void QphotoMutex::signal    () { pthread_cond_signal( & _cond ); }

void QphotoMutex::broadcast () { pthread_cond_broadcast( & _cond ); }