/*
 * rwlock.h
 *
 *  Created on: Aug 14, 2013
 *      Author: uplusplus
 */

#ifndef RWLOCK_H_
#define RWLOCK_H_

#include <pthread.h>

#ifndef PTHREAD_RWLOCK_INITIALIZER

typedef int pthread_rwlockattr_t;

typedef struct {
    pthread_mutex_t  lock;
    pthread_cond_t   cond;
    int              numLocks;
    int              writerThreadId;
    int              pendingReaders;
    int              pendingWriters;
    void*            reserved[4];  /* for future extensibility */
} pthread_rwlock_t;

#define PTHREAD_RWLOCK_INITIALIZER  { PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 0, 0, 0, { NULL, NULL, NULL, NULL } }

int pthread_rwlockattr_init(pthread_rwlockattr_t *attr);
int pthread_rwlockattr_destroy(pthread_rwlockattr_t *attr);
int pthread_rwlockattr_setpshared(pthread_rwlockattr_t *attr, int  pshared);
int pthread_rwlockattr_getpshared(pthread_rwlockattr_t *attr, int *pshared);

int pthread_rwlock_init(pthread_rwlock_t *rwlock, const pthread_rwlockattr_t *attr);
int pthread_rwlock_destroy(pthread_rwlock_t *rwlock);

int pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);
int pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock);
int pthread_rwlock_timedrdlock(pthread_rwlock_t *rwlock, const struct timespec *abs_timeout);

int pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);
int pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock);
int pthread_rwlock_timedwrlock(pthread_rwlock_t *rwlock, const struct timespec *abs_timeout);

int pthread_rwlock_unlock(pthread_rwlock_t *rwlock);

#endif /*PTHREAD_RWLOCK_INITIALIZER*/
#endif /* RWLOCK_H_ */
