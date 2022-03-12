#include "thirdparty/lock.h"
#include <time.h>
#include <stdio.h>

struct MeekLock meekGenLock() {
	struct MeekLock newLock;
	pthread_mutex_init(&newLock.mutex, NULL);
	return newLock;
}

int meekTryLock(struct MeekLock *lock) {
	return pthread_mutex_trylock(&lock->mutex);
}

void meekLock(struct MeekLock *lock) {
	pthread_mutex_lock(&lock->mutex);
}

int meekLockTimeout(struct MeekLock *lock, size_t ms) {
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	size_t curms = t.tv_sec*1000 + t.tv_nsec/1e6;
	
	curms += ms;

	t.tv_sec = curms/1000;
	t.tv_nsec = (curms%1000)*1e6;

	return pthread_mutex_timedlock(&lock->mutex, &t);
}

void meekUnlock(struct MeekLock *lock) {
	pthread_mutex_unlock(&lock->mutex);
}

lockable::lockable() {
	d_lock = meekGenLock();
}

void lockable::lock() {
	meekLock(&d_lock);
}

void lockable::unLock() {
	meekUnlock(&d_lock);
}

bool lockable::tryLock() {
	return meekTryLock(&d_lock);
}

bool lockable::lockTimeout(size_t ms) {
	return !meekLockTimeout(&d_lock, ms);
}

