#ifndef _MEEKTK_LOCK_H
#define _MEEKTK_LOCK_H

#include <pthread.h>

#define MEEK_SUCCESS 0
#define MEEK_FAILURE -1

#ifdef __cplusplus
extern "C" {
#endif

struct MeekLock {
	pthread_mutex_t mutex;
	int rvalue;
};

struct MeekLock meekGenLock();

// Tries to lock without blocking.
// Returns true or false if locking was or was not possible, respectively.
int meekTryLock(struct MeekLock*);

// Locks.  Blocks until locking is possible.
void meekLock(struct MeekLock*);
// Or has a timeout
void meekLockTimeout(struct MeekLock*);

// Unlocks, no matter who locked it in the first place.
// Works for locked and unlocked locks similarly.
void meekUnlock(struct MeekLock*);

#ifdef __cplusplus
}
#endif // extern "C"

#ifdef __cplusplus
class lockable {
	public:
	lockable();
	void lock();
	bool lockTimeout(size_t ms);
	void unLock();
	bool tryLock();

	private:
	struct MeekLock d_lock;
};
#endif

#endif
