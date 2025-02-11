// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"

namespace physx
{
	namespace SnippetUtils
	{
		/////

		class BasicRandom
		{
			public:
										BasicRandom(PxU32 seed=0)	: mRnd(seed)	{}
										~BasicRandom()								{}

			PX_FORCE_INLINE	void		setSeed(PxU32 seed)			{ mRnd = seed;											}
			PX_FORCE_INLINE	PxU32		getCurrentValue()	const	{ return mRnd;											}
							PxU32		randomize()					{ mRnd = mRnd * 2147001325 + 715136305; return mRnd;	}

			PX_FORCE_INLINE	PxU32		rand()						{ return randomize() & 0xffff;							}
			PX_FORCE_INLINE	PxU32		rand32()					{ return randomize() & 0xffffffff;						}

							PxF32		rand(PxF32 a, PxF32 b)
										{
											const PxF32 r = rand32()/(static_cast<PxF32>(0xffffffff));
											return r*(b-a) + a;
										}

							PxI32		rand(PxI32 a, PxI32 b)
										{
											return a + static_cast<PxI32>(rand32()%(b-a));
										}

							PxF32		randomFloat()
										{
											return rand()/(static_cast<PxF32>(0xffff)) - 0.5f;
										}									
							PxF32		randomFloat32()
										{
											return rand32()/(static_cast<PxF32>(0xffffffff)) - 0.5f;
										}

							PxF32		randomFloat32(PxReal a, PxReal b) { return rand32()/PxF32(0xffffffff)*(b-a)+a; }
							void		unitRandomPt(physx::PxVec3& v);	
							void		unitRandomQuat(physx::PxQuat& v);

							PxVec3		unitRandomPt();
							PxQuat		unitRandomQuat();
		private:
							PxU32		mRnd;
		};

		/////

		PxU32			Bunny_getNbVerts();
		PxU32			Bunny_getNbFaces();
		const PxVec3*	Bunny_getVerts();
		const PxU32*	Bunny_getFaces();

		/////

		/* Increment the specified location. Return the incremented value. */
		PxI32 atomicIncrement(volatile PxI32* val);

		/* Decrement the specified location. Return the decremented value. */
		PxI32 atomicDecrement(volatile PxI32* val);

		//******************************************************************************//
		
		/* Return the number of physical cores (does not include hyper-threaded cores), returns 0 on failure. */
		PxU32 getNbPhysicalCores();

		//******************************************************************************//

		/* Return the id of a thread. */
		PxU32 getThreadId();

		//******************************************************************************//

		/* Return the current time */
		PxU64 getCurrentTimeCounterValue();

		/* Convert to milliseconds an elapsed time computed from the difference of the times returned from two calls to getCurrentTimeCounterValue. */
		PxReal getElapsedTimeInMilliseconds(const PxU64 elapsedTime);

		/* Convert to microseconds an elapsed time computed from the difference of the times returned from two calls to getCurrentTimeCounterValue. */
		PxReal getElapsedTimeInMicroSeconds(const PxU64 elapsedTime);

		//******************************************************************************//

		struct Sync;

		/* Create a sync object. Returns a unique handle to the sync object so that it may be addressed through syncWait etc. */
		Sync* syncCreate();

		/* Wait indefinitely until the specified sync object is signaled. */
		void syncWait(Sync* sync);

		/* Signal the specified synchronization object, waking all threads waiting on it. */
		void syncSet(Sync* sync);

		/** Reset the specified synchronization object. */
		void syncReset(Sync* sync);

		/* Release the specified sync object so that it may be reused with syncCreate. */
		void syncRelease(Sync* sync);

		//******************************************************************************//

		struct Thread;

		/* Prototype of callback passed to threadCreate. */
		typedef void (*ThreadEntryPoint)(void*);

		/* Create a thread object and return a unique handle to the thread object so that it may be addressed through threadStart etc. 
		entryPoint implements ThreadEntryPoint and data will be passed as a function argument, POSIX-style. */
		Thread* threadCreate(ThreadEntryPoint entryPoint, void* data);

		/* Cleanly shut down the specified thread. Called in the context of the spawned thread. */
		void threadQuit(Thread* thread);

		/* Stop the specified thread. Signals the spawned thread that it should stop, so the 
		thread should check regularly. */
		void threadSignalQuit(Thread* thread);

		/* Wait for the specified thread to stop. Should be called in the context of the spawning
		thread. Returns false if the thread has not been started.*/
		bool threadWaitForQuit(Thread* thread);

		/* Check whether the thread is signalled to quit. Called in the context of the
		spawned thread. */
		bool threadQuitIsSignalled(Thread* thread);

		/* Release the specified thread object so that it may be reused with threadCreate. */
		void threadRelease(Thread* thread);

		//******************************************************************************//

		struct Mutex;

		/* Create a mutex object and return a unique handle to the mutex object so that it may be addressed through mutexLock etc. */
		Mutex* mutexCreate();

		/* Acquire (lock) the specified mutex. If the mutex is already locked by another thread, this method blocks until the mutex is unlocked.*/
		void mutexLock(Mutex* mutex);

		/* Release (unlock) the specified mutex, the calling thread must have previously called lock() or method will error. */
		void mutexUnlock(Mutex* mutex);

		/* Release the specified mutex so that it may be reused with mutexCreate. */
		void mutexRelease(Mutex* mutex);
	}
}
