// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_TASK_H
#define CM_TASK_H

#include "task/PxTask.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxMutex.h"
#include "foundation/PxInlineArray.h"
#include "foundation/PxFPU.h"

// PT: this shouldn't be in Cm. The whole task manager is in the PhysX DLL so we cannot use any of these inside the Common DLL

namespace physx
{
namespace Cm
{
	// wrapper around the public PxLightCpuTask
	// internal SDK tasks should be inherited from
	// this and override the runInternal() method
	// to ensure that the correct floating point 
	// state is set / reset during execution
	class Task : public physx::PxLightCpuTask
	{
	public:
		Task(PxU64 contextId)
		{
			mContextID = contextId;
		}

		virtual void run() PX_OVERRIDE
		{
#if PX_SWITCH  // special case because default rounding mode is not nearest
			PX_FPU_GUARD;
#else
			PX_SIMD_GUARD
#endif
			runInternal();
		}

		virtual void runInternal()=0;
	};

	// same as Cm::Task but inheriting from physx::PxBaseTask
	// instead of PxLightCpuTask
	class BaseTask : public physx::PxBaseTask
	{
	public:

		virtual void run() PX_OVERRIDE
		{
#if PX_SWITCH  // special case because default rounding mode is not nearest
			PX_FPU_GUARD;
#else
			PX_SIMD_GUARD
#endif
			runInternal();
		}

		virtual void runInternal()=0;
	};

	template <class T, void (T::*Fn)(physx::PxBaseTask*) >
	class DelegateTask : public Cm::Task, public PxUserAllocated
	{
	public:

		DelegateTask(PxU64 contextID, T* obj, const char* name) : Cm::Task(contextID), mObj(obj), mName(name) {}

		virtual void run() PX_OVERRIDE
		{
#if PX_SWITCH  // special case because default rounding mode is not nearest
			PX_FPU_GUARD;
#else
			PX_SIMD_GUARD
#endif
			(mObj->*Fn)(mCont);
		}

		virtual void runInternal() PX_OVERRIDE
		{
			(mObj->*Fn)(mCont);
		}

		virtual const char* getName() const PX_OVERRIDE
		{
			return mName;
		}

		void setObject(T* obj) { mObj = obj; }

	private:
		T* mObj;
		const char* mName;
	};


	/**
	\brief A task that maintains a list of dependent tasks.
	
	This task maintains a list of dependent tasks that have their reference counts 
	reduced on completion of the task.

	The refcount is incremented every time a dependent task is added.
	*/
	class FanoutTask : public Cm::BaseTask
	{
		PX_NOCOPY(FanoutTask)
	public:
		FanoutTask(PxU64 contextID, const char* name) : Cm::BaseTask(), mRefCount(0), mName(name), mNotifySubmission(false) { mContextID = contextID; }

		virtual void runInternal() PX_OVERRIDE {}

		virtual const char* getName() const PX_OVERRIDE { return mName; }

		/**
		Swap mDependents with mReferencesToRemove when refcount goes to 0.
		*/
		virtual void removeReference() PX_OVERRIDE
		{
			bool submit;
			{
				PxMutex::ScopedLock lock(mMutex);
				submit = removeReferenceInternal();
			}

			// It is important that the mutex is unlocked before the task is dispatched and that no member variables are
			// touched after that. Dispatching the task might stall this method. Meanwhile the dispatched task can finish,
			// get released and follow-up events might even free its memory. Once this method here resumes running, the task
			// memory might not be accessible any longer. Even if the memory was still valid, a fiber-based scheduler might
			// resume this method on a different thread than when it started but the mutex might rely on the same thread
			// doing the locking and unlocking.
			if (submit)
				mTm->getCpuDispatcher()->submitTask(*this);
		}

		/** 
		\brief Increases reference count
		*/
		virtual void addReference() PX_OVERRIDE
		{
			PxMutex::ScopedLock lock(mMutex);
			physx::PxAtomicIncrement(&mRefCount);
			mNotifySubmission = true;
		}

		/** 
		\brief Return the ref-count for this task 
		*/
		virtual PX_INLINE PxI32 getReference() const	PX_OVERRIDE
		{
			return mRefCount;
		}

		/**
		Sets the task manager. Doesn't increase the reference count.
		*/
		PX_INLINE void setTaskManager(physx::PxTaskManager& tm)
		{
			mTm = &tm;
		}

		/**
		Adds a dependent task. It also sets the task manager querying it from the dependent task.  
		The refcount is incremented every time a dependent task is added.
		*/
		PX_INLINE void addDependent(physx::PxBaseTask& dependent)
		{
			PxMutex::ScopedLock lock(mMutex);
			physx::PxAtomicIncrement(&mRefCount);
			mTm = dependent.getTaskManager();
			mDependents.pushBack(&dependent);
			dependent.addReference();
			mNotifySubmission = true;
		}

		/**
		Reduces reference counts of the continuation task and the dependent tasks, also 
		clearing the copy of continuation and dependents task list.
		*/
		virtual void release() PX_OVERRIDE
		{
			PxInlineArray<physx::PxBaseTask*, 10> referencesToRemove;
			bool submit = false;

			{
				PxMutex::ScopedLock lock(mMutex);

				const PxU32 contCount = mReferencesToRemove.size(); 
				referencesToRemove.reserve(contCount);
				for (PxU32 i=0; i < contCount; ++i)
					referencesToRemove.pushBack(mReferencesToRemove[i]);
				
				mReferencesToRemove.clear();
				// allow access to mReferencesToRemove again
				if (mNotifySubmission)
				{
					submit = removeReferenceInternal();
				}
				else
				{
					physx::PxAtomicDecrement(&mRefCount);
				}
			}

			// It is important that the mutex is unlocked before the task is dispatched and that no member variables are
			// touched after that. See the comment in removeReference() for details.
			if (submit)
				mTm->getCpuDispatcher()->submitTask(*this);

			for (PxU32 i=0; i < referencesToRemove.size(); ++i)
				referencesToRemove[i]->removeReference();
		}

	protected:
		volatile PxI32 mRefCount;
		const char* mName;
		PxInlineArray<physx::PxBaseTask*, 4> mDependents;
		PxInlineArray<physx::PxBaseTask*, 4> mReferencesToRemove;
		bool mNotifySubmission;
		PxMutex mMutex; // guarding mDependents and mNotifySubmission

	private:
		// refcount bookkeeping, call with mMutex held. Returns true if the task has to be handed to the
		// dispatcher, which the caller must do once mMutex is released.
		bool removeReferenceInternal()
		{
			if (!physx::PxAtomicDecrement(&mRefCount))
			{
				// prevents access to mReferencesToRemove until release
				physx::PxAtomicIncrement(&mRefCount);
				mNotifySubmission = false;
				PX_ASSERT(mReferencesToRemove.empty());
				for (PxU32 i = 0; i < mDependents.size(); i++)
					mReferencesToRemove.pushBack(mDependents[i]);
				mDependents.clear();
				return true;
			}
			return false;
		}
	};


	/**
	\brief Specialization of FanoutTask class in order to provide the delegation mechanism.
	*/
	template <class T, void (T::*Fn)(physx::PxBaseTask*) >
	class DelegateFanoutTask : public FanoutTask, public PxUserAllocated
	{
	public:
		DelegateFanoutTask(PxU64 contextID, T* obj, const char* name) : 
		  FanoutTask(contextID, name), mObj(obj) { }

		  virtual void runInternal() PX_OVERRIDE
		  {
			  physx::PxBaseTask* continuation = mReferencesToRemove.empty() ? NULL : mReferencesToRemove[0];
			  (mObj->*Fn)(continuation);
		  }

		  void setObject(T* obj) { mObj = obj; }

	private:
		T* mObj;
	};

	PX_FORCE_INLINE void startTask(Cm::Task* task, PxBaseTask* continuation)
	{
		if(continuation)
		{
			// PT: TODO: just make this a PxBaseTask function?
			task->setContinuation(continuation);
			task->removeReference();
		}
		else
			task->runInternal();
	}

	template<class T>
	PX_FORCE_INLINE void updateTaskLinkedList(T*& previousTask, T* task, T*& head)
	{
		if(previousTask)
			previousTask->mNext = task;
		else
			head = task;

		previousTask = task;
	}

} // namespace Cm

}

#endif
