// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_REFCOUNTABLE_H
#define CM_REFCOUNTABLE_H

#include "foundation/PxAssert.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxAllocator.h"
#include "common/PxBase.h"

namespace physx
{
namespace Cm
{
	// PT: this is used to re-implement RefCountable using the ref-counter in PxBase, i.e. to dissociate
	// the RefCountable data from the RefCountable code. The goal is to be able to store the ref counter
	// in the padding bytes of PxBase, and also to avoid two v-table pointers in the class.
	class RefCountableExt : public PxRefCounted
	{
		public:

		RefCountableExt() : PxRefCounted(0, PxBaseFlags(0))	{}

		void	preExportDataReset()
		{
			mBuiltInRefCount = 1;
		}

		void	incRefCount()
		{
			volatile PxI32* val = reinterpret_cast<volatile PxI32*>(&mBuiltInRefCount);
			PxAtomicIncrement(val);
			// value better be greater than 1, or we've created a ref to an undefined object
			PX_ASSERT(mBuiltInRefCount>1);
		}

		void	decRefCount()
		{
			PX_ASSERT(mBuiltInRefCount>0);
			volatile PxI32* val = reinterpret_cast<volatile PxI32*>(&mBuiltInRefCount);
			if(physx::PxAtomicDecrement(val) == 0)
				onRefCountZero();
		}

		PX_FORCE_INLINE PxU32 getRefCount()	const
		{
			return mBuiltInRefCount;
		}
	};

	PX_FORCE_INLINE void RefCountable_preExportDataReset(PxRefCounted& base)	{ static_cast<RefCountableExt&>(base).preExportDataReset();			}
	PX_FORCE_INLINE void RefCountable_incRefCount(PxRefCounted& base)			{ static_cast<RefCountableExt&>(base).incRefCount();				}
	PX_FORCE_INLINE void RefCountable_decRefCount(PxRefCounted& base)			{ static_cast<RefCountableExt&>(base).decRefCount();				}
	PX_FORCE_INLINE PxU32 RefCountable_getRefCount(const PxRefCounted& base)	{ return static_cast<const RefCountableExt&>(base).getRefCount();	}

	// simple thread-safe reference count
	// when the ref count is zero, the object is in an undefined state (pending delete)

	class RefCountable
	{
	public:
// PX_SERIALIZATION
		RefCountable(const PxEMPTY) { PX_ASSERT(mRefCount == 1); }
				void	preExportDataReset() { mRefCount = 1; }
//~PX_SERIALIZATION
		explicit RefCountable(PxU32 initialCount = 1)
			: mRefCount(PxI32(initialCount))
		{
			PX_ASSERT(mRefCount!=0);
		}

		virtual ~RefCountable() {}

		/**
		Calls 'delete this;'. It needs to be overloaded for classes also deriving from 
		PxBase and call 'Cm::deletePxBase(this);' instead.
		*/
		virtual	void onRefCountZero()
		{
			PX_DELETE_THIS;
		}

		void incRefCount()
		{
			physx::PxAtomicIncrement(&mRefCount);
			// value better be greater than 1, or we've created a ref to an undefined object
			PX_ASSERT(mRefCount>1);
		}

		void decRefCount()
		{
			PX_ASSERT(mRefCount>0);
			if(physx::PxAtomicDecrement(&mRefCount) == 0)
				onRefCountZero();
		}

		PX_FORCE_INLINE PxU32 getRefCount() const
		{
			return PxU32(mRefCount);
		}
	private:
		volatile PxI32 mRefCount;
	};


} // namespace Cm

}

#endif
