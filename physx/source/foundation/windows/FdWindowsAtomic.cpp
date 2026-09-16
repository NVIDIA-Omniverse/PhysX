// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/windows/PxWindowsInclude.h"
#include "foundation/PxAtomic.h"

namespace physx
{

PxI32 PxAtomicExchange(volatile PxI32* val, PxI32 val2)
{
	return (PxI32)InterlockedExchange((volatile LONG*)val, (LONG)val2);
}

PxI64 PxAtomicExchange(volatile PxI64* val, PxI64 val2)
{
	return (PxI64)InterlockedExchange64((volatile LONG64*)val, (LONG64)val2);
}

PxI32 PxAtomicCompareExchange(volatile PxI32* dest, PxI32 exch, PxI32 comp)
{
	return (PxI32)InterlockedCompareExchange((volatile LONG*)dest, exch, comp);
}

PxI64 PxAtomicCompareExchange(volatile PxI64* dest, PxI64 exch, PxI64 comp)
{
	return (PxI64)InterlockedCompareExchange64((volatile LONG64*)dest, exch, comp);
}

void* PxAtomicCompareExchangePointer(volatile void** dest, void* exch, void* comp)
{
	return InterlockedCompareExchangePointer((volatile PVOID*)dest, exch, comp);
}

PxI32 PxAtomicIncrement(volatile PxI32* val)
{
	return (PxI32)InterlockedIncrement((volatile LONG*)val);
}

PxI64 PxAtomicIncrement(volatile PxI64* val)
{
	return (PxI64)InterlockedIncrement64((volatile LONG64*)val);
}

PxI32 PxAtomicDecrement(volatile PxI32* val)
{
	return (PxI32)InterlockedDecrement((volatile LONG*)val);
}

PxI64 PxAtomicDecrement(volatile PxI64* val)
{
	return (PxI64)InterlockedDecrement64((volatile LONG64*)val);
}

PxI32 PxAtomicAdd(volatile PxI32* val, PxI32 delta)
{
	if(1)
	{
		return (PxI32)InterlockedAdd((volatile LONG*)val, delta);
	}
	else
	{
		LONG newValue, oldValue;
		do
		{
			oldValue = *val;
			newValue = oldValue + delta;
		} while(InterlockedCompareExchange((volatile LONG*)val, newValue, oldValue) != oldValue);

		return newValue;
	}
}

PxI64 PxAtomicAdd(volatile PxI64* val, PxI64 delta)
{
	if(1)
	{
		return (PxI64)InterlockedAdd64((volatile LONG64*)val, delta);
	}
	else
	{
		LONG64 newValue, oldValue;
		do
		{
			oldValue = *val;
			newValue = oldValue + delta;
		} while(InterlockedCompareExchange64((volatile LONG64*)val, newValue, oldValue) != oldValue);

		return newValue;
	}
}

PxI32 PxAtomicMax(volatile PxI32* val, PxI32 val2)
{
	// Could do this more efficiently in asm...

	LONG newValue, oldValue;

	do
	{
		oldValue = *val;

		newValue = val2 > oldValue ? val2 : oldValue;

	} while(InterlockedCompareExchange((volatile LONG*)val, newValue, oldValue) != oldValue);

	return newValue;
}

PxI64 PxAtomicMax(volatile PxI64* val, PxI64 val2)
{
	// Could do this more efficiently in asm...

	LONG64 newValue, oldValue;

	do
	{
		oldValue = *val;

		newValue = val2 > oldValue ? val2 : oldValue;

	} while(InterlockedCompareExchange64((volatile LONG64*)val, newValue, oldValue) != oldValue);

	return newValue;
}

PxI32 PxAtomicOr(volatile PxI32* val, PxI32 mask)
{
	return (PxI32)InterlockedOr((volatile LONG*)val, mask);
}

PxI64 PxAtomicOr(volatile PxI64* val, PxI64 mask)
{
	return (PxI64)InterlockedOr64((volatile LONG64*)val, mask);
}

PxI32 PxAtomicAnd(volatile PxI32* val, PxI32 mask)
{
	return (PxI32)InterlockedAnd((volatile LONG*)val, mask);
}

PxI64 PxAtomicAnd(volatile PxI64* val, PxI64 mask)
{
	return (PxI64)InterlockedAnd64((volatile LONG64*)val, mask);
}

} // namespace physx
