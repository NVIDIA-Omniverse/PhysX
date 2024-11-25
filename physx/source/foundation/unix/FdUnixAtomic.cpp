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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxAtomic.h"

#if ! PX_EMSCRIPTEN
#define PAUSE() asm("nop")
#else
#define PAUSE()
#endif

namespace physx
{

void* PxAtomicCompareExchangePointer(volatile void** dest, void* exch, void* comp)
{
	return __sync_val_compare_and_swap(const_cast<void**>(dest), comp, exch);
}

PxI32 PxAtomicCompareExchange(volatile PxI32* dest, PxI32 exch, PxI32 comp)
{
	return __sync_val_compare_and_swap(dest, comp, exch);
}

PxI64 PxAtomicCompareExchange(volatile PxI64* dest, PxI64 exch, PxI64 comp)
{
	return __sync_val_compare_and_swap(dest, comp, exch);
}

PxI32 PxAtomicIncrement(volatile PxI32* val)
{
	return __sync_add_and_fetch(val, 1);
}

PxI64 PxAtomicIncrement(volatile PxI64* val)
{
	return __sync_add_and_fetch(val, 1);
}

PxI32 PxAtomicDecrement(volatile PxI32* val)
{
	return __sync_sub_and_fetch(val, 1);
}

PxI64 PxAtomicDecrement(volatile PxI64* val)
{
	return __sync_sub_and_fetch(val, 1);
}

PxI32 PxAtomicAdd(volatile PxI32* val, PxI32 delta)
{
	return __sync_add_and_fetch(val, delta);
}

PxI64 PxAtomicAdd(volatile PxI64* val, PxI64 delta)
{
	return __sync_add_and_fetch(val, delta);
}

PxI32 PxAtomicMax(volatile PxI32* val, PxI32 val2)
{
	PxI32 oldVal, newVal;

	do
	{
		PAUSE();
		oldVal = *val;

		if(val2 > oldVal)
			newVal = val2;
		else
			newVal = oldVal;

	} while(PxAtomicCompareExchange(val, newVal, oldVal) != oldVal);

	return *val;
}

PxI64 PxAtomicMax(volatile PxI64* val, PxI64 val2)
{
	PxI64 oldVal, newVal;

	do
	{
		PAUSE();
		oldVal = *val;

		if(val2 > oldVal)
			newVal = val2;
		else
			newVal = oldVal;

	} while(PxAtomicCompareExchange(val, newVal, oldVal) != oldVal);

	return *val;
}

PxI32 PxAtomicExchange(volatile PxI32* val, PxI32 val2)
{
	PxI32 newVal, oldVal;

	do
	{
		PAUSE();
		oldVal = *val;
		newVal = val2;
	} while(PxAtomicCompareExchange(val, newVal, oldVal) != oldVal);

	return oldVal;
}

PxI64 PxAtomicExchange(volatile PxI64* val, PxI64 val2)
{
	PxI64 newVal, oldVal;

	do
	{
		PAUSE();
		oldVal = *val;
		newVal = val2;
	} while(PxAtomicCompareExchange(val, newVal, oldVal) != oldVal);

	return oldVal;
}

PxI32 PxAtomicOr(volatile PxI32* val, PxI32 mask)
{
	return __sync_or_and_fetch(val, mask);
}

PxI64 PxAtomicOr(volatile PxI64* val, PxI64 mask)
{
	return __sync_or_and_fetch(val, mask);
}

PxI32 PxAtomicAnd(volatile PxI32* val, PxI32 mask)
{
	return __sync_and_and_fetch(val, mask);
}

PxI64 PxAtomicAnd(volatile PxI64* val, PxI64 mask)
{
	return __sync_and_and_fetch(val, mask);
}

} // namespace physx
