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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxTime.h"
#include "foundation/windows/PxWindowsInclude.h"

using namespace physx;

static int64_t getTimeTicks()
{
	LARGE_INTEGER a;
	QueryPerformanceCounter(&a);
	return a.QuadPart;
}

static double getTickDuration()
{
	LARGE_INTEGER a;
	QueryPerformanceFrequency(&a);
	return 1.0f / double(a.QuadPart);
}

static double sTickDuration = getTickDuration();

static const PxCounterFrequencyToTensOfNanos gCounterFreq = PxTime::getCounterFrequency();

const PxCounterFrequencyToTensOfNanos& PxTime::getBootCounterFrequency()
{
	return gCounterFreq;
}

PxCounterFrequencyToTensOfNanos PxTime::getCounterFrequency()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return PxCounterFrequencyToTensOfNanos(PxTime::sNumTensOfNanoSecondsInASecond, (uint64_t)freq.QuadPart);
}

uint64_t PxTime::getCurrentCounterValue()
{
	LARGE_INTEGER ticks;
	QueryPerformanceCounter(&ticks);
	return (uint64_t)ticks.QuadPart;
}

PxTime::PxTime() : mTickCount(0)
{
	getElapsedSeconds();
}

PxTime::Second PxTime::getElapsedSeconds()
{
	int64_t lastTickCount = mTickCount;
	mTickCount = getTimeTicks();
	return (mTickCount - lastTickCount) * sTickDuration;
}

PxTime::Second PxTime::peekElapsedSeconds()
{
	return (getTimeTicks() - mTickCount) * sTickDuration;
}

PxTime::Second PxTime::getLastTime() const
{
	return mTickCount * sTickDuration;
}

