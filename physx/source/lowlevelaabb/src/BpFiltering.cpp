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

#include "BpFiltering.h"

using namespace physx;
using namespace Bp;

BpFilter::BpFilter(bool discardKineKine, bool discardStaticKine)
{
	for(int j = 0; j < Bp::FilterType::COUNT; j++)
		for(int i = 0; i < Bp::FilterType::COUNT; i++)
			mLUT[j][i] = false;

	mLUT[Bp::FilterType::STATIC][Bp::FilterType::DYNAMIC] = mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::STATIC] = true;
	mLUT[Bp::FilterType::STATIC][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::STATIC] = !discardStaticKine;
	mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::DYNAMIC] = true;

	mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::DYNAMIC] = true;
	mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::KINEMATIC] = !discardKineKine;

	mLUT[Bp::FilterType::STATIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::STATIC] = true;
	mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::KINEMATIC] = true;
	mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::DYNAMIC] = true;
	mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::AGGREGATE] = true;

	//Enable deformable surface interactions
	mLUT[Bp::FilterType::DEFORMABLE_SURFACE][Bp::FilterType::DYNAMIC] = mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[Bp::FilterType::DEFORMABLE_SURFACE][Bp::FilterType::STATIC] = mLUT[Bp::FilterType::STATIC][Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[Bp::FilterType::DEFORMABLE_SURFACE][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::DEFORMABLE_SURFACE] = true;
	mLUT[Bp::FilterType::DEFORMABLE_SURFACE][Bp::FilterType::DEFORMABLE_SURFACE] = true;

	//Enable deformable volume interactions
	mLUT[Bp::FilterType::DEFORMABLE_VOLUME][Bp::FilterType::DYNAMIC] = mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[Bp::FilterType::DEFORMABLE_VOLUME][Bp::FilterType::STATIC] = mLUT[Bp::FilterType::STATIC][Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[Bp::FilterType::DEFORMABLE_VOLUME][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::DEFORMABLE_VOLUME] = true;
	mLUT[Bp::FilterType::DEFORMABLE_VOLUME][Bp::FilterType::DEFORMABLE_VOLUME] = true;

	//Enable particle system interactions
	mLUT[Bp::FilterType::PARTICLESYSTEM][Bp::FilterType::DYNAMIC] = mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[Bp::FilterType::PARTICLESYSTEM][Bp::FilterType::STATIC] = mLUT[Bp::FilterType::STATIC][Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[Bp::FilterType::PARTICLESYSTEM][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::PARTICLESYSTEM] = true;
	mLUT[Bp::FilterType::PARTICLESYSTEM][Bp::FilterType::PARTICLESYSTEM] = true;
}

BpFilter::~BpFilter()
{
}

