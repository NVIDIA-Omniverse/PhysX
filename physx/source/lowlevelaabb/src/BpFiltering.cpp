// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

