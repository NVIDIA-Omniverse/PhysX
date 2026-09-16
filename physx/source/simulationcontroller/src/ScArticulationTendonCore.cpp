// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationTendonCore.h"
#include "ScArticulationTendonSim.h"

using namespace physx;

void Sc::ArticulationSpatialTendonCore::setStiffness(const PxReal stiffness)
{
	mStiffness = stiffness;

	if (mSim)
		mSim->setStiffness(stiffness);
}

PxReal Sc::ArticulationSpatialTendonCore::getStiffness() const
{
	return mStiffness;
}

void Sc::ArticulationSpatialTendonCore::setDamping(const PxReal damping)
{
	mDamping = damping;

	if (mSim)
		mSim->setDamping(damping);
}

PxReal Sc::ArticulationSpatialTendonCore::getDamping() const
{
	return mDamping;
}

void Sc::ArticulationSpatialTendonCore::setLimitStiffness(const PxReal stiffness)
{
	mLimitStiffness = stiffness;

	if (mSim)
		mSim->setLimitStiffness(stiffness);
}

PxReal Sc::ArticulationSpatialTendonCore::getLimitStiffness() const
{
	return mLimitStiffness;
}

void Sc::ArticulationSpatialTendonCore::setOffset(const PxReal offset)
{
	mOffset = offset;

	if (mSim)
		mSim->setOffset(offset);
}

PxReal Sc::ArticulationSpatialTendonCore::getOffset() const
{
	return mOffset;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::ArticulationFixedTendonCore::setStiffness(const PxReal stiffness)
{
	mStiffness = stiffness;

	if (mSim)
		mSim->setStiffness(stiffness);
}

PxReal Sc::ArticulationFixedTendonCore::getStiffness() const
{
	return mStiffness;
}

void Sc::ArticulationFixedTendonCore::setDamping(const PxReal damping)
{
	mDamping = damping;

	if (mSim)
		mSim->setDamping(damping);
}

PxReal Sc::ArticulationFixedTendonCore::getDamping() const
{
	return mDamping;
}

void Sc::ArticulationFixedTendonCore::setLimitStiffness(const PxReal stiffness)
{
	mLimitStiffness = stiffness;
	if (mSim)
		mSim->setLimitStiffness(stiffness);
}

PxReal Sc::ArticulationFixedTendonCore::getLimitStiffness() const
{
	return mLimitStiffness;
}

void Sc::ArticulationFixedTendonCore::setSpringRestLength(const PxReal restLength)
{
	mRestLength = restLength;
	if (mSim)
		mSim->setSpringRestLength(restLength);
}

PxReal	Sc::ArticulationFixedTendonCore::getSpringRestLength() const
{
	return mRestLength;
}

void Sc::ArticulationFixedTendonCore::setLimitRange(const PxReal lowLimit, const PxReal highLimit)
{
	mLowLimit = lowLimit;
	mHighLimit = highLimit;

	if (mSim)
		mSim->setLimitRange(lowLimit, highLimit);
}

void Sc::ArticulationFixedTendonCore::getLimitRange(PxReal& lowLimit, PxReal& highLimit) const
{
	lowLimit = mLowLimit;
	highLimit = mHighLimit;
}

void Sc::ArticulationFixedTendonCore::setOffset(const PxReal offset)
{
	mOffset = offset;
	if (mSim)
		mSim->setOffset(offset);
}

PxReal Sc::ArticulationFixedTendonCore::getOffset() const
{
	return mOffset;
}
