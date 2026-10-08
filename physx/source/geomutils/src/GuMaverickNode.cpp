// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuMaverickNode.h"

using namespace physx;
using namespace Gu;

const PxU32 MaverickNode::mIndices[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

bool MaverickNode::addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp)
{
	if(mNbFree<FREE_PRUNER_SIZE)
	{
		const PxU32 index		= mNbFree++;
		mFreeObjects[index]		= object;
		mFreeHandles[index]		= handle;
		mFreeBounds[index]		= worldAABB;
		mFreeTransforms[index]	= transform;
		mFreeStamps[index]		= timeStamp;
		return true;
	}
	return false;
}

bool MaverickNode::updateObject(const PrunerPayload& object, const PxBounds3& worldAABB, const PxTransform& transform)
{
	for(PxU32 i=0;i<mNbFree;i++)
	{
		if(mFreeObjects[i]==object)
		{
			mFreeBounds[i]		= worldAABB;
			mFreeTransforms[i]	= transform;
			return true;
		}
	}
	return false;
}

bool MaverickNode::updateObject(PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform)
{
	for(PxU32 i=0;i<mNbFree;i++)
	{
		if(mFreeHandles[i]==handle)
		{
			mFreeBounds[i]		= worldAABB;
			mFreeTransforms[i]	= transform;
			return true;
		}
	}
	return false;
}

void MaverickNode::remove(PxU32 index)
{
	mNbFree--;
	if(index!=mNbFree)
	{
		mFreeBounds[index]		= mFreeBounds[mNbFree];
		mFreeTransforms[index]	= mFreeTransforms[mNbFree];
		mFreeObjects[index]		= mFreeObjects[mNbFree];
		mFreeHandles[index]		= mFreeHandles[mNbFree];
		mFreeStamps[index]		= mFreeStamps[mNbFree];
	}
}

bool MaverickNode::removeObject(const PrunerPayload& object, PxU32& timeStamp)
{
	for(PxU32 i=0;i<mNbFree;i++)
	{
		if(mFreeObjects[i]==object)
		{
			// We found the object we want to remove. Close the gap as usual.
			timeStamp = mFreeStamps[i];
			remove(i);
			return true;
		}
	}
	return false;
}

bool MaverickNode::removeObject(PrunerHandle handle, PxU32& timeStamp)
{
	for(PxU32 i=0;i<mNbFree;i++)
	{
		if(mFreeHandles[i]==handle)
		{
			// We found the object we want to remove. Close the gap as usual.
			timeStamp = mFreeStamps[i];
			remove(i);
			return true;
		}
	}
	return false;
}

PxU32 MaverickNode::removeMarkedObjects(PxU32 timeStamp)
{
	PxU32 nbRemoved=0;
	PxU32 i=0;
	while(i<mNbFree)
	{
		if(mFreeStamps[i]==timeStamp)
		{
			nbRemoved++;
			remove(i);
		}
		else i++;
	}
	return nbRemoved;
}

void MaverickNode::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0;i<mNbFree;i++)
	{
		mFreeBounds[i].minimum -= shift;
		mFreeBounds[i].maximum -= shift;
		mFreeTransforms[i].p -= shift;
	}
}
