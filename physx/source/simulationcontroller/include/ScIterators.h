// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

      
#ifndef SC_ITERATORS_H
#define SC_ITERATORS_H

#include "foundation/PxVec3.h"
#include "PxContact.h"

namespace physx
{
class PxShape;
class PxsContactManagerOutputIterator;

namespace Sc
{
	class ShapeSimBase;
	class ElementSimInteraction;
	class ActorSim;
	
	struct Contact
	{
		Contact() 
			: normal(0.0f)
			, point(0.0f)
			, separation(0.0f)
			, normalForce(0.0f)
		{}

		PxVec3 normal;
		PxVec3 point;
		PxShape* shape0;
		PxShape* shape1;
		PxReal separation;
		PxReal normalForce;
		PxU32 faceIndex0;  // these are the external indices
		PxU32 faceIndex1;
		bool normalForceAvailable;
	};

	struct FrictionAnchor
	{
		PxVec3 normal;
		PxVec3 point;
		PxVec3 impulse;
	};

	class ContactIterator
	{
		public:		

			class Pair
			{
			public:
				Pair() : mIter(NULL, NULL, NULL, 0, 0), mAnchorIter(NULL, NULL, 0) {}
				Pair(const void*& contactPatches, const void*& contactPoints, const void*& frictionPatches, const PxU32 /*contactDataSize*/, const PxReal*& forces, PxU32 numContacts, PxU32 numPatches, ShapeSimBase& shape0, ShapeSimBase& shape1, ActorSim* actor0, ActorSim* actor1);
				Contact* getNextContact();
				FrictionAnchor* getNextFrictionAnchor();
				PxActor* getActor0() { return mActor0; }
				PxActor* getActor1() { return mActor1; }

			private:
				PxU32						mIndex;
				PxU32						mNumContacts;
				PxContactStreamIterator		mIter;
				PxFrictionAnchorStreamIterator	mAnchorIter;
				const PxReal*				mForces;
				Contact						mCurrentContact;
				FrictionAnchor				mCurrentAnchor;
				PxActor*					mActor0;
				PxActor*					mActor1;
			};

			ContactIterator() {}
			explicit ContactIterator(ElementSimInteraction** first, ElementSimInteraction** last, PxsContactManagerOutputIterator& outputs): mCurrent(first), mLast(last), mOffset(0), mOutputs(&outputs)
			{
				if ((!first) || (!last) || (first == last))
				{
					mCurrent = NULL;
					mLast = NULL;
				}
			}
			Pair* getNextPair();			
		private:
			ElementSimInteraction**			mCurrent;
			ElementSimInteraction**			mLast;
			Pair							mCurrentPair;
			PxU32							mOffset;
			PxsContactManagerOutputIterator* mOutputs;

	private:
	};

}  // namespace Sc

}

#endif
