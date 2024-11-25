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

#ifndef GU_SDF_CONTACT_REDUCTION_H
#define GU_SDF_CONTACT_REDUCTION_H

#include "foundation/PxArray.h"
#include "foundation/PxAssert.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSort.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "geomutils/PxContactBuffer.h"
#include "geomutils/PxContactPoint.h"
#include "PxContact.h"

namespace physx
{
namespace Gu
{

const float PATCH_ACCEPT_DOTP = 0.9995f;
const float PATCH_REJECT_DOTP = 0.85f;
const float PXS_SEPARATION_TOLERANCE = 0.001f;

// SDF contact reduction on the CPU takes place in three steps:
//  (1) assign a contact to an existing patch or create a new one
//  (2) once the patches grow too large, reduce them
//  (3) order patches by penetration depth and select a diverse set
struct TinyContact
{
	TinyContact() : mNormal(0.0f), mSeparation(0.0f), mPoint(0.0f) { }
	TinyContact(const PxVec3& normal, PxReal separation, const PxVec3& point) : mNormal(normal), mSeparation(separation), mPoint(point) {}
	PxVec3 mNormal;
	PxReal mSeparation;
	PxVec3 mPoint;
};

template <PxU8 TMaxContactsPerPatch>
struct TinyContactPatch
{
	TinyContact mContacts[TMaxContactsPerPatch];
	PxU8 mNbContacts;
	const TinyContact* begin() const { return mContacts; }
	const TinyContact* end() const { return mContacts + mNbContacts; }

	TinyContactPatch() : mNbContacts(0) { }
};

PX_FORCE_INLINE static float square(float x) { return x * x; }

// proto-patch that may need to be reduced
template <PxU8 TMaxContactsPerPatch, PxU32 TContactBufSz, bool planarize = true>
class BufferedPatch
{
public:
	BufferedPatch() {}
	explicit BufferedPatch(const TinyContact& contact) : mRootNormal(contact.mNormal), mMinSeparation(contact.mSeparation)
	{
		addContact(contact);
	}

	// copy, skipping unused elements
	BufferedPatch(const BufferedPatch& other) : mRootNormal(other.mRootNormal), mMinSeparation(other.mMinSeparation), mNbContacts(other.mNbContacts)
	{
		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			mPointX[i] = other.mPointX[i];
			mPointY[i] = other.mPointY[i];
			mPointZ[i] = other.mPointZ[i];

			mNormalX[i] = other.mNormalX[i];
			mNormalY[i] = other.mNormalY[i];
			mNormalZ[i] = other.mNormalZ[i];

			mSeparation[i] = other.mSeparation[i];
		}
	}

	// add a contact
	PX_INLINE void addContact(const TinyContact& contact)
	{
		PX_ASSERT(mNbContacts < TContactBufSz);
		mPointX[mNbContacts] = contact.mPoint.x;
		mPointY[mNbContacts] = contact.mPoint.y;
		mPointZ[mNbContacts] = contact.mPoint.z;

		mNormalX[mNbContacts] = contact.mNormal.x;
		mNormalY[mNbContacts] = contact.mNormal.y;
		mNormalZ[mNbContacts] = contact.mNormal.z;

		mSeparation[mNbContacts] = contact.mSeparation;

		// in case of <= 4 contacts per patch, the deepest one will not usually be conserved
		if (contact.mSeparation < mMinSeparation)
			mMinSeparation = contact.mSeparation;

		++mNbContacts;
		if (mNbContacts == TContactBufSz)
			reduce();
	}

	PX_INLINE void asTinyContactPatch(TinyContactPatch<TMaxContactsPerPatch>& patch)
	{
		reduce();
		patch.mNbContacts = static_cast<PxU8>(mNbContacts);

		PX_ASSERT(mNbContacts <= TMaxContactsPerPatch);
		TinyContact* contacts = patch.mContacts;
		for (PxU32 contactIdx = 0; contactIdx < mNbContacts; ++contactIdx, ++contacts)
		{
			contacts->mPoint = PxVec3(mPointX[contactIdx], mPointY[contactIdx], mPointZ[contactIdx]);
			contacts->mNormal = PxVec3(mNormalX[contactIdx], mNormalY[contactIdx], mNormalZ[contactIdx]);
			contacts->mSeparation = mSeparation[contactIdx];
		}
	}

	// store reduced contacts into `contacts`, returning the number of contacts
	PX_INLINE PxU32 getContacts(TinyContact* contacts)
	{
		reduce();
		PX_ASSERT(mNbContacts <= TMaxContactsPerPatch);
		for (PxU32 contactIdx = 0; contactIdx < mNbContacts; ++contactIdx, ++contacts)
		{
			contacts->mPoint = PxVec3(mPointX[contactIdx], mPointY[contactIdx], mPointZ[contactIdx]);
			contacts->mNormal = PxVec3(mNormalX[contactIdx], mNormalY[contactIdx], mNormalZ[contactIdx]);
			contacts->mSeparation = mSeparation[contactIdx];
		}
		return mNbContacts;
	}

	PX_INLINE PxVec3 getPoint(PxU32 index) const
	{
		return PxVec3(mPointX[index], mPointY[index], mPointZ[index]);
	}

	// reduce the contacts to `TMaxContactsPerPatch`.
	PX_INLINE void reduce()
	{
		if (mNbContacts <= TMaxContactsPerPatch)
			return;
		// P0: most extreme point
		float maxDistOrigSq = -1;
		PxU32 P0Idx = 0;
		PxVec3 P0;
		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			const PxVec3 p = getPoint(i);
			float distOrigSq = p.magnitudeSquared();
			if (planarize)
				distOrigSq -= square(p.dot(mRootNormal));
			if (distOrigSq > maxDistOrigSq)
			{
				maxDistOrigSq = distOrigSq;
				P0Idx = i;
				P0 = p;
			}
		}
		// P1: most distant point from P0
		float maxDistP0Sq = -1;
		PxU32 P1Idx = 0;
		PxVec3 P1;
		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			const PxVec3 p = getPoint(i);
			const PxVec3 v = p - P0;
			float distP0Sq = v.magnitudeSquared();
			if (planarize)
				distP0Sq -= square(v.dot(mRootNormal));
			if (distP0Sq > maxDistP0Sq)
			{
				maxDistP0Sq = distP0Sq;
				P1Idx = i;
				P1 = p;
			}
		}

		if (P0Idx == P1Idx) // P0 == P1 => all points equal. keep only the first
		{
			// TODO(CA): account for differences in penetration? should not occur
			mNbContacts = 1;
			return;
		}

		PxVec3 P2, P3;
		// P2 & P3: most distant from P0-P1 segment in both directions
		const PxVec3 segNormal = (P0 - P1).cross(mRootNormal);
		float maxDistPos = -PX_MAX_REAL, maxDistNeg = PX_MAX_REAL;
		PxU32 P2Idx = PX_MAX_U32, P3Idx = PX_MAX_U32;
		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			if (i == P0Idx || i == P1Idx)  // ensure that we have contacts distinct from P0/P1
				continue;
			const PxVec3 p = getPoint(i);
			const PxReal dist = (p - P0).dot(segNormal);
			if (dist > maxDistPos)
			{
				maxDistPos = dist;
				P2Idx = i;
				P2 = p;
			}
			if (dist <= maxDistNeg)
			{
				maxDistNeg = dist;
				P3Idx = i;
				P3 = p;
			}
		}
		// cluster the points
		PxReal anchorSeparation[TMaxContactsPerPatch];
		PxU32 anchorDeepestIdx[TMaxContactsPerPatch];
		const PxU32 anchorIndices[4] = {P0Idx, P1Idx, P2Idx, P3Idx};
		const PxVec3 anchorPoints[4] = {P0, P1, P2, P3};
		for(PxU32 i = 0; i < 4; ++i)
		{
			const PxU32 index = anchorIndices[i];
			anchorDeepestIdx[i] = index;
			anchorSeparation[i] = mSeparation[index] - PXS_SEPARATION_TOLERANCE;
		}
		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			const PxVec3 p = getPoint(i);
			PxReal dMin = PX_MAX_REAL;
			PxU32 anchorIdx = 0;
			for(PxU32 c = 0; c < 4; ++c)  // assign to anchors
			{
				const PxReal dist = (anchorPoints[c] - p).magnitudeSquared();
				if(dist < dMin)
				{
					dMin = dist;
					anchorIdx = c;
				}
			}
			if(mSeparation[i] < anchorSeparation[anchorIdx])  // pick deepest
			{
				anchorDeepestIdx[anchorIdx] = i;
				anchorSeparation[anchorIdx] = mSeparation[i];
			}
		}
		PxU32 chosenPoints[TMaxContactsPerPatch] =
		{
			anchorDeepestIdx[0], anchorDeepestIdx[1], anchorDeepestIdx[2], anchorDeepestIdx[3]
		};

		PxReal chosenSeparations[TMaxContactsPerPatch-4];  // only relevant for extra points
		for (PxU32 i = 0; i < TMaxContactsPerPatch-4; ++i)
			chosenSeparations[i] = PX_MAX_REAL;

		for (PxU32 i = 0; i < mNbContacts; ++i)
		{
			bool alreadyChosen = false;
			for (PxU32 j = 0; j < 4; ++j)
			{
				if (i == chosenPoints[j])
				{
					alreadyChosen = true;
					break;
				}
			}
			if(alreadyChosen)
				continue;
			PxReal sep = mSeparation[i];
			for (PxU32 slotIdx = 4; slotIdx < TMaxContactsPerPatch; ++slotIdx)
			{
				if (sep < chosenSeparations[slotIdx-4])
				{
					// drop out largest contact
					for (PxU32 k = TMaxContactsPerPatch-1; k > slotIdx; --k)
					{
						chosenSeparations[k-4] = chosenSeparations[k-1-4];
						chosenPoints[k] = chosenPoints[k-1];
					}
					// assign to this slot
					chosenSeparations[slotIdx-4] = sep;
					chosenPoints[slotIdx] = i;
					break;
				}
			}
		}
		float pointXNew[TMaxContactsPerPatch],
			  pointYNew[TMaxContactsPerPatch],
			  pointZNew[TMaxContactsPerPatch],
			  normalXNew[TMaxContactsPerPatch],
			  normalYNew[TMaxContactsPerPatch],
			  normalZNew[TMaxContactsPerPatch],
			  separationNew[TMaxContactsPerPatch];

		for (PxU32 dst = 0; dst < TMaxContactsPerPatch; ++dst)
		{
			const PxU32 src = chosenPoints[dst];
			pointXNew[dst] = mPointX[src];
			pointYNew[dst] = mPointY[src];
			pointZNew[dst] = mPointZ[src];

			normalXNew[dst] = mNormalX[src];
			normalYNew[dst] = mNormalY[src];
			normalZNew[dst] = mNormalZ[src];

			separationNew[dst] = mSeparation[src];
		}
		for (PxU32 i = 0; i < TMaxContactsPerPatch; ++i)
		{
			mPointX[i] = pointXNew[i];
			mPointY[i] = pointYNew[i];
			mPointZ[i] = pointZNew[i];

			mNormalX[i] = normalXNew[i];
			mNormalY[i] = normalYNew[i];
			mNormalZ[i] = normalZNew[i];

			mSeparation[i] = separationNew[i];
		}
		mNbContacts = TMaxContactsPerPatch;
	}
protected:

	float mNormalX[TContactBufSz];
	float mNormalY[TContactBufSz];
	float mNormalZ[TContactBufSz];

	float mSeparation[TContactBufSz];

	float mPointX[TContactBufSz];
	float mPointY[TContactBufSz];
	float mPointZ[TContactBufSz];

public:
	PxVec3 mRootNormal;
	PxReal mMinSeparation = PX_MAX_REAL;
protected:
	PxU32 mNbContacts = 0;
	PxU32 pad0;
	PxU32 pad1;
	PxU32 pad2;

	PX_COMPILE_TIME_ASSERT((TContactBufSz * sizeof(float)) % 32 == 0);
	PX_COMPILE_TIME_ASSERT(TContactBufSz > TMaxContactsPerPatch);
};

template <PxU8 TMaxContactsPerPatch, PxU32 TMaxPatches, PxU32 TPatchBufSz>
class SDFContactReduction
{
public:
	using Patch = BufferedPatch<TMaxContactsPerPatch, TPatchBufSz>;
	using TinyPatch = TinyContactPatch<TMaxContactsPerPatch>;

	// attempt to add a contact to one of the patches, adding a new one if necessary
	// return false if the contact was dropped
	PX_INLINE bool addContact(const TinyContact& contact)
	{
		++mNbContactsConsumed;
		for (Patch& patch: mPatchesBuffer)
		{
			const PxReal dot = patch.mRootNormal.dot(contact.mNormal);
			if (dot >= PATCH_ACCEPT_DOTP)
				return patch.addContact(contact), true;
		}
		mPatchesBuffer.pushBack(Patch(contact));
		// TODO(CA): make this more robust, taking into account the max number of surviving patches
		if (mPatchesBuffer.size() > mPatchCountLimit)
			cullPatches(1-(1-PATCH_REJECT_DOTP)/1);

		return true;
	}

	struct PatchPenetrationPredicate
	{
		bool operator()(const Patch* a, const Patch* b) const { return a->mMinSeparation < b->mMinSeparation; }
	};

	// cull the existing patches
	PX_INLINE void cullPatches(float rejectDotP)
	{
		PxArray<Patch*> patchesSorted;
		patchesSorted.reserve(mPatchesBuffer.size());
		for (Patch& patch: mPatchesBuffer)
			patchesSorted.pushBack(&patch);

		PxSort(patchesSorted.begin(), mPatchesBuffer.size(), PatchPenetrationPredicate());

		// drop patches that have > nbAllowedNeighbors neighbors that were selected
		// allowed neighbors = 0 seems to work best, still, 3 is worst, and >> 10 deactivates
		// isotropy
		const PxU32 nbAllowedNeighbors = 0;

		PxArray<PxVec3> patchNormals;
		// Geometrical upper bound for the number of patches based on the area "reserved" for each patch
		const PxU32 nbPatchesBound = static_cast<PxU32>(PxFloor(1.0f/square(PxSin(0.25f * PxAcos(rejectDotP)))));
		patchNormals.reserve(PxMin(mPatchesBuffer.size(), nbPatchesBound));

		mPatchesBufferTmp.clear();
		for (Patch* patch: patchesSorted)
		{
			PxU32 neighborPatches = 0;
			for (const PxVec3& rootNormal: patchNormals)
				if (rootNormal.dot(patch->mRootNormal) > rejectDotP)
					if (++neighborPatches > nbAllowedNeighbors)
						break;
			if (neighborPatches > nbAllowedNeighbors)
				continue;

			// patch->reduce();
			patchNormals.pushBack(patch->mRootNormal);
			mPatchesBufferTmp.pushBack(*patch);
		}
		PxSwap(mPatchesBuffer, mPatchesBufferTmp);
		mPatchesBufferTmp.clear();
	}

	// reduce each buffered patch, sort them, and create contacts
	PX_INLINE void flushContacts()
	{
		cullPatches(PATCH_REJECT_DOTP);
		for (Patch& patch: mPatchesBuffer)
		{
			TinyContactPatch<TMaxContactsPerPatch>& finalPatch = mPatchesFinal.pushBack({});
			patch.getContacts(finalPatch.mContacts);
			finalPatch.mNbContacts = patch.mNbContacts;
		}
	}

	PX_INLINE PxU32 flushToContactBuffer(PxContactBuffer& contactBuffer)
	{
		cullPatches(PATCH_REJECT_DOTP);
		PxU32 nbContactsKept = 0;
		for (Patch& patch: mPatchesBuffer)
		{
			TinyPatch finalPatch;
			patch.asTinyContactPatch(finalPatch);
			for (PxU32 i = 0; i < finalPatch.mNbContacts; ++i)
			{
				const TinyContact& contact = finalPatch.mContacts[i];
				PxContactPoint* c = contactBuffer.contact();
				if (c == NULL)
				{
#if PX_CHECKED
					PxGetFoundation().error(
						physx::PxErrorCode::eDEBUG_WARNING,
						PX_FL,
						"Dropping contacts in contact reduction due to full contact buffer.");
#endif
					break;
				}
				++nbContactsKept;
				c->normal = contact.mNormal;
				c->point = contact.mPoint;
				c->separation = contact.mSeparation;
				c->internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
			}
		}

		return nbContactsKept;
	}

protected:
	static const PxU32 mPatchCountLimit = 4000000/sizeof(Patch);  // 4 MB limit on patch size
	PxArray<Patch> mPatchesBuffer; // store patches up to MaxPatches
	PxArray<Patch> mPatchesBufferTmp;
	PxArray<TinyPatch> mPatchesFinal; // store culled patches

	PxU32 mNbContactsConsumed = 0;
};

}
}

#endif
