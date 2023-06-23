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

#include "PxvConfig.h"
#include "DyCorrelationBuffer.h"
#include "PxsMaterialManager.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxVecMath.h"
#include "GuBounds.h"

using namespace physx;
using namespace aos;

namespace physx
{
namespace Dy
{
static PX_FORCE_INLINE void initContactPatch(CorrelationBuffer::ContactPatchData& patch, PxU16 index, PxReal restitution, PxReal staticFriction, PxReal dynamicFriction,
	PxU8 flags)
{
	patch.start = index;
	patch.count = 1;
	patch.next = 0;
	patch.flags = flags;
	patch.restitution = restitution;
	patch.staticFriction = staticFriction;
	patch.dynamicFriction = dynamicFriction;
}

bool createContactPatches(CorrelationBuffer& fb, const PxContactPoint* cb, PxU32 contactCount, PxReal normalTolerance)
{
	// PT: this rewritten version below doesn't have LHS

	PxU32 contactPatchCount = fb.contactPatchCount;
	if(contactPatchCount == PxContactBuffer::MAX_CONTACTS)
		return false;
	if(contactCount>0)
	{
		CorrelationBuffer::ContactPatchData* PX_RESTRICT currentPatchData = fb.contactPatches + contactPatchCount;
		const PxContactPoint* PX_RESTRICT contacts = cb;

		initContactPatch(fb.contactPatches[contactPatchCount++], PxTo16(0), contacts[0].restitution, 
			contacts[0].staticFriction, contacts[0].dynamicFriction, PxU8(contacts[0].materialFlags));

		Vec4V minV = V4LoadA(&contacts[0].point.x);
		Vec4V maxV = minV;

		PxU32 patchIndex = 0;
		PxU8 count = 1;

		for (PxU32 i = 1; i<contactCount; i++)
		{
			const PxContactPoint& curContact = contacts[i];
			const PxContactPoint& preContact = contacts[patchIndex];

			if(curContact.staticFriction == preContact.staticFriction
				&& curContact.dynamicFriction == preContact.dynamicFriction
				&& curContact.restitution == preContact.restitution
				&& curContact.normal.dot(preContact.normal)>=normalTolerance)
			{
				const Vec4V ptV = V4LoadA(&curContact.point.x);
				minV = V4Min(minV, ptV);
				maxV = V4Max(maxV, ptV);

				count++;
			}
			else
			{
				if(contactPatchCount == PxContactBuffer::MAX_CONTACTS)
					return false;
				patchIndex = i;
				currentPatchData->count = count;
				count = 1;
				StoreBounds(currentPatchData->patchBounds, minV, maxV);
				currentPatchData = fb.contactPatches + contactPatchCount;

				initContactPatch(fb.contactPatches[contactPatchCount++], PxTo16(i), curContact.restitution,
					curContact.staticFriction, curContact.dynamicFriction, PxU8(curContact.materialFlags));

				minV = V4LoadA(&curContact.point.x);
				maxV = minV;
			}
		}
		if(count!=1)
			currentPatchData->count = count;

		StoreBounds(currentPatchData->patchBounds, minV, maxV);
	}
	fb.contactPatchCount = contactPatchCount;
	return true;
}

static PX_FORCE_INLINE void initFrictionPatch(FrictionPatch& p, const PxVec3& worldNormal, const PxTransform& body0Pose, const PxTransform& body1Pose, 
	PxReal restitution, PxReal staticFriction, PxReal dynamicFriction, PxU8 materialFlags)
{
	p.body0Normal = body0Pose.rotateInv(worldNormal);
	p.body1Normal = body1Pose.rotateInv(worldNormal);
	p.relativeQuat = body0Pose.q.getConjugate() * body1Pose.q;
	p.anchorCount = 0;
	p.broken = 0;
	p.staticFriction = staticFriction;
	p.dynamicFriction = dynamicFriction;
	p.restitution = restitution;
	p.materialFlags = materialFlags;
}

bool correlatePatches(CorrelationBuffer& fb, 
					  const PxContactPoint* cb,
					  const PxTransform& bodyFrame0,
					  const PxTransform& bodyFrame1,
					  PxReal normalTolerance,
					  PxU32 startContactPatchIndex,
					  PxU32 startFrictionPatchIndex)
{
	bool overflow = false;
	PxU32 frictionPatchCount = fb.frictionPatchCount;

	for(PxU32 i=startContactPatchIndex;i<fb.contactPatchCount;i++)
	{
		CorrelationBuffer::ContactPatchData &c = fb.contactPatches[i];
		const PxVec3 patchNormal = cb[c.start].normal;

		PxU32 j=startFrictionPatchIndex;
		for(;j<frictionPatchCount && ((patchNormal.dot(fb.frictionPatchWorldNormal[j]) < normalTolerance) 
			|| fb.frictionPatches[j].restitution != c.restitution|| fb.frictionPatches[j].staticFriction != c.staticFriction || 
			fb.frictionPatches[j].dynamicFriction != c.dynamicFriction);j++)
			;

		if(j==frictionPatchCount)
		{
			overflow |= j==CorrelationBuffer::MAX_FRICTION_PATCHES;
			if(overflow)
				continue;

			initFrictionPatch(fb.frictionPatches[frictionPatchCount], patchNormal, bodyFrame0, bodyFrame1, c.restitution, c.staticFriction, c.dynamicFriction, c.flags);
			fb.frictionPatchWorldNormal[j] = patchNormal;
			fb.frictionPatchContactCounts[frictionPatchCount] = c.count;
			fb.patchBounds[frictionPatchCount] = c.patchBounds;
			fb.contactID[frictionPatchCount][0] = 0xffff;
			fb.contactID[frictionPatchCount++][1] = 0xffff;
			c.next = CorrelationBuffer::LIST_END;
		}
		else
		{
			fb.patchBounds[j].include(c.patchBounds);
			fb.frictionPatchContactCounts[j] += c.count;
			c.next = PxTo16(fb.correlationListHeads[j]);
		}

		fb.correlationListHeads[j] = i;
	}

	fb.frictionPatchCount = frictionPatchCount;

	return overflow;
}

// run over the friction patches, trying to find two anchors per patch. If we already have
// anchors that are close, we keep them, which gives us persistent spring behavior

void growPatches(CorrelationBuffer& fb,
				 const PxContactPoint* cb,
				 const PxTransform& bodyFrame0,
				 const PxTransform& bodyFrame1,
				 PxU32 frictionPatchStartIndex,
				 PxReal frictionOffsetThreshold)
{
	for(PxU32 i=frictionPatchStartIndex;i<fb.frictionPatchCount;i++)
	{
		FrictionPatch& fp = fb.frictionPatches[i];

		if (fp.anchorCount == 2 || fb.correlationListHeads[i] == CorrelationBuffer::LIST_END)
		{
			const PxReal frictionPatchDiagonalSq = fb.patchBounds[i].getDimensions().magnitudeSquared();
			const PxReal anchorSqDistance = (fp.body0Anchors[0] - fp.body0Anchors[1]).magnitudeSquared();

			//If the squared distance between the anchors is more than a quarter of the patch diagonal, we can keep, 
			//otherwise the anchors are potentially clustered around a corner so force a rebuild of the patch
			if (fb.frictionPatchContactCounts[i] == 0 || (anchorSqDistance * 4.f) >= frictionPatchDiagonalSq)
				continue;

			fp.anchorCount = 0;
		}

		PxVec3 worldAnchors[2];
		PxU16 anchorCount = 0;
		PxReal pointDistSq = 0.0f, dist0, dist1;

		// if we have an anchor already, keep it
		if(fp.anchorCount == 1)
		{
			worldAnchors[anchorCount++] = bodyFrame0.transform(fp.body0Anchors[0]);
		}

		const PxReal eps = 1e-8f;

		for(PxU32 patch = fb.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = fb.contactPatches[patch].next)
		{
			CorrelationBuffer::ContactPatchData& cp = fb.contactPatches[patch];
			for(PxU16 j=0;j<cp.count;j++)
			{
				const PxVec3& worldPoint = cb[cp.start+j].point;

				if(cb[cp.start+j].separation < frictionOffsetThreshold)
				{
					switch(anchorCount)
					{
					case 0:
						fb.contactID[i][0] = PxU16(cp.start+j);
						worldAnchors[0] = worldPoint;
						anchorCount++;
						break;
					case 1:
						pointDistSq = (worldPoint-worldAnchors[0]).magnitudeSquared(); 
						if (pointDistSq > eps)
						{
							fb.contactID[i][1] = PxU16(cp.start+j);
							worldAnchors[1] = worldPoint;
							anchorCount++;
						}
						break;
					default: //case 2
						dist0 = (worldPoint-worldAnchors[0]).magnitudeSquared();
						dist1 = (worldPoint-worldAnchors[1]).magnitudeSquared();
						if (dist0 > dist1)
						{
							if(dist0 > pointDistSq)
							{
								fb.contactID[i][1] = PxU16(cp.start+j);
								worldAnchors[1] = worldPoint;
								pointDistSq = dist0;
							}
						}
						else if (dist1 > pointDistSq)
						{
							fb.contactID[i][0] = PxU16(cp.start+j);
							worldAnchors[0] = worldPoint;
							pointDistSq = dist1;
						}
					}
				}
			}
		}

		//PX_ASSERT(anchorCount > 0);

		// add the new anchor(s) to the patch
		for(PxU32 j = fp.anchorCount; j < anchorCount; j++)
		{
			fp.body0Anchors[j] = bodyFrame0.transformInv(worldAnchors[j]);
			fp.body1Anchors[j] = bodyFrame1.transformInv(worldAnchors[j]);
		}

		// the block contact solver always reads at least one anchor per patch for performance reasons even if there are no valid patches,
		// so we need to initialize this in the unexpected case that we have no anchors

		if(anchorCount==0)
			fp.body0Anchors[0] = fp.body1Anchors[0] = PxVec3(0);

		fp.anchorCount = anchorCount;
	}
}

}

}

