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

#include "extensions/PxParticleExt.h"

#include "foundation/PxUserAllocated.h"
#include "PxScene.h"
#include "PxPhysics.h"
#include "PxRigidBody.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

namespace physx
{
namespace ExtGpu
{

void PxDmaDataToDevice(PxCudaContextManager* cudaContextManager, PxParticleBuffer* particleBuffer, const PxParticleBufferDesc& desc)
{
	cudaContextManager->acquireContext();

	PxVec4* posInvMass = particleBuffer->getPositionInvMasses();
	PxVec4* velocities = particleBuffer->getVelocities();
	PxU32* phases = particleBuffer->getPhases();
	PxParticleVolume* volumes = particleBuffer->getParticleVolumes();

	PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

	//KS - TODO - use an event to wait for this
	cudaContext->memcpyHtoDAsync(CUdeviceptr(posInvMass), desc.positions, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(velocities), desc.velocities, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(phases), desc.phases, desc.numActiveParticles * sizeof(PxU32), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(volumes), desc.volumes, desc.numVolumes * sizeof(PxParticleVolume), 0);

	particleBuffer->setNbActiveParticles(desc.numActiveParticles);
	particleBuffer->setNbParticleVolumes(desc.numVolumes);

	cudaContext->streamSynchronize(0);

	cudaContextManager->releaseContext();
}

PxParticleBuffer* PxCreateAndPopulateParticleBuffer(const PxParticleBufferDesc& desc, PxCudaContextManager* cudaContextManager)
{
	PxParticleBuffer* particleBuffer = PxGetPhysics().createParticleBuffer(desc.maxParticles, desc.maxVolumes, cudaContextManager);
	PxDmaDataToDevice(cudaContextManager, particleBuffer, desc);
	return particleBuffer;
}

PxParticleAndDiffuseBuffer* PxCreateAndPopulateParticleAndDiffuseBuffer(const PxParticleAndDiffuseBufferDesc& desc, PxCudaContextManager* cudaContextManager)
{
	PxParticleAndDiffuseBuffer* particleBuffer = PxGetPhysics().createParticleAndDiffuseBuffer(desc.maxParticles, desc.maxVolumes, desc.maxDiffuseParticles, cudaContextManager);
	PxDmaDataToDevice(cudaContextManager, particleBuffer, desc);
	particleBuffer->setMaxActiveDiffuseParticles(desc.maxActiveDiffuseParticles);
	return particleBuffer;
}


PxParticleClothBuffer* PxCreateAndPopulateParticleClothBuffer(const PxParticleBufferDesc& desc, const PxParticleClothDesc& clothDesc, PxPartitionedParticleCloth& output, PxCudaContextManager* cudaContextManager)
{
	cudaContextManager->acquireContext();

	PxParticleClothBuffer* clothBuffer = PxGetPhysics().createParticleClothBuffer(desc.maxParticles, desc.maxVolumes, clothDesc.nbCloths, clothDesc.nbTriangles, clothDesc.nbSprings, cudaContextManager);

	PxVec4* posInvMass = clothBuffer->getPositionInvMasses();
	PxVec4* velocities = clothBuffer->getVelocities();
	PxU32* phases = clothBuffer->getPhases();
	PxParticleVolume* volumes = clothBuffer->getParticleVolumes();
	PxU32* triangles = clothBuffer->getTriangles();
	PxVec4* restPositions = clothBuffer->getRestPositions();

	PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

	cudaContext->memcpyHtoDAsync(CUdeviceptr(posInvMass), desc.positions, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(velocities), desc.velocities, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(phases), desc.phases, desc.numActiveParticles * sizeof(PxU32), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(volumes), desc.volumes, desc.numVolumes * sizeof(PxParticleVolume), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(triangles), clothDesc.triangles, clothDesc.nbTriangles * sizeof(PxU32) * 3, 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(restPositions), clothDesc.restPositions, desc.numActiveParticles * sizeof(PxVec4), 0);

	clothBuffer->setNbActiveParticles(desc.numActiveParticles);
	clothBuffer->setNbParticleVolumes(desc.numVolumes);
	clothBuffer->setNbTriangles(clothDesc.nbTriangles);

	clothBuffer->setCloths(output);

	cudaContext->streamSynchronize(0);

	cudaContextManager->releaseContext();


	return clothBuffer;
}

PxParticleRigidBuffer* PxCreateAndPopulateParticleRigidBuffer(const PxParticleBufferDesc& desc, const PxParticleRigidDesc& rigidDesc, PxCudaContextManager* cudaContextManager)
{
	cudaContextManager->acquireContext();

	PxParticleRigidBuffer* rigidBuffer = PxGetPhysics().createParticleRigidBuffer(desc.maxParticles, desc.maxVolumes, rigidDesc.maxRigids, cudaContextManager);

	PxVec4* posInvMassd = rigidBuffer->getPositionInvMasses();
	PxVec4* velocitiesd = rigidBuffer->getVelocities();
	PxU32* phasesd = rigidBuffer->getPhases();
	PxParticleVolume* volumesd = rigidBuffer->getParticleVolumes();
	PxU32* rigidOffsetsd = rigidBuffer->getRigidOffsets();
	PxReal* rigidCoefficientsd = rigidBuffer->getRigidCoefficients();
	PxVec4* rigidTranslationsd = rigidBuffer->getRigidTranslations();
	PxVec4* rigidRotationsd = rigidBuffer->getRigidRotations();
	PxVec4* rigidLocalPositionsd = rigidBuffer->getRigidLocalPositions();
	PxVec4* rigidLocalNormalsd = rigidBuffer->getRigidLocalNormals();

	PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

	const PxU32 numRigids = rigidDesc.numActiveRigids;
	const PxU32 numActiveParticles = desc.numActiveParticles;

	cudaContext->memcpyHtoDAsync(CUdeviceptr(posInvMassd), desc.positions, numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(velocitiesd), desc.velocities, numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(phasesd), desc.phases, numActiveParticles * sizeof(PxU32), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(volumesd), desc.volumes, desc.numVolumes * sizeof(PxParticleVolume), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidOffsetsd), rigidDesc.rigidOffsets, sizeof(PxU32) * (numRigids + 1), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidCoefficientsd), rigidDesc.rigidCoefficients, sizeof(PxReal) * numRigids, 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidTranslationsd), rigidDesc.rigidTranslations, sizeof(PxVec4) * numRigids, 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidRotationsd), rigidDesc.rigidRotations, sizeof(PxQuat) * numRigids, 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidLocalPositionsd), rigidDesc.rigidLocalPositions, sizeof(PxVec4) * desc.numActiveParticles, 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(rigidLocalNormalsd), rigidDesc.rigidLocalNormals, sizeof(PxVec4) * desc.numActiveParticles, 0);

	rigidBuffer->setNbActiveParticles(numActiveParticles);
	rigidBuffer->setNbRigids(numRigids);
	rigidBuffer->setNbParticleVolumes(desc.numVolumes);

	cudaContext->streamSynchronize(0);

	cudaContextManager->releaseContext();

return rigidBuffer;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxParticleAttachmentBuffer::PxParticleAttachmentBuffer(PxParticleBuffer& particleBuffer, PxParticleSystem& particleSystem) : mParticleBuffer(particleBuffer),
	mDeviceAttachments(NULL), mDeviceFilters(NULL), mNumDeviceAttachments(0), mNumDeviceFilters(0), mCudaContextManager(particleSystem.getCudaContextManager()),
	mParticleSystem(particleSystem), mDirty(false)
{
}

PxParticleAttachmentBuffer::~PxParticleAttachmentBuffer()
{
	mCudaContextManager->acquireContext();

	PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

	if (mDeviceAttachments)
		cudaContext->memFree((CUdeviceptr)mDeviceAttachments);
	if (mDeviceFilters)
		cudaContext->memFree((CUdeviceptr)mDeviceFilters);

	mDeviceAttachments = NULL;
	mDeviceFilters = NULL;

	mCudaContextManager->releaseContext();
}

void PxParticleAttachmentBuffer::copyToDevice(CUstream stream)
{
	mCudaContextManager->acquireContext();

	PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

	if (mAttachments.size() > mNumDeviceAttachments)
	{
		if (mDeviceAttachments)
			cudaContext->memFree((CUdeviceptr)mDeviceAttachments);

		cudaContext->memAlloc((CUdeviceptr*)&mDeviceAttachments, sizeof(PxParticleRigidAttachment)*mAttachments.size());
		mNumDeviceAttachments = mAttachments.size();
	}
	if (mFilters.size() > mNumDeviceFilters)
	{
		if (mDeviceFilters)
			cudaContext->memFree((CUdeviceptr)mDeviceFilters);

		cudaContext->memAlloc((CUdeviceptr*)&mDeviceFilters, sizeof(PxParticleRigidFilterPair)*mFilters.size());
		mNumDeviceFilters = mFilters.size();
	}

	if (mAttachments.size())
		cudaContext->memcpyHtoDAsync((CUdeviceptr)mDeviceAttachments, mAttachments.begin(), sizeof(PxParticleRigidAttachment)*mAttachments.size(), stream);

	if (mFilters.size())
		cudaContext->memcpyHtoDAsync((CUdeviceptr)mDeviceFilters, mFilters.begin(), sizeof(PxParticleRigidFilterPair)*mFilters.size(), stream);

	mParticleBuffer.setRigidAttachments(mDeviceAttachments, mAttachments.size());
	mParticleBuffer.setRigidFilters(mDeviceFilters, mFilters.size());

	mDirty = true;

	for (PxU32 i = 0; i < mNewReferencedBodies.size(); ++i)
	{
		if (mReferencedBodies[mNewReferencedBodies[i]] > 0)
			mParticleSystem.addRigidAttachment(mNewReferencedBodies[i]);
	}

	for (PxU32 i = 0; i < mDestroyedRefrencedBodies.size(); ++i)
	{
		if (mReferencedBodies[mDestroyedRefrencedBodies[i]] == 0)
			mParticleSystem.removeRigidAttachment(mDestroyedRefrencedBodies[i]);
	}

	mNewReferencedBodies.resize(0);
	mDestroyedRefrencedBodies.resize(0);

	mCudaContextManager->releaseContext();
}

void PxParticleAttachmentBuffer::addRigidAttachment(PxRigidActor* rigidActor, const PxU32 particleID, const PxVec3& localPose, PxConeLimitedConstraint* coneLimit)
{
	PX_CHECK_AND_RETURN(coneLimit == NULL || coneLimit->isValid(), "PxParticleAttachmentBuffer::addRigidAttachment: PxConeLimitedConstraint needs to be valid if specified.");

	PX_ASSERT(particleID < mParticleBuffer.getNbActiveParticles());
	PxParticleRigidAttachment attachment(PxConeLimitedConstraint(), PxVec4(0.0f));

	if (rigidActor == NULL)
	{
		PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleAttachmentBuffer::addRigidAttachment: rigidActor cannot be NULL.");
			return;
	}
		
	if (coneLimit)
	{
		attachment.mConeLimitParams.axisAngle = PxVec4(coneLimit->mAxis, coneLimit->mAngle);
		attachment.mConeLimitParams.lowHighLimits = PxVec4(coneLimit->mLowLimit, coneLimit->mHighLimit, 0.f, 0.f);
	}
	else
	{
		attachment.mConeLimitParams.axisAngle = PxVec4(0.f, 0.f, 0.f, -1.f);
		attachment.mConeLimitParams.lowHighLimits = PxVec4(0.f);
	}

	if (rigidActor->getType() == PxActorType::eRIGID_STATIC)
	{
		// attachments to rigid static work in global space
		attachment.mLocalPose0 = PxVec4(static_cast<PxRigidBody*>(rigidActor)->getGlobalPose().transform(localPose), 0.0f);
		attachment.mID0 = PxNodeIndex().getInd();
	}
	else
	{
		// others use body space.
		PxRigidBody* rigid = static_cast<PxRigidBody*>(rigidActor);
		PxTransform body2Actor = rigid->getCMassLocalPose();
		attachment.mLocalPose0 = PxVec4(body2Actor.transformInv(localPose), 0.f);
		attachment.mID0 = rigid->getInternalIslandNodeIndex().getInd();
	}
	attachment.mID1 = particleID;
			
	//Insert in order...

	PxU32 l = 0, r = PxU32(mAttachments.size());

	while (l < r) //If difference is just 1, we've found an item...
	{
		PxU32 index = (l + r) / 2;

		if (attachment < mAttachments[index])
			r = index;
		else if (attachment > mAttachments[index])
			l = index + 1;
		else
			l = r = index; //This is a match so insert before l
	}

	mAttachments.insert();

	for (PxU32 i = mAttachments.size()-1; i > l; --i)
	{
		mAttachments[i] = mAttachments[i - 1];
	}
	mAttachments[l] = attachment;

	mDirty = true;

	if (rigidActor)
	{
		PxU32& refCount = mReferencedBodies[rigidActor];

		if (refCount == 0)
			mNewReferencedBodies.pushBack(rigidActor);
		refCount++;
	}
}

bool PxParticleAttachmentBuffer::removeRigidAttachment(PxRigidActor* rigidActor, const PxU32 particleID)
{
	PX_ASSERT(particleID < mParticleBuffer.getNbActiveParticles());

	if (rigidActor == NULL)
	{
		PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleAttachmentBuffer::removeRigidAttachment: rigidActor cannot be NULL.");
		return false;
	}

	if (rigidActor)
	{
		PxU32& refCount = mReferencedBodies[rigidActor];
		refCount--;

		if (refCount == 0)
			mDestroyedRefrencedBodies.pushBack(rigidActor);
	}

	PxParticleRigidFilterPair attachment;
	attachment.mID0 = rigidActor->getType() != PxActorType::eRIGID_STATIC ? static_cast<PxRigidBody*>(rigidActor)->getInternalIslandNodeIndex().getInd() :
		PxNodeIndex().getInd();
	attachment.mID1 = particleID;
	PxU32 l = 0, r = PxU32(mAttachments.size());

	while (l < r) //If difference is just 1, we've found an item...
	{
		PxU32 index = (l + r) / 2;

		if (attachment < mAttachments[index])
			r = index;
		else if (attachment > mAttachments[index])
			l = index + 1;
		else
			l = r = index; //This is a match so insert before l
	}

	if (mAttachments[l] == attachment)
	{
		mDirty = true;
		//Remove
		mAttachments.remove(l);
		return true;
	}
	return false;
}

void PxParticleAttachmentBuffer::addRigidFilter(PxRigidActor* rigidActor, const PxU32 particleID)
{
	PX_ASSERT(particleID < mParticleBuffer.getNbActiveParticles());
	PxParticleRigidFilterPair attachment;
	attachment.mID0 = rigidActor->getType() != PxActorType::eRIGID_STATIC ? static_cast<PxRigidBody*>(rigidActor)->getInternalIslandNodeIndex().getInd() :
		PxNodeIndex().getInd();
	attachment.mID1 = particleID;

	//Insert in order...

	PxU32 l = 0, r = PxU32(mFilters.size());

	while (l < r) //If difference is just 1, we've found an item...
	{
		PxU32 index = (l + r) / 2;

		if (attachment < mFilters[index])
			r = index;
		else if (attachment > mFilters[index])
			l = index + 1;
		else
			l = r = index; //This is a match so insert before l
	}

	mFilters.insert();

	for (PxU32 i = mFilters.size() - 1; i > l; --i)
	{
		mFilters[i] = mFilters[i - 1];
	}
	mFilters[l] = attachment;

	mDirty = true;
}

bool PxParticleAttachmentBuffer::removeRigidFilter(PxRigidActor* rigidActor, const PxU32 particleID)
{
	PX_ASSERT(particleID < mParticleBuffer.getNbActiveParticles());
	PxParticleRigidFilterPair attachment;
	attachment.mID0 = rigidActor->getType() != PxActorType::eRIGID_STATIC ? static_cast<PxRigidBody*>(rigidActor)->getInternalIslandNodeIndex().getInd() :
		PxNodeIndex().getInd();
	attachment.mID1 = particleID;
	PxU32 l = 0, r = PxU32(mFilters.size());

	while (l < r) //If difference is just 1, we've found an item...
	{
		PxU32 index = (l + r) / 2;

		if (attachment < mFilters[index])
			r = index;
		else if (attachment > mFilters[index])
			l = index + 1;
		else
			l = r = index; //This is a match so insert before l
	}

	if (mFilters[l] == attachment)
	{
		mDirty = true;
		//Remove
		mFilters.remove(l);
		return true;
	}
	return false;
}

PxParticleAttachmentBuffer* PxCreateParticleAttachmentBuffer(PxParticleBuffer& buffer, PxParticleSystem& particleSystem)
{
	return PX_NEW(PxParticleAttachmentBuffer)(buffer, particleSystem);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ParticleClothBuffersImpl : public PxParticleClothBufferHelper, public PxUserAllocated
{
	ParticleClothBuffersImpl(const PxU32 maxCloths, const PxU32 maxTriangles, const PxU32 maxSprings, const PxU32 maxParticles, PxCudaContextManager* cudaContextManager)
		: mCudaContextManager(cudaContextManager)
	{
		mMaxParticles = maxParticles;
		mClothDesc.nbParticles = 0;
		mClothDesc.restPositions = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);

		mMaxTriangles = maxTriangles;
		mClothDesc.nbTriangles = 0;
		mClothDesc.triangles = mCudaContextManager->allocPinnedHostBuffer<PxU32>(maxTriangles * 3);

		mMaxSprings = maxSprings;
		mClothDesc.nbSprings = 0;
		mClothDesc.springs = mCudaContextManager->allocPinnedHostBuffer<PxParticleSpring>(maxSprings);

		mMaxCloths = maxCloths;
		mClothDesc.nbCloths = 0;
		mClothDesc.cloths = mCudaContextManager->allocPinnedHostBuffer<PxParticleCloth>(maxCloths);
	}

	void release()
	{
		mCudaContextManager->freePinnedHostBuffer(mClothDesc.cloths);
		mCudaContextManager->freePinnedHostBuffer(mClothDesc.restPositions);
		mCudaContextManager->freePinnedHostBuffer(mClothDesc.triangles);
		mCudaContextManager->freePinnedHostBuffer(mClothDesc.springs);
		PX_DELETE_THIS;
	}

	PxU32 getMaxCloths() const { return mMaxCloths; }
	PxU32 getNumCloths() const { return mClothDesc.nbCloths; }
	PxU32 getMaxSprings() const { return mMaxSprings; }
	PxU32 getNumSprings() const { return mClothDesc.nbSprings; }
	PxU32 getMaxTriangles() const { return mMaxTriangles; }
	PxU32 getNumTriangles() const { return mClothDesc.nbTriangles; }
	PxU32 getMaxParticles() const { return mMaxParticles; }
	PxU32 getNumParticles() const { return mClothDesc.nbParticles; }

	void addCloth(const PxParticleCloth& particleCloth, const PxU32* triangles, const PxU32 numTriangles,
		const PxParticleSpring* springs, const PxU32 numSprings, const PxVec4* restPositions, const PxU32 numParticles)
	{
		if (mClothDesc.nbCloths + 1 > mMaxCloths)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleClothBufferHelper::addCloth: exceeding maximal number of cloths that can be added.");
			return;
		}

		if (mClothDesc.nbSprings + numSprings > mMaxSprings)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleClothBufferHelper::addCloth: exceeding maximal number of springs that can be added.");
			return;
		}

		if (mClothDesc.nbTriangles + numTriangles > mMaxTriangles)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleClothBufferHelper::addCloth: exceeding maximal number of triangles that can be added.");
			return;
		}

		if (mClothDesc.nbParticles + numParticles > mMaxParticles)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
				"PxParticleClothBufferHelper::addCloth: exceeding maximal number of particles that can be added.");
			return;
		}

		mClothDesc.cloths[mClothDesc.nbCloths] = particleCloth;
		mClothDesc.nbCloths += 1;

		for (PxU32 i = 0; i < numSprings; ++i)
		{
			PxParticleSpring& dst = mClothDesc.springs[mClothDesc.nbSprings + i];
			dst = springs[i];
			dst.ind0 += mClothDesc.nbParticles;
			dst.ind1 += mClothDesc.nbParticles;
		}
		mClothDesc.nbSprings += numSprings;

		for (PxU32 i = 0; i < numTriangles*3; ++i)
		{
			PxU32& dst = mClothDesc.triangles[mClothDesc.nbTriangles*3 + i];
			dst = triangles[i] + mClothDesc.nbParticles;
		}
		mClothDesc.nbTriangles += numTriangles;

		PxMemCopy(mClothDesc.restPositions + mClothDesc.nbParticles, restPositions, sizeof(PxVec4)*numParticles);
		mClothDesc.nbParticles += numParticles;
	}

	void addCloth(const PxReal blendScale, const PxReal restVolume, const PxReal pressure, const PxU32* triangles, const PxU32 numTriangles,
		const PxParticleSpring* springs, const PxU32 numSprings, const PxVec4* restPositions, const PxU32 numParticles)
	{
		PX_UNUSED(blendScale);
		PxParticleCloth particleCloth;
		//particleCloth.clothBlendScale = blendScale;
		particleCloth.restVolume = restVolume;
		particleCloth.pressure = pressure;
		particleCloth.startVertexIndex = mClothDesc.nbParticles;
		particleCloth.numVertices = numParticles;
		particleCloth.startTriangleIndex = mClothDesc.nbTriangles * 3;
		particleCloth.numTriangles = numTriangles;

		addCloth(particleCloth, triangles, numTriangles, springs, numSprings, restPositions, numParticles);
	}

	PxParticleClothDesc& getParticleClothDesc()
	{
		return mClothDesc;
	}

	PxU32 mMaxCloths;
	PxU32 mMaxSprings;
	PxU32 mMaxTriangles;
	PxU32 mMaxParticles;
	PxParticleClothDesc mClothDesc;
	PxCudaContextManager* mCudaContextManager;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ParticleVolumeBuffersImpl : public PxParticleVolumeBufferHelper, public PxUserAllocated
{
	ParticleVolumeBuffersImpl(PxU32 maxVolumes, PxU32 maxTriangles, PxCudaContextManager* cudaContextManager)
	{
		mMaxVolumes = maxVolumes;
		mNumVolumes = 0;
		mMaxTriangles = maxTriangles;
		mNumTriangles = 0;
		mParticleVolumeMeshes = reinterpret_cast<PxParticleVolumeMesh*>(PX_ALLOC(sizeof(PxParticleVolumeMesh) * maxVolumes, "ParticleVolumeBuffersImpl::mParticleVolumeMeshes"));
		mTriangles = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * maxTriangles * 3, "ParticleVolumeBuffersImpl::mTriangles"));

		mParticleVolumes = cudaContextManager->allocPinnedHostBuffer<PxParticleVolume>(maxVolumes);
		mCudaContextManager = cudaContextManager;
	}

	void release()
	{
		mCudaContextManager->freePinnedHostBuffer(mParticleVolumes);
		PX_FREE(mParticleVolumeMeshes);
		PX_FREE(mTriangles);
		PX_DELETE_THIS;
	}

	virtual PxU32 getMaxVolumes() const
	{
		return mMaxVolumes;
	}

	virtual PxU32 getNumVolumes() const
	{
		return mNumVolumes;
	}

	virtual PxU32 getMaxTriangles() const
	{
		return mMaxTriangles;
	}

	virtual PxU32 getNumTriangles() const
	{
		return mNumTriangles;
	}

	virtual PxParticleVolume* getParticleVolumes()
	{
		return mParticleVolumes;
	}

	virtual PxParticleVolumeMesh* getParticleVolumeMeshes()
	{
		return mParticleVolumeMeshes;
	}

	virtual PxU32* getTriangles()
	{
		return mTriangles;
	}

	virtual void addVolume(const PxParticleVolume& volume, const PxParticleVolumeMesh& volumeMesh, const PxU32* triangles, const PxU32 numTriangles)
	{
		if (mNumVolumes < mMaxVolumes && mNumTriangles + numTriangles <=  mMaxTriangles)
		{
			PX_ASSERT(volumeMesh.startIndex == mNumTriangles);

			mParticleVolumes[mNumVolumes] = volume;
			mParticleVolumeMeshes[mNumVolumes] = volumeMesh;
			mNumVolumes++;

			for (PxU32 i = 0; i < numTriangles*3; ++i)
			{
				mTriangles[mNumTriangles*3 + i] = triangles[i] + volumeMesh.startIndex;
			}
			mNumTriangles += numTriangles;
		}
	}

	virtual void addVolume(const PxU32 particleOffset, const PxU32 numParticles, const PxU32* triangles, const PxU32 numTriangles)
	{
		if (mNumVolumes < mMaxVolumes && mNumTriangles + numTriangles <=  mMaxTriangles)
		{
			PxParticleVolume particleVolume;
			particleVolume.bound.setEmpty();
			particleVolume.particleIndicesOffset = particleOffset;
			particleVolume.numParticles = numParticles;
			PxParticleVolumeMesh particleVolumeMesh;
			particleVolumeMesh.startIndex = mNumTriangles;
			particleVolumeMesh.count = numTriangles;

			mParticleVolumes[mNumVolumes] = particleVolume;
			mParticleVolumeMeshes[mNumVolumes] = particleVolumeMesh;
			mNumVolumes++;

			for (PxU32 i = 0; i < numTriangles*3; ++i)
			{
				mTriangles[mNumTriangles*3 + i] = triangles[i] + particleOffset;
			}
			mNumTriangles += numTriangles;
		}
	}

	PxU32 mMaxVolumes;
	PxU32 mNumVolumes;
	PxU32 mMaxTriangles;
	PxU32 mNumTriangles;
	PxParticleVolume* mParticleVolumes;
	PxParticleVolumeMesh* mParticleVolumeMeshes;
	PxU32* mTriangles;
	PxCudaContextManager* mCudaContextManager;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ParticleRigidBuffersImpl : public PxParticleRigidBufferHelper, public PxUserAllocated
{
	ParticleRigidBuffersImpl(PxU32 maxRigids, PxU32 maxParticles, PxCudaContextManager* cudaContextManager)
		: mCudaContextManager(cudaContextManager)
	{
		mRigidDesc.maxRigids = maxRigids;
		mRigidDesc.numActiveRigids = 0;
		mMaxParticles = maxParticles;
		mNumParticles = 0;

		mCudaContextManager->allocPinnedHostBuffer<PxU32>(mRigidDesc.rigidOffsets, maxRigids + 1);
		mCudaContextManager->allocPinnedHostBuffer<PxReal>(mRigidDesc.rigidCoefficients, maxRigids);
		mCudaContextManager->allocPinnedHostBuffer<PxVec4>(mRigidDesc.rigidTranslations, maxRigids);
		mCudaContextManager->allocPinnedHostBuffer<PxQuat>(mRigidDesc.rigidRotations, maxRigids);
		mCudaContextManager->allocPinnedHostBuffer<PxVec4>(mRigidDesc.rigidLocalPositions, maxParticles);
		mCudaContextManager->allocPinnedHostBuffer<PxVec4>(mRigidDesc.rigidLocalNormals, maxParticles);
	}

	void release()
	{
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidOffsets);
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidCoefficients);
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidTranslations);
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidRotations);
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidLocalPositions);
		mCudaContextManager->freePinnedHostBuffer(mRigidDesc.rigidLocalNormals);
		PX_DELETE_THIS;
	}

	virtual PxU32 getMaxRigids() const { return mRigidDesc.maxRigids; }
	virtual PxU32 getNumRigids() const { return mRigidDesc.numActiveRigids; }
	virtual PxU32 getMaxParticles() const { return mMaxParticles; }
	virtual PxU32 getNumParticles() const { return mNumParticles; }

	void addRigid(const PxVec3& translation, const PxQuat& rotation, const PxReal coefficient,
		const PxVec4* localPositions, const PxVec4* localNormals, PxU32 numParticles)
	{
		PX_ASSERT(numParticles > 0);
		const PxU32 numRigids = mRigidDesc.numActiveRigids;

		if (numParticles > 0 && numRigids < mRigidDesc.maxRigids && mNumParticles + numParticles <= mMaxParticles)
		{
			mRigidDesc.rigidOffsets[numRigids] = mNumParticles;
			mRigidDesc.rigidOffsets[numRigids + 1] = mNumParticles + numParticles;
			mRigidDesc.rigidTranslations[numRigids] = PxVec4(translation, 0.0f);
			mRigidDesc.rigidRotations[numRigids] = rotation;
			mRigidDesc.rigidCoefficients[numRigids] = coefficient;
			PxMemCopy(mRigidDesc.rigidLocalPositions + mNumParticles, localPositions, numParticles * sizeof(PxVec4));
			PxMemCopy(mRigidDesc.rigidLocalNormals + mNumParticles, localNormals, numParticles * sizeof(PxVec4));
			mRigidDesc.numActiveRigids += 1;
			mNumParticles += numParticles;
		}
	}

	PxParticleRigidDesc& getParticleRigidDesc()
	{
		return mRigidDesc;
	}

	PxU32 mMaxParticles;
	PxU32 mNumParticles;
	PxParticleRigidDesc mRigidDesc;
	PxCudaContextManager* mCudaContextManager;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxParticleVolumeBufferHelper* PxCreateParticleVolumeBufferHelper(PxU32 maxVolumes, PxU32 maxTriangles, PxCudaContextManager* cudaContextManager)
{
	PxParticleVolumeBufferHelper* ret = NULL;
#if PX_SUPPORT_GPU_PHYSX
	ret = PX_NEW(ParticleVolumeBuffersImpl)(maxVolumes, maxTriangles, cudaContextManager);
#else
	PX_UNUSED(maxVolumes);
	PX_UNUSED(maxTriangles);
	PX_UNUSED(cudaContextManager);
#endif
	return ret;
}

PxParticleClothBufferHelper* PxCreateParticleClothBufferHelper(const PxU32 maxCloths, const PxU32 maxTriangles, const PxU32 maxSprings, const PxU32 maxParticles, PxCudaContextManager* cudaContextManager)
{
	PxParticleClothBufferHelper* ret = NULL;
#if PX_SUPPORT_GPU_PHYSX
	ret = PX_NEW(ParticleClothBuffersImpl)(maxCloths, maxTriangles, maxSprings, maxParticles, cudaContextManager);
#else
	PX_UNUSED(maxCloths);
	PX_UNUSED(maxTriangles);
	PX_UNUSED(maxSprings);
	PX_UNUSED(maxParticles);
	PX_UNUSED(cudaContextManager);
#endif
	return ret;
}

PxParticleRigidBufferHelper* PxCreateParticleRigidBufferHelper(const PxU32 maxRigids, const PxU32 maxParticles, PxCudaContextManager* cudaContextManager)
{
	PxParticleRigidBufferHelper* ret = NULL;
#if PX_SUPPORT_GPU_PHYSX
	ret = PX_NEW(ParticleRigidBuffersImpl)(maxRigids, maxParticles, cudaContextManager);
#else
	PX_UNUSED(maxRigids);
	PX_UNUSED(maxParticles);
	PX_UNUSED(cudaContextManager);
#endif
	return ret;
}

} //namespace ExtGpu
} //namespace physx
