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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxgSimulationCore.h"
#include "PxDirectGPUAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "common/PxProfileZone.h"
#include "CudaKernelWrangler.h"
#include "foundation/PxAllocator.h"
#include "PxgArticulationBuffer.h"
#include "PxgCudaSolverCore.h"
#include "PxgArticulationCore.h"
#include "PxgNarrowphaseCore.h"
#include "PxgPBDParticleSystemCore.h"
#include "PxgParticleSystemBuffer.h"
#include "PxgSoftBodyBuffer.h"
#include "PxgFEMClothBuffer.h"
#include "PxgSimulationCoreKernelIndices.h"
#include "PxgBodySim.h"
#include "PxsCachedTransform.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgCudaUtils.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgKernelLauncher.h"
#include "PxgD6JointData.h"
#include "PxgConstraintPrep.h"
#include "PxgCudaUtils.h"
#include "PxgArticulationLink.h"
#include "PxgArticulationTendon.h"
#include "DyArticulationJointCore.h"
#include "DyArticulationMimicJointCore.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "PxgArticulation.h"
#include "PxgSoftBody.h"
#include "PxgFEMCloth.h"
#include "PxgParticleSystem.h"
#include "PxSpatialMatrix.h"
#include "DyFeatherstoneArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgBodySimManager.h"
#include "DyParticleSystem.h"
#include "PxgSoftBodyCore.h"
#include "PxgFEMClothCore.h"
#include "DyDeformableSurface.h"
#include "DyDeformableVolume.h"
#include "PxgContext.h"
#include "PxgAABBManager.h"
#include "PxgSimulationController.h"
#include "PxArticulationTendonData.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaTypes.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxAssert.h"
#include "foundation/PxVec4.h"

#define SC_GPU_DEBUG 0

#define GPU_VERIFY_JOINT_UPDATE	0

using namespace physx;

PxgArticulationBuffer::PxgArticulationBuffer(PxgHeapMemoryAllocator& deviceAlloc) :
	links(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	linkWakeCounters(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	linkSleepData(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	linkProps(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	joints(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	jointData(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	coriolisVectors(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	zAForces(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	pathToRoots(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	spatialTendonParams(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	spatialTendons(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	fixedTendonParams(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	fixedTendons(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mimicJoints(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	externalAccelerations(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	jointForce(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	jointTargetPositions(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	jointTargetVelocities(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	jointOffsets(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	parents(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	motionMatrix(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	motionMatrixW(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	spatialArticulatedInertiaW(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	spatialImpulseResponseW(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	linkAndJointAndRootStates(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	linkBody2Actors(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	children(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	relativeQuats(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	cfms(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	cfmScale(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	tempParentToChilds(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	tempRs(deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION)
{
}

PxgArticulationBuffer::~PxgArticulationBuffer()
{
	const PxU32 numTendonAttachmentFixedBuffer = attachmentFixedData.size();
	for (PxU32 i = 0; i < numTendonAttachmentFixedBuffer; ++i)
	{
		PxgCudaBuffer* buffer = attachmentFixedData[i];
		if (buffer)
		{
			buffer->deallocate();
			PX_FREE(buffer);
		}
	}

	const PxU32 numTendonAttachmentModBuffer = attachmentModData.size();
	for (PxU32 i = 0; i < numTendonAttachmentModBuffer; ++i)
	{
		PxgCudaBuffer* buffer = attachmentModData[i];
		if (buffer)
		{
			buffer->deallocate();
			PX_FREE(buffer);
		}
	}

	const PxU32 numTendonTendonJointFixedBuffer = tendonJointFixData.size();
	for (PxU32 i = 0; i < numTendonTendonJointFixedBuffer; ++i)
	{
		PxgCudaBuffer* buffer = tendonJointFixData[i];
		if (buffer)
		{
			buffer->deallocate();
			PX_FREE(buffer);
		}
	}

	const PxU32 numTendonTendonJointCoefficientBuffer = tendonJointCoefficientData.size();
	for (PxU32 i = 0; i < numTendonTendonJointCoefficientBuffer; ++i)
	{
		PxgCudaBuffer* buffer = tendonJointCoefficientData[i];
		if (buffer)
		{
			buffer->deallocate();
			PX_FREE(buffer);
		}
	}
}


PxgParticleSystemBuffer::PxgParticleSystemBuffer(PxgAllocatorDesc& allocDesc) :
	originalPosition_mass(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	grid_particle_hash(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	grid_particle_index(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	sorted_position_mass(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	sorted_velocity(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	accumDeltaV(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	sortedDeltaP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	cell_start(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	cell_end(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	collision_headers(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	collision_counts(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	collision_index(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	collision_impulses(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	phases(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	unsortedpositions(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	unsortedvelocities(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	restArray(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	normal(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	density(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	staticDensity(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	surfaceNormal(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	delta(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	curl(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	sorted_originalPosition_mass(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	sortedPhaseArray(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	particleOneWayContacts(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	particleOneWayForces(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	particleOneWayContactsNodeIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	particleOneWayContactsSurfaceVelocities(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	particleOneWayContactCount(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	reverseLookup(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	phase_group_to_material_handle(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	derivedPBDMaterialProperties(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_particle_buffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_particle_buffer_runsum(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_particle_buffer_sorted_unique_ids(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_particle_buffer_runsum_sorted_unique_ids_original_index(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_cloth_buffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_rigid_buffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	user_diffuse_buffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	attachmentRunSum(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	referencedRigidsRunsum(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mHostParticleBuffers(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mHostClothBuffers(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mHostRigidBuffers(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mHostDiffuseBuffers(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mAttachmentRunSum(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mParticleBufferRunSum(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mReferencedRigidsRunsum(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mParticleBufferSortedUniqueIds(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mParticleBufferSortedUniqueIdsOriginalIndex(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mRandomTable(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	mHostPhaseGroupToMaterialHandle(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES)
{
}

PxgParticleSystemDiffuseBuffer::PxgParticleSystemDiffuseBuffer(PxgAllocatorDesc& allocDesc) :
	diffuse_positions(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_velocities(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_potentials(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_cell_start(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_cell_end(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_grid_particle_hash(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_sorted_to_unsorted_mapping(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_unsorted_to_sorted_mapping(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_origin_pos_life_time(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_sorted_pos_life_time(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_sorted_origin_pos_life_time(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_sorted_vel(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_one_way_contacts(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_one_way_forces(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_one_way_contacts_node_indices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_one_way_contact_count(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
	diffuse_particle_count(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_PARTICLES)
	{
	}

PxgSoftBodyBuffer::PxgSoftBodyBuffer(PxgAllocatorDesc& allocDesc) :
	tetMeshData(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetMeshSurfaceHint(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetIndicesRemapTable(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetStresses(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetStressCoefficient(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetRestPoses(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetRotations(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetIndicesGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pPostion_InvMassGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	vertsAreDeformed(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	vertsCantDeform(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetRestPosesGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	origTetRestPosesGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetRotationsGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	orderedTetGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	jacobiVertIndicesGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetMultipliersGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pDeltaVGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pBarycentricGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pRemapGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetRemapColToSim(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	tetAccumulatedRemapColToSim(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	surfaceVertsHint(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
	surfaceVertToTetRemap(allocDesc.deviceAlloc, PxsHeapStats::eSHARED_SOFTBODY),
	pDeltaPosGM(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pPosition_InvMassGMCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pVelocity_InvMassGMCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	remapOutputGMCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	accumulatedPartitionsGMCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	accumulatedCopiesGMCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	pullIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	orderedMaterialIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	materialIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	packedNodeBounds(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	filterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY)
{
}

PxgFEMClothBuffer::PxgFEMClothBuffer(PxgAllocatorDesc& allocDesc) :
	triangleMeshData(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	deltaPos(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	accumulatedDeltaPos(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	accumulatedDeltaVel(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	prevPositionInContactOffset(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	prevPositionInRestOffset(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	materialIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	dynamicfrictions(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	trianglesWithActiveEdges(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	triangleVertexIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedNonSharedTriangleVertexIndices_triIndex(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedSharedTriangleLambdas(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedNonSharedTriangleLambdas(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedNonSharedTriangleRestPoseInv(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedSharedTrianglePairVertexIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedNonSharedTrianglePairVertexIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedSharedRestBendingAngle_flexuralStiffness_damping(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedNonSharedRestBendingAngle_flexuralStiffness_damping(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedSharedRestEdge0_edge1(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	orderedSharedRestEdgeLength_material0_material1(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	sharedBendingLambdas(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	nonSharedBendingLambdas(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	position_InvMassCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	nonSharedTriAccumulatedPartitionsCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	sharedTriPairRemapOutputCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	nonSharedTriPairRemapOutputCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	sharedTriPairAccumulatedCopiesCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	nonSharedTriPairAccumulatedCopiesCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	sharedTriPairAccumulatedPartitionsCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	nonSharedTriPairAccumulatedPartitionsCP(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	packedNodeBounds(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	numPenetratedTets(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH)
{
}

PxgSimulationCore::PxgSimulationCore(PxgCudaKernelWranglerManager* gpuKernelWrangler,
	PxCudaContextManager* cudaContextManager,
	PxgAllocatorDesc& allocDesc,
	PxgGpuContext* gpuContext,
	const bool useGpuBroadphase) :
	mGpuContext(gpuContext),
	mGpuKernelWranglerManager(gpuKernelWrangler),
	mCudaContextManager(cudaContextManager),
	mCudaContext(cudaContextManager->getCudaContext()),
	mAllocDesc(allocDesc),
	mUseGpuBp(useGpuBroadphase),
	mUpdatedCacheAndBoundsDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mNewBodiesDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mUpdateArticulationDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mUpdatedBodiesDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedJointsDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedActorDataDesc(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mFrozenBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUnfrozenBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mFrozenBlockAndResBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUnfrozenBlockAndResBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mActivateBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mDeactivateBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedDirectBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBodySimCudaBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBodySimPreviousVelocitiesCudaBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBodySimAccelerationsCudaBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBodySimAccelerationsPinned(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION),
	mPxgShapeSimManager(allocDesc),
	mArticulationBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationSleepDataBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationLinkBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationTraversalStackBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mTempPathToRootBitFieldStackBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mTempSharedBitFieldStackBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mTempRootBitFieldStackBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mPathToRootBitFieldStackBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationDofBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationMimicJointBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationSpatialTendonBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationSpatialTendonConstraintsBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationAttachmentBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationFixedTendonBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationFixedTendonConstraintsBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mArticulationTendonJointBatchBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mSoftBodyBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSoftBodyBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSelfCollisionSoftBodyBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyElementIndexBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mFEMClothBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mActiveFEMClothBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mFEMClothElementIndexBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mActiveFEMClothStateChangedMap(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mFEMClothWakeCounts(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mFEMClothWakeCountsGPU(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mActiveSBStateChangedMap(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mSBWakeCounts(allocDesc.hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mSBWakeCountsGPU(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mRigidJointBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mArtiJointBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mRigidJointPrePrepBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mArtiJointPrePrepBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mGpuConstraintIdMapDevice(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedBodySimBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewBodySimBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewArticulationBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mNewLinkBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkWakeCounterBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkExtAccelBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkPropBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkParentBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkChildBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkBody2WorldsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkBody2ActorsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewJointCoreBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewJointDataBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewLinkIndexBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewSpatialTendonParamsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewSpatialTendonsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewMimicJointBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewPathToRootBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewAttachmentFixedBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewAttachmentModBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewTendonAttachmentRemapBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewFixedTendonParamsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewFixedTendonsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewTendonJointsFixedBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewTendonJointsCoefficientBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mNewTendonTendonJointRemapBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mActiveNodeIndices(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedCacheAndBoundsDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBodiesDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mArticulationDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION),
	mUpdatedBodiesDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedJointDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mUpdatedActorDescBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mBoundsBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mChangedAABBMgrHandlesBuffer(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION),
	mSoftBodyRigidAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyRigidFilterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyRigidConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSoftBodyRigidConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mNumSoftBodyRigidAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyRigidAttachmentIds(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodySoftBodyAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodySoftBodyConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSoftBodySoftBodyConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mNumSoftBodySoftBodyAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyClothAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyClothFilterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyClothConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSoftBodyClothConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mNumSoftBodyClothAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyParticleAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyParticleFilterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mSoftBodyParticleConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mActiveSoftBodyParticleConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
	mClothRigidAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mActiveClothRigidAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothRigidFilterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothRigidConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mNumClothRigidAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothRigidAttachmentIds(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothClothAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mActiveClothClothAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothClothVertTriFilterPairs(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mClothClothConstraints(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mNumClothClothAttachments(allocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
	mNbTotalBodySim(0), mNbTotalArticulations(0),
	mNbTotalSoftBodies(0), mNbTotalFEMCloths(0),
	mMaxTetraVerts(0), mMaxTetrahedrons(0),
	mGMMaxPartitions(0), mGMMaxTetraVerts(0), 
	mGMMaxTetrahedrons(0), 
	mGMMaxTetrahedronsPerPartition(0),
	mGMMaxJacobiTetrahedrons(0),
	mGMMaxJacobiVertices(0), 
	mMaxNbClothVerts(0),
	mMaxNbClothTriangles(0),
	mMaxNbClothTrianglesWithActiveEdges(0),
	mMaxNbNonSharedTriangles(0),
	mMaxNbNonSharedTriPartitions(0),
	mMaxNbNonSharedTrianglesPerPartition(0),
	mMaxNbSharedTrianglePairs(0),
	mMaxNbNonSharedTrianglePairs(0),
	mMaxNbSharedTriPairPartitions(0),
	mMaxNbNonSharedTriPairPartitions(0),
	mMaxNonSharedTriClusterId(0),
	mMaxSharedTriPairClusterId(0),
	mMaxNonSharedTriPairClusterId(0),
	mMaxNbSharedTrianglePairsPerPartition(0),
	mMaxNbNonSharedTrianglePairsPerPartition(0),
	mMaxNbCollisionPairUpdatesPerTimestep(1),
	mMaxNbCollisionSubsteps(1),
	mNbTotalRigidJoints(0), mNbTotalArtiJoints(0),
	mUsePartitionAveraging(false),
	mHasActiveBendingPairs(false)
#if PX_SUPPORT_OMNI_PVD
	,mOvdDataBuffer(allocDesc.hostAlloc, PxsHeapStats::eOTHER)
	,mOvdIndexBuffer(allocDesc.hostAlloc, PxsHeapStats::eOTHER)
#endif
{
	mEventMapped = NULL;
	mCudaContextManager->acquireContext();

	mNbRigidSoftBodyAttachments = 0;
	mNbRigidSoftBodyFilters = 0;

	mNbSoftBodySoftBodyAttachments = 0;

	mNbClothSoftBodyAttachments = 0;
	mNbClothSoftBodyFilters = 0;

	mNbSoftBodyParticleAttachments = 0;
	mNbSoftBodyParticleFilters = 0;

	mNbRigidClothAttachments = 0;
	mNbRigidClothFilters = 0;

	mNbClothClothAttachments = 0;
	mNbClothClothVertTriFilters = 0;

	mUpdatedCacheAndBoundsDescBuffer.allocate(sizeof(PxgSimulationCoreDesc), PX_FL);
	mBodiesDescBuffer.allocate(sizeof(PxgNewBodiesDesc), PX_FL);
	mArticulationDescBuffer.allocate(sizeof(PxgUpdateArticulationDesc), PX_FL);
	mUpdatedBodiesDescBuffer.allocate(sizeof(PxgUpdatedBodiesDesc), PX_FL);
	mUpdatedJointDescBuffer.allocate(sizeof(PxgUpdatedJointsDesc), PX_FL);
	mUpdatedActorDescBuffer.allocate(sizeof(PxgUpdateActorDataDesc), PX_FL);

	mNumSoftBodyRigidAttachments.allocate(sizeof(PxU32), PX_FL);
	mNumSoftBodySoftBodyAttachments.allocate(sizeof(PxU32), PX_FL);
	mNumSoftBodyClothAttachments.allocate(sizeof(PxU32), PX_FL);
	mNumClothRigidAttachments.allocate(sizeof(PxU32), PX_FL);
	mNumClothClothAttachments.allocate(sizeof(PxU32), PX_FL);

	createGpuStreamsAndEvents();

	mCudaContextManager->releaseContext();
}

PxgSimulationCore::~PxgSimulationCore()
{
	mCudaContextManager->acquireContext();
	
	// AD: the tofree lists could contain entries if they were added less than one
	// frame ago. 
	for (PxU32 i = 0; i < mSoftBodiesToFree.size(); ++i)
	{
		mSoftBodiesToFree[i].deallocate(mAllocDesc.hostAlloc);
	}

	for (PxU32 i = 0; i < mClothsToFree.size(); ++i)
	{
		mClothsToFree[i].deallocate(mAllocDesc.hostAlloc);
	}

	for (PxU32 i = 0; i < mArticulationDataBuffer.size(); ++i)
	{
		PX_DELETE(mArticulationDataBuffer[i]);
	}

	for (PxU32 i = 0; i < mSoftBodyDataBuffer.size(); ++i)
	{
		PX_DELETE(mSoftBodyDataBuffer[i]);
	}

	for (PxU32 i = 0; i < mFEMClothDataBuffer.size(); ++i)
	{
		PX_DELETE(mFEMClothDataBuffer[i]);
	}

	PxgParticleSystemCore** particleCores = mGpuContext->getGpuParticleSystemCores();
	const PxU32 numCores = mGpuContext->getNbGpuParticleSystemCores();
	for (PxU32 i = 0; i < numCores; ++i)
	{
		PxgParticleSystemCore* particleCore = particleCores[i];
		particleCore->releaseParticleSystemDataBuffer();
	}

	releaseGpuStreamsAndEvents();

#if PX_SUPPORT_OMNI_PVD		
	mOvdDataBuffer.reset();
	mOvdIndexBuffer.reset();
#endif
	
	mCudaContextManager->releaseContext();
}

void PxgSimulationCore::createGpuStreamsAndEvents()
{
	//create stream
	mCudaContext->streamCreate(&mStream, CU_STREAM_NON_BLOCKING);

	//create event
	mCudaContext->eventCreate(&mEvent, CU_EVENT_DISABLE_TIMING);

	//create event
	mCudaContext->eventCreate(&mDmaEvent, CU_EVENT_DISABLE_TIMING);

	mEventMapped = PX_PINNED_MEMORY_ALLOC_FLAGS(PxU32, *mCudaContextManager, 1, CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP);
	if(!mEventMapped)
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgSimulationCore: failed to allocate pinned event");
		mCudaContextManager->getCudaContext()->setAbortMode(true);
	}
}

void PxgSimulationCore::releaseGpuStreamsAndEvents()
{
	//destroy stream
	mCudaContext->streamDestroy(mStream);
	mStream = NULL;

	//destroy event
	mCudaContext->eventDestroy(mEvent);
	mEvent = NULL;

	//destroy event
	mCudaContext->eventDestroy(mDmaEvent);
	mDmaEvent = NULL;

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mEventMapped);
}

void PxgSimulationCore::constructDescriptor(CUdeviceptr boundsd, CUdeviceptr changedAABBMgrHandlesd, const PxU32 nbTotalShapes, const PxU32 bitMapWordCounts)
{
	PxgSimulationCoreDesc& desc = mUpdatedCacheAndBoundsDesc.get();
	desc.mChangedAABBMgrHandles = reinterpret_cast<PxU32*>(changedAABBMgrHandlesd);
	desc.mFrozen = mFrozenBuffer.getTypedPtr();
	desc.mUnfrozen = mUnfrozenBuffer.getTypedPtr();
	desc.mFrozenBlockAndRes = mFrozenBlockAndResBuffer.getTypedPtr();
	desc.mUnfrozenBlockAndRes = mUnfrozenBlockAndResBuffer.getTypedPtr();
	desc.mUpdated = mUpdatedBuffer.getTypedPtr();
	desc.mActivate = mActivateBuffer.getTypedPtr();
	desc.mDeactivate = mDeactivateBuffer.getTypedPtr();

	desc.mSleepData = mGpuContext->mGpuSolverCore->getSolverBodySleepData().getPointer();

	desc.mTransformCache = reinterpret_cast<PxsCachedTransform*>(mGpuContext->mGpuNpCore->getTransformCache().getDevicePtr());

	desc.mBounds = reinterpret_cast<PxBounds3*>(boundsd);

	desc.mBodyDataIndices = mGpuContext->mGpuSolverCore->getSolverBodyIndices().getPointer();

	desc.mShapes = reinterpret_cast<PxgShape*>(mGpuContext->mGpuNpCore->mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	desc.mBodySimBufferDeviceData = getBodySimBufferDeviceData().getPointer();
	desc.mShapeSimsBufferDeviceData = mPxgShapeSimManager.getShapeSimsDeviceTypedPtr();
	desc.mArticulationPool = mArticulationBuffer.getTypedPtr();
	desc.mArticulationSleepDataPool = mArticulationSleepDataBuffer.getTypedPtr();
	desc.mNbTotalShapes = nbTotalShapes;
	desc.mBitMapWordCounts = bitMapWordCounts;

	desc.mTotalFrozenShapes = 0;
	desc.mTotalUnfrozenShapes = 0;
}

//new articulation and rigid bodies are both in newBodySimPool. We store the new rigid bodies first, then store new articulations
//bodySimOffset is articulation start point
void PxgSimulationCore::gpuMemDmaUpBodySim(Cm::PinnableArray<PxgBodySimVelocityUpdate>& updatedBodySimPool,
	Cm::PinnableArray<PxgBodySim>& newBodySimPool,
	Cm::PinnableArray<PxgArticulationLink>& newLinkPool,
	Cm::PinnableArray<PxReal>& newLinkWakeCounterPool,
	Cm::PinnableArray<Cm::UnAlignedSpatialVector>& newLinkExtAccelPool,
	Cm::PinnableArray<PxgArticulationLinkProp>& newLinkPropPool,
	Cm::PinnableArray<PxU32>& newLinkParentsPool,
	Cm::PinnableArray<Dy::ArticulationBitField>& newLinkChildPool,
	Cm::PinnableArray<PxTransform>& newLinkBody2WorldsPool,
	Cm::PinnableArray<PxTransform>& newLinkBody2ActorsPool,
	Cm::PinnableArray<Dy::ArticulationJointCore>& newJointCorePool,
	Cm::PinnableArray<Dy::ArticulationJointCoreData>& newJointDataPool,
	Cm::PinnableArray<PxgArticulationSimUpdate>& newLinkJointIndexPool,
	Cm::PinnableArray<PxgArticulation>& newArticulationPool,
	Cm::PinnableArray<PxGpuSpatialTendonData>& newSpatialTendonParamPool,
	Cm::PinnableArray<PxgArticulationTendon>& newSpatialTendonPool,
	Cm::PinnableArray<PxgArticulationTendonElementFixedData>& newAttachmentFixedPool,
	Cm::PinnableArray<PxGpuTendonAttachmentData>& newAttachmentModPool,
	Cm::PinnableArray<PxU32>& newTendonToAttachmentRemapPool,
	Cm::PinnableArray<PxGpuFixedTendonData>& newFixedTendonParamPool,
	Cm::PinnableArray<PxgArticulationTendon>& newFixedTendonPool,
	Cm::PinnableArray<PxgArticulationTendonElementFixedData>& newTendonJointFixedPool,
	Cm::PinnableArray<PxGpuTendonJointCoefficientData>& newTendonJointCoefficientPool,
	Cm::PinnableArray<PxU32>& newTendonToTendonJointRemapPool,
	Cm::PinnableArray<Dy::ArticulationMimicJointCore>& newMimicJointPool,
	Cm::PinnableArray<PxU32>& newPathToRootPool,
	PxU32 nbTotalBodies, PxU32 nbTotalArticulations, PxU32 maxLinks,
	PxU32 maxDofs, PxU32 maxMimicJoints, PxU32 maxSpatialTendons,
	PxU32 maxAttachments, PxU32 maxFixedTendons, PxU32 maxTendonJoints,
	bool enableBodyAccelerations)
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaUpBodySim", 0);
		
	const PxU32 nbNewBodies = newBodySimPool.size(); //this include rigid bodies and articulations
	//nbNewLinks = nbNewJoints = nbNewJointData
	const PxU32 nbNewLinks = newLinkPool.size();

	//This will dma rigid body and articulation altogether
	if (nbTotalBodies > mNbTotalBodySim)
	{
		{
			const PxU64 oldCapacity = mBodySimCudaBuffer.getSize();
			mBodySimCudaBuffer.allocateCopyOldDataAsync(nbTotalBodies*sizeof(PxgBodySim), mCudaContext, mStream, PX_FL);

			if (oldCapacity < mBodySimCudaBuffer.getSize())
				mCudaContext->memsetD32Async(mBodySimCudaBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mBodySimCudaBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		// PT: GPU acceleration buffers for acceleration getters
		if(enableBodyAccelerations)
		{
			// PT: we may be allocating more than needed here, as this is only needed for rigid bodies
			const PxU64 oldCapacity = mBodySimPreviousVelocitiesCudaBuffer.getSize();
			mBodySimPreviousVelocitiesCudaBuffer.allocateCopyOldDataAsync(nbTotalBodies*sizeof(PxgBodySimVelocities), mCudaContext, mStream, PX_FL);

			// PT: initialize this buffer to zero to make sure the initial previous velocities are 0.0
			if (oldCapacity < mBodySimPreviousVelocitiesCudaBuffer.getSize())
				mCudaContext->memsetD32Async(mBodySimPreviousVelocitiesCudaBuffer.getDevicePtr() + oldCapacity, 0, (mBodySimPreviousVelocitiesCudaBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);

			mBodySimAccelerationsCudaBuffer.allocateCopyOldDataAsync(nbTotalBodies * sizeof(PxgRigidBodyAcceleration), mCudaContext, mStream, PX_FL);
			if (mBodySimAccelerationsPinned.capacity() < nbTotalBodies)
			{
				mBodySimAccelerationsPinned.reserve(nbTotalBodies);
				mBodySimAccelerationsPinned.forceSize_Unsafe(nbTotalBodies);
			}
		}
		else
		{
			PX_ASSERT(getBodySimPrevVelocitiesBufferDevicePtr().mPtr == 0);
			PX_ASSERT(getBodySimPrevVelocitiesBufferDeviceData().mPtr == 0);
		}

		mNbTotalBodySim = nbTotalBodies;
	}

	//This will dma articulation 
	if (nbTotalArticulations > mNbTotalArticulations)
	{
		mArticulationDataBuffer.reserve(nbTotalArticulations);
		mArticulationDataBuffer.resize(nbTotalArticulations);

		const PxU64 oldCapacity = mArticulationBuffer.getSize();
		const PxU64 oldSleepCapacity = mArticulationSleepDataBuffer.getSize();

		//calculate the size of articulations
		mArticulationBuffer.allocateCopyOldDataAsync(nbTotalArticulations * sizeof(PxgArticulation), mCudaContext, mStream, PX_FL);
		mArticulationSleepDataBuffer.allocateCopyOldDataAsync(nbTotalArticulations * sizeof(PxgSolverBodySleepData), mCudaContext, mStream, PX_FL);

		if (oldCapacity < mArticulationBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mArticulationBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mArticulationBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		if (oldSleepCapacity < mArticulationSleepDataBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mArticulationSleepDataBuffer.getDevicePtr() + oldSleepCapacity, 0xFFFFFFFF, (mArticulationSleepDataBuffer.getSize() - oldSleepCapacity) / sizeof(PxU32), mStream);
		}

		mNbTotalArticulations = nbTotalArticulations;
	}

	//we has new bodies
	if (nbNewBodies)
	{
		mNewBodySimBuffer.allocate(nbNewBodies*sizeof(PxgBodySim), PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewBodySimBuffer.getDevicePtr(), newBodySimPool.begin(), sizeof(PxgBodySim)* nbNewBodies, mStream);
	}

	const PxU32 nbNewArticulations = newArticulationPool.size();
	//const PxU32 nbNewArticulationData = newArticulationDataPool.size();
	//new articulation
	if (nbNewArticulations)
	{
		for (PxU32 i = 0; i < newArticulationPool.size(); ++i)
		{
			PxgArticulation& newArticulation = newArticulationPool[i];

			const PxgArticulationData& data = newArticulation.data;
				
			PxgArticulationBuffer* buffer = mArticulationDataBuffer[data.index];

			if(!buffer)
			{
				buffer = PX_NEW(PxgArticulationBuffer)(mAllocDesc.deviceAlloc);
				mArticulationDataBuffer[data.index] = buffer;
			}

			const PxU32 numLinks = data.numLinks;
			const PxU32 numDofs = data.numJointDofs;
			const PxU32 numSpatialTendons = data.numSpatialTendons;
			const PxU32 numFixedTendons = data.numFixedTendons;
			const PxU32 numMimicJoints = data.numMimicJoints;
	
			buffer->links.allocate(sizeof(PxgArticulationLink) * numLinks, PX_FL);
			buffer->linkWakeCounters.allocate(sizeof(PxReal) * numLinks, PX_FL);
			buffer->linkSleepData.allocate(sizeof(PxgArticulationLinkSleepData) * numLinks, PX_FL);
			buffer->linkProps.allocate(sizeof(PxgArticulationLinkProp) * numLinks, PX_FL);
			buffer->joints.allocate(sizeof(Dy::ArticulationJointCore) * numLinks, PX_FL);
			buffer->jointData.allocate(sizeof(Dy::ArticulationJointCoreData) * numLinks, PX_FL);
			buffer->spatialTendonParams.allocate(sizeof(PxGpuSpatialTendonData) * numSpatialTendons, PX_FL);
			buffer->spatialTendons.allocate(sizeof(PxgArticulationTendon) * numSpatialTendons, PX_FL);

			if (buffer->attachmentFixedData.capacity() <= numSpatialTendons)
			{
				buffer->attachmentFixedData.reserve(numSpatialTendons);
				buffer->attachmentModData.reserve(numSpatialTendons);
			}

			buffer->fixedTendonParams.allocate(sizeof(PxGpuFixedTendonData) * numFixedTendons, PX_FL);
			buffer->fixedTendons.allocate(sizeof(PxgArticulationTendon) * numFixedTendons, PX_FL);

			if (buffer->tendonJointFixData.capacity() <= numFixedTendons)
			{
				buffer->tendonJointFixData.reserve(numFixedTendons);
				buffer->tendonJointCoefficientData.reserve(numFixedTendons);
			}

			PxgArticulationSimUpdate& simUpdate = newLinkJointIndexPool[i];
			PxU32 tendonStartIndex = simUpdate.spatialTendonStartIndex;

			for (PxU32 j = 0; j < numSpatialTendons; ++j)
			{
				PxgArticulationTendon& tendon = newSpatialTendonPool[tendonStartIndex + j];

				PxgCudaBuffer* attachmentFixedBuffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgCudaBuffer), "PxgCudaBuffer"), PxgCudaBuffer)(mAllocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION);
				PxgCudaBuffer* attachmentModBuffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgCudaBuffer), "PxgCudaBuffer"), PxgCudaBuffer)(mAllocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION);

				attachmentFixedBuffer->allocate(sizeof(PxgArticulationTendonElementFixedData) * tendon.mNbElements, PX_FL);
				attachmentModBuffer->allocate(sizeof(PxGpuTendonAttachmentData) * tendon.mNbElements, PX_FL);
				tendon.mFixedElements = reinterpret_cast<PxgArticulationTendonElementFixedData*>(attachmentFixedBuffer->getDevicePtr());
				tendon.mModElements = reinterpret_cast<PxGpuTendonAttachmentData*>(attachmentModBuffer->getDevicePtr());

				buffer->attachmentFixedData.pushBack(attachmentFixedBuffer);
				buffer->attachmentModData.pushBack(attachmentModBuffer);
			}

			tendonStartIndex = simUpdate.fixedTendonStartIndex;

			for (PxU32 j = 0; j < numFixedTendons; ++j)
			{
				PxgArticulationTendon& tendon = newFixedTendonPool[tendonStartIndex + j];

				PxgCudaBuffer* tendonJointFixedDataBuffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgCudaBuffer), "PxgCudaBuffer"), PxgCudaBuffer)(mAllocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION);
				PxgCudaBuffer* tendonJointCoefficientDataBuffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgCudaBuffer), "PxgCudaBuffer"), PxgCudaBuffer)(mAllocDesc.deviceAlloc, PxsHeapStats::eSIMULATION_ARTICULATION);

				tendonJointFixedDataBuffer->allocate(sizeof(PxgArticulationTendonElementFixedData) * tendon.mNbElements, PX_FL);
				tendonJointCoefficientDataBuffer->allocate(sizeof(PxGpuTendonJointCoefficientData) * tendon.mNbElements, PX_FL);

				tendon.mFixedElements = reinterpret_cast<PxgArticulationTendonElementFixedData*>(tendonJointFixedDataBuffer->getDevicePtr());
				tendon.mModElements = reinterpret_cast<PxGpuTendonJointCoefficientData*>(tendonJointCoefficientDataBuffer->getDevicePtr());
					
				buffer->tendonJointFixData.pushBack(tendonJointFixedDataBuffer);
				buffer->tendonJointCoefficientData.pushBack(tendonJointCoefficientDataBuffer);
			}

			buffer->mimicJoints.allocate(numMimicJoints*sizeof(Dy::ArticulationMimicJointCore), PX_FL);
				
			const PxU32 linkSize = sizeof(Cm::UnAlignedSpatialVector) * numLinks;
			buffer->coriolisVectors.allocate(linkSize, PX_FL);
			buffer->zAForces.allocate(linkSize, PX_FL);

			buffer->externalAccelerations.allocate(linkSize, PX_FL);

			//Allocate link/joint/root state data for 1 articulation with numLinks, numDofs.
			const PxU32 linkJointRootStateBufferByteSize = 
				PxgArticulationLinkJointRootStateData::computeStateDataBufferByteSizeAligned16(
					numLinks, numDofs, 1);
			buffer->linkAndJointAndRootStates.allocate(linkJointRootStateBufferByteSize, PX_FL);			
#if PX_DEBUG
			const PxU32 linkJointRootStateBufferByteSizeRaw =
				PxgArticulationLinkJointRootStateData::computeSingleArticulationStateDataBufferByteSizeRaw(
					numLinks, numDofs);
			if (linkJointRootStateBufferByteSize != linkJointRootStateBufferByteSizeRaw)
				mCudaContext->memsetD8Async(buffer->linkAndJointAndRootStates.getDevicePtr() + linkJointRootStateBufferByteSizeRaw, 0xce, linkJointRootStateBufferByteSize - linkJointRootStateBufferByteSizeRaw, mStream); //Improves readability of non-initialized memory sanitizer output
#endif

			buffer->linkBody2Actors.allocate(sizeof(PxTransform) * numLinks, PX_FL);
				
			buffer->jointOffsets.allocate(sizeof(PxU32) * numLinks, PX_FL);
			buffer->parents.allocate(sizeof(PxU32) * numLinks, PX_FL);
			buffer->motionMatrix.allocate(sizeof(Dy::SpatialSubspaceMatrix)*numLinks, PX_FL);
			buffer->motionMatrixW.allocate(sizeof(Dy::SpatialSubspaceMatrix)*numLinks, PX_FL);

			buffer->spatialArticulatedInertiaW.allocate(sizeof(PxSpatialMatrix) * numLinks, PX_FL);
			buffer->spatialImpulseResponseW.allocate(sizeof(PxSpatialMatrix)*numLinks, PX_FL);

			buffer->linkCount = numLinks;

			const PxU32 jointSize = sizeof(PxReal) * numDofs;
			buffer->jointForce.allocate(jointSize, PX_FL);
			buffer->jointTargetPositions.allocate(jointSize, PX_FL);
			buffer->jointTargetVelocities.allocate(jointSize, PX_FL);

			buffer->children.allocate(sizeof(ArticulationBitField)*numLinks, PX_FL);
			buffer->pathToRoots.allocate(sizeof(PxU32) * data.numPathToRoots, PX_FL);

			buffer->relativeQuats.allocate(sizeof(PxQuat)*numLinks, PX_FL);
			buffer->cfms.allocate(sizeof(PxReal)*numLinks, PX_FL);
			buffer->cfmScale.allocate(sizeof(PxReal)*numLinks, PX_FL);
			buffer->tempParentToChilds.allocate(sizeof(PxQuat) * numLinks, PX_FL);
			buffer->tempRs.allocate(sizeof(PxVec3) * numLinks, PX_FL);
			
			newArticulation.links = buffer->links.getTypedPtr();
			newArticulation.linkWakeCounters = buffer->linkWakeCounters.getTypedPtr();
			newArticulation.linkSleepData = buffer->linkSleepData.getTypedPtr();
			newArticulation.linkProps = buffer->linkProps.getTypedPtr();
			newArticulation.joints = buffer->joints.getTypedPtr();
			newArticulation.jointData = buffer->jointData.getTypedPtr();

			//links: body2World + velocity + incoming joint force
			//joints: position + velocity + acceleration
			//roots: pre-sim velocity
			newArticulation.linkJointRootStateDataBuffer = reinterpret_cast<PxU8*>(buffer->linkAndJointAndRootStates.getDevicePtr());
			PxgArticulationLinkJointRootStateData::decomposeArticulationStateDataBuffer(
				reinterpret_cast<PxU8*>(buffer->linkAndJointAndRootStates.getDevicePtr()),
				numLinks, numDofs,
				newArticulation.linkBody2Worlds, newArticulation.motionVelocities, newArticulation.motionAccelerations, newArticulation.linkIncomingJointForces,
				newArticulation.jointPositions, newArticulation.jointVelocities, newArticulation.jointAccelerations,
				newArticulation.rootPreMotionVelocity);

			newArticulation.linkBody2Actors = buffer->linkBody2Actors.getTypedPtr();
			newArticulation.coriolisVectors = buffer->coriolisVectors.getTypedPtr();
			newArticulation.externalAccelerations = buffer->externalAccelerations.getTypedPtr();
			newArticulation.zAForces = buffer->zAForces.getTypedPtr();

			newArticulation.jointTargetPositions = buffer->jointTargetPositions.getTypedPtr();
			newArticulation.jointTargetVelocities = buffer->jointTargetVelocities.getTypedPtr();
			newArticulation.jointForce = buffer->jointForce.getTypedPtr();
			newArticulation.jointOffsets = buffer->jointOffsets.getTypedPtr();
			newArticulation.parents = buffer->parents.getTypedPtr();
			newArticulation.motionMatrix = buffer->motionMatrix.getTypedPtr();
			newArticulation.worldMotionMatrix = buffer->motionMatrixW.getTypedPtr();
			newArticulation.worldSpatialArticulatedInertia = buffer->spatialArticulatedInertiaW.getTypedPtr();
			newArticulation.spatialResponseMatrixW = buffer->spatialImpulseResponseW.getTypedPtr();

			newArticulation.children = buffer->children.getTypedPtr();
			newArticulation.pathToRoot = buffer->pathToRoots.getTypedPtr();

			newArticulation.relativeQuat = buffer->relativeQuats.getTypedPtr();
			newArticulation.cfms = buffer->cfms.getTypedPtr();
			newArticulation.cfmScale = buffer->cfmScale.getTypedPtr();

			newArticulation.tempParentToChilds = buffer->tempParentToChilds.getTypedPtr();
			newArticulation.tempRs = buffer->tempRs.getTypedPtr();

			newArticulation.spatialTendonParams = buffer->spatialTendonParams.getTypedPtr();
			newArticulation.spatialTendons = buffer->spatialTendons.getTypedPtr();

			newArticulation.fixedTendonParams = buffer->fixedTendonParams.getTypedPtr();
			newArticulation.fixedTendons = buffer->fixedTendons.getTypedPtr();

			newArticulation.mimicJointCores = buffer->mimicJoints.getTypedPtr();
		}
		mNewArticulationBuffer.allocate(nbNewArticulations * sizeof(PxgArticulation), PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewArticulationBuffer.getDevicePtr(), newArticulationPool.begin(), sizeof(PxgArticulation)* nbNewArticulations, mStream);
	}

	if (nbNewLinks)
	{
		//KS - TODO - we are only going to access this memory once. Furthermore, some data may be sparse (may not even exist for a given articulation)
		//so perhaps we could figure out a way to use mapped memory to avoid all these DMAs?
		mNewLinkBuffer.allocate(sizeof(PxgArticulationLink) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkBuffer.getDevicePtr(), newLinkPool.begin(), sizeof(PxgArticulationLink) * nbNewLinks, mStream);

		mNewLinkWakeCounterBuffer.allocate(sizeof(PxReal) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkWakeCounterBuffer.getDevicePtr(), newLinkWakeCounterPool.begin(), sizeof(PxReal) * nbNewLinks, mStream);

		mNewLinkExtAccelBuffer.allocate(sizeof(Cm::UnAlignedSpatialVector) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkExtAccelBuffer.getDevicePtr(), newLinkExtAccelPool.begin(), sizeof(Cm::UnAlignedSpatialVector) * nbNewLinks, mStream);
			
		mNewLinkPropBuffer.allocate(sizeof(PxgArticulationLinkProp) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkPropBuffer.getDevicePtr(), newLinkPropPool.begin(), sizeof(PxgArticulationLinkProp) * nbNewLinks, mStream);
	
		mNewLinkParentBuffer.allocate(sizeof(PxU32) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkParentBuffer.getDevicePtr(), newLinkParentsPool.begin(), sizeof(PxU32) * nbNewLinks, mStream);

		mNewLinkChildBuffer.allocate(sizeof(ArticulationBitField)*nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkChildBuffer.getDevicePtr(), newLinkChildPool.begin(), sizeof(ArticulationBitField)*nbNewLinks, mStream);

		mNewLinkBody2WorldsBuffer.allocate(sizeof(PxTransform) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkBody2WorldsBuffer.getDevicePtr(), newLinkBody2WorldsPool.begin(), sizeof(PxTransform) * nbNewLinks, mStream);

		mNewLinkBody2ActorsBuffer.allocate(sizeof(PxTransform) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkBody2ActorsBuffer.getDevicePtr(), newLinkBody2ActorsPool.begin(), sizeof(PxTransform) * nbNewLinks, mStream);

		mNewJointCoreBuffer.allocate(sizeof(Dy::ArticulationJointCore) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewJointCoreBuffer.getDevicePtr(), newJointCorePool.begin(), sizeof(Dy::ArticulationJointCore) * nbNewLinks, mStream);

		mNewJointDataBuffer.allocate(sizeof(Dy::ArticulationJointCoreData) * nbNewLinks, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewJointDataBuffer.getDevicePtr(), newJointDataPool.begin(), sizeof(Dy::ArticulationJointCoreData) * nbNewLinks, mStream);
	}

	const PxU32 nbNewMimicJoints = newMimicJointPool.size();
	if(nbNewMimicJoints)
	{
		mNewMimicJointBuffer.allocate(sizeof(Dy::ArticulationMimicJointCore) * nbNewMimicJoints, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewMimicJointBuffer.getDevicePtr(), newMimicJointPool.begin(), sizeof(Dy::ArticulationMimicJointCore)*nbNewMimicJoints, mStream);
	}

	const PxU32 nbNewPathToRoot = newPathToRootPool.size();
	if (nbNewPathToRoot)
	{
		mNewPathToRootBuffer.allocate(sizeof(PxU32) * nbNewPathToRoot, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewPathToRootBuffer.getDevicePtr(), newPathToRootPool.begin(), sizeof(PxU32) * nbNewPathToRoot, mStream);
	}

	const PxU32 nbLinkJointIndex = newLinkJointIndexPool.size();
		
	if (nbLinkJointIndex)
	{
		mNewLinkIndexBuffer.allocate(nbLinkJointIndex * sizeof(PxgArticulationSimUpdate), PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewLinkIndexBuffer.getDevicePtr(), newLinkJointIndexPool.begin(), sizeof(PxgArticulationSimUpdate) * nbLinkJointIndex, mStream);
	}

	const PxU32 nbNewSpatialTendons = newSpatialTendonPool.size();
	if (nbNewSpatialTendons)
	{
		mNewSpatialTendonParamsBuffer.allocate(sizeof(PxGpuSpatialTendonData) * nbNewSpatialTendons, PX_FL);
		mNewSpatialTendonsBuffer.allocate(sizeof(PxgArticulationTendon) * nbNewSpatialTendons, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewSpatialTendonParamsBuffer.getDevicePtr(), newSpatialTendonParamPool.begin(), sizeof(PxGpuSpatialTendonData) * nbNewSpatialTendons, mStream);
		mCudaContext->memcpyHtoDAsync(mNewSpatialTendonsBuffer.getDevicePtr(), newSpatialTendonPool.begin(), sizeof(PxgArticulationTendon) * nbNewSpatialTendons, mStream);

		const PxU32 nbNewAttachments = newAttachmentFixedPool.size();

		if (nbNewAttachments)
		{
			mNewAttachmentFixedBuffer.allocate(sizeof(PxgArticulationTendonElementFixedData) * nbNewAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewAttachmentFixedBuffer.getDevicePtr(), newAttachmentFixedPool.begin(), sizeof(PxgArticulationTendonElementFixedData) * nbNewAttachments, mStream);

			mNewAttachmentModBuffer.allocate(sizeof(PxGpuTendonAttachmentData) * nbNewAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewAttachmentModBuffer.getDevicePtr(), newAttachmentModPool.begin(), sizeof(PxGpuTendonAttachmentData) * nbNewAttachments, mStream);
			
			mNewTendonAttachmentRemapBuffer.allocate(sizeof(PxU32) * nbNewSpatialTendons, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewTendonAttachmentRemapBuffer.getDevicePtr(), newTendonToAttachmentRemapPool.begin(), sizeof(PxU32) * nbNewSpatialTendons, mStream);
		}
	}

	const PxU32 nbNewFixedTendons = newFixedTendonPool.size();
	if (nbNewFixedTendons)
	{
		mNewFixedTendonParamsBuffer.allocate(sizeof(PxGpuFixedTendonData) * nbNewFixedTendons, PX_FL);
		mNewFixedTendonsBuffer.allocate(sizeof(PxgArticulationTendon) * nbNewFixedTendons, PX_FL);
		mCudaContext->memcpyHtoDAsync(mNewFixedTendonParamsBuffer.getDevicePtr(), newFixedTendonParamPool.begin(), sizeof(PxGpuFixedTendonData) * nbNewFixedTendons, mStream);
		mCudaContext->memcpyHtoDAsync(mNewFixedTendonsBuffer.getDevicePtr(), newFixedTendonPool.begin(), sizeof(PxgArticulationTendon) * nbNewFixedTendons, mStream);

		const PxU32 nbNewTendonJoints = newTendonJointFixedPool.size();

		if (nbNewTendonJoints)
		{
			mNewTendonJointsFixedBuffer.allocate(sizeof(PxgArticulationTendonElementFixedData) * nbNewTendonJoints, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewTendonJointsFixedBuffer.getDevicePtr(), newTendonJointFixedPool.begin(), sizeof(PxgArticulationTendonElementFixedData) * nbNewTendonJoints, mStream);

			mNewTendonJointsCoefficientBuffer.allocate(sizeof(PxGpuTendonJointCoefficientData) * nbNewTendonJoints, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewTendonJointsCoefficientBuffer.getDevicePtr(), newTendonJointCoefficientPool.begin(), sizeof(PxGpuTendonJointCoefficientData) * nbNewTendonJoints, mStream);

			mNewTendonTendonJointRemapBuffer.allocate(sizeof(PxU32) * nbNewFixedTendons, PX_FL);
			mCudaContext->memcpyHtoDAsync(mNewTendonTendonJointRemapBuffer.getDevicePtr(), newTendonToTendonJointRemapPool.begin(), sizeof(PxU32) * nbNewFixedTendons, mStream);
		}
	}

	const PxU32 nbUpdatedBodies = updatedBodySimPool.size();
	if (nbUpdatedBodies)
	{
		mUpdatedBodySimBuffer.allocate(nbUpdatedBodies*sizeof(PxgBodySimVelocityUpdate), PX_FL);
		mCudaContext->memcpyHtoDAsync(mUpdatedBodySimBuffer.getDevicePtr(), updatedBodySimPool.begin(), sizeof(PxgBodySimVelocityUpdate)* nbUpdatedBodies, mStream);
	}

	if (nbNewBodies > 0 || nbNewLinks > 0 || nbNewSpatialTendons > 0 || nbNewFixedTendons > 0 || nbNewMimicJoints > 0)
	{
		//fill in descriptor
		PxgNewBodiesDesc& newBodiesDesc = mNewBodiesDesc.get();
		newBodiesDesc.mNewBodySim = mNewBodySimBuffer.getTypedPtr();
		newBodiesDesc.mBodySimBufferDeviceData = getBodySimBufferDeviceData().getPointer();
		// PdHC: For new bodies, kernel will initialize their previous velocities to their initial velocity
		newBodiesDesc.mPrevVelocitiesBuffer = getBodySimPrevVelocitiesBufferDeviceData().getPointer();
		newBodiesDesc.mNbNewBodies = nbNewBodies;
			
		PxgUpdateArticulationDesc& updateArtiDesc = mUpdateArticulationDesc.get();
		updateArtiDesc.mNewArticulations = mNewArticulationBuffer.getTypedPtr();
		updateArtiDesc.mNewLinks = mNewLinkBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkWakeCounters = mNewLinkWakeCounterBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkExtAccels = mNewLinkExtAccelBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkProps = mNewLinkPropBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkParents = mNewLinkParentBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkChildren = mNewLinkChildBuffer.getTypedPtr();
		updateArtiDesc.mNewLinkBody2Worlds = mNewLinkBody2WorldsBuffer.getTypedPtr();

		updateArtiDesc.mNewLinkBody2Actors = mNewLinkBody2ActorsBuffer.getTypedPtr();
		updateArtiDesc.mNewJointCores = mNewJointCoreBuffer.getTypedPtr();
		updateArtiDesc.mNewJointData = mNewJointDataBuffer.getTypedPtr();
		updateArtiDesc.mIndicesOffset = mNewLinkIndexBuffer.getTypedPtr();
		updateArtiDesc.mArticulationPool = mArticulationBuffer.getTypedPtr();
		updateArtiDesc.mArticulationSleepDataPool = mArticulationSleepDataBuffer.getTypedPtr();
		updateArtiDesc.mNewSpatialTendonParamsPool = mNewSpatialTendonParamsBuffer.getTypedPtr();
		updateArtiDesc.mNewSpatialTendonPool = mNewSpatialTendonsBuffer.getTypedPtr();
		updateArtiDesc.mNewAttachmentFixedPool = mNewAttachmentFixedBuffer.getTypedPtr();
		updateArtiDesc.mNewAttachmentModPool = mNewAttachmentModBuffer.getTypedPtr();
		updateArtiDesc.mNewTendonAttachmentRemapPool = mNewTendonAttachmentRemapBuffer.getTypedPtr();
		updateArtiDesc.mNewFixedTendonParamsPool = mNewFixedTendonParamsBuffer.getTypedPtr();
		updateArtiDesc.mNewFixedTendonPool = mNewFixedTendonsBuffer.getTypedPtr();
		updateArtiDesc.mNewTendonJointFixedPool = mNewTendonJointsFixedBuffer.getTypedPtr();
		updateArtiDesc.mNewTendonJointCoefficientPool = mNewTendonJointsCoefficientBuffer.getTypedPtr();
		updateArtiDesc.mNewTendonTendonJointRemapPool = mNewTendonTendonJointRemapBuffer.getTypedPtr();
		updateArtiDesc.mNewArticulationMimicJointPool = mNewMimicJointBuffer.getTypedPtr();

		updateArtiDesc.mNewPathToRootPool = mNewPathToRootBuffer.getTypedPtr();

		updateArtiDesc.mNbNewArticulations = nbNewArticulations;

		//dma up descriptor
		mCudaContext->memcpyHtoDAsync(mBodiesDescBuffer.getDevicePtr(), &newBodiesDesc, sizeof(PxgNewBodiesDesc), mStream);
		mCudaContext->memcpyHtoDAsync(mArticulationDescBuffer.getDevicePtr(), &updateArtiDesc, sizeof(PxgUpdateArticulationDesc), mStream);
	}

	if (nbUpdatedBodies > 0 || nbNewBodies > 0)
	{
		PxgUpdatedBodiesDesc& updatedBodiesDesc = mUpdatedBodiesDesc.get();
		updatedBodiesDesc.mBodySimBufferDeviceData = getBodySimBufferDeviceData().getPointer();
		updatedBodiesDesc.mUpdatedBodySim = reinterpret_cast<PxgBodySimVelocityUpdate*>(mUpdatedBodySimBuffer.getDevicePtr());
		updatedBodiesDesc.mNbUpdatedBodies = nbUpdatedBodies;

		mCudaContext->memcpyHtoDAsync(mUpdatedBodiesDescBuffer.getDevicePtr(), &updatedBodiesDesc, sizeof(PxgUpdatedBodiesDesc), mStream);
	}

	const PxU32 nbArticulationBatches = (nbTotalArticulations + 31)/32;

	mArticulationBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockData), PX_FL);
	mArticulationLinkBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockLinkData) * maxLinks, PX_FL);
	mArticulationTraversalStackBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationTraversalStackData) * maxLinks, PX_FL);
	const PxU32 wordSize = (maxLinks + 63) / 64;
	mTempPathToRootBitFieldStackBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBitFieldStackData) * wordSize, PX_FL);
	mTempSharedBitFieldStackBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBitFieldStackData) * wordSize, PX_FL);
	mTempRootBitFieldStackBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBitFieldStackData) * wordSize, PX_FL);
	mPathToRootBitFieldStackBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBitFieldData) * maxLinks * wordSize, PX_FL);
	mArticulationDofBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockDofData) * maxDofs, PX_FL);

	mArticulationMimicJointBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockMimicJointData) * maxMimicJoints, PX_FL);
		
	mArticulationSpatialTendonBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockSpatialTendonData) * maxSpatialTendons, PX_FL);
	mArticulationSpatialTendonConstraintsBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationInternalTendonConstraintData) *  maxSpatialTendons * maxAttachments, PX_FL);
	mArticulationAttachmentBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockAttachmentData) *  maxSpatialTendons * maxAttachments, PX_FL);
		
	mArticulationFixedTendonBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockFixedTendonData) * maxFixedTendons, PX_FL);
	mArticulationFixedTendonConstraintsBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationInternalTendonConstraintData) *  maxFixedTendons * maxTendonJoints, PX_FL);
	mArticulationTendonJointBatchBuffer.allocate(nbArticulationBatches * sizeof(PxgArticulationBlockTendonJointData) *  maxFixedTendons * maxTendonJoints, PX_FL);
}

void PxgSimulationCore::gpuMemDmaUpSoftBodies(Cm::PinnableArray<PxgSoftBody>& newSoftBodyPool,
	PxU32* newTetMeshByteSizePool,
	PxArray<PxgSoftBodyData>& newSoftBodyDataPool,
	PxArray<PxU32>& newSoftBodyNodeIndexPool,
	PxArray<PxU32>& newSoftBodyElememtIndexPool,
	Cm::PinnableArray<PxgSoftBody>& softBodyPool,
	PxArray<PxgSoftBodyData>& softBodyDataPool,
	Cm::PinnableArray<PxU32>& softBodyElementIndexPool,
	PxArray<PxU32>& softBodyNodeIndexPool,
	PxgBodySimManager& bodySimManager,
	SoftBodyAttachmentAndFilterData& data
	)
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaUpSoftBodies", 0);

	const PxU32 nbTotalSoftBodies = bodySimManager.mTotalNumSoftBodies;

	const PxU32 nbNewSoftBodies = newSoftBodyPool.size(); 
	CUstream bpStream = 0;
	if(mGpuContext->mGpuBp)
		bpStream = mGpuContext->mGpuBp->getBpStream();
	//This will allocate/dma soft body data 
	if (nbTotalSoftBodies > mNbTotalSoftBodies)
	{
		//mSoftBodyDataBuffer.reserve(nbTotalSoftBodies);
		mSoftBodyDataBuffer.resize(nbTotalSoftBodies);

		const PxU64 oldCapacity = mSoftBodyBuffer.getSize();
		const PxU64 eOldCapacity = mSoftBodyElementIndexBuffer.getSize();

		//calculate the total size of soft body 
		mSoftBodyBuffer.allocateCopyOldDataAsync(nbTotalSoftBodies * sizeof(PxgSoftBody), mCudaContext, bpStream, PX_FL);
		mSoftBodyElementIndexBuffer.allocateCopyOldDataAsync(nbTotalSoftBodies * sizeof(PxU32), mCudaContext, bpStream, PX_FL);

		if (oldCapacity < mSoftBodyBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mSoftBodyBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mSoftBodyBuffer.getSize() - oldCapacity) / sizeof(PxU32), bpStream);
			mCudaContext->memsetD32Async(mSoftBodyElementIndexBuffer.getDevicePtr() + eOldCapacity, 0xFFFFFFFF, (mSoftBodyElementIndexBuffer.getSize() - eOldCapacity) / sizeof(PxU32), bpStream);
		}

		softBodyPool.resize(nbTotalSoftBodies);
		softBodyDataPool.resize(nbTotalSoftBodies);
		softBodyElementIndexPool.resize(nbTotalSoftBodies);
		softBodyNodeIndexPool.resize(nbTotalSoftBodies);
		mActiveSBStateChangedMap.resize(nbTotalSoftBodies);
		mSBWakeCounts.resize(nbTotalSoftBodies);
		mSBWakeCountsGPU.allocate(sizeof(PxReal)*nbTotalSoftBodies, PX_FL);

		mNbTotalSoftBodies = nbTotalSoftBodies;
	}

	for (PxU32 i = 0; i < mSoftBodiesToFree.size(); ++i)
	{
		mSoftBodiesToFree[i].deallocate(mAllocDesc.hostAlloc);
	}

	mSoftBodiesToFree.forceSize_Unsafe(0);

	void** bodySimsLL = bodySimManager.mBodies.begin();

	//need to fill in soft body data
	for (PxU32 i = 0; i < nbNewSoftBodies; ++i)
	{
		PxgSoftBody& newSoftBody = newSoftBodyPool[i];
		PxgSoftBodyData& newSoftBodyData = newSoftBodyDataPool[i];

		const PxU32 gpuRemapIndex = newSoftBody.mGpuRemapIndex;

		mSoftBodiesToFree.pushBack(newSoftBody);

		PxU32 nodeIndex = newSoftBodyNodeIndexPool[i];
		Dy::DeformableVolume* dyDeformableVolume = reinterpret_cast<Dy::DeformableVolume*>(bodySimsLL[nodeIndex]);
		const Dy::DeformableVolumeCore& dySoftbodyCore = dyDeformableVolume->getCore();

		softBodyNodeIndexPool[gpuRemapIndex] = nodeIndex;

		PxgSoftBodyBuffer* buffer = mSoftBodyDataBuffer[gpuRemapIndex];

		if (!buffer)
		{
			buffer = PX_NEW(PxgSoftBodyBuffer)(mAllocDesc);
				
			mSoftBodyDataBuffer[gpuRemapIndex] = buffer;
		}

		const PxU32 numVerts = newSoftBody.mNumVerts;
		const PxU32 numTets = newSoftBody.mNumTets;
		const PxU32 tetMeshByteSize = newTetMeshByteSizePool[i];
		
		buffer->tetMeshData.allocate(tetMeshByteSize, PX_FL);

		buffer->tetMeshSurfaceHint.allocateElements(numTets, PX_FL);
		buffer->tetIndices.allocateElements(numTets, PX_FL);
		buffer->tetIndicesRemapTable.allocateElements(numTets, PX_FL);
		buffer->tetStresses.allocateElements(numTets, PX_FL);
		buffer->tetStressCoefficient.allocateElements(numTets, PX_FL);
		buffer->tetRestPoses.allocateElements(numTets, PX_FL);
		buffer->tetRotations.allocateElements(numTets, PX_FL);

		const PxU32 numVertsGM = newSoftBody.mNumVertsGM;
		const PxU32 numTetsGM = newSoftBody.mNumTetsGM;
		const PxU32 numPartitionsGM = newSoftBody.mNumPartitionsGM;
		const PxU32 numTetsPerElementGM = newSoftBody.mNumTetsPerElement;
		const PxU32 numElementsGM = numTetsGM / numTetsPerElementGM;
		const PxU32 numVertsPerElementGM = numTetsPerElementGM == 1 ? 4 : 8;

		buffer->pPostion_InvMassGM.allocateElements(numVertsGM, PX_FL);
		buffer->vertsAreDeformed.allocateElements(numVertsGM, PX_FL);
		buffer->vertsCantDeform.allocateElements(numVertsGM, PX_FL);
		
		buffer->pDeltaPosGM.allocateElements(numVertsGM, PX_FL);

		buffer->tetIndicesGM.allocateElements(numTetsGM, PX_FL);
		//This is a 32-wide block format
		buffer->tetRestPosesGM.allocateElements((numTetsGM+31)/32, PX_FL);
		buffer->origTetRestPosesGM.allocateElements((numTetsGM+31)/32, PX_FL);
		buffer->tetRotationsGM.allocateElements(numTetsGM, PX_FL);
		buffer->orderedTetGM.allocateElements(numTetsGM, PX_FL);
		buffer->jacobiVertIndicesGM.allocateElements(newSoftBody.mNumJacobiVertices, PX_FL);
		buffer->tetMultipliersGM.allocateElements((numTetsGM+31)/32, PX_FL);

		buffer->pDeltaVGM.allocateElements(numVertsGM, PX_FL);

		buffer->pBarycentricGM.allocateElements(numVerts, PX_FL);
		buffer->pRemapGM.allocateElements(numVerts, PX_FL);
		
		buffer->tetRemapColToSim.allocateElements(newSoftBodyData.mTetsRemapSize, PX_FL);
		buffer->tetAccumulatedRemapColToSim.allocateElements(numTets, PX_FL);
		buffer->surfaceVertToTetRemap.allocateElements(numVerts, PX_FL);
		buffer->surfaceVertsHint.allocateElements(numVerts, PX_FL);

		buffer->pVelocity_InvMassGMCP.allocateElements(newSoftBodyData.mRemapOutputSizeGM, PX_FL);
		buffer->pPosition_InvMassGMCP.allocateElements(newSoftBodyData.mRemapOutputSizeGM, PX_FL);
	
		if (newSoftBodyData.mRemapOutputSizeGM) // used for tet mesh only
		{
			buffer->remapOutputGMCP.allocateElements(numElementsGM * numVertsPerElementGM, PX_FL);
			buffer->accumulatedCopiesGMCP.allocateElements(numVertsGM, PX_FL);
		}

		buffer->accumulatedPartitionsGMCP.allocateElements(newSoftBody.mNumPartitionsGM, PX_FL);
		buffer->pullIndices.allocateElements(numElementsGM * (numVertsPerElementGM / 4), PX_FL); // considering pullIndices uses uint4.

		buffer->orderedMaterialIndices.allocateElements(numTetsGM, PX_FL);
		buffer->materialIndices.allocateElements(numTetsGM, PX_FL);
		buffer->packedNodeBounds.allocateElements(newSoftBodyData.mNbPackedNodes, PX_FL);

		//DMA data to GPU
		mCudaContext->memcpyHtoDAsync(buffer->tetMeshData.getDevicePtr(), newSoftBody.mTetMeshData, tetMeshByteSize, bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetMeshSurfaceHint.getDevicePtr(), newSoftBody.mTetMeshSurfaceHint, numTets * sizeof(PxU8), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetIndices.getDevicePtr(), newSoftBody.mTetIndices, numTets * sizeof(uint4), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetIndicesRemapTable.getDevicePtr(), newSoftBody.mTetIndicesRemapTable, numTets * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetRestPoses.getDevicePtr(), newSoftBody.mTetraRestPoses, numTets * sizeof(PxMat33), bpStream);
		
		mCudaContext->memcpyHtoDAsync(buffer->tetIndicesGM.getDevicePtr(), newSoftBody.mSimTetIndices, numTetsGM * sizeof(uint4), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetRestPosesGM.getDevicePtr(), newSoftBody.mSimTetraRestPoses, ((numTetsGM+31)/32) * sizeof(PxgMat33Block), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->origTetRestPosesGM.getDevicePtr(), newSoftBody.mSimTetraRestPoses, ((numTetsGM + 31) / 32) * sizeof(PxgMat33Block), bpStream);
			
		mCudaContext->memcpyHtoDAsync(buffer->orderedTetGM.getDevicePtr(), newSoftBody.mSimOrderedTetrahedrons, numElementsGM * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->jacobiVertIndicesGM.getDevicePtr(), newSoftBody.mSimJacobiVertIndices, newSoftBody.mNumJacobiVertices * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->pBarycentricGM.getDevicePtr(), newSoftBody.mVertsBarycentricInGridModel, numVerts * sizeof(float4), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->pRemapGM.getDevicePtr(), newSoftBody.mVertsRemapInGridModel, numVerts * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetRemapColToSim.getDevicePtr(), newSoftBody.mTetsRemapColToSim, newSoftBodyData.mTetsRemapSize * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->tetAccumulatedRemapColToSim.getDevicePtr(), newSoftBody.mTetsAccumulatedRemapColToSim, numTets * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->surfaceVertToTetRemap.getDevicePtr(), newSoftBody.mSurfaceVertToTetRemap, numVerts * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->surfaceVertsHint.getDevicePtr(), newSoftBody.mSurfaceVertsHint, numVerts * sizeof(PxU8), bpStream);

		if (newSoftBodyData.mRemapOutputSizeGM) // used for tet mesh only
		{
			mCudaContext->memcpyHtoDAsync(buffer->remapOutputGMCP.getDevicePtr(), newSoftBody.mSimRemapOutputCP, numElementsGM * numVertsPerElementGM * sizeof(PxU32), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->accumulatedCopiesGMCP.getDevicePtr(), newSoftBody.mSimAccumulatedCopiesCP, numVertsGM * sizeof(PxU32), bpStream);
		}

		mCudaContext->memcpyHtoDAsync(buffer->accumulatedPartitionsGMCP.getDevicePtr(), newSoftBody.mSimAccumulatedPartitionsCP, newSoftBody.mNumPartitionsGM * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->pullIndices.getDevicePtr(), newSoftBody.mSimPullIndices, (numElementsGM * numVertsPerElementGM) * sizeof(PxU32), bpStream);

		mCudaContext->memcpyHtoDAsync(buffer->orderedMaterialIndices.getDevicePtr(), newSoftBody.mOrderedMaterialIndices, numTetsGM * sizeof(PxU16), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->materialIndices.getDevicePtr(), newSoftBody.mMaterialIndices, numTetsGM * sizeof(PxU16), bpStream);
			
		mCudaContext->memsetD32Async(buffer->tetRotations.getDevicePtr(), 0, (sizeof(PxQuat) * numTets)/sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->tetRotationsGM.getDevicePtr(), 0, (sizeof(PxQuat) * numTetsGM)/sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->pDeltaPosGM.getDevicePtr(), 0, (sizeof(float4) * numVertsGM) / sizeof(PxU32), bpStream);
		
		mCudaContext->memsetD32Async(buffer->pDeltaVGM.getDevicePtr(), 0, (sizeof(float4) * numVertsGM) / sizeof(PxU32), bpStream);

		PxgSoftBody& softBody = softBodyPool[gpuRemapIndex];
		PxgSoftBodyData& softBodyData = softBodyDataPool[gpuRemapIndex];

		//this memory is allocated on shape attach/detach. User is responsible for initialization and DMA to GPU.
		softBody.mPosition_InvMass = reinterpret_cast<float4*>(dySoftbodyCore.positionInvMass);
		softBody.mRestPosition = reinterpret_cast<float4*>(dySoftbodyCore.restPosition);

		//this memory is allocated on simulation mesh attach/detach. User is responsible for initialization and DMA to GPU.
		softBody.mSimPosition_InvMass = reinterpret_cast<float4*>(dySoftbodyCore.simPositionInvMass);
		softBody.mSimVelocity_InvMass = reinterpret_cast<float4*>(dySoftbodyCore.simVelocity);

		softBody.mSimKinematicTarget = reinterpret_cast<const float4*>(dySoftbodyCore.kinematicTarget);

		softBody.mTetMeshData = reinterpret_cast<void*>(buffer->tetMeshData.getDevicePtr());
		softBody.mTetMeshSurfaceHint = buffer->tetMeshSurfaceHint.getTypedPtr();
		softBody.mTetIndices = buffer->tetIndices.getTypedPtr();
		softBody.mTetIndicesRemapTable = buffer->tetIndicesRemapTable.getTypedPtr();
		softBody.mTetraStresses = buffer->tetStresses.getTypedPtr();
		softBody.mTetraStressCoefficient = buffer->tetStressCoefficient.getTypedPtr();
		softBody.mTetraRestPoses = buffer->tetRestPoses.getTypedPtr();
		softBody.mTetraRotations = buffer->tetRotations.getTypedPtr();

		softBody.mSimTetIndices = buffer->tetIndicesGM.getTypedPtr();
		softBody.mSimTetraRestPoses = buffer->tetRestPosesGM.getTypedPtr();
		softBody.mOrigQinv = buffer->origTetRestPosesGM.getTypedPtr();
		softBody.mSimTetraRotations = buffer->tetRotationsGM.getTypedPtr();

		softBody.mVertsAreDeformed = buffer->vertsAreDeformed.getTypedPtr();
		softBody.mVertsCanNotDeform = buffer->vertsCantDeform.getTypedPtr();

		softBody.mSimDeltaPos = buffer->pDeltaPosGM.getTypedPtr();

		softBody.mSimOrderedTetrahedrons = buffer->orderedTetGM.getTypedPtr();
		softBody.mSimJacobiVertIndices = buffer->jacobiVertIndicesGM.getTypedPtr();

		softBody.mSimTetraMultipliers = buffer->tetMultipliersGM.getTypedPtr();
		softBody.mSimDelta = buffer->pDeltaVGM.getTypedPtr();

		softBody.mSimPosition_InvMassCP = buffer->pPosition_InvMassGMCP.getTypedPtr();
		softBody.mSimVelocity_invMassCP = buffer->pVelocity_InvMassGMCP.getTypedPtr();
		softBody.mSimAccumulatedPartitionsCP = buffer->accumulatedPartitionsGMCP.getTypedPtr();
		softBody.mSimPullIndices = buffer->pullIndices.getTypedPtr();

		if (newSoftBodyData.mRemapOutputSizeGM) // used for tet mesh only
		{
			softBody.mSimRemapOutputCP = reinterpret_cast<uint4*>(buffer->remapOutputGMCP.getDevicePtr());
			softBody.mSimAccumulatedCopiesCP = buffer->accumulatedCopiesGMCP.getTypedPtr();
		}

		softBody.mVertsBarycentricInGridModel = buffer->pBarycentricGM.getTypedPtr();
		softBody.mVertsRemapInGridModel = buffer->pRemapGM.getTypedPtr();
		softBody.mTetsRemapColToSim = buffer->tetRemapColToSim.getTypedPtr();
		softBody.mTetsAccumulatedRemapColToSim = buffer->tetAccumulatedRemapColToSim.getTypedPtr();
		softBody.mSurfaceVertsHint = buffer->surfaceVertsHint.getTypedPtr();
		softBody.mSurfaceVertToTetRemap = buffer->surfaceVertToTetRemap.getTypedPtr();

		softBody.mOrderedMaterialIndices = buffer->orderedMaterialIndices.getTypedPtr();
		softBody.mMaterialIndices = buffer->materialIndices.getTypedPtr();

		softBody.mPackedNodeBounds = buffer->packedNodeBounds.getTypedPtr();

		softBody.mNumTets = newSoftBody.mNumTets;
		softBody.mElementIndex = newSoftBody.mElementIndex;
		softBody.mGpuRemapIndex = newSoftBody.mGpuRemapIndex;
		softBody.mNumVerts = newSoftBody.mNumVerts;
			
		softBody.mLinearDamping = newSoftBody.mLinearDamping;
		softBody.mMaxLinearVelocity = newSoftBody.mMaxLinearVelocity;
		softBody.mPenBiasClamp = newSoftBody.mPenBiasClamp;

		softBody.mSettlingThreshold = newSoftBody.mSettlingThreshold;
		softBody.mSleepThreshold = newSoftBody.mSleepThreshold;
		softBody.mSettlingDamping = newSoftBody.mSettlingDamping;
		softBody.mSelfCollisionFilterDistance = newSoftBody.mSelfCollisionFilterDistance;
		softBody.mSelfCollisionStressTolerance = newSoftBody.mSelfCollisionStressTolerance;

		softBody.mInitialRotation = newSoftBody.mInitialRotation;
		softBody.mActorFlags = newSoftBody.mActorFlags;
		softBody.mBodyFlags = newSoftBody.mBodyFlags;
		softBody.mVolumeFlags = newSoftBody.mVolumeFlags;
		softBody.mNumVertsGM = numVertsGM;
		softBody.mNumTetsGM = numTetsGM;
		softBody.mNumPartitionsGM = numPartitionsGM;
		softBody.mNumTetsPerElement = newSoftBody.mNumTetsPerElement;
		softBody.mRestDistance = newSoftBody.mRestDistance;
		softBody.mOriginalContactOffset = newSoftBody.mOriginalContactOffset;
		softBody.mJacobiScale = newSoftBody.mJacobiScale;
		softBody.mNumJacobiVertices = newSoftBody.mNumJacobiVertices;

		softBodyData.mRemapOutputSizeGM = newSoftBodyData.mRemapOutputSizeGM;
		softBodyData.mMaxTetsPerPartitionsGM = newSoftBodyData.mMaxTetsPerPartitionsGM;

		softBodyElementIndexPool[gpuRemapIndex] = newSoftBodyElememtIndexPool[i];

		mMaxTetraVerts = PxMax(numVerts, mMaxTetraVerts);
		mMaxTetrahedrons = PxMax(numTets, mMaxTetrahedrons);

		mGMMaxPartitions = PxMax(numPartitionsGM, mGMMaxPartitions);
		mGMMaxTetraVerts = PxMax(numVertsGM, mGMMaxTetraVerts);
		mGMMaxTetrahedrons = PxMax(numTetsGM, mGMMaxTetrahedrons);
		
		mGMMaxTetrahedronsPerPartition = PxMax(newSoftBodyData.mMaxTetsPerPartitionsGM, mGMMaxTetrahedronsPerPartition);

		if (numPartitionsGM > SB_PARTITION_LIMIT)
		{
			const PxU32 numJacobiTets = newSoftBody.mSimAccumulatedPartitionsCP[SB_PARTITION_LIMIT] -
			                            newSoftBody.mSimAccumulatedPartitionsCP[SB_PARTITION_LIMIT - 1];
			mGMMaxJacobiTetrahedrons = PxMax(mGMMaxJacobiTetrahedrons, numJacobiTets);
			mGMMaxJacobiVertices = PxMax(mGMMaxJacobiVertices, newSoftBody.mNumJacobiVertices);
		}

		mUsePartitionAveraging = (newSoftBody.mNumTetsPerElement == 1) || mUsePartitionAveraging;
		
		mSBWakeCounts[gpuRemapIndex] = dySoftbodyCore.wakeCounter;
	}
		
	PxArray<Dy::DeformableVolume*>& dirtyDeformableVolumeForFilterPairs = *data.dirtyDeformableVolumeForFilterPairs;

	const PxU32 nbDirtyFilterPairs = dirtyDeformableVolumeForFilterPairs.size();
	for (PxU32 i = 0; i < nbDirtyFilterPairs; ++i)
	{
		Dy::DeformableVolume* deformableVolume = dirtyDeformableVolumeForFilterPairs[i];
		if (deformableVolume)
		{
			PX_COMPILE_TIME_ASSERT(sizeof(Dy::VolumeVolumeFilter) == sizeof(PxgNonRigidFilterPair));
			Cm::PinnableArray<PxgNonRigidFilterPair>& filterPairs = reinterpret_cast<Cm::PinnableArray<PxgNonRigidFilterPair>&>(*deformableVolume->mVolumeVolumeFilterPairs);

			const PxU32 gpuRemapIndex = deformableVolume->getGpuRemapId();

			PxgSoftBodyBuffer* buffer = mSoftBodyDataBuffer[gpuRemapIndex];

			const PxU32 nbFilterPairs = filterPairs.size();
			buffer->filterPairs.allocate(sizeof(PxgNonRigidFilterPair) * nbFilterPairs, PX_FL);

			mCudaContext->memcpyHtoDAsync(buffer->filterPairs.getDevicePtr(), filterPairs.begin(), nbFilterPairs * sizeof(PxgNonRigidFilterPair), bpStream);

			PxgSoftBody& gpuSoftBody = softBodyPool[gpuRemapIndex];
			gpuSoftBody.mFilteringPairs = reinterpret_cast<PxgNonRigidFilterPair*>(buffer->filterPairs.getDevicePtr());
			gpuSoftBody.mNumFilterPairs = nbFilterPairs;

			deformableVolume->mFilterDirty = false;
			deformableVolume->mFilterInDirtyList = false;
		}
	}

	if (data.dirtyRigidAttachments)
	{
		Cm::PinnableArray<PxgFEMRigidAttachment>& rigidAttachments = *data.rigidAttachments;
		const PxU32 numDirtyAttachments = rigidAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mSoftBodyRigidAttachments.allocate(sizeof(PxgFEMRigidAttachment)* numDirtyAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mSoftBodyRigidAttachments.getDevicePtr(), rigidAttachments.begin(), numDirtyAttachments * sizeof(PxgFEMRigidAttachment), bpStream);

			mSoftBodyRigidConstraints.allocate(sizeof(PxgFEMRigidAttachmentConstraint)*(numDirtyAttachments + 31) / 32, PX_FL);

			mSoftBodyRigidAttachmentIds.allocate(sizeof(PxNodeIndex)*numDirtyAttachments, PX_FL);

			mGpuContext->getGpuSoftBodyCore()->reserveRigidDeltaVelBuf(numDirtyAttachments);
		}

		Cm::PinnableArray<PxgRigidFilterPair>& rigidFilterPairs = *data.rigidFilterPairs;
		const PxU32 numDirtyPairs = rigidFilterPairs.size();

		mSoftBodyRigidFilterPairs.allocate(sizeof(PxgRigidFilterPair)* numDirtyPairs, PX_FL);
		mCudaContext->memcpyHtoDAsync(mSoftBodyRigidFilterPairs.getDevicePtr(), rigidFilterPairs.begin(), numDirtyPairs * sizeof(PxgRigidFilterPair), bpStream);

		mNbRigidSoftBodyFilters = numDirtyPairs;
	}

	if (data.dirtyActiveRigidAttachments)
	{
		Cm::PinnableArray<PxU32>& activeRigidAttachments = *data.activeRigidAttachments;
		const PxU32 nbActiveAttachments = activeRigidAttachments.size();
		if (nbActiveAttachments > 0)
		{
			mActiveSoftBodyRigidConstraints.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);

			mCudaContext->memcpyHtoDAsync(mActiveSoftBodyRigidConstraints.getDevicePtr(), activeRigidAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}

		mCudaContext->memsetD32Async(mNumSoftBodyRigidAttachments.getDevicePtr(), nbActiveAttachments, 1, bpStream);
		mNbRigidSoftBodyAttachments = nbActiveAttachments;
	}

	if (data.dirtySoftBodyAttachments)
	{
		Cm::PinnableArray<PxgFEMFEMAttachment>& softBodyAttachments = *data.softBodyAttachments;
		const PxU32 numDirtyAttachments = softBodyAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mSoftBodySoftBodyAttachments.allocate(sizeof(PxgFEMFEMAttachment)* numDirtyAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mSoftBodySoftBodyAttachments.getDevicePtr(), softBodyAttachments.begin(), numDirtyAttachments * sizeof(PxgFEMFEMAttachment), bpStream);

			mSoftBodySoftBodyConstraints.allocate(sizeof(PxgFEMFEMAttachmentConstraint)*(numDirtyAttachments + 31) / 32, PX_FL);
		}
	}

	if (data.dirtyActiveSoftBodyAttachments)
	{
		Cm::PinnableArray<PxU32>& activeSoftBodyAttachments = *data.activeSoftBodyAttachments;
		const PxU32 nbActiveAttachments = activeSoftBodyAttachments.size();
		if (nbActiveAttachments > 0)
		{
			mActiveSoftBodySoftBodyConstraints.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);

			mCudaContext->memcpyHtoDAsync(mActiveSoftBodySoftBodyConstraints.getDevicePtr(), activeSoftBodyAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}

		mCudaContext->memsetD32Async(mNumSoftBodySoftBodyAttachments.getDevicePtr(), nbActiveAttachments, 1, bpStream);
		mNbSoftBodySoftBodyAttachments = nbActiveAttachments;
	}

	if (data.dirtyClothAttachments)
	{
		Cm::PinnableArray<PxgFEMFEMAttachment>& clothAttachments = *data.clothAttachments;
		const PxU32 numDirtyAttachments = clothAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mSoftBodyClothAttachments.allocate(sizeof(PxgFEMFEMAttachment)* numDirtyAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mSoftBodyClothAttachments.getDevicePtr(), clothAttachments.begin(), numDirtyAttachments * sizeof(PxgFEMFEMAttachment), bpStream);

			mSoftBodyClothConstraints.allocate(sizeof(PxgFEMFEMAttachmentConstraint)*(numDirtyAttachments + 31) / 32, PX_FL);
		}

		Cm::PinnableArray<PxgNonRigidFilterPair>& clothFilterPairs = *data.clothFilterPairs;
		const PxU32 numDirtyPairs = clothFilterPairs.size();

		mSoftBodyClothFilterPairs.allocate(sizeof(PxgNonRigidFilterPair)* numDirtyPairs, PX_FL);
		mCudaContext->memcpyHtoDAsync(mSoftBodyClothFilterPairs.getDevicePtr(), clothFilterPairs.begin(), numDirtyPairs * sizeof(PxgNonRigidFilterPair), bpStream);

		mNbClothSoftBodyFilters = numDirtyPairs;
	}

	if (data.dirtyActiveClothAttachments)
	{
		Cm::PinnableArray<PxU32>& activeClothAttachments = *data.activeClothAttachments;
		const PxU32 nbActiveAttachments = activeClothAttachments.size();
		if (nbActiveAttachments > 0)
		{
			mActiveSoftBodyClothConstraints.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);

			mCudaContext->memcpyHtoDAsync(mActiveSoftBodyClothConstraints.getDevicePtr(), activeClothAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}

		mCudaContext->memsetD32Async(mNumSoftBodyClothAttachments.getDevicePtr(), nbActiveAttachments, 1, bpStream);
		mNbClothSoftBodyAttachments = nbActiveAttachments;
	}

	if (data.dirtyParticleAttachments)
	{
		Cm::PinnableArray<PxgFEMFEMAttachment>& particleAttachments = *data.particleAttachments;
		const PxU32 numDirtyAttachments = particleAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mSoftBodyParticleAttachments.allocate(sizeof(PxgFEMFEMAttachment)* numDirtyAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mSoftBodyParticleAttachments.getDevicePtr(), particleAttachments.begin(), numDirtyAttachments * sizeof(PxgFEMFEMAttachment), bpStream);

			mSoftBodyParticleConstraints.allocate(sizeof(PxgFEMFEMAttachmentConstraint)*(numDirtyAttachments + 31) / 32, PX_FL);
		}

		Cm::PinnableArray<PxgNonRigidFilterPair>& particleFilterPairs = *data.particleFilterPairs;
		const PxU32 numDirtyPairs = particleFilterPairs.size();

		mSoftBodyParticleFilterPairs.allocate(sizeof(PxgNonRigidFilterPair)* numDirtyPairs, PX_FL);
		mCudaContext->memcpyHtoDAsync(mSoftBodyParticleFilterPairs.getDevicePtr(), particleFilterPairs.begin(), numDirtyPairs * sizeof(PxgNonRigidFilterPair), bpStream);

		mNbSoftBodyParticleFilters = numDirtyPairs;
	}

	if (data.dirtyActiveParticleAttachments)
	{
		Cm::PinnableArray<PxU32>& activeParticlesAttachments = *data.activeParticleAttachments;
		const PxU32 nbActiveAttachments = activeParticlesAttachments.size();
		if (nbActiveAttachments > 0)
		{
			mActiveSoftBodyParticleConstraints.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);

			mCudaContext->memcpyHtoDAsync(mActiveSoftBodyParticleConstraints.getDevicePtr(), activeParticlesAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}

		mNbSoftBodyParticleAttachments = nbActiveAttachments;
	}

	const PxU32 nbActiveSoftBodies = bodySimManager.mActiveSoftbodiesStaging.size();
	bool activeBodiesDirty = bodySimManager.mActiveSoftbodiesDirty;
	if (activeBodiesDirty)
	{
		bodySimManager.mActiveSoftbodies.reserve(bodySimManager.mActiveSoftbodiesStaging.capacity());

		bodySimManager.mActiveSoftbodies.forceSize_Unsafe(nbActiveSoftBodies);

		PxU32* activeSoftBodies = bodySimManager.mActiveSoftbodies.begin();

		//Copy from staging to actual sim buffers!
		PxMemCopy(activeSoftBodies, bodySimManager.mActiveSoftbodiesStaging.begin(), sizeof(PxU32) * bodySimManager.mActiveSoftbodiesStaging.size());

		mActiveSoftBodyBuffer.allocate(sizeof(PxU32) * nbActiveSoftBodies, PX_FL);

		mCudaContext->memcpyHtoDAsync(mActiveSoftBodyBuffer.getDevicePtr(), activeSoftBodies, sizeof(PxU32) * nbActiveSoftBodies, bpStream);

		//Repeat above but this time for soft bodies with self collision
		const PxU32 nbActiveSelfCollisionSoftBodies = bodySimManager.mActiveSelfCollisionSoftBodiesStaging.size();

		bodySimManager.mActiveSelfCollisionSoftbodies.reserve(bodySimManager.mActiveSelfCollisionSoftBodiesStaging.capacity());

		bodySimManager.mActiveSelfCollisionSoftbodies.forceSize_Unsafe(nbActiveSelfCollisionSoftBodies);

		PxU32* activeSelfCollisionSoftBodies = bodySimManager.mActiveSelfCollisionSoftbodies.begin();

		//Copy from staging to actual sim buffers!
		PxMemCopy(activeSelfCollisionSoftBodies, bodySimManager.mActiveSelfCollisionSoftBodiesStaging.begin(), sizeof(PxU32) * bodySimManager.mActiveSelfCollisionSoftBodiesStaging.size());

		mActiveSelfCollisionSoftBodyBuffer.allocate(sizeof(PxU32) * nbActiveSelfCollisionSoftBodies, PX_FL);

		mCudaContext->memcpyHtoDAsync(mActiveSelfCollisionSoftBodyBuffer.getDevicePtr(), activeSelfCollisionSoftBodies, sizeof(PxU32) * nbActiveSelfCollisionSoftBodies, bpStream);

		bodySimManager.mActiveSoftbodiesDirty = false;
	}
	if (nbNewSoftBodies || activeBodiesDirty)
		mCudaContext->memcpyHtoDAsync(mSBWakeCountsGPU.getDevicePtr(), mSBWakeCounts.begin(), sizeof(PxReal)*nbTotalSoftBodies, bpStream);

	bool anyDirty = mGpuContext->mGpuSoftBodyCore->updateUserData(softBodyPool, softBodyNodeIndexPool, 
		bodySimManager.mActiveSoftbodies.begin(), nbActiveSoftBodies,
		bodySimsLL) || nbDirtyFilterPairs > 0;
		
	if (anyDirty)
	{
		//we have a cpu mirror of the soft body and we just dma the whole softbody buffer to gpu
		mCudaContext->memcpyHtoDAsync(mSoftBodyBuffer.getDevicePtr(), softBodyPool.begin(), sizeof(PxgSoftBody)* mNbTotalSoftBodies, bpStream);
		mCudaContext->memcpyHtoDAsync(mSoftBodyElementIndexBuffer.getDevicePtr(), softBodyElementIndexPool.begin(), sizeof(PxU32)* mNbTotalSoftBodies, bpStream);
	}
}

void PxgSimulationCore::gpuMemDmaUpFEMCloths(Cm::PinnableArray<PxgFEMCloth>& newFEMClothPool,
	PxU32* newTriangleMeshByteSizePool,
	PxArray<PxgFEMClothData>& newFEMClothDataPool,
	PxArray<PxU32>& newFEMClothNodeIndexPool,
	PxArray<PxU32>& newFEMClothElememtIndexPool,
	Cm::PinnableArray<PxgFEMCloth>& femClothPool,
	PxArray<PxgFEMClothData>& femClothDataPool,
	Cm::PinnableArray<PxU32>& femClothElementIndexPool,
	PxArray<PxU32>& femClothNodeIndexPool,
	PxgBodySimManager& bodySimManager,
	Cm::PinnableArray<PxgFEMRigidAttachment>& rigidAttachments,
	Cm::PinnableArray<PxgRigidFilterPair>& rigidAttachmentIds,
	bool dirtyRigidAttachments,
	Cm::PinnableArray<PxU32>& activeRigidAttachments,
	bool dirtyActiveRigidAttachments,
	Cm::PinnableArray<PxgFEMFEMAttachment>& clothAttachments,
	Cm::PinnableArray<PxgNonRigidFilterPair>& clothVertTriFilterIds,
	bool dirtyClothAttachments,
	Cm::PinnableArray<PxU32>& activeClothAttachments,
	bool dirtyActiveClothAttachments
) 
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaUpFEMCloths", 0);

	const PxU32 nbTotalFEMCloths = bodySimManager.mTotalNumFEMCloths;

	const PxU32 nbNewFEMCloths = newFEMClothPool.size();
	CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

	// This will allocate/dma FEM-cloth data 
	if (nbTotalFEMCloths > mNbTotalFEMCloths)
	{
		//mFEMClothDataBuffer.reserve(nbTotalFEMCloths);
		mFEMClothDataBuffer.resize(nbTotalFEMCloths);

		const PxU64 oldCapacity = mFEMClothBuffer.getSize();
		const PxU64 eOldCapacity = mFEMClothElementIndexBuffer.getSize();

		// calculate the total size of FEM-cloth
		mFEMClothBuffer.allocateCopyOldDataAsync(nbTotalFEMCloths * sizeof(PxgFEMCloth), mCudaContext, bpStream, PX_FL);
		mFEMClothElementIndexBuffer.allocateCopyOldDataAsync(nbTotalFEMCloths * sizeof(PxU32), mCudaContext, bpStream, PX_FL);

		if (oldCapacity < mFEMClothBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mFEMClothBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mFEMClothBuffer.getSize() - oldCapacity) / sizeof(PxU32), bpStream);
			mCudaContext->memsetD32Async(mFEMClothElementIndexBuffer.getDevicePtr() + eOldCapacity, 0xFFFFFFFF, (mFEMClothElementIndexBuffer.getSize() - eOldCapacity) / sizeof(PxU32), bpStream);
		}

		femClothPool.resize(nbTotalFEMCloths);
		femClothDataPool.resize(nbTotalFEMCloths);
		femClothElementIndexPool.resize(nbTotalFEMCloths);
		femClothNodeIndexPool.resize(nbTotalFEMCloths);

		mActiveFEMClothStateChangedMap.resize(nbTotalFEMCloths);

		mFEMClothWakeCounts.resize(nbTotalFEMCloths);
		mFEMClothWakeCountsGPU.allocate(sizeof(PxReal)*nbTotalFEMCloths, PX_FL);
		
		mNbTotalFEMCloths = nbTotalFEMCloths;
	}

	void** bodySimsLL = bodySimManager.mBodies.begin();

	for (PxU32 i = 0; i < mClothsToFree.size(); ++i)
	{
		mClothsToFree[i].deallocate(mAllocDesc.hostAlloc);	
	}
	mClothsToFree.forceSize_Unsafe(0);

	// need to fill in FEM-cloth data
	for (PxU32 i = 0; i < nbNewFEMCloths; ++i)
	{
		PxgFEMCloth& newFEMCloth = newFEMClothPool[i];
		PxgFEMClothData& newFEMClothData = newFEMClothDataPool[i];

		mClothsToFree.pushBack(newFEMCloth);

		const PxU32 gpuRemapIndex = newFEMCloth.mGpuRemapIndex;

		PxU32 nodeIndex = newFEMClothNodeIndexPool[i];
		Dy::DeformableSurface* dyDeformableSurface = reinterpret_cast<Dy::DeformableSurface*>(bodySimsLL[nodeIndex]);
		const Dy::DeformableSurfaceCore& dyDeformableSurfaceCore = dyDeformableSurface->getCore();

		femClothNodeIndexPool[gpuRemapIndex] = nodeIndex;

		PxgFEMClothBuffer* buffer = mFEMClothDataBuffer[gpuRemapIndex];

		if (!buffer)
		{
			buffer = PX_NEW(PxgFEMClothBuffer)(mAllocDesc);

			mFEMClothDataBuffer[gpuRemapIndex] = buffer;
		}

		const PxU32 numVerts = newFEMCloth.mNbVerts;
		const PxU32 numTriangles = newFEMCloth.mNbTriangles;
		const PxU32 numNonSharedTriangles = newFEMCloth.mNbNonSharedTriangles;
		const PxU32 numSharedTriangles = numTriangles - numNonSharedTriangles;

		const PxU32 numSharedTrianglePairs = newFEMCloth.mNbSharedTrianglePairs;
		const PxU32 numNonSharedTrianglePairs = newFEMCloth.mNbNonSharedTrianglePairs;

		if (numNonSharedTriangles)
		{
			buffer->orderedNonSharedTriangleVertexIndices_triIndex.allocateElements(numNonSharedTriangles, PX_FL);
			buffer->orderedNonSharedTriangleRestPoseInv.allocateElements(numNonSharedTriangles, PX_FL);
			buffer->nonSharedTriAccumulatedPartitionsCP.allocateElements(newFEMCloth.mNbNonSharedTriPartitions, PX_FL);

			buffer->orderedNonSharedTriangleLambdas.allocateElements(numNonSharedTriangles, PX_FL);
		}
		if (numSharedTrianglePairs)
		{
			buffer->orderedSharedTrianglePairVertexIndices.allocateElements(numSharedTrianglePairs, PX_FL);
			buffer->orderedSharedRestBendingAngle_flexuralStiffness_damping.allocateElements(numSharedTrianglePairs, PX_FL);
			buffer->orderedSharedRestEdge0_edge1.allocateElements(numSharedTrianglePairs, PX_FL);
			buffer->orderedSharedRestEdgeLength_material0_material1.allocateElements(numSharedTrianglePairs, PX_FL);

			buffer->sharedTriPairRemapOutputCP.allocateElements(newFEMClothData.mSharedTriPairRemapOutputSize, PX_FL);
			buffer->sharedTriPairAccumulatedCopiesCP.allocateElements(numVerts, PX_FL);
			buffer->sharedTriPairAccumulatedPartitionsCP.allocateElements(newFEMCloth.mNbSharedTriPairPartitions, PX_FL);

			buffer->orderedSharedTriangleLambdas.allocateElements(numSharedTriangles, PX_FL);
			buffer->sharedBendingLambdas.allocateElements(numSharedTrianglePairs, PX_FL);
		}
		if (numNonSharedTrianglePairs)
		{
			buffer->orderedNonSharedTrianglePairVertexIndices.allocateElements(numNonSharedTrianglePairs, PX_FL);
			buffer->orderedNonSharedRestBendingAngle_flexuralStiffness_damping.allocateElements(numNonSharedTrianglePairs, PX_FL);
			buffer->nonSharedTriPairRemapOutputCP.allocateElements(newFEMClothData.mNonSharedTriPairRemapOutputSize, PX_FL);
			buffer->nonSharedTriPairAccumulatedCopiesCP.allocateElements(numVerts, PX_FL);
			buffer->nonSharedTriPairAccumulatedPartitionsCP.allocateElements(newFEMCloth.mNbNonSharedTriPairPartitions, PX_FL);

			buffer->nonSharedBendingLambdas.allocateElements(numNonSharedTrianglePairs, PX_FL);
		}

		const PxU32 triangleMeshByteSize = newTriangleMeshByteSizePool[i];

		buffer->deltaPos.allocateElements(numVerts, PX_FL);
		buffer->accumulatedDeltaPos.allocateElements(numVerts, PX_FL);
		buffer->accumulatedDeltaVel.allocateElements(numVerts, PX_FL);
		buffer->prevPositionInContactOffset.allocateElements(numVerts, PX_FL);
		buffer->prevPositionInRestOffset.allocateElements(numVerts, PX_FL);

		buffer->triangleMeshData.allocate(triangleMeshByteSize, PX_FL);
		buffer->trianglesWithActiveEdges.allocateElements(newFEMCloth.mNbTrianglesWithActiveEdges, PX_FL);
		buffer->triangleVertexIndices.allocateElements(numTriangles, PX_FL);

		PxU32 remapOutputSize = PxMax(newFEMClothData.mSharedTriPairRemapOutputSize, newFEMClothData.mNonSharedTriPairRemapOutputSize);
		buffer->position_InvMassCP.allocateElements(remapOutputSize, PX_FL);

		buffer->packedNodeBounds.allocateElements(newFEMClothData.mNbPackedNodes, PX_FL);

		buffer->numPenetratedTets.allocateElements(numVerts, PX_FL);

		buffer->materialIndices.allocateElements(numTriangles, PX_FL);
		buffer->dynamicfrictions.allocateElements(numVerts, PX_FL);

		// initialize delta positional change to zero
		mCudaContext->memsetD32Async(buffer->position_InvMassCP.getDevicePtr(), 0, (sizeof(float4) * remapOutputSize) / sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->deltaPos.getDevicePtr(), 0, (sizeof(float4) * numVerts) / sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->accumulatedDeltaPos.getDevicePtr(), 0, (sizeof(float4) * numVerts) / sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->accumulatedDeltaVel.getDevicePtr(), 0, (sizeof(float4) * numVerts) / sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->prevPositionInContactOffset.getDevicePtr(), 0, (sizeof(float4) * numVerts) / sizeof(PxU32), bpStream);
		mCudaContext->memsetD32Async(buffer->prevPositionInRestOffset.getDevicePtr(), 0, (sizeof(float4)* numVerts) / sizeof(PxU32), bpStream);

		// DMA data to GPU
		if (numNonSharedTriangles)
		{
			mCudaContext->memcpyHtoDAsync(buffer->orderedNonSharedTriangleVertexIndices_triIndex.getDevicePtr(), newFEMCloth.mOrderedNonSharedTriangleVertexIndices_triIndex, numNonSharedTriangles * sizeof(uint4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->orderedNonSharedTriangleRestPoseInv.getDevicePtr(), newFEMCloth.mOrderedNonSharedTriangleRestPoseInv, numNonSharedTriangles * sizeof(float4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->nonSharedTriAccumulatedPartitionsCP.getDevicePtr(), newFEMCloth.mNonSharedTriAccumulatedPartitionsCP, newFEMCloth.mNbNonSharedTriPartitions * sizeof(PxU32), bpStream);
		}

		if (numSharedTrianglePairs)
		{
			mCudaContext->memcpyHtoDAsync(buffer->orderedSharedTrianglePairVertexIndices.getDevicePtr(), newFEMCloth.mOrderedSharedTrianglePairVertexIndices, numSharedTrianglePairs * sizeof(uint4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->orderedSharedRestBendingAngle_flexuralStiffness_damping.getDevicePtr(), newFEMCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping, numSharedTrianglePairs * sizeof(float4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->orderedSharedRestEdge0_edge1.getDevicePtr(), newFEMCloth.mOrderedSharedRestEdge0_edge1, numSharedTrianglePairs * sizeof(float4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->orderedSharedRestEdgeLength_material0_material1.getDevicePtr(), newFEMCloth.mOrderedSharedRestEdgeLength_material0_material1, numSharedTrianglePairs * sizeof(float4), bpStream);

			mCudaContext->memcpyHtoDAsync(buffer->sharedTriPairRemapOutputCP.getDevicePtr(), newFEMCloth.mSharedTriPairRemapOutputCP, newFEMClothData.mSharedTriPairRemapOutputSize * sizeof(PxU32), bpStream);

			mCudaContext->memcpyHtoDAsync(buffer->sharedTriPairAccumulatedCopiesCP.getDevicePtr(), newFEMCloth.mSharedTriPairAccumulatedCopiesCP, numVerts * sizeof(PxU32), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->sharedTriPairAccumulatedPartitionsCP.getDevicePtr(), newFEMCloth.mSharedTriPairAccumulatedPartitionsCP, newFEMCloth.mNbSharedTriPairPartitions * sizeof(PxU32), bpStream);
		}

		if (numNonSharedTrianglePairs)
		{
			mCudaContext->memcpyHtoDAsync(buffer->orderedNonSharedTrianglePairVertexIndices.getDevicePtr(), newFEMCloth.mOrderedNonSharedTrianglePairVertexIndices, numNonSharedTrianglePairs * sizeof(uint4), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->orderedNonSharedRestBendingAngle_flexuralStiffness_damping.getDevicePtr(), newFEMCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping, numNonSharedTrianglePairs * sizeof(float4), bpStream);

			mCudaContext->memcpyHtoDAsync(buffer->nonSharedTriPairRemapOutputCP.getDevicePtr(), newFEMCloth.mNonSharedTriPairRemapOutputCP, newFEMClothData.mNonSharedTriPairRemapOutputSize * sizeof(PxU32), bpStream);

			mCudaContext->memcpyHtoDAsync(buffer->nonSharedTriPairAccumulatedCopiesCP.getDevicePtr(), newFEMCloth.mNonSharedTriPairAccumulatedCopiesCP, numVerts * sizeof(PxU32), bpStream);
			mCudaContext->memcpyHtoDAsync(buffer->nonSharedTriPairAccumulatedPartitionsCP.getDevicePtr(), newFEMCloth.mNonSharedTriPairAccumulatedPartitionsCP, newFEMCloth.mNbNonSharedTriPairPartitions * sizeof(PxU32), bpStream);
		}

		mCudaContext->memcpyHtoDAsync(buffer->triangleMeshData.getDevicePtr(), newFEMCloth.mTriMeshData, triangleMeshByteSize, bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->trianglesWithActiveEdges.getDevicePtr(), newFEMCloth.mTrianglesWithActiveEdges, newFEMCloth.mNbTrianglesWithActiveEdges * sizeof(PxU32), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->triangleVertexIndices.getDevicePtr(), newFEMCloth.mTriangleVertexIndices, numTriangles * sizeof(uint4), bpStream);

		mCudaContext->memcpyHtoDAsync(buffer->materialIndices.getDevicePtr(), newFEMCloth.mMaterialIndices, numTriangles * sizeof(PxU16), bpStream);
		mCudaContext->memcpyHtoDAsync(buffer->dynamicfrictions.getDevicePtr(), newFEMCloth.mDynamicFrictions, numVerts * sizeof(float), bpStream);

		PxgFEMCloth& femCloth = femClothPool[gpuRemapIndex];

		// allocated on attachShape, deallocated on detachShape, user is responsible for initialization.
		femCloth.mPosition_InvMass = reinterpret_cast<float4*>(dyDeformableSurfaceCore.positionInvMass);
		femCloth.mVelocity_InvMass = reinterpret_cast<float4*>(dyDeformableSurfaceCore.velocity);
		femCloth.mRestPosition = reinterpret_cast<float4*>(dyDeformableSurfaceCore.restPosition);

		femCloth.mPrevPositionInContactOffset = buffer->prevPositionInContactOffset.getTypedPtr();
		femCloth.mPrevPositionInRestOffset = buffer->prevPositionInRestOffset.getTypedPtr();

		femCloth.mTrianglesWithActiveEdges = buffer->trianglesWithActiveEdges.getTypedPtr();
		femCloth.mTriangleVertexIndices = buffer->triangleVertexIndices.getTypedPtr();

		if (numNonSharedTriangles)
		{
			femCloth.mOrderedNonSharedTriangleVertexIndices_triIndex = buffer->orderedNonSharedTriangleVertexIndices_triIndex.getTypedPtr();
			femCloth.mOrderedNonSharedTriangleRestPoseInv = buffer->orderedNonSharedTriangleRestPoseInv.getTypedPtr();
			femCloth.mNonSharedTriAccumulatedPartitionsCP = buffer->nonSharedTriAccumulatedPartitionsCP.getTypedPtr();
			
			femCloth.mOrderedNonSharedTriangleLambdas = buffer->orderedNonSharedTriangleLambdas.getTypedPtr();
		}
		if (numSharedTrianglePairs)
		{
			femCloth.mOrderedSharedTrianglePairVertexIndices = buffer->orderedSharedTrianglePairVertexIndices.getTypedPtr();
			femCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping = buffer->orderedSharedRestBendingAngle_flexuralStiffness_damping.getTypedPtr();
			femCloth.mOrderedSharedRestEdge0_edge1 = buffer->orderedSharedRestEdge0_edge1.getTypedPtr();
			femCloth.mOrderedSharedRestEdgeLength_material0_material1 = buffer->orderedSharedRestEdgeLength_material0_material1.getTypedPtr();

			femCloth.mSharedTriPairRemapOutputCP = buffer->sharedTriPairRemapOutputCP.getTypedPtr();
			femCloth.mSharedTriPairAccumulatedCopiesCP = buffer->sharedTriPairAccumulatedCopiesCP.getTypedPtr();
			femCloth.mSharedTriPairAccumulatedPartitionsCP = buffer->sharedTriPairAccumulatedPartitionsCP.getTypedPtr();

			femCloth.mOrderedSharedTriangleLambdas = buffer->orderedSharedTriangleLambdas.getTypedPtr();
			femCloth.mSharedBendingLambdas = buffer->sharedBendingLambdas.getTypedPtr();
		}
		if (numNonSharedTrianglePairs)
		{
			femCloth.mOrderedNonSharedTrianglePairVertexIndices = buffer->orderedNonSharedTrianglePairVertexIndices.getTypedPtr();
			femCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping = buffer->orderedNonSharedRestBendingAngle_flexuralStiffness_damping.getTypedPtr();

			femCloth.mNonSharedTriPairRemapOutputCP = buffer->nonSharedTriPairRemapOutputCP.getTypedPtr();

			femCloth.mNonSharedTriPairAccumulatedCopiesCP = buffer->nonSharedTriPairAccumulatedCopiesCP.getTypedPtr();
			femCloth.mNonSharedTriPairAccumulatedPartitionsCP = buffer->nonSharedTriPairAccumulatedPartitionsCP.getTypedPtr();

			femCloth.mNonSharedBendingLambdas = buffer->nonSharedBendingLambdas.getTypedPtr();
		}

		femCloth.mTriMeshData = reinterpret_cast<void*>(buffer->triangleMeshData.getDevicePtr());

		femCloth.mPosition_InvMassCP = buffer->position_InvMassCP.getTypedPtr();

		femCloth.mDeltaPos = buffer->deltaPos.getTypedPtr();
		femCloth.mAccumulatedDeltaPos = buffer->accumulatedDeltaPos.getTypedPtr(); 
		femCloth.mAccumulatedDeltaVel = buffer->accumulatedDeltaVel.getTypedPtr();

		femCloth.mPackedNodeBounds = buffer->packedNodeBounds.getTypedPtr();

		femCloth.mMaterialIndices = buffer->materialIndices.getTypedPtr();
		femCloth.mDynamicFrictions = buffer->dynamicfrictions.getTypedPtr();

		femCloth.mLinearDamping = newFEMCloth.mLinearDamping;
		femCloth.mMaxLinearVelocity = newFEMCloth.mMaxLinearVelocity;
		femCloth.mPenBiasClamp = newFEMCloth.mPenBiasClamp;

		femCloth.mSettlingThreshold = newFEMCloth.mSettlingThreshold;
		femCloth.mSleepThreshold = newFEMCloth.mSleepThreshold;
		femCloth.mSettlingDamping = newFEMCloth.mSettlingDamping;
		femCloth.mSelfCollisionFilterDistance = newFEMCloth.mSelfCollisionFilterDistance;

		femCloth.mNbVerts = newFEMCloth.mNbVerts;
		femCloth.mNbTriangles = newFEMCloth.mNbTriangles;
		femCloth.mNbNonSharedTriangles = newFEMCloth.mNbNonSharedTriangles;
		femCloth.mNbTrianglesWithActiveEdges = newFEMCloth.mNbTrianglesWithActiveEdges;

		femCloth.mNbTrianglePairs = newFEMCloth.mNbTrianglePairs;
		femCloth.mNbSharedTrianglePairs = newFEMCloth.mNbSharedTrianglePairs;
		femCloth.mNbNonSharedTrianglePairs = newFEMCloth.mNbNonSharedTrianglePairs;
		femCloth.mRestDistance = newFEMCloth.mRestDistance;
		femCloth.mOriginalContactOffset = newFEMCloth.mOriginalContactOffset;

		femCloth.mElementIndex = newFEMCloth.mElementIndex;
		femCloth.mGpuRemapIndex = newFEMCloth.mGpuRemapIndex;
		femCloth.mActorFlags = newFEMCloth.mActorFlags;
		femCloth.mBodyFlags = newFEMCloth.mBodyFlags;
		femCloth.mSurfaceFlags = newFEMCloth.mSurfaceFlags;

		femCloth.mNbCollisionPairUpdatesPerTimestep = newFEMCloth.mNbCollisionPairUpdatesPerTimestep;
		femCloth.mNbCollisionSubsteps = newFEMCloth.mNbCollisionSubsteps;

		femCloth.mNonSharedTriPair_hasActiveBending = newFEMCloth.mNonSharedTriPair_hasActiveBending;

		PxgFEMClothData& femClothData = femClothDataPool[gpuRemapIndex];

		femCloth.mNbNonSharedTriPartitions = newFEMCloth.mNbNonSharedTriPartitions;
		femClothData.mMaxNbNonSharedTrisPerPartition = newFEMClothData.mMaxNbNonSharedTrisPerPartition;

		femCloth.mNbSharedTriPairPartitions = newFEMCloth.mNbSharedTriPairPartitions;
		femCloth.mNbNonSharedTriPairPartitions = newFEMCloth.mNbNonSharedTriPairPartitions;

		femCloth.mNonSharedTriClusterId = newFEMCloth.mNonSharedTriClusterId;
		femCloth.mSharedTriPairClusterId = newFEMCloth.mSharedTriPairClusterId;
		femCloth.mNonSharedTriPairClusterId = newFEMCloth.mNonSharedTriPairClusterId;

		femClothData.mSharedTriPairRemapOutputSize = newFEMClothData.mSharedTriPairRemapOutputSize;
		femClothData.mNonSharedTriPairRemapOutputSize = newFEMClothData.mNonSharedTriPairRemapOutputSize;

		femClothData.mMaxNbSharedTriPairsPerPartition = newFEMClothData.mMaxNbSharedTriPairsPerPartition;
		femClothData.mMaxNbNonSharedTriPairsPerPartition = newFEMClothData.mMaxNbNonSharedTriPairsPerPartition;

		femClothElementIndexPool[gpuRemapIndex] = newFEMClothElememtIndexPool[i];

		mMaxNbClothVerts = PxMax(numVerts, mMaxNbClothVerts);
		mMaxNbClothTriangles = PxMax(numTriangles, mMaxNbClothTriangles);
		mMaxNbClothTrianglesWithActiveEdges = PxMax(newFEMCloth.mNbTrianglesWithActiveEdges, mMaxNbClothTrianglesWithActiveEdges);

		mMaxNbNonSharedTriPartitions = PxMax(newFEMCloth.mNbNonSharedTriPartitions, mMaxNbNonSharedTriPartitions);

		mMaxNbNonSharedTriangles = PxMax(numNonSharedTriangles, mMaxNbNonSharedTriangles);

		mMaxNbNonSharedTrianglesPerPartition = PxMax(newFEMClothData.mMaxNbNonSharedTrisPerPartition, mMaxNbNonSharedTrianglesPerPartition);
		mMaxNbSharedTriPairPartitions = PxMax(newFEMCloth.mNbSharedTriPairPartitions, mMaxNbSharedTriPairPartitions);
		mMaxNbNonSharedTriPairPartitions = PxMax(newFEMCloth.mNbNonSharedTriPairPartitions, mMaxNbNonSharedTriPairPartitions);

		mMaxNonSharedTriClusterId = PxMax(newFEMCloth.mNonSharedTriClusterId, mMaxNonSharedTriClusterId);
		mMaxSharedTriPairClusterId = PxMax(newFEMCloth.mSharedTriPairClusterId, mMaxSharedTriPairClusterId);
		mMaxNonSharedTriPairClusterId = PxMax(newFEMCloth.mNonSharedTriPairClusterId, mMaxNonSharedTriPairClusterId);

		mMaxNbSharedTrianglePairsPerPartition = PxMax(newFEMClothData.mMaxNbSharedTriPairsPerPartition, mMaxNbSharedTrianglePairsPerPartition);
		mMaxNbNonSharedTrianglePairsPerPartition = PxMax(newFEMClothData.mMaxNbNonSharedTriPairsPerPartition, mMaxNbNonSharedTrianglePairsPerPartition);

		mMaxNbSharedTrianglePairs = PxMax(numSharedTrianglePairs, mMaxNbSharedTrianglePairs);
		mMaxNbNonSharedTrianglePairs = PxMax(numNonSharedTrianglePairs, mMaxNbNonSharedTrianglePairs);

		if(newFEMCloth.mNbCollisionPairUpdatesPerTimestep == 0u)
		{
			mMaxNbCollisionPairUpdatesPerTimestep = 0u;
		}

		if (mMaxNbCollisionPairUpdatesPerTimestep != 0u)
		{
			mMaxNbCollisionPairUpdatesPerTimestep =
				PxMin(PxMax(newFEMCloth.mNbCollisionPairUpdatesPerTimestep, mMaxNbCollisionPairUpdatesPerTimestep),
					  static_cast<PxU32>(dyDeformableSurfaceCore.solverIterationCounts));
		}

		mMaxNbCollisionSubsteps = PxMax(newFEMCloth.mNbCollisionSubsteps, mMaxNbCollisionSubsteps);
		 
		mHasActiveBendingPairs = newFEMCloth.mNonSharedTriPair_hasActiveBending | mHasActiveBendingPairs;

		newFEMCloth.mMaxLinearVelocity = (newFEMCloth.mMaxLinearVelocity > 1.e15f) ? PX_MAX_REAL : newFEMCloth.mMaxLinearVelocity;

		mFEMClothWakeCounts[gpuRemapIndex] = dyDeformableSurfaceCore.wakeCounter;
	}

	if (dirtyRigidAttachments)
	{
		const PxU32 numDirtyAttachments = rigidAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mClothRigidAttachments.allocate(sizeof(PxgFEMRigidAttachment)*rigidAttachments.size(), PX_FL);
			mCudaContext->memcpyHtoDAsync(mClothRigidAttachments.getDevicePtr(), rigidAttachments.begin(), rigidAttachments.size() * sizeof(PxgFEMRigidAttachment), bpStream);

			mClothRigidConstraints.allocate(sizeof(PxgFEMRigidAttachmentConstraint)*(rigidAttachments.size() + 31) / 32, PX_FL);

			mClothRigidAttachmentIds.allocate(sizeof(PxNodeIndex)*rigidAttachments.size(), PX_FL);

			mGpuContext->getGpuFEMClothCore()->reserveRigidDeltaVelBuf(numDirtyAttachments);
		}

		mClothRigidFilterPairs.allocate(sizeof(PxgRigidFilterPair)*rigidAttachmentIds.size(), PX_FL);
		mCudaContext->memcpyHtoDAsync(mClothRigidFilterPairs.getDevicePtr(), rigidAttachmentIds.begin(), rigidAttachmentIds.size() * sizeof(PxgRigidFilterPair), bpStream);

		mNbRigidClothFilters = rigidAttachmentIds.size();
	}

	if (dirtyActiveRigidAttachments)
	{
		const PxU32 nbActiveAttachments = activeRigidAttachments.size();

		if (nbActiveAttachments > 0)
		{
			mActiveClothRigidAttachments.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mActiveClothRigidAttachments.getDevicePtr(), activeRigidAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}
		mCudaContext->memsetD32Async(mNumClothRigidAttachments.getDevicePtr(), nbActiveAttachments, 1, bpStream);
		mNbRigidClothAttachments = nbActiveAttachments;
	}

	//cloth vs cloth attachments
	if (dirtyClothAttachments)
	{
		const PxU32 numDirtyAttachments = clothAttachments.size();
		if (numDirtyAttachments > 0)
		{
			mClothClothAttachments.allocate(sizeof(PxgFEMFEMAttachment)*clothAttachments.size(), PX_FL);
			mCudaContext->memcpyHtoDAsync(mClothClothAttachments.getDevicePtr(), clothAttachments.begin(), clothAttachments.size() * sizeof(PxgFEMFEMAttachment), bpStream);

			mClothClothConstraints.allocate(sizeof(PxgFEMFEMAttachmentConstraint)*(clothAttachments.size() + 31) / 32, PX_FL);
		}

		mClothClothVertTriFilterPairs.allocate(sizeof(PxgNonRigidFilterPair)*clothVertTriFilterIds.size(), PX_FL);
		mCudaContext->memcpyHtoDAsync(mClothClothVertTriFilterPairs.getDevicePtr(), clothVertTriFilterIds.begin(), clothVertTriFilterIds.size() * sizeof(PxgNonRigidFilterPair), bpStream);

		mNbClothClothVertTriFilters = clothVertTriFilterIds.size();
	}

	if (dirtyActiveClothAttachments)
	{
		const PxU32 nbActiveAttachments = activeClothAttachments.size();

		if (nbActiveAttachments > 0)
		{
			mActiveClothClothAttachments.allocate(sizeof(PxU32)*nbActiveAttachments, PX_FL);
			mCudaContext->memcpyHtoDAsync(mActiveClothClothAttachments.getDevicePtr(), activeClothAttachments.begin(), nbActiveAttachments * sizeof(PxU32), bpStream);
		}
		mCudaContext->memsetD32Async(mNumClothClothAttachments.getDevicePtr(), nbActiveAttachments, 1, bpStream);
		mNbClothClothAttachments = nbActiveAttachments;
	}

	const PxU32 nbActiveFEMCloths = bodySimManager.mActiveFEMClothStaging.size();
		
	bool activeFEMClothsDirty = bodySimManager.mActiveFEMClothsDirty;
	if (activeFEMClothsDirty)
	{
		bodySimManager.mActiveFEMCloths.reserve(bodySimManager.mActiveFEMClothStaging.capacity());
		bodySimManager.mActiveFEMCloths.forceSize_Unsafe(nbActiveFEMCloths);
		PxU32* activeFEMCloths = bodySimManager.mActiveFEMCloths.begin();

		PxMemCopy(activeFEMCloths, bodySimManager.mActiveFEMClothStaging.begin(), sizeof(PxU32) * nbActiveFEMCloths);

		mActiveFEMClothBuffer.allocate(sizeof(PxU32) * nbActiveFEMCloths, PX_FL);
		mCudaContext->memcpyHtoDAsync(mActiveFEMClothBuffer.getDevicePtr(), activeFEMCloths, sizeof(PxU32) * nbActiveFEMCloths, bpStream);
		bodySimManager.mActiveFEMClothsDirty = false;
	}

	if (nbNewFEMCloths || activeFEMClothsDirty)
		mCudaContext->memcpyHtoDAsync(mFEMClothWakeCountsGPU.getDevicePtr(), mFEMClothWakeCounts.begin(), sizeof(PxReal)*nbTotalFEMCloths, bpStream);

	PxU32* activeFEMCloths = bodySimManager.mActiveFEMCloths.begin();

	bool anyDirty = mGpuContext->mGpuFEMClothCore->updateUserData(femClothPool, femClothNodeIndexPool, activeFEMCloths, nbActiveFEMCloths,
		bodySimsLL);

	if (anyDirty)
	{
		//we have a cpu mirror of the soft body and we just dma the whole particle buffer to gpu
		mCudaContext->memcpyHtoDAsync(mFEMClothBuffer.getDevicePtr(), femClothPool.begin(), sizeof(PxgFEMCloth)* mNbTotalFEMCloths, bpStream);
		mCudaContext->memcpyHtoDAsync(mFEMClothElementIndexBuffer.getDevicePtr(), femClothElementIndexPool.begin(), sizeof(PxU32)* mNbTotalFEMCloths, bpStream);
	}
}

void PxgSimulationCore::gpuMemDmaUpParticleSystem(PxgBodySimManager& bodySimManager)
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaUpParticleSystem", 0);

	CUstream bpStream = 0;
	if (mGpuContext->mGpuBp)
		bpStream = mGpuContext->mGpuBp->getBpStream();
	
	const PxU32 numParticleCores = mGpuContext->getNbGpuParticleSystemCores();
	PxgParticleSystemCore** particleCores = mGpuContext->getGpuParticleSystemCores();
	for (PxU32 i = 0; i < numParticleCores; ++i)
	{
		PxgParticleSystemCore* particleCore = particleCores[i];
		particleCore->gpuMemDmaUpParticleSystem(bodySimManager, bpStream);
	}
}

//This is called before Bp - only runs if direct-GPU API is on.
void PxgSimulationCore::mergeChangedAABBMgHandle()
{
	// AD: the scary thing here is that we initialized this during the previous sim step, so the pointers could all be wrong?
	
	CUstream stream = mGpuContext->getGpuBroadPhase()->getBpStream();

	CUdeviceptr updatedActorDescd = mUpdatedActorDescBuffer.getDevicePtr();
	CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::MERGE_AABBMGR_HANDLES);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(updatedActorDescd)
	};

	CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_AABBMGR_HANDLES, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_AABBMGR_HANDLES, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_UNUSED(result);
	PX_ASSERT(result == CUDA_SUCCESS);
}

//This is called after solver
void PxgSimulationCore::gpuMemDmaUp(const PxU32 nbTotalBodies, const PxU32 nbTotalShapes,
	Cm::PinnableBitMap& changedHandleMap, const bool enableDirectGPUAPI)
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaUp", 0);
	//these free buffers are based on shapes count
		
	mFrozenBuffer.allocate(nbTotalShapes*sizeof(PxU32), PX_FL);
	mUnfrozenBuffer.allocate(nbTotalShapes*sizeof(PxU32), PX_FL);

	// these are only needed for SQ tree updates.
	if (!enableDirectGPUAPI)
	{
		mFrozenBlockAndResBuffer.allocate(PxMax(32u, nbTotalShapes)*sizeof(PxU32), PX_FL);
		mUnfrozenBlockAndResBuffer.allocate(PxMax(32u, nbTotalShapes)*sizeof(PxU32), PX_FL);
	}

	//these two buffers are based on bodies count
	mActivateBuffer.allocate(nbTotalBodies*sizeof(PxU32), PX_FL);
	mDeactivateBuffer.allocate(nbTotalBodies*sizeof(PxU32), PX_FL);

	mActiveNodeIndices.allocate(nbTotalShapes * sizeof(PxU32), PX_FL);

	const PxU32 bitMapWordCounts = changedHandleMap.getWordCount();
	//const PxU32 roundElement = (nbTotalShapes + 31) & (~31);
	const PxU32 roundElement = bitMapWordCounts * 32;
	mUpdatedBuffer.allocate(roundElement * sizeof(PxU32), PX_FL);

	//initialize all the sleeping stage buffers to be 0
	mCudaContext->memsetD32Async(mFrozenBuffer.getDevicePtr(), 0, nbTotalShapes, mStream);
	mCudaContext->memsetD32Async(mUnfrozenBuffer.getDevicePtr(), 0, nbTotalShapes, mStream);
	mCudaContext->memsetD32Async(mUpdatedBuffer.getDevicePtr(), 0, roundElement, mStream);
	mCudaContext->memsetD32Async(mActivateBuffer.getDevicePtr(), 0, nbTotalBodies, mStream);
	mCudaContext->memsetD32Async(mDeactivateBuffer.getDevicePtr(), 0, nbTotalBodies, mStream);

	if (mUseGpuBp)
	{
		//re-allocate mChangedAABBMgrHandlesBuffer and mUpdatedBuffer(bitMapWordCounts is based on getElementIDPool().getMaxID())
		PxgAABBManager* aabbManager = mGpuContext->getGpuBroadPhase()->getAABBManager();

		constructDescriptor(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr(), aabbManager->getChangedAABBMgrHandles(), nbTotalShapes, bitMapWordCounts);
	}
	else
	{
		const PxU32 boundSize = mBoundArray->size();
		//dma up bounds
		mBoundsBuffer.allocate(sizeof(PxBounds3) * boundSize, PX_FL);
		mCudaContext->memcpyHtoDAsync(mBoundsBuffer.getDevicePtr(), mBoundArray->begin(), sizeof(PxBounds3) * boundSize, mStream);
		mChangedAABBMgrHandlesBuffer.allocate(sizeof(PxU32) * bitMapWordCounts, PX_FL);
		mCudaContext->memcpyHtoDAsync(mChangedAABBMgrHandlesBuffer.getDevicePtr(), changedHandleMap.getWords(), sizeof(PxU32) * bitMapWordCounts, mStream);

		constructDescriptor(mBoundsBuffer.getDevicePtr(), mChangedAABBMgrHandlesBuffer.getDevicePtr(), nbTotalShapes, bitMapWordCounts);
	}

	//DMA descriptor up
	mCudaContext->memcpyHtoDAsync(mUpdatedCacheAndBoundsDescBuffer.getDevicePtr(), mUpdatedCacheAndBoundsDesc.data(), sizeof(PxgSimulationCoreDesc), mStream);
}

//simulation controller need to call syncDmaback in the application before all datas are back to cpu
void PxgSimulationCore::gpuMemDmaBack(Cm::PinnableArray<PxU32>& frozenArray,
	Cm::PinnableArray<PxU32>& unfrozenArray,
	Cm::PinnableArray<PxU32>& activateArray,
	Cm::PinnableArray<PxU32>& deactiveArray,
	PxsCachedTransform* cachedTransforms,
	const PxU32 cachedCapacity, Bp::BoundsArray& boundArray,
	Cm::PinnableBitMap&  changedAABBMgrHandles,
	const PxU32 numShapes, const PxU32 numActiveBodies,
	bool enableDirectGPUAPI)
{
	PX_PROFILE_ZONE("GpuSimulationController.gpuMemDmaBack", 0);
	PxBounds3* bounds = boundArray.getBounds();
	PxU32 boundCapacity = boundArray.size();

	// AD: DtoH memcopies, need to be skip safe!
	mCudaContext->memcpyDtoHAsync(mUpdatedCacheAndBoundsDesc.data(), mUpdatedCacheAndBoundsDescBuffer.getDevicePtr(), sizeof(PxgSimulationCoreDesc), mStream);
	mCudaContext->memcpyDtoHAsync(activateArray.begin(), mActivateBuffer.getDevicePtr(), sizeof(PxU32)*numActiveBodies, mStream);
	mCudaContext->memcpyDtoHAsync(deactiveArray.begin(), mDeactivateBuffer.getDevicePtr(), sizeof(PxU32)*numActiveBodies, mStream);

	// DMA accelerations back to CPU (kernel was launched in update())
	if (hasAccelerationBuffers())
	{
		const PxU32 nbBodies = mNbTotalBodySim;
		mCudaContext->memcpyDtoHAsync(mBodySimAccelerationsPinned.begin(), mBodySimAccelerationsCudaBuffer.getDevicePtr(), sizeof(PxgRigidBodyAcceleration) * nbBodies, mStream);
	}

	// AD: frozen/unfrozen arrays are only needed for SQ tree updates, skipping for direct-GPU.
	if (!enableDirectGPUAPI)
	{
		mCudaContext->memcpyDtoHAsync(frozenArray.begin(), mFrozenBlockAndResBuffer.getDevicePtr(), sizeof(PxU32)*numShapes, mStream);
		mCudaContext->memcpyDtoHAsync(unfrozenArray.begin(), mUnfrozenBlockAndResBuffer.getDevicePtr(), sizeof(PxU32)*numShapes, mStream);
	}
	
	// AD safety if the copies above fail.
	// Fine to do sync because we never dispatch the copies if we abort.
	if (mCudaContext->isInAbortMode())
	{
		mUpdatedCacheAndBoundsDesc.get().mTotalFrozenShapes = 0;
		mUpdatedCacheAndBoundsDesc.get().mTotalUnfrozenShapes = 0;

		for (PxU32 i = 0; i < activateArray.size(); ++i)
			activateArray[i] = 0;

		for (PxU32 i = 0; i < deactiveArray.size(); ++i)
			deactiveArray[i] = 0;
	}


	if(!mGpuContext->getEnableDirectGPUAPI())
	{	
		CUdeviceptr boundsd = mUseGpuBp ? mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr() : mBoundsBuffer.getDevicePtr();
		mCudaContext->memcpyDtoHAsync(bounds, boundsd, sizeof(PxBounds3)*boundCapacity, mStream);
		mCudaContext->memcpyDtoHAsync(cachedTransforms, mGpuContext->mGpuNpCore->getTransformCache().getDevicePtr(), sizeof(PxsCachedTransform)*cachedCapacity, mStream);
	}
	else
	{
		//reset the changes in case they were made through cpu api and were already copied over
		PxgBoundsArray& directGPUBoundsArray = static_cast<PxgBoundsArray&>(boundArray);
		directGPUBoundsArray.resetChanges();
	}

	// AD: I'm not sure about that one. It will bring back the list of changed AABB manager handles to CPU, which will then be appended
	// with changes done using the public API. Then we copy back to GPU. Should be fine to skip if we're not allowing CPU-side updates?
	CUdeviceptr changeAABBHandlesd = mUseGpuBp ? mGpuContext->getGpuBroadPhase()->getAABBManager()->getChangedAABBMgrHandles() : mChangedAABBMgrHandlesBuffer.getDevicePtr();
	mCudaContext->memcpyDtoHAsync(changedAABBMgrHandles.getWords(), changeAABBHandlesd, sizeof(PxU32)*changedAABBMgrHandles.getWordCount(), mStream);

	
	*mEventMapped = 0;

	CUfunction signalFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::BP_SIGNAL_COMPLETE);

	void* devicePtr = getMappedDevicePtr(mCudaContext, mEventMapped);
	PxCudaKernelParam signalParams[] =
	{
		PX_CUDA_KERNEL_PARAM(devicePtr)
	};

	CUresult resultR = mCudaContext->launchKernel(signalFunction, 1, 1, 1, 1, 1, 1, 0, mStream, signalParams, sizeof(signalParams), 0, PX_FL);
	PX_UNUSED(resultR);
	PX_ASSERT(resultR == CUDA_SUCCESS);

#if SC_GPU_DEBUG
	mCudaContext->streamSynchronize(mStream);
#else
	//This push all the commands in the queue to execute, but it doesn't wait for the result
	mCudaContext->streamFlush(mStream);
#endif
}

void PxgSimulationCore::syncDmaback(PxU32& nbFrozenShapesThisFrame, PxU32& nbUnfrozenShapesThisFrame, bool didSimulate)
{
	PX_PROFILE_ZONE("PxgSimulationCore::syncDmaBack", 0);

	//make sure all the data has been back to cpu
	//mCudaContext->streamSynchronize(mStream);

	if (didSimulate)
	{
		volatile PxU32* pEvent = mEventMapped;
			
		if (!spinWait(*pEvent, 0.1f))
			mCudaContext->streamSynchronize(mStream);
	}

	nbFrozenShapesThisFrame = mUpdatedCacheAndBoundsDesc.get().mTotalFrozenShapes;
	nbUnfrozenShapesThisFrame = mUpdatedCacheAndBoundsDesc.get().mTotalUnfrozenShapes;
}

void PxgSimulationCore::updateBodies(const PxU32 nbUpdatedBodies, const PxU32 nbNewBodies)
{
	{
		if (nbUpdatedBodies > 0 && !mGpuContext->getEnableDirectGPUAPI())
		{
			CUdeviceptr descptr = mUpdatedBodiesDescBuffer.getDevicePtr();
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(descptr)
			};

			CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_BODY_EXTERNAL_VELOCITIES);
			//update bodies and shapes
			CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_BODY_EXTERNAL_VELOCITIES, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_BODY_EXTERNAL_VELOCITIES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			PX_UNUSED(result);

#if SC_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU update bodies velocities kernel fail!\n");
#endif
		}
	}

	{
		if (nbNewBodies > 0)
		{
			CUdeviceptr descptr = mBodiesDescBuffer.getDevicePtr();
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(descptr)
			};

			CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_BODIES);
			// if direct GPU API is enabled, we must not write stale transform data to the GPU which the DIRECT_API kernel version implements
			// this kernel will still write the full data in the first step - we check that inside the kernel. Otherwise this would not work with the immutability of the direct-GPU flag.
			if(mGpuContext->getEnableDirectGPUAPI()) // AD: switch to initialization check as soon as possible, for now let's stay on the safe side.
			{
				kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_BODIES_DIRECT_API);
			}

			//update bodies and shapes
			CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_BODIES_AND_SHAPES, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_BODIES_AND_SHAPES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			PX_UNUSED(result);

#if SC_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "updateBodiesLaunch kernel fail!\n");
#endif
		}
	}
}

void PxgSimulationCore::updateArticulations(const PxU32 nbNewArticulations, PxgArticulationSimUpdate* updates, 
	const PxU32 nbUpdatedArticulations, PxReal* dofData)
{
	if (nbNewArticulations > 0)
	{
		void* mappedDofs = getMappedDevicePtr(mCudaContext, dofData);
		CUdeviceptr descptr = mArticulationDescBuffer.getDevicePtr();
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(descptr),
			PX_CUDA_KERNEL_PARAM(mappedDofs)
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::NEW_ARTICULATIONS);

		PxU32 warpSize = WARP_SIZE;
		PxU32 numWarpsPerBlock = PxgSimulationCoreKernelBlockDim::NEW_ARTICULATION / warpSize;
		PxU32 numBlocks = (nbNewArticulations + numWarpsPerBlock - 1) / numWarpsPerBlock;

		//update links and joints for the articulation
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, 32, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_UNUSED(result);

#if SC_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU update articulation links and joints kernel fail!\n");
#endif
		/*const PxU32 numLinks = 10;
		const PxU32 numDofs = 27;
		PxgArticulation articulation;
		PxgArticulationLink links[numLinks];
		PxTransform body2Worlds[numLinks];
		Dy::ArticulationJointCoreBase jointCores[numLinks];
		Dy::ArticulationJointCoreData jointData[numLinks];

		mCudaContext->memcpyDtoH(&articulation, mArticulationBuffer.getDevicePtr(), sizeof(PxgArticulation));
		mCudaContext->memcpyDtoH(links, (CUdeviceptr)articulation.links, sizeof(PxgArticulationLink) * numLinks);
		mCudaContext->memcpyDtoH(body2Worlds, (CUdeviceptr)articulation.linkBody2Worlds, sizeof(PxTransform) * numLinks);
		mCudaContext->memcpyDtoH(jointCores, (CUdeviceptr)articulation.joints, sizeof(Dy::ArticulationJointCoreBase) * numLinks);
		mCudaContext->memcpyDtoH(jointData, (CUdeviceptr)articulation.jointData, sizeof(Dy::ArticulationJointCoreData) * numLinks);
	
			
		PxReal jointPosition[numDofs];
		PxgArticulationBuffer* buffer = mArticulationDataBuffer[0];

		mCudaContext->memcpyDtoH(&jointPosition, buffer->jointPositions.getDevicePtr(), sizeof(PxReal) * numDofs);*/

		/*PxgArticulation articulation;
		mCudaContext->memcpyDtoH(&articulation, mArticulationBuffer.getDevicePtr(), sizeof(PxgArticulation));

		PxgArticulationSpatialTendon tendon;
		mCudaContext->memcpyDtoH(&tendon, (CUdeviceptr)articulation.tendons, sizeof(PxgArticulationSpatialTendon));

		PxgArticulationAttachment attachments[5];
		mCudaContext->memcpyDtoH(attachments, (CUdeviceptr)tendon.mAttachments, sizeof(PxgArticulationAttachment) * 5);

		int bob = 0;
		PX_UNUSED(bob);*/	
	}

	if (nbUpdatedArticulations > 0)
	{
		CUdeviceptr descptr = mArticulationDescBuffer.getDevicePtr();

		void* mappedUpdates = getMappedDevicePtr(mCudaContext, updates);
		void* mappedDofs = getMappedDevicePtr(mCudaContext, dofData);

		const bool directAPI = mGpuContext->getEnableDirectGPUAPI();
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(descptr),
			PX_CUDA_KERNEL_PARAM(mappedUpdates),
			PX_CUDA_KERNEL_PARAM(nbUpdatedArticulations),
			PX_CUDA_KERNEL_PARAM(mappedDofs),
			PX_CUDA_KERNEL_PARAM(directAPI)
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_ARTICULATIONS);

		PxU32 warpSize = 32;
		PxU32 numWarpsPerBlock = PxgSimulationCoreKernelBlockDim::NEW_ARTICULATION / warpSize;
		PxU32 numBlocks = (nbUpdatedArticulations + numWarpsPerBlock - 1) / numWarpsPerBlock;

		//update links and joints for the articulation
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, 32, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_UNUSED(result);

#if SC_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU update articulation links and joints kernel fail!\n");
#endif
	}
}

template<typename T>
static PX_FORCE_INLINE void resizeAndCopyToDeviceBuffer(const Cm::PinnableArray<T>& hostBuffer, 
	PxgTypedCudaBuffer<T>& deviceBuffer,
	PxCudaContext& cudaContext, CUstream& stream)
{
	const size_t bufferByteSize = hostBuffer.size() * sizeof(T);

	if (hostBuffer.size() > deviceBuffer.getNbElements())
	{
		deviceBuffer.allocate(bufferByteSize, PX_FL);
	}

	cudaContext.memcpyHtoDAsync(deviceBuffer.getDevicePtr(), hostBuffer.begin(), bufferByteSize, stream);
}

void PxgSimulationCore::updateJointsAndSyncData(
	const Cm::PinnableArray<PxgD6JointData>& rigidJointData,
	const Cm::PinnableArray<PxU32>& dirtyRigidJointIndices,
	const Cm::PinnableArray<PxgD6JointData>& artiJointData, 
	const Cm::PinnableArray<PxU32>& dirtyArtiJointIndices,
	const Cm::PinnableArray<PxgConstraintPrePrep>& rigidJointPrePrep, 
	const Cm::PinnableArray<PxgConstraintPrePrep>& artiJointPrePrep,
	const PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
	bool isGpuConstraintIdMapDirty,
	PxU32 nbTotalRigidJoints, PxU32 nbTotalArtiJoints)
{
	const bool directGpuApiEnabled = mGpuContext->getEnableDirectGPUAPI();
	if (directGpuApiEnabled && isGpuConstraintIdMapDirty)
	{
		resizeAndCopyToDeviceBuffer<PxgConstraintIdMapEntry>(gpuConstraintIdMapHost, mGpuConstraintIdMapDevice,
			*mCudaContext, mStream);
	}

	if (nbTotalRigidJoints > mNbTotalRigidJoints)
	{
		PxU64 oldCapacity = mRigidJointBuffer.getSize();
		mRigidJointBuffer.allocateCopyOldDataAsync(nbTotalRigidJoints *sizeof(PxgD6JointData), mCudaContext, mStream, PX_FL);	

		if (oldCapacity < mRigidJointBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mRigidJointBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mRigidJointBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		oldCapacity = mRigidJointPrePrepBuffer.getSize();
		mRigidJointPrePrepBuffer.allocateCopyOldDataAsync(nbTotalRigidJoints * sizeof(PxgConstraintPrePrep), mCudaContext, mStream, PX_FL);
		if (oldCapacity < mRigidJointPrePrepBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mRigidJointPrePrepBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mRigidJointPrePrepBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		mNbTotalRigidJoints = nbTotalRigidJoints;
	}

	if (nbTotalArtiJoints > mNbTotalArtiJoints)
	{
		PxU64 oldCapacity = mArtiJointBuffer.getSize();
		mArtiJointBuffer.allocateCopyOldDataAsync(nbTotalArtiJoints * sizeof(PxgD6JointData), mCudaContext, mStream, PX_FL);

		if (oldCapacity < mArtiJointBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mArtiJointBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mArtiJointBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		oldCapacity = mArtiJointPrePrepBuffer.getSize();
		mArtiJointPrePrepBuffer.allocateCopyOldDataAsync(nbTotalArtiJoints * sizeof(PxgConstraintPrePrep), mCudaContext, mStream, PX_FL);
		if (oldCapacity < mArtiJointPrePrepBuffer.getSize())
		{
			mCudaContext->memsetD32Async(mArtiJointPrePrepBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mArtiJointPrePrepBuffer.getSize() - oldCapacity) / sizeof(PxU32), mStream);
		}

		mNbTotalArtiJoints = nbTotalArtiJoints;
	}

	if (dirtyRigidJointIndices.size() > 0 || dirtyArtiJointIndices.size())
	{
		//PxgIterator<PxgD6JointData> iter(jointData);
		//PxgIterator<PxU32> indicesIter(updatedJointIndices);

		const PxU32 nbUpdatedRigidJoints = dirtyRigidJointIndices.size();
		PxgUpdatedJointsDesc& updatedJointsDesc = mUpdatedJointsDesc.get();
		updatedJointsDesc.mD6RigidJointCPUPool = reinterpret_cast<const PxgD6JointData*>(getMappedDeviceConstPtr(mCudaContext, rigidJointData.begin()));
		updatedJointsDesc.mD6RigidJointGPUPool = reinterpret_cast<PxgD6JointData*>(mRigidJointBuffer.getDevicePtr());
			
		updatedJointsDesc.mD6RigidJointPrePrepCPUPool = reinterpret_cast<const PxgConstraintPrePrep*>(getMappedDeviceConstPtr(mCudaContext, rigidJointPrePrep.begin()));
		updatedJointsDesc.mD6RigidJointPrePrepGPUPool = reinterpret_cast<PxgConstraintPrePrep*>(mRigidJointPrePrepBuffer.getDevicePtr());
			
		updatedJointsDesc.mUpdatedRigidJointIndices = reinterpret_cast<const PxU32*>(getMappedDeviceConstPtr(mCudaContext, dirtyRigidJointIndices.begin()));
		updatedJointsDesc.mNbUpdatedRigidJoints = nbUpdatedRigidJoints;

		const PxU32 nbUpdatedArtiJoints = dirtyArtiJointIndices.size();

		updatedJointsDesc.mD6ArtiJointCPUPool = reinterpret_cast<const PxgD6JointData*>(getMappedDeviceConstPtr(mCudaContext, artiJointData.begin()));
		updatedJointsDesc.mD6ArtiJointGPUPool = reinterpret_cast<PxgD6JointData*>(mArtiJointBuffer.getDevicePtr());

		updatedJointsDesc.mD6ArtiJointPrePrepCPUPool = reinterpret_cast<const PxgConstraintPrePrep*>(getMappedDeviceConstPtr(mCudaContext, artiJointPrePrep.begin()));
		updatedJointsDesc.mD6ArtiJointPrePrepGPUPool = reinterpret_cast<PxgConstraintPrePrep*>(mArtiJointPrePrepBuffer.getDevicePtr());

		updatedJointsDesc.mUpdatedArtiJointIndices = reinterpret_cast<const PxU32*>(getMappedDeviceConstPtr(mCudaContext, dirtyArtiJointIndices.begin()));
		updatedJointsDesc.mNbUpdatedArtiJoints = nbUpdatedArtiJoints;

		//dma up descriptor
		mCudaContext->memcpyHtoDAsync(mUpdatedJointDescBuffer.getDevicePtr(), &updatedJointsDesc, sizeof(PxgUpdatedJointsDesc), mStream);

		const PxU32 maxUpdatedJoints = PxMax(nbUpdatedArtiJoints, nbUpdatedRigidJoints);
		const PxU32 numWarpPerBlocks = 8;
		const PxU32 numBlocks = (maxUpdatedJoints + numWarpPerBlocks - 1) / numWarpPerBlocks;

		CUdeviceptr descptr = mUpdatedJointDescBuffer.getDevicePtr();
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(descptr),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_JOINTS);
		//update bodies and shapes
		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, 2, 1, 32, numWarpPerBlocks, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		PX_UNUSED(result);
		PX_ASSERT(result == CUDA_SUCCESS);

#if SC_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateJointsLaunch fail!\n");

#endif

#if GPU_VERIFY_JOINT_UPDATE
		{
			PxArray<PxgD6JointData> tJointData(mNbTotalJoints);
			//dma back the joint data and verify
			mCudaContext->memcpyDtoH((void*)tJointData.begin(), mJointBuffer.getDevicePtr(), sizeof(PxgD6JointData)*mNbTotalJoints);


			for (PxU32 j = 0; j < nbUpdatedJoints; ++j)
			{
				const PxU32 jointIndex = dirtyJointIndices[j];
				PxgD6JointData& data = jointData[jointIndex];

				PxgD6JointData& tData = tJointData[jointIndex];
				for (PxU32 k = 0; k < 6; ++k)
				{
					PX_ASSERT(data.motion[k] == tData.motion[k]);
					PX_ASSERT(data.drive[k].damping == tData.drive[k].damping && data.drive[k].forceLimit == tData.drive[k].forceLimit &&
						data.drive[k].stiffness == tData.drive[k].stiffness);
				}

				PX_ASSERT(data.driveAngularVelocity.x == tData.driveAngularVelocity.x && data.driveAngularVelocity.y == tData.driveAngularVelocity.y && data.driveAngularVelocity.z == tData.driveAngularVelocity.z);
				PX_ASSERT(data.driveLinearVelocity.x == tData.driveLinearVelocity.x && data.driveLinearVelocity.y == tData.driveLinearVelocity.y && data.driveLinearVelocity.z == tData.driveLinearVelocity.z);
				PX_ASSERT(data.driving == tData.driving);
				PX_ASSERT(data.edgeIndex == tData.edgeIndex);
				PX_UNUSED(data);
				PX_UNUSED(tData);

			}


			PxArray<PxgConstraintPrePrep> tPrePrep(mNbTotalJoints);
			//dma back the joint data and verify
			mCudaContext->memcpyDtoH((void*)tPrePrep.begin(), mJointPrePrepBuffer.getDevicePtr(), sizeof(PxgConstraintPrePrep)*mNbTotalJoints);
			for (PxU32 j = 0; j < nbUpdatedJoints; ++j)
			{
				const PxU32 jointIndex = dirtyJointIndices[j];
				PxgConstraintPrePrep& prePrep = jointPrePrep[jointIndex];
				PxgConstraintPrePrep& tPreP = tPrePrep[jointIndex];
				PX_ASSERT(prePrep.mAngBreakForce == tPreP.mAngBreakForce);
				PX_ASSERT(prePrep.mLinBreakForce == tPreP.mLinBreakForce);
				PX_ASSERT(prePrep.mNodeIndexA == tPreP.mNodeIndexA);
				PX_ASSERT(prePrep.mNodeIndexB == tPreP.mNodeIndexB);
				PX_UNUSED(prePrep);
				PX_UNUSED(tPreP);
			}
		}
#endif
	}

	//ML: this is very important and must be outside of dirtyJointIndices.size() > 0 because if we don't have dirty joint, we
	//still need to wait for all the bodysim and shapesim data finish before we run the solver. Otherwise, this will cause random crash
	//make solver run after all bodysim and shapesim data and updated are already in GPU
	syncData();
}

//this will be called after updateJoint
void PxgSimulationCore::syncData()
{
	CUstream stream = mGpuContext->mGpuSolverCore->getStream();

	synchronizeStreams(mCudaContext, mStream, stream, mDmaEvent);
}

void PxgSimulationCore::update(bool enableDirectGPUAPI)
{
	PX_PROFILE_ZONE("GpuSimulationController.updateTransformCacheAndBoundArray", 0);

	CUdeviceptr descptr = mUpdatedCacheAndBoundsDescBuffer.getDevicePtr();
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(descptr)
	};

	//we need to wait for the solver integration kernel finish before we can run any of these kernels
	CUstream stream = mGpuContext->mGpuSolverCore->getStream();

	synchronizeStreams(mCudaContext, stream, mStream, mEvent);
		
	CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_TRANSFORMCACHE_AND_BOUNDARRAY);

	//update transform cache and bounds
	CUresult result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_TRANSFORMCACHE_AND_BOUNDARRAY, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_TRANSFORMCACHE_AND_BOUNDARRAY, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_UNUSED(result);
	PX_ASSERT(result == CUDA_SUCCESS);
	
#if SC_GPU_DEBUG
	result = mCudaContext->streamSynchronize(mStream);
	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateTransformCacheAndBoundArray kernel fail!\n");
#endif

	// AD: this could theoretically be pushed to the start of the next sim step, but we need to figure out what happens if we don't have this info on CPU!
	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_AABBMGR_HANDLES);
	result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_AABBMGR_HANDLES, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_AABBMGR_HANDLES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);
#if SC_GPU_DEBUG
	result = mCudaContext->streamSynchronize(mStream);
	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateTransformCacheAndBoundArray kernel fail!\n");
#endif

	// AD: frozen/unfrozen arrays are only needed for SQ tree updates.
	if (!enableDirectGPUAPI)
	{
		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_FROZEN_UNFROZEN_HISTOGRAM);
		result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::COMPUTE_FROZEN_UNFROZEN_HISTOGRAM, 1, 1, PxgSimulationCoreKernelBlockDim::COMPUTE_FROZEN_UNFROZEN_HISTOGRAM, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		PX_ASSERT(result == CUDA_SUCCESS);

		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::OUTPUT_FROZEN_UNFROZEN_HISTOGRAM);
		result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::OUTPUT_FROZEN_UNFROZEN_HISTOGRAM, 1, 1, PxgSimulationCoreKernelBlockDim::OUTPUT_FROZEN_UNFROZEN_HISTOGRAM, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		PX_ASSERT(result == CUDA_SUCCESS);

		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CREATE_FROZEN_UNFROZEN_ARRAY);
		result = mCudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::CREATE_FROZEN_UNFROZEN_ARRAY, 1, 1, PxgSimulationCoreKernelBlockDim::CREATE_FROZEN_UNFROZEN_ARRAY, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	}

	//mCudaContext->streamFlush(mStream);

	PX_ASSERT(result == CUDA_SUCCESS);
#if SC_GPU_DEBUG
	result = mCudaContext->streamSynchronize(mStream);
	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU create frozen/unfrozen array kernel fail!\n");
#endif

	// Compute rigid body accelerations using velocity-delta method
	if (hasAccelerationBuffers())
	{
		const PxReal dt = mGpuContext->getDt();
		const PxU32 nbBodies = mNbTotalBodySim;
		const float oneOverDt = 1.0f / dt;
		launchComputeRigidBodyAccelerations(nbBodies, oneOverDt);
	}
}

void PxgSimulationCore::setBounds(Bp::BoundsArray* boundArray)
{
	mBoundArray = boundArray;
}

PxgTypedCudaBuffer<PxBounds3>*	PxgSimulationCore::getBoundArrayBuffer()
{
	if (mUseGpuBp)
	{
		return &mGpuContext->mGpuBp->getBoundsBuffer();
	}
	else
	{
		return &mBoundsBuffer;
	}
}

bool PxgSimulationCore::getRigidDynamicData(void* data, const PxRigidDynamicGPUIndex* gpuIndices,
											PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements,
											CUevent startEvent, CUevent finishEvent) const
{
	PxScopedCudaLock _lock(*mCudaContextManager);
	bool success = true;
	CUresult result = CUDA_SUCCESS;

	if(startEvent)
	{
		mCudaContext->streamWaitEvent(mStream, startEvent);
	}

	const CUdeviceptr bodySimBufferDevicePtr = getBodySimBufferDevicePtr();

	switch(dataType)
	{
	case PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE:
	{
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(bodySimBufferDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_GET_GLOBAL_POSE);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE - 1) / PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE;

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY:
	{
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(bodySimBufferDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_GET_LINVEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINVEL - 1) / PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINVEL;

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINVEL, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY:
	{
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(bodySimBufferDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_GET_ANGVEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGVEL - 1) / PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGVEL;

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGVEL, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION:
	{
		// Gather from pre-computed accelerations buffer (computed during gpuMemDmaBack)
		const CUdeviceptr accelerationsDevicePtr = mBodySimAccelerationsCudaBuffer.getDevicePtr();
		PX_ASSERT(accelerationsDevicePtr);

		PxCudaKernelParam accelKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(accelerationsDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_GET_LINACCEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINACCEL - 1) / PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINACCEL;

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_LINACCEL, 1, 1, 0, mStream, accelKernelParams, sizeof(accelKernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION:
	{
		// Gather from pre-computed accelerations buffer (computed during gpuMemDmaBack)
		const CUdeviceptr accelerationsDevicePtr = mBodySimAccelerationsCudaBuffer.getDevicePtr();
		PX_ASSERT(accelerationsDevicePtr);

		PxCudaKernelParam accelKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(accelerationsDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements),
		};

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_GET_ANGACCEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGACCEL - 1) / PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGACCEL;

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_ANGACCEL, 1, 1, 0, mStream, accelKernelParams, sizeof(accelKernelParams), 0, PX_FL);
		break;
	}
	default:
		PX_ALWAYS_ASSERT();
	}

	if(finishEvent)
	{
		mCudaContext->eventRecord(finishEvent, mStream);
	}
	else
	{
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "getRigidDynamicData: CUDA error, code %u\n", result);

		success = (result == CUDA_SUCCESS);
	}

	return success;
}

#if PX_SUPPORT_OMNI_PVD

PxU64 PxgSimulationCore::getRigidBodyDataTypeElementSize(PxRigidDynamicGPUAPIWriteType::Enum dataType)
{
	PxU64 elementSize;
	switch(dataType)
	{
	case PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE:
	{
		elementSize = sizeof(PxTransform);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY:
	{
		elementSize = sizeof(PxVec3);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY:
	{
		elementSize = sizeof(PxVec3);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eFORCE:
	{
		elementSize = sizeof(PxVec3);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eTORQUE:
	{
		elementSize = sizeof(PxVec3);
		break;
	}
	default:
		elementSize = 0;
		break;
	}
	return elementSize;

}

void PxgSimulationCore::ovdRigidBodyCallback(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements)
{
	PxgSimulationController* controller = mGpuContext->getSimulationController();
	if (controller->getEnableOVDReadback() && controller->getOVDCallbacks())
	{		
		PxU32 dataBufferBytes = PxU32(getRigidBodyDataTypeElementSize(dataType)) * PxU32(nbElements);
		mOvdDataBuffer.resizeUninitialized(dataBufferBytes);

		PxU32 indexBufferBytes = sizeof(PxRigidDynamicGPUIndex) * PxU32(nbElements);
		mOvdIndexBuffer.resizeUninitialized(indexBufferBytes);

		if (mOvdDataBuffer.begin() && mOvdIndexBuffer.begin())
		{				
			////////////////////////////////////////////////////////////////////////////////
			// Copy the forces and gpuIndices from GPU -> CPU
			////////////////////////////////////////////////////////////////////////////////
			PxCUresult resultData = mCudaContext->memcpyDtoH(mOvdDataBuffer.begin(), CUdeviceptr(data), dataBufferBytes);
			PxCUresult resultIndices = mCudaContext->memcpyDtoH(mOvdIndexBuffer.begin(), CUdeviceptr(gpuIndices), indexBufferBytes);
			if (!resultData && !resultIndices)
			{
				////////////////////////////////////////////////////////////////////////////////
				// Call NpDirectGPUAPI layer callback
				////////////////////////////////////////////////////////////////////////////////
				controller->getOVDCallbacks()->processRigidDynamicSet(reinterpret_cast<PxsRigidBody **>(controller->getBodySimManager().mBodies.begin()),
					mOvdDataBuffer.begin(), reinterpret_cast<PxRigidDynamicGPUIndex*>(mOvdIndexBuffer.begin()), dataType, nbElements);
			}
		}
	}
}

#endif

bool PxgSimulationCore::setRigidDynamicData(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
                                                PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements,
                                                CUevent startEvent, CUevent finishEvent)
{
	PxScopedCudaLock _lock(*mCudaContextManager);
	bool success = true;
	CUresult result = CUDA_SUCCESS;

	if(startEvent)
	{
		mCudaContext->streamWaitEvent(mStream, startEvent);
	}

	CUdeviceptr updatedActorDescd = mUpdatedActorDescBuffer.getDevicePtr();

#if PX_SUPPORT_OMNI_PVD
	ovdRigidBodyCallback(data, gpuIndices, dataType, nbElements);
#endif

	switch(dataType)
	{
	case PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE:
	{
		CUfunction kernelFunction =
		    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_SET_GLOBAL_POSE);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_GLOBAL_POSE - 1) /
		                        PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_GLOBAL_POSE;

		// AD: this is the worst.
		mGpuContext->mGpuBp->getAABBManager()->setGPUStateChanged();

		const PxU32 numTotalShapes = getNumTotalShapes();

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(data), PX_CUDA_KERNEL_PARAM(gpuIndices),
			                                 PX_CUDA_KERNEL_PARAM(updatedActorDescd), PX_CUDA_KERNEL_PARAM(nbElements),
			                                 PX_CUDA_KERNEL_PARAM(numTotalShapes) };
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1,
		                                    PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0,
		                                    mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY:
	{
		CUfunction kernelFunction =
		    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_SET_LINVEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_LINVEL - 1) /
		                        PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_LINVEL;

		const CUdeviceptr bodySimPrevVelocitiesBufferDevicePtr = getBodySimPrevVelocitiesBufferDevicePtr();

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(updatedActorDescd),
			PX_CUDA_KERNEL_PARAM(bodySimPrevVelocitiesBufferDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1,
		                                    PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0,
		                                    mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY:
	{
		CUfunction kernelFunction =
		    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_SET_ANGVEL);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_ANGVEL - 1) /
		                        PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_ANGVEL;

		const CUdeviceptr bodySimPrevVelocitiesBufferDevicePtr = getBodySimPrevVelocitiesBufferDevicePtr();

		PxCudaKernelParam kernelParams[] = {
			PX_CUDA_KERNEL_PARAM(data),
			PX_CUDA_KERNEL_PARAM(gpuIndices),
			PX_CUDA_KERNEL_PARAM(updatedActorDescd),
			PX_CUDA_KERNEL_PARAM(bodySimPrevVelocitiesBufferDevicePtr),
			PX_CUDA_KERNEL_PARAM(nbElements)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1,
		                                    PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0,
		                                    mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eFORCE:
	{
		CUfunction kernelFunction =
		    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_SET_FORCE);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_FORCE - 1) /
		                        PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_FORCE;

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(data), PX_CUDA_KERNEL_PARAM(gpuIndices),
			                                 PX_CUDA_KERNEL_PARAM(updatedActorDescd), PX_CUDA_KERNEL_PARAM(nbElements) };

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1,
		                                    PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0,
		                                    mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eTORQUE:
	{
		CUfunction kernelFunction =
		    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_DYNAMIC_SET_TORQUE);
		const PxU32 numBlocks = (nbElements + PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_TORQUE - 1) /
		                        PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_SET_TORQUE;

		PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(data), PX_CUDA_KERNEL_PARAM(gpuIndices),
			                                 PX_CUDA_KERNEL_PARAM(updatedActorDescd), PX_CUDA_KERNEL_PARAM(nbElements) };

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1,
		                                    PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_GET_GLOBAL_POSE, 1, 1, 0,
		                                    mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		break;
	}
	default:
		PX_ALWAYS_ASSERT();
	}

	if(finishEvent)
	{
		mCudaContext->eventRecord(finishEvent, mStream);
	}
	else
	{
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "setRigidDynamicData: CUDA error, code %u\n", result);

		success = (result == CUDA_SUCCESS);
	}

	return success;
}

bool PxgSimulationCore::getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, PxF32 oneOverDt, 
	PxU32 constraintIdMapHostSize, CUevent startEvent, CUevent finishEvent) const
{
	PxScopedCudaLock _lock(*mCudaContextManager);
	bool success = true;
	CUresult result = CUDA_SUCCESS;

	if (startEvent)
	{
		mCudaContext->streamWaitEvent(mStream, startEvent);
	}

	const CUdeviceptr constraintIdMap = mGpuConstraintIdMapDevice.getDevicePtr();
	// note that the size (number of entries) is not taken from the device side map because
	// it might be larger than the host side one. Thus, the size of the host side map gets
	// passed into the method via constraintIdMapHostSize

	PxgSolverCore* solverCore = mGpuContext->getGpuSolverCore();
	const CUdeviceptr constraintWriteBackBufferDevicePtr = solverCore->getConstraintWriteBackBufferDevicePtr();

	static const PxU16 kernelIDList[] = { PxgKernelIds::D6_JOINT_GET_FORCE, PxgKernelIds::D6_JOINT_GET_TORQUE };
	static const PxU32 blockDimList[] = { PxgSimulationCoreKernelBlockDim::D6_JOINT_GET_FORCE, PxgSimulationCoreKernelBlockDim::D6_JOINT_GET_TORQUE };

	PX_COMPILE_TIME_ASSERT(PxD6JointGPUAPIReadType::eJOINT_FORCE == 0);
	PX_COMPILE_TIME_ASSERT(PxD6JointGPUAPIReadType::eJOINT_TORQUE == 1);
	PX_ASSERT((dataType >= PxD6JointGPUAPIReadType::eJOINT_FORCE) && (dataType <= PxD6JointGPUAPIReadType::eJOINT_TORQUE));

	const PxU16 kernelID = kernelIDList[dataType];
	const PxU32 blockDim = blockDimList[dataType];

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(data),
		PX_CUDA_KERNEL_PARAM(gpuIndices),
		PX_CUDA_KERNEL_PARAM(nbElements),
		PX_CUDA_KERNEL_PARAM(constraintWriteBackBufferDevicePtr),
		PX_CUDA_KERNEL_PARAM(oneOverDt),
		PX_CUDA_KERNEL_PARAM(constraintIdMap),
		PX_CUDA_KERNEL_PARAM(constraintIdMapHostSize)
	};

	CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(kernelID);
	const PxU32 numBlocks = (nbElements + blockDim - 1) / blockDim;

	result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, blockDim, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	if (finishEvent)
	{
		mCudaContext->eventRecord(finishEvent, mStream);
	}
	else
	{
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "getD6JointData: CUDA error, code %u\n", result);

		success = (result == CUDA_SUCCESS);
	}

	return success;
}

void PxgSimulationCore::gpuDmaUpdateData()
{
	// this descriptor is only for the direct-GPU updates.
	PxgUpdateActorDataDesc& desc = mUpdatedActorDataDesc.get();

	desc.mBodySimBufferDeviceData = getBodySimBufferDeviceData().getPointer();
	desc.mBounds = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());

	desc.mUpdated = reinterpret_cast<PxU32*>(mUpdatedDirectBuffer.getDevicePtr());
	
	PxgAABBManager* aabbManager = mGpuContext->mGpuBp->getAABBManager();
	desc.mChangedAABBMgrHandles = reinterpret_cast<PxU32*>(aabbManager->getChangedAABBMgrHandles());
	desc.mBitMapWordCounts = aabbManager->getChangedAABBMgrHandlesWordCount();

	PxgShapeManager& shapeManager = mGpuContext->mGpuNpCore->getGpuShapeManager();
	desc.mRigidNodeIndices = reinterpret_cast<PxNodeIndex*>(shapeManager.mGpuRigidIndiceBuffer.getDevicePtr());
	desc.mShapeIndices = reinterpret_cast<PxU32*>(shapeManager.mGpuShapeIndiceBuffer.getDevicePtr());
	desc.mShapes = reinterpret_cast<PxgShape*>(mGpuContext->mGpuNpCore->mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	desc.mTransformCache = reinterpret_cast<PxsCachedTransform*>(mGpuContext->mGpuNpCore->getTransformCache().getDevicePtr());
	desc.mShapeSimsBufferDeviceData = mPxgShapeSimManager.getShapeSimsDeviceTypedPtr();
	mCudaContext->memcpyHtoDAsync(mUpdatedActorDescBuffer.getDevicePtr(), &desc, sizeof(PxgUpdateActorDataDesc), mStream);
}

void PxgSimulationCore::initDirectGPUAPIDescriptor()
{
	PxgAABBManager* aabbManager = mGpuContext->mGpuBp->getAABBManager();
	const PxU32 bitMapWordCounts = aabbManager->getChangedAABBMgrHandlesWordCount();
	const PxU32 roundElement = bitMapWordCounts * 32;

	mUpdatedDirectBuffer.allocate(roundElement * sizeof(PxU32), PX_FL);
	mCudaContext->memsetD32Async(mUpdatedDirectBuffer.getDevicePtr(), 0, roundElement, mStream);

	gpuDmaUpdateData();
}

void PxgSimulationCore::launchComputeRigidBodyAccelerations(const PxU32 nbBodies, const float oneOverDt)
{
	PX_ASSERT(nbBodies == mNbTotalBodySim);
	PX_ASSERT(mBodySimAccelerationsCudaBuffer.getSize() >= nbBodies * sizeof(PxgRigidBodyAcceleration));
	PX_ASSERT(mBodySimPreviousVelocitiesCudaBuffer.getSize() >= nbBodies * sizeof(PxgBodySimVelocities));

	CUdeviceptr accPtr = mBodySimAccelerationsCudaBuffer.getDevicePtr();
	CUdeviceptr bodySimPtr = mBodySimCudaBuffer.getDevicePtr();
	CUdeviceptr prevVelPtr = mBodySimPreviousVelocitiesCudaBuffer.getDevicePtr();

	PxgKernelLauncher::launchKernelThreadsAutoPackParameters(mCudaContext, mGpuKernelWranglerManager,
		PxgKernelIds::RIGID_DYNAMIC_COMPUTE_ACCELERATIONS, nbBodies, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_COMPUTE_ACCELERATIONS, 0, mStream,
		accPtr, bodySimPtr, prevVelPtr, nbBodies, oneOverDt);
}

void PxgSimulationCore::launchCopyRigidBodyVelocitiesToPrevious(const PxU32 nbBodies)
{
	const PxU64 bufferSize = mBodySimPreviousVelocitiesCudaBuffer.getSize();
	if (bufferSize == 0)
		return;

	const PxU32 capacity = PxU32(bufferSize / sizeof(PxgBodySimVelocities));
	const PxU32 nbToCopy = PxMin(nbBodies, capacity);

	if (nbToCopy == 0)
		return;

	CUdeviceptr prevVelPtr = mBodySimPreviousVelocitiesCudaBuffer.getDevicePtr();
	CUdeviceptr bodySimPtr = mBodySimCudaBuffer.getDevicePtr();

	PxgKernelLauncher::launchKernelThreadsAutoPackParameters(mCudaContext, mGpuKernelWranglerManager,
		PxgKernelIds::RIGID_DYNAMIC_COPY_PREVIOUS_VELOCITIES, nbToCopy, PxgSimulationCoreKernelBlockDim::RIGID_DYNAMIC_COPY_PREVIOUS_VELOCITIES, 0, mStream,
		prevVelPtr, bodySimPtr, nbToCopy);
}

PxU32 PxgSimulationCore::getMaxArticulationLinks() const { return mGpuContext->getSimulationController()->mMaxLinks; }
PxU32 PxgSimulationCore::getMaxArticulationDofs() const { return mGpuContext->getSimulationController()->mMaxDofs; }
PxU32 PxgSimulationCore::getMaxArticulationMimicJoints() const { return mGpuContext->getSimulationController()->mMaxMimicJoints; }
PxU32 PxgSimulationCore::getMaxArticuationSpatialTendons() const { return mGpuContext->getSimulationController()->mMaxSpatialTendons; }
PxU32 PxgSimulationCore::getMaxArticuationAttachments() const { return mGpuContext->getSimulationController()->mMaxAttachments; }
PxU32 PxgSimulationCore::getMaxArticuationFixedTendons() const { return mGpuContext->getSimulationController()->mMaxFixedTendons; }
PxU32 PxgSimulationCore::getMaxArticuationTendonJoints() const { return mGpuContext->getSimulationController()->mMaxTendonJoints; }

void PxgSimulationCore::setSoftBodyWakeCounter(const PxU32 remapId, const PxReal wakeCounter, const PxU32 numSoftBodies)
{
	if (mSBWakeCounts.size() <= numSoftBodies)
	{
		mSBWakeCounts.resize(numSoftBodies);
	}

	mSBWakeCounts[remapId] = wakeCounter;
}

void PxgSimulationCore::setFEMClothWakeCounter(const PxU32 remapId, const PxReal wakeCounter, const PxU32 numClothes)
{
	if (mFEMClothWakeCounts.size() <= numClothes)
	{
		mFEMClothWakeCounts.resize(numClothes);
	}

	mFEMClothWakeCounts[remapId] = wakeCounter;
}

