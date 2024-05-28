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

// ****************************************************************************
// This snippet illustrates GPU-accelerated rigid body simulation with
// .the direct GPU API feature on and used for rendering and resetting.
//
// It creates a number of box stacks on a plane, and a rolling ball that
// hits the stacks. User can reset the simulation by pressing 'R'.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxCudaContextManager* gCudaContextManager = NULL;

static PxReal stackZ = 10.0f;

static PxArray<PxU32> gRBIndices;
static PxArray<PxGeometryHolder> gRBGeometries;
static PxArray<PxTransform> gRBPoses;

static CUdeviceptr gRBIndicesD;
static CUdeviceptr gRBPosesD;
static CUdeviceptr gRBInitPosesD;
static CUdeviceptr gRBInitLinVelsD;
static CUdeviceptr gRBInitAngVelsD;

PxU32 getRBCount()
{
	return gRBIndices.size();
}

const PxGeometryHolder* getRBGeometries()
{
	return gRBGeometries.empty() ? NULL : &gRBGeometries[0];
}

const PxTransform* getRBPoses()
{
	return gRBPoses.empty() ? NULL : &gRBPoses[0];
}

static PxRigidDynamic* createDynamic(const PxTransform& pose, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, pose, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);

	gRBIndices.pushBack(dynamic->getGPUIndex());
	gRBGeometries.pushBack(geometry);
	gRBPoses.pushBack(pose);

	return dynamic;
}

static void createStack(const PxTransform& pose, PxU32 size, PxReal halfExtent)
{
	PxBoxGeometry boxGeometry(halfExtent, halfExtent, halfExtent);
	PxShape* shape = gPhysics->createShape(boxGeometry, *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localPose(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxTransform globalPose = pose.transform(localPose);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(globalPose);
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);

			gRBIndices.pushBack(body->getGPUIndex());
			gRBGeometries.pushBack(boxGeometry);
			gRBPoses.pushBack(globalPose);
		}
	}
	shape->release();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());

	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc);
	if (!gCudaContextManager || !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The direct GPU API feature is only supported on the GPU.\n");
		return;
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	sceneDesc.cudaContextManager = gCudaContextManager;

	// enable GPU simulstion and direct GPU access
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;

	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	for (PxU32 i = 0; i < 40; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 20, 1.0f);

	PxRigidDynamic* ball = createDynamic(PxTransform(PxVec3(0, 20, 100)), PxSphereGeometry(5), PxVec3(0, -25, -100));
	PxRigidBodyExt::updateMassAndInertia(*ball, 1000.f);

	PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();

	// prepare RBs indices
	cudaContext->memAlloc(&gRBIndicesD, getRBCount() * sizeof(PxU32));
	cudaContext->memcpyHtoD(gRBIndicesD, &gRBIndices[0], getRBCount() * sizeof(PxU32));
	// a buffer to read poses for rendering
	cudaContext->memAlloc(&gRBPosesD, getRBCount() * sizeof(PxTransform));

	// direct GPU API data is only valid *after*
	// the 1st simulation step. so we step.
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);

	// store initial poses and velocities for reset
	cudaContext->memAlloc(&gRBInitPosesD, getRBCount() * sizeof(PxTransform));
	gScene->getDirectGPUAPI().getRigidDynamicData(reinterpret_cast<void*>(gRBInitPosesD),
												  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
												  PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
												  getRBCount());
	cudaContext->memAlloc(&gRBInitLinVelsD, getRBCount() * sizeof(PxVec3));
	gScene->getDirectGPUAPI().getRigidDynamicData(reinterpret_cast<void*>(gRBInitLinVelsD),
												  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
												  PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY,
												  getRBCount());
	cudaContext->memAlloc(&gRBInitAngVelsD, getRBCount() * sizeof(PxVec3));
	gScene->getDirectGPUAPI().getRigidDynamicData(reinterpret_cast<void*>(gRBInitAngVelsD),
												  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
												  PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY,
												  getRBCount());
}

void stepPhysics(bool /*interactive*/)
{
	if (gCudaContextManager)
	{
		gScene->simulate(1.0f / 60.0f);
		gScene->fetchResults(true);

		// read current poses for rendering
		gScene->getDirectGPUAPI().getRigidDynamicData(reinterpret_cast<void*>(gRBPosesD),
			reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
			PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
			getRBCount());
		PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();
		cudaContext->memcpyDtoH(&gRBPoses[0], gRBPosesD, getRBCount() * sizeof(PxTransform));
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	if (gCudaContextManager)
	{
		PxCudaContext* cudaContext = gCudaContextManager->getCudaContext();

		cudaContext->memFree(gRBInitAngVelsD);
		cudaContext->memFree(gRBInitLinVelsD);
		cudaContext->memFree(gRBInitPosesD);
		cudaContext->memFree(gRBPosesD);
		cudaContext->memFree(gRBIndicesD);

		gRBIndices.reset();
		gRBGeometries.reset();
		gRBPoses.reset();
	}
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gCudaContextManager);
	PX_RELEASE(gFoundation);

	printf("SnippetRBDirectGPUAPI done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	switch (toupper(key))
	{
		case 'R':
		{
			// reset poses and velocities
			gScene->getDirectGPUAPI().setRigidDynamicData(reinterpret_cast<void*>(gRBInitPosesD),
														  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
														  PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE,
														  getRBCount());
			gScene->getDirectGPUAPI().setRigidDynamicData(reinterpret_cast<void*>(gRBInitLinVelsD),
														  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
														  PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
														  getRBCount());
			gScene->getDirectGPUAPI().setRigidDynamicData(reinterpret_cast<void*>(gRBInitAngVelsD),
														  reinterpret_cast<const PxRigidDynamicGPUIndex*>(gRBIndicesD),
														  PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY,
														  getRBCount());
		}
		break;
	}
}

int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
