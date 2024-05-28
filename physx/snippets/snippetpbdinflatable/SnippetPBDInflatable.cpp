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
// NOTE: Particle cloth has been DEPRECATED. Please use PxFEMCloth instead.
// This snippet illustrates inflatable simulation using position-based dynamics
// particle simulation. It creates an inflatable body that drops to the ground.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxParticleExt.h"
#include "extensions/PxParticleClothCooker.h"
#include "extensions/PxCudaHelpersExt.h"

using namespace physx;
using namespace ExtGpu;

static PxDefaultAllocator			gAllocator;
static PxDefaultErrorCallback		gErrorCallback;
static PxFoundation*				gFoundation			= NULL;
static PxPhysics*					gPhysics			= NULL;
static PxDefaultCpuDispatcher*		gDispatcher			= NULL;
static PxCudaContextManager*		gCudaContextManager	= NULL;
static PxScene*						gScene				= NULL;
static PxMaterial*					gMaterial			= NULL;
static PxPvd*						gPvd				= NULL;
static PxPBDParticleSystem*			gParticleSystem		= NULL;
static PxParticleClothBuffer*		gUserClothBuffer	= NULL;
static bool							gIsRunning			= true;


static void initObstacles()
{
	PxShape* shape = gPhysics->createShape(PxCapsuleGeometry(0.5f, 4.f), *gMaterial);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, 5.0f, 2.f)));
	body->attachShape(*shape);
	body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*body);
	shape->release();

	shape = gPhysics->createShape(PxCapsuleGeometry(0.5f, 4.f), *gMaterial);
	body = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, 5.0f, -2.f)));
	body->attachShape(*shape);
	body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*body);
	shape->release();
}

// -----------------------------------------------------------------------------------------------------------------
static void initScene()
{
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cudaContextManager = gCudaContextManager;
	sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.solverType = PxSolverType::eTGS;
	gScene = gPhysics->createScene(sceneDesc);
}

// -----------------------------------------------------------------------------------------------------------------

PxVec3 cubeVertices[] = { PxVec3(0.5f, -0.5f, -0.5f), PxVec3(0.5f, -0.5f, 0.5f),  PxVec3(-0.5f, -0.5f, 0.5f),  PxVec3(-0.5f, -0.5f, -0.5f),
	PxVec3(0.5f, 0.5f, -0.5f), PxVec3(0.5f, 0.5f, 0.5f), PxVec3(-0.5f, 0.5f, 0.5f), PxVec3(-0.5f, 0.5f, -0.5f) };

PxU32 cubeIndices[] = { 1, 2, 3,  7, 6, 5,  4, 5, 1,  5, 6, 2,  2, 6, 7,  0, 3, 7,  0, 1, 3,  4, 7, 5,  0, 4, 1,  1, 5, 2,  3, 2, 7,  4, 0, 7 };

static void projectPointsOntoSphere(PxArray<PxVec3>& triVerts, const PxVec3& center, PxReal radius)
{
	for (PxU32 i = 0; i < triVerts.size(); ++i)
	{
		PxVec3 dir = triVerts[i] - center;
		dir.normalize();
		triVerts[i] = center + radius * dir;
	}
}

static void createSphere(PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices, const PxVec3& center, PxReal radius, const PxReal maxEdgeLength)
{
	for (PxU32 i = 0; i < 8; ++i)
		triVerts.pushBack(cubeVertices[i] * radius + center);
	for (PxU32 i = 0; i < 36; ++i)
		triIndices.pushBack(cubeIndices[i]);
	projectPointsOntoSphere(triVerts, center, radius);
	while (PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength, 1))
		projectPointsOntoSphere(triVerts, center, radius);
}

static void initInflatable(PxArray<PxVec3>& verts, PxArray<PxU32>& indices, const PxReal restOffset = 0.1f, const PxReal totalInflatableMass = 10.f)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return;

	PxArray<PxVec4> vertices;
	vertices.resize(verts.size());
	PxReal invMass = 1.0f / (totalInflatableMass / verts.size());
	for (PxU32 i = 0; i < verts.size(); ++i)
		vertices[i] = PxVec4(verts[i], invMass);

	const PxU32 numParticles = vertices.size();

	const PxReal stretchStiffness = 100000.f;
	const PxReal shearStiffness = 1000.f;
	const PxReal bendStiffness = 1000.f;
	
	const PxReal pressure = 1.0f; //Pressure is used to compute the target volume of the inflatable by scaling its rest volume

	// Cook cloth
	PxParticleClothCooker* cooker = PxCreateParticleClothCooker(vertices.size(), vertices.begin(), indices.size(), indices.begin(),
		PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT | PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT | PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT);
	cooker->cookConstraints();
	cooker->calculateMeshVolume();

	// Apply cooked constraints to particle springs
	PxU32 constraintCount = cooker->getConstraintCount();
	PxParticleClothConstraint* constraintBuffer = cooker->getConstraints();
	PxArray<PxParticleSpring> springs;
	springs.reserve(constraintCount);
	for (PxU32 i = 0; i < constraintCount; i++)
	{
		const PxParticleClothConstraint& c = constraintBuffer[i];
		PxReal stiffness = 0.0f;
		switch (c.constraintType)
		{
		case PxParticleClothConstraint::eTYPE_INVALID_CONSTRAINT:
			continue;
		case PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT:
		case PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT:
			stiffness = stretchStiffness;
			break;
		case PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT:
			stiffness = shearStiffness;
			break;
		case PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT:
			stiffness = bendStiffness;
			break;
		default:
			PX_ASSERT("Invalid cloth constraint generated by PxParticleClothCooker");
		}

		PxParticleSpring spring;
		spring.ind0 = c.particleIndexA;
		spring.ind1 = c.particleIndexB;
		spring.stiffness = stiffness;
		spring.damping = 0.001f;
		spring.length = c.length;
		springs.pushBack(spring);
	}
	const PxU32 numSprings = springs.size();

	// Read triangles from cooker
	const PxU32 numTriangles = cooker->getTriangleIndicesCount() / 3;
	const PxU32* triangles = cooker->getTriangleIndices();

	// Material setup
	PxPBDMaterial* defaultMat = gPhysics->createPBDMaterial(0.8f, 0.05f, 1e+6f, 0.001f, 0.5f, 0.005f, 0.05f, 0.f, 0.f);

	PxPBDParticleSystem *particleSystem = gPhysics->createPBDParticleSystem(*cudaContextManager);
	gParticleSystem = particleSystem;

	// General particle system setting
	
	particleSystem->setRestOffset(restOffset);
	particleSystem->setContactOffset(restOffset + 0.02f);
	particleSystem->setParticleContactOffset(restOffset + 0.02f);
	particleSystem->setSolidRestOffset(restOffset);
	particleSystem->setFluidRestOffset(0.0f);

	gScene->addActor(*particleSystem);

	// Create particles and add them to the particle system
	const PxU32 particlePhase = particleSystem->createPhase(defaultMat, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

	PxU32* phases = PX_EXT_PINNED_MEMORY_ALLOC(PxU32, *cudaContextManager, numParticles);
	PxVec4* positionInvMass = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, numParticles);
	PxVec4* velocity = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, numParticles);

	
	for (PxU32 v = 0; v < numParticles; v++)
	{
		positionInvMass[v] = vertices[v];
		velocity[v] = PxVec4(0.0f, 0.0f, 0.0f, 0.0f);
		phases[v] = particlePhase;
	}

	PxParticleVolumeBufferHelper* volumeBuffers = PxCreateParticleVolumeBufferHelper(1, numTriangles, cudaContextManager); //Volumes are optional. They are used to accelerate scene queries, e. g. to support picking.
	PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, cudaContextManager);

	clothBuffers->addCloth(0.0f, cooker->getMeshVolume(), pressure, triangles, numTriangles, springs.begin(), numSprings, positionInvMass, numParticles);
	volumeBuffers->addVolume(0, numParticles, triangles, numTriangles);
	cooker->release();

	ExtGpu::PxParticleBufferDesc bufferDesc;
	bufferDesc.maxParticles = numParticles;
	bufferDesc.numActiveParticles = numParticles;
	bufferDesc.positions = positionInvMass;
	bufferDesc.velocities = velocity;
	bufferDesc.phases = phases;
	bufferDesc.maxVolumes = volumeBuffers->getMaxVolumes();
	bufferDesc.numVolumes = volumeBuffers->getNumVolumes();
	bufferDesc.volumes = volumeBuffers->getParticleVolumes();

	PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(cudaContextManager);

	PxPartitionedParticleCloth output;
	const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
	clothPreProcessor->partitionSprings(clothDesc, output);
	clothPreProcessor->release();

	gUserClothBuffer = physx::ExtGpu::PxCreateAndPopulateParticleClothBuffer(bufferDesc, clothDesc, output, cudaContextManager);
	gParticleSystem->addParticleBuffer(gUserClothBuffer);

	clothBuffers->release();
	volumeBuffers->release();

	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, positionInvMass);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, velocity);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, phases);
}

PxPBDParticleSystem* getParticleSystem()
{
	return gParticleSystem;
}

PxParticleClothBuffer* getUserClothBuffer()
{
	return gUserClothBuffer;
}

// -----------------------------------------------------------------------------------------------------------------
void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The particle cloth feature is currently only supported on GPU.\n");
	}

	initScene();

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Setup Cloth
	const PxReal totalInflatableMass = 100.0f;
	
	PxReal particleSpacing = 0.05f;

	PxArray<PxVec3> vertices;
	PxArray<PxU32> indices;
	createSphere(vertices, indices, PxVec3(0, 10, 0), 3, 0.25f);
	initInflatable(vertices, indices, particleSpacing, totalInflatableMass);

	initObstacles();

	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.0f), *gMaterial));
	

	// Setup rigid bodies
	const PxReal boxSize = 0.75f;
	const PxReal boxMass = 0.25f;
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.5f * boxSize, 0.5f * boxSize, 0.5f * boxSize), *gMaterial);
	for (int i = 0; i < 5; ++i)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(PxVec3(i - 2.0f, 10, 0.f)));
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, boxMass);
		gScene->addActor(*body);
	}
	shape->release();
}

// ---------------------------------------------------
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		const PxReal dt = 1.0f / 60.0f;
		gScene->simulate(dt);
		gScene->fetchResults(true);
		gScene->fetchResultsParticleSystem();		
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gCudaContextManager);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetPBDInflatable done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	switch(toupper(key))
	{
	case 'P':	gIsRunning = !gIsRunning;	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
