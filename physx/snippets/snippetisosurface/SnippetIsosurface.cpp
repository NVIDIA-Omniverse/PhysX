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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a position based fluid simulation that gets rendered using an
// isosurface triangle mesh.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

#include "gpuextensions/PxIsosurfaceExtraction.h"
#include "gpuextensions/PxAnisotropy.h"
#include "gpuextensions/PxSmoothing.h"
#include "gpuextensions/PxParticleExt.h"
#include "gpuextensions/PxGpuMemory.h"
#include "gpuextensions/PxUtility.h"

#include "gpuextensions/PxGpuExtensionsAPI.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation 		= NULL;
static PxPhysics*				gPhysics 			= NULL;
static PxDefaultCpuDispatcher*	gDispatcher 		= NULL;
static PxScene*					gScene 				= NULL;
static PxMaterial*				gMaterial 			= NULL;
static PxPvd*					gPvd 				= NULL;
static PxPBDParticleSystem*		gParticleSystem 	= NULL;
static PxParticleBuffer*		gParticleBuffer 	= NULL;
static bool						gIsRunning 			= true;


PxRigidDynamic* movingWall;

using namespace ExtGpu;

PxArray<PxVec4> gIsosurfaceVertices;
PxArray<PxU32> gIsosurfaceIndices;
PxArray<PxVec4> gIsosurfaceNormals;
ExtGpu::PxIsosurfaceExtractor* gIsosurfaceExtractor;
extern void* gVerticesGpu;
extern void* gNormalsGpu;
extern void* gInterleavedVerticesAndNormalsGpu;

class IsosurfaceCallback : public PxParticleSystemCallback
{
public:
	ExtGpu::PxIsosurfaceExtractor* mIsosurfaceExtractor;
	ExtGpu::PxAnisotropyGenerator* mAnisotropyGenerator;
	ExtGpu::PxSmoothedPositionGenerator* mSmoothedPositionGenerator;
	PxVec4* mSmoothedPositionsDeviceBuffer;

	PxVec4* mAnisotropyDeviceBuffer1;
	PxVec4* mAnisotropyDeviceBuffer2;
	PxVec4* mAnisotropyDeviceBuffer3;

	PxU32 mMaxVertices;
	PxCudaContextManager* mCudaContextManager;

	IsosurfaceCallback() : mIsosurfaceExtractor(NULL), mAnisotropyGenerator(NULL) { }

	void initialize(PxCudaContextManager* cudaContextManager,
		const PxSparseGridParams& sparseGridParams, ExtGpu::PxIsosurfaceParams& p,
		PxU32 maxNumVertices, PxU32 maxNumTriangles, PxU32 maxNumParticles)
	{
		mCudaContextManager = cudaContextManager;
		mMaxVertices = maxNumVertices;
		/*ExtGpu::PxIsosurfaceParams p;
		p.isosurfaceValue =threshold;
		p.clearFilteringPasses();*/	
		
		mSmoothedPositionGenerator = ExtGpu::PxCreateSmoothedPositionGenerator(cudaContextManager, maxNumParticles, 0.5f);
		ExtGpu::PxGpuMemory::allocDeviceBuffer(cudaContextManager, mSmoothedPositionsDeviceBuffer, maxNumParticles);
		mSmoothedPositionGenerator->setResultBufferDevice(mSmoothedPositionsDeviceBuffer);

		//Too small minAnisotropy values will shrink particles to ellipsoids that are smaller than a isosurface grid cell which can lead to unpleasant aliasing/flickering 
		PxReal minAnisotropy = 1.0f;// 0.5f; // 0.1f;
		PxReal anisotropyScale = 5.0f;
		mAnisotropyGenerator = ExtGpu::PxCreateAnisotropyGenerator(cudaContextManager, maxNumParticles, anisotropyScale, minAnisotropy, 2.0f);
		ExtGpu::PxGpuMemory::allocDeviceBuffer(cudaContextManager, mAnisotropyDeviceBuffer1, maxNumParticles);
		ExtGpu::PxGpuMemory::allocDeviceBuffer(cudaContextManager, mAnisotropyDeviceBuffer2, maxNumParticles);
		ExtGpu::PxGpuMemory::allocDeviceBuffer(cudaContextManager, mAnisotropyDeviceBuffer3, maxNumParticles);
		mAnisotropyGenerator->setResultBufferDevice(mAnisotropyDeviceBuffer1, mAnisotropyDeviceBuffer2, mAnisotropyDeviceBuffer3);

		gIsosurfaceVertices.resize(maxNumVertices);
		gIsosurfaceNormals.resize(maxNumVertices);
		gIsosurfaceIndices.resize(3 * maxNumTriangles);

		mIsosurfaceExtractor = PxCreateSparseGridIsosurfaceExtractor(cudaContextManager, sparseGridParams, p, maxNumParticles, maxNumVertices, maxNumTriangles);

		gIsosurfaceExtractor = mIsosurfaceExtractor;
	}

	virtual void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
	{
		PxGpuParticleSystem& p = *gpuParticleSystem.mHostPtr;
		
		if (mAnisotropyGenerator) 
		{
			mAnisotropyGenerator->generateAnisotropy(gpuParticleSystem.mDevicePtr, p.mCommonData.mMaxParticles, stream);
		}
		
		mSmoothedPositionGenerator->generateSmoothedPositions(gpuParticleSystem.mDevicePtr, p.mCommonData.mMaxParticles, stream);

		mIsosurfaceExtractor->extractIsosurface(mSmoothedPositionsDeviceBuffer/*reinterpret_cast<PxVec4*>(p.mUnsortedPositions_InvMass)*/, p.mCommonData.mNumParticles, stream, p.mUnsortedPhaseArray, PxParticlePhaseFlag::eParticlePhaseFluid,
			NULL, mAnisotropyDeviceBuffer1, mAnisotropyDeviceBuffer2, mAnisotropyDeviceBuffer3, p.mCommonData.mParticleContactDistance);

		if (gInterleavedVerticesAndNormalsGpu) 
		{
			//Bring the data into a form that is better suited for rendering
			ExtGpu::PxInterleaveGpuBuffers(static_cast<PxVec4*>(gVerticesGpu), static_cast<PxVec4*>(gNormalsGpu), mMaxVertices, static_cast<PxVec3*>(gInterleavedVerticesAndNormalsGpu), stream);
		}
	}

	virtual void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

	virtual void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

	virtual ~IsosurfaceCallback() { }

	void release()
	{
		mIsosurfaceExtractor->release();
		PX_DELETE(mIsosurfaceExtractor);

		if (mAnisotropyGenerator)
		{
			mAnisotropyGenerator->release();
			PX_DELETE(mAnisotropyGenerator);
			ExtGpu::PxGpuMemory::freeDeviceBuffer(mCudaContextManager, mAnisotropyDeviceBuffer1);
			ExtGpu::PxGpuMemory::freeDeviceBuffer(mCudaContextManager, mAnisotropyDeviceBuffer2);
			ExtGpu::PxGpuMemory::freeDeviceBuffer(mCudaContextManager, mAnisotropyDeviceBuffer3);
		}
		if (mSmoothedPositionGenerator)
		{
			mSmoothedPositionGenerator->release();
			PX_DELETE(mSmoothedPositionGenerator);
			ExtGpu::PxGpuMemory::freeDeviceBuffer(mCudaContextManager, mSmoothedPositionsDeviceBuffer);
		}
	}
};

static IsosurfaceCallback gIsosuraceCallback;

// -----------------------------------------------------------------------------------------------------------------
static void initScene()
{
	PxCudaContextManager* cudaContextManager = NULL;
	if (PxGetSuggestedCudaDeviceOrdinal(gFoundation->getErrorCallback()) >= 0)
	{
		// initialize CUDA
		PxCudaContextManagerDesc cudaContextManagerDesc;
		cudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
		if (cudaContextManager && !cudaContextManager->contextIsValid())
		{
			cudaContextManager->release();
			cudaContextManager = NULL;
		}
		else
			PxInitGpuExtensions(*cudaContextManager);
	}
	if (cudaContextManager == NULL)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Failed to initialize CUDA!\n");
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cudaContextManager = cudaContextManager;
	sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.solverType = PxSolverType::eTGS;
	gScene = gPhysics->createScene(sceneDesc);
}

// -----------------------------------------------------------------------------------------------------------------
static PxReal initParticles(const PxU32 numX, const PxU32 numY, const PxU32 numZ, const PxVec3& position = PxVec3(0, 0, 0), const PxReal particleSpacing = 0.2f, const PxReal fluidDensity = 1000.f)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return -1;

	const PxU32 maxParticles = numX * numY * numZ;

	const PxReal fluidRestOffset = 0.5f * particleSpacing;
	
	// Material setup
	PxPBDMaterial* defaultMat = gPhysics->createPBDMaterial(0.05f, 0.05f, 0.f, 0.001f, 0.5f, 0.005f, 0.01f, 0.f, 0.f);	

	PxPBDParticleSystem *particleSystem = gPhysics->createPBDParticleSystem(*cudaContextManager, 96);
	gParticleSystem = particleSystem;

	bool highCohesion = false;
	if (highCohesion)
	{
		defaultMat->setViscosity(50.0f);
		defaultMat->setSurfaceTension(0.f);
		defaultMat->setCohesion(100.0f);
		particleSystem->setSolverIterationCounts(20, 0);
	}
	else 
	{
		defaultMat->setViscosity(0.001f);
		defaultMat->setSurfaceTension(0.00704f);
		defaultMat->setCohesion(0.704f);
		defaultMat->setVorticityConfinement(10.f);
	}

	// General particle system setting
	
	const PxReal restOffset = fluidRestOffset / 0.6f;
	const PxReal solidRestOffset = restOffset;	
	const PxReal particleMass = fluidDensity * 1.333f * 3.14159f * particleSpacing * particleSpacing * particleSpacing;
	particleSystem->setRestOffset(restOffset);
	particleSystem->setContactOffset(restOffset + 0.01f);
	particleSystem->setParticleContactOffset(fluidRestOffset / 0.6f);
	particleSystem->setSolidRestOffset(solidRestOffset);
	particleSystem->setFluidRestOffset(fluidRestOffset);
	particleSystem->enableCCD(false);
	particleSystem->setMaxVelocity(100.f);

	gScene->addParticleSystem(*particleSystem);
	

	// Create particles and add them to the particle system
	const PxU32 particlePhase = particleSystem->createPhase(defaultMat, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

	PxPinnedHostMemoryPointer<PxU32> phaseBuf(cudaContextManager, maxParticles);
	PxPinnedHostMemoryPointer<PxVec4> positionInvMassBuf(cudaContextManager, maxParticles);
	PxPinnedHostMemoryPointer<PxVec4> velocityBuf(cudaContextManager, maxParticles);

	PxU32* phase = phaseBuf.getHostPointer();
	PxVec4* positionInvMass = positionInvMassBuf.getHostPointer();
	PxVec4* velocity = velocityBuf.getHostPointer();

	PxReal x = position.x;
	PxReal y = position.y;
	PxReal z = position.z;
	PxReal maxY = y;
	PxReal maxZ = z;

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numY; ++j)
		{
			for (PxU32 k = 0; k < numZ; ++k)
			{
				const PxU32 index = i * (numY * numZ) + j * numZ + k;

				PxVec4 pos(x, y, z, 1.0f / particleMass);
				phase[index] = particlePhase;
				positionInvMass[index] = pos;
				velocity[index] = PxVec4(0.0f);

				z += particleSpacing;
			}
			maxZ = z - particleSpacing;
			z = position.z;
			y += particleSpacing;
		}
		maxY = y - particleSpacing;
		y = position.y;
		x += particleSpacing;
	}

	ExtGpu::PxParticleBufferDesc bufferDesc;
	bufferDesc.maxParticles = maxParticles;
	bufferDesc.numActiveParticles = maxParticles;
	

	bufferDesc.positions = positionInvMass;
	bufferDesc.velocities = velocity;
	bufferDesc.phases = phase;

	gParticleBuffer = physx::ExtGpu::PxCreateAndPopulateParticleBuffer(bufferDesc, cudaContextManager);
	gParticleSystem->addParticleBuffer(gParticleBuffer);

	return particleSpacing;
}

PxPBDParticleSystem* getParticleSystem()
{
	return gParticleSystem;
}

PxParticleBuffer* getParticleBuffer()
{
	return gParticleBuffer;
}

void addKinematicBox(PxVec3 boxSize, PxVec3 boxCenter)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(boxSize.x, boxSize.y, boxSize.z), *gMaterial);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(boxCenter));
	body->attachShape(*shape);
	body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*body);
	shape->release();
}

// -----------------------------------------------------------------------------------------------------------------
void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	initScene();

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Setup PBF
	bool useMovingWall = true;

	const PxReal fluidDensity = 1000.0f;

	PxU32 numX = 50;
	PxU32 numY = 200;
	PxU32 numZ = 100;
	PxReal particleSpacing = initParticles(numX, numY, numZ, PxVec3(1.5f, /*3.f*/8, -4.f), 0.1f, fluidDensity);

	addKinematicBox(PxVec3(7.5f,0.25f,7.5f), PxVec3(3,7.5f,0));
	addKinematicBox(PxVec3(0.25f, 7.5f, 7.5f), PxVec3(-2.f, 7.5f+ 7.5f+0.5f, 0));

	// Setup container
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.0f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(-1.f, 0.f, 0.f, 7.5f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, 1.f, 7.5f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, -1.f, 7.5f), *gMaterial));

	if (!useMovingWall)
	{
		gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(1.f, 0.f, 0.f, 7.5f), *gMaterial));
		movingWall = NULL;
	}
	else
	{
		PxTransform trans = PxTransformFromPlaneEquation(PxPlane(1.f, 0.f, 0.f, 20.f));
		movingWall = gPhysics->createRigidDynamic(trans);
		movingWall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		PxRigidActorExt::createExclusiveShape(*movingWall, PxPlaneGeometry(), *gMaterial);
		gScene->addActor(*movingWall);
	}

	const PxReal fluidRestOffset = 0.5f * particleSpacing;

	PxSparseGridParams sgIsosurfaceParams;
	sgIsosurfaceParams.subgridSizeX = 16;
	sgIsosurfaceParams.subgridSizeY = 16;
	sgIsosurfaceParams.subgridSizeZ = 16;
	sgIsosurfaceParams.haloSize = 0;
	sgIsosurfaceParams.maxNumSubgrids = 4096;
	sgIsosurfaceParams.gridSpacing = 1.5f*fluidRestOffset;

	ExtGpu::PxIsosurfaceParams p;
	p.particleCenterToIsosurfaceDistance = 1.6f*fluidRestOffset;
	p.clearFilteringPasses();
	p.numMeshSmoothingPasses = 4;
	p.numMeshNormalSmoothingPasses = 4;

	gIsosuraceCallback.initialize(gScene->getCudaContextManager(), sgIsosurfaceParams, p, 2*1024 * 1024, 4*1024 * 1024, numX * numY * numZ);
	gParticleSystem->setParticleSystemCallback(&gIsosuraceCallback);

	// Setup rigid bodies
	const PxReal dynamicsDensity = fluidDensity * 0.5f;
	const PxReal boxSize = 1.0f;
	const PxReal boxMass = boxSize * boxSize * boxSize * dynamicsDensity;
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.5f * boxSize, 0.5f * boxSize, 0.5f * boxSize), *gMaterial);
	for (int i = 0; i < 5; ++i)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(PxVec3(i - 8.0f, 10, 7.5f)));
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, boxMass);
		gScene->addActor(*body);
	}
	shape->release();
}

// ---------------------------------------------------
PxI32 stepCounter = 0;
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		const PxReal dt = 1.0f / 60.0f;

		if (movingWall)
		{
			static bool moveOut = false;
			const PxReal speed = stepCounter > 1200 ? 2.0f : 0.0f;
			PxTransform pose = movingWall->getGlobalPose();
			if (moveOut)
			{
				pose.p.x += dt * speed;
				if (pose.p.x > -7.f)
					moveOut = false;
			}
			else
			{
				pose.p.x -= dt * speed;
				if (pose.p.x < -11.5f)
					moveOut = true;
			}
			movingWall->setKinematicTarget(pose);
		}

		gScene->simulate(dt);
		gScene->fetchResults(true);
		gScene->fetchResultsParticleSystem();
		++stepCounter;
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	gParticleSystem->setParticleSystemCallback(NULL);
	gIsosuraceCallback.release();

	PxCloseGpuExtensions();

	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetIsosurface done.\n");
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
