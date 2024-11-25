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
// This snippet demonstrates how to setup triangle meshes with SDFs.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetdeformablesurfaceskinning/SnippetDeformableSurfaceSkinning.h"
#include "PxDeformableSkinning.h"
#include "gpu/PxPhysicsGpu.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxDeformableSkinningExt.h"
#include "extensions/PxRemeshingExt.h"

using namespace physx;
using namespace physx::Ext;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation			= NULL;
static PxPhysics*				gPhysics			= NULL;
static PxCudaContextManager*	gCudaContextManager	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher			= NULL;
static PxScene*					gScene				= NULL;
static PxMaterial*				gMaterial			= NULL;
static PxPvd*					gPvd				= NULL;
static bool						gIsRunning			= true;
PxArray<DeformableSurface>		gDeformableSurfaces;
PxArray<SkinnedMesh>			gSkinnedMeshes;
BasePostSolveCallback*			gSkinning;

PxRigidDynamic* sphere;

template<typename T>
struct HostAndDeviceBuffer
{
	PxCudaContextManager* mContextManager;
	T* mDeviceData;
	T* mHostData;
	PxU32 mNumElements;

	HostAndDeviceBuffer() :
		mContextManager(NULL), mDeviceData(NULL), mHostData(NULL), mNumElements(0)
	{}

	HostAndDeviceBuffer(PxCudaContextManager* contextManager, PxU32 numElements) : 
		mContextManager(contextManager), mDeviceData(NULL), mHostData(NULL), mNumElements(0)
	{
		allocate(numElements);
	}

	void initialize(PxCudaContextManager* contextManager, PxU32 numElements)
	{
		mContextManager = contextManager;
		allocate(numElements);
	}
	
	void initialize(PxCudaContextManager* contextManager, const T* dataSource, PxU32 numElements)
	{
		mContextManager = contextManager;
		allocate(numElements);
		PxMemCopy(mHostData, dataSource, numElements * sizeof(T));
	}

	void allocate(PxU32 numElements)
	{
		release(); 
		mDeviceData = PxCudaHelpersExt::allocDeviceBuffer<T>(*mContextManager, numElements);
		mHostData = PxCudaHelpersExt::allocPinnedHostBuffer<T>(*mContextManager, numElements);
		mNumElements = numElements;
	}

	void copyDeviceToHost(PxU32 numElementsToCopy = 0xFFFFFFFF)
	{
		PxCudaHelpersExt::copyDToH(*mContextManager, mHostData, mDeviceData, PxMin(numElementsToCopy, mNumElements));
	}

	void copyHostToDevice(PxU32 numElementsToCopy = 0xFFFFFFFF)
	{
		PxCudaHelpersExt::copyHToD<T>(*mContextManager, mDeviceData, mHostData, PxMin(numElementsToCopy, mNumElements));
	}

	void copyDeviceToHostAsync(CUstream stream, PxU32 numElementsToCopy = 0xFFFFFFFF)
	{
		PxCudaHelpersExt::copyDToHAsync(*mContextManager, mHostData, mDeviceData, PxMin(numElementsToCopy, mNumElements), stream);
	}

	void release()
	{
		PxCudaHelpersExt::freeDeviceBuffer(*mContextManager, mDeviceData);
		PxCudaHelpersExt::freePinnedHostBuffer(*mContextManager, mHostData);
	}
};


struct SurfaceSkinningHelper
{
	PxDeformableSurface* mDeformableSurface;
	HostAndDeviceBuffer<PxU32> mSurfaceTriangles;
	HostAndDeviceBuffer<PxVec3> mNormalVectors;
	HostAndDeviceBuffer<PxTriangleMeshEmbeddingInfo> mSkinningInfo;
	HostAndDeviceBuffer<PxVec3> mSkinnedVertices;
	PxU32 mNumSkinnedVertices;
	PxReal mHalfThickness;


	SurfaceSkinningHelper() : mDeformableSurface(NULL), mNumSkinnedVertices(0)
	{ }

	SurfaceSkinningHelper(PxCudaContextManager* contextManager, PxDeformableSurface* deformableSurface, PxVec3* skinnedPointsRestPosition, PxU32 nbSkinnedPoints)
		: mDeformableSurface(deformableSurface)
	{
		const physx::PxTriangleMeshGeometry& triangleMeshGeom =
			static_cast<const physx::PxTriangleMeshGeometry&>(deformableSurface->getShape()->getGeometry());

		PxU32 nbTriangles = triangleMeshGeom.triangleMesh->getNbTriangles();

		bool uses16bit = triangleMeshGeom.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
		mSurfaceTriangles.initialize(contextManager, 3 * nbTriangles);

		if (uses16bit)
		{
			const PxU16* triangleIndices = reinterpret_cast<const PxU16*>(triangleMeshGeom.triangleMesh->getTriangles());
			for (PxU32 i = 0; i < mSurfaceTriangles.mNumElements; ++i)
				mSurfaceTriangles.mHostData[i] = triangleIndices[i];
		}
		else
		{
			const PxU32* triangleIndices = reinterpret_cast<const PxU32*>(triangleMeshGeom.triangleMesh->getTriangles());
			for (PxU32 i = 0; i < mSurfaceTriangles.mNumElements; ++i)
				mSurfaceTriangles.mHostData[i] = triangleIndices[i];
		}

		mNormalVectors.initialize(contextManager, triangleMeshGeom.triangleMesh->getNbVertices());
		mSkinnedVertices.initialize(contextManager, skinnedPointsRestPosition, nbSkinnedPoints);
		mNumSkinnedVertices = nbSkinnedPoints;
		mSkinningInfo.initialize(contextManager, nbSkinnedPoints);

		PxDeformableSkinningExt::initializeInterpolatedVertices(
			mSkinningInfo.mHostData, triangleMeshGeom.triangleMesh->getVertices(), NULL, mSurfaceTriangles.mHostData,
			nbTriangles, skinnedPointsRestPosition, nbSkinnedPoints);

		mSurfaceTriangles.copyHostToDevice();
		mSkinnedVertices.copyHostToDevice();
		mSkinningInfo.copyHostToDevice();

		PxShape* surfaceShape = deformableSurface->getShape();
		mHalfThickness = surfaceShape->getRestOffset();
	}

	void packageGpuData(PxTrimeshSkinningGpuData& target)
	{
		target.guideVerticesD.data = reinterpret_cast<PxVec3*>(mDeformableSurface->getPositionInvMassBufferD());
		target.guideVerticesD.stride = sizeof(PxVec4);
		target.guideVerticesD.count = mNormalVectors.mNumElements;

		target.guideNormalsD = mNormalVectors.mDeviceData;
		target.guideTrianglesD = mSurfaceTriangles.mDeviceData;
		target.skinningInfoPerVertexD = mSkinningInfo.mDeviceData;
		target.skinnedVerticesD.data = mSkinnedVertices.mHostData; //Works because host data is pinned memory
		target.skinnedVerticesD.stride = sizeof(PxVec3);
		target.skinnedVerticesD.count = mNumSkinnedVertices;
		
		target.halfSurfaceThickness = mHalfThickness;
		target.nbGuideTriangles = mSurfaceTriangles.mNumElements / 3;
		
	}

	void release()
	{
		mSurfaceTriangles.release();
		mNormalVectors.release();
		mSkinnedVertices.release();
		mSkinningInfo.release();
	}
};

struct PostSolveCallback : BasePostSolveCallback, PxUserAllocated
{
	CUstream mSkinningStream;
	PxCudaContextManager* mContextManager;
	PxDeformableSkinning* skinning;
	PxArray<SurfaceSkinningHelper> skinningHelpers;
	HostAndDeviceBuffer<PxTrimeshSkinningGpuData> packagedSkinningData;


	PostSolveCallback(PxCudaContextManager* contextManager, PxU32 maxNumCloths) :
		mContextManager(contextManager)
	{
		const PxU32 CU_STREAM_NON_BLOCKING = 0x1;
		mContextManager->getCudaContext()->streamCreate(&mSkinningStream, CU_STREAM_NON_BLOCKING);

		skinning = PxGetPhysicsGpu()->createDeformableSkinning(contextManager);

		packagedSkinningData.initialize(contextManager, maxNumCloths);
		skinningHelpers.resize(maxNumCloths);
	}

	void setCloth(PxU32 index, PxDeformableSurface* deformableSurface, PxVec3* skinnedPointsRestPosition, PxU32 nbSkinnedPoints)
	{
		skinningHelpers[index] = SurfaceSkinningHelper(mContextManager, deformableSurface, skinnedPointsRestPosition, nbSkinnedPoints);
	}

	virtual void onPostSolve(CUevent startEvent)
	{
		mContextManager->getCudaContext()->streamWaitEvent(mSkinningStream, startEvent);

		for (PxU32 i = 0; i < skinningHelpers.size(); ++i)
		{
			skinningHelpers[i].packageGpuData(packagedSkinningData.mHostData[i]);
		}
		packagedSkinningData.copyHostToDevice(skinningHelpers.size());

		skinning->computeNormalVectors(packagedSkinningData.mDeviceData, skinningHelpers.size(), mSkinningStream);
		skinning->evaluateVerticesEmbeddedIntoSurface(packagedSkinningData.mDeviceData, skinningHelpers.size(), mSkinningStream);

		//mSkinnedVertices.copyDeviceToHostAsync(mSkinningStream);
	}

	virtual void synchronize()
	{
		mContextManager->getCudaContext()->streamSynchronize(mSkinningStream);
	}

	virtual PxVec3* getSkinnedVertices(PxU32 clothIndex)
	{
		return skinningHelpers[clothIndex].mSkinnedVertices.mHostData;
	}

	~PostSolveCallback()
	{
		mContextManager->getCudaContext()->streamDestroy(mSkinningStream);
		for (PxU32 i = 0; i < skinningHelpers.size(); ++i)
			skinningHelpers[i].release();
		PX_DELETE(skinning);
	}
};

PostSolveCallback* postSolveCallback;


static void initObstacles()
{
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(3.0f), *gMaterial);
	sphere = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, 5.0f, 0.f)));
	sphere->attachShape(*shape);
	sphere->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*sphere);
	shape->release();
}

static PxDeformableSurface* createDeformableSurface(PxPhysics& physics, PxTriangleMesh* triangleMesh, PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager)
{
	if (!triangleMesh)
		return NULL;

	PxDeformableSurface* deformableSurface = physics.createDeformableSurface(*cudaContextManager);
	if (deformableSurface)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

		PxTriangleMeshGeometry geometry(triangleMesh);
		PxShape* shape = physics.createShape(geometry, materials, PxU16(nbMaterials), true, shapeFlags);
		if (shape)
		{
			deformableSurface->attachShape(*shape);
		}
	}

	return deformableSurface;
}

static PxDeformableSurface* createDeformableSurface(PxPhysics& physics, const PxCookingParams& ckParams, PxArray<PxVec3>& vertices, PxArray<PxU32>& triangles,
	PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = vertices.size();
	meshDesc.triangles.count = triangles.size() / 3;
	meshDesc.points.stride = sizeof(float) * 3;
	meshDesc.triangles.stride = sizeof(int) * 3;
	meshDesc.points.data = vertices.begin();
	meshDesc.triangles.data = triangles.begin();

	PxDeformableMaterialTableIndex* materialIndices = NULL;
	if (nbMaterials > 1)
	{
		const PxU32 totalTriangles = meshDesc.triangles.count;
		materialIndices = new PxDeformableMaterialTableIndex[totalTriangles];
		const PxU32 averageTrianglePerMaterials = totalTriangles / nbMaterials;

		PxU32 accumulatedTriangle = averageTrianglePerMaterials;

		PxU32 index = 0;
		for (PxU32 i = 0; i < totalTriangles; ++i)
		{
			materialIndices[i] = PxDeformableMaterialTableIndex(index);
			if (i == accumulatedTriangle)
			{
				index = index < (nbMaterials - 1) ? index + 1 : index;
				accumulatedTriangle += averageTrianglePerMaterials;
			}
		}
		meshDesc.materialIndices.stride = sizeof(PxDeformableMaterialTableIndex);
		meshDesc.materialIndices.data = materialIndices;
	}

	PxTriangleMesh* triangleMesh = PxCreateTriangleMesh(ckParams, meshDesc, physics.getPhysicsInsertionCallback());

	return createDeformableSurface(physics, triangleMesh, materials, nbMaterials, cudaContextManager);
}

static PX_FORCE_INLINE PxU32 id(PxU32 x, PxU32 y, PxU32 numY)
{
	return x * numY + y;
}

static PX_FORCE_INLINE PxReal computeTriangleMass(const PxU32* triangle, const PxVec3* vertices, PxReal thickness, PxReal density)
{
	PxReal area = 0.5f * (vertices[triangle[1]] - vertices[triangle[0]]).cross(vertices[triangle[2]] - vertices[triangle[0]]).magnitude();
	return area * thickness * density;
}

static PxDeformableSurface* addQuadDeformableSurface(PxPhysics& physics, const PxCookingParams& ckParams, const PxTransform& transform, PxU32 numX, PxU32 numZ, PxReal sizeX, PxReal sizeZ,
	PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager, PxReal thickness, PxReal density)
{
	if (gCudaContextManager == NULL)
	{
		return NULL;
	}

	PxArray<PxVec3> vertices;
	PxArray<PxU32> triangles;
	PxArray<PxVec3> velocity;
	PxArray<PxReal> triangleMasses;

	vertices.reserve(numX * numZ);
	velocity.reserve(numX * numZ);
	triangles.reserve(3 * 2 * (numX - 1) * (numZ - 1));
	triangleMasses.reserve(2 * (numX - 1) * (numZ - 1));

	PxReal scalingX = sizeX / (numX - 1);
	PxReal scalingZ = sizeZ / (numZ - 1);

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numZ; ++j)
		{
			PxVec3 pos(i * scalingX, 0.0f, j * scalingZ);
			vertices.pushBack(pos);
			velocity.pushBack(PxVec3(0.0f));
		}
	}

	for (PxU32 i = 1; i < numX; ++i)
	{
		for (PxU32 j = 1; j < numZ; ++j)
		{
			triangles.pushBack(id(i - 1, j - 1, numZ));
			triangles.pushBack(id(i, j - 1, numZ));
			triangles.pushBack(id(i - 1, j, numZ));			
			triangleMasses.pushBack(computeTriangleMass(&triangles[triangles.size() - 3], vertices.begin(), thickness, density));

			triangles.pushBack(id(i - 1, j, numZ));
			triangles.pushBack(id(i, j - 1, numZ));
			triangles.pushBack(id(i, j, numZ));
			triangleMasses.pushBack(computeTriangleMass(&triangles[triangles.size() - 3], vertices.begin(), thickness, density));
		}
	}

	PxArray<PxU32> subdividedTriangles = triangles;
	PxArray<PxVec3> subdividedVertices = vertices;
	PxRemeshingExt::limitMaxEdgeLength(subdividedTriangles, subdividedVertices, 0.0001f, 3);

	SkinnedMesh mesh;
	for (PxU32 i = 0; i < subdividedTriangles.size(); ++i)
		mesh.mTriangles.pushBack(subdividedTriangles[i]);
	for (PxU32 i = 0; i < subdividedVertices.size(); ++i)
		mesh.mVertices.pushBack(subdividedVertices[i]);

	gSkinnedMeshes.pushBack(mesh);
	

	PxDeformableSurface* deformableSurface = createDeformableSurface(physics, ckParams, vertices, triangles, materials, nbMaterials, cudaContextManager);
	gScene->addActor(*deformableSurface);

	PxVec4* posInvMassPinned;
	PxVec4* velocityPinned;
	PxVec4* restPositionPinned;
	PxDeformableSurfaceExt::allocateAndInitializeHostMirror(*deformableSurface, vertices.begin(), velocity.begin(), vertices.begin(), 0.5f,
		transform, cudaContextManager, posInvMassPinned, velocityPinned, restPositionPinned);

	PxDeformableSurfaceExt::distributeTriangleMassToVertices(*deformableSurface, triangleMasses.begin(), posInvMassPinned);

	PxShape* surfaceShape = deformableSurface->getShape();
	surfaceShape->setContactOffset(2.0f * thickness);
	surfaceShape->setRestOffset(thickness);
	surfaceShape->getDeformableSurfaceMaterials(materials, PxU16(nbMaterials));

	PxDeformableSurfaceExt::copyToDevice(*deformableSurface, PxDeformableSurfaceDataFlag::eALL, vertices.size(), posInvMassPinned, velocityPinned, restPositionPinned);
	
	DeformableSurface cloth(deformableSurface, gCudaContextManager);

	gDeformableSurfaces.pushBack(cloth);

	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, posInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, velocityPinned);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, restPositionPinned);

	return deformableSurface;
}

static void createScene(PxCookingParams& cookingParams)
{
	PxReal thickness = 0.075f;
	PxReal bendingStiffness = 0.00001f;

	PxDeformableSurfaceMaterial* clothMaterial = gPhysics->createDeformableSurfaceMaterial(1.e8f, 0.3f, 0.5f, thickness, bendingStiffness);

	PxReal size = 15.0f;

	for (PxU32 i = 0; i < 2; ++i)
	{
		PxDeformableSurface* deformableSurface = addQuadDeformableSurface(*gPhysics, cookingParams, PxTransform(PxVec3(-0.5f * size, 10.0f + i, -0.5f * size)), 30, 30, size, size, &clothMaterial, 1, gCudaContextManager, thickness, 500.0f);
		if (deformableSurface)
		{
			deformableSurface->setSelfCollisionFilterDistance(thickness * 2.5f);
			deformableSurface->setLinearDamping(0.f);
			deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, false);
			deformableSurface->setMaxVelocity(1000.0f);

			PxU32 collisionPairUpdateFrequency = 1;
			PxU32 nbCollisionSubsteps = 1;
			deformableSurface->setNbCollisionPairUpdatesPerTimestep(collisionPairUpdateFrequency);
			deformableSurface->setNbCollisionSubsteps(nbCollisionSubsteps);

		}
	}

	postSolveCallback = PX_NEW(PostSolveCallback)(gCudaContextManager, PxU32(gSkinnedMeshes.size()));
	gSkinning = postSolveCallback;
	gScene->setDeformableSurfaceGpuPostSolveCallback(postSolveCallback);

	for (PxU32 i = 0; i < gSkinnedMeshes.size(); ++i)
	{
		SkinnedMesh& skinnedMesh = gSkinnedMeshes[i];		
		postSolveCallback->setCloth(i, gDeformableSurfaces[i].mDeformableSurface, &skinnedMesh.mVertices[0], PxU32(skinnedMesh.mVertices.size()));
	}


	initObstacles();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	
	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The DeformableSurface feature is currently only supported on GPU.\n");
	}

	PxTolerancesScale scale;
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxCookingParams params(scale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	params.buildTriangleAdjacencies = false;
	params.buildGPUData = true;
	params.midphaseDesc = PxMeshMidPhase::eBVH34;
	//params.meshPreprocessParams |= PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING;

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	if (!sceneDesc.cudaContextManager)
		sceneDesc.cudaContextManager = gCudaContextManager;
	
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.gpuMaxNumPartitions = 8;

	sceneDesc.solverType = PxSolverType::eTGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	createScene(params);
}

PxReal simTime = 0;
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		const PxReal dt = 1.0f / 60.0f;

		bool rotatingSphere = true;
		if (rotatingSphere)
		{
			const PxReal speed = 2.0f;
			PxTransform pose = sphere->getGlobalPose();
			sphere->setKinematicTarget(PxTransform(pose.p, PxQuat(PxCos(simTime*speed), PxVec3(0, 1, 0))));
		}

		gScene->simulate(dt);
		gScene->fetchResults(true);
		
		for (PxU32 i = 0; i < gDeformableSurfaces.size(); i++)
		{
			DeformableSurface* c = &gDeformableSurfaces[i];
			c->copyDeformedVerticesFromGPU();
		}

		simTime += dt;
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_DELETE(postSolveCallback);

	for (PxU32 i = 0; i < gDeformableSurfaces.size(); i++)
		gDeformableSurfaces[i].release();
	gDeformableSurfaces.reset();

	gSkinnedMeshes.reset();

	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PxCloseExtensions();
	PX_RELEASE(gCudaContextManager);
	PX_RELEASE(gFoundation);

	printf("SnippetDeformableSurfaceSkinning done.\n");
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
