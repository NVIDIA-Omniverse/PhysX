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

// ****************************************************************************
// This snippet illustrates the use of the dll delay load hooks in physx.
//
// The hooks are needed if the application executable either doesn't reside 
// in the same directory as the PhysX dlls, or if the PhysX dlls have been renamed.
// Some PhysX dlls delay load the PhysXFoundation, PhysXCommon or PhysXGpu dlls and
// the non-standard names or loactions of these dlls need to be communicated so the 
// delay loading can succeed.
//
// This snippet shows how this can be done using the delay load hooks.
//
// In order to show functionality with the renamed dlls some basic physics 
// simulation is performed.
// ****************************************************************************

#include <ctype.h>
#include <wtypes.h>
#include "PxPhysicsAPI.h"
// Include the delay load hook headers
#include "common/windows/PxWindowsDelayLoadHook.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

// This snippet uses the default PhysX distro dlls, making the example here somewhat artificial, 
// as default locations and default naming makes implementing delay load hooks unnecessary.
#define APP_BIN_DIR "..\\"
#if PX_WIN64
	#define DLL_NAME_BITS "64" 
#else
	#define DLL_NAME_BITS "32" 
#endif
#if PX_DEBUG
	#define DLL_DIR "debug\\"
#elif PX_CHECKED
	#define DLL_DIR "checked\\" 
#elif PX_PROFILE
	#define DLL_DIR "profile\\" 
#else
	#define DLL_DIR "release\\" 
#endif

const char* foundationLibraryPath = APP_BIN_DIR DLL_DIR "PhysXFoundation_" DLL_NAME_BITS ".dll";
const char* commonLibraryPath = APP_BIN_DIR DLL_DIR "PhysXCommon_" DLL_NAME_BITS ".dll";
const char* physxLibraryPath = APP_BIN_DIR DLL_DIR "PhysX_" DLL_NAME_BITS ".dll";
const char* gpuLibraryPath = APP_BIN_DIR DLL_DIR "PhysXGpu_" DLL_NAME_BITS ".dll";

HMODULE foundationLibrary = NULL;
HMODULE commonLibrary = NULL;
HMODULE physxLibrary = NULL;

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*						gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;

// typedef the PhysX entry points
typedef PxFoundation*(PxCreateFoundation_FUNC)(PxU32, PxAllocatorCallback&, PxErrorCallback&);
typedef PxPhysics* (PxCreatePhysics_FUNC)(PxU32,PxFoundation&,const PxTolerancesScale& scale,bool,PxPvd*);
typedef void (PxSetPhysXDelayLoadHook_FUNC)(const PxDelayLoadHook* hook);
typedef void (PxSetPhysXCommonDelayLoadHook_FUNC)(const PxDelayLoadHook* hook);
#if PX_SUPPORT_GPU_PHYSX
typedef void (PxSetPhysXGpuLoadHook_FUNC)(const PxGpuLoadHook* hook);
typedef int (PxGetSuggestedCudaDeviceOrdinal_FUNC)(PxErrorCallback& errc);
typedef PxCudaContextManager* (PxCreateCudaContextManager_FUNC)(PxFoundation& foundation, const PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback);
#endif

// set the function pointers to NULL
PxCreateFoundation_FUNC* s_PxCreateFoundation_Func = NULL;
PxCreatePhysics_FUNC* s_PxCreatePhysics_Func = NULL;
PxSetPhysXDelayLoadHook_FUNC* s_PxSetPhysXDelayLoadHook_Func = NULL;
PxSetPhysXCommonDelayLoadHook_FUNC* s_PxSetPhysXCommonDelayLoadHook_Func = NULL;
#if PX_SUPPORT_GPU_PHYSX
PxSetPhysXGpuLoadHook_FUNC* s_PxSetPhysXGpuLoadHook_Func = NULL;
PxGetSuggestedCudaDeviceOrdinal_FUNC* s_PxGetSuggestedCudaDeviceOrdinal_Func = NULL;
PxCreateCudaContextManager_FUNC* s_PxCreateCudaContextManager_Func = NULL;
#endif

bool loadPhysicsExplicitely()
{
	// load the dlls
	foundationLibrary = LoadLibraryA(foundationLibraryPath);	
	if(!foundationLibrary)
		return false;

	commonLibrary = LoadLibraryA(commonLibraryPath);	
	if(!commonLibrary)
	{
		FreeLibrary(foundationLibrary);
		return false;
	}

	physxLibrary = LoadLibraryA(physxLibraryPath);	
	if(!physxLibrary)
	{
		FreeLibrary(foundationLibrary);
		FreeLibrary(commonLibrary);
		return false;
	}

	// get the function pointers
	s_PxCreateFoundation_Func = (PxCreateFoundation_FUNC*)GetProcAddress(foundationLibrary, "PxCreateFoundation");
	s_PxCreatePhysics_Func = (PxCreatePhysics_FUNC*)GetProcAddress(physxLibrary, "PxCreatePhysics");
	s_PxSetPhysXDelayLoadHook_Func = (PxSetPhysXDelayLoadHook_FUNC*)GetProcAddress(physxLibrary, "PxSetPhysXDelayLoadHook");
	s_PxSetPhysXCommonDelayLoadHook_Func = (PxSetPhysXCommonDelayLoadHook_FUNC*)GetProcAddress(commonLibrary, "PxSetPhysXCommonDelayLoadHook");

#if PX_SUPPORT_GPU_PHYSX
	s_PxSetPhysXGpuLoadHook_Func = (PxSetPhysXGpuLoadHook_FUNC*)GetProcAddress(physxLibrary, "PxSetPhysXGpuLoadHook");
	s_PxGetSuggestedCudaDeviceOrdinal_Func = (PxGetSuggestedCudaDeviceOrdinal_FUNC*)GetProcAddress(physxLibrary, "PxGetSuggestedCudaDeviceOrdinal");
	s_PxCreateCudaContextManager_Func = (PxCreateCudaContextManager_FUNC*)GetProcAddress(physxLibrary, "PxCreateCudaContextManager");
#endif

	// check if we have all required function pointers
	if(s_PxCreateFoundation_Func == NULL || s_PxCreatePhysics_Func == NULL || s_PxSetPhysXDelayLoadHook_Func == NULL || s_PxSetPhysXCommonDelayLoadHook_Func == NULL)
		return false;

#if PX_SUPPORT_GPU_PHYSX
	if(s_PxSetPhysXGpuLoadHook_Func == NULL || s_PxGetSuggestedCudaDeviceOrdinal_Func == NULL || s_PxCreateCudaContextManager_Func == NULL)
		return false;
#endif
	return true;
}

// unload the dlls
void unloadPhysicsExplicitely()
{
	FreeLibrary(physxLibrary);
	FreeLibrary(commonLibrary);
	FreeLibrary(foundationLibrary);
}

// Overriding the PxDelayLoadHook allows the load of a custom name dll inside PhysX, PhysXCommon and PhysXCooking dlls
struct SnippetDelayLoadHook : public PxDelayLoadHook
{
	virtual const char* getPhysXFoundationDllName() const 
	{
		return foundationLibraryPath;
	}

	virtual const char* getPhysXCommonDllName() const 
	{
		return commonLibraryPath;
	}
};

#if PX_SUPPORT_GPU_PHYSX
// Overriding the PxGpuLoadHook allows the load of a custom GPU name dll
struct SnippetGpuLoadHook : public PxGpuLoadHook
{
	virtual const char* getPhysXGpuDllName() const
	{
		return gpuLibraryPath;
	}
};
#endif

PxReal stackZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool interactive)
{	
	// load the explictely named dlls
	const bool isLoaded = loadPhysicsExplicitely();
	if (!isLoaded)
		return;

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	// set PhysX and PhysXCommon delay load hook, this must be done before the create physics is called, before
	// the PhysXFoundation, PhysXCommon delay load happens.
	SnippetDelayLoadHook delayLoadHook;
	s_PxSetPhysXDelayLoadHook_Func(&delayLoadHook);
	s_PxSetPhysXCommonDelayLoadHook_Func(&delayLoadHook);

#if PX_SUPPORT_GPU_PHYSX
	// set PhysXGpu load hook
	SnippetGpuLoadHook gpuLoadHook;
	s_PxSetPhysXGpuLoadHook_Func(&gpuLoadHook);
#endif

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	// We setup the delay load hooks first

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	for(PxU32 i=0;i<5;i++)
		createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
}

void stepPhysics(bool /*interactive*/)
{
	if (gScene)
	{
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
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

	unloadPhysicsExplicitely();
	
	printf("SnippetDelayLoadHook done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
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
