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

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

#define CUDA_SUCCESS 0

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxParticleSystem* getParticleSystem();
extern PxParticleBuffer* getParticleBuffer();

namespace
{
Snippets::Camera* sCamera;

PxArray<PxVec4>* sPosBufferH;
PxArray<PxVec3>* sPosBuffer3H;
PxArray<PxVec4>* sVelBufferH;
PxArray<PxU32>* sPhasesH;
PxArray<PxVec3>* sColorBuffer3H;

template <typename T>
T* allocD(PxU32 size)
{
	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();
	cudaContexManager->acquireContext();
	CUdeviceptr ptrD;
	PxCUresult result = cudaContexManager->getCudaContext()->memAlloc(&ptrD, sizeof(T) * size);
	if (result != CUDA_SUCCESS)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Mem allocation failed!\n");
	}
	cudaContexManager->releaseContext();
	return (T*)ptrD;
}

template <typename T>
void freeD(T* ptrD)
{
	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();
	cudaContexManager->acquireContext();
	PxCUresult result = cudaContexManager->getCudaContext()->memFree((CUdeviceptr)ptrD);
	if (result != CUDA_SUCCESS)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Mem free failed!\n");
	}
	cudaContexManager->releaseContext();
}

template <typename T>
void copyD2H(PxArray<T>& bufferH, const T* bufferD)
{
	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();

	cudaContexManager->acquireContext();
	PxCUresult copyResult = cudaContexManager->getCudaContext()->memcpyDtoH(bufferH.begin(), reinterpret_cast<CUdeviceptr>(bufferD), sizeof(T) * bufferH.size());
	if (copyResult != CUDA_SUCCESS)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Copy particle data failed!\n");
	}
	cudaContexManager->releaseContext();
}

void copyVec4ToVec3(PxArray<PxVec3>& vec3s, const PxArray<PxVec4>& vec4s)
{
	for (PxU32 i = 0; i < vec4s.size(); ++i)
		vec3s[i] = vec4s[i].getXYZ();
}

PxU32 getGroup(PxU32 phase)
{
	return phase & PxParticlePhaseFlag::eParticlePhaseGroupMask;
}

void mapVec4ToColor3(PxArray<PxVec3>& colors, const PxArray<PxVec4>& vec4s, const PxArray<PxU32>& phases)
{
	for (PxU32 i = 0; i < vec4s.size(); ++i)
	{
		float mag = vec4s[i].getXYZ().magnitude();
		float c = PxMin(0.1f * mag, 1.0f);

		switch (getGroup(phases[i]) % 6)
		{
		case 0:
			colors[i] = PxVec3(1.0f, c, c);
			break;
		case 1:
			colors[i] = PxVec3(c, 1.0f, c);
			break;
		case 2:
			colors[i] = PxVec3(c, c, 1.0f);
			break;
		case 3:
			colors[i] = PxVec3(c, 1.0f, 1.0f);
			break;
		case 4:
			colors[i] = PxVec3(1.0f, c, 1.0f);
			break;
		case 5:
			colors[i] = PxVec3(1.0f, 1.0f, c);
			break;
		default:
			colors[i] = PxVec3(c, c, c);
		}
	}

}

void onBeforeRenderParticles()
{
	PxParticleSystem* particleSystem = getParticleSystem();
	
	if (particleSystem)
	{
		PxParticleBuffer* userBuffer = getParticleBuffer();
		PxVec4* positions = userBuffer->getPositionInvMasses();
		PxVec4* vels = userBuffer->getVelocities();
		PxU32* phases = userBuffer->getPhases();

		const PxU32 numParticles = userBuffer->getNbActiveParticles();

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();

		cudaContexManager->acquireContext();

		PxCudaContext* cudaContext = cudaContexManager->getCudaContext();
		cudaContext->memcpyDtoH(sPosBufferH->begin(), CUdeviceptr(positions), sizeof(PxVec4) * numParticles);
		cudaContext->memcpyDtoH(sVelBufferH->begin(), CUdeviceptr(vels), sizeof(PxVec4) * numParticles);
		cudaContext->memcpyDtoH(sPhasesH->begin(), CUdeviceptr(phases), sizeof(PxU32) * numParticles);

		copyVec4ToVec3(*sPosBuffer3H, *sPosBufferH);
		mapVec4ToColor3(*sColorBuffer3H, *sVelBufferH, *sPhasesH);

		cudaContexManager->releaseContext();
	}

}

void renderParticles()
{

	Snippets::DrawPoints(*sPosBuffer3H, *sColorBuffer3H, 2.0f);
}

void allocParticleBuffers()
{
	PxParticleSystem* particleSystem = getParticleSystem();
	//const PxU32 maxParticles = particleSystem->getMaxParticles();
	if (particleSystem)
	{
		PxParticleBuffer* userBuffer = getParticleBuffer();
		const PxU32 maxParticles = userBuffer->getMaxParticles();

		sPosBufferH = new PxArray<PxVec4>(maxParticles);
		sPosBuffer3H = new PxArray<PxVec3>(maxParticles);
		sVelBufferH = new PxArray<PxVec4>(maxParticles);
		sColorBuffer3H = new PxArray<PxVec3>(maxParticles);
		sPhasesH = new PxArray<PxU32>(maxParticles);
	}
}

void clearupParticleBuffers()
{
	delete sPosBuffer3H;
	delete sPosBufferH;
	delete sVelBufferH;
	delete sColorBuffer3H;
	delete sPhasesH;
}

void renderCallback()
{
	onBeforeRenderParticles();

	stepPhysics(true);

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}
	
	renderParticles();

	Snippets::finishRender();
}

void cleanup()
{
	delete sCamera;
	clearupParticleBuffers();
	cleanupPhysics(true);
}

void exitCallback(void)
{
#if PX_WINDOWS
		cleanup();
#endif
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(15.0f, 10.0f, 15.0f), PxVec3(-0.6f,-0.2f,-0.6f));

	Snippets::setupDefault("PhysX Snippet PBFFluid", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);

	allocParticleBuffers();

	glutMainLoop();

#if PX_LINUX_FAMILY
	cleanup();
#endif
}
#endif
