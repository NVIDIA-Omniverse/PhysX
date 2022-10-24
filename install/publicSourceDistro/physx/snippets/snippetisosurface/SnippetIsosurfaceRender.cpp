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

#include "gpuextensions/PxIsosurfaceExtraction.h"
#include "foundation/PxArray.h"

#define CUDA_SUCCESS 0
#define SHOW_SOLID_SDF_SLICE 0
#define IDX(i, j, k, offset) ((i) + dimX * ((j) + dimY * ((k) + dimZ * (offset))))
using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxPBDParticleSystem* getParticleSystem();
extern PxParticleBuffer* getParticleBuffer();


#if PX_SUPPORT_GPU_PHYSX
extern PxArray<PxVec4> gIsosurfaceVertices;
extern PxArray<PxU32> gIsosurfaceIndices;
extern PxArray<PxVec4> gIsosurfaceNormals;
extern ExtGpu::PxIsosurfaceExtractor* gIsosurfaceExtractor;
void* gVerticesGpu;
void* gNormalsGpu;
void* gInterleavedVerticesAndNormalsGpu;
#endif

namespace
{
	Snippets::Camera* sCamera;

#if PX_SUPPORT_GPU_PHYSX

	bool directGpuRendering = true;
	Snippets::SharedGLBuffer sPosBuffer;
	Snippets::SharedGLBuffer sNormalBuffer;
	Snippets::SharedGLBuffer sTriangleBuffer;
	Snippets::SharedGLBuffer sInterleavedPosNormalBuffer;
	Snippets::SharedGLBuffer sParticlePosBuffer;

	void onBeforeRenderParticles()
	{
		PxPBDParticleSystem* particleSystem = getParticleSystem();
		if (particleSystem)
		{
			PxParticleBuffer* userBuffer = getParticleBuffer();
			PxVec4* positions = userBuffer->getPositionInvMasses();
		
			const PxU32 numParticles = userBuffer->getNbActiveParticles();
		
			PxScene* scene;
			PxGetPhysics().getScenes(&scene, 1);
			PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();

			cudaContexManager->acquireContext();

			PxCudaContext* cudaContext = cudaContexManager->getCudaContext();
			cudaContext->memcpyDtoH(sParticlePosBuffer.mapForCudaAccess(), CUdeviceptr(positions), sizeof(PxVec4) * numParticles);
			
			cudaContexManager->releaseContext();
		}
	}

	void renderParticles()
	{
		sParticlePosBuffer.unmap();

		sPosBuffer.unmap();
		sNormalBuffer.unmap();
		sTriangleBuffer.unmap();
		if (directGpuRendering) 
		{	
			PxVec3 color(0.5f, 0.5f, 1);
			Snippets::DrawMeshIndexed(sInterleavedPosNormalBuffer.vbo, sTriangleBuffer.vbo, gIsosurfaceExtractor->getNumTriangles(), color);
		
			//PxVec3 particleColor(1.0f, 1.0f, 0.0f);
			//Snippets::DrawPoints(sParticlePosBuffer.vbo, sParticlePosBuffer.size / sizeof(PxVec4), particleColor, 2.f);
		}
		else 
		{
			//Draw a triangle mesh where the data gets copied to the host and back to the device for rendering
			Snippets::renderMesh(gIsosurfaceExtractor->getNumVertices(), gIsosurfaceVertices.begin(), gIsosurfaceExtractor->getNumTriangles(),
				gIsosurfaceIndices.begin(), PxVec3(1, 0, 0), gIsosurfaceNormals.begin());

			//Check for unused vertices
			PxVec4 marker(10000000, 0, 0, 0);			
			PxU32 numIndices = 3 * gIsosurfaceExtractor->getNumTriangles();
			for (PxU32 i = 0; i < numIndices; ++i)
			{
				gIsosurfaceNormals[gIsosurfaceIndices[i]] = marker;
			}
			for (PxU32 i = 0; i < gIsosurfaceExtractor->getNumVertices(); ++i)
			{
				if (gIsosurfaceNormals[i] != marker)
				{
					printf("Isosurface mesh contains unreferenced vertices\n");
				}
			}
		}

		Snippets::DrawFrame(PxVec3(0, 0, 0));
	}

	void allocParticleBuffers()
	{
		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxCudaContextManager* cudaContexManager = scene->getCudaContextManager();

		//PxPBDParticleSystem* particleSystem = getParticleSystem();
		PxU32 maxVertices = gIsosurfaceExtractor->getMaxVertices();
		PxU32 maxIndices = gIsosurfaceExtractor->getMaxTriangles() * 3;

		sParticlePosBuffer.initialize(cudaContexManager);
		sParticlePosBuffer.allocate(gIsosurfaceExtractor->getMaxParticles() * sizeof(PxVec4));

		sPosBuffer.initialize(cudaContexManager);
		sPosBuffer.allocate(maxVertices * sizeof(PxVec4));
		gVerticesGpu = sPosBuffer.mapForCudaAccess();
		sNormalBuffer.initialize(cudaContexManager);
		sNormalBuffer.allocate(maxVertices * sizeof(PxVec4));
		gNormalsGpu = sNormalBuffer.mapForCudaAccess();
		sTriangleBuffer.initialize(cudaContexManager);
		sTriangleBuffer.allocate(maxIndices * sizeof(PxU32));

		sInterleavedPosNormalBuffer.initialize(cudaContexManager);
		sInterleavedPosNormalBuffer.allocate(2 * maxVertices * sizeof(PxVec3));
		gInterleavedVerticesAndNormalsGpu = sInterleavedPosNormalBuffer.mapForCudaAccess();

		if (directGpuRendering) 
		{
			gIsosurfaceExtractor->setResultBufferDevice(reinterpret_cast<PxVec4*>(sPosBuffer.mapForCudaAccess()),
				reinterpret_cast<PxU32*>(sTriangleBuffer.mapForCudaAccess()), reinterpret_cast<PxVec4*>(sNormalBuffer.mapForCudaAccess()));
		}
		else
		{
			gIsosurfaceExtractor->setResultBufferHost(gIsosurfaceVertices.begin(), gIsosurfaceIndices.begin(), gIsosurfaceNormals.begin());
			gInterleavedVerticesAndNormalsGpu = NULL;
		}
	}

	void clearupParticleBuffers()
	{
		sParticlePosBuffer.release();
		sPosBuffer.release();
		sNormalBuffer.release();
		sTriangleBuffer.release();
		sInterleavedPosNormalBuffer.release();
	}
#else
	void onBeforeRenderParticles()
	{
	}

	void renderParticles()
	{
	}

	void allocParticleBuffers()
	{
	}

	void clearupParticleBuffers()
	{
	}
#endif

	void renderCallback()
	{
		onBeforeRenderParticles();

		stepPhysics(true);

		Snippets::startRender(sCamera);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
		}

		renderParticles();

		Snippets::showFPS();

		Snippets::finishRender();
	}

	void cleanup()
	{
		gIsosurfaceVertices.reset();
		gIsosurfaceIndices.reset();
		gIsosurfaceNormals.reset();

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
	sCamera = new Snippets::Camera(PxVec3(15.0f, 10.0f, 15.0f), PxVec3(-0.6f, -0.2f, -0.6f));

	Snippets::setupDefault("PhysX Snippet Isosurface", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	Snippets::initFPS();

	allocParticleBuffers();

	glutMainLoop();

#if PX_LINUX_FAMILY
	cleanup();
#endif
}
#endif
