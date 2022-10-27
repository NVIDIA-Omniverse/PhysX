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

#ifndef PHYSX_SNIPPET_RENDER_H
#define PHYSX_SNIPPET_RENDER_H

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"

#if PX_WINDOWS
	#include <windows.h>
	#pragma warning(disable: 4505)
	#include <GL/glew.h>
	#include <GL/freeglut.h>
#elif PX_LINUX_FAMILY
	#if PX_CLANG
		#pragma clang diagnostic push
		#pragma clang diagnostic ignored "-Wreserved-identifier"
	#endif
	#include <GL/glew.h>
	#include <GL/freeglut.h>	
	#if PX_CLANG
		#pragma clang diagnostic pop
	#endif
#elif PX_OSX
	#include <GL/glew.h>
	#include <GLUT/glut.h>	
#else
	#error platform not supported.
#endif

typedef	void	(*KeyboardCallback)	(unsigned char key, const physx::PxTransform& camera);
typedef	void	(*RenderCallback)	();
typedef	void	(*ExitCallback)		();

namespace Snippets
{
	class Camera;
	void setupDefault(const char* name, Camera* camera, KeyboardCallback kbcb, RenderCallback rdcb, ExitCallback excb);
	Camera* getCamera();
	physx::PxVec3 computeWorldRayF(float xs, float ys, const physx::PxVec3& camDir);
	PX_FORCE_INLINE physx::PxVec3 computeWorldRay(int xs, int ys, const physx::PxVec3& camDir)
	{
		return computeWorldRayF(float(xs), float(ys), camDir);
	}

	physx::PxU32	getScreenWidth();
	physx::PxU32	getScreenHeight();

	void	enableVSync(bool vsync);

	void	startRender(const Camera* camera, float nearClip = 1.0f, float farClip = 10000.0f, float fov=60.0f, bool setupLighting=true);
	void	finishRender();
	void	print(const char* text);

	class TriggerRender
	{
		public:
		virtual	bool	isTrigger(physx::PxShape*)	const	= 0;
	};

#if PX_SUPPORT_GPU_PHYSX
	class SharedGLBuffer
	{
	public:
		SharedGLBuffer();
		~SharedGLBuffer();
		void initialize(physx::PxCudaContextManager* contextManager);
		void allocate(physx::PxU32 sizeInBytes);
		void release();
		void* map();
		void unmap();

	private:
		physx::PxCudaContextManager* cudaContextManager;
		void* vbo_res;
		void* devicePointer;
	public:
		GLuint vbo; //Opengl vertex buffer object
		physx::PxU32 size;
	};
#endif

	void	initFPS();
	void	showFPS(int updateIntervalMS = 30, const char* info = NULL);

	void	renderSoftBody(physx::PxSoftBody* softBody, const physx::PxArray<physx::PxVec4>& deformedPositionsInvMass, bool shadows, const physx::PxVec3& color = physx::PxVec3(0.0f, 0.75f, 0.0f));
	void	renderActors(physx::PxRigidActor** actors, const physx::PxU32 numActors, bool shadows = false, const physx::PxVec3& color = physx::PxVec3(0.0f, 0.75f, 0.0f), TriggerRender* cb = NULL, bool changeColorForSleepingActors = true, bool wireframePass=true);
//	void	renderGeoms(const physx::PxU32 nbGeoms, const physx::PxGeometry* geoms, const physx::PxTransform* poses, bool shadows, const physx::PxVec3& color);
	void	renderGeoms(const physx::PxU32 nbGeoms, const physx::PxGeometryHolder* geoms, const physx::PxTransform* poses, bool shadows, const physx::PxVec3& color);
	void	renderMesh(physx::PxU32 nbVerts, const physx::PxVec3* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec3* normals = NULL, bool flipFaceOrientation = false);
	void	renderMesh(physx::PxU32 nbVerts, const physx::PxVec4* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec4* normals = NULL, bool flipFaceOrientation = false);
	void	renderHairSystem(physx::PxHairSystem* hairSystem, const physx::PxVec4* vertexPositionInvMass, physx::PxU32 numVertices);

	void	DrawLine(const physx::PxVec3& p0, const physx::PxVec3& p1, const physx::PxVec3& color);
	void	DrawPoints(const physx::PxArray<physx::PxVec3>& pts, const physx::PxVec3& color, float scale);
	void	DrawPoints(const physx::PxArray<physx::PxVec3>& pts, const physx::PxArray<physx::PxVec3>& colors, float scale);
	void	DrawPoints(const physx::PxArray<physx::PxVec4>& pts, const physx::PxVec3& color, float scale);
	void	DrawPoints(const physx::PxArray<physx::PxVec4>& pts, const physx::PxArray<physx::PxVec3>& colors, float scale);
	void	DrawPoints(GLuint vbo, physx::PxU32 numPoints, const physx::PxVec3& color, float scale, physx::PxU32 coordinatesPerPoint = 3, physx::PxU32 stride = 4 * sizeof(float), size_t offset = 0);
	
	void	DrawLines(GLuint vbo, physx::PxU32 numPoints, const physx::PxVec3& color, float scale, physx::PxU32 coordinatesPerPoint = 3, physx::PxU32 stride = 4 * sizeof(float), size_t offset = 0);
	
	void	DrawIcosahedraPoints(const physx::PxArray<physx::PxVec3>& pts, const physx::PxVec3& color, float radius);
	void	DrawFrame(const physx::PxVec3& pt, float scale=1.0f);
	void	DrawBounds(const physx::PxBounds3& box);
	void	DrawBounds(const physx::PxBounds3& box, const physx::PxVec3& color);
	void	DrawMeshIndexedNoNormals(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const physx::PxVec3& color, physx::PxU32 stride = 4 * sizeof(float));
	void	DrawMeshIndexed(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const physx::PxVec3& color, physx::PxU32 stride = 6 * sizeof(float));

	GLuint	CreateTexture(physx::PxU32 width, physx::PxU32 height, const GLubyte* buffer, bool createMipmaps);
	void	UpdateTexture(GLuint texId, physx::PxU32 width, physx::PxU32 height, const GLubyte* buffer, bool createMipmaps);
	void	ReleaseTexture(GLuint texId);
	void	DrawRectangle(float x_start, float x_end, float y_start, float y_end, const physx::PxVec3& color_top, const physx::PxVec3& color_bottom, float alpha, physx::PxU32 screen_width, physx::PxU32 screen_height, bool draw_outline, bool texturing);
	void	DisplayTexture(GLuint texId, physx::PxU32 size, physx::PxU32 margin);
}

#endif //PHYSX_SNIPPET_RENDER_H
