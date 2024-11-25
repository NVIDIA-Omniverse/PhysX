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

#include "foundation/PxPreprocessor.h"

#define USE_CUDA_INTEROP (!PX_PUBLIC_RELEASE)

#if (PX_SUPPORT_GPU_PHYSX && USE_CUDA_INTEROP)
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include "cuda.h"
#include "SnippetRender.h"
#include "cudaGL.h"
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif
#else
#include "SnippetRender.h"
#endif

#include "SnippetFontRenderer.h"
#include "SnippetCamera.h"
#include "foundation/PxArray.h"
#include "foundation/PxMathUtils.h"
#include <vector>

#define MAX_NUM_ACTOR_SHAPES	128
#define	INITIAL_SCREEN_WIDTH	768
#define	INITIAL_SCREEN_HEIGHT	768

using namespace physx;

static GLFontRenderer	gTexter;

static float gCylinderData[]={
	1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
	0.866025f,0.500000f,1.0f,0.866025f,0.500000f,1.0f,0.866025f,0.500000f,0.0f,0.866025f,0.500000f,0.0f,
	0.500000f,0.866025f,1.0f,0.500000f,0.866025f,1.0f,0.500000f,0.866025f,0.0f,0.500000f,0.866025f,0.0f,
	-0.0f,1.0f,1.0f,-0.0f,1.0f,1.0f,-0.0f,1.0f,0.0f,-0.0f,1.0f,0.0f,
	-0.500000f,0.866025f,1.0f,-0.500000f,0.866025f,1.0f,-0.500000f,0.866025f,0.0f,-0.500000f,0.866025f,0.0f,
	-0.866025f,0.500000f,1.0f,-0.866025f,0.500000f,1.0f,-0.866025f,0.500000f,0.0f,-0.866025f,0.500000f,0.0f,
	-1.0f,-0.0f,1.0f,-1.0f,-0.0f,1.0f,-1.0f,-0.0f,0.0f,-1.0f,-0.0f,0.0f,
	-0.866025f,-0.500000f,1.0f,-0.866025f,-0.500000f,1.0f,-0.866025f,-0.500000f,0.0f,-0.866025f,-0.500000f,0.0f,
	-0.500000f,-0.866025f,1.0f,-0.500000f,-0.866025f,1.0f,-0.500000f,-0.866025f,0.0f,-0.500000f,-0.866025f,0.0f,
	0.0f,-1.0f,1.0f,0.0f,-1.0f,1.0f,0.0f,-1.0f,0.0f,0.0f,-1.0f,0.0f,
	0.500000f,-0.866025f,1.0f,0.500000f,-0.866025f,1.0f,0.500000f,-0.866025f,0.0f,0.500000f,-0.866025f,0.0f,
	0.866026f,-0.500000f,1.0f,0.866026f,-0.500000f,1.0f,0.866026f,-0.500000f,0.0f,0.866026f,-0.500000f,0.0f,
	1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f
};

static std::vector<PxVec3>* gVertexBuffer = NULL;
static int gLastTime = 0;
static int gFrameCounter = 0;
static char gTitle[256];
static bool gWireFrame = false;

static PX_FORCE_INLINE void prepareVertexBuffer()
{
	if(!gVertexBuffer)
		gVertexBuffer = new std::vector<PxVec3>;
	gVertexBuffer->clear();
}

static PX_FORCE_INLINE void pushVertex(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n)
{
	PX_ASSERT(gVertexBuffer);
	gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v0);
	gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v1);
	gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v2);
}

static PX_FORCE_INLINE void pushVertex(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n0, const PxVec3& n1, const PxVec3& n2, float normalSign = 1)
{
	PX_ASSERT(gVertexBuffer);
	gVertexBuffer->push_back(normalSign * n0); gVertexBuffer->push_back(v0);
	gVertexBuffer->push_back(normalSign * n1); gVertexBuffer->push_back(v1);
	gVertexBuffer->push_back(normalSign * n2); gVertexBuffer->push_back(v2);
}

static PX_FORCE_INLINE const PxVec3* getVertexBuffer()
{
	PX_ASSERT(gVertexBuffer);
	return &(*gVertexBuffer)[0];
}

static void releaseVertexBuffer()
{
	if(gVertexBuffer)
	{
		delete gVertexBuffer;
		gVertexBuffer = NULL;
	}
}

static void renderDeformableVolumeGeometry(const PxTetrahedronMesh& mesh, const PxVec4* deformedPositionsInvMass)
{
	const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

	//Get the deformed vertices			
	//const PxVec3* vertices = mesh.getVertices();
	const PxU32 tetCount = mesh.getNbTetrahedrons();
	const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
	const void* indexBuffer = mesh.getTetrahedrons();

	prepareVertexBuffer();

	const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
	const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
	PxU32 numTotalTriangles = 0;
	PX_UNUSED(numTotalTriangles);

	for (PxU32 i = 0; i < tetCount; ++i)
	{
		PxU32 vref[4];
		if (has16BitIndices)
		{
			vref[0] = *shortIndices++;
			vref[1] = *shortIndices++;
			vref[2] = *shortIndices++;
			vref[3] = *shortIndices++;
		}
		else
		{
			vref[0] = *intIndices++;
			vref[1] = *intIndices++;
			vref[2] = *intIndices++;
			vref[3] = *intIndices++;
		}

		for (PxU32 j = 0; j < 4; ++j)
		{
			const PxVec4& v0 = deformedPositionsInvMass[vref[tetFaces[j][0]]];
			const PxVec4& v1 = deformedPositionsInvMass[vref[tetFaces[j][1]]];
			const PxVec4& v2 = deformedPositionsInvMass[vref[tetFaces[j][2]]];

			PxVec3 fnormal = (v1.getXYZ() - v0.getXYZ()).cross(v2.getXYZ() - v0.getXYZ());
			fnormal.normalize();

			pushVertex(v0.getXYZ(), v1.getXYZ(), v2.getXYZ(), fnormal);
			numTotalTriangles++;
		}
	}
	glPushMatrix();
	glScalef(1.0f, 1.0f, 1.0f);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	const PxVec3* vertexBuffer = getVertexBuffer();
	glNormalPointer(GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer);
	glVertexPointer(3, GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer + 1);
	glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glPopMatrix();
}

static void renderGeometry(const PxGeometry& geom)
{
	if (gWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	switch(geom.getType())
	{
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
			glScalef(boxGeom.halfExtents.x, boxGeom.halfExtents.y, boxGeom.halfExtents.z);
			glutSolidCube(2);
		}
		break;

		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
			glutSolidSphere(GLdouble(sphereGeom.radius), 10, 10);
		}
		break;

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);
			const PxF32 radius = capsuleGeom.radius;
			const PxF32 halfHeight = capsuleGeom.halfHeight;

			//Sphere
			glPushMatrix();
			glTranslatef(halfHeight, 0.0f, 0.0f);
			glScalef(radius,radius,radius);
			glutSolidSphere(1, 10, 10);		
			glPopMatrix();

			//Sphere
			glPushMatrix();
			glTranslatef(-halfHeight, 0.0f, 0.0f);
			glScalef(radius,radius,radius);
			glutSolidSphere(1, 10, 10);		
			glPopMatrix();

			//Cylinder
			glPushMatrix();
			glTranslatef(-halfHeight, 0.0f, 0.0f);
			glScalef(2.0f*halfHeight, radius,radius);
			glRotatef(90.0f,0.0f,1.0f,0.0f);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), gCylinderData);
			glNormalPointer(GL_FLOAT, 2*3*sizeof(float), gCylinderData+3);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 13*2);
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glPopMatrix();
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

			//Compute triangles for each polygon.
			const PxVec3& scale = convexGeom.scale.scale;
			PxConvexMesh* mesh = convexGeom.convexMesh;
			const PxU32 nbPolys = mesh->getNbPolygons();
			const PxU8* polygons = mesh->getIndexBuffer();
			const PxVec3* verts = mesh->getVertices();
			PxU32 nbVerts = mesh->getNbVertices();
			PX_UNUSED(nbVerts);

			prepareVertexBuffer();

			PxU32 numTotalTriangles = 0;
			for(PxU32 i = 0; i < nbPolys; i++)
			{
				PxHullPolygon data;
				mesh->getPolygonData(i, data);

				const PxU32 nbTris = PxU32(data.mNbVerts - 2);
				const PxU8 vref0 = polygons[data.mIndexBase + 0];
				PX_ASSERT(vref0 < nbVerts);
				for(PxU32 j=0;j<nbTris;j++)
				{
					const PxU32 vref1 = polygons[data.mIndexBase + 0 + j + 1];
					const PxU32 vref2 = polygons[data.mIndexBase + 0 + j + 2];

					//generate face normal:
					PxVec3 e0 = verts[vref1] - verts[vref0];
					PxVec3 e1 = verts[vref2] - verts[vref0];

					PX_ASSERT(vref1 < nbVerts);
					PX_ASSERT(vref2 < nbVerts);

					PxVec3 fnormal = e0.cross(e1);
					fnormal.normalize();

					pushVertex(verts[vref0], verts[vref1], verts[vref2], fnormal);
					numTotalTriangles++;
				}
			}
			glPushMatrix();
			glScalef(scale.x, scale.y, scale.z);
			glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_VERTEX_ARRAY);
			const PxVec3* vertexBuffer = getVertexBuffer();
			glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
			glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);
			glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
			glPopMatrix();
		}
		break;

		case PxGeometryType::eTRIANGLEMESH:
		{
			const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

			const PxTriangleMesh& mesh = *triGeom.triangleMesh;
			const PxVec3 scale = triGeom.scale.scale;

			const PxU32 triangleCount = mesh.getNbTriangles();
			const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
			const void* indexBuffer = mesh.getTriangles();

			const PxVec3* vertices = mesh.getVertices();

			prepareVertexBuffer();

			const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
			const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
			PxU32 numTotalTriangles = 0;
			for(PxU32 i=0; i < triangleCount; ++i)
			{
				PxU32 vref0, vref1, vref2;
				if(has16BitIndices)
				{
					vref0 = *shortIndices++;
					vref1 = *shortIndices++;
					vref2 = *shortIndices++;
				}
				else
				{
					vref0 = *intIndices++;
					vref1 = *intIndices++;
					vref2 = *intIndices++;
				}

				const PxVec3& v0 = vertices[vref0];
				const PxVec3& v1 = vertices[vref1];
				const PxVec3& v2 = vertices[vref2];

				PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
				fnormal.normalize();

				pushVertex(v0, v1, v2, fnormal);
				numTotalTriangles++;
			}
			glPushMatrix();
			glScalef(scale.x, scale.y, scale.z);
			glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_VERTEX_ARRAY);
			const PxVec3* vertexBuffer = getVertexBuffer();
			glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
			glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);
			glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glPopMatrix();
		}
		break;

		case PxGeometryType::eTETRAHEDRONMESH: 
		{
			const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

			const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(geom);

			const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;

			//Get the deformed vertices			
			const PxVec3* vertices = mesh.getVertices();
			const PxU32 tetCount = mesh.getNbTetrahedrons();
			const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
			const void* indexBuffer = mesh.getTetrahedrons();

			prepareVertexBuffer();

			const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
			const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
			PxU32 numTotalTriangles = 0;
			for (PxU32 i = 0; i < tetCount; ++i)
			{
				PxU32 vref[4];
				if (has16BitIndices)
				{
					vref[0] = *shortIndices++;
					vref[1] = *shortIndices++;
					vref[2] = *shortIndices++;
					vref[3] = *shortIndices++;
				}
				else
				{
					vref[0] = *intIndices++;
					vref[1] = *intIndices++;
					vref[2] = *intIndices++;
					vref[3] = *intIndices++;
				}

				for (PxU32 j = 0; j < 4; ++j)
				{
					const PxVec3& v0 = vertices[vref[tetFaces[j][0]]];
					const PxVec3& v1 = vertices[vref[tetFaces[j][1]]];
					const PxVec3& v2 = vertices[vref[tetFaces[j][2]]];
					
					PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
					fnormal.normalize();

					pushVertex(v0, v1, v2, fnormal);
					numTotalTriangles++;
				}
			}
			glPushMatrix();
			glScalef(1.0f, 1.0f, 1.0f);
			glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_VERTEX_ARRAY);
			const PxVec3* vertexBuffer = getVertexBuffer();
			glNormalPointer(GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer);
			glVertexPointer(3, GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer + 1);
			glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glPopMatrix();
		}
		break;

		default:
			break;
	}

	if (gWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static Snippets::Camera* gCamera = NULL;
static KeyboardCallback gKbCb = NULL;

static PxU32 gScreenWidth	= 0;
static PxU32 gScreenHeight	= 0;

static void defaultReshapeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
	gTexter.setScreenResolution(width, height);
	gScreenWidth = width;
	gScreenHeight = height;
}

static void defaultIdleCallback()
{
	glutPostRedisplay();
}

static bool gMoveCamera = false;

static void defaultMotionCallback(int x, int y)
{
	if(gCamera && gMoveCamera)
		gCamera->handleMotion(x, y);
}

static void defaultMouseCallback(int button, int state, int x, int y)
{
	if(button==0)
		gMoveCamera = state==0;

	if(gCamera)
		gCamera->handleMouse(button, state, x, y);
}

static void defaultKeyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)
		glutLeaveMainLoop();

	if (key == 110) //n
		gWireFrame = !gWireFrame;

	const bool status = gCamera ? gCamera->handleKey(key, x, y) : false;
	if(!status && gKbCb)
		gKbCb(key, gCamera ? gCamera->getTransform() : PxTransform(PxIdentity));
}

static void defaultSpecialCallback(int key, int x, int y)
{
	switch(key)
	{
		case GLUT_KEY_UP:		key='W'; break;
		case GLUT_KEY_DOWN:		key='S'; break;
		case GLUT_KEY_LEFT:		key='A'; break;
		case GLUT_KEY_RIGHT:	key='D'; break;
	}

	const bool status = gCamera ? gCamera->handleKey((unsigned char)key, x, y) : false;
	if(!status && gKbCb)
		gKbCb((unsigned char)key, gCamera ? gCamera->getTransform() : PxTransform(PxIdentity));
}

static ExitCallback gUserExitCB = NULL;
static void defaultExitCallback()
{
	releaseVertexBuffer();

	if(gUserExitCB)
		(gUserExitCB)();
}

static void setupDefaultWindow(const char* name, RenderCallback rdcb)
{
	char* namestr = new char[strlen(name)+1];
	strcpy(namestr, name);
	int argc = 1;
	char* argv[1] = { namestr };

	glutInit(&argc, argv);
	
	gScreenWidth	= INITIAL_SCREEN_WIDTH;
	gScreenHeight	= INITIAL_SCREEN_HEIGHT;

	gTexter.init();
	gTexter.setScreenResolution(gScreenWidth, gScreenHeight);
	gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);

	glutInitWindowSize(gScreenWidth, gScreenHeight);
	glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);
	int mainHandle = glutCreateWindow(name);
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{		
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}
	glutSetWindow(mainHandle);
	glutReshapeFunc(defaultReshapeCallback);
	glutIdleFunc(defaultIdleCallback);
	glutKeyboardFunc(defaultKeyboardCallback);
	glutSpecialFunc(defaultSpecialCallback);
	glutMouseFunc(defaultMouseCallback);
	glutMotionFunc(defaultMotionCallback);
	if(rdcb)
		glutDisplayFunc(rdcb);
	defaultMotionCallback(0,0);

	delete[] namestr;
}

static void setupDefaultRenderState()
{
	// Setup default render states
//	glClearColor(0.3f, 0.4f, 0.5f, 1.0);
	glClearColor(0.2f, 0.2f, 0.2f, 1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	// Setup lighting
/*	glEnable(GL_LIGHTING);
	PxReal ambientColor[]	= { 0.0f, 0.1f, 0.2f, 0.0f };
	PxReal diffuseColor[]	= { 1.0f, 1.0f, 1.0f, 0.0f };		
	PxReal specularColor[]	= { 0.0f, 0.0f, 0.0f, 0.0f };		
	PxReal position[]		= { 100.0f, 100.0f, 400.0f, 1.0f };		
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_LIGHT0);*/
}

static void InitLighting()
{
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);

	const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, zero);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, zero);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, zero);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, zero);

	glEnable(GL_LIGHTING);
	PxVec3 Dir(-1.0f, 1.0f, 0.5f);
//	PxVec3 Dir(0.0f, 1.0f, 0.0f);
	Dir.normalize();

	const float AmbientValue = 0.3f;
	const float ambientColor0[]		= { AmbientValue, AmbientValue, AmbientValue, 0.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor0);

	const float specularColor0[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor0);

	const float diffuseColor0[]	= { 1.0f, 1.0f, 1.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor0);

	const float position0[]		= { Dir.x, Dir.y, Dir.z, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, position0);

	glEnable(GL_LIGHT0);

//	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
//	glColor4f(0.0f, 0.0f, 0.0f, 0.0f);

/*		glEnable(GL_LIGHTING);
		PxReal ambientColor[]	= { 0.0f, 0.1f, 0.2f, 0.0f };
		PxReal diffuseColor[]	= { 1.0f, 1.0f, 1.0f, 0.0f };		
		PxReal specularColor[]	= { 0.0f, 0.0f, 0.0f, 0.0f };		
		PxReal position[]		= { 100.0f, 100.0f, 400.0f, 1.0f };		
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
		glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
		glLightfv(GL_LIGHT0, GL_POSITION, position);
		glEnable(GL_LIGHT0);*/

	if(0)
	{
		glEnable(GL_FOG);
		glFogi(GL_FOG_MODE,GL_LINEAR); 
		//glFogi(GL_FOG_MODE,GL_EXP); 
		//glFogi(GL_FOG_MODE,GL_EXP2); 
		glFogf(GL_FOG_START, 0.0f);
		glFogf(GL_FOG_END, 100.0f);
		glFogf(GL_FOG_DENSITY, 0.005f);
//		glClearColor(0.2f, 0.2f, 0.2f, 1.0);
//		const PxVec3 FogColor(0.2f, 0.2f, 0.2f);
		const PxVec3 FogColor(0.3f, 0.4f, 0.5f);
//		const PxVec3 FogColor(1.0f);
		glFogfv(GL_FOG_COLOR, &FogColor.x);
	}
}

namespace Snippets
{
void initFPS()
{
	gLastTime = glutGet(GLUT_ELAPSED_TIME);
}

void showFPS(int updateIntervalMS, const char* info)
{
	++gFrameCounter;
	int currentTime = glutGet(GLUT_ELAPSED_TIME);
	if (currentTime - gLastTime > updateIntervalMS)
	{
		if (info)
			sprintf(gTitle, " FPS : %4.0f%s", gFrameCounter * 1000.0 / (currentTime - gLastTime), info);
		else
			sprintf(gTitle, " FPS : %4.0f", gFrameCounter * 1000.0 / (currentTime - gLastTime));
		glutSetWindowTitle(gTitle);
		gLastTime = currentTime;
		gFrameCounter = 0;
	}
}

PxU32 getScreenWidth()
{
	return gScreenWidth;
}

PxU32 getScreenHeight()
{
	return gScreenHeight;
}

void enableVSync(bool vsync)
{
#if PX_WIN32 || PX_WIN64
	typedef void (APIENTRY * PFNWGLSWAPINTERVALPROC) (GLenum interval);
	PFNWGLSWAPINTERVALPROC wglSwapIntervalEXT = (PFNWGLSWAPINTERVALPROC)wglGetProcAddress("wglSwapIntervalEXT");
	wglSwapIntervalEXT(vsync);
#else
	PX_UNUSED(vsync);
#endif
}

void setupDefault(const char* name, Camera* camera, KeyboardCallback kbcb, RenderCallback rdcb, ExitCallback excb)
{
	gCamera = camera;
	gKbCb = kbcb;
	setupDefaultWindow(name, rdcb);
	setupDefaultRenderState();
	enableVSync(true);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);


	gUserExitCB = excb;
	atexit(defaultExitCallback);
}

Camera* getCamera()
{
	return gCamera;
}

static float	gNearClip = 0.0f;
static float	gFarClip = 0.0f;
static float	gFOV = 60.0f;

static float	gTextScale = 0.0f;
static float	gTextY = 0.0f;

PxVec3 computeWorldRayF(float xs, float ys, const PxVec3& camDir)
{
	const float Width = float(gScreenWidth);
	const float Height = float(gScreenHeight);

	// Recenter coordinates in camera space ([-1, 1])
	const float u = ((xs - Width*0.5f)/Width)*2.0f;
	const float v = -((ys - Height*0.5f)/Height)*2.0f;

	// Adjust coordinates according to camera aspect ratio
	const float HTan = tanf(0.25f * fabsf(PxDegToRad(gFOV * 2.0f)));
	const float VTan = HTan*(Width/Height);

	// Ray in camera space
	const PxVec3 CamRay(VTan*u, HTan*v, 1.0f);

	// Compute ray in world space
	PxVec3 Right, Up;
	PxComputeBasisVectors(camDir, Right, Up);

	const PxMat33 invView(-Right, Up, camDir);

	return invView.transform(CamRay).getNormalized();
}

void startRender(const Camera* camera, float clipNear, float clipFar, float fov, bool setupLighting)
{
	const PxVec3 cameraEye = camera->getEye();
	const PxVec3 cameraDir = camera->getDir();

	gNearClip = clipNear;
	gFarClip = clipFar;
	gFOV = fov;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(GLdouble(fov), GLdouble(glutGet(GLUT_WINDOW_WIDTH)) / GLdouble(glutGet(GLUT_WINDOW_HEIGHT)), GLdouble(clipNear), GLdouble(clipFar));

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(GLdouble(cameraEye.x), GLdouble(cameraEye.y), GLdouble(cameraEye.z), GLdouble(cameraEye.x + cameraDir.x), GLdouble(cameraEye.y + cameraDir.y), GLdouble(cameraEye.z + cameraDir.z), 0.0, 1.0, 0.0);

	glColor4f(0.4f, 0.4f, 0.4f, 1.0f);

	if(setupLighting)
		InitLighting();

	gTextScale = 0.0175f * float(INITIAL_SCREEN_HEIGHT) / float(gScreenHeight);
	gTextY = 1.0f - gTextScale;
	gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void finishRender()
{
	glutSwapBuffers();
}

void print(const char* text)
{
	gTexter.print(0.0f, gTextY, gTextScale, text);
	gTextY -= gTextScale;
}

const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
const PxReal shadowMat[] = { 1,0,0,0, -shadowDir.x / shadowDir.y,0,-shadowDir.z / shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

void renderDeformableVolume(PxDeformableVolume* deformableVolume, const PxVec4* deformedPositionsInvMass, bool shadows, const PxVec3& color)
{
	PxShape* shape = deformableVolume->getShape();

	const PxMat44 shapePose(PxIdentity); // (PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
	const PxGeometry& geom = shape->getGeometry();

	const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(geom);

	const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;

	glPushMatrix();
	glMultMatrixf(&shapePose.column0.x);
	glColor4f(color.x, color.y, color.z, 1.0f);
	renderDeformableVolumeGeometry(mesh, deformedPositionsInvMass);
	glPopMatrix();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	if (shadows)
	{
		glPushMatrix();
		glMultMatrixf(shadowMat);
		glMultMatrixf(&shapePose.column0.x);
		glDisable(GL_LIGHTING);
		//glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
		glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
		renderDeformableVolumeGeometry(mesh, deformedPositionsInvMass);
		glEnable(GL_LIGHTING);
		glPopMatrix();
	}
}


void renderActors(PxRigidActor** actors, const PxU32 numActors, bool shadows, const PxVec3& color, TriggerRender* cb,
	bool changeColorForSleepingActors, bool wireframePass)
{
	PxShape* shapes[MAX_NUM_ACTOR_SHAPES];
	for(PxU32 i=0;i<numActors;i++)
	{
		const PxU32 nbShapes = actors[i]->getNbShapes();
		PX_ASSERT(nbShapes <= MAX_NUM_ACTOR_SHAPES);
		actors[i]->getShapes(shapes, nbShapes);
		bool sleeping;
		if (changeColorForSleepingActors)
			sleeping = actors[i]->is<PxRigidDynamic>() ? actors[i]->is<PxRigidDynamic>()->isSleeping() : false;
		else
			sleeping = false;

		for(PxU32 j=0;j<nbShapes;j++)
		{
			const PxMat44 shapePose(PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
			const PxGeometry& geom = shapes[j]->getGeometry();

			const bool isTrigger = cb ? cb->isTrigger(shapes[j]) : shapes[j]->getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
			if(isTrigger)
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			
			// render object
			glPushMatrix();						
			glMultMatrixf(&shapePose.column0.x);
			if(sleeping)
			{
				const PxVec3 darkColor = color * 0.25f;
				glColor4f(darkColor.x, darkColor.y, darkColor.z, 1.0f);
			}
			else
				glColor4f(color.x, color.y, color.z, 1.0f);
			renderGeometry(geom);
			glPopMatrix();

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			if(shadows)
			{
				glPushMatrix();						
				glMultMatrixf(shadowMat);
				glMultMatrixf(&shapePose.column0.x);
				glDisable(GL_LIGHTING);
				//glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
				glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
				renderGeometry(geom);
				glEnable(GL_LIGHTING);
				glPopMatrix();
			}
		}
	}

	if(wireframePass)
	{
		const GLdouble aspect = GLdouble(glutGet(GLUT_WINDOW_WIDTH)) / GLdouble(glutGet(GLUT_WINDOW_HEIGHT));
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip*1.005f), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);

		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
		for(PxU32 i=0;i<numActors;i++)
		{
			const PxU32 nbShapes = actors[i]->getNbShapes();
			PX_ASSERT(nbShapes <= MAX_NUM_ACTOR_SHAPES);
			actors[i]->getShapes(shapes, nbShapes);

			for(PxU32 j=0;j<nbShapes;j++)
			{
				const PxMat44 shapePose(PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
				glPushMatrix();						
				glMultMatrixf(&shapePose.column0.x);
				renderGeometry(shapes[j]->getGeometry());
				glPopMatrix();
			}
		}
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_LIGHTING);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);
	}
}

/*static const PxU32 gGeomSizes[] = {
	sizeof(PxSphereGeometry),
	sizeof(PxPlaneGeometry),
	sizeof(PxCapsuleGeometry),
	sizeof(PxBoxGeometry),
	sizeof(PxConvexMeshGeometry),
	sizeof(PxTriangleMeshGeometry),
	sizeof(PxHeightFieldGeometry),
};

void renderGeoms(const PxU32 nbGeoms, const PxGeometry* geoms, const PxTransform* poses, bool shadows, const PxVec3& color)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
	const PxReal shadowMat[]={ 1,0,0,0, -shadowDir.x/shadowDir.y,0,-shadowDir.z/shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

	const PxU8* stream = reinterpret_cast<const PxU8*>(geoms);
	for(PxU32 j=0;j<nbGeoms;j++)
	{
		const PxMat44 shapePose(poses[j]);

		const PxGeometry& geom = *reinterpret_cast<const PxGeometry*>(stream);
		stream += gGeomSizes[geom.getType()];

		// render object
		glPushMatrix();						
		glMultMatrixf(&shapePose.column0.x);
		glColor4f(color.x, color.y, color.z, 1.0f);
		renderGeometry(geom);
		glPopMatrix();

		if(shadows)
		{
			glPushMatrix();						
			glMultMatrixf(shadowMat);
			glMultMatrixf(&shapePose.column0.x);
			glDisable(GL_LIGHTING);
			glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
			renderGeometry(geom);
			glEnable(GL_LIGHTING);
			glPopMatrix();
		}
	}
}*/

void renderGeoms(const PxU32 nbGeoms, const PxGeometryHolder* geoms, const PxTransform* poses, bool shadows, const PxVec3& color)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	//const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
	//const PxReal shadowMat[]={ 1,0,0,0, -shadowDir.x/shadowDir.y,0,-shadowDir.z/shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

	for(PxU32 j=0;j<nbGeoms;j++)
	{
		const PxMat44 shapePose(poses[j]);

		const PxGeometry& geom = geoms[j].any();

		// render object
		glPushMatrix();						
		glMultMatrixf(&shapePose.column0.x);
		glColor4f(color.x, color.y, color.z, 1.0f);
		renderGeometry(geom);
		glPopMatrix();

		if(shadows)
		{
			glPushMatrix();						
			glMultMatrixf(shadowMat);
			glMultMatrixf(&shapePose.column0.x);
			glDisable(GL_LIGHTING);
			//glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
			glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
			renderGeometry(geom);
			glEnable(GL_LIGHTING);
			glPopMatrix();
		}
	}

	if(1)
	{
		const GLdouble aspect = GLdouble(glutGet(GLUT_WINDOW_WIDTH)) / GLdouble(glutGet(GLUT_WINDOW_HEIGHT));
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip*1.005f), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);

		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glColor4f(0.0f, 0.0f, 0.0f, 1.0f);


		for(PxU32 j=0;j<nbGeoms;j++)
		{
			const PxMat44 shapePose(poses[j]);

			const PxGeometry& geom = geoms[j].any();

			// render object
			glPushMatrix();						
			glMultMatrixf(&shapePose.column0.x);
			renderGeometry(geom);
			glPopMatrix();
		}


		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_LIGHTING);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);
	}
}

PX_FORCE_INLINE PxVec3 getVec3(const physx::PxU8* data, const PxU32 index, const PxU32 sStrideInBytes)
{
	return *reinterpret_cast<const PxVec3*>(data + index * sStrideInBytes);
}

void renderMesh(physx::PxU32 /*nbVerts*/, const physx::PxU8* verts, const PxU32 vertsStrideInBytes, physx::PxU32 nbTris, const void* indices, bool has16bitIndices, const physx::PxVec3& color,
	const physx::PxU8* normals, const PxU32 normalsStrideInBytes, bool flipFaceOrientation, bool enableBackFaceCulling = true)
{
	if (nbTris == 0)
		return;

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	const PxMat44 idt(PxIdentity);
	glPushMatrix();						
	glMultMatrixf(&idt.column0.x);
	glColor4f(color.x, color.y, color.z, 1.0f);
	if (!enableBackFaceCulling)
		glDisable(GL_CULL_FACE);
	{
		prepareVertexBuffer();

		PxU32 numTotalTriangles = 0;
		PX_UNUSED(numTotalTriangles);

		for(PxU32 i=0; i <nbTris; ++i)
		{
			PxU32 vref0, vref1, vref2;
			if (has16bitIndices)
			{
				vref0 = ((const PxU16*)indices)[i * 3 + 0];
				vref1 = ((const PxU16*)indices)[i * 3 + 1];
				vref2 = ((const PxU16*)indices)[i * 3 + 2];
			}
			else
			{
				vref0 = ((const PxU32*)indices)[i * 3 + 0];
				vref1 = ((const PxU32*)indices)[i * 3 + 1];
				vref2 = ((const PxU32*)indices)[i * 3 + 2];
			}

			const PxVec3& v0 = getVec3(verts, vref0, vertsStrideInBytes);
			const PxVec3& v1 = flipFaceOrientation ? getVec3(verts, vref2, vertsStrideInBytes) : getVec3(verts, vref1, vertsStrideInBytes);
			const PxVec3& v2 = flipFaceOrientation ? getVec3(verts, vref1, vertsStrideInBytes) : getVec3(verts, vref2, vertsStrideInBytes);
			
			if (normals)
			{
				const PxVec3& n0 = getVec3(normals, vref0, normalsStrideInBytes);
				const PxVec3& n1 = flipFaceOrientation ? getVec3(normals, vref2, normalsStrideInBytes) : getVec3(normals, vref1, normalsStrideInBytes);
				const PxVec3& n2 = flipFaceOrientation ? getVec3(normals, vref1, normalsStrideInBytes) : getVec3(normals, vref2, normalsStrideInBytes);
				pushVertex(v0, v1, v2, n0, n1, n2, flipFaceOrientation ? -1.0f : 1.0f);
			}
			else 
			{
				PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
				fnormal.normalize();
				pushVertex(v0, v1, v2, fnormal);
			}
			numTotalTriangles++;
		}
		glScalef(1.0f, 1.0f, 1.0f);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);
		const PxVec3* vertexBuffer = getVertexBuffer();
		glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
		glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);
		glDrawArrays(GL_TRIANGLES, 0, int(nbTris * 3));
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
	glPopMatrix();
	if (!enableBackFaceCulling)
		glEnable(GL_CULL_FACE);

	if(0)
	{
		const GLdouble aspect = GLdouble(glutGet(GLUT_WINDOW_WIDTH)) / GLdouble(glutGet(GLUT_WINDOW_HEIGHT));
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip*1.005f), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);

		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

			glPushMatrix();						
			glMultMatrixf(&idt.column0.x);

				glEnableClientState(GL_NORMAL_ARRAY);
				glEnableClientState(GL_VERTEX_ARRAY);
				const PxVec3* vertexBuffer = getVertexBuffer();
				glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
				glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);				
				glDrawArrays(GL_TRIANGLES, 0, int(nbTris * 3));
				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_NORMAL_ARRAY);

			glPopMatrix();

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_LIGHTING);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
		glMatrixMode(GL_MODELVIEW);
	}
}

void renderMesh(physx::PxU32 nbVerts, const physx::PxVec3* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec3* normals, bool flipFaceOrientation)
{
	renderMesh(nbVerts, reinterpret_cast<const PxU8*>(verts), sizeof(PxVec3), nbTris, indices, false, color, reinterpret_cast<const PxU8*>(normals), sizeof(PxVec3), flipFaceOrientation);
}

void renderMesh(physx::PxU32 nbVerts, const physx::PxVec4* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec4* normals, bool flipFaceOrientation)
{
	renderMesh(nbVerts, reinterpret_cast<const PxU8*>(verts), sizeof(PxVec4), nbTris, indices, false, color, reinterpret_cast<const PxU8*>(normals), sizeof(PxVec4), flipFaceOrientation);
}

void renderMesh(physx::PxU32 nbVerts, const physx::PxVec4* verts, physx::PxU32 nbTris, const void* indices, bool hasSixteenBitIndices,
	const physx::PxVec3& color, const physx::PxVec4* normals, bool flipFaceOrientation, bool enableBackFaceCulling)
{
	renderMesh(nbVerts, reinterpret_cast<const PxU8*>(verts), sizeof(PxVec4), nbTris, indices, hasSixteenBitIndices,
		color, reinterpret_cast<const PxU8*>(normals), sizeof(PxVec4), flipFaceOrientation, enableBackFaceCulling);
}

void DrawLine(const PxVec3& p0, const PxVec3& p1, const PxVec3& color)
{
	glDisable(GL_LIGHTING);
	glColor4f(color.x, color.y, color.z, 1.0f);
	const PxVec3 Pts[] = { p0, p1 };
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(PxVec3), &Pts[0].x);
	glDrawArrays(GL_LINES, 0, 2);
	glDisableClientState(GL_VERTEX_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

const physx::PxVec3 icosahedronPoints[12] = { PxVec3(0, -0.525731, 0.850651),
PxVec3(0.850651, 0, 0.525731),
PxVec3(0.850651, 0, -0.525731),
PxVec3(-0.850651, 0, -0.525731),
PxVec3(-0.850651, 0, 0.525731),
PxVec3(-0.525731, 0.850651, 0),
PxVec3(0.525731, 0.850651, 0),
PxVec3(0.525731, -0.850651, 0),
PxVec3(-0.525731, -0.850651, 0),
PxVec3(0, -0.525731, -0.850651),
PxVec3(0, 0.525731, -0.850651),
PxVec3(0, 0.525731, 0.850651) };
const PxU32 icosahedronIndices[3 * 20] = { 1  ,2  ,6  ,
1  ,7  ,2  ,
3  ,4  ,5  ,
4  ,3  ,8  ,
6  ,5  ,11 ,
5  ,6  ,10 ,
9  ,10 ,2  ,
10 ,9  ,3  ,
7  ,8  ,9  ,
8  ,7  ,0  ,
11 ,0  ,1  ,
0  ,11 ,4  ,
6  ,2  ,10 ,
1  ,6  ,11 ,
3  ,5  ,10 ,
5  ,4  ,11 ,
2  ,7  ,9  ,
7  ,1  ,0  ,
3  ,9  ,8  ,
4  ,8  ,0 };

void DrawIcosahedraPoints(const PxArray<PxVec3>& pts, const PxVec3& color, float radius)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	const PxMat44 idt(PxIdentity);
	glPushMatrix();
	glMultMatrixf(&idt.column0.x);
	glColor4f(color.x, color.y, color.z, 1.0f);
	PxU32 numTotalTriangles = 0;
	{
		prepareVertexBuffer();

		for (PxU32 i = 0; i < pts.size(); ++i)
		{
			PxVec3 center = pts[i];

			for (PxU32 j = 0; j < 20; ++j) 
			{
				const PxVec3& v0 = icosahedronPoints[icosahedronIndices[3 * j + 0]] * radius + center;
				const PxVec3& v1 = icosahedronPoints[icosahedronIndices[3 * j + 1]] * radius + center;
				const PxVec3& v2 = icosahedronPoints[icosahedronIndices[3 * j + 2]] * radius + center;

				{
					PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
					fnormal.normalize();
					pushVertex(v0, v1, v2, fnormal);
				}
				numTotalTriangles++;
			}
		}
		glScalef(1.0f, 1.0f, 1.0f);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);
		const PxVec3* vertexBuffer = getVertexBuffer();
		glNormalPointer(GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer);
		glVertexPointer(3, GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer + 1);
		glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
	glPopMatrix();
}

void DrawPoints(const PxArray<PxVec3>& pts, const PxVec3& color, float scale)
{
	glPointSize(scale);
	glDisable(GL_LIGHTING);
	glColor4f(color.x, color.y, color.z, 1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(PxVec3), pts.begin());
	glDrawArrays(GL_POINTS, 0, pts.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawPoints(const PxArray<PxVec4>& pts, const PxVec3& color, float scale)
{
	glPointSize(scale);
	glDisable(GL_LIGHTING);
	glColor4f(color.x, color.y, color.z, 1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(PxVec4), pts.begin());
	glDrawArrays(GL_POINTS, 0, pts.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawPoints(const physx::PxArray<physx::PxVec3>& pts, const physx::PxArray<physx::PxVec3>& colors, float scale)
{
	glPointSize(scale);
	glDisable(GL_LIGHTING);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(PxVec3), pts.begin());
	glColorPointer(3, GL_FLOAT, sizeof(PxVec3), colors.begin());
	glDrawArrays(GL_POINTS, 0, pts.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawPoints(const physx::PxArray<physx::PxVec4>& pts, const physx::PxArray<physx::PxVec3>& colors, float scale)
{
	glPointSize(scale);
	glDisable(GL_LIGHTING);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(PxVec4), pts.begin());
	glColorPointer(3, GL_FLOAT, sizeof(PxVec3), colors.begin());
	glDrawArrays(GL_POINTS, 0, pts.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

#if PX_SUPPORT_GPU_PHYSX

namespace
{
	void createVBO(GLuint* vbo, PxU32 size)
	{
		PX_ASSERT(vbo);

		// create buffer object
		glGenBuffers(1, vbo);
		glBindBuffer(GL_ARRAY_BUFFER, *vbo);

		// initialize buffer object
		glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	void deleteVBO(GLuint* vbo)
	{
		glBindBuffer(1, *vbo);
		glDeleteBuffers(1, vbo);
		*vbo = 0;
	}

#if USE_CUDA_INTEROP
	//Returns the pointer to the cuda buffer
	void* mapCudaGraphicsResource(CUgraphicsResource* vbo_resource, size_t& numBytes, CUstream stream = 0)
	{
		CUresult result0 = cuGraphicsMapResources(1, vbo_resource, stream);
		PX_UNUSED(result0);
		void* dptr;
		CUresult result1 = cuGraphicsResourceGetMappedPointer((CUdeviceptr*)&dptr, &numBytes, *vbo_resource);
		PX_UNUSED(result1);
		return dptr;
	}

	void unmapCudaGraphicsResource(CUgraphicsResource* vbo_resource, CUstream stream = 0)
	{
		CUresult result2 = cuGraphicsUnmapResources(1, vbo_resource, stream);
		PX_UNUSED(result2);
	}
#endif

} // namespace

SharedGLBuffer::SharedGLBuffer() : vbo_res(NULL), devicePointer(NULL), vbo(0), size(0)
{
}

void SharedGLBuffer::initialize(PxCudaContextManager* contextManager)
{
	cudaContextManager = contextManager;
}

void SharedGLBuffer::allocate(PxU32 sizeInBytes)
{
	release();
	createVBO(&vbo, sizeInBytes);
#if USE_CUDA_INTEROP
	physx::PxCudaInteropRegisterFlags flags = physx::PxCudaInteropRegisterFlags();
	cudaContextManager->acquireContext();
	CUresult result = cuGraphicsGLRegisterBuffer(reinterpret_cast<CUgraphicsResource*>(&vbo_res), vbo, flags);
	PX_UNUSED(result);
	cudaContextManager->releaseContext();
#endif
	size = sizeInBytes;
}

void SharedGLBuffer::release()
{
	if (vbo)
	{
		deleteVBO(&vbo);
		vbo = 0;
	}
#if USE_CUDA_INTEROP
	if (vbo_res)
	{
		cudaContextManager->acquireContext();
		CUresult result = cuGraphicsUnregisterResource(reinterpret_cast<CUgraphicsResource>(vbo_res));
		PX_UNUSED(result);
		cudaContextManager->releaseContext();
		vbo_res = NULL;
	}
#endif
}

SharedGLBuffer::~SharedGLBuffer()
{
	//release();
}

void* SharedGLBuffer::map()
{
	if (devicePointer)
		return devicePointer;
	
#if USE_CUDA_INTEROP
	size_t numBytes;
	cudaContextManager->acquireContext();
	devicePointer = mapCudaGraphicsResource(reinterpret_cast<CUgraphicsResource*>(&vbo_res), numBytes, 0);
	cudaContextManager->releaseContext();
#else
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	devicePointer = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
	return devicePointer;
}

void SharedGLBuffer::unmap()
{
	if (!devicePointer)
		return;
#if USE_CUDA_INTEROP
	cudaContextManager->acquireContext();
	unmapCudaGraphicsResource(reinterpret_cast<CUgraphicsResource*>(&vbo_res), 0);
	cudaContextManager->releaseContext();
#else
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
	devicePointer = NULL;
}

#endif // PX_SUPPORT_GPU_PHYSX

void DrawPoints(GLuint vbo, PxU32 numPoints, const PxVec3& color, float scale, PxU32 coordinatesPerPoint, PxU32 stride, size_t offset)
{
	glPointSize(scale);	
	glDisable(GL_LIGHTING);
	glColor4f(color.x, color.y, color.z, 1.0f);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(coordinatesPerPoint, GL_FLOAT, stride, (void*)offset /*offsetof(Vertex, pos)*/);
	glDrawArrays(GL_POINTS, 0, numPoints);
	glDisableClientState(GL_VERTEX_ARRAY);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawLines(GLuint vbo, PxU32 numPoints, const PxVec3& color, float scale, PxU32 coordinatesPerPoint, PxU32 stride, size_t offset)
{
	PX_ASSERT(numPoints % 2 == 0);
	glLineWidth(scale);	
	glDisable(GL_LIGHTING);
	glColor4f(color.x, color.y, color.z, 1.0f);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(coordinatesPerPoint, GL_FLOAT, stride, (void*)offset /*offsetof(Vertex, pos)*/);
	glDrawArrays(GL_LINES, 0, numPoints);
	glDisableClientState(GL_VERTEX_ARRAY);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawMeshIndexed(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const physx::PxVec3& color, physx::PxU32 stride)
{
	/*glPushMatrix();
	glScalef(1.0f, 1.0f, 1.0f);*/

	if(gWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glColor4f(color.x, color.y, color.z, 1.0f);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableClientState(GL_VERTEX_ARRAY);	
	glVertexPointer(3, GL_FLOAT, stride, (void*)0);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, stride, (void*)(3 * sizeof(float)));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

	glDrawElements(GL_TRIANGLES, int(numTriangles * 3), GL_UNSIGNED_INT, (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//glPopMatrix();
	if (gWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void DrawMeshIndexedNoNormals(GLuint vbo, GLuint elementbuffer, GLuint numTriangles, const physx::PxVec3& color, physx::PxU32 stride)
{
	/*glPushMatrix();
	glScalef(1.0f, 1.0f, 1.0f);*/

	glColor4f(color.x, color.y, color.z, 1.0f);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, stride, (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

	glDrawElements(GL_TRIANGLES, int(numTriangles * 3), GL_UNSIGNED_INT, (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//glPopMatrix();
}

void DrawFrame(const PxVec3& pt, float scale)
{
	DrawLine(pt, pt + PxVec3(scale, 0.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
	DrawLine(pt, pt + PxVec3(0.0f, scale, 0.0f), PxVec3(0.0f, 1.0f, 0.0f));
	DrawLine(pt, pt + PxVec3(0.0f, 0.0f, scale), PxVec3(0.0f, 0.0f, 1.0f));
}

void DrawBounds(const physx::PxBounds3& box, const physx::PxVec3& color)
{
	DrawLine(PxVec3(box.minimum.x, box.minimum.y, box.minimum.z), PxVec3(box.maximum.x, box.minimum.y, box.minimum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.minimum.y, box.minimum.z), PxVec3(box.maximum.x, box.maximum.y, box.minimum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.maximum.y, box.minimum.z), PxVec3(box.minimum.x, box.maximum.y, box.minimum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.maximum.y, box.minimum.z), PxVec3(box.minimum.x, box.minimum.y, box.minimum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.minimum.y, box.minimum.z), PxVec3(box.minimum.x, box.minimum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.minimum.y, box.maximum.z), PxVec3(box.maximum.x, box.minimum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.minimum.y, box.maximum.z), PxVec3(box.maximum.x, box.maximum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.maximum.y, box.maximum.z), PxVec3(box.minimum.x, box.maximum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.maximum.y, box.maximum.z), PxVec3(box.minimum.x, box.minimum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.minimum.y, box.maximum.z), PxVec3(box.minimum.x, box.minimum.y, box.minimum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.minimum.y, box.minimum.z), PxVec3(box.maximum.x, box.minimum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.maximum.x, box.maximum.y, box.minimum.z), PxVec3(box.maximum.x, box.maximum.y, box.maximum.z), color);
	DrawLine(PxVec3(box.minimum.x, box.maximum.y, box.minimum.z), PxVec3(box.minimum.x, box.maximum.y, box.maximum.z), color);
}

void DrawBounds(const PxBounds3& box)
{
	const PxVec3 color(1.0f, 1.0f, 0.0f);
	DrawBounds(box, color);
}

GLuint CreateTexture(PxU32 width, PxU32 height, const GLubyte* buffer, bool createMipmaps)
{
	GLuint texId;
	glGenTextures(1, &texId);

	glBindTexture(GL_TEXTURE_2D, texId);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	if(buffer)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
		if(createMipmaps)
		{
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
		}
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	return texId;
}

void UpdateTexture(GLuint texId, physx::PxU32 width, physx::PxU32 height, const GLubyte* buffer, bool createMipmaps)
{
	glBindTexture(GL_TEXTURE_2D, texId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
	if(createMipmaps)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
	}
    glBindTexture( GL_TEXTURE_2D, 0);
}

void ReleaseTexture(GLuint texId)
{
	glDeleteTextures(1, &texId);
}

void DrawRectangle(float x_start, float x_end, float y_start, float y_end, const PxVec3& color_top, const PxVec3& color_bottom, float alpha, PxU32 screen_width, PxU32 screen_height, bool draw_outline, bool texturing)
{
	if(alpha!=1.0f)
	{
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	else
	{
		glDisable(GL_BLEND);
	}
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	if(texturing)
		glEnable(GL_TEXTURE_2D);
	else
		glDisable(GL_TEXTURE_2D);

	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, screen_width, 0, screen_height, -1, 1);
	}
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const float x0 = x_start * float(screen_width);
	const float x1 = x_end * float(screen_width);
	const float y0 = y_start * float(screen_height);
	const float y1 = y_end * float(screen_height);

	const PxVec3 p0(x0,	y0, 0.0f);
	const PxVec3 p1(x0,	y1, 0.0f);
	const PxVec3 p2(x1,	y1, 0.0f);
	const PxVec3 p3(x1,	y0, 0.0f);

//	const float u = (texture_flags & SQT_FLIP_U) ? 1.0f : 0.0f;
//	const float v = (texture_flags & SQT_FLIP_V) ? 1.0f : 0.0f;
	const float u = 0.0f;
	const float v = 1.0f;

	const PxVec3 t0(u, v, 0.0f);
	const PxVec3 t1(u, 1.0f - v, 0.0f);
	const PxVec3 t2(1.0f - u, 1.0f - v, 0.0f);
	const PxVec3 t3(1.0f - u, v, 0.0f);

	glBegin(GL_TRIANGLES);
		glTexCoord2f(t0.x, t0.y);
		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p0.x, p0.y, p0.z);

		glTexCoord2f(t1.x, t1.y);
		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p1.x, p1.y, p1.z);

		glTexCoord2f(t2.x, t2.y);
		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p2.x, p2.y, p2.z);

		glTexCoord2f(t0.x, t0.y);
		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p0.x, p0.y, p0.z);

		glTexCoord2f(t2.x, t2.y);
		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p2.x, p2.y, p2.z);

		glTexCoord2f(t3.x, t3.y);
		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p3.x, p3.y, p3.z);
	glEnd();

	if(draw_outline)
	{
		glDisable(GL_TEXTURE_2D);
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		glBegin(GL_LINES);
			glVertex3f(p0.x, p0.y, p0.z);
			glVertex3f(p1.x, p1.y, p1.z);

			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p2.x, p2.y, p2.z);

			glVertex3f(p2.x, p2.y, p2.z);
			glVertex3f(p3.x, p3.y, p3.z);

			glVertex3f(p3.x, p3.y, p3.z);
			glVertex3f(p0.x, p0.y, p0.z);
		glEnd();
	}

	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glDisable(GL_BLEND);
}

void DisplayTexture(GLuint texId, PxU32 size, PxU32 margin)
{
	const PxU32 screenWidth = gScreenWidth;
	const PxU32 screenHeight = gScreenHeight;

	glBindTexture(GL_TEXTURE_2D, texId);
//	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_ADD);
//	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

//	glDisable(GL_CULL_FACE);
	const PxVec3 color(1.0f, 1.0f, 1.0f);
//	const float Alpha = 0.999f;
//	const float Alpha = 0.5f;
	const float Alpha = 1.0f;
	PxU32 PixelSizeX = size;
	PxU32 PixelSizeY = size;

	if(!size)
	{
		PixelSizeX = gScreenWidth;
		PixelSizeY = gScreenHeight;
		margin = 0;
	}

	const float x0 = float(screenWidth-PixelSizeX-margin)/float(screenWidth);
	const float y0 = float(screenHeight-PixelSizeY-margin)/float(screenHeight);
	const float x1 = float(screenWidth-margin)/float(screenWidth);
	const float y1 = float(screenHeight-margin)/float(screenHeight);

	DrawRectangle(x0, x1, y0, y1, color, color, Alpha, screenWidth, screenHeight, false, true);

	glDisable(GL_TEXTURE_2D);
}

} //namespace Snippets

