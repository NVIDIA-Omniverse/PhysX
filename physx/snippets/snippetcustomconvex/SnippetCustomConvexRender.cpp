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

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern void debugRender();

namespace
{
	Snippets::Camera* sCamera;

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			const PxVec3 dynColor(1.0f, 0.5f, 0.25f);

			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
		}

		debugRender();

		Snippets::finishRender();
	}

	void exitCallback(void)
	{
		delete sCamera;
		cleanupPhysics(true);
	}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(-20.0f, 20.0f, -20.0f), PxVec3(0.6f, -0.4f, 0.6f));

	Snippets::setupDefault("PhysX Snippet CustomConvex", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

struct RenderMesh
{
	std::vector<PxVec3> positions, normals;
};

RenderMesh* createRenderCylinder(float height, float radius, float margin)
{
	struct InternalRenderHelper
	{
		InternalRenderHelper(float height_, float radius_, float margin_)
			:
			height(height_), radius(radius_), margin(margin_)
		{
			mesh = new RenderMesh();
			halfHeight = height * 0.5f;
			sides = (int)ceilf(6.2832f / (2 * acosf((radius - err) / radius)));
			step = 6.2832f / sides;
		}

		float height, radius, margin;
		RenderMesh* mesh;

		std::vector<PxVec3> positions;
		std::vector<PxVec3> normals;

		float halfHeight;

		float err = 0.001f;

		int sides;
		float step;

		void addVertex(int index)
		{
			mesh->positions.push_back(positions[index]);
			mesh->normals.push_back(normals[index]);
		}

		void addTop(const PxVec3& p0, const PxVec3& n0, const PxVec3& p1, const PxVec3& n1, const PxVec3& ax)
		{
			int base = int(positions.size());
			positions.push_back(p0);
			normals.push_back(n0);
			for (int i = 0; i < sides; ++i)
			{
				positions.push_back(PxQuat(i * step, ax).rotate(p1));
				normals.push_back(PxQuat(i * step, ax).rotate(n1));
			}
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base);
				addVertex(base + 1 + i);
				addVertex(base + 1 + (i + 1) % sides);
			}
		}

		void addRing(const PxVec3& p0, const PxVec3& n0, const PxVec3& ax)
		{
			int base = int(positions.size());
			for (int i = 0; i < sides; ++i)
			{
				positions.push_back(PxQuat(i * step, ax).rotate(p0));
				normals.push_back(PxQuat(i * step, ax).rotate(n0));
			}
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base - sides + i);
				addVertex(base + i);
				addVertex(base - sides + (i + 1) % sides);
				addVertex(base - sides + (i + 1) % sides);
				addVertex(base + i);
				addVertex(base + (i + 1) % sides);
			}
		}

		void addBottom(const PxVec3& p0, const PxVec3& n0, const PxVec3& /*ax*/)
		{
			int base = int(positions.size());
			positions.push_back(p0);
			normals.push_back(n0);
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base - sides + i);
				addVertex(base);
				addVertex(base - sides + (i + 1) % sides);
			}
		}

		void run()
		{
			int sides2 = margin > 0 ? (int)ceilf(1.5708f / (2 * acosf((margin - err) / margin))) : 1;
			float step2 = 1.5708f / sides2;

			addTop(PxVec3(halfHeight + margin, 0, 0), PxVec3(1, 0, 0), PxVec3(halfHeight + margin, radius, 0), PxVec3(1, 0, 0), PxVec3(1, 0, 0));

			for (int i = 1; i <= sides2; ++i)
			{
				PxVec3 n = PxQuat(i * step2, PxVec3(0, 0, 1)).rotate(PxVec3(1, 0, 0));
				addRing(PxVec3(halfHeight, radius, 0) + n * margin, n, PxVec3(1, 0, 0));
			}

			for (int i = 0; i <= sides2; ++i)
			{
				PxVec3 n = PxQuat(i * step2, PxVec3(0, 0, 1)).rotate(PxVec3(0, 1, 0));
				addRing(PxVec3(-halfHeight, radius, 0) + n * margin, n, PxVec3(1, 0, 0));
			}

			addBottom(PxVec3(-halfHeight - margin, 0, 0), PxVec3(-1, 0, 0), PxVec3(1, 0, 0));
		}
	};

	InternalRenderHelper renderHelper(height, radius, margin);

	renderHelper.run();

	return renderHelper.mesh;
}

RenderMesh* createRenderCone(float height, float radius, float margin)
{
	struct InternalRenderHelper
	{
		InternalRenderHelper(float height_, float radius_, float margin_)
			:
			height(height_), radius(radius_), margin(margin_)
		{
			mesh = new RenderMesh();

			halfHeight = height * 0.5f;

			sides = (int)ceilf(6.2832f / (2 * acosf(((radius + margin) - err) / (radius + margin))));
			step = 6.2832f / sides;
		}

		float height, radius, margin;
		RenderMesh* mesh;

		std::vector<PxVec3> positions;
		std::vector<PxVec3> normals;

		float halfHeight;

		float err = 0.001f;

		int sides;
		float step;

		void addVertex(int index)
		{
			mesh->positions.push_back(positions[index]);
			mesh->normals.push_back(normals[index]);
		}

		void addTop(const PxVec3& p0, const PxVec3& n0, const PxVec3& p1, const PxVec3& n1, const PxVec3& ax)
		{
			int base = int(positions.size());
			positions.push_back(p0);
			normals.push_back(n0);
			for (int i = 0; i < sides; ++i)
			{
				positions.push_back(PxQuat(i * step, ax).rotate(p1));
				normals.push_back(PxQuat(i * step, ax).rotate(n1));
			}
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base);
				addVertex(base + 1 + i);
				addVertex(base + 1 + (i + 1) % sides);
			}
		}

		void addRing(const PxVec3& p0, const PxVec3& n0, const PxVec3& ax)
		{
			int base = int(positions.size());
			for (int i = 0; i < sides; ++i)
			{
				positions.push_back(PxQuat(i * step, ax).rotate(p0));
				normals.push_back(PxQuat(i * step, ax).rotate(n0));
			}
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base - sides + i);
				addVertex(base + i);
				addVertex(base - sides + (i + 1) % sides);
				addVertex(base - sides + (i + 1) % sides);
				addVertex(base + i);
				addVertex(base + (i + 1) % sides);
			}
		}

		void addBottom(const PxVec3& p0, const PxVec3& n0, const PxVec3& /*ax*/)
		{
			int base = int(positions.size());
			positions.push_back(p0);
			normals.push_back(n0);
			for (int i = 0; i < sides; ++i)
			{
				addVertex(base - sides + i);
				addVertex(base);
				addVertex(base - sides + (i + 1) % sides);
			}
		}

		void run()
		{
			addTop(PxVec3(halfHeight + margin, 0, 0), PxVec3(1, 0, 0), PxVec3(halfHeight + margin, 0, 0), PxVec3(1, 0, 0), PxVec3(1, 0, 0));

			float cosAlph = radius / sqrtf(height * height + radius * radius);
			float alph = acosf(cosAlph);
			int sides2 = margin > 0 ? (int)ceilf(alph / (2 * acosf((margin - err) / margin))) : 1;
			float step2 = alph / sides2;

			for (int i = 1; i <= sides2; ++i)
			{
				PxVec3 n = PxQuat(i * step2, PxVec3(0, 0, 1)).rotate(PxVec3(1, 0, 0));
				addRing(PxVec3(halfHeight, 0, 0) + n * margin, n, PxVec3(1, 0, 0));
			}

			sides2 = margin > 0 ? (int)ceilf((3.1416f - alph) / (2 * acosf((margin - err) / margin))) : 1;
			step2 = (3.1416f - alph) / sides2;

			for (int i = 0; i <= sides2; ++i)
			{
				PxVec3 n = PxQuat(alph + i * step2, PxVec3(0, 0, 1)).rotate(PxVec3(1, 0, 0));
				addRing(PxVec3(-halfHeight, radius, 0) + n * margin, n, PxVec3(1, 0, 0));
			}

			addBottom(PxVec3(-halfHeight - margin, 0, 0), PxVec3(-1, 0, 0), PxVec3(1, 0, 0));
		}
	};

	InternalRenderHelper renderHelper(height, radius, margin);
	renderHelper.run();

	return renderHelper.mesh;
}

void destroyRenderMesh(RenderMesh* mesh)
{
	delete mesh;
}

void renderMesh(const RenderMesh& mesh, const PxTransform& pose, bool sleeping)
{
	const PxVec3 color(1.0f, 0.5f, 0.25f);
	const PxMat44 shapePose(pose);

	glPushMatrix();
	glMultMatrixf(&shapePose.column0.x);
	if (sleeping)
	{
		const PxVec3 darkColor = color * 0.25f;
		glColor4f(darkColor.x, darkColor.y, darkColor.z, 1.0f);
	}
	else
	{
		glColor4f(color.x, color.y, color.z, 1.0f);
	}

	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 3 * sizeof(float), mesh.positions.data());
	glNormalPointer(GL_FLOAT, 3 * sizeof(float), mesh.normals.data());
	glDrawArrays(GL_TRIANGLES, 0, int(mesh.positions.size()));
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glPopMatrix();

	bool shadows = true;

	if (shadows)
	{
		const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
		const PxReal shadowMat[] = { 1,0,0,0, -shadowDir.x / shadowDir.y,0,-shadowDir.z / shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

		glPushMatrix();
		glMultMatrixf(shadowMat);
		glMultMatrixf(&shapePose.column0.x);
		glDisable(GL_LIGHTING);
		//glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
		glColor4f(0.1f, 0.1f, 0.1f, 1.0f);

		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 3 * sizeof(float), mesh.positions.data());
		glNormalPointer(GL_FLOAT, 3 * sizeof(float), mesh.normals.data());
		glDrawArrays(GL_TRIANGLES, 0, int(mesh.positions.size()));
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		glEnable(GL_LIGHTING);
		glPopMatrix();
	}

}

static void PxVertex3f(const PxVec3& v) { ::glVertex3f(v.x, v.y, v.z); };
static void PxScalef(const PxVec3& v) { ::glScalef(v.x, v.y, v.z); };

void renderRaycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxRaycastHit* hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * hit->distance);
		PxVertex3f(hit->position);
		PxVertex3f(hit->position + hit->normal);
		glEnd();
	}
	else
	{
		glColor4f(0.6f, 0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * maxDist);
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void renderSweepBox(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxVec3& halfExtents, const PxSweepHit* hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(0.0f, 0.6f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * hit->distance);
		PxVertex3f(hit->position);
		PxVertex3f(hit->position + hit->normal);
		glEnd();
		PxTransform boxPose(origin + unitDir * hit->distance);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	else
	{
		glColor4f(0.0f, 0.3f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * maxDist);
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void renderOverlapBox(const PxVec3& origin, const PxVec3& halfExtents, bool hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		PxTransform boxPose(origin);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	else
	{
		glColor4f(0.0f, 0.0f, 0.6f, 1.0f);
		PxTransform boxPose(origin);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	glEnable(GL_LIGHTING);
}

#endif
