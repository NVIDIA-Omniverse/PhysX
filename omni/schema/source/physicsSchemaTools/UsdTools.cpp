// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "UsdTools.h"

#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/cone.h"
#include "pxr/usd/usdGeom/plane.h"

// -- physics schema
#include "pxr/usd/usdPhysics/rigidBodyAPI.h"
#include "pxr/usd/usdPhysics/collisionAPI.h"
#include "pxr/usd/usdPhysics/massAPI.h"
#include "pxr/usd/usdPhysics/materialAPI.h"
#include "pxr/usd/usdPhysics/scene.h"

#include <numeric>
#include <ostream>

PXR_NAMESPACE_OPEN_SCOPE

UsdGeomMesh createMesh(const UsdStagePtr & stage,
    const SdfPath& path,
    const std::vector<GfVec3f>& points,
    const std::vector<GfVec3f>& normals,
    const std::vector<int>& indices,
    const std::vector<int>& vertexCounts)
{
    UsdGeomMesh mesh = UsdGeomMesh::Define(stage, path);

    // Fill in VtArrays
	pxr::VtArray<int> vertexCountsVt;
    vertexCountsVt.assign(vertexCounts.begin(), vertexCounts.end());
	pxr::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(indices.begin(), indices.end());
	pxr::VtArray<GfVec3f> pointArrayVt;
    pointArrayVt.assign(points.begin(), points.end());
	pxr::VtArray<GfVec3f> normalsVt;
    normalsVt.assign(normals.begin(), normals.end());

    mesh.CreateFaceVertexCountsAttr().Set(vertexCountsVt);
    mesh.CreateFaceVertexIndicesAttr().Set(vertexIndicesVt);
    mesh.CreatePointsAttr().Set(pointArrayVt);
    mesh.CreateDoubleSidedAttr().Set(true);
    mesh.CreateNormalsAttr().Set(normalsVt);

    return mesh;
}


UsdGeomMesh createMeshSquare(const UsdStagePtr & stage, const SdfPath& path, float halfHeight, float halfWidth)
{
    const std::vector<GfVec3f> points = { { 0.0, -halfHeight, -halfWidth },
                                            { 0.0, halfHeight, -halfWidth },
                                            { 0.0, halfHeight, halfWidth },
                                            { 0.0, -halfHeight, halfWidth } };
    const std::vector<GfVec3f> normals = { { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 } };
    const std::vector<int> indices = { 0, 1, 2, 3 };
    const std::vector<int> vertexCounts(1, 4);

    // Create the mesh
    return createMesh(stage, path, points, normals, indices, vertexCounts);
}

UsdGeomMesh createMeshSquareAxis(const UsdStagePtr & stage, const SdfPath& path, const TfToken& axis, float halfSize)
{
	if (axis == TfToken("X"))
	{
		const std::vector<GfVec3f> points = { { 0.0, -halfSize, -halfSize },
												{ 0.0, halfSize, -halfSize },
												{ 0.0, halfSize, halfSize },
												{ 0.0, -halfSize, halfSize } };
		const std::vector<GfVec3f> normals = { { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 } };
		const std::vector<int> indices = { 0, 1, 2, 3 };
		const std::vector<int> vertexCounts(1, 4);

		// Create the mesh
		return createMesh(stage, path, points, normals, indices, vertexCounts);
	}
	else if (axis == TfToken("Y"))
	{
		const std::vector<GfVec3f> points = { { -halfSize, 0.0, -halfSize },
												{ halfSize, 0.0, -halfSize },
												{ halfSize, 0.0, halfSize },
												{ -halfSize, 0.0 , halfSize } };
		const std::vector<GfVec3f> normals = { { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 } };
		const std::vector<int> indices = { 0, 1, 2, 3 };
		const std::vector<int> vertexCounts(1, 4);

		// Create the mesh
		return createMesh(stage, path, points, normals, indices, vertexCounts);
	}

	const std::vector<GfVec3f> points = { { -halfSize, -halfSize, 0.0 },
											{ halfSize, -halfSize, 0.0 },
											{ halfSize, halfSize, 0.0 },
											{ -halfSize, halfSize, 0.0 } };
	const std::vector<GfVec3f> normals = { { 0, 0, 1 }, { 0, 0, 1 }, { 0, 0, 1 }, { 0, 0, 1 } };
	const std::vector<int> indices = { 0, 1, 2, 3 };
	const std::vector<int> vertexCounts(1, 4);

	// Create the mesh
	return createMesh(stage, path, points, normals, indices, vertexCounts);
}

UsdGeomPlane createPlane(const UsdStagePtr & stage, const SdfPath& path, const pxr::TfToken& axis, float halfHeight, float halfWidth)
{
	UsdGeomPlane mesh = UsdGeomPlane::Define(stage, path);

	mesh.CreateAxisAttr().Set(axis);

    return mesh;
}


UsdGeomMesh createMeshBox(const UsdStagePtr & stage, const SdfPath& path, const pxr::GfVec3f& halfExtent)
{
    std::vector<GfVec3f> points = { { -1.0, -1.0, 1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 },   { 1.0, 1.0, 1.0 },
                                    { -1.0, 1.0, -1.0 }, { 1.0, 1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { 1.0, -1.0, -1.0 },
                                    { -1.0, -1.0, 1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 },   { 1.0, 1.0, 1.0 },
                                    { -1.0, 1.0, -1.0 }, { 1.0, 1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { 1.0, -1.0, -1.0 },
                                    { -1.0, -1.0, 1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 },   { 1.0, 1.0, 1.0 },
                                    { -1.0, 1.0, -1.0 }, { 1.0, 1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { 1.0, -1.0, -1.0 } };

    std::transform(
        points.begin(), points.end(), points.begin(), [halfExtent](GfVec3f& c) { return GfCompMult(c, halfExtent); });

    const std::vector<GfVec3f> normals = { { 0, 0, 1 },  { 0, 0, 1 },  { 0, 0, 1 },  { 0, 0, 1 },  { 0, 0, -1 },
                                            { 0, 0, -1 }, { 0, 0, -1 }, { 0, 0, -1 }, { 0, -1, 0 }, { 0, -1, 0 },
                                            { 0, 1, 0 },  { 0, 1, 0 },  { 0, 1, 0 },  { 0, 1, 0 },  { 0, -1, 0 },
                                            { 0, -1, 0 }, { -1, 0, 0 }, { 1, 0, 0 },  { -1, 0, 0 }, { 1, 0, 0 },
                                            { -1, 0, 0 }, { 1, 0, 0 },  { -1, 0, 0 }, { 1, 0, 0 } };

    const std::vector<int> indices = { 0,  1,  3, 2, 4,  5,  7,  6,  10, 11, 13, 12,
                                        14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20 };

    const std::vector<int> vertexCounts(6, 4);

    // Create the mesh
    return createMesh(stage, path, points, normals, indices, vertexCounts);
}

UsdGeomMesh createMeshSphere(
    const UsdStagePtr & stage, const SdfPath& path, float radius, int latitudeSegments, int longitudeSegments)
{
    std::vector<GfVec3f> points;
    std::vector<GfVec3f> normals;
    std::vector<int> indices;
    std::vector<int> vertexCounts;

    // Bottom cap
    const float thetaCap = (1.0f / (float)latitudeSegments - 0.5f) * (float)M_PI;
    float cosTheta0 = cosf(thetaCap);
    float sinTheta0 = sinf(thetaCap);
    float cosPhi0 = 1.0f; // cos(0)
    float sinPhi0 = 0.0f; // sin(0)
    for (int m = 0; m < longitudeSegments; ++m)
    {
        const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
        const float cosPhi1 = cosf(phi1);
        const float sinPhi1 = sinf(phi1);
        normals.push_back(GfVec3f(-1.0f, 0.0f, 0.0f));
        normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
        normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
        cosPhi0 = cosPhi1;
        sinPhi0 = sinPhi1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments, 3); // Triangles

    // Middle
    for (int n = 1; n < latitudeSegments - 1; ++n)
    {
        const float theta1 = ((n + 1) / (float)latitudeSegments - 0.5f) * (float)M_PI;
        const float cosTheta1 = cosf(theta1);
        const float sinTheta1 = sinf(theta1);
        cosPhi0 = 1.0f; // cos(0)
        sinPhi0 = 0.0f; // sin(0)
        for (int m = 0; m < longitudeSegments; ++m)
        {
            const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
            const float cosPhi1 = cosf(phi1);
            const float sinPhi1 = sinf(phi1);
            normals.push_back(GfVec3f(sinTheta1, cosPhi0 * cosTheta1, sinPhi0 * cosTheta1));
            normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta1, cosPhi1 * cosTheta1, sinPhi1 * cosTheta1));
            cosPhi0 = cosPhi1;
            sinPhi0 = sinPhi1;
        }
        cosTheta0 = cosTheta1;
        sinTheta0 = sinTheta1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments * (latitudeSegments - 2), 4); // Quads

    // Top cap
    cosPhi0 = 1.0f; // cos(0)
    sinPhi0 = 0.0f; // sin(0)
    for (int m = 0; m < longitudeSegments; ++m)
    {
        const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
        const float cosPhi1 = cosf(phi1);
        const float sinPhi1 = sinf(phi1);
        normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
        normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
        normals.push_back(GfVec3f(1.0f, 0.0f, 0.0f));
        cosPhi0 = cosPhi1;
        sinPhi0 = sinPhi1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments, 3); // Triangles

    // For a sphere the points are proportional to the normals
    points.resize(normals.size());
    std::transform(normals.begin(), normals.end(), points.begin(), [radius](GfVec3f& c) { return c * radius; });

    // Consecutive indices
    indices.resize(points.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Create the mesh
    return createMesh(stage, path, points, normals, indices, vertexCounts);
}

UsdGeomMesh createMeshCapsule(const UsdStagePtr & stage,
    const SdfPath& path,
    float radius,
    float halfHeight,
    int latitudeSegments,
    int longitudeSegments)
{
    latitudeSegments &= ~1; // Make sure latitudeSegments is even

    std::vector<GfVec3f> points;
    std::vector<GfVec3f> normals;
    std::vector<int> indices;
    std::vector<int> vertexCounts;

    // Bottom cap
    const float thetaCap = (1.0f / (float)latitudeSegments - 0.5f) * (float)M_PI;
    float cosTheta0 = cosf(thetaCap);
    float sinTheta0 = sinf(thetaCap);
    float cosPhi0 = 1.0f; // cos(0)
    float sinPhi0 = 0.0f; // sin(0)
    for (int m = 0; m < longitudeSegments; ++m)
    {
        const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
        const float cosPhi1 = cosf(phi1);
        const float sinPhi1 = sinf(phi1);
        normals.push_back(GfVec3f(-1.0f, 0.0f, 0.0f));
        normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
        normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
        cosPhi0 = cosPhi1;
        sinPhi0 = sinPhi1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments, 3); // Triangles

    // Middle bottom
    for (int n = 1; n < latitudeSegments / 2; ++n)
    {
        const float theta1 = ((n + 1) / (float)latitudeSegments - 0.5f) * (float)M_PI;
        const float cosTheta1 = cosf(theta1);
        const float sinTheta1 = sinf(theta1);
        cosPhi0 = 1.0f; // cos(0)
        sinPhi0 = 0.0f; // sin(0)
        for (int m = 0; m < longitudeSegments; ++m)
        {
            const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
            const float cosPhi1 = cosf(phi1);
            const float sinPhi1 = sinf(phi1);
            normals.push_back(GfVec3f(sinTheta1, cosPhi0 * cosTheta1, sinPhi0 * cosTheta1));
            normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta1, cosPhi1 * cosTheta1, sinPhi1 * cosTheta1));
            cosPhi0 = cosPhi1;
            sinPhi0 = sinPhi1;
        }
        cosTheta0 = cosTheta1;
        sinTheta0 = sinTheta1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments * (latitudeSegments / 2 - 1), 4); // Quads

    points.resize(normals.size());
    std::transform(normals.begin(), normals.end(), points.begin(), [radius, halfHeight](GfVec3f& c) {
        GfVec3f v = c * radius;
        v[0] -= halfHeight;
        return v;
        });

    // Center
    cosPhi0 = 1.0f; // cos(0)
    sinPhi0 = 0.0f; // sin(0)
    for (int m = 0; m < longitudeSegments; ++m)
    {
        const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
        const float cosPhi1 = cosf(phi1);
        const float sinPhi1 = sinf(phi1);
        normals.push_back(GfVec3f(0.0f, cosPhi0, sinPhi0));
        points.push_back(normals.back() * radius);
        points.back()[0] -= halfHeight;
        normals.push_back(GfVec3f(0.0f, cosPhi1, sinPhi1));
        points.push_back(normals.back() * radius);
        points.back()[0] -= halfHeight;
        normals.push_back(GfVec3f(0.0f, cosPhi1, sinPhi1));
        points.push_back(normals.back() * radius);
        points.back()[0] += halfHeight;
        normals.push_back(GfVec3f(0.0f, cosPhi0, sinPhi0));
        points.push_back(normals.back() * radius);
        points.back()[0] += halfHeight;
        cosPhi0 = cosPhi1;
        sinPhi0 = sinPhi1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments, 4); // Quads

    const size_t lastSize = normals.size();

    // Middle bottom
    for (int n = latitudeSegments / 2; n < latitudeSegments - 1; ++n)
    {
        const float theta1 = ((n + 1) / (float)latitudeSegments - 0.5f) * (float)M_PI;
        const float cosTheta1 = cosf(theta1);
        const float sinTheta1 = sinf(theta1);
        cosPhi0 = 1.0f; // cos(0)
        sinPhi0 = 0.0f; // sin(0)
        for (int m = 0; m < longitudeSegments; ++m)
        {
            const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
            const float cosPhi1 = cosf(phi1);
            const float sinPhi1 = sinf(phi1);
            normals.push_back(GfVec3f(sinTheta1, cosPhi0 * cosTheta1, sinPhi0 * cosTheta1));
            normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
            normals.push_back(GfVec3f(sinTheta1, cosPhi1 * cosTheta1, sinPhi1 * cosTheta1));
            cosPhi0 = cosPhi1;
            sinPhi0 = sinPhi1;
        }
        cosTheta0 = cosTheta1;
        sinTheta0 = sinTheta1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments * (latitudeSegments / 2 - 1), 4); // Quads

    // Top cap
    cosPhi0 = 1.0f; // cos(0)
    sinPhi0 = 0.0f; // sin(0)
    for (int m = 0; m < longitudeSegments; ++m)
    {
        const float phi1 = (float)(m + 1) * (float)M_PI * 2.0f / (float)longitudeSegments;
        const float cosPhi1 = cosf(phi1);
        const float sinPhi1 = sinf(phi1);
        normals.push_back(GfVec3f(sinTheta0, cosPhi0 * cosTheta0, sinPhi0 * cosTheta0));
        normals.push_back(GfVec3f(sinTheta0, cosPhi1 * cosTheta0, sinPhi1 * cosTheta0));
        normals.push_back(GfVec3f(1.0f, 0.0f, 0.0f));
        cosPhi0 = cosPhi1;
        sinPhi0 = sinPhi1;
    }
    vertexCounts.insert(vertexCounts.end(), (size_t)longitudeSegments, 3); // Triangles

    points.resize(normals.size());
    std::transform(normals.begin() + lastSize, normals.end(), points.begin() + lastSize, [radius, halfHeight](GfVec3f& c) {
        GfVec3f v = c * radius;
        v[0] += halfHeight;
        return v;
        });

    // Consecutive indices
    indices.resize(points.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Create the mesh
    return createMesh(stage, path, points, normals, indices, vertexCounts);
}

UsdGeomMesh createMeshCylinder(
    const UsdStagePtr & stage, const SdfPath& path, float radius, float halfLength, uint32_t tesselation)
{
    std::vector<GfVec3f> points(tesselation * 4); // *2 to split vertices on edges
    std::vector<GfVec3f> normals(tesselation * 4); // *2 to split vertices on edges
    std::vector<int> indices(tesselation * 6);
    std::vector<int> vertexCounts(tesselation + 2);

    for (uint32_t i = 0; i < tesselation; ++i)
    {
        const float z = cos(2.0f * i * (float)M_PI / tesselation);
        const float y = sin(2.0f * i * (float)M_PI / tesselation);
        points[i] = points[i + 2 * tesselation] = { -halfLength, radius * y, radius * z };
        points[i + tesselation] = points[i + 3 * tesselation] = { halfLength, radius * y, radius * z };
        normals[i] = normals[i + tesselation] = { 0, y, z };
        normals[i + 2 * tesselation] = { -1, 0, 0 };
        normals[i + 3 * tesselation] = { 1, 0, 0 };
        indices[4 * i] = i;
        indices[4 * i + 1] = i + tesselation;
        indices[4 * i + 2] = (i + 1) % (tesselation)+tesselation;
        indices[4 * i + 3] = (i + 1) % (tesselation);
        indices[i + 4 * tesselation] = i + 2 * tesselation;
        indices[i + 5 * tesselation] = tesselation - i - 1 + 3 * tesselation;
        vertexCounts[i] = 4;
    }
    vertexCounts[tesselation] = vertexCounts[tesselation + 1] = tesselation;

    // Create the mesh
    return createMesh(stage, path, points, normals, indices, vertexCounts);
}

void addPhysicsScene(const UsdStagePtr & stage, const std::string& path)
{
    UsdPhysicsScene::Define(stage, SdfPath(path));
}

void addPhysicsToPrim(const UsdPrim& prim, const float density, const pxr::GfVec3f& linVelocity, const pxr::GfVec3f& angularVelocity)
{
	auto physicsAPI = UsdPhysicsRigidBodyAPI::Apply(prim);	
	physicsAPI.CreateVelocityAttr().Set(linVelocity);
	physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity);
	auto massAPI = UsdPhysicsMassAPI::Apply(prim);
	massAPI.CreateDensityAttr().Set(density);
}

UsdGeomXform addActor(const UsdStagePtr & stage, const std::string& path)
{
    return UsdGeomXform::Define(stage, SdfPath(path));
}

UsdGeomXform addCollisionShape(const UsdStagePtr & stage, const std::string& path)
{
    return UsdGeomXform::Define(stage, SdfPath(path));
}

void addRigidBody(const UsdStagePtr& stage, const std::string& path)
{
    auto prim = stage->GetPrimAtPath(SdfPath(path));
    auto physicsAPI = UsdPhysicsRigidBodyAPI::Apply(prim);
}

// physics schema (internal implementation subject to change)
void addPosition(UsdGeomXformable& xformable, const pxr::GfVec3f& position)
{
    auto component = xformable.AddXformOp(UsdGeomXformOp::Type::TypeTranslate, UsdGeomXformOp::Precision::PrecisionFloat);
    component.Set(position);
}

void addOrientation(UsdGeomXformable& xformable, const pxr::GfQuatf& orientation)
{
    auto component = xformable.AddXformOp(UsdGeomXformOp::Type::TypeOrient, UsdGeomXformOp::Precision::PrecisionFloat);
    component.Set(orientation);
}

void addVelocity(const UsdStagePtr & stage, const std::string& path, const GfVec3f& linearVelocity, const GfVec3f& angularVelocity)
{    
    auto rbAPI = UsdPhysicsRigidBodyAPI::Get(stage, SdfPath(path));
    rbAPI.CreateVelocityAttr().Set(linearVelocity);
    rbAPI.CreateAngularVelocityAttr().Set(angularVelocity);
}

void addDensity(const UsdStagePtr & stage, const std::string& path, float value)
{
    auto rbPrim = stage->GetPrimAtPath(SdfPath(path));
    auto density = UsdPhysicsMassAPI::Apply(rbPrim);
    density.CreateDensityAttr().Set((double)value);
}

void addDisplayColor(UsdGeomGprim& gprim, const GfVec3f& color)
{
	gprim.CreateDisplayColorAttr().Set(pxr::VtArray<GfVec3f>({ color }));    
}

void addBoxCollisionShape(const UsdStagePtr & stage,
    const std::string& path,
    float size,
    const pxr::GfVec3f& position,
    const pxr::GfQuatf& orientation,
    const pxr::GfVec3f& color)
{
    // (Graphics) Box mesh
    std::string primPath = path + "/graphicsBox";
    UsdGeomMesh entityBox = createMeshBox(stage, SdfPath(primPath), GfVec3f(size));
    addDisplayColor(entityBox, color);

    // Transform
    addPosition(entityBox, position);
    addOrientation(entityBox, orientation);

    // (Collision) Box0
    std::string collisionBoxPath = path + "/collisionBox";
    auto collisionBox = UsdGeomCube::Define(stage, SdfPath(collisionBoxPath));
    collisionBox.CreatePurposeAttr().Set(UsdGeomTokens->guide);
    collisionBox.CreateSizeAttr().Set((double)size*2.0f);    
    addPosition(collisionBox, position);
    addOrientation(collisionBox, orientation);

    auto prim = stage->GetPrimAtPath(SdfPath(collisionBoxPath));
    auto CollisionAPI = UsdPhysicsCollisionAPI::Apply(prim);
}

void addRigidBox(const UsdStagePtr & stage,
    const std::string& path,
    const pxr::GfVec3f& size,
    const pxr::GfVec3f& position,
    const pxr::GfQuatf& orientation,
    const pxr::GfVec3f& color,
    float density,
    const pxr::GfVec3f& linVelocity,
    const pxr::GfVec3f& angularVelocity)
{
    // Box
    {
		UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, SdfPath(path));
		UsdPrim cubePrim = stage->GetPrimAtPath(SdfPath(path));
		cubeGeom.CreateSizeAttr().Set(1.0);
		cubeGeom.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
		cubeGeom.AddOrientOp().Set(orientation);
		cubeGeom.AddScaleOp().Set(size);
		addDisplayColor(cubeGeom, color);

		UsdPhysicsCollisionAPI::Apply(cubePrim);

		if (density != 0.0f)
		{
			addPhysicsToPrim(cubePrim, density, linVelocity, angularVelocity);
		}
    }
}

void addRigidSphere(const UsdStagePtr & stage,
    const std::string& path,
    float radius,
    const pxr::GfVec3f& position,
    const pxr::GfQuatf& orientation,
    const pxr::GfVec3f& color,
    float density,
    const pxr::GfVec3f& linVelocity,
    const pxr::GfVec3f& angularVelocity)
{
    // Sphere
    {
		UsdGeomSphere sphereGeom = UsdGeomSphere::Define(stage, SdfPath(path));
		UsdPrim spherePrim = stage->GetPrimAtPath(SdfPath(path));
		sphereGeom.CreateRadiusAttr().Set(double(radius));
		sphereGeom.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
		sphereGeom.AddOrientOp().Set(orientation);
		addDisplayColor(sphereGeom, color);

		UsdPhysicsCollisionAPI::Apply(spherePrim);

		if (density != 0.0f)
		{
			addPhysicsToPrim(spherePrim, density, linVelocity, angularVelocity);
		}
    }
}

void addRigidCapsule(const UsdStagePtr & stage,
    const std::string& path,
    float radius,
    float halfHeight,
	const pxr::TfToken& axis,
    const pxr::GfVec3f& position,
    const pxr::GfQuatf& orientation,
    const pxr::GfVec3f& color,
    float density,
    const pxr::GfVec3f& linVelocity,
    const pxr::GfVec3f& angularVelocity)
{
    // Capsule
    {
		UsdGeomCapsule capsuleGeom = UsdGeomCapsule::Define(stage, SdfPath(path));
		UsdPrim capsulePrim = stage->GetPrimAtPath(SdfPath(path));
		capsuleGeom.CreateRadiusAttr().Set(double(radius));
		capsuleGeom.CreateHeightAttr().Set(double(halfHeight));
		capsuleGeom.CreateAxisAttr().Set(axis);
		capsuleGeom.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
		capsuleGeom.AddOrientOp().Set(orientation);
		addDisplayColor(capsuleGeom, color);

		UsdPhysicsCollisionAPI::Apply(capsulePrim);

		if (density != 0.0f)
		{
			addPhysicsToPrim(capsulePrim, density, linVelocity, angularVelocity);
		}
    }
}

void addRigidCone(const UsdStagePtr & stage,
	const std::string& path,
	float radius,
	float halfHeight,
	const pxr::TfToken& axis,
	const pxr::GfVec3f& position,
	const pxr::GfQuatf& orientation,
	const pxr::GfVec3f& color,
	float density,
	const pxr::GfVec3f& linVelocity,
	const pxr::GfVec3f& angularVelocity)
{
	// Capsule
	{
		UsdGeomCone coneGeom = UsdGeomCone::Define(stage, SdfPath(path));
		UsdPrim conePrim = stage->GetPrimAtPath(SdfPath(path));
		coneGeom.CreateRadiusAttr().Set(double(radius));
		coneGeom.CreateHeightAttr().Set(double(halfHeight));
		coneGeom.CreateAxisAttr().Set(axis);
		coneGeom.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
		coneGeom.AddOrientOp().Set(orientation);
		addDisplayColor(coneGeom, color);

		UsdPhysicsCollisionAPI::Apply(conePrim);

		if (density != 0.0f)
		{
			addPhysicsToPrim(conePrim, density, linVelocity, angularVelocity);
		}
	}
}

void addRigidCylinder(const UsdStagePtr & stage,
    const std::string& path,
    float radius,
    float halfHeight,
	const pxr::TfToken& axis,
    const pxr::GfVec3f& position,
    const pxr::GfQuatf& orientation,
    const pxr::GfVec3f& color,
    float density,
    const pxr::GfVec3f& linVelocity,
    const pxr::GfVec3f& angularVelocity)
{
    // Cylinder
    {
		UsdGeomCylinder cylinderGeom = UsdGeomCylinder::Define(stage, SdfPath(path));
		UsdPrim cylinderPrim = stage->GetPrimAtPath(SdfPath(path));
		cylinderGeom.CreateRadiusAttr().Set(double(radius));
		cylinderGeom.CreateHeightAttr().Set(double(halfHeight));
		cylinderGeom.CreateAxisAttr().Set(axis);
		cylinderGeom.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
		cylinderGeom.AddOrientOp().Set(orientation);
		addDisplayColor(cylinderGeom, color);

		UsdPhysicsCollisionAPI::Apply(cylinderPrim);

		if (density != 0.0f)
		{
			addPhysicsToPrim(cylinderPrim, density, linVelocity, angularVelocity);
		}
    }
}

void addRigidBoxForInstancing(const UsdStagePtr & stage,
    const std::string& path,
    const pxr::GfVec3f& size,
    const pxr::GfVec3f& color,
    float density)
{
    // Box
    {
        // Box mesh
        std::string primPath = path;
        UsdGeomMesh entityBox = createMeshBox(stage, SdfPath(primPath), GfVec3f(size));
        addDisplayColor(entityBox, color);

        // rigid body
        {
            addRigidBody(stage, path);

            // Density component
            addDensity(stage, path, density);
        }

        // (Collision) Box0
        std::string collisionBoxPath = path + "/collisionBox";
        auto component = UsdGeomCube::Define(stage, SdfPath(collisionBoxPath));
        component.CreatePurposeAttr().Set(UsdGeomTokens->guide);
        component.AddScaleOp(UsdGeomXformOp::Precision::PrecisionFloat).Set(GfVec3f(size));

        auto prim = stage->GetPrimAtPath(SdfPath(collisionBoxPath));
        auto CollisionAPI = UsdPhysicsCollisionAPI::Apply(prim);
    }
}


void addGroundPlane(const UsdStagePtr & stage,
    const std::string& planePath,
	const pxr::TfToken& axis,
    float size,
    const pxr::GfVec3f& position,
    const pxr::GfVec3f& color)
{
	UsdGeomXform planeXform = UsdGeomXform::Define(stage, SdfPath(planePath));

	planeXform.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
	planeXform.AddOrientOp().Set(GfQuatf(1.0));
	planeXform.AddScaleOp().Set(GfVec3f(1.0));

	// (Graphics) Plane mesh
	UsdGeomMesh entityPlane = createMeshSquareAxis(stage, SdfPath(planePath + "/geom"), axis, size);
	addDisplayColor(entityPlane, color);

	// (Collision) Plane
	UsdGeomPlane planeGeom = UsdGeomPlane::Define(stage, SdfPath(planePath + "/collisionPlane"));
	planeGeom.CreatePurposeAttr().Set(TfToken("guide"));
	planeGeom.CreateAxisAttr().Set(axis);

	UsdPrim prim = stage->GetPrimAtPath(SdfPath(planePath + "/collisionPlane"));
	UsdPhysicsCollisionAPI::Apply(prim);
}

void addGroundTriMesh(const UsdStagePtr & stage,
    const std::string& physScenePath,
    float size,
    const pxr::GfVec3f& position,
    const pxr::GfVec3f& color)
{

    // Top level actor, contains rigid body and its shapes
    const std::string& path = physScenePath + "/staticTriMeshActor";

    // (Graphics) Plane mesh
    UsdGeomMesh entityPlane = createMeshSquare(stage, SdfPath(path), size, size);
    addDisplayColor(entityPlane, color);

    // Transform
    const float hRt2 = sqrt(2.0f) / 2.0f;
    const GfQuatf orientation(hRt2, 0.0f, -hRt2, 0.0f);

    addPosition(entityPlane, position);
    addOrientation(entityPlane, orientation);

#define USE_RENDER_MESH_FOR_COLLISION 1

#if USE_RENDER_MESH_FOR_COLLISION
    auto prim = stage->GetPrimAtPath(SdfPath(path));
    auto collision = UsdPhysicsCollisionAPI::Apply(prim);
#else
    // (Collision) Mesh
    std::string meshPath = path + "/collisionMesh";
    UsdGeomMesh component = createMeshSquare(stage, SdfPath(meshPath), size, size);
    component.CreatePurposeAttr().Set(UsdGeomTokens->guide);

    auto prim = stage->GetPrimAtPath(SdfPath(meshPath));
    auto collision = UsdPhysicsCollisionAPI::Apply(prim);
#endif
}

void encodeSdfPath(const SdfPath& path, uint32_t& ePart0, uint32_t& ePart1)
{
    uint64_t ui64Path;
    std::memcpy(&ui64Path, &path, sizeof(SdfPath));
    ePart0 = ui64Path & 0xFFffFFff;
    ePart1 = (ui64Path >> 32);
}


SdfPath decodeSdfPath(const uint32_t ePart0, const uint32_t ePart1)
{
    const uint64_t part0 = uint64_t(ePart0);
    const uint64_t part1 = uint64_t(ePart1);
    const uint64_t uintPath = part0 + (part1 << 32);
    return *(pxr::SdfPath*)&uintPath;
}

uint64_t sdfPathToInt(const SdfPath& path)
{
    uint64_t retInt;
    std::memcpy(&retInt, &path, sizeof(SdfPath));
    return retInt;
}


SdfPath intToSdfPath(const uint64_t intPath)
{
    return *(pxr::SdfPath*)&intPath;
}

pxr::GfQuatf indexedRotation(uint32_t axis, float s, float c)
{
    float v[3] = { 0, 0, 0 };
    v[axis] = s;
    return pxr::GfQuatf(v[0], v[1], v[2], c);
}

uint32_t getNextIndex3(uint32_t i)
{
    return (i + 1 + (i >> 1)) & 3;
}


pxr::GfVec3f diagonalize(const pxr::GfMatrix3f& m, pxr::GfQuatf& massFrame)
{
    // jacobi rotation using quaternions (from an idea of Stan Melax, with fix for precision issues)

    const uint32_t MAX_ITERS = 24;

    pxr::GfQuatf q = pxr::GfQuatf(1.0);

    pxr::GfMatrix3f d;
    for (uint32_t i = 0; i < MAX_ITERS; i++)
    {
        pxr::GfMatrix3f axes(q);
        d = axes.GetTranspose() * m * axes;

        float d0 = fabs(d[1][2]), d1 = fabs(d[0][2]), d2 = fabs(d[0][1]);
        uint32_t a = uint32_t(d0 > d1 && d0 > d2 ? 0 : d1 > d2 ? 1 : 2); // rotation axis index, from largest
                                                                         // off-diagonal
        // element

        uint32_t a1 = getNextIndex3(a), a2 = getNextIndex3(a1);
        if (d[a1][a2] == 0.0f || fabs(d[a1][a1] - d[a2][a2]) > 2e6 * fabs(2.0 * d[a1][a2]))
            break;

        float w = (d[a1][a1] - d[a2][a2]) / (2.0f * d[a1][a2]); // cot(2 * phi), where phi is the rotation angle
        float absw = fabs(w);

        pxr::GfQuatf r;
        if (absw > 1000)
            r = indexedRotation(a, 1 / (4 * w), 1.0f); // h will be very close to 1, so use small angle approx instead
        else
        {
            float t = 1 / (absw + sqrt(w * w + 1)); // absolute value of tan phi
            float h = 1 / sqrt(t * t + 1); // absolute value of cos phi

            r = indexedRotation(a, sqrt((1 - h) / 2) * ((w >= 0.0f) ? 1.0f : -1.0f), sqrt((1 + h) / 2));
        }

        q = (q * r).GetNormalized();
    }

    massFrame = q;
    return pxr::GfVec3f(d.GetColumn(0)[0], d.GetColumn(1)[1], d.GetColumn(2)[2]);
}

pxr::GfVec3f getMassSpaceInertia(const pxr::GfMatrix3f& inertia, pxr::GfQuatf& massFrame)
{
        pxr::GfVec3f diagT = diagonalize(inertia, massFrame);
        return diagT;
}

PXR_NAMESPACE_CLOSE_SCOPE