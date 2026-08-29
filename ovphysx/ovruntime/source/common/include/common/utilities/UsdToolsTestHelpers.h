// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// USD scene-authoring helpers (createMesh*, addRigid*, addGround*, ...) used by ovruntime unit
// tests / benchmarks; definitions live in UsdToolsTestHelpers.cpp, compiled only into the test
// targets. Encoding helpers live in SdfPathEncoding.h.

#include <common/utilities/SdfPathEncoding.h>

#include "pxr/pxr.h"
#include "pxr/usd/usd/typed.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xform.h"

#include "pxr/usd/sdf/path.h"

#include "pxr/base/vt/value.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/matrix3f.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"

#include <vector>

PXR_NAMESPACE_OPEN_SCOPE


// Meshes
UsdGeomMesh createMesh(const UsdStagePtr& stage,
    const SdfPath& path,
    const std::vector<GfVec3f>& points,
    const std::vector<GfVec3f>& normals,
    const std::vector<int>& indices,
    const std::vector<int>& vertexCounts);

UsdGeomMesh createMeshSquare(const UsdStagePtr& stage, const PXR_NS::SdfPath& path, float halfHeight, float halfWidth);

UsdGeomMesh createMeshBox(const UsdStagePtr& stage, const PXR_NS::SdfPath& path, const PXR_NS::GfVec3f& halfExtent = { 1, 1, 1 });

UsdGeomMesh createMeshSphere(const UsdStagePtr& stage,
    const PXR_NS::SdfPath& path,
    float radius,
    int latitudeSegments = 16,
    int longitudeSegments = 32);

UsdGeomMesh createMeshCapsule(const UsdStagePtr& stage,
    const PXR_NS::SdfPath& path,
    float radius,
    float halfHeight,
    int latitudeSegments = 16,
    int longitudeSegments = 32);

UsdGeomMesh createMeshCylinder(
    const UsdStagePtr& stage, const PXR_NS::SdfPath& path, float radius, float halfLength, uint32_t tesselation);

// Add USD schema for basic stuffs
void addPhysicsScene(const UsdStagePtr& stage, const std::string& path);

UsdGeomXform addActor(const UsdStagePtr& stage, const std::string& path);

UsdGeomXform addCollisionShape(const UsdStagePtr& stage, const std::string& path);

void addRigidBody(const UsdStagePtr& stage, const std::string& path);

void addPosition(UsdGeomXformable& xformable, const PXR_NS::GfVec3f& position);

void addOrientation(UsdGeomXformable& xformable, const PXR_NS::GfQuatf& orientation);

void addDisplayColor(PXR_NS::UsdGeomGprim& gprim, const PXR_NS::GfVec3f& color);

void addVelocity(const UsdStagePtr& stage,
    const std::string& path,
    const PXR_NS::GfVec3f& linearVelocity,
    const PXR_NS::GfVec3f& angularVelocity);

void addDensity(const UsdStagePtr& stage, const std::string& path, float value);

void addBoxCollisionShape(const UsdStagePtr& stage,
    const std::string& path,
    float size,
    const PXR_NS::GfVec3f& position,
    const PXR_NS::GfQuatf& orientation,
    const PXR_NS::GfVec3f& color);

void addGroundPlane(const UsdStagePtr& stage,
    const std::string& path,
	const PXR_NS::TfToken& axis = TfToken("Z"),
    float size = 100.0f,
    const PXR_NS::GfVec3f& position = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& color = PXR_NS::GfVec3f(0.5f));

void addGroundTriMesh(const UsdStagePtr& stage,
    const std::string& path,
    float size = 100.0f,
    const PXR_NS::GfVec3f& position = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& color = PXR_NS::GfVec3f(0.5f));

void addRigidBox(const UsdStagePtr& stage,
    const std::string& path,
    const PXR_NS::GfVec3f& size,
    const PXR_NS::GfVec3f& position,
    const PXR_NS::GfQuatf& orientation,
    const PXR_NS::GfVec3f& color,
    float density,
    const PXR_NS::GfVec3f& linVelocity = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& angularVelocity = PXR_NS::GfVec3f(0.0f));

void addRigidSphere(const UsdStagePtr& stage,
    const std::string& path,
    float radius,
    const PXR_NS::GfVec3f& position,
    const PXR_NS::GfQuatf& orientation,
    const PXR_NS::GfVec3f& color,
    float density,
    const PXR_NS::GfVec3f& linVelocity = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& angularVelocity = PXR_NS::GfVec3f(0.0f));

void addRigidCapsule(const UsdStagePtr& stage,
    const std::string& path,
    float radius,
    float halfHeight,
	const PXR_NS::TfToken& axis,
    const PXR_NS::GfVec3f& position,
    const PXR_NS::GfQuatf& orientation,
    const PXR_NS::GfVec3f& color,
    float density,
    const PXR_NS::GfVec3f& linVelocity = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& angularVelocity = PXR_NS::GfVec3f(0.0f));

void addRigidCylinder(const UsdStagePtr& stage,
    const std::string& path,
    float radius,
    float halfHeight,
	const PXR_NS::TfToken& axis,
    const PXR_NS::GfVec3f& position,
    const PXR_NS::GfQuatf& orientation,
    const PXR_NS::GfVec3f& color,
    float density,
    const PXR_NS::GfVec3f& linVelocity = PXR_NS::GfVec3f(0.0f),
    const PXR_NS::GfVec3f& angularVelocity = PXR_NS::GfVec3f(0.0f));

void addRigidCone(const UsdStagePtr& stage,
	const std::string& path,
	float radius,
	float halfHeight,
	const PXR_NS::TfToken& axis,
	const PXR_NS::GfVec3f& position,
	const PXR_NS::GfQuatf& orientation,
	const PXR_NS::GfVec3f& color,
	float density,
	const PXR_NS::GfVec3f& linVelocity = PXR_NS::GfVec3f(0.0f),
	const PXR_NS::GfVec3f& angularVelocity = PXR_NS::GfVec3f(0.0f));

void addRigidBoxForInstancing(const UsdStagePtr& stage,
    const std::string& path,
    const PXR_NS::GfVec3f& size,
    const PXR_NS::GfVec3f& color,
    float density);

PXR_NAMESPACE_CLOSE_SCOPE
