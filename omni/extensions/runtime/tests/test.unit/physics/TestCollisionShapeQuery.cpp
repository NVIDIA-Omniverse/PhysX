// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSceneQuery.h>

using namespace omni::physx;
using namespace pxr;
using namespace carb;

struct ShapeData
{
    uint64_t    sdfPath;
    Float3      position;
    Float4      orientation;
};

struct SphereData: public ShapeData
{
    float       radius;
};

struct BoxData : public ShapeData
{
    Float3      halfExtent;
};

struct CapsuleData : public ShapeData
{
    CollisionShapeAxis::Enum axis;
    float       radius;
    float       height;
};

struct CylinderData : public ShapeData
{
    CollisionShapeAxis::Enum axis;
    float       radius;
    float       height;
};

struct ConeData : public ShapeData
{
    CollisionShapeAxis::Enum axis;
    float       radius;
    float       height;
};

struct ConvexData : public ShapeData
{
    Float3 meshScale;
    uint32_t numVertices;
    const Float3* vertices;
    const uint8_t* indices;
    uint32_t numPolygons;
    std::vector<ConvexMeshPolygon> polygons;
};

struct MeshData : public ShapeData
{
    Float3 meshScale;
    uint32_t numVertices;
    const Float3* vertices;    
    uint32_t numTriangles;
    const uint32_t* triangles;
};


void reportSphere(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, float radius, void* userData)
{
    SphereData* sphereData = (SphereData*)userData;
    sphereData->sdfPath = sdfPath;
    sphereData->position = position;
    sphereData->orientation = orientation;
    sphereData->radius = radius;
};

void reportCapsule(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, CollisionShapeAxis::Enum axis, float radius, float height, void* userData)
{
    CapsuleData* capsuleData = (CapsuleData*)userData;
    capsuleData->sdfPath = sdfPath;
    capsuleData->position = position;
    capsuleData->orientation = orientation;
    capsuleData->radius = radius;
    capsuleData->height = height;
    capsuleData->axis = axis;
};

void reportCylinder(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, CollisionShapeAxis::Enum axis, float radius, float height, void* userData)
{
    CylinderData* capsuleData = (CylinderData*)userData;
    capsuleData->sdfPath = sdfPath;
    capsuleData->position = position;
    capsuleData->orientation = orientation;
    capsuleData->radius = radius;
    capsuleData->height = height;
    capsuleData->axis = axis;
};

void reportCone(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, CollisionShapeAxis::Enum axis, float radius, float height, void* userData)
{
    ConeData* capsuleData = (ConeData*)userData;
    capsuleData->sdfPath = sdfPath;
    capsuleData->position = position;
    capsuleData->orientation = orientation;
    capsuleData->radius = radius;
    capsuleData->height = height;
    capsuleData->axis = axis;
};

void reportBox(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, const carb::Float3& halfExtent, void* userData)
{
    BoxData* boxData = (BoxData*)userData;
    boxData->sdfPath = sdfPath;
    boxData->position = position;
    boxData->orientation = orientation;
    boxData->halfExtent = halfExtent;
};

void reportMesh(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, const carb::Float3& meshScale,
    uint32_t numVertices, const carb::Float3* vertices, uint32_t numTriangles, const uint32_t* triangles, void* userData)
{
    MeshData* meshData = (MeshData*)userData;
    meshData->sdfPath = sdfPath;
    meshData->position = position;
    meshData->orientation = orientation;
    meshData->meshScale = meshScale;
    meshData->numVertices = numVertices;
    meshData->vertices = vertices;
    meshData->numTriangles = numTriangles;
    meshData->triangles = triangles;
};


void reportConvex(uint64_t sdfPath, const carb::Float3& position, const carb::Float4& orientation, const carb::Float3& meshScale, uint32_t numVertices,
    const carb::Float3* vertices, const uint8_t* indices, uint32_t numPolygons, const ConvexMeshPolygon* polygons, void* userData)
{
    std::vector<ConvexData>* meshDatas = (std::vector<ConvexData>*)userData;
    ConvexData meshData;
    meshData.sdfPath = sdfPath;
    meshData.position = position;
    meshData.orientation = orientation;
    meshData.meshScale = meshScale;
    meshData.numVertices = numVertices;
    meshData.vertices = vertices;
    meshData.indices = indices;
    meshData.numPolygons = numPolygons;
    meshData.polygons.resize(numPolygons);
    memcpy(meshData.polygons.data(), polygons, sizeof(ConvexMeshPolygon) * numPolygons);
    meshDatas->push_back(meshData);
};

TEST_CASE("Collision Shape Query Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxSceneQuery* physxSq = physicsTests.acquirePhysxSceneQueryInterface();
    REQUIRE(physxSq);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    SUBCASE("Sphere Shape")
    {
        double radius = 100.0;
        GfVec3f position(200.0f);
        const SdfPath sphereShapePath = defaultPrimPath.AppendChild(TfToken("sphereShape"));
        UsdGeomSphere sphere = UsdGeomSphere::Define(stage, sphereShapePath);
        sphere.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        sphere.GetRadiusAttr().Set(radius);

        UsdPhysicsCollisionAPI::Apply(sphere.GetPrim());

        physxSim->attachStage(stageId);

        SphereData sphereData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&sphereData;
        cb.sphereShapeReportFn = &reportSphere;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(fabsf(sphereData.radius - float(radius)) < epsilon);
        CHECK(sphereShapePath == intToSdfPath(sphereData.sdfPath));
        compare(sphereData.position, position, epsilon);
        compare(sphereData.orientation, GfQuatf(1.0f), epsilon);
    }

    SUBCASE("Box Shape")
    {
        double size = 100.0;
        GfVec3f scale(2.0f, 3.0f, 4.0f);
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCube shape = UsdGeomCube::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);
        shape.GetSizeAttr().Set(size);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        BoxData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.boxShapeReportFn = &reportBox;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);
        
        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        compare(shapeData.halfExtent, GfVec3f(100.0f, 150.0f, 200.0f), epsilon);
    }

    SUBCASE("Capsule Shape")
    {
        double radius = 100.0;
        double height = 200.0;
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCapsule shape = UsdGeomCapsule::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);        
        shape.GetRadiusAttr().Set(radius);
        shape.GetHeightAttr().Set(height);
        shape.GetAxisAttr().Set(UsdGeomTokens->y);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        CapsuleData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.capsuleShapeReportFn = &reportCapsule;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        CHECK(fabsf(shapeData.height - float(height)) < epsilon);
        CHECK(fabsf(shapeData.radius - float(radius)) < epsilon);
        CHECK(shapeData.axis == CollisionShapeAxis::eY);
    }

    SUBCASE("Cylinder Shape")
    {
        double radius = 100.0;
        double height = 200.0;
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCylinder shape = UsdGeomCylinder::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.GetRadiusAttr().Set(radius);
        shape.GetHeightAttr().Set(height);
        shape.GetAxisAttr().Set(UsdGeomTokens->y);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        CylinderData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.cylinderShapeReportFn = &reportCylinder;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        CHECK(fabsf(shapeData.height - float(height)) < epsilon);
        CHECK(fabsf(shapeData.radius - float(radius)) < epsilon);
        CHECK(shapeData.axis == CollisionShapeAxis::eY);
    }

    SUBCASE("Cone Shape")
    {
        double radius = 100.0;
        double height = 200.0;
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCone shape = UsdGeomCone::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.GetRadiusAttr().Set(radius);
        shape.GetHeightAttr().Set(height);
        shape.GetAxisAttr().Set(UsdGeomTokens->y);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        ConeData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.coneShapeReportFn = &reportCone;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        CHECK(fabsf(shapeData.height - float(height)) < epsilon);
        CHECK(fabsf(shapeData.radius - float(radius)) < epsilon);
        CHECK(shapeData.axis == CollisionShapeAxis::eY);
    }

    SUBCASE("Cylinder Shape Custom")
    {
        double radius = 100.0;
        double height = 200.0;
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCylinder shape = UsdGeomCylinder::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.GetRadiusAttr().Set(radius);
        shape.GetHeightAttr().Set(height);
        shape.GetAxisAttr().Set(UsdGeomTokens->y);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        CylinderData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.cylinderShapeReportFn = &reportCylinder;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        CHECK(fabsf(shapeData.height - float(height)) < epsilon);
        CHECK(fabsf(shapeData.radius - float(radius)) < epsilon);
        CHECK(shapeData.axis == CollisionShapeAxis::eY);
    }

    SUBCASE("Cone Shape Custom")
    {
        double radius = 100.0;
        double height = 200.0;
        GfVec3f position(200.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomCone shape = UsdGeomCone::Define(stage, shapePath);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.GetRadiusAttr().Set(radius);
        shape.GetHeightAttr().Set(height);
        shape.GetAxisAttr().Set(UsdGeomTokens->y);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());

        physxSim->attachStage(stageId);

        ConeData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.coneShapeReportFn = &reportCone;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        CHECK(fabsf(shapeData.height - float(height)) < epsilon);
        CHECK(fabsf(shapeData.radius - float(radius)) < epsilon);
        CHECK(shapeData.axis == CollisionShapeAxis::eY);
    }

    SUBCASE("TriangleMesh Shape")
    {
        float halfSize = 100.0f;
        GfVec3f position(200.0f);
        GfVec3f scale(1.0f, 2.0f, 3.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomMesh shape = createMeshBox(stage, shapePath, GfVec3f(halfSize));
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->none);

        physxSim->attachStage(stageId);

        MeshData shapeData;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeData;
        cb.triangleMeshShapeReportFn = &reportMesh;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        compare(shapeData.meshScale, scale, epsilon);                
        CHECK(shapeData.numVertices == 8);
        compare(shapeData.vertices[0], GfVec3f(-100.0f, -100.0f, 100.0f), epsilon);
        CHECK(shapeData.numTriangles == 12);
    }

    SUBCASE("ConvexMesh Shape")
    {
        float halfSize = 100.0f;
        GfVec3f position(200.0f);
        GfVec3f scale(1.0f, 2.0f, 3.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomMesh shape = createMeshBox(stage, shapePath, GfVec3f(halfSize));
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);

        physxSim->attachStage(stageId);

        std::vector<ConvexData> shapeDatas; 

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeDatas;
        cb.convexMeshShapeReportFn = &reportConvex;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        REQUIRE(shapeDatas.size() == 1);
        const ConvexData& shapeData = shapeDatas[0];

        CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
        compare(shapeData.position, position, epsilon);
        compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        compare(shapeData.meshScale, scale, epsilon);
        CHECK(shapeData.numVertices == 8);
        CHECK(shapeData.numPolygons == 6);
    }

    SUBCASE("ConvexMesh Decomposition Shape")
    {
        float halfSize = 100.0f;
        GfVec3f position(200.0f);
        GfVec3f scale(1.0f, 2.0f, 3.0f);
        const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
        UsdGeomMesh shape = createConcaveMesh(stage, shapePath, halfSize);
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);

        physxSim->attachStage(stageId);

        std::vector<ConvexData> shapeDatas;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeDatas;
        cb.convexMeshShapeReportFn = &reportConvex;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        CHECK(shapeDatas.size() > 1);
        for (size_t i = 0; i < shapeDatas.size(); i++)
        {
            const ConvexData& shapeData = shapeDatas[i];

            CHECK(shapePath == intToSdfPath(shapeData.sdfPath));
            compare(shapeData.meshScale, scale, epsilon);
            compare(shapeData.position, position, epsilon);
            compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
        }
    }

    SUBCASE("PointInstanced ConvexMesh Shape")
    {
        float halfSize = 100.0f;
        GfVec3f position(200.0f);
        GfVec3f scale(1.0f, 2.0f, 3.0f);

        const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
        const SdfPath shapePath = geomPointInstancerPath.AppendChild(TfToken("shape"));
        UsdGeomMesh shape = createMeshBox(stage, shapePath, GfVec3f(halfSize));
        shape.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(position);
        shape.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);
        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);
        UsdPhysicsRigidBodyAPI::Apply(shape.GetPrim());

        VtArray<int> meshIndices = { 0, 0 };
        VtArray<GfVec3f> positions = { GfVec3f(-125.0, 0.0, 500.0), GfVec3f(-125.0, 0.0, 500.0) };
        VtArray<GfQuath> orientations = { GfQuath(1.0), GfQuath(1.0) };
        VtArray<GfVec3f> linearVelocities = { GfVec3f(0.0), GfVec3f(0.0, 0.0, 0.0) };
        VtArray<GfVec3f> angularVelocities = { GfVec3f(0.0, 10.0, 0.0), GfVec3f(0.0) };

        UsdGeomPointInstancer shapeList = UsdGeomPointInstancer::Define(stage, geomPointInstancerPath);
        shapeList.GetPrototypesRel().AddTarget(shapePath);

        shapeList.GetProtoIndicesAttr().Set(meshIndices);
        shapeList.GetPositionsAttr().Set(positions);
        shapeList.GetOrientationsAttr().Set(orientations);
        shapeList.GetVelocitiesAttr().Set(linearVelocities);
        shapeList.GetAngularVelocitiesAttr().Set(angularVelocities);


        physxSim->attachStage(stageId);

        std::vector<ConvexData> shapeDatas;

        ICollisionShapeQueryCallback cb;
        cb.userData = (void*)&shapeDatas;
        cb.convexMeshShapeReportFn = &reportConvex;

        physxSq->reportCollisionShapes(sdfPathToInt(defaultPrimPath), cb);

        REQUIRE(shapeDatas.size() == 2);
        for (size_t i = 0; i < shapeDatas.size(); i++)
        {
            const ConvexData& shapeData = shapeDatas[i];

            CHECK(shapePath == intToSdfPath(shapeData.sdfPath));            
            compare(shapeData.orientation, GfQuatf(1.0f), epsilon);
            compare(shapeData.meshScale, scale, epsilon);
            CHECK(shapeData.numVertices == 8);
            CHECK(shapeData.numPolygons == 6);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

}
