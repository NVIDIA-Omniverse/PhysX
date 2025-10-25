#ifndef WEB_IDL_BINDINGS_H
#define WEB_IDL_BINDINGS_H

#include <cstring>
#include <iostream>

#include "PxPhysicsAPI.h"
#include "common/PxRenderOutput.h"
#include "extensions/PxCollectionExt.h"
#include "extensions/PxGjkQueryExt.h"
#include "extensions/PxTetMakerExt.h"
#include "geomutils/PxContactBuffer.h"

#include "omnipvd/PxOmniPvd.h"
#include "pvd/PxPvdTransport.h"

#include "vehicle/Base.h"
#include "vehicle/DirectDrivetrain.h"
#include "vehicle/EngineDrivetrain.h"
#include "vehicle/PhysXIntegration.h"

#include "PxTypeMappings.h"

class PassThroughFilterShader {
    public:
        virtual physx::PxU32 filterShader(physx::PxU32 attributes0,
                                          physx::PxU32 filterData0w0, physx::PxU32 filterData0w1, physx::PxU32 filterData0w2, physx::PxU32 filterData0w3,
                                          physx::PxU32 attributes1,
                                          physx::PxU32 filterData1w0, physx::PxU32 filterData1w1, physx::PxU32 filterData1w2, physx::PxU32 filterData1w3) = 0;

        virtual ~PassThroughFilterShader() { }

        physx::PxU32 outputPairFlags;
};

physx::PxFilterFlags passThrFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
                                         physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
                                         physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize) {
    PX_UNUSED(constantBlockSize);

    PassThroughFilterShader* shader = *((PassThroughFilterShader* const *) constantBlock);
    shader->outputPairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;

    physx::PxFilterFlags result = physx::PxFilterFlags(static_cast<physx::PxU16>(shader->filterShader(
            (physx::PxU32) attributes0, filterData0.word0, filterData0.word1, filterData0.word2, filterData0.word3,
            (physx::PxU32) attributes1, filterData1.word0, filterData1.word1, filterData1.word2, filterData1.word3)));

    pairFlags = physx::PxPairFlags(static_cast<physx::PxU16>(shader->outputPairFlags));
    return result;
}

// default scene filter / query shaders, implemented in C++ for performance reasons
physx::PxFilterFlags defaultFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
                                         physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
                                         physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize) {
    PX_UNUSED(constantBlock);
    PX_UNUSED(constantBlockSize);

    if ((0 == (filterData0.word0 & filterData1.word1)) && (0 == (filterData1.word0 & filterData0.word1))) {
        return physx::PxFilterFlag::eSUPPRESS;
    }

    if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
    } else {
        pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
    }
    pairFlags |= physx::PxPairFlags(physx::PxU16(filterData0.word2 | filterData1.word2));

    return physx::PxFilterFlag::eDEFAULT;
}

// Slightly simplified SimulationEventCallback which can be implemented in non-native code
class SimpleSimulationEventCallback : physx::PxSimulationEventCallback {
    public:
        virtual void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) = 0;
        virtual void onWake(physx::PxActor**, physx::PxU32) = 0;
        virtual void onSleep(physx::PxActor**, physx::PxU32) = 0;
        virtual void onContact(const physx::PxContactPairHeader&, const physx::PxContactPair*, physx::PxU32 nbPairs) = 0;
        virtual void onTrigger(physx::PxTriggerPair*, physx::PxU32) = 0;

        // implement onAdvance with empty body so it does not have to be implemented
        // in non-native code (for the sake of performance)
        virtual void onAdvance(const physx::PxRigidBody *const *, const physx::PxTransform*, const physx::PxU32) { }
};

// Slightly simplified PxCustomGeometry::Callbacks which can be implemented in non-native code
class SimpleCustomGeometryCallbacks : public physx::PxCustomGeometry::Callbacks {
    public:
        // non-const virtual methods
        virtual physx::PxBounds3* getLocalBoundsImpl(const physx::PxGeometry& geometry) = 0;
        virtual bool generateContactsImpl(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
            physx::PxReal contactDistance, physx::PxReal meshContactMargin, physx::PxReal toleranceLength,
            physx::PxContactBuffer& contactBuffer) = 0;
        virtual physx::PxU32 raycastImpl(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
            physx::PxReal maxDist, physx::PxHitFlags& hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride) = 0;
        virtual bool overlapImpl(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1) = 0;
        virtual bool sweepImpl(const physx::PxVec3& unitDir, physx::PxReal maxDist,
            const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
            physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags& hitFlags, physx::PxReal inflation) = 0;
        virtual void computeMassPropertiesImpl(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) = 0;
        virtual bool usePersistentContactManifoldImpl(const physx::PxGeometry& geometry) = 0;

        // original callbacks methods, forwarding to non-const methods
        virtual physx::PxBounds3 getLocalBounds(const physx::PxGeometry& geometry) const {
            return *((SimpleCustomGeometryCallbacks*) this)->getLocalBoundsImpl(geometry);
        }
        virtual bool generateContacts(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
            const physx::PxReal contactDistance, const physx::PxReal meshContactMargin, const physx::PxReal toleranceLength,
            physx::PxContactBuffer& contactBuffer) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->generateContactsImpl(geom0, geom1, pose0, pose1, contactDistance, meshContactMargin, toleranceLength, contactBuffer);
        }
        virtual physx::PxU32 raycast(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
            physx::PxReal maxDist, physx::PxHitFlags hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride, physx::PxRaycastThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->raycastImpl(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride);
        }
        virtual bool overlap(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1, physx::PxOverlapThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->overlapImpl(geom0, pose0, geom1, pose1);
        }
        virtual bool sweep(const physx::PxVec3& unitDir, const physx::PxReal maxDist,
            const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
            physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags hitFlags, const physx::PxReal inflation, physx::PxSweepThreadContext*) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->sweepImpl(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation);
        }
        virtual void computeMassProperties(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) const
        {
            return ((SimpleCustomGeometryCallbacks*) this)->computeMassPropertiesImpl(geometry, massProperties);
        }
        virtual bool usePersistentContactManifold(const physx::PxGeometry& geometry, physx::PxReal& breakingThreshold) const
        {
            bool retVal = ((SimpleCustomGeometryCallbacks*) this)->usePersistentContactManifoldImpl(geometry);
            breakingThreshold = persistentContactManifold_outBreakingThreshold;
            return retVal;
        }

        // unused / not-available in non-native code
        virtual physx::PxCustomGeometry::Type getCustomType() const { return physx::PxCustomGeometry::Type(); }
        virtual void visualize(const physx::PxGeometry&, physx::PxRenderOutput&, const physx::PxTransform&, const physx::PxBounds3&) const { }

        physx::PxReal persistentContactManifold_outBreakingThreshold;
};

class SimplePvdTransport : physx::PxPvdTransport {
    public:
        SimplePvdTransport() { }

        virtual bool connect() = 0;
        virtual bool isConnected() = 0;
        virtual void send(void* inBytes, uint32_t inLength) = 0;
        virtual void flush() = 0;
        virtual void disconnect() = 0;

        bool write(const uint8_t *inBytes, uint32_t inLength) {
            send((void*) inBytes, inLength);
            return true;
        }

        PxPvdTransport &lock() {
            return *this;
        }

        void unlock() { }
        uint64_t getWrittenDataSize() { return 0; }
        void release() { }
};

class SimpleControllerBehaviorCallback : physx::PxControllerBehaviorCallback {
    public:
        virtual physx::PxU32 getShapeBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor) = 0;
        virtual physx::PxU32 getControllerBehaviorFlags(const physx::PxController& controller) = 0;
        virtual physx::PxU32 getObstacleBehaviorFlags(const physx::PxObstacle& obstacle) = 0;

        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getShapeBehaviorFlags(shape, actor)));
        }
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxController& controller) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getControllerBehaviorFlags(controller)));
        }
        virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxObstacle& obstacle) {
            return physx::PxControllerBehaviorFlags(static_cast<physx::PxU8>(getObstacleBehaviorFlags(obstacle)));
        }

        virtual ~SimpleControllerBehaviorCallback() { }
};

class SimpleQueryFilterCallback : physx::PxQueryFilterCallback {
    public:
        virtual physx::PxU32 simplePreFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags &queryFlags) = 0;
        virtual physx::PxU32 simplePostFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit, const physx::PxShape* shape, const physx::PxRigidActor* actor) = 0;

        virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags &queryFlags) {
            return static_cast<physx::PxQueryHitType::Enum>(simplePreFilter(filterData, shape, actor, queryFlags));
        }
        virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit, const physx::PxShape* shape, const physx::PxRigidActor* actor) {
            return static_cast<physx::PxQueryHitType::Enum>(simplePostFilter(filterData, hit, shape, actor));
        }

        virtual ~SimpleQueryFilterCallback() { }
};

template<typename HitType>
class PxHitResult : public physx::PxHitCallback<HitType> {
    public:
        PxHitResult() : physx::PxHitCallback<HitType>(resultBuf, 8) { }

        PX_INLINE physx::PxU32 getNbAnyHits() const {
            return getNbTouches() + physx::PxU32(this->hasBlock);
        }
        PX_INLINE const HitType& getAnyHit(const physx::PxU32 index) const {
            PX_ASSERT(index < getNbTouches() + physx::PxU32(this->hasBlock));
            return index < getNbTouches() ? hits[index] : this->block;
        }

        PX_INLINE physx::PxU32 getNbTouches() const {
            return static_cast<physx::PxU32>(hits.size());
        }
        PX_INLINE const HitType& getTouch(const physx::PxU32 index) const {
            PX_ASSERT(index < getNbTouches());
            return hits[index];
        }

        PX_INLINE void clear() {
            hits.clear();
        }

    protected:
        virtual physx::PxAgain processTouches(const HitType* buffer, physx::PxU32 nbHits) {
            if (isFinalized) {
                hits.clear();
                isFinalized = false;
            }
            for (physx::PxU32 i = 0; i < nbHits; i++) {
                hits.pushBack(buffer[i]);
            }
            return true;
        }
        virtual void finalizeQuery() {
            isFinalized = true;
        }

    private:
        bool isFinalized = true;
        HitType resultBuf[8];
        physx::PxArray<HitType> hits;
};

typedef PxHitResult<physx::PxOverlapHit> PxOverlapResult;
typedef PxHitResult<physx::PxRaycastHit> PxRaycastResult;
typedef PxHitResult<physx::PxSweepHit> PxSweepResult;

// top-level functions are not supported by webidl binder, we need to wrap them in a class
struct PxTopLevelFunctions {
    static const physx::PxU32 PHYSICS_VERSION = PX_PHYSICS_VERSION;

    static physx::PxSimulationFilterShader DefaultFilterShader() {
        return &defaultFilterShader;
    }

    static void setupPassThroughFilterShader(physx::PxSceneDesc* sceneDesc, PassThroughFilterShader* filterShader) {
        PassThroughFilterShader** data = new PassThroughFilterShader*[1];
        data[0] = filterShader;
        sceneDesc->filterShader = &passThrFilterShader;
        sceneDesc->filterShaderData = data;
        sceneDesc->filterShaderDataSize = sizeof(PassThroughFilterShader*);
    }

    static physx::PxFoundation* CreateFoundation(physx::PxU32 version, physx::PxDefaultAllocator& allocator, physx::PxErrorCallback& errorCallback) {
        return PxCreateFoundation(version, allocator, errorCallback);
    }

    static physx::PxPhysics *CreatePhysics(physx::PxU32 version, physx::PxFoundation &foundation, const physx::PxTolerancesScale &scale, physx::PxPvd* pvd = NULL, physx::PxOmniPvd* omniPvd = NULL) {
        return PxCreatePhysics(version, foundation, scale, false, pvd, omniPvd);
    }

    static physx::PxControllerManager* CreateControllerManager(physx::PxScene& scene, bool lockingEnabled = false) {
        return PxCreateControllerManager(scene, lockingEnabled);
    }

    static physx::PxPvd *CreatePvd(physx::PxFoundation &foundation) {
        return PxCreatePvd(foundation);
    }

#ifndef __EMSCRIPTEN__
    static physx::PxPvdTransport* DefaultPvdSocketTransportCreate(const char *host, int port, unsigned int timeoutInMilliseconds) {
        return physx::PxDefaultPvdSocketTransportCreate(host, port, timeoutInMilliseconds);
    }

    static physx::PxOmniPvd *CreateOmniPvd(physx::PxFoundation &foundation) {
        return PxCreateOmniPvd(foundation);
    }
#endif

    static physx::PxDefaultCpuDispatcher* DefaultCpuDispatcherCreate(physx::PxU32 numThreads) {
        return physx::PxDefaultCpuDispatcherCreate(numThreads);
    }

    static bool InitExtensions(physx::PxPhysics& physics) {
        return PxInitExtensions(physics, NULL);
    }

    static void CloseExtensions() {
        PxCloseExtensions();
    }

    static physx::PxD6Joint* D6JointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxDistanceJoint* DistanceJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxDistanceJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxFixedJoint* FixedJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxFixedJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxPrismaticJoint* PrismaticJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxPrismaticJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxRevoluteJoint* RevoluteJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxRevoluteJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxSphericalJoint* SphericalJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxGearJoint* GearJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxGearJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxRackAndPinionJoint* RackAndPinionJointCreate(physx::PxPhysics& physics, physx::PxRigidActor* actor0, physx::PxTransform& localFrame0, physx::PxRigidActor* actor1, physx::PxTransform& localFrame1) {
        return physx::PxRackAndPinionJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
    }

    static physx::PxConvexMesh* CreateConvexMesh(const physx::PxCookingParams& params, const physx::PxConvexMeshDesc& desc) {
        return PxCreateConvexMesh(params, desc);
    }

    static physx::PxTriangleMesh* CreateTriangleMesh(const physx::PxCookingParams &params, const physx::PxTriangleMeshDesc &desc) {
        return PxCreateTriangleMesh(params, desc);
    }

    static physx::PxHeightField* CreateHeightField(const physx::PxHeightFieldDesc &desc) {
        return PxCreateHeightField(desc);
    }

    static physx::PxRigidDynamic* CreateDynamicFromShape(physx::PxPhysics& sdk, const physx::PxTransform& transform, physx::PxShape& shape, physx::PxReal density) {
        return PxCreateDynamic(sdk, transform, shape, density);
    }

    static physx::PxRigidDynamic* CreateDynamic(physx::PxPhysics& sdk, const physx::PxTransform& transform, const physx::PxGeometry& geometry, physx::PxMaterial& material, physx::PxReal density, const physx::PxTransform& shapeOffset = physx::PxTransform(physx::PxIdentity)) {
        return PxCreateDynamic(sdk, transform, geometry, material, density, shapeOffset);
    }

    static physx::PxRigidDynamic* CreateKinematicFromShape(physx::PxPhysics& sdk, const physx::PxTransform& transform, physx::PxShape& shape, physx::PxReal density) {
        return PxCreateKinematic(sdk, transform, shape, density);
    }

    static physx::PxRigidDynamic* CreateKinematic(physx::PxPhysics& sdk, const physx::PxTransform& transform, const physx::PxGeometry& geometry, physx::PxMaterial& material, physx::PxReal density, const physx::PxTransform& shapeOffset = physx::PxTransform(physx::PxIdentity)) {
        return PxCreateKinematic(sdk, transform, geometry, material, density, shapeOffset);
    }

    static physx::PxRigidStatic* CreateStaticFromShape(physx::PxPhysics& sdk, const physx::PxTransform& transform, physx::PxShape& shape) {
        return PxCreateStatic(sdk, transform, shape);
    }

    static physx::PxRigidStatic* CreateStatic(physx::PxPhysics& sdk, const physx::PxTransform& transform, const physx::PxGeometry& geometry, physx::PxMaterial& material, const physx::PxTransform& shapeOffset) {
        return PxCreateStatic(sdk, transform, geometry, material, shapeOffset);
    }

    static physx::PxRigidStatic* CreatePlane(physx::PxPhysics& sdk, const physx::PxPlane& plane, physx::PxMaterial& material) {
        return PxCreatePlane(sdk, plane, material);
    }

    static physx::PxShape* CloneShape(physx::PxPhysics& physics, const physx::PxShape& from, bool isExclusive) {
        return PxCloneShape(physics, from, isExclusive);
    }

    static physx::PxRigidStatic* CloneStatic(physx::PxPhysics& physicsSDK, const physx::PxTransform& transform, const physx::PxRigidActor& from) {
        return PxCloneStatic(physicsSDK, transform, from);
    }

    static physx::PxRigidDynamic* CloneDynamic(physx::PxPhysics& physicsSDK, const physx::PxTransform& transform, const physx::PxRigidDynamic& from) {
        return PxCloneDynamic(physicsSDK, transform, from);
    }

    static void ScaleRigidActor(physx::PxRigidActor& actor, physx::PxReal scale, bool scaleMassProps) {
        return PxScaleRigidActor(actor, scale, scaleMassProps);
    }

    static void IntegrateTransform(const physx::PxTransform& curTrans, const physx::PxVec3& linvel, const physx::PxVec3& angvel, physx::PxReal timeStep, physx::PxTransform& result) {
        return PxIntegrateTransform(curTrans, linvel, angvel, timeStep, result);
    }

    static bool CookTriangleMesh(const physx::PxCookingParams& params, const physx::PxTriangleMeshDesc& desc, physx::PxOutputStream& stream) {
        return PxCookTriangleMesh(params, desc, stream);
    }

    static bool CookConvexMesh(const physx::PxCookingParams& params, const physx::PxConvexMeshDesc& desc, physx::PxOutputStream& stream) {
        return PxCookConvexMesh(params, desc, stream);
    }
};

struct PxVehicleTopLevelFunctions {

    static bool InitVehicleExtension(physx::PxFoundation& foundation) {
        return physx::vehicle2::PxInitVehicleExtension(foundation);
    }

    static void CloseVehicleExtension() {
        physx::vehicle2::PxCloseVehicleExtension();
    }

    static bool VehicleComputeSprungMasses(physx::PxU32 nbSprungMasses, Vector_PxVec3& sprungMassCoordinates, physx::PxReal totalMass, PxVehicleAxesEnum gravityDirection, Vector_PxReal& sprungMasses) {
        return physx::vehicle2::PxVehicleComputeSprungMasses(nbSprungMasses, sprungMassCoordinates.data(), totalMass, gravityDirection, sprungMasses.data());
    }

    static physx::PxConvexMesh* VehicleUnitCylinderSweepMeshCreate(const physx::vehicle2::PxVehicleFrame& vehicleFrame, physx::PxPhysics& physics, const physx::PxCookingParams& params) {
        return physx::vehicle2::PxVehicleUnitCylinderSweepMeshCreate(vehicleFrame, physics, params);
    }

    static void VehicleUnitCylinderSweepMeshDestroy(physx::PxConvexMesh* mesh) {
        physx::vehicle2::PxVehicleUnitCylinderSweepMeshDestroy(mesh);
    }

    static const physx::PxU32 MAX_NB_ENGINE_TORQUE_CURVE_ENTRIES = physx::vehicle2::PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES;
};

struct PxExtensionTopLevelFunctions {
    static physx::PxRigidStatic* CreatePlane(physx::PxPhysics &sdk, const physx::PxPlane &plane, physx::PxMaterial &material, const physx::PxFilterData &filterData) {
        physx::PxRigidStatic* actor = physx::PxCreatePlane(sdk, plane, material);
        physx::PxShape* shapes[1];
        actor->getShapes(shapes, 1);
        shapes[0]->setSimulationFilterData(filterData);
        return actor;
    }
};

struct PxVehicleTireForceParamsExt {
    static void setFrictionVsSlip(physx::vehicle2::PxVehicleTireForceParams* tireForceParams, int i, int j, float value) {
        tireForceParams->frictionVsSlip[i][j] = value;
    }

    static void setLoadFilter(physx::vehicle2::PxVehicleTireForceParams* tireForceParams, int i, int j, float value) {
        tireForceParams->loadFilter[i][j] = value;
    }
};

// Various helper functions for pointer access and conversion
struct NativeArrayHelpers {
    static physx::PxU8 getU8At(const physx::PxU8* base, int index) {
        return base[index];
    }

    static physx::PxU16 getU16At(const physx::PxU16* base, int index) {
        return base[index];
    }

    static physx::PxU32 getU32At(const physx::PxU32* base, int index) {
        return base[index];
    }

    static physx::PxReal getRealAt(physx::PxReal* base, int index) {
        return base[index];
    }

    static void setU8At(void* base, int index, physx::PxU8 value) {
        static_cast<physx::PxU8*>(base)[index] = value;
    }

    static void setU16At(void* base, int index, physx::PxU16 value) {
        static_cast<physx::PxU16*>(base)[index] = value;
    }

    static void setU32At(void* base, int index, physx::PxU32 value) {
        static_cast<physx::PxU32*>(base)[index] = value;
    }

    static void setRealAt(void* base, int index, physx::PxReal value) {
        static_cast<physx::PxReal*>(base)[index] = value;
    }

    static physx::PxActor* getActorAt(physx::PxActor* base, int index) {
        return &base[index];
    }

    static physx::PxBounds3* getBounds3At(physx::PxBounds3* base, int index) {
        return &base[index];
    }

    static physx::PxContactPair* getContactPairAt(physx::PxContactPair* base, int index) {
        return &base[index];
    }

    static physx::PxContactPairHeader* getContactPairHeaderAt(physx::PxContactPairHeader* base, int index) {
        return &base[index];
    }

    static physx::PxController* getControllerAt(physx::PxController* base, int index) {
        return &base[index];
    }

    static physx::PxControllerShapeHit* getControllerShapeHitAt(physx::PxControllerShapeHit* base, int index) {
        return &base[index];
    }

    static physx::PxControllersHit* getControllersHitAt(physx::PxControllersHit* base, int index) {
        return &base[index];
    }

    static physx::PxControllerObstacleHit* getControllerObstacleHitAt(physx::PxControllerObstacleHit* base, int index) {
        return &base[index];
    }

    static physx::PxObstacle* getObstacleAt(physx::PxObstacle* base, int index) {
        return &base[index];
    }

    static physx::PxShape* getShapeAt(physx::PxShape* base, int index) {
        return &base[index];
    }

    static physx::PxTriggerPair* getTriggerPairAt(physx::PxTriggerPair* base, int index) {
        return &base[index];
    }

    static physx::PxVec3* getVec3At(physx::PxVec3* base, int index) {
        return &base[index];
    }

    static physx::PxDebugPoint* getDebugPointAt(physx::PxDebugPoint* base, int index) {
        return &base[index];
    }

    static physx::PxDebugLine* getDebugLineAt(physx::PxDebugLine* base, int index) {
        return &base[index];
    }

    static physx::PxDebugTriangle* getDebugTriangleAt(physx::PxDebugTriangle* base, int index) {
        return &base[index];
    }

    static PxU8Ptr voidToU8Ptr(void* voidPtr) {
        return (PxU8Ptr) voidPtr;
    }

    static PxU16Ptr voidToU16Ptr(void* voidPtr) {
        return (PxU16Ptr) voidPtr;
    }

    static PxU32Ptr voidToU32Ptr(void* voidPtr) {
        return (PxU32Ptr) voidPtr;
    }

    static PxI32Ptr voidToI32Ptr(void* voidPtr) {
        return (PxI32Ptr) voidPtr;
    }

    static PxRealPtr voidToRealPtr(void* voidPtr) {
        return (PxRealPtr) voidPtr;
    }
};

// Helper functions for accessing functions, which don't map well to JS / Java
struct SupportFunctions {
    static physx::PxShape* PxActor_getShape(physx::PxRigidActor& actor, physx::PxU32 i) {
        physx::PxShape* shapePtr;
        actor.getShapes(&shapePtr, 1, i);
        return shapePtr;
    }

    static physx::PxActor* PxContactPairHeader_getActor(physx::PxContactPairHeader& pairHeader, physx::PxU32 i) {
        return pairHeader.actors[i];
    }

    static PxArray_PxActorPtr& PxScene_getActiveActors(physx::PxScene* scene) {
        static PxArray_PxActorPtr activeActors;
        physx::PxU32 nbActors;
        physx::PxActor** actors = scene->getActiveActors(nbActors);

        activeActors.resize(static_cast<size_t>(nbActors));
        std::memcpy(activeActors.begin(), actors, static_cast<size_t>(sizeof(physx::PxActor*) * nbActors));
        return activeActors;
    }

    static physx::PxU32 PxArticulationReducedCoordinate_getMinSolverPositionIterations(physx::PxArticulationReducedCoordinate* articulation) {
        physx::PxU32 minPosIters;
        physx::PxU32 minVelIters;
        articulation->getSolverIterationCounts(minPosIters, minVelIters);
        return minPosIters;
    }

    static physx::PxU32 PxArticulationReducedCoordinate_getMinSolverVelocityIterations(physx::PxArticulationReducedCoordinate* articulation) {
        physx::PxU32 minPosIters;
        physx::PxU32 minVelIters;
        articulation->getSolverIterationCounts(minPosIters, minVelIters);
        return minVelIters;
    }
};

struct PxGjkQueryProximityInfoResult {
    bool success;
    physx::PxVec3 pointA;
    physx::PxVec3 pointB;
    physx::PxVec3 separatingAxis;
    physx::PxReal separation;
};

struct PxGjkQueryRaycastResult {
    bool success;
    physx::PxReal t;
    physx::PxVec3 n;
    physx::PxVec3 p;
};

struct PxGjkQuerySweepResult {
    bool success;
    physx::PxReal t;
    physx::PxVec3 n;
    physx::PxVec3 p;
};

// physx::PxGjkQuery wrapper using result objects instead of output primitive-references
struct PxGjkQuery {
    static bool proximityInfo(const physx::PxGjkQuery::Support& a, const physx::PxGjkQuery::Support& b, const physx::PxTransform& poseA, const physx::PxTransform& poseB,
        physx::PxReal contactDistance, physx::PxReal toleranceLength, PxGjkQueryProximityInfoResult& result) {
        result.success = physx::PxGjkQuery::proximityInfo(a, b, poseA, poseB, contactDistance, toleranceLength, result.pointA, result.pointB, result.separatingAxis, result.separation);
        return result.success;
    }

    static bool raycast(const physx::PxGjkQuery::Support &shape, const physx::PxTransform &pose, const physx::PxVec3 &rayStart, const physx::PxVec3 &unitDir, physx::PxReal maxDist, PxGjkQueryRaycastResult& result) {
        result.success = physx::PxGjkQuery::raycast(shape, pose, rayStart, unitDir, maxDist, result.t, result.n, result.p);
        return result.success;
    }

    static bool overlap(const physx::PxGjkQuery::Support &a, const physx::PxGjkQuery::Support &b, const physx::PxTransform &poseA, const physx::PxTransform &poseB) {
        return physx::PxGjkQuery::overlap(a, b, poseA, poseB);
    }

    static bool sweep(const physx::PxGjkQuery::Support &a, const physx::PxGjkQuery::Support &b, const physx::PxTransform &poseA, const physx::PxTransform &poseB,
        const physx::PxVec3 &unitDir, physx::PxReal maxDist, PxGjkQuerySweepResult& result) {
        result.success = physx::PxGjkQuery::sweep(a, b, poseA, poseB, unitDir, maxDist, result.t, result.n, result.p);
        return result.success;
    }
};

struct CustomSupport : physx::PxGjkQuery::Support {
    public:
        ~CustomSupport() { }

        virtual float getCustomMargin() = 0;
        virtual void getCustomSupportLocal(const physx::PxVec3& dir, physx::PxVec3& result) = 0;

        virtual physx::PxReal getMargin() const {
            CustomSupport* nonConst = (CustomSupport*) this;
            return nonConst->getCustomMargin();
        }

        virtual physx::PxVec3 supportLocal(const physx::PxVec3& dir) const {
            CustomSupport* nonConst = (CustomSupport*) this;
            nonConst->getCustomSupportLocal(dir, nonConst->supportBuffer);
            return nonConst->supportBuffer;
        }

    private:
        physx::PxVec3 supportBuffer;
};

#endif