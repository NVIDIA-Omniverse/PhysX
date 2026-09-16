// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// NVBug 6532970: OVStage scene-graph instancing attach cost with a fixed physics workload.
// Population and sealing happen before the timer. The measured region is the
// initial ovphysx attach plus its completion wait. Ordinal 1 is not replayed as
// an update. Each fixture has five non-instanced leaf colliders and a varying
// number of unrelated render-only prototype/instance pairs.
//
// The `flat` control rows keep composition and prim count identical but set
// `instanceable = false`. If a flat row costs the same as its instanced
// sibling, the cost is not specific to scene-graph instancing.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"

#include <ovphysx/ovphysx.h>
#include <ovphysx/experimental/ovphysx.hpp>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>


void initInstancingAttach()
{
}

namespace
{

constexpr uint32_t kPhysicsBodyCount = 5;
constexpr ovstage_ordinal_t kInitialOrdinal = 1;

constexpr char kMeshAttributes[] = R"usda(        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 1, 3, 2, 4, 6, 7, 5, 6, 2, 3, 7, 4, 5, 1, 0, 4, 0, 2, 6, 5, 7, 3, 1]
        point3f[] points = [(-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5),
            (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5)]
        uniform token subdivisionScheme = "none"
)usda";

std::string buildInstancingFixture(uint32_t prototypeCount, bool instanceable)
{
    std::ostringstream usda;
    usda << R"usda(#usda 1.0
(
    metersPerUnit = 1
    upAxis = "Z"
)

def PhysicsScene "PhysicsScene"
{
    vector3f physics:gravityDirection = (0, 0, -1)
    float physics:gravityMagnitude = 9.81
}
)usda";

    for (uint32_t i = 0; i < kPhysicsBodyCount; ++i)
    {
        usda << "\ndef Xform \"Body" << i << "\"\n"
             << "{\n"
             << "    double3 xformOp:translate = (0, 0, " << (2 + i * 2) << ")\n"
             << "    uniform token[] xformOpOrder = [\"xformOp:translate\"]\n\n"
             << "    def Mesh \"Geom\" (\n"
             << "        prepend apiSchemas = [\"PhysicsCollisionAPI\", \"PhysicsRigidBodyAPI\"]\n"
             << "    )\n"
             << "    {\n"
             << kMeshAttributes << "    }\n"
             << "}\n";
    }

    for (uint32_t i = 0; i < prototypeCount; ++i)
    {
        usda << "\nclass Xform \"ProtoSrc" << i << "\"\n"
             << "{\n"
             << "    def Mesh \"Geom\"\n"
             << "    {\n"
             << kMeshAttributes << "    }\n"
             << "}\n";
    }

    for (uint32_t i = 0; i < prototypeCount; ++i)
    {
        usda << "\ndef Xform \"Inst" << i << "\" (\n"
             << "    instanceable = " << (instanceable ? "true" : "false") << "\n"
             << "    prepend references = </ProtoSrc" << i << ">\n"
             << ")\n"
             << "{\n"
             << "    double3 xformOp:translate = (" << (10 + i) << ", 0, 5)\n"
             << "    uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
             << "}\n";
    }

    return usda.str();
}

std::string ovphysxError(const char* operation)
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    const std::string message = error.ptr ? std::string(error.ptr, error.length) : std::string();
    return std::string(operation) + " failed: " + message;
}

std::string populationError(const char* operation)
{
    const ovx_string_t error = ovstage_population_get_last_error();
    const std::string message = error.ptr ? std::string(error.ptr, error.length) : std::string();
    return std::string(operation) + " failed: " + message;
}

class InstancingAttachBase : public BmBenchmark
{
public:
    InstancingAttachBase(uint32_t prototypeCount, uint32_t populationDomains, bool instanceable = true)
        : mUsda(buildInstancingFixture(prototypeCount, instanceable)), mPopulationDomains(populationDomains)
    {
    }

    ~InstancingAttachBase() override
    {
        if (!mStage)
            return;
        if (mAttached && mPhysX)
        {
            const ovphysx_result_t detach = ovphysx_detach_ovstage(mPhysX->handle());
            // An attached Stage remains caller-owned. Retain it if detach fails
            // rather than destroying storage still referenced by ovphysx.
            if (detach.status != OVPHYSX_API_SUCCESS)
                return;
        }
        ovstage_destroy_instance(mStage);
    }

    bool isValid() const override
    {
        if (BmGlobals::getInstance().forceGpu())
            return false;
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override
    {
        return 1;
    }
    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
    }

    void preStep() override
    {
        if (mStage)
            cleanup();
        prepareStage();
    }

    Time::Second timedStep() override
    {
        Time timer;
        const ovphysx_result_t attach = ovphysx_attach_ovstage(mPhysX->handle(), mStage, kInitialOrdinal);
        if (attach.status != OVPHYSX_API_SUCCESS)
            throw std::runtime_error(ovphysxError("ovphysx_attach_ovstage"));
        mAttached = true;

        ovphysx_op_wait_result_t waitResult{};
        const ovphysx_result_t waitStatus =
            ovphysx_wait_op(mPhysX->handle(), OVPHYSX_OP_INDEX_ALL, UINT64_MAX, &waitResult);
        const Time::Second elapsed = timer.getElapsedSeconds();

        std::string waitError;
        if (waitStatus.status != OVPHYSX_API_SUCCESS || waitResult.num_errors != 0)
        {
            waitError = ovphysxError("ovphysx_wait_op");
            // A failed operation sets the aggregate status as well as num_errors,
            // so the specific per-op message is only reachable from inside this
            // branch. Read it after ovphysxError() has copied the aggregate text.
            if (waitResult.num_errors != 0 && waitResult.error_op_indices)
            {
                const ovphysx_string_t error = ovphysx_get_last_op_error(waitResult.error_op_indices[0]);
                if (error.ptr)
                    waitError += " (op " + std::to_string(waitResult.error_op_indices[0]) + ": " +
                                 std::string(error.ptr, error.length) + ")";
            }
        }
        ovphysx_destroy_wait_result(&waitResult);
        if (!waitError.empty())
            throw std::runtime_error(waitError);

        validateBodyCount();
        return elapsed;
    }

    void endRun() override
    {
        cleanup();
    }

protected:
    void step() override
    {
    }

private:
    void prepareStage()
    {
        mPhysX = BmGlobals::getInstance().getPhysX();

        // ovphysx ships its PhysX schemas as codeless data and never registers
        // them. The application does, before the first population in the process.
        ovphysx_string_t schemaRoot{};
        if (ovphysx_get_codeless_schema_root(&schemaRoot).status != OVPHYSX_API_SUCCESS)
        {
            throw std::runtime_error("ovphysx_get_codeless_schema_root failed");
        }
        const ovx_string_t schemaPath{ schemaRoot.ptr, schemaRoot.length };
        if (ovstage_population_register_usd_schemas(&schemaPath, 1) != OVSTAGE_OK)
        {
            failPreparation(populationError("ovstage_population_register_usd_schemas"));
        }

        ovstage_instance_desc_t desc{};
        desc.name = "ovphysx-instancing-attach-benchmark";
        const ovstage_api_status_t createStatus = ovstage_create_instance(&desc, &mStage);
        if (createStatus != OVSTAGE_OK || !mStage)
        {
            if (mStage)
            {
                ovstage_destroy_instance(mStage);
                mStage = nullptr;
            }
            throw std::runtime_error("ovstage_create_instance failed");
        }

        const ovx_string_t usda{ mUsda.data(), mUsda.size() };
        const ovstage_population_enqueue_result_t population =
            ovstage_population_open_usd_from_string(mStage, usda, kInitialOrdinal, 0.0, mPopulationDomains);
        if (population.status != OVSTAGE_OK || population.op_index == OVSTAGE_POPULATION_INVALID_OP_ID)
        {
            failPreparation(populationError("ovstage_population_open_usd_from_string"));
        }

        ovstage_population_op_wait_result_t populationWait{};
        const ovstage_api_status_t populationStatus =
            ovstage_population_wait_op(mStage, population.op_index, OVSTAGE_TIMEOUT_INFINITE, &populationWait);
        if (populationStatus != OVSTAGE_OK || populationWait.error_op_id_count != 0)
            failPreparation(populationError("ovstage_population_wait_op"));

        ovstage_write_floor_desc_t floorDesc{};
        floorDesc.ordinal = kInitialOrdinal;
        floorDesc.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(mStage, &floorDesc);
        if (floor.status != OVSTAGE_OK || floor.op_index == OVSTAGE_INVALID_OP_ID)
            failPreparation("ovstage_advance_write_floor failed");

        ovstage_op_wait_result_t floorWait{};
        const ovstage_api_status_t floorStatus =
            ovstage_wait_op(mStage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floorWait);
        if (floorStatus != OVSTAGE_OK || floorWait.error_op_id_count != 0)
        {
            (void)ovstage_release_op(mStage, floor.op_index);
            failPreparation("ovstage_wait_op for write floor failed");
        }
        if (ovstage_release_op(mStage, floor.op_index) != OVSTAGE_OK)
            failPreparation("ovstage_release_op for write floor failed");
    }

    void validateBodyCount()
    {
        ovphysx_query_handle_t query = 0;
        const ovphysx_result_t queryStatus =
            ovphysx_query(mPhysX->handle(), OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query);
        if (queryStatus.status != OVPHYSX_API_SUCCESS || query == 0)
            throw std::runtime_error(ovphysxError("ovphysx_query"));

        ovstage_query_result_t result{};
        const ovphysx_result_t fetchStatus = ovphysx_fetch_query_result(mPhysX->handle(), query, &result);
        const std::string fetchError =
            fetchStatus.status == OVPHYSX_API_SUCCESS ? std::string() : ovphysxError("ovphysx_fetch_query_result");
        const size_t bodyCount = result.total_prim_count;
        const ovphysx_result_t releaseStatus = ovphysx_release_query(mPhysX->handle(), query);
        if (fetchStatus.status != OVPHYSX_API_SUCCESS)
            throw std::runtime_error(fetchError);
        if (releaseStatus.status != OVPHYSX_API_SUCCESS)
            throw std::runtime_error(ovphysxError("ovphysx_release_query"));
        if (bodyCount != kPhysicsBodyCount)
        {
            throw std::runtime_error("InstancingAttach fixture realized " + std::to_string(bodyCount) +
                                     " rigid bodies; expected " + std::to_string(kPhysicsBodyCount));
        }
    }

    void cleanup()
    {
        if (!mStage)
            return;
        if (mAttached)
        {
            const ovphysx_result_t detach = ovphysx_detach_ovstage(mPhysX->handle());
            if (detach.status != OVPHYSX_API_SUCCESS)
                throw std::runtime_error(ovphysxError("ovphysx_detach_ovstage"));
            mAttached = false;
        }
        ovstage_destroy_instance(mStage);
        mStage = nullptr;
    }

    [[noreturn]] void failPreparation(const std::string& message)
    {
        ovstage_destroy_instance(mStage);
        mStage = nullptr;
        throw std::runtime_error(message);
    }

    std::string mUsda;
    uint32_t mPopulationDomains = OVSTAGE_POPULATION_DOMAIN_NONE;
    ovphysx::PhysX* mPhysX = nullptr;
    ovstage_instance_t* mStage = nullptr;
    bool mAttached = false;
};

class InstancingAttach_PhysicsN0100 : public InstancingAttachBase
{
public:
    InstancingAttach_PhysicsN0100() : InstancingAttachBase(100, OVSTAGE_POPULATION_DOMAIN_PHYSICS)
    {
    }
};

class InstancingAttach_PhysicsN0250 : public InstancingAttachBase
{
public:
    InstancingAttach_PhysicsN0250() : InstancingAttachBase(250, OVSTAGE_POPULATION_DOMAIN_PHYSICS)
    {
    }
};

class InstancingAttach_PhysicsN0500 : public InstancingAttachBase
{
public:
    InstancingAttach_PhysicsN0500() : InstancingAttachBase(500, OVSTAGE_POPULATION_DOMAIN_PHYSICS)
    {
    }
};

class InstancingAttach_AllN0100 : public InstancingAttachBase
{
public:
    InstancingAttach_AllN0100() : InstancingAttachBase(100, OVSTAGE_POPULATION_DOMAIN_ALL)
    {
    }
};

class InstancingAttach_AllN0250 : public InstancingAttachBase
{
public:
    InstancingAttach_AllN0250() : InstancingAttachBase(250, OVSTAGE_POPULATION_DOMAIN_ALL)
    {
    }
};

class InstancingAttach_AllN0500 : public InstancingAttachBase
{
public:
    InstancingAttach_AllN0500() : InstancingAttachBase(500, OVSTAGE_POPULATION_DOMAIN_ALL)
    {
    }
};

// Control rows: identical composition and populated prim count, but
// `instanceable = false`. They separate instancing-specific attach cost from
// cost that merely scales with the number of populated render-only prims.
class InstancingAttach_PhysicsFlatN0500 : public InstancingAttachBase
{
public:
    InstancingAttach_PhysicsFlatN0500() : InstancingAttachBase(500, OVSTAGE_POPULATION_DOMAIN_PHYSICS, false)
    {
    }
};

class InstancingAttach_AllFlatN0500 : public InstancingAttachBase
{
public:
    InstancingAttach_AllFlatN0500() : InstancingAttachBase(500, OVSTAGE_POPULATION_DOMAIN_ALL, false)
    {
    }
};

Register<InstancingAttach_PhysicsN0100, true> sInstancingAttachPhysicsN0100("InstancingAttach.physics_n0100_cpu");
Register<InstancingAttach_PhysicsN0250, true> sInstancingAttachPhysicsN0250("InstancingAttach.physics_n0250_cpu");
Register<InstancingAttach_PhysicsN0500, true> sInstancingAttachPhysicsN0500("InstancingAttach.physics_n0500_cpu");
Register<InstancingAttach_AllN0100, true> sInstancingAttachAllN0100("InstancingAttach.all_n0100_cpu");
Register<InstancingAttach_AllN0250, true> sInstancingAttachAllN0250("InstancingAttach.all_n0250_cpu");
Register<InstancingAttach_AllN0500, true> sInstancingAttachAllN0500("InstancingAttach.all_n0500_cpu");
Register<InstancingAttach_PhysicsFlatN0500, true> sInstancingAttachPhysicsFlatN0500(
    "InstancingAttach.physics_flat_n0500_cpu");
Register<InstancingAttach_AllFlatN0500, true> sInstancingAttachAllFlatN0500("InstancingAttach.all_flat_n0500_cpu");

} // namespace
