// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <omni/physx/IPhysxReplicator.h>

#include <carb/BindingsPythonUtils.h>

inline uint64_t asInt(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(pxr::SdfPath));

    return ret;
}

inline const pxr::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");

    return reinterpret_cast<const pxr::SdfPath&>(path);
}

static void replicatorAttachGlobal(uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData);
static void replicatorAttachEndGlobal(uint64_t stageId, void* userData);
static uint64_t hierarchyRenameGlobal(uint64_t replicatePath, uint32_t index, void* userData);

using PyReplicationAttachFn = std::function<py::list(uint64_t stageId)>;
using PyReplicationAttachEndFn = std::function<void(uint64_t stageId)>;
using PyHierarchyRenameFn = std::function<py::str(const char* replicatePath, uint32_t index)>;

struct CallbackInfo
{
    PyReplicationAttachFn replicatorAttachFn;
    PyReplicationAttachEndFn replicatorAttachEndFn;
    PyHierarchyRenameFn hierarchyRenameFn;
    uint64_t  stageId;    
};
using SubscriberMap = std::unordered_map<size_t, CallbackInfo>;

struct MarshalCallbacks
{
    void subscribeCallback(const omni::physx::IPhysxReplicator* replicator, uint64_t stageId, const PyReplicationAttachFn& replicatorAttach, const PyReplicationAttachEndFn& replicatorAttachEnd, const PyHierarchyRenameFn& hierarchyRename)
    {
        omni::physx::IReplicatorCallback cb;
        cb.replicationAttachFn = replicatorAttachGlobal;
        cb.replicationAttachEndFn = replicatorAttachEndGlobal;
        cb.hierarchyRenameFn = hierarchyRenameGlobal;
        cb.userData = (void*)stageId;
        replicator->registerReplicator(stageId, cb);
        mRegistryMap[stageId] = { replicatorAttach, replicatorAttachEnd, hierarchyRename, stageId };        
    }

    void unsubscribeCallback(const omni::physx::IPhysxReplicator* replicator, uint64_t stageId)
    {
        SubscriberMap::iterator fit = mRegistryMap.find(stageId);
        if (fit != mRegistryMap.end())
        {
            replicator->unregisterReplicator(stageId);
            mRegistryMap.erase(fit);
        }
    }

    const CallbackInfo* getCallbackInfo(uint64_t stageId)
    {
        SubscriberMap::iterator fit = mRegistryMap.find(stageId);
        if (fit != mRegistryMap.end())
        {
            return &fit->second;
        }
        return nullptr;
    }

public:    
    SubscriberMap mRegistryMap;    
};

static MarshalCallbacks gMarshalCallbacks;

void replicatorAttachGlobal(uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
{
    numExludePaths = 0;
    const CallbackInfo* cbInfo = gMarshalCallbacks.getCallbackInfo((size_t)userData);
    if (cbInfo && cbInfo->replicatorAttachFn)
    {
        static py::list excludeList = cbInfo->replicatorAttachFn(stageId);
        static std::vector<uint64_t> pathsList;
        pathsList.resize(excludeList.size());
        for (size_t i = 0; i < excludeList.size(); i++)
        {
            const py::str path = excludeList[i].cast<py::str>();
            pathsList[i] = asInt(pxr::SdfPath(path));
        }

        numExludePaths = uint32_t(pathsList.size());
        excludePaths = pathsList.data();
    }
}

void replicatorAttachEndGlobal(uint64_t stageId, void* userData)
{
    const CallbackInfo* cbInfo = gMarshalCallbacks.getCallbackInfo((size_t)userData);
    if (cbInfo && cbInfo->replicatorAttachEndFn)
    {
        cbInfo->replicatorAttachEndFn(stageId);
    }
}


uint64_t hierarchyRenameGlobal(uint64_t replicatePath, uint32_t index, void* userData)
{
    const CallbackInfo* cbInfo = gMarshalCallbacks.getCallbackInfo((size_t)userData);
    if (cbInfo && cbInfo->hierarchyRenameFn)
    {
        const pxr::SdfPath rPath = intToPath(replicatePath);
        const py::str newPathStr = cbInfo->hierarchyRenameFn(rPath.GetText(), index);
        const std::string newPath = newPathStr;
        return asInt(pxr::SdfPath(newPath));
    }
    return 0;
}

void PhysXReplicatorBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;

    const char* docString;

    auto physxReplicator = defineInterfaceClass<IPhysxReplicator>(
        m, "IPhysxReplicator", "acquire_physx_replicator_interface", "release_physx_replicator_interface");

    m.def("release_physx_replicator_interface_scripting", [](IPhysxReplicator* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });  // OM-60917

    docString = R"(
        Register replicator for given stage.

        stage_id: current stageId

        replicator_attach_fn: std::function<py::list(uint64_t stageId)>;

        replicator_attach_end_fn: std::function<py::list(uint64_t stageId)>;

        hierarchy_rename_fn: std::function<const char* (const char* replicatePath, uint32_t index)>;
    )";
    physxReplicator.def("register_replicator",
        [](const IPhysxReplicator* self,
            uint64_t stageId,
            PyReplicationAttachFn replicator_attach_fn,
            PyReplicationAttachEndFn replicator_attach_end_fn,
            PyHierarchyRenameFn hierarchy_rename_fn)
        {
            return gMarshalCallbacks.subscribeCallback(self, stageId, replicator_attach_fn, replicator_attach_end_fn, hierarchy_rename_fn);
        }, docString,
        py::arg("stage_id"), py::arg("replicator_attach_fn"), py::arg("replicator_attach_end_fn"), py::arg("hierarchy_rename_fn"));

    docString = R"(
        Unregister replicator from stage.
    )";
    physxReplicator.def("unregister_replicator",
        [](const IPhysxReplicator* self,
            uint64_t stageId)
        {
            gMarshalCallbacks.unsubscribeCallback(self, stageId);
        }, docString,
        py::arg("stage_id"));

    docString = R"(
        Replicate hierarchy.

        Args:
            stage_id: stage id

            path: path to replicate

            numReplications: number of replications

            useEnvIds: setup EnvIds. See below.

            useFabricForReplication: replicate hierarchy through Fabric

        Environment IDs are a more efficient way to filter out independent parts of a scene (the different environments). When enabled, filtering is automatically done early within the simulation pipeline, without the need to specify extra filtering parameters like collision groups, etc. EnvIDs also potentially make co-located envs faster.

        When using environment IDs (i.e. useEnvIds = True), it is also possible to encode these IDs into the bounds of the GPU broadphase to improve its performance. This is controlled by a custom attribute of the UsdPhysicsScene (physxScene:envIdInBoundsBitCount). This defines the number of bits reserved for environment IDs in the encoded bounds of the GPU broadphase. Set it to -1 to disable that feature (default). Set it to a positive number in [1;16] to enable the feature (4 or 8 are good values for most cases). Additionally, setting it to 0 disables the feature but improves the accuracy of GPU bounds, which can lead to better performance when objects are located far away from the origin.
    )";
    physxReplicator.def("replicate",
        [](const IPhysxReplicator* self, uint64_t stageId, const char* path, uint32_t numReplications, bool useEnvIds,
           bool useFabricForReplication)
        {
            return self->replicate(
                stageId, asInt(pxr::SdfPath(path)), numReplications, useEnvIds, useFabricForReplication);
        },
        docString, py::arg("stage_id"), py::arg("path"), py::arg("numReplications"), py::arg("useEnvIds") = false,
        py::arg("useFabricForReplication") = false);

}
