// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <carb/BindingsPythonUtils.h>
#include "omni/convexdecomposition/ConvexDecomposition.h"
#include <unordered_map>
#include <vector>
#include <functional>
#include <mutex>

CARB_BINDINGS("omni.convexdecomposition.python")

// Scoped mutex lock
using lock_guard = std::lock_guard<std::mutex>;

namespace
{

using notify_complete_callback_function = std::function<void(omni::convexdecomposition::VHACDHANDLE)>;

// This class is used to marshal the Python version of a 'SimpleMesh'. The Python version uses an std::vector
struct MarshalSimpleMesh
{
    std::vector<double> vertices;
    std::vector<uint32_t> indices;
    double volume{}; // optional volume of the mesh
    carb::Double3 center{}; // optional center of mesh
};

struct MarshalParameters
{
    double errorPercentage{ 10 }; // Error percentage threshold
    uint32_t maxHullVertices{ 60 }; // Maximum number of vertices in output convex hull
    uint32_t maxConvexHullCount{ 64 }; // Maximum number of convex hulls to produce.
    uint32_t voxelResolution{ 500000 }; // Approximate number of voxels to use, default value is one million.
    omni::convexdecomposition::VoxelFillMode voxelFillMode{omni::convexdecomposition::VoxelFillMode::eFloodFill}; // Fill mode type to use
    notify_complete_callback_function notifyCompleteCallback{};
};


struct ConvexDecompositionCallbacks
{
    notify_complete_callback_function notifyCompleteCallback{};
};

typedef std::unordered_map<omni::convexdecomposition::VHACDHANDLE, ConvexDecompositionCallbacks> ConvexDecompositionCallbacksMap;

struct MarshalCallbacks
{

    void registerCallback(omni::convexdecomposition::VHACDHANDLE handle,const notify_complete_callback_function& _notifyCompleteCallback)
    {
        lock_guard _lock(mMutex);
        ConvexDecompositionCallbacksMap::iterator found = mCallbacks.find(handle);
        if (found == mCallbacks.end())
        {
            ConvexDecompositionCallbacks cdc;
            cdc.notifyCompleteCallback = _notifyCompleteCallback;
            mCallbacks[handle] = cdc;
        }
        else
        {
            ConvexDecompositionCallbacks& cdc = (*found).second;
            cdc.notifyCompleteCallback = _notifyCompleteCallback;
        }
    }

    void deregisterCallback(omni::convexdecomposition::VHACDHANDLE handle)
    {
        lock_guard _lock(mMutex);
        ConvexDecompositionCallbacksMap::iterator found = mCallbacks.find(handle);
        if (found != mCallbacks.end())
        {
            mCallbacks.erase(found);
        }
    }

    void notifyCompleteCallback(omni::convexdecomposition::VHACDHANDLE handle)
    {
        lock_guard _lock(mMutex);
        ConvexDecompositionCallbacksMap::iterator found = mCallbacks.find(handle);
        if (found != mCallbacks.end())
        {
            ConvexDecompositionCallbacks& cdc = (*found).second;
            if (cdc.notifyCompleteCallback)
            {
                //printf("PtyhonCallback:NotifyComplete\n");
                carb::StdFuncUtils<decltype(cdc.notifyCompleteCallback)>::callPythonCodeSafe(
                    cdc.notifyCompleteCallback, handle);
            }
        }
    }

    std::mutex mMutex;
    ConvexDecompositionCallbacksMap mCallbacks;
};

static MarshalCallbacks gMarshalCallbacks;

void NotifyVHACDComplete(omni::convexdecomposition::VHACDHANDLE id, void* userPtr)
{
    gMarshalCallbacks.notifyCompleteCallback(id);
}

PYBIND11_MODULE(_convexdecomposition, m)
{
    using namespace carb;
    using namespace omni::convexdecomposition;

    m.doc() = "pybind11 omni.convexdecomposition bindings";

    py::class_<MarshalSimpleMesh>(m, "SimpleMesh")
        .def(py::init<>())
        .def_readwrite("vertices", &MarshalSimpleMesh::vertices)
        .def_readwrite("indices", &MarshalSimpleMesh::indices)
        .def_readwrite("volume", &MarshalSimpleMesh::volume)
        .def_readwrite("center", &MarshalSimpleMesh::center);


    py::enum_<VoxelFillMode>(m, "VoxelFillMode")
        .value("FLOOD_FILL", VoxelFillMode::eFloodFill)
        .value("SURFACE_ONLY", VoxelFillMode::eSurfaceOnly)
        .value("RAYCAST_FILL", VoxelFillMode::eRaycastFill);

    py::class_<MarshalParameters>(m, "Parameters")
        .def(py::init<>())
        .def_readwrite("error_percentage", &MarshalParameters::errorPercentage)
        .def_readwrite("max_hull_vertices", &MarshalParameters::maxHullVertices)
        .def_readwrite("max_convex_hull_count", &MarshalParameters::maxConvexHullCount)
        .def_readwrite("voxel_resolution", &MarshalParameters::voxelResolution)
        .def_readwrite("voxel_fill_mode", &MarshalParameters::voxelFillMode)
        .def_readwrite("notify_complete_callback", &MarshalParameters::notifyCompleteCallback);

    m.def("release_convexdecomposition_interface_scripting", [](ConvexDecomposition* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });  // OM-60917
    defineInterfaceClass<ConvexDecomposition>(
        m, "ConvexDecomposition", "acquire_convexdecomposition_interface", "release_convexdecomposition_interface")
        .def("create_vhacd", wrapInterfaceFunction(&ConvexDecomposition::createVHACD))
        .def("release_vhacd",
             [](ConvexDecomposition* cd, VHACDHANDLE handle) { gMarshalCallbacks.deregisterCallback(handle); },
             py::arg("handle"))
        .def("begin_vhacd",
             [](ConvexDecomposition* cd, VHACDHANDLE handle, const MarshalSimpleMesh* smm, const MarshalParameters* mp) {
                 bool ret = false;

                 if (smm && !smm->vertices.empty() && !smm->indices.empty() && mp)
                 {
                     gMarshalCallbacks.registerCallback(handle, mp->notifyCompleteCallback);
                     uint32_t vcount = uint32_t(smm->vertices.size()) / 3;
                     uint32_t tcount = uint32_t(smm->indices.size()) / 3;
                     omni::convexdecomposition::Parameters p;
                     p.notifyCompleteCallback = NotifyVHACDComplete;
                     p.errorPercentage = mp->errorPercentage;
                     p.maxHullVertices = mp->maxHullVertices;
                     p.maxConvexHullCount = mp->maxConvexHullCount;
                     p.voxelResolution = mp->voxelResolution;
                     p.voxelFillMode = mp->voxelFillMode;
                     SimpleMesh sm;
                     sm.triangleCount = tcount;
                     sm.vertexCount = vcount;
                     sm.vertices = smm->vertices.data();
                     sm.indices = smm->indices.data();
                     ret = cd->beginVHACD(handle, p, sm);
                 }
                 return ret;
             },
             py::arg("handle"), py::arg("smm"), py::arg("mp"))
        .def("apply_sphere_approximation", wrapInterfaceFunction(&ConvexDecomposition::applySphereApproximation))
        .def("is_complete", wrapInterfaceFunction(&ConvexDecomposition::isComplete))
        .def("get_convex_hull_count", wrapInterfaceFunction(&ConvexDecomposition::getConvexHullCount))
        .def("get_convex_hull",
             [](ConvexDecomposition* cd, omni::convexdecomposition::VHACDHANDLE handle, uint32_t index,
                MarshalSimpleMesh* smm) {
                 omni::convexdecomposition::SimpleMesh hr;
                 bool ret = cd->getConvexHull(handle, index, hr);
                 if (ret)
                 {
                     smm->vertices.clear();
                     for (uint32_t i = 0; i < hr.vertexCount * 3; i++)
                     {
                         smm->vertices.push_back(hr.vertices[i]);
                     }
                     smm->indices.clear();
                     for (uint32_t i = 0; i < hr.triangleCount * 3; i++)
                     {
                         smm->indices.push_back(hr.indices[i]);
                     }
                     smm->center = hr.center;
                     smm->volume = hr.volume;
                 }
                 return ret;
             },
             py::arg("handle"), py::arg("index"), py::arg("smm"))
        .def("cancel_vhacd",
             [](ConvexDecomposition* cd, omni::convexdecomposition::VHACDHANDLE handle) {
                 bool ret = cd->cancelVHACD(handle);
                 return ret;
             },
             py::arg("handle"))
        .def("save_obj",
             [](ConvexDecomposition* cd, const char* fname, const MarshalSimpleMesh* smm, bool flipWindingOrder) {
                 bool ret = false;
                 if (smm && !smm->vertices.empty() && !smm->indices.empty())
                 {
                     uint32_t vcount = uint32_t(smm->vertices.size()) / 3;
                     uint32_t tcount = uint32_t(smm->indices.size()) / 3;
                     omni::convexdecomposition::Parameters p;
                     SimpleMesh sm;
                     sm.triangleCount = tcount;
                     sm.vertexCount = vcount;
                     sm.vertices = smm->vertices.data();
                     sm.indices = smm->indices.data();
                     ret = cd->saveOBJ(fname, sm, flipWindingOrder);
                 }
                 return ret;
             },
             py::arg("fname"), py::arg("smm"), py::arg("flip_winding_order"));

}
} // namespace
