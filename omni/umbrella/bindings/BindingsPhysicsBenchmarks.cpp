// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/Framework.h>
#include <carb/BindingsPythonUtils.h>
#include <omni/physics/simulation/IPhysicsBenchmark.h>
#include <pybind11/functional.h>

namespace
{
void bindPhysicsBenchmarks(py::module& m)
{
    using namespace carb;
    using namespace omni::physics;

    // Define PhysicsProfileStats struct
    py::class_<PhysicsProfileStats>(m, "PhysicsProfileStats")
        .def(py::init<>())
        .def_readwrite("zone_name", &PhysicsProfileStats::zoneName, "Name of the profiling zone")
        .def_readwrite("ms", &PhysicsProfileStats::ms, "Time in milliseconds for this zone");

    // Define IPhysicsBenchmarks interface
    defineInterfaceClass<IPhysicsBenchmarks>(m, "IPhysicsBenchmarks", "acquire_physics_benchmarks_interface", "release_physics_benchmarks_interface")
        .def("subscribe_profile_stats_events", [](IPhysicsBenchmarks& self, ProfileStatsNotificationFn callback)
        {
            using namespace std::placeholders;

    return carb::createPySubscription(
                std::move(callback),
                [self](auto fn, void* userData) -> SubscriptionId {
                    auto wrap = [fn, userData](const std::vector<omni::physics::PhysicsProfileStats>& profileStats) {
                        fn(profileStats, userData);
                    };
                    return self.subscribeProfileStatsEvents(wrap);
                },
                [self](SubscriptionId id) {
                    // Release the GIL since unsubscribe can block on a mutex and deadlock
                    py::gil_scoped_release gsr;
                    self.unsubscribeProfileStatsEvents(id);
                });            
        }, py::arg("callback"), 
           "Subscribe to physics simulation profile stats events.\n\n"
           "Note: Subscription cannot be changed in the callback.\n"
           "If subscription is used, getProfileStats will not return any results\n"
           "as the results are cleared after the subscription sends them.\n\n"
           "Args:\n"
           "    callback: The callback function to be called with the profile data.\n\n"
           "Returns:\n"
           "    SubscriptionId for release, kInvalidSubscriptionId if the operation failed");
}
} // namespace

void bindPhysicsBenchmarks(py::module& m); 
