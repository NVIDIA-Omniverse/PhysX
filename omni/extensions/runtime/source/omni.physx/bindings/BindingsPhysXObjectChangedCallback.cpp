// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <omni/physx/IPhysx.h>

#include <common/utilities/Utilities.h>
#include <carb/BindingsPythonUtils.h>


void PhysXObjectChangedCallbackBindings(pybind11::class_<omni::physx::IPhysx>& dev)
{
    using PyObjectCreationNotificationFn =
        std::function<void(uint64_t path, int objectId, int physxType)>;
    using PyObjectDestructionNotificationFn =
        std::function<void(uint64_t path, int objectId, int physxType)>;
    using PyAllObjectsDestructionNotificationFn = std::function<void()>;
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;
    const char* docString;
    docString = R"(
        Subscribes to Object Changed Notifications

        Args:
            object_creation_fn:         function(path, object_id, physx_type)   Notification when a physics object gets created during simulation.
            object_destruction_fn:      function(path, object_id, physx_type)   Notification when a physics object gets destroyed during simulation.
            all_objects_destruction_fn: function                                Notification when all physics objects get destroyed during simulation.
            stopCallbackWhenSimStopped: bool                                    Whether these callbacks should not be sent when the simulation stops

        Returns:
            The subscription holder (SubscriptionId) to use with unsubscribe_object_change_notifications
    )";
    dev.def(
        "subscribe_object_changed_notifications",
        [](const IPhysx* self, PyObjectCreationNotificationFn object_creation_fn,
           PyObjectDestructionNotificationFn object_destruction_fn,
           PyAllObjectsDestructionNotificationFn all_objects_destruction_fn,
           bool stopCallbackWhenSimStopped) -> SubscriptionId {
            IPhysicsObjectChangeCallback cb;
            PyObjectCreationNotificationFn wrappedCreateFn;
            if (object_creation_fn)
                wrappedCreateFn = carb::wrapPythonCallback(std::move(object_creation_fn));
            PyObjectDestructionNotificationFn wrappedDestructionFn;
            if (object_destruction_fn)
                wrappedDestructionFn = carb::wrapPythonCallback(std::move(object_destruction_fn));
            PyAllObjectsDestructionNotificationFn wrappedAllDestructionFn;
            if (all_objects_destruction_fn)
                wrappedAllDestructionFn = carb::wrapPythonCallback(std::move(all_objects_destruction_fn));
            cb.objectCreationNotifyFn = [wrappedCreateFn=std::move(wrappedCreateFn)](const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId, PhysXType type,
                                            void*) {
                if (wrappedCreateFn)
                    wrappedCreateFn(asInt(sdfPath), static_cast<int>(objectId), static_cast<int>(type));
            };
            cb.objectDestructionNotifyFn = [wrappedDestructionFn=std::move(wrappedDestructionFn)](const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId,
                                               PhysXType type, void*) {
                if (wrappedDestructionFn)
                    wrappedDestructionFn(asInt(sdfPath), static_cast<int>(objectId), static_cast<int>(type));
            };
            cb.allObjectsDestructionNotifyFn = [wrappedAllDestructionFn=std::move(wrappedAllDestructionFn)](void*) {
                if (wrappedAllDestructionFn)
                    wrappedAllDestructionFn();
            };
            cb.stopCallbackWhenSimStopped = stopCallbackWhenSimStopped;
            SubscriptionId subId = self->subscribeObjectChangeNotifications(cb);
            return subId;
        },
        docString, py::arg("object_creation_fn") = PyObjectCreationNotificationFn(),
        py::arg("object_destruction_fn") = PyObjectDestructionNotificationFn(),
        py::arg("all_objects_destruction_fn") = PyAllObjectsDestructionNotificationFn(),
        py::arg("stop_callback_when_sim_stopped") = true);

    docString = R"(
        Unsubscribes object change notifications with the subscriptionID as returned by subscribe_object_changed_notifications
    )";

    dev.def(
        "unsubscribe_object_change_notifications",
        [](const IPhysx* self, SubscriptionId subId) 
        { 
            self->unsubscribeObjectChangeNotifications(subId); 
        }, docString,
        py::arg("subscription_id"));
}
