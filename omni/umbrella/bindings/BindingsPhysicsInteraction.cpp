// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/Framework.h>
#include <carb/BindingsPythonUtils.h>
#include <carb/dictionary/DictionaryBindingsPython.h>

#include <omni/physics/simulation/IPhysicsInteraction.h>
#include <omni/physics/simulation/simulator/Interaction.h>
#include <omni/physics/simulation/simulator/InteractionTools.h>
#include <pybind11/functional.h>


// Type caster for GetPrimDebugDataFn to/from Python function(const char*)->dict.
namespace pybind11::detail {
template <>
struct type_caster<omni::physics::GetPrimDebugDataFn> {
public:
    PYBIND11_TYPE_CASTER(omni::physics::GetPrimDebugDataFn, _("Callable[[str], dict]"));

    // Python → C++:
    bool load(handle src, bool /*convert*/) {
        if (!src) {
            return false;
        }
        
        // Handle None
        if (src.is_none()) {
            value = nullptr;
            return true;
        }
        
        // Must be callable
        if (!PyCallable_Check(src.ptr())) {
            return false;
        }
        
        // Wrap the Python callable
        py::function pyFunc = py::reinterpret_borrow<py::function>(src);
        
        // Create C++ function that wraps Python function
        value = [pyFunc](const char* prim_path) -> carb::dictionary::Item* {
            py::gil_scoped_acquire acquire;  // Acquire GIL for Python call
            
            try {
                // Call Python function, expecting dict
                py::object result = pyFunc(prim_path);
                
                if (result.is_none()) {
                    return nullptr;
                }
                
                // Convert py::dict to Item*
                auto* dict = carb::dictionary::getDictionary();
                carb::dictionary::Item* item = dict->createItem(nullptr, "", 
                    carb::dictionary::ItemType::eDictionary);
                
                if (py::isinstance<py::dict>(result)) {
                    carb::dictionary::setPyObject(dict, item, "", result);
                }
                
                return item;
            }
            catch (const std::exception& e) {
                PyErr_SetString(PyExc_RuntimeError, e.what());
                return nullptr;
            }
        };
        
        return true;
    }

    // C++ → Python:
    static handle cast(const omni::physics::GetPrimDebugDataFn& src, return_value_policy policy, handle /*parent*/) {
        if (!src) {
            return py::none().release();
        }
        
        // Wrap the C++ function for Python
        auto pyFunc = py::cpp_function(
            [src](const char* prim_path) -> py::object {
                // Call C++ function
                carb::dictionary::Item* item = src(prim_path);
                
                if (!item) {
                    return py::none();
                }
                
                // Convert Item* to py::dict
                auto* dict = carb::dictionary::getDictionary();
                py::object result = carb::dictionary::getPyObject(dict, item);
                
                // Note: Caller is responsible for Item* lifetime management
                
                return result;
            },
            policy
        );
        
        return pyFunc.release();
    }
};
} // namespace pybind11::detail

namespace
{
void bindPhysicsInteraction(py::module& m)
{
    using namespace carb;
    using namespace omni::physics;

    // Bind the DebugDataItemType enum as a class with plain integers.
    // This is necessary for carb dictionary compatibility.
    py::class_<DebugDataItemType> debugDataItemType(m, "DebugDataItemType", R"(
        Debug data item type constants.
        
        Note: type identifiers are stored as plain integers for compatibility with carb dictionaries.
        
        List of types and corresponding value format:
            FLOAT (int): Single floating-point value (0)
            VECTOR (int): 3D vector as a tuple of 3 floats (x, y, z) (1)
            POINT (int): 3D point as a tuple of 3 floats (x, y, z) (2)
            QUATERNION (int): Quaternion rotation as a tuple of 4 floats (x, y, z, w) (3)
            STRING (int): String value (4)
            BOOL (int): Boolean value (5)
            INT (int): Integer value (6)  
            UNDEFINED (int): Undefined or unsupported type (7)
        
        Example:
            >>> debug_data = {
            ...     "Position": {
            ...         "type": DebugDataItemType.POINT,
            ...         "value": (1.0, 2.0, 3.0),
            ...         "doc": "Position in world space"
            ...     }
            ... }
    )");
    
    // Add class attributes as plain integers
    debugDataItemType.attr("FLOAT") = py::int_(static_cast<int>(DebugDataItemType::eFloat));
    debugDataItemType.attr("VECTOR") = py::int_(static_cast<int>(DebugDataItemType::eVector));
    debugDataItemType.attr("POINT") = py::int_(static_cast<int>(DebugDataItemType::ePoint));
    debugDataItemType.attr("QUATERNION") = py::int_(static_cast<int>(DebugDataItemType::eQuaternion));
    debugDataItemType.attr("STRING") = py::int_(static_cast<int>(DebugDataItemType::eString));
    debugDataItemType.attr("BOOL") = py::int_(static_cast<int>(DebugDataItemType::eBool));
    debugDataItemType.attr("INT") = py::int_(static_cast<int>(DebugDataItemType::eInt));
    debugDataItemType.attr("UNDEFINED") = py::int_(static_cast<int>(DebugDataItemType::eUndefined));

    // Define IPhysicsInteraction interface
    defineInterfaceClass<IPhysicsInteraction>(
        m, "IPhysicsInteraction", "acquire_physics_interaction_interface", "release_physics_interaction_interface", R"(
        Interface for controlling physics interaction behavior.
        Provides functionality for managing physics reset behavior and raycast handling.
    )")
        .def("disable_reset_on_stop", wrapInterfaceFunction(&IPhysicsInteraction::disableResetOnStop),
             py::arg("disable"), R"(
            Controls the behavior of the ResetOnStop setting.
            
            Args:
                disable (bool): Disable/enable the reset on stop override.
        )")
        .def("is_disabled_reset_on_stop", wrapInterfaceFunction(&IPhysicsInteraction::isDisabledResetOnStop), R"(
            Returns the current state of the ResetOnStop setting.
            Args:
                simulation_id: Simulation ID
            Returns:
                bool: Disable/enable for the reset on stop.
        )")
        .def(
            "handle_raycast",
            [](IPhysicsInteraction* self, py::object orig, py::object dir, bool input) {
                carb::Float3 orig_array = { 0.0f, 0.0f, 0.0f };
                carb::Float3 dir_array = { 0.0f, 0.0f, 0.0f };
                bool orig_valid = false;
                bool dir_valid = false;

                if (!orig.is_none())
                {
                    auto float3 = orig.cast<carb::Float3>();
                    orig_array = float3;
                    orig_valid = true;
                }

                if (!dir.is_none())
                {
                    auto float3 = dir.cast<carb::Float3>();
                    dir_array = float3;
                    dir_valid = true;
                }

                self->handleRaycast(orig_valid ? &orig_array.x : nullptr, dir_valid ? &dir_array.x : nullptr, input);
            },
            py::arg("orig").none(true), py::arg("dir").none(true), py::arg("input"), R"(
            Called when a raycast request is executed - used for picking.
            
            Args:
                orig (Optional[carb.Float3]): Start position of the raycast, can be None
                dir (Optional[carb.Float3]): Direction of the raycast, can be None
                input (bool): Whether the input control is set or reset (e.g. mouse down)
        )")
        .def("get_prim_debug_data", [](IPhysicsInteraction* self, const char* prim_path) -> py::dict {
            py::dict result;
            carb::dictionary::IDictionary* iDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
            carb::dictionary::Item* debugData = self->getPrimDebugData(prim_path);
            if (!debugData || iDictionary->getItemType(debugData) != carb::dictionary::ItemType::eDictionary)
                return result;

            return getPyObject(iDictionary, debugData);
        }, py::arg("prim_path"), R"(
            Gets debug data for a prim from all active simulations.
            
            Args:
                prim_path (str): The prim path as a string (e.g., "/World/Cube")
                
            Returns:
                dict[str, dict[str, Any]]: Dictionary of simulation debug data for the prim. 
                Each entry is itself a dictionary with the following keys:
                - "type": The type of the debug data item (one of DebugDataItemType)
                - "doc": The documentation string for the debug data item
                - "value": The value of the debug data item (see DebugDataItemType for the value format)
        )")
        ;
}
} // namespace

void bindPhysicsInteraction(py::module& m); 
