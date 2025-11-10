//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#include ".//physxVehicleControllerAPI.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
#include "pxr/base/tf/pyAnnotatedBoolResult.h"
#include "pxr/base/tf/pyContainerConversions.h"
#include "pxr/base/tf/pyResultConversions.h"
#include "pxr/base/tf/pyUtils.h"
#include "pxr/base/tf/wrapTypeHelpers.h"

#include <boost/python.hpp>

#include <string>

using namespace boost::python;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

#define WRAP_CUSTOM                                                     \
    template <class Cls> static void _CustomWrapCode(Cls &_class)

// fwd decl.
WRAP_CUSTOM;

        
static UsdAttribute
_CreateAcceleratorAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateAcceleratorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateBrake0Attr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBrake0Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateBrake1Attr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBrake1Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateBrakeAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBrakeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateHandbrakeAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateHandbrakeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSteerAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSteerAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSteerLeftAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSteerLeftAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSteerRightAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSteerRightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateTargetGearAttr(PhysxSchemaPhysxVehicleControllerAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTargetGearAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleControllerAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleControllerAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleControllerAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleControllerAPI()
{
    typedef PhysxSchemaPhysxVehicleControllerAPI This;

    PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleControllerAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleControllerAPI");

    cls
        .def(init<UsdPrim>(arg("prim")))
        .def(init<UsdSchemaBase const&>(arg("schemaObj")))
        .def(TfTypePythonClass())

        .def("Get", &This::Get, (arg("stage"), arg("path")))
        .staticmethod("Get")

        .def("CanApply", &_WrapCanApply, (arg("prim")))
        .staticmethod("CanApply")

        .def("Apply", &This::Apply, (arg("prim")))
        .staticmethod("Apply")

        .def("GetSchemaAttributeNames",
             &This::GetSchemaAttributeNames,
             arg("includeInherited")=true,
             return_value_policy<TfPySequenceToList>())
        .staticmethod("GetSchemaAttributeNames")

        .def("_GetStaticTfType", (TfType const &(*)()) TfType::Find<This>,
             return_value_policy<return_by_value>())
        .staticmethod("_GetStaticTfType")

        .def(!self)

        
        .def("GetAcceleratorAttr",
             &This::GetAcceleratorAttr)
        .def("CreateAcceleratorAttr",
             &_CreateAcceleratorAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBrake0Attr",
             &This::GetBrake0Attr)
        .def("CreateBrake0Attr",
             &_CreateBrake0Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBrake1Attr",
             &This::GetBrake1Attr)
        .def("CreateBrake1Attr",
             &_CreateBrake1Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBrakeAttr",
             &This::GetBrakeAttr)
        .def("CreateBrakeAttr",
             &_CreateBrakeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetHandbrakeAttr",
             &This::GetHandbrakeAttr)
        .def("CreateHandbrakeAttr",
             &_CreateHandbrakeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSteerAttr",
             &This::GetSteerAttr)
        .def("CreateSteerAttr",
             &_CreateSteerAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSteerLeftAttr",
             &This::GetSteerLeftAttr)
        .def("CreateSteerLeftAttr",
             &_CreateSteerLeftAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSteerRightAttr",
             &This::GetSteerRightAttr)
        .def("CreateSteerRightAttr",
             &_CreateSteerRightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTargetGearAttr",
             &This::GetTargetGearAttr)
        .def("CreateTargetGearAttr",
             &_CreateTargetGearAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        .def("__repr__", ::_Repr)
    ;

    _CustomWrapCode(cls);
}

// ===================================================================== //
// Feel free to add custom code below this line, it will be preserved by 
// the code generator.  The entry point for your custom code should look
// minimally like the following:
//
// WRAP_CUSTOM {
//     _class
//         .def("MyCustomMethod", ...)
//     ;
// }
//
// Of course any other ancillary or support code may be provided.
// 
// Just remember to wrap code in the appropriate delimiters:
// 'namespace {', '}'.
//
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--

namespace {

WRAP_CUSTOM {
}

}
