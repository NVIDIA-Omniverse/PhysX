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
#include ".//physxCameraDroneAPI.h"
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
_CreateFollowHeightAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowHeightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowDistanceAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxDistanceAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxSpeedAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateHorizontalVelocityGainAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateHorizontalVelocityGainAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVerticalVelocityGainAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVerticalVelocityGainAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFeedForwardVelocityGainAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFeedForwardVelocityGainAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVelocityFilterTimeConstantAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVelocityFilterTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateRotationFilterTimeConstantAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRotationFilterTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePositionOffsetAttr(PhysxSchemaPhysxCameraDroneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePositionOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxCameraDroneAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxCameraDroneAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxCameraDroneAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxCameraDroneAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxCameraDroneAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxCameraDroneAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxCameraDroneAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxCameraDroneAPI()
{
    typedef PhysxSchemaPhysxCameraDroneAPI This;

    PhysxSchemaPhysxCameraDroneAPI_CanApplyResult::Wrap<PhysxSchemaPhysxCameraDroneAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxCameraDroneAPI");

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

        
        .def("GetFollowHeightAttr",
             &This::GetFollowHeightAttr)
        .def("CreateFollowHeightAttr",
             &_CreateFollowHeightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowDistanceAttr",
             &This::GetFollowDistanceAttr)
        .def("CreateFollowDistanceAttr",
             &_CreateFollowDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxDistanceAttr",
             &This::GetMaxDistanceAttr)
        .def("CreateMaxDistanceAttr",
             &_CreateMaxDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxSpeedAttr",
             &This::GetMaxSpeedAttr)
        .def("CreateMaxSpeedAttr",
             &_CreateMaxSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetHorizontalVelocityGainAttr",
             &This::GetHorizontalVelocityGainAttr)
        .def("CreateHorizontalVelocityGainAttr",
             &_CreateHorizontalVelocityGainAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVerticalVelocityGainAttr",
             &This::GetVerticalVelocityGainAttr)
        .def("CreateVerticalVelocityGainAttr",
             &_CreateVerticalVelocityGainAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFeedForwardVelocityGainAttr",
             &This::GetFeedForwardVelocityGainAttr)
        .def("CreateFeedForwardVelocityGainAttr",
             &_CreateFeedForwardVelocityGainAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVelocityFilterTimeConstantAttr",
             &This::GetVelocityFilterTimeConstantAttr)
        .def("CreateVelocityFilterTimeConstantAttr",
             &_CreateVelocityFilterTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRotationFilterTimeConstantAttr",
             &This::GetRotationFilterTimeConstantAttr)
        .def("CreateRotationFilterTimeConstantAttr",
             &_CreateRotationFilterTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPositionOffsetAttr",
             &This::GetPositionOffsetAttr)
        .def("CreatePositionOffsetAttr",
             &_CreatePositionOffsetAttr,
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
