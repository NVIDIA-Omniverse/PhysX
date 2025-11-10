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
#include ".//physxCameraFollowAPI.h"
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
_CreateYawAngleAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateYawAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePitchAngleAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePitchAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePitchAngleTimeConstantAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePitchAngleTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSlowSpeedPitchAngleScaleAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSlowSpeedPitchAngleScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSlowPitchAngleSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSlowPitchAngleSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVelocityNormalMinSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVelocityNormalMinSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowMinSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowMinSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowMinDistanceAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowMinDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowMaxSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowMaxSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowMaxDistanceAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowMaxDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateYawRateTimeConstantAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateYawRateTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowTurnRateGainAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowTurnRateGainAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCameraPositionTimeConstantAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCameraPositionTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreatePositionOffsetAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePositionOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateLookAheadMinSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookAheadMinSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookAheadMinDistanceAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookAheadMinDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookAheadMaxSpeedAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookAheadMaxSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookAheadMaxDistanceAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookAheadMaxDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookAheadTurnRateGainAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookAheadTurnRateGainAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookPositionHeightAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookPositionHeightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLookPositionTimeConstantAttr(PhysxSchemaPhysxCameraFollowAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLookPositionTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxCameraFollowAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxCameraFollowAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxCameraFollowAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxCameraFollowAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxCameraFollowAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxCameraFollowAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxCameraFollowAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxCameraFollowAPI()
{
    typedef PhysxSchemaPhysxCameraFollowAPI This;

    PhysxSchemaPhysxCameraFollowAPI_CanApplyResult::Wrap<PhysxSchemaPhysxCameraFollowAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxCameraFollowAPI");

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

        
        .def("GetYawAngleAttr",
             &This::GetYawAngleAttr)
        .def("CreateYawAngleAttr",
             &_CreateYawAngleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPitchAngleAttr",
             &This::GetPitchAngleAttr)
        .def("CreatePitchAngleAttr",
             &_CreatePitchAngleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPitchAngleTimeConstantAttr",
             &This::GetPitchAngleTimeConstantAttr)
        .def("CreatePitchAngleTimeConstantAttr",
             &_CreatePitchAngleTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSlowSpeedPitchAngleScaleAttr",
             &This::GetSlowSpeedPitchAngleScaleAttr)
        .def("CreateSlowSpeedPitchAngleScaleAttr",
             &_CreateSlowSpeedPitchAngleScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSlowPitchAngleSpeedAttr",
             &This::GetSlowPitchAngleSpeedAttr)
        .def("CreateSlowPitchAngleSpeedAttr",
             &_CreateSlowPitchAngleSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVelocityNormalMinSpeedAttr",
             &This::GetVelocityNormalMinSpeedAttr)
        .def("CreateVelocityNormalMinSpeedAttr",
             &_CreateVelocityNormalMinSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowMinSpeedAttr",
             &This::GetFollowMinSpeedAttr)
        .def("CreateFollowMinSpeedAttr",
             &_CreateFollowMinSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowMinDistanceAttr",
             &This::GetFollowMinDistanceAttr)
        .def("CreateFollowMinDistanceAttr",
             &_CreateFollowMinDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowMaxSpeedAttr",
             &This::GetFollowMaxSpeedAttr)
        .def("CreateFollowMaxSpeedAttr",
             &_CreateFollowMaxSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowMaxDistanceAttr",
             &This::GetFollowMaxDistanceAttr)
        .def("CreateFollowMaxDistanceAttr",
             &_CreateFollowMaxDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetYawRateTimeConstantAttr",
             &This::GetYawRateTimeConstantAttr)
        .def("CreateYawRateTimeConstantAttr",
             &_CreateYawRateTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowTurnRateGainAttr",
             &This::GetFollowTurnRateGainAttr)
        .def("CreateFollowTurnRateGainAttr",
             &_CreateFollowTurnRateGainAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCameraPositionTimeConstantAttr",
             &This::GetCameraPositionTimeConstantAttr)
        .def("CreateCameraPositionTimeConstantAttr",
             &_CreateCameraPositionTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPositionOffsetAttr",
             &This::GetPositionOffsetAttr)
        .def("CreatePositionOffsetAttr",
             &_CreatePositionOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookAheadMinSpeedAttr",
             &This::GetLookAheadMinSpeedAttr)
        .def("CreateLookAheadMinSpeedAttr",
             &_CreateLookAheadMinSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookAheadMinDistanceAttr",
             &This::GetLookAheadMinDistanceAttr)
        .def("CreateLookAheadMinDistanceAttr",
             &_CreateLookAheadMinDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookAheadMaxSpeedAttr",
             &This::GetLookAheadMaxSpeedAttr)
        .def("CreateLookAheadMaxSpeedAttr",
             &_CreateLookAheadMaxSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookAheadMaxDistanceAttr",
             &This::GetLookAheadMaxDistanceAttr)
        .def("CreateLookAheadMaxDistanceAttr",
             &_CreateLookAheadMaxDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookAheadTurnRateGainAttr",
             &This::GetLookAheadTurnRateGainAttr)
        .def("CreateLookAheadTurnRateGainAttr",
             &_CreateLookAheadTurnRateGainAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookPositionHeightAttr",
             &This::GetLookPositionHeightAttr)
        .def("CreateLookPositionHeightAttr",
             &_CreateLookPositionHeightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLookPositionTimeConstantAttr",
             &This::GetLookPositionTimeConstantAttr)
        .def("CreateLookPositionTimeConstantAttr",
             &_CreateLookPositionTimeConstantAttr,
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
