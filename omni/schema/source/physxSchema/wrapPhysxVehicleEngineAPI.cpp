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
#include ".//physxVehicleEngineAPI.h"
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
_CreateMoiAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMoiAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePeakTorqueAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePeakTorqueAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxRotationSpeedAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxRotationSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateIdleRotationSpeedAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateIdleRotationSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateTorqueCurveAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTorqueCurveAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float2Array), writeSparsely);
}
        
static UsdAttribute
_CreateDampingRateFullThrottleAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingRateFullThrottleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDampingRateZeroThrottleClutchEngagedAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingRateZeroThrottleClutchEngagedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDampingRateZeroThrottleClutchDisengagedAttr(PhysxSchemaPhysxVehicleEngineAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingRateZeroThrottleClutchDisengagedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleEngineAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleEngineAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleEngineAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleEngineAPI()
{
    typedef PhysxSchemaPhysxVehicleEngineAPI This;

    PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleEngineAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleEngineAPI");

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

        
        .def("GetMoiAttr",
             &This::GetMoiAttr)
        .def("CreateMoiAttr",
             &_CreateMoiAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPeakTorqueAttr",
             &This::GetPeakTorqueAttr)
        .def("CreatePeakTorqueAttr",
             &_CreatePeakTorqueAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxRotationSpeedAttr",
             &This::GetMaxRotationSpeedAttr)
        .def("CreateMaxRotationSpeedAttr",
             &_CreateMaxRotationSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetIdleRotationSpeedAttr",
             &This::GetIdleRotationSpeedAttr)
        .def("CreateIdleRotationSpeedAttr",
             &_CreateIdleRotationSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTorqueCurveAttr",
             &This::GetTorqueCurveAttr)
        .def("CreateTorqueCurveAttr",
             &_CreateTorqueCurveAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingRateFullThrottleAttr",
             &This::GetDampingRateFullThrottleAttr)
        .def("CreateDampingRateFullThrottleAttr",
             &_CreateDampingRateFullThrottleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingRateZeroThrottleClutchEngagedAttr",
             &This::GetDampingRateZeroThrottleClutchEngagedAttr)
        .def("CreateDampingRateZeroThrottleClutchEngagedAttr",
             &_CreateDampingRateZeroThrottleClutchEngagedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingRateZeroThrottleClutchDisengagedAttr",
             &This::GetDampingRateZeroThrottleClutchDisengagedAttr)
        .def("CreateDampingRateZeroThrottleClutchDisengagedAttr",
             &_CreateDampingRateZeroThrottleClutchDisengagedAttr,
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
