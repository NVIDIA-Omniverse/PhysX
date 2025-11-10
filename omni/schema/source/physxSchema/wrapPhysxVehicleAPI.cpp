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
#include ".//physxVehicleAPI.h"
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
_CreateVehicleEnabledAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVehicleEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSuspensionLineQueryTypeAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSuspensionLineQueryTypeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateLimitSuspensionExpansionVelocityAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLimitSuspensionExpansionVelocityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSubStepThresholdLongitudinalSpeedAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSubStepThresholdLongitudinalSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLowForwardSpeedSubStepCountAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLowForwardSpeedSubStepCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateHighForwardSpeedSubStepCountAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateHighForwardSpeedSubStepCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMinLongitudinalSlipDenominatorAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinLongitudinalSlipDenominatorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMinPassiveLongitudinalSlipDenominatorAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinPassiveLongitudinalSlipDenominatorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMinActiveLongitudinalSlipDenominatorAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinActiveLongitudinalSlipDenominatorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMinLateralSlipDenominatorAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinLateralSlipDenominatorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalStickyTireThresholdSpeedAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalStickyTireThresholdSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalStickyTireThresholdTimeAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalStickyTireThresholdTimeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalStickyTireDampingAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalStickyTireDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLateralStickyTireThresholdSpeedAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLateralStickyTireThresholdSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLateralStickyTireThresholdTimeAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLateralStickyTireThresholdTimeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLateralStickyTireDampingAttr(PhysxSchemaPhysxVehicleAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLateralStickyTireDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleAPI()
{
    typedef PhysxSchemaPhysxVehicleAPI This;

    PhysxSchemaPhysxVehicleAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleAPI");

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

        
        .def("GetVehicleEnabledAttr",
             &This::GetVehicleEnabledAttr)
        .def("CreateVehicleEnabledAttr",
             &_CreateVehicleEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSuspensionLineQueryTypeAttr",
             &This::GetSuspensionLineQueryTypeAttr)
        .def("CreateSuspensionLineQueryTypeAttr",
             &_CreateSuspensionLineQueryTypeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLimitSuspensionExpansionVelocityAttr",
             &This::GetLimitSuspensionExpansionVelocityAttr)
        .def("CreateLimitSuspensionExpansionVelocityAttr",
             &_CreateLimitSuspensionExpansionVelocityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSubStepThresholdLongitudinalSpeedAttr",
             &This::GetSubStepThresholdLongitudinalSpeedAttr)
        .def("CreateSubStepThresholdLongitudinalSpeedAttr",
             &_CreateSubStepThresholdLongitudinalSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLowForwardSpeedSubStepCountAttr",
             &This::GetLowForwardSpeedSubStepCountAttr)
        .def("CreateLowForwardSpeedSubStepCountAttr",
             &_CreateLowForwardSpeedSubStepCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetHighForwardSpeedSubStepCountAttr",
             &This::GetHighForwardSpeedSubStepCountAttr)
        .def("CreateHighForwardSpeedSubStepCountAttr",
             &_CreateHighForwardSpeedSubStepCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinLongitudinalSlipDenominatorAttr",
             &This::GetMinLongitudinalSlipDenominatorAttr)
        .def("CreateMinLongitudinalSlipDenominatorAttr",
             &_CreateMinLongitudinalSlipDenominatorAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinPassiveLongitudinalSlipDenominatorAttr",
             &This::GetMinPassiveLongitudinalSlipDenominatorAttr)
        .def("CreateMinPassiveLongitudinalSlipDenominatorAttr",
             &_CreateMinPassiveLongitudinalSlipDenominatorAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinActiveLongitudinalSlipDenominatorAttr",
             &This::GetMinActiveLongitudinalSlipDenominatorAttr)
        .def("CreateMinActiveLongitudinalSlipDenominatorAttr",
             &_CreateMinActiveLongitudinalSlipDenominatorAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinLateralSlipDenominatorAttr",
             &This::GetMinLateralSlipDenominatorAttr)
        .def("CreateMinLateralSlipDenominatorAttr",
             &_CreateMinLateralSlipDenominatorAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalStickyTireThresholdSpeedAttr",
             &This::GetLongitudinalStickyTireThresholdSpeedAttr)
        .def("CreateLongitudinalStickyTireThresholdSpeedAttr",
             &_CreateLongitudinalStickyTireThresholdSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalStickyTireThresholdTimeAttr",
             &This::GetLongitudinalStickyTireThresholdTimeAttr)
        .def("CreateLongitudinalStickyTireThresholdTimeAttr",
             &_CreateLongitudinalStickyTireThresholdTimeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalStickyTireDampingAttr",
             &This::GetLongitudinalStickyTireDampingAttr)
        .def("CreateLongitudinalStickyTireDampingAttr",
             &_CreateLongitudinalStickyTireDampingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLateralStickyTireThresholdSpeedAttr",
             &This::GetLateralStickyTireThresholdSpeedAttr)
        .def("CreateLateralStickyTireThresholdSpeedAttr",
             &_CreateLateralStickyTireThresholdSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLateralStickyTireThresholdTimeAttr",
             &This::GetLateralStickyTireThresholdTimeAttr)
        .def("CreateLateralStickyTireThresholdTimeAttr",
             &_CreateLateralStickyTireThresholdTimeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLateralStickyTireDampingAttr",
             &This::GetLateralStickyTireDampingAttr)
        .def("CreateLateralStickyTireDampingAttr",
             &_CreateLateralStickyTireDampingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetDriveRel",
             &This::GetDriveRel)
        .def("CreateDriveRel",
             &This::CreateDriveRel)
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
