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
#include ".//physxVehicleWheelAPI.h"
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
_CreateRadiusAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRadiusAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateWidthAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWidthAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMassAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMassAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMoiAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMoiAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDampingRateAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingRateAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxBrakeTorqueAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxBrakeTorqueAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxHandBrakeTorqueAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxHandBrakeTorqueAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxSteerAngleAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxSteerAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateToeAngleAttr(PhysxSchemaPhysxVehicleWheelAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateToeAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleWheelAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleWheelAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleWheelAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleWheelAPI()
{
    typedef PhysxSchemaPhysxVehicleWheelAPI This;

    PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleWheelAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleWheelAPI");

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

        
        .def("GetRadiusAttr",
             &This::GetRadiusAttr)
        .def("CreateRadiusAttr",
             &_CreateRadiusAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWidthAttr",
             &This::GetWidthAttr)
        .def("CreateWidthAttr",
             &_CreateWidthAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMassAttr",
             &This::GetMassAttr)
        .def("CreateMassAttr",
             &_CreateMassAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMoiAttr",
             &This::GetMoiAttr)
        .def("CreateMoiAttr",
             &_CreateMoiAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingRateAttr",
             &This::GetDampingRateAttr)
        .def("CreateDampingRateAttr",
             &_CreateDampingRateAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxBrakeTorqueAttr",
             &This::GetMaxBrakeTorqueAttr)
        .def("CreateMaxBrakeTorqueAttr",
             &_CreateMaxBrakeTorqueAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxHandBrakeTorqueAttr",
             &This::GetMaxHandBrakeTorqueAttr)
        .def("CreateMaxHandBrakeTorqueAttr",
             &_CreateMaxHandBrakeTorqueAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxSteerAngleAttr",
             &This::GetMaxSteerAngleAttr)
        .def("CreateMaxSteerAngleAttr",
             &_CreateMaxSteerAngleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetToeAngleAttr",
             &This::GetToeAngleAttr)
        .def("CreateToeAngleAttr",
             &_CreateToeAngleAttr,
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
