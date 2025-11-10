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
#include ".//physxVehicleSuspensionAPI.h"
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
_CreateSpringStrengthAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSpringStrengthAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSpringDamperRateAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSpringDamperRateAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateTravelDistanceAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTravelDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxCompressionAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxCompressionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxDroopAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxDroopAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSprungMassAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSprungMassAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCamberAtRestAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCamberAtRestAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCamberAtMaxCompressionAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCamberAtMaxCompressionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCamberAtMaxDroopAttr(PhysxSchemaPhysxVehicleSuspensionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCamberAtMaxDroopAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleSuspensionAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleSuspensionAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleSuspensionAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleSuspensionAPI()
{
    typedef PhysxSchemaPhysxVehicleSuspensionAPI This;

    PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleSuspensionAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleSuspensionAPI");

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

        
        .def("GetSpringStrengthAttr",
             &This::GetSpringStrengthAttr)
        .def("CreateSpringStrengthAttr",
             &_CreateSpringStrengthAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSpringDamperRateAttr",
             &This::GetSpringDamperRateAttr)
        .def("CreateSpringDamperRateAttr",
             &_CreateSpringDamperRateAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTravelDistanceAttr",
             &This::GetTravelDistanceAttr)
        .def("CreateTravelDistanceAttr",
             &_CreateTravelDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxCompressionAttr",
             &This::GetMaxCompressionAttr)
        .def("CreateMaxCompressionAttr",
             &_CreateMaxCompressionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxDroopAttr",
             &This::GetMaxDroopAttr)
        .def("CreateMaxDroopAttr",
             &_CreateMaxDroopAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSprungMassAttr",
             &This::GetSprungMassAttr)
        .def("CreateSprungMassAttr",
             &_CreateSprungMassAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCamberAtRestAttr",
             &This::GetCamberAtRestAttr)
        .def("CreateCamberAtRestAttr",
             &_CreateCamberAtRestAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCamberAtMaxCompressionAttr",
             &This::GetCamberAtMaxCompressionAttr)
        .def("CreateCamberAtMaxCompressionAttr",
             &_CreateCamberAtMaxCompressionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCamberAtMaxDroopAttr",
             &This::GetCamberAtMaxDroopAttr)
        .def("CreateCamberAtMaxDroopAttr",
             &_CreateCamberAtMaxDroopAttr,
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
