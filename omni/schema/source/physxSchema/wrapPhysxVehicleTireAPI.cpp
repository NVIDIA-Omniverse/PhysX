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
#include ".//physxVehicleTireAPI.h"
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
_CreateLatStiffXAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLatStiffXAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLatStiffYAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLatStiffYAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLateralStiffnessGraphAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLateralStiffnessGraphAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float2), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalStiffnessPerUnitGravityAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalStiffnessPerUnitGravityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalStiffnessAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalStiffnessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCamberStiffnessPerUnitGravityAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCamberStiffnessPerUnitGravityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCamberStiffnessAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCamberStiffnessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFrictionVsSlipGraphAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionVsSlipGraphAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float2Array), writeSparsely);
}
        
static UsdAttribute
_CreateRestLoadAttr(PhysxSchemaPhysxVehicleTireAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRestLoadAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleTireAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleTireAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleTireAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleTireAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleTireAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleTireAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleTireAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleTireAPI()
{
    typedef PhysxSchemaPhysxVehicleTireAPI This;

    PhysxSchemaPhysxVehicleTireAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleTireAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleTireAPI");

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

        
        .def("GetLatStiffXAttr",
             &This::GetLatStiffXAttr)
        .def("CreateLatStiffXAttr",
             &_CreateLatStiffXAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLatStiffYAttr",
             &This::GetLatStiffYAttr)
        .def("CreateLatStiffYAttr",
             &_CreateLatStiffYAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLateralStiffnessGraphAttr",
             &This::GetLateralStiffnessGraphAttr)
        .def("CreateLateralStiffnessGraphAttr",
             &_CreateLateralStiffnessGraphAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalStiffnessPerUnitGravityAttr",
             &This::GetLongitudinalStiffnessPerUnitGravityAttr)
        .def("CreateLongitudinalStiffnessPerUnitGravityAttr",
             &_CreateLongitudinalStiffnessPerUnitGravityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalStiffnessAttr",
             &This::GetLongitudinalStiffnessAttr)
        .def("CreateLongitudinalStiffnessAttr",
             &_CreateLongitudinalStiffnessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCamberStiffnessPerUnitGravityAttr",
             &This::GetCamberStiffnessPerUnitGravityAttr)
        .def("CreateCamberStiffnessPerUnitGravityAttr",
             &_CreateCamberStiffnessPerUnitGravityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCamberStiffnessAttr",
             &This::GetCamberStiffnessAttr)
        .def("CreateCamberStiffnessAttr",
             &_CreateCamberStiffnessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFrictionVsSlipGraphAttr",
             &This::GetFrictionVsSlipGraphAttr)
        .def("CreateFrictionVsSlipGraphAttr",
             &_CreateFrictionVsSlipGraphAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRestLoadAttr",
             &This::GetRestLoadAttr)
        .def("CreateRestLoadAttr",
             &_CreateRestLoadAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetFrictionTableRel",
             &This::GetFrictionTableRel)
        .def("CreateFrictionTableRel",
             &This::CreateFrictionTableRel)
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
