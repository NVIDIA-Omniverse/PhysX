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
#include ".//physxDeformableAPI.h"
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
_CreateDeformableEnabledAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDeformableEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSolverPositionIterationCountAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolverPositionIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateVertexVelocityDampingAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVertexVelocityDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSleepDampingAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSleepDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSleepThresholdAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSleepThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSettlingThresholdAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSettlingThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxDepenetrationVelocityAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxDepenetrationVelocityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSelfCollisionAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSelfCollisionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSelfCollisionFilterDistanceAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSelfCollisionFilterDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnableCCDAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableCCDAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateRestPointsAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRestPointsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreateSimulationVelocitiesAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSimulationVelocitiesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreateSimulationIndicesAttr(PhysxSchemaPhysxDeformableAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSimulationIndicesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxDeformableAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxDeformableAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxDeformableAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxDeformableAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxDeformableAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxDeformableAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxDeformableAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxDeformableAPI()
{
    typedef PhysxSchemaPhysxDeformableAPI This;

    PhysxSchemaPhysxDeformableAPI_CanApplyResult::Wrap<PhysxSchemaPhysxDeformableAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxDeformableAPI");

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

        
        .def("GetDeformableEnabledAttr",
             &This::GetDeformableEnabledAttr)
        .def("CreateDeformableEnabledAttr",
             &_CreateDeformableEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolverPositionIterationCountAttr",
             &This::GetSolverPositionIterationCountAttr)
        .def("CreateSolverPositionIterationCountAttr",
             &_CreateSolverPositionIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVertexVelocityDampingAttr",
             &This::GetVertexVelocityDampingAttr)
        .def("CreateVertexVelocityDampingAttr",
             &_CreateVertexVelocityDampingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSleepDampingAttr",
             &This::GetSleepDampingAttr)
        .def("CreateSleepDampingAttr",
             &_CreateSleepDampingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSleepThresholdAttr",
             &This::GetSleepThresholdAttr)
        .def("CreateSleepThresholdAttr",
             &_CreateSleepThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSettlingThresholdAttr",
             &This::GetSettlingThresholdAttr)
        .def("CreateSettlingThresholdAttr",
             &_CreateSettlingThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxDepenetrationVelocityAttr",
             &This::GetMaxDepenetrationVelocityAttr)
        .def("CreateMaxDepenetrationVelocityAttr",
             &_CreateMaxDepenetrationVelocityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSelfCollisionAttr",
             &This::GetSelfCollisionAttr)
        .def("CreateSelfCollisionAttr",
             &_CreateSelfCollisionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSelfCollisionFilterDistanceAttr",
             &This::GetSelfCollisionFilterDistanceAttr)
        .def("CreateSelfCollisionFilterDistanceAttr",
             &_CreateSelfCollisionFilterDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableCCDAttr",
             &This::GetEnableCCDAttr)
        .def("CreateEnableCCDAttr",
             &_CreateEnableCCDAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRestPointsAttr",
             &This::GetRestPointsAttr)
        .def("CreateRestPointsAttr",
             &_CreateRestPointsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSimulationVelocitiesAttr",
             &This::GetSimulationVelocitiesAttr)
        .def("CreateSimulationVelocitiesAttr",
             &_CreateSimulationVelocitiesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSimulationIndicesAttr",
             &This::GetSimulationIndicesAttr)
        .def("CreateSimulationIndicesAttr",
             &_CreateSimulationIndicesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetSimulationOwnerRel",
             &This::GetSimulationOwnerRel)
        .def("CreateSimulationOwnerRel",
             &This::CreateSimulationOwnerRel)
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
