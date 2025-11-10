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
#include ".//physxDiffuseParticlesAPI.h"
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
_CreateDiffuseParticlesEnabledAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDiffuseParticlesEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateMaxDiffuseParticleMultiplierAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxDiffuseParticleMultiplierAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateThresholdAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLifetimeAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLifetimeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateAirDragAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateAirDragAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateBubbleDragAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBubbleDragAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateBuoyancyAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBuoyancyAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateKineticEnergyWeightAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateKineticEnergyWeightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePressureWeightAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePressureWeightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDivergenceWeightAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDivergenceWeightAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCollisionDecayAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCollisionDecayAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateUseAccurateVelocityAttr(PhysxSchemaPhysxDiffuseParticlesAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUseAccurateVelocityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxDiffuseParticlesAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxDiffuseParticlesAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxDiffuseParticlesAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxDiffuseParticlesAPI()
{
    typedef PhysxSchemaPhysxDiffuseParticlesAPI This;

    PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult::Wrap<PhysxSchemaPhysxDiffuseParticlesAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxDiffuseParticlesAPI");

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

        
        .def("GetDiffuseParticlesEnabledAttr",
             &This::GetDiffuseParticlesEnabledAttr)
        .def("CreateDiffuseParticlesEnabledAttr",
             &_CreateDiffuseParticlesEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxDiffuseParticleMultiplierAttr",
             &This::GetMaxDiffuseParticleMultiplierAttr)
        .def("CreateMaxDiffuseParticleMultiplierAttr",
             &_CreateMaxDiffuseParticleMultiplierAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetThresholdAttr",
             &This::GetThresholdAttr)
        .def("CreateThresholdAttr",
             &_CreateThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLifetimeAttr",
             &This::GetLifetimeAttr)
        .def("CreateLifetimeAttr",
             &_CreateLifetimeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetAirDragAttr",
             &This::GetAirDragAttr)
        .def("CreateAirDragAttr",
             &_CreateAirDragAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBubbleDragAttr",
             &This::GetBubbleDragAttr)
        .def("CreateBubbleDragAttr",
             &_CreateBubbleDragAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBuoyancyAttr",
             &This::GetBuoyancyAttr)
        .def("CreateBuoyancyAttr",
             &_CreateBuoyancyAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetKineticEnergyWeightAttr",
             &This::GetKineticEnergyWeightAttr)
        .def("CreateKineticEnergyWeightAttr",
             &_CreateKineticEnergyWeightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPressureWeightAttr",
             &This::GetPressureWeightAttr)
        .def("CreatePressureWeightAttr",
             &_CreatePressureWeightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDivergenceWeightAttr",
             &This::GetDivergenceWeightAttr)
        .def("CreateDivergenceWeightAttr",
             &_CreateDivergenceWeightAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCollisionDecayAttr",
             &This::GetCollisionDecayAttr)
        .def("CreateCollisionDecayAttr",
             &_CreateCollisionDecayAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetUseAccurateVelocityAttr",
             &This::GetUseAccurateVelocityAttr)
        .def("CreateUseAccurateVelocityAttr",
             &_CreateUseAccurateVelocityAttr,
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
