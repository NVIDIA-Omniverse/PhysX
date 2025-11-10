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
#include ".//physxPBDMaterialAPI.h"
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
_CreateFrictionAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateParticleFrictionScaleAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateParticleFrictionScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDampingAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateViscosityAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateViscosityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVorticityConfinementAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVorticityConfinementAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSurfaceTensionAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSurfaceTensionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCohesionAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCohesionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateAdhesionAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateAdhesionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateParticleAdhesionScaleAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateParticleAdhesionScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateAdhesionOffsetScaleAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateAdhesionOffsetScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateGravityScaleAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGravityScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLiftAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLiftAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDragAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDragAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDensityAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDensityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCflCoefficientAttr(PhysxSchemaPhysxPBDMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCflCoefficientAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxPBDMaterialAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxPBDMaterialAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxPBDMaterialAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxPBDMaterialAPI()
{
    typedef PhysxSchemaPhysxPBDMaterialAPI This;

    PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult::Wrap<PhysxSchemaPhysxPBDMaterialAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxPBDMaterialAPI");

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

        
        .def("GetFrictionAttr",
             &This::GetFrictionAttr)
        .def("CreateFrictionAttr",
             &_CreateFrictionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetParticleFrictionScaleAttr",
             &This::GetParticleFrictionScaleAttr)
        .def("CreateParticleFrictionScaleAttr",
             &_CreateParticleFrictionScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingAttr",
             &This::GetDampingAttr)
        .def("CreateDampingAttr",
             &_CreateDampingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetViscosityAttr",
             &This::GetViscosityAttr)
        .def("CreateViscosityAttr",
             &_CreateViscosityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVorticityConfinementAttr",
             &This::GetVorticityConfinementAttr)
        .def("CreateVorticityConfinementAttr",
             &_CreateVorticityConfinementAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSurfaceTensionAttr",
             &This::GetSurfaceTensionAttr)
        .def("CreateSurfaceTensionAttr",
             &_CreateSurfaceTensionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCohesionAttr",
             &This::GetCohesionAttr)
        .def("CreateCohesionAttr",
             &_CreateCohesionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetAdhesionAttr",
             &This::GetAdhesionAttr)
        .def("CreateAdhesionAttr",
             &_CreateAdhesionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetParticleAdhesionScaleAttr",
             &This::GetParticleAdhesionScaleAttr)
        .def("CreateParticleAdhesionScaleAttr",
             &_CreateParticleAdhesionScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetAdhesionOffsetScaleAttr",
             &This::GetAdhesionOffsetScaleAttr)
        .def("CreateAdhesionOffsetScaleAttr",
             &_CreateAdhesionOffsetScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGravityScaleAttr",
             &This::GetGravityScaleAttr)
        .def("CreateGravityScaleAttr",
             &_CreateGravityScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLiftAttr",
             &This::GetLiftAttr)
        .def("CreateLiftAttr",
             &_CreateLiftAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDragAttr",
             &This::GetDragAttr)
        .def("CreateDragAttr",
             &_CreateDragAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDensityAttr",
             &This::GetDensityAttr)
        .def("CreateDensityAttr",
             &_CreateDensityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCflCoefficientAttr",
             &This::GetCflCoefficientAttr)
        .def("CreateCflCoefficientAttr",
             &_CreateCflCoefficientAttr,
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
