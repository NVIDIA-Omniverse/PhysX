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
#include ".//physxParticleSystem.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
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
_CreateParticleSystemEnabledAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateParticleSystemEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateContactOffsetAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateContactOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateRestOffsetAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRestOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateParticleContactOffsetAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateParticleContactOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSolidRestOffsetAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolidRestOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFluidRestOffsetAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFluidRestOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnableCCDAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableCCDAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSolverPositionIterationCountAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolverPositionIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMaxDepenetrationVelocityAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxDepenetrationVelocityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateWindAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWindAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateMaxNeighborhoodAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxNeighborhoodAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateNeighborhoodScaleAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateNeighborhoodScaleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxVelocityAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxVelocityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateGlobalSelfCollisionEnabledAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGlobalSelfCollisionEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateNonParticleCollisionEnabledAttr(PhysxSchemaPhysxParticleSystem &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateNonParticleCollisionEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxParticleSystem &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxParticleSystem(%s)",
        primRepr.c_str());
}

} // anonymous namespace

void wrapPhysxSchemaPhysxParticleSystem()
{
    typedef PhysxSchemaPhysxParticleSystem This;

    class_<This, bases<UsdGeomGprim> >
        cls("PhysxParticleSystem");

    cls
        .def(init<UsdPrim>(arg("prim")))
        .def(init<UsdSchemaBase const&>(arg("schemaObj")))
        .def(TfTypePythonClass())

        .def("Get", &This::Get, (arg("stage"), arg("path")))
        .staticmethod("Get")

        .def("Define", &This::Define, (arg("stage"), arg("path")))
        .staticmethod("Define")

        .def("GetSchemaAttributeNames",
             &This::GetSchemaAttributeNames,
             arg("includeInherited")=true,
             return_value_policy<TfPySequenceToList>())
        .staticmethod("GetSchemaAttributeNames")

        .def("_GetStaticTfType", (TfType const &(*)()) TfType::Find<This>,
             return_value_policy<return_by_value>())
        .staticmethod("_GetStaticTfType")

        .def(!self)

        
        .def("GetParticleSystemEnabledAttr",
             &This::GetParticleSystemEnabledAttr)
        .def("CreateParticleSystemEnabledAttr",
             &_CreateParticleSystemEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetContactOffsetAttr",
             &This::GetContactOffsetAttr)
        .def("CreateContactOffsetAttr",
             &_CreateContactOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRestOffsetAttr",
             &This::GetRestOffsetAttr)
        .def("CreateRestOffsetAttr",
             &_CreateRestOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetParticleContactOffsetAttr",
             &This::GetParticleContactOffsetAttr)
        .def("CreateParticleContactOffsetAttr",
             &_CreateParticleContactOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolidRestOffsetAttr",
             &This::GetSolidRestOffsetAttr)
        .def("CreateSolidRestOffsetAttr",
             &_CreateSolidRestOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFluidRestOffsetAttr",
             &This::GetFluidRestOffsetAttr)
        .def("CreateFluidRestOffsetAttr",
             &_CreateFluidRestOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableCCDAttr",
             &This::GetEnableCCDAttr)
        .def("CreateEnableCCDAttr",
             &_CreateEnableCCDAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolverPositionIterationCountAttr",
             &This::GetSolverPositionIterationCountAttr)
        .def("CreateSolverPositionIterationCountAttr",
             &_CreateSolverPositionIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxDepenetrationVelocityAttr",
             &This::GetMaxDepenetrationVelocityAttr)
        .def("CreateMaxDepenetrationVelocityAttr",
             &_CreateMaxDepenetrationVelocityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWindAttr",
             &This::GetWindAttr)
        .def("CreateWindAttr",
             &_CreateWindAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxNeighborhoodAttr",
             &This::GetMaxNeighborhoodAttr)
        .def("CreateMaxNeighborhoodAttr",
             &_CreateMaxNeighborhoodAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetNeighborhoodScaleAttr",
             &This::GetNeighborhoodScaleAttr)
        .def("CreateNeighborhoodScaleAttr",
             &_CreateNeighborhoodScaleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxVelocityAttr",
             &This::GetMaxVelocityAttr)
        .def("CreateMaxVelocityAttr",
             &_CreateMaxVelocityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGlobalSelfCollisionEnabledAttr",
             &This::GetGlobalSelfCollisionEnabledAttr)
        .def("CreateGlobalSelfCollisionEnabledAttr",
             &_CreateGlobalSelfCollisionEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetNonParticleCollisionEnabledAttr",
             &This::GetNonParticleCollisionEnabledAttr)
        .def("CreateNonParticleCollisionEnabledAttr",
             &_CreateNonParticleCollisionEnabledAttr,
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
