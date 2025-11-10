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
#include ".//physxParticleIsosurfaceAPI.h"
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
_CreateIsosurfaceEnabledAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateIsosurfaceEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateMaxVerticesAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxVerticesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMaxTrianglesAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxTrianglesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMaxSubgridsAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxSubgridsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateGridSpacingAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGridSpacingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSurfaceDistanceAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSurfaceDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateGridFilteringPassesAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGridFilteringPassesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->String), writeSparsely);
}
        
static UsdAttribute
_CreateGridSmoothingRadiusAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGridSmoothingRadiusAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateNumMeshSmoothingPassesAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateNumMeshSmoothingPassesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateNumMeshNormalSmoothingPassesAttr(PhysxSchemaPhysxParticleIsosurfaceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateNumMeshNormalSmoothingPassesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxParticleIsosurfaceAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxParticleIsosurfaceAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxParticleIsosurfaceAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxParticleIsosurfaceAPI()
{
    typedef PhysxSchemaPhysxParticleIsosurfaceAPI This;

    PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult::Wrap<PhysxSchemaPhysxParticleIsosurfaceAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxParticleIsosurfaceAPI");

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

        
        .def("GetIsosurfaceEnabledAttr",
             &This::GetIsosurfaceEnabledAttr)
        .def("CreateIsosurfaceEnabledAttr",
             &_CreateIsosurfaceEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxVerticesAttr",
             &This::GetMaxVerticesAttr)
        .def("CreateMaxVerticesAttr",
             &_CreateMaxVerticesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxTrianglesAttr",
             &This::GetMaxTrianglesAttr)
        .def("CreateMaxTrianglesAttr",
             &_CreateMaxTrianglesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxSubgridsAttr",
             &This::GetMaxSubgridsAttr)
        .def("CreateMaxSubgridsAttr",
             &_CreateMaxSubgridsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGridSpacingAttr",
             &This::GetGridSpacingAttr)
        .def("CreateGridSpacingAttr",
             &_CreateGridSpacingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSurfaceDistanceAttr",
             &This::GetSurfaceDistanceAttr)
        .def("CreateSurfaceDistanceAttr",
             &_CreateSurfaceDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGridFilteringPassesAttr",
             &This::GetGridFilteringPassesAttr)
        .def("CreateGridFilteringPassesAttr",
             &_CreateGridFilteringPassesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGridSmoothingRadiusAttr",
             &This::GetGridSmoothingRadiusAttr)
        .def("CreateGridSmoothingRadiusAttr",
             &_CreateGridSmoothingRadiusAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetNumMeshSmoothingPassesAttr",
             &This::GetNumMeshSmoothingPassesAttr)
        .def("CreateNumMeshSmoothingPassesAttr",
             &_CreateNumMeshSmoothingPassesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetNumMeshNormalSmoothingPassesAttr",
             &This::GetNumMeshNormalSmoothingPassesAttr)
        .def("CreateNumMeshNormalSmoothingPassesAttr",
             &_CreateNumMeshNormalSmoothingPassesAttr,
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
