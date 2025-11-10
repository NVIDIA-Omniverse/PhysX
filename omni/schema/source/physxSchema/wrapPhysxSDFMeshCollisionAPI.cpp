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
#include ".//physxSDFMeshCollisionAPI.h"
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
_CreateSdfResolutionAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfResolutionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateSdfSubgridResolutionAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfSubgridResolutionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateSdfBitsPerSubgridPixelAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfBitsPerSubgridPixelAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateSdfNarrowBandThicknessAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfNarrowBandThicknessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSdfMarginAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfMarginAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSdfEnableRemeshingAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfEnableRemeshingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSdfTriangleCountReductionFactorAttr(PhysxSchemaPhysxSDFMeshCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSdfTriangleCountReductionFactorAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxSDFMeshCollisionAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxSDFMeshCollisionAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxSDFMeshCollisionAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxSDFMeshCollisionAPI()
{
    typedef PhysxSchemaPhysxSDFMeshCollisionAPI This;

    PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult::Wrap<PhysxSchemaPhysxSDFMeshCollisionAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxSDFMeshCollisionAPI");

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

        
        .def("GetSdfResolutionAttr",
             &This::GetSdfResolutionAttr)
        .def("CreateSdfResolutionAttr",
             &_CreateSdfResolutionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfSubgridResolutionAttr",
             &This::GetSdfSubgridResolutionAttr)
        .def("CreateSdfSubgridResolutionAttr",
             &_CreateSdfSubgridResolutionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfBitsPerSubgridPixelAttr",
             &This::GetSdfBitsPerSubgridPixelAttr)
        .def("CreateSdfBitsPerSubgridPixelAttr",
             &_CreateSdfBitsPerSubgridPixelAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfNarrowBandThicknessAttr",
             &This::GetSdfNarrowBandThicknessAttr)
        .def("CreateSdfNarrowBandThicknessAttr",
             &_CreateSdfNarrowBandThicknessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfMarginAttr",
             &This::GetSdfMarginAttr)
        .def("CreateSdfMarginAttr",
             &_CreateSdfMarginAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfEnableRemeshingAttr",
             &This::GetSdfEnableRemeshingAttr)
        .def("CreateSdfEnableRemeshingAttr",
             &_CreateSdfEnableRemeshingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSdfTriangleCountReductionFactorAttr",
             &This::GetSdfTriangleCountReductionFactorAttr)
        .def("CreateSdfTriangleCountReductionFactorAttr",
             &_CreateSdfTriangleCountReductionFactorAttr,
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
