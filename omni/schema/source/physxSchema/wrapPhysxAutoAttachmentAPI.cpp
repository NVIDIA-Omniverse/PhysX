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
#include ".//physxAutoAttachmentAPI.h"
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
_CreateEnableDeformableVertexAttachmentsAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableDeformableVertexAttachmentsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateDeformableVertexOverlapOffsetAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDeformableVertexOverlapOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnableRigidSurfaceAttachmentsAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableRigidSurfaceAttachmentsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateRigidSurfaceSamplingDistanceAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRigidSurfaceSamplingDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnableCollisionFilteringAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableCollisionFilteringAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateCollisionFilteringOffsetAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCollisionFilteringOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnableDeformableFilteringPairsAttr(PhysxSchemaPhysxAutoAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableDeformableFilteringPairsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxAutoAttachmentAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxAutoAttachmentAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxAutoAttachmentAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxAutoAttachmentAPI()
{
    typedef PhysxSchemaPhysxAutoAttachmentAPI This;

    PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult::Wrap<PhysxSchemaPhysxAutoAttachmentAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxAutoAttachmentAPI");

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

        
        .def("GetEnableDeformableVertexAttachmentsAttr",
             &This::GetEnableDeformableVertexAttachmentsAttr)
        .def("CreateEnableDeformableVertexAttachmentsAttr",
             &_CreateEnableDeformableVertexAttachmentsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDeformableVertexOverlapOffsetAttr",
             &This::GetDeformableVertexOverlapOffsetAttr)
        .def("CreateDeformableVertexOverlapOffsetAttr",
             &_CreateDeformableVertexOverlapOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableRigidSurfaceAttachmentsAttr",
             &This::GetEnableRigidSurfaceAttachmentsAttr)
        .def("CreateEnableRigidSurfaceAttachmentsAttr",
             &_CreateEnableRigidSurfaceAttachmentsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRigidSurfaceSamplingDistanceAttr",
             &This::GetRigidSurfaceSamplingDistanceAttr)
        .def("CreateRigidSurfaceSamplingDistanceAttr",
             &_CreateRigidSurfaceSamplingDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableCollisionFilteringAttr",
             &This::GetEnableCollisionFilteringAttr)
        .def("CreateEnableCollisionFilteringAttr",
             &_CreateEnableCollisionFilteringAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCollisionFilteringOffsetAttr",
             &This::GetCollisionFilteringOffsetAttr)
        .def("CreateCollisionFilteringOffsetAttr",
             &_CreateCollisionFilteringOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableDeformableFilteringPairsAttr",
             &This::GetEnableDeformableFilteringPairsAttr)
        .def("CreateEnableDeformableFilteringPairsAttr",
             &_CreateEnableDeformableFilteringPairsAttr,
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
