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
#include ".//physxPhysicsAttachment.h"
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
_CreateAttachmentEnabledAttr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateAttachmentEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreatePoints0Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePoints0Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreatePoints1Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePoints1Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreateCollisionFilterIndices0Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCollisionFilterIndices0Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UIntArray), writeSparsely);
}
        
static UsdAttribute
_CreateFilterType0Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFilterType0Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateCollisionFilterIndices1Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCollisionFilterIndices1Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UIntArray), writeSparsely);
}
        
static UsdAttribute
_CreateFilterType1Attr(PhysxSchemaPhysxPhysicsAttachment &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFilterType1Attr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxPhysicsAttachment &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxPhysicsAttachment(%s)",
        primRepr.c_str());
}

} // anonymous namespace

void wrapPhysxSchemaPhysxPhysicsAttachment()
{
    typedef PhysxSchemaPhysxPhysicsAttachment This;

    class_<This, bases<UsdTyped> >
        cls("PhysxPhysicsAttachment");

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

        
        .def("GetAttachmentEnabledAttr",
             &This::GetAttachmentEnabledAttr)
        .def("CreateAttachmentEnabledAttr",
             &_CreateAttachmentEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPoints0Attr",
             &This::GetPoints0Attr)
        .def("CreatePoints0Attr",
             &_CreatePoints0Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPoints1Attr",
             &This::GetPoints1Attr)
        .def("CreatePoints1Attr",
             &_CreatePoints1Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCollisionFilterIndices0Attr",
             &This::GetCollisionFilterIndices0Attr)
        .def("CreateCollisionFilterIndices0Attr",
             &_CreateCollisionFilterIndices0Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFilterType0Attr",
             &This::GetFilterType0Attr)
        .def("CreateFilterType0Attr",
             &_CreateFilterType0Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCollisionFilterIndices1Attr",
             &This::GetCollisionFilterIndices1Attr)
        .def("CreateCollisionFilterIndices1Attr",
             &_CreateCollisionFilterIndices1Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFilterType1Attr",
             &This::GetFilterType1Attr)
        .def("CreateFilterType1Attr",
             &_CreateFilterType1Attr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetActor0Rel",
             &This::GetActor0Rel)
        .def("CreateActor0Rel",
             &This::CreateActor0Rel)
        
        .def("GetActor1Rel",
             &This::GetActor1Rel)
        .def("CreateActor1Rel",
             &This::CreateActor1Rel)
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
      _class
          .def("GetPointsAttr", &PhysxSchemaPhysxPhysicsAttachment::GetPointsAttr, (arg("index")))
          .def("GetFilterTypeAttr", &PhysxSchemaPhysxPhysicsAttachment::GetFilterTypeAttr, (arg("index")))
          .def("GetCollisionFilterIndicesAttr", &PhysxSchemaPhysxPhysicsAttachment::GetCollisionFilterIndicesAttr, (arg("index")))
          .def("GetActorRel", &PhysxSchemaPhysxPhysicsAttachment::GetActorRel, (arg("index")))
      ;
}

}
