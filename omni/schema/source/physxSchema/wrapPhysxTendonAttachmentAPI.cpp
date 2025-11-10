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
#include ".//physxTendonAttachmentAPI.h"
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
_CreateGearingAttr(PhysxSchemaPhysxTendonAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGearingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateLocalPosAttr(PhysxSchemaPhysxTendonAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLocalPosAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3f), writeSparsely);
}
        
static UsdAttribute
_CreateParentAttachmentAttr(PhysxSchemaPhysxTendonAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateParentAttachmentAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static bool _WrapIsPhysxTendonAttachmentAPIPath(const SdfPath &path) {
    TfToken collectionName;
    return PhysxSchemaPhysxTendonAttachmentAPI::IsPhysxTendonAttachmentAPIPath(
        path, &collectionName);
}

static std::string
_Repr(const PhysxSchemaPhysxTendonAttachmentAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    std::string instanceName = TfPyRepr(self.GetName());
    return TfStringPrintf(
        "PhysxSchema.PhysxTendonAttachmentAPI(%s, '%s')",
        primRepr.c_str(), instanceName.c_str());
}

struct PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim, const TfToken& name)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxTendonAttachmentAPI::CanApply(prim, name, &whyNot);
    return PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxTendonAttachmentAPI()
{
    typedef PhysxSchemaPhysxTendonAttachmentAPI This;

    PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult::Wrap<PhysxSchemaPhysxTendonAttachmentAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxTendonAttachmentAPI");

    cls
        .def(init<UsdPrim, TfToken>((arg("prim"), arg("name"))))
        .def(init<UsdSchemaBase const&, TfToken>((arg("schemaObj"), arg("name"))))
        .def(TfTypePythonClass())

        .def("Get",
            (PhysxSchemaPhysxTendonAttachmentAPI(*)(const UsdStagePtr &stage, 
                                       const SdfPath &path))
               &This::Get,
            (arg("stage"), arg("path")))
        .def("Get",
            (PhysxSchemaPhysxTendonAttachmentAPI(*)(const UsdPrim &prim,
                                       const TfToken &name))
               &This::Get,
            (arg("prim"), arg("name")))
        .staticmethod("Get")

        .def("GetAll",
            (std::vector<PhysxSchemaPhysxTendonAttachmentAPI>(*)(const UsdPrim &prim))
                &This::GetAll,
            arg("prim"),
            return_value_policy<TfPySequenceToList>())
        .staticmethod("GetAll")

        .def("CanApply", &_WrapCanApply, (arg("prim"), arg("name")))
        .staticmethod("CanApply")

        .def("Apply", &This::Apply, (arg("prim"), arg("name")))
        .staticmethod("Apply")

        .def("GetSchemaAttributeNames",
             (const TfTokenVector &(*)(bool))&This::GetSchemaAttributeNames,
             arg("includeInherited")=true,
             return_value_policy<TfPySequenceToList>())
        .def("GetSchemaAttributeNames",
             (TfTokenVector(*)(bool, const TfToken &))
                &This::GetSchemaAttributeNames,
             arg("includeInherited"),
             arg("instanceName"),
             return_value_policy<TfPySequenceToList>())
        .staticmethod("GetSchemaAttributeNames")

        .def("_GetStaticTfType", (TfType const &(*)()) TfType::Find<This>,
             return_value_policy<return_by_value>())
        .staticmethod("_GetStaticTfType")

        .def(!self)

        
        .def("GetGearingAttr",
             &This::GetGearingAttr)
        .def("CreateGearingAttr",
             &_CreateGearingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLocalPosAttr",
             &This::GetLocalPosAttr)
        .def("CreateLocalPosAttr",
             &_CreateLocalPosAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetParentAttachmentAttr",
             &This::GetParentAttachmentAttr)
        .def("CreateParentAttachmentAttr",
             &_CreateParentAttachmentAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetParentLinkRel",
             &This::GetParentLinkRel)
        .def("CreateParentLinkRel",
             &This::CreateParentLinkRel)
        .def("IsPhysxTendonAttachmentAPIPath", _WrapIsPhysxTendonAttachmentAPIPath)
            .staticmethod("IsPhysxTendonAttachmentAPIPath")
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
