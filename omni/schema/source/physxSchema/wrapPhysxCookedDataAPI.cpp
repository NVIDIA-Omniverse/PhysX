//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxCookedDataAPI.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
#include "pxr/base/tf/pyAnnotatedBoolResult.h"
#include "pxr/base/tf/pyContainerConversions.h"
#include "pxr/base/tf/pyResultConversions.h"
#include "pxr/base/tf/pyUtils.h"
#include "pxr/base/tf/wrapTypeHelpers.h"

#include "pxr/external/boost/python.hpp"

#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace pxr_boost::python;

namespace {

#define WRAP_CUSTOM                                                     \
    template <class Cls> static void _CustomWrapCode(Cls &_class)

// fwd decl.
WRAP_CUSTOM;

        
static UsdAttribute
_CreateBufferAttr(PhysxSchemaPhysxCookedDataAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBufferAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UCharArray), writeSparsely);
}

static bool _WrapIsPhysxCookedDataAPIPath(const SdfPath &path) {
    TfToken collectionName;
    return PhysxSchemaPhysxCookedDataAPI::IsPhysxCookedDataAPIPath(
        path, &collectionName);
}

static std::string
_Repr(const PhysxSchemaPhysxCookedDataAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    std::string instanceName = TfPyRepr(self.GetName());
    return TfStringPrintf(
        "PhysxSchema.PhysxCookedDataAPI(%s, '%s')",
        primRepr.c_str(), instanceName.c_str());
}

struct PhysxSchemaPhysxCookedDataAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxCookedDataAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxCookedDataAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim, const TfToken& name)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxCookedDataAPI::CanApply(prim, name, &whyNot);
    return PhysxSchemaPhysxCookedDataAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxCookedDataAPI()
{
    typedef PhysxSchemaPhysxCookedDataAPI This;

    PhysxSchemaPhysxCookedDataAPI_CanApplyResult::Wrap<PhysxSchemaPhysxCookedDataAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxCookedDataAPI");

    cls
        .def(init<UsdPrim, TfToken>((arg("prim"), arg("name"))))
        .def(init<UsdSchemaBase const&, TfToken>((arg("schemaObj"), arg("name"))))
        .def(TfTypePythonClass())

        .def("Get",
            (PhysxSchemaPhysxCookedDataAPI(*)(const UsdStagePtr &stage, 
                                       const SdfPath &path))
               &This::Get,
            (arg("stage"), arg("path")))
        .def("Get",
            (PhysxSchemaPhysxCookedDataAPI(*)(const UsdPrim &prim,
                                       const TfToken &name))
               &This::Get,
            (arg("prim"), arg("name")))
        .staticmethod("Get")

        .def("GetAll",
            (std::vector<PhysxSchemaPhysxCookedDataAPI>(*)(const UsdPrim &prim))
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

        
        .def("GetBufferAttr",
             &This::GetBufferAttr)
        .def("CreateBufferAttr",
             &_CreateBufferAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        .def("IsPhysxCookedDataAPIPath", _WrapIsPhysxCookedDataAPIPath)
            .staticmethod("IsPhysxCookedDataAPIPath")
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
