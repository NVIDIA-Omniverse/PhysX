//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxForceAPI.h"
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
_CreateForceEnabledAttr(PhysxSchemaPhysxForceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateForceEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateWorldFrameEnabledAttr(PhysxSchemaPhysxForceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWorldFrameEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateModeAttr(PhysxSchemaPhysxForceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateModeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateForceAttr(PhysxSchemaPhysxForceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateForceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Vector3f), writeSparsely);
}
        
static UsdAttribute
_CreateTorqueAttr(PhysxSchemaPhysxForceAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTorqueAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Vector3f), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxForceAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxForceAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxForceAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxForceAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxForceAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxForceAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxForceAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxForceAPI()
{
    typedef PhysxSchemaPhysxForceAPI This;

    PhysxSchemaPhysxForceAPI_CanApplyResult::Wrap<PhysxSchemaPhysxForceAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxForceAPI");

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

        
        .def("GetForceEnabledAttr",
             &This::GetForceEnabledAttr)
        .def("CreateForceEnabledAttr",
             &_CreateForceEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWorldFrameEnabledAttr",
             &This::GetWorldFrameEnabledAttr)
        .def("CreateWorldFrameEnabledAttr",
             &_CreateWorldFrameEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetModeAttr",
             &This::GetModeAttr)
        .def("CreateModeAttr",
             &_CreateModeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetForceAttr",
             &This::GetForceAttr)
        .def("CreateForceAttr",
             &_CreateForceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTorqueAttr",
             &This::GetTorqueAttr)
        .def("CreateTorqueAttr",
             &_CreateTorqueAttr,
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
