//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxMaterialAPI.h"
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
_CreateFrictionCombineModeAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionCombineModeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateRestitutionCombineModeAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateRestitutionCombineModeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateDampingCombineModeAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDampingCombineModeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateCompliantContactAccelerationSpringAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCompliantContactAccelerationSpringAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateCompliantContactStiffnessAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCompliantContactStiffnessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCompliantContactDampingAttr(PhysxSchemaPhysxMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCompliantContactDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxMaterialAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxMaterialAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxMaterialAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxMaterialAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxMaterialAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxMaterialAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxMaterialAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxMaterialAPI()
{
    typedef PhysxSchemaPhysxMaterialAPI This;

    PhysxSchemaPhysxMaterialAPI_CanApplyResult::Wrap<PhysxSchemaPhysxMaterialAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxMaterialAPI");

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

        
        .def("GetFrictionCombineModeAttr",
             &This::GetFrictionCombineModeAttr)
        .def("CreateFrictionCombineModeAttr",
             &_CreateFrictionCombineModeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetRestitutionCombineModeAttr",
             &This::GetRestitutionCombineModeAttr)
        .def("CreateRestitutionCombineModeAttr",
             &_CreateRestitutionCombineModeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDampingCombineModeAttr",
             &This::GetDampingCombineModeAttr)
        .def("CreateDampingCombineModeAttr",
             &_CreateDampingCombineModeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCompliantContactAccelerationSpringAttr",
             &This::GetCompliantContactAccelerationSpringAttr)
        .def("CreateCompliantContactAccelerationSpringAttr",
             &_CreateCompliantContactAccelerationSpringAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCompliantContactStiffnessAttr",
             &This::GetCompliantContactStiffnessAttr)
        .def("CreateCompliantContactStiffnessAttr",
             &_CreateCompliantContactStiffnessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCompliantContactDampingAttr",
             &This::GetCompliantContactDampingAttr)
        .def("CreateCompliantContactDampingAttr",
             &_CreateCompliantContactDampingAttr,
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
