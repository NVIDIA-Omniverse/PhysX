//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleContextAPI.h"
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
_CreateUpdateModeAttr(PhysxSchemaPhysxVehicleContextAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpdateModeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateUpAxisAttr(PhysxSchemaPhysxVehicleContextAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpAxisAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateForwardAxisAttr(PhysxSchemaPhysxVehicleContextAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateForwardAxisAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateVerticalAxisAttr(PhysxSchemaPhysxVehicleContextAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVerticalAxisAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateLongitudinalAxisAttr(PhysxSchemaPhysxVehicleContextAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLongitudinalAxisAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleContextAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleContextAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleContextAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleContextAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleContextAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleContextAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleContextAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleContextAPI()
{
    typedef PhysxSchemaPhysxVehicleContextAPI This;

    PhysxSchemaPhysxVehicleContextAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleContextAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleContextAPI");

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

        
        .def("GetUpdateModeAttr",
             &This::GetUpdateModeAttr)
        .def("CreateUpdateModeAttr",
             &_CreateUpdateModeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetUpAxisAttr",
             &This::GetUpAxisAttr)
        .def("CreateUpAxisAttr",
             &_CreateUpAxisAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetForwardAxisAttr",
             &This::GetForwardAxisAttr)
        .def("CreateForwardAxisAttr",
             &_CreateForwardAxisAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVerticalAxisAttr",
             &This::GetVerticalAxisAttr)
        .def("CreateVerticalAxisAttr",
             &_CreateVerticalAxisAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLongitudinalAxisAttr",
             &This::GetLongitudinalAxisAttr)
        .def("CreateLongitudinalAxisAttr",
             &_CreateLongitudinalAxisAttr,
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
