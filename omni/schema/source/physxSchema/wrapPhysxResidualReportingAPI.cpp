//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxResidualReportingAPI.h"
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
_CreatePhysxResidualReportingRmsResidualPositionIterationAttr(PhysxSchemaPhysxResidualReportingAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysxResidualReportingRmsResidualPositionIterationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePhysxResidualReportingMaxResidualPositionIterationAttr(PhysxSchemaPhysxResidualReportingAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysxResidualReportingMaxResidualPositionIterationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePhysxResidualReportingRmsResidualVelocityIterationAttr(PhysxSchemaPhysxResidualReportingAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysxResidualReportingRmsResidualVelocityIterationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePhysxResidualReportingMaxResidualVelocityIterationAttr(PhysxSchemaPhysxResidualReportingAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysxResidualReportingMaxResidualVelocityIterationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxResidualReportingAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxResidualReportingAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxResidualReportingAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxResidualReportingAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxResidualReportingAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxResidualReportingAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxResidualReportingAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxResidualReportingAPI()
{
    typedef PhysxSchemaPhysxResidualReportingAPI This;

    PhysxSchemaPhysxResidualReportingAPI_CanApplyResult::Wrap<PhysxSchemaPhysxResidualReportingAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxResidualReportingAPI");

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

        
        .def("GetPhysxResidualReportingRmsResidualPositionIterationAttr",
             &This::GetPhysxResidualReportingRmsResidualPositionIterationAttr)
        .def("CreatePhysxResidualReportingRmsResidualPositionIterationAttr",
             &_CreatePhysxResidualReportingRmsResidualPositionIterationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysxResidualReportingMaxResidualPositionIterationAttr",
             &This::GetPhysxResidualReportingMaxResidualPositionIterationAttr)
        .def("CreatePhysxResidualReportingMaxResidualPositionIterationAttr",
             &_CreatePhysxResidualReportingMaxResidualPositionIterationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysxResidualReportingRmsResidualVelocityIterationAttr",
             &This::GetPhysxResidualReportingRmsResidualVelocityIterationAttr)
        .def("CreatePhysxResidualReportingRmsResidualVelocityIterationAttr",
             &_CreatePhysxResidualReportingRmsResidualVelocityIterationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysxResidualReportingMaxResidualVelocityIterationAttr",
             &This::GetPhysxResidualReportingMaxResidualVelocityIterationAttr)
        .def("CreatePhysxResidualReportingMaxResidualVelocityIterationAttr",
             &_CreatePhysxResidualReportingMaxResidualVelocityIterationAttr,
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
