//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxDeformableSurfaceMaterialAPI.h"
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
_CreateDensityAttr(PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDensityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDynamicFrictionAttr(PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDynamicFrictionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateYoungsModulusAttr(PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateYoungsModulusAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreatePoissonsRatioAttr(PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePoissonsRatioAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateThicknessAttr(PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateThicknessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxDeformableSurfaceMaterialAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxDeformableSurfaceMaterialAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxDeformableSurfaceMaterialAPI()
{
    typedef PhysxSchemaPhysxDeformableSurfaceMaterialAPI This;

    PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult::Wrap<PhysxSchemaPhysxDeformableSurfaceMaterialAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxDeformableSurfaceMaterialAPI");

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

        
        .def("GetDensityAttr",
             &This::GetDensityAttr)
        .def("CreateDensityAttr",
             &_CreateDensityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDynamicFrictionAttr",
             &This::GetDynamicFrictionAttr)
        .def("CreateDynamicFrictionAttr",
             &_CreateDynamicFrictionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetYoungsModulusAttr",
             &This::GetYoungsModulusAttr)
        .def("CreateYoungsModulusAttr",
             &_CreateYoungsModulusAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPoissonsRatioAttr",
             &This::GetPoissonsRatioAttr)
        .def("CreatePoissonsRatioAttr",
             &_CreatePoissonsRatioAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetThicknessAttr",
             &This::GetThicknessAttr)
        .def("CreateThicknessAttr",
             &_CreateThicknessAttr,
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
