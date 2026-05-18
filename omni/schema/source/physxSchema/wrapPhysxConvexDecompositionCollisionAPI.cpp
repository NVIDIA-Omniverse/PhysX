//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxConvexDecompositionCollisionAPI.h"
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
_CreateHullVertexLimitAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateHullVertexLimitAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMaxConvexHullsAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxConvexHullsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateMinThicknessAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinThicknessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVoxelResolutionAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVoxelResolutionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateErrorPercentageAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateErrorPercentageAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateShrinkWrapAttr(PhysxSchemaPhysxConvexDecompositionCollisionAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateShrinkWrapAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxConvexDecompositionCollisionAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxConvexDecompositionCollisionAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxConvexDecompositionCollisionAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxConvexDecompositionCollisionAPI()
{
    typedef PhysxSchemaPhysxConvexDecompositionCollisionAPI This;

    PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult::Wrap<PhysxSchemaPhysxConvexDecompositionCollisionAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxConvexDecompositionCollisionAPI");

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

        
        .def("GetHullVertexLimitAttr",
             &This::GetHullVertexLimitAttr)
        .def("CreateHullVertexLimitAttr",
             &_CreateHullVertexLimitAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxConvexHullsAttr",
             &This::GetMaxConvexHullsAttr)
        .def("CreateMaxConvexHullsAttr",
             &_CreateMaxConvexHullsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinThicknessAttr",
             &This::GetMinThicknessAttr)
        .def("CreateMinThicknessAttr",
             &_CreateMinThicknessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVoxelResolutionAttr",
             &This::GetVoxelResolutionAttr)
        .def("CreateVoxelResolutionAttr",
             &_CreateVoxelResolutionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetErrorPercentageAttr",
             &This::GetErrorPercentageAttr)
        .def("CreateErrorPercentageAttr",
             &_CreateErrorPercentageAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetShrinkWrapAttr",
             &This::GetShrinkWrapAttr)
        .def("CreateShrinkWrapAttr",
             &_CreateShrinkWrapAttr,
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
