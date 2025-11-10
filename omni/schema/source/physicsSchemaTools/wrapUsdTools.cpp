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

#include ".//UsdTools.h"
#include "pxr/pxr.h"

#include <boost/python/def.hpp>

#include "pxr/base/tf/fileUtils.h"

#include <string>
#include <vector>

using std::string;
using std::vector;

using namespace boost::python;

PXR_NAMESPACE_USING_DIRECTIVE

static boost::python::tuple 
_encodeSdfPath(const SdfPath& path) {
	uint32_t part0;
	uint32_t part1;
	encodeSdfPath(path, part0, part1);
	return boost::python::make_tuple(int(part0), int(part1));
}

static boost::python::tuple 
_getMassSpaceInertia(const GfMatrix3f& matrix) {
	GfQuatf pa;
	GfVec3f diagonal = getMassSpaceInertia(matrix, pa);		
	return boost::python::make_tuple(diagonal, pa);
}

void wrapUsdTools() {

	def("createMesh",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, const vector<GfVec3f>&, const vector<GfVec3f>&,
			const vector<int>&, const vector<int>&))
		&createMesh, (arg("stage"), arg("path"),
			arg("points"), arg("normals"), arg("indices"), arg("vertexCounts")));

	def("createMeshSquare",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, float, float))
		&createMeshSquare, (arg("stage"), arg("path"), arg("halfHeight"), arg("halfWidth")));

	def("createMeshBox",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, const pxr::GfVec3f&))
		&createMeshBox, (arg("stage"), arg("path"), arg("halfExtent")));

	def("createMeshSphere",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, float, int, int))
		&createMeshSphere, (arg("stage"), arg("path"), arg("radius"), arg("latitudeSegments"), arg("latitudeSegments")));

	def("createMeshCapsule",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, float, float, int, int))
		&createMeshCapsule, (arg("stage"), arg("path"), arg("radius"), arg("halfHeight"), arg("latitudeSegments"), arg("latitudeSegments")));

	def("createMeshCylinder",
		(UsdGeomMesh(*)(const UsdStagePtr&, const SdfPath&, float, float, uint32_t))
		&createMeshCylinder, (arg("stage"), arg("path"), arg("radius"), arg("halfLength"), arg("tesselation")));

	def("addPhysicsScene",
		(void(*)(const UsdStagePtr&, const string&))
		&addPhysicsScene, (arg("stage"), arg("path")));

	def("addActor",
		(UsdGeomXform(*)(const UsdStagePtr&, const string&))
		&addActor, (arg("stage"), arg("path")));

	def("addCollisionShape",
		(UsdGeomXform(*)(const UsdStagePtr&, const string&))
		&addCollisionShape, (arg("stage"), arg("path")));

	def("addRigidBody",
		(void(*)(const UsdStagePtr&, const string&))
		&addRigidBody, (arg("stage"), arg("path")));

	def("addPosition",
		(void(*)(const UsdGeomXformable&, const GfVec3f&))
		&addPosition, (arg("xformable"), arg("position")));

	def("addOrientation",
		(void(*)(const UsdGeomXformable&, const GfQuatf&))
		&addOrientation, (arg("xformable"), arg("orientation")));

	def("addDisplayColor",
		(void(*)(const UsdGeomXformable&, const GfVec3f&))
		&addDisplayColor, (arg("xformable"), arg("color")));

	def("addVelocity",
		(void(*)(const UsdStagePtr&, const string&, const GfVec3f&, const GfVec3f&))
		&addVelocity, (arg("stage"), arg("path"), arg("linearVelocity"), arg("angularVelocity")));

	def("addDensity",
		(void(*)(const UsdStagePtr&, const string&, float))
		&addDensity, (arg("stage"), arg("path"), arg("value")));

	def("addBoxCollisionShape",
		(void(*)(const UsdStagePtr&, const string&, float, const GfVec3f&,
			const GfQuatf&, const GfVec3f&))
		&addBoxCollisionShape, (arg("stage"), arg("path"),
			arg("size"), arg("position"), arg("orientation"), arg("color")));

	def("addGroundPlane",
		(void(*)(const UsdStagePtr&, const string&, const pxr::TfToken& , float, const GfVec3f&, const GfVec3f&))
		&addGroundPlane, (arg("stage"), arg("path"), arg("axis"),
			arg("size"), arg("position"), arg("color")));

	def("addGroundTriMesh",
		(void(*)(const UsdStagePtr&, const string&, float, const GfVec3f&, const GfVec3f&))
		&addGroundTriMesh, (arg("stage"), arg("path"),
			arg("size"), arg("position"), arg("color")));

	def("addRigidBox",
		(void(*)(const UsdStagePtr&, const string&, const GfVec3f&, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidBox, (arg("stage"), arg("path"),
			arg("size"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidSphere",
		(void(*)(const UsdStagePtr&, const string&, float, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidSphere, (arg("stage"), arg("path"),
			arg("radius"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidSphere",
		(void(*)(const UsdStagePtr&, const string&, float, float, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidSphere, (arg("stage"), arg("path"),
			arg("radius"), arg("halfHeight"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidCapsule",
		(void(*)(const UsdStagePtr&, const string&, float, float, const pxr::TfToken&, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidCapsule, (arg("stage"), arg("path"),
			arg("radius"), arg("halfHeight"), arg("axis"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidCone",
		(void(*)(const UsdStagePtr&, const string&, float, float, const pxr::TfToken&, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidCone, (arg("stage"), arg("path"),
			arg("radius"), arg("halfHeight"), arg("axis"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidCylinder",
		(void(*)(const UsdStagePtr&, const string&, float, float, const pxr::TfToken&, const GfVec3f&,
			const GfQuatf&, const GfVec3f&, float, const GfVec3f&, const GfVec3f&))
		&addRigidCylinder, (arg("stage"), arg("path"),
			arg("radius"), arg("halfHeight"), arg("axis"), arg("position"), arg("orientation"), arg("color"), arg("density"), arg("linVelocity"), arg("angularVelocity")));

	def("addRigidBoxForInstancing",
		(void(*)(const UsdStagePtr&, const string&, const GfVec3f&, const GfVec3f&, float))
		&addRigidBoxForInstancing, (arg("stage"), arg("path"),
			arg("size"), arg("color"), arg("density")));


	def("encodeSdfPath",	&_encodeSdfPath);

	def("decodeSdfPath",
		(SdfPath(*)(const uint32_t, const uint32_t))
		&decodeSdfPath, (arg("part0"), arg("part1")));

	def("intToSdfPath",
		(SdfPath(*)(const uint64_t))
		&intToSdfPath, (arg("intPath")));

	def("sdfPathToInt",
		(uint64_t(*)(const SdfPath&))
		&sdfPathToInt, (arg("SdfPath")));

	def("getMassSpaceInertia",	&_getMassSpaceInertia);

}

