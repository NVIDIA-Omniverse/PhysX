# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
import omni.physx
import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os
import carb
import unittest

class OmniGraphPhysxImmediateTest(ogts.OmniGraphTestCase):

    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()
 
    async def _load_usd(self, filename):
        data_path = "../../../../data/tests"
        tests_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        tests_folder = tests_folder.replace("\\", "/") + "/"
        filepath = tests_folder + filename + ".usda"
        await omni.usd.get_context().open_stage_async(filepath)
        self._usd_context = omni.usd.get_context()
        self.assertIn(filename, self._usd_context.get_stage_url())
        self._stage = self._usd_context.get_stage()    

    async def _timeline_play_steps(self, num_frames):

        omni.timeline.get_timeline_interface().play()
        for _ in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

    def _assertAlmostEqualArray(self, vectorA, vectorB):
        for a, b in zip(vectorA, vectorB):
            self.assertAlmostEqual(a, b, delta=0.0001)    

    async def test_immediate_compute_mesh_intersecting_faces(self):
        # This tests loads a usda file that creates a scene with 2 cubes intersecting and 2 cubes not intersecting.
        # It's suggested loading the scene to see how the graph is composed.
        # After computing intersecting triangles we have other nodes that check that the actual triangle vertices
        # are the ones that we expect in the case of intersecting cubes, and the "empty" one (1e-10 sized) for
        # the missing intersecting (as it's not possible to delete a mesh in OG or into the stage from OG as of today).
        # All of the tests are combined into a boolean condition that resides in the 'test_pass' node as its result.
        #
        await self._load_usd("immediate/OgnPhysXImmediateComputeMeshIntersectingFaces")
        await omni.kit.app.get_app().next_update_async()
        ogController = og.Controller()
        graph = ogController.graph("/World/ActionGraph")
        await self._timeline_play_steps(10)
        test_pass = ogController.node("/World/ActionGraph/test_pass")
        test_result = og.Controller(ogController.attribute("outputs:result", test_pass)).get()
        self.assertEqual(test_result, True)
        await omni.usd.get_context().close_stage_async()

    async def test_immediate_generate_geometry_contacts(self):
        # This is a twin test as test_immediate_compute_mesh_intersecting_faces (read comments over there for details)
        await self._load_usd("immediate/OgnPhysXImmediateGenerateGeometryContacts")
        await omni.kit.app.get_app().next_update_async()
        ogController = og.Controller()
        graph = ogController.graph("/World/ActionGraph")
        await self._timeline_play_steps(10)
        test_pass = ogController.node("/World/ActionGraph/test_pass")
        test_result = og.Controller(ogController.attribute("outputs:result", test_pass)).get()
        self.assertEqual(test_result, True)
        await omni.usd.get_context().close_stage_async()

    async def test_immediate_compound(self):
        # This compound test with the loaded scene needs correct behaviour from all nodes in order for the result to be
        # correct, as all node results are chained into each oter. For now this is the only test that we need, we may
        # be adding more specialized tests later on
        await self._load_usd("immediate/OgnPhysXImmediateCompound")

        
        og.Controller.edit({
                "evaluator_name": "execution",
                "graph_path": "/TestGraph",
            })

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, test_nodes, _, _) = ogController.edit("/TestGraph",
            {
                keys.CREATE_NODES: [
                    ("physx_compute_bounding_boxes", "omni.physx.graph.ImmediateComputeGeometryBounds"),
                    ("physx_bounding_boxes_overlap", "omni.physx.graph.ImmediateComputeBoundsOverlaps"),
                    ("physx_geometries_overlap", "omni.physx.graph.ImmediateComputeGeometryOverlaps"),
                    ("physx_geometries_penetration", "omni.physx.graph.ImmediateComputeGeometryPenetrations"),
                    ("tick", "omni.graph.action.OnTick"),
                    ("readPrims", "omni.graph.nodes.ReadPrims"),
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "physx_compute_bounding_boxes.inputs:execIn"),
                    ("readPrims.outputs_primsBundle", "physx_compute_bounding_boxes.inputs:primsBundle"),
                    ("physx_compute_bounding_boxes.outputs:execOut", "physx_bounding_boxes_overlap.inputs:execIn"),
                    ("physx_compute_bounding_boxes.outputs_primsBundle", "physx_bounding_boxes_overlap.inputs:primsBundle"),
                    ("physx_compute_bounding_boxes.outputs_primsBundle", "physx_geometries_overlap.inputs:primsBundle"),
                    ("physx_compute_bounding_boxes.outputs_primsBundle", "physx_geometries_penetration.inputs:primsBundle"),
                    ("physx_bounding_boxes_overlap.outputs:execOut", "physx_geometries_overlap.inputs:execIn"),
                    ("physx_bounding_boxes_overlap.outputs:execOut", "physx_geometries_penetration.inputs:execIn"),
                    ("physx_bounding_boxes_overlap.outputs:overlapsPair0", "physx_geometries_overlap.inputs:overlapsPair0"),
                    ("physx_bounding_boxes_overlap.outputs:overlapsPair1", "physx_geometries_overlap.inputs:overlapsPair1"),
                    ("physx_bounding_boxes_overlap.outputs:overlapsPair0", "physx_geometries_penetration.inputs:overlapsPair0"),
                    ("physx_bounding_boxes_overlap.outputs:overlapsPair1", "physx_geometries_penetration.inputs:overlapsPair1"),
                ],
                keys.SET_VALUES: [
                    ("readPrims.inputs:useFindPrims", True),
                    ("readPrims.inputs:pathPattern", "/World/AllPrims/*"),
                    ("readPrims.inputs:typePattern", ""),
                    ("readPrims.inputs:attrNamesToImport", "*"),
                ]
            }
        )

        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)

        in_overlapsPair0 = og.Controller(ogController.attribute("inputs:overlapsPair0", test_nodes[2])).get()
        in_overlapsPair1 = og.Controller(ogController.attribute("inputs:overlapsPair1", test_nodes[2])).get()
        out_overlaps = og.Controller(ogController.attribute("outputs:overlaps", test_nodes[2]))
        overlaps = out_overlaps.get()
        self.assertEqual(len(overlaps), 2)
        overlapPrim = '/World/AllPrims/ConeMeshOverlap'
        if in_overlapsPair0[0] == overlapPrim or in_overlapsPair1[0] == overlapPrim:
            self.assertEqual(overlaps[0], True)
            self.assertEqual(overlaps[1], False)
        else:
            self.assertEqual(overlaps[0], False)
            self.assertEqual(overlaps[1], True)

        # Compute Penetration Node
        out_overlaps = og.Controller(ogController.attribute("outputs:overlaps", test_nodes[3]))
        overlaps = out_overlaps.get()
        out_penetration_depths = og.Controller(ogController.attribute("outputs:penetrationDepths", test_nodes[3]))
        penetration_depths = out_penetration_depths.get()
        out_penetration_vectors = og.Controller(ogController.attribute("outputs:penetrationVectors", test_nodes[3]))
        penetration_vectors = out_penetration_vectors.get()
        self.assertEqual(len(overlaps), 2)
        self.assertEqual(len(penetration_depths), 2)
        self.assertEqual(len(penetration_vectors), 2)
        if in_overlapsPair0[0] == overlapPrim or in_overlapsPair1[0] == overlapPrim:
            self.assertEqual(overlaps[0], True)
            self.assertEqual(overlaps[1], False)
            self.assertAlmostEqual(penetration_depths[0], 8.998001, delta=0.0001)
            self.assertEqual(penetration_depths[1], 0)
            if in_overlapsPair0[0] == overlapPrim:
                self._assertAlmostEqualArray(penetration_vectors[0], [0,-1,0])
            else:
                self._assertAlmostEqualArray(penetration_vectors[0], [0,1,0])
            self._assertAlmostEqualArray(penetration_vectors[1], [0,0,0])
        else:
            self.assertEqual(overlaps[0], False)
            self.assertEqual(overlaps[1], True)
            self.assertEqual(penetration_depths[0], 0)
            self.assertAlmostEqual(penetration_depths[1], 8.998001, delta=0.0001)
            self._assertAlmostEqualArray(penetration_vectors[0], [0,0,0])
            if in_overlapsPair0[1] == overlapPrim:
                self._assertAlmostEqualArray(penetration_vectors[1], [0,-1,0])
            else:
                self._assertAlmostEqualArray(penetration_vectors[1], [0,1,0])

        await omni.usd.get_context().close_stage_async()


    async def _test_immediate_raycast_targets(self):
        test_error_msg_prefix = "Test failed for node \"Raycast\" (target input): "
        ogController = og.Controller()
        keys = ogController.Keys
        (graph, test_nodes, _, _) = ogController.edit("/TestGraph",
            {
                keys.CREATE_NODES: [
                    ("physxogn", "omni.physx.graph.ImmediateComputeGeometryRaycastHits"),
                    ("const_direction", "omni.graph.nodes.ConstantFloat3"),
                    ("const_origin", "omni.graph.nodes.ConstantPoint3f"),
                    ("tick", "omni.graph.action.OnTick"),
                    ("const_range", "omni.graph.nodes.ConstantFloat"),
                    ("find_prims", "omni.graph.nodes.FindPrims")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "physxogn.inputs:execIn"),
                    ("find_prims.outputs:prims", "physxogn.inputs:prims"),
                    ("const_origin.inputs:value", "physxogn.inputs:origin"),
                    ("const_range.inputs:value", "physxogn.inputs:raycastRange"),
                    ("const_direction.inputs:value", "physxogn.inputs:direction")
                ],
                keys.SET_VALUES: [
                    ("find_prims.inputs:rootPrim", "/World"),
                    ("find_prims.inputs:type", "Mesh*"),
                    ("find_prims.inputs:recursive", True),
                    ("physxogn.inputs:bothSides", True),
                    ("const_range.inputs:value", 100.0),
                    ("const_origin.inputs:value", [10.0, 50.0, 0.0])
                ]
            }
        )

        out_prims = og.Controller(ogController.attribute("outputs:primsHit", test_nodes[0]))
        in_direction = og.Controller(ogController.attribute("inputs:value", test_nodes[1]))

        in_direction.set([0.0, 1.0, 0.0])
        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)
        prims = out_prims.get()
        if len(prims) == 0:
            self.fail(test_error_msg_prefix + "failed to find prim.")
        else:
            prim_paths = ""
            if(len(prims) > 1):
                for n in range(len(prims)):
                    prim_paths += " " + str(prims[n].GetPrimPath())
                    
            self.assertEqual(len(prims), 1, test_error_msg_prefix + "expected to find 1 prim in but found "+str(len(prims))+":"+prim_paths)
            self.assertEqual(prims[0].GetPrimPath(), "/World/Cube", test_error_msg_prefix + "found unexpected prim: "+str(prims[0].GetPrimPath()))
                    

        in_direction.set([0.0, -1.0, 0.0])
        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)
        prims = out_prims.get()
        if len(prims) == 0:
            self.fail(test_error_msg_prefix + "failed to find prim.")
        else:
            self.assertEqual(len(prims), 2, test_error_msg_prefix + "expected to find 2 prims in but found "+str(len(prims)))

        ogController.delete_node(test_nodes)


    async def _test_immediate_raycast_bundle(self):
        test_error_msg_prefix = "Test failed for node \"Raycast\" (bundle input): "
        ogController = og.Controller()
        keys = ogController.Keys
        (graph, test_nodes, _, _) = ogController.edit("/TestGraph",
            {
                keys.CREATE_NODES: [
                    ("physxogn", "omni.physx.graph.ImmediateComputeGeometryRaycastHits"),
                    ("const_direction", "omni.graph.nodes.ConstantFloat3"),
                    ("const_origin", "omni.graph.nodes.ConstantPoint3f"),
                    ("tick", "omni.graph.action.OnTick"),
                    ("const_range", "omni.graph.nodes.ConstantFloat"),
                    ("find_prims", "omni.graph.nodes.FindPrims"),
                    ("readPrims", "omni.graph.nodes.ReadPrimsV2")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "physxogn.inputs:execIn"),
                    ("find_prims.outputs:prims", "readPrims.inputs:prims"),
                    ("const_origin.inputs:value", "physxogn.inputs:origin"),
                    ("const_range.inputs:value", "physxogn.inputs:raycastRange"),
                    ("const_direction.inputs:value", "physxogn.inputs:direction"),
                    ("readPrims.outputs_primsBundle", "physxogn.inputs:primsBundle")
                ],
                keys.SET_VALUES: [
                    ("find_prims.inputs:rootPrim", "/World"),
                    ("find_prims.inputs:type", "Mesh*"),
                    ("find_prims.inputs:recursive", True),
                    ("physxogn.inputs:bothSides", True),
                    ("const_range.inputs:value", 100.0),
                    ("const_origin.inputs:value", [10.0, 50.0, 0.0]),
                    ("readPrims.inputs:computeBoundingBox", True)
                ]
            }
        )

        out_prims = og.Controller(ogController.attribute("outputs:primsHit", test_nodes[0]))
        in_direction = og.Controller(ogController.attribute("inputs:value", test_nodes[1]))

        in_direction.set([0.0, 1.0, 0.0])
        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)
        prims = out_prims.get()
        if len(prims) == 0:
            self.fail(test_error_msg_prefix + "failed to find prim.")
        else:
            prim_paths = ""
            if(len(prims) > 1):
                for n in range(len(prims)):
                    prim_paths += " " + str(prims[n].GetPrimPath())
                    
            self.assertEqual(len(prims), 1, test_error_msg_prefix + "expected to find 1 prim in but found "+str(len(prims))+":"+prim_paths)
            self.assertEqual(prims[0].GetPrimPath(), "/World/Cube", test_error_msg_prefix + "found unexpected prim: "+str(prims[0].GetPrimPath()))
                    

        in_direction.set([0.0, -1.0, 0.0])
        await self._timeline_play_steps(1)
        await ogController.evaluate(graph)
        prims = out_prims.get()
        if len(prims) == 0:
            self.fail(test_error_msg_prefix + "failed to find prim.")
        else:
            self.assertEqual(len(prims), 2, test_error_msg_prefix + "expected to find 2 prims in but found "+str(len(prims)))

        ogController.delete_node(test_nodes)


    async def test_immediate_raycast(self):
        await self._load_usd("immediate/OgnPhysXImmediateComputeRaycast")

        og.Controller.edit({
                "evaluator_name": "execution",
                "graph_path": "/TestGraph",
            })

        await self._test_immediate_raycast_targets()
        await self._test_immediate_raycast_bundle()

        await omni.usd.get_context().close_stage_async()
