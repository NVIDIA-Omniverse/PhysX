# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-IO-CONTRACT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Generator regression checks; no runtime or GPU dependencies."""

import copy
import importlib.util
import json
import re
import shutil
import tempfile
import unittest
from pathlib import Path

DOMAIN = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("gen_contract", DOMAIN / "tools/gen_contract.py")
gen = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gen)


class ContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.bindings = gen.load_contract()

    def rejected_edit(self, filename, change, message):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            domain = root / "physics_io_domain"
            shutil.copytree(DOMAIN, domain, ignore=shutil.ignore_patterns("__pycache__"))
            shutil.copytree(DOMAIN.parent / "physics_population_domain/apis", root / "physics_population_domain/apis")
            shutil.copytree(
                DOMAIN.parent / "source/omni.physx/plugins",
                root / "source/omni.physx/plugins",
                ignore=shutil.ignore_patterns("tensors", "particles", "__pycache__"),
            )
            path = domain / filename
            data = json.loads(path.read_text())
            change(data)
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, message):
                gen.load_contract(domain)

    def test_bad_metadata_is_rejected(self):
        cases = [
            ("unknown units", lambda a: a["uses"][0].update(units="mystery")),
            ("unknown policy", lambda a: a["uses"][0]["read"].update(conditions=["mystery"])),
            ("missing count companion", lambda a: a["uses"][0]["read"]["layout"].update(count="missing")),
            ("required property", lambda a: a["uses"][0].pop("frame")),
            ("duplicate use", lambda a: a["uses"].append(copy.deepcopy(a["uses"][0]))),
            ("alias shadows", lambda a: a["uses"][0].update(aliases=[a["name"]])),
        ]
        for message, change in cases:
            with self.subTest(message=message):
                self.rejected_edit("attributes/rigid_body.json", lambda data: change(data["attributes"][0]), message)

    def test_compatibility_baseline_is_enforced(self):
        self.rejected_edit(
            "attributes/rigid_body.json",
            lambda data: data["attributes"][0].update(name="renamed"),
            "compatibility|missing canonical request",
        )
        self.rejected_edit(
            "attributes/rigid_body.json",
            lambda data: data["attributes"][0].update(c_symbol="OVPHYSX_ATTR_RENAMED"),
            "compatibility",
        )
        self.rejected_edit("targets.json", lambda data: data["targets"].update(RIGID_BODY=99), "compatibility")

    def test_binding_cannot_contain_code_or_unknown_attribute(self):
        def alter(data):
            values = data["tables"][0]["rows"][0]["values"]
            values[next(iter(values))] = {"cpp": "run_arbitrary_code()"}

        self.rejected_edit("bindings/OvxPhysicsRead.json", alter, "not executable code")
        self.rejected_edit(
            "bindings/OvxPhysicsRead.json",
            lambda d: d["tables"][0]["rows"][0].update(attribute="missing"),
            "unknown or duplicate",
        )

    def test_wire_metadata_compatibility(self):
        baseline = gen.read_json(DOMAIN / "compatibility.json")
        changes = [
            lambda u: u.update(dtype="u32"),
            lambda u: u.update(units="dimensionless"),
            lambda u: u.update(frame="local"),
            lambda u: u["read"]["layout"].update(lanes=7),
            lambda u: u["read"].update(residency="cuda"),
        ]
        for change in changes:
            model = copy.deepcopy(self.model)
            attr = next(a for a in model["attributes"] if a["name"] == "linearVelocity")
            change(attr["uses"][0])
            with self.assertRaisesRegex(ValueError, "compatibility: changed metadata"):
                gen.check_compatibility(model, baseline)
        model = copy.deepcopy(self.model)
        added = copy.deepcopy(model["attributes"][0])
        added.update(id="new_attribute", name="newAttribute", c_symbol="OVPHYSX_ATTR_NEW_ATTRIBUTE", public=True)
        model["attributes"].append(added)
        with self.assertRaisesRegex(ValueError, "public symbols require"):
            gen.check_compatibility(model, baseline)

    def test_shared_target_metadata(self):
        # Exercise binding checks directly so the compatibility guard cannot mask them.
        for target in ("ARTICULATION_LINK", "SPATIAL_TENDON"):
            for field in ("dtype", "lanes", "residency"):
                with self.subTest(target=target, field=field):
                    model = copy.deepcopy(self.model)
                    table = next(
                        t
                        for doc in self.bindings
                        for t in doc["tables"]
                        if target in t["targets"] and t["target"] != target and t["direction"] == "read"
                    )
                    attr = next(
                        a
                        for a in model["attributes"]
                        if a["id"] in {r["attribute"] for r in table["rows"]}
                        and any(u["target"] == target for u in a["uses"])
                    )
                    use = next(u for u in attr["uses"] if u["target"] == target)
                    if field == "dtype":
                        use["dtype"] = "f64"
                    elif field == "lanes":
                        use["read"]["layout"]["lanes"] = 17
                    else:
                        use["read"]["residency"] = "cuda"
                    with self.assertRaisesRegex(ValueError, "inconsistent shared target metadata"):
                        gen.check_bindings(model, self.bindings)

    def test_vehicle_constraints_and_generated_documentation(self):
        outputs = gen.generate(self.model, self.bindings)
        vehicle = outputs["docs/read_write/attributes/vehicle.md"]
        wheel = outputs["docs/read_write/attributes/vehicle_wheel.md"]
        for phrase in (
            "[0,1]",
            "[-1,1]",
            "[-1,30]",
            "255",
            "0 or 1",
            "DriveBasic / DriveStandard",
            "DriveStandard",
            "Tank drive only",
        ):
            self.assertIn(phrase, vehicle)
        self.assertIn(">= 0", wheel)
        header = outputs["include/ovphysx/ovphysx_attributes.h"]
        for phrase in ("xyzw", "scene_device", "frame: world", "J=J_physx*T"):
            self.assertIn(phrase, header)
        self.assertNotIn("&#", header)
        self.assertNotIn(" See the runtime", header)
        self.assertEqual(gen.comment("dense Jacobian"), "dense Jacobian.")
        self.assertEqual(gen.comment("rotation &#8212; xyzw"), "rotation -- xyzw.")

    def test_public_header_shared_conditions_and_layouts(self):
        header = gen.generate(self.model, self.bindings)["include/ovphysx/ovphysx_attributes.h"]
        for policy in self.model["policies"].values():
            self.assertEqual(header.count(gen.comment(policy["description"])), 1)
        anchors = set(re.findall(r"@subsection (ovphysx_io_\w+)", header))
        references = set(re.findall(r"@ref (ovphysx_io_\w+)", header))
        self.assertTrue(references <= anchors)
        self.assertNotIn("@ref ovphysx_io_session", header)
        self.assertNotIn("shape ['", header)
        self.assertIn("shape [objects]", header)
        self.assertIn("column major", header)
        self.assertIn("count `shapeCount`; padding 0", header)
        velocity = next(a for a in self.model["attributes"] if a["name"] == "linearVelocity")
        lines = gen.public_comment(velocity, self.model).splitlines()
        self.assertTrue(any("Read:" in line and "@ref ovphysx_io_directgpu_warmup" in line for line in lines))
        self.assertTrue(any("Write:" in line and "@ref ovphysx_io_rigid_velocity" in line for line in lines))
        self.assertFalse(any("Read:" in line and "@ref ovphysx_io_rigid_velocity" in line for line in lines))

    def test_explicit_runtime_units(self):
        units = self.model["units"]
        expected = {
            "dimensionless": "dimensionless",
            "length": "distance",
            "mass": "mass",
            "time": "sec",
            "velocity": "distance / sec",
            "angular_velocity": "rad / sec",
            "force": "mass * distance / sec^2",
            "inertia": "mass * distance^2",
            "linear_stiffness": "mass / sec^2",
        }
        for name, display in expected.items():
            self.assertEqual(units[name]["display"], display)
        self.assertEqual(units["elasticity_damping"]["variants"], {"volume": "sec", "surface": "1 / sec"})
        self.assertEqual(units["joint_stiffness"]["variants"]["angular_acceleration_drive"], "rad / (deg * sec^2)")
        self.assertEqual(
            units["centroidal_momentum_matrix"]["variants"]["angular_row_bias"], "mass * distance^2 / sec^2"
        )
        for unit in units.values():
            self.assertNotIn("dimensions", unit)
            self.assertTrue(all(isinstance(v, str) for v in unit.get("variants", {}).values()))
        outputs = gen.generate(self.model, self.bindings)
        self.assertIn("distance / sec", outputs["docs/read_write/attributes/rigid_body.md"])
        self.assertIn("rad / (deg * sec^2)", outputs["include/ovphysx/ovphysx_attributes.h"])
        reference = outputs["docs/read_write/attributes/units.md"]
        self.assertIn("bias force/torque", reference)
        self.assertIn("Unit authored gearing angular", reference)

    def test_population_exponents_and_invalid_units_rejected(self):
        cases = [
            ("unknown unit field", lambda u: u["length"].update(dimensions={"length": 1})),
            ("explicit unit display", lambda u: u["dimensionless"].update(display="1")),
            ("explicit unit display", lambda u: u["length"].update(display=1)),
            ("explicit unit display", lambda u: u["length"].update(display=" ")),
            ("explicit unit strings", lambda u: u["elasticity_damping"].update(variants={"volume": {"time": 1}})),
            ("explicit unit strings", lambda u: u["elasticity_damping"].update(variants={"volume": 1})),
            ("explicit unit strings", lambda u: u["elasticity_damping"].update(variants={"volume": ""})),
            ("unknown component unit", lambda u: u["wrench"].update(components=["missing"])),
            ("unknown component unit", lambda u: u["wrench"].update(components="axis_dependent")),
            ("component unit count", lambda u: u["wrench"].update(components=["force"])),
        ]
        for message, change in cases:
            with self.subTest(message=message):
                self.rejected_edit("units.json", lambda d: change(d["units"]), message)

    def test_reversed_value_range_rejected(self):
        self.rejected_edit(
            "attributes/vehicle.json",
            lambda d: d["attributes"][0]["uses"][0]["write"].update(value_constraints={"minimum": 1, "maximum": 0}),
            "reversed value range",
        )

    def test_missing_backend_binding_rejected(self):
        self.rejected_edit(
            "bindings/OvxPhysicsRead.json", lambda d: d["tables"][0]["rows"].pop(), "missing backend bindings"
        )
        self.rejected_edit(
            "bindings/Dispatch.json",
            lambda d: d["handlers"][0].update(symbol="missingHandler"),
            "missing backend handler",
        )

    def test_duplicate_json_keys_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text('{"x":1,"x":2}')
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                gen.read_json(path)

    def test_schema_rule_validation(self):
        # Exercise every assertion used by the committed schema, including JSON's
        # distinction between booleans and numbers and nested equality for uniqueness.
        cases = [
            ({"type": "object"}, {}, []),
            ({"type": "array"}, [], {}),
            ({"type": "string"}, "a", 1),
            ({"type": "boolean"}, True, 1),
            ({"type": "integer"}, 2, True),
            ({"type": "integer"}, 2.0, 2.5),
            ({"type": "number"}, 0.5, False),
            ({"type": "number"}, 0, float("inf")),
            ({"type": "number"}, -1, float("nan")),
            ({"const": 1}, 1, True),
            ({"enum": [0, 1]}, 0, False),
            ({"type": "string", "minLength": 1}, "a", ""),
            ({"type": "string", "pattern": "^[a-z]+$"}, "abc", "a-b"),
            ({"type": "string", "pattern": "^[a-z]+$"}, "abc", "abc\n"),
            ({"type": "string", "pattern": "part"}, "a-part-b", "absent"),
            ({"type": "string", "pattern": r"price\$"}, "price$ suffix", "price"),
            ({"type": "string", "pattern": "[$]"}, "a$b", "abc"),
            ({"type": "number", "minimum": 1}, 1, 0),
            ({"type": "number", "maximum": 1}, 1, 2),
            ({"type": "array", "minItems": 1}, [1], []),
            ({"type": "array", "items": {"type": "string"}}, ["a"], ["a", 1]),
            ({"uniqueItems": True}, [0, False], [1, 1.0]),
            ({"uniqueItems": True}, [{"a": [1]}, {"a": [True]}], [{"a": [1]}, {"a": [1.0]}]),
            ({"type": "object", "minProperties": 1}, {"a": 1}, {}),
            ({"required": ["a"]}, {"a": 1}, {}),
            ({"properties": {"a": {"type": "boolean"}}, "additionalProperties": False}, {"a": True}, {"b": True}),
            ({"properties": {"a": {"type": "boolean"}}}, {"a": True}, {"a": 1}),
            ({"dependentRequired": {"maximum": ["minimum"]}}, {"minimum": 0, "maximum": 1}, {"maximum": 1}),
            ({"not": {"required": ["values", "minimum"]}}, {"values": [1]}, {"values": [1], "minimum": 0}),
            ({"oneOf": [{"type": "integer"}, {"type": "string"}]}, "width", False),
            ({"oneOf": [{"type": "number"}, {"type": "integer"}]}, 0.5, 1),
        ]
        for schema, accepted, rejected in cases:
            with self.subTest(schema=schema, rejected=rejected):
                gen.check_schema_rules(schema)
                gen.validate_schema(accepted, schema)
                with self.assertRaises(ValueError):
                    gen.validate_schema(rejected, schema)

    def test_unsupported_schema_rules_fail_closed(self):
        for schema in [
            {"format": "uri"},
            {"properties": {"x": {"minLenght": 1}}},
            {"items": {"format": "uri"}},
            {"oneOf": [{"type": "string"}, {"format": "uri"}]},
            {"not": {"format": "uri"}},
            {"additionalProperties": True},
            {"type": "null"},
            {"$schema": "https://example.com/schema"},
        ]:
            with self.subTest(schema=schema), self.assertRaises(ValueError):
                gen.check_schema_rules(schema)

    def test_contract_rejects_wrong_types_and_unknown_fields(self):
        mutations = [
            lambda d: d.update(schema_version=True),
            lambda d: d["attributes"][0].update(public=1),
            lambda d: d["attributes"][0].update(unexpected=True),
            lambda d: d["attributes"][0]["uses"][0]["read"]["layout"].update(lanes=False),
            lambda d: d["attributes"][0]["uses"][0]["read"]["layout"].update(lanes=0),
            lambda d: d["attributes"][0]["uses"][0]["read"]["layout"].update(lanes=65536),
            lambda d: d["attributes"][0]["uses"][0]["read"]["layout"].update(shape=[]),
            lambda d: d["attributes"][0]["uses"][0]["read"].update(conditions=["session", "session"]),
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.rejected_edit("attributes/rigid_body.json", mutation, "rigid_body.json")

    def test_nonfinite_json_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for value in ("NaN", "Infinity", "-Infinity"):
                path.write_text('{"value":' + value + "}")
                with self.subTest(value=value), self.assertRaisesRegex(ValueError, "non-finite JSON"):
                    gen.read_json(path)

    def test_semantic_table_is_sorted_and_complete(self):
        output = gen.generate(self.model, self.bindings)[
            "ovruntime/source/omni.physx/plugins/generated/AttributeSemantics.inc"
        ]
        rows = re.findall(r"\{ omni::physx::OvxAttr::(\w+), OVSTAGE_SEMANTIC_(\w+) \}", output)
        names = {a["cpp_symbol"]: a["name"] for a in self.model["attributes"]}
        actual = [(names[symbol], role) for symbol, role in rows]
        expected = sorted((a["name"], a["semantic"]) for a in self.model["attributes"] if a["semantic"] != "NONE")
        self.assertEqual(actual, expected)
        self.assertEqual(len(actual), len(set(name for name, _ in actual)))
        self.assertIn(("positions", "POINT"), actual)
        self.assertIn(("orientations", "QUATERNION"), actual)
        self.assertIn(("inertia", "MATRIX"), actual)

    def test_deterministic_and_committed(self):
        first = gen.generate(self.model, self.bindings)
        self.assertEqual(first, gen.generate(*gen.load_contract()))
        self.assertEqual(gen.write_outputs(first, check=True), [])

    def test_authoring_defaults_and_explicit_overrides(self):
        document = gen.read_json(DOMAIN / "attributes/rigid_body.json")
        original = copy.deepcopy(document)
        result = gen.normalize(document)
        self.assertEqual(document, original)
        for authored, resolved in zip(document["attributes"], result["attributes"]):
            for use in resolved["uses"]:
                self.assertIn("representation", use)
                self.assertIn("aliases", use)
                for direction in ("read", "write"):
                    access = use[direction]
                    if access["supported"]:
                        self.assertIn("request_name", access)
                        self.assertIn("order", access["layout"])
                    else:
                        self.assertNotIn("layout", access)
                        self.assertNotIn("residency", access)
            self.assertTrue(resolved["c_symbol"].startswith("OVPHYSX_ATTR_"))
        self.assertEqual(
            gen.symbol_defaults("centerOfMassPosition"),
            {"c_symbol": "OVPHYSX_ATTR_CENTER_OF_MASS_POSITION", "cpp_symbol": "kCenterOfMassPosition"},
        )
        attr = document["attributes"][0]
        attr["cpp_symbol"] = "kExplicitOverride"
        access = attr["uses"][0]["read"]
        access.update(request_name="specialRequest", conditions=[], residency="host", aliases=["lookup"])
        access["layout"]["order"] = "column_major"
        resolved = gen.normalize(document)["attributes"][0]
        self.assertEqual(resolved["cpp_symbol"], "kExplicitOverride")
        for key in ("request_name", "conditions", "residency", "aliases", "layout"):
            self.assertEqual(resolved["uses"][0]["read"][key], access[key])
        self.rejected_edit(
            "attributes/rigid_body.json",
            lambda doc: doc["attributes"][0].update(name="invalid-name"),
            "invalid string pattern",
        )
        self.rejected_edit(
            "attributes/rigid_body.json",
            lambda doc: doc["attributes"][0]["uses"][0]["read"].update(residency=None),
            "allowed form",
        )

    def test_complete_baseline_and_explicit_update(self):
        baseline = gen.compatibility_baseline(self.model)
        self.assertEqual(gen.read_json(DOMAIN / "compatibility.json"), baseline)
        self.assertEqual(json.loads(gen.baseline_text(self.model)), baseline)
        for section in ("cpp_symbols", "targets", "writability", "uses"):
            for addition in (False, True):
                changed = copy.deepcopy(baseline)
                rows = changed[section]
                if isinstance(rows, dict):
                    if addition:
                        rows["extra"] = 123
                    else:
                        rows.pop(next(iter(rows)))
                elif addition:
                    rows.append({**rows[0], "name": "extra"})
                else:
                    rows.pop()
                with (
                    self.subTest(section=section, addition=addition),
                    self.assertRaisesRegex(ValueError, "compatibility"),
                ):
                    gen.check_compatibility(self.model, changed)
        model = copy.deepcopy(self.model)
        model["attributes"][0]["cpp_symbol"] = "kRenamed"
        with self.assertRaisesRegex(ValueError, "cpp_symbols"):
            gen.check_compatibility(model, baseline)
        gen.check_compatibility(model, gen.compatibility_baseline(model))
        # Validation alone must not silently update the baseline.
        before = (DOMAIN / "compatibility.json").read_bytes()
        gen.load_contract()
        self.assertEqual(before, (DOMAIN / "compatibility.json").read_bytes())

    def test_ascii_outputs_and_target_completeness(self):
        outputs = gen.generate(self.model, self.bindings)
        for name, content in outputs.items():
            if Path(name).suffix in (".inc", ".h", ".cpp"):
                self.assertTrue(content.isascii(), name)
        self.assertIn(
            f'OVPHYSX_QUERY_TARGET_COUNT == {len(self.model["targets"])}',
            outputs["src/ovphysx/generated/AttributeNames.inc"],
        )
        self.assertNotIn("include/ovphysx/ovphysx_attribute_tokens.h", outputs)
        self.assertNotIn("ovruntime/include/omni/physx/generated/AttributeDescriptors.h", outputs)
        with self.assertRaisesRegex(ValueError, "non-ASCII"):
            gen.comment("unhandled \u2603")

    def test_directional_culling_box_units_and_tendon_gearing(self):
        units = self.model["units"]
        self.assertEqual(units["tendon_coordinate"]["variants"]["unit_authored_gearing_angular"], "deg")
        self.assertEqual(
            units["tendon_stiffness"]["variants"]["unit_authored_gearing_angular"], "mass * distance^2 / (sec^2 * deg)"
        )
        self.assertEqual(
            units["tendon_damping"]["variants"]["unit_authored_gearing_angular"], "mass * distance^2 / (sec * deg)"
        )
        self.assertFalse(any("without_degree_conversion" in k for u in units.values() for k in u.get("variants", {})))
        use = next(a for a in self.model["attributes"] if a["name"] == "cullingBox")["uses"][0]
        self.assertEqual(use["units"], "length")
        self.assertEqual(units[use["write"]["units"]]["components"], ["dimensionless"] + ["length"] * 6)
        outputs = gen.generate(self.model, self.bindings)
        self.assertIn("Write units: enable flag: dimensionless", outputs["include/ovphysx/ovphysx_attributes.h"])
        self.assertIn("units.md#culling-box-write", outputs["docs/read_write/attributes/debug_render.md"])

    def test_rejected_output_changes_are_atomic(self):
        for bad in (
            "../outside.md",
            "docs/read_write/attributes/../../../include/manual.h",
            "/outside/outside.md",
            "old.md",
        ):
            for check in (False, True):
                with self.subTest(bad=bad, check=check), tempfile.TemporaryDirectory() as directory:
                    project = Path(directory)
                    manifest = project / "ovruntime/physics_io_domain/generated-files.json"
                    manifest.parent.mkdir(parents=True)
                    manifest.write_text(json.dumps(["current.inc", bad]))
                    (project / "current.inc").write_text(gen.BANNER + "original")
                    (project / "old.md").write_text("hand-written document")
                    before = {str(f.relative_to(project)): f.read_bytes() for f in project.rglob("*") if f.is_file()}
                    with self.assertRaisesRegex(ValueError, "unsafe|unowned"):
                        gen.write_outputs({"current.inc": gen.BANNER + "changed"}, project, check=check)
                    self.assertEqual(
                        before, {str(f.relative_to(project)): f.read_bytes() for f in project.rglob("*") if f.is_file()}
                    )

    def test_output_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            (project / "real").mkdir()
            try:
                (project / "link").symlink_to(project / "real", target_is_directory=True)
            except OSError as exc:
                self.skipTest(f"Symlinks unavailable on this host: {exc}")
            with self.assertRaisesRegex(ValueError, "symlink"):
                gen.write_outputs({"link/generated.inc": gen.BANNER}, project)
            self.assertFalse((project / "real/generated.inc").exists())

    def test_missing_modified_and_obsolete_outputs(self):
        outputs = gen.generate(self.model, self.bindings)
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            (project / "ovruntime/physics_io_domain").mkdir(parents=True)
            self.assertTrue(gen.write_outputs(outputs, project))
            self.assertEqual(gen.write_outputs(outputs, project, check=True), [])
            path = next(iter(outputs))
            (project / path).unlink()
            self.assertIn(path, gen.write_outputs(outputs, project, check=True))
            gen.write_outputs(outputs, project)
            (project / path).write_text("modified")
            self.assertIn(path, gen.write_outputs(outputs, project, check=True))
            gen.write_outputs(outputs, project)
            obsolete = "src/ovphysx/generated/Old.inc"
            (project / obsolete).write_text(gen.BANNER)
            manifest = project / "ovruntime/physics_io_domain/generated-files.json"
            manifest.write_text(json.dumps(list(outputs) + [obsolete]))
            self.assertIn(obsolete, gen.write_outputs(outputs, project, check=True))
            self.assertTrue((project / obsolete).exists())
            gen.write_outputs(outputs, project)
            self.assertFalse((project / obsolete).exists())


if __name__ == "__main__":
    unittest.main()
