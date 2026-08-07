# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Contract checks for public documentation that mirrors C API semantics."""

import re
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

# Every place the contact-binding sensor requirement is stated to users.
CONTACT_BINDING_DOC_SITES = (
    "docs/tutorials/contact_binding.md",
    "docs/developer_guide.md",
    "include/ovphysx/ovphysx.h",
    "python/ovphysx/api.py",
)

# The claim this contract exists to keep out: contact bindings need
# PhysxContactReportAPI on the sensor prim, so "nothing beyond the rigid
# bodies" is false. See NVBug 6543101 / OMPE-103946.
_NO_EXTRA_AUTHORING_CLAIM = re.compile(
    r"no extra (?:USD )?(?:authoring|schema)\b",
    re.IGNORECASE,
)

# The developer guide states the requirement in a comparison table whose other
# column is about get_contact_report. A file-wide token check would pass on that
# column alone, so the contact-binding cell is asserted directly.
_USD_REQUIREMENT_ROW = re.compile(
    r"^\|\s*\*\*USD requirement\*\*\s*\|([^|]*)\|([^|]*)\|",
    re.MULTILINE,
)


def _blanket_no_authoring_claims(text):
    """Matches of the stale claim, minus the legitimate statement that *filter*
    prims need no schema. The defect is the blanket claim; scoping it to filters
    is correct and must not trip the guard."""
    for match in _NO_EXTRA_AUTHORING_CLAIM.finditer(text):
        context = text[max(0, match.start() - 80) : match.end()]
        if "filter" in context.lower():
            continue
        yield match


def _normalize_jacobian_formula(formula):
    return re.sub(
        r"\s+",
        "",
        formula.replace("numLinks", "L").replace("numDofs", "D"),
    )


def test_fixed_base_jacobian_shape_matches_public_header():
    tutorial = (PROJECT_ROOT / "docs/tutorials/tensor_bindings.md").read_text(
        encoding="utf-8"
    )
    public_header = (PROJECT_ROOT / "include/ovphysx/ovphysx_types.h").read_text(
        encoding="utf-8"
    )

    tutorial_match = re.search(
        r"^- `R`, `C`: Jacobian shape from `getJacobianShape\(\)` .*"
        r"fixed-base:\s*`([^`]+)`;",
        tutorial,
        re.MULTILINE,
    )
    header_matches = re.findall(
        r"^\s*\*\s+(?:Fixed-base|For fixed-base):\s+"
        r"R\s*=\s*([^,]+),\s+C\s*=\s*([A-Za-z][A-Za-z0-9]*)",
        public_header,
        re.MULTILINE,
    )

    assert tutorial_match is not None
    assert len(header_matches) == 2

    tutorial_formula = _normalize_jacobian_formula(tutorial_match.group(1))
    header_formulas = {
        _normalize_jacobian_formula(f"R={rows},C={cols}")
        for rows, cols in header_matches
    }

    assert tutorial_formula == "R=(L-1)*6,C=D"
    assert header_formulas == {tutorial_formula}


def _read(relative_path):
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


# The mass unit is stage metadata named kilogramsPerUnit. USD accepts an unknown
# key in the layer header and drops it without a warning, so a wrong spelling
# leaves the stage at the default mass unit and nothing reports it. See NVBug
# 6557394 / OMPE-104387.
# Most usda blocks on the page are prim fragments; only a complete stage carries
# the header stanza that stage metadata lives in.
_FULL_STAGE_USDA_BLOCK = re.compile(
    r"^```usda\n(#usda 1\.0\n.*?)^```",
    re.DOTALL | re.MULTILINE,
)


def _assert_mass_unit_is_authored(label, usda_text, tmp_path):
    # Imported here, as elsewhere in this suite, so the text-only contracts in
    # this file still collect where USD is unavailable.
    from pxr import Usd

    layer_path = tmp_path / "stage.usda"
    layer_path.write_text(usda_text, encoding="utf-8")

    stage = Usd.Stage.Open(str(layer_path))
    assert stage, f"{label} is not a loadable USD layer"

    info_keys = set(stage.GetRootLayer().pseudoRoot.ListInfoKeys())
    assert "kilogramsPerUnit" in info_keys, (
        f"{label} does not author kilogramsPerUnit; USD kept the default mass "
        f"unit. Authored stage metadata: {sorted(info_keys)}"
    )
    assert "kilogramsPerMass" not in usda_text, (
        f"{label} uses kilogramsPerMass, which is not a USD stage-metadata key "
        "and is silently discarded"
    )


def test_physics_scene_usda_template_authors_the_mass_unit(tmp_path):
    """The template readers copy must really set the mass unit, not just appear
    to. Authored through USD rather than matched as text, so a key USD rejects
    fails here the same way it fails for a reader."""
    doc = _read("docs/simulation_setup/physics_scene.md")
    blocks = _FULL_STAGE_USDA_BLOCK.findall(doc)
    assert len(blocks) == 1, (
        f"expected exactly one complete .usda stage in physics_scene.md, found {len(blocks)}"
    )

    _assert_mass_unit_is_authored(
        "the physics_scene.md .usda template", blocks[0], tmp_path
    )


def test_reference_scene_authors_the_mass_unit(tmp_path):
    """physics_scene.md sends readers to this scene right below the template, so
    it has to set the mass unit the same, valid way."""
    _assert_mass_unit_is_authored(
        "tests/data/simple_physics_scene.usda",
        _read("tests/data/simple_physics_scene.usda"),
        tmp_path,
    )


def test_contact_binding_docs_require_contact_report_api_on_sensors():
    """Every user-facing description of contact bindings must state the schema
    requirement, and none may repeat the 'rigid bodies are enough' claim."""
    for site in CONTACT_BINDING_DOC_SITES:
        text = _read(site)

        stale = next(_blanket_no_authoring_claims(text), None)
        assert stale is None, f"{site} still claims no extra USD authoring: {stale.group(0)!r}"

        assert "PhysxContactReportAPI" in text, (
            f"{site} does not mention PhysxContactReportAPI, which sensor prims require"
        )

    guide = _read("docs/developer_guide.md")
    requirement_row = _USD_REQUIREMENT_ROW.search(guide)
    assert requirement_row is not None, "developer guide lost its USD requirement row"
    assert "PhysxContactReportAPI" in requirement_row.group(1), (
        "the contact-binding column of the USD requirement row must state the "
        f"schema requirement, got: {requirement_row.group(1).strip()!r}"
    )
