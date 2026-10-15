# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-7

"""Contract checks for public documentation that mirrors shipped behavior."""

import ast
import json
import platform
import re
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _python_usd_expected():
    """Whether python USD (`pxr`) must be importable in this run.

    The mass-unit contract opens a .usda through USD in a clean subprocess (see
    _stage_info_keys). The python tests supply that USD from stock pip usd-core
    (REQ-PACKAGING-PYTESTUSD-001), whose pyproject marker is
    ``platform_machine != 'aarch64'`` -- so PyPI ships no wheel only on aarch64,
    and only there is pxr legitimately absent. Everywhere else it must import: a
    failure to (a broken venv, a slow-machine timeout, a usd-core resolution
    change) is a real failure, not a silent skip, or this enforcement quietly
    drops on the very platforms it guards.
    """
    return platform.machine() != "aarch64"


_REQUIRES_PXR = pytest.mark.skipif(
    not _python_usd_expected(),
    reason="stock usd-core has no linux-aarch64 wheel (pyproject marker "
    "platform_machine != 'aarch64')",
)

# Every place the contact-binding sensor requirement is stated to users.
CONTACT_BINDING_DOC_SITES = (
    "docs/tutorials/contact_binding.md",
    "docs/developer_guide.md",
    "include/ovphysx/ovphysx_legacy.h",
    "python/ovphysx/api.py",
)

# The claim this contract exists to keep out: contact bindings need
# PhysxContactReportAPI on the sensor prim, so "nothing beyond the rigid
# bodies" is false. See NVBug 6543101 / OMPE-103946.
_NO_EXTRA_AUTHORING_CLAIM = re.compile(
    r"no extra (?:USD )?(?:authoring|schema)\b",
    re.IGNORECASE,
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

# The developer guide states the requirement in the contact-binding compatibility
# table. A file-wide token check could pass on unrelated prose, so the
# contact-binding USD-requirement cell is asserted directly.
_USD_REQUIREMENT_ROW = re.compile(
    r"^\|\s*\*\*USD requirement\*\*\s*\|([^|]*)\|",
    re.MULTILINE,
)


def _normalize_jacobian_formula(formula):
    return re.sub(
        r"\s+",
        "",
        formula.replace("numLinks", "L").replace("numDofs", "D"),
    )


def _method_calls_on(tree, receiver):
    return {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == receiver
    }


# NVBug 6482059 / OMPE-102267: keep the getting-started surfaces aligned with
# the sample data copied into wheel and SDK packages, and with the prebuilt CUDA
# and OVStage version contracts. NVBug 6664718 / OMPE-107056: keep these minimal
# workflows on step_sync() rather than teaching async enqueue/wait choreography.
# NVBug 6660128 / OMPE-107003: tutorial literalinclude samples must resolve bundled
# scenes from the installed package, not from the reader's working directory.
# NVBug 6904316 / OMPE-115103: the same contract covers profiling, contact binding,
# kinematic support, and the ovstage output-read sample.
TUTORIAL_PYTHON_SAMPLE_SITES = (
    ("docs/tutorials/hello_world.md", "tests/python_samples/hello_world.py"),
    ("docs/tutorials/cloning.md", "tests/python_samples/clone.py"),
    ("docs/tutorials/tensor_bindings.md", "tests/python_samples/tensor_bindings.py"),
    ("docs/tutorials/omnipvd_recording.md", "tests/python_samples/omnipvd_recording.py"),
    ("docs/tutorials/profiling.md", "tests/python_samples/profiling.py"),
    ("docs/tutorials/contact_binding.md", "tests/python_samples/contact_binding.py"),
    ("docs/simulation_setup/kinematic_support.md", "tests/python_samples/kinematic_support.py"),
    ("docs/ovstage_integration.md", "tests/python_samples/output_read.py"),
)

# NVBug 6904272 / OMPE-115102: pages that embed a sample via {literalinclude}
# must keep that include, and every shipped sample must import schema discovery
# from the installed wheel namespace. A folder-local schema_support import only
# resolves beside the sample tree. session_write.py and tensor_bindings_views.py
# are listed in the developer guide rather than embedded; the sample glob below
# still rejects a folder-local import in those files.
_INSTALLED_SCHEMA_SUPPORT_MODULE = "ovphysx.samples.python_samples.schema_support"
COPY_PASTE_SCHEMA_SUPPORT_SAMPLE_SITES = (
    ("docs/tutorials/hello_world.md", "tests/python_samples/hello_world.py"),
    ("docs/tutorials/cloning.md", "tests/python_samples/clone.py"),
    ("docs/tutorials/contact_binding.md", "tests/python_samples/contact_binding.py"),
    ("docs/tutorials/tensor_bindings.md", "tests/python_samples/tensor_bindings.py"),
    ("docs/tutorials/omnipvd_recording.md", "tests/python_samples/omnipvd_recording.py"),
    (
        "docs/simulation_setup/kinematic_support.md",
        "tests/python_samples/kinematic_support.py",
    ),
    ("docs/ovstage_integration.md", "tests/python_samples/output_read.py"),
)


def _assert_sample_resolves_bundled_data(sample_path):
    text = (PROJECT_ROOT / sample_path).read_text(encoding="utf-8")
    tree = ast.parse(text)
    strings = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert "samples" in strings
    assert "data" in strings
    assert "ovphysx.__file__" in text
    assert "parent.parent" in text
    assert "FileNotFoundError" in text
    assert "ovphysx sample data is missing" in text
    assert '".." / "data"' not in text


def test_tutorial_python_samples_resolve_bundled_data():
    for tutorial_site, sample_path in TUTORIAL_PYTHON_SAMPLE_SITES:
        tutorial = (PROJECT_ROOT / tutorial_site).read_text(encoding="utf-8")
        assert sample_path.replace("/", "\\") in tutorial or sample_path in tutorial, (
            f"{tutorial_site} no longer literalincludes {sample_path}"
        )
        _assert_sample_resolves_bundled_data(sample_path)


def _local_schema_support_import(tree):
    """Return the folder-local schema_support import, if the module tree has one."""
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.split(".", 1)[0] == "schema_support":
                    return alias.name
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if module == "schema_support" or module.startswith("schema_support."):
                return module or "schema_support"
            if node.level and any(
                alias.name.split(".", 1)[0] == "schema_support" for alias in node.names
            ):
                return "schema_support"
    return None


def _imports_installed_physx_schema_paths(tree):
    return any(
        isinstance(node, ast.ImportFrom)
        and node.module == _INSTALLED_SCHEMA_SUPPORT_MODULE
        and any(alias.name == "physx_schema_paths" for alias in node.names)
        for node in ast.walk(tree)
    )


def _uses_physx_schema_paths(tree):
    return any(
        (isinstance(node, ast.Name) and node.id == "physx_schema_paths")
        or (isinstance(node, ast.Attribute) and node.attr == "physx_schema_paths")
        for node in ast.walk(tree)
    )


def test_copy_paste_samples_use_installed_schema_support():
    """Copied samples must not need a sibling schema_support/ directory."""
    samples_dir = PROJECT_ROOT / "tests/python_samples"
    for sample_path in sorted(samples_dir.glob("*.py")):
        rel = sample_path.relative_to(PROJECT_ROOT).as_posix()
        tree = ast.parse(sample_path.read_text(encoding="utf-8"))
        local_import = _local_schema_support_import(tree)
        assert local_import is None, (
            f"{rel} imports folder-local schema_support ({local_import})"
        )
        if _uses_physx_schema_paths(tree):
            assert _imports_installed_physx_schema_paths(tree), (
                f"{rel} must import physx_schema_paths from "
                f"{_INSTALLED_SCHEMA_SUPPORT_MODULE}"
            )

    for doc_site, sample_path in COPY_PASTE_SCHEMA_SUPPORT_SAMPLE_SITES:
        doc_path = PROJECT_ROOT / doc_site
        depth = len(doc_path.parent.relative_to(PROJECT_ROOT).parts)
        include_path = "/".join(("..",) * depth + (sample_path,))
        doc = doc_path.read_text(encoding="utf-8")
        assert f"{{literalinclude}} {include_path}" in doc, (
            f"{doc_site} no longer literalincludes {include_path}"
        )


@pytest.mark.parametrize("skill", (
    "basic-workflow", "clone-environments", "tensor-bindings-cpu", "tensor-bindings-gpu",
))
def test_skill_population_registers_physx_and_newton_schemas(skill):
    """NVBug 6900449: copied workflows must preserve authored newton:* values."""
    text = (PROJECT_ROOT / "skills" / skill / "SKILL.md").read_text(encoding="utf-8")
    blocks = re.findall(r"^```python\n(.*?)^```", text, re.DOTALL | re.MULTILINE)
    population_blocks = [block for block in blocks if "open_usd" in block]
    assert population_blocks, f"{skill} has no Python population example"
    physx_roots = [Path("PhysxSchema/resources"), Path("OmniUsdPhysicsDeformableSchema/resources")]
    newton_root = Path("newton_usd_schemas")
    for block in population_blocks:
        tree = ast.parse(block)
        # Require unconditional registration, not a scan of the stage's attributes.
        registrations = [
            node.value for node in tree.body
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
            and ast.unparse(node.value.func) == "ovstage.population.register_usd_schemas"
        ]
        assert len(registrations) == 1, f"{skill} must register schemas unconditionally"
        registration = registrations[0]
        roots = eval(compile(ast.Expression(registration.args[0]), skill, "eval"), {
            "__builtins__": {"str": str},
            "physx_schema_paths": lambda: physx_roots,
            "ovphysx": SimpleNamespace(newton_schema_root=lambda: newton_root),
        })
        assert roots == [str(root) for root in [*physx_roots, newton_root]]
        population_calls = [
            node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and ast.unparse(node.func) == "ovstage.population.open_usd"
        ]
        assert population_calls
        assert registration.lineno < min(call.lineno for call in population_calls)
    assert "newton-usd-schemas" in text


def test_getting_started_docs_match_shipped_packages():
    docs = {
        site: (PROJECT_ROOT / site).read_text(encoding="utf-8")
        for site in (
            "docs/tutorials/quickstart.md",
            "docs/tutorials/hello_world.md",
        )
    }
    prerequisites = {}
    for site, text in docs.items():
        match = re.search(
            r"^## Prerequisites\n(.*?)(?=^## |\Z)",
            text,
            re.DOTALL | re.MULTILINE,
        )
        assert match is not None, f"{site} has no Prerequisites section"
        prerequisites[site] = re.sub(r"\s+", " ", match.group(1))
        assert "samples/data/" in prerequisites[site]
        assert "CUDA 12.8" in prerequisites[site]

    quickstart_prerequisites = prerequisites["docs/tutorials/quickstart.md"]
    hello_world_prerequisites = prerequisites["docs/tutorials/hello_world.md"]
    assert "do not require a CUDA Toolkit installation" in quickstart_prerequisites
    assert "no CUDA Toolkit installation" in hello_world_prerequisites
    assert "cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html#id6" in quickstart_prerequisites

    for stage_name in (
        "simple_physics_scene.usda",
        "links_chain_sample.usda",
        "basic_simulation.usda",
    ):
        assert (PROJECT_ROOT / "tests/data" / stage_name).is_file()

    python_getting_started_sites = (
        "docs/tutorials/quickstart.md",
        "README.md",
        "python/README.md",
        "skills/basic-workflow/SKILL.md",
    )
    for site in python_getting_started_sites:
        text = (PROJECT_ROOT / site).read_text(encoding="utf-8")
        python_blocks = re.findall(
            r"^```python\n(.*?)^```",
            text,
            re.DOTALL | re.MULTILINE,
        )
        population_blocks = [block for block in python_blocks if "open_usd" in block]
        assert population_blocks, f"{site} has no Python population example"
        for block in population_blocks:
            tree = ast.parse(block)
            strings = {
                node.value
                for node in ast.walk(tree)
                if isinstance(node, ast.Constant) and isinstance(node.value, str)
            }
            assert "scene.usda" not in strings
            assert {"samples", "data"} <= strings
            assert {
                "simple_physics_scene.usda",
                "basic_simulation.usda",
            } & strings

        physx_call_sets = []
        for block in python_blocks:
            tree = ast.parse(block)
            physx_call_sets.append(_method_calls_on(tree, "physx"))

        assert any(
            "attach_ovstage" in calls and "step_sync" in calls
            for calls in physx_call_sets
        ), f"{site} has no Python physx.attach_ovstage()/step_sync() workflow"
        assert all("step" not in calls for calls in physx_call_sets), (
            f"{site} teaches async physx.step() in a getting-started example"
        )

    basic_workflow = (PROJECT_ROOT / "skills/basic-workflow/SKILL.md").read_text(
        encoding="utf-8"
    )
    c_blocks = re.findall(
        r"^```c\n(.*?)^```",
        basic_workflow,
        re.DOTALL | re.MULTILINE,
    )
    assert len(c_blocks) == 1
    assert '"scene.usda"' not in c_blocks[0]
    assert '"samples/data/simple_physics_scene.usda"' in c_blocks[0]

    hello_world = (PROJECT_ROOT / "tests/python_samples/hello_world.py").read_text(
        encoding="utf-8"
    )
    hello_world_tree = ast.parse(hello_world)
    assert "samples" in hello_world and "links_chain_sample.usda" in hello_world
    hello_world_calls = _method_calls_on(hello_world_tree, "physx")
    assert "step_sync" in hello_world_calls
    assert "step" not in hello_world_calls

    for packaging_script in ("scripts/build_wheel.cmake", "scripts/install.cmake"):
        packaging = (PROJECT_ROOT / packaging_script).read_text(encoding="utf-8")
        assert '"${PROJECT_ROOT}/tests/data" "${SAMPLES_DST}/data"' in packaging

    fetch_script = (PROJECT_ROOT / "scripts/fetch_ovstage_release.py").read_text(
        encoding="utf-8"
    )
    version_match = re.search(r'^OVSTAGE_VERSION = "([^"]+)"$', fetch_script, re.MULTILINE)
    assert version_match is not None
    ovstage_version = version_match.group(1)
    # pyproject.toml pins the pip/wheel version, which drops any trailing git-hash
    # component (the optional 5th). See fetch_ovstage_release._numeric_version and
    # scripts/sync_ovstage_version.py, which stamps this pin.
    numeric_ovstage_version = ".".join(ovstage_version.split(".")[:4])
    pyproject = (PROJECT_ROOT / "python/pyproject.toml").read_text(encoding="utf-8")
    assert f'"ovstage=={numeric_ovstage_version}"' in pyproject
    for site in ("docs/tutorials/quickstart.md", "SDK_README.md"):
        text = (PROJECT_ROOT / site).read_text(encoding="utf-8")
        assert not re.search(r"\bOVStage\s+`?\d+\.\d+", text, re.IGNORECASE), (
            f"{site} must describe OVStage acquisition without repeating a version pin"
        )


def test_basic_workflow_c_sample_build_and_launch():
    skill = (PROJECT_ROOT / "skills/basic-workflow/SKILL.md").read_text(encoding="utf-8")
    section = re.search(
        r"^### Build and run the C sample\n(.*?)(?=^#{1,3} |\Z)",
        skill,
        re.DOTALL | re.MULTILINE,
    )
    assert section is not None
    instructions = section.group(1)
    assert "Visual Studio Developer PowerShell" in " ".join(instructions.split())
    assert "-Arch amd64 -HostArch amd64" in instructions
    cmake_blocks = re.findall(
        r"^```cmake\n(.*?)^```", instructions, re.DOTALL | re.MULTILINE
    )
    sample_cmake = next(
        (block for block in cmake_blocks if "project(ovphysx_example LANGUAGES C)" in block),
        None,
    )
    assert sample_cmake is not None
    windows_setup = re.search(r"if\(WIN32\)(.*?)endif\(\)", sample_cmake, re.DOTALL)
    assert windows_setup is not None
    assert "ovphysx_copy_runtime_dlls(my_app)" in windows_setup.group(1)
    assert "ovphysx_set_ovstage_runtime_env(my_app)" in windows_setup.group(1)

    configure = re.search(r"^cmake -B ([\w-]+) ", instructions, re.MULTILINE)
    assert configure is not None
    build_dir = configure.group(1)
    assert f"cmake --build {build_dir} --config Release" in instructions
    sdk_readme = (PROJECT_ROOT / "SDK_README.md").read_text(encoding="utf-8")
    assert build_dir not in re.findall(r"^cmake -B (\S+)", sdk_readme, re.MULTILINE)

    bash_blocks = re.findall(
        r"^```bash\n(.*?)^```", instructions, re.DOTALL | re.MULTILINE
    )
    assert any(
        re.search(r'^export PHYSX_USD_SCHEMAS_ROOT="[^"\n]+"\n', block)
        and f"./{build_dir}/my_app" in block
        for block in bash_blocks
    )
    powershell_setup = re.search(
        r'^```powershell\n\$env:PHYSX_USD_SCHEMAS_ROOT\s*=\s*"[^"\n]+"\n',
        instructions,
        re.MULTILINE,
    )
    assert powershell_setup is not None
    for generator, subdir in (("Visual Studio", "Release\\"), ("Ninja", "")):
        command = f".\\{build_dir}\\{subdir}run_my_app.cmd"
        launch = f"- Windows, {generator} generator: `{command}`."
        assert launch in instructions[powershell_setup.end() :]


def test_fixed_base_jacobian_shape_matches_public_header():
    tutorial = (PROJECT_ROOT / "docs/tutorials/tensor_bindings.md").read_text(
        encoding="utf-8"
    )
    public_header = (PROJECT_ROOT / "include/ovphysx/ovphysx_legacy_types.h").read_text(
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


# Read in a CLEAN subprocess, not in-process.
#
# libovstage, which ovphysx links, loads the OV namespaced monolithic USD
# (ovphysx itself ships and loads no USD at all). Once any earlier test has
# imported ovphysx._bindings -- test_abi_checks.py, the first file collected,
# does -- that monolith is resident, and a later in-process `from pxr import Usd`
# must resolve against the same process. Python USD is stock pip usd-core
# (REQ-PACKAGING-PYTESTUSD-001), a *different* build than ovstage's monolith, so
# importing it in-process would double-register USD's process-wide singletons and
# abort. It is the one-USD-runtime-per-process rule in AGENTS.md, reached from the
# Python side, which is exactly why this file is on the pxr allow-list in
# test_pytest_usd_source.py despite never importing it here.
#
# A fresh interpreter sidesteps all of it: it loads whichever USD the run provides
# with nothing else resident, so the contract is really asserted no matter what ran
# before. (When no python USD is installed at all -- aarch64 has no usd-core wheel
# -- the test skips up front via _REQUIRES_PXR rather than failing on this read.)
_READ_INFO_KEYS = """
import sys
from pxr import Usd
stage = Usd.Stage.Open(sys.argv[1])
if not stage:
    sys.exit("NOT_A_LAYER")
print("\\n".join(stage.GetRootLayer().pseudoRoot.ListInfoKeys()))
"""


# Bounded, because this is a child process in a CI job. Opening a small .usda is
# sub-second, so a run that reaches this limit is wedged (a USD import that never
# returns, or a resolver blocking on a network path). Without the bound the job
# hangs until the runner's own timeout kills it with no useful message. Failing
# here names the layer instead.
_STAGE_READ_TIMEOUT_SECONDS = 120


def _stage_info_keys(layer_path):
    try:
        proc = subprocess.run(
            [sys.executable, "-c", _READ_INFO_KEYS, str(layer_path)],
            capture_output=True,
            text=True,
            timeout=_STAGE_READ_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as expired:
        # Converted rather than propagated: TimeoutExpired out of a helper reads as a broken test,
        # and its message names only the interpreter, not the layer that wedged it.
        raise AssertionError(
            f"reading stage metadata for {layer_path} did not finish within "
            f"{_STAGE_READ_TIMEOUT_SECONDS}s; the USD import or the resolver is wedged"
        ) from expired
    if proc.returncode != 0:
        raise AssertionError(
            f"could not read stage metadata via USD: {proc.stdout}{proc.stderr}"
        )
    return set(proc.stdout.split())


def _assert_mass_unit_is_authored(label, usda_text, tmp_path):
    layer_path = tmp_path / "stage.usda"
    layer_path.write_text(usda_text, encoding="utf-8")

    info_keys = _stage_info_keys(layer_path)
    assert "kilogramsPerUnit" in info_keys, (
        f"{label} does not author kilogramsPerUnit; USD kept the default mass "
        f"unit. Authored stage metadata: {sorted(info_keys)}"
    )
    assert "kilogramsPerMass" not in usda_text, (
        f"{label} uses kilogramsPerMass, which is not a USD stage-metadata key "
        "and is silently discarded"
    )


@_REQUIRES_PXR
def test_physics_scene_usda_template_authors_the_mass_unit(tmp_path):
    """The template readers copy must really set the mass unit, not just appear
    to. Authored through USD rather than matched as text, so a key USD rejects
    fails here the same way it fails for a reader."""
    doc = (PROJECT_ROOT / "docs/simulation_setup/physics_scene.md").read_text(
        encoding="utf-8"
    )
    blocks = _FULL_STAGE_USDA_BLOCK.findall(doc)
    assert len(blocks) == 1, (
        f"expected exactly one complete .usda stage in physics_scene.md, found {len(blocks)}"
    )

    _assert_mass_unit_is_authored(
        "the physics_scene.md .usda template", blocks[0], tmp_path
    )


@_REQUIRES_PXR
def test_reference_scene_authors_the_mass_unit(tmp_path):
    """physics_scene.md sends readers to this scene right below the template, so
    it has to set the mass unit the same, valid way."""
    _assert_mass_unit_is_authored(
        "tests/data/simple_physics_scene.usda",
        (PROJECT_ROOT / "tests/data/simple_physics_scene.usda").read_text(
            encoding="utf-8"
        ),
        tmp_path,
    )


def test_contact_binding_docs_require_contact_report_api_on_sensors():
    """Every user-facing description of contact bindings must state the schema
    requirement, and none may repeat the 'rigid bodies are enough' claim."""
    for site in CONTACT_BINDING_DOC_SITES:
        text = (PROJECT_ROOT / site).read_text(encoding="utf-8")

        stale = next(_blanket_no_authoring_claims(text), None)
        assert stale is None, f"{site} still claims no extra USD authoring: {stale.group(0)!r}"

        assert "PhysxContactReportAPI" in text, (
            f"{site} does not mention PhysxContactReportAPI, which sensor prims require"
        )

    guide = (PROJECT_ROOT / "docs/developer_guide.md").read_text(encoding="utf-8")
    requirement_row = _USD_REQUIREMENT_ROW.search(guide)
    assert requirement_row is not None, "developer guide lost its USD requirement row"
    assert "PhysxContactReportAPI" in requirement_row.group(1), (
        "the contact-binding column of the USD requirement row must state the "
        f"schema requirement, got: {requirement_row.group(1).strip()!r}"
    )


# NVBug 6660134 / OMPE-107004: skills must declare the canonical frontmatter
# keys SKILLS.md documents, and GPU-dynamics prose must match the schema default.
_SKILL_FRONTMATTER = re.compile(
    r"^---\n(.*?)\n---",
    re.DOTALL | re.MULTILINE,
)
_METADATA_VERSION = re.compile(r"^  version: .+", re.MULTILINE)
_METADATA_AUTHOR = re.compile(r"^  author: .+", re.MULTILINE)
_METADATA_TAGS = re.compile(r"^  tags: .+", re.MULTILINE)
_REQUIRES_GPU_DYNAMICS_AUTHORING = re.compile(
    r"(?:enabled by|require(?:s|d)?|must(?: be)?)\s+author(?:ing|ed)\s+"
    r"[`\"]?physxScene:enableGPUDynamics\s*=\s*true",
    re.IGNORECASE,
)


def test_skill_frontmatter_declares_canonical_metadata():
    skills_root = PROJECT_ROOT / "skills"
    skill_files = sorted(skills_root.glob("*/SKILL.md"))
    assert skill_files, "expected at least one skill under skills/"

    for skill_path in skill_files:
        text = skill_path.read_text(encoding="utf-8")
        frontmatter_match = _SKILL_FRONTMATTER.match(text)
        assert frontmatter_match is not None, f"{skill_path} has no YAML frontmatter"
        frontmatter = frontmatter_match.group(1)
        assert "compatibility:" in frontmatter, (
            f"{skill_path} frontmatter missing compatibility:"
        )
        assert "metadata:" in frontmatter, (
            f"{skill_path} frontmatter missing metadata:"
        )
        for label, pattern in (
            ("version", _METADATA_VERSION),
            ("author", _METADATA_AUTHOR),
            ("tags", _METADATA_TAGS),
        ):
            assert pattern.search(frontmatter), (
                f"{skill_path} frontmatter metadata missing {label}:"
            )


# NVBug 6763283 / OMPE-109807: eval rubrics must use the current lifecycle API
# and route new read/write code away from the deprecated tensor-binding skills.
def test_skill_evals_use_current_api_and_deprecation_routes():
    def eval_case(skill_name, case_id):
        dataset_path = (
            PROJECT_ROOT / "skills" / skill_name / "evals" / "evals.json"
        )
        dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
        return next(case for case in dataset["evals"] if case["id"] == case_id)

    basic_lifecycle = eval_case("basic-workflow", 1)
    lifecycle_rubric = " ".join(
        [basic_lifecycle["expected_output"], *basic_lifecycle["expectations"]]
    )
    assert "physx.destroy" in lifecycle_rubric
    assert "physx.release" not in lifecycle_rubric

    expected_routes = (
        ("basic-workflow", 4, ("clone-environments", "ovphysx-session-write")),
        ("ovphysx-output-read", 3, ("ovphysx-output-read",)),
        (
            "ovphysx-usd-authoring",
            5,
            ("basic-workflow", "ovphysx-output-read"),
        ),
    )

    def _is_negated_mention(clause):
        lower = clause.lower()
        return (
            lower.startswith("does not select")
            or "does not use" in lower
            or (
                "not " in lower
                and "deprecated tensor-binding" in lower
            )
        )

    for skill_name, case_id, required_skills in expected_routes:
        case = eval_case(skill_name, case_id)
        rubric_clauses = [case["expected_output"], *case["expectations"]]
        rubric = " ".join(rubric_clauses)
        for required_skill in required_skills:
            assert required_skill in rubric, (
                f"{skill_name} eval {case_id} does not route to "
                f"{required_skill}"
            )

        stale_route = any(
            deprecated in clause
            for clause in rubric_clauses
            if not _is_negated_mention(clause)
            for deprecated in ("tensor-bindings-cpu", "tensor-bindings-gpu")
        )
        assert not stale_route, (
            f"{skill_name} eval {case_id} positively routes new code to a "
            "deprecated skill"
        )

    output_read = eval_case("ovphysx-output-read", 3)
    output_read_rubric = " ".join(
        [output_read["expected_output"], *output_read["expectations"]]
    )
    assert "preallocated" in output_read_rubric
    assert "copies" in output_read_rubric.lower()


_GPU_SCENE_DOCUMENTATION = (
    "docs/simulation_setup/physics_scene.md",
    "docs/simulation_setup/deformables.md",
    "docs/simulation_setup/particles.md",
)


@pytest.mark.parametrize("site", (
    "docs/developer_guide.md",
    "skills/tensor-bindings-gpu/SKILL.md",
    *_GPU_SCENE_DOCUMENTATION,
))
def test_gpu_dynamics_default_documentation_consistent(site):
    """GPU dynamics default to on; docs must not imply authoring true is required."""
    text = (PROJECT_ROOT / site).read_text(encoding="utf-8")
    stale = _REQUIRES_GPU_DYNAMICS_AUTHORING.search(text)
    assert stale is None, (
        f"{site} still implies GPU dynamics require authoring enableGPUDynamics=true: "
        f"{stale.group(0)!r}"
    )
    assert "defaults to `true`" in text, (
        f"{site} does not state that physxScene:enableGPUDynamics defaults to true"
    )


# NVBug 6904483 / OMPE-115114: scene setup pages must explain both GPU defaults
# and show that per-scene CPU simulation also needs a CPU broadphase.
@pytest.mark.parametrize("site", _GPU_SCENE_DOCUMENTATION)
def test_gpu_broadphase_default_documentation_consistent(site):
    text = (PROJECT_ROOT / site).read_text(encoding="utf-8")
    assert 'defaults to `"GPU"`' in text, (
        f"{site} does not state that physxScene:broadphaseType defaults to GPU"
    )


def test_physics_scene_cpu_example_sets_both_dynamics_and_broadphase():
    text = (PROJECT_ROOT / "docs/simulation_setup/physics_scene.md").read_text(
        encoding="utf-8"
    )
    cpu_examples = [
        example for example in re.findall(r"```usda\n(.*?)```", text, re.DOTALL)
        if "bool physxScene:enableGPUDynamics = false" in example
    ]
    assert cpu_examples, "expected a per-scene CPU simulation example"
    for example in cpu_examples:
        assert 'def PhysicsScene "' in example
        assert 'prepend apiSchemas = ["PhysxSceneAPI"]' in example
        assert re.search(
            r'uniform token physxScene:broadphaseType = "(?:MBP|SAP)"', example
        ), "disabling GPU dynamics alone leaves the GPU broadphase enabled"


def test_physics_scene_links_to_developer_guide_determinism_section():
    text = (PROJECT_ROOT / "docs/simulation_setup/physics_scene.md").read_text(
        encoding="utf-8"
    )
    assert "(../developer_guide.md#warmup-and-determinism)" in text


# ---------------------------------------------------------------------------
# Failure paths of the subprocess helper itself.
# ---------------------------------------------------------------------------
# The helper is the only part of this file that leaves the interpreter, so it is the only part with
# failure modes of its own. Both are asserted because both are silent otherwise: a child that fails
# would surface as a confusing "kilogramsPerUnit missing", and a child that hangs would surface as a
# CI job killed by the runner with no indication of which layer did it.
def test_stage_info_keys_reports_a_child_failure_rather_than_a_missing_key(tmp_path):
    """A file USD cannot open must fail as a read error, not as an absent mass unit."""
    not_a_layer = tmp_path / "not_a_layer.usda"
    not_a_layer.write_text("this is not a usda layer", encoding="utf-8")

    with pytest.raises(AssertionError) as excinfo:
        _stage_info_keys(not_a_layer)
    # The message has to point at the read, not at the contract being checked. Otherwise a broken
    # environment reads as a documentation bug.
    assert "could not read stage metadata" in str(excinfo.value)


def test_stage_info_keys_bounds_a_wedged_child(tmp_path, monkeypatch):
    """A wedged child fails the test with the layer named, rather than hanging the job.

    The timeout is asserted through the handler rather than by actually waiting: a test that slept
    past a real bound would cost more than the bug it guards. What matters is that TimeoutExpired
    becomes an AssertionError naming the layer, and that a bound is passed at all.
    """
    layer_path = tmp_path / "stage.usda"
    layer_path.write_text("#usda 1.0\n", encoding="utf-8")

    seen = {}

    def fake_run(*args, **kwargs):
        seen["timeout"] = kwargs.get("timeout")
        raise subprocess.TimeoutExpired(cmd="python", timeout=kwargs.get("timeout"))

    monkeypatch.setattr(subprocess, "run", fake_run)

    with pytest.raises(AssertionError) as excinfo:
        _stage_info_keys(layer_path)

    # A bound was actually requested. Without this the case would pass against a helper that
    # dropped the timeout argument, since the stub raises regardless.
    assert seen["timeout"] == _STAGE_READ_TIMEOUT_SECONDS
    assert str(layer_path) in str(excinfo.value)
    assert "wedged" in str(excinfo.value)
