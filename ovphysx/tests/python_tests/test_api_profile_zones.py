# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-PROFILER-001
# @covers AC-5

"""Keep the public C API and its profiler-zone inventory in lockstep."""

import re
from collections import Counter, defaultdict
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _code_only(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r'"(?:\\.|[^"\\])*"', '""', text)


def _public_api_names(header: Path) -> set[str]:
    text = _code_only(header.read_text(encoding="utf-8"))
    declarations = re.findall(
        r"\bOVPHYSX_API\b(?:(?!;).)*?\b(ovphysx_[A-Za-z0-9_]+)\s*\(",
        text,
        flags=re.DOTALL,
    )
    return set(declarations)


def _profile_zone_names(source_dir: Path) -> Counter[str]:
    names: list[str] = []
    for source in source_dir.glob("*.cpp"):
        names.extend(
            re.findall(
                r"\bOVPHYSX_API_PROFILE_ZONE\s*\(\s*([A-Za-z0-9_]+)\s*\)",
                _code_only(source.read_text(encoding="utf-8")),
            )
        )
    return Counter(names)


def _matching_delimiter(text: str, start: int, opening: str, closing: str) -> int | None:
    depth = 0
    for offset in range(start, len(text)):
        if text[offset] == opening:
            depth += 1
        elif text[offset] == closing:
            depth -= 1
            if depth == 0:
                return offset
    return None


def _profiled_definitions_in_text(text: str, public_names: set[str]) -> dict[str, list[str | None]]:
    """Map each public definition to its first-statement profile zone, if any."""
    code = _code_only(text)
    definitions: dict[str, list[str | None]] = defaultdict(list)
    for match in re.finditer(r"\b(ovphysx_[A-Za-z0-9_]+)\s*\(", code):
        api_name = match.group(1)
        if api_name not in public_names:
            continue

        close_paren = _matching_delimiter(code, match.end() - 1, "(", ")")
        if close_paren is None:
            continue
        body_start = close_paren + 1
        while body_start < len(code) and code[body_start].isspace():
            body_start += 1
        if body_start >= len(code) or code[body_start] != "{":
            continue

        zone = re.match(
            r"\s*OVPHYSX_API_PROFILE_ZONE\s*\(\s*([A-Za-z0-9_]+)\s*\)\s*;",
            code[body_start + 1 :],
        )
        definitions[api_name].append(zone.group(1) if zone else None)
    return dict(definitions)


def _profiled_definitions(source_dir: Path, public_names: set[str]) -> dict[str, list[str | None]]:
    definitions: dict[str, list[str | None]] = defaultdict(list)
    for source in source_dir.glob("*.cpp"):
        for api_name, zones in _profiled_definitions_in_text(source.read_text(encoding="utf-8"), public_names).items():
            definitions[api_name].extend(zones)
    return dict(definitions)


def test_every_public_c_api_has_exactly_one_profile_zone():
    """A new or renamed public API must not ship without one canonical API zone."""
    root = _project_root()
    public_names = set()
    for header_name in ("ovphysx.h", "ovphysx_geometry_query.h", "ovphysx_legacy.h"):
        public_names.update(_public_api_names(root / "include" / "ovphysx" / header_name))
    expected_zone_names = {name.removeprefix("ovphysx_") for name in public_names}
    source_dir = root / "src" / "ovphysx"
    zone_counts = _profile_zone_names(source_dir)
    definitions = _profiled_definitions(source_dir, public_names)

    missing = sorted(expected_zone_names - zone_counts.keys())
    extra = sorted(zone_counts.keys() - expected_zone_names)
    duplicate = sorted(name for name, count in zone_counts.items() if count != 1)
    missing_definitions = sorted(public_names - definitions.keys())
    duplicate_definitions = sorted(name for name, zones in definitions.items() if len(zones) != 1)
    misplaced = {name: zones for name, zones in sorted(definitions.items()) if zones != [name.removeprefix("ovphysx_")]}

    assert public_names, "failed to discover any OVPHYSX_API declarations"
    assert not missing, f"public C APIs missing OVPHYSX_API_PROFILE_ZONE: {missing}"
    assert not extra, f"profile zones without a matching public C API: {extra}"
    assert not duplicate, f"public C APIs must have exactly one profile zone: {duplicate}"
    assert not missing_definitions, f"public C APIs missing definitions: {missing_definitions}"
    assert not duplicate_definitions, f"public C APIs must have one definition: {duplicate_definitions}"
    assert not misplaced, f"profile zone must match its API and be the first statement: {misplaced}"


def test_public_api_parser_includes_deprecated_declarations():
    header = """
        OVPHYSX_API int ovphysx_current(void);
        OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
            "deprecated; use ovphysx_current")
        int ovphysx_legacy(void);
    """
    declarations = re.findall(
        r"\bOVPHYSX_API\b(?:(?!;).)*?\b(ovphysx_[A-Za-z0-9_]+)\s*\(", _code_only(header), flags=re.DOTALL
    )
    assert declarations == ["ovphysx_current", "ovphysx_legacy"]


def test_definition_parser_associates_only_the_first_statement_zone():
    source = """
        int ovphysx_current(void)
        {
            OVPHYSX_API_PROFILE_ZONE(current);
            return 0;
        }
        int ovphysx_legacy(void)
        {
            if (something_failed())
                return -1;
            OVPHYSX_API_PROFILE_ZONE(legacy);
            return 0;
        }
    """
    assert _profiled_definitions_in_text(source, {"ovphysx_current", "ovphysx_legacy"}) == {
        "ovphysx_current": ["current"],
        "ovphysx_legacy": [None],
    }


def test_final_destroy_closes_zone_before_profiler_teardown():
    source = (_project_root() / "src" / "ovphysx" / "ovphysx.cpp").read_text(encoding="utf-8")
    code = _code_only(source)

    internal_signature = re.search(r"\bomni_sdk_physx_destroy\s*\(", code)
    assert internal_signature is not None
    internal_close_paren = _matching_delimiter(code, internal_signature.end() - 1, "(", ")")
    assert internal_close_paren is not None
    internal_body_start = code.find("{", internal_close_paren)
    assert internal_body_start >= 0
    internal_body_end = _matching_delimiter(code, internal_body_start, "{", "}")
    assert internal_body_end is not None
    internal_body = code[internal_body_start + 1 : internal_body_end]
    assert "deferredCarbonite = std::move(instanceShared->carbonite)" in internal_body
    assert "instanceShared->carbonite.reset()" not in internal_body

    signature = re.search(r"\bovphysx_destroy_instance\s*\(", code)
    assert signature is not None
    close_paren = _matching_delimiter(code, signature.end() - 1, "(", ")")
    assert close_paren is not None
    body_start = code.find("{", close_paren)
    assert body_start >= 0
    body_end = _matching_delimiter(code, body_start, "{", "}")
    assert body_end is not None
    body = code[body_start + 1 : body_end]

    ordered_steps = [
        "OVPHYSX_API_PROFILE_ZONE(destroy_instance)",
        "omni_sdk_physx_destroy(handle, deferredCarbonite)",
        "OVPHYSX_API_PROFILE_ZONE_END()",
        "deferredCarbonite.reset()",
    ]
    positions = [body.find(step) for step in ordered_steps]
    assert all(position >= 0 for position in positions), f"missing destroy teardown step: {positions}"
    assert positions == sorted(positions), f"destroy teardown order can truncate its profile zone: {positions}"
