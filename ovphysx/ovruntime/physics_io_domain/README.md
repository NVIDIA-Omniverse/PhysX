<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Runtime physics I/O contract

This directory owns the static metadata for physics read/write sessions. The
population domain describes initialization; this contract describes live API
values. A population mapping is an explicit cross-reference, never a rule for
inferring runtime units, support, or names from USD.

## Files and ownership

- `attributes/*.json`: canonical names and target/representation-specific type,
  units, frame, layouts, residency, capabilities, conditions, and write effects.
  Files group names by their first target; names shared across targets stay in
  one record with multiple `uses`.
- `contract.schema.json`: format version 1, usable by JSON-aware editors.
- `units.json`, `policies.json`, `targets.json`: shared definitions. Runtime units
  use `distance`, `mass`, `sec`, `deg` and `rad`. A unit has a readable `display`,
  optional `description`, conditional `variants` mapping to unit strings, and/or
  ordered `components` referencing other unit IDs. No population scale-exponent
  maps belong here. `dimensionless` is spelled out; `not_applicable` is for
  identities/counts, not an unknown physical unit. See [the SDK unit audit](UNITS.md)
  for the runtime conversion boundaries and mixed-coordinate cases.
- `bindings/*.json`: ordered native table fields binding metadata to existing
  C++ handlers; `Dispatch.json` explicitly accounts for specialized dispatch.
  Every supported direction must have a table or a named handler. Handlers,
  conversions, validation, synchronization and CUDA algorithms stay in C++.
- `compatibility.json`: reviewed baseline of names, symbols, target enum values
  and public writability classes, plus each use's type, units, frame, semantic, directional request names, layout and residency. Every entry must appear in the baseline. Update it explicitly with `--update-baseline` and review the resulting API diff; `--write` never updates it.
- `generated-files.json`: ownership manifest used to detect and remove obsolete
  generated files. Do not edit generated files directly.

## Add or change an attribute

From `ovphysx/`, using Python 3.10 or later:

1. Implement or reuse the C++ read/write handler and its live behavior tests.
2. Add the attribute to `attributes/*.json`: name, type, units, frame, semantics,
   target and supported layouts. See the defaults below.
3. Bind it in `bindings/*.json`, or name its specialized handler in `Dispatch.json`.
4. Run `python ovruntime/physics_io_domain/tools/gen_contract.py --update-baseline`.
   Review the compatibility diff as an API change. This validates the contract and
   bindings but deliberately replaces the old baseline; it is never part of CI.
5. Run `python ovruntime/physics_io_domain/tools/gen_contract.py --write`, then
   `python -S ovruntime/physics_io_domain/tools/check_codegen.py`.
6. Update the affected PLC requirement/test and run the relevant live read/write
   and writability parity suites. Regeneration alone does not prove runtime behavior.

The generator and its tests use only the Python standard library. Normal native,
SDK, wheel and docs builds consume committed artifacts. `--check` writes nothing
and detects missing, modified and obsolete **manifest-listed** outputs; it does
not scan for unlisted files. `--write` validates all paths and stale ownership
before changing any output, and refuses traversal, symlinks and unowned deletion.

Generated copyright notices are stable and excluded from formatter rewriting.
The closed schema validator rejects unsupported keywords, including nested rules;
extend validation and its tests together when adding a schema feature.

## Authoring defaults

Validation rejects unknown fields and wrong types before expanding defaults.
Supported access is an object; unsupported access is simply `"read": false` or
`"write": false`, with no invented layout. Types, units, frames, target, semantics,
layout kind/shape/lanes and write effects remain explicit.

| Omitted field | Default |
|---|---|
| `c_symbol` | `OVPHYSX_ATTR_` + name converted to upper snake case |
| `cpp_symbol` | `k` + name with its first letter capitalized |
| `representation` | `default` |
| `aliases` (use or direction) | empty list |
| `supported` (access object) | `true` |
| `request_name` | canonical attribute name |
| `conditions` | `["session"]` |
| `residency` | `scene_device` |
| layout `order` | `row_major` |

Explicit values override defaults. Keep exceptional symbols (for example
`OVPHYSX_ATTR_DEBUG_ENABLED`) and host/CUDA residency explicit. A direction can
specify `units` when its encoding differs from the use's default units, as with
`cullingBox` writes. The compatibility baseline stores resolved metadata, so
omitting a default cannot conceal a public API change.

Directional `value_constraints` describe inclusive bounds, allowed values, finite
values and additional sentinel values; `applicability` describes supported drive
or object subtypes. These appear in discovery, headers and generated references.

## Layout and capability rules

`read` and `write` describe separate request names, accepted aliases and layouts.
The canonical attribute name can be an emitted group spelling: instancer
`positions` is requested as `position` in both directions. The particle-set
`positions` alias is accepted only by reads. Use `request_name` when calling
the API, not the descriptor name or a catalog lookup alias. `lanes` is DLPack's tuple width,
not a trailing tensor dimension. `shape` describes the tensor extents. An array
use emits a tensor per object; fixed uses stack objects. Padded columns state a
count companion and padding value. Cohort widths and symbolic extents are
explanatory names, never Python expressions to evaluate. The actual session
group and its prim membership are authoritative. Symbolic lane counts mean a width determined at runtime.

`scene_device` means CPU for CPU/readback scenes or CUDA for DirectGPU; `host`
means CPU regardless of scene. CUDA-only state is marked `cuda`. Conditions
refer to named policies and do not replace backend admission checks. Unsupported
directions expose `supported: false` and no layout; inspect support before reading
directional request, layout or residency fields.

`writability` preserves the existing public C classifier, including
`UNCLASSIFIED` for legacy instancer-only spellings. Directional `supported`
fields describe runtime capabilities. Static metadata cannot promise that a
specific live object, drive type, material, scene mode or session accepts a
write. See the [attribute reference](../../docs/read_write/attributes/index.md)
and [session guides](../../docs/read_write/index.md).

## Generated consumers

The generator emits public `OVPHYSX_ATTR_*` macros and string-backed
`OVPHYSX_NAME_*` objects, runtime constants, declarative backend
tables, the static C writability table, reference pages, and Python metadata.
`ovphysx.attributes` exposes immutable descriptors without loading PhysX, Warp,
NumPy, OpenUSD or ovstage. See [discovery examples](../../docs/read_write/discovery.md).

The hand-written token helper and dictionary lifetime rules are documented in
[the C API reference](../../docs/api.md#attribute-token-interning).

The regular Python test suite collects `test_io_contract.py`, which runs the
generator regression and drift checks through the existing test job. The
standalone commands above remain useful for a fast edit loop.
