<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx coding and doc style

Load this when writing C++, Python, CMake, or `docs/` Markdown under `ovphysx/`.
`ovruntime/` uses the same C++ rules.

## Source files

- C API uses the `ovphysx_` prefix. Preferred declarations are in `ovphysx.h`;
  deprecated families are in `ovphysx_legacy.h` (`ovphysx.h` still includes it).
- Python API is the `PhysX` class in `ovphysx.api`.
- Source files (`.cpp`, `.h`, `.py`, `.cmake`, `.sh`, `.bat`, etc.) must be
  ASCII-only in code and comments. Use `--` or `-`, not em dashes or other
  Unicode punctuation. Markdown under `docs/` is exempt.

## Avoid `auto` for type deduction

New code under `ovphysx/` (including `ovruntime/`) should use **explicit types**
instead of `auto` in variable declarations. A reviewer looking at a line should
be able to tell what type is being constructed without jumping to the function
declaration.

**Allowed exceptions** -- use `auto` when:

- The right-hand side is a **lambda** (you cannot spell the closure type).
- **STL iterators** in for-loops or as the result of `find` / `lower_bound`.
- Template-only constructs with no spelled type: `decltype` chains, parameter
  packs, fold expressions.
- **Structured bindings** (`auto& [k, v] : map`).

**Disallowed** -- write the type out, including range-based for loops when the
element type is a single typed item. Spell `DescPtr<T>`; do not hide pointer vs
value behind `auto`.

## Doc authoring (`docs/`)

Docs under `docs/` are Markdown for Sphinx and for plain viewers (GitHub, SDK,
wheel, LLMs):

- Use **standard Markdown** for links, headings, blockquotes, and code blocks.
  Do not use MyST-only directives like `doc`, `tab-set`, `note`, or
  colon-fence syntax.
- **`{literalinclude}` is the one allowed MyST directive** for including source
  from test files. `scripts/preprocess_markdown.py` resolves it at SDK/wheel
  packaging time. Supported options: `:language:`, `:lines:`, `:start-after:`,
  `:end-before:`. Prefer `:start-after:` / `:end-before:` with
  `[tutorial-start]` / `[tutorial-end]` markers over `:lines:` ranges.
- Doc-to-doc cross-references use standard Markdown links:
  `[Developer guide](developer_guide.md)`.
- New docs must be added to the hidden `{toctree}` at the bottom of
  `docs/index.md`.
- `docs/api.md` is the only file that should contain `{eval-rst}` (for Doxygen
  rendering). The preprocessor strips these blocks from shipped copies.
