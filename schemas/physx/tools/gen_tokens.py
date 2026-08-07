#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Generate PhysxSchema tokens DIRECTLY from the codeless schema.usda -- no codefull
usdGenSchema run, no generated .cpp/class files.

It calls usdGenSchema's own GatherTokens (its exact token logic) on the parsed schema,
then emits:
  1. a HEADER-ONLY physxSchema/tokens.h  (inline PhysxSchemaTokens, nothing to link), and
  2. a pure-Python `Tokens` object (PhysxSchema.Tokens.<name>, no compiled binding).

PhysxSchemaTokens (C++) and PhysxSchema.Tokens (Python) stay byte-identical to the old
codefull symbols, with zero compiled artifacts.

Requires: pxr on PYTHONPATH (its bundled usdGenSchema.py) + jinja2 (usdGenSchema imports it).
"""
import argparse
import os
import sys


def gather(schema_usda):
    import pxr
    sys.path.insert(0, os.path.join(os.path.dirname(pxr.__file__), "Usd"))  # find bundled usdGenSchema.py
    import usdGenSchema as g
    g.InitializeResolver()  # resolve @usdGeom/schema.usda@ etc. (search paths -> schema resource dirs)
    parsed = g.ParseUsd(schema_usda)   # parses only; writes no files
    lib_name, lib_tokens, class_infos = parsed[0], parsed[5], parsed[8]
    return g.GatherTokens(class_infos.values(), lib_name, lib_tokens, includeSchemaIdentifierTokens=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schema", required=True, help="path to the codeless schema.usda")
    ap.add_argument("--out-h", required=True, help="header-only tokens.h output")
    ap.add_argument("--out-py", required=True, help="python Tokens module output")
    ap.add_argument("--prefix", default="PhysxSchema",
                    help="C++/Python library prefix; drives the token struct/var/guard/module names "
                         "(e.g. PhysxSchema -> PhysxSchemaTokens). Default: PhysxSchema.")
    ap.add_argument("--lib-dir", default="physxSchema",
                    help="include directory name for the \\file comment (e.g. physxSchema/tokens.h). "
                         "Default: physxSchema.")
    args = ap.parse_args()

    prefix = args.prefix                  # e.g. "PhysxSchema" / "OmniUsdPhysicsDeformableSchema"
    lib_dir = args.lib_dir                # e.g. "physxSchema" / "omniUsdPhysicsDeformableSchema"
    tokens_type = f"{prefix}TokensType"   # struct name
    tokens_var = f"{prefix}Tokens"        # inline TfStaticData global
    guard = f"{prefix.upper()}_TOKENS_H"  # include guard (PhysxSchema -> PHYSXSCHEMA_TOKENS_H)

    tokens = gather(args.schema)          # list of entries with .id (C++ member) and .value (string)

    # --- C++ header-only ----------------------------------------------------
    h = []
    w = h.append
    w("// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.")
    w("// SPDX-License-Identifier: Apache-2.0")
    w("//")
    w("// GENERATED (header-only) by tools/gen_tokens.py from schema.usda via usdGenSchema's")
    w(f"// GatherTokens. DO NOT EDIT. Codeless tokens: {tokens_var} is defined inline (no lib).")
    w(f"/// \\file {lib_dir}/tokens.h")
    w(f"#ifndef {guard}")
    w(f"#define {guard}")
    w("")
    # OpenUSD (pxr/*) headers are project-external -> angle-bracket includes.
    w("#include <pxr/pxr.h>")
    w("#include <pxr/base/tf/staticData.h>")
    w("#include <pxr/base/tf/token.h>")
    w("#include <vector>")
    w("")
    w("PXR_NAMESPACE_OPEN_SCOPE")
    w("")
    # \class block (matches usdGenSchema's tokens.h); example uses the first token.
    example = tokens[0].id if tokens else "acceleration"
    w(f"/// \\class {tokens_type}")
    w("///")
    w(f"/// \\link {tokens_var} \\endlink provides static, efficient")
    w("/// \\link TfToken TfTokens\\endlink for use in all public USD API.")
    w("///")
    w("/// These tokens are auto-generated from the module's schema, representing")
    w("/// property names, for when you need to fetch an attribute or relationship")
    w("/// directly by name, e.g. UsdPrim::GetAttribute(), in the most efficient")
    w("/// manner, and allow the compiler to verify that you spelled the name")
    w("/// correctly.")
    w("///")
    w(f"/// {tokens_var} also contains all of the \\em allowedTokens values")
    w("/// declared for schema builtin attributes of 'token' scene description type.")
    w(f"/// Use {tokens_var} like so:")
    w("///")
    w("/// \\code")
    w(f"///     gprim.GetMyTokenValuedAttr().Set({tokens_var}->{example});")
    w("/// \\endcode")
    w(f"struct {tokens_type} {{")
    w(f"    {tokens_type}();")
    for t in tokens:
        # Per-token Doxygen comment, mirroring usdGenSchema: \brief "value" + the usage
        # description (Fallback/Possible value for ..., declaring class, or schema identifier).
        w(f'    /// \\brief "{t.value}"')
        w("    ///")
        # Emit desc verbatim (only trailing ws stripped) -- usdGenSchema preserves the desc's
        # own leading spaces (some tokens render as '///  <text>' with two spaces).
        for line in (getattr(t, "desc", "") or "").splitlines():
            w(f"    /// {line}".rstrip())
        w(f"    const TfToken {t.id};")
    w("    /// A vector of all of the tokens listed above.")
    w("    const std::vector<TfToken> allTokens;")
    w("};")
    w("")
    w(f"inline {tokens_type}::{tokens_type}() :")
    for t in tokens:
        w(f'    {t.id}("{t.value}", TfToken::Immortal),')
    w("    allTokens({")
    for t in tokens:
        w(f"        {t.id},")
    w("    })")
    w("{")
    w("}")
    w("")
    w(f"/// \\var {tokens_var}")
    w("///")
    w("/// A global variable with static, efficient \\link TfToken TfTokens\\endlink")
    w(f"/// for use in all public USD API.  \\sa {tokens_type}")
    w("///")
    w("/// Codeless: defined inline (header-only) -- no compiled tokens.cpp / library.")
    w(f"inline TfStaticData<{tokens_type}> {tokens_var};")
    w("")
    w("PXR_NAMESPACE_CLOSE_SCOPE")
    w("")
    w("#endif")
    with open(args.out_h, "w") as fh:
        fh.write("\n".join(h) + "\n")

    # --- Python Tokens ------------------------------------------------------
    p = []
    a = p.append
    a(f"# GENERATED by tools/gen_tokens.py -- pure-Python {prefix}.Tokens (codeless, no binding).")
    a(f"class _{prefix}Tokens(object):")
    a(f'    """Token table mirroring C++ {tokens_var} (values are the USD token strings)."""')
    a("    def __init__(self):")
    for t in tokens:
        a(f"        self.{t.id} = {t.value!r}")
    a(f"        self.allTokens = [{', '.join(repr(t.value) for t in tokens)}]")
    a("")
    a(f"Tokens = _{prefix}Tokens()")
    with open(args.out_py, "w") as fp:
        fp.write("\n".join(p) + "\n")
    print(f"gen_tokens: {len(tokens)} tokens from schema.usda (no codefull) -> {args.out_h} + {args.out_py}")


if __name__ == "__main__":
    main()
