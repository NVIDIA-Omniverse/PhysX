#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Generate a codeless schema's DATA (generatedSchema.usda + plugInfo.json) using
NATIVE OpenUSD `usdGenSchema` -- no repo_usd.

repo_usd's `repo.sh usd` was only a thin wrapper around OpenUSD's own
usdGenSchema (it then substituted a few plugInfo path placeholders). This tool
calls usdGenSchema directly (the "minimal native OpenUSD tooling") and performs
the same placeholder substitution, so the schema build no longer depends on the
deprecated repo_usd for generation.

Run with the target-deps USD python (pxr + jinja2 on PYTHONPATH); see gen_codeless.sh.
"""
import argparse
import os
import re
import subprocess
import sys

import pxr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schema-dir", required=True,
                    help="directory containing schema.usda; generatedSchema.usda + plugInfo.json are written here")
    args = ap.parse_args()

    pxr_dir = os.path.dirname(pxr.__file__)
    usdgenschema = os.path.join(pxr_dir, "Usd", "usdGenSchema.py")          # OpenUSD's own generator
    templates = os.path.normpath(os.path.join(pxr_dir, "..", "..", "usd", "usd", "resources", "codegenTemplates"))
    if not os.path.isfile(usdgenschema):
        sys.exit(f"gen_schema_data: usdGenSchema.py not found at {usdgenschema}")

    # usdGenSchema writes generatedSchema.usda + plugInfo.json into the cwd. For a codeless
    # schema (skipCodeGeneration=true) it emits only those data files -- no C++/bindings.
    subprocess.run([sys.executable, usdgenschema, "schema.usda", ".", "-t", templates, "-q"],
                   cwd=args.schema_dir, check=True)

    # usdGenSchema on Windows writes CRLF. Normalize generatedSchema.usda to LF so the
    # working tree matches the git-stored LF content and `git status` stays clean.
    generated_schema = os.path.join(args.schema_dir, "generatedSchema.usda")
    with open(generated_schema, "rb") as f:
        raw = f.read()
    with open(generated_schema, "wb") as f:
        f.write(raw.replace(b"\r\n", b"\n"))

    # usdGenSchema leaves plugInfo path placeholders for the build system to fill in
    # (repo_usd substituted these post-gen). Root/ResourcePath are the standard plugin
    # layout; LibraryPath is empty because a codeless schema ships no compiled library --
    # USD registers its types data-driven from generatedSchema.usda and never dlopens one.
    plug_info = os.path.join(args.schema_dir, "plugInfo.json")
    with open(plug_info) as f:
        text = f.read()
    text = (text.replace("@PLUG_INFO_ROOT@", "..")
                .replace("@PLUG_INFO_RESOURCE_PATH@", "resources"))
    # A codeless schema ships no library -- the plugin is Type "resource", so USD never
    # dlopens one (types are registered data-driven from generatedSchema.usda). usdGenSchema
    # always emits a LibraryPath line anyway (placeholder / stale value); drop it entirely so
    # the plugInfo is honest. Verified: registration + Apply still work with no LibraryPath.
    text = re.sub(r'^[ \t]*"LibraryPath"[ \t]*:[ \t]*"[^"]*",[ \t]*\r?\n', '', text, flags=re.M)
    # Write with explicit LF so Windows text-mode doesn't convert \n back to \r\n.
    with open(plug_info, "w", newline="\n") as f:
        f.write(text)
    print(f"gen_schema_data: {args.schema_dir} -> generatedSchema.usda + plugInfo.json (native usdGenSchema, no repo_usd)")


if __name__ == "__main__":
    main()
