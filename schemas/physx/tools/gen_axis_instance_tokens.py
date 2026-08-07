#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Generate a header-only PhysxAxisInstanceTokens table: the concrete per-axis instances of the
physxJointAxis / physxDrivePerformanceEnvelope multiple-apply schemas (angular/linear/rotX/rotY/
rotZ). They are DERIVED at static-init from the codeless PhysxSchemaTokens
*_MultipleApplyTemplate_* tokens via UsdSchemaRegistry::MakeMultipleApplyNameInstance -- no
hand-authored strings, so no drift: when a jointAxis/driveEnvelope attribute is added to
physxSchema, the matching template token appears in tokens.h and this generator emits the
per-axis members automatically.

Member names follow the <attrLowerCamel><AxisProper> layout (e.g. maxJointVelocityAngular).

Input : physxSchema/tokens.h (authoritative *_MultipleApplyTemplate_* member set)
Output: physxSchema/axisInstanceTokens.h (header-only, inline PhysxAxisInstanceTokens)
"""
import argparse
import re

# Multiple-apply schemas whose instances are addressed per joint DOF, and the DOF instance names.
# These DOF names are application convention (not authored in the schema), so they live here.
AXES = ["angular", "linear", "rotX", "rotY", "rotZ"]
TEMPLATE_PREFIXES = ["physxJointAxis", "physxDrivePerformanceEnvelope"]


def proper(s):
    return s[:1].upper() + s[1:]


def lower_first(s):
    return s[:1].lower() + s[1:]


def collect_templates(tokens_h):
    """Return [(template_member, attr_stem)] for each *_MultipleApplyTemplate_* token of the
    configured schemas, in file order (stable output)."""
    out = []
    rx = re.compile(r'const\s+TfToken\s+([A-Za-z_]\w*)\s*;')
    with open(tokens_h) as f:
        for line in f:
            mo = rx.search(line)
            if not mo:
                continue
            member = mo.group(1)
            for pfx in TEMPLATE_PREFIXES:
                marker = pfx + "_MultipleApplyTemplate_"
                if member.startswith(marker):
                    attr = member[len(marker):]            # e.g. "MaxJointVelocity"
                    out.append((member, lower_first(attr)))  # ("physxJointAxis_..._MaxJointVelocity","maxJointVelocity")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tokens-h", required=True, help="generated physxSchema/tokens.h")
    ap.add_argument("--out-h", required=True, help="output axisInstanceTokens.h")
    args = ap.parse_args()

    templates = collect_templates(args.tokens_h)
    # (member_name, template_member, axis_instance_name)
    entries = [(stem + proper(axis), tmpl, axis)
               for (tmpl, stem) in templates for axis in AXES]

    L = []
    w = L.append
    w("// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.")
    w("// SPDX-License-Identifier: Apache-2.0")
    w("//")
    w("// GENERATED (header-only) by tools/gen_axis_instance_tokens.py. DO NOT EDIT.")
    w("// Concrete per-axis instances of the physxJointAxis / physxDrivePerformanceEnvelope")
    w("// multiple-apply schemas, derived from PhysxSchemaTokens templates (no hand-authored")
    w("// strings).")
    w("/// \\file physxSchema/axisInstanceTokens.h")
    w("#ifndef PHYSXSCHEMA_AXIS_INSTANCE_TOKENS_H")
    w("#define PHYSXSCHEMA_AXIS_INSTANCE_TOKENS_H")
    w("")
    w('#include "physxSchema/tokens.h"')
    w("#include <pxr/pxr.h>")
    w("#include <pxr/base/tf/staticData.h>")
    w("#include <pxr/base/tf/token.h>")
    w("#include <pxr/usd/usd/schemaRegistry.h>")
    w("#include <vector>")
    w("")
    w("PXR_NAMESPACE_OPEN_SCOPE")
    w("")
    w("/// \\class PhysxAxisInstanceTokensType")
    w("///")
    w("/// Per-axis (angular/linear/rotX/rotY/rotZ) instance tokens for the physxJointAxis and")
    w("/// physxDrivePerformanceEnvelope multiple-apply schemas, e.g.")
    w('/// PhysxAxisInstanceTokens->maxJointVelocityAngular == "physxJointAxis:angular:maxJointVelocity".')
    w("struct PhysxAxisInstanceTokensType {")
    w("    PhysxAxisInstanceTokensType();")
    for member, _, _ in entries:
        w(f"    const TfToken {member};")
    w("    /// A vector of all of the tokens listed above.")
    w("    const std::vector<TfToken> allTokens;")
    w("};")
    w("")
    w("inline PhysxAxisInstanceTokensType::PhysxAxisInstanceTokensType() :")
    for member, tmpl, axis in entries:
        w(f'    {member}(UsdSchemaRegistry::MakeMultipleApplyNameInstance('
          f'PhysxSchemaTokens->{tmpl}, TfToken("{axis}"))),')
    w("    allTokens({")
    for member, _, _ in entries:
        w(f"        {member},")
    w("    })")
    w("{")
    w("}")
    w("")
    w("/// \\var PhysxAxisInstanceTokens")
    w("///")
    w("/// Codeless: defined inline (header-only) -- no compiled tokens.cpp / library.")
    w("inline TfStaticData<PhysxAxisInstanceTokensType> PhysxAxisInstanceTokens;")
    w("")
    w("PXR_NAMESPACE_CLOSE_SCOPE")
    w("")
    w("#endif")
    with open(args.out_h, "w") as fh:
        fh.write("\n".join(L) + "\n")
    print(f"gen_axis_instance_tokens: {len(templates)} templates x {len(AXES)} axes "
          f"= {len(entries)} instance tokens -> {args.out_h}")


if __name__ == "__main__":
    main()
