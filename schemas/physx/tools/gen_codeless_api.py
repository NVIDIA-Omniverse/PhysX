#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Generate header-only C++ wrappers (and a Python helper module) that preserve the
PhysxSchema.* API surface for a CODELESS PhysxSchema.

Why this exists
---------------
When PhysxSchema becomes codeless (skipCodeGeneration=true), usdGenSchema emits no
C++ classes and no Python bindings -- only plugInfo.json + generatedSchema.usda, which
USD uses to *register* the prim/API types. This tool regenerates the convenience API
that ~147 C++ files and ~45 Python files rely on, WITHOUT a compiled schema library:

  * C++  -> one inline, header-only wrapper per class. Existing call sites and
            #includes keep working unchanged. No .cpp, no libphysxSchema.so.
  * Py   -> a module reproducing PhysxSchema.PhysxFooAPI backed by raw USD access.

Type *registration* is data-driven by the codeless plugin, so these wrappers are pure
convenience: Apply() calls UsdPrim::ApplyAPI(TfToken), attribute accessors wrap
UsdPrim::GetAttribute / UsdSchemaBase::_CreateAttr, and tokens come from the header-only
physxSchema/tokens.h (PhysxSchemaTokens->...), which keeps the ~8,725 token call
sites untouched.

Input : source/physxSchema/schema.usda            (read as a raw Sdf.Layer, no compose)
Token names verified against the generated tokens.h (authoritative member set).
"""
import argparse
import os
import re
import sys

from pxr import Sdf


# ---------------------------------------------------------------------------
# Library naming -- overridden in main() from --prefix/--lib-dir. The defaults
# reproduce the original PhysxSchema output byte-for-byte.
#   CXX_PREFIX : C++/Python class prefix + tokens-var stem (e.g. "PhysxSchema")
#   LIB_DIR    : include directory + umbrella-header stem  (e.g. "physxSchema")
#   TOKENS_VAR : inline TfStaticData global from gen_tokens.py (e.g. "PhysxSchemaTokens")
# ---------------------------------------------------------------------------
CXX_PREFIX = "PhysxSchema"
LIB_DIR = "physxSchema"
TOKENS_VAR = "PhysxSchemaTokens"


# ---------------------------------------------------------------------------
# Sdf value type  ->  C++ "SdfValueTypeNames->X" member name (exact, by reflection)
# ---------------------------------------------------------------------------
def build_type_member_map():
    m = {}
    for member in dir(Sdf.ValueTypeNames):
        v = getattr(Sdf.ValueTypeNames, member)
        if isinstance(v, Sdf.ValueTypeName):
            m.setdefault(v, member)
    return m


# ---------------------------------------------------------------------------
# Token member name for a property full-name, matching usdGenSchema's CamelCase:
#   "physxCollision:contactOffset" -> "physxCollisionContactOffset"
#   "gearing"                      -> "gearing"
# Verified against the authoritative set parsed from tokens.h.
# ---------------------------------------------------------------------------
def camel(full_name):
    """camelCase a (possibly namespaced) property name: 'physics:protoIndices' -> 'physicsProtoIndices'."""
    parts = full_name.split(":")
    return parts[0] + "".join(p[:1].upper() + p[1:] for p in parts[1:])


def proper(name):
    return name[:1].upper() + name[1:]


def doc_comment(doc, indent=""):
    """Render schema documentation as Doxygen-style `///` comment lines (or [] if empty).
    Preserves the doc strings the codefull headers carried.
    Each line is left/right-stripped to match usdGenSchema: schema.usda authors the doc as an
    indented block string, but the generated header emits every line flush after `/// ` (the
    source indentation must NOT leak into the comment)."""
    if not doc:
        return []
    return [f"{indent}/// {line.strip()}".rstrip() for line in doc.strip().splitlines()]


def block(indent, text):
    """Emit a fixed boilerplate doc block verbatim: each line of `text` becomes
    '<indent>/// <line>' (internal indentation preserved, trailing ws stripped; blank -> '///').
    Used to reproduce usdGenSchema's method docs (constructor/Apply/Get/...) exactly."""
    out = []
    for line in text.split("\n"):
        out.append((f"{indent}/// {line}" if line else f"{indent}///").rstrip())
    return out


def build_decl_defaults(schema_usda):
    """Map (className, propName) -> raw default text exactly as authored in schema.usda, for
    the Doxygen metadata table's `Declaration` line. Read from raw text because parsed values
    lose the authored spelling (float 0.00005 -> 4.9999e-05). bool true/false normalized to 1/0
    to match usdGenSchema. Returns {} silently if anything is off (table just omits defaults)."""
    out = {}
    cls = None
    cls_re = re.compile(r'^(?:class|over)\s+(?:"([^"]+)"|(\w+))')
    prop_re = re.compile(
        r'^\s+(?:custom\s+)?(?:uniform\s+)?[A-Za-z][\w]*(?:\[\])?\s+([A-Za-z_][\w:]*)\s*'
        r'(?:=\s*(.+?))?\s*\(?\s*$')
    try:
        with open(schema_usda) as f:
            for line in f:
                m = cls_re.match(line)
                if m:
                    cls = m.group(1) or m.group(2)
                    continue
                if cls is None:
                    continue
                pm = prop_re.match(line.rstrip("\n"))
                if pm:
                    name, default = pm.group(1), pm.group(2)
                    if default is not None:
                        default = default.strip()
                        if default.lower() == "true":
                            default = "1"        # usdGenSchema renders bool true/True -> 1
                        elif default.lower() == "false":
                            default = "0"        # ... and false/False -> 0
                        else:
                            # usdGenSchema renders floats canonically: 0.0 -> 0, 1.50 -> 1.5,
                            # but keeps 0.00005. Trim trailing zeros on each decimal literal
                            # (also fixes vector components like (0.0, -981.0, 0.0)).
                            default = re.sub(r'-?\d+\.\d+',
                                             lambda m: m.group(0).rstrip("0").rstrip("."),
                                             default)
                            # and normalize tuple spacing: (0,0,0) -> (0, 0, 0)
                            default = re.sub(r',(?=\S)', ', ', default)
                    out[(cls, name)] = default
    except OSError:
        return {}
    return out


def token_member_for(full_name, kind="single", ns_prefix=None):
    if kind == "multiple" and ns_prefix:
        # usdGenSchema multiple-apply convention:
        #   <nsPrefix>_MultipleApplyTemplate_<ProperCase(property base name)>
        base = "".join(p[:1].upper() + p[1:] for p in full_name.split(":"))
        return f"{ns_prefix}_MultipleApplyTemplate_{base}"
    parts = full_name.split(":")
    return parts[0] + "".join(p[:1].upper() + p[1:] for p in parts[1:])


def load_token_members(tokens_h):
    members = set()
    if tokens_h and os.path.isfile(tokens_h):
        rx = re.compile(r"const\s+TfToken\s+([A-Za-z_]\w*)\s*;")
        with open(tokens_h) as f:
            for line in f:
                mo = rx.search(line)
                if mo:
                    members.add(mo.group(1))
    return members


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
class Attr:
    def __init__(self, full_name, api_name, type_member, variability, token_member):
        self.full_name = full_name
        self.api_name = api_name              # e.g. "contactOffset"
        self.type_member = type_member        # e.g. "Float"
        self.variability = variability        # "SdfVariabilityVarying" | "...Uniform"
        self.token_member = token_member      # e.g. "physxCollisionContactOffset"
        self.doc = ""                         # property documentation (from schema.usda)
        self.cpp_type = ""                    # C++ type for the Doxygen metadata table (e.g. "GfVec3f")
        self.usd_decl = ""                    # USD declaration type, e.g. "uniform token" / "bool"
        self.default_text = None              # raw default as authored in schema.usda (or None)
        self.allowed_tokens = []              # allowedTokens (for the table's "Allowed Values" row)
        self.is_uniform = False               # uniform variability (adds a "Variability" table row)


# Hand-written custom methods that usdGenSchema preserves below the codegen marker.
# (className -> list of (cpp_decl_lines, py_method_lines)). Kept tiny and explicit.
CUSTOM_METHODS = {
    "PhysxMeshMergeCollisionAPI": {
        "token": "collisionmeshes", "method": "GetCollisionMeshesCollectionAPI",
        "doc": ("Return the UsdCollectionAPI interface used for defining\n"
                "what prims belong to the mesh merge API."),
    },
    "PhysxSceneQuasistaticAPI": {
        "token": "quasistaticactors", "method": "GetQuasistaticActorsCollectionAPI",
        "doc": ("Return the UsdCollectionAPI interface used for defining\n"
                "what prims belong to the quasistatic API."),
    },
}


class Rel:
    def __init__(self, full_name, api_name, token_member):
        self.full_name = full_name
        self.api_name = api_name
        self.token_member = token_member
        self.doc = ""                         # relationship documentation (from schema.usda)


class Klass:
    def __init__(self, usd_type, cpp_name, file_base, kind, ns_prefix):
        self.usd_type = usd_type              # registered schema identifier (prim name), for ApplyAPI
        self.cpp_name = cpp_name              # e.g. "PhysxSchemaJointStateAPI" (honors className)
        self.file_base = file_base            # header file stem, e.g. "jointStateAPI"
        self.kind = kind                      # 'single' | 'multiple' | 'typed'
        self.ns_prefix = ns_prefix            # for multiple-apply
        self.attrs = []
        self.rels = []
        self.doc = ""                         # class documentation (from schema.usda)
        self.type_token_expr = f'TfToken("{usd_type}")'   # schema-identifier token expr for Apply/registry
        self.inherit_base = None              # raw `inherits` target name (physx class or USD type)
        self.parent = None                    # resolved physx-schema parent Klass (if any)
        self.base_cpp = None                  # base class (physx wrapper or upstream USD class)
        self.base_include = None              # include for that base


def _inherit_base(prim_spec):
    """First inherit target name from schema.usda (physx class or upstream USD type)."""
    il = prim_spec.inheritPathList
    items = (list(il.explicitItems) + list(il.prependedItems) +
             list(il.appendedItems) + list(il.addedItems))
    return items[0].name if items else None


def _usd_base_include(base_cpp, plugin_name):
    """Header for an upstream USD base class, derived from its plugin + class name:
    (UsdPhysicsJoint, usdPhysics) -> 'pxr/usd/usdPhysics/joint.h'.
    Returns None for the 'usd' core lib (UsdAPISchemaBase/UsdTyped are always #included)."""
    if not plugin_name or plugin_name == "usd":
        return None
    prefix = "Usd" + plugin_name[3:]                 # usdGeom -> UsdGeom
    name = base_cpp[len(prefix):] if base_cpp.startswith(prefix) else base_cpp
    return f"pxr/usd/{plugin_name}/{name[:1].lower() + name[1:]}.h"


def resolve_bases(classes, by_usd):
    """Set base_cpp/base_include on each class from the schema's `inherits` + the USD registry.
    Physx parent -> the physx wrapper; upstream USD type -> its C++ class + header. No codefull."""
    from pxr import Usd, Plug
    reg = Usd.SchemaRegistry
    preg = Plug.Registry()
    for k in classes:
        ib = k.inherit_base
        if not ib:
            continue
        if ib in by_usd:                              # inherits another physx schema class
            pk = by_usd[ib]
            k.parent = pk
            k.base_cpp = pk.cpp_name
            k.base_include = f"{LIB_DIR}/{pk.file_base}.h"
        else:                                         # inherits an upstream USD schema
            t = reg.GetTypeFromSchemaTypeName(ib)
            if t and t.typeName:
                plug = preg.GetPluginForType(t)
                k.base_cpp = t.typeName
                k.base_include = _usd_base_include(t.typeName, plug.name if plug else None)


def classify(prim_spec):
    cd = prim_spec.customData
    api_type = cd.get("apiSchemaType")
    if api_type == "multipleApply":
        return "multiple"
    if api_type == "singleApply" or prim_spec.name.endswith("API"):
        return "single"
    return "typed"


def parse_schema(schema_usda, token_members):
    layer = Sdf.Layer.FindOrOpen(schema_usda)
    if not layer:
        sys.exit(f"could not open {schema_usda}")
    type_map = build_type_member_map()
    decl_defaults = build_decl_defaults(schema_usda)   # (class, prop) -> raw default text
    classes = []
    skipped_tokens = []
    for ps in layer.rootPrims:
        if ps.name == "GLOBAL":
            continue
        kind = classify(ps)
        ns = ps.customData.get("propertyNamespacePrefix")
        class_name = ps.customData.get("className", ps.name)   # className overrides C++ name only
        file_base = class_name[:1].lower() + class_name[1:]
        k = Klass(ps.name, CXX_PREFIX + class_name, file_base, kind, ns)
        k.inherit_base = _inherit_base(ps)
        # Prefer the generated schema-identifier token (PhysxSchemaTokens->PhysxFooAPI) over a
        # bare TfToken("PhysxFooAPI") literal; fall back to the literal if it's not in tokens.h.
        k.type_token_expr = (f"{TOKENS_VAR}->{ps.name}"
                             if (not token_members or ps.name in token_members)
                             else f'TfToken("{ps.name}")')
        k.doc = (ps.documentation or "").strip()   # class doc (was lost vs the codefull headers)
        def resolve_token(pname):
            tmember = token_member_for(pname, kind, ns)
            if token_members and tmember not in token_members:
                for alt in (token_member_for(pname),
                            token_member_for(ps.properties[pname].customData.get("apiName", pname))):
                    if alt in token_members:
                        return alt
                skipped_tokens.append((k.usd_type, pname, tmember))
            return tmember

        for pname, prop in ps.properties.items():
            api_name = prop.customData.get("apiName") or camel(pname)
            tmember = resolve_token(pname)
            doc = (prop.documentation or "").strip()
            if isinstance(prop, Sdf.RelationshipSpec):
                r = Rel(pname, api_name, tmember)
                r.doc = doc
                k.rels.append(r)
                continue
            var = ("SdfVariabilityUniform"
                   if prop.variability == Sdf.VariabilityUniform
                   else "SdfVariabilityVarying")
            tmem = type_map.get(prop.typeName, None)
            a = Attr(pname, api_name, tmem, var, tmember)
            a.doc = doc
            # Doxygen metadata-table data (matches usdGenSchema's `| Declaration |` block)
            aliases = list(prop.typeName.aliasesAsStrings) if prop.typeName else []
            alias = aliases[0] if aliases else str(prop.typeName)
            a.usd_decl = ("uniform " if prop.variability == Sdf.VariabilityUniform else "") + alias
            a.cpp_type = prop.typeName.cppTypeName if prop.typeName else ""
            a.default_text = decl_defaults.get((ps.name, pname))
            a.is_uniform = (prop.variability == Sdf.VariabilityUniform)
            try:
                at = prop.GetInfo("allowedTokens")
                a.allowed_tokens = [str(t) for t in at] if at else []
            except Exception:
                a.allowed_tokens = []
            k.attrs.append(a)
        classes.append(k)

    # resolve base classes (physx parents + upstream USD bases) from `inherits` + USD registry
    by_usd = {k.usd_type: k for k in classes}
    resolve_bases(classes, by_usd)
    # order so a parent is always emitted before its child (matters for the Python module)
    ordered, seen = [], set()
    def add(k):
        if k.usd_type in seen:
            return
        if k.parent:
            add(k.parent)
        seen.add(k.usd_type)
        ordered.append(k)
    for k in classes:
        add(k)
    return ordered, skipped_tokens


# ---------------------------------------------------------------------------
# C++ header-only emitter (single + multiple apply)
# ---------------------------------------------------------------------------
CPP_KIND = {"single": "SingleApplyAPI", "multiple": "MultipleApplyAPI", "typed": "ConcreteTyped"}


def emit_cpp_header(k):
    guard = f"{CXX_PREFIX.upper()}_GENERATED_{k.usd_type.upper()}_API_H"
    L = []
    w = L.append
    w("// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.")
    w("// SPDX-License-Identifier: Apache-2.0")
    w("//")
    w("// GENERATED by tools/gen_codeless_api.py -- header-only codeless wrapper. DO NOT EDIT.")
    w(f"/// \\file {LIB_DIR}/{k.file_base}.h")
    w(f"#ifndef {guard}")
    w(f"#define {guard}")
    w("")
    w(f'#include "{LIB_DIR}/tokens.h"')
    # OpenUSD (pxr/*) headers are project-external -> angle-bracket includes.
    w("#include <pxr/usd/usd/apiSchemaBase.h>")
    w("#include <pxr/usd/usd/typed.h>")
    w("#include <pxr/usd/usd/prim.h>")
    w("#include <pxr/usd/usd/stage.h>")
    w("#include <pxr/usd/usd/schemaRegistry.h>")
    w("#include <pxr/usd/usd/collectionAPI.h>")
    w("#include <pxr/usd/sdf/path.h>")
    w("#include <pxr/base/tf/type.h>")
    w("#include <algorithm>")
    w("#include <string>")
    w("#include <vector>")
    # authoritative base + its include (from the codefull header): handles physx-schema
    # parents AND upstream USD bases (UsdPhysicsJoint, UsdGeomGprim, UsdGeomImageable, ...).
    # pxr/* bases use angle brackets (external); sibling physxSchema/* headers use quotes.
    if k.base_include:
        inc = (f"<{k.base_include}>" if k.base_include.startswith("pxr/") else f'"{k.base_include}"')
        w(f"#include {inc}")
    elif k.parent:
        w(f'#include "{LIB_DIR}/{k.parent.file_base}.h"')
    w("")
    w("PXR_NAMESPACE_OPEN_SCOPE")
    w("")
    if k.base_cpp:
        base = k.base_cpp
    elif k.parent:
        base = k.parent.cpp_name
    else:
        base = "UsdAPISchemaBase" if k.kind in ("single", "multiple") else "UsdTyped"
    # Class doc, wrapped like usdGenSchema's \class block.
    w(f"/// \\class {k.cpp_name}")
    w("///")
    for line in doc_comment(k.doc):
        w(line)
    w("///")
    # usdGenSchema appends a token-usage paragraph iff the class has a SCALAR token attr
    # (cpp type TfToken); array token[] attrs (VtArray<TfToken>) do not trigger it.
    if any(a.cpp_type == "TfToken" for a in k.attrs):
        w("/// For any described attribute \\em Fallback \\em Value or \\em Allowed \\em Values below")
        w(f"/// that are text/tokens, the actual token is published and defined in \\ref {TOKENS_VAR}.")
        w(f'/// So to set an attribute to the value "rightHanded", use {TOKENS_VAR}->rightHanded')
        w("/// as the value.")
        w("///")
    w(f"class {k.cpp_name} : public {base}")
    w("{")
    w("public:")
    L.extend(block("    ",
        "Compile time constant representing what kind of schema this class is.\n"
        "\n"
        "\\sa UsdSchemaKind"))
    w(f"    static const UsdSchemaKind schemaKind = UsdSchemaKind::{CPP_KIND[k.kind]};")
    w("")
    # Codeless static TfType: resolve from the data-driven schema registry (no TF_REGISTRY).
    # This lets templated UsdPrim::IsA<T>() / HasAPI<T>() work unchanged for consumers.
    w("    static const TfType &_GetStaticTfType()")
    w("    {")
    w(f'        static TfType t = UsdSchemaRegistry::GetTypeFromSchemaTypeName({k.type_token_expr});')
    w("        return t;")
    w("    }")
    w("")
    if k.kind == "multiple":
        L.extend(block("    ",
            f"Construct a {k.cpp_name} on UsdPrim \\p prim with\n"
            "name \\p name . Equivalent to\n"
            f"{k.cpp_name}::Get(\n"
            "   prim.GetStage(),\n"
            "   prim.GetPath().AppendProperty(\n"
            f'       "{k.ns_prefix}:name"));\n'
            "\n"
            "for a \\em valid \\p prim, but will not immediately throw an error for\n"
            "an invalid \\p prim"))
        w(f"    explicit {k.cpp_name}(const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())")
        w(f"        : {base}(prim, name) {{}}")
        L.extend(block("    ",
            f"Construct a {k.cpp_name} on the prim held by \\p schemaObj with\n"
            "name \\p name.  Should be preferred over\n"
            f"{k.cpp_name}(schemaObj.GetPrim(), name), as it preserves\n"
            "SchemaBase state."))
        w(f"    explicit {k.cpp_name}(const UsdSchemaBase& schemaObj, const TfToken &name)")
        w(f"        : {base}(schemaObj, name) {{}}")
        w("    /// Returns the name of this multiple-apply schema instance")
        w("    TfToken GetName() const { return _GetInstanceName(); }")
    else:
        L.extend(block("    ",
            f"Construct a {k.cpp_name} on UsdPrim \\p prim .\n"
            f"Equivalent to {k.cpp_name}::Get(prim.GetStage(), prim.GetPath())\n"
            "for a \\em valid \\p prim, but will not immediately throw an error for\n"
            "an invalid \\p prim"))
        w(f"    explicit {k.cpp_name}(const UsdPrim& prim=UsdPrim()) : {base}(prim) {{}}")
        L.extend(block("    ",
            f"Construct a {k.cpp_name} on the prim held by \\p schemaObj .\n"
            f"Should be preferred over {k.cpp_name}(schemaObj.GetPrim()),\n"
            "as it preserves SchemaBase state."))
        w(f"    explicit {k.cpp_name}(const UsdSchemaBase& schemaObj) : {base}(schemaObj) {{}}")
    w("    /// Destructor.")
    w(f"    virtual ~{k.cpp_name}() {{}}")
    w("")
    # GetSchemaAttributeNames
    L.extend(block("    ",
        "Return a vector of names of all pre-declared attributes for this schema\n"
        "class and all its ancestor classes.  Does not include attributes that\n"
        "may be authored by custom/extended methods of the schemas involved."))
    w("    static const TfTokenVector &GetSchemaAttributeNames(bool includeInherited=true)")
    w("    {")
    w("        static TfTokenVector localNames = {")
    for a in k.attrs:
        w(f"            {TOKENS_VAR}->{a.token_member},")
    w("        };")
    w("        return localNames;")
    w("    }")
    w("")
    if k.kind == "multiple":
        schema_prefix_member = token_member_for(k.ns_prefix)
        path_helper = f"Is{k.usd_type}Path"
        L.extend(block("    ",
            "Return a vector of names of all pre-declared attributes for this schema\n"
            "class and all its ancestor classes for a given instance name.  Does not\n"
            "include attributes that may be authored by custom/extended methods of\n"
            "the schemas involved. The names returned will have the proper namespace\n"
            "prefix."))
        w("    static TfTokenVector GetSchemaAttributeNames(bool includeInherited, const TfToken &instanceName)")
        w("    {")
        w("        const TfTokenVector &attrNames = GetSchemaAttributeNames(includeInherited);")
        w("        if (instanceName.IsEmpty()) {")
        w("            return attrNames;")
        w("        }")
        w("        TfTokenVector result;")
        w("        result.reserve(attrNames.size());")
        w("        for (const TfToken &attrName : attrNames) {")
        w("            result.push_back(UsdSchemaRegistry::MakeMultipleApplyNameInstance(attrName, instanceName));")
        w("        }")
        w("        return result;")
        w("    }")
        w("")
        L.extend(block("    ",
            "Checks if the given name \\p baseName is the base name of a property\n"
            f"of {k.usd_type}."))
        w("    static bool IsSchemaPropertyBaseName(const TfToken &baseName)")
        w("    {")
        w("        static TfTokenVector attrsAndRels = {")
        for p in list(k.attrs) + list(k.rels):
            w("            UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(")
            w(f"                {TOKENS_VAR}->{p.token_member}),")
        w("        };")
        w("        return std::find(attrsAndRels.begin(), attrsAndRels.end(), baseName) != attrsAndRels.end();")
        w("    }")
        w("")
        L.extend(block("    ",
            "Checks if the given path \\p path is of an API schema of type\n"
            f"{k.usd_type}. If so, it stores the instance name of\n"
            "the schema in \\p name and returns true. Otherwise, it returns false."))
        w(f"    static bool {path_helper}(const SdfPath &path, TfToken *name)")
        w("    {")
        w("        if (!path.IsPropertyPath()) {")
        w("            return false;")
        w("        }")
        w("        const std::string propertyName = path.GetName();")
        w("        const TfTokenVector tokens = SdfPath::TokenizeIdentifierAsTokens(propertyName);")
        w("        if (tokens.empty()) {")
        w("            return false;")
        w("        }")
        w("        const TfToken &baseName = *tokens.rbegin();")
        w("        if (IsSchemaPropertyBaseName(baseName)) {")
        w("            return false;")
        w("        }")
        w("        if (tokens.size() >= 2")
        w(f"            && tokens[0] == {TOKENS_VAR}->{schema_prefix_member}) {{")
        w("            if (name) {")
        w(f"                *name = TfToken(propertyName.substr({TOKENS_VAR}->{schema_prefix_member}.GetString().size() + 1));")
        w("            }")
        w("            return true;")
        w("        }")
        w("        return false;")
        w("    }")
        w("")
    # Get
    if k.kind == "multiple":
        L.extend(block("    ",
            f"Return a {k.cpp_name} with name \\p name holding the\n"
            f"prim \\p prim. Shorthand for {k.cpp_name}(prim, name);"))
        w(f"    static {k.cpp_name} Get(const UsdPrim &prim, const TfToken &name)")
        w(f"    {{ return {k.cpp_name}(prim, name); }}")
        L.extend(block("    ",
            f"Return a {k.cpp_name} holding the prim adhering to this\n"
            "schema at \\p path on \\p stage.  If no prim exists at \\p path on\n"
            "\\p stage, or if the prim at that path does not adhere to this schema,\n"
            "return an invalid schema object.  \\p path must be of the format\n"
            "<path>." + f"{k.ns_prefix}:name .\n"
            "\n"
            "This is shorthand for the following:\n"
            "\n"
            "\\code\n"
            "TfToken name = SdfPath::StripNamespace(path.GetToken());\n"
            f"{k.cpp_name}(\n"
            "    stage->GetPrimAtPath(path.GetPrimPath()), name);\n"
            "\\endcode\n"))
        w(f"    static {k.cpp_name} Get(const UsdStagePtr &stage, const SdfPath &path)")
        w("    {")
        w("        if (!stage) {")
        w(f"            return {k.cpp_name}();")
        w("        }")
        w("        TfToken name;")
        w(f"        if (!{path_helper}(path, &name)) {{")
        w(f"            return {k.cpp_name}();")
        w("        }")
        w(f"        return {k.cpp_name}(stage->GetPrimAtPath(path.GetPrimPath()), name);")
        w("    }")
        L.extend(block("    ",
            f"Return a vector of all named instances of {k.cpp_name} on the\n"
            "given \\p prim."))
        w(f"    static std::vector<{k.cpp_name}> GetAll(const UsdPrim &prim)")
        w("    {")
        w(f"        std::vector<{k.cpp_name}> schemas;")
        w(f'        const TfType &type = UsdSchemaRegistry::GetTypeFromSchemaTypeName({k.type_token_expr});')
        w("        for (const auto &n : _GetMultipleApplyInstanceNames(prim, type)) {")
        w("            schemas.emplace_back(prim, n);")
        w("        }")
        w("        return schemas;")
        w("    }")
    else:
        L.extend(block("    ",
            f"Return a {k.cpp_name} holding the prim adhering to this\n"
            "schema at \\p path on \\p stage.  If no prim exists at \\p path on\n"
            "\\p stage, or if the prim at that path does not adhere to this schema,\n"
            "return an invalid schema object.  This is shorthand for the following:\n"
            "\n"
            "\\code\n"
            f"{k.cpp_name}(stage->GetPrimAtPath(path));\n"
            "\\endcode\n"))
        w(f"    static {k.cpp_name} Get(const UsdStagePtr &stage, const SdfPath &path)")
        w(f"    {{ return {k.cpp_name}(stage->GetPrimAtPath(path)); }}")
    w("")
    # Apply / CanApply / Define  (codeless-safe: string ApplyAPI)
    sa_tail = ("\\sa UsdPrim::GetAppliedSchemas()\n"
               "\\sa UsdPrim::HasAPI()\n"
               "\\sa UsdPrim::CanApplyAPI()\n"
               "\\sa UsdPrim::ApplyAPI()\n"
               "\\sa UsdPrim::RemoveAPI()")
    if k.kind == "single":
        L.extend(block("    ",
            "Returns true if this <b>single-apply</b> API schema can be applied to\n"
            "the given \\p prim. If this schema can not be a applied to the prim,\n"
            "this returns false and, if provided, populates \\p whyNot with the\n"
            "reason it can not be applied.\n"
            "\n"
            "Note that if CanApply returns false, that does not necessarily imply\n"
            "that calling Apply will fail. Callers are expected to call CanApply\n"
            "before calling Apply if they want to ensure that it is valid to\n"
            "apply a schema.\n"
            "\n" + sa_tail))
        w("    static bool CanApply(const UsdPrim &prim, std::string *whyNot=nullptr)")
        w(f'    {{ return prim.CanApplyAPI({k.type_token_expr}, whyNot); }}')
        L.extend(block("    ",
            "Applies this <b>single-apply</b> API schema to the given \\p prim.\n"
            f'This information is stored by adding "{k.usd_type}" to the\n'
            "token-valued, listOp metadata \\em apiSchemas on the prim.\n"
            "\n"
            f"\\return A valid {k.cpp_name} object is returned upon success.\n"
            f"An invalid (or empty) {k.cpp_name} object is returned upon\n"
            "failure. See \\ref UsdPrim::ApplyAPI() for conditions\n"
            "resulting in failure.\n"
            "\n" + sa_tail))
        w(f"    static {k.cpp_name} Apply(const UsdPrim &prim)")
        w("    {")
        w(f'        if (prim.ApplyAPI({k.type_token_expr})) {{ return {k.cpp_name}(prim); }}')
        w(f"        return {k.cpp_name}();")
        w("    }")
    elif k.kind == "multiple":
        L.extend(block("    ",
            "Returns true if this <b>multiple-apply</b> API schema can be applied,\n"
            "with the given instance name, \\p name, to the given \\p prim. If this\n"
            "schema can not be a applied the prim, this returns false and, if\n"
            "provided, populates \\p whyNot with the reason it can not be applied.\n"
            "\n"
            "Note that if CanApply returns false, that does not necessarily imply\n"
            "that calling Apply will fail. Callers are expected to call CanApply\n"
            "before calling Apply if they want to ensure that it is valid to\n"
            "apply a schema.\n"
            "\n" + sa_tail))
        w("    static bool CanApply(const UsdPrim &prim, const TfToken &name, std::string *whyNot=nullptr)")
        w(f'    {{ return prim.CanApplyAPI({k.type_token_expr}, name, whyNot); }}')
        L.extend(block("    ",
            "Applies this <b>multiple-apply</b> API schema to the given \\p prim\n"
            "along with the given instance name, \\p name.\n"
            "\n"
            f'This information is stored by adding "{k.usd_type}:<i>name</i>"\n'
            "to the token-valued, listOp metadata \\em apiSchemas on the prim.\n"
            "For example, if \\p name is 'instance1', the token\n"
            f"'{k.usd_type}:instance1' is added to 'apiSchemas'.\n"
            "\n"
            f"\\return A valid {k.cpp_name} object is returned upon success.\n"
            f"An invalid (or empty) {k.cpp_name} object is returned upon\n"
            "failure. See \\ref UsdPrim::ApplyAPI() for\n"
            "conditions resulting in failure.\n"
            "\n" + sa_tail))
        w(f"    static {k.cpp_name} Apply(const UsdPrim &prim, const TfToken &name)")
        w("    {")
        w(f'        if (prim.ApplyAPI({k.type_token_expr}, name)) {{ return {k.cpp_name}(prim, name); }}')
        w(f"        return {k.cpp_name}();")
        w("    }")
    else:  # typed
        L.extend(block("    ",
            "Attempt to ensure a \\a UsdPrim adhering to this schema at \\p path\n"
            "is defined (according to UsdPrim::IsDefined()) on this stage.\n"
            "\n"
            "If a prim adhering to this schema at \\p path is already defined on this\n"
            "stage, return that prim.  Otherwise author an \\a SdfPrimSpec with\n"
            "\\a specifier == \\a SdfSpecifierDef and this schema's prim type name for\n"
            "the prim at \\p path at the current EditTarget.  Author \\a SdfPrimSpec s\n"
            "with \\p specifier == \\a SdfSpecifierDef and empty typeName at the\n"
            "current EditTarget for any nonexistent, or existing but not \\a Defined\n"
            "ancestors.\n"
            "\n"
            "The given \\a path must be an absolute prim path that does not contain\n"
            "any variant selections.\n"
            "\n"
            "If it is impossible to author any of the necessary PrimSpecs, (for\n"
            "example, in case \\a path cannot map to the current UsdEditTarget's\n"
            "namespace) issue an error and return an invalid \\a UsdPrim.\n"
            "\n"
            "Note that this method may return a defined prim whose typeName does not\n"
            "specify this schema class, in case a stronger typeName opinion overrides\n"
            "the opinion at the current EditTarget."))
        w(f"    static {k.cpp_name} Define(const UsdStagePtr &stage, const SdfPath &path)")
        w(f'    {{ return {k.cpp_name}(stage->DefinePrim(path, {k.type_token_expr})); }}')
    w("")
    # Attribute accessors
    for a in k.attrs:
        if a.type_member is None:
            w(f"    // NOTE: attribute {a.full_name!r} has an unmapped value type -- skipped")
            continue
        getter = "Get" + proper(a.api_name) + "Attr"
        creator = "Create" + proper(a.api_name) + "Attr"
        if k.kind == "multiple":
            tokexpr = ("UsdSchemaRegistry::MakeMultipleApplyNameInstance("
                       f"{TOKENS_VAR}->{a.token_member}, _GetInstanceName())")
        else:
            tokexpr = f"{TOKENS_VAR}->{a.token_member}"
        # usdGenSchema-style section divider before each attribute group.
        w("")
        w("public:")
        w("    // " + "-" * 69 + " //")
        w(f"    // {a.api_name.upper()}")
        w("    // " + "-" * 69 + " //")
        for line in doc_comment(a.doc, "    "):
            w(line)
        # Doxygen metadata table (matches usdGenSchema): Declaration / C++ Type / Usd Type.
        decl = f"{a.usd_decl} {a.full_name}"
        if a.default_text is not None:
            decl += f" = {a.default_text}"
        rows = ["", "| ||", "| -- | -- |",
                f"| Declaration | `{decl}` |",
                f"| C++ Type | {a.cpp_type} |",
                f'| \\ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->{a.type_member} |']
        if a.is_uniform:
            rows.append('| \\ref SdfVariability "Variability" | SdfVariabilityUniform |')
        if a.allowed_tokens:
            rows.append(f'| \\ref {TOKENS_VAR} "Allowed Values" | {", ".join(a.allowed_tokens)} |')
        L.extend(block("    ", "\n".join(rows)))
        w(f"    UsdAttribute {getter}() const")
        w(f"    {{ return GetPrim().GetAttribute({tokexpr}); }}")
        # Standard create-accessor doc, matching usdGenSchema's codefull output.
        w(f"    /// See {getter}(), and also")
        w(f"    /// \\ref Usd_Create_Or_Get_Property for when to use Get vs Create.")
        w(f"    /// If specified, author \\p defaultValue as the attribute's default,")
        w(f"    /// sparsely (when it makes sense to do so) if \\p writeSparsely is \\c true -")
        w(f"    /// the default for \\p writeSparsely is \\c false.")
        w(f"    UsdAttribute {creator}(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const")
        w("    {")
        w(f"        return UsdSchemaBase::_CreateAttr({tokexpr},")
        w(f"                   SdfValueTypeNames->{a.type_member}, /* custom = */ false,")
        w(f"                   {a.variability}, defaultValue, writeSparsely);")
        w("    }")
    # Relationship accessors
    for r in k.rels:
        getter = "Get" + proper(r.api_name) + "Rel"
        creator = "Create" + proper(r.api_name) + "Rel"
        if k.kind == "multiple":
            tokexpr = ("UsdSchemaRegistry::MakeMultipleApplyNameInstance("
                       f"{TOKENS_VAR}->{r.token_member}, _GetInstanceName())")
        else:
            tokexpr = f"{TOKENS_VAR}->{r.token_member}"
        # usdGenSchema-style section divider before each relationship group.
        w("")
        w("public:")
        w("    // " + "-" * 69 + " //")
        w(f"    // {r.api_name.upper()}")
        w("    // " + "-" * 69 + " //")
        for line in doc_comment(r.doc, "    "):
            w(line)
        w(f"    UsdRelationship {getter}() const")
        w(f"    {{ return GetPrim().GetRelationship({tokexpr}); }}")
        # Standard create-accessor doc, matching usdGenSchema's codefull output.
        w(f"    /// See {getter}(), and also")
        w(f"    /// \\ref Usd_Create_Or_Get_Property for when to use Get vs Create")
        w(f"    UsdRelationship {creator}() const")
        w(f"    {{ return GetPrim().CreateRelationship({tokexpr}, /* custom = */ false); }}")
    # Hand-written custom methods preserved from the codefull schema
    cm = CUSTOM_METHODS.get(k.usd_type)
    if cm:
        if cm.get("doc"):
            L.extend(block("    ", cm["doc"]))
        w(f"    UsdCollectionAPI {cm['method']}() const")
        w(f"    {{ return UsdCollectionAPI(GetPrim(), {TOKENS_VAR}->{cm['token']}); }}")
    w("")
    w("protected:")
    L.extend(block("    ",
        "Returns the kind of schema this class belongs to.\n"
        "\n"
        "\\sa UsdSchemaKind"))
    w("    UsdSchemaKind _GetSchemaKind() const override { return schemaKind; }")
    # The virtual _GetTfType() backs UsdSchemaBase::operator bool / _IsCompatible():
    # without it the base type is used and the validity/HasAPI check fails, so a freshly
    # Apply()'d API schema reads as falsey (breaking `if (auto s = X::Apply(prim))`).
    w("    const TfType &_GetTfType() const override { return _GetStaticTfType(); }")
    w("};")
    w("")
    w("PXR_NAMESPACE_CLOSE_SCOPE")
    w("")
    w("#endif")

    # Vertical spacing: ensure a blank line separates each member function/accessor (codefull
    # spacing). Insert a blank before any member doc-comment block that directly follows a code
    # line, then collapse accidental double blanks.
    spaced = []
    for line in L:
        if (line.startswith("    ///") and spaced and spaced[-1] != ""
                and not spaced[-1].lstrip().startswith("//")
                and spaced[-1].strip() not in ("public:", "private:", "protected:", "{")):
            spaced.append("")
        if line == "" and spaced and spaced[-1] == "":
            continue
        spaced.append(line)
    return "\n".join(spaced) + "\n"


# ---------------------------------------------------------------------------
# Python emitter (generate-once module preserving PhysxSchema.* without bindings)
# ---------------------------------------------------------------------------
PY_TYPE_TO_SDF = None  # built lazily via pxr in the runtime module


def py_class_name(cpp_name):
    return cpp_name[len(CXX_PREFIX):] if cpp_name.startswith(CXX_PREFIX) else cpp_name


def emit_python_module(classes):
    L = []
    w = L.append
    w("# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.")
    w("# SPDX-License-Identifier: Apache-2.0")
    w(f'"""GENERATED by tools/gen_codeless_api.py -- codeless {CXX_PREFIX} Python API.')
    w("")
    w(f"Preserves the {CXX_PREFIX}.<Class> surface (Apply/Get/Get<Attr>Attr/Create<Attr>Attr)")
    w(f"for a codeless schema, backed by raw USD access. No compiled _{LIB_DIR} bindings.")
    w('"""')
    w("from pxr import Usd, Sdf, Tf")
    w("")
    class_names = [py_class_name(k.cpp_name) for k in classes]
    w(f"__all__ = {class_names!r}")
    w("")
    w("")
    w("class _Base:")
    w("    def __init__(self, prim=None, name=''):")
    w("        if prim is not None and hasattr(prim, 'GetPrim') and not hasattr(prim, 'IsValid'):")
    w("            prim = prim.GetPrim()")
    w("        self._prim = prim")
    w("        self._name = name")
    w("    def GetPrim(self): return self._prim")
    w("    def GetPath(self): return self._prim.GetPath() if self._prim else Sdf.Path()")
    w("    def GetName(self): return self._name")
    w("    def _IsCompatible(self): return True")
    w("    def __bool__(self):")
    w("        return bool(self._prim and hasattr(self._prim, 'IsValid')")
    w("                    and self._prim.IsValid() and self._IsCompatible())")
    w("")
    w("")
    # Mirror UsdSchemaBase::_CreateAttr's writeSparsely compare: it coerces the
    # default to the attribute's type before comparing to the fallback, so a float
    # default matching a float32 fallback is not needlessly authored.
    w("def _coerce_to_type(value, sdf_type):")
    w("    try:")
    w("        return sdf_type.arrayType.type.pythonClass([value])[0]")
    w("    except Exception:")
    w("        return value")
    w("")
    # Note: UsdPrim.HasAPI/CanApplyAPI/ApplyAPI/RemoveAPI accept the schema *name*
    # string natively, so the wrapper class methods below just pass cls._schemaName.
    # (No global monkey-patching of Usd.Prim.)
    for k in classes:
        cn = py_class_name(k.cpp_name)
        pybase = py_class_name(k.parent.cpp_name) if k.parent else "_Base"
        w(f"class {cn}({pybase}):")
        attr_names = [f"{k.ns_prefix}:__INSTANCE_NAME__:{a.full_name}" if k.kind == "multiple" else a.full_name
                      for a in k.attrs]
        property_base_names = sorted({p.full_name.split(":")[-1] for p in list(k.attrs) + list(k.rels)})
        w(f'    _schemaName = "{k.usd_type}"')
        w(f"    schemaKind = {k.kind!r}")
        w(f"    _schemaAttributeNames = {attr_names!r}")
        if k.kind == "multiple":
            w(f"    _propertyNamespacePrefix = {k.ns_prefix!r}")
            w(f"    _schemaPropertyBaseNames = {property_base_names!r}")
        w("    @classmethod")
        w("    def _GetStaticTfType(cls):")
        w("        return Usd.SchemaRegistry.GetTypeFromSchemaTypeName(cls._schemaName)")
        w("    @classmethod")
        w("    def GetSchemaAttributeNames(cls, includeInherited=True, instanceName=None):")
        w("        names = list(cls._schemaAttributeNames)")
        w("        base = cls.__bases__[0]")
        w("        if includeInherited and base is not _Base and hasattr(base, 'GetSchemaAttributeNames'):")
        w("            names = list(base.GetSchemaAttributeNames(True)) + names")
        w("        if instanceName is not None:")
        w("            instanceName = str(instanceName)")
        w("            if instanceName:")
        w("                names = [name.replace(':__INSTANCE_NAME__:', ':' + instanceName + ':') for name in names]")
        w("        return names")
        if k.kind == "multiple":
            path_helper = f"Is{k.usd_type}Path"
            w("    @classmethod")
            w("    def _GetMultipleApplyInstanceNameFromPath(cls, path):")
            w("        if not path.IsPropertyPath():")
            w("            return ''")
            w("        property_name = path.name")
            w("        tokens = property_name.split(':')")
            w("        if len(tokens) < 2 or tokens[-1] in cls._schemaPropertyBaseNames:")
            w("            return ''")
            w("        prefix = cls._propertyNamespacePrefix")
            w("        if property_name.startswith(prefix + ':'):")
            w("            return property_name[len(prefix) + 1:]")
            w("        return ''")
            w("    @classmethod")
            w(f"    def {path_helper}(cls, path):")
            w("        return bool(cls._GetMultipleApplyInstanceNameFromPath(path))")
        # constructors / Get / Apply / Define
        if k.kind == "multiple":
            w("    @classmethod")
            w("    def CanApply(cls, prim, name):")
            w("        return prim.CanApplyAPI(cls._schemaName, name)  # native _CanApplyAPIResult (bool + .whyNot)")
            w("    @classmethod")
            w("    def Apply(cls, prim, name):")
            w("        if prim.ApplyAPI(cls._schemaName, name):")
            w("            return cls(prim, name)")
            w("        return cls()")
            w("    @classmethod")
            w("    def Get(cls, prim_or_stage, name_or_path):")
            w("        if isinstance(name_or_path, Sdf.Path):")
            w("            instance_name = cls._GetMultipleApplyInstanceNameFromPath(name_or_path)")
            w("            if not instance_name:")
            w("                return cls()")
            w("            return cls(prim_or_stage.GetPrimAtPath(name_or_path.GetPrimPath()), instance_name)")
            w("        return cls(prim_or_stage, name_or_path)")
            w("    def _IsCompatible(self):")
            w("        return bool(self._name) and self._prim.HasAPI(self._schemaName, self._name)")
        elif k.kind == "single":
            w("    @classmethod")
            w("    def CanApply(cls, prim):")
            w("        return prim.CanApplyAPI(cls._schemaName)  # native _CanApplyAPIResult (bool + .whyNot)")
            w("    @classmethod")
            w("    def Apply(cls, prim):")
            w("        if prim.ApplyAPI(cls._schemaName):")
            w("            return cls(prim)")
            w("        return cls()")
            w("    @classmethod")
            w("    def Get(cls, stage, path): return cls(stage.GetPrimAtPath(path))")
            w("    def _IsCompatible(self):")
            w("        return self._prim.HasAPI(self._schemaName)")
        else:  # typed
            w("    @classmethod")
            w("    def Define(cls, stage, path): return cls(stage.DefinePrim(path, cls._schemaName))")
            w("    @classmethod")
            w("    def Get(cls, stage, path): return cls(stage.GetPrimAtPath(path))")
            w("    def _IsCompatible(self):")
            w("        t = self._GetStaticTfType()")
            w("        return bool(t.typeName) and self._prim.IsA(t)")
        # attribute accessors
        for a in k.attrs:
            if a.type_member is None:
                continue
            base = proper(a.api_name)
            if k.kind == "multiple":
                # multiple-apply instance property name: <nsPrefix>:<instance>:<baseName>
                tok = '"%s:" + self._name + ":%s"' % (k.ns_prefix, a.full_name)
            else:
                tok = f'"{a.full_name}"'
            w(f"    def Get{base}Attr(self):")
            w(f"        return self._prim.GetAttribute({tok})")
            w(f"    def Create{base}Attr(self, defaultValue=None, writeSparsely=False):")
            w(f"        _t = Sdf.ValueTypeNames.{a.type_member}")
            # Mirror UsdSchemaBase::_CreateAttr: when writeSparsely, don't author a
            # value that already matches the (unauthored) fallback -- comparing the
            # default coerced to the attribute type, as the compiled binding does.
            w("        if writeSparsely and defaultValue is not None:")
            w(f"            attr = self._prim.GetAttribute({tok})")
            w("            if attr and not attr.HasAuthoredValue() and attr.Get() == _coerce_to_type(defaultValue, _t):")
            w("                return attr")
            w(f"        attr = self._prim.CreateAttribute({tok}, _t, False)")
            w("        if defaultValue is not None: attr.Set(defaultValue)")
            w("        return attr")
        # relationships
        for r in k.rels:
            base = proper(r.api_name)
            if k.kind == "multiple":
                tok = '"%s:" + self._name + ":%s"' % (k.ns_prefix, r.full_name)
            else:
                tok = f'"{r.full_name}"'
            w(f"    def Get{base}Rel(self):")
            w(f"        return self._prim.GetRelationship({tok})")
            w(f"    def Create{base}Rel(self):")
            w(f"        return self._prim.CreateRelationship({tok}, False)")
        # GetAll for multiple-apply
        if k.kind == "multiple":
            w("    @classmethod")
            w("    def GetAll(cls, prim):")
            w('        prefix = cls._schemaName + ":"')
            w("        out = []")
            w("        for s in prim.GetAppliedSchemas():")
            w("            if s == cls._schemaName:")
            w("                out.append(cls(prim, ''))")
            w("            elif s.startswith(prefix):")
            w("                out.append(cls(prim, s[len(prefix):]))")
            w("        return out")
        # hand-written custom methods
        cm = CUSTOM_METHODS.get(k.usd_type)
        if cm:
            w(f"    def {cm['method']}(self):")
            w(f"        return Usd.CollectionAPI(self._prim, \"{cm['token']}\")")
        w("")
    return "\n".join(L).rstrip() + "\n"


def emit_umbrella_header(classes):
    """A convenience header pulling in every wrapper + the tokens, so a consumer can
    `#include "physxSchema/physxSchema.h"` instead of listing each per-class header."""
    L = []
    w = L.append
    w("// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.")
    w("// SPDX-License-Identifier: Apache-2.0")
    w("//")
    w(f"// GENERATED by tools/gen_codeless_api.py -- umbrella include for the codeless {CXX_PREFIX}")
    w("// C++ API. Includes every header-only wrapper and the token table. DO NOT EDIT.")
    w(f"#ifndef {CXX_PREFIX.upper()}_GENERATED_{LIB_DIR.upper()}_H")
    w(f"#define {CXX_PREFIX.upper()}_GENERATED_{LIB_DIR.upper()}_H")
    w("")
    w(f'#include "{LIB_DIR}/tokens.h"')
    for k in sorted(classes, key=lambda c: c.file_base):
        w(f'#include "{LIB_DIR}/{k.file_base}.h"')
    w("")
    w("#endif")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schema", required=True, help="path to schema.usda")
    ap.add_argument("--tokens-h", default=None, help="generated tokens.h (authoritative token set)")
    ap.add_argument("--out-cpp", required=True, help="output dir for header-only C++ wrappers")
    ap.add_argument("--out-py", default=None, help="output path for the Python module")
    ap.add_argument("--prefix", default="PhysxSchema",
                    help="C++/Python class prefix + tokens-var stem (e.g. PhysxSchema -> "
                         "PhysxSchemaFooAPI / PhysxSchemaTokens). Default: PhysxSchema.")
    ap.add_argument("--lib-dir", default="physxSchema",
                    help="include directory + umbrella-header stem (e.g. physxSchema -> "
                         '#include "physxSchema/...", umbrella physxSchema.h). Default: physxSchema.')
    args = ap.parse_args()

    global CXX_PREFIX, LIB_DIR, TOKENS_VAR
    CXX_PREFIX = args.prefix
    LIB_DIR = args.lib_dir
    TOKENS_VAR = f"{args.prefix}Tokens"

    token_members = load_token_members(args.tokens_h)
    classes, skipped = parse_schema(args.schema, token_members)   # resolves bases from schema + registry

    os.makedirs(args.out_cpp, exist_ok=True)
    counts = {"single": 0, "multiple": 0, "typed": 0}
    nattrs = 0
    generated_headers = []
    for k in classes:
        counts[k.kind] += 1
        nattrs += len(k.attrs)
        fname = k.file_base + ".h"
        with open(os.path.join(args.out_cpp, fname), "w") as f:
            f.write(emit_cpp_header(k))
        generated_headers.append(fname)

    umbrella = f"{LIB_DIR}.h"
    with open(os.path.join(args.out_cpp, umbrella), "w") as f:
        f.write(emit_umbrella_header(classes))
    generated_headers.append(umbrella)

    # Record exactly the headers generated here so install.cmake ships this set
    # rather than globbing *.h -- a class deleted from schema.usda drops out of
    # the manifest instead of leaving an orphan header that the glob republishes.
    with open(os.path.join(args.out_cpp, "codeless_headers.manifest"), "w") as f:
        f.write("\n".join(sorted(generated_headers)) + "\n")

    if args.out_py:
        os.makedirs(os.path.dirname(args.out_py) or ".", exist_ok=True)
        with open(args.out_py, "w") as f:
            f.write(emit_python_module(classes))

    print(f"classes: {len(classes)}  (single={counts['single']} "
          f"multiple={counts['multiple']} typed={counts['typed']})  attrs={nattrs}")
    print(f"token members loaded: {len(token_members)}")
    if skipped:
        print(f"WARNING: {len(skipped)} properties had no matching token member:")
        for c, p, t in skipped[:15]:
            print(f"   {c}.{p} -> {t}")


if __name__ == "__main__":
    main()
