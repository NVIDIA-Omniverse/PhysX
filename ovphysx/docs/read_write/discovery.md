<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Discovering attribute metadata

The [attribute reference](attributes/index.md), C names and Python catalog are
generated from the same versioned runtime I/O contract. They describe static
capabilities; a live query/session determines membership and availability.

## Python

Metadata discovery works without loading the native library or creating a scene:

```python
from ovphysx import attributes

angular = attributes.get_attribute("RIGID_BODY", "angularVelocity")
print(angular.dtype, angular.unit["display"], angular.frame)
# f32 rad / sec world
print(angular.read["layout"]["lanes"])  # 3

for field in attributes.list_attributes("ARTICULATION_JOINT", writable=True):
    print(field.name, field.unit["display"], field.write["conditions"])

position = attributes.get_attribute("RIGID_BODY", attributes.NAMES["position"])
instanced = attributes.get_attribute("RIGID_BODY", "positions", representation="instancer")
```

Use `field.read["request_name"]` or `field.write["request_name"]` when making an
API call. For example, instancer `positions` is an emitted group name; its request
name is `position`. The directional `aliases` list states which extra request
spellings that direction accepts. Particle `positions` is a read alias of `points`
and is not accepted for writes. Catalog lookup aliases resolve descriptors and
do not imply acceptance by both directions.

Descriptors are hashable and their nested mappings are immutable. Targets accept a name (case
insensitive), `OVPHYSX_QUERY_TARGET_*` spelling, a plain integer, or `SimObjectType`.
Other enums, including legacy `ObjectType`, are rejected because their numbering differs. An
unknown target or invalid representation raises `ValueError`; an unsupported target/name/representation
combination raises `KeyError`. `list_attributes` returns canonical uses in stable
ID order, supports `readable`, `writable` and `representation` filters, and
returns an empty tuple for a valid target with no matching attributes. Aliases
resolve to the canonical descriptor.

Read and write layouts can differ. Check `supported` before interpreting either.
Symbolic widths depend on the returned group and must not be evaluated as code.
Use `POLICIES` to resolve condition descriptions and `UNITS` to inspect unit
metadata. `unit["display"]` gives readable runtime units such as `distance / sec`
and `mass * distance^2 / sec^2`. `unit["variants"]` gives conditional unit strings
where the axis, drive or material matters; `unit["components"]` lists registry IDs
for ordered packed values. Population scale exponents are not part of this metadata.
A direction may override the default unit ID in `read["units"]` or `write["units"]`
(for example the packed culling-box write). Unsupported directions have no layout.
See [unit conventions](attributes/units.md). `SCHEMA_VERSION` versions the metadata
format, not the runtime ABI.

Directional `value_constraints`, when present, describe finite values, inclusive
`minimum`/`maximum` bounds, allowed `values`, and `additional_values` outside the
range (for example requested gear 255). `applicability` describes supported drive
or object subtypes. These are static descriptions; backend checks remain authoritative.

## C and C++

Include `ovphysx.h` for the generated string-backed name objects. Existing
`OVPHYSX_ATTR_*` string macros remain available in `ovphysx_attributes.h`.
Use a name directly in the session API:

```c
const ovx_token_or_string_t names[] = { OVPHYSX_NAME_POSITION, OVPHYSX_NAME_ORIENTATION };
/* Pass names and their count to ovphysx_read or ovphysx_write. */
```

For repeated lookups, convert the string to a numeric ID in the attached ovstage's dictionary:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attribute_tokens.h>

/* dictionary is the live dictionary supplied by the stage/session owner. */
ovx_token_t token = OVX_INVALID_TOKEN;
ovx_api_result_t result = ovphysx_intern_attribute_name(
    dictionary, OVPHYSX_NAME_POSITION.string, &token);
if (result.status == OVX_API_SUCCESS) {
    ovx_token_or_string_t local_name = { token, { 0, 0 } };
    /* Use local_name only with this dictionary's owner. */
} else if (result.error.ptr) {
    path_dictionary_release_error(dictionary, result.error);
}
```

The helper clears the output on failure and forwards the dictionary error.
Tokens are dictionary-local and become invalid when the owner destroys the
dictionary. Store any token cache with that owner. Portable `OVPHYSX_NAME_*`
objects contain strings and never cache a token.

See [token interning](../api.md#attribute-token-interning) for dictionary ownership and error handling.
