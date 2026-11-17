<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Publication selector cache experiment

`build.py --output-query-cache` is an opt-in experiment, disabled by default. It
prepares a WASM-only copy of `OvStageOutput.cpp`; the native source and public
header are unchanged. Use a separate complete runtime build directory and add
this option to the normal runtime-object build. Rebuild the API component and
link through the existing integration recipe. Do not write into an overlay whose
object files are symlinks to a preserved baseline.

The existing output cache binds to one instance, stage and attachment. This
experiment additionally retains its ALL rigid-body and ALL articulation-link
selectors. These are the native API's reusable selectors: each publication opens
fresh read sessions for current positions, orientations and signed world scales.
Read groups, tensor pointers, dictionary prim-list handles and stage queries never
enter the persistent cache. Hierarchy computation, bounds, ordinal validation and
matrix composition retain the existing implementation.

Call `refresh()` before consuming a known topology change. It releases the two
selectors as well as obsolete matrix buffers; it keeps the attachment binding and
does not permit reuse after reattachment. A rejected release retains ownership
and reports failure through the existing C++ exception path; an accepted release
is never retried, including when its completion reports an error. Destruction
uses the existing best-effort cleanup path and must precede instance/stage
destruction. The WASM binding already destroys its cache before detaching.

`host/prepare-output-query-cache.py` saves the original source, generated source
and hashes. The option is recorded in the runtime receipt and compiled-cache
identity. Threading, exception mode and optimization level remain independent
experiments; measure this change separately before combining results.

## Qualification

The normal package exposes no new JavaScript API. A separate qualification image
adds test-only exports for cache refresh, uncached publication and the existing
public `ovphysx_update_from_ovstage` API:

```sh
python3 ovphysx/experimental/wasm/integration/build-output-query-test.py \
  --candidate /path/to/query-cache-runtime --out /tmp/query-cache-qualification

timeout 90s /path/to/node24 \
  ovphysx/experimental/wasm/integration/test-runtime-output-query-cache.mjs \
  /tmp/query-cache-qualification/ovstage.mjs \
  /tmp/query-cache-qualification/ovphysx.wasm /tmp/query-cache-result.json
```

The test compares cached publication against a fresh-query publication without
stepping between them, for rigid bodies and articulation links. It also exercises
refresh, real body addition/deletion/recreation in one attachment, empty and
repopulated selections, rejected output ordinals, signed scale, hierarchy and
owner recreation. The receipt hashes the actual three native artifacts. The
qualification image is separate from the uninstrumented timed candidate.
