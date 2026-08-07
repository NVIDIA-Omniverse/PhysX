---
name: ovphysx-output-read
description: Read ovphysx simulation output with the ovstage-native output-read API when results must retain ovstage identity or write back without rebuilding paths or repacking tensors. Covers Python `PhysX.read()` / `PhysX.read_tokens()`, C `ovphysx_query()` / `ovphysx_read()`, emitted attribute tokens, borrowed groups, and closed-loop physics-to-ovstage write-back.
compatibility: "Requires ovphysx >=0.5.2 and an attached ovstage Stage; supports the Python wheel and C SDK."
metadata:
  author: "NVIDIA Omniverse Physics Team"
  version: "0.1.0"
  tags: "ovphysx, ovstage, physics-output, query-read"
---

# Read ovphysx Simulation Output

Use the output-read API when results must retain ovstage prim identity, emitted
attribute tokens, and fixed/array column layout. TensorBindingsAPI is the route
for caller-owned bulk buffers; raycast, sweep, and overlap are geometry queries.

## Workflow

1. Select the language branch and read its bundled reference:

   - Python: [Python output reads](references/python.md)
   - C: [C query/read lifecycle](references/c.md)

   Read only the selected language reference. Select the simulated object type,
   semantic attributes, `ALL` or `ACTIVE` scope, and destination. When selecting
   `ACTIVE` or caching layout state, also read
   [Scope and layout constraints](references/scope_and_layout.md); support is
   type-dependent and the current layout metadata is not an invalidation signal.

   **Complete when:** the language, type, attributes, scope, and destination are
   explicit.

2. Keep an ovstage Stage attached and alive. When attachment and process
   lifecycle are not already established, invoke the `basic-workflow` skill by
   name.

   **Complete when:** the attached Stage owns the shared path dictionary for the
   whole read.

3. Complete the simulation work whose output will be read. Use `step_sync()` /
   `ovphysx_step_sync()`, or wait for the asynchronous step before opening the
   read.

   **Complete when:** the intended step has completed.

4. Open and drain the read exactly as the selected language reference describes.
   A zero-object match is successful and yields no groups.

   **Complete when:** every returned group is consumed and normal exhaustion is
   distinguished from failure.

5. Consume each group according to its destination.

   - For local Python inspection, retain the copied NumPy values.
   - For native C inspection, copy values that must outlive the native group.
   - For ovstage write-back, first read
     [Closed-loop ovstage write-back](references/closed_loop.md), then complete
     the write while the group's identity and borrowed data remain valid.

   **Complete when:** retained values are caller-owned, or the ovstage write has
   completed before releasing its source group.

6. Release resources and validate the selected branch. Python uses the
   `ReadResult` context manager. C releases every fetched group, then the read
   and query handles on success and error paths. Exercise an integration test
   that checks values and, for write-back, ordinal separation.

   **Complete when:** no read resources remain live, supported `ACTIVE` reads are
   reopened per frame, structural changes trigger fresh query and downstream
   layout setup, and the test observes the expected output.

## Shared Contracts

- Query by simulated type, not USD schema or prim-path pattern.
- Request semantic names such as `position`, `orientation`,
  `linearVelocity`, `angularVelocity`, `jointPosition`, `jointVelocity`,
  `points`, or `velocities`.
- Treat a query as a lazy selector. A read observes the most recently completed
  step, not a snapshot captured when the query was opened.
- Before using `ACTIVE` or caching prim order and shapes, apply the bundled
  [scope and layout constraints](references/scope_and_layout.md). Do not assume
  every object type filters `ACTIVE`, discovery counts always match an `ACTIVE`
  read, or `layout_generation` changes after structural edits.

## Installed API Sources

For the caller's installed ovphysx version, prefer:

- C SDK headers: `include/ovphysx/ovphysx.h` and
  `include/ovphysx/ovphysx_types.h`
- Python docstrings: `ovphysx.api.PhysX`, `ReadResult`, and `ReadGroup`

The public release documentation starts at
<https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>.
