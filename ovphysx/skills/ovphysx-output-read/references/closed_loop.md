# Closed-Loop ovstage Write-Back

Read this file only when physics output is written back into the attached
ovstage Stage.

## Ordinal Ownership, Not Magic Parity

Application controls flow from ovstage into physics. Physics output flows back
to ovstage through application-owned write-back. Physics must never drain its
own output, so the application assigns disjoint ordinal lanes to the two
directions.

Ordinals have no built-in odd/even or producer semantics. Even/odd is only a
simple two-lane allocation:

- even ordinals: application controls
- odd ordinals: physics output

An update range is closed and does not skip intervening ordinals. For example,
`[2, 4]` includes output ordinal 3; use control-only ranges such as `[2, 2]` and
`[4, 4]` instead. Keep every ordinal single-purpose:

- Author every change intended to affect physics at a sealed control ordinal
  that `update_from_ovstage()` / `ovphysx_update_from_ovstage()` will cover.
- Do not mix a physics input into an output ordinal. Excluding that ordinal
  would hide the input; including it would also re-ingest physics output.
- Output ordinals are excluded only from ovphysx input ranges. They remain
  available to downstream readers and other participants.
- With more participants, assign explicit lanes per producer or purpose and
  define which lanes each consumer drains. Do not extend the odd/even convention
  as if parity were part of the ovstage protocol.

For each frame:

1. Author control attributes at the next control ordinal.
2. Advance the ovstage write floor to seal that ordinal.
3. Pass only a sealed, control-only ordinal range to
   `update_from_ovstage()` / `ovphysx_update_from_ovstage()`.
4. Complete the simulation step.
5. Read physics output and write it at the next output ordinal.
6. Seal the output ordinal for downstream readers.
7. Advance both lanes without ever including an output ordinal in a later
   `update_from_ovstage()` range.

## Preserve ovstage Identity

Use the group's interned prim list rather than rebuilding paths. Use the emitted
attribute token rather than assuming it equals the requested semantic name.
Forward `semantic` and `is_array`; tensor count and shape do not determine
whether the attribute is an array.

In C, the group's DLPack tensors are borrowed and can feed the ovstage write
payload without repacking. Wait for the write to finish before releasing the
group. In Python, the numeric tensors are NumPy copies, while `prim_list`,
`attribute`, and the shared dictionary remain context-bound.

Point-instancer `position` and `orientation` requests emit `positions` and
`orientations` in instancer-local space.

## Validation

Verify that:

- every control ordinal is sealed before physics drains it;
- every drain range contains control ordinals exclusively;
- no ordinal mixes changes intended for ovphysx with physics write-back;
- every output write completes before its borrowed C group is released;
- downstream readers see the sealed output ordinal;
- structural layout changes rebuild any cached `ALL`-scope setup.
