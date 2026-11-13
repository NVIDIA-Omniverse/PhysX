# PhysX SDK OVD skills

These product skills work with installed SDK tools on PATH or in the install's
`bin` directory. The source SDK supplies the inspector in `pvddom` and the finite
real-physics example in `pvddom/workflow`. No viewer installation is required.
CMake installation places this index and the skills in `share/physx`.

For a source build, see `physx/pvddom/README.md` in the repository.
The optional, separately distributed `physx-ovd` wheel also includes the inspector
and these skills. `python -m physx_ovd --version` checks its CLI;
`physx_ovd.skills_path()` locates the index without configuring agents.
Its bundled README describes installation, supported platforms and provenance.

In a debugging conversation, use `ovd-inspect` to open an existing finalized
capture. Use `ovd-record` when a new capture is requested and the recorder is
installed; compare two captures only after object matching and frame alignment
are established. Run the helpers, then explain the observed values, provenance,
limits, and a useful next diagnostic to the human in the same conversation.
The tools' stdout JSON is evidence for that explanation. Export a sidecar with
`--annotations` only when the human asks for a file. A sidecar is not a supported
viewer-import format, and no viewer is needed for this workflow.

A standalone `pvddom` install may provide only `ovd-inspect`. The full checked,
profile, or debug SDK workflow install provides both tools; Release disables
OmniPVD recording. Recording in an existing application requires OmniPVD to be
attached when its `PxPhysics` is created; see `ovd-record` for late sampling and
file finalization. The inspector accepts supported flat SDK/OvPhysX captures.

Example user request: "Use the installed PhysX OVD skills to record two fresh
CPU captures, one with the sample velocity change at step 3. Inspect the moving
body, compare aligned post-step velocities, and explain the first difference
with values and source frames here in our conversation. Keep both captures."

## ovd-record

[Instructions](skills/ovd-record/SKILL.md): finite CPU/GPU capture, late attach,
scene/object and primary/secondary selection, TCP discovery/control, and explicit
file finalization before teardown. Native API workflows include particle,
deformable and DirectGPU scope; the installed CLI remains a finite FILE example.

**Evaluation**: [dataset](skills/ovd-record/evals/evals.json) covers the installed
sample, late two-step capture, GPU skips, owner/dependency selection, contacts,
TCP catalogs and acknowledgments, legacy peers, synchronized callback lifetimes,
and routing to comparison.

## ovd-inspect

[Instructions](skills/ovd-inspect/SKILL.md): open finalized files, discover bounded
scene/frame/object data, and inspect typed values with provenance.

**Evaluation**: [dataset](skills/ovd-inspect/evals/evals.json) covers pose discovery,
carried evidence, pagination, untrusted names, and routing to recording.

## ovd-compare

[Instructions](skills/ovd-compare/SKILL.md): explicit matching and alignment,
first linear-velocity divergence, and optional evidence export.

**Evaluation**: [dataset](skills/ovd-compare/evals/evals.json) covers independent
captures, unknown alignment, missing observations, and routing to inspection.
