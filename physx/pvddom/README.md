# Bounded OVD file inspection

`PvdDomInspect::Document` owns an SDK DOM, recorded-frame metadata and stream messages.
It uses PVDRuntime's decoder and PvdDom's command handlers. It can inspect recordings without running a viewer.
Configure `PVDDOM_BUILD_INSPECTION=ON` to build it (default OFF). This enables
C++ exceptions for PvdDom; the default core build remains exception-free.

## Installed tools and CMake consumers

The standalone build needs C++14 and CMake 3.15. GoogleTest tests fetch their
dependency and run as one aggregate before CMake 3.18, with individual discovery
from 3.18. Installed-tool tests require CMake 3.21 when tests and tools are enabled;
tools and libraries can be built with tests disabled on CMake 3.15.
From an SDK source distribution:

```sh
cmake -S physx/pvddom -B inspect-build -DPVDRUNTIME_ROOT="$PWD/physx/pvdruntime" -DPVDDOM_BUILD_INSPECTION=ON -DPVDDOM_BUILD_TOOLS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build inspect-build --config Release
cmake --install inspect-build --config Release --prefix "$PWD/ovd-install"
```

Use `ovd-install/bin/ovd-inspect --help` or put that `bin` on PATH. `--version` reports
`sdkVersion`, `build`, numeric `evidenceVersion` (1) and `supportedEvidenceVersions`
(an array of numbers, [1,2]).
It does not claim to know the producer SDK version of an input file. Capture SDK/hash
fields remain unspecified when not independently known. JSON strings preserve valid
UTF-8; native Windows file paths follow the active ANSI code page. POSIX paths use UTF-8.

An external CMake project sets `CMAKE_PREFIX_PATH` to the install prefix, then uses
`find_package(PvdDom CONFIG REQUIRED)` and `target_link_libraries(app PRIVATE PhysX::PvdDom)`.
The export carries PvdDom, static PVDRuntime and public headers. Its standalone runtime
target is `PvdDom::PVDRuntime`, separate from the SDK's `PhysX::PVDRuntime`, so both
packages can be loaded in either order. On Windows, use the
same CRT as the library build (`NV_USE_STATIC_WINCRT` selects static CRT).

`summary FILE` reports contexts, segments, bounds, paginated frame markers and load
counts for ignored native no-ops, unresolved optional edges, handler steps and
membership visits. `unresolvedOptionalEdges` counts remaining optional edge
commands, including repeated references to one missing target; it is not a count
of corrupt data or distinct target handles. Add
`--segment N --context N --frame N` for paginated scene-scoped object names/IDs.
Frame and object lists default to 100 each. `--offset`/`--limit` set their shared
defaults; `--frame-offset`/`--frame-limit` and `--object-offset`/`--object-limit`
override them independently. `framePage` and `objectPage` report offset, count,
hasMore and nextOffset; frames also report total. Each object has an independently
paged `attributes` schema list controlled by `--attribute-offset` and
`--attribute-limit`. It includes inherited and custom names, declaring classes,
numeric `dataType`, its `dataTypeName` (such as `FLOAT32` or `OBJECT_HANDLE`),
`fieldCount` and boolean `hasRecordedWrite`. For `FLAGS_WORD` and `ENUM_VALUE`,
`fieldCount` is not a numeric component count. A registered attribute is not evidence of a value at
the selected frame; query it with `inspect`.
For large-frame name/ID discovery add `--attribute-limit 1`. Keep segment/context/frame
fixed, then pin the chosen object with `--object-offset N --object-limit 1`, where
N is `objectPage.offset` plus the zero-based index in that page's `objects` array,
not its `objectId`. Verify the returned `objectId` matches before paging its
schema with `--attribute-offset`/`--attribute-limit`.
`inspect FILE` requires those frame flags, exactly one `--name`/`--id`, and `--attribute`
for a finite scalar float, signed/unsigned integer, recorded flag mask, vec3 or SDK
transform. Optional `--class` restricts the exact class.
Use reported context IDs rather than assuming a context is the application step number.

Numeric scalars require a one-field `FLOAT32`/`FLOAT64` or `INT8`/`INT16`/`INT32`/`INT64`
or unsigned equivalent declaration. `FLAGS_WORD` values are raw unsigned masks from
one-, two- or four-byte storage; unknown bits are preserved without enum-label or
boolean conversion. Scalars and vec3 use `value`; transforms retain top-level
`position` and `quaternion` arrays. Missing samples use `value:null`, distinct from
a recorded zero. Arbitrary arrays, strings, references and lists remain unsupported.
Variable-count declarations retain legacy float vec3/transform support, but do not
become scalar values merely because one element was written.

`compare LEFT RIGHT` requires independent `--left-name`/`--left-id` and
`--right-name`/`--right-id`, both `--*-segment`/`--*-context` values, inclusive
`--start`/`--end`, signed `--offset`, and finite nonnegative `--tolerance`.
Right frame = left frame + offset. Optional `--stride` is positive, `--units` labels
without rescaling, and `--allow-carried` permits carried prior known state, never missing values.
Without metric flags it compares linear velocity with version 1 evidence.
Paired `--attribute NAME --distance vec3|position|orientation` selects a finite
vec3, SDK transform position or quaternion orientation distance and emits version 2
evidence with nested `measure {attributeName, distanceKind, units}` and raw
`leftValue`/`rightValue` arrays (quaternions in xyzw order). Arrays retain both
decoded values even when measurement is invalid; null means unavailable values.
`hasEvidence` and non-null `distance` indicate valid measurement.
`leftFrameCompleted`/`rightFrameCompleted` are false for an open START interval,
true for a completed frame, and null for an absent frame. Report open frames
alongside Equal; equality of sampled values does not prove simulation completion.
Vec3 and position require an explicit `--units` label. Orientation uses
radians and rejects other units. Neither mode converts units.
Stdout is JSON, diagnostics use stderr. Exit 0 means success/Equal, 1 Different,
2 load rejection/Incomparable/usage/I/O failure. A rejected load emits one
`physx.ovd.error` version 1 JSON document with code, message, byte offset,
one-based command index, input role (`input`, `left`, or `right`) and `limitOption`.
IDs, counters, offsets, command indices, field counts and page integers are decimal
strings; `version`, `dataType`, `evidenceVersion`, entries of `supportedEvidenceVersions`
and version 2 `componentCount` are numbers. Scalar attribute values are also numbers.
Integer attribute values use exact decimal JSON tokens, including signed and unsigned
64-bit boundaries, without conversion through floating point. Consumers that parse
all numbers as binary64 can round integers beyond 2^53; use an integer-preserving
JSON parser when exact values are required. Scalar floats retain a decimal point or
exponent, including `1.0` and `-0.0`. Metadata integer encodings are unchanged.
`componentCount` is 3 for vec3/position, 4 for orientation, or 0 when nothing decoded. Boolean `textWasReplaced` reports
replacement of invalid UTF-8. Comparison evidence has format
`physx.ovd.inspection-evidence`. Every Incomparable result, including selection/query
failures and valid queries lacking exact samples, emits this JSON with `status: Incomparable`,
an explanatory `message`, plus stderr, and exits 2. Command-line parsing errors and summary/inspect selection/query
failures emit stderr only. Output failures return 2 with diagnostics and may follow valid or partial
stdout; no extra error JSON is appended. Failed stdout also returns 2;
the recorder keeps a finalized capture if only its final JSON output fails.
An annotation I/O failure can return 2
after valid comparison JSON was delivered; inspect its status, not only the exit code.
No second JSON document is appended after an annotation failure.
`--annotations NEW_PATH` exclusively creates a sidecar
and refuses existing files, including source aliases. The sidecar is not a viewer import.
On Windows, alternate-stream and device destinations are refused.
Loader `--max-*` overrides are positive integers; run `--help` for the complete list.
Retry a trusted capture of justified size only when `code` is `Limit` and
`limitOption` names a known `--max-*` option: increase that option alone by a
justified bounded amount. Null or unknown `limitOption` and other codes stop
budget retries. English messages do not identify adjustable limits. Both
`--max-attribute-bytes` and `--max-retained-bytes` remain active; increasing the
identified payload cap may expose the other cap on a later retry. Each later
limit needs its own justification. Allocation and fixed limits have null identity.

`summary` and `inspect` accept `--recover-tail` for a stopped file cut during its
last command. A successful recovered load reports `loadInfo.recoveredTail`, input
`sourceBytes`, and the excluded command's `terminalError` in JSON and warns on
stderr. Inspection exit 0 refers to the retained validated prefix; it does not
mean the capture finished. Open frames can remain available. Corrupt complete
commands, unsupported input and limit failures remain rejected. `compare` always
loads strictly and does not accept this flag.

## Real SDK capture workflow

The optional example builds the actual CPU PhysX SDK, records six finite steps and
checks twelve pre/post pose/velocity states against live SDK values before teardown:

```sh
cmake -S physx/pvddom/workflow -B capture-build -DCMAKE_BUILD_TYPE=checked
cmake --build capture-build --config checked --target ovd_record ovd_inspect
ctest --test-dir capture-build -C checked --output-on-failure
cmake --install capture-build --config checked --prefix "$PWD/ovd-install"
ovd-install/bin/ovd-record "new capture.ovd" --late-steps 20
ovd-install/bin/ovd-inspect summary "new capture.ovd"
```

Use checked/profile/debug, because Release disables SDK recording. The example uses
static SDK libraries by default and no rendering. `--change-step 3` adds one X-velocity
unit before the third captured step. The workflow tests create two independent equal
captures and a changed capture, then prove the first post-step divergence is frame 6.
It installs the tools and executes them from outside the source checkout. The source
recordings remain intact, including when capture/export paths collide.
On Windows, `-DPX_GENERATE_STATIC_LIBRARIES=OFF` places the SDK runtime DLLs
beside the recorder and its workflow test executables; the installed recorder
includes the same DLLs. No SDK DLL directory needs to be added to `PATH`.
On Linux, the same `-DPX_GENERATE_STATIC_LIBRARIES=OFF` workflow installs the
recorder with a search path matching `CMAKE_INSTALL_LIBDIR`. Relative library
directories such as `lib`, `lib64` and `lib/x86_64-linux-gnu` support moving the
whole install prefix without setting `LD_LIBRARY_PATH`. An absolute library
directory stays fixed and does not support that relocation. CI exercises both
static and shared Linux workflows with `lib64`.

`--gpu` requests GPU dynamics in a build configured with `PX_GENERATE_GPU_PROJECTS=ON`
and the normal SDK CUDA prerequisites. Exit 77 reports unavailable GPU support. Such
a skip does not replace CPU evidence. GPU installations also need the SDK's matching
PhysXGpu runtime and CUDA dependencies. The optional GPU CTest reports an explicit skip.

Sampling starts after any requested late steps. `stopSampling()` quiesces writes and
the owned file is explicitly closed before scene teardown. The writer's borrowed
stream stays alive through recorder destruction. Post-step frames 2,4,...,12 are a
verified property of this sample, not a universal frame-to-application-step mapping.
The installed [skill index](../SKILLS.md) covers recording, opening, inspection and
comparison. CMake installs it with all skill/eval files under `share/physx`.

## Library API

```cpp
#include "PvdDomDocument.h"

int main()
{
    PvdDomInspect::Document document;
    PvdDomInspect::Error error;
    if (!document.load("capture.ovd", PvdDomInspect::LoadLimits(), error))
        return 1;
    const OmniPvdDOMState* dom = document.dom();
    // Read the DOM and document.frames() while document remains alive.
    return dom ? 0 : 1;
}
```

A load replaces the previous document. Failure leaves `dom()` null, empty frames and
messages, and zero statistics. `Error` distinguishes I/O, invalid input, unsupported
features and resource limits. Offsets identify the start of the rejected command,
with a one-based command index. Header and file-open failures have index zero.
`Error::limitKind` identifies the adjustable `LoadLimits` field that rejected a load,
without interpreting diagnostic text. `None` identifies allocation failures and
fixed representation or reader path-depth limits. A payload is bounded by the
smaller of `maxAttributeBytes`, `maxRetainedBytes` and the reader's 32-bit size limit.
When both user caps tie, `AttributeBytes` is reported first; raising it can expose
`RetainedBytes` on the next attempt. Increasing one cap does not waive the other.
Successful loads and non-limit errors clear the identity.
If a declared payload exceeds the remaining file extent, bounded reads verify
the physical remainder before reporting `Invalid` with `LimitKind::None`.
Complete payloads above a user cap remain adjustable limits. Allocation and
path-depth failures remain limits and cannot be recovered.

## Reader diagnostics

File loaders are quiet by default. To receive decoder errors, pass an
`OmniPvdReaderDiagnostics` with a callback. Set `verbose = true` for accepted-header
detail and completed-command traces as well:

```cpp
OmniPvdReaderDiagnostics diagnostics;
diagnostics.logFunction = logFunc; // void OMNI_PVD_CALL logFunc(char* text)
diagnostics.verbose = true;
document.load("capture.ovd", limits, error, diagnostics); // Strict
document.load("capture.ovd", limits, error,
              PvdDomInspect::LoadMode::RecoverTruncatedTail, diagnostics);
buildPvdDomStateFromFile("capture.ovd", domState, diagnostics); // Legacy loader
```

The overloads without a diagnostics argument remain quiet:
`buildPvdDomStateFromFile(path, domState)`, `document.load(path, limits, error)`
and `document.load(path, limits, error, mode)`. Each load configures a new reader.
Neither the options nor callback are retained by the document or a subsequent load.
These reader diagnostics are separate from `pvdDomSetLogFunction`, `Error`, recorded
messages and `LoadInfo`. Tracing does not change decode, DOM or recovery outcomes.
A recovered truncated tail can report a reader error even when `load` returns true.
Use the result and `loadInfo().terminalError` to interpret that case. A complete
decoded-command trace does not imply that DOM validation accepted the command.

Callbacks run synchronously with a borrowed, read-only buffer despite the legacy
`char*` signature. Copy text before returning if it must outlive the call. Do not
throw, re-enter or destroy the active reader/document. Serialize loads and legacy
parser use. Synchronize any logging state shared by callbacks when separate
documents load concurrently.
A null callback disables all reader output, including errors. Tracing is skipped
unless both verbosity and a callback are enabled. It does not change writer logging.

Factory-created readers configured only with `OmniPvdReader::setLogFunction()`
report errors without successful-read detail. Direct reader callers can enable
that detail with `setOmniPvdReaderDiagnostics(*reader, diagnostics)`, using factories
and helper from the same module. The helper does not accept custom readers or
readers from another statically linked runtime copy. It replaces both settings
without resetting parsing state or replaying diagnostics. `setLogFunction()` alone
replaces only the callback and preserves verbosity.

### Diagnostic text format

The following describes the current diagnostic formatting. Use structured reader
status, `Error` and `LoadInfo` for application decisions instead of parsing this text.

A command produces a summary and, for each name/message/file field, ordered text
chunks. Each chunk identifies the command, field, one-based chunk index/count,
raw-byte offset/count, encoded length, retained length and truncation flag. The
reader retains at most 2,047 bytes per string. Diagnostics split that retained prefix
into chunks of at most 384 raw bytes. Empty text still produces one empty chunk.
Printable ASCII stays literal except quotes and backslashes, which are escaped.
Control bytes, DEL and bytes above ASCII (including UTF-8 bytes) use `\xHH` escapes.
Embedded NUL is preserved as `\x00`. Escapes are never split across chunks. Each
trace callback is one record without a trailing newline. Payload contents are not
dumped. Treat stream text as data when forwarding it to another logger, never as
a format string.

## Truncated-tail recovery

The three-argument load is strict. To inspect an interrupted, immutable recording,
pass `LoadMode::RecoverTruncatedTail` as a fourth argument. This mode can retain the
fully validated command prefix when the last command ends in a physical EOF short
read, including an over-cap declared payload whose physical remainder is readable.
It requires at least one complete command after the header. It does not skip
corrupt commands, ignore limits, resolve missing required references, or repair an
incomplete command. A failed load still exposes no DOM.
`maxStreamBytes` is inclusive: a complete file or recoverable truncated tail can
use a cap equal to its size. Short reads before the measured file end are I/O failures.

`loadInfo()` reports `recoveredTail`, the full file size in `sourceBytes`, and the
rejected command in `terminalError`. On successful recovery, the ordinary output
`Error` is clear; `terminalError` contains its start offset and one-based index.
`stats().commands` and `stats().streamBytes` count only the applied prefix. Ordinary
success has `recoveredTail == false` and a clear `terminalError`; failure resets
all load information. Recovery requires a stopped recording or immutable copy.
The same open file is checked for size changes, but concurrent in-place writes
cannot all be detected. A clean final open frame remains valid and is identified
by `RecordedFrame::completed == false`, independently of tail recovery.

The defaults allow 512 MiB of stream bytes, 2,000,000 commands, 100,000 object creates,
4,096 classes, 65,535 attributes, 64 inheritance levels, 64 MiB per attribute payload,
2,000,000 frame records and 512 MiB of accounted retained data. Counts include destroyed
objects, replacement incarnations and enum aliases. Zero is a zero allowance, not unlimited.
DOM representation limits still apply when a configured limit is larger.
The default command allowance covers about 333 frames of 1,000 active rigid dynamics
when each emits six state SETs per frame. Three creates per single-shape actor reach
the create allowance at about 33,333 actors. Schema and other commands reduce these
figures. Captures beyond any configured allowance are rejected even when the file is
smaller than the stream-byte cap.
The ratio between accounted retained bytes and file size is workload-dependent.
Configure `maxRetainedBytes` independently from `maxStreamBytes`. No single
multiplier converts one limit to the other.

The default 100,000,000 handler-step allowance counts each scene visited by a legacy
pre-1.4 START_FRAME handler and each object visited when an articulation create scans
for floating links. Schema name checks charge each ancestor and registered class;
each reverse ancestry search reserves its full chain length before scanning.
Each extension-joint attachment charges one constant-time scene-branch lookup.
The Joints branch is cached on its scene incarnation, including across constraint toggles.
`LoadStats::handlerSteps` reports that selected work allowance. It does not
measure wall-clock time, process memory or every parser traversal.

Retained-byte accounting is a conservative logical budget, not a hard process RSS limit.
It charges the initial DOM, schema and inheritance maps, each object's expanded schema
slots, samples and payloads, possible synthetic reference nodes and lifespan growth,
frame records, pending-reference metadata and messages before applying the command.
Subtree lifespan charges cover only new spans on the root and descendants reopened by
that command. Unrelated objects and already-open spans do not add a lifespan charge.
Fixed-width reference handlers retain an 8,192-byte allowance for synthetic branches and 2,048 bytes
per handle for reference nodes and their container growth, including single-node lifespan
reopens. These allowances also apply when a handler reuses an existing node.
Charges include container growth allowances and are not refunded during a load. Budget
256 bytes plus twice the payload size per sample, with the extra node allowances only
for fixed-width reference handlers and default-parsed reference attributes. Scene contact
handle arrays pay the base sample charge. Each pending typed SET/ADD costs another 192 bytes.
The decoder's temporary payload buffer is separately capped by the smaller of the payload
and retained limits. Its retained capacity may include bounded growth slack, but never
exceeds that cap; this scratch capacity is not charged to the logical retained-data budget.
It can allocate before discovering that the file ends early. Clamping it to file size would
change that truncated-input error from invalid input to a resource limit, so this loader
keeps the current classification.
Allocator overhead, I/O buffering and legacy-handler temporary traversal storage
are not measured as RSS. An OS process limit is needed for a hard memory ceiling.

The supported input is the flat SDK/OvPhysX schema. Nested class attributes and paths,
unknown commands, incompatible schema/payload widths and invalid parenting edges are
rejected. SDK flags may have one-, two- or four-byte storage because the wire schema omits
that width. Missing typed SET/ADD targets preserve samples and skip linking. A later
CREATE validates the target class without replaying parenting; existing and later
wrong-class targets are rejected. Only actor `inboundJoint` and `articulation` targets
must resolve before EOF. `LoadStats::unresolvedOptionalEdges` counts remaining optional
validation records per edge command, including repeated commands for the same target.
Owner destruction and recording segments retain samples and their validation records;
records retire on a matching CREATE or at EOF. Missing REMOVE targets create no future
class obligation. Scene/joint constraint handles are
opaque correlation identifiers and do not require a streamed constraint object.

SET, ADD and REMOVE commands with an unknown owner or attribute, and DESTROY commands
with an unknown owner, are native DOM no-ops. They are skipped and counted once in
`LoadStats::ignoredCommands`. They still consume command and stream-byte allowances,
and their encoded payloads must be complete and within decoder limits. Known-object
schema mismatches, incompatible target classes and parenting cycles remain errors.

The native reader retains a terminated 2,047-byte prefix of longer strings and consumes
the complete encoded tail. A truncated tail fails. An empty unstarted stream can be
polled again when data arrives. A partial header can be retried after rewinding the
stream with an explicit `startReading` call. Polling after a failed partial header or
active command preserves the failure. Payload allocation failure returns sticky `eLIMIT`,
including in native builds without exceptions; a new reader can retry the rewound stream.
Truncated commands and payloads are rejected.

Frame records preserve the source START_FRAME value, its context and recording segment.
The SDK DOM uses that value as its recorded-frame number. A matching STOP_FRAME marks
the record complete. A final open frame is valid. Header-only files are valid empty
documents. Sample timestamps retain the legacy DOM's behavior, including its timestamp
clamping. `history()` preserves per-write command ordinal, raw context, effective scene
context and frame separately from those legacy sample timestamps. `frameIndex()` and
per-attribute indexes support bounded lookup without rescanning the entire stream.

Callers must serialize mutation of a document against its readers. Different documents
can load concurrently if the shared PvdDom log callback is thread-safe and remains fixed.
Stream messages are document-local and do not append to `gOmniPvdMessages`. Legacy parser
entry points still use that global message vector and require external serialization.

The inspection API is optional. Configure `PVDDOM_BUILD_INSPECTION=ON` to compile
`Document`; this enables C++ exceptions for the complete PvdDom target so allocation
failures unwind its owned state. The default OFF build contains the ordinary PvdDom core,
compiles it without exceptions and does not provide the inspection implementation.

For standalone inspection tests, configure both `PVDDOM_BUILD_INSPECTION=ON` and
`PVDDOM_BUILD_TESTS=ON`, and set `PVDRUNTIME_ROOT` to the SDK's `pvdruntime` directory.
CTest discovers individual GoogleTest cases on CMake 3.18
or newer; CMake 3.15-3.17 runs them as one aggregate test. The tracked
OVD fixture is resolved from the source directory, independent of the test working directory.
The generated and tracked fixtures both use the scene handle as frame context and are
checked through the inspection API for an actor pose at scene frame 1.

## Scene queries and comparison

`PvdDomInspection.h` provides scene-membership-scoped discovery, selection, sample reads,
quaternion distance and typed attribute comparison. This initial surface covers objects
with a recorded `PxScene` membership. It does not infer scene ownership for global meshes,
materials, shared resources, or generic context-only streams. Unsupported scope and
historical object-reference/unique-list queries return an explicit diagnostic. The full
validated DOM and capture statistics remain available through `Document`.

Use `listObjects(document, frame, offset, limit, objects, error)` for paged discovery.
`FrameRef` identifies a recording segment, scene context and actual recorded frame.
`ObjectSelector` requires exactly one capture-local object ID or exact object name, with
an optional exact class-name restriction. Duplicate names are ambiguous. Names are read
at the requested frame, including rename history. External handles are shown as provenance
and are never used to infer identity between files. Identity lifetime boundaries describe
the selected scene residency and are half-open; a missing end is open-ended.

```cpp
#include "PvdDomInspection.h"

int main()
{
    PvdDomInspect::Document document;
    PvdDomInspect::Error loadError;
    if (!document.load("capture.ovd", PvdDomInspect::LoadLimits(), loadError))
        return 1;
    if (document.frames().empty())
        return 1;
    const PvdDomInspect::RecordedFrame& recorded = document.frames().front();
    PvdDomInspect::FrameRef frame;
    frame.recordingSegment = recorded.recordingSegment;
    frame.context = recorded.context;
    frame.recordedFrame = recorded.recordedFrame;
    std::vector<PvdDomInspect::ObjectIdentity> objects;
    std::string error;
    if (!PvdDomInspect::listObjects(document, frame, 0, 100, objects, error))
        return 1;
    // Select an ID or exact name from objects before reading an attribute.
    return 0;
}
```

`readSample` returns `Exact`, `Carried` or `Missing`. Missing is a successful query with
no value. An unknown/ambiguous attribute, absent frame, unsupported scope or inactive
object is a failed query with a diagnostic. Vector and transform readers accept only
finite FLOAT32/FLOAT64 payloads of the exact width. The CLI and Python interface also
decode the scalar and flag values described above. Unsupported types, unsupported
declaration shapes and nonfinite values produce query diagnostics. Malformed fixed
payload lengths and unsupported flag storage widths fail loading as `Invalid` before
inspection; variable-count empty writes remain query failures rather than Missing.
SDK transforms store quaternion xyzw
before position xyz; the synthetic recording generator uses that same layout.
`quaternionAngularDistance` normalizes finite nonzero xyzw quaternions, treats q and -q
as equivalent, and returns radians with stable behavior for small rotations.

Ordinary SDK writes use raw command context 1 even when the actor belongs to a scene.
The loader records scene membership when each command is applied. Queries never recover
historical membership from the final graph or the latest external-handle map. Samples
written before scene membership or before the first scene frame can be carried, but
cannot become Exact merely because their DOM timestamp equals the requested frame.
Moving or rejoining the same incarnation carries its last pre-join state within the same
recording segment, preserving the original source context/frame. Frame age is unavailable
across different scene counters or when no source frame is known. Unknown and missing
source frames appear as null in evidence JSON; an existing write retains its raw context
and command ordinal.

A SET belongs to the most recent START_FRAME interval in its effective scene, until that
scene's next START_FRAME. STOP_FRAME marks completion metadata; it does not freeze values.
An Exact sample can therefore have been written after STOP_FRAME. Recorded frame numbers
are not application simulation steps. Repeated or backward numbers in one segment/context
are unsupported because FrameRef cannot distinguish them. Separate recording segments
may restart their counters and remain independently queryable. Re-creating a scene with
the same context handle in one segment makes both incarnations' frames unsupported;
the loader clears the old clock and queries report that scene context was re-created.
Superseded scene memberships close at the previous valid scene frame before that clock
is cleared. Joints attach only to live scenes in their recording segment, whether the
joint's constraint SET or the scene's constraints ADD comes last.
The bounded loader rejects recorded frame UINT64_MAX because it cannot represent a
closing half-open boundary. UINT64_MAX-1 remains valid; failed loads cannot compare Equal.

Scene/object destruction closes membership at the current recorded frame. A snapshot
followed by teardown in its only START_FRAME interval can therefore have no live objects
at that frame, even though its full command history is retained. Stop recording before
teardown when the final scene state must remain queryable.

`compareLinearVelocity(left, right, options)` requires explicit selectors, scene contexts,
segments, an inclusive left frame range, positive stride, a signed right frame offset,
and finite nonnegative absolute Euclidean tolerance. Right frame equals left frame plus
offset with checked arithmetic. It resolves each selector once at the first aligned
frame and pins those two incarnations across the range, including later renames. It never
skips a missing first frame, picks a nearest frame, or rebinds a destroyed object's name.
Missing, nonfinite, ambiguous or unsupported data returns `Incomparable`, never `Equal`.
`Different` includes the first pair exceeding tolerance; `Equal` retains the last pair.

Comparison requires Exact writes by default. Setting `requireExactSamples=false` explicitly
permits carried state while preserving source frames, states and available ages. For SDK
pre/post recording, a caller seeking post-step output can explicitly choose start 2 and
stride 2 after checking its capture. The library does not infer parity or step alignment.
The default units are `scene_length_units/s`; callers must not assume SI units.

`compareAttribute(left, right, options, measure)` uses the same selection and alignment
rules for a named attribute. `ComparisonMeasure` declares its name, distance kind and
units: `Vec3Euclidean` reads a finite FLOAT32/FLOAT64 three-component vector;
`TransformPositionEuclidean` compares the position xyz of a seven-component transform;
and `TransformOrientationRadians` compares normalized quaternion xyzw orientations,
treating q and -q as equivalent. Orientation units must be `radians`. The result keeps
the selected raw values (three or four components), measure and observed frame-completion
flags. It performs no unit conversion. A recovered-tail Document is always
`Incomparable`, while a fully decoded open START_FRAME interval is comparable when its
requested samples exist; its completion flag is false. An absent frame has no completion
flag. `compareLinearVelocity` remains the three-argument compatibility wrapper.
`hasValues` means both sides decoded as finite values; `hasEvidence` means a finite
distance was measured. Zero quaternions retain raw xyzw values but are `Incomparable`,
with a reason naming the invalid side or both sides and no measured distance.
Typed vector or transform decode failures name the failing side or both sides,
with null arrays in evidence. Default exact mode reports a missing or carried write
as `Missing exact attribute write at aligned recorded frame`; `leftSample.state`
and `rightSample.state` identify the side lacking the exact write. Each aligned pair
resets its distance and value arrays before decoding.

`evidenceToJson` exports format `physx.ovd.inspection-evidence`, schema version 1 for
`compareLinearVelocity`. Its velocity arrays also accept generic `linearVelocity`
`Vec3Euclidean` results with three components; all other measures emit null velocity
arrays. Use `attributeEvidenceToJson` for those measures. Version 1 includes
caller-supplied capture paths/hash/SDK metadata, identity and lifetime provenance, requested
and source frames, raw contexts, states, metric, values, tolerance, alignment assumptions,
severity, color and message. Selected comparison identities remain available even if a
later sample fails; a selector that never resolved has a null identity. All 64-bit integers
use JSON strings. Numbers use the classic
locale and nonfinite numbers become null. Valid UTF-8 is preserved; malformed bytes become
U+FFFD and `textWasReplaced` is true. No input recording is modified. Viewer import is not
implemented, and no application-step metadata is fabricated.

`attributeEvidenceToJson` emits schema version 2 for typed comparisons, including the
measure, component count, raw left/right values and nullable completion flags along with
the version 1 provenance, sample and alignment fields. The existing `evidenceToJson`
continues to emit version 1 and its `leftVelocity`/`rightVelocity` fields.
Version 2 emits both raw arrays when `hasValues` is true, including finite zero
quaternions, and null arrays when either side cannot be decoded. `hasValues` is a
library member, not a JSON field. `hasEvidence` remains false and `distance` null
when no valid distance is available. `measure.distanceKind` identifies the metric.

Returned sample pointers remain valid only while both Documents remain loaded and alive.
Membership, write and frame indexes are charged to `maxRetainedBytes`. Lookup uses indexes
for the selected object/attribute and scene frame scope. Graph mutations refresh only the
affected subtrees. Generic attribute-reference branches are skipped; the scene Joints
aggregation remains traversable. Each affected stream object is refreshed once per command.
Ordinary value writes do not refresh membership. A recording-segment change refreshes only
open memberships to close them. `LoadStats::membershipVisits` counts traversal and refresh
work per load. Reusable traversal scratch is charged before growth, and walks are bounded
by the number of owned DOM nodes.
