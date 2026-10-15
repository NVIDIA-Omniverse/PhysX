# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-2 AC-3
# @implements REQ-COMP-USD-EDIT-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-IDENTITY-001
# @covers AC-3 AC-4
# @implements REQ-COMP-CAMERA-001
# @covers AC-3

"""Exclusive, thread-affine authoring batches with conservative layer rollback.

The host must serialize all stage/layer writers for the duration of a batch.
Patches capture the whole target layer and refuse undo/redo after unrelated edits.
They never save a layer. This is intended for editor-sized layers, not recording
an unlimited bake in a single layer. Mapped/variant edit targets are not supported.
"""

from contextlib import nullcontext
from dataclasses import dataclass
from threading import get_ident
from uuid import uuid4


class DependencyError(RuntimeError):
    """The customer USD installation lacks a required module/schema."""


class StaleResultError(RuntimeError):
    """The result no longer refers to the current source contents."""


class RecoveryError(RuntimeError):
    """Automatic rollback failed; before_text is the recoverable layer content."""

    def __init__(self, layer, before_text):
        super().__init__(f"Rollback failed for {layer.identifier}; recover from before_text")
        self.layer = layer
        self.before_text = before_text


def _usd():
    try:
        from pxr import Sdf, Tf, Usd, UsdGeom, Vt
    except ImportError as exc:
        raise DependencyError("USD helpers require customer-installed pxr (OpenUSD); no USD is bundled") from exc
    required=((Usd, ('Stage','EditContext','SchemaRegistry','GetVersion')),
              (UsdGeom, ('XformCache','Mesh','GetStageMetersPerUnit')),
              (Tf, ('Notice',)), (Sdf, ('Layer','Path')))
    missing=[module.__name__+'.'+name for module,names in required for name in names if not hasattr(module,name)]
    if missing:
        raise DependencyError('Unsupported customer USD: missing '+', '.join(missing))
    return Sdf, Tf, Usd, UsdGeom, Vt


@dataclass(frozen=True)
class SourceRevision:
    session: str
    revision: int


class EditSession:
    """Bind edits to one stage, creating thread and source revision.

    External USD notices invalidate captured results. Call invalidate() for host
    events without USD notices (selection, cancellation, mapping or runtime owner
    changes). output_guard is a context-manager factory supplied by the host to
    suppress solver-input feedback while output is published, rolled back or undone.
    The default is suitable only when no solver-input observer is installed.
    """

    def __init__(self, stage, *, output_guard=nullcontext):
        _, Tf, Usd, _, _ = _usd()
        if not isinstance(stage, Usd.Stage):
            raise TypeError("stage must be a customer USD stage")
        if not callable(output_guard):
            raise TypeError("output_guard must be a context-manager factory")
        self.stage = stage
        self._thread = get_ident()
        self._id = uuid4().hex
        self._revision = 0
        self._closed = False
        self._editing = False
        self._close_callbacks = {}
        self._output_guard = output_guard
        self._notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._changed, stage)

    def _changed(self, notice, sender):
        if not self._editing:
            self._revision += 1

    def _check(self):
        if get_ident() != self._thread:
            raise RuntimeError("USD edits must execute on the session's approved thread")
        if self._closed:
            raise RuntimeError("USD edit session is closed")
        if self._editing:
            raise RuntimeError("Reentrant USD edit operations are not supported")

    @property
    def source(self):
        self._check()
        return SourceRevision(self._id, self._revision)

    def validate(self, source):
        if source != self.source:
            raise StaleResultError("Result belongs to a stale or foreign source revision")

    def invalidate(self):
        self._check()
        self._revision += 1

    def close(self):
        if self._closed:
            return
        self._check()
        # Resource adapters restore output while the stage and edit session are
        # still usable. A failed restoration leaves close retryable.
        for key, callback in tuple(self._close_callbacks.items()):
            if key in self._close_callbacks:
                callback()
                self._close_callbacks.pop(key, None)
        self._notice.Revoke()
        self._closed = True

    def on_close(self, callback):
        """Register same-thread cleanup before stage release; return its remover."""
        self._check()
        if not callable(callback):
            raise TypeError('callback must be callable')
        key = object()
        self._close_callbacks[key] = callback
        return lambda: self._close_callbacks.pop(key, None)

    def __enter__(self):
        self._check()
        return self

    def __exit__(self, *args):
        self.close()

    def _validate_layer(self, layer):
        self._check()
        Sdf, _, _, _, _ = _usd()
        if not isinstance(layer, Sdf.Layer) or layer not in self.stage.GetLayerStack():
            raise ValueError("target must be an explicit local layer in the stage's layer stack")
        if not layer.permissionToEdit:
            raise ValueError("target layer is not writable")

    def _apply(self, layer, edit, *, output=False):
        """Internal batch: callers must preflight all operations before entry."""
        self._validate_layer(layer)
        _, _, Usd, _, _ = _usd()
        before = layer.ExportToString()
        after = None
        self._editing = True
        try:
            with self._output_guard() if output else nullcontext():
                try:
                    with Usd.EditContext(self.stage, self.stage.GetEditTargetForLocalLayer(layer)):
                        edit()
                    after = layer.ExportToString()
                except BaseException:
                    try:
                        if not layer.ImportFromString(before):
                            raise RuntimeError("USD rejected rollback")
                    except BaseException as exc:
                        raise RecoveryError(layer, before) from exc
                    raise
            if after is None:
                raise RuntimeError("Host output guard suppressed a failed USD edit")
        except BaseException as exc:
            self._revision += 1
            # A host guard can itself fail on entry/exit, outside the edit's
            # rollback scope. Never leave that case looking like a clean failure.
            if layer.ExportToString() != before and not isinstance(exc, RecoveryError):
                raise RecoveryError(layer, before) from exc
            raise
        finally:
            self._editing = False
            if not output:
                self._revision += 1
        return EditPatch(self, layer, before, after, output)


class EditPatch:
    """An applied batch. Host commands can call undo()/redo() on the same thread.

    Conflicting edits fail rather than overwrite unrelated opinions. Keep patches
    only as long as needed: they retain the session and two layer snapshots.
    """

    def __init__(self, session, layer, before, after, output):
        self._session, self._layer = session, layer
        self._before, self._after = before, after
        self._output, self._applied = output, True

    def _restore(self, *, applied):
        self._session._validate_layer(self._layer)
        if self._applied == applied:
            raise RuntimeError("Patch is already in the requested state")
        expected = self._after if self._applied else self._before
        if self._layer.ExportToString() != expected:
            raise StaleResultError("Target layer changed; undo/redo would overwrite unrelated edits")
        replacement = self._after if applied else self._before

        def edit():
            if not self._layer.ImportFromString(replacement):
                raise RuntimeError("USD rejected patch restoration")

        self._session._apply(self._layer, edit, output=self._output)
        self._session.invalidate()  # Undo/redo invalidates pending results, including output frames.
        self._applied = applied

    def undo(self):
        self._restore(applied=False)

    def redo(self):
        self._restore(applied=True)
