# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Session read/write helpers for tests: one attribute of one object type, over the whole set or a prim-set
selection, keyed by prim path. A fixed group contributes each row's components; an array group (one tensor per
prim, e.g. per-axis joint columns) contributes each prim's tensor. Values travel as flat float64 numpy rows
whatever the column's dtype or device."""

import numpy as np
import ovstage
import warp as wp


def handles_for(stage, paths):
    """Interned prim-path handles for `paths`, the form a `prim_paths` selection takes."""
    with ovstage.PathDictionary(stage) as dictionary:
        return [dictionary.intern_path(path) for path in paths]


def _group_paths(dictionary, group):
    handles = dictionary.get_paths(group.prim_list)
    return [dictionary.path_to_string(handles[group.prim_offset + row]) for row in range(group.prim_count)]


def read_column(physx, stage, object_type, attribute, *, pattern=None, prim_paths=None):
    """{prim path -> flat float64 row} for `attribute` of every `object_type` object, or of those selected."""
    out = {}
    with physx.read_sync(object_type, [attribute], prim_paths=prim_paths, pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                index_map = group.index_map.numpy() if group.index_map is not None else None
                rows = None if group.is_array else np.asarray(group.tensors[0].numpy()).reshape(
                    group.tensors[0].shape[0], -1
                )
                for row, path in enumerate(_group_paths(dictionary, group)):
                    data_row = int(index_map[row]) if index_map is not None else row
                    if group.is_array:
                        values = np.asarray(group.tensors[data_row].numpy())
                    else:
                        values = rows[data_row]
                    out[path] = np.asarray(values, dtype=np.float64).reshape(-1).copy()
    return out


def write_column(physx, stage, object_type, attribute, fill, *, pattern=None, prim_paths=None):
    """Write `attribute` of every `object_type` object, or of those selected, committing every group.

    `fill(path, row)` returns the prim's new values; `row` is the prim's slice of the write buffer as a flat
    float64 array of its component count. It does not hold the column's current value (read first to write
    relative to it), so return every component. Returns the number of prims written.
    """
    written = 0
    with physx.write_sync(object_type, attribute, prim_paths=prim_paths, pattern=pattern) as session:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in session.groups:
                paths = _group_paths(dictionary, group)
                # One tensor per prim (array transport) versus one stacked tensor; a single prim reads the
                # same either way.
                per_prim = len(group.tensors) > 1 or (len(group.tensors) == 1 and group.prim_count == 1)
                if per_prim:
                    for row, path in enumerate(paths):
                        tensor = group.tensors[row]
                        current = np.asarray(tensor.numpy())
                        new = np.asarray(fill(path, current.astype(np.float64).reshape(-1).copy()))
                        tensor.assign(new.reshape(current.shape).astype(current.dtype))
                elif group.tensors:
                    tensor = group.tensors[0]
                    current = np.asarray(tensor.numpy())
                    rows = current.reshape(current.shape[0], -1).astype(np.float64)
                    for row, path in enumerate(paths):
                        rows[row] = np.asarray(fill(path, rows[row].copy()), dtype=np.float64).reshape(-1)
                    tensor.assign(rows.reshape(current.shape).astype(current.dtype))
                cuda_tensor = next((t for t in group.tensors if t.size and t.device.is_cuda), None)
                if cuda_tensor is not None:
                    session.commit_sync(group, cuda_stream=int(wp.get_stream(cuda_tensor.device).cuda_stream or 1))
                else:
                    session.commit_sync(group)
                written += group.prim_count
    return written


def write_uniform(physx, stage, object_type, attribute, values, *, pattern=None, prim_paths=None):
    """Write the same `values` to every selected prim, truncated or zero-padded to its component count."""
    values = np.asarray(values, dtype=np.float64).reshape(-1)

    def fill(_path, row):
        row[:] = 0.0
        count = min(row.size, values.size)
        row[:count] = values[:count]
        return row

    return write_column(physx, stage, object_type, attribute, fill, pattern=pattern, prim_paths=prim_paths)


def write_is_refused(physx, object_type, attribute, *, pattern=None, prim_paths=None):
    """Whether opening a write of `attribute` over the `object_type` objects is refused."""
    try:
        with physx.write_sync(object_type, attribute, prim_paths=prim_paths, pattern=pattern):
            return False
    except RuntimeError:
        return True
