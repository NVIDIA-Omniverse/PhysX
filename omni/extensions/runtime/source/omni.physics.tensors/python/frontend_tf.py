# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""TensorFlow frontend for data-oriented interface"""

import os
import subprocess
import tensorflow as tf

import omni.physics.tensors

from .frontend_base import *


def _build_data_ptr_module():
    print("~!~!~! Building TF DataPtr module")

    src_root = "../../../source/extensions/omni.physics.tensors/interop/tensorflow"
    src_path = os.path.join(src_root, "DataPtr.cpp")
    target_path = "./libtfdataptr.so"

    tf_includes = tf.sysconfig.get_include()
    tf_libs = tf.sysconfig.get_lib()

    # print("TF includes:", tf_includes)
    # print("TF libs:", tf_libs)

    if not os.path.exists(src_path):
        print("*** Failed to build TF DataPtr module: source path not found")
        return

    if not os.path.exists(tf_includes) or not os.path.exists(tf_libs):
        print("*** Failed to build TF DataPtr module: TF headers or libs not found")
        return

    if os.path.exists(target_path):
        # check if we need to rebuild
        target_mtime = os.path.getmtime(target_path)
        if target_mtime > os.path.getmtime(src_path) and target_mtime > os.path.getmtime(tf_includes):
            # no need to rebuild
            return

    tf_cflags = " ".join(tf.sysconfig.get_compile_flags())
    tf_ldflags = " ".join(tf.sysconfig.get_link_flags())

    # print("TF cflags:", tf_cflags)
    # print("TF ldflags:", tf_ldflags)

    cmd = "g++ -std=c++11 -shared %s -o %s -fPIC %s %s -O2" % (src_path, target_path, tf_cflags, tf_ldflags)
    # print(cmd)

    result = subprocess.call(cmd.split(), shell=False)
    if result != 0:
        raise Exception("Failed to build TF DataPtr module")


_build_data_ptr_module()


class FrontendTensorflow(FrontendBase):

    DTYPE_TO_TF = dict(
        [
            (omni.physics.tensors.float32, tf.float32),
            (omni.physics.tensors.float64, tf.float64),
            (omni.physics.tensors.int8, tf.int8),
            (omni.physics.tensors.int16, tf.int16),
            (omni.physics.tensors.int32, tf.int32),
            (omni.physics.tensors.int64, tf.int64),
            (omni.physics.tensors.uint8, tf.uint8),
            (omni.physics.tensors.uint16, tf.uint16),
            (omni.physics.tensors.uint32, tf.uint32),
            (omni.physics.tensors.uint64, tf.uint64),
        ]
    )

    DTYPE_FROM_TF = dict(
        [
            (tf.float32, omni.physics.tensors.float32),
            (tf.float64, omni.physics.tensors.float64),
            (tf.int8, omni.physics.tensors.int8),
            (tf.int16, omni.physics.tensors.int16),
            (tf.int32, omni.physics.tensors.int32),
            (tf.int64, omni.physics.tensors.int64),
            (tf.uint8, omni.physics.tensors.uint8),
            (tf.uint16, omni.physics.tensors.uint16),
            (tf.uint32, omni.physics.tensors.uint32),
            (tf.uint64, omni.physics.tensors.uint64),
        ]
    )

    def __init__(self, device_ordinal=-1):
        super().__init__()

        if device_ordinal != -1:
            raise NotImplementedError("Only CPU device is currently supported in TF frontend")

        self.device_ordinal = device_ordinal
        self.device = "/CPU:0"

        print("~!~!~! Loading TF DataPtr module")
        self.data_ptr_module = tf.load_op_library("./libtfdataptr.so")
        # print(self.data_ptr_module)

        self.data_ptr_op = self.data_ptr_module.data_ptr

    def _get_data_ptr(self, tensor):
        result = self.data_ptr_op(tensor)
        return int(result)

    def _get_device(self, tensor):
        # TODO
        return -1

    def create_tensor(self, shape, dtype, device=None):

        # TODO: allocate on specific device

        tf_dtype = FrontendTensorflow.DTYPE_TO_TF[dtype]

        # We can create a tf.Tensor or tf.Variable here.  Variables can be .assigned to, but tensors can't.
        # Variables seem like a better choice for user convenience, but there's a problem:
        # Once a variable is .assigned to, it's data pointer can change every time it is used.
        # I.e., even after we use the data_ptr op, its address may change... which makes the op useless.
        #
        # So the current strategy is to work with Tensors only.  Users can wrap them in Variables if they want to
        # modify them, but if they want to pass the buffer to the API, they will need to pass tf.identity(var.value()).
        # Using the tf.identity operation on the underlying tensor is a way to "pin" its location in memory.
        # TODO: To be revisited for a cleaner or more efficient solution.
        #
        # TF Tensor vs. Variable:
        # https://stackoverflow.com/questions/40866675/implementation-difference-between-tensorflow-variable-and-tensorflow-tensor
        #
        # Why it's not possible to assign to tf.Tensor:
        # https://github.com/tensorflow/tensorflow/issues/33131
        #
        # This seems related:
        # https://stackoverflow.com/questions/41289882/do-input-output-tensors-to-tensorflows-opkernelcompute-function-change-th

        tensor = tf.zeros(shape, dtype=tf_dtype)
        # tensor = tf.Variable(tf.zeros(shape, dtype=tf_dtype))

        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = dtype
        desc.shape = shape
        desc.data_address = self._get_data_ptr(tensor)
        desc.device = self.device_ordinal
        return tensor, desc

    def get_tensor_desc(self, tensor):
        desc = omni.physics.tensors.TensorDesc()
        desc.dtype = FrontendTensorflow.DTYPE_FROM_TF[tensor.dtype]
        desc.shape = tensor.shape
        if tensor is tf.Variable:
            raise ValueError("Can't use tf.Variable in this operation.  Use tf.identity(var.value()) instead.")
        else:
            desc.data_address = self._get_data_ptr(tensor)
        desc.device = self._get_device(tensor)
        return desc

    def as_contiguous_float32(self, tensor):
        # TODO!
        return tensor

    def as_contiguous_uint32(self, tensor):
        # TODO!
        return tensor
