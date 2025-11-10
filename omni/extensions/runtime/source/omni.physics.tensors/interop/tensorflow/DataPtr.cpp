// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include <tensorflow/core/framework/op.h>
#include <tensorflow/core/framework/op_kernel.h>
#include <tensorflow/core/framework/shape_inference.h>

using namespace tensorflow;

REGISTER_OP("DataPtr")
    .Attr("T: {float, double, int8, int16, int32, uint64, uint8, uint16, uint32, uint64} = DT_FLOAT")
    .Input("input: T") // tensor
    .Output("output: uint64") // scalar
    .SetShapeFn([](::tensorflow::shape_inference::InferenceContext* c) {
        c->set_output(0, {});
        return Status::OK();
    });

template <typename T>
class DataPtrOp : public OpKernel
{
public:
    explicit DataPtrOp(OpKernelConstruction* context) : OpKernel(context)
    {
    }

    void Compute(OpKernelContext* context) override
    {
        // grab the input tensor
        const Tensor& inputTensor = context->input(0);

        // get data pointer
        const T* ptr = reinterpret_cast<const T*>(inputTensor.tensor_data().data());
        uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);

        // allocate a scalar output tensor on CPU
        Tensor* outputTensor = nullptr;
        AllocatorAttributes allocAttribs;
        allocAttribs.set_on_host(true);
        OP_REQUIRES_OK(context, context->allocate_output(0, {}, &outputTensor, allocAttribs));

        // write input address
        auto outputFlat = outputTensor->flat<uint64>();
        outputFlat(0) = (uint64)addr;
    }
};

// CPU
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<float>("T"), DataPtrOp<float>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<double>("T"), DataPtrOp<double>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<int8>("T"), DataPtrOp<int8>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<int16>("T"), DataPtrOp<int16>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<int32>("T"), DataPtrOp<int32>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<int64>("T"), DataPtrOp<int64>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<uint8>("T"), DataPtrOp<uint8>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<uint16>("T"), DataPtrOp<uint16>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<uint32>("T"), DataPtrOp<uint32>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_CPU).TypeConstraint<uint64>("T"), DataPtrOp<uint64>);

// GPU
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<float>("T"),
                        DataPtrOp<float>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<double>("T"),
                        DataPtrOp<double>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<int8>("T"),
                        DataPtrOp<int8>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<int16>("T"),
                        DataPtrOp<int16>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<int32>("T"),
                        DataPtrOp<int32>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<int64>("T"),
                        DataPtrOp<int64>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<uint8>("T"),
                        DataPtrOp<uint8>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<uint16>("T"),
                        DataPtrOp<uint16>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<uint32>("T"),
                        DataPtrOp<uint32>);
REGISTER_KERNEL_BUILDER(Name("DataPtr").Device(DEVICE_GPU).HostMemory("output").TypeConstraint<uint64>("T"),
                        DataPtrOp<uint64>);
