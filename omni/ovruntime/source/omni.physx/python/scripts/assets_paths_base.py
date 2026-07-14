# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# this needs to rely on no imports except what's in base kitsdk so it's importable from sync_to_aws.py

OV_PATH = ""
S3_BUCKET = "omniverse-content-staging"
S3_REGION = "us-west-2"
S3_PATH = "DoNotDelete/PhysicsDemoAssets/Versioned"

def get_s3_upload_path():
    return f"s3://{S3_BUCKET}/{S3_PATH}"
