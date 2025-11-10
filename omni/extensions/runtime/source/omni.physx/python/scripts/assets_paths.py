# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.physx.bindings._physx as pxb

OV_PATH = ""
S3_BUCKET = "omniverse-content-staging"
S3_REGION = "us-west-2"
S3_PATH = "DoNotDelete/PhysicsDemoAssets/Versioned"

deformable_beta_on = carb.settings.get_settings().get(pxb.SETTING_ENABLE_DEFORMABLE_BETA)

class AssetFolders:
    ANALOG_DIGITAL_CLOCK = "AnalogDigitalClock/1/"
    BLOCKWORLD = "Blockworld/1/"
    CHAIR_STACKING = "ChairStacking/1/"
    CLOTH_DECK_CHAIR = "ClothDeckChair/1/"
    FLUID_ISOSURFACE_GLASS_BOX = "FluidIsosurfaceGlassBox/1/"
    FRANKA_BRICK_STACK = "FrankaBrickStack/1/"
    FRANKA_DEFORMABLE = "FrankaDeformable/1/"
    FRANKA_NUT_BOLT = "FrankaNutBolt/1/"
    HONEY = "Honey/1/"
    LEGO_BUGGY = "LegoBuggy/1/"
    MIXER = "Mixer/1/"
    NUTS_AND_BOLTS = "NutsAndBolts/1/"
    TEDDY_ON_ICE = "TeddyOnIce/2/" if deformable_beta_on else "TeddyOnIce/1/"


def get_server_path(force_ov_path=False, force_s3_path=False):
    if force_ov_path:
        demo_devel_mode = False
    elif force_s3_path:
        demo_devel_mode = True
    else:
        import carb.settings
        demo_devel_mode = carb.settings.get_settings().get("physics/demoDevelopmentMode")

    server_path = OV_PATH if demo_devel_mode else get_s3_web_path()
    asset_path = f"{server_path}/"

    return asset_path


def get_asset_path(server_setting_path: str, resource_folder: str, resource_relative_path: str) -> str:
    """
    Helper to construct an asset path to the test asset location based on setting '/physics/testsAssetsPath'

    Args:
        server_setting_path: Setting path defining a server path.
        resource_folder: String path defining a folder with a version number. Use omni.physx.scripts.assets_paths.AssetFolders for a list of versioned folders.
        resource_relative_path: String path relative to root folder of the demo assets.

    Example:
        Get path to Mixer.usd:
        asset_path = get_tests_asset_path(AssetFolders.MIXER, "Mixer.usd")
    """
    assets_path_setting = carb.settings.get_settings().get(server_setting_path)
    assert assets_path_setting, f"Empty string in setting {server_setting_path}!"
    return assets_path_setting + resource_folder + resource_relative_path


def get_s3_web_path():
    return f"https://{S3_BUCKET}.s3.{S3_REGION}.amazonaws.com/{S3_PATH}"


def get_s3_upload_path():
    return f"s3://{S3_BUCKET}/{S3_PATH}"
