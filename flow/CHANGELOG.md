# Changelog

## [2.1.0] - 2024-05-30

* Fixes in point emitter:
  * Reduce flickering and artifacts when restreaming the same points.
  * Smoother level transitions.
* Force clear on rescale parameter. If cell size is changed, clear instead of migrating.
* Flow LOD layer support.
* UsdVol support. Vec4 NanoVDB support. Reduce NanoVDB emitter VRAM usage.
* Support for aliasing resources.
* Flow shaders are compiled by the Slang compiler. DXC replaced FXC.

## [2.0.2] - 2022-10-31

* Initial Flow 2.0.2 SDK Docs creation.
