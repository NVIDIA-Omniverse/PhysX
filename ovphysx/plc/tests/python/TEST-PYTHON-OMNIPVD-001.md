<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-001
maps_to: REQ-PYTHON-OMNIPVD-001
type: integration
---

## Scenario

Python removes startup configuration and explicitly streams to a ready TCP listener.

## Given

- Python public-surface/config-enum parity checks and removed-field cases.
- Base `PhysXConfig`, plain/ABC/dataclass subclasses, mixin/variadic/positional-only/custom `__new__` constructors, direct/inherited metaclass `__call__`, a callable instance, and a wrapped factory; no native instance is needed for these checks.
- A loopback listener bound to an ephemeral port before explicit recording start.

## When

- Import the retired module, inspect public enums, and construct `PhysXConfig` with each removed OVD field.
- Inspect and bind constructor/callable-instance signatures; construct normally, replace, deepcopy, pickle, try an unknown keyword, and try retired keywords on subclasses.
- Inspect the wrapped factory and reconstruct its signature with replace, copy, deepcopy and pickle. Reload the config module in an isolated process while retaining the original class references.
- A subprocess sets scoped capability, creates plain `PhysX`, attaches `simple_physics_scene.usda`, explicitly starts TCP before or after the first step, steps, and stops.

## Then

- The retired module is not importable and removed enum slots are absent; unrelated enum values match C (REQ AC-1).
- Removed startup fields, including `omnipvd_recording_capable`, raise `TypeError`
  before native creation, with guidance naming the global capability, explicit
  FILE/TCP recording and migration guide (REQ AC-2).
- Default `inspect.signature` reports effective base/subclass constructor parameters and annotations,
  including added dataclass fields and inherited, mixin, variadic or custom
  `__new__` constructors. A custom metaclass `__call__` takes precedence over
  class constructors; callable instances retain their `__call__` signatures. Normal
  positional construction, replace, deepcopy and pickle remain usable; unknown
  keywords raise the standard `TypeError` (REQ AC-2).
- Wrapped factories and retained class references after reload keep their constructor signatures (REQ AC-2).
- On Python 3.14+, a dataclass subclass with a lazy forward reference can be
  created and inspected with FORWARDREF/STRING formats before the referenced
  class exists, then inspected normally after it is defined (REQ AC-2).
- The bounded listener completes without error and receives more than eight bytes (REQ AC-3).

## Executable coverage

- `tests/python_tests/cpu_tests/test_public_ovd_api.py`: `PublicOvdApiTests.test_normal_config_rejects_startup_controls`, `test_retired_startup_module_is_not_importable`, and `test_current_enums_exclude_legacy_values`.
- `tests/python_tests/test_omnipvd_recording.py::test_tcp_streams_to_ready_listener` (before-first-step and after-first-step cases).
- `tests/python_tests/cpu_tests/test_physxconfig_validation.py`:
  `test_retired_ovd_field_type_error_explains_migration`,
  `test_config_constructor_retains_dataclass_signature_and_positional_order`,
  `test_config_signature_matches_generated_init_exactly`,
  `test_config_subclass_with_lazy_forward_reference`,
  `test_config_plain_and_abc_subclass_signature_matches_base`,
  `test_config_subclass_signature_includes_added_fields`,
  `test_config_mixin_signature_uses_nearest_constructor`,
  `test_config_variadic_constructor_signature_retains_args`,
  `test_config_callable_instance_signature_uses_call`,
  `test_config_custom_new_signature_is_not_overridden`,
  `test_config_metaclass_signature_uses_call`,
  `test_config_inherited_metaclass_call_takes_precedence`,
  `test_config_positional_only_self_is_dropped`,
  `test_config_wrapped_factory_preserves_signature`,
  `test_config_signature_survives_module_reload`,
  `test_config_subclass_retired_fields_keep_migration_error`,
  `test_config_retains_replace_deepcopy_and_pickle`, and
  `test_config_unknown_keyword_still_raises_standard_type_error`.
