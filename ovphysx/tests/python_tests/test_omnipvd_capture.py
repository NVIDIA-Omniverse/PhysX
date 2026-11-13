# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-CAPTURE-001
# @covers AC-1

"""Pure Python filter descriptor tests; runnable before a native build."""

import unittest

import ovphysx


class OmniPvdFilterConfigTest(unittest.TestCase):
    def test_secondary_data_defaults_enabled_and_accepts_boolean_selection(self):
        self.assertIs(ovphysx.OmniPvdFilter().record_secondary_data, True)
        for value in (False, True):
            with self.subTest(value=value):
                self.assertIs(
                    ovphysx.OmniPvdFilter(record_secondary_data=value).record_secondary_data,
                    value,
                )

    def test_paths_are_copied_to_immutable_tuples(self):
        paths = ["/World/robot"]
        selection = ovphysx.OmniPvdFilter(objects=paths)
        paths[0] = "/World/other"
        self.assertEqual(selection.objects, ("/World/robot",))
        self.assertEqual(selection.scenes, ())

    def test_selection_lists_preserve_all_roots_and_buffer_granularity(self):
        selection = ovphysx.OmniPvdFilter(
            scenes=["/World/scene"], objects=["/World/body", "/World/robot", "/World/particles"]
        )
        self.assertEqual(selection.scenes, ("/World/scene",))
        self.assertEqual(selection.objects, ("/World/body", "/World/robot", "/World/particles"))

    def test_secondary_data_requires_a_boolean(self):
        for value in (0, 1, -1, 0xFFFFFFFF, "secondary", 1.5, None):
            with self.subTest(value=value), self.assertRaises(TypeError):
                ovphysx.OmniPvdFilter(record_secondary_data=value)

    def test_invalid_paths_rejected(self):
        for field in ("scenes", "objects"):
            for path in ("", "relative", "/World/*", "/World/body\0tail", "/",
                         "//World/body", "/World//body", "/World/body/", "/World/body.x",
                         "/World/body:attr", "/World/body name", "/World/body\tname",
                         "/World/body\nname", "/World/1body", "/World/body-name",
                         "/World/\ud800"):
                with self.subTest(field=field, path=path), self.assertRaises(ValueError):
                    ovphysx.OmniPvdFilter(**{field: [path]})

    def test_unicode_prim_names_and_ascii_identifiers_are_preserved(self):
        paths = ("/World/_body123", "/World/\u00e9", "/World/\u4e2d", "/World/env\u00e9")
        for field in ("scenes", "objects"):
            self.assertEqual(getattr(ovphysx.OmniPvdFilter(**{field: paths}), field), paths)

    def test_invalid_path_list_types_rejected(self):
        for field in ("scenes", "objects"):
            for value in ("/World/body", None, [42]):
                with self.subTest(field=field, value=value), self.assertRaises(TypeError):
                    ovphysx.OmniPvdFilter(**{field: value})


if __name__ == "__main__":
    unittest.main()
