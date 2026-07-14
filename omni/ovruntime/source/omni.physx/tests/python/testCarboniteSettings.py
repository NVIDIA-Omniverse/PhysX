# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import unittest
import carb.settings
import carb.dictionary
import carb.tokens


class TestCarboniteSettings(unittest.TestCase):
    """Verify Carbonite settings subsystem is functional."""

    def test_settings_interface_available(self):
        """Acquire the ISettings interface and verify it is not None."""
        settings = carb.settings.get_settings()
        self.assertIsNotNone(settings)

    def test_set_and_get_string(self):
        """Set a string setting and read it back."""
        settings = carb.settings.get_settings()
        settings.set("/test/ovruntime/greeting", "hello")
        value = settings.get("/test/ovruntime/greeting")
        self.assertEqual(value, "hello")

    def test_set_and_get_int(self):
        """Set an integer setting and read it back."""
        settings = carb.settings.get_settings()
        settings.set("/test/ovruntime/count", 42)
        value = settings.get("/test/ovruntime/count")
        self.assertEqual(value, 42)

    def test_set_and_get_float(self):
        """Set a float setting and read it back."""
        settings = carb.settings.get_settings()
        settings.set("/test/ovruntime/gravity", 9.81)
        value = settings.get("/test/ovruntime/gravity")
        self.assertAlmostEqual(value, 9.81, places=2)

    def test_set_and_get_bool(self):
        """Set a boolean setting and read it back."""
        settings = carb.settings.get_settings()
        settings.set("/test/ovruntime/enabled", True)
        value = settings.get("/test/ovruntime/enabled")
        self.assertTrue(value)

    def test_nonexistent_setting_returns_none(self):
        """Reading a setting that was never set returns None."""
        settings = carb.settings.get_settings()
        value = settings.get("/test/ovruntime/does_not_exist")
        self.assertIsNone(value)


class TestCarboniteDictionary(unittest.TestCase):
    """Verify Carbonite dictionary subsystem is functional."""

    def test_dictionary_interface_available(self):
        """Acquire the IDictionary interface and verify it is not None."""
        dictionary = carb.dictionary.get_dictionary()
        self.assertIsNotNone(dictionary)


class TestCarboniteTokens(unittest.TestCase):
    """Verify Carbonite tokens subsystem is functional."""

    def test_tokens_interface_available(self):
        """Acquire the ITokens interface and verify it is not None."""
        tokens = carb.tokens.get_tokens_interface()
        self.assertIsNotNone(tokens)

    def test_resolve_plain_text(self):
        """Resolve a string with no tokens passes through unchanged."""
        tokens = carb.tokens.get_tokens_interface()
        result = tokens.resolve("plain_text")
        self.assertEqual(result, "plain_text")


if __name__ == "__main__":
    unittest.main()
