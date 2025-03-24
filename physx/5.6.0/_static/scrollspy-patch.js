/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: LicenseRef-NvidiaProprietary
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */

document.addEventListener("DOMContentLoaded", () => {
    // patch ScrollSpy's initializeTargetsAndObservables function
    bootstrap.ScrollSpy.prototype._initializeTargetsAndObservables = function () {
        // get all links in the nav
        const targetLinks = this._config.target.querySelectorAll('[href]');
        for (const anchor of targetLinks) {
            if (!anchor.hash) {
                continue;
            }
            // find the element that the link targets
            const target = document.getElementById(decodeURI(anchor.hash).substring(1));
            if (!target) {
                continue;
            }
            this._targetLinks.set(decodeURI(anchor.hash), anchor);
            this._observableSections.set(anchor.hash, target);
        }
    }

    // rebuild the scrollspy for all targets
    const scrollSpy = bootstrap.ScrollSpy.getOrCreateInstance(document.body);
    scrollSpy.refresh();
});
