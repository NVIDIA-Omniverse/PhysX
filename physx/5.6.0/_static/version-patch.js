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

/* allow a relative path for version json file */
if (DOCUMENTATION_OPTIONS.theme_switcher_json_url.startsWith(".")) {
    const relative_root = document.documentElement.getAttribute("data-content_root");
    const relative_json_url = DOCUMENTATION_OPTIONS.theme_switcher_json_url;
    const json_url = new URL(`${relative_root}${relative_json_url}`, window.location.href);
    DOCUMENTATION_OPTIONS.theme_switcher_json_url = json_url.href;
}

/* the elements to watch are populated by embedded JS, so wait for them to load before observing */
document.addEventListener("DOMContentLoaded", () => {
    /* update version dropdown links */
    function updateVersionDropdown(records, observer) {
        /* allow relative paths for versions */
        const relative_root = document.documentElement.getAttribute("data-content_root");
        for (const record of records) {
            for (const addedNode of record.addedNodes) {
                const href = addedNode.getAttribute("href");
                if (href.startsWith(".")) {
                    addedNode.setAttribute("href", `${relative_root}${href}`);
                }
            }
        }
    }
    var versionObserver = new MutationObserver(updateVersionDropdown);
    document.querySelectorAll(".version-switcher__menu").forEach((menu) => {
        versionObserver.observe(menu, {childList: true});
    });

    /* update version header link */
    function updateVersionHeader(records, observer) {
        /* allow relative paths for versions */
        const relative_root = document.documentElement.getAttribute("data-content_root");
        for (const record of records) {
            for (const addedNode of record.addedNodes) {
                const links = addedNode.getElementsByTagName("A");
                for (const link of links) {
                    const href = link.getAttribute("href");
                    if (href !== null && href.startsWith(".")) {
                        link.setAttribute("href", `${relative_root}${href}`);
                    }
                }
            }
        }
    }
    var versionHeaderObserver = new MutationObserver(updateVersionHeader);
    document.querySelectorAll("#bd-header-version-warning").forEach((header) => {
        versionHeaderObserver.observe(header, {childList: true});
    });
})
