/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: LicenseRef-NvidiaProprietary
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */

const initMermaid = (isDark) => {
    // reset the graph text for each graph
    for (const mermaidDiv of document.getElementsByClassName('mermaid')) {
        mermaidDiv.textContent = mermaidDiv.parentElement.getAttribute('data-mermaid');
        mermaidDiv.removeAttribute('data-processed');
    }

    if (typeof isDark === 'undefined') {
        isDark = document.documentElement.getAttribute('data-theme') === 'dark';
    };

    // intialize mermaid with our theme dependent config
    mermaid.initialize({
        logLevel: 'error',
        securityLevel: 'strict',
        theme: (isDark ? 'dark' : 'forest'),
        darkMode: isDark,
        background: (isDark ? '#FCFCFC' : '#000000')
    });
    mermaid.init();
};
