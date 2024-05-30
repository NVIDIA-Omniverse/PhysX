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

const isThemeDark = () => {
    var isDark = false;
    if (sessionStorage.getItem('repo-docs-theme')) {
        isDark = sessionStorage.getItem('repo-docs-theme') === 'dark';
    }
    else if (window.matchMedia('(prefers-color-scheme: dark)').matches) {
        isDark = true;
    }
    return isDark;
};

const createThemeSwitcher = () => {
    let btn = document.createElement('BUTTON');
    btn.className = 'theme-switcher';
    btn.id = 'theme-switcher';
    btn.onclick = function () {
        const newTheme = isThemeDark() ? 'light' : 'dark';

        sessionStorage.setItem('repo-docs-theme', newTheme);
        document.documentElement.setAttribute('data-theme', newTheme);

        setThemeIcon(this);
        initMermaid(isThemeDark());
    };
    document.body.appendChild(btn);
    setThemeIcon(btn);
};

const setThemeIcon = (element) => {
    const icon = isThemeDark() ? 'sun' : 'moon';
    element.innerHTML = `<i class='fa fa-${icon}-o'></i>`;
};

document.addEventListener('DOMContentLoaded', () => {
    createThemeSwitcher();
}, false);

document.documentElement.setAttribute('data-theme', isThemeDark() ? 'dark' : 'light');
