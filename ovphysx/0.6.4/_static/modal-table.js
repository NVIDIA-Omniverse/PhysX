/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
  /* created required elements */
  const tables = document.querySelectorAll("table.table-expandable[id]");
  tables.forEach((table) => {
    injectExpandButton(table);
  });
  const modal = injectModalPlaceholder();

  /* copy content to modal */
  modal.addEventListener("shown.bs.modal", event => {
    const content = document.querySelector(event.relatedTarget.dataset.bsTargetContent);
    const parent = content.parentNode;
    const newContent = modalDiv.querySelector(".modal-body").appendChild(content);
    modal.addEventListener("hide.bs.modal", event => {
      parent.appendChild(newContent);
    },
    { once: true });
  });

  function injectExpandButton(table) {
    /* create expand button */
    let btn = document.createElement("button");
    btn.classList.add("btn", "btn-sm", "btn-outline-secondary", "d-none", "d-lg-block", "position-absolute", "end-0");
    btn.dataset.bsTarget = `#--modal-placeholder`;
    btn.dataset.bsToggle = "modal";
    btn.dataset.bsTargetContent = `#${table.id}`;
    btn.setAttribute("aria-label", "Expand");

    /* populate button */
    let icon = document.createElement("i");
    icon.classList.add("fa", "fa-solid", "fa-up-right-and-down-left-from-center");
    btn.appendChild(icon);

    /* insert button */
    table.before(btn);
  };

  function injectModalPlaceholder() {
    /* create modal placeholder */
    modalDiv = document.createElement("div");
    modalDiv.classList.add("modal", "fade");
    modalDiv.id = "--modal-placeholder";
    modalDiv.tabIndex = "-1";
    modalDiv.setAttribute("aria-hidden", "true");

    /* create modal dialog */
    dialog = document.createElement("div");
    dialog.classList.add("modal-dialog", "modal-dialog-scrollable", "modal-fullscreen");

    /* create modal content */
    content = document.createElement("div");
    content.classList.add("modal-content");

    /* create modal header */
    header = document.createElement("div");
    header.classList.add("modal-header");
    headerBtn = document.createElement("button");
    headerBtn.classList.add("nav-link", "pst-navbar-icon");
    headerBtn.setAttribute("data-bs-dismiss", "modal");
    headerBtn.setAttribute("aria-label", "Close");
    headerBtnIcon = document.createElement("i");
    headerBtnIcon.classList.add("fa-solid", "fa-xmark");
    headerBtn.appendChild(headerBtnIcon);
    header.appendChild(headerBtn);

    /* create modal body */
    body = document.createElement("div");
    body.classList.add("modal-body", "mx-auto");

    /* populate modal */
    content.appendChild(header);
    content.appendChild(body);
    dialog.appendChild(content);
    modalDiv.appendChild(dialog);

    /* inject modal placeholder */
    document.body.appendChild(modalDiv);

    return modalDiv;
  };
});
