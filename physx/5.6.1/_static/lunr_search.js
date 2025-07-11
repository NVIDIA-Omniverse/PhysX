/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: LicenseRef-NvidiaProprietary
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */


// get the search term from the url query string
async function getSearchTerm() {
  const params = (new URL(document.location)).searchParams;
  // use ~ as a substitute for whitespace which is not encoded (normalising quotes first)
  const term = encodeURIComponent(params.get('q').replace(/"/g, "'").replace(/\s+/g, "~"));

  const quoteSplitter = /'(.*?)'/g;  // when splitting regexes have a group, the group is included in results

  // separate out quoted phrases
  const searchString = Array.from(term.split(quoteSplitter))
    .map(function (e, i) {
      return e.split(/[\W]+/g)
        .map(d => d.trim())
        .filter(d => d.length > 0)
        // `+` in lunr means: "must have this term", so use it for quoted terms
        // add `*` so users searching `wor` will get `world` results when unquoted
        .map(d => i % 2 == 1 ? `+${d}` : `${d} ${d}*`)
        .join(" ");
    })
    .join(" ");

  return [term, searchString];
}


// build the lunr search index based on the project data
async function buildIndex(searchData) {
  // split the text on any non-word characters
  lunr.tokenizer.separator = /[\W]+/;

  var idx = lunr(function () {
    this.ref('id');
    this.field('display_name', { boost: 10 });
    this.field('keywords', { boost: 5 });
    this.field('content');
    this.metadataWhitelist = ['position'];

    searchData.data.forEach(function (doc) {
      this.add(doc);
    }, this);
  });

  return idx;
}


// build the text surrounding the search match to be displayed in the results
function buildSearchResultContext(ob) {
  const match = Object.values(ob.matchData.metadata)[0]['content'];
  const offset = 100;

  // get the positions of the first match
  const [matchStart, matchLength] = match !== undefined ? match.position[0] : [0, 0];
  const matchEnd = matchStart + matchLength;

  // clamp to string length
  const contextStartUnclamped = matchStart - offset;
  const contextStart = Math.max(0, contextStartUnclamped);
  const isStartTrimmed = contextStart === contextStartUnclamped && matchLength !== 0;
  const contextEndUnclamped = matchEnd + offset;
  const contextEnd = Math.min(ob.data.content.length, contextEndUnclamped);
  const isEndTrimmed = contextEnd === contextEndUnclamped;

  // short circuit if there is not text context (maybe page title match)
  if (matchLength === 0) {
    return null;
  }

  // trim the context to match desired bounds
  let context = ob.data.content.substring(0, contextEnd);

  // create the container element for the context
  let contextElement = document.createElement("p");
  contextElement.classList.add("context");

  let startText = document.createTextNode(`${isStartTrimmed ? '...' : ''}${context.slice(0, matchStart).substring(contextStart)}`);
  contextElement.appendChild(startText);

  let highlightElement = document.createElement("span");
  highlightElement.classList.add("highlighted");
  highlightElement.textContent = context.slice(matchStart, matchEnd);
  contextElement.appendChild(highlightElement);

  let endText = document.createTextNode(`${context.slice(matchEnd)}${isEndTrimmed ? '...' : ''}`);
  contextElement.appendChild(endText);

  return contextElement;
}


// build the list item element for each search result
function buildSearchResultListItem(ob) {
  let item = document.createElement("li");

  if (ob.sitename) {
    let siteName = document.createElement("div");
    siteName.textContent = ob.sitename;
    item.appendChild(siteName);
  }

  let link = document.createElement("a")
  link.textContent = `${ob.parent_name}${ob.parent_name ? ' — ' : ''}${ob.name}`;
  link.href = encodeURI(ob.uri);
  item.appendChild(link);

  let displayType = document.createElement("span");
  displayType.textContent = ob.display_type;
  item.appendChild(displayType);

  if (ob.context) {
    item.appendChild(ob.context);
  }

  return item;
}


// show the resulting object, its type, and the context around the result
async function displayResults(highlightText, searchData, searchResults) {
  let resultsdiv = document.getElementById('search-results');

  let heading = document.createElement("h2");
  heading.textContent = 'Search results';
  resultsdiv.appendChild(heading);

  let summary = document.createElement("p");
  summary.classList.add("search-summary");
  summary.textContent = `Found ${searchResults.length} match${searchResults.length === 1 ? '' : 'es'}.`;
  resultsdiv.appendChild(summary);

  let results = document.createElement("ul")
  results.classList.add("search")

  searchResults
    .map(ele => ({ data: searchData.data[ele.ref], matchData: ele.matchData }))
    .map(ob => ({
      uri: `${ob.data.filename}?highlight=${highlightText}#${ob.data.anchor}`,
      name: `${ob.data.display_name}`,
      sitename: Object.hasOwn(ob.data, 'sitename') ? ob.data.sitename : null,
      parent_name: `${ob.data.type !== 'doc' ? `${searchData.data[ob.data.doc_id].display_name}` : ''}`,
      display_type: `${ob.data.display_type}`,
      context: buildSearchResultContext(ob),
    }))
    .forEach(ob => results.appendChild(buildSearchResultListItem(ob)));

  resultsdiv.appendChild(results);
};


async function main() {
  const [term, searchString] = await getSearchTerm();
  // set the search box content, .value is a safe sink so we can set it directly (and replacing whitespace placeholders)
  // see: https://cheatsheetseries.owasp.org/cheatsheets/Cross_Site_Scripting_Prevention_Cheat_Sheet.html#safe-sinks
  document.getElementById('search-input').value = decodeURIComponent(term).replace(/~/g, " ");
  const idx = await buildIndex(searchData);
  const searchResults = idx.search(searchString);
  const highlightText = encodeURIComponent(term.replace(/\W+/g, " ").trim());
  displayResults(highlightText, searchData, searchResults);
}


document.addEventListener('DOMContentLoaded', main);
