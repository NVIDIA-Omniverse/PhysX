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
  let params = (new URL(document.location)).searchParams;
  let term = params.get('q');
  return term
}


// set the search box content
async function setSearchBoxContent(term) {
  document.getElementById('rtd-search-form').querySelector('[name="q"]').value = term;
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


// query the index with an augmented query term and return results
async function getResults(idx, term) {
  const quoteSplitter = /['"](.*?)['"]/g;  // when splitting regexes have a group the group is included in results
  const termSplitter = /[^\w+-:^~*]+/g;  // exclude characters that have special meaning for lunr

  // separate out quoted phrases
  const searchString = Array.from(term.split(quoteSplitter))
    .map(function (e, i) {
      if (i % 2 == 1) {
        // handle quoted
        return e.split(/[\W]+/g)
          .map(d => d.trim())
          .filter(d => d.length > 0)
          .map(d => `+${d}`)  // `+` in lunr means: "must have this term"
          .join(" ");
      }
      else {
        // handle unquoted
        return e.split(termSplitter)
          .map(d => d.trim())
          .filter(d => d.length > 0)
          .map(d => d.endsWith("*") ? d : `${d} ${d}*`)  // add `*` so users searching `wor` will get `world` results
          .join(" ")
      }
    })
    .join(" ");

  const searchResults = idx.search(searchString);
  return searchResults;
}


// build the text surrounding the search match to be displayed in the results
function buildSearchResultContext(ob) {
  const term = Object.keys(ob.matchData.metadata)[0];
  const offset = 100;

  // get the positions of the first match
  const match = ob.matchData.metadata[term]['content'];
  const [matchStart, matchLength] = match !== undefined ? match.position[0] : [0, 0];
  const matchEnd = matchStart + matchLength;

  // clamp to string length
  const contextStartUnclamped = matchStart - offset;
  const contextStart = Math.max(0, contextStartUnclamped);
  const isStartTrimmed = contextStart === contextStartUnclamped && matchLength !== 0;
  const contextEndUnclamped = matchEnd + offset;
  const contextEnd = Math.min(ob.data.content.length, contextEndUnclamped);
  const isEndTrimmed = contextEnd === contextEndUnclamped;

  // trim the context to match desired bounds
  let context = ob.data.content.substring(0, contextEnd);
  const highlighted = matchLength !== 0 ? `<span class=highlighted>${context.slice(matchStart, matchEnd)}</span>` : '';
  context = `${context.slice(0, matchStart)}${highlighted}${context.slice(matchEnd)}`.substring(contextStart);

  return `${isStartTrimmed ? '...' : ''}${context}${isEndTrimmed ? '...' : ''}`;
}


// show the resulting object, its type, and the context around the result
async function displayResults(term, searchData, searchResults) {
  const highlightText = term.replaceAll(/\W+/g, " ").trim();
  let resultsdiv = document.getElementById('search-results');
  resultsdiv.innerHTML = `
  <h2>Search results</h2>
  <p class='search-summary'>Found ${searchResults.length} match${searchResults.length === 1 ? '' : 'es'}.</p>
  <ul class='search'>
    ${searchResults
      .map(ele => ({ data: searchData.data[ele.ref], matchData: ele.matchData }))
      .map(ob => ({
        uri: `${ob.data.filename}?highlight=${encodeURIComponent(highlightText)}#${ob.data.anchor}`,
        name: `${ob.data.display_name}`,
        sitename: Object.hasOwn(ob.data, 'sitename') ? ob.data.sitename : null,
        parent_name: `${ob.data.type !== 'doc' ? `${searchData.data[ob.data.doc_id].display_name}` : ''}`,
        display_type: `${ob.data.display_type}`,
        context: `${buildSearchResultContext(ob)}`,
      }))
      .map(ob => `
      <li>
        ${ob.sitename ? `<div>${ob.sitename}: </div>` : ''}
        <a href='${ob.uri}'>${ob.parent_name}${ob.parent_name ? ' &mdash; ' : ''}${ob.name}</a>
        <span>${ob.display_type}</span>
        ${ob.context ? `<p class='context'>${ob.context}</p>` : ''}
      </li>`
      )
      .join('')
    }
  </ul>`
};


async function main() {
  const term = await getSearchTerm();
  setSearchBoxContent(term);
  const idx = await buildIndex(searchData);
  const searchResults = await getResults(idx, term);
  displayResults(term, searchData, searchResults);
}


document.addEventListener('DOMContentLoaded', main);
