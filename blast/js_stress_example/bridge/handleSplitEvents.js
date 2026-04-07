// Core split event handler, extracted for integration testing.
// Depends only on Rapier world and plain bridge state, no DOM/Three.

import { enqueueSplitResults } from './rapierHierarchyApplier.js';

export function handleSplitEvents(bridge, splitResults, RAPIER, contactForceThreshold = 0.0) {
  if (!Array.isArray(splitResults) || splitResults.length === 0) {
    return;
  }

  // Delegate to reusable applier (phase-1 only here)
  enqueueSplitResults(bridge, splitResults);
}


