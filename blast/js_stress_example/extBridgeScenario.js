/**
 * Bridge scenario builder — thin wrapper around blast-stress-solver/scenarios.
 *
 * Re-exports buildBeamBridgeScenario as buildBridgeScenario for backwards
 * compatibility with existing consumers. Also reconstructs legacy fields
 * (topColumnNodes, supportIndices, supportLinks) from the ScenarioDesc output.
 */
import { buildBeamBridgeScenario } from '../blast-stress-solver/src/scenarios/bridgeScenario.js';

export function buildBridgeScenario(options = {}) {
  const scenario = buildBeamBridgeScenario(options);

  // Reconstruct legacy fields expected by old consumers (buildBridge.headless.ts etc.)
  const gc = scenario.gridCoordinates || [];
  const params = scenario.parameters || {};
  const spanSegs = params.spanSegments ?? options.spanSegments ?? 30;

  // topColumnNodes: for each span column, collect highest-iy nodes
  const topColumnNodes = Array.from({ length: spanSegs }, () => []);
  const thicknessLayers = params.thicknessLayers ?? options.thicknessLayers ?? 2;
  for (let i = 0; i < gc.length; i++) {
    const coord = gc[i];
    if (coord && coord.iy >= 0 && coord.iy === thicknessLayers - 1 && coord.ix < spanSegs) {
      topColumnNodes[coord.ix].push(i);
    }
  }

  // supportIndices: nodes with mass=0
  const supportIndices = [];
  const supportLinks = [];
  for (let i = 0; i < scenario.nodes.length; i++) {
    if (scenario.nodes[i].mass === 0) {
      supportIndices.push(i);
    }
  }

  return {
    ...scenario,
    topColumnNodes,
    supportIndices,
    supportLinks,
    spanSegments: spanSegs,
    widthSegments: params.widthSegments ?? options.widthSegments ?? 10,
    thicknessLayers,
  };
}
