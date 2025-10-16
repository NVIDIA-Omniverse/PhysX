import { vec3 } from './stress.js';

/**
 * Procedurally generate a simplified bridge deck + pier layout for stress simulations.
 *
 * The deck is discretized into a 3D grid of nodes (span × width × thickness). Each node
 * stores its centroid, mass, and volume. Bonds are created between immediate neighbours along
 * the span (X), thickness (Y), and width (Z) axes. Pier supports are added at both ends of the
 * bridge and bonded to the deck's bottom layer.
 *
 * @param {Object} options
 * @param {number} options.span - Total deck length along the X axis.
 * @param {number} options.deckWidth - Deck width along the Z axis.
 * @param {number} options.deckThickness - Deck thickness along the Y axis.
 * @param {number} options.spanSegments - Number of nodes distributed along the span.
 * @param {number} options.widthSegments - Number of nodes distributed across the deck width.
 * @param {number} options.thicknessLayers - Number of nodes stacked through the deck thickness.
 * @param {number} options.deckMass - Total mass attributed to the deck nodes.
 * @param {number} options.pierHeight - Height of the pier supports below the deck.
 * @param {number} options.areaScale - Scale factor applied to the bond cross-sectional area.
 * @returns {Object} Structured node/bond data for the bridge deck and supports.
 */
export function buildBridgeScenario({
  span = 20.0,
  deckWidth = 8.0,
  deckThickness = 0.6,
  // spanSegments = 40,
  // widthSegments = 10,
  spanSegments = 20,
  widthSegments = 6,
  // spanSegments = 15,
  // widthSegments = 5,
  // spanSegments = 15,
  // widthSegments = 5,
  // spanSegments = 12,
  // widthSegments = 4,
  // spanSegments = 4,
  // widthSegments = 4,
  thicknessLayers = 3,
  // thicknessLayers = 2,
  // thicknessLayers = 1,
  // deckMass = 60_000.0,
  // deckMass = 1_000.0,
  deckMass = 80_000.0,
  pierHeight = 3.0,
  // areaScale = 0.05
  areaScale = 0.10
} = {}) {
  const nodes = [];
  const bonds = [];

  // Grid spacing between nodes along each principal axis. With a single segment/layer we fall
  // back to the overall span/width/thickness so that the geometry remains centred.
  const spacingX = spanSegments > 1 ? span / (spanSegments - 1) : span;
  const spacingZ = widthSegments > 1 ? deckWidth / (widthSegments - 1) : deckWidth;
  const spacingY = thicknessLayers > 1 ? deckThickness / (thicknessLayers - 1) : deckThickness;

  // Position the deck so that it is centred around the origin on X/Z and symmetric in Y.
  const originX = -span * 0.5;
  const originZ = widthSegments > 1 ? -deckWidth * 0.5 : 0.0;
  const originY = -deckThickness * 0.5;

  // Distribute the deck's total mass/volume equally among all deck nodes.
  const totalDeckNodes = spanSegments * widthSegments * thicknessLayers;
  const massPerDeckNode = deckMass / Math.max(totalDeckNodes, 1);
  const volumePerDeckNode = spacingX * spacingZ * spacingY;

  // 3D lookup table mapping (ix, iy, iz) to node indices inside `nodes`.
  const nodeIndex = Array.from({ length: spanSegments }, () =>
    Array.from({ length: thicknessLayers }, () => Array(widthSegments))
  );

  // Additional bookkeeping:
  //  - `gridCoordinates` lets us recover the original grid coordinates given a node index.
  //  - `topColumnNodes` collects the top-most nodes for each span column (useful for loads).
  //  - `deckBottomNodes` tracks the bottom layer (useful for attaching supports / gravity).
  const gridCoordinates = [];
  const topColumnNodes = Array.from({ length: spanSegments }, () => []);
  const deckBottomNodes = [];

  // Helper for accessing the node index at a particular grid coordinate.
  const indexAt = (ix, iy, iz) => nodeIndex[ix][iy][iz];

  // Populate the deck node grid. Each node receives centroid/mass/volume entries.
  for (let ix = 0; ix < spanSegments; ++ix) {
    for (let iy = 0; iy < thicknessLayers; ++iy) {
      for (let iz = 0; iz < widthSegments; ++iz) {
        const centroid = vec3(
          originX + ix * spacingX,
          originY + iy * spacingY,
          originZ + iz * spacingZ
        );
        const node = { centroid, mass: massPerDeckNode, volume: volumePerDeckNode };
        const index = nodes.length;
        nodes.push(node);
        nodeIndex[ix][iy][iz] = index;
        gridCoordinates[index] = { ix, iy, iz };
        if (iy === thicknessLayers - 1) {
          topColumnNodes[ix].push(index);
        }
        if (iy === 0) {
          deckBottomNodes.push(index);
        }
      }
    }
  }

  const deckBondAreaX = spacingY * spacingZ * areaScale;
  const deckBondAreaY = spacingX * spacingZ * areaScale;
  const deckBondAreaZ = spacingX * spacingY * areaScale;

  const addBond = (a, b, area) => {
    const na = nodes[a];
    const nb = nodes[b];
    const centroid = vec3(
      (na.centroid.x + nb.centroid.x) * 0.5,
      (na.centroid.y + nb.centroid.y) * 0.5,
      (na.centroid.z + nb.centroid.z) * 0.5
    );
    const normal = normalize(subtract(nb.centroid, na.centroid));
    bonds.push({ centroid, normal, area, node0: a, node1: b });
  };

  for (let ix = 0; ix < spanSegments; ++ix) {
    for (let iy = 0; iy < thicknessLayers; ++iy) {
      for (let iz = 0; iz < widthSegments; ++iz) {
        const current = indexAt(ix, iy, iz);
        if (ix + 1 < spanSegments) {
          addBond(current, indexAt(ix + 1, iy, iz), deckBondAreaX);
        }
        if (iy + 1 < thicknessLayers) {
          addBond(current, indexAt(ix, iy + 1, iz), deckBondAreaY);
        }
        if (iz + 1 < widthSegments) {
          addBond(current, indexAt(ix, iy, iz + 1), deckBondAreaZ);
        }
      }
    }
  }

  // Create pier support nodes under the first and last span columns, bonding them directly to
  // the deck's bottom layer. Supports have higher mass/volume to approximate solid columns.
  const supportIndices = [];
  const supportLinks = [];
  const supportArea = spacingX * spacingZ * areaScale;
  const supportMass = massPerDeckNode * 6.0;
  const supportVolume = spacingX * spacingZ * pierHeight;

  for (const ix of [0, spanSegments - 1]) {
    for (let iz = 0; iz < widthSegments; ++iz) {
      const baseIndex = indexAt(ix, 0, iz);
      const baseNode = nodes[baseIndex];
      const deckBottomY = baseNode.centroid.y - spacingY * 0.5;
      const supportCentroid = vec3(
        baseNode.centroid.x,
        deckBottomY - pierHeight * 0.5,
        baseNode.centroid.z
      );
      const supportIndex = nodes.length;
      nodes.push({ centroid: supportCentroid, mass: supportMass, volume: supportVolume });
      supportIndices.push(supportIndex);
      supportLinks.push({ supportIndex, deckIndex: baseIndex });
      gridCoordinates[supportIndex] = { ix, iy: -1, iz };
      addBond(baseIndex, supportIndex, supportArea);
    }
  }

  console.log('nodes:', nodes.length, nodes);
  console.log('supportIndices:', supportIndices);

  return {
    nodes,
    bonds,
    topColumnNodes,
    supportIndices,
    supportLinks,
    spanSegments,
    widthSegments,
    thicknessLayers,
    spacing: { x: spacingX, y: spacingY, z: spacingZ },
    origins: { x: originX, y: originY, z: originZ },
    parameters: { span, deckWidth, deckThickness, deckMass, pierHeight, areaScale },
    gridCoordinates
  };
}

function subtract(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function normalize(v) {
  const len = Math.hypot(v.x, v.y, v.z);
  if (len === 0) {
    return vec3();
  }
  return vec3(v.x / len, v.y / len, v.z / len);
}


