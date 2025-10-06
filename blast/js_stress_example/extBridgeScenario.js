import { vec3 } from './stress.js';

export function buildBridgeScenario({
  span = 20.0,
  deckWidth = 8.0,
  deckThickness = 0.6,
  spanSegments = 12,
  widthSegments = 4,
  // spanSegments = 2,
  // widthSegments = 2,
  thicknessLayers = 2,
  deckMass = 60000.0,
  pierHeight = 3.0,
  areaScale = 0.05
} = {}) {
  const nodes = [];
  const bonds = [];

  const spacingX = spanSegments > 1 ? span / (spanSegments - 1) : span;
  const spacingZ = widthSegments > 1 ? deckWidth / (widthSegments - 1) : deckWidth;
  const spacingY = thicknessLayers > 1 ? deckThickness / (thicknessLayers - 1) : deckThickness;

  const originX = -span * 0.5;
  const originZ = widthSegments > 1 ? -deckWidth * 0.5 : 0.0;
  const originY = -deckThickness * 0.5;

  const totalDeckNodes = spanSegments * widthSegments * thicknessLayers;
  const massPerDeckNode = deckMass / Math.max(totalDeckNodes, 1);
  const volumePerDeckNode = spacingX * spacingZ * spacingY;

  const nodeIndex = Array.from({ length: spanSegments }, () =>
    Array.from({ length: thicknessLayers }, () => Array(widthSegments))
  );
  const gridCoordinates = [];
  const topColumnNodes = Array.from({ length: spanSegments }, () => []);
  const deckBottomNodes = [];

  const indexAt = (ix, iy, iz) => nodeIndex[ix][iy][iz];

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


