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
export function buildBridgeScenario({ span = 20.0, deckWidth = 8.0, deckThickness = 0.6, 
// spanSegments = 40,
// widthSegments = 10,
// spanSegments = 20,
// widthSegments = 6,
spanSegments = 15, widthSegments = 5, 
// spanSegments = 15,
// widthSegments = 5,
// spanSegments = 12,
// widthSegments = 4,
// spanSegments = 4,
// widthSegments = 4,
// thicknessLayers = 3,
thicknessLayers = 2, 
// thicknessLayers = 1,
deckMass = 60000, 
// deckMass = 1_000.0,
// deckMass = 80_000.0,
pierHeight = 3.0, areaScale = 0.05, 
// NEW: make supports stiffer than deck bonds
supportAreaFactor = 2.5, 
// ---- isotropy helpers ----
addDiagonals = true, diagScale = 0.7, // diagonals are weaker than faces
bondJitter = 0.12, // 0–0.25; heterogeneity to avoid grid cracks
normalizeAreas = true // per-axis area renormalization
// areaScale = 0.10
// areaScale = 1.00
 } = {}) {
    const nodes = [];
    const bonds = [];
    normalizeAreas = true;
    addDiagonals = false;
    diagScale = 0.75;
    // bondJitter = 0.00;
    // Debug/verification controls for bond correctness
    const DEBUG_VERIFY_BONDS = true;
    const seenBondPairs = new Set();
    let __DEBUG_SUPPORT_AREA = null; // set after support area is computed
    // Stable, order-independent key for a bond between node indices a and b
    const bondPairKey = (a, b) => (a < b ? `${a}-${b}` : `${b}-${a}`);
    // Floating point comparison with tolerance
    const approximatelyEqual = (a, b, eps = 1e-6) => Math.abs(a - b) <= eps * Math.max(1, Math.max(Math.abs(a), Math.abs(b)));
    // Cell dimensions (count-based). This keeps strength invariant under refinement.
    const cellX = spanSegments > 0 ? span / Math.max(spanSegments, 1) : span;
    const cellZ = widthSegments > 0 ? deckWidth / Math.max(widthSegments, 1) : deckWidth;
    const cellY = thicknessLayers > 0 ? deckThickness / Math.max(thicknessLayers, 1) : deckThickness;
    // Position the deck so that it is centred around the origin on X/Z and symmetric in Y.
    // Place nodes at cell centers.
    const originX = -span * 0.5 + 0.5 * cellX;
    const originZ = widthSegments > 1 ? -deckWidth * 0.5 + 0.5 * cellZ : 0.0;
    const originY = -deckThickness * 0.5 + 0.5 * cellY;
    // Distribute the deck's total mass/volume equally among all deck nodes.
    const totalDeckNodes = spanSegments * widthSegments * thicknessLayers;
    const massPerDeckNode = deckMass / Math.max(totalDeckNodes, 1);
    const volumePerDeckNode = cellX * cellZ * cellY;
    // 3D lookup table mapping (ix, iy, iz) to node indices inside `nodes`.
    const nodeIndex = Array.from({ length: spanSegments }, () => Array.from({ length: thicknessLayers }, () => Array(widthSegments)));
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
                const centroid = vec3(originX + ix * cellX, originY + iy * cellY, originZ + iz * cellZ);
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
    // Cross-sectional areas used for bonds along principal axes (cell-based)
    const deckBondAreaX = cellY * cellZ * areaScale; // faces perpendicular to X
    const deckBondAreaY = cellX * cellZ * areaScale; // faces perpendicular to Y
    const deckBondAreaZ = cellX * cellY * areaScale; // faces perpendicular to Z
    const addBond = (a, b, area) => {
        const na = nodes[a];
        const nb = nodes[b];
        const centroid = vec3((na.centroid.x + nb.centroid.x) * 0.5, (na.centroid.y + nb.centroid.y) * 0.5, (na.centroid.z + nb.centroid.z) * 0.5);
        const normal = normalize(subtract(nb.centroid, na.centroid));
        if (DEBUG_VERIFY_BONDS) {
            // 1) Ensure we don't add duplicate bonds regardless of order
            const key = bondPairKey(a, b);
            if (seenBondPairs.has(key)) {
                console.warn('Duplicate bond detected between nodes', { a, b });
            }
            else {
                seenBondPairs.add(key);
            }
            // 2) Verify adjacency in grid space: deck neighbors differ by exactly 1 in a single axis.
            // Allow plane diagonals when addDiagonals = true.
            const ga = gridCoordinates[a];
            const gb = gridCoordinates[b];
            if (ga && gb) {
                // Support bonds: one endpoint is a support (iy === -1) directly beneath a bottom deck node (iy === 0)
                const isSupportPair = ((ga.iy === -1 && gb.iy === 0) || (ga.iy === 0 && gb.iy === -1)) &&
                    ga.ix === gb.ix &&
                    ga.iz === gb.iz;
                if (!isSupportPair) {
                    const dx = Math.abs(ga.ix - gb.ix);
                    const dy = Math.abs(ga.iy - gb.iy);
                    const dz = Math.abs(ga.iz - gb.iz);
                    const manhattan = dx + dy + dz;
                    const isPlaneDiagonal = (manhattan === 2) && ((dx === 1 && dz === 1 && dy === 0) || (dx === 1 && dy === 1 && dz === 0) || (dy === 1 && dz === 1 && dx === 0));
                    if (manhattan !== 1 && !(addDiagonals && isPlaneDiagonal)) {
                        console.warn('Non-adjacent nodes bonded (expected immediate neighbors)', { a, b, ga, gb });
                    }
                    else {
                        // 3) Check area matches the axis of adjacency for deck bonds
                        if (dx === 1 && manhattan === 1 && !(approximatelyEqual(area, deckBondAreaX))) {
                            console.warn('Bond area mismatch for X-adjacent nodes', { a, b, area, expected: deckBondAreaX });
                        }
                        if (dy === 1 && manhattan === 1 && !(approximatelyEqual(area, deckBondAreaY))) {
                            console.warn('Bond area mismatch for Y-adjacent nodes', { a, b, area, expected: deckBondAreaY });
                        }
                        if (dz === 1 && manhattan === 1 && !(approximatelyEqual(area, deckBondAreaZ))) {
                            console.warn('Bond area mismatch for Z-adjacent nodes', { a, b, area, expected: deckBondAreaZ });
                        }
                    }
                }
                else {
                    // For support bonds, validate against support area if available
                    if (__DEBUG_SUPPORT_AREA != null && !(approximatelyEqual(area, __DEBUG_SUPPORT_AREA))) {
                        console.warn('Support bond area mismatch', { a, b, area, expected: __DEBUG_SUPPORT_AREA });
                    }
                }
            }
        }
        if (bondJitter === 0.0) {
            bonds.push({ centroid, normal, area: area, node0: a, node1: b });
        }
        else {
            const jitter = 1 + (Math.random() * 2 - 1) * bondJitter;
            const areaJittered = Math.max(area * jitter, 1e-8);
            bonds.push({ centroid, normal, area: areaJittered, node0: a, node1: b });
        }
    };
    // Establish bonds only to immediate neighbors in +X, +Y, +Z to avoid duplicates.
    // This creates a 6-connected grid (face neighbors) across the deck volume.
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
                if (addDiagonals) {
                    // XZ plane diagonals
                    if (ix + 1 < spanSegments && iz + 1 < widthSegments) {
                        const a = 0.5 * (deckBondAreaX + deckBondAreaZ) * diagScale;
                        addBond(current, indexAt(ix + 1, iy, iz + 1), a);
                        addBond(indexAt(ix, iy, iz + 1), indexAt(ix + 1, iy, iz), a);
                    }
                    // XY plane diagonals
                    if (ix + 1 < spanSegments && iy + 1 < thicknessLayers) {
                        const a = 0.5 * (deckBondAreaX + deckBondAreaY) * diagScale;
                        addBond(current, indexAt(ix + 1, iy + 1, iz), a);
                        addBond(indexAt(ix, iy + 1, iz), indexAt(ix + 1, iy, iz), a);
                    }
                    // YZ plane diagonals
                    if (iy + 1 < thicknessLayers && iz + 1 < widthSegments) {
                        const a = 0.5 * (deckBondAreaY + deckBondAreaZ) * diagScale;
                        addBond(current, indexAt(ix, iy + 1, iz + 1), a);
                        addBond(indexAt(ix, iy + 1, iz), indexAt(ix, iy, iz + 1), a);
                    }
                }
            }
        }
    }
    // Create pier support nodes under the first and last span columns, bonding them directly to
    // the deck's bottom layer. Supports have higher mass/volume to approximate solid columns.
    const supportIndices = [];
    const supportLinks = [];
    const supportArea = cellX * cellZ * areaScale * supportAreaFactor;
    __DEBUG_SUPPORT_AREA = supportArea; // expose for debug verification in addBond
    const supportMass = massPerDeckNode * 6.0;
    const supportVolume = cellX * cellZ * pierHeight;
    for (const ix of [0, spanSegments - 1]) {
        for (let iz = 0; iz < widthSegments; ++iz) {
            const baseIndex = indexAt(ix, 0, iz);
            const baseNode = nodes[baseIndex];
            const deckBottomY = baseNode.centroid.y - cellY * 0.5;
            const supportCentroid = vec3(baseNode.centroid.x, deckBottomY - pierHeight * 0.5, baseNode.centroid.z);
            const supportIndex = nodes.length;
            nodes.push({ centroid: supportCentroid, mass: supportMass, volume: supportVolume });
            supportIndices.push(supportIndex);
            supportLinks.push({ supportIndex, deckIndex: baseIndex });
            gridCoordinates[supportIndex] = { ix, iy: -1, iz };
            // Bond each support directly to the deck node directly above it
            addBond(baseIndex, supportIndex, supportArea);
        }
    }
    console.log('nodes:', nodes.length, nodes);
    console.log('bonds:', bonds.length);
    console.log('supportIndices:', supportIndices);
    // Optional: normalize total resisting area per axis to geometric cross-sections
    if (normalizeAreas && bonds.length) {
        const target = {
            x: deckThickness * deckWidth,
            y: span * deckWidth,
            z: span * deckThickness
        };
        const sum = { x: 0, y: 0, z: 0 };
        const pick = (n) => {
            const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
            return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
        };
        bonds.forEach((b) => { sum[pick(b.normal)] += b.area; });
        const scale = {
            x: sum.x > 0 ? target.x / sum.x : 1,
            y: sum.y > 0 ? target.y / sum.y : 1,
            z: sum.z > 0 ? target.z / sum.z : 1
        };
        bonds.forEach((b) => { b.area *= scale[pick(b.normal)]; });
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
        spacing: { x: cellX, y: cellY, z: cellZ },
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
