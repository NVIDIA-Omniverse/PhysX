/**
 * Pre-built fractured bridge scenario using Voronoi fracturing.
 *
 * Builds a beam bridge from a fractured deck slab, fractured support posts,
 * and simple cuboid footing supports. Each component is independently
 * fractured via three-pinata, then assembled into a single ScenarioDesc.
 *
 * Default geometry:
 * - 18m span, 5m wide deck, 0.6m thick
 * - 4 support posts per side at each end
 * - Posts sit on thin footing plates (mass=0 supports)
 *
 * Requires @dgreenheck/three-pinata as an optional peer dependency.
 */
import * as THREE from 'three';
import type { ScenarioDesc } from '../rapier/types';
import type { FragmentInfo } from '../three/fracture';
import type { AreaNormalizationMode } from '../three/scenarioFromFragments';
import {
  buildFloorFragments,
  buildColumnFragments,
} from '../three/fractureBuilders';
import {
  buildScenarioFromFragments,
  buildScenarioFromFragmentsAsync,
} from '../three/scenarioFromFragments';

export type FracturedBridgeOptions = {
  /** X length of deck in meters (default: 18) */
  span?: number;
  /** Z width of deck in meters (default: 5) */
  deckWidth?: number;
  /** Y thickness of deck in meters (default: 0.6) */
  deckThickness?: number;
  /** Distance from ground to deck bottom in meters (default: 2.8) */
  pierHeight?: number;
  /** Number of vertical posts at each end across Z (default: 4) */
  supportsPerSide?: number;
  /** Cross-section size of each support post in meters (default: 0.4) */
  postSize?: number;
  /** Thin foundation plate thickness in meters (default: 0.12) */
  footingThickness?: number;
  /** Number of Voronoi fragments for the deck (default: 40) */
  fragmentCountPerDeck?: number;
  /** Number of Voronoi fragments per post (default: 5) */
  fragmentCountPerPost?: number;
  /** Total mass distributed among deck blocks in kg (default: 60000) */
  deckMass?: number;
  /** Bond detection mode: 'proximity' (default) or 'auto' (WASM triangle-based) */
  bondMode?: 'proximity' | 'auto';
  /** Bond area normalization mode (default: 'perAxis') */
  areaNormalization?: AreaNormalizationMode;
  /** Rapier module for convex hull colliders. If omitted, colliderDescForNode entries are null. */
  rapier?: { ColliderDesc: { cuboid(hx: number, hy: number, hz: number): unknown; convexHull(points: Float32Array): unknown | null } };
};

/**
 * Build a fractured beam bridge scenario.
 *
 * The deck is a single fractured slab spanning the full bridge length.
 * Support posts at each end are independently fractured vertical columns.
 * Footings under each post are simple cuboid supports (mass=0).
 */
export async function buildFracturedBridgeScenario(
  options?: FracturedBridgeOptions,
): Promise<ScenarioDesc> {
  const {
    span = 18.0,
    deckWidth = 5.0,
    deckThickness = 0.6,
    pierHeight = 2.8,
    supportsPerSide = 4,
    postSize = 0.4,
    footingThickness = 0.12,
    fragmentCountPerDeck = 40,
    fragmentCountPerPost = 5,
    deckMass = 60_000,
    bondMode = 'proximity',
    areaNormalization = 'perAxis',
    rapier,
  } = options ?? {};

  const groundClearance = 0.001;
  const deckBottomY = groundClearance + footingThickness + pierHeight;
  const deckCenterY = deckBottomY + deckThickness * 0.5;

  const allFragments: FragmentInfo[] = [];

  // ── Deck slab ──────────────────────────────────────────────
  allFragments.push(...buildFloorFragments({
    spanX: span,
    spanZ: deckWidth,
    thickness: deckThickness,
    fragmentCount: fragmentCountPerDeck,
    centerX: 0,
    centerY: deckCenterY,
    centerZ: 0,
  }));

  // ── Support posts at each end ──────────────────────────────
  const halfSpan = span * 0.5;
  const halfWidth = deckWidth * 0.5;

  // Evenly distribute posts across deck width
  const postZPositions: number[] = [];
  for (let i = 0; i < supportsPerSide; i++) {
    const t = supportsPerSide === 1 ? 0.5 : i / (supportsPerSide - 1);
    postZPositions.push(-halfWidth + postSize * 0.5 + t * (deckWidth - postSize));
  }

  // Posts at X = -halfSpan and X = +halfSpan
  const postXPositions = [
    -halfSpan + postSize * 0.5,
    halfSpan - postSize * 0.5,
  ];

  for (const postX of postXPositions) {
    for (const postZ of postZPositions) {
      // Fractured support post
      allFragments.push(...buildColumnFragments({
        sizeX: postSize,
        sizeZ: postSize,
        height: pierHeight,
        fragmentCount: fragmentCountPerPost,
        centerX: postX,
        baseY: groundClearance + footingThickness,
        centerZ: postZ,
      }));

      // Footing support under each post (simple cuboid, mass=0)
      allFragments.push({
        worldPosition: {
          x: postX,
          y: groundClearance + footingThickness * 0.5,
          z: postZ,
        },
        halfExtents: {
          x: postSize * 0.5,
          y: footingThickness * 0.5,
          z: postSize * 0.5,
        },
        geometry: new THREE.BoxGeometry(postSize, footingThickness, postSize),
        isSupport: true,
        fragmentType: 'foundation',
      });
    }
  }

  // ── Build scenario ─────────────────────────────────────────
  const dims = {
    x: span,
    y: deckThickness + pierHeight + footingThickness,
    z: deckWidth,
  };

  const scenarioOptions = {
    totalMass: deckMass,
    bondMode: bondMode as 'proximity' | 'auto',
    areaNormalization,
    dimensions: dims,
    rapier,
  };

  const scenario = bondMode === 'auto'
    ? await buildScenarioFromFragmentsAsync(allFragments, scenarioOptions)
    : buildScenarioFromFragments(allFragments, scenarioOptions);

  // Store metadata
  scenario.parameters = {
    ...scenario.parameters,
    span, deckWidth, deckThickness, pierHeight,
    supportsPerSide, postSize, footingThickness,
    fragmentCountPerDeck, fragmentCountPerPost,
  };

  return scenario;
}
