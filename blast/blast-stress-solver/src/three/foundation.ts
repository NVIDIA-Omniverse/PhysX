/**
 * Foundation / support plate generation for destructible structures.
 *
 * Generates support fragments (mass=0) that anchor a destructible object
 * to the ground plane, preventing the whole structure from falling through.
 */
import * as THREE from 'three';
import type { Vec3 } from '../rapier/types';
import type { FragmentInfo } from './fracture';

export type FoundationOptions = {
  /** Footprint dimensions of the object above (x = width, y = height, z = depth) */
  span: Vec3;
  /** Foundation height as a ratio of the object's height (default: 0.06) */
  heightRatio?: number;
  /** Maximum foundation height in world units (default: 0.08) */
  maxHeight?: number;
  /** Gap between foundation bottom and ground (default: 0.001) */
  groundClearance?: number;
  /** Number of segments along X. If omitted, auto-computed as max(6, round(span.x / 0.5)) */
  segments?: number;
  /** Foundation center X/Z offset (default: 0) */
  centerX?: number;
  centerZ?: number;
};

/**
 * Build a row of support fragments forming a thin foundation slab.
 * Each segment is a box geometry with `isSupport: true` (mass=0 in the solver).
 *
 * Returns the fragments and the Y coordinate of the foundation top surface,
 * which callers can use to position the structure above.
 */
export function buildFoundationFragments(
  options: FoundationOptions,
): { fragments: FragmentInfo[]; foundationTopY: number } {
  const {
    span,
    heightRatio = 0.06,
    maxHeight = 0.08,
    groundClearance = 0.001,
    centerX = 0,
    centerZ = 0,
  } = options;

  const foundationHeight = Math.min(maxHeight, span.y * heightRatio);
  const segments = options.segments ?? Math.max(6, Math.round(span.x / 0.5));
  const cellW = span.x / segments;
  const foundationTopY = groundClearance + foundationHeight;

  const fragments: FragmentInfo[] = [];
  for (let ix = 0; ix < segments; ix++) {
    const cx = centerX - span.x * 0.5 + cellW * (ix + 0.5);
    const cy = groundClearance + foundationHeight * 0.5;
    const geometry = new THREE.BoxGeometry(cellW, foundationHeight, span.z);

    fragments.push({
      worldPosition: { x: cx, y: cy, z: centerZ },
      halfExtents: { x: cellW * 0.5, y: foundationHeight * 0.5, z: span.z * 0.5 },
      geometry,
      isSupport: true,
    });
  }

  return { fragments, foundationTopY };
}

/**
 * Build a single support plate sized to a given footprint.
 * Useful for GLB models where a segmented foundation isn't needed.
 */
export function buildSingleFoundationPlate(
  options: Omit<FoundationOptions, 'segments'>,
): { fragment: FragmentInfo; foundationTopY: number } {
  const {
    span,
    heightRatio = 0.06,
    maxHeight = 0.08,
    groundClearance = 0.001,
    centerX = 0,
    centerZ = 0,
  } = options;

  const foundationHeight = Math.min(maxHeight, span.y * heightRatio);
  const foundationTopY = groundClearance + foundationHeight;
  const geometry = new THREE.BoxGeometry(span.x, foundationHeight, span.z);

  return {
    fragment: {
      worldPosition: { x: centerX, y: groundClearance + foundationHeight * 0.5, z: centerZ },
      halfExtents: { x: span.x * 0.5, y: foundationHeight * 0.5, z: span.z * 0.5 },
      geometry,
      isSupport: true,
    },
    foundationTopY,
  };
}
