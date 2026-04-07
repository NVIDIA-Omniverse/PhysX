/**
 * Tests for the Three.js bundle helper and SolverDebugLinesHelper.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { SolverDebugLinesHelper } from '../three/solver-debug-lines';

describe('SolverDebugLinesHelper', () => {
  it('creates with a valid Three.js object', () => {
    const helper = new SolverDebugLinesHelper();
    expect(helper.object).toBeInstanceOf(THREE.Object3D);
  });

  it('update with empty lines does not crash', () => {
    const helper = new SolverDebugLinesHelper();
    helper.update([], []);
    // Should not throw
    expect(helper.object).toBeDefined();
  });

  it('update with debug lines creates line segments', () => {
    const helper = new SolverDebugLinesHelper();
    const lines = [
      { p0: { x: 0, y: 0, z: 0 }, p1: { x: 1, y: 0, z: 0 }, color0: 0xff0000, color1: 0x00ff00 },
      { p0: { x: 0, y: 0, z: 0 }, p1: { x: 0, y: 1, z: 0 }, color0: 0x0000ff, color1: 0xffff00 },
    ];
    const chunks = [
      { nodeIndex: 0, bodyHandle: null, active: true, baseLocalOffset: { x: 0, y: 0, z: 0 } },
    ];
    helper.update(lines as any, chunks as any);
    expect(helper.object).toBeDefined();
  });

  it('dispose cleans up resources', () => {
    const helper = new SolverDebugLinesHelper();
    helper.update(
      [{ p0: { x: 0, y: 0, z: 0 }, p1: { x: 1, y: 0, z: 0 }, color0: 0xff0000, color1: 0x00ff00 }] as any,
      [] as any,
    );
    helper.dispose();
    // Should not throw on second dispose
    helper.dispose();
  });
});
