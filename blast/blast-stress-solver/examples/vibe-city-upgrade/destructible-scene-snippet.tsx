import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  applyAutoBondingToScenario,
  createDestructibleThreeBundle,
} from 'blast-stress-solver/three';
import { useFrame } from '@react-three/fiber';
import type {
  ScenarioDesc,
  DebrisCollisionMode,
  OptimizationMode,
} from 'blast-stress-solver/rapier';

// ── Physics configuration ────────────────────────────────────

type PhysicsConfig = {
  /** Collision mode for debris bodies (default: 'all') */
  debrisCollisionMode?: DebrisCollisionMode;
  /** Surface friction coefficient (default: 0.25) */
  friction?: number;
  /** Bounce coefficient (default: 0.0) */
  restitution?: number;
  /** Scale factor for contact forces fed into the stress solver (default: 30) */
  contactForceScale?: number;
  /** Destroy single-node bodies instead of creating physics bodies (default: false) */
  skipSingleBodies?: boolean;
};

type OptimizationConfig = {
  /** Small body damping mode (default: 'off') */
  smallBodyDampingMode?: OptimizationMode;
  /** Maximum collider count to be a "small body" for damping (default: 3) */
  smallBodyColliderThreshold?: number;
  /** Minimum linear damping for small bodies (default: 2) */
  minLinearDamping?: number;
  /** Minimum angular damping for small bodies (default: 2) */
  minAngularDamping?: number;
  /** Debris cleanup mode (default: 'always') */
  debrisCleanupMode?: OptimizationMode;
  /** Time-to-live for debris in ms (default: 10000) */
  debrisTtlMs?: number;
  /** Maximum collider count for a body to be classified as debris (default: 2) */
  maxCollidersForDebris?: number;
};

type DamageConfig = {
  /** Enable the damage system (default: true) */
  enabled?: boolean;
  /** Auto-detach destroyed nodes (default: true) */
  autoDetachOnDestroy?: boolean;
  /** Auto-cleanup physics for destroyed nodes (default: true) */
  autoCleanupPhysics?: boolean;
};

// ── Component props ──────────────────────────────────────────

type Props = {
  scenario: ScenarioDesc;
  /** Gravity in m/s² (default: -9.81) */
  gravity?: number;
  /** Material strength scale (default: 1) */
  materialScale?: number;
  autoBonding?: boolean;
  useBatchedMesh?: boolean;
  showStressDebug?: boolean;
  /** Physics configuration — collision mode, friction, etc. */
  physics?: PhysicsConfig;
  /** Optimization configuration — damping, debris cleanup, etc. */
  optimization?: OptimizationConfig;
  /** Damage system configuration */
  damage?: DamageConfig;
};

export function DestructibleSceneSnippet({
  scenario: inputScenario,
  gravity = -9.81,
  materialScale = 1,
  autoBonding = false,
  useBatchedMesh = true,
  showStressDebug = false,
  physics = {},
  optimization = {},
  damage = {},
}: Props) {
  const rootRef = useRef(new THREE.Group());
  const coreRef = useRef<Awaited<ReturnType<typeof buildDestructibleCore>> | null>(null);
  const visualsRef = useRef<ReturnType<typeof createDestructibleThreeBundle> | null>(null);

  // Destructure with defaults
  const {
    debrisCollisionMode = 'all',
    friction = 0.25,
    restitution = 0.0,
    contactForceScale = 30,
    skipSingleBodies = false,
  } = physics;

  const {
    smallBodyDampingMode = 'off',
    smallBodyColliderThreshold = 3,
    minLinearDamping = 2,
    minAngularDamping = 2,
    debrisCleanupMode = 'always',
    debrisTtlMs = 10000,
    maxCollidersForDebris = 2,
  } = optimization;

  const {
    enabled: damageEnabled = true,
    autoDetachOnDestroy = true,
    autoCleanupPhysics = true,
  } = damage;

  useEffect(() => {
    let disposed = false;

    (async () => {
      let scenario = inputScenario;
      if (autoBonding) {
        scenario = await applyAutoBondingToScenario(scenario);
      }

      const core = await buildDestructibleCore({
        scenario,
        gravity,
        materialScale,
        friction,
        restitution,
        contactForceScale,
        debrisCollisionMode,
        skipSingleBodies,
        damage: {
          enabled: damageEnabled,
          autoDetachOnDestroy,
          autoCleanupPhysics,
        },
        smallBodyDamping: {
          mode: smallBodyDampingMode,
          colliderCountThreshold: smallBodyColliderThreshold,
          minLinearDamping,
          minAngularDamping,
        },
        debrisCleanup: {
          mode: debrisCleanupMode,
          debrisTtlMs,
          maxCollidersForDebris,
        },
      });

      if (disposed) {
        core.dispose();
        return;
      }

      const visuals = createDestructibleThreeBundle({
        core,
        scenario,
        root: rootRef.current,
        useBatchedMesh,
        batchedMeshOptions: {
          enableBVH: false,
          bvhMargin: 5,
        },
        includeDebugLines: true,
      });

      coreRef.current = core;
      visualsRef.current = visuals;
    })();

    return () => {
      disposed = true;
      visualsRef.current?.dispose();
      visualsRef.current = null;
      coreRef.current?.dispose();
      coreRef.current = null;
    };
  }, [
    inputScenario, autoBonding, useBatchedMesh, gravity, materialScale,
    friction, restitution, contactForceScale, debrisCollisionMode,
    skipSingleBodies, damageEnabled, autoDetachOnDestroy, autoCleanupPhysics,
    smallBodyDampingMode, smallBodyColliderThreshold, minLinearDamping,
    minAngularDamping, debrisCleanupMode, debrisTtlMs, maxCollidersForDebris,
  ]);

  useFrame((_, delta) => {
    const core = coreRef.current;
    const visuals = visualsRef.current;
    if (!core || !visuals) return;

    core.step(Math.min(delta, 1 / 30));
    visuals.update({
      debug: showStressDebug,
      updateBVH: false,
      updateProjectiles: true,
    });
  });

  return <primitive object={rootRef.current} />;
}
