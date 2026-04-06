import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  applyAutoBondingToScenario,
  createDestructibleThreeBundle,
} from 'blast-stress-solver/three';
import { useFrame } from '@react-three/fiber';
import type { ScenarioDesc } from 'blast-stress-solver/rapier';

type Props = {
  scenario: ScenarioDesc;
  autoBonding?: boolean;
  useBatchedMesh?: boolean;
  showStressDebug?: boolean;
};

export function DestructibleSceneSnippet({
  scenario: inputScenario,
  autoBonding = false,
  useBatchedMesh = true,
  showStressDebug = false,
}: Props) {
  const rootRef = useRef(new THREE.Group());
  const coreRef = useRef<Awaited<ReturnType<typeof buildDestructibleCore>> | null>(null);
  const visualsRef = useRef<ReturnType<typeof createDestructibleThreeBundle> | null>(null);

  useEffect(() => {
    let disposed = false;

    (async () => {
      let scenario = inputScenario;
      if (autoBonding) {
        scenario = await applyAutoBondingToScenario(scenario);
      }

      const core = await buildDestructibleCore({
        scenario,
        gravity: -9.81,
        materialScale: 1,
        debrisCollisionMode: 'noDebrisPairs',
        damage: {
          enabled: true,
          autoDetachOnDestroy: true,
          autoCleanupPhysics: true,
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
  }, [inputScenario, autoBonding, useBatchedMesh]);

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
