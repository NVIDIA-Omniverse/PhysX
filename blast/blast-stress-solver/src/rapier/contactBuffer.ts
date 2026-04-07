/**
 * Buffered contact replay system for rollback correctness.
 *
 * During the initial physics pass, contacts are recorded with their
 * already-computed effective magnitudes (speed-scaled, momentum-boosted).
 * During resimulation rollback, these buffered contacts are replayed
 * into the damage system instead of re-draining from Rapier, which
 * prevents double-counting damage.
 */
import type { DestructibleDamageSystem } from './damage';
import type { Vec3 } from './types';

export type BufferedExternalContact = {
  nodeIndex: number;
  effMag: number;
  dt: number;
  localPoint?: Vec3;
};

export type BufferedInternalContact = {
  nodeA: number;
  nodeB: number;
  effMag: number;
  dt: number;
  localPointA?: Vec3;
  localPointB?: Vec3;
};

/**
 * Manages buffered contacts for a single frame's rollback cycle.
 *
 * Usage:
 *  1. Call `clear()` at the start of each frame
 *  2. During initial drain, call `recordExternal()` / `recordInternal()`
 *  3. If rollback is needed, call `replay(damageSystem)` to apply
 *     the buffered contacts without re-draining from Rapier
 */
export class ContactBuffer {
  private readonly external: BufferedExternalContact[] = [];
  private readonly internal: BufferedInternalContact[] = [];

  /** Discard all buffered contacts (start of a new frame). */
  clear(): void {
    this.external.length = 0;
    this.internal.length = 0;
  }

  /** Record an external contact (structure ↔ ground/projectile). */
  recordExternal(contact: BufferedExternalContact): void {
    this.external.push(contact);
  }

  /** Record an internal contact (node ↔ node on same body). */
  recordInternal(contact: BufferedInternalContact): void {
    this.internal.push(contact);
  }

  /**
   * Replay all buffered contacts into the damage system.
   * Called during resimulation to restore damage state without
   * re-draining from Rapier (which would double-count contacts).
   */
  replay(damageSystem: DestructibleDamageSystem): void {
    if (!damageSystem.isEnabled()) return;
    for (const e of this.external) {
      damageSystem.onImpact(
        e.nodeIndex,
        e.effMag,
        e.dt,
        e.localPoint ? { localPoint: e.localPoint } : undefined,
      );
    }
    for (const i of this.internal) {
      damageSystem.onInternalImpact(
        i.nodeA,
        i.nodeB,
        i.effMag,
        i.dt,
        { localPointA: i.localPointA, localPointB: i.localPointB },
      );
    }
  }

  get externalCount(): number {
    return this.external.length;
  }

  get internalCount(): number {
    return this.internal.length;
  }
}
