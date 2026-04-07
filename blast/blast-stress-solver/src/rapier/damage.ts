import type { ChunkData, ScenarioDesc } from './types';

export type DamageTickOptions = {
  preview?: boolean;
};

export type DamageStateSnapshot = {
  timeMs: number;
  pendingDamage: number[];
  health: number[];
  destroyed: boolean[];
  nextAllowedImpactTimeMs: number[];
};

export type DamageOptions = {
  enabled?: boolean;
  strengthPerVolume?: number;
  kImpact?: number;
  enableSupportsDamage?: boolean;
  contactDamageScale?: number;
  minImpulseThreshold?: number;
  contactCooldownMs?: number;
  internalContactScale?: number;
  // New: reduce mass immunity and improve collapse responsiveness
  massExponent?: number; // damage denominator uses mass^massExponent (was 1.0)
  internalMinImpulseThreshold?: number; // threshold specifically for self-contacts
  // Splash AOE around impact point (actor-local coordinates)
  splashRadius?: number; // meters
  splashFalloffExp?: number; // exponent for smooth falloff
  // Speed scaling controls (applied in core when mapping contact forces → damage)
  speedMinExternal?: number; // m/s threshold for external contacts where boost begins
  speedMinInternal?: number; // m/s threshold for internal contacts where boost begins
  speedMax?: number; // m/s at which boost fully applies
  speedExponent?: number; // curve exponent for boost falloff
  slowSpeedFactor?: number; // multiplier at/below vMin (<=1 suppresses resting)
  fastSpeedFactor?: number; // multiplier at/above vMax (>=1 boosts fast impacts)
};

export type DestroyCallback = (nodeIndex: number, reason: string) => void;

export class DestructibleDamageSystem {
  private chunks: ChunkData[];
  private nodes: ScenarioDesc['nodes'];
  private options: Required<DamageOptions>;
  private materialScale: number;
  private timeMs = 0;
  private nextAllowedImpactTimeMs: number[];
  private nodesForBody?: (
    bodyHandle: number,
  ) => Iterable<number> | undefined;

  constructor(args: {
    chunks: ChunkData[];
    scenario: ScenarioDesc;
    materialScale: number;
    options?: DamageOptions;
    nodesForBody?: (bodyHandle: number) => Iterable<number> | undefined;
  }) {
    const defaults: Required<DamageOptions> = {
      enabled: false,
      strengthPerVolume: 10000,
      kImpact: 0.002,
      enableSupportsDamage: false,
      contactDamageScale: 1.0,
      minImpulseThreshold: 50,
      contactCooldownMs: 120,
      internalContactScale: 0.5,
      massExponent: 0.5,
      internalMinImpulseThreshold: 15,
      splashRadius: 1.5,
      splashFalloffExp: 2.0,
      speedMinExternal: 0.5,
      speedMinInternal: 0.25,
      speedMax: 6.0,
      speedExponent: 1.0,
      slowSpeedFactor: 0.9,
      fastSpeedFactor: 3.0,
    };
    const opts = { ...defaults, ...(args.options ?? {}) };
    this.chunks = args.chunks;
    this.nodes = args.scenario.nodes;
    this.options = opts;
    this.materialScale = Math.max(1e-9, args.materialScale ?? 1);
    this.nextAllowedImpactTimeMs = new Array(this.chunks.length).fill(0);
    this.nodesForBody = args.nodesForBody;

    // Initialize per-node health if enabled
    if (opts.enabled) {
      for (let i = 0; i < this.chunks.length; i++) {
        const ch = this.chunks[i];
        const node = this.nodes[i];
        const vol = Math.max(1e-6, (node?.volume ?? 1));
        // const maxH = Math.max(1, opts.strengthPerVolume * vol * this.materialScale);
        const maxH = Math.max(1, opts.strengthPerVolume * vol);
        ch.maxHealth = maxH;
        ch.health = maxH;
        ch.pendingDamage = 0;
        ch.destroyed = false;
      }
    }
  }

  public isEnabled() {
    return !!this.options.enabled;
  }

  public getOptions() {
    return this.options;
  }

  public onImpact(
    nodeIndex: number,
    forceMagnitude: number,
    dt: number,
    opts?: { localPoint?: { x:number; y:number; z:number } }
  ) {
    if (!this.options.enabled) {
      return;
    }

    const ch = this.chunks[nodeIndex];
    if (!ch || ch.destroyed) {
      return;
    }

    const node = this.nodes[nodeIndex];
    const rawMass = node?.mass ?? 1;
    const isInfiniteMass = rawMass === 0; // supports mass=0 are infinite mass
    if (!this.options.enableSupportsDamage && isInfiniteMass) {
      // Ignore support impacts when support damage is disabled
    }

    const impulse = Math.max(0, forceMagnitude) * Math.max(0, dt);
    if (impulse < this.options.minImpulseThreshold) {
      return;
    }
    if (this.timeMs < (this.nextAllowedImpactTimeMs[nodeIndex] ?? 0)) {
      return;
    }
    const nodeMass = Math.max(1, rawMass);
    const denom = Math.max(1, Math.pow(nodeMass, this.options.massExponent));
    const dmgBase = this.options.kImpact * (impulse / denom);
    const dmgScaled = dmgBase * this.options.contactDamageScale;
    const dmg = Number.isFinite(dmgScaled) ? dmgScaled : 0;

    const lp = opts?.localPoint;
    const radius = Math.max(1e-6, this.options.splashRadius ?? 0);
    const exp = Math.max(0.01, this.options.splashFalloffExp ?? 1);
    const hitBody = this.chunks[nodeIndex]?.bodyHandle;

    if (lp && hitBody != null) {
      const targetNodes =
        typeof this.nodesForBody === 'function'
          ? this.nodesForBody(hitBody)
          : undefined;
      const iterateNodes = targetNodes ?? this.allNodeIndices();
      for (const i of iterateNodes) {
        const c = this.chunks[i];
        if (!c || c.destroyed) continue;
        if (c.bodyHandle !== hitBody) continue;
        const center = c.baseLocalOffset ?? { x: 0, y: 0, z: 0 };
        const dx = (center.x ?? 0) - (lp.x ?? 0);
        const dy = (center.y ?? 0) - (lp.y ?? 0);
        const dz = (center.z ?? 0) - (lp.z ?? 0);
        const d = Math.hypot(dx, dy, dz);
        let w = 0;
        if (i === nodeIndex) {
          w = 1;
        } else if (d <= radius) {
          w = Math.pow(Math.max(0, 1 - d / radius), exp);
        }
        if (w <= 0) continue;
        c.pendingDamage = (c.pendingDamage ?? 0) + dmg * w;
      }
      this.nextAllowedImpactTimeMs[nodeIndex] = this.timeMs + this.options.contactCooldownMs;
      return;
    }

    // Fallback: single-chunk damage if no impact point provided
    ch.pendingDamage = (ch.pendingDamage ?? 0) + dmg;
    this.nextAllowedImpactTimeMs[nodeIndex] = this.timeMs + this.options.contactCooldownMs;
  }

  public onInternalImpact(
    nodeA: number,
    nodeB: number,
    forceMagnitude: number,
    dt: number,
    opts?: { localPointA?: { x:number; y:number; z:number }; localPointB?: { x:number; y:number; z:number } }
  ) {
    if (!this.options.enabled) return;

    const impulse = Math.max(0, forceMagnitude) * Math.max(0, dt);
    const threshold = this.options.internalMinImpulseThreshold ?? this.options.minImpulseThreshold;
    if (impulse < threshold) return;
    const scale = this.options.internalContactScale;

    this.onImpact(nodeA, forceMagnitude * scale, dt, { localPoint: opts?.localPointA });
    this.onImpact(nodeB, forceMagnitude * scale, dt, { localPoint: opts?.localPointB });
  }

  public applyDirect(nodeIndex: number, amount: number) {
    if (!this.options.enabled) return;
    const ch = this.chunks[nodeIndex];
    if (!ch || ch.destroyed) return;
    ch.pendingDamage = (ch.pendingDamage ?? 0) + Math.max(0, amount);
  }

  public getHealth(nodeIndex: number) {
    const ch = this.chunks[nodeIndex];
    if (!ch || ch.maxHealth == null || ch.health == null) return null;
    return { health: ch.health, maxHealth: ch.maxHealth, destroyed: !!ch.destroyed };
  }

  public tick(_dt: number, onDestroyed?: DestroyCallback, options?: DamageTickOptions) {
    const destroyedNodes: number[] = [];
    if (!this.options.enabled) return destroyedNodes;
    const preview = !!options?.preview;
    const dtMs = Math.max(0, _dt) * 1000.0;
    if (!preview) this.timeMs += dtMs;
    for (let i = 0; i < this.chunks.length; i++) {
      const ch = this.chunks[i];
      const isSupport = this.chunks[i]?.isSupport;
      if (!ch) continue;
      if (ch.destroyed) continue;
      const maxH = ch.maxHealth ?? 0;
      if (!(maxH > 0)) continue;
      const dmg = ch.pendingDamage ?? 0;
      if (dmg <= 0) continue;
      const currentHealth = ch.health ?? maxH;
      const nextHealth = Math.max(0, currentHealth - dmg);
      if (!preview) {
        ch.health = nextHealth;
        ch.pendingDamage = 0;
      }
      if (nextHealth <= 0 && !ch.destroyed) {
        if (isSupport) {
          // Supports currently remain even when depleted
          continue;
        }
        destroyedNodes.push(i);
        if (!preview) {
          ch.destroyed = true;
          if (onDestroyed) onDestroyed(i, 'impact');
        }
      }
    }
    return destroyedNodes;
  }

  public previewTick(_dt: number): number[] {
    return this.tick(_dt, undefined, { preview: true }) ?? [];
  }

  public captureImpactState(): DamageStateSnapshot {
    const pendingDamage = this.chunks.map((ch) => ch?.pendingDamage ?? 0);
    const health = this.chunks.map((ch) => {
      if (!ch) return 0;
      if (typeof ch.health === 'number') return ch.health;
      if (typeof ch.maxHealth === 'number') return ch.maxHealth;
      return 0;
    });
    const destroyed = this.chunks.map((ch) => !!ch?.destroyed);
    return {
      timeMs: this.timeMs,
      pendingDamage,
      health,
      destroyed,
      nextAllowedImpactTimeMs: [...this.nextAllowedImpactTimeMs],
    };
  }

  public restoreImpactState(snapshot: DamageStateSnapshot | null | undefined) {
    if (!snapshot) return;
    this.timeMs = snapshot.timeMs;
    this.nextAllowedImpactTimeMs = [...snapshot.nextAllowedImpactTimeMs];
    for (let i = 0; i < this.chunks.length; i++) {
      const ch = this.chunks[i];
      if (!ch) continue;
      ch.pendingDamage = snapshot.pendingDamage[i] ?? 0;
      const restoredHealth = snapshot.health[i];
      if (typeof restoredHealth === 'number') {
        ch.health = restoredHealth;
      }
      ch.destroyed = snapshot.destroyed[i] ?? false;
    }
  }

  private *allNodeIndices() {
    for (let i = 0; i < this.chunks.length; i += 1) {
      yield i;
    }
  }

  public applyPreDestruction(nodes: number[]) {
    if (!Array.isArray(nodes) || nodes.length === 0) return;
    for (const idx of nodes) {
      const ch = this.chunks[idx];
      if (!ch) continue;
      ch.pendingDamage = 0;
      ch.health = 0;
    }
  }
}
