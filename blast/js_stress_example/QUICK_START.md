# Bridge Stress Demo - Quick Start

## 30-Second Overview

The Bridge Stress Demo now has a full configuration system:

- **Change projectile properties** (mass, size, type) → Immediate effect on next spawn
- **Change gravity** → Immediate effect on physics
- **Change bridge config** (segments, materials) → Shows "⚠️ pending reset" indicator → Click "Reset Bridge"

## Quick Tests

### Test 1: Light vs Heavy Projectiles (2 minutes)

```
1. Drag "Ball Mass" slider to 1,000 kg
2. Click bridge to drop projectile → minimal damage
3. Drag to 30,000 kg
4. Click bridge to drop projectile → severe damage
5. Done! You just tested impact scaling
```

### Test 2: Change Gravity (1 minute)

```
1. Drag "Gravity" slider to -20 (stronger)
2. Click bridge to drop projectile → bridge bends more under its own weight
3. Drag to -5 (weaker)
4. Click bridge to drop projectile → bridge is more stable
```

### Test 3: Bridge Material Strength (3 minutes)

```
1. Find "Bond Area Scale" slider (requires reset)
2. Drag to 0.01 (weak material)
3. Click "Reset Bridge" button
4. Drop projectile → breaks very easily
5. Drag to 0.2 (strong material)
6. Click "Reset Bridge"
7. Drop projectile → much more durable
```

## UI Organization

```
┌─ Environment (⚙️ Immediate)
│  └─ Gravity
├─ Projectile (🎱 Immediate)
│  ├─ Type (ball/box)
│  ├─ Radius, Mass
│  ├─ Drop Height, Velocity
│  └─ Friction, Restitution
├─ Bridge Config (🌉 Deferred)
│  ├─ Span, Width, Thickness
│  ├─ Segments (X, Z, Y counts)
│  ├─ Deck Mass, Pier Height
│  └─ Bond Area Scale
├─ Solver Config (🧮 Deferred)
│  ├─ Max Iterations
│  └─ Stress Limits (compression, tension, shear)
└─ Actions
   ├─ Reset Bridge button
   ├─ Debug Wireframe toggle
   └─ ⚠️ Pending indicator
```

## Key Rules

| Category | Effect | Reset Needed? |
|----------|--------|:-------------:|
| **Gravity** | Immediate | ❌ No |
| **Projectile Type** | Next spawn | ❌ No |
| **Projectile Mass** | Next spawn | ❌ No |
| **Projectile Size** | Next spawn | ❌ No |
| **Drop Height** | Next spawn | ❌ No |
| **Bridge Geometry** | Rebuild structure | ✅ Yes |
| **Bridge Segments** | Rebuild structure | ✅ Yes |
| **Solver Limits** | Rebuild solver | ✅ Yes |

## Common Workflows

### Workflow A: Find the Right Projectile Mass

1. Leave bridge unchanged
2. Change projectile mass to minimum (1,000 kg)
3. Spawn → observe minimal damage
4. Increase mass gradually (5,000, 10,000, 15,000, 20,000 kg)
5. Find the mass that causes desired damage level

**Why this works**: Projectile changes apply immediately, no reset needed.

### Workflow B: Calibrate Bridge Strength

1. Set projectile to medium mass (15,000 kg)
2. Note bridge config (or keep defaults)
3. Adjust "Bond Area Scale" slider
4. Click "Reset Bridge"
5. Test with projectile
6. Adjust again if needed
7. Repeat until satisfied

**Why this works**: Bond area scale directly controls material strength.

### Workflow C: Test Different Segment Resolutions

1. Set "Span Segments" to 5 (coarse, fast)
2. Click "Reset Bridge"
3. Test fracture pattern
4. Set "Span Segments" to 20 (fine, slow)
5. Click "Reset Bridge"
6. Compare fracture patterns

**Why this works**: Segments control how detailed the fracture simulation is.

## Indicator Guide

| Indicator | Meaning | Action |
|-----------|---------|--------|
| ⚠️ Bridge changes pending reset | Bridge/solver config changed | Click "Reset Bridge" |
| (No indicator) | Everything is synced | Keep testing |
| Segments: 25 attached, 0 detached | Bridge is intact | Keep testing |
| Segments: 10 attached, 15 detached | Bridge is fractured | Keep testing or reset |

## Status Panel (Bottom Right)

```
Segments: 25 attached, 0 detached
Balls: 2
Rigidbodies: 27
Solver Actors: 1
Solver Nodes: 75
Overstressed Bonds: 5
Total Bonds: 120
Stress Error: lin=0.00001 ang=0.00002
```

- **Segments**: How many bridge pieces are still connected
- **Overstressed Bonds**: Number of bonds that want to break (will break on next update)
- **Stress Error**: How close the solver is to equilibrium (lower = converged)

## Advanced Tips

### Tip 1: Extreme Testing

```
Try: Gravity = -30, Mass = 50,000 kg, Bond Area = 0.01
Result: Bridge shatters instantly
```

### Tip 2: Stability Testing

```
Try: Gravity = -2, Mass = 1,000 kg, Bond Area = 0.5
Result: Bridge barely damaged even with heavy stress
```

### Tip 3: Fine-Tuning Limits

```
You'll want: Compression Fatal ≫ Elastic
Example: Elastic = 0.3, Fatal = 1.0
(Limits scale down from defaults)
```

## Troubleshooting

**Q: I changed a setting but nothing happened**
A: Check if it's in a "Deferred" section (🌉 🧮). If yes, click "Reset Bridge".

**Q: Bridge won't break at all**
A: Increase projectile mass and/or decrease Bond Area Scale.

**Q: Bridge breaks too easily**
A: Decrease projectile mass and/or increase Bond Area Scale.

**Q: Changes reverted**
A: You clicked "Reset Bridge" while having pending changes. Redo the changes.

## Next Steps

1. **Read CONFIG_GUIDE.md** for detailed parameter descriptions
2. **Read IMPLEMENTATION_SUMMARY.md** for technical details
3. **Experiment!** Try different combinations and see what breaks

---

**Happy testing! 🌉💥**

