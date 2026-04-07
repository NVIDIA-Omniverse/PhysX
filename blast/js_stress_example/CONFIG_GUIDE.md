# Bridge Stress Demo - Configuration Guide

## Overview

The Bridge Stress Demo features a comprehensive configuration system that allows you to test and iterate on destructible bridge parameters in real-time. Configuration changes are divided into two categories:

1. **Immediate Changes** - Applied instantly without reloading (projectile properties, gravity)
2. **Deferred Changes** - Require bridge reset to take effect (bridge geometry, solver settings)

## Configuration Categories

### 1. Environment (Immediate) ⚙️

**Gravity (m/s²)** `[-30, -0.5]` default: `-9.81`
- Controls the downward acceleration in the simulation
- Changes apply instantly to the physics simulation
- Affects how quickly projectiles fall and how much weight is on the bridge

### 2. Projectile (Immediate) 🎱

All projectile settings are **immediate** and apply to the next ball/box you spawn. No page reload needed.

**Type** `['ball', 'box']` default: `'box'`
- Toggle between spherical projectiles (ball) and cubic (box)
- Change anytime; next spawn uses the new type

**Radius (m)** `[0.1, 1.0]` default: `0.35`
- Size of the projectile (sphere radius or half-extent of box)
- Affects how it interacts with bridge geometry

**Mass (kg)** `[1,000, 50,000]` default: `15,000`
- Weight of the projectile
- Heavier projectiles cause more damage but move slower
- Change between shots to test damage scaling

**Drop Height (m)** `[2, 15]` default: `8`
- Initial Y position above the bridge
- Controls potential energy (speed at impact)
- Lower = less energy, higher = more destructive

**Initial Velocity (m/s)** `[-30, -1]` default: `-10`
- Downward velocity when spawned
- More negative = faster falling
- Combined with drop height to control impact energy

**Friction** `[0, 1]` default: `0.6`
- Projectile sliding friction
- Higher = more sliding resistance

**Restitution** `[0, 1]` default: `0.2`
- Bounce coefficient (0 = no bounce, 1 = perfect bounce)
- Lower = absorbs energy, higher = bounces higher

### 3. Bridge Configuration (Deferred) 🌉

**These settings require a bridge reset to take effect.** Changing them will show a pending indicator.

**Span (m)** `[10, 40]` default: `20`
- Total bridge length along the X axis
- Longer bridges have more mass to support and more nodes to compute

**Deck Width (m)** `[4, 16]` default: `8`
- Width of the bridge along the Z axis
- Affects stress distribution across the width

**Deck Thickness (m)** `[0.2, 2]` default: `0.6`
- How thick the bridge deck is (Y axis)
- Thicker = more material = more strength

**Span Segments** `[5, 30]` default: `15`
- Number of nodes distributed along the bridge length
- More segments = finer fracture resolution but higher computational cost
- Fewer segments = coarser but faster

**Width Segments** `[2, 10]` default: `5`
- Number of nodes across the bridge width
- Affects how the fracture spreads laterally

**Thickness Layers** `[1, 5]` default: `2`
- Number of layers through the deck thickness
- More layers = more complex internal structure

**Deck Mass (kg)** `[10,000, 200,000]` default: `60,000`
- Total mass of the bridge deck
- Affects gravitational stress on supports

**Pier Height (m)** `[1, 8]` default: `3`
- Height of the support piers below the deck
- Only affects visual representation; supports are fixed bodies

**Bond Area Scale** `[0.01, 0.5]` default: `0.05`
- Multiplier for cross-sectional area of bonds between nodes
- Lower = weaker bonds (easier to break)
- Higher = stronger bonds (more damage needed)

### 4. Solver Configuration (Deferred) 🧮

**These are advanced stress solver settings. Changes require reset.**

**Max Solver Iterations** `[1, 256]` default: `64`
- Maximum number of solver iterations per frame
- Higher = more accurate stress calculation but slower
- Lower = faster but may miss some stress

**Compression Elastic Limit (Pa)** `[100k, 10M]` default: `3M`
- Below this stress level, bonds can recover from compression damage
- Stress below this doesn't cause permanent damage

**Compression Fatal Limit (Pa)** `[500k, 50M]` default: `10M`
- Above this stress level, compression bonds immediately break
- The breaking threshold for squeeze forces

**Tension Elastic Limit (Pa)** `[10k, 1M]` default: `300k`
- Elastic threshold for tensile (pulling) stress

**Tension Fatal Limit (Pa)** `[100k, 5M]` default: `1M`
- Breaking threshold for tensile stress

**Shear Elastic Limit (Pa)** `[10k, 1M]` default: `400k`
- Elastic threshold for shear (sideways) stress

**Shear Fatal Limit (Pa)** `[100k, 5M]` default: `1.3M`
- Breaking threshold for shear stress

## Workflow: Iterative Testing

### Quick Testing Cycle

1. **Adjust projectile properties** (no reset needed)
   - Change mass, radius, drop height
   - Click on the bridge to spawn with new properties
   - See immediate results

2. **Adjust gravity** (no reset needed)
   - Change gravity value
   - Observe how it affects bridge stability and projectile motion

3. **Fine-tune bridge until satisfied**
   - Make bridge config or solver changes
   - See "⚠️ Bridge changes pending reset" indicator
   - Click "Reset Bridge" button to apply changes
   - Bridge reconstructs with new parameters

### Example Testing Scenarios

#### Scenario A: Light vs Heavy Impacts
1. Keep bridge config constant
2. Set projectile mass to minimum (1,000 kg), spawn ball
3. Gradually increase projectile mass (5,000, 10,000, 20,000 kg)
4. Observe how impact force scales with mass

#### Scenario B: Different Bridge Configurations
1. Note current configuration
2. Try different span segment counts (5, 10, 15, 20, 30)
3. Use same projectile to test each configuration
4. Compare robustness and fracture patterns

#### Scenario C: Material Strength Testing
1. Fix bridge geometry (span, segments, etc.)
2. Vary bond area scale (0.01, 0.05, 0.1, 0.2)
3. Keep projectile constant
4. Observe how material strength affects fracture behavior

#### Scenario D: Gravity Effects
1. Keep all other settings constant
2. Increase gravity from -9.81 to -15, -20, -30
3. Observe how increased gravity affects bridge stability under its own weight

## Technical Details

### Immediate vs Deferred Architecture

**Immediate Changes** (environment, projectile):
- Applied directly to the running simulation
- No bridge reconstruction needed
- Perfect for real-time testing and tweaking

**Deferred Changes** (bridge, solver):
- Queued for later application
- Require explicit bridge reset via button
- Trigger pending indicator when changed
- Prevents accidental performance issues from sudden rebuilds

### Configuration Manager

The `ConfigManager` class (in `config.ts`) handles:
- Tracking current configuration
- Identifying deferred vs immediate changes
- Providing pending change indicators
- Managing configuration state

### Value Formatting

The UI automatically:
- Formats large numbers with commas (e.g., "60,000")
- Rounds small decimals appropriately
- Updates live as you drag sliders

## Tips & Tricks

1. **Safe Experimentation**
   - Start with default values
   - Change one parameter at a time to isolate effects
   - Use pending indicator as your reminder to reset

2. **Performance**
   - Fewer segments = faster (5-10 segments is good for iteration)
   - More segments = better fracture detail but slower
   - Find the sweet spot for your testing needs

3. **Damage Calibration**
   - Use bond area scale to calibrate material strength
   - Start with 0.05, adjust up/down to get desired behavior
   - Fatal limits should always be > elastic limits

4. **Real-time Feedback**
   - Status panel shows segments, bonds, solver state
   - Watch "Overstressed Bonds" count to see fracture progression
   - Converged indicator shows solver stability

## Files

- `config.ts` - Configuration management and defaults
- `split-bridge-stress.ts` - Main demo loop with config integration
- `extBridgeScenario.js` - Bridge scenario builder
- `bridge/buildBridge.headless.ts` - Core physics bridge creation
- `bridge-split-demo.html` - UI with configuration controls
- `styles/bridge-demo.css` - Styling for configuration panels

## Reset Behavior

When you click "Reset Bridge":
1. Page reloads completely
2. Bridge is rebuilt with current configuration
3. All previously fractured segments are restored
4. Projectile history is cleared (no balls in scene)
5. Pending changes indicator is cleared
6. Configuration baseline is updated

