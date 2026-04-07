# Stable Identity & Delta Tracking System

## Overview

This system provides human-readable names and comprehensive delta tracking for all Rapier physics objects in the stress-based bridge fracture demo. It solves the problem of tracking objects across handle changes when colliders are destroyed and recreated during splits.

## Features

### 1. Stable Metadata System

Every rigid body and collider gets persistent metadata:

```javascript
// Body metadata
{
  type: 'bridge-root' | 'bridge-fragment' | 'projectile',
  name: 'Bridge-Root-Fixed' | 'Bridge-Actor1-Body2 [Seg5,Seg6]' | 'Ball-3',
  actorIndex: 0,  // Blast stress solver actor index
  createdAt: 1234567890
}

// Collider metadata
{
  type: 'bridge-segment' | 'projectile',
  name: 'Segment-5' | 'Ball-3-Collider',
  segmentIndex: 5,    // Stable segment ID
  nodeIndex: 5,       // Stress solver node ID
  createdAt: 1234567890
}
```

### 2. Snapshot & Delta Tracking

Before any physics world mutations (creating/migrating bodies/colliders):
- **Captures snapshot** of current world state
- Indexes colliders by both handle AND stable ID (segment index)
- After mutations, **compares current vs previous** to detect:
  - ✨ **NEW** - newly created bodies/colliders
  - 🔄 **MOVED** - colliders that changed parent body
  - 🔄 **Recreated** - colliders with new handles (destroyed & recreated)
  - ❌ **REMOVED** - bodies that no longer exist

### 3. Stable Identity Across Handle Changes

When a collider is migrated to a new body:
1. Old collider (handle X) is disabled and queued for removal
2. New collider (handle Y) is created on target body
3. **Stable metadata is preserved**: name "Segment-5" stays "Segment-5"
4. **createdAt timestamp preserved** to track original creation time
5. Delta tracking shows: `🔄 MOVED from Bridge-Root-Fixed Segment-5 (Handle: 42, recreated, old handle: 6)`

This means you can track "Segment-5" across multiple handle changes as it moves between bodies.

## Usage

### Automatic Logging

The system automatically prints hierarchy after every split:

```javascript
// In applyPendingMigrations():
captureWorldSnapshot();  // Before mutations
// ... create bodies, migrate colliders ...
printWorldHierarchy();   // Show deltas
```

### Manual Debugging

Access debug tools from browser console:

```javascript
// Print current hierarchy
window.debugBridge.printHierarchy()

// Capture new baseline snapshot
window.debugBridge.captureSnapshot()

// Access raw state
window.debugBridge.state.bodyMetadata
window.debugBridge.state.colliderMetadata
```

## Example Output

```
🌍 WORLD HIERARCHY AFTER MIGRATIONS

📖 LEGEND:
  Bodies:    🏛️ = Root (fixed)  |  📦 = Dynamic  |  🔒 = Fixed
  Status:    ✨ NEW = Just created  |  ✅ = Existed before  |  🔄 MOVED = Changed parent
  Handles:   "(recreated, old handle: N)" = Collider was destroyed and recreated

📦 RIGID BODIES:

  🏛️ ✅ Bridge-Root-Fixed (Handle: 0)
  ├─ Type: Fixed
  ├─ Actor Index: 0
  ├─ Translation: (0.00, 0.50, 0.00)
  └─ Colliders: 20 total (0 added, 20 retained)
     ├─ ✅ Segment-0 (Handle: 1)
     │  ├─ Segment Index: 0
     │  ├─ Node Index: 0
     │  └─ Parent Body: Bridge-Root-Fixed
     ├─ ✅ Segment-1 (Handle: 2)
     ...

  📦 ✨ NEW Bridge-Actor1-Body1 [Seg25,Seg26,Seg27,Seg28,Seg29,Seg30,Seg31] (Handle: 5)
  ├─ Type: Dynamic
  ├─ Actor Index: 1
  ├─ Translation: (8.53, 0.48, 0.00)
  ├─ Linear Velocity: (0.12, -0.03, 0.00)
  ├─ Angular Velocity: (0.00, 0.00, 0.01)
  └─ Colliders: 7 total (7 added, 0 retained)
     ├─ 🔄 MOVED from Bridge-Root-Fixed Segment-25 (Handle: 39, recreated, old handle: 26)
     │  ├─ Segment Index: 25
     │  ├─ Node Index: 25
     │  └─ Parent Body: Bridge-Actor1-Body1 [Seg25,Seg26,Seg27,Seg28,Seg29,Seg30,Seg31]
     ├─ 🔄 MOVED from Bridge-Root-Fixed Segment-26 (Handle: 40, recreated, old handle: 27)
     ...

📊 SUMMARY:
├─ Total Actors: 2
├─ Rigid Bodies: 2 (1 new)
├─ Colliders: 32
├─   New Colliders: 0
├─   Moved Colliders: 7
└─ Removed Bodies: 0
```

## Implementation Details

### Key Data Structures

```javascript
state = {
  // Metadata storage (keyed by handle)
  bodyMetadata: Map<handle, {type, name, actorIndex, createdAt}>,
  colliderMetadata: Map<handle, {type, name, segmentIndex, nodeIndex, createdAt}>,
  
  // Snapshot for delta tracking
  previousWorldState: {
    bodies: Map<handle, info>,
    colliders: Map<handle, info>,
    collidersByStableId: Map<'segment-N', info>  // For tracking across handle changes
  },
  
  // ID counters for unique naming
  bodyIdCounter: 0,
  ballIdCounter: 0
}
```

### Metadata Registration Points

1. **Bridge root body** - at creation in `buildBridge()`
2. **Initial segment colliders** - at creation in `buildBridge()`
3. **Ball bodies/colliders** - at spawn in `spawnBallNow()`
4. **Fragment bodies** - at creation in `applyPendingMigrations()`
5. **Migrated colliders** - preserving stable names in `applyPendingMigrations()`

### Metadata Cleanup

Metadata is cleaned up when objects are removed:
- `removeDisabledHandles()` - removes metadata for disabled colliders/bodies
- `syncBallMeshes()` - removes metadata for balls that fall off world
- `applyPendingMigrations()` - removes old collider metadata before recreating

## Benefits

1. **Human-readable logs** - "Segment-5" instead of "Handle 42"
2. **Track object lifecycles** - see when objects were created, moved, recreated
3. **Debug splits easily** - understand exactly what changed during fracture
4. **Stable identity** - track colliders across handle changes
5. **Rich context** - know which actor/node each collider belongs to

## Future Enhancements

Possible additions:
- Visual overlay showing metadata in 3D view
- Timeline/history of all splits
- Export hierarchy to JSON for analysis
- Performance metrics (creation/destruction rates)
- Filter output by object type or actor index

