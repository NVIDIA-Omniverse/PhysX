# Reset Button Configuration Persistence Fix

## Problem
When clicking "Reset Bridge", the page would reload and **all configuration settings would revert to defaults**, losing all the user's parameter adjustments.

## Solution
Implemented **localStorage persistence** to save and restore configuration across page reloads.

## How It Works

### Before Reset (User Click)
```
User adjusts bridge parameters
    ↓
Indicator shows "⚠️ Bridge changes pending reset"
    ↓
User clicks "Reset Bridge" button
    ↓
Reset button handler saves config to localStorage
    ↓
Page reloads
```

### After Reload
```
Page loads (init())
    ↓
Check localStorage for saved config
    ↓
If found: Initialize ConfigManager with saved config
If not found: Initialize with defaults
    ↓
Build bridge with current config
    ↓
UI sliders reflect saved settings (not defaults)
    ↓
Clear the saved config from localStorage (consumed)
```

## Implementation Details

### Storage Functions (config.ts)

**`saveConfigToStorage(config: DemoConfig)`**
- Serializes current configuration to JSON
- Stores in localStorage under key `'bridgeStressConfig'`
- Safely handles storage errors

**`loadConfigFromStorage(): DemoConfig | null`**
- Retrieves saved configuration from localStorage
- Returns null if nothing saved or storage error occurs

**`clearConfigFromStorage()`**
- Removes saved configuration from localStorage
- Called after config is restored (one-time use)

### Reset Button Handler (split-bridge-stress.ts)

```typescript
resetButton.addEventListener('click', () => {
  // Save current configuration before reset
  if (state.configManager) {
    const currentConfig = state.configManager.getConfig();
    saveConfigToStorage(currentConfig);  // ← Store to localStorage
    state.configManager.confirmBridgeReset();
  }
  // Reload page - will restore config from localStorage
  location.reload();
});
```

### Initialization (split-bridge-stress.ts)

```typescript
async function init() {
  // Try to restore saved config first
  const savedConfig = loadConfigFromStorage();
  if (savedConfig) {
    console.log('🔄 Restoring configuration from previous reset...');
    state.configManager = new ConfigManager(savedConfig);
  } else {
    console.log('🆕 Initializing with default configuration');
    state.configManager = new ConfigManager(DEFAULT_CONFIG);
  }
  // ... rest of init
}
```

### Bridge Building (split-bridge-stress.ts)

```typescript
function buildBridge(scene, world) {
  // Get current config (which may be restored from previous reset)
  const currentConfig = state.configManager?.getConfig();
  const bridgeConfig = currentConfig?.bridge || DEFAULT_CONFIG.bridge;
  
  // Build bridge with current config
  const scenario = buildBridgeScenario(bridgeConfig);
  
  const core = createBridgeCore({ 
    runtime: state.runtime,
    world: world,
    scenario,
    gravity: currentConfig?.environment?.gravity,
    solverSettings: currentConfig?.solver  // ← Apply solver settings
  });
}
```

### Solver Settings Application (buildBridge.headless.ts)

```typescript
export function createBridgeCore({ 
  runtime, 
  world, 
  scenario, 
  gravity = -9.81, 
  strengthScale = 0.03, 
  solverSettings  // ← New parameter
}: CreateBridgeCoreOptions): BridgeCore {
  const settings = runtime.defaultExtSettings();
  
  if (solverSettings) {
    // Apply all solver config settings
    settings.maxSolverIterationsPerFrame = solverSettings.maxSolverIterationsPerFrame ?? 64;
    settings.compressionElasticLimit = solverSettings.compressionElasticLimit ?? 0.30;
    // ... apply all other settings
  } else {
    // Use defaults if no config provided
    settings.maxSolverIterationsPerFrame = 64;
    // ... defaults
  }
}
```

## Workflow Now

### Before (Broken)
```
1. Change "Span Segments" to 20
2. ⚠️ pending indicator shows
3. Click "Reset Bridge"
4. Page reloads
5. ❌ "Span Segments" reverts to 15 (default)
6. All other settings lost
```

### After (Fixed)
```
1. Change "Span Segments" to 20
2. ⚠️ pending indicator shows
3. Click "Reset Bridge"
4. Config saved to localStorage
5. Page reloads
6. ✅ Config restored from localStorage
7. "Span Segments" still shows 20
8. All settings preserved
9. Bridge built with all new parameters
10. ⚠️ indicator cleared (pending changes resolved)
```

## What Gets Preserved

✅ **Projectile settings** (mass, radius, type, velocity, etc.)  
✅ **Bridge geometry** (span, segments, width, thickness, etc.)  
✅ **Solver settings** (iterations, stress limits)  
✅ **Environment** (gravity)  
✅ **UI slider positions** (reflect saved config, not defaults)  

## What Resets

✅ **Fractured bridge** (rebuilt from scratch)  
✅ **Projectiles in scene** (cleared)  
✅ **Bridge status** (shows 0 detached segments)  
✅ **Physics bodies** (all recreated)  

## Edge Cases Handled

| Scenario | Behavior |
|----------|----------|
| First visit | Uses DEFAULT_CONFIG |
| After reset | Restores saved config |
| localStorage full | Falls back to defaults (logged) |
| Corrupt stored data | Falls back to defaults (logged) |
| Manual page reload | Uses defaults (not a reset action) |
| Browser close/reopen | Uses defaults (localStorage cleared) |

## Browser Compatibility

Works on all modern browsers supporting localStorage:
- Chrome/Edge ✅
- Firefox ✅
- Safari ✅
- Safari Private/Incognito ❌ (localStorage not available, falls back to defaults gracefully)

## Testing the Fix

### Test 1: Basic Reset Preservation
```
1. Change "Ball Mass" to 30,000
2. Change "Span Segments" to 20
3. Click "Reset Bridge"
4. ✅ Verify both settings preserved
```

### Test 2: Multiple Changes
```
1. Change gravity to -15
2. Change projectile type to "ball"
3. Change bond area scale to 0.1
4. Click "Reset Bridge"
5. ✅ Verify all three changes preserved
```

### Test 3: Fallback to Defaults
```
1. Don't change anything from defaults
2. Click "Reset Bridge"
3. ✅ See "🆕 Initializing with default configuration" in console
4. Bridge builds with defaults
```

### Test 4: Verify Bridge Rebuilds
```
1. Change "Span Segments" to 5
2. Click "Reset Bridge"
3. ✅ Verify bridge has fewer, coarser segments
4. Change to "Span Segments" to 30
5. Click "Reset Bridge"
6. ✅ Verify bridge has more, finer segments
```

## Files Modified

- `config.ts` - Added storage functions
- `split-bridge-stress.ts` - Updated reset handler and init, buildBridge
- `bridge/buildBridge.headless.ts` - Updated to accept and apply solver settings

## Console Messages

You'll see messages indicating what's happening:

```
🆕 Initializing with default configuration
  → First load, no saved config

🔄 Restoring configuration from previous reset...
  → Found saved config, applying it

⚠️ Bridge changes pending reset
  → You've made deferred changes
```

## Summary

Now when you click "Reset Bridge":
- Your configuration settings are **preserved**
- The bridge is **rebuilt** with your settings
- The page **reloads cleanly**
- Everything is **in sync** after the reset

**Try it: Change some parameters → Click Reset Bridge → Your settings stay!** 🎉

