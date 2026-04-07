# Number Input Toggle Feature

## Overview

Each slider parameter now has a small toggle button (≡) that lets you switch between **slider mode** and **direct number input mode**.

## Why This Feature?

Sometimes you want:
- **Slider mode**: Fine-grained control within reasonable ranges ✓ (default)
- **Input mode**: Access to extreme or precise values beyond slider limits ✓ (click toggle)

Example: Set projectile mass to 1,000,000 kg for insane impacts, or 0.5 kg for feather-light testing.

## How to Use

### Switch to Input Mode
1. Locate the parameter (e.g., "Mass")
2. Click the small **≡** button at the end of the row
3. The slider is replaced with a number input field
4. Type any number you want (no min/max constraints)
5. The button turns blue/active to indicate input mode

### Switch Back to Slider Mode
- **Press Enter** to confirm and switch back
- **Click outside** (blur) to auto-switch back
- The button returns to normal (inactive) state

## Features

### Input Mode
✅ **No range limits** - Type any number you want  
✅ **Precise entry** - Direct typing is more accurate than slider dragging  
✅ **Visual feedback** - Button highlights when in input mode  
✅ **Quick confirm** - Press Enter to apply and switch back  
✅ **Auto-return** - Click elsewhere to exit input mode  

### Slider Mode (Default)
✅ **Easy adjustment** - Drag for quick approximate values  
✅ **Visual feedback** - See values as you drag  
✅ **Constrained ranges** - Prevents accidental invalid entries  
✅ **Touch-friendly** - Great for quick tweaks  

## Technical Details

### Toggle Button Styling
- Small square button (24x24px)
- Shows "≡" symbol (menu icon)
- Blue background when inactive
- Lighter blue when hovered
- Active state (bright blue) when in input mode

### Number Input Styling
- Matches the slider layout and color scheme
- Same height as slider (24px)
- Accepts any numeric value
- Focus highlights with blue glow
- Monospace font for clarity

### Behavior
1. **Click toggle** → Stores current value and slider range (min/max/step)
2. **User enters number** → Value updates config immediately
3. **User presses Enter or clicks away** → Reconstructs original slider with stored range
4. **Slider value is updated** → If number was valid, slider shows new value; display updates

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| **Enter** | Apply and switch back to slider mode |
| **Escape** | Revert and switch back to slider mode (planned) |
| **Tab** | Move to next field (auto-exits input mode on blur) |

## Examples

### Example 1: Extreme Projectile Mass
```
Default: Mass [====●] 15,000 kg
Click ≡: Mass [input: 15000]
Type: 1000000 (one million!)
Press Enter → Mass [====●] 1,000,000 kg
Bridge gets destroyed instantly! 💥
```

### Example 2: Precise Bridge Strength
```
Default: Bond Area [●====] 0.05
Click ≡: Bond Area [input: 0.05]
Type: 0.12345 (super precise!)
Press Enter → Bond Area [●====] 0.12
Fine-tune material properties exactly
```

### Example 3: Stress Testing Solver
```
Default: Max Iterations [==●] 64
Click ≡: Max Iterations [input: 64]
Type: 500 (stress test the solver)
Press Enter → Max Iterations [====●] 500
See if it can handle extreme stress
```

## Visual Changes

### Row with Toggle Button (Active)
```
Mass    [====●] 15,000 kg  ≡
        ↑        ↑        ↑
      slider   value    toggle button
```

### Row in Input Mode
```
Mass    [input: 15000]  ≡
        ↑                ↑
    number input    active toggle
    (focused)
```

## Implementation Details

### Files Modified
- `bridge-split-demo.html` - Added toggle button to each config row
- `styles/bridge-demo.css` - Added toggle button and number input styling
- `split-bridge-stress.ts` - Added toggle functionality logic

### CSS Classes
- `.config-toggle` - Toggle button styling
- `.config-toggle.active` - Active state styling
- `.config-toggle:hover` - Hover effect
- `.config-number-input` - Number input field styling

### JavaScript Logic
- `toggleInputMode(sliderId)` - Main toggle function
- Stores/restores min/max/step from data attributes
- Handles both slider→input and input→slider transitions
- Auto-saves on Enter or blur

## Edge Cases Handled

✅ **Rapid toggling** - Safely handles multiple quick toggles  
✅ **Invalid numbers** - Number input only accepts numeric values  
✅ **Large numbers** - No practical limits on input values  
✅ **Negative values** - Works with any sign  
✅ **Decimal precision** - Supports fractional values  
✅ **Copy/paste** - Works with pasted values  

## Performance

- **Zero overhead** - Toggle buttons have no performance impact
- **Lazy creation** - Number input only created when needed
- **DOM reuse** - Sliders/inputs are swapped, not duplicated
- **Event cleanup** - Old listeners are replaced with new ones

## Browser Compatibility

Works on all modern browsers that support:
- HTML5 `<input type="number">`
- CSS Flexbox
- JavaScript DOM manipulation
- Event listeners (standard)

## Future Enhancements (Ideas)

- [ ] Undo/redo history for number input changes
- [ ] Preset quick buttons for common extreme values
- [ ] Slider range expansion based on input value
- [ ] Auto-revert on Escape key
- [ ] Remember last used input mode per parameter
- [ ] Animation when switching modes

## Tips & Tricks

1. **Quick extreme testing** - Toggle input, type huge number, press Enter, see what happens!
2. **Precision tuning** - Use input mode for decimal places (0.123456)
3. **Parameter sweeps** - Test multiple extreme values in sequence
4. **Copy values** - Select all (Ctrl+A) and copy to try same value elsewhere
5. **Combination testing** - Use extreme values across multiple parameters

---

**The toggle button gives you both slick UI and complete control!** 🎚️➡️🔢

