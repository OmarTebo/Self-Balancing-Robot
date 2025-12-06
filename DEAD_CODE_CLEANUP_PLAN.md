# Dead Code Cleanup Plan

## Overview
This document outlines the plan for removing dead code identified in the codebase. All changes will be committed after verification.

## Dead Code Items Identified

### ✅ COMPLETED
1. **BNO055 support** - `beginBNO()` declaration removed from `include/IMU.h`
   - Status: ✅ Removed and committed

---

### 🔴 HIGH PRIORITY - Safe to Remove

#### 1. **Standalone `compute()` function in PIDController.h**
   - **Location**: `include/PIDController.h` line 7
   - **Issue**: Function declared but never implemented
   - **Impact**: None - only the class method `PIDController::compute()` is used
   - **Action**: Remove the standalone function declaration
   - **Verification**: 
     - Check that `PIDController::compute()` is still used in BotController.cpp
     - Verify no external code calls standalone `compute()`

#### 2. **`SerialCmdResult` struct in SerialBridge.h**
   - **Location**: `include/SerialBridge.h` lines 7-10
   - **Issue**: Struct defined but never used anywhere
   - **Impact**: None - struct is never referenced
   - **Action**: Remove the struct definition
   - **Verification**: 
     - Confirm no usage in SerialBridge.cpp or any other file

#### 3. **`beginMPU()` declaration in IMU.h**
   - **Location**: `include/IMU.h` line 26
   - **Issue**: Private method declared but never implemented
   - **Impact**: None - `begin()` directly initializes MPU6050
   - **Action**: Remove the declaration
   - **Verification**: 
     - Confirm `begin()` works directly with MPU6050 (already verified)
     - Ensure no code references `beginMPU()`

#### 4. **Unused display.h and display.cpp files**
   - **Location**: `include/display.h` and `src/display.cpp`
   - **Issue**: Files exist but are never included or used
   - **Impact**: None - BotController has its own display implementation
   - **Action**: Delete both files
   - **Verification**: 
     - Confirmed: No `#include "display.h"` in any source file (except display.cpp itself)
     - Confirmed: BotController implements its own display functionality
     - Confirmed: Test files use inline display code, not display.h

---

## Execution Plan

### Phase 1: Header File Cleanup (Low Risk)
1. Remove standalone `compute()` from PIDController.h
2. Remove `SerialCmdResult` struct from SerialBridge.h
3. Remove `beginMPU()` declaration from IMU.h

**Verification Steps:**
- Run linter to check for compilation errors
- Verify no broken references
- Test that MPU6050 still works (if possible)

**Commit**: "Remove dead code declarations from headers"

---

### Phase 2: File Removal (Medium Risk)
4. Delete `include/display.h`
5. Delete `src/display.cpp`

**Verification Steps:**
- Confirm files are not referenced in build system
- Check platformio.ini doesn't reference them
- Verify project still compiles

**Commit**: "Remove unused display.h and display.cpp files"

---

## Safety Checklist

Before each removal:
- [ ] Verify the code is truly unused (grep/search)
- [ ] Check that removal won't break compilation
- [ ] Ensure no external dependencies
- [ ] Verify core functionality (MPU6050, PID, motors) remains intact

After each removal:
- [ ] Run linter/compiler check
- [ ] Verify no broken references
- [ ] Commit changes with descriptive message

---

## Risk Assessment

| Item | Risk Level | Reason |
|------|-----------|--------|
| Standalone `compute()` | 🟢 Low | Never implemented, never called |
| `SerialCmdResult` struct | 🟢 Low | Never used anywhere |
| `beginMPU()` declaration | 🟢 Low | Never implemented, never called |
| `display.h`/`display.cpp` | 🟡 Medium | Files exist but unused - verify build system |

---

## Notes

- All changes will be committed per project preferences
- Binary backup exists at `binaries/` with tag `pre-dead-code-cleanup`
- MPU6050 functionality must remain completely unaffected
- BotController's display implementation is separate and should not be touched

---

## Estimated Impact

- **Lines of code removed**: ~100-150 lines
- **Files removed**: 2 (display.h, display.cpp)
- **Compilation time**: No change expected
- **Functionality**: No impact - all items are unused

