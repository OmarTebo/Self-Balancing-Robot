# Pre-compiled Binary Backup

This directory contains a backup of the compiled firmware before dead code cleanup.

**Tag:** `pre-dead-code-cleanup`  
**Date:** Created before dead code removal

## Files

- `firmware_pre-dead-code-cleanup.bin` - Main firmware binary
- `bootloader_pre-dead-code-cleanup.bin` - Bootloader binary
- `partitions_pre-dead-code-cleanup.bin` - Partition table binary
- `commit_hash.txt` - Git commit hash for this build

## How to Upload This Binary

### Method 1: Restore and Upload (Recommended)

1. Copy the firmware binary back to the build directory:
   ```powershell
   Copy-Item binaries\firmware_pre-dead-code-cleanup.bin .pio\build\esp32\firmware.bin
   ```

2. Upload using PlatformIO:
   ```bash
   pio run -e esp32 -t upload
   ```

### Method 2: Direct Upload with esptool

Use PlatformIO's esptool directly (this will upload without rebuilding):

```bash
pio run -e esp32 -t uploadfs
```

Or use esptool.py directly (you'll need to find the port):
```bash
pio run -e esp32 -t upload
# But first ensure the binary is in .pio/build/esp32/
```

### Method 3: Restore from Git Tag

If you want to restore the entire codebase to this state:

```bash
git checkout pre-dead-code-cleanup
pio run -e esp32 -t upload
```

Then rebuild the binary and upload.

## Notes

- The binary was compiled for environment: `esp32`
- Board: `esp32dev`
- To verify the tag: `git show pre-dead-code-cleanup`

