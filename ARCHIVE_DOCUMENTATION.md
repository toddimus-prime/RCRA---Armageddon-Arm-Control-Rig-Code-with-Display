# RCRA Project Archive Documentation

**Date:** November 20, 2025  
**Maintained by:** Todd (toddimus-prime)

---

## Active Projects

### ✅ RCRA V1.1 - Oct 25 (PRIMARY)
**Status:** ACTIVE - Current production code  
**Location:** `c:\Users\manea\OneDrive\Documents\PlatformIO\Projects\RCRA V1.1 - Oct 25`  
**GitHub:** https://github.com/toddimus-prime/RCRA---Armageddon-Arm-Control-Rig-Code-with-Display

**Description:**  
Complete RCRA Display and Control System with:
- ST7789 TFT Display (320x170)
- 4x AS5600 Encoder support via I2C multiplexer
- CRSF Protocol for ELRS/ExpressLRS transmission
- FRAM I2C persistence for calibration data
- Full calibration menu system
- Real-time sensor visualization with color-gradient bars
- v1.4.8 stable release

**Hardware:** ESP32-S3-DevKitC-1 (8MB Flash + 8MB PSRAM)

---

## Archived Projects (OBSOLETE - DO NOT USE)

### ⚠️ Exo_Arm_TX_CRSF
**Status:** ARCHIVED (November 20, 2025)  
**Location:** `c:\Users\manea\Platform IO Projects\Exo_Arm_TX_CRSF`  
**Archive Notice:** `ARCHIVED_README.md` added to project root

**Reason for Archival:**  
This ESP-NOW transmitter code is superseded by the CRSF-based implementation in RCRA V1.1. The ESP-NOW protocol was replaced with CRSF for better compatibility with ELRS systems.

**DO NOT USE:** This code will cause confusion as it has no display support and uses an incompatible protocol.

---

### ⚠️ Armageddon-RCRA-Hand-Display-Board
**Status:** ARCHIVED (November 20, 2025)  
**Location:** `c:\Users\manea\OneDrive\Documents\PlatformIO\Projects\Armageddon-RCRA-Hand-Display-Board`  
**Archive Notice:** `ARCHIVED_README.md` added to project root

**Reason for Archival:**  
Early prototype of the hand display board that has been fully integrated into the comprehensive RCRA V1.1 project. All functionality has been merged and improved.

**DO NOT USE:** Incomplete implementation superseded by RCRA V1.1.

---

## Migration Notes

If you encounter references to archived projects:

1. **Exo_Arm_TX_CRSF** → Use **RCRA V1.1 - Oct 25**
   - ESP-NOW replaced with CRSF protocol
   - All transmitter functionality included
   - Added display and calibration features

2. **Armageddon-RCRA-Hand-Display-Board** → Use **RCRA V1.1 - Oct 25**
   - All display functionality integrated
   - Enhanced with full sensor suite
   - Complete calibration system added

---

## Upload Safety

To avoid accidentally uploading archived code:

1. **Always verify the project name** before uploading
2. **Check serial monitor output** after upload:
   - ✅ Should see: `=== RCRA Display with CRSF TX ===`
   - ❌ If you see ESP-NOW messages, wrong code is uploaded
3. **Close serial monitors** before uploading new code
4. **Check COM port** - ensure only one board is connected

---

## Project Maintenance

**Current Version:** v1.4.8  
**Last Updated:** November 20, 2025  
**Platform:** PlatformIO + Arduino ESP32  
**Framework:** Arduino

For questions or issues, refer to the main RCRA V1.1 - Oct 25 project.
