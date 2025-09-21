# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FreeDMO is an STM32F103-based firmware that emulates DYMO 550 series label printer RFID tags, providing "endless freedom" for label printing by bypassing vendor lock-in. The firmware emulates SLIX2 RFID tags with various D.MO label format configurations.

## Build System

### Build Commands
- `make` - Compile the firmware (outputs `build/freedmo.bin`, `build/freedmo.hex`, `build/freedmo.elf`)
- `make clean` - Remove build artifacts
- `make DMO_SKU=<sku_name>` - Build with specific SKU (e.g., `make DMO_SKU=S0722430`)

### Toolchain Setup
The project uses ARM GCC toolchain (`arm-none-eabi-gcc`). Either:
1. Install from distribution packages (e.g., `gcc-arm-none-eabi` on Debian/Ubuntu)
2. Download from ARM and set `GCC_PATH` in Makefile to point to the `bin` folder

### Build Configuration
- Target: STM32F103C8T6 (Blue Pill board)
- Linker script: `STM32F103XX_FLASH.ld`
- Optimization: `-Os` (size optimized)
- The `DMO_SKU` parameter defines which label format to build as default

## Architecture

### Core Components

1. **RFID Tag Emulation**: The main purpose - emulates SLIX2 RFID tags for D.MO printers
2. **USB CDC Interface**: Provides serial communication for configuration and debugging
3. **I2C Communication**: Two I2C buses for communicating with printer (I2C1) and optional real RFID reader (I2C2)
4. **Flash Storage**: Persistent storage for selected SKU configuration

### Key Files Structure

- `Src/main.c` - Main application logic and hardware initialization
- `Src/dmo_data.h` - SKU definitions and RFID tag data structures
- `Src/usbd_cdc_if.c` - USB virtual COM port implementation
- `Inc/main.h` - Main header with GPIO pin definitions
- `freedmo.ioc` - STM32CubeMX project file for pin configuration

### Hardware Configuration

**Pin Assignments** (defined in `Inc/main.h` and `freedmo.ioc`):
- **I2C1** (PB6/PB7): Communication with printer main board
- **I2C2** (PB10/PB11): Communication with optional real RFID reader
- **GPIO**: Various control pins for power management and IRQ handling
- **USB**: Full-speed USB device for CDC communication

### Label Format System

The firmware contains multiple predefined label formats (`dmo_sku_t` structures):
- Each SKU defines: name, SLIX2 tag data, media data, and label count
- Default SKU selection can be changed via USB commands or flash storage
- Label counter decrements during emulation and resets on power cycle

### Data Flow

1. **Startup**: Load default SKU from flash or hardcoded fallback
2. **USB Communication**: Accept commands to change SKU configuration
3. **RFID Emulation**: Respond to printer I2C requests with emulated tag data
4. **Optional Real Tag Reading**: Update emulation data from physical RFID tags

## Development Workflow

### STM32CubeMX Integration
- Project uses STM32CubeMX for pin configuration (`freedmo.ioc`)
- All custom code is within `USER_CODE` sections, so CubeMX regeneration is safe
- Pin assignments and peripheral initialization are managed through CubeMX

### Flash Programming
The compiled `build/freedmo.bin` can be flashed via:
- **SWD**: Using ST-Link and official ST tools
- **UART**: Using USB bootloader and ST Flash Loader Demonstrator

### Configuration Management
- SKU selection persists in flash memory at `0x0800FC00`
- USB interface allows runtime SKU switching
- Build-time SKU selection via `DMO_SKU` make parameter

## Testing and Validation

### Functional Testing
Test the firmware by:
1. Connecting to USB CDC interface
2. Verifying SKU selection commands work
3. Testing with actual D.MO printer hardware
4. Verifying label counting behavior

### Hardware Requirements
- STM32F103C8T6 Blue Pill board (not C6T6 variant)
- 2x JST GH 6-pin cables for printer connections
- 2x 4.7kΩ pull-up resistors (optional, for RFID reader connection)

## Common Development Tasks

### Adding New Label Formats
1. Add new `dmo_sku_t` entry in `Src/dmo_data.h`
2. Update `dmo_skus[]` array and `dmo_skus_count`
3. Rebuild firmware with appropriate SKU selection

### Modifying Pin Configuration
1. Open `freedmo.ioc` in STM32CubeMX
2. Modify pin assignments as needed
3. Regenerate code (preserves USER_CODE sections)
4. Rebuild firmware

### Debugging Communication Issues
- Use USB CDC interface for debug output
- Monitor I2C communications with logic analyzer
- Check GPIO pin states and timing