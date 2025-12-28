# Tang Nano 20K SPI Flash Emulator

FPGA-based SPI flash emulator for the Sipeed Tang Nano 20K board. Emulates a 32MB Micron N25Q256A SPI flash chip using the GW2AR-18's internal 64Mbit SDRAM.

This is a port of the [ULX3S SPI flash emulator](https://github.com/your-repo/spi_flash) to the Tang Nano 20K, built entirely with the open-source toolchain (Yosys + nextpnr-himbaechel + Apycula).

## Features

- Emulates a 32MB SPI flash (Micron N25Q256A compatible)
- Supports 3-byte and 4-byte addressing modes
- Handles SPI clock speeds up to 48 MHz
- 3 Mbaud UART interface for loading flash contents
- Uses internal 64Mbit SDRAM (no external memory needed)
- Supports READ, WRITE, ERASE, READ ID, and READ STATUS commands

## Hardware Requirements

- Sipeed Tang Nano 20K (GW2AR-LV18QN88C8/I7)
- USB-C cable for programming and UART communication

## Pin Connections

### SPI Interface

Connect these pins to your SPI master device:

| Signal | FPGA Pin | Board Location | Description |
|--------|----------|----------------|-------------|
| CS     | 73       | Edge connector | Chip Select (active low) |
| CLK    | 74       | Edge connector | SPI Clock |
| MOSI   | 75       | Edge connector | Master Out, Slave In |
| MISO   | 76       | Edge connector | Master In, Slave Out |
| POWER  | 77       | Edge connector | Power detection (active high) |
| DEBUG  | 27       | Edge connector | Debug output |

The POWER pin should be connected to the SPI master's power supply (via voltage divider if needed) to detect when the master is powered on. This enables proper reset sequencing.

### UART Interface

The UART is exposed via the onboard BL616 USB debugger:

| Signal | FPGA Pin | Description |
|--------|----------|-------------|
| TX     | 69       | FPGA transmit (to host) |
| RX     | 70       | FPGA receive (from host) |

Settings: **3,000,000 baud**, 8N1 (8 data bits, no parity, 1 stop bit)

### Other Pins

| Signal | FPGA Pin | Description |
|--------|----------|-------------|
| LED[0:5] | 15-20  | Status LEDs (active low) |
| BTN S1 | 88       | User button 1 |
| BTN S2 | 87       | User button 2 |

## Building

### Prerequisites

Install the open-source FPGA toolchain:
- [Yosys](https://github.com/YosysHQ/yosys) - Verilog synthesis
- [nextpnr-himbaechel](https://github.com/YosysHQ/nextpnr) - Place and route with Gowin support
- [Apycula](https://github.com/YosysHQ/apicula) - Gowin bitstream tools (gowin_pack)
- [openFPGALoader](https://github.com/trabucayre/openFPGALoader) - FPGA programming

Or use the provided Nix flake:

```bash
nix develop
```

### Build Commands

```bash
make          # Build bitstream
make prog     # Program FPGA (volatile - lost on power cycle)
make flash    # Program to flash (persistent)
make tool     # Build spi-flash-tool
make clean    # Clean build artifacts
```

## Loading Flash Contents

Use the serial interface to load data into the emulated flash before connecting to the SPI master.

### Using spi-flash-tool (Recommended)

The `spi-flash-tool` provides a convenient CLI for interacting with the flash emulator:

```bash
# Build the tool
make tool

# List available serial ports
./tool/target/release/spi-flash-tool ports

# Check connection and protocol version
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 version

# Load a firmware image
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 load firmware.bin

# Load with verification
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 load -v firmware.bin

# Load to a specific address
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 load -a 0x10000 firmware.bin

# Read and display memory (hex dump)
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 read 0x0 256

# Read memory to file
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 read 0x0 0x100000 -o dump.bin

# Dump memory to file
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 dump -a 0x0 -l 0x100000 dump.bin

# Write hex data directly
./tool/target/release/spi-flash-tool -p /dev/ttyUSB0 write 0x0 "deadbeefcafebabe"
```

#### Tool Commands

| Command | Description |
|---------|-------------|
| `version` | Get protocol version from device |
| `load <file>` | Load a file into flash memory |
| `read <addr> <len>` | Read memory and display/save |
| `write <addr> <hex>` | Write hex data to memory |
| `dump <file>` | Dump memory region to file |
| `ports` | List available serial ports |

#### Tool Options

| Option | Description |
|--------|-------------|
| `-p, --port <PORT>` | Serial port (default: `/dev/ttyUSB0`) |
| `-a, --address <ADDR>` | Start address (hex or decimal) |
| `-v, --verify` | Verify after writing (for `load`) |
| `-o, --output <FILE>` | Output file (for `read`) |
| `-l, --length <LEN>` | Length in bytes (for `dump`) |

### Serial Protocol (Low-Level)

The UART runs at 3 Mbaud (8N1). The following commands are available:

| Command | Description |
|---------|-------------|
| `0x30`  | Get protocol version (returns `0x01`) |
| `0x31`  | Read data from SDRAM |
| `0x32`  | Write data to SDRAM |

#### Read/Write Command Format

Both read and write commands are followed by 4 bytes:
- Bytes 0-2: Address (MSB first, in 8-byte units)
- Byte 3: Length (in 8-byte units, 0 = 8 bytes, 255 = 2048 bytes)

**Read**: Returns the requested data immediately after the command.

**Write**: Send the data after the command bytes. A `0x01` byte is returned when the write completes.

### Example: Loading with Python

```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 3000000, timeout=1)

def write_block(addr, data):
    """Write up to 2048 bytes at addr (must be 8-byte aligned)"""
    addr_units = addr // 8
    len_units = (len(data) // 8) - 1
    cmd = bytes([0x32, 
                 (addr_units >> 16) & 0xFF,
                 (addr_units >> 8) & 0xFF,
                 addr_units & 0xFF,
                 len_units])
    ser.write(cmd)
    ser.write(data)
    ser.read(1)  # Wait for completion byte

# Load firmware.bin
with open('firmware.bin', 'rb') as f:
    data = f.read()

for offset in range(0, len(data), 2048):
    block = data[offset:offset+2048]
    if len(block) % 8:
        block += b'\xFF' * (8 - len(block) % 8)
    write_block(offset, block)
    print(f"Wrote {offset + len(block)} / {len(data)} bytes")

ser.close()
```

## Supported SPI Commands

| Command | Code | Description |
|---------|------|-------------|
| READ | 0x03 | Read data (3-byte address) |
| READ4 | 0x13 | Read data (4-byte address) |
| FAST_READ | 0x0B | Fast read with dummy byte |
| READ_ID | 0x9F | Read JEDEC ID |
| READ_STATUS | 0x05 | Read status register |
| WRITE_ENABLE | 0x06 | Enable writes |
| PAGE_PROGRAM | 0x02 | Program page (256 bytes) |
| SECTOR_ERASE | 0x20 | Erase 4KB sector |
| BLOCK_ERASE | 0xD8 | Erase 64KB block |
| EN4B | 0xB7 | Enter 4-byte address mode |
| EX4B | 0xE9 | Exit 4-byte address mode |

## Technical Details

- **Clock**: 27 MHz input, PLL generates ~132 MHz for SDRAM
- **SDRAM**: Internal 64Mbit (8MB) with 32-bit data bus
- **Storage**: Data is interleaved in SDRAM for optimized SPI read timing
- **Timing**: Main clock 167 MHz max, SPI clock 198 MHz max (both pass timing)

### Module Overview

| File | Description |
|------|-------------|
| `top.v` | Top-level module with I/O and interconnects |
| `spi_trx.v` | SPI transceiver and command decoder |
| `sdram.v` | SDRAM controller (adapted for 32-bit internal SDRAM) |
| `glue.v` | Serial protocol handler and write buffer |
| `uart.v` | UART transmitter/receiver |
| `fifo.v` | FIFO buffer for UART |
| `pll.v` | PLL configuration (27 MHz -> 132 MHz) |
| `util.v` | Utility modules |

## Usage Notes

1. **Load before connecting**: Load flash contents via UART before powering on the SPI master
2. **Power sequencing**: The POWER pin detects when the SPI master is powered, triggering proper initialization
3. **Serial conflicts**: Do not use the serial interface while the SPI bus is active
4. **Voltage levels**: All I/O is 3.3V LVCMOS

## Differences from ULX3S Version

- Uses internal 32-bit SDRAM instead of external 16-bit SDRAM
- PLL adapted for 27 MHz input (was 25 MHz)
- Removed ECP5-specific IO primitives for portability
- Pin assignments for Tang Nano 20K edge connector

## License

See original project for license information.

## Credits

Based on the ULX3S SPI flash emulator, which was originally inspired by [spispy](https://github.com/osresearch/spispy/).
