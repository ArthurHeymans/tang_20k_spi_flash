# Makefile for Tang Nano 20K SPI Flash Emulator
# Using open source toolchain: yosys + nextpnr-himbaechel + gowin_pack

# GW2AR-LV18QN88C8/I7 - Tang Nano 20K FPGA
# Note: The -18C suffix indicates the internal SDRAM variant
DEVICE = GW2AR-LV18QN88C8/I7
FAMILY = GW2A-18C

# Flash chip selection:
#   0 = Winbond W25Q64FV (8MB, default) - fits in available SDRAM
#   1 = Micron N25Q256A (32MB) - requires 4-byte addressing, exceeds SDRAM
FLASH_CHIP ?= 0

# Source files
VERILOG_FILES = \
	src/top.v \
	src/spi_trx.v \
	src/sdram.v \
	src/glue.v \
	src/uart.v \
	src/fifo.v \
	src/util.v \
	src/pll.v

# Output files
BUILD_DIR = build
TOP_MODULE = top
PROJ_NAME = spi_flash

# Constraints
CST_FILE = tangnano20k.cst

.PHONY: all clean prog flash tool

all: $(BUILD_DIR)/$(PROJ_NAME).fs

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Synthesize with yosys
$(BUILD_DIR)/$(PROJ_NAME).json: $(VERILOG_FILES) | $(BUILD_DIR)
	yosys -p "read_verilog -DFLASH_CHIP=$(FLASH_CHIP) $(VERILOG_FILES); synth_gowin -top $(TOP_MODULE) -json $@"

# Place and route with nextpnr-himbaechel (Gowin backend)
$(BUILD_DIR)/$(PROJ_NAME)_pnr.json: $(BUILD_DIR)/$(PROJ_NAME).json $(CST_FILE)
	nextpnr-himbaechel --json $< --write $@ --device $(DEVICE) --vopt family=$(FAMILY) --vopt cst=$(CST_FILE)

# Pack bitstream with gowin_pack (-c for compression)
$(BUILD_DIR)/$(PROJ_NAME).fs: $(BUILD_DIR)/$(PROJ_NAME)_pnr.json
	gowin_pack -c -d $(FAMILY) -o $@ $<

# Program the device (volatile - lost on power cycle)
prog: $(BUILD_DIR)/$(PROJ_NAME).fs
	openFPGALoader -b tangnano20k $<

# Program to flash (persistent)
flash: $(BUILD_DIR)/$(PROJ_NAME).fs
	openFPGALoader -b tangnano20k -f $<

clean:
	rm -rf $(BUILD_DIR)

# Show resource usage
stats: $(BUILD_DIR)/$(PROJ_NAME).json
	yosys -p "read_json $<; stat"

# Build the spi-flash-tool
tool:
	cargo build --release --manifest-path tool/Cargo.toml
	@echo "Tool built: tool/target/release/spi-flash-tool"
