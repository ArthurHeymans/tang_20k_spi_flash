// SPI Flash Transceiver Module
// Emulates SPI Flash chips:
//   - Winbond W25Q64FV (8MB, default)
//   - Micron N25Q256A (32MB, requires 4-byte addressing)
//
// Adapted for Gowin FPGAs - uses portable Verilog

`default_nettype none

module spi_trx #(
    // Flash chip selection:
    //   0 = Winbond W25Q64FV (8MB, 3-byte address only)
    //   1 = Micron N25Q256A (32MB, supports 4-byte address)
    parameter FLASH_CHIP = 0
)(
    input wire clk,

    input wire spi_clk,
    input wire spi_reset,    // Active high
    input wire spi_csel,     // Active low chip select
    input wire spi_mosi,
    output reg spi_miso = 0,
    output reg spi_miso_enable = 0,
    output reg spi_debug = 0,
    
    output wire spi_active,
    
    // SDRAM control signals
    output reg ram_inhibit_refresh = 0,
    output reg ram_activate = 0,
    output reg ram_read = 0,
    
    output reg [21:0] ram_addr,
    input wire [63:0] ram_read_buffer,
    input wire ram_read_busy,
    
    // For writing
    output reg write_cmd,
    output reg [1:0] write_type,  // 0=page program, 1=sector/block erase, 2=chip erase
    output reg [21:0] write_addr,
    output reg [12:0] write_len,
    input wire write_done,
    
    output reg write_buf_strobe,
    output reg [7:0] write_buf_offset,
    output reg [7:0] write_buf_val,
    
    output reg log_strobe = 0,
    output reg [7:0] log_val = 0
);

    wire is_selected = !spi_reset && !spi_csel;
    
    assign spi_active = is_selected;
    
    // Reset detection using async set flip-flops
    // These detect CS/power changes while SPI clock is idle
    reg reset_cs = 1;
    reg reset_power = 1;
    
    // CS reset detection - async preset when CS high, cleared on first clock when selected
    always @(posedge spi_clk or posedge spi_csel) begin
        if (spi_csel)
            reset_cs <= 1;
        else if (is_selected)
            reset_cs <= 0;
    end
    
    // Power reset detection - async preset on power reset, cleared on first clock when selected  
    always @(posedge spi_clk or posedge spi_reset) begin
        if (spi_reset)
            reset_power <= 1;
        else if (is_selected)
            reset_power <= 0;
    end

    reg [2:0] bit_count_in;
    reg [7:0] mosi_byte;
    reg [7:0] miso_byte;
    
    // JEDEC ID configuration based on flash chip selection
    // Byte order: [7:0]=manufacturer, [15:8]=type, [23:16]=capacity
    // Winbond W25Q64FV:  Manufacturer=EFh, Type=40h, Capacity=17h (8MB)
    // Micron N25Q256A:   Manufacturer=20h, Type=BAh, Capacity=19h (32MB)
    localparam [23:0] JEDEC_WINBOND = {8'h17, 8'h40, 8'hEF};
    localparam [23:0] JEDEC_MICRON  = {8'h19, 8'hBA, 8'h20};
    
    wire [23:0] jedec_id = (FLASH_CHIP == 0) ? JEDEC_WINBOND : JEDEC_MICRON;
    
    // 4-byte address mode is only supported on Micron N25Q256A
    wire supports_4byte = (FLASH_CHIP == 1);
    
    // SPI Flash command definitions
    localparam
        CMD_PAGEPROGRAM     = 8'h02,
        CMD_READ            = 8'h03,
        CMD_WRITEDISABLE    = 8'h04,
        CMD_READSTATUS      = 8'h05,
        CMD_WRITEENABLE     = 8'h06,
        CMD_FASTREAD        = 8'h0B,
        CMD_FASTREAD_4B     = 8'h0C,  // Fast Read with 4-byte address
        CMD_READ_4B         = 8'h13,  // Read with 4-byte address
        CMD_SECTORERASE_4K  = 8'h20,  // 4KB Sector Erase
        CMD_BLOCKERASE_32K  = 8'h52,  // 32KB Block Erase
        CMD_CHIPERASE1      = 8'h60,  // Chip Erase (alternate)
        CMD_READID1         = 8'h9E,
        CMD_READID2         = 8'h9F,
        CMD_4BYTEENABLE     = 8'hB7,
        CMD_CHIPERASE2      = 8'hC7,  // Chip Erase
        CMD_BLOCKERASE_64K  = 8'hD8,  // 64KB Block Erase
        CMD_4BYTEDISABLE    = 8'hE9,
        CMD_LOG             = 8'hF2;
    
    // State machine states
    localparam
        STA_CMD         = 0,  // Receiving command byte
        STA_READSTATUS  = 1,  // Reading status register
        STA_ADDR_READ   = 2,  // Receiving address for read command
        STA_READ        = 3,  // Sending data for read command
        STA_READID      = 4,  // Reading JEDEC ID
        STA_ADDR_WRITE  = 5,  // Receiving address for write command
        STA_WRITE       = 6,  // Receiving data for write command
        STA_ADDR_ERASE  = 7,  // Receiving address for erase command
        STA_ERASE       = 8,  // Erasing
        STA_LOG         = 9,  // Logging (passthrough to serial)
        STA_DUMMY       = 10; // Dummy clocks for fast read
    
    reg [3:0] state;
    reg [2:0] dummy_count = 0; // Counter for dummy clocks in fast read
    reg is_fast_read = 0;      // Flag to indicate fast read command
    
    reg [31:0] addr;
    reg [4:0] addr_count;
    reg addr_4byte;
    
    reg fresh_read = 0;
    
    // Status register
    // bit1 = write enable latch
    // bit0 = write in progress
    reg [7:0] status_reg = 8'b00000000;
    
    reg [1:0] write_done_buf;

    // Main SPI state machine - runs on positive edge of SPI clock
    always @(posedge spi_clk) begin
        if (is_selected) begin
            fresh_read <= 0;
            
            // Always update write_done synchronizer and clear busy when done
            // This must happen even during reset/command phase so status is correct
            write_done_buf <= {write_done_buf[0], write_done};
            if (status_reg[0] && write_done_buf[1])
                status_reg[0] <= 0;
            
            if (reset_cs || reset_power) begin
                // Starting a new command - reinitialize state
                bit_count_in <= 6;
                mosi_byte <= {spi_mosi, 7'b0};
                miso_byte <= 0;
                
                spi_miso_enable <= 0;
                
                state <= STA_CMD;
                
                addr <= 0;
                addr_count <= 0;
                dummy_count <= 0;
                is_fast_read <= 0;
                
                log_strobe <= 0;
                log_val <= 0;
                
                ram_inhibit_refresh <= 0;
                ram_activate <= 0;
                ram_read <= 0;
                
                write_cmd <= 0;
                
                write_buf_strobe <= 0;
                
                if (reset_power) begin
                    // If we received a power reset, reset some internal registers
                    status_reg[1:0] <= 2'b00;
                    addr_4byte <= 0;
                    
                    log_strobe <= 1;
                    log_val <= 8'hE2;
                end
            end
            else begin
                log_strobe <= 0;
                    
                write_buf_strobe <= 0;
                
                // Sample MOSI, advance bit count
                mosi_byte[bit_count_in] <= spi_mosi;

                if ((state == STA_CMD) && (bit_count_in == 0)) begin
                    // Received full command byte
                    
                    case ({mosi_byte[7:1], spi_mosi})
                        
                    CMD_READSTATUS: begin
                        state <= STA_READSTATUS;
                        spi_miso_enable <= 1;
                        miso_byte <= status_reg;
                    end
                    
                    CMD_WRITEDISABLE: begin
                        status_reg[1] <= 0;
                    end
                    
                    CMD_WRITEENABLE: begin
                        status_reg[1] <= 1;
                    end
                    
                    CMD_4BYTEENABLE: begin
                        if (status_reg[1] && supports_4byte) addr_4byte <= 1;
                    end
                    
                    CMD_4BYTEDISABLE: begin
                        if (status_reg[1] && supports_4byte) addr_4byte <= 0;
                    end
                        
                    CMD_READ: begin
                        state <= STA_ADDR_READ;
                        addr_count <= addr_4byte ? 31 : 23;
                    end
                    
                    CMD_READ_4B: begin
                        // 4-byte address read only supported on Micron
                        if (supports_4byte) begin
                            state <= STA_ADDR_READ;
                            addr_count <= 31;  // Always 4-byte address
                        end
                    end
                    
                    CMD_FASTREAD: begin
                        state <= STA_ADDR_READ;
                        addr_count <= addr_4byte ? 31 : 23;
                        is_fast_read <= 1;
                    end
                    
                    CMD_FASTREAD_4B: begin
                        // 4-byte address fast read only supported on Micron
                        if (supports_4byte) begin
                            state <= STA_ADDR_READ;
                            addr_count <= 31;  // Always 4-byte address
                            is_fast_read <= 1;
                        end
                    end

                    CMD_SECTORERASE_4K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 13'h01FF; // 4KB sector (512 x 8-byte units)
                        end
                    end
                    
                    CMD_BLOCKERASE_32K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 13'h0FFF; // 32KB block (4096 x 8-byte units)
                        end
                    end
                    
                    CMD_BLOCKERASE_64K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 13'h1FFF; // 64KB block (8192 x 8-byte units)
                        end
                    end
                    
                    CMD_CHIPERASE1,
                    CMD_CHIPERASE2: begin
                        if (status_reg[1]) begin
                            state <= STA_ERASE;
                            write_cmd <= 1;
                            write_type <= 2'd2;   // Chip erase
                            write_addr <= 22'b0;  // Start at address 0
                            write_len <= 13'h0;   // Not used for chip erase
                            status_reg[1] <= 0;   // Reset write enable
                            status_reg[0] <= 1;   // Write in progress
                        end
                    end
                    
                    CMD_PAGEPROGRAM: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_WRITE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 13'h1F; // 256 byte page
                        end
                    end
                    
                    CMD_READID1,
                    CMD_READID2: begin
                        state <= STA_READID;
                        spi_miso_enable <= 1;
                        miso_byte <= jedec_id[7:0];  // First byte: 0x20 (manufacturer)
                        addr_count <= 1;
                    end
                    
                    CMD_LOG: begin
                        state <= STA_LOG;
                    end
                        
                    endcase
                    
                    log_strobe <= 1;
                    log_val <= {mosi_byte[7:1], spi_mosi};
                end
                else if ((state == STA_READSTATUS) && (bit_count_in == 0)) begin
                    miso_byte <= status_reg;
                end
                else if (state == STA_ADDR_READ) begin
                    // Receiving address bytes for read
                    
                    if (addr_count == 7) begin
                        ram_inhibit_refresh <= 1;
                    end
                    else if (addr_count == 4) begin
                        ram_activate <= 1;
                        ram_addr[21:4] <= addr[24:7];
                    end
                    else if (addr_count == 3) begin
                        ram_read <= 1;
                        ram_addr[3:0] <= {addr[6:4], spi_mosi};
                    end
                    else if (addr_count == 0) begin
                        ram_inhibit_refresh <= 0;
                        ram_activate <= 0;
                        ram_read <= 0;

                        if (is_fast_read) begin
                            // Fast read: need 8 dummy clocks first
                            state <= STA_DUMMY;
                            dummy_count <= 7;
                        end
                        else begin
                            // Normal read: go straight to data
                            state <= STA_READ;
                            spi_miso_enable <= 1;
                            fresh_read <= 1;
                        end
                    end
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_mosi};
                    end
                    
                    addr[addr_count] <= spi_mosi;
                    addr_count <= addr_count - 1;
                end
                else if (state == STA_DUMMY) begin
                    // Dummy clock cycles for fast read command
                    // dummy_count counts down from 7 to 0 (8 clocks total)
                    if (dummy_count == 0) begin
                        state <= STA_READ;
                        spi_miso_enable <= 1;
                        fresh_read <= 1;
                    end
                    else begin
                        dummy_count <= dummy_count - 1;
                    end
                end
                else if (state == STA_READ) begin
                    // Advance the read
                    
                    if (addr[2:0] == 7) begin
                        // Reaching end of burst, start reading new one
                        
                        if (bit_count_in == 7) begin
                            ram_inhibit_refresh <= 1;
                        end
                        else if (bit_count_in == 4) begin
                            ram_activate <= 1;
                            ram_addr <= ram_addr + 1;
                        end
                        else if (bit_count_in == 3) begin
                            ram_read <= 1;
                        end
                        else if (bit_count_in == 0) begin
                            ram_inhibit_refresh <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                            
                            fresh_read <= 1;
                        end
                    end
                    
                    if (bit_count_in == 0) begin
                        miso_byte <= ram_read_buffer[(addr[2:0]+1)*8 +: 8];
                        addr <= addr + 1;
                    end
                    
                    if (fresh_read) 
                        miso_byte <= ram_read_buffer[addr[2:0]*8 +: 8];
                end
                else if (state == STA_READID) begin
                    if (bit_count_in == 0) begin
                        if (addr_count < 3) begin
                            miso_byte <= jedec_id[addr_count*8 +: 8];
                            addr_count <= addr_count + 1;
                        end
                        else
                            miso_byte <= 0;
                    end
                end
                else if (state == STA_ADDR_ERASE) begin
                    if (addr_count == 0) begin
                        state <= STA_ERASE;
                        write_cmd <= 1;
                        write_type <= 2'd1;  // Sector/block erase
                        
                        // Align address based on erase size (write_len in 8-byte units):
                        //   4KB  = 0x01FF (512 units)  -> align to 12 bits (addr[24:12])
                        //   32KB = 0x0FFF (4096 units) -> align to 15 bits (addr[24:15])
                        //   64KB = 0x1FFF (8192 units) -> align to 16 bits (addr[24:16])
                        if (write_len == 13'h1FFF)
                            write_addr <= {addr[24:16], 13'b0};  // 64KB aligned
                        else if (write_len == 13'h0FFF)
                            write_addr <= {addr[24:15], 12'b0};  // 32KB aligned
                        else
                            write_addr <= {addr[24:12], 9'b0};   // 4KB aligned
                            
                        status_reg[1] <= 0; // Reset write enable
                        status_reg[0] <= 1; // Write in progress
                    end
                    
                    addr[addr_count] <= spi_mosi;
                    addr_count <= addr_count - 1;
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_mosi};
                    end
                end
                else if (state == STA_ADDR_WRITE) begin
                    if (addr_count == 0) begin
                        state <= STA_WRITE;
                        write_cmd <= 1;
                        write_type <= 0;
                        
                        // Page-aligned address in 8-byte burst units
                        // Flash byte addr[24:0] -> RAM addr = byte_addr[24:3] (divide by 8)
                        // Page is 256 bytes = 32 x 8-byte bursts, so align to addr[24:8]
                        write_addr <= {addr[24:8], 5'b0};
                            
                        status_reg[1] <= 0; // Reset write enable
                        status_reg[0] <= 1; // Write in progress
                    end
                    
                    addr[addr_count] <= spi_mosi;
                    addr_count <= addr_count - 1;
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_mosi};
                    end
                end
                else if ((state == STA_WRITE) && (bit_count_in == 0)) begin
                    // Incoming data for write command
                    write_buf_strobe <= 1;
                    write_buf_offset <= addr[7:0];
                    write_buf_val <= {mosi_byte[7:1], spi_mosi};
                    
                    addr[7:0] <= addr[7:0] + 1;
                end
                else if (state == STA_LOG) begin
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_mosi};
                    end
                end
                
                bit_count_in <= bit_count_in - 1;
            end
        end
    end
    
    // MISO output on falling edge of SPI clock
    // SPI flash outputs data on falling edge, master samples on rising edge
    always @(negedge spi_clk) begin
        if (fresh_read)
            spi_miso <= ram_read_buffer[addr[2:0]*8 + 7];
        else
            spi_miso <= miso_byte[bit_count_in];
    end

endmodule
