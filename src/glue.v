// Glue Logic Module
// Handles serial protocol, SPI erase/page program, and SDRAM coordination

`default_nettype none

module glue(
    input wire clk,
    input wire reset,

    // UART interface
    input wire        rxd_strobe,
    input wire  [7:0] rxd_data,
    input wire        txd_ready,
    output reg        txd_strobe,
    output reg  [7:0] txd_data,

    // SDRAM control signals
    output reg  [1:0] sdram_access_cmd,    // 00=nop 01=read 10=write 11=activate
    output reg [23:0] sdram_access_addr,
    output reg        sdram_inhibit_refresh,
    input wire        sdram_cmd_busy,
    input wire [63:0] sdram_read_buffer,
    input wire        sdram_read_busy,
    output reg [63:0] sdram_write_buffer,

    // SPI signals
    input wire        spi_reset,
    input wire        spi_csel,
    input wire        spi_cmd_write,
    input wire  [1:0] spi_write_type,      // 0=page program, 1=sector/block erase, 2=chip erase
    input wire [21:0] spi_write_addr,
    input wire [12:0] spi_write_len,
    output reg        spi_write_done,
    input wire        spi_write_buf_strobe,
    input wire  [7:0] spi_write_buf_offset,
    input wire  [7:0] spi_write_buf_val,

    // Debug
    input wire        log_strobe,
    input wire  [7:0] log_val,
    output reg  [7:0] led
);

    // =========================================================================
    // Constants
    // =========================================================================

    // Serial protocol commands
    localparam CMD_NOP      = 8'h00;
    localparam CMD_VERSION  = 8'h30;
    localparam CMD_RAMREAD  = 8'h31;
    localparam CMD_RAMWRITE = 8'h32;
    localparam VERSION      = 8'h01;

    // SDRAM commands
    localparam SDRAM_NOP      = 2'b00;
    localparam SDRAM_READ     = 2'b01;
    localparam SDRAM_WRITE    = 2'b10;
    localparam SDRAM_ACTIVATE = 2'b11;

    // SPI write types
    localparam SPI_PAGE_PROGRAM = 2'd0;
    localparam SPI_SECTOR_ERASE = 2'd1;
    localparam SPI_CHIP_ERASE   = 2'd2;

    // =========================================================================
    // Registers
    // =========================================================================

    // Serial protocol state
    reg [7:0] cmd;
    reg [3:0] in_count;
    reg [21:0] addr;
    reg [7:0] len;

    // SDRAM read state machine
    reg [2:0] read_state;
    reg [2:0] read_pos;

    // SDRAM write state machine
    reg [2:0] write_state;
    reg [2:0] write_pos;
    reg [63:0] write_buffer;
    reg [7:0] write_mask;
    reg write_strobe;

    // TX/RX buffering
    reg txd_strobe_buf;
    reg [7:0] txd_data_buf;
    reg rxd_strobe_buf;
    reg [7:0] rxd_data_buf;

    // Derived busy signal
    wire sdram_busy = (sdram_access_cmd != 0) || sdram_cmd_busy;

    // Log handling
    reg [1:0] log_strobe_buf;
    reg log_ack;

    // SPI synchronization
    reg [1:0] spi_csel_buf;
    reg [1:0] spi_cmd_write_buf;
    reg [1:0] spi_write_buf_strobe_buf;

    // SPI write state
    reg spi_writing;
    reg spi_write_ack;
    reg spi_write_buf_ack;
    reg [1:0] i_spi_write_type;
    reg [2:0] i_spi_write_state;
    reg [12:0] i_spi_len;
    reg [19:0] i_chip_erase_count;  // 8MB = 1M x 8-byte units

    // Page program buffer (256 bytes + write flags)
    reg [8:0] i_spi_write_data [0:255];

    // Heartbeat
    reg [25:0] heartbeat;

    integer i;

    // =========================================================================
    // Synchronizers (active even during reset for clean startup)
    // =========================================================================

    always @(posedge clk) begin
        log_strobe_buf           <= {log_strobe_buf[0], log_strobe};
        spi_csel_buf             <= {spi_csel_buf[0], spi_csel};
        spi_cmd_write_buf        <= {spi_cmd_write_buf[0], spi_cmd_write};
        spi_write_buf_strobe_buf <= {spi_write_buf_strobe_buf[0], spi_write_buf_strobe};
    end

    // =========================================================================
    // Heartbeat counter
    // =========================================================================

    always @(posedge clk) begin
        if (reset)
            heartbeat <= 0;
        else
            heartbeat <= heartbeat + 1;
    end

    // =========================================================================
    // LED status indicators
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            led <= 0;
        end
        else begin
            led[7] <= !spi_reset && !spi_csel_buf[1];  // SPI active
            led[6] <= sdram_cmd_busy;
            led[5] <= spi_writing;
            led[4] <= spi_reset;
            led[3] <= !spi_csel_buf[1];                // CS low
            led[2] <= 0;
            led[1] <= 0;
            led[0] <= heartbeat[25];                   // ~2Hz heartbeat
        end
    end

    // =========================================================================
    // TX output register (one cycle delay for timing)
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            txd_strobe <= 0;
            txd_data   <= 0;
        end
        else begin
            txd_strobe <= txd_strobe_buf;
            txd_data   <= txd_data_buf;
        end
    end

    // =========================================================================
    // RX input register
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            rxd_strobe_buf <= 0;
            rxd_data_buf   <= 0;
        end
        else begin
            rxd_strobe_buf <= rxd_strobe;
            rxd_data_buf   <= rxd_data;
        end
    end

    // =========================================================================
    // SDRAM output registers
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            sdram_access_addr    <= 0;
            sdram_write_buffer   <= 0;
            sdram_inhibit_refresh <= 0;
        end
        else begin
            sdram_access_addr    <= {addr, 2'b0};
            sdram_write_buffer   <= write_buffer;
            sdram_inhibit_refresh <= 0;
        end
    end

    // =========================================================================
    // Log strobe handling
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            log_ack <= 0;
        end
        else begin
            if (!log_strobe_buf[1])
                log_ack <= 0;
            else if (!log_ack)
                log_ack <= 1;
        end
    end

    // =========================================================================
    // SPI write buffer ack handling
    // =========================================================================

    always @(posedge clk) begin
        if (reset)
            spi_write_buf_ack <= 0;
        else if (!spi_write_buf_strobe_buf[1])
            spi_write_buf_ack <= 0;
        else if (!spi_write_buf_ack)
            spi_write_buf_ack <= 1;
    end

    // =========================================================================
    // Main state machine
    // Handles: SDRAM commands, SPI writes, serial protocol
    // These are coupled through shared registers (addr, write_buffer, sdram_access_cmd)
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            cmd                <= CMD_NOP;
            in_count           <= 0;
            addr               <= 0;
            len                <= 0;
            read_state         <= 0;
            read_pos           <= 0;
            write_state        <= 0;
            write_pos          <= 0;
            write_buffer       <= 0;
            write_mask         <= 0;
            write_strobe       <= 0;
            sdram_access_cmd   <= SDRAM_NOP;
            txd_strobe_buf     <= 0;
            txd_data_buf       <= 0;
            spi_writing        <= 0;

            for (i = 0; i < 256; i = i + 1)
                i_spi_write_data[i] <= 0;
            spi_write_ack      <= 0;
            spi_write_done     <= 0;
            i_spi_write_type   <= 0;
            i_spi_write_state  <= 0;
            i_spi_len          <= 0;
            i_chip_erase_count <= 0;
        end
        else begin
            // Default: clear strobe, auto-clear SDRAM command
            txd_strobe_buf <= 0;

            if (sdram_access_cmd != SDRAM_NOP)
                sdram_access_cmd <= SDRAM_NOP;

            // -----------------------------------------------------------------
            // Log output (directly drives TX when active)
            // -----------------------------------------------------------------
            if (log_strobe_buf[1] && !log_ack) begin
                txd_strobe_buf <= 1;
                txd_data_buf   <= log_val;
            end

            // -----------------------------------------------------------------
            // SPI write buffer capture
            // -----------------------------------------------------------------
            if (spi_write_buf_strobe_buf[1] && !spi_write_buf_ack)
                i_spi_write_data[spi_write_buf_offset] <= {1'b1, spi_write_buf_val};

            // -----------------------------------------------------------------
            // SPI write command capture
            // -----------------------------------------------------------------
            if (!spi_cmd_write_buf[1])
                spi_write_ack <= 0;

            if (spi_cmd_write_buf[1] && !spi_write_ack && spi_csel_buf[1]) begin
                spi_writing      <= 1;
                spi_write_ack    <= 1;
                i_spi_write_type <= spi_write_type;
                addr             <= spi_write_addr;
                i_spi_len        <= spi_write_len;
                spi_write_done   <= 0;

                // State 0 = page program (read-modify-write), State 3 = erase (direct write)
                i_spi_write_state <= (spi_write_type != SPI_PAGE_PROGRAM) ? 3 : 0;

                if (spi_write_type == SPI_CHIP_ERASE)
                    i_chip_erase_count <= 20'hFFFFF;

                if (spi_write_type != SPI_PAGE_PROGRAM)
                    write_buffer <= 64'hFFFFFFFFFFFFFFFF;
            end

            // -----------------------------------------------------------------
            // SPI write state machine
            // -----------------------------------------------------------------
            if (spi_writing && !sdram_busy) begin
                case (i_spi_write_state)
                    3'd0: begin  // Activate for read (page program)
                        sdram_access_cmd <= SDRAM_ACTIVATE;
                        for (i = 0; i < 8; i = i + 1) begin
                            write_buffer[i*8 +: 8] <= i_spi_write_data[{addr[4:0], 3'b000} + i][7:0];
                            write_mask[i]          <= i_spi_write_data[{addr[4:0], 3'b000} + i][8];
                            i_spi_write_data[{addr[4:0], 3'b000} + i][8] <= 0;
                        end
                        i_spi_write_state <= 1;
                    end

                    3'd1: begin  // Issue read command
                        sdram_access_cmd  <= SDRAM_READ;
                        i_spi_write_state <= 6;
                    end

                    3'd6: begin  // Wait for read complete
                        if (!sdram_read_busy)
                            i_spi_write_state <= 7;
                    end

                    3'd7: begin  // Extra cycle for data stability
                        i_spi_write_state <= 2;
                    end

                    3'd2: begin  // Merge with existing data
                        for (i = 0; i < 8; i = i + 1)
                            if (!write_mask[i])
                                write_buffer[i*8 +: 8] <= sdram_read_buffer[i*8 +: 8];
                        i_spi_write_state <= 3;
                    end

                    3'd3: begin  // Activate for write
                        sdram_access_cmd  <= SDRAM_ACTIVATE;
                        i_spi_write_state <= 4;
                    end

                    3'd4: begin  // Issue write command
                        sdram_access_cmd  <= SDRAM_WRITE;
                        i_spi_write_state <= 5;
                    end

                    3'd5: begin  // Check completion / advance
                        if (i_spi_write_type == SPI_CHIP_ERASE) begin
                            if (i_chip_erase_count == 0) begin
                                spi_writing    <= 0;
                                spi_write_done <= 1;
                            end
                            else begin
                                i_spi_write_state  <= 3;
                                addr               <= addr + 1;
                                i_chip_erase_count <= i_chip_erase_count - 1;
                            end
                        end
                        else if (i_spi_len == 0) begin
                            spi_writing    <= 0;
                            spi_write_done <= 1;
                        end
                        else begin
                            i_spi_write_state <= (i_spi_write_type != SPI_PAGE_PROGRAM) ? 3 : 0;
                            addr              <= addr + 1;
                            i_spi_len         <= i_spi_len - 1;
                        end
                    end
                endcase
            end

            // -----------------------------------------------------------------
            // Serial protocol (only when SPI inactive)
            // -----------------------------------------------------------------
            if ((spi_reset || spi_csel_buf[1]) && !spi_writing) begin

                if (rxd_strobe_buf) begin
                    // --- Receiving bytes ---
                    if (in_count == 0) begin
                        case (rxd_data_buf)
                            CMD_VERSION: begin
                                txd_strobe_buf <= 1;
                                txd_data_buf   <= VERSION;
                            end
                            CMD_RAMREAD, CMD_RAMWRITE: begin
                                cmd         <= rxd_data_buf;
                                in_count    <= 1;
                                read_state  <= 0;
                                read_pos    <= 0;
                                write_state <= 0;
                                write_pos   <= 0;
                            end
                        endcase
                    end
                    else begin
                        if (in_count <= 3)
                            addr <= {addr[13:0], rxd_data_buf};
                        else if (in_count == 4)
                            len <= rxd_data_buf;

                        if (cmd == CMD_RAMREAD && in_count == 4)
                            read_state <= 1;

                        if (cmd == CMD_RAMWRITE && in_count > 4) begin
                            write_buffer[write_pos*8 +: 8] <= rxd_data_buf;
                            if (write_pos == 7)
                                write_strobe <= 1;
                            write_pos <= write_pos + 1;
                        end

                        if (in_count <= 4)
                            in_count <= in_count + 1;
                    end
                end
                else begin
                    // --- Not receiving: run state machines ---

                    if (write_strobe && !sdram_busy)
                        write_state <= 1;

                    // Read state machine
                    case (read_state)
                        3'd1: if (!sdram_busy) begin
                            sdram_access_cmd <= SDRAM_ACTIVATE;
                            read_state       <= 2;
                        end

                        3'd2: if (!sdram_busy) begin
                            sdram_access_cmd <= SDRAM_READ;
                            read_state       <= 3;
                        end

                        3'd3: if (!sdram_busy && txd_ready) begin
                            txd_strobe_buf <= 1;
                            txd_data_buf   <= sdram_read_buffer[read_pos*8 +: 8];

                            if (read_pos == 7) begin
                                if (len == 1) begin
                                    read_state <= 0;
                                    in_count   <= 0;
                                    cmd        <= CMD_NOP;
                                end
                                else begin
                                    addr       <= addr + 1;
                                    len        <= len - 1;
                                    read_state <= 1;
                                    read_pos   <= 0;
                                end
                            end
                            else begin
                                read_pos <= read_pos + 1;
                            end
                        end
                    endcase

                    // Write state machine
                    case (write_state)
                        3'd1: if (!sdram_busy) begin
                            sdram_access_cmd <= SDRAM_ACTIVATE;
                            write_strobe     <= 0;
                            write_state      <= 2;
                        end

                        3'd2: if (!sdram_busy) begin
                            sdram_access_cmd <= SDRAM_WRITE;
                            write_state      <= 3;
                        end

                        3'd3: if (!sdram_busy) begin
                            if (len == 1) begin
                                if (txd_ready) begin
                                    txd_strobe_buf <= 1;
                                    txd_data_buf   <= 8'h01;
                                    write_state    <= 0;
                                    in_count       <= 0;
                                    cmd            <= CMD_NOP;
                                end
                            end
                            else begin
                                write_state <= 0;
                                addr        <= addr + 1;
                                len         <= len - 1;
                            end
                        end
                    endcase
                end
            end
        end
    end

endmodule
