// SDRAM Controller for GW2AR-18 Internal 64Mbit SDRAM
//
// The GW2AR-18 has internal 32-bit wide SDRAM (2M x 32 = 64Mbit)
// Organization: 4 banks, 2048 rows, 256 columns
// 
// Memory data is stored interleaved to optimize data retrieval for SPI FLASH emulation
// within a given 8-byte burst:
// * first byte of memory contains each data byte's bit 7
// * second byte contains each data byte's bit 6
// * and so on
// This ensures bit 7 is always received first from SDRAM regardless of start address

`default_nettype none

module sdram(
    input wire clk,
    input wire reset,

    // SDRAM physical interface
    output reg [1:0] ba_o,
    output reg [10:0] a_o,       // 11-bit address for GW2AR-18
    output reg cs_o,
    output reg ras_o,
    output reg cas_o,
    output reg we_o,
    output reg [31:0] dq_o,
    output reg [3:0] dqm_o,
    input wire [31:0] dq_i,
    output reg dq_oe_o,
    output reg cke_o,
    output wire sdram_clk_o,
    
    // Control signals from spi_trx
    input wire spi_inhibit_refresh,
    input wire spi_cmd_activate,
    input wire spi_cmd_read,
    input wire [21:0] spi_addr,

    // Control signals from glue
    input wire [1:0] access_cmd,    // 00=nop 01=read 10=write 11=activate
    input wire [23:0] access_addr,  // Access address
    input wire inhibit_refresh,
    output reg cmd_busy,

    output reg [63:0] read_buffer,
    output reg read_busy,

    input wire [63:0] write_buffer
);

    parameter CLK_FREQ_MHZ = 132;
    parameter BURST_LEN = 4;
    
    // SDRAM clock output
    assign sdram_clk_o = clk;

    // Timing parameters (in clock cycles at ~132MHz)
    localparam integer tINIT        = 100 * CLK_FREQ_MHZ;   // 100us init
    localparam integer tREFRESH     = (CLK_FREQ_MHZ * 32000) / 8192;  // ~516 cycles
    localparam integer tRP          = 3;   // 15ns precharge
    localparam integer tRC          = 9;   // 60ns row cycle
    localparam integer tMRD         = 2;   // 2 cycles mode register set
    localparam integer tRCD         = 3;   // 15ns RAS to CAS delay
    localparam integer tDPL         = 2;   // Write recovery
    localparam integer tRAS         = 6;   // 37ns row active time
    localparam integer tCAS         = 2;   // CAS latency = 2
    
    // Derived timing
    localparam integer tREAD  = tCAS + BURST_LEN + 1;
    localparam integer tWRITE = BURST_LEN + tDPL + tRP;

    // State machine states
    localparam
        STA_INIT            = 0,
        STA_INIT_PRECHARGE  = 1,
        STA_INIT_REFRESH    = 2,
        STA_IDLE            = 3,
        STA_SETMODE         = 4,
        STA_REFRESH         = 5,
        STA_ACTIVATE        = 6,
        STA_READ            = 7,
        STA_WRITE           = 8;

    // Burst mode encoding
    localparam [2:0] BURST_MODE =
        (BURST_LEN == 1) ? 3'b000 :
        (BURST_LEN == 2) ? 3'b001 :
        (BURST_LEN == 4) ? 3'b010 :
        (BURST_LEN == 8) ? 3'b011 :
        3'b111;

    reg [3:0] state;
    reg [$clog2(tINIT)-1:0] initcount;
    reg initrefreshcount;
    reg [4:0] cmdcount;
    reg [4:0] cmdtarget;
    reg [$clog2(tREFRESH):0] refreshcount;
    
    // Buffer control signals from SPI domain
    reg [1:0] spi_inhibit_refresh_buf;
    reg [1:0] spi_cmd_activate_buf;
    reg [1:0] spi_cmd_read_buf;
    
    reg spi_cmd_activate_ack;
    reg spi_cmd_read_ack;
    
    wire do_inhibit_refresh = (spi_inhibit_refresh_buf[1] || inhibit_refresh);
    
    // Address decoding for GW2AR-18 (2M x 32 = 64Mbit)
    // Row: 11 bits (2048 rows), Bank: 2 bits (4 banks), Column: 8 bits (256 columns)
    // For 8-byte burst access with 32-bit data bus:
    // Each burst read gives us 4 x 32-bit = 16 bytes
    // We use 8 bytes per logical access (interleaved)
    wire [10:0] spi_row = spi_addr[21:11];
    wire [1:0] spi_bank = spi_addr[10:9];
    wire [7:0] spi_col = {spi_addr[8:2], 1'b0};  // Column aligned

    wire [10:0] access_row = access_addr[21:11];
    wire [1:0] access_bank = access_addr[10:9];
    wire [7:0] access_col = access_addr[8:1];

    reg [3:0] readcount;
    reg [1:0] rdbuf_write_ptr;
    reg [1:0] wrbuf_read_ptr;

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            state <= STA_INIT;
            cs_o <= 1;
            ras_o <= 1;
            cas_o <= 1;
            we_o <= 1;
            ba_o <= 0;
            a_o <= 0;
            dq_oe_o <= 0;
            dq_o <= 0;
            dqm_o <= 4'b1111;
            cke_o <= 1;

            cmd_busy <= 1;

            initcount <= 0;
            initrefreshcount <= 0;
            cmdcount <= 0;
            cmdtarget <= 0;
            refreshcount <= 0;
            
            read_buffer <= 0;
            read_busy <= 0;
            readcount <= 0;

            rdbuf_write_ptr <= 0;
            wrbuf_read_ptr <= 0;
            
            spi_inhibit_refresh_buf <= 0;
            spi_cmd_activate_buf <= 0;
            spi_cmd_read_buf <= 0;
            
            spi_cmd_activate_ack <= 0;
            spi_cmd_read_ack <= 0;
        end
        else begin
            refreshcount <= refreshcount + 1;
            
            // Synchronize SPI control signals
            spi_inhibit_refresh_buf <= {spi_inhibit_refresh_buf[0], spi_inhibit_refresh};
            spi_cmd_activate_buf <= {spi_cmd_activate_buf[0], spi_cmd_activate};
            spi_cmd_read_buf <= {spi_cmd_read_buf[0], spi_cmd_read};
            
            if (spi_cmd_activate_ack && !spi_cmd_activate_buf[1]) spi_cmd_activate_ack <= 0;
            if (spi_cmd_read_ack && !spi_cmd_read_buf[1]) spi_cmd_read_ack <= 0;

            // Update busy flag
            cmd_busy <= (state <= STA_INIT_REFRESH) ||
                        ((state != STA_IDLE) && (cmdcount < cmdtarget-1)) ||
                        (access_cmd != 2'b00) ||
                        ((refreshcount >= tREFRESH-1) && !do_inhibit_refresh);

            if (state == STA_INIT) begin
                // Wait for SDRAM power-up (100us)
                if (initcount >= tINIT) begin
                    state <= STA_INIT_PRECHARGE;
                    cmdcount <= 1;
                    cmdtarget <= tRP;

                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;  // PRECHARGE
                    dqm_o <= 4'b1111;
                    a_o[10] <= 1; // All banks
                end
                else begin
                    initcount <= initcount + 1;
                    // NOP
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                end
            end
            else if ((state != STA_IDLE) && (cmdcount < cmdtarget)) begin
                // Waiting for command to complete
                // Issue NOP
                cs_o <= 0;
                ras_o <= 1;
                cas_o <= 1;
                we_o <= 1;

                if (state == STA_WRITE) begin
                    if (cmdcount < BURST_LEN) begin
                        // Feed write data (32-bit at a time)
                        dq_oe_o <= 1;
                        
                        // Interleaved data layout for 32-bit bus
                        for (i = 0; i < 8; i = i + 1) begin
                            dq_o[i*4+0] <= write_buffer[i*8 + 7 - wrbuf_read_ptr*2];
                            dq_o[i*4+1] <= write_buffer[i*8 + 6 - wrbuf_read_ptr*2];
                            dq_o[i*4+2] <= write_buffer[i*8 + 5 - wrbuf_read_ptr*2];
                            dq_o[i*4+3] <= write_buffer[i*8 + 4 - wrbuf_read_ptr*2];
                        end
                        dqm_o <= 4'b0000;

                        wrbuf_read_ptr <= wrbuf_read_ptr + 1;
                    end
                    else begin
                        dq_oe_o <= 0;
                        dqm_o <= 4'b1111;
                    end
                end

                cmdcount <= cmdcount + 1;
            end
            else begin
                // No command running, determine next command
                cmdcount <= 1;

                if (state == STA_INIT_PRECHARGE) begin
                    state <= STA_INIT_REFRESH;
                    cmdtarget <= tRC;
                    initrefreshcount <= 0;

                    // REFRESH command
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                end
                else if (state == STA_INIT_REFRESH) begin
                    if (initrefreshcount == 1) begin
                        state <= STA_SETMODE;
                        cmdtarget <= tMRD;
                        refreshcount <= 1;

                        // MODE REGISTER SET
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 0;
                        dqm_o <= 4'b1111;
                        ba_o <= 2'b00;          // Reserved
                        a_o[10] <= 1'b0;        // Reserved
                        a_o[9] <= 1'b0;         // Write burst: enabled
                        a_o[8:7] <= 2'b00;      // Operating mode
                        a_o[6:4] <= tCAS;       // CAS latency
                        a_o[3] <= 1'b0;         // Burst type: sequential
                        a_o[2:0] <= BURST_MODE; // Burst length
                    end
                    else begin
                        initrefreshcount <= 1;

                        // Another REFRESH
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 1;
                        dqm_o <= 4'b1111;
                    end
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack) begin
                    // SPI fast-path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    spi_cmd_activate_ack <= 1;

                    // ACTIVATE command
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 4'b1111;
                    ba_o <= spi_bank;
                    a_o <= spi_row;
                end
                else if (spi_cmd_read_buf[1] && !spi_cmd_read_ack) begin
                    // SPI fast-path read
                    state <= STA_READ;
                    cmdtarget <= tREAD;
                    read_busy <= 1;
                    spi_cmd_read_ack <= 1;

                    // READ command with auto-precharge
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= spi_bank;
                    a_o[7:0] <= spi_col;
                    a_o[10] <= 1; // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 4'b0000;
                end
                else if (access_cmd == 2'b11) begin
                    // Serial path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;

                    // ACTIVATE command
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 4'b1111;
                    ba_o <= access_bank;
                    a_o <= access_row;
                end
                else if (access_cmd == 2'b01) begin
                    // Serial path read
                    state <= STA_READ;
                    cmdtarget <= tREAD + 2;
                    read_busy <= 1;

                    // READ command with auto-precharge
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= access_bank;
                    a_o[7:0] <= access_col;
                    a_o[10] <= 1; // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 4'b0000;
                end
                else if (access_cmd == 2'b10) begin
                    // Serial path write
                    state <= STA_WRITE;
                    cmdtarget <= tWRITE;
                    wrbuf_read_ptr <= 1;

                    // WRITE command with auto-precharge
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 0;
                    ba_o <= access_bank;
                    a_o[7:0] <= access_col;
                    a_o[10] <= 1; // Auto precharge
                    dq_oe_o <= 1;
                    
                    // First write data word (interleaved)
                    for (i = 0; i < 8; i = i + 1) begin
                        dq_o[i*4+0] <= write_buffer[i*8 + 7];
                        dq_o[i*4+1] <= write_buffer[i*8 + 6];
                        dq_o[i*4+2] <= write_buffer[i*8 + 5];
                        dq_o[i*4+3] <= write_buffer[i*8 + 4];
                    end
                    dqm_o <= 4'b0000;
                end
                else if ((refreshcount >= tREFRESH) && !do_inhibit_refresh) begin
                    // Auto refresh
                    state <= STA_REFRESH;
                    cmdtarget <= tRC;
                    refreshcount <= 1;

                    // REFRESH command
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 4'b1111;
                end
                else begin
                    state <= STA_IDLE;
                    cs_o <= 1;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 4'b1111;
                end
            end
            
            // Read data capture (de-interleave 32-bit data to 64-bit buffer)
            if ((readcount > tCAS) && (readcount <= tCAS + BURST_LEN)) begin
                // Capture read data and de-interleave
                for (i = 0; i < 8; i = i + 1) begin
                    read_buffer[i*8 + 7 - rdbuf_write_ptr*2] <= dq_i[i*4+0];
                    read_buffer[i*8 + 6 - rdbuf_write_ptr*2] <= dq_i[i*4+1];
                    read_buffer[i*8 + 5 - rdbuf_write_ptr*2] <= dq_i[i*4+2];
                    read_buffer[i*8 + 4 - rdbuf_write_ptr*2] <= dq_i[i*4+3];
                end
                
                if (rdbuf_write_ptr == BURST_LEN - 1) read_busy <= 0;
                rdbuf_write_ptr <= rdbuf_write_ptr + 1;
            end
            else
                rdbuf_write_ptr <= 0;
            
            readcount <= (state == STA_READ) ? cmdcount : 0;
        end
    end

endmodule
