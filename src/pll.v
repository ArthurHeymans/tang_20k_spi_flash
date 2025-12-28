// PLL Configuration for Tang Nano 20K (GW2AR-18)
// Input: 27MHz crystal oscillator
// Output: 133.333MHz for SDRAM operation
//
// Using Gowin rPLL primitive for open-source tooling compatibility
// 
// Calculation:
// Fout = Fclkin * FBDIV / IDIV / ODIV
// 133.333MHz = 27MHz * 59 / 3 / 4 = 27 * 59 / 12 = 1593/12 = 132.75MHz (close enough)
// Or: 27MHz * 40 / 2 / 4 = 27 * 40 / 8 = 135MHz
// 
// For better precision: 27MHz * 44 / 9 = 132MHz (close to target)

`default_nettype none

module pll(
    input wire clkin,      // 27MHz input
    output wire clkout,    // ~133MHz output
    output wire locked     // PLL lock indicator
);

    // Gowin rPLL primitive
    // Using parameters for ~133MHz from 27MHz input
    // FCLKIN: 27MHz
    // IDIV_SEL: 8 (divide by 9)
    // FBDIV_SEL: 43 (multiply by 44)
    // ODIV_SEL: 4 (divide by 4)
    // Fvco = 27 * 44 / 9 = 132MHz
    // Fout = 132MHz (close to 133.333MHz target)
    
    wire clkoutp;  // Phase shifted output (unused)
    wire clkoutd;  // Divided output (unused)
    wire clkoutd3; // Divided by 3 output (unused)
    
    rPLL #(
        .FCLKIN("27"),           // Input clock frequency in MHz
        .IDIV_SEL(8),            // IDIV = 9 (0-63, actual = SEL+1)
        .FBDIV_SEL(43),          // FBDIV = 44 (0-63, actual = SEL+1)
        .ODIV_SEL(4),            // ODIV = 4 (2,4,8,16,32,48,64,80,96,112,128)
        .PSDA_SEL("0000"),       // Phase shift (not used)
        .DYN_SDIV_SEL(2),        // Dynamic SDIV select
        .DYN_DA_EN("false"),     // Disable dynamic adjustment
        .DYN_FBDIV_SEL("false"), // Disable dynamic FBDIV
        .DYN_IDIV_SEL("false"),  // Disable dynamic IDIV
        .DYN_ODIV_SEL("false"),  // Disable dynamic ODIV
        .DUTYDA_SEL("1000"),     // 50% duty cycle
        .CLKOUT_FT_DIR(1'b1),    // Fine tune direction
        .CLKOUTP_FT_DIR(1'b1),   // Fine tune direction
        .CLKOUT_DLY_STEP(0),     // Output delay steps
        .CLKOUTP_DLY_STEP(0),    // Phase output delay steps
        .CLKOUTD3_SRC("CLKOUT"), // CLKOUTD3 source
        .CLKOUTD_SRC("CLKOUT"),  // CLKOUTD source
        .CLKOUTD_BYPASS("false"), // Don't bypass divider
        .CLKOUTP_BYPASS("false"), // Don't bypass phase
        .CLKOUT_BYPASS("false"),  // Don't bypass output
        .DEVICE("GW2AR-18C")      // Device variant
    ) pll_inst (
        .CLKIN(clkin),           // Input clock
        .CLKFB(1'b0),            // Feedback clock (internal)
        .RESET(1'b0),            // Reset (active high)
        .RESET_P(1'b0),          // Reset for phase
        .FBDSEL(6'b000000),      // Dynamic FBDIV select
        .IDSEL(6'b000000),       // Dynamic IDIV select
        .ODSEL(6'b000000),       // Dynamic ODIV select
        .PSDA(4'b0000),          // Phase shift
        .FDLY(4'b0000),          // Fine delay
        .DUTYDA(4'b0000),        // Duty cycle adjust
        .LOCK(locked),           // Lock output
        .CLKOUT(clkout),         // Main output clock
        .CLKOUTP(clkoutp),       // Phase shifted output
        .CLKOUTD(clkoutd),       // Divided output
        .CLKOUTD3(clkoutd3)      // Divided by 3 output
    );

endmodule
