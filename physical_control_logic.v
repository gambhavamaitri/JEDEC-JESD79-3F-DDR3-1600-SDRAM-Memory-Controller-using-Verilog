// ============================================
// physical_control_logic.v
// ODT (On-Die Termination) + Drive Strength
//
// WHAT IT DOES:
//   Controls the DRAM chip's on-die termination
//   resistors via the ODT pin.
//
//   During WRITE: ODT must be OFF on target DRAM
//   (termination would fight the controller driver)
//
//   During READ: ODT should be ON on non-target
//   DRAMs (terminate signal for signal integrity)
//
//   Also manages ZQ calibration trigger timing
//   and drive strength settings from MR1.
//
// JEDEC tAOND/tAOFD:
//   tAOND = 8.5ns = ODT turn-ON delay
//   tAOFD = 8.5ns = ODT turn-OFF delay
//   At 800MHz: ~7 nCK for ODT switching
//
// ODT encoding:
//   RTT_NOM = RZQ/6 = 40Ω (MR1[9,6,2]=011)
//   RTT_WR  = RZQ/2 = 120Ω (MR2[10:9]=01)
//   ODT pin HIGH = termination active
//   ODT pin LOW  = termination off
// ============================================
`include "ddr3_params.v"

module physical_control_logic (
    input  wire clk,
    input  wire rst_n,

    // ── Command notifications ─────────────────
    input  wire read_issued,    // 1-cycle: READ command sent
    input  wire write_issued,   // 1-cycle: WRITE command sent
    input  wire refresh_issued, // 1-cycle: REF command sent

    // ── ZQ calibration ────────────────────────
    input  wire zq_short_due,   // periodic ZQCS needed
    output reg  zq_short_req,   // request ZQCS to zq_cal_ctrl

    // ── ODT control ───────────────────────────
    // ODT pin to DRAM (active HIGH = terminate)
    output reg  odt,

    // ── Drive strength ────────────────────────
    // Reflects MR1 ODS setting
    // 0 = RZQ/6 (34Ω), 1 = RZQ/7 (40Ω)
    // (actual drive strength set via MRS, this
    //  output is for monitoring/debug only)
    output wire drive_strength,

    // ── Status ───────────────────────────────
    output reg  odt_active,     // ODT currently on
    output wire [3:0] state_out
);

    // ── ODT state machine ─────────────────────
    localparam S_ODT_OFF    = 2'd0; // ODT disabled
    localparam S_ODT_ON_RD  = 2'd1; // ODT on for READ
    localparam S_ODT_OFF_WR = 2'd2; // ODT off for WRITE
    localparam S_ODT_DELAY  = 2'd3; // waiting tAOND/tAOFD

    reg [1:0] odt_state;

    // tAOND/tAOFD = 8.5ns → ceil(8.5/1.25) = 7 nCK
    localparam TAOND = 7;
    reg [$clog2(TAOND+1)-1:0] odt_delay_cnt;

    // Track whether last command was read or write
    reg last_was_write;

    assign state_out      = {2'b00, odt_state};
    // Drive strength from MR1_VAL bit A[1] (ODS[0])
    // MR1_VAL = 14'h0046 → bit1 = 1 → RZQ/7 = 34Ω
    assign drive_strength = MR1_VAL[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            odt_state     <= S_ODT_OFF;
            odt           <= 1'b0;
            odt_active    <= 1'b0;
            odt_delay_cnt <= 0;
            last_was_write<= 0;
            zq_short_req  <= 0;

        end else begin
            zq_short_req <= 0; // default deassert

            // ── ZQ short request ───────────────
            // Forward zq_short_due to calibration
            // controller as a 1-cycle request pulse
            if (zq_short_due)
                zq_short_req <= 1;

            // ── ODT state machine ──────────────
            case (odt_state)

                // ─────────────────────────────
                // ODT off — wait for command
                S_ODT_OFF: begin
                    odt        <= 1'b0;
                    odt_active <= 1'b0;

                    if (read_issued) begin
                        // READ: turn ODT ON (far-end termination)
                        // Wait tAOND before asserting ODT
                        last_was_write <= 0;
                        odt_delay_cnt  <= TAOND - 1;
                        odt_state      <= S_ODT_DELAY;

                    end else if (write_issued) begin
                        // WRITE: ODT stays OFF on target DRAM
                        // (controller is driving, termination off)
                        last_was_write <= 1;
                        odt_state      <= S_ODT_OFF_WR;
                    end
                end

                // ─────────────────────────────
                // Delay before ODT switches
                // (tAOND = ODT turn-on delay)
                S_ODT_DELAY: begin
                    if (odt_delay_cnt > 0) begin
                        odt_delay_cnt <= odt_delay_cnt - 1;
                    end else begin
                        if (!last_was_write) begin
                            // Delay done: turn ODT on for READ
                            odt        <= 1'b1;
                            odt_active <= 1'b1;
                            odt_state  <= S_ODT_ON_RD;
                        end else begin
                            odt_state <= S_ODT_OFF;
                        end
                    end
                end

                // ─────────────────────────────
                // ODT on during READ burst
                // Stays on for BL/2 + tAOFD cycles
                S_ODT_ON_RD: begin
                    odt        <= 1'b1;
                    odt_active <= 1'b1;

                    // Turn off when next command arrives
                    // or after burst window
                    if (write_issued) begin
                        // Switch to write mode: turn ODT off
                        last_was_write <= 1;
                        odt_delay_cnt  <= TAOND - 1;
                        odt_state      <= S_ODT_DELAY;
                        odt            <= 1'b0;
                        odt_active     <= 1'b0;
                    end else if (refresh_issued) begin
                        // REFRESH: ODT off
                        odt        <= 1'b0;
                        odt_active <= 1'b0;
                        odt_state  <= S_ODT_OFF;
                    end
                    // else: stay ON (open-page reads)
                end

                // ─────────────────────────────
                // ODT off during WRITE
                // Return to OFF when write burst ends
                S_ODT_OFF_WR: begin
                    odt        <= 1'b0;
                    odt_active <= 1'b0;

                    if (read_issued) begin
                        // Switch to read: turn ODT back on
                        last_was_write <= 0;
                        odt_delay_cnt  <= TAOND - 1;
                        odt_state      <= S_ODT_DELAY;
                    end else if (!write_issued) begin
                        // Write burst done: return to idle
                        odt_state <= S_ODT_OFF;
                    end
                end

                default: odt_state <= S_ODT_OFF;
            endcase
        end
    end

endmodule
