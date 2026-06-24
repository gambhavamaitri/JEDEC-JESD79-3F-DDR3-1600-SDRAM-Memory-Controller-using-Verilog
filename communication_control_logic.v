// ============================================
// communication_control_logic.v
// DDR3 command bus manager
//
// Enforces inter-command timing constraints:
//   tCCD: CAS-to-CAS delay (4 nCK)
//   tRRD: RAS-to-RAS delay (6 nCK)
//   tWTR: Write-to-Read delay (6 nCK)
//   tRTW: Read-to-Write delay (tRTW nCK)
//
// Manages multi-rank CS# selection
// Registers all DDR3 command outputs
// ============================================
`include "ddr3_params.v"

module communication_control_logic (
    input  wire clk,
    input  wire rst_n,

    // From control_unit — raw command intent
    input  wire [2:0]           cmd_in,       // {RAS#,CAS#,WE#} encoding
    input  wire [BANK_BITS-1:0] ba_in,
    input  wire [ROW_BITS-1:0]  addr_in,
    input  wire                 cmd_valid,    // 1=issue cmd this cycle
    input  wire [1:0]           rank_sel,     // 0-3 rank select

    // Registered DDR3 outputs to PHY pins
    output reg        ras_n,
    output reg        cas_n,
    output reg        we_n,
    output reg [3:0]  cs_n,           // one bit per rank
    output reg [BANK_BITS-1:0] ba,
    output reg [ROW_BITS-1:0]  addr,

    // Handshake
    output wire cmd_ready,    // 1 = bus available for new command
    output wire cmd_stall     // 1 = timing constraint blocking
);

    // ── Inter-command timing counters ─────────
    reg [$clog2(tCCD+1)-1:0] tccd_cnt;
    reg [$clog2(tRRD+1)-1:0] trrd_cnt;
    reg [$clog2(tWTR+1)-1:0] twtr_cnt;
    reg [$clog2(tRTW+1)-1:0] trtw_cnt;

    // ── Track previous command type ───────────
    wire is_read  = (cmd_in == CMD_READ);
    wire is_write = (cmd_in == CMD_WRITE);
    wire is_act   = (cmd_in == CMD_ACTIVATE);

    // ── Stall conditions ──────────────────────
    wire stall_ccd = ((is_read || is_write) && (tccd_cnt > 0));
    wire stall_rrd = (is_act && (trrd_cnt > 0));
    wire stall_wtr = (is_read && (twtr_cnt > 0));
    wire stall_rtw = (is_write && (trtw_cnt > 0));

    assign cmd_stall = stall_ccd | stall_rrd | stall_wtr | stall_rtw;
    assign cmd_ready = !cmd_stall;

    // ── Sequential logic ──────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tccd_cnt <= 0;
            trrd_cnt <= 0;
            twtr_cnt <= 0;
            trtw_cnt <= 0;
            cs_n     <= 4'b1111;   // all deselected
            ras_n    <= 1'b1;
            cas_n    <= 1'b1;
            we_n     <= 1'b1;
            ba       <= 0;
            addr     <= 0;
        end else begin

            // ── Decrement counters ─────────────
            if (tccd_cnt > 0) tccd_cnt <= tccd_cnt - 1;
            if (trrd_cnt > 0) trrd_cnt <= trrd_cnt - 1;
            if (twtr_cnt > 0) twtr_cnt <= twtr_cnt - 1;
            if (trtw_cnt > 0) trtw_cnt <= trtw_cnt - 1;

            // ── Issue command if valid and no stall ──
            if (cmd_valid && !cmd_stall) begin

                // Register command outputs
                ras_n <= cmd_in[2];
                cas_n <= cmd_in[1];
                we_n  <= cmd_in[0];
                ba    <= ba_in;
                addr  <= addr_in;

                // Multi-rank CS# decode
                case (rank_sel)
                    2'b00: cs_n <= 4'b1110;
                    2'b01: cs_n <= 4'b1101;
                    2'b10: cs_n <= 4'b1011;
                    2'b11: cs_n <= 4'b0111;
                endcase

                // Load timing counters based on cmd type
                if (is_read || is_write)
                    tccd_cnt <= tCCD;

                if (is_act)
                    trrd_cnt <= tRRD;

                if (is_write)
                    twtr_cnt <= tWTR;

                if (is_read)
                    trtw_cnt <= tRTW;

            end else begin
                // NOP / deselect
                cs_n  <= 4'b1111;
                ras_n <= 1'b1;
                cas_n <= 1'b1;
                we_n  <= 1'b1;
            end
        end
    end

endmodule
