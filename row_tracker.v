// ============================================
// row_tracker.v
// Page policy engine for DDR3 controller
// Decides: do_activate / do_precharge / skip_activate
// based on open/closed/adaptive page policy
//
// Consumes: bank_active, row_hit signals
//           from dram_bank_control
// Produces: scheduling hints for control_unit
//
// Policy:
//   CLOSED   (0): always PRE after transaction
//   OPEN     (1): keep row open, skip ACT on hit
//   ADAPTIVE (2): per-bank, switches based on
//                 hit/miss streak vs threshold
// ============================================
`include "ddr3_params.v"

module row_tracker (
    input  wire clk,
    input  wire rst_n,

    // ── Request info ──────────────────────────
    input  wire [BANK_BITS-1:0] req_bank,
    input  wire [ROW_BITS-1:0]  req_row,
    input  wire                 new_request,    // 1-cycle pulse

    // ── From dram_bank_control ────────────────
    // (bank_sel in dram_bank_control = req_bank)
    input  wire                 bank_active,    // selected bank has open row
    input  wire                 row_hit,        // open row == req_row

    // ── Policy select ─────────────────────────
    input  wire [1:0] page_policy,

    // ── Outputs to control_unit ───────────────
    output reg  do_precharge,    // close row before access
    output reg  do_activate,     // need ACTIVATE for this request
    output reg  skip_activate,   // row hit on open policy — go straight to RD/WR

    // ── Performance counters ──────────────────
    output reg [31:0] total_hits,
    output reg [31:0] total_misses
);

    // ── Policy encodings ──────────────────────
    localparam POLICY_CLOSED   = 2'd0;
    localparam POLICY_OPEN     = 2'd1;
    localparam POLICY_ADAPTIVE = 2'd2;

    // ── Adaptive: per-bank hit/miss streak ────
    localparam ADAPT_THRESHOLD = 3;

    reg [3:0] hit_streak  [0:NUM_BANKS-1];
    reg [3:0] miss_streak [0:NUM_BANKS-1];

    // Effective policy per bank (used in adaptive mode)
    // Starts optimistic (OPEN) — switches on miss streak
    reg [1:0] effective_policy [0:NUM_BANKS-1];

    integer i;

    // ── Combinational: decision on new_request ─
    always @(*) begin
        do_precharge  = 1'b0;
        do_activate   = 1'b0;
        skip_activate = 1'b0;

        if (new_request) begin

            case (page_policy)

                // ────────────────────────────────────
                // CLOSED: always precharge + activate
                // PRE if bank is currently open
                // ACT always (row must be opened fresh)
                // ────────────────────────────────────
                POLICY_CLOSED: begin
                    if (bank_active)
                        do_precharge = 1'b1;
                    do_activate = 1'b1;
                end

                // ────────────────────────────────────
                // OPEN: keep row open across accesses
                // HIT  → skip ACT entirely
                // MISS → PRE + ACT (different row)
                // IDLE → just ACT (bank was closed)
                // ────────────────────────────────────
                POLICY_OPEN: begin
                    if (bank_active && row_hit) begin
                        skip_activate = 1'b1;
                    end else if (bank_active && !row_hit) begin
                        do_precharge  = 1'b1;
                        do_activate   = 1'b1;
                    end else begin
                        // bank idle
                        do_activate = 1'b1;
                    end
                end

                // ────────────────────────────────────
                // ADAPTIVE: per-bank effective policy
                // switches between OPEN and CLOSED
                // based on hit/miss streak threshold
                // ────────────────────────────────────
                POLICY_ADAPTIVE: begin
                    if (effective_policy[req_bank] == POLICY_OPEN) begin
                        if (bank_active && row_hit) begin
                            skip_activate = 1'b1;
                        end else if (bank_active && !row_hit) begin
                            do_precharge = 1'b1;
                            do_activate  = 1'b1;
                        end else begin
                            do_activate = 1'b1;
                        end
                    end else begin
                        // effective = CLOSED for this bank
                        if (bank_active)
                            do_precharge = 1'b1;
                        do_activate = 1'b1;
                    end
                end

                default: begin
                    do_activate = 1'b1;
                end

            endcase
        end
    end

    // ── Sequential: counters + adaptive update ─
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_hits  <= 32'd0;
            total_misses<= 32'd0;
            for (i = 0; i < NUM_BANKS; i = i + 1) begin
                hit_streak[i]       <= 4'd0;
                miss_streak[i]      <= 4'd0;
                effective_policy[i] <= POLICY_OPEN; // start optimistic
            end

        end else if (new_request) begin

            if (bank_active && row_hit) begin
                // ── HIT ──────────────────────────
                total_hits <= total_hits + 1;

                // saturating increment hit streak
                if (hit_streak[req_bank] < 4'hF)
                    hit_streak[req_bank] <= hit_streak[req_bank] + 1;

                // reset miss streak
                miss_streak[req_bank] <= 4'd0;

                // adaptive: enough hits → switch to OPEN
                if (page_policy == POLICY_ADAPTIVE &&
                    hit_streak[req_bank] >= ADAPT_THRESHOLD)
                    effective_policy[req_bank] <= POLICY_OPEN;

            end else begin
                // ── MISS (or bank idle — counts as miss) ──
                total_misses <= total_misses + 1;

                // saturating increment miss streak
                if (miss_streak[req_bank] < 4'hF)
                    miss_streak[req_bank] <= miss_streak[req_bank] + 1;

                // reset hit streak
                hit_streak[req_bank] <= 4'd0;

                // adaptive: enough misses → switch to CLOSED
                if (page_policy == POLICY_ADAPTIVE &&
                    miss_streak[req_bank] >= ADAPT_THRESHOLD)
                    effective_policy[req_bank] <= POLICY_CLOSED;
            end
        end
    end

endmodule
