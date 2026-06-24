// ============================================
// refresh_control_logic.v  —  ARCHITECTURE v2
// Credit-based refresh scheduler
// ============================================
`include "ddr3_params.v"

module refresh_control_logic (
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,
    input  wire ref_ack,
    input  wire high_temp,

    output reg  ref_req,
    output wire ref_urgent,
    output reg  ref_busy,

    output reg  cs_n,
    output reg  ras_n,
    output reg  cas_n,
    output reg  we_n,

    output reg [3:0] ref_credits
);

    // ── States ────────────────────────────────
    localparam S_IDLE       = 3'd0;
    localparam S_COUNTING   = 3'd1;   // Ready state — waiting for credits
    localparam S_REQUESTING = 3'd2;
    localparam S_REFRESHING = 3'd3;
    localparam S_WAIT_RFC   = 3'd4;

    reg [2:0] state, next_state;
    localparam MAX_CREDITS = 4'd8;

    // ── Global tREFI tracker ─────────────────
    // Runs in ALL states except reset. Never frozen.
    reg [12:0] trefi_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            trefi_cnt <= 0;
        else if (init_done) begin
            if (trefi_cnt > 0)
                trefi_cnt <= trefi_cnt - 1;
            else
                trefi_cnt <= high_temp ? tREFI_HOT : tREFI;
        end
    end

    // ── Credit accumulator ─────────────────────
    // Single source of truth. Explicit arbitration when inc & dec collide.
    wire credit_inc = init_done && (trefi_cnt == 0) && (ref_credits < MAX_CREDITS);
    wire credit_dec = (state == S_REFRESHING) && (ref_credits > 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ref_credits <= 4'd0;
        else begin
            case ({credit_inc, credit_dec})
                2'b10: ref_credits <= ref_credits + 1;
                2'b01: ref_credits <= ref_credits - 1;
                2'b11: ref_credits <= ref_credits;   // +1 and -1 cancel
                default: ref_credits <= ref_credits;
            endcase
        end
    end

    // ── tRFC timer ─────────────────────────────
    reg [7:0] trfc_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            trfc_cnt <= 0;
        else if (state == S_REFRESHING)
            trfc_cnt <= tRFC - 1;
        else if (state == S_WAIT_RFC && trfc_cnt > 0)
            trfc_cnt <= trfc_cnt - 1;
    end

    // ── State register ───────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // ── Combinational FSM: next_state + outputs ─
    // All outputs assigned default values before case to prevent latches.
    always @(*) begin
        // Default outputs (NOP / idle)
        next_state = state;
        ref_req    = 1'b0;
        ref_busy   = 1'b0;
        cs_n       = 1'b1;
        ras_n      = 1'b1;
        cas_n      = 1'b1;
        we_n       = 1'b1;

        case (state)

            S_IDLE: begin
                if (init_done)
                    next_state = S_COUNTING;
            end

            S_COUNTING: begin
                // Global tREFI is running; credits accumulate independently.
                // When at least one credit is available, request refresh.
                if (ref_credits > 0)
                    next_state = S_REQUESTING;
            end

            S_REQUESTING: begin
                ref_req = 1'b1;
                if (ref_ack)
                    next_state = S_REFRESHING;
                // Self-loop: if no ref_ack, stay here. tREFI continues globally.
            end

            S_REFRESHING: begin
                // Issue REFRESH command: {CS=0, RAS=0, CAS=0, WE=1}
                ref_busy = 1'b1;
                cs_n     = 1'b0;
                ras_n    = 1'b0;
                cas_n    = 1'b0;
                we_n     = 1'b1;
                next_state = S_WAIT_RFC;
            end

            S_WAIT_RFC: begin
                ref_busy = 1'b1;
                if (trfc_cnt == 0) begin
                    // tRFC complete. If more credits pending, chain immediately.
                    if (ref_credits > 0)
                        next_state = S_REQUESTING;
                    else
                        next_state = S_COUNTING;
                end
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ── Urgent flag: combinational, always current ─
    assign ref_urgent = init_done && (ref_credits == MAX_CREDITS);

endmodule
