// ============================================
// mode_register_control.v
// Production-quality MRS command sequencer
// JEDEC sequence: MR2 -> MR3 -> MR1 -> MR0
//
// ASSUMED CALLER SEQUENCE for full init:
//   1. Assert mrs_req with target=000, timing_ok=1
//   2. Wait for mrs_done
//   3. Check tdllk_required — if 1, wait tDLLK
//   4. Run ZQCL (not handled by this module)
//   5. Wait tZQinit

//
// Why MR2->MR3->MR1->MR0 order:
//   MR2 = CWL (CAS Write Latency)
//   MR3 = MPR, MRS command ordering
//   MR1 = DLL, ODT, drive strength
//   MR0 = BL, CL, DLL reset, WR recovery
//   JEDEC mandates this exact initialization sequence.
//
// MRS command timing:
//   Command is driven during S_MRx state (cmd_valid=1, cmd_type=CMD_MRS).
//   tMRD/tMOD countdown begins the FOLLOWING cycle in S_WAIT_MRx.
// ============================================
`include "ddr3_params.v"

module mode_register_control (
    input  wire clk,
    input  wire rst_n,

    // Trigger and timing qualification
    // timing_ok: asserted by command_arbiter when no refresh/ZQ/active banks.
    // Only sampled at sequence start. command_arbiter must guarantee
    // exclusive ownership for the entire MRS sequence.
    input  wire       mrs_req,        // 1-cycle pulse (edge-detected internally)
    input  wire [2:0] mrs_target,     // 000=full, 001=MR0, 010=MR1, 011=MR2, 100=MR3
    input  wire       timing_ok,      // 1 = controller grants MRS slot

    // Runtime override values
    input  wire [13:0] mr0_override,
    input  wire [13:0] mr1_override,
    input  wire [13:0] mr2_override,
    input  wire [13:0] mr3_override,
    input  wire        use_override,  // 1 = use override ports, 0 = use params

    // Command interface to command_arbiter
    // MRS command is driven during S_MRx state (single cycle).
    // tMRD/tMOD wait begins in S_WAIT_MRx state.
    output reg        cmd_valid,      // 1 = valid command this cycle
    output reg [3:0]  cmd_type,       // CMD_MRS = 4'b0100
    output reg [2:0]  cmd_bank,       // BA[2:0]
    output reg [13:0] cmd_addr,       // A[13:0]

    // Current mode register values (updated on MRS issuance)
    output reg [13:0] current_mr0,
    output reg [13:0] current_mr1,
    output reg [13:0] current_mr2,
    output reg [13:0] current_mr3,

    // DLL reset tracking
    // Set when MR0 with DLL reset bit (A8) = 1 is issued.
    // Cleared when a new MRS sequence is requested.
    // Caller must check after mrs_done and wait tDLLK if 1.
    output reg        tdllk_required,

    // Debug
    output wire [3:0] current_state,
    output reg  [31:0] mrs_count,     // Total MRS commands issued (cumulative)

    // Status
    output reg        mrs_busy,       // 1 while sequence in progress
    output reg        mrs_done,       // 1-cycle pulse when complete
    output reg        mrs_error       // 1-cycle pulse on illegal request
);

    // ── Command type encoding ──────────────────
    localparam [3:0] CMD_NOP = 4'b0000;
    localparam [3:0] CMD_MRS = 4'b0100;

    // ── State encoding ─────────────────────────
    localparam [3:0]
        S_IDLE     = 4'd0,
        S_MR2      = 4'd1,
        S_WAIT_MR2 = 4'd2,
        S_MR3      = 4'd3,
        S_WAIT_MR3 = 4'd4,
        S_MR1      = 4'd5,
        S_WAIT_MR1 = 4'd6,
        S_MR0      = 4'd7,
        S_WAIT_MOD = 4'd8,
        S_DONE     = 4'd9;

    reg [3:0]  state;
    reg [13:0] timer;
    reg [2:0]  target_latch;
    reg [13:0] mr0_val, mr1_val, mr2_val, mr3_val;

    // ── Edge detection for mrs_req ─────────────
    reg mrs_req_d;
    wire mrs_req_rise = mrs_req & ~mrs_req_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mrs_req_d <= 1'b0;
        else
            mrs_req_d <= mrs_req;
    end

    // ── Combinational command decode ───────────
    always @(*) begin
        // Defaults: NOP
        cmd_valid = 1'b0;
        cmd_type  = CMD_NOP;
        cmd_bank  = 3'b000;
        cmd_addr  = 14'd0;

        case (state)
            S_MR2: begin
                cmd_valid = 1'b1;
                cmd_type  = CMD_MRS;
                cmd_bank  = 3'd2;
                cmd_addr  = mr2_val;
            end

            S_MR3: begin
                cmd_valid = 1'b1;
                cmd_type  = CMD_MRS;
                cmd_bank  = 3'd3;
                cmd_addr  = mr3_val;
            end

            S_MR1: begin
                cmd_valid = 1'b1;
                cmd_type  = CMD_MRS;
                cmd_bank  = 3'd1;
                cmd_addr  = mr1_val;
            end

            S_MR0: begin
                cmd_valid = 1'b1;
                cmd_type  = CMD_MRS;
                cmd_bank  = 3'd0;
                cmd_addr  = mr0_val;
            end

            default: ; // NOP defaults
        endcase
    end

    // ── Debug: combinatorial state visibility ──
    assign current_state = state;

    // ── Sequential state machine ─────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            timer          <= 14'd0;
            target_latch   <= 3'b000;
            mrs_busy       <= 1'b0;
            mrs_done       <= 1'b0;
            mrs_error      <= 1'b0;
            mrs_count      <= 32'd0;
            tdllk_required <= 1'b0;

            mr0_val     <= MR0_VAL;
            mr1_val     <= MR1_VAL;
            mr2_val     <= MR2_VAL;
            mr3_val     <= MR3_VAL;

            current_mr0 <= MR0_VAL;
            current_mr1 <= MR1_VAL;
            current_mr2 <= MR2_VAL;
            current_mr3 <= MR3_VAL;
        end else begin
            mrs_done  <= 1'b0;
            mrs_error <= 1'b0;

            // ── Request validation (edge-detected, outside FSM) ──
            if (mrs_req_rise) begin
                if (state != S_IDLE) begin
                    mrs_error <= 1'b1;              // ERR_BUSY
                end else if (!timing_ok) begin
                    mrs_error <= 1'b1;              // ERR_TIMING
                end else if (mrs_target > 3'b100) begin
                    mrs_error <= 1'b1;              // ERR_BAD_TARGET
                end
            end

            case (state)

                S_IDLE: begin
                    mrs_busy <= 1'b0;

                    if (mrs_req_rise && timing_ok && (mrs_target <= 3'b100)) begin
                        // Clear DLL reset flag from previous sequence
                        tdllk_required <= 1'b0;
                        target_latch   <= mrs_target;
                        mr0_val <= use_override ? mr0_override : MR0_VAL;
                        mr1_val <= use_override ? mr1_override : MR1_VAL;
                        mr2_val <= use_override ? mr2_override : MR2_VAL;
                        mr3_val <= use_override ? mr3_override : MR3_VAL;
                        mrs_busy <= 1'b1;

                        case (mrs_target)
                            3'b001: state <= S_MR0;  // MR0 only
                            3'b010: state <= S_MR1;  // MR1 only
                            3'b011: state <= S_MR2;  // MR2 only
                            3'b100: state <= S_MR3;  // MR3 only
                            default: state <= S_MR2;  // Full sequence (000)
                        endcase
                    end
                end

                S_MR2: begin
                    current_mr2 <= mr2_val;
                    mrs_count   <= mrs_count + 1;
                    timer <= (tMRD == 0) ? 14'd0 : (tMRD - 1);
                    state <= S_WAIT_MR2;
                end

                S_WAIT_MR2: begin
                    if (timer > 0)
                        timer <= timer - 1;
                    else begin
                        state <= (target_latch == 3'b011) ? S_DONE : S_MR3;
                    end
                end

                S_MR3: begin
                    current_mr3 <= mr3_val;
                    mrs_count   <= mrs_count + 1;
                    timer <= (tMRD == 0) ? 14'd0 : (tMRD - 1);
                    state <= S_WAIT_MR3;
                end

                S_WAIT_MR3: begin
                    if (timer > 0)
                        timer <= timer - 1;
                    else begin
                        state <= (target_latch == 3'b100) ? S_DONE : S_MR1;
                    end
                end

                S_MR1: begin
                    current_mr1 <= mr1_val;
                    mrs_count   <= mrs_count + 1;
                    timer <= (tMRD == 0) ? 14'd0 : (tMRD - 1);
                    state <= S_WAIT_MR1;
                end

                S_WAIT_MR1: begin
                    if (timer > 0)
                        timer <= timer - 1;
                    else begin
                        state <= (target_latch == 3'b010) ? S_DONE : S_MR0;
                    end
                end

                S_MR0: begin
                    current_mr0 <= mr0_val;
                    mrs_count   <= mrs_count + 1;
                    // Track DLL reset: JEDEC MR0 bit 8 = DLL reset
                    if (mr0_val[8])
                        tdllk_required <= 1'b1;
                    timer <= (tMOD == 0) ? 14'd0 : (tMOD - 1);
                    state <= S_WAIT_MOD;
                end

                S_WAIT_MOD: begin
                    if (timer > 0)
                        timer <= timer - 1;
                    else begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    mrs_done <= 1'b1;
                    mrs_busy <= 1'b0;
                    state    <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
