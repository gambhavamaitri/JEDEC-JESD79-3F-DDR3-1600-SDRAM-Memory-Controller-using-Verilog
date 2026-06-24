

`include "ddr3_params.v"

// ============================================
// bank_machine — FIXED
// ============================================
module bank_machine (
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,

    input  wire                  req_valid,
    input  wire                  req_wr,
    input  wire [ROW_BITS-1:0]   req_row,
    input  wire [COL_BITS-1:0]   req_col,
    output reg                   req_ready,

    input  wire can_activate,
    input  wire can_read,
    input  wire can_write,
    input  wire can_precharge,

    input  wire do_precharge,
    input  wire do_activate,
    input  wire skip_activate,
    input  wire bank_active,
    input  wire row_hit,

    output reg  cmd_req,
    input  wire cmd_grant,

    output reg [2:0]            cmd_out,
    output reg [ROW_BITS-1:0]   addr_out,
    output wire [BANK_BITS-1:0] ba_out,

    output reg activate_issued,
    output reg read_issued,
    output reg write_issued,
    output reg precharge_issued,
    output reg req_accepted,
    output reg data_valid,
    output wire [3:0] state_dbg
);

    localparam S_IDLE       = 4'd0;
    localparam S_WAIT_GATE  = 4'd1;
    localparam S_PRECHARGE  = 4'd2;
    localparam S_WAIT_TRP   = 4'd3;
    localparam S_ACTIVATE   = 4'd4;
    localparam S_WAIT_TRCD  = 4'd5;
    localparam S_READ_WRITE = 4'd6;
    localparam S_WAIT_CL    = 4'd7;
    localparam S_WAIT_TWR   = 4'd8;

    reg [3:0] state;
    assign state_dbg = state;

    reg                  latched_wr;
    reg [ROW_BITS-1:0]   latched_row;
    reg [COL_BITS-1:0]   latched_col;
    reg                  close_after_access;
    reg                  precharge_for_miss;
    reg [3:0]            cl_cnt;

    // JEDEC command encodings {RAS#,CAS#,WE#}
    localparam CMD_NONE = 3'b111; // NOP
    localparam CMD_PRE  = 3'b010; // PRECHARGE
    localparam CMD_ACT  = 3'b011; // ACTIVATE
    localparam CMD_RD   = 3'b101; // READ
    localparam CMD_WR   = 3'b100; // WRITE

    reg [2:0] pending_cmd;

    // This bank machine always drives its own bank
    // In full array: BANK_ID parameter would be used
    assign ba_out = 3'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            req_ready         <= 1;
            req_accepted      <= 0;
            data_valid        <= 0;
            cmd_req           <= 0;
            cmd_out           <= CMD_NONE;
            addr_out          <= 0;
            activate_issued   <= 0;
            read_issued       <= 0;
            write_issued      <= 0;
            precharge_issued  <= 0;
            latched_wr        <= 0;
            latched_row       <= 0;
            latched_col       <= 0;
            close_after_access<= 0;
            precharge_for_miss<= 0;
            cl_cnt            <= 0;
            pending_cmd       <= CMD_NONE;

        end else begin
            // Default deasserts
            req_accepted      <= 0;
            data_valid        <= 0;
            activate_issued   <= 0;
            read_issued       <= 0;
            write_issued      <= 0;
            precharge_issued  <= 0;
            cmd_out           <= CMD_NONE;

            case (state)

                S_IDLE: begin
                    cmd_req   <= 0;
                    req_ready <= 1;

                    if (init_done && req_valid) begin
                        req_ready          <= 0;
                        latched_wr         <= req_wr;
                        latched_row        <= req_row;
                        latched_col        <= req_col;
                        close_after_access <= do_precharge;
                        precharge_for_miss <= 0;
                        state              <= S_WAIT_GATE;
                    end
                end

                S_WAIT_GATE: begin
                    cmd_req   <= 0;
                    req_ready <= 0;

                    // Path 1: Row buffer hit — go straight to RD/WR
                    if (skip_activate && !latched_wr && can_read) begin
                        pending_cmd <= CMD_RD;
                        state       <= S_READ_WRITE;

                    end else if (skip_activate && latched_wr && can_write) begin
                        pending_cmd <= CMD_WR;
                        state       <= S_READ_WRITE;

                    // Path 2: Row miss — precharge first
                    // FIX: check bank_active here (scheduler prediction
                    // may be stale if back-to-back requests)
                    end else if (do_precharge && bank_active && can_precharge) begin
                        pending_cmd        <= CMD_PRE;
                        precharge_for_miss <= 1;
                        state              <= S_PRECHARGE;

                    // Path 3: Bank idle — just activate
                    end else if (do_activate && can_activate && !bank_active) begin
                        pending_cmd <= CMD_ACT;
                        state       <= S_ACTIVATE;

                    // Path 4: do_activate=1 but bank_active=1
                    // Scheduler predicted idle but bank is open
                    // Must precharge before activate
                    end else if (do_activate && bank_active && can_precharge) begin
                        pending_cmd        <= CMD_PRE;
                        precharge_for_miss <= 1;
                        state              <= S_PRECHARGE;
                    end
                    // else: waiting for gate — stay here
                end

                S_PRECHARGE: begin
                    cmd_req <= 1;
                    if (cmd_grant) begin
                        cmd_out          <= CMD_PRE;
                        addr_out         <= {ROW_BITS{1'b0}}; // A10=0
                        precharge_issued <= 1;
                        cmd_req          <= 0;
                        state            <= S_WAIT_TRP;
                    end
                end

                S_WAIT_TRP: begin
                    cmd_req <= 0;
                    if (can_activate) begin
                        if (precharge_for_miss) begin
                            // Row miss path: ACT next
                            pending_cmd        <= CMD_ACT;
                            precharge_for_miss <= 0;
                            state              <= S_ACTIVATE;
                        end else begin
                            // Post-access close path: done
                            state     <= S_IDLE;
                            req_ready <= 1;
                        end
                    end
                end

                S_ACTIVATE: begin
                    cmd_req <= 1;
                    if (cmd_grant) begin
                        cmd_out         <= CMD_ACT;
                        addr_out        <= latched_row;
                        activate_issued <= 1;
                        cmd_req         <= 0;
                        state           <= S_WAIT_TRCD;
                    end
                end

                S_WAIT_TRCD: begin
                    cmd_req <= 0;
                    if (!latched_wr && can_read) begin
                        pending_cmd <= CMD_RD;
                        state       <= S_READ_WRITE;
                    end else if (latched_wr && can_write) begin
                        pending_cmd <= CMD_WR;
                        state       <= S_READ_WRITE;
                    end
                end

                S_READ_WRITE: begin
                    cmd_req <= 1;
                    if (cmd_grant) begin
                        cmd_out <= pending_cmd;

                        if (latched_wr) begin
                            write_issued <= 1;
                            cl_cnt       <= 0;
                        end else begin
                            read_issued  <= 1;
                            cl_cnt       <= CL - 1; // countdown from CL-1
                        end

                        // Column address on addr pins, A10=0
                        addr_out <= {ROW_BITS{1'b0}};
                        addr_out[COL_BITS-1:0] <= latched_col;

                        req_accepted <= 1;
                        cmd_req      <= 0;

                        state <= latched_wr ? S_WAIT_TWR : S_WAIT_CL;
                    end
                end

                S_WAIT_CL: begin
                    cmd_req <= 0;
                    if (cl_cnt > 0) begin
                        cl_cnt <= cl_cnt - 1;
                    end else begin
                        data_valid <= 1;
                        if (close_after_access && can_precharge) begin
                            pending_cmd <= CMD_PRE;
                            state       <= S_PRECHARGE;
                        end else begin
                            state     <= S_IDLE;
                            req_ready <= 1;
                        end
                    end
                end

                S_WAIT_TWR: begin
                    cmd_req <= 0;
                    if (can_precharge) begin
                        if (close_after_access) begin
                            pending_cmd <= CMD_PRE;
                            state       <= S_PRECHARGE;
                        end else begin
                            state     <= S_IDLE;
                            req_ready <= 1;
                        end
                    end
                end

                default: begin
                    state   <= S_IDLE;
                    cmd_req <= 0;
                end
            endcase
        end
    end

endmodule

// ============================================
// bank_scheduler — FIXED
// FR-FCFS (First-Ready, First-Come-First-Served)
// ============================================
module bank_scheduler (
    input  wire clk,
    input  wire rst_n,

    // External request interface
    input  wire                  req_valid,
    input  wire                  req_wr,
    input  wire [ROW_BITS-1:0]   req_row,
    input  wire [COL_BITS-1:0]   req_col,
    output wire                  req_ready,

    // Bank state (from dram_bank_control or timing model)
    input  wire                  bank_active_actual,
    input  wire [ROW_BITS-1:0]   open_row_actual,

    // From bank_machine (to update prediction)
    input  wire                  activate_issued,
    input  wire                  precharge_issued,
    input  wire [ROW_BITS-1:0]   activate_row,

    // To bank_machine
    output reg                   bm_req_valid,
    output reg                   bm_req_wr,
    output reg [ROW_BITS-1:0]    bm_req_row,
    output reg [COL_BITS-1:0]    bm_req_col,
    output reg                   bm_do_precharge,
    output reg                   bm_do_activate,
    output reg                   bm_skip_activate,
    input  wire                  bm_req_ready
);

    parameter QUEUE_DEPTH = 4;
    parameter TICKET_BITS = 16;

    // ── Queue storage ─────────────────────────
    reg [QUEUE_DEPTH-1:0]        q_valid;
    reg [QUEUE_DEPTH-1:0]        q_wr;
    reg [ROW_BITS-1:0]           q_row   [0:QUEUE_DEPTH-1];
    reg [COL_BITS-1:0]           q_col   [0:QUEUE_DEPTH-1];
    reg [TICKET_BITS-1:0]        q_ticket[0:QUEUE_DEPTH-1];

    reg [TICKET_BITS-1:0] ticket_counter;

    // ── Predicted bank state ──────────────────
    // Track predicted state based on issued commands
    // (actual state lags by a few cycles due to timing)
    reg                   pred_bank_active;
    reg [ROW_BITS-1:0]    pred_open_row;

    // ── Full/empty ────────────────────────────
    assign req_ready = !(&q_valid); // not full

    // ── Enqueue: find first empty slot ────────
    reg [1:0] enq_idx;
    integer e;
    always @(*) begin
        enq_idx = 0;
        for (e = QUEUE_DEPTH-1; e >= 0; e = e - 1)
            if (!q_valid[e]) enq_idx = e;
    end

    // ── FR-FCFS selection ─────────────────────
    // Priority: row hits first, then oldest ticket
    reg [1:0]            sel_idx;
    reg                  sel_hit;
    reg [TICKET_BITS-1:0] min_ticket;
    reg [ROW_BITS-1:0]   sel_row;
    reg                  sel_wr;
    reg [COL_BITS-1:0]   sel_col;
    reg                  found;
    integer s;

    always @(*) begin
        sel_idx    = 0;
        sel_hit    = 0;
        sel_row    = 0;
        sel_wr     = 0;
        sel_col    = 0;
        min_ticket = {TICKET_BITS{1'b1}};
        found      = 0;

        // First pass: look for row hits
        for (s = 0; s < QUEUE_DEPTH; s = s + 1) begin
            if (q_valid[s] && pred_bank_active
                && (q_row[s] == pred_open_row)) begin
                if (!sel_hit || q_ticket[s] < min_ticket) begin
                    sel_hit    = 1;
                    min_ticket = q_ticket[s];
                    sel_idx    = s;
                    sel_row    = q_row[s];
                    sel_wr     = q_wr[s];
                    sel_col    = q_col[s];
                    found      = 1;
                end
            end
        end

        // Second pass: if no hits, oldest miss
        if (!sel_hit) begin
            min_ticket = {TICKET_BITS{1'b1}};
            for (s = 0; s < QUEUE_DEPTH; s = s + 1) begin
                if (q_valid[s] && q_ticket[s] < min_ticket) begin
                    min_ticket = q_ticket[s];
                    sel_idx    = s;
                    sel_row    = q_row[s];
                    sel_wr     = q_wr[s];
                    sel_col    = q_col[s];
                    found      = 1;
                end
            end
        end
    end

    wire request_available = |q_valid;
    wire row_hit_sched = pred_bank_active
                         && (sel_row == pred_open_row);

    // ── Sequential logic ─────────────────────
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_valid          <= 0;
            q_wr             <= 0;
            for (i = 0; i < QUEUE_DEPTH; i = i + 1) begin
                q_row[i]    <= 0;
                q_col[i]    <= 0;
                q_ticket[i] <= 0;
            end
            ticket_counter   <= 0;
            pred_bank_active <= 0;
            pred_open_row    <= 0;
            bm_req_valid     <= 0;
            bm_req_wr        <= 0;
            bm_req_row       <= 0;
            bm_req_col       <= 0;
            bm_do_precharge  <= 0;
            bm_do_activate   <= 0;
            bm_skip_activate <= 0;

        end else begin
            // Default
            bm_req_valid <= 0;

            // ── Update predicted bank state ───
            // FIX: Update BEFORE dequeue check
            // so page policy uses fresh prediction
            if (activate_issued) begin
                pred_bank_active <= 1;
                pred_open_row    <= activate_row;
            end else if (precharge_issued) begin
                pred_bank_active <= 0;
                pred_open_row    <= 0;
            end else if (!request_available) begin
                // Queue empty: sync prediction with actual
                pred_bank_active <= bank_active_actual;
                pred_open_row    <= open_row_actual;
            end

            // ── Enqueue new request ───────────
            if (req_valid && !(&q_valid)) begin
                q_valid[enq_idx]  <= 1;
                q_wr[enq_idx]     <= req_wr;
                q_row[enq_idx]    <= req_row;
                q_col[enq_idx]    <= req_col;
                q_ticket[enq_idx] <= ticket_counter;
                ticket_counter    <= ticket_counter + 1;
            end

            // ── Dequeue to bank_machine ───────
            // Only when bank_machine is ready AND queue has entries
            if (request_available && bm_req_ready && !bm_req_valid) begin
                bm_req_valid     <= 1;
                bm_req_wr        <= sel_wr;
                bm_req_row       <= sel_row;
                bm_req_col       <= sel_col;
                bm_skip_activate <= row_hit_sched;
                bm_do_precharge  <= pred_bank_active && !row_hit_sched;
                bm_do_activate   <= !row_hit_sched;
                q_valid[sel_idx] <= 0; // consume entry
            end
        end
    end

endmodule
