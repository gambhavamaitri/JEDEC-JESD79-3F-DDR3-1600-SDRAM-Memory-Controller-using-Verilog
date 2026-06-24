// ============================================
// control_unit.v 
// ============================================
`include "ddr3_params.v"

module control_unit (
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,

    // ── From request_queue ────────────────────
    input  wire                  host_req,
    input  wire                  host_wr,
    input  wire [ROW_BITS-1:0]   host_row,
    input  wire [BANK_BITS-1:0]  host_bank,
    input  wire [COL_BITS-1:0]   host_col,
    output reg                   host_ready,  // handshake: request accepted
    output reg                   data_valid,  // read data ready

    // ── From timing_engine ────────────────────
    input  wire can_activate,
    input  wire can_read,
    input  wire can_write,
    input  wire can_precharge,
    input  wire can_refresh,

    // ── From dram_bank_control ────────────────
    input  wire bank_active,       // target bank is ACTIVE (need PRE before re-ACTIVATE)
    input  wire row_hit,
    input  wire [NUM_BANKS-1:0] bank_idle_all,

    // ── From row_tracker ─────────────────────
    input  wire do_precharge,
    input  wire do_activate,
    input  wire skip_activate,

    // ── From refresh_control_logic ────────────
    input  wire ref_req,
    input  wire ref_urgent,
    input  wire ref_busy,
    output reg  ref_ack,

    // ── DDR3 command bus ─────────────────────
    output reg        cs_n,
    output reg        ras_n,
    output reg        cas_n,
    output reg        we_n,
    output reg [ROW_BITS-1:0]  addr,
    output reg [BANK_BITS-1:0] ba,

    // ── Notification pulses (1 cycle each) ────
    output reg [BANK_BITS-1:0] cmd_bank_sel,
    output reg activate_issued,
    output reg read_issued,
    output reg write_issued,
    output reg precharge_issued,
    output reg refresh_issued,
    output reg act_cmd,
    output reg pre_cmd,
    output reg ref_cmd,

    // ── Debug ────────────────────────────────
    output reg [3:0] state_out,
    output reg       busy
);

    // ── States ────────────────────────────────
    localparam S_IDLE           = 4'd0;
    localparam S_PRECHARGE      = 4'd1;
    localparam S_WAIT_TRP       = 4'd2;
    localparam S_ACTIVATE       = 4'd3;
    localparam S_WAIT_TRCD      = 4'd4;
    localparam S_READ_WRITE     = 4'd5;
    localparam S_WAIT_CL        = 4'd6;
    localparam S_WAIT_TWR       = 4'd7;
    localparam S_DATA           = 4'd8;
    localparam S_REFRESH        = 4'd9;
    localparam S_WAIT_TRFC      = 4'd10;
    localparam S_PRECHARGE_ALL  = 4'd11;
    localparam S_WAIT_TRP_ALL   = 4'd12;

    reg [3:0] state;

    // ── Latched request ───────────────────────
    reg                  latched_wr;
    reg [ROW_BITS-1:0]   latched_row;
    reg [BANK_BITS-1:0]  latched_bank;
    reg [COL_BITS-1:0]   latched_col;
    reg                  req_pending;

    // ── Latched row_tracker decisions ─────────
    reg latched_do_precharge;
    reg latched_do_activate;
    reg latched_skip;
    reg close_page;

    // ── CL counter ────────────────────────────
    reg [$clog2(CL+1)-1:0] cl_cnt;
    reg                     cl_active;

    // ── Refresh aging ─────────────────────────
    localparam REFRESH_AGE_LIMIT = 64;
    reg [$clog2(REFRESH_AGE_LIMIT+1)-1:0] refresh_age;
    wire refresh_force = (refresh_age >= REFRESH_AGE_LIMIT) && ref_req;

    // ── Bank idle reduction ───────────────────
    wire all_banks_idle = &bank_idle_all;

    // ── Precharge-all address mask ────────────
    // A10=1, all others 0. Build once, use in S_PRECHARGE_ALL.
    localparam [ROW_BITS-1:0] PRECHARGE_ALL_MASK = (1'b1 << 10);

    // ── Combinational command outputs ─────────
    always @(*) begin
        case (state)
            S_PRECHARGE,
            S_PRECHARGE_ALL: begin
                cs_n=1'b0; ras_n=1'b0; cas_n=1'b1; we_n=1'b0;
            end
            S_ACTIVATE: begin
                cs_n=1'b0; ras_n=1'b0; cas_n=1'b1; we_n=1'b1;
            end
            S_READ_WRITE: begin
                cs_n=1'b0; ras_n=1'b1; cas_n=1'b0;
                we_n = latched_wr ? 1'b0 : 1'b1;
            end
            S_REFRESH: begin
                cs_n=1'b0; ras_n=1'b0; cas_n=1'b0; we_n=1'b1;
            end
            default: begin
                cs_n=1'b0; ras_n=1'b1; cas_n=1'b1; we_n=1'b1;
            end
        endcase
    end

    // ── Sequential FSM ────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            busy             <= 0;
            host_ready       <= 0;
            data_valid       <= 0;
            ref_ack          <= 0;
            activate_issued  <= 0;
            read_issued      <= 0;
            write_issued     <= 0;
            precharge_issued <= 0;
            refresh_issued   <= 0;
            act_cmd          <= 0;
            pre_cmd          <= 0;
            ref_cmd          <= 0;
            cmd_bank_sel     <= 0;
            addr             <= 0;
            ba               <= 0;
            latched_wr       <= 0;
            latched_row      <= 0;
            latched_bank     <= 0;
            latched_col      <= 0;
            req_pending      <= 0;
            close_page       <= 0;
            latched_do_precharge <= 0;
            latched_do_activate  <= 0;
            latched_skip     <= 0;
            cl_cnt           <= 0;
            cl_active        <= 0;
            state_out        <= 0;
            refresh_age      <= 0;

        end else begin

            // Default: deassert all 1-cycle pulses
            host_ready       <= 0;
            data_valid       <= 0;
            ref_ack          <= 0;
            activate_issued  <= 0;
            read_issued      <= 0;
            write_issued     <= 0;
            precharge_issued <= 0;
            refresh_issued   <= 0;
            act_cmd          <= 0;
            pre_cmd          <= 0;
            ref_cmd          <= 0;
            state_out        <= state;

            // Refresh age: saturate while request pending; clear when serviced
            if (ref_ack || refresh_issued) begin
                refresh_age <= 0;
            end else if (ref_req && (refresh_age < REFRESH_AGE_LIMIT)) begin
                refresh_age <= refresh_age + 1;
            end

            // CL countdown
            if (cl_active) begin
                if (cl_cnt > 0)
                    cl_cnt <= cl_cnt - 1;
                else
                    cl_active <= 0;
            end

            case (state)

                // ─────────────────────────────
                // PRIORITY 0: refresh_force (breaks everything)
                // PRIORITY 1: ref_urgent
                // PRIORITY 2: req_pending continue
                // PRIORITY 3: new host request
                // PRIORITY 4: convenient refresh
                // ─────────────────────────────
                S_IDLE: begin
                    busy <= 0;

                    if (!init_done) begin
                        // Wait for initialization FSM

                    // Forced refresh: preempt host even if req_pending
                    end else if (refresh_force && can_refresh) begin
                        if (all_banks_idle)
                            state <= S_REFRESH;
                        else
                            state <= S_PRECHARGE_ALL;

                    // Urgent refresh: credits critical
                    end else if (ref_urgent && can_refresh) begin
                        if (all_banks_idle)
                            state <= S_REFRESH;
                        else
                            state <= S_PRECHARGE_ALL;

                    // Continue previously accepted request
                    end else if (req_pending) begin
                        busy <= 1;
                        if (latched_skip) begin
                            if (!latched_wr && can_read)
                                state <= S_READ_WRITE;
                            else if (latched_wr && can_write)
                                state <= S_READ_WRITE;
                        end else if (latched_do_precharge && can_precharge) begin
                            state <= S_PRECHARGE;
                        end else if (latched_do_activate && can_activate) begin
                            state <= S_ACTIVATE;
                        end
                        // else: gate closed, stay IDLE with req_pending=1

                    // Accept new host request
                    end else if (host_req) begin
                        req_pending          <= 1;
                        busy                 <= 1;
                        latched_wr           <= host_wr;
                        latched_row          <= host_row;
                        latched_bank         <= host_bank;
                        latched_col          <= host_col;
                        cmd_bank_sel         <= host_bank;
                        latched_do_precharge <= do_precharge;
                        latched_do_activate  <= do_activate;
                        latched_skip         <= skip_activate;
                        close_page           <= do_precharge;
                        host_ready           <= 1;

                        if (skip_activate) begin
                            if (!host_wr && can_read)
                                state <= S_READ_WRITE;
                            else if (host_wr && can_write)
                                state <= S_READ_WRITE;
                        end else if (do_precharge && can_precharge) begin
                            state <= S_PRECHARGE;
                        end else if (do_activate && can_activate) begin
                            state <= S_ACTIVATE;
                        end
                        // else: gate closed, req_pending=1, wait in IDLE

                    // Convenient refresh (idle time only)
                    end else if (ref_req && all_banks_idle && can_refresh) begin
                        state <= S_REFRESH;
                    end
                end

                // ─────────────────────────────
                // Per-bank precharge (row miss)
                // Guard: only issue if bank is actually active.
                // dram_bank_control should prevent double-precharge,
                // but we protect here too.
                // ─────────────────────────────
                S_PRECHARGE: begin
                    if (bank_active) begin
                        ba               <= latched_bank;
                        addr             <= {ROW_BITS{1'b0}}; // A10=0, single-bank
                        pre_cmd          <= 1;
                        precharge_issued <= 1;
                        cmd_bank_sel     <= latched_bank;
                    end
                    state <= S_WAIT_TRP;
                end

                S_WAIT_TRP: begin
                    if (can_activate)
                        state <= S_ACTIVATE;
                end

                // ─────────────────────────────
                // Activate
                // ─────────────────────────────
                S_ACTIVATE: begin
                    ba              <= latched_bank;
                    addr            <= latched_row;
                    act_cmd         <= 1;
                    activate_issued <= 1;
                    cmd_bank_sel    <= latched_bank;
                    state           <= S_WAIT_TRCD;
                end

                S_WAIT_TRCD: begin
                    if (!latched_wr && can_read)
                        state <= S_READ_WRITE;
                    else if (latched_wr && can_write)
                        state <= S_READ_WRITE;
                end

                // ─────────────────────────────
                // Issue READ or WRITE
                // ─────────────────────────────
                S_READ_WRITE: begin
                    ba <= latched_bank;
                    addr <= {{(ROW_BITS-COL_BITS-1){1'b0}},
                             1'b0, latched_col};

                    if (latched_wr) begin
                        write_issued <= 1;
                        state        <= S_WAIT_TWR;
                    end else begin
                        read_issued <= 1;
                        cl_cnt      <= (CL > 0) ? (CL - 1) : 0;
                        cl_active   <= 1;
                        state       <= S_WAIT_CL;
                    end

                    req_pending <= 0;
                end

                // ─────────────────────────────
                // Wait CL for read data
                // ─────────────────────────────
                S_WAIT_CL: begin
                    if (!cl_active)
                        state <= S_DATA;
                end

                // ─────────────────────────────
                // Read data valid
                // Guard precharge: only if bank still active.
                // ─────────────────────────────
                S_DATA: begin
                    data_valid <= 1;

                    if (refresh_force && all_banks_idle && can_refresh) begin
                        state <= S_REFRESH;
                    end else if (close_page && bank_active && can_precharge) begin
                        state <= S_PRECHARGE;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                // ─────────────────────────────
                // Wait tWR / tRAS for write
                // Guard precharge: only if bank still active.
                // ─────────────────────────────
                S_WAIT_TWR: begin
                    if (can_precharge) begin
                        if (refresh_force && all_banks_idle && can_refresh) begin
                            state <= S_REFRESH;
                        end else if (close_page && bank_active) begin
                            state <= S_PRECHARGE;
                        end else begin
                            state <= S_IDLE;
                        end
                    end
                end

                // ─────────────────────────────
                // Issue REFRESH
                // ref_ack here: scheduler knows command is on the bus.
                // ─────────────────────────────
                S_REFRESH: begin
                    ref_ack        <= 1;
                    ref_cmd        <= 1;
                    refresh_issued <= 1;
                    state          <= S_WAIT_TRFC;
                end

                // ─────────────────────────────
                // Wait tRFC
                // Loop back only if ref_urgent still asserted AND
                // refresh scheduler has more credits to drain.
                // If scheduler keeps ref_urgent high forever, this
                // is a scheduler bug, not a control_unit bug.
                // ─────────────────────────────
                S_WAIT_TRFC: begin
                    if (!ref_busy && can_refresh) begin
                        if (ref_urgent && all_banks_idle && can_refresh)
                            state <= S_REFRESH;
                        else
                            state <= S_IDLE;
                    end
                end

                // ─────────────────────────────
                // Precharge all banks (for refresh)
                // Single NBA to addr. A10=1, all others 0.
                // ─────────────────────────────
                S_PRECHARGE_ALL: begin
                    addr             <= PRECHARGE_ALL_MASK;
                    ba               <= {BANK_BITS{1'b0}};
                    pre_cmd          <= 1;
                    precharge_issued <= 1;
                    cmd_bank_sel     <= {BANK_BITS{1'b0}};
                    state            <= S_WAIT_TRP_ALL;
                end

                S_WAIT_TRP_ALL: begin
                    if (can_activate && can_refresh)
                        state <= S_REFRESH;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
