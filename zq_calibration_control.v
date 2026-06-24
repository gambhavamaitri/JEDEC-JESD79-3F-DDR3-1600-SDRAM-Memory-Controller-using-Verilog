// ============================================
// zq_calibration_control.v

// ============================================
`include "ddr3_params.v"
module zq_calibration_control #(
    parameter TIMER_WIDTH = 10    // <-- add this
)(
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,
    input  wire zq_long_req,
    input  wire zq_short_req,

    output reg        cs_n,
    output reg        ras_n,
    output reg        cas_n,
    output reg        we_n,
    output reg [ROW_BITS-1:0] addr,

    output wire zq_busy,
    output reg  zq_done,
    output reg  zq_error,

    output wire [2:0]  current_state,
    output wire [TIMER_WIDTH-1:0] timer_dbg   // now valid
);

    
    localparam S_IDLE      = 3'd0;
    localparam S_ZQCL      = 3'd1;
    localparam S_WAIT_LONG = 3'd2;
    localparam S_ZQCS      = 3'd3;
    localparam S_WAIT_SHORT= 3'd4;
    localparam S_DONE      = 3'd5;

    reg [2:0] state;
    assign current_state = state;

    assign zq_busy = (state != S_IDLE);

    reg [TIMER_WIDTH-1:0] timer;
    assign timer_dbg = timer;

    localparam ZQCS_PERIOD = 27'd102_400_000;
    reg [26:0] periodic_cnt;

    reg zq_long_req_d, zq_short_req_d;
    wire zq_long_rise  = zq_long_req  & ~zq_long_req_d;
    wire zq_short_rise = zq_short_req & ~zq_short_req_d;

    reg zq_short_pending;  // SINGLE queue: user + periodic

    localparam [ROW_BITS-1:0] A10_MASK = {{(ROW_BITS-11){1'b0}}, 1'b1, 10'b0};

    wire periodic_expired = init_done && !zq_busy && (periodic_cnt == 0);

    always @(*) begin
        case (state)
            S_IDLE:       {cs_n, ras_n, cas_n, we_n, addr} = {1'b1, 1'b1, 1'b1, 1'b1, {ROW_BITS{1'b0}}};
            S_ZQCL:       {cs_n, ras_n, cas_n, we_n, addr} = {1'b0, 1'b1, 1'b1, 1'b0, A10_MASK};
            S_WAIT_LONG:  {cs_n, ras_n, cas_n, we_n, addr} = {1'b1, 1'b1, 1'b1, 1'b1, {ROW_BITS{1'b0}}};
            S_ZQCS:       {cs_n, ras_n, cas_n, we_n, addr} = {1'b0, 1'b1, 1'b1, 1'b0, {ROW_BITS{1'b0}}};
            S_WAIT_SHORT: {cs_n, ras_n, cas_n, we_n, addr} = {1'b1, 1'b1, 1'b1, 1'b1, {ROW_BITS{1'b0}}};
            S_DONE:       {cs_n, ras_n, cas_n, we_n, addr} = {1'b1, 1'b1, 1'b1, 1'b1, {ROW_BITS{1'b0}}};
            default:      {cs_n, ras_n, cas_n, we_n, addr} = {1'b1, 1'b1, 1'b1, 1'b1, {ROW_BITS{1'b0}}};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            timer            <= 0;
            zq_done          <= 1'b0;
            zq_error         <= 1'b0;
            zq_short_pending <= 1'b0;
            periodic_cnt     <= ZQCS_PERIOD - 1;

            zq_long_req_d    <= 1'b0;
            zq_short_req_d   <= 1'b0;
        end else begin
            zq_done  <= 1'b0;
            zq_error <= 1'b0;

            zq_long_req_d  <= zq_long_req;
            zq_short_req_d <= zq_short_req;

            // Queue short requests while busy (overflow detection)
            if (zq_busy && zq_short_rise) begin
                if (!zq_short_pending)
                    zq_short_pending <= 1'b1;
                else
                    zq_error <= 1'b1;   // Overflow
            end
            // Long request while busy: always error
            if (zq_busy && zq_long_rise)
                zq_error <= 1'b1;

            // SINGLE periodic mechanism: queue internally
            if (periodic_expired && !zq_short_pending)
                zq_short_pending <= 1'b1;

            // Counter management (no external pulse)
            if (init_done && !zq_busy) begin
                if (periodic_cnt > 0)
                    periodic_cnt <= periodic_cnt - 1;
                else
                    periodic_cnt <= ZQCS_PERIOD - 1;
            end

            case (state)

                S_IDLE: begin
                    if (zq_long_rise && zq_short_rise) begin
                        zq_short_pending <= 1'b1;   // queue ZQCS
                        state            <= S_ZQCL;
                    end else if (zq_long_rise) begin
                        state <= S_ZQCL;
                    end else if (zq_short_rise || zq_short_pending) begin
                        zq_short_pending <= 1'b0;
                        state            <= S_ZQCS;
                    end
                end

                S_ZQCL: begin
                    timer <= tZQINIT - 1;
                    state <= S_WAIT_LONG;
                end

                S_WAIT_LONG: begin
                    if (timer > 0) timer <= timer - 1;
                    else           state <= S_DONE;
                end

                S_ZQCS: begin
                    timer <= tZQCS - 1;
                    state <= S_WAIT_SHORT;
                end

                S_WAIT_SHORT: begin
                    if (timer > 0) timer <= timer - 1;
                    else           state <= S_DONE;
                end

                S_DONE: begin
                    if (zq_short_pending) begin
                        zq_short_pending <= 1'b0;
                        state            <= S_ZQCS;
                    end else begin
                        zq_done <= 1'b1;
                        state   <= S_IDLE;
                    end
                end

                default: begin
                    zq_error <= 1'b1;
                    state    <= S_IDLE;
                end
            endcase
        end
    end

endmodule
