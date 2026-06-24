// ============================================
// bank_machine_array.v — 8 parallel bank FSMs + arbiter
// ============================================
`ifndef DDR3_PARAMS_V
`include "ddr3_params.v"
`endif

module bank_machine_array (
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,

    input  wire                  req_valid,
    input  wire                  req_wr,
    input  wire [`BANK_BITS-1:0]  req_bank,
    input  wire [`ROW_BITS-1:0]   req_row,
    input  wire [`COL_BITS-1:0]   req_col,
    output wire                  req_ready,

    input  wire [`NUM_BANKS-1:0]  can_activate_vec,
    input  wire [`NUM_BANKS-1:0]  can_read_vec,
    input  wire [`NUM_BANKS-1:0]  can_write_vec,
    input  wire [`NUM_BANKS-1:0]  can_precharge_vec,

    input  wire [`NUM_BANKS-1:0]  do_precharge_vec,
    input  wire [`NUM_BANKS-1:0]  do_activate_vec,
    input  wire [`NUM_BANKS-1:0]  skip_activate_vec,

    input  wire [`NUM_BANKS-1:0]  bank_active_vec,
    input  wire [`NUM_BANKS-1:0]  row_hit_vec,

    input  wire                  ref_req_bus,
    input  wire [2:0]            ref_cmd_out,
    output reg                   ref_granted,

    input  wire                  refresh_urgent,
    output reg                   refresh_ack,

    output reg        cs_n,
    output reg        ras_n,
    output reg        cas_n,
    output reg        we_n,
    output reg [`ROW_BITS-1:0]  addr,
    output reg [`BANK_BITS-1:0] ba,

    output wire activate_issued,
    output wire read_issued,
    output wire write_issued,
    output wire precharge_issued,

    output wire [`NUM_BANKS-1:0] bank_req_ready,
    output wire data_valid,
    output wire req_accepted
);

    localparam BANK_MASK = `NUM_BANKS - 1;

    initial begin
        if ((`NUM_BANKS & BANK_MASK) != 0)
            $error("FATAL: NUM_BANKS=%0d must be power of two", `NUM_BANKS);
    end

    wire                  bm_req_ready    [0:`NUM_BANKS-1];
    wire                  bm_req_valid    [0:`NUM_BANKS-1];
    wire                  bm_cmd_req      [0:`NUM_BANKS-1];
    wire                  bm_cmd_grant    [0:`NUM_BANKS-1];
    wire [2:0]            bm_cmd_out      [0:`NUM_BANKS-1];
    wire [`ROW_BITS-1:0]   bm_addr_out     [0:`NUM_BANKS-1];
    wire [`BANK_BITS-1:0]  bm_ba_out       [0:`NUM_BANKS-1];
    wire                  bm_activate_iss [0:`NUM_BANKS-1];
    wire                  bm_read_iss     [0:`NUM_BANKS-1];
    wire                  bm_write_iss    [0:`NUM_BANKS-1];
    wire                  bm_pre_iss      [0:`NUM_BANKS-1];
    wire                  bm_req_accepted [0:`NUM_BANKS-1];
    wire                  bm_data_valid   [0:`NUM_BANKS-1];
    wire                  bm_bank_idle    [0:`NUM_BANKS-1];
    wire [3:0]            bm_state_dbg    [0:`NUM_BANKS-1];

    genvar g;
    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : bm_route
            assign bm_req_valid[g] = req_valid && (req_bank == g[`BANK_BITS-1:0]);
        end
    endgenerate

    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : bm_inst
            bank_machine #(
                .BANK_ID(g)
            ) u_bm (
                .clk              (clk),
                .rst_n            (rst_n),
                .init_done        (init_done),
                .req_valid        (bm_req_valid[g]),
                .req_wr           (req_wr),
                .req_row          (req_row),
                .req_col          (req_col),
                .req_ready        (bm_req_ready[g]),
                .can_activate     (can_activate_vec[g]),
                .can_read         (can_read_vec[g]),
                .can_write        (can_write_vec[g]),
                .can_precharge    (can_precharge_vec[g]),
                .do_precharge     (do_precharge_vec[g]),
                .do_activate      (do_activate_vec[g]),
                .skip_activate    (skip_activate_vec[g]),
                .bank_active      (bank_active_vec[g]),
                .row_hit          (row_hit_vec[g]),
                .cmd_req          (bm_cmd_req[g]),
                .cmd_grant        (bm_cmd_grant[g]),
                .cmd_out          (bm_cmd_out[g]),
                .addr_out         (bm_addr_out[g]),
                .ba_out           (bm_ba_out[g]),
                .activate_issued  (bm_activate_iss[g]),
                .read_issued      (bm_read_iss[g]),
                .write_issued     (bm_write_iss[g]),
                .precharge_issued (bm_pre_iss[g]),
                .req_accepted     (bm_req_accepted[g]),
                .data_valid       (bm_data_valid[g]),
                .bank_idle        (bm_bank_idle[g]),
                .state_dbg        (bm_state_dbg[g])
            );
        end
    endgenerate

    assign req_ready = bm_req_ready[req_bank];

    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : ready_out
            assign bank_req_ready[g] = bm_req_ready[g];
        end
    endgenerate

    wire [`NUM_BANKS-1:0] activate_vec;
    wire [`NUM_BANKS-1:0] read_vec;
    wire [`NUM_BANKS-1:0] write_vec;
    wire [`NUM_BANKS-1:0] pre_vec;
    wire [`NUM_BANKS-1:0] data_vec;
    wire [`NUM_BANKS-1:0] accepted_vec;

    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : vec_gen
            assign activate_vec[g]  = bm_activate_iss[g];
            assign read_vec[g]      = bm_read_iss[g];
            assign write_vec[g]     = bm_write_iss[g];
            assign pre_vec[g]       = bm_pre_iss[g];
            assign data_vec[g]      = bm_data_valid[g];
            assign accepted_vec[g]  = bm_req_accepted[g];
        end
    endgenerate

    assign activate_issued  = |activate_vec;
    assign read_issued      = |read_vec;
    assign write_issued     = |write_vec;
    assign precharge_issued = |pre_vec;
    assign data_valid       = |data_vec;
    assign req_accepted     = |accepted_vec;

    wire [`NUM_BANKS-1:0] bank_idle_vec;
    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : idle_gen
            assign bank_idle_vec[g] = bm_bank_idle[g];
        end
    endgenerate
    wire all_banks_idle = &bank_idle_vec;

    reg [`BANK_BITS-1:0] rr_ptr;
    reg [`BANK_BITS-1:0] granted_bank;

    reg [`BANK_BITS-1:0] next_granted_bank;
    reg                 next_grant_valid;
    reg [`BANK_BITS-1:0] next_rr_ptr;

    integer k;
    always @(*) begin
        next_grant_valid  = 0;
        next_granted_bank = granted_bank;
        next_rr_ptr       = rr_ptr;

        if (!ref_req_bus) begin
            for (k = 0; k < `NUM_BANKS; k = k + 1) begin
                if (!next_grant_valid &&
                    bm_cmd_req[(rr_ptr + k) & BANK_MASK]) begin
                    next_grant_valid  = 1;
                    next_granted_bank = (rr_ptr + k) & BANK_MASK;
                    next_rr_ptr       = (rr_ptr + k + 1) & BANK_MASK;
                end
            end
        end
    end

    generate
        for (g = 0; g < `NUM_BANKS; g = g + 1) begin : grant_gen
            assign bm_cmd_grant[g] = next_grant_valid &&
                                     (next_granted_bank == g[`BANK_BITS-1:0]);
        end
    endgenerate

    reg [2:0]           arb_cmd_out;
    reg [`ROW_BITS-1:0]  arb_addr_out;
    reg [`BANK_BITS-1:0] arb_ba_out;

    always @(*) begin
        arb_cmd_out  = `CMD_NONE;
        arb_addr_out = {`ROW_BITS{1'b0}};
        arb_ba_out   = {`BANK_BITS{1'b0}};
        case (next_granted_bank)
            3'd0: begin arb_cmd_out = bm_cmd_out[0];  arb_addr_out = bm_addr_out[0];  arb_ba_out = bm_ba_out[0];  end
            3'd1: begin arb_cmd_out = bm_cmd_out[1];  arb_addr_out = bm_addr_out[1];  arb_ba_out = bm_ba_out[1];  end
            3'd2: begin arb_cmd_out = bm_cmd_out[2];  arb_addr_out = bm_addr_out[2];  arb_ba_out = bm_ba_out[2];  end
            3'd3: begin arb_cmd_out = bm_cmd_out[3];  arb_addr_out = bm_addr_out[3];  arb_ba_out = bm_ba_out[3];  end
            3'd4: begin arb_cmd_out = bm_cmd_out[4];  arb_addr_out = bm_addr_out[4];  arb_ba_out = bm_ba_out[4];  end
            3'd5: begin arb_cmd_out = bm_cmd_out[5];  arb_addr_out = bm_addr_out[5];  arb_ba_out = bm_ba_out[5];  end
            3'd6: begin arb_cmd_out = bm_cmd_out[6];  arb_addr_out = bm_addr_out[6];  arb_ba_out = bm_ba_out[6];  end
            3'd7: begin arb_cmd_out = bm_cmd_out[7];  arb_addr_out = bm_addr_out[7];  arb_ba_out = bm_ba_out[7];  end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr       <= 0;
            granted_bank <= 0;
            ref_granted  <= 0;
            refresh_ack  <= 0;
            cs_n         <= 1;
            ras_n        <= 1;
            cas_n        <= 1;
            we_n         <= 1;
            addr         <= {`ROW_BITS{1'b0}};
            ba           <= {`BANK_BITS{1'b0}};

        end else begin
            rr_ptr       <= next_rr_ptr;
            granted_bank <= next_granted_bank;
            ref_granted  <= 0;
            refresh_ack  <= 0;

            cs_n  <= 1'b0;
            ras_n <= 1'b1;
            cas_n <= 1'b1;
            we_n  <= 1'b1;
            addr  <= {`ROW_BITS{1'b0}};
            ba    <= {`BANK_BITS{1'b0}};

            if (ref_req_bus) begin
                ref_granted  <= 1;
                {ras_n,cas_n,we_n} <= ref_cmd_out;
                cs_n <= 1'b0;

            end else if (next_grant_valid) begin
                {ras_n,cas_n,we_n} <= arb_cmd_out;
                addr               <= arb_addr_out;
                ba                 <= arb_ba_out;
                cs_n               <= 1'b0;
            end

            if (refresh_urgent && all_banks_idle)
                refresh_ack <= 1;
        end
    end

endmodule
