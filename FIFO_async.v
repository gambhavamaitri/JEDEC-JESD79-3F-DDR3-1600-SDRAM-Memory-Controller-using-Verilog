// ============================================
// FIFO_async.v — Async FIFO for datapath.v
// Clifford Cummings-style gray-code CDC
// Port names match datapath.v instantiation
// ============================================

module fifo_async_burst #(
    parameter DEPTH = 16,
    parameter WIDTH = 8
)(
    // Write port (wclk domain)
    input  wire             wclk,
    input  wire             wrst_n,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wdata,
    output wire             wfull,

    // Read port (rclk domain)
    input  wire             rclk,
    input  wire             rrst_n,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rdata,
    output wire             rempty
);

    localparam ADDR_WIDTH = $clog2(DEPTH);

    // ----------------------------------------------------------------
    // Memory
    // ----------------------------------------------------------------
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // ----------------------------------------------------------------
    // Write pointer (binary + gray) — wclk domain
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH:0] wptr_bin, wptr_gray;
    wire [ADDR_WIDTH:0] wptr_bin_next  = wptr_bin  + (wr_en & ~wfull);
    wire [ADDR_WIDTH:0] wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin  <= 0;
            wptr_gray <= 0;
        end else begin
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
        end
    end

    // Write address (truncated)
    wire [ADDR_WIDTH-1:0] waddr = wptr_bin[ADDR_WIDTH-1:0];

    // Memory write
    always @(posedge wclk) begin
        if (wr_en & ~wfull)
            mem[waddr] <= wdata;
    end

    // ----------------------------------------------------------------
    // Read pointer (binary + gray) — rclk domain
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    wire [ADDR_WIDTH:0] rptr_bin_next  = rptr_bin  + (rd_en & ~rempty);
    wire [ADDR_WIDTH:0] rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin  <= 0;
            rptr_gray <= 0;
        end else begin
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
        end
    end

    // Read address (truncated)
    wire [ADDR_WIDTH-1:0] raddr = rptr_bin[ADDR_WIDTH-1:0];

    // Memory read (registered or combinational — combinational for low latency)
    assign rdata = mem[raddr];

    // ----------------------------------------------------------------
    // CDC Synchronizers (2-FF)
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH:0] wq1_rptr, wq2_rptr; // sync read ptr to write clk
    reg [ADDR_WIDTH:0] rq1_wptr, rq2_wptr; // sync write ptr to read clk

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) {wq2_rptr, wq1_rptr} <= 0;
        else          {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr_gray};
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) {rq2_wptr, rq1_wptr} <= 0;
        else          {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr_gray};
    end

    // ----------------------------------------------------------------
    // Full / Empty flags
    // ----------------------------------------------------------------
    // Full: next write gray ptr == {~synced_read_ptr[MSB:MSB-1], synced_read_ptr[MSB-2:0]}
    wire [ADDR_WIDTH:0] wfull_val = (wptr_gray_next == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1],
                                                          wq2_rptr[ADDR_WIDTH-2:0]});
    reg wfull_reg;
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) wfull_reg <= 1'b0;
        else         wfull_reg <= wfull_val;
    end
    assign wfull = wfull_reg;

    // Empty: next read gray ptr == synced write ptr
    wire rempty_val = (rptr_gray_next == rq2_wptr);
    reg rempty_reg;
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) rempty_reg <= 1'b1;
        else         rempty_reg <= rempty_val;
    end
    assign rempty = rempty_reg;

endmodule
