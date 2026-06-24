// ============================================
// request_queue.v
// Simple synchronous FIFO for host requests
// Operates in DDR3 clock domain (after CDC FIFO)
// Depth = 8 entries
// Entry = {wr_flag, addr[27:0], wdata[63:0]}
//
// Used AFTER fifo_async (CDC already done here)
// Row hit reordering: optional (not implemented
// here — add as enhancement)
// ============================================
`include "ddr3_params.v"

module request_queue #(
    parameter DEPTH    = 8,
    parameter ADDR_W   = 28,
    parameter DATA_W   = 64
)(
    input  wire clk,
    input  wire rst_n,

    // Push side (from interface_control_unit, DDR clk domain)
    input  wire               push,
    input  wire               req_wr,       // 1=write, 0=read
    input  wire [ADDR_W-1:0]  req_addr,
    input  wire [DATA_W-1:0]  req_wdata,
    output wire               full,
    output wire               almost_full,  // DEPTH-2 entries used

    // Pop side (to control_unit, same DDR clk domain)
    input  wire               pop,
    output wire               req_wr_out,
    output wire [ADDR_W-1:0]  req_addr_out,
    output wire [DATA_W-1:0]  req_wdata_out,
    output wire               empty,
    output wire               valid,          // entry at head is valid

    // Status
    output wire [$clog2(DEPTH):0] count       // current occupancy
);

    localparam PTR_W = $clog2(DEPTH);
    localparam ENTRY_W = 1 + ADDR_W + DATA_W; // wr_flag + addr + wdata

    // ── Memory ────────────────────────────────
    reg [ENTRY_W-1:0] mem [0:DEPTH-1];

    // ── Pointers ──────────────────────────────
    reg [PTR_W-1:0]   wr_ptr;
    reg [PTR_W-1:0]   rd_ptr;
    reg [$clog2(DEPTH):0] cnt;

    // ── Write ─────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
        end else if (push && !full) begin
            mem[wr_ptr] <= {req_wr, req_addr, req_wdata};
            wr_ptr      <= wr_ptr + 1;
        end
    end

    // ── Read ──────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= 0;
        end else if (pop && !empty) begin
            rd_ptr <= rd_ptr + 1;
        end
    end

    // ── Count ─────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0;
        end else begin
            case ({push && !full, pop && !empty})
                2'b10: cnt <= cnt + 1;
                2'b01: cnt <= cnt - 1;
                default: cnt <= cnt;
            endcase
        end
    end

    // ── Combinational outputs ─────────────────
    assign full        = (cnt == DEPTH);
    assign almost_full = (cnt >= DEPTH - 2);
    assign empty       = (cnt == 0);
    assign valid       = !empty;
    assign count       = cnt;

    // Head of queue (combinational read)
    assign {req_wr_out, req_addr_out, req_wdata_out} = mem[rd_ptr];

endmodule
