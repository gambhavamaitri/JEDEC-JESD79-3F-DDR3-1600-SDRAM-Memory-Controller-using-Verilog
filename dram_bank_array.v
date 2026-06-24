// ============================================
// dram_bank_array.v
// Behavioral DRAM bank model — simulation only
//
// Models one DDR3 bank:
//   - 2^ROW_BITS rows
//   - 2^COL_BITS columns
//   - DQ_WIDTH bits per column
//
// Operations:
//   ACTIVATE:  open row → copy to row buffer
//   READ:      return row_buffer[col]
//   WRITE:     row_buffer[col] = data (with DM)
//   PRECHARGE: write row buffer back → close row
//
// JEDEC behavior modeled:
//   - Must ACTIVATE before READ/WRITE
//   - Must PRECHARGE before next ACTIVATE
//   - DM=1 masks that byte (not written)
//   - Row buffer persists until PRECHARGE
// ============================================
`include "ddr3_params.v"

module dram_bank_array (
    input  wire clk,
    input  wire rst_n,

    // ── Commands (1-cycle pulses) ─────────────
    input  wire                  act_cmd,   // ACTIVATE
    input  wire                  rd_cmd,    // READ
    input  wire                  wr_cmd,    // WRITE
    input  wire                  pre_cmd,   // PRECHARGE

    // ── Address ───────────────────────────────
    input  wire [ROW_BITS-1:0]   row_addr,  // row (on ACT)
    input  wire [COL_BITS-1:0]   col_addr,  // col (on RD/WR)

    // ── Data ──────────────────────────────────
    input  wire [DQ_WIDTH-1:0]   wr_data,   // write data
    input  wire                  dm,         // data mask (1=mask)
    output reg  [DQ_WIDTH-1:0]   rd_data,   // read data

    // ── Status ───────────────────────────────
    output reg                   bank_active, // row is open
    output reg  [ROW_BITS-1:0]   open_row,    // which row is open
    output reg                   error_flag   // protocol violation
);

    // ── Memory array ─────────────────────────
    // [row][col] → DQ_WIDTH bits
    // Using 2D reg array
    reg [DQ_WIDTH-1:0] mem [0:(1<<ROW_BITS)-1][0:(1<<COL_BITS)-1];

    // ── Row buffer ────────────────────────────
    // Simulates sense amplifiers
    // Loaded on ACTIVATE, written back on PRECHARGE
    reg [DQ_WIDTH-1:0] row_buf [0:(1<<COL_BITS)-1];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank_active <= 0;
            open_row    <= 0;
            error_flag  <= 0;
            rd_data     <= 0;
        end else begin
            error_flag <= 0;

            if (act_cmd) begin
                if (bank_active) begin
                    // Protocol violation: ACT while active
                    error_flag <= 1;
                end else begin
                    // Load row into row buffer
                    for (i = 0; i < (1<<COL_BITS); i = i + 1)
                        row_buf[i] <= mem[row_addr][i];
                    bank_active <= 1;
                    open_row    <= row_addr;
                end
            end

            if (rd_cmd) begin
                if (!bank_active) begin
                    error_flag <= 1; // READ without ACTIVATE
                end else begin
                    rd_data <= row_buf[col_addr];
                end
            end

            if (wr_cmd) begin
                if (!bank_active) begin
                    error_flag <= 1; // WRITE without ACTIVATE
                end else begin
                    // DM=0 → write this byte
                    // DM=1 → mask this byte (keep old value)
                    if (!dm)
                        row_buf[col_addr] <= wr_data;
                end
            end

            if (pre_cmd) begin
                if (!bank_active) begin
                    // PRE on idle bank is allowed (NOP per JEDEC)
                end else begin
                    // Write row buffer back to memory
                    for (i = 0; i < (1<<COL_BITS); i = i + 1)
                        mem[open_row][i] <= row_buf[i];
                    bank_active <= 0;
                end
            end
        end
    end

endmodule
