// ============================================
// cpu_addr_decoder.v
// DDR3-1600K | 2Gb x8 device
// Source params: ddr3_params.v (ROW_BITS=15,
//                 COL_BITS=10, BANK_BITS=3)
//
// Linear address width = ROW_BITS+BANK_BITS+COL_BITS
//                       = 15 + 3 + 10 = 28 bits
//
// Address map:
//   [27:13] = Row    (15 bits → 32,768 rows)
//   [12:10] = Bank   (3 bits  → 8 banks)
//   [9:0]   = Column (10 bits → 1,024 cols)


`include "ddr3_params.v"

module cpu_addr_decoder (
    input  wire [ROW_BITS+BANK_BITS+COL_BITS-1:0] cpu_addr, // [27:0]
    output wire [ROW_BITS-1:0]  row_addr,   // [14:0]
    output wire [BANK_BITS-1:0] bank_addr,  // [2:0]
    output wire [COL_BITS-1:0]  col_addr    // [9:0]
);

    // Row    = upper bits
    assign row_addr  = cpu_addr[ROW_BITS+BANK_BITS+COL_BITS-1 : BANK_BITS+COL_BITS];
    // Bank   = middle bits
    assign bank_addr = cpu_addr[BANK_BITS+COL_BITS-1 : COL_BITS];
    // Column = lower bits
    assign col_addr  = cpu_addr[COL_BITS-1 : 0];

endmodule
