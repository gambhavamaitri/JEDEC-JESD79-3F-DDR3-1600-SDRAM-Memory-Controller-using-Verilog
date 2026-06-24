// ============================================
// tri_state_converter.v
// Bidirectional DQ bus interface
//
// PURPOSE:
//   DDR3 DQ pins are bidirectional — shared
//   between controller (write) and DRAM (read).
//   Only ONE driver allowed at a time.
//
// WRITE mode (oe=1):
//   Controller drives data_out onto DQ bus
//   DRAM's output drivers are in high-Z
//
// READ mode (oe=0):
//   Controller releases DQ (high-Z)
//   DRAM drives read data onto DQ bus
//   data_in captures whatever is on DQ
//
// JEDEC: DQ is bidirectional per Figure 1
//   "During READ: DQ driven by SDRAM"
//   "During WRITE: DQ driven by controller"
//
// WHY HIGH-Z:
//   If both controller AND DRAM drive DQ
//   simultaneously → bus contention → current
//   spike → potential latch-up → data corruption
//   High-Z disconnects our driver safely
// ============================================
`include "ddr3_params.v"

module tri_state_converter (
    input  wire [DQ_WIDTH-1:0] data_out,  // from datapath (write data)
    input  wire                oe,         // output enable
                                           // 1 = controller drives DQ
                                           // 0 = controller releases DQ
    inout  wire [DQ_WIDTH-1:0] dq,         // DDR3 DQ pin (bidirectional)
    output wire [DQ_WIDTH-1:0] data_in     // to datapath (captured read data)
);

    // ── Drive DQ during write, release during read ──
    // oe=1: dq = data_out  (controller drives)
    // oe=0: dq = 8'bz      (high impedance, DRAM drives)
    assign dq = oe ? data_out : {DQ_WIDTH{1'bz}};

    // ── Always capture what is on DQ bus ───────────
    // During write: captures our own driven data
    //   (useful for loopback verification)
    // During read:  captures DRAM's driven data
    //   (this is the actual read data)
    assign data_in = dq;

endmodule
