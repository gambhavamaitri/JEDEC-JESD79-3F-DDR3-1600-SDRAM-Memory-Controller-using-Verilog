// ============================================
// datapath_control_logic.v

//
// WHAT IT DOES:
//   After READ command issued: waits exactly CL
//   cycles then enables deserializer capture
//
//   After WRITE command issued: waits exactly CWL
//   cycles then enables serializer output
//
//   Also generates DQS gate window:
//   Only capture DQ during valid burst window
//   (prevents false capture during preamble/
//    postamble)
//
// WHY SEPARATE FROM control_unit:
//   control_unit knows WHEN commands are issued
//   datapath_control knows WHAT DATA timing to use
//   Clean separation: command path vs data path
//
// JEDEC timing:
//   Read: data appears CL cycles after READ cmd
//   Write: controller must present data CWL cycles
//          after WRITE cmd (Write Latency = AL+CWL)
// ============================================
`include "ddr3_params.v"

module datapath_control_logic (
    input  wire clk,
    input  wire rst_n,

    // ── Command notifications from control_unit ─
    input  wire read_issued,   // 1-cycle pulse: READ cmd sent
    input  wire write_issued,  // 1-cycle pulse: WRITE cmd sent

    // ── Outputs to datapath.v ─────────────────
    output reg  serializer_en,   // enable write data output to DQ
    output reg  deserializer_en, // enable read data capture from DQ
    output reg  dqs_gate_en,     // open DQS capture window
    output reg  wr_fifo_rd_en,   // read from write FIFO
    output reg  rd_fifo_wr_en,   // write to read FIFO

    // ── Data valid to control_unit ────────────
    output reg  data_valid_out   // read data is ready
);

    // ── CL pipeline (READ path) ───────────────
    // Shift register: 1 travels through CL stages
    // When it exits: data is valid on DQ pins
    reg [CL-1:0]  cl_pipe;

    // ── CWL pipeline (WRITE path) ─────────────
    // Shift register: 1 travels through CWL stages
    // When it exits: time to drive data onto DQ
    reg [CWL-1:0] cwl_pipe;

    // ── Burst window counter ──────────────────
    // Counts BL/2 = 4 cycles after data starts
    // (DDR: 8 transfers = 4 clock cycles)
    reg [2:0] burst_cnt;
    reg       burst_active;

    // ── CWL burst window counter ─────────────
    reg [2:0] wr_burst_cnt;
    reg       wr_burst_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cl_pipe          <= 0;
            cwl_pipe         <= 0;
            burst_cnt        <= 0;
            burst_active     <= 0;
            wr_burst_cnt     <= 0;
            wr_burst_active  <= 0;
            serializer_en    <= 0;
            deserializer_en  <= 0;
            dqs_gate_en      <= 0;
            wr_fifo_rd_en    <= 0;
            rd_fifo_wr_en    <= 0;
            data_valid_out   <= 0;

        end else begin

            // ── Default deasserts ──────────────
            serializer_en  <= 0;
            deserializer_en<= 0;
            dqs_gate_en    <= 0;
            wr_fifo_rd_en  <= 0;
            rd_fifo_wr_en  <= 0;
            data_valid_out <= 0;

            // ── READ path ─────────────────────
            // Load pipeline with 1 when READ issued
            // Shift right each cycle
            // When MSB exits: data appears on DQ
            cl_pipe <= {cl_pipe[CL-2:0], read_issued};

            // CL-th stage: data valid window starts
            if (cl_pipe[CL-1]) begin
                burst_active     <= 1;
                burst_cnt        <= BL/2 - 1; // 4 cycles
            end

            // During burst window: enable capture
            if (burst_active) begin
                deserializer_en <= 1;
                dqs_gate_en     <= 1;    // open DQS gate
                rd_fifo_wr_en   <= 1;    // push to read FIFO
                data_valid_out  <= 1;    // tell control_unit

                if (burst_cnt > 0)
                    burst_cnt <= burst_cnt - 1;
                else
                    burst_active <= 0;   // close window
            end

            // ── WRITE path ────────────────────
            // Load pipeline with 1 when WRITE issued
            // Shift right each cycle
            // When MSB exits: time to drive DQ
            cwl_pipe <= {cwl_pipe[CWL-2:0], write_issued};

            // CWL-th stage: start driving data
            if (cwl_pipe[CWL-1]) begin
                wr_burst_active <= 1;
                wr_burst_cnt    <= BL/2 - 1; // 4 cycles
            end

            // During write burst window: enable drive
            if (wr_burst_active) begin
                serializer_en  <= 1;    // drive DQ
                wr_fifo_rd_en  <= 1;    // read from write FIFO
                dqs_gate_en    <= 1;    // drive DQS

                if (wr_burst_cnt > 0)
                    wr_burst_cnt <= wr_burst_cnt - 1;
                else
                    wr_burst_active <= 0;
            end
        end
    end

endmodule
