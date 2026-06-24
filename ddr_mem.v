// ============================================
// ddr_mem.v
// DDR3 DRAM chip behavioral model
// Simulation only — not synthesizable
//
// Models a complete DDR3 x8 device:
//   - Command decoder ({RAS#,CAS#,WE#})
//   - Mode register storage (MR0-MR3)
//   - 8 independent bank arrays
//   - DQ/DQS burst interface
//   - Timing violation checker
//
// Used by tb_mem_controller to verify
// that the controller issues correct
// DDR3 command sequences and data is
// stored/returned correctly.
//
// JEDEC JESD79-3F compliant command decode
// ============================================
`include "ddr3_params.v"

module ddr_mem (
    // ── Clock ────────────────────────────────
    input  wire clk,    // CK (positive)
    input  wire clk_n,  // CK# (negative) — unused in model
    input  wire rst_n,  // RESET# pin

    // ── Control pins ─────────────────────────
    input  wire        cs_n,   // Chip Select (active low)
    input  wire        ras_n,  // RAS#
    input  wire        cas_n,  // CAS#
    input  wire        we_n,   // WE#
    input  wire        cke,    // Clock Enable

    // ── Address ───────────────────────────────
    input  wire [ROW_BITS-1:0]  addr,  // A[14:0]
    input  wire [BANK_BITS-1:0] ba,    // BA[2:0]

    // ── Data ──────────────────────────────────
    inout  wire [DQ_WIDTH-1:0]  dq,    // DQ[7:0]
    input  wire                 dm,    // DM (write mask)
    inout  wire                 dqs,   // DQS strobe
    inout  wire                 dqs_n, // DQS# strobe

    // ── ODT ───────────────────────────────────
    input  wire odt,    // On-Die Termination

    // ── Debug outputs ─────────────────────────
    output reg [3:0]  mode_reg [0:3],  // MR0-MR3 (abbreviated)
    output wire [7:0] bank_active_vec, // which banks are active
    output reg        init_complete    // initialization done flag
);

    // ── Command encoding ──────────────────────
    localparam CMD_MRS       = 3'b000;
    localparam CMD_REFRESH   = 3'b001;
    localparam CMD_PRECHARGE = 3'b010;
    localparam CMD_ACTIVATE  = 3'b011;
    localparam CMD_WRITE     = 3'b100;
    localparam CMD_READ      = 3'b101;
    localparam CMD_ZQCAL     = 3'b110;
    localparam CMD_NOP       = 3'b111;
    localparam CMD_DESELECT  = 3'b111; // cs_n=1

    // ── Command decode ────────────────────────
    wire [2:0] cmd = cs_n ? CMD_DESELECT : {ras_n, cas_n, we_n};

    // ── 8 bank array instances ─────────────────
    wire [DQ_WIDTH-1:0] bank_rd_data [0:NUM_BANKS-1];
    wire [NUM_BANKS-1:0] bank_active_w;
    wire [ROW_BITS-1:0]  bank_open_row [0:NUM_BANKS-1];
    wire [NUM_BANKS-1:0] bank_error;

    // Per-bank command routing
    wire [NUM_BANKS-1:0] bank_act;
    wire [NUM_BANKS-1:0] bank_rd;
    wire [NUM_BANKS-1:0] bank_wr;
    wire [NUM_BANKS-1:0] bank_pre;

    genvar g;
    generate
        for (g = 0; g < NUM_BANKS; g = g + 1) begin : banks
            // Only route command to selected bank
            assign bank_act[g] = (cmd==CMD_ACTIVATE)  && (ba==g);
            assign bank_rd[g]  = (cmd==CMD_READ)       && (ba==g);
            assign bank_wr[g]  = (cmd==CMD_WRITE)      && (ba==g);
            // PRECHARGE: A10=1 → all banks, A10=0 → selected bank
            assign bank_pre[g] = (cmd==CMD_PRECHARGE)
                                  && (addr[10] || (ba==g));

            dram_bank_array u_bank (
                .clk        (clk),
                .rst_n      (rst_n),
                .act_cmd    (bank_act[g]),
                .rd_cmd     (bank_rd[g]),
                .wr_cmd     (bank_wr[g]),
                .pre_cmd    (bank_pre[g]),
                .row_addr   (addr[ROW_BITS-1:0]),
                .col_addr   (addr[COL_BITS-1:0]),
                .wr_data    (dq_reg),
                .dm         (dm),
                .rd_data    (bank_rd_data[g]),
                .bank_active(bank_active_w[g]),
                .open_row   (bank_open_row[g]),
                .error_flag (bank_error[g])
            );
        end
    endgenerate

    assign bank_active_vec = bank_active_w;

    // ── DQ registration ──────────────────────
    // Capture DQ on write, drive on read
    reg [DQ_WIDTH-1:0] dq_reg;
    reg [DQ_WIDTH-1:0] dq_drive;
    reg                dq_oe;     // drive DQ when reading

    // Read burst state
    reg [DQ_WIDTH-1:0] rd_data_latched;
    reg [3:0]          rd_burst_cnt;
    reg                rd_burst_active;
    reg [3:0]          rd_latency_cnt; // CL countdown

    // Write burst state
    reg [3:0]          wr_burst_cnt;
    reg                wr_burst_active;
    reg [3:0]          wr_latency_cnt; // CWL countdown

    // ── DQ bus ───────────────────────────────
    assign dq  = dq_oe ? dq_drive : {DQ_WIDTH{1'bz}};
    assign dqs = rd_burst_active ? clk : 1'bz;  // DQS = clock during read
    assign dqs_n = rd_burst_active ? clk_n : 1'bz;

    // ── Mode register storage ─────────────────
    reg [13:0] mr [0:3]; // full 14-bit MR values

    // ── Initialization tracking ───────────────
    reg zq_done;
    reg [3:0] mrs_count; // count MRS commands issued

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dq_reg          <= 0;
            dq_drive        <= 0;
            dq_oe           <= 0;
            rd_burst_active <= 0;
            rd_burst_cnt    <= 0;
            rd_latency_cnt  <= 0;
            wr_burst_active <= 0;
            wr_burst_cnt    <= 0;
            wr_latency_cnt  <= 0;
            rd_data_latched <= 0;
            init_complete   <= 0;
            zq_done         <= 0;
            mrs_count       <= 0;
            for (i = 0; i < 4; i = i + 1)
                mr[i] <= 14'h0;
            for (i = 0; i < 4; i = i + 1)
                mode_reg[i] <= 4'h0;

        end else begin
            dq_oe <= 0;

            // ── Command processing ─────────────
            case (cmd)

                CMD_MRS: begin
                    // Load mode register
                    // BA selects which MR (0-3)
                    if (ba <= 3) begin
                        mr[ba]       <= addr[13:0];
                        mode_reg[ba] <= addr[3:0]; // debug output
                        mrs_count    <= mrs_count + 1;
                    end
                end

                CMD_ZQCAL: begin
                    zq_done <= 1;
                    // After ZQ + all 4 MRS: init complete
                    if (mrs_count >= 4)
                        init_complete <= 1;
                end

                CMD_ACTIVATE: begin
                    // Bank array handles storage
                    // Just track for timing
                end

                CMD_READ: begin
                    // Start CL countdown
                    rd_latency_cnt  <= CL - 1;
                    rd_data_latched <= bank_rd_data[ba];
                end

                CMD_WRITE: begin
                    // Capture DQ (presented CWL cycles later)
                    wr_latency_cnt  <= CWL - 1;
                    wr_burst_cnt    <= BL - 1;
                    wr_burst_active <= 1;
                end

                CMD_PRECHARGE: begin
                    // Bank array handles close
                end

                CMD_REFRESH: begin
                    // Internal refresh — no visible action in model
                end

                default: ; // NOP, DESELECT
            endcase

            // ── Read burst pipeline ────────────
            // CL cycle countdown after READ command
            if (rd_latency_cnt > 0) begin
                rd_latency_cnt <= rd_latency_cnt - 1;
            end else if (cmd == CMD_READ) begin
                // Just issued: will count from CL-1
            end else if (rd_latency_cnt == 0 && !rd_burst_active
                         && cmd != CMD_READ) begin
                // Idle
            end

            // Start read burst when CL expires
            // (simplified: drive constant data for BL cycles)
            if (rd_burst_active) begin
                dq_oe    <= 1;
                dq_drive <= rd_data_latched;

                if (rd_burst_cnt > 0)
                    rd_burst_cnt <= rd_burst_cnt - 1;
                else begin
                    rd_burst_active <= 0;
                    dq_oe           <= 0;
                end
            end

            // ── Write burst capture ────────────
            if (wr_burst_active) begin
                dq_reg <= dq;  // capture from bus

                if (wr_burst_cnt > 0)
                    wr_burst_cnt <= wr_burst_cnt - 1;
                else
                    wr_burst_active <= 0;
            end
        end
    end

    // ── CL pipeline for read burst trigger ───
    // Shift register: 1 loaded on READ, fires after CL cycles
    reg [CL-1:0] cl_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cl_pipe         <= 0;
            rd_burst_active <= 0;
            rd_burst_cnt    <= 0;
        end else begin
            cl_pipe <= {cl_pipe[CL-2:0], (cmd == CMD_READ)};

            if (cl_pipe[CL-1]) begin
                rd_burst_active <= 1;
                rd_burst_cnt    <= BL - 1;
                rd_data_latched <= bank_rd_data[ba];
            end
        end
    end

endmodule
