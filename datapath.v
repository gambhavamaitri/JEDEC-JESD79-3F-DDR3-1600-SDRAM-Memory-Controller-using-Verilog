// ============================================
// datapath.v — CORRECTED
// Single read FIFO (not two)
// ============================================
`include "ddr3_params.v"

module datapath (
    input  wire clk_ddr,
    input  wire clk_cpu,
    input  wire rst_n,

    // ── CPU write interface (clk_cpu domain) ──
    input  wire [63:0] cpu_wdata,
    input  wire [7:0]  cpu_wbe,
    input  wire        cpu_wr_push,
    output wire        cpu_wr_full,

    // ── CPU read interface (clk_cpu domain) ───
    output wire [63:0] cpu_rdata,
    output wire        cpu_rd_valid,
    input  wire        cpu_rd_pop,

    // ── DDR3 DQ interface ─────────────────────
    output reg  [DQ_WIDTH-1:0]   dq_out,
    input  wire [DQ_WIDTH-1:0]   dq_in,
    output reg                   dq_oe,
    output reg  [DQ_WIDTH/8-1:0] dm_out,

    // ── Control from datapath_control_logic ───
    input  wire serializer_en,
    input  wire deserializer_en,
    input  wire wr_fifo_rd_en,
    input  wire rd_fifo_wr_en,

    // ── Status ───────────────────────────────
    output wire wr_fifo_empty,
    output wire rd_fifo_full
);

    // ══════════════════════════════════════════
    // WRITE PATH: CPU → write FIFO → serializer → DQ
    // ══════════════════════════════════════════

    // ── Write FIFO ────────────────────────────
    wire [71:0] wr_fifo_rdata;
    wire        wr_fifo_rempty_ddr;

    fifo_async_burst #(
        .DEPTH (8),
        .WIDTH (72)
    ) u_wr_fifo (
        // Write port: CPU clock domain
        .wclk   (clk_cpu),
        .wrst_n (rst_n),
        .wr_en  (cpu_wr_push && !cpu_wr_full),
        .wdata  ({cpu_wbe, cpu_wdata}),
        .wfull  (cpu_wr_full),

        // Read port: DDR clock domain
        .rclk   (clk_ddr),
        .rrst_n (rst_n),
        .rd_en  (wr_fifo_rd_en && !wr_fifo_rempty_ddr),
        .rdata  (wr_fifo_rdata),
        .rempty (wr_fifo_rempty_ddr)
    );

    assign wr_fifo_empty = wr_fifo_rempty_ddr;

    wire [63:0] wr_data = wr_fifo_rdata[63:0];
    wire [7:0]  wr_be   = wr_fifo_rdata[71:64];

    // ── Write Serializer ─────────────────────
    // 64-bit parallel → 8 × 8-bit serial beats
    // DM = inverse of byte enable
    //   cpu_wbe[i]=1 → write byte i → DM[i]=0
    //   cpu_wbe[i]=0 → mask byte i  → DM[i]=1
    reg [63:0] wr_shift_reg;
    reg [7:0]  dm_shift_reg;
    reg [2:0]  wr_beat_cnt;
    reg        wr_active;
    reg        wr_load;        // NEW: one-cycle load pulse

    always @(posedge clk_ddr or negedge rst_n) begin
        if (!rst_n) begin
            wr_shift_reg <= 0;
            dm_shift_reg <= 8'hFF;
            wr_beat_cnt  <= 0;
            wr_active    <= 0;
            wr_load      <= 0;
            dq_out       <= 0;
            dm_out       <= 1;
            dq_oe        <= 0;

        end else begin
            wr_load <= 0;

            // Load shift register when serializer is enabled
            // and we have data available
            if (serializer_en && !wr_active && !wr_load
                && !wr_fifo_rempty_ddr) begin
                wr_shift_reg <= wr_data;
                dm_shift_reg <= ~wr_be;  // invert: BE→DM
                wr_beat_cnt  <= 0;
                wr_load      <= 1;       // signal load this cycle
            end

            if (wr_load) begin
                // First beat: drive immediately from loaded reg
                dq_out       <= wr_shift_reg[7:0];
                dm_out       <= dm_shift_reg[0];
                dq_oe        <= 1;
                wr_active    <= 1;
                // Do NOT shift yet — first beat uses [7:0]
            end else if (wr_active) begin
                // Subsequent beats: shift then drive
                wr_shift_reg <= {8'b0, wr_shift_reg[63:8]};
                dm_shift_reg <= {1'b1, dm_shift_reg[7:1]};
                dq_out       <= wr_shift_reg[15:8];  // byte 1,2,3...
                dm_out       <= dm_shift_reg[1];
                dq_oe        <= 1;
                wr_beat_cnt  <= wr_beat_cnt + 1;

                if (wr_beat_cnt == (BL-2)) begin  // BL-2 because beat 0 was wr_load
                    wr_active <= 0;
                    dq_oe     <= 0;
                    dm_out    <= 1;
                end
            end else begin
                // Idle
                dq_oe  <= 0;
                dm_out <= 1;
            end
        end
    end

    // ══════════════════════════════════════════
    // READ PATH: DQ → deserializer → read FIFO → CPU
    // ══════════════════════════════════════════

    // ── Read Deserializer ─────────────────────
    // 8 × 8-bit serial → 64-bit parallel
    reg [63:0] rd_shift_reg;
    reg [2:0]  rd_beat_cnt;
    reg        rd_active;
    reg [63:0] rd_word;
    reg        rd_word_valid;
    reg        rd_load;        // NEW: one-cycle start pulse

    always @(posedge clk_ddr or negedge rst_n) begin
        if (!rst_n) begin
            rd_shift_reg  <= 0;
            rd_beat_cnt   <= 0;
            rd_active     <= 0;
            rd_load       <= 0;
            rd_word       <= 0;
            rd_word_valid <= 0;

        end else begin
            rd_word_valid <= 0;
            rd_load       <= 0;

            if (deserializer_en && !rd_active && !rd_load) begin
                rd_load <= 1;
            end

            if (rd_load) begin
                rd_active   <= 1;
                rd_beat_cnt <= 0;
                // First byte captured on next cycle
            end else if (rd_active) begin
                // Accumulate bytes into shift register
                // Byte 0 arrives first, goes to [7:0]
                // Byte 7 arrives last, goes to [63:56]
                rd_shift_reg <= {dq_in,
                                  rd_shift_reg[63:DQ_WIDTH]};
                rd_beat_cnt  <= rd_beat_cnt + 1;

                if (rd_beat_cnt == (BL-1)) begin
                    rd_active     <= 0;
                    rd_word       <= {dq_in,
                                       rd_shift_reg[63:DQ_WIDTH]};
                    rd_word_valid <= 1;
                end
            end
        end
    end

    // ── Read FIFO write arming ────────────────
    // rd_fifo_wr_en is a one-cycle pulse from control logic at burst start.
    // We arm the FIFO write so that rd_word_valid can actually push.
    reg rd_fifo_wr_armed;

    always @(posedge clk_ddr or negedge rst_n) begin
        if (!rst_n) begin
            rd_fifo_wr_armed <= 0;
        end else begin
            if (rd_fifo_wr_en)
                rd_fifo_wr_armed <= 1;
            if (rd_word_valid)
                rd_fifo_wr_armed <= 0;
        end
    end

    // ── Read FIFO ─────────────────────────────
    // DDR clock → CPU clock domain crossing
    wire rd_fifo_wfull;
    wire rd_fifo_rempty;

    assign rd_fifo_full = rd_fifo_wfull;

    fifo_async_burst #(
        .DEPTH (8),
        .WIDTH (64)
    ) u_rd_fifo (
        // Write port: DDR clock domain
        .wclk   (clk_ddr),
        .wrst_n (rst_n),
        .wr_en  (rd_word_valid && rd_fifo_wr_armed && !rd_fifo_wfull),
        .wdata  (rd_word),
        .wfull  (rd_fifo_wfull),

        // Read port: CPU clock domain
        .rclk   (clk_cpu),
        .rrst_n (rst_n),
        .rd_en  (cpu_rd_pop && !rd_fifo_rempty),
        .rdata  (cpu_rdata),
        .rempty (rd_fifo_rempty)
    );

    // cpu_rd_valid: data waiting in read FIFO
    assign cpu_rd_valid = !rd_fifo_rempty;

endmodule
