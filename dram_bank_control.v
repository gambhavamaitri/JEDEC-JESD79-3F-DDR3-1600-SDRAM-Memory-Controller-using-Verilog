/*
 * module: dram_bank_control
 * -----------------
 * PT: Controlador de Estado dos Bancos DRAM.
 *     8 máquinas de estado independentes (uma por banco).
 *     Timers inline (sem temp_param) para zero latência de carga.
 *     Usa parâmetros de ddr3_params.v.
 *
 * EN: DRAM Bank State Controller.
 *     8 independent state machines (one per bank).
 *     Inline counters (no temp_param) for zero load latency.
 *     Uses parameters from ddr3_params.v.
 */
`include "ddr3_params.v"

module dram_bank_control (
    
    input  wire        clk,
    input  wire        rst_n,

   
    input  wire        act_cmd,       // ACTIVATE issued
    input  wire        pre_cmd,       // PRECHARGE issued
    input  wire        rd_cmd,        // READ issued
    input  wire        wr_cmd,        // WRITE issued
    input  wire [BANK_BITS-1:0] bank_addr,     // Target bank
    input  wire [ROW_BITS-1:0]  row_addr,      // Row address for ACTIVATE

    
    // Bank Status Outputs (combinational)
    
    output reg  [NUM_BANKS-1:0] bank_idle,      // 1 = bank in IDLE (ready for ACT)
    output reg  [NUM_BANKS-1:0] bank_active,    // 1 = bank in ACTIVE (row open)
    output reg  [NUM_BANKS-1:0] bank_busy,      // 1 = bank in ACTIVATING or PRECHARGING

    
    // Row Tracking Outputs (for row_tracker / scheduler)
    
    output reg  [NUM_BANKS-1:0]        row_valid,   // 1 = open_row[bank] is valid
    output reg  [ROW_BITS-1:0] open_row [0:NUM_BANKS-1]  // Which row is open per bank
);

 
    // State definitions
 
    reg [1:0] state [0:NUM_BANKS-1];
    localparam IDLE        = 2'd0;
    localparam ACTIVATING  = 2'd1;
    localparam ACTIVE      = 2'd2;
    localparam PRECHARGING = 2'd3;

    integer i;

  
    // Inline counters 
  
    reg [$clog2(tRCD+1)-1:0] trcd_cnt [0:NUM_BANKS-1];
    reg [$clog2(tRP +1)-1:0] trp_cnt  [0:NUM_BANKS-1];
    reg [$clog2(tRAS+1)-1:0] tras_cnt [0:NUM_BANKS-1];
    reg [$clog2(tWR +1)-1:0] twr_cnt  [0:NUM_BANKS-1];

  
    // Sequential: State + Counters + Row Storage
  
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_BANKS; i = i + 1) begin
                state[i]     <= IDLE;
                trcd_cnt[i]  <= 0;
                trp_cnt[i]   <= 0;
                tras_cnt[i]  <= 0;
                twr_cnt[i]   <= 0;
                row_valid[i] <= 1'b0;
                open_row[i]  <= {ROW_BITS{1'b0}};
            end
        end else begin
            for (i = 0; i < NUM_BANKS; i = i + 1) begin

                // Decrement all counters every cycle
                if (trcd_cnt[i] > 0) trcd_cnt[i] <= trcd_cnt[i] - 1'b1;
                if (trp_cnt[i]  > 0) trp_cnt[i]  <= trp_cnt[i]  - 1'b1;
                if (tras_cnt[i] > 0) tras_cnt[i] <= tras_cnt[i] - 1'b1;
                if (twr_cnt[i]  > 0) twr_cnt[i]  <= twr_cnt[i]  - 1'b1;

                case (state[i])

     
                    // IDLE: Bank closed, ready for ACTIVATE
                    IDLE: begin
                        if (act_cmd && (bank_addr == i)) begin
                            state[i]     <= ACTIVATING;
                            trcd_cnt[i]  <= tRCD;        // From ddr3_params.v
                            tras_cnt[i]  <= tRAS;        // From ddr3_params.v
                            open_row[i]  <= row_addr;    // CAPTURE row address
                            row_valid[i] <= 1'b1;
                        end
                    end

    
                    // ACTIVATING: Waiting for tRCD to elapse
                    ACTIVATING: begin
                        if (trcd_cnt[i] == 0)
                            state[i] <= ACTIVE;
                    end

                    // ACTIVE: Row open, ready for READ/WRITE
                    ACTIVE: begin  // <-- FIXED: added missing label + begin
                        // Reload tWR on every WRITE to this bank
                        if (wr_cmd && (bank_addr == i))
                            twr_cnt[i] <= tWR;           // From ddr3_params.v

                        // PRECHARGE: only if tRAS and tWR both elapsed
                        if (pre_cmd && (bank_addr == i)) begin
                            if ((tras_cnt[i] == 0) && (twr_cnt[i] == 0)) begin
                                state[i]     <= PRECHARGING;
                                trp_cnt[i]   <= tRP;     // From ddr3_params.v
                                row_valid[i] <= 1'b0;    // Invalidate row
                            end
                        end
                    end 

                    // PRECHARGING: Waiting for tRP to elapse
                    PRECHARGING: begin
                        if (trp_cnt[i] == 0)
                            state[i] <= IDLE;
                    end

                endcase
            end
        end
    end

  
    // Combinational: Status Outputs
   
    always @(*) begin
        for (i = 0; i < NUM_BANKS; i = i + 1) begin
            bank_idle[i]   = (state[i] == IDLE);
            bank_active[i] = (state[i] == ACTIVE);
            bank_busy[i]   = (state[i] == ACTIVATING) || (state[i] == PRECHARGING);
        end
    end

endmodule
