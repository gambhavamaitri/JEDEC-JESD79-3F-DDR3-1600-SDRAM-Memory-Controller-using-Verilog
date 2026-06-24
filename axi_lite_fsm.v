module axi_lite_fsm(
    Low).
// =========================================================================
    //  AXI-Lite Interface (Control)
    // =========================================================================
    input      AWVALID,    // Write address valid.
    input      WVALID,     // Write data valid.
    input      BREADY,     // Master ready for write resp.
    input      ARVALID,    //  Read address valid.
    input      RREADY,     //  Master ready for read resp.

    output reg AWREADY,    //  Ready for write address.
    output reg WREADY,     //  Ready for write data.
    output reg BVALID,     //  Write response valid.
    output reg ARREADY,    // PT: Pronto p/ receber end. leitura. | EN: Ready for read address.
    output reg RVALID,     //  Read response valid.

    // =========================================================================
    // Memory Interface (ddr_mem)
    // =========================================================================
    input      cpu_ready,  //  Memory ready for new cmds.
    input      rx_valid,   // Data from memory is valid.
    input      tx_full,    //  TX buffer is full.
    input      init_done,  // JEDEC calibration done.

    output reg cpu_req,    //  Request command to memory.
    output reg cpu_rnw,    // 1=Read, 0=Write.
    output reg cpu_wr_en,  //  Enable write to buffer.

    // =========================================================================
    // Signals for Internal Datapath
    // =========================================================================
    output reg latch_aw,   //  Latch AXI write address.
    output reg latch_w,    //  Latch AXI write data.
    output reg latch_ar,   // Latch AXI read address.
    output reg latch_r,    //  Latch data read from memory.
    output reg sel_read    // Select AR (1) or AW (0) addr.
);


    reg [3:0] state;

    
    parameter IDLE = 4'd0, 
              READ_MEM = 4'd1, 
              READ_R1 = 4'd2, 
              READ_R2 = 4'd3, 
              READ_R3 = 4'd4, 
              READ_R4 = 4'd5, 
              READ_RESP = 4'd6, 
              WRITE_W1 = 4'd7, 
              WRITE_W2 = 4'd8, 
              WRITE_W3 = 4'd9, 
              WRITE_W4 = 4'd10, 
              WRITE_RESP = 4'd11;

    always @ (*) begin
        AWREADY   = 1'b0;
        WREADY    = 1'b0;
        BVALID    = 1'b0;
        ARREADY   = 1'b0;
        RVALID    = 1'b0;
        cpu_req   = 1'b0;
        cpu_rnw   = 1'b0;
        cpu_wr_en = 1'b0;
        latch_ar  = 1'b0;
        latch_aw  = 1'b0;
        latch_w   = 1'b0;
        latch_r   = 1'b0;
        sel_read  = 1'b0;

        case (state)
            IDLE: begin
                if(init_done) begin
                    if(ARVALID)
                        latch_ar = 1'b1;
                    else if(!ARVALID && AWVALID && WVALID) begin
                        latch_aw = 1'b1;
                        latch_w  = 1'b1;
                    end
                end
            end
            
READ_MEM: begin
                sel_read = 1'b1;
                if(cpu_ready) begin
                    cpu_req  = 1'b1;
                    cpu_rnw  = 1'b1;
                    ARREADY  = 1'b1;
                end
            end
            READ_R1: begin 
                if (rx_valid) latch_r = 1'b1;t
            end
            READ_R2: begin end             
            READ_R3: begin end               
            READ_R4: begin end                
            READ_RESP: RVALID = 1'b1;         
            
            WRITE_W1: if (!tx_full) cpu_wr_en = 1'b1; 
            WRITE_W2: if (!tx_full) cpu_wr_en = 1'b1; 
            WRITE_W3: if (!tx_full) cpu_wr_en = 1'b1; 
            WRITE_W4: begin
                if(cpu_ready && !tx_full) begin 
                    cpu_req   = 1'b1;               
                    cpu_wr_en = 1'b1;               
                    AWREADY   = 1'b1;
                    WREADY    = 1'b1;
                end
            end 
            WRITE_RESP: BVALID = 1'b1;
            default: begin end
        endcase
    end

    always @ (posedge ACLK or negedge ARESETn) begin
        if (!ARESETn)
            state <= IDLE;
        else
            case (state)
                IDLE: begin
                    if(init_done) begin
                        if(ARVALID) state <= READ_MEM;
                        else if(!ARVALID && AWVALID && WVALID) state <= WRITE_W1;
                    end
                end
                
                READ_MEM:  if (cpu_ready) state <= READ_R1;
                READ_R1:   if (rx_valid)  state <= READ_R2;
                READ_R2:   if (rx_valid)  state <= READ_R3;
                READ_R3:   if (rx_valid)  state <= READ_R4;
                READ_R4:   if (rx_valid)  state <= READ_RESP;
                READ_RESP: if (RREADY)    state <= IDLE;
                
                WRITE_W1:   if (!tx_full) state <= WRITE_W2;
                WRITE_W2:   if (!tx_full) state <= WRITE_W3;
                WRITE_W3:   if (!tx_full) state <= WRITE_W4;
                WRITE_W4:   if (cpu_ready && !tx_full) state <= WRITE_RESP;
                WRITE_RESP: if (BREADY)   state <= IDLE;
                
                default: state <= IDLE;
            endcase
    end
endmodule
