module axi_lite_datapath (
    // =========================================================================
    // Clocks and Resets
    // =========================================================================
    input          ACLK,
    input          ARESETn,

    // =========================================================================
    //  Control Signals (from FSM)
    // =========================================================================
    input          latch_aw,   // Enable AWADDR register.
    input          latch_w,    //  Enable WDATA/WSTRB register.
    input          latch_ar,   // Enable ARADDR register.
	 input          latch_r,    // Enable read data capture.
    input          sel_read,   // 1=Use ARADDR, 0=Use AWADDR.
    
    // =========================================================================
    //  AXI-Lite Interface (Data Signals)
    // =========================================================================
    input  [26:0]  AWADDR,
    input  [15:0]  WDATA,
    input  [26:0]  ARADDR,
	 input  [1:0]   WSTRB,
    output [15:0]  RDATA,      // Output to AXI bus.
    output [1:0]   BRESP,      //  Write response.
    output [1:0]   RRESP,      //  Read response.
    
    // =========================================================================
    // Memory Interface (Physical Side)
    // =========================================================================
    input  [15:0]  cpu_rdata,  // Raw data from memory.
    output [26:0]  cpu_addr,   // Address to DDR3 core.
    output [15:0]  cpu_wdata,  // Data to DDR3 core.
	 output [1:0]   cpu_wstrb   //Strobes to DDR3 core.
);


    reg [26:0] reg_awaddr, reg_araddr;
    reg [15:0] reg_wdata;
	 reg [15:0] reg_rdata;
	 reg [1:0]  reg_wstrb;

always@(posedge ACLK or negedge ARESETn) begin
        if(!ARESETn) begin
            reg_araddr <= 0;
            reg_awaddr <= 0;
            reg_wdata  <= 0;
            reg_rdata  <= 16'd0;
				reg_wstrb <= 2'b0;
        end else begin
            if(latch_aw) reg_awaddr <= AWADDR;
            if(latch_ar) reg_araddr <= ARADDR;
            if(latch_w) begin
					reg_wdata  <= WDATA;
					reg_wstrb  <= WSTRB;
				end
            if(latch_r)  reg_rdata  <= cpu_rdata;
        end
    end

    assign BRESP = 2'b00;
    assign RRESP = 2'b00;
    assign RDATA = reg_rdata;
    
    assign cpu_addr  = (sel_read) ? reg_araddr : reg_awaddr;
    assign cpu_wdata = reg_wdata;
	 assign cpu_wstrb = reg_wstrb;
endmodule
