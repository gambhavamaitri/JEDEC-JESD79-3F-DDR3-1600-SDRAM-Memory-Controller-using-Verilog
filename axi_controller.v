/*
 * module: axi_controller
 * -----------------

module axi_controller (
    // =========================================================================
    // Clocks e Resets | Clocks and Resets
    // =========================================================================
    input  wire        ACLK,      
                                   // AXI Clock (Synchronous with Master).
    input  wire        ARESETn,    //AXI Reset (Active Low).

    // =========================================================================
    // Interface AXI-Lite (Lado Mestre/CPU) | AXI-Lite Interface (Master/CPU Side)
    // =========================================================================
    
    // Canal de Endereço de Escrita (AW) | Write Address Channel (AW)
  input  wire [26:0] AWADDR,   // Write address captured during handshake.
    input  wire        AWVALID,    // Address validity signal from Master.
    output wire        AWREADY,    // Indicates the controller accepted the address.

    // Canal de Dados de Escrita (W) | Write Data Channel (W)
    input  wire [15:0] WDATA,      
                                   // Write data (16 bits).
    input  wire        WVALID,     
                                   //  Data validity signal from Master.
	 input wire  [1:0]  WSTRB,     
	                               // Byte mask (Strobes).
    output wire        WREADY,     //indicates the controller accepted the data.

    // Canal de Resposta de Escrita (B) | Write Response Channel (B)
  output wire [1:0]  BRESP,      // Write response (Always OKAY in this implementation).
    output wire        BVALID,     
                                   //  Write response valid.
    input  wire        BREADY,     
                                   //  Master ready to receive response.

    // Canal de Endereço de Leitura (AR) | Read Address Channel (AR)
  input  wire [26:0] ARADDR,     // Read address.
    input  wire        ARVALID,    //Read address validity signal.
    output wire        ARREADY,    // Indicates readiness to receive read address.

    // Canal de Dados de Leitura (R) | Read Data Channel (R)
    output wire [15:0] RDATA,   
                                   //  Read data returned to master.
    output wire [1:0]  RRESP,    
                                   //  Read response (Always OKAY).
    output wire        RVALID,     
                                   // Read data available and valid.
    input  wire        RREADY,  
                                   // Master ready to read the data.

    // =========================================================================
// Custom Interface (Slave/DDR3 Side)
    // =========================================================================
    output wire        cpu_req,   
                                   // Command request to memory.
    output wire        cpu_rnw,    
                                   //Read/Write_n selector.
    output wire [26:0] cpu_addr,  
                                   // Translated address for memory.
    output wire        cpu_wr_en,  
                                   //Enables write to memory internal buffers.
    output wire [15:0] cpu_wdata,  
                                   //  Data to be written.
  output wire [1:0]  cpu_wstrb,  //Byte strobe mask.
  input  wire [15:0] cpu_rdata,  // Read data coming from memory.
    input  wire        cpu_ready,  
                                   //Indicates memory is ready to accept requests.
    input  wire        rx_valid,   
                                   // Indicates read data from memory is valid.
    input  wire        tx_full,    
                                   //  Indicates write queue is full.
	 input  wire        init_done   
	                               // Signals JEDEC calibration sequence has finished.
);


    // =========================================================================
    //  (FSM <-> Datapath)
    // =========================================================================
    wire latch_aw;
    wire latch_w;
    wire latch_ar;
	 wire latch_r;
    wire sel_read;

    // =========================================================================
    // (Controle)
    // =========================================================================
    axi_lite_fsm u_fsm (
        .ACLK       (ACLK),
        .ARESETn    (ARESETn),
        
        // Entradas/Saídas AXI
        .AWVALID    (AWVALID),
        .WVALID     (WVALID),
        .BREADY     (BREADY),
        .ARVALID    (ARVALID),
        .RREADY     (RREADY),
        .AWREADY    (AWREADY),
        .WREADY     (WREADY),
        .BVALID     (BVALID),
        .ARREADY    (ARREADY),
        .RVALID     (RVALID),
        
        // Entradas/Saídas da Memória
        .cpu_ready  (cpu_ready),
        .rx_valid   (rx_valid),
        .tx_full    (tx_full),
        .cpu_req    (cpu_req),
        .cpu_rnw    (cpu_rnw),
        .cpu_wr_en  (cpu_wr_en),
        .init_done  (init_done),
        // Sinais de controle para o Datapath
        .latch_aw   (latch_aw),
        .latch_w    (latch_w),
        .latch_ar   (latch_ar),
        .sel_read   (sel_read),
		  .latch_r	  (latch_r)
    );

    // =========================================================================
    //(Datapath)
    // =========================================================================
    axi_lite_datapath u_datapath (
        .ACLK       (ACLK),
        .ARESETn    (ARESETn),
        
        // Sinais de controle vindos da FSM
        .latch_aw   (latch_aw),
        .latch_w    (latch_w),
        .latch_ar   (latch_ar),
        .sel_read   (sel_read),
		  .latch_r    (latch_r),
      
        .AWADDR     (AWADDR),
        .WDATA      (WDATA),
        .ARADDR     (ARADDR),
        .RDATA      (RDATA),
        .BRESP      (BRESP),
        .RRESP      (RRESP),
        .WSTRB      (WSTRB),
		
        .cpu_rdata  (cpu_rdata),
        .cpu_addr   (cpu_addr),
        .cpu_wdata  (cpu_wdata),
		  .cpu_wstrb  (cpu_wstrb)
    );

endmodule
