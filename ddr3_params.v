// ddr3_params.v
// JEDEC JESD79-3F | DDR3-1600K (CL11-11-11)
// tCK = 1.25ns | 800MHz | 2Gb x8 device

`ifndef DDR3_PARAMS_V
`define DDR3_PARAMS_V

// ── DEVICE CONFIG ────────────────────────────────────
localparam ROW_BITS  = 15;   // 2^15 = 32,768 rows
localparam COL_BITS  = 10;   // 2^10 = 1,024 cols
localparam BANK_BITS = 3;    // 2^3  = 8 banks
localparam NUM_BANKS = 8;
localparam DQ_WIDTH  = 8;    // x8 device
localparam BL        = 8;    // Burst Length 8 

// ── READ/WRITE LATENCY ───────────────────────────────
localparam CL  = 11;  // CAS Latency         
localparam CWL = 8;   // CAS Write Latency   

// ── CORE TIMING (nCK = ceil(ns ÷ 1.25)) ─────────────
localparam tRCD  = 11;   // 13.75ns ÷ 1.25  
localparam tRP   = 11;   // 13.75ns ÷ 1.25  
localparam tRAS  = 28;   // 35ns    ÷ 1.25  
localparam tRC   = 39;   // tRAS+tRP=39      
localparam tWR   = 12;   // 15ns    ÷ 1.25  
localparam tCCD  = 4;    // 4 nCK min        
localparam tRRD  = 6;    // 7.5ns   ÷ 1.25  
localparam tWTR  = 6;    // 7.5ns   ÷ 1.25  
localparam tRTP  = 6;    // 7.5ns   ÷ 1.25  
localparam tFAW  = 24;   // 30ns    ÷ 1.25  
localparam tRTW  =  4;     // READ-to-WRITE turnaround

// ── REFRESH ──────────────────────────────────────────
localparam tRFC      = 128;  
localparam tREFI     = 6240; // 7800ns ÷ 1.25 
localparam tREFI_HOT = 3120; // 3900ns ÷ 1.25  

// ── INITIALIZATION ───────────────────────────────────
localparam tMRD    = 4;       // 4 nCK           
localparam tMOD    = 12;      // 15ns   ÷ 1.25  
localparam tZQINIT = 512;     // 640ns  ÷ 1.25  
localparam tZQCS   = 64;      // 80ns   ÷ 1.25  
localparam tDLLK   = 512;     // 512 nCK         
localparam tXPR    = 136;     // tRFC+10÷1.25   
localparam TINIT_RST = 40_000;  // 200µs ÷ 5ns 
localparam TINIT_CKE = 100_000; // 500µs ÷ 5ns  

// ── COMMAND ENCODING {RAS#,CAS#,WE#} ─────────────────
localparam CMD_MRS       = 3'b000;
localparam CMD_REFRESH   = 3'b001;
localparam CMD_PRECHARGE = 3'b010;
localparam CMD_ACTIVATE  = 3'b011;
localparam CMD_WRITE     = 3'b100;
localparam CMD_READ      = 3'b101;
localparam CMD_ZQCAL     = 3'b110;
localparam CMD_NOP       = 3'b111;

// ── MRS VALUES ───────────────────────────────────────
localparam MR0_VAL = 14'h1D70; // CL=11,BL8,tWR=12
localparam MR1_VAL = 14'h0046; // RTT=RZQ/6,ODS=RZQ/7
localparam MR2_VAL = 14'h0018; // CWL=8
localparam MR3_VAL = 14'h0000; // MPR off

`endif
