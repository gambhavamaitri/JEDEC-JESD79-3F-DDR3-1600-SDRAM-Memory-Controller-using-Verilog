module dqs_phase_shifter #(
    parameter NUM_TAPS = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire dqs_in,
    input  wire [$clog2(NUM_TAPS)-1:0] delay_sel,
    output wire dqs_delayed,
    input  wire dqs_n_in,
    output wire dqs_n_delayed,
    output reg  locked
);

    reg [NUM_TAPS-1:0] tap_chain;
    reg [NUM_TAPS-1:0] tap_n_chain;
    reg [$clog2(NUM_TAPS)-1:0] delay_sel_reg;  // << REGISTERED

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_chain      <= 0;
            tap_n_chain    <= 0;
            locked         <= 0;
            delay_sel_reg  <= 0;  // << reset
        end else begin
            tap_chain     <= {tap_chain[NUM_TAPS-2:0], dqs_in};
            tap_n_chain   <= {tap_n_chain[NUM_TAPS-2:0], dqs_n_in};
            delay_sel_reg <= delay_sel;  // << register it
            locked        <= 1;
        end
    end

    assign dqs_delayed   = tap_chain[delay_sel_reg];
    assign dqs_n_delayed = tap_n_chain[delay_sel_reg];

endmodule
