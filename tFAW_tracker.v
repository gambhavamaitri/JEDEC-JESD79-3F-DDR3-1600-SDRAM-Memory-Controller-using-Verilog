`include "ddr3_params.v"

module tfaw_ctrl #(
    parameter integer TFAW = 24
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        act_req,      // activate_issued pulse
    output wire        tfaw_ok       // 1 = 5th ACT allowed
);

    // 4 slots: countdown timers for the rolling window
    reg [$clog2(TFAW+1)-1:0] tfaw_slot [0:3];
    reg [1:0] tfaw_wptr;
    integer i;

    // Count non-zero slots (active window entries)
    reg [2:0] active_cnt;

    always @(*) begin
        active_cnt = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (tfaw_slot[i] != 0)
                active_cnt = active_cnt + 1;
        end
    end

    // All 4 slots must be 0 to allow another activate
    assign tfaw_ok = (active_cnt < 4);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tfaw_slot[0] <= 0;
            tfaw_slot[1] <= 0;
            tfaw_slot[2] <= 0;
            tfaw_slot[3] <= 0;
            tfaw_wptr    <= 0;
        end else begin
            // Decrement all slots every cycle
            if (tfaw_slot[0] > 0) tfaw_slot[0] <= tfaw_slot[0] - 1;
            if (tfaw_slot[1] > 0) tfaw_slot[1] <= tfaw_slot[1] - 1;
            if (tfaw_slot[2] > 0) tfaw_slot[2] <= tfaw_slot[2] - 1;
            if (tfaw_slot[3] > 0) tfaw_slot[3] <= tfaw_slot[3] - 1;

            // Load new slot on activate request
            if (act_req) begin
                tfaw_slot[tfaw_wptr] <= TFAW;
                tfaw_wptr            <= tfaw_wptr + 1;
            end
        end
    end

endmodule
