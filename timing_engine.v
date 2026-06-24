`include "ddr3_params.v"

module timing_engine (
    input  wire clk,
    input  wire rst_n,
    input  wire [BANK_BITS-1:0] bank_sel,

    input  wire activate_issued,
    input  wire read_issued,
    input  wire write_issued,
    input  wire precharge_issued,
    input  wire refresh_issued,

    input  wire [NUM_BANKS-1:0] bank_idle,

    output wire can_activate,
    output wire can_read,
    output wire can_write,
    output wire can_precharge,
    output wire can_refresh,

    output wire trcd_done,
    output wire tras_done,
    output wire trp_done
);

    reg [$clog2(tRCD+1)-1:0] trcd_cnt [0:NUM_BANKS-1];
    reg [$clog2(tRAS+1)-1:0] tras_cnt [0:NUM_BANKS-1];
    reg [$clog2(tRP+1)-1:0]  trp_cnt  [0:NUM_BANKS-1];
    reg [$clog2(tWR+1)-1:0]  twr_cnt  [0:NUM_BANKS-1];
    reg [$clog2(tRC+1)-1:0]  trc_cnt  [0:NUM_BANKS-1];
    reg [$clog2(tRTP+1)-1:0] trtp_cnt [0:NUM_BANKS-1];

    reg [$clog2(tCCD+1)-1:0] tccd_cnt;
    reg [$clog2(tWTR+1)-1:0] twtr_cnt;
    reg [$clog2(tRRD+1)-1:0] trrd_cnt;
    reg [$clog2(tRFC+1)-1:0] trfc_cnt;
    reg [$clog2(tRTW+1)-1:0] trtw_cnt;

    reg [NUM_BANKS-1:0] bank_was_activated;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_BANKS; i = i + 1) begin
                trcd_cnt[i] <= 0;
                tras_cnt[i] <= 0;
                trp_cnt[i]  <= 0;
                twr_cnt[i]  <= 0;
                trc_cnt[i]  <= 0;
                trtp_cnt[i] <= 0;
            end
            tccd_cnt <= 0;
            twtr_cnt <= 0;
            trrd_cnt <= 0;
            trfc_cnt <= 0;
            trtw_cnt <= 0;
            bank_was_activated <= {NUM_BANKS{1'b0}};
        end else begin
            for (i = 0; i < NUM_BANKS; i = i + 1) begin
                if (trcd_cnt[i] > 0) trcd_cnt[i] <= trcd_cnt[i] - 1;
                if (tras_cnt[i] > 0) tras_cnt[i] <= tras_cnt[i] - 1;
                if (trp_cnt[i]  > 0) trp_cnt[i]  <= trp_cnt[i]  - 1;
                if (twr_cnt[i]  > 0) twr_cnt[i]  <= twr_cnt[i]  - 1;
                if (trc_cnt[i]  > 0) trc_cnt[i]  <= trc_cnt[i]  - 1;
                if (trtp_cnt[i] > 0) trtp_cnt[i] <= trtp_cnt[i] - 1;
            end
            if (tccd_cnt > 0) tccd_cnt <= tccd_cnt - 1;
            if (twtr_cnt > 0) twtr_cnt <= twtr_cnt - 1;
            if (trrd_cnt > 0) trrd_cnt <= trrd_cnt - 1;
            if (trfc_cnt > 0) trfc_cnt <= trfc_cnt - 1;
            if (trtw_cnt > 0) trtw_cnt <= trtw_cnt - 1;

            if (activate_issued) begin
                trcd_cnt[bank_sel] <= tRCD;
                tras_cnt[bank_sel] <= tRAS;
                trrd_cnt           <= tRRD;
                trc_cnt[bank_sel]  <= tRC;
                bank_was_activated[bank_sel] <= 1'b1;
            end
            if (read_issued) begin
                tccd_cnt <= tCCD;
                trtw_cnt <= tRTW;
                trtp_cnt[bank_sel] <= tRTP;
            end
            if (write_issued) begin
                tccd_cnt          <= tCCD;
                twr_cnt[bank_sel] <= tWR;
                twtr_cnt          <= tWTR;
            end
            if (precharge_issued) begin
                trp_cnt[bank_sel] <= tRP;
            end
            if (refresh_issued) begin
                trfc_cnt <= tRFC;
            end
        end
    end

    // tFAW removed — tied to 1'b1 (always allow)
    assign can_activate  = (trrd_cnt == 0) &&
                           (trp_cnt[bank_sel] == 0) &&
                           ((trc_cnt[bank_sel] == 0) || !bank_was_activated[bank_sel]) &&
                           (trfc_cnt == 0) &&
                           (bank_idle[bank_sel]) &&
                           1'b1;

    assign can_read      = !bank_idle[bank_sel] &&
                           (trcd_cnt[bank_sel] == 0) &&
                           (tccd_cnt == 0) &&
                           (twtr_cnt == 0) &&
                           (trfc_cnt == 0);

    assign can_write     = !bank_idle[bank_sel] &&
                           (trcd_cnt[bank_sel] == 0) &&
                           (tccd_cnt == 0) &&
                           (trtw_cnt == 0) &&
                           (trfc_cnt == 0);

    assign can_precharge = (tras_cnt[bank_sel] == 0) &&
                           (twr_cnt[bank_sel]  == 0) &&
                           (trtp_cnt[bank_sel] == 0);

    assign can_refresh   = (trfc_cnt == 0) && (&bank_idle);

    assign trcd_done = (trcd_cnt[bank_sel] == 0);
    assign tras_done = (tras_cnt[bank_sel] == 0);
    assign trp_done  = (trp_cnt[bank_sel]  == 0);

endmodule
