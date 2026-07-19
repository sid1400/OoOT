//convoluted
//`include "abstractions.v"
// why this has tag and write biforcations..... Ik it makes sense, but kills generallity...
/*
module regs #(
    parameter RP   = 2,
    parameter WP   = 2,
    parameter TAGL = 4,
    parameter TP   = 1
) (
    input clk,
    input rst,

    input  [5*RP-1:0]    r_addr,
    output [32*RP-1:0]   r_data,
    output [TAGL*RP-1:0] r_tag,
    output [RP-1:0]      r_valid,

    input [5*WP-1:0]  w_addr,
    input [32*WP-1:0] w_data,
    input [WP-1:0]    w_en,

    input [5*TP-1:0]    t_addr,
    input [TAGL*TP-1:0] t_tag,
    input [TP-1:0]      t_valid,
    input [TP-1:0]      t_en
);

    reg [31:0]     reg_data  [0:31];
    reg [TAGL-1:0] reg_tag   [0:31];
    reg            reg_valid [0:31];

    genvar gi, gj;

    wire [32*WP-1:0] w_onehot;
    wire [32*TP-1:0] t_onehot;
    wire [32*RP-1:0] r_onehot;

    generate
        for (gj = 0; gj < WP; gj = gj + 1) begin : g_wdec
            decoder #(.AW(5), .OW(32)) u_wdec (
                .addr   (w_addr[gj*5 +: 5]),
                .onehot (w_onehot[gj*32 +: 32])
            );
        end
        for (gj = 0; gj < TP; gj = gj + 1) begin : g_tdec
            decoder #(.AW(5), .OW(32)) u_tdec (
                .addr   (t_addr[gj*5 +: 5]),
                .onehot (t_onehot[gj*32 +: 32])
            );
        end
        for (gj = 0; gj < RP; gj = gj + 1) begin : g_rdec
            decoder #(.AW(5), .OW(32)) u_rdec (
                .addr   (r_addr[gj*5 +: 5]),
                .onehot (r_onehot[gj*32 +: 32])
            );
        end
    endgenerate

    generate
        for (gi = 0; gi < 32; gi = gi + 1) begin : g_reg

            // ---------------- Write data port, priority-resolved ----------------
            // w_match[gj]     : raw "port gj addresses register gi and is enabled"
            // w_match_pri[gj] : same, but forced low if any *earlier* port (lower gj)
            //                   also matched this cycle -> lowest port index wins.
            wire [WP-1:0]    w_match;
            wire [WP-1:0]    w_match_pri;
            wire [32*WP-1:0] w_masked;

            for (gj = 0; gj < WP; gj = gj + 1) begin : g_wmatch
                assign w_match[gj] = w_onehot[gj*32 + gi] & w_en[gj];

                if (gj == 0) begin : g_wpri0
                    assign w_match_pri[gj] = w_match[gj];
                end else begin : g_wpriN
                    assign w_match_pri[gj] = w_match[gj] & ~(|w_match[gj-1:0]);
                end

                assign w_masked[gj*32 +: 32] = w_data[gj*32 +: 32] & {32{w_match_pri[gj]}};
            end

            // ---------------- Tag/valid write port, priority-resolved ----------------
            wire [TP-1:0]      t_match;
            wire [TP-1:0]      t_match_pri;
            wire [TAGL*TP-1:0] t_tag_masked;
            wire [TP-1:0]      t_valid_masked;

            for (gj = 0; gj < TP; gj = gj + 1) begin : g_tmatch
                assign t_match[gj] = t_onehot[gj*32 + gi] & t_en[gj];

                if (gj == 0) begin : g_tpri0
                    assign t_match_pri[gj] = t_match[gj];
                end else begin : g_tpriN
                    assign t_match_pri[gj] = t_match[gj] & ~(|t_match[gj-1:0]);
                end

                assign t_tag_masked[gj*TAGL +: TAGL] = t_tag[gj*TAGL +: TAGL] & {TAGL{t_match_pri[gj]}};
                assign t_valid_masked[gj] = t_valid[gj] & t_match_pri[gj];
            end

            wire w_en_final;
            wire [31:0] w_data_final;
            btree_or_bit  #(.N(WP))         u_wen_tree   (.in(w_match_pri), .out(w_en_final));
            btree_or_word #(.N(WP), .W(32)) u_wdata_tree (.in(w_masked),    .out(w_data_final));

            wire t_en_final;
            wire t_valid_final;
            wire [TAGL-1:0] t_tag_final;
            btree_or_bit  #(.N(TP))           u_ten_tree    (.in(t_match_pri),     .out(t_en_final));
            btree_or_bit  #(.N(TP))           u_tvalid_tree (.in(t_valid_masked),  .out(t_valid_final));
            btree_or_word #(.N(TP), .W(TAGL)) u_ttag_tree   (.in(t_tag_masked),    .out(t_tag_final));

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    reg_data[gi]  <= 32'd0;
                    reg_tag[gi]   <= {TAGL{1'b0}};
                    reg_valid[gi] <= 1'b0;
                end else begin
                    if (w_en_final)
                        reg_data[gi] <= w_data_final;
                    if (t_en_final) begin
                        reg_tag[gi]   <= t_tag_final;
                        reg_valid[gi] <= t_valid_final;
                    end
                end
            end

        end
    endgenerate

    generate
        for (gi = 0; gi < RP; gi = gi + 1) begin : g_rport
            assign r_data[gi*32 +: 32]    = reg_data[r_addr[gi*5 +: 5]];
            assign r_tag[gi*TAGL +: TAGL] = reg_tag[r_addr[gi*5 +: 5]];
            assign r_valid[gi]            = reg_valid[r_addr[gi*5 +: 5]];
        end
    endgenerate

endmodule
*/