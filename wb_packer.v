// wb_packer: compacts sparse writeback channels into a contiguous prefix of OUT slots.
// Lowest input index wins if more than OUT are valid; the rest see ready=0 and hold.
`include "preROB_queue.v"
module wb_packer #(
    parameter IN   = 3,
    parameter OUT  = 3,
    parameter TW   = 5,
    parameter DW   = 32,
    parameter EXPL = 4,
    parameter RW   = $clog2(IN) + 1,
    parameter W    = TW + DW + EXPL
)(
    input  [IN*TW-1:0]    in_tag,
    input  [IN*DW-1:0]    in_data,
    input  [IN*EXPL-1:0]  in_exception,
    input  [IN-1:0]       in_valid,
    output [IN-1:0]       in_ready,

    output [OUT*TW-1:0]   out_tag,
    output [OUT*DW-1:0]   out_data,
    output [OUT*EXPL-1:0] out_exception,
    output [OUT-1:0]      out_valid,
    input  [OUT-1:0]      out_ready
);
    genvar gi, gk;

    wire [IN*W-1:0]   in_word;
    wire [RW-1:0]     rank [0:IN-1];
    wire [IN*OUT-1:0] rsel;   // input gi would land on slot gk (ignores valid)
    wire [IN*OUT-1:0] sel;    // rsel & valid

    generate
        for (gi = 0; gi < IN; gi = gi + 1) begin : g_in
            wire [OUT-1:0] rdy_terms;

            assign in_word[gi*W +: W] = {in_tag[gi*TW +: TW], in_data[gi*DW +: DW], in_exception[gi*EXPL +: EXPL]};

            // rank = number of valid inputs below this one
            if (gi == 0) begin : g_r0
                assign rank[gi] = {RW{1'b0}};
            end else begin : g_rn
                assign rank[gi] = rank[gi-1] + in_valid[gi-1];
            end

            for (gk = 0; gk < OUT; gk = gk + 1) begin : g_sel
                assign rsel[gi*OUT+gk] = (rank[gi] == gk);
                assign sel[gi*OUT+gk]  = rsel[gi*OUT+gk] & in_valid[gi];
                assign rdy_terms[gk]   = rsel[gi*OUT+gk] & out_ready[gk];
            end

            // accepted if its slot exists and the slot is ready
            btree_or_bit #(.N(OUT)) u_rdy (.in(rdy_terms), .out(in_ready[gi]));
        end

        for (gk = 0; gk < OUT; gk = gk + 1) begin : g_out
            wire [IN*W-1:0] masked;
            wire [IN-1:0]   col;
            wire [W-1:0]    oword;

            for (gi = 0; gi < IN; gi = gi + 1) begin : g_m
                assign col[gi] = sel[gi*OUT+gk];
                assign masked[gi*W +: W] = in_word[gi*W +: W] & {W{col[gi]}};
            end

            btree_or_word #(.N(IN), .W(W)) u_w (.in(masked), .out(oword));
            btree_or_bit  #(.N(IN))        u_v (.in(col),    .out(out_valid[gk]));

            assign out_tag[gk*TW +: TW]           = oword[W-1 -: TW];
            assign out_data[gk*DW +: DW]          = oword[EXPL +: DW];
            assign out_exception[gk*EXPL +: EXPL] = oword[EXPL-1:0];
        end
    endgenerate
endmodule