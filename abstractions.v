// technically thermometer is somewhere else...
// these are moreso templates than actual modules
`include "boiling.v"
module decoder #(
    parameter AW = 5,
    parameter OW = 32
) (
    input  [AW-1:0] addr,
    output [OW-1:0] onehot
);
    genvar k;
    generate
        for (k = 0; k < OW; k = k + 1) begin : g_bit
            assign onehot[k] = (addr == k);
        end
    endgenerate
endmodule

module btree_or_bit #(
    parameter N = 2
) (
    input  [N-1:0] in,
    output         out
);
    generate
        if (N == 1) begin
            assign out = in[0];
        end else begin
            localparam NL = (N + 1) / 2;
            localparam NR = N - NL;
            wire left, right;
            btree_or_bit #(.N(NL)) u_left  (.in(in[NL-1:0]), .out(left));
            btree_or_bit #(.N(NR)) u_right (.in(in[N-1:NL]), .out(right));
            assign out = left | right;
        end
    endgenerate
endmodule


module btree_or_word #(
    parameter N = 2,
    parameter W = 32
) (
    input  [N*W-1:0] in,
    output [W-1:0]   out
);
    generate
        if (N == 1) begin
            assign out = in[W-1:0];
        end else begin
            localparam NL = (N + 1) / 2;
            localparam NR = N - NL;
            wire [W-1:0] left, right;
            btree_or_word #(.N(NL), .W(W)) u_left  (.in(in[0  +: NL*W]),  .out(left));
            btree_or_word #(.N(NR), .W(W)) u_right (.in(in[NL*W +: NR*W]), .out(right));
            assign out = left | right;
        end
    endgenerate
endmodule


module btree_mux_word #(
    parameter N  = 32,
    parameter AW = 5,
    parameter W  = 32
) (
    input  [N*W-1:0] in,
    input  [AW-1:0]  addr,
    output [W-1:0]   out
);
    generate
        if (N == 1) begin
            assign out = in[W-1:0];
        end else begin
            localparam NL = (N + 1) / 2;
            localparam NR = N - NL;
            wire [W-1:0]  left, right;
            wire          sel;
            wire [AW-1:0] raddr;
            assign sel   = (addr >= NL);
            assign raddr = addr - NL;
            btree_mux_word #(.N(NL), .AW(AW), .W(W)) u_left  (.in(in[0 +: NL*W]),    .addr(addr),  .out(left));
            btree_mux_word #(.N(NR), .AW(AW), .W(W)) u_right (.in(in[NL*W +: NR*W]), .addr(raddr), .out(right));
            assign out = sel ? right : left;
        end
    endgenerate
endmodule

module regs #(
    parameter S      = 32,
    parameter AW     = 5,
    parameter W      = 32,
    parameter RP     = 2,
    parameter WP     = 2,
    parameter [W-1:0] RSTVAL = {W{1'b0}}
) (
    input                  clk,
    input                  rst,
    input      [WP-1:0]    we,
    input      [WP*AW-1:0] waddr,
    input      [WP*W-1:0]  wdata,
    input      [WP*W-1:0]  wmask,
    input      [RP*AW-1:0] raddr,
    output     [RP*W-1:0]  rdata
);

    reg [W-1:0] mem [0:S-1];

    wire [WP*S-1:0]     wp_onehot;
    wire [WP*S-1:0]     wp_hit;
    wire [S*WP-1:0]     cell_col;
    wire [WP-1:0]       has_unmasked;
    wire [S*WP-1:0]     eff_col;
    wire [S*WP-1:0]     higher_any;
    wire [S*WP-1:0]     win;
    wire [S*WP*W-1:0]   masked_wdata;
    wire [S*W-1:0]      cell_wdata;
    wire [S*W-1:0]      mem_flat;
    wire [S*W*WP-1:0]   winbit;
    wire [S*W-1:0]      cell_we_bits;

    genvar wp, i, rp, b;

    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_wdec
            decoder #(.AW(AW), .OW(S)) u_dec (
                .addr   (waddr[wp*AW +: AW]),
                .onehot (wp_onehot[wp*S +: S])
            );
            assign wp_hit[wp*S +: S] = wp_onehot[wp*S +: S] & {S{we[wp]}};
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_col
            for (wp = 0; wp < WP; wp = wp + 1) begin : g_col_wp
                assign cell_col[i*WP+wp] = wp_hit[wp*S+i];
            end
        end
    endgenerate

    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_hasunmasked
            wire [W-1:0] not_mask;
            assign not_mask = ~wmask[wp*W +: W];
            btree_or_bit #(.N(W)) u_hu (
                .in  (not_mask),
                .out (has_unmasked[wp])
            );
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_effcol
            for (wp = 0; wp < WP; wp = wp + 1) begin : g_effcol_wp
                assign eff_col[i*WP+wp] = cell_col[i*WP+wp] & has_unmasked[wp];
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_pri
            for (wp = 0; wp < WP; wp = wp + 1) begin : g_pri_wp
                if (wp == WP-1) begin
                    assign higher_any[i*WP+wp] = 1'b0;
                end else begin
                    btree_or_bit #(.N(WP-1-wp)) u_hi (
                        .in  (eff_col[i*WP+wp+1 +: WP-1-wp]),
                        .out (higher_any[i*WP+wp])
                    );
                end
                assign win[i*WP+wp] = eff_col[i*WP+wp] & ~higher_any[i*WP+wp];
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_wdmask
            for (wp = 0; wp < WP; wp = wp + 1) begin : g_wdmask_wp
                assign masked_wdata[(i*WP+wp)*W +: W] = wdata[wp*W +: W] & {W{win[i*WP+wp]}};
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_wdtree
            btree_or_word #(.N(WP), .W(W)) u_wd (
                .in  (masked_wdata[i*WP*W +: WP*W]),
                .out (cell_wdata[i*W +: W])
            );
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_wmb_i
            for (b = 0; b < W; b = b + 1) begin : g_wmb_b
                for (wp = 0; wp < WP; wp = wp + 1) begin : g_wmb_wp
                    assign winbit[(i*W+b)*WP+wp] = win[i*WP+wp] & ~wmask[wp*W+b];
                end
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_web_i
            for (b = 0; b < W; b = b + 1) begin : g_web_b
                btree_or_bit #(.N(WP)) u_web (
                    .in  (winbit[(i*W+b)*WP +: WP]),
                    .out (cell_we_bits[i*W+b])
                );
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_mem
            wire [W-1:0] mem_next;
            assign mem_next = (cell_wdata[i*W +: W] & cell_we_bits[i*W +: W])
                             | (mem[i]              & ~cell_we_bits[i*W +: W]);
            always @(posedge clk) begin
                if (rst)
                    mem[i] <= RSTVAL;
                else
                    mem[i] <= mem_next;
            end
        end
    endgenerate

    generate
        for (i = 0; i < S; i = i + 1) begin : g_flat
            assign mem_flat[i*W +: W] = mem[i];
        end
    endgenerate

    generate
        for (rp = 0; rp < RP; rp = rp + 1) begin : g_rd
            btree_mux_word #(.N(S), .AW(AW), .W(W)) u_rd (
                .in   (mem_flat),
                .addr (raddr[rp*AW +: AW]),
                .out  (rdata[rp*W +: W])
            );
        end
    endgenerate

endmodule