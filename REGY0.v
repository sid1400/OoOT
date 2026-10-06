//`include "REGUNT.v"
// REGY0: same as REGY, but reg0 is hardwired data=0/valid=1/tag=0 and unwritable.
// Any read at addr==0 bypasses meta storage entirely; any write/temp-tag at addr==0
// is masked off before reaching the underlying regs primitives.
`include "ROBY.v"
`include "abstractions.v"
module REGY0 #(
    parameter S    = 32,
    parameter W    = 32,
    parameter TAGL = 5,
    parameter ADDR = 5,
    parameter RP   = 3,
    parameter WP   = 3,
    parameter TP   = 3
) (
    input                     clk,
    input                     rst,

    input      [WP-1:0]       We,
    input      [WP*ADDR-1:0]  WAddr,
    input      [WP*TAGL-1:0]  Wtag,
    input      [WP*W-1:0]     Wdata,

    input      [RP*ADDR-1:0]  RAddr,
    output     [RP*W-1:0]     Rdata,
    output     [RP*TAGL-1:0]  Rtag,
    output     [RP-1:0]       Rvalid,

    input      [TP-1:0]       Te,
    input      [TP*ADDR-1:0]  TAddr,
    input      [TP*TAGL-1:0]  Ttag
);

    localparam MW  = TAGL + 1;
    localparam MRP = RP + WP;
    localparam MWP = WP + TP;

    wire [MRP*ADDR-1:0] meta_raddr;
    wire [MRP*MW-1:0]   meta_rdata;

    wire [WP*TAGL-1:0]  wr_check_tag;
    wire [WP-1:0]       tag_match;
    wire [WP-1:0]       we_data;
    wire [WP-1:0]       we_data0;

    wire [MWP-1:0]      meta_we;
    wire [MWP-1:0]      meta_we0;
    wire [MWP*ADDR-1:0] meta_waddr;
    wire [MWP*MW-1:0]   meta_wdata;

    wire [WP*W-1:0]     data_wmask;
    wire [MWP*MW-1:0]   meta_wmask;

    wire [RP-1:0]       rd_is_zero;
    wire [RP*W-1:0]      Rdata_int;

    genvar wp, t, rp;

    // ---------------- read address fanout (real read ports + write-check ports) ----------------
    generate
        for (rp = 0; rp < RP; rp = rp + 1) begin : g_metar_real
            assign meta_raddr[rp*ADDR +: ADDR] = RAddr[rp*ADDR +: ADDR];
            assign rd_is_zero[rp] = (RAddr[rp*ADDR +: ADDR] == {ADDR{1'b0}});
        end
    endgenerate

    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_metar_chk
            assign meta_raddr[(RP+wp)*ADDR +: ADDR] = WAddr[wp*ADDR +: ADDR];
            assign wr_check_tag[wp*TAGL +: TAGL] = meta_rdata[(RP+wp)*MW +: TAGL];
            assign tag_match[wp] = (wr_check_tag[wp*TAGL +: TAGL] == Wtag[wp*TAGL +: TAGL]);
            assign we_data[wp] = We[wp] & tag_match[wp];// we dont really need we_data
            assign we_data0[wp] = We[wp] & (WAddr[wp*ADDR +: ADDR] != {ADDR{1'b0}});
        end
    endgenerate

    // ---------------- writes: mask off any target address == 0 ----------------
    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_metaw_wr
            assign meta_we[wp]                  = We[wp] & tag_match[wp];
            assign meta_we0[wp]                 = meta_we[wp] & (WAddr[wp*ADDR +: ADDR] != {ADDR{1'b0}});
            assign meta_waddr[wp*ADDR +: ADDR]  = WAddr[wp*ADDR +: ADDR];
            assign meta_wdata[wp*MW +: MW]      = {1'b1, Wtag[wp*TAGL +: TAGL]};
        end
    endgenerate

    generate
        for (t = 0; t < TP; t = t + 1) begin : g_metaw_te
            assign meta_we[WP+t]                   = Te[t];
            assign meta_we0[WP+t]                  = Te[t] & (TAddr[t*ADDR +: ADDR] != {ADDR{1'b0}});
            assign meta_waddr[(WP+t)*ADDR +: ADDR] = TAddr[t*ADDR +: ADDR];
            assign meta_wdata[(WP+t)*MW +: MW]     = {1'b0, Ttag[t*TAGL +: TAGL]};
        end
    endgenerate

    assign data_wmask = {WP*W{1'b0}};
    assign meta_wmask = {MWP*MW{1'b0}};

    regs #(
        .S  (S),
        .AW (ADDR),
        .W  (W),
        .RP (RP),
        .WP (WP)
    ) data_reg (
        .clk   (clk),
        .rst   (rst),
        .we    (we_data0),
        .waddr (WAddr),
        .wdata (Wdata),
        .wmask (data_wmask),
        .raddr (RAddr),
        .rdata (Rdata_int)
    );

    regs #(
        .S      (S),
        .AW     (ADDR),
        .W      (MW),
        .RP     (MRP),
        .WP     (MWP),
        .RSTVAL ({1'b1, {TAGL{1'b0}}})
    ) meta_reg (
        .clk   (clk),
        .rst   (rst),
        .we    (meta_we0),
        .waddr (meta_waddr),
        .wdata (meta_wdata),
        .wmask (meta_wmask),
        .raddr (meta_raddr),
        .rdata (meta_rdata)
    );

    // ---------------- read outputs: reg0 forced to data=0/valid=1/tag=0 ----------------
    generate
        for (rp = 0; rp < RP; rp = rp + 1) begin : g_rout
            assign Rvalid[rp]            = rd_is_zero[rp] ? 1'b1 : meta_rdata[rp*MW + TAGL];
            assign Rtag[rp*TAGL +: TAGL] = rd_is_zero[rp] ? {TAGL{1'b0}} : meta_rdata[rp*MW +: TAGL];
            assign Rdata[rp*W +: W]      = rd_is_zero[rp] ? {W{1'b0}} : Rdata_int[rp*W +: W];
        end
    endgenerate

endmodule