`include "abstractions.v"
module REGY #(
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

    // write side
    input      [WP-1:0]       We,
    input      [WP*ADDR-1:0]  WAddr,
    input      [WP*TAGL-1:0]  Wtag,
    input      [WP*W-1:0]     Wdata,

    // read side
    input      [RP*ADDR-1:0]  RAddr,
    output     [RP*W-1:0]     Rdata,
    output     [RP*TAGL-1:0]  Rtag,
    output     [RP-1:0]       Rvalid,

    // temp ports
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

    wire [MWP-1:0]      meta_we;
    wire [MWP*ADDR-1:0] meta_waddr;
    wire [MWP*MW-1:0]   meta_wdata;

    wire [WP*W-1:0]     data_wmask;
    wire [MWP*MW-1:0]   meta_wmask;

    genvar wp, t, rp;

    generate
        for (rp = 0; rp < RP; rp = rp + 1) begin : g_metar_real
            assign meta_raddr[rp*ADDR +: ADDR] = RAddr[rp*ADDR +: ADDR];
        end
    endgenerate

    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_metar_chk
            assign meta_raddr[(RP+wp)*ADDR +: ADDR] = WAddr[wp*ADDR +: ADDR];
            assign wr_check_tag[wp*TAGL +: TAGL] = meta_rdata[(RP+wp)*MW +: TAGL];
            assign tag_match[wp] = (wr_check_tag[wp*TAGL +: TAGL] == Wtag[wp*TAGL +: TAGL]);
            assign we_data[wp] = We[wp] & tag_match[wp];
        end
    endgenerate

    generate
        for (wp = 0; wp < WP; wp = wp + 1) begin : g_metaw_wr
            assign meta_we[wp]                  = We[wp] & tag_match[wp];
            assign meta_waddr[wp*ADDR +: ADDR]  = WAddr[wp*ADDR +: ADDR];
            assign meta_wdata[wp*MW +: MW]      = {1'b1, Wtag[wp*TAGL +: TAGL]};
        end
    endgenerate

    generate
        for (t = 0; t < TP; t = t + 1) begin : g_metaw_te
            assign meta_we[WP+t]                 = Te[t];
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
        .we    (we_data),
        .waddr (WAddr),
        .wdata (Wdata),
        .wmask (data_wmask),
        .raddr (RAddr),
        .rdata (Rdata)
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
        .we    (meta_we),
        .waddr (meta_waddr),
        .wdata (meta_wdata),
        .wmask (meta_wmask),
        .raddr (meta_raddr),
        .rdata (meta_rdata)
    );

    generate
        for (rp = 0; rp < RP; rp = rp + 1) begin : g_rout
            assign Rvalid[rp]            = meta_rdata[rp*MW + TAGL];
            assign Rtag[rp*TAGL +: TAGL] = meta_rdata[rp*MW +: TAGL];
        end
    endgenerate

endmodule