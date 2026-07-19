`include "ROBY.v"

// REGUNT: combines REGY (register file w/ rename tags) and ROBY (reorder buffer).
// Simplification: TP (REGY temp ports) is tied to WP (REGY write ports), both = PN,
// since PN also drives ROBY's SP/OP. A variant with WP > TP is possible (lets the
// regfile drain/commit faster than it renames) but is not built here.
module REGUNT #(
    parameter PN     = 3,   // shared port count: rename(TP)=push(SP)=commit(OP)=regfile-write(WP)
    parameter DW     = 32,  // data width (REGY.W = ROBY.DAT)
    parameter REG_S  = 64,  // number of physical registers
    parameter REG_RP = 3,   // REGY free read ports
    parameter ROB_S  = 32,  // ROB depth
    parameter ROB_RP = 3,   // ROBY free random read ports
    parameter ROB_WP = 3,   // ROBY writeback ports (from FUs)
    parameter EXPL   = 4,   // exception field width
    parameter CRW    = 4    // external crash reason width
)(
    input clk,
    input rst,

    // register file read ports
    input  [REG_RP*REG_ADDR-1:0] RAddr,
    output [REG_RP*DW-1:0]       Rdata,
    output [REG_RP*TAGL-1:0]     Rtag,
    output [REG_RP-1:0]          Rvalid,

    // rename request: push a new ROB entry + tag the dest register in one shot
    input  [PN-1:0]              i_temp_valid,
    input  [PN*REG_ADDR-1:0]     i_temp_addr,
    output                       o_ready,
    output [PN*TAGL-1:0]         o_temp_tag,

    // ROB writeback (FU results)
    input  [ROB_WP*TAGL-1:0]    w_addr_i,
    input  [ROB_WP*DW-1:0]      w_data_i,
    input  [ROB_WP*EXPL-1:0]    w_exception_i,
    input  [ROB_WP-1:0]         w_en_i,

    // ROB random read
    input  [ROB_RP*TAGL-1:0]    r_addr_i,
    output [ROB_RP*DW-1:0]      r_data_o,
    output [ROB_RP*REG_ADDR-1:0] r_index_o,
    output [ROB_RP-1:0]         r_valid_o,
    output [ROB_RP*EXPL-1:0]    r_exception_o,

    output [TAGL-1:0] head_ptr_o,
    output [TAGL-1:0] tail_ptr_o,

    input               crash_i,
    input  [TAGL-1:0]   crash_tail_i,
    input  [CRW-1:0]    crash_reason_i,

    output              o_crash,
    output [TAGL-1:0]   o_crash_tail,
    output              o_crash_src,
    output [CRW-1:0]    o_crash_reason,
    output [EXPL-1:0]   o_exception
);

    localparam REG_ADDR = $clog2(REG_S);
    localparam TAGL     = $clog2(ROB_S);

    wire crash;
    assign crash = o_crash;

    // gate rename requests: only fire if ROB has room and no crash this cycle
    wire [PN-1:0] eff_temp_en;
    genvar k;
    generate
        for (k = 0; k < PN; k = k + 1) begin : g_gate
            assign eff_temp_en[k] = i_temp_valid[k] & o_ready & ~crash;
        end
    endgenerate

    // port k always targets tail+k (positional, mirrors ROBY's own push_addr logic)
    wire [PN*TAGL-1:0] temp_tag;
    generate
        for (k = 0; k < PN; k = k + 1) begin : g_tag
            assign temp_tag[k*TAGL +: TAGL] = tail_ptr_o + k[TAGL-1:0];
        end
    endgenerate
    assign o_temp_tag = temp_tag;

    // commit -> regfile write bus, tag = head+k for the anti-stale-write check in REGY
    wire [PN-1:0]           op_valid;
    wire [PN*DW-1:0]        op_data;
    wire [PN*REG_ADDR-1:0]  op_index;
    wire [PN*TAGL-1:0]      wr_tag;
    generate
        for (k = 0; k < PN; k = k + 1) begin : g_wrtag
            assign wr_tag[k*TAGL +: TAGL] = head_ptr_o + k[TAGL-1:0];
        end
    endgenerate

    REGY #(
        .S    (REG_S),
        .W    (DW),
        .TAGL (TAGL),
        .ADDR (REG_ADDR),
        .RP   (REG_RP),
        .WP   (PN),
        .TP   (PN)
    ) u_regy (
        .clk   (clk),
        .rst   (rst),
        .We    (op_valid),
        .WAddr (op_index),
        .Wtag  (wr_tag),
        .Wdata (op_data),
        .RAddr (RAddr),
        .Rdata (Rdata),
        .Rtag  (Rtag),
        .Rvalid(Rvalid),
        .Te    (eff_temp_en),
        .TAddr (i_temp_addr),
        .Ttag  (temp_tag)
    );

    ROBY #(
        .S    (ROB_S),
        .DAT  (DW),
        .OP   (PN),
        .SP   (PN),
        .RP   (ROB_RP),
        .WP   (ROB_WP),
        .REFW (REG_ADDR),
        .EXPL (EXPL),
        .CRW  (CRW)
    ) u_roby (
        .clk   (clk),
        .rst   (rst),
        .sp_index_i (i_temp_addr),
        .sp_valid_i (i_temp_valid),
        .ready_o    (o_ready),
        .op_ready_i (1'b1),        // regfile write has no backpressure, always accept
        .op_data_o  (op_data),
        .op_index_o (op_index),
        .op_valid_o (op_valid),
        .r_addr_i     (r_addr_i),
        .r_data_o     (r_data_o),
        .r_index_o    (r_index_o),
        .r_valid_o    (r_valid_o),
        .r_exception_o(r_exception_o),
        .w_addr_i      (w_addr_i),
        .w_data_i      (w_data_i),
        .w_exception_i (w_exception_i),
        .w_en_i        (w_en_i),
        .head_ptr_o (head_ptr_o),
        .tail_ptr_o (tail_ptr_o),
        .crash_i        (crash_i),
        .crash_tail_i   (crash_tail_i),
        .crash_reason_i (crash_reason_i),
        .o_crash        (o_crash),
        .o_crash_tail   (o_crash_tail),
        .o_crash_src    (o_crash_src),
        .o_crash_reason (o_crash_reason),
        .o_exception    (o_exception)
    );

endmodule