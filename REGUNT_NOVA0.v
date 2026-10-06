`include "REGY0.v"
module REGUNT_NOVA0 #(
    parameter PN     = 3,
    parameter DW     = 32,
    parameter REG_S  = 32,
    parameter REG_RP = 3,
    parameter ROB_S  = 32,
    parameter ROB_RP = 3,
    parameter ROB_WP = 3,
    parameter EXPL   = 4,
    parameter CRW    = 4
)(
    input clk,
    input rst,

    input  [REG_RP*REG_ADDR-1:0] RAddr,
    output [REG_RP*DW-1:0]       Rdata,
    output [REG_RP*TAGL-1:0]     Rtag,
    output [REG_RP-1:0]          Rvalid,

    input  [PN-1:0]              i_temp_valid,
    input  [PN*REG_ADDR-1:0]     i_temp_addr,
    output                       o_ready,
    output [PN*TAGL-1:0]         o_temp_tag,

    input  [ROB_WP*TAGL-1:0]     w_addr_i,
    input  [ROB_WP*DW-1:0]       w_data_i,
    input  [ROB_WP*EXPL-1:0]     w_exception_i,
    input  [ROB_WP-1:0]          w_en_i,

    input  [ROB_RP*TAGL-1:0]     r_addr_i,
    output [ROB_RP*DW-1:0]       r_data_o,
    output [ROB_RP*REG_ADDR-1:0] r_index_o,
    output [ROB_RP-1:0]          r_valid_o,
    output [ROB_RP*EXPL-1:0]     r_exception_o,

    output [TAGL-1:0] head_ptr_o,
    output [TAGL-1:0] tail_ptr_o,

    input               crash_i,
    input  [TAGL-1:0]   crash_tail_i,
    input  [CRW-1:0]    crash_reason_i,

    output              o_crash,
    output [TAGL-1:0]   o_crash_tail,
    output              o_crash_src,
    output [CRW-1:0]    o_crash_reason,
    output [EXPL-1:0]   o_exception,

    output [PN-1:0]     o_commit_valid,
    output              o_replay_busy
);

    localparam REG_ADDR = $clog2(REG_S);
    localparam TAGL     = $clog2(ROB_S);
    localparam CTRW     = ((REG_ADDR > TAGL) ? REG_ADDR : TAGL) + 2;

    localparam S_IDLE   = 2'd0;
    localparam S_SWEEP  = 2'd1;
    localparam S_REPLAY = 2'd2;

    reg [1:0]      state;
    reg [CTRW-1:0] base;

    wire crash  = o_crash;
    wire busy   = (state != S_IDLE);
    wire sweep  = (state == S_SWEEP);
    wire replay = (state == S_REPLAY);
    assign o_replay_busy = busy;

    wire roby_ready;
    assign o_ready = roby_ready & ~busy;

    wire [PN-1:0] eff_temp_en;
    wire [PN-1:0] sp_valid_w;
    genvar k;
    generate
        for (k = 0; k < PN; k = k + 1) begin : g_gate
            assign eff_temp_en[k] = i_temp_valid[k] & o_ready & ~crash;
            assign sp_valid_w[k]  = i_temp_valid[k] & ~busy;
        end
    endgenerate

    wire [PN*TAGL-1:0] temp_tag;
    wire [PN*TAGL-1:0] wr_tag;
    generate
        for (k = 0; k < PN; k = k + 1) begin : g_tag
            assign temp_tag[k*TAGL +: TAGL] = tail_ptr_o + k[TAGL-1:0];
            assign wr_tag[k*TAGL +: TAGL]   = head_ptr_o + k[TAGL-1:0];
        end
    endgenerate
    assign o_temp_tag = temp_tag;

    wire [PN-1:0]              op_valid;
    wire [PN*DW-1:0]           op_data;
    wire [PN*REG_ADDR-1:0]     op_index;

    assign o_commit_valid = op_valid & {PN{~busy}};

    wire [PN-1:0]              y_we;
    wire [PN*REG_ADDR-1:0]     y_waddr;
    wire [PN*TAGL-1:0]         y_wtag;
    wire [PN*DW-1:0]           y_wdata;
    wire [REG_RP*REG_ADDR-1:0] y_raddr;
    wire [REG_RP*DW-1:0]       y_rdata;
    wire [REG_RP*TAGL-1:0]     y_rtag;
    wire [REG_RP-1:0]          y_rvalid;
    wire [PN-1:0]              y_te;
    wire [PN*REG_ADDR-1:0]     y_taddr;
    wire [PN*TAGL-1:0]         y_ttag;

    wire [ROB_RP*TAGL-1:0]     y_rob_raddr;
    wire [ROB_RP*REG_ADDR-1:0] rob_rindex;

    wire [CTRW-1:0] live_cnt = {{(CTRW-TAGL){1'b0}}, (tail_ptr_o - head_ptr_o)};

    wire sweep_last  = ((base + PN) >= REG_S);
    wire replay_last = ((base + PN) >= live_cnt);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            base  <= {CTRW{1'b0}};
        end else if (crash) begin
            state <= S_SWEEP;
            base  <= {{(CTRW-1){1'b0}}, 1'b1};
        end else begin
            case (state)
                S_SWEEP: begin
                    if (sweep_last) begin
                        state <= S_REPLAY;
                        base  <= {CTRW{1'b0}};
                    end else begin
                        base <= base + PN;
                    end
                end
                S_REPLAY: begin
                    if (replay_last)
                        state <= S_IDLE;
                    else
                        base <= base + PN;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    generate
        for (k = 0; k < PN; k = k + 1) begin : g_sw
            wire [CTRW-1:0] sreg  = base + k;
            wire            s_act = sweep & (sreg < REG_S);
            wire [CTRW-1:0] roff  = base + k;
            wire            r_live = replay & (roff < live_cnt);
            wire [TAGL-1:0] rslot = head_ptr_o + roff[TAGL-1:0];
            wire [REG_ADDR-1:0] ridx = rob_rindex[k*REG_ADDR +: REG_ADDR];

            assign y_raddr[k*REG_ADDR +: REG_ADDR] = sweep ? sreg[REG_ADDR-1:0] : RAddr[k*REG_ADDR +: REG_ADDR];

            assign y_we[k]                        = sweep ? s_act : (op_valid[k] & ~busy);
            assign y_waddr[k*REG_ADDR +: REG_ADDR] = sweep ? sreg[REG_ADDR-1:0] : op_index[k*REG_ADDR +: REG_ADDR];
            assign y_wtag[k*TAGL +: TAGL]          = sweep ? y_rtag[k*TAGL +: TAGL] : wr_tag[k*TAGL +: TAGL];
            assign y_wdata[k*DW +: DW]             = sweep ? y_rdata[k*DW +: DW] : op_data[k*DW +: DW];

            assign y_te[k]                         = replay ? (r_live & (ridx != {REG_ADDR{1'b0}})) : eff_temp_en[k];
            assign y_taddr[k*REG_ADDR +: REG_ADDR] = replay ? ridx  : i_temp_addr[k*REG_ADDR +: REG_ADDR];
            assign y_ttag[k*TAGL +: TAGL]          = replay ? rslot : temp_tag[k*TAGL +: TAGL];

            assign y_rob_raddr[k*TAGL +: TAGL]     = replay ? rslot : r_addr_i[k*TAGL +: TAGL];
        end
        for (k = PN; k < REG_RP; k = k + 1) begin : g_rpass
            assign y_raddr[k*REG_ADDR +: REG_ADDR] = RAddr[k*REG_ADDR +: REG_ADDR];
        end
        for (k = PN; k < ROB_RP; k = k + 1) begin : g_robpass
            assign y_rob_raddr[k*TAGL +: TAGL] = r_addr_i[k*TAGL +: TAGL];
        end
    endgenerate

    assign Rdata  = y_rdata;
    assign Rtag   = y_rtag;
    assign Rvalid = y_rvalid;
    assign r_index_o = rob_rindex;

    REGY0 #(
        .S    (REG_S),
        .W    (DW),
        .TAGL (TAGL),
        .ADDR (REG_ADDR),
        .RP   (REG_RP),
        .WP   (PN),
        .TP   (PN)
    ) u_regy0 (
        .clk   (clk),
        .rst   (rst),
        .We    (y_we),
        .WAddr (y_waddr),
        .Wtag  (y_wtag),
        .Wdata (y_wdata),
        .RAddr (y_raddr),
        .Rdata (y_rdata),
        .Rtag  (y_rtag),
        .Rvalid(y_rvalid),
        .Te    (y_te),
        .TAddr (y_taddr),
        .Ttag  (y_ttag)
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
        .sp_valid_i (sp_valid_w),
        .ready_o    (roby_ready),
        .op_ready_i (~busy),
        .op_data_o  (op_data),
        .op_index_o (op_index),
        .op_valid_o (op_valid),
        .r_addr_i     (y_rob_raddr),
        .r_data_o     (r_data_o),
        .r_index_o    (rob_rindex),
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