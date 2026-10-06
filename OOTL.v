`include "RAM_MUL.v"
`include "branch_predictorY.v"
`include "wb_packer.v"
`include "branch.v"
`include "crash.v"
`include "Instr_Buf.v"
`include "RSY.v"
`include "maths.v"
`include "LSQ.v"
`include "ls_unit.v"
module ooo_top #(
    parameter PN      = 3,
    parameter DW      = 32,
    parameter PCW     = 32,
    parameter ADRB    = 12,
    parameter LINEL   = 4,
    parameter TAGL_L1 = 3,
    parameter TAGL_L2 = 3,
    parameter REG_S   = 32,
    parameter ROB_S   = 32,
    parameter OPW     = 17,
    parameter IMMW    = 21,
    parameter EXPL    = 4,
    parameter CRW     = 4,
    parameter WBQ_N   = 16,
    parameter LSQ_D   = 16,
    parameter TAGL    = $clog2(ROB_S)
)(
    input              clk,
    input              rst,
    output [TAGL-1:0]  dbg_rob_head,
    output [TAGL-1:0]  dbg_rob_tail,
    output             dbg_crash
);

    localparam ITYPEW = 3;
    localparam REGW   = $clog2(REG_S);
    localparam WBP    = PN;
    localparam BEN    = DW / 8;
    localparam LP     = 1;
    localparam SP     = 1;
    localparam CP     = 1;
    localparam IDW    = $clog2(LSQ_D);

    localparam NALU = 2;
    localparam NMUL = 2;
    localparam NDIV = 2;
    localparam NIN     = 1 + NDIV + NMUL + NALU + LP + CP;
    localparam IDX_DIV = 1;
    localparam IDX_MUL = 1 + NDIV;
    localparam IDX_ALU = 1 + NDIV + NMUL;
    localparam IDX_LD  = 1 + NDIV + NMUL + NALU;
    localparam IDX_ST  = IDX_LD + LP;

    localparam OFF_IMM   = PCW;
    localparam OFF_DEST  = OFF_IMM + IMMW;
    localparam OFF_REG2  = OFF_DEST + REGW;
    localparam OFF_REG1  = OFF_REG2 + REGW;
    localparam OFF_OP    = OFF_REG1 + REGW;
    localparam OFF_ITYPE = OFF_OP + OPW;
    localparam EW        = OFF_ITYPE + ITYPEW;

    wire rst_n = ~rst;
    genvar k, p;

    wire              br_crash_valid, br_pc_valid;
    wire [TAGL-1:0]   br_crash_tail;
    wire [CRW-1:0]    br_crash_reason;
    wire [PCW-1:0]    br_new_pc;
    wire              crash_valid, pc_valid;
    wire [TAGL-1:0]   crash_tail;
    wire [CRW-1:0]    crash_reason;
    wire [PCW-1:0]    new_pc;
    wire [TAGL-1:0]   rob_head, rob_tail;

    assign dbg_rob_head = rob_head;
    assign dbg_rob_tail = rob_tail;
    assign dbg_crash    = crash_valid;

    crash_arbitrator #(.TAGL(TAGL), .PCW(PCW), .CRW(CRW)) u_ca (
        .clk(clk), .rst(rst),
        .branch_crash_valid(br_crash_valid), .branch_crash_tail(br_crash_tail),
        .branch_crash_reason(br_crash_reason),
        .branch_pc_valid(br_pc_valid), .branch_new_pc(br_new_pc),
        .crash_valid(crash_valid), .crash_tail(crash_tail), .crash_reason(crash_reason),
        .pc_valid(pc_valid), .new_pc(new_pc)
    );

    wire [PN*ADRB-1:0] mem_raddr;
    wire [PN-1:0]      mem_ren, mem_rvalid, mem_rready;
    wire [PN*DW-1:0]   mem_rdata;

    wire [LP*ADRB-1:0] ls_raddr;
    wire [LP-1:0]      ls_ren, ls_rvalid, ls_rready;
    wire [LP*DW-1:0]   ls_rdata;
    wire [SP*ADRB-1:0] ls_waddr;
    wire [SP*DW-1:0]   ls_wdata;
    wire [SP*BEN-1:0]  ls_wstrb;
    wire [SP-1:0]      ls_wen, ls_bvalid, ls_bready;

    wire [(PN+LP)*ADRB-1:0] arb_raddr = {ls_raddr, mem_raddr};
    wire [PN+LP-1:0]        arb_ren   = {ls_ren, mem_ren};
    wire [PN+LP-1:0]        arb_rready = {ls_rready, mem_rready};
    wire [PN+LP-1:0]        arb_rvalid;
    wire [(PN+LP)*DW-1:0]   arb_rdata;

    assign mem_rvalid = arb_rvalid[PN-1:0];
    assign ls_rvalid  = arb_rvalid[PN +: LP];
    assign mem_rdata  = arb_rdata[PN*DW-1:0];
    assign ls_rdata   = arb_rdata[PN*DW +: LP*DW];

    arbiter #(.DAT(DW), .ADRB(ADRB), .LINEL(LINEL), .TAGL_L1(TAGL_L1), .TAGL_L2(TAGL_L2),
              .RP(PN+LP), .WP(SP), .BEN(BEN)) u_arb (
        .clk(clk), .rst(rst),
        .raddr(arb_raddr), .ren(arb_ren), .rvalid(arb_rvalid), .rdata(arb_rdata), .rready(arb_rready),
        .waddr(ls_waddr), .wdata(ls_wdata), .wstrb(ls_wstrb), .wen(ls_wen),
        .bvalid(ls_bvalid), .bready(ls_bready)
    );

    wire [PN*PCW-1:0] pred_addr;
    wire [PN-1:0]     pred_vr, pred_taken, pred_vo;

    branch_pred_simple #(.DEPTH_BITS(8), .TAG_BITS(8), .INSTRL(PCW),
                         .NUM_READ_PORTS(PN), .NUM_WRITE_PORTS(1)) u_bp (
        .clk(clk), .rst_n(rst_n),
        .addr_r_flat(pred_addr), .valid_r(pred_vr), .pred_o(pred_taken), .pred_valid_o(pred_vo),
        .addr_w_flat({PCW{1'b0}}), .valid_w(1'b0), .learn_w(1'b0)
    );

    wire [PN*DW-1:0]  pf_instr;
    wire [PN*PCW-1:0] pf_pc;
    wire [PN-1:0]     pf_path, pf_valid;
    wire              ib_ready_all;

    prefetcher #(.IP(PN), .ADRB(ADRB), .DAT(DW), .PCW(PCW)) u_pf (
        .clk(clk), .rst(rst),
        .mem_raddr_o(mem_raddr), .mem_ren_o(mem_ren), .mem_rvalid_i(mem_rvalid),
        .mem_rdata_i(mem_rdata), .mem_rready_o(mem_rready),
        .pred_addr_r_o(pred_addr), .pred_valid_r_o(pred_vr),
        .pred_taken_i(pred_taken), .pred_valid_i(pred_vo),
        .instr_o(pf_instr), .pc_o(pf_pc), .path_taken_o(pf_path), .valid_o(pf_valid),
        .ready_i(ib_ready_all),
        .crash_i(pc_valid), .crash_pc_i(new_pc)
    );

    wire [PN-1:0]    ib_ready_o, ib_notready, ib_valid_in, ib_valid, d_ready;
    wire [PN*EW-1:0] ib_entry;
    wire             ib_any_block;

    assign ib_notready = ~ib_ready_o;
    btree_or_bit #(.N(PN)) u_ibblk (.in(ib_notready), .out(ib_any_block));
    assign ib_ready_all = ~ib_any_block;
    assign ib_valid_in  = pf_valid & {PN{ib_ready_all}};

    instr_buf #(.PN(PN)) u_ib (
        .clk(clk), .rst_n(rst_n),
        .instr_i(pf_instr), .pc_i(pf_pc), .branchside_i(pf_path),
        .valid_i(ib_valid_in), .ready_o(ib_ready_o),
        .entry_o(ib_entry), .valid_o(ib_valid), .ready_i(d_ready),
        .flush_i(crash_valid)
    );

    wire [PN*ITYPEW-1:0] q_itype;
    wire [PN*OPW-1:0]    q_op;
    wire [PN*REGW-1:0]   q_reg1, q_reg2, q_dest;
    wire [PN*IMMW-1:0]   q_imm;
    wire [PN*PCW-1:0]    q_pc;

    generate
        for (k = 0; k < PN; k = k + 1) begin : g_split
            assign q_pc[k*PCW +: PCW]           = ib_entry[k*EW +: PCW];
            assign q_imm[k*IMMW +: IMMW]        = ib_entry[k*EW + OFF_IMM +: IMMW];
            assign q_dest[k*REGW +: REGW]       = ib_entry[k*EW + OFF_DEST +: REGW];
            assign q_reg2[k*REGW +: REGW]       = ib_entry[k*EW + OFF_REG2 +: REGW];
            assign q_reg1[k*REGW +: REGW]       = ib_entry[k*EW + OFF_REG1 +: REGW];
            assign q_op[k*OPW +: OPW]           = ib_entry[k*EW + OFF_OP +: OPW];
            assign q_itype[k*ITYPEW +: ITYPEW]  = ib_entry[k*EW + OFF_ITYPE +: ITYPEW];
        end
    endgenerate

    wire [WBP*TAGL-1:0] cdb_tag;
    wire [WBP*DW-1:0]   cdb_data;
    wire [WBP*EXPL-1:0] cdb_exc;
    wire [WBP-1:0]      cdb_valid;

    wire [PN*TAGL-1:0] db_s1_tag, db_s2_tag, db_dest_tag;
    wire [PN-1:0]      db_s1_val, db_s2_val;
    wire [PN*DW-1:0]   db_s1_value, db_s2_value;
    wire [PN*OPW-1:0]  db_opcode;
    wire [PN*IMMW-1:0] db_imm;
    wire [PN*PCW-1:0]  db_pc;
    wire [PN-1:0]      v_alu, v_mul, v_div, v_br, v_load, v_str, v_trap;
    wire               r_alu, r_mul, r_div, r_br;
    wire               lsq_ready;
    wire [PN-1:0]      commit_valid;
    wire               d_o_crash;
    wire [EXPL-1:0]    d_o_exception;

    dispatcher #(.PN(PN), .REG_S(REG_S), .ROB_S(ROB_S), .DW(DW), .OPW(OPW), .IMMW(IMMW),
                 .PCW(PCW), .ITYPEW(ITYPEW), .EXPL(EXPL), .CRW(CRW), .ROB_WP(WBP)) u_disp (
        .clk(clk), .rst(rst), .crash_i(crash_valid),
        .valid_i(ib_valid), .itype_i(q_itype), .op_i(q_op),
        .reg1_i(q_reg1), .reg2_i(q_reg2), .dest_i(q_dest),
        .imm_i(q_imm), .pc_i(q_pc), .ready_o(d_ready),
        .alu_ready_i(r_alu), .mult_ready_i(r_mul), .div_ready_i(r_div),
        .load_ready_i(lsq_ready), .str_ready_i(lsq_ready), .branch_ready_i(r_br), .trap_ready_i(1'b0),
        .src1_tag_o(db_s1_tag), .src1_val_o(db_s1_val), .src1_value_o(db_s1_value),
        .src2_tag_o(db_s2_tag), .src2_val_o(db_s2_val), .src2_value_o(db_s2_value),
        .dest_tag_o(db_dest_tag), .opcode_o(db_opcode), .imm_o(db_imm), .pc_o(db_pc),
        .alu_valid_o(v_alu), .mult_valid_o(v_mul), .div_valid_o(v_div),
        .load_valid_o(v_load), .str_valid_o(v_str), .branch_valid_o(v_br), .trap_valid_o(v_trap),
        .w_addr_i(cdb_tag), .w_data_i(cdb_data), .w_exception_i(cdb_exc), .w_en_i(cdb_valid),
        .crash_tail_i(crash_tail),
        .head_ptr_o(rob_head), .tail_ptr_o(rob_tail),
        .commit_valid_o(commit_valid),
        .o_crash(d_o_crash), .o_exception(d_o_exception),
        .q_head_ptr_o(), .q_tail_ptr_o()
    );

    wire [NALU-1:0]      rsa_ovalid, rsa_iready;
    wire [NALU*DW-1:0]   rsa_s1, rsa_s2;
    wire [NALU*TAGL-1:0] rsa_dest;
    wire [NALU*OPW-1:0]  rsa_op;
    wire [NALU*IMMW-1:0] rsa_imm;
    wire [NALU*PCW-1:0]  rsa_pc;

    rs_module #(.ROWS(4), .WP(NALU), .DP(PN), .BUSP(WBP), .OPB(OPW), .TAGW(TAGL),
                .DATAW(DW), .EXPL(EXPL), .IMMS(IMMW), .PCW(PCW)) u_rs_alu (
        .clk(clk), .rst(rst),
        .i_valid(v_alu), .o_ready(r_alu),
        .i_opcode(db_opcode), .i_src1_tag(db_s1_tag), .i_src1_val(db_s1_val), .i_src1_data(db_s1_value),
        .i_src2_tag(db_s2_tag), .i_src2_val(db_s2_val), .i_src2_data(db_s2_value),
        .i_dest_tag(db_dest_tag), .i_imm(db_imm), .i_pc(db_pc),
        .bus_tag(cdb_tag), .bus_value(cdb_data), .bus_exception(cdb_exc), .bus_valid(cdb_valid),
        .o_valid(rsa_ovalid), .o_src1_data(rsa_s1), .o_src2_data(rsa_s2), .o_dest_tag(rsa_dest),
        .o_opcode(rsa_op), .o_imm(rsa_imm), .o_pc(rsa_pc), .i_ready(rsa_iready),
        .crash_valid(crash_valid), .crash_head(rob_head), .crash_tail(crash_tail)
    );

    wire [NMUL-1:0]      rsm_ovalid, rsm_iready;
    wire [NMUL*DW-1:0]   rsm_s1, rsm_s2;
    wire [NMUL*TAGL-1:0] rsm_dest;
    wire [NMUL*OPW-1:0]  rsm_op;
    wire [NMUL*IMMW-1:0] rsm_imm;
    wire [NMUL*PCW-1:0]  rsm_pc;

    rs_module #(.ROWS(4), .WP(NMUL), .DP(PN), .BUSP(WBP), .OPB(OPW), .TAGW(TAGL),
                .DATAW(DW), .EXPL(EXPL), .IMMS(IMMW), .PCW(PCW)) u_rs_mul (
        .clk(clk), .rst(rst),
        .i_valid(v_mul), .o_ready(r_mul),
        .i_opcode(db_opcode), .i_src1_tag(db_s1_tag), .i_src1_val(db_s1_val), .i_src1_data(db_s1_value),
        .i_src2_tag(db_s2_tag), .i_src2_val(db_s2_val), .i_src2_data(db_s2_value),
        .i_dest_tag(db_dest_tag), .i_imm(db_imm), .i_pc(db_pc),
        .bus_tag(cdb_tag), .bus_value(cdb_data), .bus_exception(cdb_exc), .bus_valid(cdb_valid),
        .o_valid(rsm_ovalid), .o_src1_data(rsm_s1), .o_src2_data(rsm_s2), .o_dest_tag(rsm_dest),
        .o_opcode(rsm_op), .o_imm(rsm_imm), .o_pc(rsm_pc), .i_ready(rsm_iready),
        .crash_valid(crash_valid), .crash_head(rob_head), .crash_tail(crash_tail)
    );

    wire [NDIV-1:0]      rsd_ovalid, rsd_iready;
    wire [NDIV*DW-1:0]   rsd_s1, rsd_s2;
    wire [NDIV*TAGL-1:0] rsd_dest;
    wire [NDIV*OPW-1:0]  rsd_op;
    wire [NDIV*IMMW-1:0] rsd_imm;
    wire [NDIV*PCW-1:0]  rsd_pc;

    rs_module #(.ROWS(4), .WP(NDIV), .DP(PN), .BUSP(WBP), .OPB(OPW), .TAGW(TAGL),
                .DATAW(DW), .EXPL(EXPL), .IMMS(IMMW), .PCW(PCW)) u_rs_div (
        .clk(clk), .rst(rst),
        .i_valid(v_div), .o_ready(r_div),
        .i_opcode(db_opcode), .i_src1_tag(db_s1_tag), .i_src1_val(db_s1_val), .i_src1_data(db_s1_value),
        .i_src2_tag(db_s2_tag), .i_src2_val(db_s2_val), .i_src2_data(db_s2_value),
        .i_dest_tag(db_dest_tag), .i_imm(db_imm), .i_pc(db_pc),
        .bus_tag(cdb_tag), .bus_value(cdb_data), .bus_exception(cdb_exc), .bus_valid(cdb_valid),
        .o_valid(rsd_ovalid), .o_src1_data(rsd_s1), .o_src2_data(rsd_s2), .o_dest_tag(rsd_dest),
        .o_opcode(rsd_op), .o_imm(rsd_imm), .o_pc(rsd_pc), .i_ready(rsd_iready),
        .crash_valid(crash_valid), .crash_head(rob_head), .crash_tail(crash_tail)
    );

    wire [0:0]       rsb_ovalid, rsb_iready;
    wire [DW-1:0]    rsb_s1, rsb_s2;
    wire [TAGL-1:0]  rsb_dest;
    wire [OPW-1:0]   rsb_op;
    wire [IMMW-1:0]  rsb_imm;
    wire [PCW-1:0]   rsb_pc;

    rs_module #(.ROWS(4), .WP(1), .DP(PN), .BUSP(WBP), .OPB(OPW), .TAGW(TAGL),
                .DATAW(DW), .EXPL(EXPL), .IMMS(IMMW), .PCW(PCW)) u_rs_br (
        .clk(clk), .rst(rst),
        .i_valid(v_br), .o_ready(r_br),
        .i_opcode(db_opcode), .i_src1_tag(db_s1_tag), .i_src1_val(db_s1_val), .i_src1_data(db_s1_value),
        .i_src2_tag(db_s2_tag), .i_src2_val(db_s2_val), .i_src2_data(db_s2_value),
        .i_dest_tag(db_dest_tag), .i_imm(db_imm), .i_pc(db_pc),
        .bus_tag(cdb_tag), .bus_value(cdb_data), .bus_exception(cdb_exc), .bus_valid(cdb_valid),
        .o_valid(rsb_ovalid), .o_src1_data(rsb_s1), .o_src2_data(rsb_s2), .o_dest_tag(rsb_dest),
        .o_opcode(rsb_op), .o_imm(rsb_imm), .o_pc(rsb_pc), .i_ready(rsb_iready),
        .crash_valid(crash_valid), .crash_head(rob_head), .crash_tail(crash_tail)
    );

    wire [NALU-1:0]      aw_valid, aw_ready;
    wire [NALU*TAGL-1:0] aw_tag;
    wire [NALU*DW-1:0]   aw_data;
    wire [NMUL-1:0]      mw_valid, mw_ready;
    wire [NMUL*TAGL-1:0] mw_tag;
    wire [NMUL*DW-1:0]   mw_data;
    wire [NDIV-1:0]      dw_valid, dw_ready;
    wire [NDIV*TAGL-1:0] dw_tag;
    wire [NDIV*DW-1:0]   dw_data;
    wire                 bw_valid, bw_ready;
    wire [TAGL-1:0]      bw_tag;
    wire [DW-1:0]        bw_data;

    generate
        for (p = 0; p < NALU; p = p + 1) begin : g_alu
            alu #(.TAGL(TAGL), .OPW(OPW), .DW(DW), .IMMW(IMMW)) u_alu (
                .clk(clk), .rst(rst),
                .in_valid(rsa_ovalid[p]), .in_ready(rsa_iready[p]),
                .opcode(rsa_op[p*OPW +: OPW]),
                .src1_value(rsa_s1[p*DW +: DW]), .src2_value(rsa_s2[p*DW +: DW]),
                .dest_tag(rsa_dest[p*TAGL +: TAGL]), .imm(rsa_imm[p*IMMW +: IMMW]),
                .wb_valid(aw_valid[p]), .wb_ready(aw_ready[p]),
                .wb_dest_tag(aw_tag[p*TAGL +: TAGL]), .wb_data(aw_data[p*DW +: DW]),
                .kill_i(crash_valid), .kill_head_i(rob_head), .kill_tail_i(crash_tail)
            );
        end
        for (p = 0; p < NMUL; p = p + 1) begin : g_mul
            mul_unit #(.LAT(4), .TAGL(TAGL), .OPW(OPW), .DW(DW)) u_mul (
                .clk(clk), .rst(rst),
                .in_valid(rsm_ovalid[p]), .in_ready(rsm_iready[p]),
                .opcode(rsm_op[p*OPW +: OPW]),
                .src1_value(rsm_s1[p*DW +: DW]), .src2_value(rsm_s2[p*DW +: DW]),
                .dest_tag(rsm_dest[p*TAGL +: TAGL]),
                .wb_valid(mw_valid[p]), .wb_ready(mw_ready[p]),
                .wb_dest_tag(mw_tag[p*TAGL +: TAGL]), .wb_data(mw_data[p*DW +: DW]),
                .kill_i(crash_valid), .kill_head_i(rob_head), .kill_tail_i(crash_tail)
            );
        end
        for (p = 0; p < NDIV; p = p + 1) begin : g_div
            rv32m_div #(.TAGL(TAGL), .OPW(OPW), .DW(DW)) u_div (
                .clk(clk), .rst(rst),
                .in_valid(rsd_ovalid[p]), .in_ready(rsd_iready[p]),
                .opcode(rsd_op[p*OPW +: OPW]),
                .src1_value(rsd_s1[p*DW +: DW]), .src2_value(rsd_s2[p*DW +: DW]),
                .dest_tag(rsd_dest[p*TAGL +: TAGL]),
                .wb_valid(dw_valid[p]), .wb_ready(dw_ready[p]),
                .wb_dest_tag(dw_tag[p*TAGL +: TAGL]), .wb_data(dw_data[p*DW +: DW]),
                .kill_i(crash_valid), .kill_head_i(rob_head), .kill_tail_i(crash_tail)
            );
        end
    endgenerate

    branch_fu #(.TAGL(TAGL), .OPW(OPW), .DW(DW), .IMMW(IMMW), .PCW(PCW), .CRW(CRW)) u_br (
        .clk(clk), .rst(rst),
        .in_valid(rsb_ovalid[0]), .in_ready(rsb_iready[0]),
        .opcode(rsb_op), .src1_value(rsb_s1), .src2_value(rsb_s2),
        .dest_tag(rsb_dest), .imm(rsb_imm), .pc(rsb_pc),
        .wb_valid(bw_valid), .wb_ready(bw_ready), .wb_dest_tag(bw_tag), .wb_data(bw_data),
        .crash_valid(br_crash_valid), .crash_tail(br_crash_tail), .crash_reason(br_crash_reason),
        .pc_valid(br_pc_valid), .new_pc(br_new_pc),
        .kill_i(crash_valid), .kill_head_i(rob_head), .kill_tail_i(crash_tail)
    );

    wire [LP-1:0]       lq_ld_valid, lq_ld_ready, lq_ld_fwd;
    wire [LP*TAGL-1:0]  lq_ld_tag;
    wire [LP*3-1:0]     lq_ld_f3;
    wire [LP*DW-1:0]    lq_ld_addr, lq_ld_fdata;
    wire [SP-1:0]       lq_st_valid, lq_st_ready, lq_done_valid;
    wire [SP*DW-1:0]    lq_st_addr, lq_st_data;
    wire [SP*3-1:0]     lq_st_f3;
    wire [SP*IDW-1:0]   lq_st_id, lq_done_id;
    wire [CP-1:0]       lq_cmp_valid, lq_cmp_ready;
    wire [CP*TAGL-1:0]  lq_cmp_tag;

    lsq #(.DEPTH(LSQ_D), .DP(PN), .WP(WBP), .RP(PN), .LP(LP), .SP(SP), .CP(CP),
          .HA(PN), .TAGW(TAGL), .DW(DW), .OPW(OPW), .IMMW(IMMW)) u_lsq (
        .clk(clk), .rst(rst),
        .dp_ld_valid_i(v_load), .dp_st_valid_i(v_str), .ready_o(lsq_ready),
        .dp_s1_tag_i(db_s1_tag), .dp_s1_val_i(db_s1_val), .dp_s1_value_i(db_s1_value),
        .dp_s2_tag_i(db_s2_tag), .dp_s2_val_i(db_s2_val), .dp_s2_value_i(db_s2_value),
        .dp_dest_tag_i(db_dest_tag), .dp_op_i(db_opcode), .dp_imm_i(db_imm),
        .bus_tag_i(cdb_tag), .bus_value_i(cdb_data), .bus_valid_i(cdb_valid),
        .ret_vec_i(commit_valid),
        .crash_i(crash_valid), .rob_head_i(rob_head), .crash_tail_i(crash_tail),
        .ld_valid_o(lq_ld_valid), .ld_ready_i(lq_ld_ready), .ld_fwd_o(lq_ld_fwd),
        .ld_tag_o(lq_ld_tag), .ld_funct3_o(lq_ld_f3),
        .ld_addr_o(lq_ld_addr), .ld_fdata_o(lq_ld_fdata),
        .st_valid_o(lq_st_valid), .st_ready_i(lq_st_ready),
        .st_addr_o(lq_st_addr), .st_data_o(lq_st_data), .st_funct3_o(lq_st_f3),
        .st_id_o(lq_st_id), .st_done_valid_i(lq_done_valid), .st_done_id_i(lq_done_id),
        .cmp_valid_o(lq_cmp_valid), .cmp_ready_i(lq_cmp_ready), .cmp_tag_o(lq_cmp_tag)
    );

    wire [LP-1:0]       ls_wb_valid, ls_wb_ready;
    wire [LP*TAGL-1:0]  ls_wb_tag;
    wire [LP*DW-1:0]    ls_wb_data;

    ls_unit #(.LP(LP), .SP(SP), .TAGW(TAGL), .DW(DW), .ADRB(ADRB), .BEN(BEN), .IDW(IDW)) u_ls (
        .clk(clk), .rst(rst),
        .ld_valid_i(lq_ld_valid), .ld_ready_o(lq_ld_ready), .ld_fwd_i(lq_ld_fwd),
        .ld_tag_i(lq_ld_tag), .ld_funct3_i(lq_ld_f3),
        .ld_addr_i(lq_ld_addr), .ld_fdata_i(lq_ld_fdata),
        .wb_valid_o(ls_wb_valid), .wb_ready_i(ls_wb_ready),
        .wb_tag_o(ls_wb_tag), .wb_data_o(ls_wb_data),
        .st_valid_i(lq_st_valid), .st_ready_o(lq_st_ready),
        .st_addr_i(lq_st_addr), .st_data_i(lq_st_data), .st_funct3_i(lq_st_f3),
        .st_id_i(lq_st_id), .st_done_valid_o(lq_done_valid), .st_done_id_o(lq_done_id),
        .kill_i(crash_valid), .kill_head_i(rob_head), .kill_tail_i(crash_tail),
        .mem_raddr_o(ls_raddr), .mem_ren_o(ls_ren), .mem_rvalid_i(ls_rvalid),
        .mem_rdata_i(ls_rdata), .mem_rready_o(ls_rready),
        .mem_waddr_o(ls_waddr), .mem_wdata_o(ls_wdata), .mem_wstrb_o(ls_wstrb),
        .mem_wen_o(ls_wen), .mem_bvalid_i(ls_bvalid), .mem_bready_o(ls_bready)
    );

    wire [NIN-1:0]      pk_in_valid, pk_in_ready;
    wire [NIN*TAGL-1:0] pk_in_tag;
    wire [NIN*DW-1:0]   pk_in_data;
    wire [NIN*EXPL-1:0] pk_in_exc;
    assign pk_in_exc = {(NIN*EXPL){1'b0}};

    assign pk_in_valid[0]        = bw_valid;
    assign pk_in_tag[0 +: TAGL]  = bw_tag;
    assign pk_in_data[0 +: DW]   = bw_data;
    assign bw_ready              = pk_in_ready[0];

    generate
        for (p = 0; p < NDIV; p = p + 1) begin : g_pk_div
            assign pk_in_valid[IDX_DIV+p]                  = dw_valid[p];
            assign pk_in_tag[(IDX_DIV+p)*TAGL +: TAGL]     = dw_tag[p*TAGL +: TAGL];
            assign pk_in_data[(IDX_DIV+p)*DW +: DW]        = dw_data[p*DW +: DW];
            assign dw_ready[p]                             = pk_in_ready[IDX_DIV+p];
        end
        for (p = 0; p < NMUL; p = p + 1) begin : g_pk_mul
            assign pk_in_valid[IDX_MUL+p]                  = mw_valid[p];
            assign pk_in_tag[(IDX_MUL+p)*TAGL +: TAGL]     = mw_tag[p*TAGL +: TAGL];
            assign pk_in_data[(IDX_MUL+p)*DW +: DW]        = mw_data[p*DW +: DW];
            assign mw_ready[p]                             = pk_in_ready[IDX_MUL+p];
        end
        for (p = 0; p < NALU; p = p + 1) begin : g_pk_alu
            assign pk_in_valid[IDX_ALU+p]                  = aw_valid[p];
            assign pk_in_tag[(IDX_ALU+p)*TAGL +: TAGL]     = aw_tag[p*TAGL +: TAGL];
            assign pk_in_data[(IDX_ALU+p)*DW +: DW]        = aw_data[p*DW +: DW];
            assign aw_ready[p]                             = pk_in_ready[IDX_ALU+p];
        end
        for (p = 0; p < LP; p = p + 1) begin : g_pk_ld
            assign pk_in_valid[IDX_LD+p]                   = ls_wb_valid[p];
            assign pk_in_tag[(IDX_LD+p)*TAGL +: TAGL]      = ls_wb_tag[p*TAGL +: TAGL];
            assign pk_in_data[(IDX_LD+p)*DW +: DW]         = ls_wb_data[p*DW +: DW];
            assign ls_wb_ready[p]                          = pk_in_ready[IDX_LD+p];
        end
        for (p = 0; p < CP; p = p + 1) begin : g_pk_st
            assign pk_in_valid[IDX_ST+p]                   = lq_cmp_valid[p];
            assign pk_in_tag[(IDX_ST+p)*TAGL +: TAGL]      = lq_cmp_tag[p*TAGL +: TAGL];
            assign pk_in_data[(IDX_ST+p)*DW +: DW]         = {DW{1'b0}};
            assign lq_cmp_ready[p]                         = pk_in_ready[IDX_ST+p];
        end
    endgenerate

    wire [WBP-1:0]      pk_out_valid, wbq_push_ready;
    wire [WBP*TAGL-1:0] pk_out_tag;
    wire [WBP*DW-1:0]   pk_out_data;
    wire [WBP*EXPL-1:0] pk_out_exc;

    wb_packer #(.IN(NIN), .OUT(WBP), .TW(TAGL), .DW(DW), .EXPL(EXPL)) u_pk (
        .in_tag(pk_in_tag), .in_data(pk_in_data), .in_exception(pk_in_exc),
        .in_valid(pk_in_valid), .in_ready(pk_in_ready),
        .out_tag(pk_out_tag), .out_data(pk_out_data), .out_exception(pk_out_exc),
        .out_valid(pk_out_valid), .out_ready(wbq_push_ready)
    );

    wb_queue #(.N(WBQ_N), .INP(WBP), .BUSP(WBP), .TW(TAGL), .DW(DW), .EXPL(EXPL)) u_wbq (
        .clk(clk), .rst(rst),
        .push_tag_i(pk_out_tag), .push_data_i(pk_out_data), .push_exception_i(pk_out_exc),
        .push_valid_i(pk_out_valid), .push_ready_o(wbq_push_ready),
        .bus_tag_o(cdb_tag), .bus_data_o(cdb_data), .bus_exception_o(cdb_exc),
        .bus_entry_valid_o(cdb_valid), .bus_valid_o(),
        .bus_ready_i({WBP{1'b1}}),
        .crash_i(crash_valid), .rob_head_i(rob_head), .rob_tail_i(crash_tail)
    );

endmodule