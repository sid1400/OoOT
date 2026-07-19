
`include "REGUNT0.v"
// dispatcher: takes PN decoded instr_buf entries per cycle, resolves SR1/SR2
// operands against REGUNT0 (rename regfile + ROB fallback), allocates ROB
// tags/dest renames, and issues an in-order thermometered prefix of them to
// per-itype RS banks. MULTI (JAL/JALR) is not a normal bank: when it reaches
// lane 0 it is split into two same-cycle micro-ops (ALU save-pointer on
// port0, BRANCH jump on port1) with no separate FSM state needed, since both
// fire in the same cycle.
// instr_queue: small circular buffer sitting between instr_buf and dispatcher.
// Strict FIFO (no holes, unlike ROB), so occupancy at pop offset k is just
// count>k - no per-slot valid bits and no antitherm needed here. Push side
// assumes instr_buf's valid_i is thermometer-shaped (contiguous prefix),
// addresses tail+k are distinct by construction so writes are direct,
// no decoder/onehot/tree. Pop advance is driven externally by pop_count_i
// (the dispatcher's own antitherm count, since dispatch grant logic still
// lives in the dispatcher, not here).

// src_bypass: for each of PN query positions, checks lanes strictly above it
// (lower index, earlier in program order within this cycle's dispatch group)
// for a same-cycle destination-register match. Needed because the regfile's
// stored tag for a register is only updated on the NEXT edge - if two
// instructions in the same dispatch group write/read the same register
// (e.g. R1=... ; R2=R1+...), the regfile read this cycle is stale for the
// second one. On a hit, the caller must treat the source as valid=0 with
// the matched lane's speculative tag (data isn't computed yet, only the
// tag exists) - never as a resolved value. Reg 0 can never match since a
// dest of 0 is masked out (reg0 is never a real write target).
// Priority: nearest lane above wins (highest matching index below query k).
module src_bypass #(
    parameter PN   = 3,
    parameter REGW = 5,
    parameter TAGW = 5
)(
    input  [PN*REGW-1:0] dest_i,
    input  [PN-1:0]      dest_valid_i,
    input  [PN*TAGW-1:0] dest_tag_i,
    input  [PN*REGW-1:0] src_i,
    output [PN-1:0]      hit_o,
    output [PN*TAGW-1:0] tag_o
);
    genvar gk, gj;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_query
            wire [PN-1:0] match;
            wire [PN-1:0] match_pri;

            for (gj = 0; gj < PN; gj = gj + 1) begin : g_match
                if (gj >= gk) begin : g_mask
                    assign match[gj] = 1'b0;
                end else begin : g_cmp
                    assign match[gj] = dest_valid_i[gj]
                                      & (dest_i[gj*REGW +: REGW] == src_i[gk*REGW +: REGW])
                                      & (dest_i[gj*REGW +: REGW] != {REGW{1'b0}});
                end
            end

            for (gj = 0; gj < PN; gj = gj + 1) begin : g_pri
                if (gj == PN-1) begin : g_pri_top
                    assign match_pri[gj] = match[gj];
                end else begin : g_pri_rest
                    assign match_pri[gj] = match[gj] & ~(|match[PN-1:gj+1]);
                end
            end

            wire [PN*TAGW-1:0] tag_masked;
            for (gj = 0; gj < PN; gj = gj + 1) begin : g_tagmask
                assign tag_masked[gj*TAGW +: TAGW] = dest_tag_i[gj*TAGW +: TAGW] & {TAGW{match_pri[gj]}};
            end

            btree_or_bit  #(.N(PN))          u_hit (.in(match_pri), .out(hit_o[gk]));
            btree_or_word #(.N(PN), .W(TAGW)) u_tag (.in(tag_masked), .out(tag_o[gk*TAGW +: TAGW]));
        end
    endgenerate

endmodule


module instr_queue #(
    parameter PN     = 3,
    parameter S      = 8,
    parameter ITYPEW = 3,
    parameter OPW    = 17,
    parameter REGW   = 5,
    parameter IMMW   = 21,
    parameter PCW    = 32
)(
    input clk,
    input rst,
    input crash_i,

    // ---------------- push side (from instr_buf) ----------------
    input  [PN-1:0]          valid_i,
    input  [PN*ITYPEW-1:0]   itype_i,
    input  [PN*OPW-1:0]      op_i,
    input  [PN*REGW-1:0]     reg1_i,
    input  [PN*REGW-1:0]     reg2_i,
    input  [PN*REGW-1:0]     dest_i,
    input  [PN*IMMW-1:0]     imm_i,
    input  [PN*PCW-1:0]      pc_i,
    output [PN-1:0]          ready_o,

    // ---------------- pop side (to dispatcher) ----------------
    output [PN-1:0]          occ_o,
    output [PN*ITYPEW-1:0]   itype_o,
    output [PN*OPW-1:0]      op_o,
    output [PN*REGW-1:0]     reg1_o,
    output [PN*REGW-1:0]     reg2_o,
    output [PN*REGW-1:0]     dest_o,
    output [PN*IMMW-1:0]     imm_o,
    output [PN*PCW-1:0]      pc_o,
    input  [CW-1:0]          pop_count_i,

    output [PTRW-1:0] head_ptr_o,
    output [PTRW-1:0] tail_ptr_o
);

    localparam PTRW = $clog2(S);
    localparam CW   = (PN > 1) ? $clog2(PN) + 1 : 1;

    reg [PTRW-1:0]   head, tail;
    reg [PTRW:0]     count;

    assign head_ptr_o = head;
    assign tail_ptr_o = tail;

    // ---------------- push accept: all-or-nothing, capacity for a full PN burst ----------------
    integer pj;
    reg [CW-1:0] n_push;
    always @(*) begin
        n_push = {CW{1'b0}};
        for (pj = 0; pj < PN; pj = pj + 1)
            n_push = n_push + valid_i[pj];
    end

    wire push_ready_w = ((S - count) >= PN);
    wire [PN-1:0] eff_push;
    genvar gp;
    generate
        for (gp = 0; gp < PN; gp = gp + 1) begin : g_effpush
            assign eff_push[gp] = valid_i[gp] & push_ready_w & ~crash_i;
        end
    endgenerate
    assign ready_o = {PN{push_ready_w}};

    wire [CW-1:0] n_push_eff = push_ready_w ? n_push : {CW{1'b0}};

    // ---------------- pop advance: clamp to occupancy, matches self-gating convention ----------------
    wire [PTRW:0] eff_pop = (pop_count_i > count) ? count : pop_count_i;

    always @(posedge clk or posedge rst) begin
        if (rst || crash_i) begin
            head  <= {PTRW{1'b0}};
            tail  <= {PTRW{1'b0}};
            count <= {(PTRW+1){1'b0}};
        end else begin
            head  <= head + eff_pop[PTRW-1:0];
            tail  <= tail + n_push_eff[PTRW-1:0];
            count <= count + n_push_eff - eff_pop;
        end
    end

    // ---------------- field storage: plain per-field arrays, direct-indexed ----------------
    reg [ITYPEW-1:0] itype_mem [0:S-1];
    reg [OPW-1:0]    op_mem    [0:S-1];
    reg [REGW-1:0]   reg1_mem  [0:S-1];
    reg [REGW-1:0]   reg2_mem  [0:S-1];
    reg [REGW-1:0]   dest_mem  [0:S-1];
    reg [IMMW-1:0]   imm_mem   [0:S-1];
    reg [PCW-1:0]    pc_mem    [0:S-1];

    generate
        for (gp = 0; gp < PN; gp = gp + 1) begin : g_push
            wire [PTRW-1:0] push_addr = tail + gp[PTRW-1:0];
            always @(posedge clk) begin
                if (eff_push[gp]) begin
                    itype_mem[push_addr] <= itype_i[gp*ITYPEW +: ITYPEW];
                    op_mem[push_addr]    <= op_i[gp*OPW +: OPW];
                    reg1_mem[push_addr]  <= reg1_i[gp*REGW +: REGW];
                    reg2_mem[push_addr]  <= reg2_i[gp*REGW +: REGW];
                    dest_mem[push_addr]  <= dest_i[gp*REGW +: REGW];
                    imm_mem[push_addr]   <= imm_i[gp*IMMW +: IMMW];
                    pc_mem[push_addr]    <= pc_i[gp*PCW +: PCW];
                end
            end
        end
    endgenerate

    // ---------------- pop read: combinational, contiguous from head ----------------
    genvar gk;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_pop
            wire [PTRW-1:0] pop_addr = head + gk[PTRW-1:0];
            assign occ_o[gk] = (count > gk);
            assign itype_o[gk*ITYPEW +: ITYPEW] = itype_mem[pop_addr];
            assign op_o[gk*OPW +: OPW]          = op_mem[pop_addr];
            assign reg1_o[gk*REGW +: REGW]      = reg1_mem[pop_addr];
            assign reg2_o[gk*REGW +: REGW]      = reg2_mem[pop_addr];
            assign dest_o[gk*REGW +: REGW]      = dest_mem[pop_addr];
            assign imm_o[gk*IMMW +: IMMW]       = imm_mem[pop_addr];
            assign pc_o[gk*PCW +: PCW]          = pc_mem[pop_addr];
        end
    endgenerate

endmodule

// dispatcher: takes PN decoded instr_buf entries per cycle, resolves SR1/SR2
// operands against REGUNT0 (rename regfile + ROB fallback), allocates ROB
// tags/dest renames, and issues an in-order thermometered prefix of them to
// per-itype RS banks. MULTI (JAL/JALR) is not a normal bank: when it reaches
// lane 0 it is split into two same-cycle micro-ops (ALU save-pointer on
// port0, BRANCH jump on port1) with no separate FSM state needed, since both
// fire in the same cycle.
module dispatcher #(
    parameter PN      = 3,
    parameter REG_S   = 32,
    parameter ROB_S   = 32,
    parameter DW      = 32,
    parameter OPW     = 17,
    parameter IMMW    = 21,
    parameter PCW     = 32,
    parameter ITYPEW  = 3,
    parameter EXPL    = 4,
    parameter CRW     = 4,
    parameter ROB_WP  = PN,
    parameter [DW-1:0] PC_INCR = 32'd4
)(
    input clk,
    input rst,
    input crash_i,

    // ---------------- pop side of instr_queue ----------------
    input  [PN-1:0]           occ_i,
    input  [PN*ITYPEW-1:0]    itype_i,
    input  [PN*OPW-1:0]       op_i,
    input  [PN*REG_ADDR-1:0]  reg1_i,
    input  [PN*REG_ADDR-1:0]  reg2_i,
    input  [PN*REG_ADDR-1:0]  dest_i,
    input  [PN*IMMW-1:0]      imm_i,
    input  [PN*PCW-1:0]       pc_i,
    output [CW-1:0]           pop_count_o,

    // ---------------- per-bank ready-in ----------------
    input alu_ready_i,
    input mult_ready_i,
    input div_ready_i,
    input load_ready_i,
    input str_ready_i,
    input branch_ready_i,
    input trap_ready_i,

    // ---------------- shared dispatch data bus (broadcast to all banks) ----------------
    output [PN*TAGL-1:0] src1_tag_o,
    output [PN-1:0]      src1_val_o,
    output [PN*DW-1:0]   src1_value_o,
    output [PN*TAGL-1:0] src2_tag_o,
    output [PN-1:0]      src2_val_o,
    output [PN*DW-1:0]   src2_value_o,
    output [PN*TAGL-1:0] dest_tag_o,
    output [PN*OPW-1:0]  opcode_o,
    output [PN*IMMW-1:0] imm_o,
    output [PN*PCW-1:0]  pc_o,

    // ---------------- per-bank valid (which of the shared-bus lanes belong to that bank) ----------------
    output [PN-1:0] alu_valid_o,
    output [PN-1:0] mult_valid_o,
    output [PN-1:0] div_valid_o,
    output [PN-1:0] load_valid_o,
    output [PN-1:0] str_valid_o,
    output [PN-1:0] branch_valid_o,
    output [PN-1:0] trap_valid_o,

    // ---------------- FU writeback into ROB (pass-through to REGUNT0) ----------------
    input  [ROB_WP*TAGL-1:0] w_addr_i,
    input  [ROB_WP*DW-1:0]   w_data_i,
    input  [ROB_WP*EXPL-1:0] w_exception_i,
    input  [ROB_WP-1:0]      w_en_i,

    output [TAGL-1:0] head_ptr_o,
    output [TAGL-1:0] tail_ptr_o,
    output             o_crash,
    output [EXPL-1:0]  o_exception
);

    localparam REG_ADDR = $clog2(REG_S);
    localparam TAGL      = $clog2(ROB_S);
    localparam CW        = (PN > 1) ? $clog2(PN) + 1 : 1;

    localparam ITYPE_ALU    = 3'd0;
    localparam ITYPE_MULT   = 3'd1;
    localparam ITYPE_DIV    = 3'd2;
    localparam ITYPE_LOAD   = 3'd3;
    localparam ITYPE_STR    = 3'd4;
    localparam ITYPE_BRANCH = 3'd5;
    localparam ITYPE_TRAP   = 3'd6;
    localparam ITYPE_MULTI  = 3'd7;

    genvar gk;

    // ================= REGUNT0 instance =================
    wire [2*PN*REG_ADDR-1:0] regfile_raddr;
    wire [2*PN*DW-1:0]       regfile_rdata;
    wire [2*PN*TAGL-1:0]     regfile_rtag;
    wire [2*PN-1:0]          regfile_rvalid;

    wire [PN-1:0]         temp_valid_w;
    wire [PN*REG_ADDR-1:0] temp_addr_w;
    wire                   regunt_ready_w;
    wire [PN*TAGL-1:0]     temp_tag_w;

    wire [2*PN*TAGL-1:0] rob_raddr;
    wire [2*PN*DW-1:0]   rob_rdata;
    wire [2*PN*REG_ADDR-1:0] rob_rindex;
    wire [2*PN-1:0]      rob_rvalid;
    wire [2*PN*EXPL-1:0] rob_rexc;

    wire crash_w = o_crash;

    REGUNT0 #(
        .PN     (PN),
        .DW     (DW),
        .REG_S  (REG_S),
        .REG_RP (2*PN),
        .ROB_S  (ROB_S),
        .ROB_RP (2*PN),
        .ROB_WP (ROB_WP),
        .EXPL   (EXPL),
        .CRW    (CRW)
    ) u_regunt0 (
        .clk (clk),
        .rst (rst),
        .RAddr  (regfile_raddr),
        .Rdata  (regfile_rdata),
        .Rtag   (regfile_rtag),
        .Rvalid (regfile_rvalid),
        .i_temp_valid (temp_valid_w),
        .i_temp_addr  (temp_addr_w),
        .o_ready      (regunt_ready_w),
        .o_temp_tag   (temp_tag_w),
        .w_addr_i (w_addr_i),
        .w_data_i (w_data_i),
        .w_exception_i (w_exception_i),
        .w_en_i (w_en_i),
        .r_addr_i (rob_raddr),
        .r_data_o (rob_rdata),
        .r_index_o (rob_rindex),
        .r_valid_o (rob_rvalid),
        .r_exception_o (rob_rexc),
        .head_ptr_o (head_ptr_o),
        .tail_ptr_o (tail_ptr_o),
        .crash_i (crash_i),
        .crash_tail_i ({TAGL{1'b0}}),
        .crash_reason_i ({CRW{1'b0}}),
        .o_crash (o_crash),
        .o_crash_tail (),
        .o_crash_src (),
        .o_crash_reason (),
        .o_exception (o_exception)
    );

    assign dest_tag_o = temp_tag_w;

    // ================= intra-cycle same-group hazard bypass =================
    wire [PN-1:0]      bypass1_hit_w, bypass2_hit_w;
    wire [PN*TAGL-1:0] bypass1_tag_w, bypass2_tag_w;

    src_bypass #(.PN(PN), .REGW(REG_ADDR), .TAGW(TAGL)) u_bypass1 (
        .dest_i       (dest_i),
        .dest_valid_i (occ_i),
        .dest_tag_i   (temp_tag_w),
        .src_i        (reg1_i),
        .hit_o        (bypass1_hit_w),
        .tag_o        (bypass1_tag_w)
    );

    src_bypass #(.PN(PN), .REGW(REG_ADDR), .TAGW(TAGL)) u_bypass2 (
        .dest_i       (dest_i),
        .dest_valid_i (occ_i),
        .dest_tag_i   (temp_tag_w),
        .src_i        (reg2_i),
        .hit_o        (bypass2_hit_w),
        .tag_o        (bypass2_tag_w)
    );

    // ================= SR1/SR2 resolution (2*PN ports, port 2k=src1, 2k+1=src2) =================
    wire [PN*TAGL-1:0] s1_tag_w, s2_tag_w;
    wire [PN-1:0]      s1_val_w, s2_val_w;
    wire [PN*DW-1:0]   s1_value_w, s2_value_w;

    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_sr
            assign regfile_raddr[(2*gk)*REG_ADDR   +: REG_ADDR] = reg1_i[gk*REG_ADDR +: REG_ADDR];
            assign regfile_raddr[(2*gk+1)*REG_ADDR +: REG_ADDR] = reg2_i[gk*REG_ADDR +: REG_ADDR];

            assign rob_raddr[(2*gk)*TAGL   +: TAGL] = regfile_rtag[(2*gk)*TAGL   +: TAGL];
            assign rob_raddr[(2*gk+1)*TAGL +: TAGL] = regfile_rtag[(2*gk+1)*TAGL +: TAGL];

            assign s1_tag_w[gk*TAGL +: TAGL] = bypass1_hit_w[gk] ? bypass1_tag_w[gk*TAGL +: TAGL]
                                              : regfile_rtag[(2*gk)*TAGL +: TAGL];
            assign s1_val_w[gk] = bypass1_hit_w[gk] ? 1'b0 : (regfile_rvalid[2*gk] | rob_rvalid[2*gk]);
            assign s1_value_w[gk*DW +: DW] = bypass1_hit_w[gk] ? {DW{1'b0}}
                                             : (regfile_rvalid[2*gk] ? regfile_rdata[(2*gk)*DW +: DW]
                                             : (rob_rvalid[2*gk] ? rob_rdata[(2*gk)*DW +: DW] : {DW{1'b0}}));

            assign s2_tag_w[gk*TAGL +: TAGL] = bypass2_hit_w[gk] ? bypass2_tag_w[gk*TAGL +: TAGL]
                                              : regfile_rtag[(2*gk+1)*TAGL +: TAGL];
            assign s2_val_w[gk] = bypass2_hit_w[gk] ? 1'b0 : (regfile_rvalid[2*gk+1] | rob_rvalid[2*gk+1]);
            assign s2_value_w[gk*DW +: DW] = bypass2_hit_w[gk] ? {DW{1'b0}}
                                             : (regfile_rvalid[2*gk+1] ? regfile_rdata[(2*gk+1)*DW +: DW]
                                             : (rob_rvalid[2*gk+1] ? rob_rdata[(2*gk+1)*DW +: DW] : {DW{1'b0}}));
        end
    endgenerate

    // ================= bank-ready mux per lane =================
    reg [PN-1:0] bank_ready_w;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_bready
            always @(*) begin
                case (itype_i[gk*ITYPEW +: ITYPEW])
                    ITYPE_ALU:    bank_ready_w[gk] = alu_ready_i;
                    ITYPE_MULT:   bank_ready_w[gk] = mult_ready_i;
                    ITYPE_DIV:    bank_ready_w[gk] = div_ready_i;
                    ITYPE_LOAD:   bank_ready_w[gk] = load_ready_i;
                    ITYPE_STR:    bank_ready_w[gk] = str_ready_i;
                    ITYPE_BRANCH: bank_ready_w[gk] = branch_ready_i;
                    ITYPE_TRAP:   bank_ready_w[gk] = trap_ready_i;
                    default:      bank_ready_w[gk] = 1'b0;
                endcase
            end
        end
    endgenerate

    // ================= antitherm: in-order allow/block, MULTI blocks itself+behind =================
    wire [PN-1:0] raw_allow_w, block_w;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_ab
            assign raw_allow_w[gk] = occ_i[gk] & bank_ready_w[gk] & regunt_ready_w;
            assign block_w[gk]     = occ_i[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_MULTI);
        end
    endgenerate

    wire [PN-1:0] stack_allow_w, stack_block_w, stack_out_w;
    wire [CW-1:0] stack_count_w;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_stackmap
            assign stack_allow_w[PN-1-gk] = raw_allow_w[gk];
            assign stack_block_w[PN-1-gk] = block_w[gk];
        end
    endgenerate

    antitherm #(.PN(PN), .CW(CW)) u_antitherm (
        .allow (stack_allow_w),
        .block (stack_block_w),
        .out   (stack_out_w),
        .count (stack_count_w)
    );

    wire [PN-1:0] grant_w;
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_grantmap
            assign grant_w[gk] = stack_out_w[PN-1-gk];
        end
    endgenerate

    // ================= MULTI same-cycle split, only meaningful when it sits at lane 0 =================
    wire multi_fire_w = occ_i[0] & (itype_i[0*ITYPEW +: ITYPEW] == ITYPE_MULTI)
                       & alu_ready_i & branch_ready_i & regunt_ready_w & ~crash_w;

    wire [OPW-1:0] add_op_const = {7'b0000000, 3'b000, 7'b0110011};

    // ---- port 0 (ALU save-pointer micro-op: dest = PC + PC_INCR, or normal lane0 dispatch) ----
    assign src1_tag_o[0*TAGL +: TAGL]   = multi_fire_w ? {TAGL{1'b0}} : s1_tag_w[0*TAGL +: TAGL];
    assign src1_val_o[0]                = multi_fire_w ? 1'b1 : s1_val_w[0];
    assign src1_value_o[0*DW +: DW]     = multi_fire_w ? pc_i[0*PCW +: DW] : s1_value_w[0*DW +: DW];
    assign src2_tag_o[0*TAGL +: TAGL]   = multi_fire_w ? {TAGL{1'b0}} : s2_tag_w[0*TAGL +: TAGL];
    assign src2_val_o[0]                = multi_fire_w ? 1'b1 : s2_val_w[0];
    assign src2_value_o[0*DW +: DW]     = multi_fire_w ? PC_INCR : s2_value_w[0*DW +: DW];
    assign opcode_o[0*OPW +: OPW]       = multi_fire_w ? add_op_const : op_i[0*OPW +: OPW];
    assign imm_o[0*IMMW +: IMMW]        = imm_i[0*IMMW +: IMMW];
    assign pc_o[0*PCW +: PCW]           = pc_i[0*PCW +: PCW];

    // ---- port 1 (BRANCH jump micro-op borrowing physical port1, or normal lane1 dispatch) ----
    // PN is always > 1 in this project, so port 1 unconditionally exists.
    assign src1_tag_o[1*TAGL +: TAGL] = multi_fire_w ? s1_tag_w[0*TAGL +: TAGL]  : s1_tag_w[1*TAGL +: TAGL];
    assign src1_val_o[1]              = multi_fire_w ? s1_val_w[0]               : s1_val_w[1];
    assign src1_value_o[1*DW +: DW]   = multi_fire_w ? s1_value_w[0*DW +: DW]    : s1_value_w[1*DW +: DW];
    assign src2_tag_o[1*TAGL +: TAGL] = multi_fire_w ? s2_tag_w[0*TAGL +: TAGL]  : s2_tag_w[1*TAGL +: TAGL];
    assign src2_val_o[1]              = multi_fire_w ? s2_val_w[0]               : s2_val_w[1];
    assign src2_value_o[1*DW +: DW]   = multi_fire_w ? s2_value_w[0*DW +: DW]    : s2_value_w[1*DW +: DW];
    assign opcode_o[1*OPW +: OPW]     = multi_fire_w ? op_i[0*OPW +: OPW]        : op_i[1*OPW +: OPW];
    assign imm_o[1*IMMW +: IMMW]      = multi_fire_w ? imm_i[0*IMMW +: IMMW]     : imm_i[1*IMMW +: IMMW];
    assign pc_o[1*PCW +: PCW]         = multi_fire_w ? pc_i[0*PCW +: PCW]        : pc_i[1*PCW +: PCW];

    // ---- remaining lanes: plain pass-through ----
    generate
        for (gk = 2; gk < PN; gk = gk + 1) begin : g_normal
            assign src1_tag_o[gk*TAGL +: TAGL] = s1_tag_w[gk*TAGL +: TAGL];
            assign src1_val_o[gk]              = s1_val_w[gk];
            assign src1_value_o[gk*DW +: DW]   = s1_value_w[gk*DW +: DW];
            assign src2_tag_o[gk*TAGL +: TAGL] = s2_tag_w[gk*TAGL +: TAGL];
            assign src2_val_o[gk]              = s2_val_w[gk];
            assign src2_value_o[gk*DW +: DW]   = s2_value_w[gk*DW +: DW];
            assign opcode_o[gk*OPW +: OPW]     = op_i[gk*OPW +: OPW];
            assign imm_o[gk*IMMW +: IMMW]      = imm_i[gk*IMMW +: IMMW];
            assign pc_o[gk*PCW +: PCW]         = pc_i[gk*PCW +: PCW];
        end
    endgenerate

    // ================= per-bank valid: normal grant + multi override on ports 0/1 =================
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_bankvalid
            wire is_alu    = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_ALU);
            wire is_mult   = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_MULT);
            wire is_div    = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_DIV);
            wire is_load   = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_LOAD);
            wire is_str    = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_STR);
            wire is_branch = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_BRANCH);
            wire is_trap   = grant_w[gk] & (itype_i[gk*ITYPEW +: ITYPEW] == ITYPE_TRAP);

            assign alu_valid_o[gk]    = is_alu    | ((gk == 0) & multi_fire_w);
            assign branch_valid_o[gk] = is_branch | ((gk == 1) & multi_fire_w);
            assign mult_valid_o[gk]   = is_mult;
            assign div_valid_o[gk]    = is_div;
            assign load_valid_o[gk]   = is_load;
            assign str_valid_o[gk]    = is_str;
            assign trap_valid_o[gk]   = is_trap;
        end
    endgenerate

    // ================= REGUNT0 push ports =================
    generate
        for (gk = 0; gk < PN; gk = gk + 1) begin : g_push
            if (gk == 0) begin : g_push0
                assign temp_valid_w[gk] = grant_w[gk] | multi_fire_w;
                assign temp_addr_w[gk*REG_ADDR +: REG_ADDR] = dest_i[gk*REG_ADDR +: REG_ADDR];
            end else if (gk == 1) begin : g_push1
                assign temp_valid_w[gk] = grant_w[gk] | multi_fire_w;
                assign temp_addr_w[gk*REG_ADDR +: REG_ADDR] = multi_fire_w ? {REG_ADDR{1'b0}} : dest_i[gk*REG_ADDR +: REG_ADDR];
            end else begin : g_pushN
                assign temp_valid_w[gk] = grant_w[gk];
                assign temp_addr_w[gk*REG_ADDR +: REG_ADDR] = dest_i[gk*REG_ADDR +: REG_ADDR];
            end
        end
    endgenerate

    // ================= pop count back to instr_queue =================
    assign pop_count_o = stack_count_w + multi_fire_w;

endmodule