// lsq: unified load/store queue. Circular, in-order alloc, holes allowed.
// Loads free at handoff to the load module. Stores free when the store module reports done.
// Needs: btree_or_bit, btree_or_word, btree_mux_word, antitherm.
//`include "abstractions.v"
module lsq #(
    parameter DEPTH = 16,   // power of 2
    parameter DP    = 3,    // dispatch lanes
    parameter WP    = 3,    // CDB snoop ports
    parameter RP    = 3,    // retire (ROB commit) slots
    parameter LP    = 1,    // load handoff ports
    parameter SP    = 1,    // store drain ports
    parameter CP    = 1,    // store completion (to ROB) ports
    parameter HA    = 3,    // max head advance per cycle
    parameter TAGW  = 5,    // ROB tag width
    parameter DW    = 32,
    parameter OPW   = 17,   // {funct7, funct3, opcode}
    parameter IMMW  = 21
)(
    input clk,
    input rst,

    // dispatch (shared bus from dispatcher, lane k = bit/slice k)
    input  [DP-1:0]       dp_ld_valid_i,
    input  [DP-1:0]       dp_st_valid_i,
    output                ready_o,
    input  [DP*TAGW-1:0]  dp_s1_tag_i,
    input  [DP-1:0]       dp_s1_val_i,
    input  [DP*DW-1:0]    dp_s1_value_i,
    input  [DP*TAGW-1:0]  dp_s2_tag_i,
    input  [DP-1:0]       dp_s2_val_i,
    input  [DP*DW-1:0]    dp_s2_value_i,
    input  [DP*TAGW-1:0]  dp_dest_tag_i,
    input  [DP*OPW-1:0]   dp_op_i,
    input  [DP*IMMW-1:0]  dp_imm_i,

    // CDB snoop
    input  [WP*TAGW-1:0]  bus_tag_i,
    input  [WP*DW-1:0]    bus_value_i,
    input  [WP-1:0]       bus_valid_i,

    // retire: bit k = ROB slot rob_head_i+k committed this cycle
    input  [RP-1:0]       ret_vec_i,

    // crash (same filter as RS)
    input                 crash_i,
    input  [TAGW-1:0]     rob_head_i,
    input  [TAGW-1:0]     crash_tail_i,

    // load handoff to load module (valid/ready)
    output [LP-1:0]       ld_valid_o,
    input  [LP-1:0]       ld_ready_i,
    output [LP-1:0]       ld_fwd_o,
    output [LP*TAGW-1:0]  ld_tag_o,
    output [LP*3-1:0]     ld_funct3_o,
    output [LP*DW-1:0]    ld_addr_o,
    output [LP*DW-1:0]    ld_fdata_o,

    // store drain to store module (valid/ready, port j older than j+1)
    output [SP-1:0]       st_valid_o,
    input  [SP-1:0]       st_ready_i,
    output [SP*DW-1:0]    st_addr_o,
    output [SP*DW-1:0]    st_data_o,
    output [SP*3-1:0]     st_funct3_o,
    output [SP*$clog2(DEPTH)-1:0] st_id_o,
    input  [SP-1:0]       st_done_valid_i,
    input  [SP*$clog2(DEPTH)-1:0] st_done_id_i,

    // store completion to ROB (valid/ready; data/exception tied 0 outside)
    output [CP-1:0]       cmp_valid_o,
    input  [CP-1:0]       cmp_ready_i,
    output [CP*TAGW-1:0]  cmp_tag_o
);

    localparam PTRW = $clog2(DEPTH);
    localparam CNTW = PTRW + 1;
    localparam RKW  = $clog2(DP + 1);
    localparam HAW  = $clog2(HA) + 1;
    localparam SPW  = $clog2(SP) + 1;

    // lane word layout, LSB first
    localparam L_DATA = 0;
    localparam L_ADDR = L_DATA + DW;
    localparam L_DV   = L_ADDR + DW;
    localparam L_AV   = L_DV + 1;
    localparam L_S2T  = L_AV + 1;
    localparam L_S1T  = L_S2T + TAGW;
    localparam L_OFF  = L_S1T + TAGW;
    localparam L_F3   = L_OFF + IMMW;
    localparam L_ST   = L_F3 + 3;
    localparam L_TAG  = L_ST + 1;
    localparam EWD    = L_TAG + TAGW;

    function in_circ_range;
        input [TAGW-1:0] idx, hd, tl;
        begin
            if (hd <= tl) in_circ_range = (idx >= hd) && (idx < tl);
            else          in_circ_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    reg [PTRW-1:0] head, tail;
    reg [CNTW-1:0] count;

    // entry state
    reg [TAGW-1:0] tag_r   [0:DEPTH-1];
    reg            valid_r [0:DEPTH-1];
    reg            isst_r  [0:DEPTH-1];
    reg            ret_r   [0:DEPTH-1];   // push_valid: store retired from ROB
    reg            sent_r  [0:DEPTH-1];   // store handed to store module
    reg            cdone_r [0:DEPTH-1];   // store completion sent to ROB
    reg            av_r    [0:DEPTH-1];
    reg            dv_r    [0:DEPTH-1];
    reg [DW-1:0]   addr_r  [0:DEPTH-1];
    reg [DW-1:0]   data_r  [0:DEPTH-1];
    reg [IMMW-1:0] off_r   [0:DEPTH-1];
    reg [TAGW-1:0] s1t_r   [0:DEPTH-1];
    reg [TAGW-1:0] s2t_r   [0:DEPTH-1];
    reg [2:0]      f3_r    [0:DEPTH-1];

    // per-entry flags exported to module level
    wire [DEPTH-1:0] valid_f, kill_f, first_f;
    wire [DEPTH-1:0] ld_cand_f, ld_fwd_f, cmp_cand_f, pend_f, infl_f;
    wire [DEPTH-1:0] ld_acc_f, cmp_acc_f, sent_set_f;
    wire [DEPTH*DW-1:0]   fdata_f;
    wire [DEPTH*LP-1:0]   ld_sel_f;
    wire [DEPTH*CP-1:0]   cmp_sel_f;
    wire [DEPTH*SP-1:0]   dsel_f;
    wire [DEPTH*PTRW-1:0] first_off_m;

    // masked per-port selections
    wire [LP*DEPTH*TAGW-1:0] ld_tag_m;
    wire [LP*DEPTH*3-1:0]    ld_f3_m;
    wire [LP*DEPTH*DW-1:0]   ld_addr_m, ld_fd_m;
    wire [LP*DEPTH-1:0]      ld_fwd_m;
    wire [CP*DEPTH*TAGW-1:0] cmp_tag_m;
    wire [SP*DEPTH*DW-1:0]   st_addr_m, st_data_m;
    wire [SP*DEPTH*3-1:0]    st_f3_m;
    wire [SP*DEPTH*PTRW-1:0] st_id_m;
    wire [SP*DEPTH-1:0]      st_ok_m;

    wire [SP-1:0] st_fire;

    genvar gi, gl, ga, gm, gr, gj, gp, ge, gh;

    // ---------------- allocation ----------------
    wire [CNTW-1:0] free_cnt = DEPTH - count;
    assign ready_o = (free_cnt >= DP) & ~crash_i;

    wire [DP-1:0]     lane_v;
    wire [RKW-1:0]    lane_rank [0:DP-1];
    wire [PTRW-1:0]   lane_slot [0:DP-1];
    wire [DP*EWD-1:0] lane_word;

    generate
        for (gl = 0; gl < DP; gl = gl + 1) begin : g_lane
            assign lane_v[gl] = (dp_ld_valid_i[gl] | dp_st_valid_i[gl]) & ready_o;
            // rank among valid lanes: non-memory lanes are skipped
            if (gl == 0) begin : g_r0
                assign lane_rank[gl] = {RKW{1'b0}};
            end else begin : g_rn
                assign lane_rank[gl] = lane_rank[gl-1] + lane_v[gl-1];
            end
            assign lane_slot[gl] = tail + lane_rank[gl];

            wire is_st = dp_st_valid_i[gl];
            wire [2:0] f3 = dp_op_i[gl*OPW + 7 +: 3];
            wire [IMMW-1:0] imm = dp_imm_i[gl*IMMW +: IMMW];
            wire [DW-1:0] imm_ext = {{(DW-IMMW){imm[IMMW-1]}}, imm};

            // dispatch-time CDB bypass, same as RS
            wire [WP-1:0]    h1, h2;
            wire [WP*DW-1:0] b1, b2;
            for (gm = 0; gm < WP; gm = gm + 1) begin : g_byp
                assign h1[gm] = bus_valid_i[gm] & (bus_tag_i[gm*TAGW +: TAGW] == dp_s1_tag_i[gl*TAGW +: TAGW]);
                assign h2[gm] = bus_valid_i[gm] & (bus_tag_i[gm*TAGW +: TAGW] == dp_s2_tag_i[gl*TAGW +: TAGW]);
                assign b1[gm*DW +: DW] = bus_value_i[gm*DW +: DW] & {DW{h1[gm]}};
                assign b2[gm*DW +: DW] = bus_value_i[gm*DW +: DW] & {DW{h2[gm]}};
            end
            wire any1, any2;
            wire [DW-1:0] byp1, byp2;
            btree_or_bit  #(.N(WP))        u_h1 (.in(h1), .out(any1));
            btree_or_bit  #(.N(WP))        u_h2 (.in(h2), .out(any2));
            btree_or_word #(.N(WP), .W(DW)) u_b1 (.in(b1), .out(byp1));
            btree_or_word #(.N(WP), .W(DW)) u_b2 (.in(b2), .out(byp2));

            wire s1v = dp_s1_val_i[gl] | any1;
            wire s2v = dp_s2_val_i[gl] | any2;
            wire [DW-1:0] s1d = dp_s1_val_i[gl] ? dp_s1_value_i[gl*DW +: DW] : byp1;
            wire [DW-1:0] s2d = dp_s2_val_i[gl] ? dp_s2_value_i[gl*DW +: DW] : byp2;

            // addr = base + offset, computed here when base is already known
            assign lane_word[gl*EWD +: EWD] = {
                dp_dest_tag_i[gl*TAGW +: TAGW], is_st, f3, imm,
                dp_s1_tag_i[gl*TAGW +: TAGW], dp_s2_tag_i[gl*TAGW +: TAGW],
                s1v, (is_st ? s2v : 1'b1), (s1d + imm_ext), s2d
            };
        end
    endgenerate

    wire [RKW-1:0] n_alloc = lane_rank[DP-1] + lane_v[DP-1];

    // ---------------- per-entry logic ----------------
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : g_e
            wire [PTRW-1:0] off_e = gi[PTRW-1:0] - head;

            assign valid_f[gi] = valid_r[gi];

            // alloc capture
            wire [DP-1:0]     amatch;
            wire [DP*EWD-1:0] amasked;
            for (ga = 0; ga < DP; ga = ga + 1) begin : g_am
                assign amatch[ga] = lane_v[ga] & (lane_slot[ga] == gi[PTRW-1:0]);
                assign amasked[ga*EWD +: EWD] = lane_word[ga*EWD +: EWD] & {EWD{amatch[ga]}};
            end
            wire alloc_en;
            wire [EWD-1:0] alloc_word;
            btree_or_bit  #(.N(DP))         u_aen (.in(amatch),  .out(alloc_en));
            btree_or_word #(.N(DP), .W(EWD)) u_aw  (.in(amasked), .out(alloc_word));

            // CDB snoop for base (s1) and store data (s2)
            wire [WP-1:0]    sm1, sm2;
            wire [WP*DW-1:0] sv1, sv2;
            for (gm = 0; gm < WP; gm = gm + 1) begin : g_sn
                assign sm1[gm] = bus_valid_i[gm] & (bus_tag_i[gm*TAGW +: TAGW] == s1t_r[gi]);
                assign sm2[gm] = bus_valid_i[gm] & (bus_tag_i[gm*TAGW +: TAGW] == s2t_r[gi]);
                assign sv1[gm*DW +: DW] = bus_value_i[gm*DW +: DW] & {DW{sm1[gm]}};
                assign sv2[gm*DW +: DW] = bus_value_i[gm*DW +: DW] & {DW{sm2[gm]}};
            end
            wire any1, any2;
            wire [DW-1:0] bval1, bval2;
            btree_or_bit  #(.N(WP))         u_s1a (.in(sm1), .out(any1));
            btree_or_bit  #(.N(WP))         u_s2a (.in(sm2), .out(any2));
            btree_or_word #(.N(WP), .W(DW)) u_s1v (.in(sv1), .out(bval1));
            btree_or_word #(.N(WP), .W(DW)) u_s2v (.in(sv2), .out(bval2));
            wire snoop1 = valid_r[gi] & ~av_r[gi] & any1;
            wire snoop2 = valid_r[gi] & isst_r[gi] & ~dv_r[gi] & any2;
            wire [DW-1:0] off_ext = {{(DW-IMMW){off_r[gi][IMMW-1]}}, off_r[gi]};
            wire [DW-1:0] addr_snoop = bval1 + off_ext;

            // retire: store whose tag == rob_head+k for a committed slot k
            wire [RP-1:0] rm;
            for (gr = 0; gr < RP; gr = gr + 1) begin : g_rt
                assign rm[gr] = ret_vec_i[gr] & (tag_r[gi] == (rob_head_i + gr[TAGW-1:0]));
            end
            wire ret_any;
            btree_or_bit #(.N(RP)) u_rm (.in(rm), .out(ret_any));
            wire ret_hit = valid_r[gi] & isst_r[gi] & ret_any;

            // crash kill: retired stores are exempt (their tags sit below rob_head)
            wire kill = crash_i & valid_r[gi] & ~ret_r[gi] & ~in_circ_range(tag_r[gi], rob_head_i, crash_tail_i);
            assign kill_f[gi] = kill;

            // search older entries, nearest first (constant offsets from this entry)
            wire [DEPTH-2:0] older, st_live, unk, wm, sel, exm, okill;
            wire [DEPTH-1:0] blk;
            wire [(DEPTH-1)*DW-1:0] fmask;
            wire [DEPTH*CNTW-1:0] lcn, ccn, dcn;
            assign blk[0] = 1'b0;
            assign lcn[0 +: CNTW] = {CNTW{1'b0}};
            assign ccn[0 +: CNTW] = {CNTW{1'b0}};
            assign dcn[0 +: CNTW] = {CNTW{1'b0}};
            for (gj = 0; gj < DEPTH-1; gj = gj + 1) begin : g_f
                localparam integer IDX = (gi + DEPTH - 1 - gj) % DEPTH;
                assign older[gj]   = (off_e > gj[PTRW-1:0]);
                assign st_live[gj] = older[gj] & valid_r[IDX] & isst_r[IDX];
                // unknown older store address -> load stalls
                assign unk[gj]     = st_live[gj] & ~av_r[IDX];
                // word-level match; youngest older match wins
                assign wm[gj]      = st_live[gj] & av_r[IDX] & (addr_r[IDX][DW-1:2] == addr_r[gi][DW-1:2]);
                assign blk[gj+1]   = blk[gj] | wm[gj];
                assign sel[gj]     = wm[gj] & ~blk[gj];
                // forward only on exact addr + same size + data ready
                assign exm[gj]     = sel[gj] & (addr_r[IDX] == addr_r[gi]) & (f3_r[IDX][1:0] == f3_r[gi][1:0]) & dv_r[IDX];
                assign fmask[gj*DW +: DW] = data_r[IDX] & {DW{sel[gj]}};
                // age ranks among candidates (oldest = rank 0)
                assign lcn[(gj+1)*CNTW +: CNTW] = lcn[gj*CNTW +: CNTW] + (older[gj] & ld_cand_f[IDX]);
                assign ccn[(gj+1)*CNTW +: CNTW] = ccn[gj*CNTW +: CNTW] + (older[gj] & cmp_cand_f[IDX]);
                assign dcn[(gj+1)*CNTW +: CNTW] = dcn[gj*CNTW +: CNTW] + (older[gj] & pend_f[IDX]);
                assign okill[gj]   = older[gj] & kill_f[IDX];
            end
            wire [CNTW-1:0] ld_rank  = lcn[(DEPTH-1)*CNTW +: CNTW];
            wire [CNTW-1:0] cmp_rank = ccn[(DEPTH-1)*CNTW +: CNTW];
            wire [CNTW-1:0] dr_rank  = dcn[(DEPTH-1)*CNTW +: CNTW];

            wire unk_any, fwd_ok, older_kill;
            wire [DW-1:0] fdata;
            btree_or_bit  #(.N(DEPTH-1))         u_unk (.in(unk),   .out(unk_any));
            btree_or_bit  #(.N(DEPTH-1))         u_fok (.in(exm),   .out(fwd_ok));
            btree_or_bit  #(.N(DEPTH-1))         u_okl (.in(okill), .out(older_kill));
            btree_or_word #(.N(DEPTH-1), .W(DW)) u_fd  (.in(fmask), .out(fdata));
            wire matched = blk[DEPTH-1];

            // first (oldest) killed entry -> new tail
            assign first_f[gi] = kill & ~older_kill;
            assign first_off_m[gi*PTRW +: PTRW] = off_e & {PTRW{first_f[gi]}};

            // load candidate: addr known, no unknown older store, no match or forwardable match
            assign ld_cand_f[gi] = valid_r[gi] & ~isst_r[gi] & av_r[gi] & ~unk_any & (~matched | fwd_ok);
            assign ld_fwd_f[gi]  = matched;
            assign fdata_f[gi*DW +: DW] = fdata;

            // store completion candidate (to ROB)
            assign cmp_cand_f[gi] = valid_r[gi] & isst_r[gi] & av_r[gi] & dv_r[gi] & ~cdone_r[gi];
            // retired, not yet handed over
            assign pend_f[gi]     = valid_r[gi] & isst_r[gi] & ret_r[gi] & ~sent_r[gi];
            assign infl_f[gi]     = valid_r[gi] & isst_r[gi] & sent_r[gi];

            // load port select
            wire [LP-1:0] lacc;
            for (gp = 0; gp < LP; gp = gp + 1) begin : g_lp
                wire s = ld_cand_f[gi] & (ld_rank == gp[CNTW-1:0]);
                assign ld_sel_f[gi*LP + gp] = s;
                assign ld_tag_m [(gp*DEPTH+gi)*TAGW +: TAGW] = tag_r[gi] & {TAGW{s}};
                assign ld_f3_m  [(gp*DEPTH+gi)*3 +: 3]       = f3_r[gi] & {3{s}};
                assign ld_addr_m[(gp*DEPTH+gi)*DW +: DW]     = addr_r[gi] & {DW{s}};
                assign ld_fd_m  [(gp*DEPTH+gi)*DW +: DW]     = fdata & {DW{s}};
                assign ld_fwd_m [gp*DEPTH+gi]                = matched & s;
                assign lacc[gp] = s & ld_ready_i[gp];
            end
            wire ld_any;
            btree_or_bit #(.N(LP)) u_la (.in(lacc), .out(ld_any));
            assign ld_acc_f[gi] = ld_any & ~crash_i;

            // completion port select
            wire [CP-1:0] cacc;
            for (gp = 0; gp < CP; gp = gp + 1) begin : g_cp
                wire s = cmp_cand_f[gi] & (cmp_rank == gp[CNTW-1:0]);
                assign cmp_sel_f[gi*CP + gp] = s;
                assign cmp_tag_m[(gp*DEPTH+gi)*TAGW +: TAGW] = tag_r[gi] & {TAGW{s}};
                assign cacc[gp] = s & cmp_ready_i[gp];
            end
            wire cmp_any;
            btree_or_bit #(.N(CP)) u_ca (.in(cacc), .out(cmp_any));
            assign cmp_acc_f[gi] = cmp_any & ~crash_i;

            // store drain port select
            wire [SP-1:0] sset;
            for (gp = 0; gp < SP; gp = gp + 1) begin : g_sp
                wire s = pend_f[gi] & (dr_rank == gp[CNTW-1:0]);
                assign dsel_f[gi*SP + gp] = s;
                assign st_addr_m[(gp*DEPTH+gi)*DW +: DW]     = addr_r[gi] & {DW{s}};
                assign st_data_m[(gp*DEPTH+gi)*DW +: DW]     = data_r[gi] & {DW{s}};
                assign st_f3_m  [(gp*DEPTH+gi)*3 +: 3]       = f3_r[gi] & {3{s}};
                assign st_id_m  [(gp*DEPTH+gi)*PTRW +: PTRW] = gi[PTRW-1:0] & {PTRW{s}};
                assign st_ok_m  [gp*DEPTH+gi]                = s & av_r[gi] & dv_r[gi];
                assign sset[gp] = s & st_fire[gp];
            end
            btree_or_bit #(.N(SP)) u_ss (.in(sset), .out(sent_set_f[gi]));

            // store done from store module (by slot id)
            wire [SP-1:0] dm;
            for (gp = 0; gp < SP; gp = gp + 1) begin : g_dn
                assign dm[gp] = st_done_valid_i[gp] & (st_done_id_i[gp*PTRW +: PTRW] == gi[PTRW-1:0]);
            end
            wire done_hit;
            btree_or_bit #(.N(SP)) u_dn (.in(dm), .out(done_hit));

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    valid_r[gi] <= 1'b0;
                    ret_r[gi]   <= 1'b0;
                    sent_r[gi]  <= 1'b0;
                    cdone_r[gi] <= 1'b0;
                    av_r[gi]    <= 1'b0;
                    dv_r[gi]    <= 1'b0;
                end else begin
                    if (kill)                          valid_r[gi] <= 1'b0;
                    else if (alloc_en)                 valid_r[gi] <= 1'b1;
                    else if (ld_acc_f[gi] | done_hit)  valid_r[gi] <= 1'b0;

                    if (alloc_en) begin
                        tag_r[gi]   <= alloc_word[L_TAG +: TAGW];
                        isst_r[gi]  <= alloc_word[L_ST];
                        f3_r[gi]    <= alloc_word[L_F3 +: 3];
                        off_r[gi]   <= alloc_word[L_OFF +: IMMW];
                        s1t_r[gi]   <= alloc_word[L_S1T +: TAGW];
                        s2t_r[gi]   <= alloc_word[L_S2T +: TAGW];
                        av_r[gi]    <= alloc_word[L_AV];
                        dv_r[gi]    <= alloc_word[L_DV];
                        addr_r[gi]  <= alloc_word[L_ADDR +: DW];
                        data_r[gi]  <= alloc_word[L_DATA +: DW];
                        ret_r[gi]   <= 1'b0;
                        sent_r[gi]  <= 1'b0;
                        cdone_r[gi] <= 1'b0;
                    end else begin
                        if (snoop1) begin
                            av_r[gi]   <= 1'b1;
                            addr_r[gi] <= addr_snoop;
                        end
                        if (snoop2) begin
                            dv_r[gi]   <= 1'b1;
                            data_r[gi] <= bval2;
                        end
                        if (ret_hit)          ret_r[gi]   <= 1'b1;
                        if (sent_set_f[gi])   sent_r[gi]  <= 1'b1;
                        if (cmp_acc_f[gi])    cdone_r[gi] <= 1'b1;
                    end
                end
            end
        end
    endgenerate

    // ---------------- load handoff ports ----------------
    generate
        for (gp = 0; gp < LP; gp = gp + 1) begin : g_lport
            wire [DEPTH-1:0] col;
            for (ge = 0; ge < DEPTH; ge = ge + 1) begin : g_c
                assign col[ge] = ld_sel_f[ge*LP + gp];
            end
            wire v, fw;
            btree_or_bit  #(.N(DEPTH)) u_v  (.in(col), .out(v));
            btree_or_bit  #(.N(DEPTH)) u_fw (.in(ld_fwd_m[gp*DEPTH +: DEPTH]), .out(fw));
            btree_or_word #(.N(DEPTH), .W(TAGW)) u_t (.in(ld_tag_m[gp*DEPTH*TAGW +: DEPTH*TAGW]), .out(ld_tag_o[gp*TAGW +: TAGW]));
            btree_or_word #(.N(DEPTH), .W(3))    u_f (.in(ld_f3_m[gp*DEPTH*3 +: DEPTH*3]),        .out(ld_funct3_o[gp*3 +: 3]));
            btree_or_word #(.N(DEPTH), .W(DW))   u_a (.in(ld_addr_m[gp*DEPTH*DW +: DEPTH*DW]),    .out(ld_addr_o[gp*DW +: DW]));
            btree_or_word #(.N(DEPTH), .W(DW))   u_d (.in(ld_fd_m[gp*DEPTH*DW +: DEPTH*DW]),      .out(ld_fdata_o[gp*DW +: DW]));
            assign ld_valid_o[gp] = v & ~crash_i;
            assign ld_fwd_o[gp]   = fw;
        end
    endgenerate

    // ---------------- store completion ports ----------------
    generate
        for (gp = 0; gp < CP; gp = gp + 1) begin : g_cport
            wire [DEPTH-1:0] col;
            for (ge = 0; ge < DEPTH; ge = ge + 1) begin : g_c
                assign col[ge] = cmp_sel_f[ge*CP + gp];
            end
            wire v;
            btree_or_bit  #(.N(DEPTH)) u_v (.in(col), .out(v));
            btree_or_word #(.N(DEPTH), .W(TAGW)) u_t (.in(cmp_tag_m[gp*DEPTH*TAGW +: DEPTH*TAGW]), .out(cmp_tag_o[gp*TAGW +: TAGW]));
            assign cmp_valid_o[gp] = v & ~crash_i;
        end
    endgenerate

    // ---------------- store drain ports ----------------
    // Only when no store is in flight; port j (older) -> arbiter write port j, so younger wins collisions.
    wire infl_any;
    btree_or_bit #(.N(DEPTH)) u_infl (.in(infl_f), .out(infl_any));
    wire st_idle = ~infl_any;

    wire [SP-1:0]  st_ok;
    wire [SP-1:0]  st_allow, st_therm, st_pre;
    wire [SPW-1:0] st_tcnt;
    wire [SP:0]    rch;
    assign rch[0] = 1'b1;

    generate
        for (gp = 0; gp < SP; gp = gp + 1) begin : g_sport
            btree_or_bit  #(.N(DEPTH)) u_ok (.in(st_ok_m[gp*DEPTH +: DEPTH]), .out(st_ok[gp]));
            btree_or_word #(.N(DEPTH), .W(DW))   u_a (.in(st_addr_m[gp*DEPTH*DW +: DEPTH*DW]),     .out(st_addr_o[gp*DW +: DW]));
            btree_or_word #(.N(DEPTH), .W(DW))   u_d (.in(st_data_m[gp*DEPTH*DW +: DEPTH*DW]),     .out(st_data_o[gp*DW +: DW]));
            btree_or_word #(.N(DEPTH), .W(3))    u_f (.in(st_f3_m[gp*DEPTH*3 +: DEPTH*3]),         .out(st_funct3_o[gp*3 +: 3]));
            btree_or_word #(.N(DEPTH), .W(PTRW)) u_i (.in(st_id_m[gp*DEPTH*PTRW +: DEPTH*PTRW]),   .out(st_id_o[gp*PTRW +: PTRW]));

            // antitherm over ports: port j allowed only if ports 0..j all have a drainable store
            assign st_allow[SP-1-gp] = st_ok[gp];
            assign st_pre[gp]        = st_therm[SP-1-gp];
            // higher port only fires if every lower port is ready too (keeps order)
            assign rch[gp+1]         = rch[gp] & st_ready_i[gp];
            assign st_valid_o[gp]    = st_pre[gp] & rch[gp] & st_idle & ~crash_i;
            assign st_fire[gp]       = st_valid_o[gp] & st_ready_i[gp];
        end
    endgenerate

    antitherm #(.PN(SP), .CW(SPW)) u_sttherm (
        .allow (st_allow),
        .block ({SP{1'b0}}),
        .out   (st_therm),
        .count (st_tcnt)
    );

    // ---------------- head advance over freed entries ----------------
    wire [HA-1:0]  ha_allow, ha_out;
    wire [HAW-1:0] adv;
    generate
        for (gh = 0; gh < HA; gh = gh + 1) begin : g_ha
            wire [PTRW-1:0] haddr = head + gh[PTRW-1:0];
            wire vb;
            btree_mux_word #(.N(DEPTH), .AW(PTRW), .W(1)) u_hv (.in(valid_f), .addr(haddr), .out(vb));
            assign ha_allow[HA-1-gh] = ~vb & (count > gh[CNTW-1:0]);
        end
    endgenerate

    antitherm #(.PN(HA), .CW(HAW)) u_ha (
        .allow (ha_allow),
        .block ({HA{1'b0}}),
        .out   (ha_out),
        .count (adv)
    );

    // ---------------- crash: rewind tail to first killed entry ----------------
    wire first_any;
    wire [PTRW-1:0] first_off;
    btree_or_bit  #(.N(DEPTH))           u_fany (.in(first_f),     .out(first_any));
    btree_or_word #(.N(DEPTH), .W(PTRW)) u_foff (.in(first_off_m), .out(first_off));
    wire [CNTW-1:0] new_count = first_any ? {1'b0, first_off} : count;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            head  <= {PTRW{1'b0}};
            tail  <= {PTRW{1'b0}};
            count <= {CNTW{1'b0}};
        end else if (crash_i) begin
            tail  <= head + new_count[PTRW-1:0];
            count <= new_count;
        end else begin
            head  <= head + adv;
            tail  <= tail + n_alloc;
            count <= count + n_alloc - adv;
        end
    end

endmodule