module rs_module #(
    parameter ROWS   = 4,
    parameter WP     = 2,
    parameter DP     = 2,
    parameter BUSP   = 2,
    parameter OPB    = 5,
    parameter TAGW   = 6,
    parameter DATAW  = 32,
    parameter AGEW   = 4,
    parameter EXPL   = 4,
    parameter IMMS   = 21,
    parameter PCW    = 32
)(
    input clk,
    input rst,

    input  [DP-1:0]        i_valid,
    output                 o_ready,
    input  [DP*OPB-1:0]    i_opcode,
    input  [DP*TAGW-1:0]   i_src1_tag,
    input  [DP-1:0]        i_src1_val,
    input  [DP*DATAW-1:0]  i_src1_data,
    input  [DP*TAGW-1:0]   i_src2_tag,
    input  [DP-1:0]        i_src2_val,
    input  [DP*DATAW-1:0]  i_src2_data,
    input  [DP*TAGW-1:0]   i_dest_tag,
    input  [DP*IMMS-1:0]   i_imm,
    input  [DP*PCW-1:0]    i_pc,

    input  [BUSP*TAGW-1:0]  bus_tag,
    input  [BUSP*DATAW-1:0] bus_value,
    input  [BUSP*EXPL-1:0]  bus_exception, // unused, kept for bus-interface uniformity
    input  [BUSP-1:0]       bus_valid,

    output [WP-1:0]        o_valid,
    output [WP*DATAW-1:0]  o_src1_data,
    output [WP*DATAW-1:0]  o_src2_data,
    output [WP*TAGW-1:0]   o_dest_tag,
    output [WP*OPB-1:0]    o_opcode,
    output [WP*IMMS-1:0]   o_imm,
    output [WP*PCW-1:0]    o_pc,
    input  [WP-1:0]        i_ready,

    input                  crash_valid,
    input  [TAGW-1:0]      crash_head,
    input  [TAGW-1:0]      crash_tail
);

    localparam N = ROWS*WP;
    localparam CW = $clog2(N+1);

    reg               active   [0:N-1];
    reg  [OPB-1:0]    opcode   [0:N-1];
    reg  [TAGW-1:0]   s1_tag   [0:N-1];
    reg               s1_val   [0:N-1];
    reg  [DATAW-1:0]  s1_data  [0:N-1];
    reg  [TAGW-1:0]   s2_tag   [0:N-1];
    reg               s2_val   [0:N-1];
    reg  [DATAW-1:0]  s2_data  [0:N-1];
    reg  [TAGW-1:0]   dest_tag [0:N-1];
    reg  [IMMS-1:0]   imm      [0:N-1];
    reg  [PCW-1:0]    pc       [0:N-1];
    reg  [AGEW-1:0]   age      [0:N-1];

    function in_circ_range;
        input [TAGW-1:0] idx, head, tail;
        begin
            if (head <= tail)
                in_circ_range = (idx >= head) && (idx < tail);
            else
                in_circ_range = (idx >= head) || (idx < tail);
        end
    endfunction

    wire [N-1:0] free_mask;
    genvar gi;
    generate
        for (gi = 0; gi < N; gi = gi + 1) begin : g_free
            assign free_mask[gi] = ~active[gi];
        end
    endgenerate

    integer fi;
    reg [CW-1:0] free_count;
    always @(*) begin
        free_count = {CW{1'b0}};
        for (fi = 0; fi < N; fi = fi + 1)
            free_count = free_count + free_mask[fi];
    end
    assign o_ready = (free_count >= DP) && !crash_valid;

    integer dk;
    reg [CW-1:0] dp_count;
    always @(*) begin
        dp_count = {CW{1'b0}};
        for (dk = 0; dk < DP; dk = dk + 1)
            dp_count = dp_count + i_valid[dk];
    end

    reg [N-1:0]  alloc_onehot [0:DP-1];
    reg          alloc_valid  [0:DP-1];
    integer ak, ai;
    reg [N-1:0] used_mask;
    always @(*) begin
        used_mask = {N{1'b0}};
        for (ak = 0; ak < DP; ak = ak + 1) begin
            alloc_onehot[ak] = {N{1'b0}};
            alloc_valid[ak]  = 1'b0;
            if (i_valid[ak] && !crash_valid) begin
                for (ai = 0; ai < N; ai = ai + 1) begin
                    if (!alloc_valid[ak] && free_mask[ai] && !used_mask[ai]) begin
                        alloc_onehot[ak][ai] = 1'b1;
                        alloc_valid[ak]      = 1'b1;
                    end
                end
                used_mask = used_mask | alloc_onehot[ak];
            end
        end
    end

    reg         match_s1 [0:N-1];
    reg [DATAW-1:0] match_s1_val [0:N-1];
    reg         match_s2 [0:N-1];
    reg [DATAW-1:0] match_s2_val [0:N-1];
    integer mi, mj;
    always @(*) begin
        for (mi = 0; mi < N; mi = mi + 1) begin
            match_s1[mi]     = 1'b0;
            match_s1_val[mi] = {DATAW{1'b0}};
            match_s2[mi]     = 1'b0;
            match_s2_val[mi] = {DATAW{1'b0}};
            for (mj = 0; mj < BUSP; mj = mj + 1) begin
                if (bus_valid[mj] && !match_s1[mi] && (bus_tag[mj*TAGW +: TAGW] == s1_tag[mi])) begin
                    match_s1[mi]     = 1'b1;
                    match_s1_val[mi] = bus_value[mj*DATAW +: DATAW];
                end
                if (bus_valid[mj] && !match_s2[mi] && (bus_tag[mj*TAGW +: TAGW] == s2_tag[mi])) begin
                    match_s2[mi]     = 1'b1;
                    match_s2_val[mi] = bus_value[mj*DATAW +: DATAW];
                end
            end
        end
    end

    reg         dp_match_s1 [0:DP-1];
    reg [DATAW-1:0] dp_match_s1_val [0:DP-1];
    reg         dp_match_s2 [0:DP-1];
    reg [DATAW-1:0] dp_match_s2_val [0:DP-1];
    integer dmi, dmj;
    always @(*) begin
        for (dmi = 0; dmi < DP; dmi = dmi + 1) begin
            dp_match_s1[dmi]     = 1'b0;
            dp_match_s1_val[dmi] = {DATAW{1'b0}};
            dp_match_s2[dmi]     = 1'b0;
            dp_match_s2_val[dmi] = {DATAW{1'b0}};
            for (dmj = 0; dmj < BUSP; dmj = dmj + 1) begin
                if (bus_valid[dmj] && !dp_match_s1[dmi] && (bus_tag[dmj*TAGW +: TAGW] == i_src1_tag[dmi*TAGW +: TAGW])) begin
                    dp_match_s1[dmi]     = 1'b1;
                    dp_match_s1_val[dmi] = bus_value[dmj*DATAW +: DATAW];
                end
                if (bus_valid[dmj] && !dp_match_s2[dmi] && (bus_tag[dmj*TAGW +: TAGW] == i_src2_tag[dmi*TAGW +: TAGW])) begin
                    dp_match_s2[dmi]     = 1'b1;
                    dp_match_s2_val[dmi] = bus_value[dmj*DATAW +: DATAW];
                end
            end
        end
    end

    reg crash_kill [0:N-1];
    integer ci;
    always @(*) begin
        for (ci = 0; ci < N; ci = ci + 1)
            crash_kill[ci] = crash_valid && active[ci] && !in_circ_range(dest_tag[ci], crash_head, crash_tail);
    end

    reg          ready_entry [0:N-1];
    integer ri;
    always @(*) begin
        for (ri = 0; ri < N; ri = ri + 1)
            ready_entry[ri] = active[ri] && s1_val[ri] && s2_val[ri];
    end

    reg [N-1:0] issue_dealloc;
    reg [WP-1:0]        o_valid_r;
    reg [DATAW-1:0]     o_src1_data_r [0:WP-1];
    reg [DATAW-1:0]     o_src2_data_r [0:WP-1];
    reg [TAGW-1:0]      o_dest_tag_r  [0:WP-1];
    reg [OPB-1:0]       o_opcode_r    [0:WP-1];
    reg [IMMS-1:0]      o_imm_r       [0:WP-1];
    reg [PCW-1:0]       o_pc_r        [0:WP-1];

    integer p, r, best_idx, slot_idx;
    reg found_best;
    always @(*) begin
        issue_dealloc = {N{1'b0}};
        for (p = 0; p < WP; p = p + 1) begin
            best_idx   = 0;
            found_best = 1'b0;
            for (r = 0; r < ROWS; r = r + 1) begin
                slot_idx = r*WP + p;
                if (ready_entry[slot_idx]) begin
                    if (!found_best || (age[slot_idx] > age[best_idx])) begin
                        best_idx   = slot_idx;
                        found_best = 1'b1;
                    end
                end
            end
            o_valid_r[p]         = found_best;
            o_src1_data_r[p]     = s1_data[best_idx];
            o_src2_data_r[p]     = s2_data[best_idx];
            o_dest_tag_r[p]      = dest_tag[best_idx];
            o_opcode_r[p]        = opcode[best_idx];
            o_imm_r[p]           = imm[best_idx];
            o_pc_r[p]            = pc[best_idx];
            if (found_best && i_ready[p])
                issue_dealloc[best_idx] = 1'b1;
        end
    end

    generate
        for (gi = 0; gi < WP; gi = gi + 1) begin : g_out
            assign o_valid[gi]                   = o_valid_r[gi];
            assign o_src1_data[gi*DATAW +: DATAW] = o_src1_data_r[gi];
            assign o_src2_data[gi*DATAW +: DATAW] = o_src2_data_r[gi];
            assign o_dest_tag[gi*TAGW +: TAGW]    = o_dest_tag_r[gi];
            assign o_opcode[gi*OPB +: OPB]        = o_opcode_r[gi];
            assign o_imm[gi*IMMS +: IMMS]         = o_imm_r[gi];
            assign o_pc[gi*PCW +: PCW]            = o_pc_r[gi];
        end
    endgenerate

    integer si, sk, sj;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (si = 0; si < N; si = si + 1) begin
                active[si] <= 1'b0;
                s1_val[si] <= 1'b0;
                s2_val[si] <= 1'b0;
                age[si]    <= {AGEW{1'b0}};
            end
        end else begin
            for (si = 0; si < N; si = si + 1) begin
                if (issue_dealloc[si] || crash_kill[si]) begin
                    active[si] <= 1'b0;
                end else if (active[si]) begin
                    if (!s1_val[si] && match_s1[si]) begin
                        s1_val[si]  <= 1'b1;
                        s1_data[si] <= match_s1_val[si];
                    end
                    if (!s2_val[si] && match_s2[si]) begin
                        s2_val[si]  <= 1'b1;
                        s2_data[si] <= match_s2_val[si];
                    end
                    age[si] <= age[si] + dp_count;
                end
            end

            for (sk = 0; sk < DP; sk = sk + 1) begin
                if (alloc_valid[sk]) begin
                    for (sj = 0; sj < N; sj = sj + 1) begin
                        if (alloc_onehot[sk][sj]) begin
                            active[sj]   <= 1'b1;
                            opcode[sj]   <= i_opcode[sk*OPB +: OPB];
                            s1_tag[sj]   <= i_src1_tag[sk*TAGW +: TAGW];
                            s2_tag[sj]   <= i_src2_tag[sk*TAGW +: TAGW];
                            dest_tag[sj] <= i_dest_tag[sk*TAGW +: TAGW];
                            imm[sj]      <= i_imm[sk*IMMS +: IMMS];
                            pc[sj]       <= i_pc[sk*PCW +: PCW];
                            age[sj]      <= sk[AGEW-1:0];

                            if (i_src1_val[sk]) begin
                                s1_val[sj]  <= 1'b1;
                                s1_data[sj] <= i_src1_data[sk*DATAW +: DATAW];
                            end else if (dp_match_s1[sk]) begin
                                s1_val[sj]  <= 1'b1;
                                s1_data[sj] <= dp_match_s1_val[sk];
                            end else begin
                                s1_val[sj] <= 1'b0;
                            end

                            if (i_src2_val[sk]) begin
                                s2_val[sj]  <= 1'b1;
                                s2_data[sj] <= i_src2_data[sk*DATAW +: DATAW];
                            end else if (dp_match_s2[sk]) begin
                                s2_val[sj]  <= 1'b1;
                                s2_data[sj] <= dp_match_s2_val[sk];
                            end else begin
                                s2_val[sj] <= 1'b0;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule