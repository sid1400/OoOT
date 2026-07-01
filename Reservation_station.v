module rs_module #(
    parameter DP = 4,
    parameter OP = 3,
    parameter SN = 2,
    parameter TW = 6,
    parameter DW = 32,
    parameter OPB = 5,
    parameter BC = 2,
    parameter ROB_SIZE = 64
) (
    input  logic clk,
    input  logic rst_n,

    input  logic                  dp_valid     [DP],
    input  logic [TW-1:0]         dp_src1_tag  [DP],
    input  logic                  dp_src1_val  [DP],
    input  logic [DW-1:0]         dp_src1_value[DP],
    input  logic [TW-1:0]         dp_src2_tag  [DP],
    input  logic                  dp_src2_val  [DP],
    input  logic [DW-1:0]         dp_src2_value[DP],
    input  logic [$clog2(ROB_SIZE)-1:0] dp_dest_tag[DP],
    input  logic [OPB-1:0]        dp_opcode    [DP],
    output logic                  ready_out,

    input  logic                  bc_valid [BC],
    input  logic [TW-1:0]         bc_tag   [BC],
    input  logic [DW-1:0]         bc_value [BC],

    output logic                  op_valid     [OP],
    output logic [DW-1:0]         op_src1_value[OP],
    output logic [DW-1:0]         op_src2_value[OP],
    output logic [$clog2(ROB_SIZE)-1:0] op_dest_tag[OP],
    output logic [OPB-1:0]        op_opcode    [OP],
    input  logic                  op_ready     [OP],

    input  logic [$clog2(ROB_SIZE)-1:0] rob_head,
    input  logic [$clog2(ROB_SIZE)-1:0] rob_tail
);

    localparam N    = SN * OP;
    localparam ROBW = $clog2(ROB_SIZE);
    localparam AGEW = (N > 1) ? $clog2(N) : 1;

    logic                  active   [N];
    logic [TW-1:0]         s1_tag   [N];
    logic                  s1_val   [N];
    logic [DW-1:0]         s1_value [N];
    logic [TW-1:0]         s2_tag   [N];
    logic                  s2_val   [N];
    logic [DW-1:0]         s2_value [N];
    logic [ROBW-1:0]       dest_tag [N];
    logic [OPB-1:0]        opcode   [N];
    logic [AGEW-1:0]       age      [N];

    logic [N-1:0] free_mask;
    logic [N-1:0] used_mask;
    logic [$clog2(N+1)-1:0] free_count;
    logic [$clog2(N+1)-1:0] dp_count;

    logic [N-1:0] alloc_onehot [DP];
    logic         alloc_valid  [DP];

    logic [N-1:0] in_range_inv;

    integer i, j, k;

    always_comb begin
        for (i = 0; i < N; i++) free_mask[i] = ~active[i];
    end

    always_comb begin
        free_count = '0;
        for (i = 0; i < N; i++) free_count += free_mask[i];
    end

    always_comb begin
        dp_count = '0;
        for (k = 0; k < DP; k++) dp_count += dp_valid[k];
    end

    assign ready_out = (free_count >= DP);

    always_comb begin
        used_mask = '0;
        for (k = 0; k < DP; k++) begin
            alloc_onehot[k] = '0;
            alloc_valid[k]  = 1'b0;
            if (dp_valid[k]) begin
                for (i = 0; i < N; i++) begin
                    if (!alloc_valid[k] && free_mask[i] && !used_mask[i]) begin
                        alloc_onehot[k][i] = 1'b1;
                        alloc_valid[k]     = 1'b1;
                    end
                end
                used_mask |= alloc_onehot[k];
            end
        end
    end

    function automatic logic in_circ_range(logic [ROBW-1:0] idx,
                                            logic [ROBW-1:0] head,
                                            logic [ROBW-1:0] tail);
        if (head <= tail)
            return (idx >= head) && (idx < tail);
        else
            return (idx >= head) || (idx < tail);
    endfunction

    always_comb begin
        for (i = 0; i < N; i++)
            in_range_inv[i] = active[i] && !in_circ_range(dest_tag[i], rob_head, rob_tail);
    end

    logic match_s1 [N];
    logic match_s2 [N];
    logic [DW-1:0] match_s1_value [N];
    logic [DW-1:0] match_s2_value [N];

    always_comb begin
        for (i = 0; i < N; i++) begin
            match_s1[i]       = 1'b0;
            match_s1_value[i] = '0;
            match_s2[i]       = 1'b0;
            match_s2_value[i] = '0;
            for (j = 0; j < BC; j++) begin
                if (bc_valid[j] && (bc_tag[j] == s1_tag[i]) && !match_s1[i]) begin
                    match_s1[i]       = 1'b1;
                    match_s1_value[i] = bc_value[j];
                end
                if (bc_valid[j] && (bc_tag[j] == s2_tag[i]) && !match_s2[i]) begin
                    match_s2[i]       = 1'b1;
                    match_s2_value[i] = bc_value[j];
                end
            end
        end
    end

    logic dp_match_s1 [DP];
    logic dp_match_s2 [DP];
    logic [DW-1:0] dp_match_s1_value [DP];
    logic [DW-1:0] dp_match_s2_value [DP];

    always_comb begin
        for (k = 0; k < DP; k++) begin
            dp_match_s1[k]       = 1'b0;
            dp_match_s1_value[k] = '0;
            dp_match_s2[k]       = 1'b0;
            dp_match_s2_value[k] = '0;
            for (j = 0; j < BC; j++) begin
                if (bc_valid[j] && (bc_tag[j] == dp_src1_tag[k]) && !dp_match_s1[k]) begin
                    dp_match_s1[k]       = 1'b1;
                    dp_match_s1_value[k] = bc_value[j];
                end
                if (bc_valid[j] && (bc_tag[j] == dp_src2_tag[k]) && !dp_match_s2[k]) begin
                    dp_match_s2[k]       = 1'b1;
                    dp_match_s2_value[k] = bc_value[j];
                end
            end
        end
    end

    logic ready_entry [N];
    always_comb begin
        for (i = 0; i < N; i++)
            ready_entry[i] = active[i] && s1_val[i] && s2_val[i];
    end

    integer slot_idx;
    integer best_idx;
    logic   found_best;

    always_comb begin
        for (int p = 0; p < OP; p++) begin
            best_idx   = 0;
            found_best = 1'b0;
            for (int s = 0; s < SN; s++) begin
                slot_idx = s * OP + p;
                if (ready_entry[slot_idx]) begin
                    if (!found_best || (age[slot_idx] > age[best_idx])) begin
                        best_idx   = slot_idx;
                        found_best = 1'b1;
                    end
                end
            end
            op_valid[p]      = found_best;
            op_src1_value[p] = s1_value[best_idx];
            op_src2_value[p] = s2_value[best_idx];
            op_dest_tag[p]   = dest_tag[best_idx];
            op_opcode[p]     = opcode[best_idx];
        end
    end

    logic [N-1:0] op_dealloc;
    always_comb begin
        op_dealloc = '0;
        for (int p = 0; p < OP; p++) begin
            best_idx   = 0;
            found_best = 1'b0;
            for (int s = 0; s < SN; s++) begin
                slot_idx = s * OP + p;
                if (ready_entry[slot_idx]) begin
                    if (!found_best || (age[slot_idx] > age[best_idx])) begin
                        best_idx   = slot_idx;
                        found_best = 1'b1;
                    end
                end
            end
            if (found_best && op_ready[p])
                op_dealloc[best_idx] = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N; i++) begin
                active[i] <= 1'b0;
                s1_val[i] <= 1'b0;
                s2_val[i] <= 1'b0;
                age[i]    <= '0;
            end
        end else begin
            for (i = 0; i < N; i++) begin
                if (op_dealloc[i] || in_range_inv[i]) begin
                    active[i] <= 1'b0;
                end else if (active[i]) begin
                    if (!s1_val[i] && match_s1[i]) begin
                        s1_val[i]   <= 1'b1;
                        s1_value[i] <= match_s1_value[i];
                    end
                    if (!s2_val[i] && match_s2[i]) begin
                        s2_val[i]   <= 1'b1;
                        s2_value[i] <= match_s2_value[i];
                    end
                    age[i] <= age[i] + dp_count;
                end
            end

            for (k = 0; k < DP; k++) begin
                if (alloc_valid[k]) begin
                    for (i = 0; i < N; i++) begin
                        if (alloc_onehot[k][i]) begin
                            active[i]   <= 1'b1;
                            s1_tag[i]   <= dp_src1_tag[k];
                            s2_tag[i]   <= dp_src2_tag[k];
                            dest_tag[i] <= dp_dest_tag[k];
                            opcode[i]   <= dp_opcode[k];
                            age[i]      <= k[AGEW-1:0];

                            if (dp_src1_val[k]) begin
                                s1_val[i]   <= 1'b1;
                                s1_value[i] <= dp_src1_value[k];
                            end else if (dp_match_s1[k]) begin
                                s1_val[i]   <= 1'b1;
                                s1_value[i] <= dp_match_s1_value[k];
                            end else begin
                                s1_val[i] <= 1'b0;
                            end

                            if (dp_src2_val[k]) begin
                                s2_val[i]   <= 1'b1;
                                s2_value[i] <= dp_src2_value[k];
                            end else if (dp_match_s2[k]) begin
                                s2_val[i]   <= 1'b1;
                                s2_value[i] <= dp_match_s2_value[k];
                            end else begin
                                s2_val[i] <= 1'b0;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule