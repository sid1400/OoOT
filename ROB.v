module rob #(
    parameter S    = 16,
    parameter EXPL = 4,
    parameter IDXL = 6,
    parameter RP   = 2,
    parameter WP   = 2,
    parameter IP   = 2,
    // dont change these
    parameter PTRW = $clog2(S);
) (
    input clk,
    input rst,

    input commit_en,

    output full,
    output [PTRW-1:0] head_ptr,
    output [PTRW-1:0] tail_ptr,

    input  [PTRW*RP-1:0] r_addr,
    output [32*RP-1:0]   r_value,
    output [IDXL*RP-1:0] r_index,
    output [EXPL*RP-1:0] r_exception,
    output [RP-1:0]      r_valid,

    input [PTRW*WP-1:0] w_addr,
    input [32*WP-1:0]   w_value,
    input [EXPL*WP-1:0] w_exception,
    input [WP-1:0]      w_en,

    input [IP-1:0]       i_en,
    input [IDXL*IP-1:0]  i_index,
    input [IP*IP-1:0]    i_prio
);

    //localparam PTRW = $clog2(S);
    localparam RKW  = (IP <= 1) ? 1 : $clog2(IP);

    reg [31:0]     value     [0:S-1];
    reg [IDXL-1:0] index     [0:S-1];
    reg [EXPL-1:0] exception [0:S-1];
    reg            valid     [0:S-1];

    reg [PTRW-1:0]   head;
    reg [PTRW-1:0]   tail;
    reg [PTRW:0]     count;

    integer ii;

    initial begin
        for (ii = 0; ii < S; ii = ii + 1) begin
            value[ii]     = 32'd0;
            index[ii]     = {IDXL{1'b0}};
            exception[ii] = {EXPL{1'b0}};
            valid[ii]     = 1'b0;
        end
        head  = {PTRW{1'b0}};
        tail  = {PTRW{1'b0}};
        count = {(PTRW+1){1'b0}};
    end

    assign head_ptr = head;
    assign tail_ptr = tail;
    assign full     = (count == S);

    integer pj;
    reg [PTRW:0] n_push;
    always @* begin
        n_push = {(PTRW+1){1'b0}};
        for (pj = 0; pj < IP; pj = pj + 1)
            n_push = n_push + i_en[pj];
    end

    wire [PTRW:0] tail_sum = tail + n_push;
    wire [PTRW-1:0] tail_next = (tail_sum >= S) ? (tail_sum - S) : tail_sum[PTRW-1:0];
    wire [PTRW-1:0] head_next = (head == S-1) ? {PTRW{1'b0}} : head + 1'b1;
    wire [PTRW:0]   count_next = count + n_push - commit_en;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            head  <= {PTRW{1'b0}};
            tail  <= {PTRW{1'b0}};
            count <= {(PTRW+1){1'b0}};
        end else begin
            tail  <= tail_next;
            head  <= commit_en ? head_next : head;
            count <= count_next;
        end
    end

    genvar gi, gj;

    wire [S*WP-1:0] w_onehot;
    generate
        for (gj = 0; gj < WP; gj = gj + 1) begin : g_wdec
            decoder #(.AW(PTRW), .OW(S)) u_wdec (
                .addr   (w_addr[gj*PTRW +: PTRW]),
                .onehot (w_onehot[gj*S +: S])
            );
        end
    endgenerate

    wire [RKW*IP-1:0] push_rank;
    wire [PTRW*IP-1:0] push_target;
    wire [S*IP-1:0]    push_onehot;
    generate
        for (gj = 0; gj < IP; gj = gj + 1) begin : g_push_addr
            onehot2bin #(.N(IP), .OW(RKW)) u_rank (
                .onehot (i_prio[gj*IP +: IP]),
                .bin    (push_rank[gj*RKW +: RKW])
            );

            wire [PTRW:0] push_sum = tail + push_rank[gj*RKW +: RKW];
            assign push_target[gj*PTRW +: PTRW] =
                (push_sum >= S) ? (push_sum - S) : push_sum[PTRW-1:0];

            decoder #(.AW(PTRW), .OW(S)) u_pdec (
                .addr   (push_target[gj*PTRW +: PTRW]),
                .onehot (push_onehot[gj*S +: S])
            );
        end
    endgenerate

    generate
        for (gi = 0; gi < S; gi = gi + 1) begin : g_entry

            wire [WP-1:0]    w_match;
            wire [32*WP-1:0] w_value_masked;
            wire [EXPL*WP-1:0] w_exception_masked;

            for (gj = 0; gj < WP; gj = gj + 1) begin : g_wmatch
                assign w_match[gj] = w_onehot[gj*S+gi] & w_en[gj];
                assign w_value_masked[gj*32 +: 32] =
                    w_value[gj*32 +: 32] & {32{w_match[gj]}};
                assign w_exception_masked[gj*EXPL +: EXPL] =
                    w_exception[gj*EXPL +: EXPL] & {EXPL{w_match[gj]}};
            end

            wire w_en_final;
            wire [31:0] w_value_final;
            wire [EXPL-1:0] w_exception_final;
            btree_or_bit  #(.N(WP))           u_wen_tree   (.in(w_match), .out(w_en_final));
            btree_or_word #(.N(WP), .W(32))   u_wval_tree  (.in(w_value_masked), .out(w_value_final));
            btree_or_word #(.N(WP), .W(EXPL)) u_wexc_tree  (.in(w_exception_masked), .out(w_exception_final));

            wire [IP-1:0]      push_match;
            wire [IDXL*IP-1:0] push_index_masked;

            for (gj = 0; gj < IP; gj = gj + 1) begin : g_pmatch
                assign push_match[gj] = push_onehot[gj*S+gi] & i_en[gj];
                assign push_index_masked[gj*IDXL +: IDXL] =
                    i_index[gj*IDXL +: IDXL] & {IDXL{push_match[gj]}};
            end

            wire push_en_final;
            wire [IDXL-1:0] push_index_final;
            btree_or_bit  #(.N(IP))           u_pen_tree  (.in(push_match), .out(push_en_final));
            btree_or_word #(.N(IP), .W(IDXL)) u_pidx_tree (.in(push_index_masked), .out(push_index_final));

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    value[gi]     <= 32'd0;
                    index[gi]     <= {IDXL{1'b0}};
                    exception[gi] <= {EXPL{1'b0}};
                    valid[gi]     <= 1'b0;
                end else begin
                    if (push_en_final) begin
                        index[gi]     <= push_index_final;
                        value[gi]     <= 32'd0;
                        exception[gi] <= {EXPL{1'b0}};
                        valid[gi]     <= 1'b0;
                    end else if (w_en_final) begin
                        value[gi]     <= w_value_final;
                        exception[gi] <= w_exception_final;
                        valid[gi]     <= 1'b1;
                    end
                end
            end

        end
    endgenerate

    generate
        for (gi = 0; gi < RP; gi = gi + 1) begin : g_rport
            assign r_value[gi*32 +: 32]       = value[r_addr[gi*PTRW +: PTRW]];
            assign r_index[gi*IDXL +: IDXL]   = index[r_addr[gi*PTRW +: PTRW]];
            assign r_exception[gi*EXPL +: EXPL] = exception[r_addr[gi*PTRW +: PTRW]];
            assign r_valid[gi]                = valid[r_addr[gi*PTRW +: PTRW]];
        end
    endgenerate

endmodule