//decrypt

module circ_queue #(
    parameter S    = 16,
    parameter DW   = 32,
    parameter IP   = 3,
    parameter OP   = 3,
    parameter PTRW = $clog2(S)
)(
    input  clk,
    input  rst,

    input  [IP-1:0]    input_valid,
    input  [IP*DW-1:0] input_value,
    output              ready_out,

    output [OP*DW-1:0] out_value,
    output [OP-1:0]    out_valid,
    input  [OP-1:0]    active_i,
    input  [OP-1:0]    block_i,
    input               ready_in,

    output [PTRW-1:0]  head_ptr,
    output [PTRW-1:0]  tail_ptr,
    output [$clog2(OP)+1-1:0] pop_count
);

    localparam CW = $clog2(OP) + 1;

    reg [DW-1:0]   data_mem [0:S-1];
    reg [PTRW-1:0] head, tail;
    reg [PTRW:0]   count;

    assign head_ptr  = head;
    assign tail_ptr  = tail;
    assign ready_out = ((S - count) >= IP);

    wire [IP-1:0] eff_valid;

    genvar gj, gi;

    generate
        for (gj = 0; gj < IP; gj = gj + 1) begin : g_push
            assign eff_valid[gj] = input_valid[gj] & ready_out;

            always @(posedge clk) begin
                if (eff_valid[gj])
                    data_mem[tail + (IP-1-gj)] <= input_value[gj*DW +: DW];
            end
        end
    endgenerate

    integer pj;
    reg [PTRW:0] n_push;
    always @* begin
        n_push = {(PTRW+1){1'b0}};
        for (pj = 0; pj < IP; pj = pj + 1)
            n_push = n_push + eff_valid[pj];
    end

    wire [OP-1:0] occ;
    wire [OP-1:0] allow_pre;
    wire [OP-1:0] at_allow_rev;
    wire [OP-1:0] at_block_rev;
    wire [OP-1:0] at_out_rev;
    wire [CW-1:0] at_count;

    generate
        for (gi = 0; gi < OP; gi = gi + 1) begin : g_pop
            assign out_value[gi*DW +: DW] = data_mem[head + gi];
            assign occ[gi]                = (count > gi);
            assign allow_pre[gi]          = occ[gi] & active_i[gi];
            assign at_allow_rev[OP-1-gi]  = allow_pre[gi];
            assign at_block_rev[OP-1-gi]  = block_i[gi];
            assign out_valid[gi]          = at_out_rev[OP-1-gi];
        end
    endgenerate

    antitherm #(.PN(OP), .CW(CW)) u_pop_therm (
        .allow (at_allow_rev),
        .block (at_block_rev),
        .out   (at_out_rev),
        .count (at_count)
    );

    assign pop_count = ready_in ? at_count : {CW{1'b0}};

    wire [PTRW-1:0] tail_next  = tail + n_push;
    wire [PTRW-1:0] head_next  = head + pop_count;
    wire [PTRW:0]   count_next = count + n_push - pop_count;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            head  <= {PTRW{1'b0}};
            tail  <= {PTRW{1'b0}};
            count <= {(PTRW+1){1'b0}};
        end else begin
            head  <= head_next;
            tail  <= tail_next;
            count <= count_next;
        end
    end

endmodule