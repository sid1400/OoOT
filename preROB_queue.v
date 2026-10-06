`include "prefetch.v"

module wb_queue #(
    parameter N    = 8,
    parameter INP  = 2,
    parameter BUSP = 2,
    parameter TW   = 6,
    parameter DW   = 32,
    parameter EXPL = 4,
    parameter PTRW = $clog2(N)
)(
    input clk,
    input rst,

    input  [INP*TW-1:0]   push_tag_i,
    input  [INP*DW-1:0]   push_data_i,
    input  [INP*EXPL-1:0] push_exception_i,
    input  [INP-1:0]      push_valid_i,
    output [INP-1:0]      push_ready_o,

    output [BUSP*TW-1:0]   bus_tag_o,
    output [BUSP*DW-1:0]   bus_data_o,
    output [BUSP*EXPL-1:0] bus_exception_o,
    output [BUSP-1:0]      bus_entry_valid_o,
    output [BUSP-1:0]      bus_valid_o,
    input  [BUSP-1:0]      bus_ready_i,

    input             crash_i,
    input [TW-1:0]    rob_head_i,
    input [TW-1:0]    rob_tail_i
);

    reg [DW-1:0]   data_r       [0:N-1];
    reg [TW-1:0]   tag_r        [0:N-1];
    reg [EXPL-1:0] exception_r  [0:N-1];
    reg            entry_valid_r[0:N-1];

    reg [PTRW-1:0] head, tail;
    reg [PTRW:0]   count;

    function in_slot_range;
        input [PTRW-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_slot_range = (idx >= hd) && (idx < tl);
            else
                in_slot_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    function in_tag_range;
        input [TW-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_tag_range = (idx >= hd) && (idx < tl);
            else
                in_tag_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    wire [PTRW:0] free_count = N - count;
    wire accept = (free_count >= INP);
    assign push_ready_o = {INP{accept}};

    integer pc;
    reg [$clog2(INP+1)-1:0] push_count;
    always @* begin
        push_count = 0;
        for (pc = 0; pc < INP; pc = pc + 1)
            push_count = push_count + (push_valid_i[pc] & accept);
    end

    wire [BUSP-1:0] avail;
    wire [BUSP-1:0] allow_bus, block_bus, therm_out;
    wire [$clog2(BUSP):0] pop_count;

    genvar gb;
    generate
        for (gb = 0; gb < BUSP; gb = gb + 1) begin : g_bus
            wire [PTRW-1:0] slot_idx = head + gb[PTRW-1:0];
            assign avail[gb] = (gb < count);
            assign bus_tag_o[gb*TW +: TW]             = tag_r[slot_idx];
            assign bus_data_o[gb*DW +: DW]             = data_r[slot_idx];
            assign bus_exception_o[gb*EXPL +: EXPL]    = exception_r[slot_idx];
            assign bus_entry_valid_o[gb]                = entry_valid_r[slot_idx] & avail[gb];
            assign bus_valid_o[gb]                      = avail[gb];

            assign allow_bus[BUSP-1-gb] = avail[gb];
            assign block_bus[BUSP-1-gb] = ~bus_ready_i[gb];
        end
    endgenerate

    antitherm #(.PN(BUSP)) u_wbq_therm (
        .allow (allow_bus),
        .block (block_bus),
        .out   (therm_out),
        .count (pop_count)
    );

    integer j, ki;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            head  <= {PTRW{1'b0}};
            tail  <= {PTRW{1'b0}};
            count <= {(PTRW+1){1'b0}};
            for (ki = 0; ki < N; ki = ki + 1)
                entry_valid_r[ki] <= 1'b0;
        end else begin
            for (j = 0; j < INP; j = j + 1) begin
                if (push_valid_i[j] & accept) begin
                    data_r[tail + j[PTRW-1:0]]        <= push_data_i[j*DW +: DW];
                    tag_r[tail + j[PTRW-1:0]]         <= push_tag_i[j*TW +: TW];
                    exception_r[tail + j[PTRW-1:0]]   <= push_exception_i[j*EXPL +: EXPL];
                    entry_valid_r[tail + j[PTRW-1:0]] <= 1'b1;
                end
            end

            if (crash_i) begin
                for (ki = 0; ki < N; ki = ki + 1) begin
                    if (in_slot_range(ki[PTRW-1:0], head, tail) &&
                        entry_valid_r[ki] &&
                        !in_tag_range(tag_r[ki], rob_head_i, rob_tail_i))
                        entry_valid_r[ki] <= 1'b0;
                end
            end

            head  <= head + pop_count;
            tail  <= tail + push_count;
            count <= count + push_count - pop_count;
        end
    end

endmodule