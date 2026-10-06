module alu #(
    parameter TAGL = 6,
    parameter OPW  = 17,
    parameter DW   = 32,
    parameter IMMW = 21
)(
    input clk,
    input rst,

    input                  in_valid,
    output                 in_ready,
    input  [OPW-1:0]       opcode,
    input  [DW-1:0]        src1_value,
    input  [DW-1:0]        src2_value,
    input  [TAGL-1:0]      dest_tag,
    input  [IMMW-1:0]      imm,

    output                 wb_valid,
    input                  wb_ready,
    output [TAGL-1:0]      wb_dest_tag,
    output [DW-1:0]        wb_data,

    input                  kill_i,
    input  [TAGL-1:0]      kill_head_i,
    input  [TAGL-1:0]      kill_tail_i
);

    function in_range;
        input [TAGL-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_range = (idx >= hd) && (idx < tl);
            else
                in_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    wire [6:0] real_opcode = opcode[6:0];
    wire [2:0] funct3      = opcode[9:7];
    wire       funct7_5    = opcode[15];
    wire       is_reg      = real_opcode[5];

    wire [DW-1:0] sext_imm = {{(DW-IMMW){imm[IMMW-1]}}, imm};
    wire [DW-1:0] operand2 = is_reg ? src2_value : sext_imm;

    wire sub = is_reg & funct7_5 & (funct3 == 3'b000);
    wire sra = funct7_5 & (funct3 == 3'b101);

    reg [DW-1:0] alu_result;
    always @(*) begin
        case (funct3)
            3'b000:  alu_result = sub ? (src1_value - operand2) : (src1_value + operand2);
            3'b001:  alu_result = src1_value << operand2[4:0];
            3'b010:  alu_result = ($signed(src1_value) < $signed(operand2)) ? {{(DW-1){1'b0}}, 1'b1} : {DW{1'b0}};
            3'b011:  alu_result = (src1_value < operand2) ? {{(DW-1){1'b0}}, 1'b1} : {DW{1'b0}};
            3'b100:  alu_result = src1_value ^ operand2;
            3'b101:  alu_result = sra ? ($signed(src1_value) >>> operand2[4:0]) : (src1_value >> operand2[4:0]);
            3'b110:  alu_result = src1_value | operand2;
            3'b111:  alu_result = src1_value & operand2;
            default: alu_result = {DW{1'b0}};
        endcase
    end

    // squashed op is accepted (drained) but never written back
    wire squash = kill_i & ~in_range(dest_tag, kill_head_i, kill_tail_i);

    assign in_ready    = wb_ready | squash;
    assign wb_valid    = in_valid & ~squash;
    assign wb_dest_tag = dest_tag;
    assign wb_data     = alu_result;

endmodule


module mul_unit #(
    parameter LAT  = 4,
    parameter TAGL = 6,
    parameter OPW  = 17,
    parameter DW   = 32,
    parameter CW   = (LAT > 1) ? $clog2(LAT) : 1
)(
    input clk,
    input rst,

    input                  in_valid,
    output                 in_ready,
    input  [OPW-1:0]       opcode,
    input  [DW-1:0]        src1_value,
    input  [DW-1:0]        src2_value,
    input  [TAGL-1:0]      dest_tag,

    output                 wb_valid,
    input                  wb_ready,
    output [TAGL-1:0]      wb_dest_tag,
    output [DW-1:0]        wb_data,

    input                  kill_i,
    input  [TAGL-1:0]      kill_head_i,
    input  [TAGL-1:0]      kill_tail_i
);

    localparam MUL    = 2'd0;
    localparam MULH   = 2'd1;
    localparam MULHSU = 2'd2;
    localparam MULHU  = 2'd3;

    function in_range;
        input [TAGL-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_range = (idx >= hd) && (idx < tl);
            else
                in_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    reg busy;
    reg hold_valid;
    reg dead_r;
    reg [CW-1:0]   count;
    reg [1:0]      op_r;
    reg [TAGL-1:0] dest_tag_r;
    reg [63:0]     prod_r;
    reg [DW-1:0]   result_r;

    wire [1:0] mul_op = opcode[8:7];
    assign in_ready = ~busy & ~hold_valid;
    wire accept = in_valid & in_ready;

    // crash-time range checks: incoming op, and op already in flight
    wire sq_in   = kill_i & ~in_range(dest_tag,   kill_head_i, kill_tail_i);
    wire sq_held = kill_i & ~in_range(dest_tag_r, kill_head_i, kill_tail_i);
    wire drop    = hold_valid & (dead_r | sq_held);

    assign wb_valid    = hold_valid & ~dead_r & ~sq_held;
    assign wb_dest_tag = dest_tag_r;
    assign wb_data     = result_r;

    reg [63:0] prod_next;
    always @(*) begin
        case (mul_op)
            MUL:     prod_next = {32'd0, src1_value} * {32'd0, src2_value};
            MULH:    prod_next = {{32{src1_value[31]}}, src1_value} * {{32{src2_value[31]}}, src2_value};
            MULHSU:  prod_next = {{32{src1_value[31]}}, src1_value} * {32'd0, src2_value};
            MULHU:   prod_next = {32'd0, src1_value} * {32'd0, src2_value};
            default: prod_next = 64'd0;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy       <= 1'b0;
            hold_valid <= 1'b0;
            dead_r     <= 1'b0;
            count      <= {CW{1'b0}};
        end else begin
            if (accept) begin
                busy       <= 1'b1;
                op_r       <= mul_op;
                dest_tag_r <= dest_tag;
                prod_r     <= prod_next;
                count      <= LAT[CW-1:0] - 1'b1;
                dead_r     <= sq_in;
            end else if (busy) begin
                if (sq_held)
                    dead_r <= 1'b1;
                if (count == {CW{1'b0}}) begin
                    busy       <= 1'b0;
                    hold_valid <= 1'b1;
                    result_r   <= (op_r == MUL) ? prod_r[31:0] : prod_r[63:32];
                end else begin
                    count <= count - 1'b1;
                end
            end

            // released by a real transfer, or silently dropped if dead
            if ((wb_valid & wb_ready) | drop)
                hold_valid <= 1'b0;
        end
    end

endmodule


module rv32m_div #(
    parameter TAGL = 6,
    parameter OPW  = 17,
    parameter DW   = 32
)(
    input clk,
    input rst,

    input                  in_valid,
    output                 in_ready,
    input  [OPW-1:0]       opcode,
    input  [DW-1:0]        src1_value,
    input  [DW-1:0]        src2_value,
    input  [TAGL-1:0]      dest_tag,

    output                 wb_valid,
    input                  wb_ready,
    output [TAGL-1:0]      wb_dest_tag,
    output [DW-1:0]        wb_data,

    input                  kill_i,
    input  [TAGL-1:0]      kill_head_i,
    input  [TAGL-1:0]      kill_tail_i
);

    localparam DIV  = 2'd0;
    localparam DIVU = 2'd1;
    localparam REM  = 2'd2;
    localparam REMU = 2'd3;

    function in_range;
        input [TAGL-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_range = (idx >= hd) && (idx < tl);
            else
                in_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    reg busy;
    reg hold_valid;
    reg dead_r;
    reg [1:0]      op_r;
    reg [TAGL-1:0] dest_tag_r;
    reg [31:0]     quotient;
    reg [32:0]     remainder;
    reg [31:0]     dividend_r;
    reg [31:0]     divisor_r;
    reg [31:0]     orig_a;      // raw dividend, for divide-by-zero REM
    reg [5:0]      count;
    reg            neg_a, neg_b;
    reg [DW-1:0]   result_r;

    wire [1:0] div_op = opcode[8:7];
    wire sgn_op = (div_op == DIV) | (div_op == REM);
    assign in_ready = ~busy & ~hold_valid;
    wire accept = in_valid & in_ready;

    wire sq_in   = kill_i & ~in_range(dest_tag,   kill_head_i, kill_tail_i);
    wire sq_held = kill_i & ~in_range(dest_tag_r, kill_head_i, kill_tail_i);
    wire drop    = hold_valid & (dead_r | sq_held);

    assign wb_valid    = hold_valid & ~dead_r & ~sq_held;
    assign wb_dest_tag = dest_tag_r;
    assign wb_data     = result_r;

    // one restoring-division step (shift in next dividend bit, then compare)
    wire [32:0] rem_shift = {remainder[31:0], dividend_r[31]};
    wire        sub_ok    = (rem_shift >= {1'b0, divisor_r});
    wire [32:0] rem_next  = sub_ok ? (rem_shift - {1'b0, divisor_r}) : rem_shift;
    wire [31:0] quo_next  = {quotient[30:0], sub_ok};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy       <= 1'b0;
            hold_valid <= 1'b0;
            dead_r     <= 1'b0;
            count      <= 6'd0;
        end else begin
            if (accept) begin
                op_r       <= div_op;
                dest_tag_r <= dest_tag;
                orig_a     <= src1_value;
                neg_a      <= sgn_op & src1_value[31];
                neg_b      <= sgn_op & src2_value[31];
                dividend_r <= (sgn_op & src1_value[31]) ? (~src1_value + 1'b1) : src1_value;
                divisor_r  <= (sgn_op & src2_value[31]) ? (~src2_value + 1'b1) : src2_value;
                quotient   <= 32'd0;
                remainder  <= 33'd0;
                count      <= 6'd32;
                busy       <= 1'b1;
                dead_r     <= sq_in;
            end else if (busy) begin
                if (sq_held)
                    dead_r <= 1'b1;
                remainder  <= rem_next;
                dividend_r <= {dividend_r[30:0], 1'b0};
                quotient   <= quo_next;
                count      <= count - 1'b1;
                if (count == 6'd1) begin
                    busy       <= 1'b0;
                    hold_valid <= 1'b1;
                    case (op_r)
                        DIV:  result_r <= (divisor_r == 0) ? 32'hFFFFFFFF : ((neg_a ^ neg_b) ? (~quo_next + 1'b1) : quo_next);
                        DIVU: result_r <= (divisor_r == 0) ? 32'hFFFFFFFF : quo_next;
                        REM:  result_r <= (divisor_r == 0) ? orig_a : (neg_a ? (~rem_next[31:0] + 1'b1) : rem_next[31:0]);
                        REMU: result_r <= (divisor_r == 0) ? orig_a : rem_next[31:0];
                    endcase
                end
            end

            if ((wb_valid & wb_ready) | drop)
                hold_valid <= 1'b0;
        end
    end

endmodule