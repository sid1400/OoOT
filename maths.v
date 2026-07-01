`timescale 1ns/1ps

module alu (
    input  [31:0] a,
    input  [31:0] b,
    input  [3:0]  alu_op,
    input start,
    input clk,

    output reg [31:0] result,
    output done
    output busy
);

    // ALU operation encoding
    localparam ALU_ADD  = 4'd0;
    localparam ALU_SUB  = 4'd1;
    localparam ALU_SLL  = 4'd2;
    localparam ALU_SLT  = 4'd3;
    localparam ALU_SLTU = 4'd4;
    localparam ALU_XOR  = 4'd5;
    localparam ALU_SRL  = 4'd6;
    localparam ALU_SRA  = 4'd7;
    localparam ALU_OR   = 4'd8;
    localparam ALU_AND  = 4'd9;

    assign busy = 0;
    always @(posedge clk) begin
        done <= start;
    end

    always @(*) begin
        case (alu_op)
            ALU_ADD:
                result = a + b;
            ALU_SUB:
                result = a - b;
            ALU_SLL:
                result = a << b[4:0];
            ALU_SLT:
                result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            ALU_SLTU:
                result = (a < b) ? 32'd1 : 32'd0;
            ALU_XOR:
                result = a ^ b;
            ALU_SRL:
                result = a >> b[4:0];
            ALU_SRA:
                result = $signed(a) >>> b[4:0];
            ALU_OR:
                result = a | b;
            ALU_AND:
                result = a & b;
            default:
                result = 32'b0;
        endcase
    end

endmodule


module rv32m_mul_pipe (
    input clk,
    input rst,
    input [3:0] start,

    input  [31:0] a0,
    input  [31:0] b0,
    input  [2:0]  op0,

    input  [31:0] a1,
    input  [31:0] b1,
    input  [2:0]  op1,

    input  [31:0] a2,
    input  [31:0] b2,
    input  [2:0]  op2,

    input  [31:0] a3,
    input  [31:0] b3,
    input  [2:0]  op3,


    output reg [3:0] busy,
    output reg [3:0] done,

    output reg [31:0] result0,
    output reg [31:0] result1,
    output reg [31:0] result2,
    output reg [31:0] result3
);


localparam MUL    = 3'd0;
localparam MULH   = 3'd1;
localparam MULHSU = 3'd2;
localparam MULHU  = 3'd3;


// Pipeline registers
reg valid0,valid1,valid2,valid3;
reg [1:0] tag0,tag1,tag2,tag3;
reg [2:0] op_pipe0,op_pipe1,op_pipe2,op_pipe3;
reg [63:0] prod0,prod1,prod2,prod3;
// Input selection

reg selected;
reg [1:0] selected_tag;
reg [31:0] selected_a;
reg [31:0] selected_b;
reg [2:0] selected_op;

always @(*) begin
    selected = 0;
    selected_tag = 0;
    selected_a = 0;
    selected_b = 0;
    selected_op = 0;
    if(start[0] && !busy[0]) begin
        selected = 1;
        selected_tag = 0;
        selected_a = a0;
        selected_b = b0;
        selected_op = op0;
    end
    else if(start[1] && !busy[1]) begin
        selected = 1;
        selected_tag = 1;
        selected_a = a1;
        selected_b = b1;
        selected_op = op1;
    end
    else if(start[2] && !busy[2]) begin
        selected = 1;
        selected_tag = 2;
        selected_a = a2;
        selected_b = b2;
        selected_op = op2;
    end
    else if(start[3] && !busy[3]) begin
        selected = 1;
        selected_tag = 3;
        selected_a = a3;
        selected_b = b3;
        selected_op = op3;
    end

end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        valid0 <= 0;
        valid1 <= 0;
        valid2 <= 0;
        valid3 <= 0;

        busy <= 0;
        done <= 0;

        result0 <= 0;
        result1 <= 0;
        result2 <= 0;
        result3 <= 0;

    end

    else begin


        done <= 0;


        //==================================
        // Stage 3 output
        //==================================

        if(valid3) begin

            done[tag3] <= 1'b1;
            busy[tag3] <= 1'b0;


            case(op_pipe3)

                MUL:
                    case(tag3)
                        0: result0 <= prod3[31:0];
                        1: result1 <= prod3[31:0];
                        2: result2 <= prod3[31:0];
                        3: result3 <= prod3[31:0];
                    endcase


                MULH,MULHSU,MULHU:
                    case(tag3)
                        0: result0 <= prod3[63:32];
                        1: result1 <= prod3[63:32];
                        2: result2 <= prod3[63:32];
                        3: result3 <= prod3[63:32];
                    endcase

            endcase

        end



        //==================================
        // Shift pipeline
        //==================================

        valid3 <= valid2;
        valid2 <= valid1;
        valid1 <= valid0;


        tag3 <= tag2;
        tag2 <= tag1;
        tag1 <= tag0;


        op_pipe3 <= op_pipe2;
        op_pipe2 <= op_pipe1;
        op_pipe1 <= op_pipe0;


        prod3 <= prod2;
        prod2 <= prod1;
        prod1 <= prod0;



        //==================================
        // New multiplication
        //==================================

        valid0 <= selected;


        if(selected) begin

            tag0 <= selected_tag;

            op_pipe0 <= selected_op;


            case(selected_op)

                MUL:
                    prod0 <= selected_a * selected_b;


                MULH:
                    prod0 <=
                    $signed(selected_a) *
                    $signed(selected_b);


                MULHSU:
                    prod0 <=
                    $signed(selected_a) *
                    $unsigned(selected_b);


                MULHU:
                    prod0 <=
                    $unsigned(selected_a) *
                    $unsigned(selected_b);


                default:
                    prod0 <= 0;

            endcase


            busy[selected_tag] <= 1'b1;

        end


    end

end


endmodule


module rv32m_div (

    input clk,
    input rst,

    input start,

    input [31:0] dividend,
    input [31:0] divisor,

    input [1:0] op,

    output reg busy,
    output reg done,

    output reg [31:0] result
);


localparam DIV  = 2'd0;
localparam DIVU = 2'd1;
localparam REM  = 2'd2;
localparam REMU = 2'd3;


reg [31:0] quotient;
reg [32:0] remainder;

reg [31:0] dividend_reg;
reg [31:0] divisor_reg;

reg [5:0] count;

reg neg_a;
reg neg_b;

reg [1:0] operation;



always @(posedge clk or posedge rst) begin

    if(rst) begin
        busy <= 0;
        done <= 0;
        result <= 0;

        quotient <= 0;
        remainder <= 0;

        count <= 0;

    end


    else begin


        done <= 0;



        if(start && !busy) begin


            operation <= op;


            neg_a <= ((op==DIV)||(op==REM)) && dividend[31];
            neg_b <= ((op==DIV)||(op==REM)) && divisor[31];


            dividend_reg <=
                (((op==DIV)||(op==REM)) &&
                 dividend[31]) ?
                 (~dividend+1):
                 dividend;


            divisor_reg <=
                (((op==DIV)||(op==REM)) &&
                 divisor[31]) ?
                 (~divisor+1):
                 divisor;


            quotient <= 0;
            remainder <= 0;


            count <= 32;


            busy <= 1;


        end



        else if(busy) begin



            remainder <=
            {remainder[31:0],
             dividend_reg[31]};


            dividend_reg <=
            {dividend_reg[30:0],1'b0};



            quotient <=
            {quotient[30:0],1'b0};



            if(remainder >= divisor_reg) begin

                remainder <=
                remainder-divisor_reg;

                quotient[0] <= 1'b1;

            end



            count <= count-1;



            if(count==1) begin


                busy <= 0;
                done <= 1;


                case(operation)


                DIV:

                    if(divisor_reg==0)
                        result <= 32'hFFFFFFFF;

                    else if(neg_a ^ neg_b)
                        result <= ~quotient + 1;

                    else
                        result <= quotient;



                DIVU:

                    if(divisor_reg==0)
                        result <= 32'hFFFFFFFF;

                    else
                        result <= quotient;



                REM:

                    if(divisor_reg==0)
                        result <= dividend_reg;

                    else if(neg_a)
                        result <= ~remainder[31:0]+1;

                    else
                        result <= remainder[31:0];



                REMU:

                    if(divisor_reg==0)
                        result <= dividend_reg;

                    else
                        result <= remainder[31:0];


                endcase

            end

        end

    end

end


endmodule

module load_unit #(
    parameter TAGW = 6
) (
    input  logic         clk,
    input  logic         rst_n,

    input  logic         disp_valid,
    output logic         disp_ready,
    input  logic [31:0]  disp_addr,
    input  logic [2:0]   disp_funct3,
    input  logic [TAGW-1:0] disp_tag,

    output logic         mem_valid,
    input  logic         mem_ready,
    output logic [31:0]  mem_addr,

    input  logic         mem_resp_valid,
    output logic         mem_resp_ready,
    input  logic [31:0]  mem_resp_data,

    output logic         out_valid,
    input  logic         out_ready,
    output logic [31:0]  out_data,
    output logic [TAGW-1:0] out_tag
);

    typedef enum logic [1:0] {IDLE, WAIT_MEM, WAIT_OUT} state_t;
    state_t state, state_n;

    logic [31:0] addr_q;
    logic [2:0]  funct3_q;
    logic [TAGW-1:0] tag_q;
    logic [31:0] data_q;

    assign disp_ready     = (state == IDLE);
    assign mem_valid      = (state == WAIT_MEM);
    assign mem_addr       = addr_q;
    assign mem_resp_ready = (state == WAIT_MEM);
    assign out_valid      = (state == WAIT_OUT);
    assign out_tag        = tag_q;

    always_comb begin
        case (funct3_q)
            3'b000: out_data = {{24{data_q[7]}},  data_q[7:0]};
            3'b001: out_data = {{16{data_q[15]}}, data_q[15:0]};
            3'b010: out_data = data_q;
            3'b100: out_data = {24'b0, data_q[7:0]};
            3'b101: out_data = {16'b0, data_q[15:0]};
            default: out_data = data_q;
        endcase
    end

    always_comb begin
        state_n = state;
        case (state)
            IDLE:     if (disp_valid && disp_ready) state_n = WAIT_MEM;
            WAIT_MEM: if (mem_resp_valid && mem_resp_ready) state_n = WAIT_OUT;
            WAIT_OUT: if (out_valid && out_ready) state_n = IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= state_n;
            if (state == IDLE && disp_valid && disp_ready) begin
                addr_q   <= disp_addr;
                funct3_q <= disp_funct3;
                tag_q    <= disp_tag;
            end
            if (state == WAIT_MEM && mem_resp_valid && mem_resp_ready) begin
                data_q <= mem_resp_data;
            end
        end
    end

endmodule