module instr_buf #(
    parameter PN = 4
)(
    input              clk,
    input              rst_n,

    input  [PN*32-1:0] instr_i,
    input  [PN-1:0]    valid_i,
    output [PN-1:0]    ready_o,

    output [PN*56-1:0] entry_o,
    output [PN-1:0]    valid_o,
    input  [PN-1:0]    ready_i
);
    localparam ITYPE_ALU    = 3'd0;
    localparam ITYPE_MULT   = 3'd1;
    localparam ITYPE_DIV    = 3'd2;
    localparam ITYPE_LOAD   = 3'd3;
    localparam ITYPE_STR    = 3'd4;
    localparam ITYPE_BRANCH = 3'd5;
    localparam ITYPE_TRAP   = 3'd6;
    localparam ITYPE_MULTI  = 3'd7;
    localparam EW = 56;

    function [EW-1:0] decode_instr;
        input [31:0] instr;
        reg [6:0]  opcode;
        reg [2:0]  funct3;
        reg [6:0]  funct7;
        reg [2:0]  itype;
        reg [16:0] op;
        reg [4:0]  reg1, reg2, dest;
        reg [20:0] imm;
        begin
            opcode = instr[6:0];
            funct3 = instr[14:12];
            funct7 = instr[31:25];
            op     = {funct7, funct3, opcode};
            case (opcode)
                7'b1101111: begin
                    itype = ITYPE_MULTI;
                    reg1  = 5'd0;
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
                end
                7'b1100111: begin
                    itype = ITYPE_MULTI;
                    reg1  = instr[19:15];
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {{9{instr[31]}}, instr[31:20]};
                end
                7'b1110011: begin
                    itype = ITYPE_TRAP;
                    reg1  = 5'd0;
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {{9{instr[31]}}, instr[31:20]};
                end
                7'b0000011: begin
                    itype = ITYPE_LOAD;
                    reg1  = instr[19:15];
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {{9{instr[31]}}, instr[31:20]};
                end
                7'b0100011: begin
                    itype = ITYPE_STR;
                    reg1  = instr[19:15];
                    reg2  = instr[24:20];
                    dest  = 5'd0;
                    imm   = {{9{instr[31]}}, instr[31:25], instr[11:7]};
                end
                7'b1100011: begin
                    itype = ITYPE_BRANCH;
                    reg1  = instr[19:15];
                    reg2  = instr[24:20];
                    dest  = 5'd0;
                    imm   = {{8{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
                end
                7'b0110011: begin
                    if (funct7[0])
                        itype = funct3[2] ? ITYPE_DIV : ITYPE_MULT;
                    else
                        itype = ITYPE_ALU;
                    reg1 = instr[19:15];
                    reg2 = instr[24:20];
                    dest = instr[11:7];
                    imm  = 21'd0;
                end
                7'b0010011: begin
                    itype = ITYPE_ALU;
                    reg1  = instr[19:15];
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {{9{instr[31]}}, instr[31:20]};
                end
                7'b0110111: begin
                    itype = ITYPE_ALU;
                    reg1  = 5'd0;
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {1'b0, instr[31:12]};
                end
                7'b0010111: begin
                    itype = ITYPE_ALU;
                    reg1  = 5'd0;
                    reg2  = 5'd0;
                    dest  = instr[11:7];
                    imm   = {1'b0, instr[31:12]};
                end
                default: begin
                    itype = ITYPE_ALU;
                    reg1  = 5'd0;
                    reg2  = 5'd0;
                    dest  = 5'd0;
                    imm   = 21'd0;
                end
            endcase
            decode_instr = {itype, op, reg1, reg2, dest, imm};
        end
    endfunction

    reg [EW-1:0] entry_r [0:PN-1];
    reg          valid_r [0:PN-1];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        for (i = 0; i < PN; i = i + 1) begin
            if (!rst_n) begin
                valid_r[i] <= 1'b0;
            end else if (ready_i[i] | ~valid_r[i]) begin
                valid_r[i] <= valid_i[i];
                if (valid_i[i])
                    entry_r[i] <= decode_instr(instr_i[i*32 +: 32]);
            end
        end
    end

    genvar j;
    generate
        for (j = 0; j < PN; j = j + 1) begin : out_assign
            assign ready_o[j]          = ready_i[j] | ~valid_r[j];
            assign valid_o[j]          = valid_r[j];
            assign entry_o[j*56 +: 56] = entry_r[j];
        end
    endgenerate
endmodule