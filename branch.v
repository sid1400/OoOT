// Branch FU. BRANCH uses the predicted-taken bit in opcode[6]; JAL/JALR are always
// treated as predicted not-taken, so they always redirect from here.
module branch_fu #(
    parameter TAGL = 6,
    parameter OPW  = 17,
    parameter DW   = 32,
    parameter IMMW = 21,
    parameter PCW  = 32,
    parameter CRW  = 4
)(
    input                  clk,
    input                  rst,

    input                  in_valid,
    output                 in_ready,
    input  [OPW-1:0]       opcode,
    input  [DW-1:0]        src1_value,
    input  [DW-1:0]        src2_value,
    input  [TAGL-1:0]      dest_tag,
    input  [IMMW-1:0]      imm,
    input  [PCW-1:0]       pc,

    output                 wb_valid,
    input                  wb_ready,
    output [TAGL-1:0]      wb_dest_tag,
    output [DW-1:0]        wb_data,

    output                 crash_valid,
    output [TAGL-1:0]      crash_tail,
    output [CRW-1:0]       crash_reason,

    output                 pc_valid,
    output [PCW-1:0]       new_pc,

    // global crash broadcast, used to suppress squashed results
    input                  kill_i,
    input  [TAGL-1:0]      kill_head_i,
    input  [TAGL-1:0]      kill_tail_i
);

    localparam [6:0] OPC_BRANCH = 7'b1100011;
    localparam [6:0] OPC_JAL    = 7'b1101111;
    localparam [6:0] OPC_JALR   = 7'b1100111;

    function in_range;
        input [TAGL-1:0] idx, hd, tl;
        begin
            if (hd <= tl)
                in_range = (idx >= hd) && (idx < tl);
            else
                in_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    // opcode[6] carries predicted-taken; real bit 6 is 1 for all three types
    wire [6:0] real_opcode = {1'b1, opcode[5:0]};
    wire [2:0] funct3      = opcode[9:7];
    wire       predicted   = opcode[6];

    wire is_branch = (real_opcode == OPC_BRANCH);
    wire is_jal    = (real_opcode == OPC_JAL);
    wire is_jalr   = (real_opcode == OPC_JALR);

    reg cond_taken;
    always @(*) begin
        case (funct3)
            3'b000:  cond_taken = (src1_value == src2_value);
            3'b001:  cond_taken = (src1_value != src2_value);
            3'b100:  cond_taken = ($signed(src1_value) <  $signed(src2_value));
            3'b101:  cond_taken = ($signed(src1_value) >= $signed(src2_value));
            3'b110:  cond_taken = (src1_value <  src2_value);
            3'b111:  cond_taken = (src1_value >= src2_value);
            default: cond_taken = 1'b0;
        endcase
    end

    wire mispredict = (is_branch & (cond_taken != predicted)) | is_jal | is_jalr;

    wire [PCW-1:0] sext_imm  = {{(PCW-IMMW){imm[IMMW-1]}}, imm};
    wire [PCW-1:0] target_pc = is_jalr ? ((src1_value + sext_imm) & ~32'h1)
                                       : (pc          + sext_imm);

    wire squash = kill_i & ~in_range(dest_tag, kill_head_i, kill_tail_i);

    assign in_ready    = wb_ready | squash;
    assign wb_valid    = in_valid & ~squash;
    assign wb_dest_tag = dest_tag;
    assign wb_data     = {DW{1'b0}};

    assign crash_valid  = in_valid & wb_ready & mispredict;
    assign crash_tail   = dest_tag;
    assign crash_reason = {CRW{1'b0}};

    assign pc_valid = crash_valid;
    assign new_pc   = target_pc;

endmodule