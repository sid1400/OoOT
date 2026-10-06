module crash_arbitrator #(
    parameter TAGL = 6,
    parameter PCW  = 32,
    parameter CRW  = 4
)(
    input clk,
    input rst,

    input              branch_crash_valid,
    input  [TAGL-1:0]  branch_crash_tail,
    input  [CRW-1:0]   branch_crash_reason,
    input              branch_pc_valid,
    input  [PCW-1:0]   branch_new_pc,

    output             crash_valid,
    output [TAGL-1:0]  crash_tail,
    output [CRW-1:0]   crash_reason,
    output             pc_valid,
    output [PCW-1:0]   new_pc
);

    assign crash_valid  = branch_crash_valid;
    assign crash_tail   = branch_crash_tail;
    assign crash_reason = branch_crash_reason;
    assign pc_valid     = branch_pc_valid;
    assign new_pc       = branch_new_pc;

endmodule