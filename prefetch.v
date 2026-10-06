`include "DISPYBB.v"

// Assumptions (flag if wrong):
//  - DAT=32, one arbiter word = one instruction; ADRB maps to pc[ADRB+1:2]
//    (byte-addressed PC, word-addressed arbiter/mem)
//  - taken branch/JAL instruction ITSELF is accepted this group; block starts
//    at the slot right after it (matches antitherm's "block excludes onward"
//    semantics while still delivering the jump instruction downstream)
//  - single group in flight at a time: 1 bubble cycle between a group commit
//    and the next request issue, and 1 bubble cycle after crash-drain completes
//  - JALR: never queried against predictor, always guessed not-taken, path bit 0
module prefetcher #(
    parameter IP   = 4,
    parameter ADRB = 12,
    parameter DAT  = 32,
    parameter PCW  = 32
)(
    input  wire clk,
    input  wire rst,

    // arbiter read ports, one per fetch slot
    output wire [IP*ADRB-1:0] mem_raddr_o,
    output wire [IP-1:0]      mem_ren_o,
    input  wire [IP-1:0]      mem_rvalid_i,
    input  wire [IP*DAT-1:0]  mem_rdata_i,
    output wire [IP-1:0]      mem_rready_o,

    // branch predictor read ports (combinational, zero-latency)
    output wire [IP*PCW-1:0]  pred_addr_r_o,
    output wire [IP-1:0]      pred_valid_r_o,
    input  wire [IP-1:0]      pred_taken_i,
    input  wire [IP-1:0]      pred_valid_i,

    // to decode/storage
    output wire [IP*DAT-1:0]  instr_o,
    output wire [IP*PCW-1:0]  pc_o,
    output wire [IP-1:0]      path_taken_o,
    output wire [IP-1:0]      valid_o,
    input  wire                ready_i,

    // crash / redirect
    input  wire               crash_i,
    input  wire [PCW-1:0]     crash_pc_i
);

    localparam OPC_BRANCH = 7'b1100011;
    localparam OPC_JAL    = 7'b1101111;
    localparam OPC_JALR   = 7'b1100111;

    reg [PCW-1:0] pc_r;
    reg [DAT-1:0] instr_buf_r [0:IP-1];
    reg [PCW-1:0] pc_buf_r    [0:IP-1];
    reg           requested_r [0:IP-1];
    reg           captured_r  [0:IP-1];
    reg           crash_pend_r;

    // idle: no port currently has an outstanding/uncaptured request
    integer q;
    reg idle;
    reg all_drained;
    reg group_complete_r;
    always @* begin
        idle              = 1'b1;
        all_drained       = 1'b1;
        group_complete_r  = 1'b1;
        for (q = 0; q < IP; q = q + 1) begin
            if (requested_r[q]) idle = 1'b0;
            if (requested_r[q] && !captured_r[q]) all_drained = 1'b0;
            if (!captured_r[q]) group_complete_r = 1'b0;
        end
    end
    wire group_complete = group_complete_r & ~crash_pend_r;

    genvar gi;

    // request-side addressing
    wire [PCW-1:0] slot_pc [0:IP-1];
    generate
        for (gi = 0; gi < IP; gi = gi + 1) begin : g_req
            assign slot_pc[gi]                   = pc_r + (gi*4);
            assign mem_raddr_o[gi*ADRB +: ADRB]  = slot_pc[gi][ADRB+1:2];
            assign mem_ren_o[gi]                 = idle & ~crash_pend_r;
            assign mem_rready_o[gi]              = requested_r[gi] & ~captured_r[gi];
        end
    endgenerate

    // decode + predictor query, per captured slot
    wire [6:0] opc [0:IP-1];
    wire is_branch [0:IP-1];
    wire is_jal    [0:IP-1];
    wire is_jalr   [0:IP-1];
    wire is_jump   [0:IP-1];
    wire signed [PCW-1:0] imm_b [0:IP-1];
    wire signed [PCW-1:0] imm_j [0:IP-1];
    wire [PCW-1:0] target [0:IP-1];
    wire taken [0:IP-1];

    generate
        for (gi = 0; gi < IP; gi = gi + 1) begin : g_decode
            assign opc[gi]       = instr_buf_r[gi][6:0];
            assign is_branch[gi] = (opc[gi] == OPC_BRANCH);
            assign is_jal[gi]    = (opc[gi] == OPC_JAL);
            assign is_jalr[gi]   = (opc[gi] == OPC_JALR);
            assign is_jump[gi]   = is_branch[gi] | is_jal[gi]; // PC-relative -> predictor queried

            assign imm_b[gi] = {{19{instr_buf_r[gi][31]}}, instr_buf_r[gi][31], instr_buf_r[gi][7],
                                 instr_buf_r[gi][30:25], instr_buf_r[gi][11:8], 1'b0};
            assign imm_j[gi] = {{11{instr_buf_r[gi][31]}}, instr_buf_r[gi][31], instr_buf_r[gi][19:12],
                                 instr_buf_r[gi][20], instr_buf_r[gi][30:21], 1'b0};

            assign target[gi] = is_jal[gi] ? (pc_buf_r[gi] + imm_j[gi]) : (pc_buf_r[gi] + imm_b[gi]);

            assign pred_addr_r_o[gi*PCW +: PCW] = pc_buf_r[gi];
            assign pred_valid_r_o[gi]           = is_jump[gi] & captured_r[gi];

            assign taken[gi] = is_jump[gi] & pred_taken_i[gi] & pred_valid_i[gi];
            // JALR: is_jump=0 -> taken=0 always, guessed not-taken by construction
        end
    endgenerate

    // antitherm block: accept up to and including the first taken jump, block after it
    reg [IP-1:0]  allow_bus;
    reg [IP-1:0]  block_bus;
    reg           found_taken;
    reg [PCW-1:0] pc_next_r;
    integer m;
    always @* begin
        found_taken = 1'b0;
        pc_next_r   = pc_r + (IP*4);
        for (m = 0; m < IP; m = m + 1) begin
            allow_bus[IP-1-m] = captured_r[m];
            block_bus[IP-1-m] = found_taken;
            if (!found_taken && taken[m]) begin
                found_taken = 1'b1;
                pc_next_r   = target[m];
            end
        end
    end

    wire [IP-1:0] therm_out;
    wire [$clog2(IP):0] therm_count;
    antitherm #(.PN(IP)) u_antitherm (
        .allow (allow_bus),
        .block (block_bus),
        .out   (therm_out),
        .count (therm_count)
    );

    // outputs to decode/storage
    generate
        for (gi = 0; gi < IP; gi = gi + 1) begin : g_out
            assign valid_o[gi]            = therm_out[IP-1-gi] & group_complete;
            assign instr_o[gi*DAT +: DAT] = instr_buf_r[gi];
            assign pc_o[gi*PCW +: PCW]    = pc_buf_r[gi];
            assign path_taken_o[gi]       = taken[gi];
        end
    endgenerate

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_r         <= {PCW{1'b0}};
            crash_pend_r <= 1'b0;
            for (i = 0; i < IP; i = i + 1) begin
                requested_r[i] <= 1'b0;
                captured_r[i]  <= 1'b0;
            end
        end else begin

            // absorb / discard unconditionally, before the crash branch, so an
            // rvalid arriving the same cycle as crash_i is never dropped silently
            for (i = 0; i < IP; i = i + 1) begin
                if (mem_rvalid_i[i] && mem_rready_o[i]) begin
                    if (crash_i || crash_pend_r) begin
                        requested_r[i] <= 1'b0; // drained, discard
                    end else begin
                        instr_buf_r[i] <= mem_rdata_i[i*DAT +: DAT];
                        captured_r[i]  <= 1'b1;
                    end
                end
            end

            if (crash_i) begin
                pc_r         <= crash_pc_i;
                crash_pend_r <= 1'b1;
                for (i = 0; i < IP; i = i + 1)
                    if (!(requested_r[i] & ~captured_r[i])) begin
                        requested_r[i] <= 1'b0; // no outstanding arbiter txn, safe to drop now
                        captured_r[i]  <= 1'b0; // also drop any already-captured data from this group
                    end
                // ports mid-transaction stay requested_r=1, drained above on future cycles
            end else begin

                if (crash_pend_r && all_drained)
                    crash_pend_r <= 1'b0;

                // issue a fresh group once fully idle
                if (idle && !crash_pend_r) begin
                    for (i = 0; i < IP; i = i + 1) begin
                        requested_r[i] <= 1'b1;
                        captured_r[i]  <= 1'b0;
                        pc_buf_r[i]    <= slot_pc[i];
                    end
                end

                // commit completed group to downstream
                if (group_complete && ready_i) begin
                    pc_r <= pc_next_r;
                    for (i = 0; i < IP; i = i + 1) begin
                        requested_r[i] <= 1'b0;
                        captured_r[i]  <= 1'b0;
                    end
                end
            end
        end
    end

endmodule