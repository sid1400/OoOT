//`include "REGY0.v"
// ROBY: reorder buffer with split addr+valid / data+exception memories,
// built on top of the shared multi-port `regs` primitive.

// one detail - this implement currently rejects all entries at the point of a crash
//the crash FSM has to be handled
// Although we may be able to keep it as is, simply cuz looking at the FSM, even if we write out of range
//nothing happens because its still out of range, and wont pop up.
//actuallu wait, we need to allow it to write...
module ROBY #(
    parameter S     = 32,  // number of ROB entries (power of 2)
    parameter DAT   = 32,  // data field width
    parameter OP    = 3,   // commit (output) ports
    parameter SP    = 3,   // push (set) ports
    parameter RP    = 3,   // random-access read ports
    parameter WP    = 3,   // writeback ports
    parameter REFW  = 5,   // width of the dest-register index field
    parameter EXPL  = 4,   // exception field width
    parameter CRW   = 4    // external crash-reason field width
)(
    input clk,
    input rst,

    // push side: pre-thermometered, all-or-nothing accept
    input  [SP*REFW-1:0] sp_index_i,
    input  [SP-1:0]       sp_valid_i,
    output                ready_o,

    // commit side: reads out entries starting at head
    input                  op_ready_i,
    output [OP*DAT-1:0]    op_data_o,
    output [OP*REFW-1:0]   op_index_o,
    output [OP-1:0]        op_valid_o,   // thermometer-coded

    // random read side
    input  [RP*ADDW-1:0]  r_addr_i,
    output [RP*DAT-1:0]   r_data_o,
    output [RP*REFW-1:0]  r_index_o,
    output [RP-1:0]       r_valid_o,
    output [RP*EXPL-1:0]  r_exception_o,

    // writeback side
    input  [WP*ADDW-1:0]  w_addr_i,
    input  [WP*DAT-1:0]   w_data_i,
    input  [WP*EXPL-1:0]  w_exception_i,
    input  [WP-1:0]       w_en_i,

    output [ADDW-1:0] head_ptr_o,
    output [ADDW-1:0] tail_ptr_o,

    // external crash (branch mispredict) input
    input               crash_i,
    input  [ADDW-1:0]   crash_tail_i,
    input  [CRW-1:0]    crash_reason_i,

    // crash output to reset/kill unit
    output              o_crash,
    output [ADDW-1:0]   o_crash_tail,
    output              o_crash_src,      // 1 = external, 0 = internal exception
    output [CRW-1:0]    o_crash_reason,
    output [EXPL-1:0]   o_exception       // exception code of the excepting head entry
);

    localparam ADDW = $clog2(S);
    localparam AVW  = REFW + 1;   // addr+valid memory word: {valid, index}
    localparam DEW  = DAT + EXPL; // data+exception memory word: {exception, data}
    localparam WPT  = WP + SP;    // total write ports into the addr+valid mem
    localparam RPT  = RP + OP;    // total read ports (explicit + commit window)

    reg [ADDW-1:0] head;
    reg [ADDW-1:0] tail;
    reg [ADDW:0]   count;

    assign head_ptr_o = head;
    assign tail_ptr_o = tail;

    // circular "is idx currently a live entry between head and tail" check
    function in_range;
        input [ADDW-1:0] idx;
        input [ADDW-1:0] hd;
        input [ADDW-1:0] tl;
        begin
            if (hd < tl)
                in_range = (idx >= hd) && (idx < tl);
            else if (hd == tl)
                if (count == S) in_range = 1;
                else in_range = 0;
            else
                in_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    // only accept a new push burst if a full SP worth of space exists
    wire [ADDW:0] free_count = S - count;
    assign ready_o = (free_count >= SP);

    wire external_crash = crash_i;
    wire internal_crash;
    wire crash = external_crash | internal_crash;

    // gate push inputs: no partial pushes, and never push on a crash cycle
    wire [SP-1:0] eff_valid = sp_valid_i & {SP{ready_o}} & {SP{~crash}};

    // count how many pushes are actually happening this cycle, to advance tail/count
    integer pj;
    reg [ADDW:0] n_push;
    always @* begin
        n_push = {(ADDW+1){1'b0}};
        for (pj = 0; pj < SP; pj = pj + 1)
            n_push = n_push + eff_valid[pj];
    end

    // push port j always targets tail+j (unique by construction, no arbitration needed)
    wire [ADDW-1:0] push_addr [0:SP-1];
    genvar gj;
    generate
        for (gj = 0; gj < SP; gj = gj + 1) begin : g_push_addr
            assign push_addr[gj] = tail + gj[ADDW-1:0];
        end
    endgenerate

    // commit port p always reads head+p
    wire [ADDW-1:0] op_addr [0:OP-1];
    generate
        for (gj = 0; gj < OP; gj = gj + 1) begin : g_op_addr
            assign op_addr[gj] = head + gj[ADDW-1:0];
        end
    endgenerate

    // ---- addr+valid memory write bus: WP writeback ports + SP push ports ----
    // writeback ports sit at low indices (mask off index bits, only set valid=1)
    // push ports sit at high indices (unmasked, write both fields) so, per the
    // regs module's "higher port wins" rule, a push always beats a stale
    // writeback hitting the same slot in the same cycle
    wire [WPT-1:0]      av_we;
    wire [WPT*ADDW-1:0] av_waddr;
    wire [WPT*AVW-1:0]  av_wdata;
    wire [WPT*AVW-1:0]  av_wmask;

    generate
        for (gj = 0; gj < WP; gj = gj + 1) begin : g_av_wport
            assign av_we[gj]                    = w_en_i[gj];
            assign av_waddr[gj*ADDW +: ADDW]     = w_addr_i[gj*ADDW +: ADDW];
            assign av_wdata[gj*AVW +: AVW]       = {1'b1, {REFW{1'b0}}};   // set valid=1
            assign av_wmask[gj*AVW +: AVW]       = {1'b0, {REFW{1'b1}}};   // index bits masked off
        end
        for (gj = 0; gj < SP; gj = gj + 1) begin : g_av_pport
            assign av_we[WP+gj]                  = eff_valid[gj];
            assign av_waddr[(WP+gj)*ADDW +: ADDW] = push_addr[gj];
            assign av_wdata[(WP+gj)*AVW +: AVW]   = {1'b0, sp_index_i[gj*REFW +: REFW]}; // new index, valid=0
            assign av_wmask[(WP+gj)*AVW +: AVW]   = {AVW{1'b0}};           // full word write
        end
    endgenerate

    // ---- data+exception memory write bus: WP ports only ----
    // push never writes here; data/exception stay garbage until writeback
    wire [WP-1:0]      de_we    = w_en_i;
    wire [WP*ADDW-1:0] de_waddr = w_addr_i;
    wire [WP*DEW-1:0]  de_wmask = {(WP*DEW){1'b0}};
    wire [WP*DEW-1:0]  de_wdata;
    generate
        for (gj = 0; gj < WP; gj = gj + 1) begin : g_de_wport
            assign de_wdata[gj*DEW +: DEW] = {w_exception_i[gj*EXPL +: EXPL], w_data_i[gj*DAT +: DAT]};
        end
    endgenerate

    // shared read-address bus: explicit random reads + the OP commit window
    wire [RPT*ADDW-1:0] raddr_all;
    generate
        for (gj = 0; gj < RP; gj = gj + 1) begin : g_rd_explicit
            assign raddr_all[gj*ADDW +: ADDW] = r_addr_i[gj*ADDW +: ADDW];
        end
        for (gj = 0; gj < OP; gj = gj + 1) begin : g_rd_commit
            assign raddr_all[(RP+gj)*ADDW +: ADDW] = op_addr[gj];
        end
    endgenerate

    wire [RPT*AVW-1:0] av_rdata;
    wire [RPT*DEW-1:0] de_rdata;

    regs #(.S(S), .AW(ADDW), .W(AVW), .RP(RPT), .WP(WPT)) u_regs_av (
        .clk   (clk),
        .rst   (rst),
        .we    (av_we),
        .waddr (av_waddr),
        .wdata (av_wdata),
        .wmask (av_wmask),
        .raddr (raddr_all),
        .rdata (av_rdata)
    );

    regs #(.S(S), .AW(ADDW), .W(DEW), .RP(RPT), .WP(WP)) u_regs_de (
        .clk   (clk),
        .rst   (rst),
        .we    (de_we),
        .waddr (de_waddr),
        .wdata (de_wdata),
        .wmask (de_wmask),
        .raddr (raddr_all),
        .rdata (de_rdata)
    );

    // unpack the explicit random-read ports
    generate
        for (gj = 0; gj < RP; gj = gj + 1) begin : g_rport
            assign r_index_o[gj*REFW +: REFW]     = av_rdata[gj*AVW +: REFW];
            assign r_valid_o[gj]                  = av_rdata[gj*AVW + REFW];
            assign r_data_o[gj*DAT +: DAT]         = de_rdata[gj*DEW +: DAT];
            assign r_exception_o[gj*EXPL +: EXPL] = de_rdata[gj*DEW + DAT +: EXPL];
        end
    endgenerate

    // commit window: build antitherm allow/block buses. Bit OP-1 = head (top
    // priority, matches boiling.v convention), bit 0 = head+(OP-1).
    wire [OP-1:0] allow_bus;
    wire [OP-1:0] block_bus;
    wire [OP-1:0] therm_out;
    wire [$clog2(OP):0] commit_count;
    wire [EXPL-1:0] exc_at_op [0:OP-1];

    generate
        for (gj = 0; gj < OP; gj = gj + 1) begin : g_op
            localparam integer RI = RP + gj;   // this port's slot in the shared read bus
            wire        val_p = av_rdata[RI*AVW + REFW];
            wire [DAT-1:0]  dat_p = de_rdata[RI*DEW +: DAT];
            assign exc_at_op[gj]  = de_rdata[RI*DEW + DAT +: EXPL];

            assign op_data_o[gj*DAT +: DAT]     = dat_p;
            assign op_index_o[gj*REFW +: REFW]  = av_rdata[RI*AVW +: REFW];

            assign allow_bus[OP-1-gj] = val_p & in_range(op_addr[gj], head, tail);
            assign block_bus[OP-1-gj] = (exc_at_op[gj] != {EXPL{1'b0}});
            assign op_valid_o[gj]     = crash ? 1'b0 : therm_out[OP-1-gj]; // force 0 during a crash cycle
        end
    endgenerate

    antitherm #(.PN(OP)) u_antitherm (
        .allow (allow_bus),
        .block (block_bus),
        .out   (therm_out),
        .count (commit_count)
    );

    // internal exception crash: the head entry itself is valid and excepting
    assign internal_crash = allow_bus[OP-1] & block_bus[OP-1];
    assign o_exception     = exc_at_op[0];

    assign o_crash        = crash;
    assign o_crash_src    = external_crash;       // external always takes priority when both fire
    assign o_crash_reason = crash_reason_i;        // pure passthrough, only meaningful on external crash
    assign o_crash_tail   = external_crash ? crash_tail_i : (head + 1'b1);

    wire [ADDW-1:0] tail_push_next     = tail + n_push;
    wire [ADDW-1:0] head_commit_next   = head + commit_count;
    wire [ADDW-1:0] head_internal_next = head + 1'b1;

    reg [ADDW-1:0] head_next;
    reg [ADDW-1:0] tail_next;
    reg [ADDW:0]   count_next;

    always @* begin
        if (external_crash) begin
            // branch misprediction: keep head, snap tail back to the given point
            head_next  = head;
            tail_next  = crash_tail_i;
            count_next = {1'b0, (crash_tail_i - head)};//(actually it does) this code doesnt work if tail > 0. head <0
            //count_next = {1'b0, (crash_tail_i >= head)?(crash_tail_i - head):};
            //it does work because of the whole wrap around, but it does sound very tweaky
        end else if (internal_crash) begin
            // internal exception: step past the excepting entry, drop everything after it
            head_next  = head_internal_next;
            tail_next  = head_internal_next;
            count_next = {(ADDW+1){1'b0}};
        end else begin
            // normal operation: push extends tail, commit advances head
            tail_next  = tail_push_next;
            head_next  = op_ready_i ? head_commit_next : head;
            count_next = count + n_push - (op_ready_i ? {1'b0, commit_count} : {(ADDW+1){1'b0}});
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            head  <= {ADDW{1'b0}};
            tail  <= {ADDW{1'b0}};
            count <= {(ADDW+1){1'b0}};
        end else begin
            head  <= head_next;
            tail  <= tail_next;
            count <= count_next;
        end
    end

endmodule