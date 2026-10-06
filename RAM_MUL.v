`include "RAM_SINGLE.v"

module arbiter #(
    parameter DAT     = 8,
    parameter ADRB    = 10,
    parameter LINEL   = 8,
    parameter TAGL_L1 = 3,
    parameter TAGL_L2 = 3,
    parameter RP      = 2,
    parameter WP      = 2,
    parameter BEN     = 1
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire [RP*ADRB-1:0]       raddr,
    input  wire [RP-1:0]            ren,
    output wire [RP-1:0]            rvalid,
    output wire [RP*DAT-1:0]        rdata,
    input  wire [RP-1:0]            rready,
    input  wire [WP*ADRB-1:0]       waddr,
    input  wire [WP*DAT-1:0]        wdata,
    input  wire [WP*BEN-1:0]        wstrb,
    input  wire [WP-1:0]            wen,
    output wire [WP-1:0]            bvalid,
    input  wire [WP-1:0]            bready
);
function integer CLOG2;
    input integer value;
    integer v;
    begin
        v = value - 1;
        for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
            v = v >> 1;
    end
endfunction

localparam OFFB  = CLOG2(LINEL);
localparam SETS  = ADRB - TAGL_L1 - OFFB;
localparam NPORT = RP + WP;
localparam NP    = 1 << CLOG2(NPORT);
localparam PBITS = CLOG2(NP);
localparam BW    = DAT / BEN;

genvar gi;
wire [ADRB-1:0] raddr_i [0:RP-1];
wire [ADRB-1:0] waddr_i [0:WP-1];
wire [DAT-1:0]  wdata_i [0:WP-1];
wire [BEN-1:0]  wstrb_i [0:WP-1];
generate
    for (gi = 0; gi < RP; gi = gi + 1) begin : UNPACK_R
        assign raddr_i[gi] = raddr[gi*ADRB +: ADRB];
    end
    for (gi = 0; gi < WP; gi = gi + 1) begin : UNPACK_W
        assign waddr_i[gi] = waddr[gi*ADRB +: ADRB];
        assign wdata_i[gi] = wdata[gi*DAT  +: DAT];
        assign wstrb_i[gi] = wstrb[gi*BEN  +: BEN];
    end
endgenerate

reg              pend_r    [0:RP-1];
reg [ADRB-1:0]   lat_raddr [0:RP-1];
reg              rvalid_o  [0:RP-1];
reg [DAT-1:0]    rdata_o   [0:RP-1];

reg              pend_w    [0:WP-1];
reg [ADRB-1:0]   lat_waddr [0:WP-1];
reg [DAT-1:0]    lat_wdata [0:WP-1];
reg [BEN-1:0]    lat_wstrb [0:WP-1];
reg              bvalid_o  [0:WP-1];

generate
    for (gi = 0; gi < RP; gi = gi + 1) begin : R_OUT
        assign rvalid[gi]           = rvalid_o[gi];
        assign rdata[gi*DAT +: DAT] = rdata_o[gi];
    end
    for (gi = 0; gi < WP; gi = gi + 1) begin : W_OUT
        assign bvalid[gi] = bvalid_o[gi];
    end
endgenerate

wire [RP+WP-1:0] active;
generate
    for (gi = 0; gi < RP; gi = gi + 1) begin : ACT_R
        assign active[gi] = pend_r[gi];
    end
    for (gi = 0; gi < WP; gi = gi + 1) begin : ACT_W
        assign active[RP+gi] = pend_w[gi];
    end
endgenerate
wire [NP-1:0] active_padded = { {(NP-NPORT){1'b0}}, active };

reg [PBITS-1:0] cur_port;
wire            cur_valid = active_padded[cur_port];
wire            need_search = mem_done_pulse | ~cur_valid;

wire [PBITS-1:0] shamt = cur_port + 1'b1;
wire [NP-1:0] shift_stage [0:PBITS];
assign shift_stage[0] = active_padded;
genvar k;
generate
    for (k = 0; k < PBITS; k = k + 1) begin : SHIFT_STAGES
        assign shift_stage[k+1] = shamt[k]
            ? { shift_stage[k][(1<<k)-1:0], shift_stage[k][NP-1:(1<<k)] }
            : shift_stage[k];
    end
endgenerate
wire [NP-1:0] shifted = shift_stage[PBITS];

reg [PBITS-1:0] dist;
reg             any_active;
integer j;
always @* begin
    dist       = {PBITS{1'b0}};
    any_active = 1'b0;
    for (j = 0; j < NP; j = j + 1) begin
        if (!any_active && shifted[j]) begin
            dist       = j[PBITS-1:0] + 1'b1;
            any_active = 1'b1;
        end
    end
end

wire [ADRB-1:0] cur_addr = (cur_port < RP) ? lat_raddr[cur_port] : lat_waddr[cur_port-RP];
wire [TAGL_L1-1:0] cur_tag = cur_addr[ADRB-1 -: TAGL_L1];
wire [SETS-1:0]    cur_set = cur_addr[ADRB-TAGL_L1-1 -: SETS];

wire [RP-1:0] match_r;
wire [WP-1:0] match_w;
generate
    for (gi = 0; gi < RP; gi = gi + 1) begin : MATCH_R
        assign match_r[gi] = pend_r[gi] &&
            (lat_raddr[gi][ADRB-1 -: TAGL_L1] == cur_tag) &&
            (lat_raddr[gi][ADRB-TAGL_L1-1 -: SETS] == cur_set);
    end
    for (gi = 0; gi < WP; gi = gi + 1) begin : MATCH_W
        assign match_w[gi] = pend_w[gi] &&
            (lat_waddr[gi][ADRB-1 -: TAGL_L1] == cur_tag) &&
            (lat_waddr[gi][ADRB-TAGL_L1-1 -: SETS] == cur_set);
    end
endgenerate

reg [LINEL*BEN-1:0] batch_wr_mask;
reg [LINEL*DAT-1:0] batch_wr_data;
integer wi, bi;
always @* begin
    batch_wr_mask = {(LINEL*BEN){1'b0}};
    batch_wr_data = {(LINEL*DAT){1'b0}};
    for (wi = 0; wi < WP; wi = wi + 1) begin
        if (match_w[wi]) begin
            for (bi = 0; bi < BEN; bi = bi + 1) begin
                if (lat_wstrb[wi][bi]) begin
                    batch_wr_mask[ lat_waddr[wi][OFFB-1:0]*BEN + bi ] = 1'b1;
                    batch_wr_data[ lat_waddr[wi][OFFB-1:0]*DAT + bi*BW +: BW ] = lat_wdata[wi][bi*BW +: BW];
                end
            end
        end
    end
end

reg                  mem_en;
reg [ADRB-1:0]       mem_addr;
reg [LINEL*BEN-1:0]  mem_wr_mask;
reg [LINEL*DAT-1:0]  mem_wr_data;
wire [LINEL*DAT-1:0] mem_rd_data;
wire                 mem_done;
reg                  mem_done_pulse;

single_access_mem #(
    .DAT(DAT), .ADRB(ADRB), .LINEL(LINEL),
    .TAGL_L1(TAGL_L1), .TAGL_L2(TAGL_L2), .BEN(BEN)
) u_mem (
    .clk(clk), .rst(rst),
    .addr(mem_addr), .wr_mask(mem_wr_mask), .wr_data(mem_wr_data),
    .rd_data(mem_rd_data), .en(mem_en), .done(mem_done)
);

localparam S_IDLE = 1'd0, S_WAIT = 1'd1;
reg s_state;

reg [RP-1:0] lat_match_r;
reg [WP-1:0] lat_match_w;
reg [ADRB-1:0] lat_off_r [0:RP-1];
reg [ADRB-1:0] lat_off_w [0:WP-1];

integer p;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        cur_port       <= {PBITS{1'b0}};
        s_state        <= S_IDLE;
        mem_en         <= 1'b0;
        mem_done_pulse <= 1'b0;
        for (p = 0; p < RP; p = p + 1) begin
            pend_r[p]   <= 1'b0;
            rvalid_o[p] <= 1'b0;
        end
        for (p = 0; p < WP; p = p + 1) begin
            pend_w[p]   <= 1'b0;
            bvalid_o[p] <= 1'b0;
        end
    end else begin
        mem_en         <= 1'b0;
        mem_done_pulse <= 1'b0;

        for (p = 0; p < RP; p = p + 1) begin
            if (ren[p] && !pend_r[p] && !rvalid_o[p]) begin
                pend_r[p]    <= 1'b1;
                lat_raddr[p] <= raddr_i[p];
            end
            if (rvalid_o[p] && rready[p])
                rvalid_o[p] <= 1'b0;
        end
        for (p = 0; p < WP; p = p + 1) begin
            if (wen[p] && !pend_w[p] && !bvalid_o[p]) begin
                pend_w[p]    <= 1'b1;
                lat_waddr[p] <= waddr_i[p];
                lat_wdata[p] <= wdata_i[p];
                lat_wstrb[p] <= wstrb_i[p];
            end
            if (bvalid_o[p] && bready[p])
                bvalid_o[p] <= 1'b0;
        end

        if (need_search && any_active)
            cur_port <= cur_port + dist;

        case (s_state)
        S_IDLE: begin
            if (cur_valid) begin
                mem_addr    <= cur_addr;
                mem_wr_mask <= batch_wr_mask;
                mem_wr_data <= batch_wr_data;
                mem_en      <= 1'b1;
                lat_match_r <= match_r;
                lat_match_w <= match_w;
                for (p = 0; p < RP; p = p + 1)
                    lat_off_r[p] <= lat_raddr[p];
                for (p = 0; p < WP; p = p + 1)
                    lat_off_w[p] <= lat_waddr[p];
                s_state <= S_WAIT;
            end
        end
        S_WAIT: begin
            if (mem_done) begin
                for (p = 0; p < RP; p = p + 1) begin
                    if (lat_match_r[p]) begin
                        rdata_o[p]  <= mem_rd_data[ lat_off_r[p][OFFB-1:0]*DAT +: DAT ];
                        rvalid_o[p] <= 1'b1;
                        pend_r[p]   <= 1'b0;
                    end
                end
                for (p = 0; p < WP; p = p + 1) begin
                    if (lat_match_w[p]) begin
                        bvalid_o[p] <= 1'b1;
                        pend_w[p]   <= 1'b0;
                    end
                end
                mem_done_pulse <= 1'b1;
                s_state        <= S_IDLE;
            end
        end
        endcase
    end
end

endmodule