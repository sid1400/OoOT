module ls_unit #(
    parameter LP   = 1,
    parameter SP   = 1,
    parameter TAGW = 5,
    parameter DW   = 32,
    parameter ADRB = 12,
    parameter BEN  = 4,
    parameter IDW  = 4
)(
    input clk,
    input rst,

    input  [LP-1:0]       ld_valid_i,
    output [LP-1:0]       ld_ready_o,
    input  [LP-1:0]       ld_fwd_i,
    input  [LP*TAGW-1:0]  ld_tag_i,
    input  [LP*3-1:0]     ld_funct3_i,
    input  [LP*DW-1:0]    ld_addr_i,
    input  [LP*DW-1:0]    ld_fdata_i,

    output [LP-1:0]       wb_valid_o,
    input  [LP-1:0]       wb_ready_i,
    output [LP*TAGW-1:0]  wb_tag_o,
    output [LP*DW-1:0]    wb_data_o,

    input  [SP-1:0]       st_valid_i,
    output [SP-1:0]       st_ready_o,
    input  [SP*DW-1:0]    st_addr_i,
    input  [SP*DW-1:0]    st_data_i,
    input  [SP*3-1:0]     st_funct3_i,
    input  [SP*IDW-1:0]   st_id_i,
    output [SP-1:0]       st_done_valid_o,
    output [SP*IDW-1:0]   st_done_id_o,

    input                 kill_i,
    input  [TAGW-1:0]     kill_head_i,
    input  [TAGW-1:0]     kill_tail_i,

    output [LP*ADRB-1:0]  mem_raddr_o,
    output [LP-1:0]       mem_ren_o,
    input  [LP-1:0]       mem_rvalid_i,
    input  [LP*DW-1:0]    mem_rdata_i,
    output [LP-1:0]       mem_rready_o,

    output [SP*ADRB-1:0]  mem_waddr_o,
    output [SP*DW-1:0]    mem_wdata_o,
    output [SP*BEN-1:0]   mem_wstrb_o,
    output [SP-1:0]       mem_wen_o,
    input  [SP-1:0]       mem_bvalid_i,
    output [SP-1:0]       mem_bready_o
);

    localparam S_IDLE = 2'd0;
    localparam S_REQ  = 2'd1;
    localparam S_WAIT = 2'd2;
    localparam S_OUT  = 2'd3;
    localparam S_DONE = 2'd3;

    function in_circ_range;
        input [TAGW-1:0] idx, hd, tl;
        begin
            if (hd <= tl) in_circ_range = (idx >= hd) && (idx < tl);
            else          in_circ_range = (idx >= hd) || (idx < tl);
        end
    endfunction

    genvar gp, gi;

    generate
        for (gp = 0; gp < LP; gp = gp + 1) begin : g_ld
            reg [1:0]      state;
            reg            dead;
            reg [TAGW-1:0] tag_r;
            reg [2:0]      f3_r;
            reg            fwd_r;
            reg [DW-1:0]   addr_r;
            reg [DW-1:0]   data_r;

            wire kill_now = kill_i & ~in_circ_range(tag_r, kill_head_i, kill_tail_i);

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    state <= S_IDLE;
                    dead  <= 1'b0;
                end else begin
                    case (state)
                        S_IDLE: begin
                            if (ld_valid_i[gp] & ~kill_i) begin
                                tag_r  <= ld_tag_i[gp*TAGW +: TAGW];
                                f3_r   <= ld_funct3_i[gp*3 +: 3];
                                fwd_r  <= ld_fwd_i[gp];
                                addr_r <= ld_addr_i[gp*DW +: DW];
                                data_r <= ld_fdata_i[gp*DW +: DW];
                                dead   <= 1'b0;
                                state  <= ld_fwd_i[gp] ? S_OUT : S_REQ;
                            end
                        end
                        S_REQ: begin
                            state <= kill_now ? S_IDLE : S_WAIT;
                        end
                        S_WAIT: begin
                            if (kill_now) dead <= 1'b1;
                            if (mem_rvalid_i[gp]) begin
                                data_r <= mem_rdata_i[gp*DW +: DW];
                                state  <= (dead | kill_now) ? S_IDLE : S_OUT;
                            end
                        end
                        S_OUT: begin
                            if (kill_now | wb_ready_i[gp]) state <= S_IDLE;
                        end
                    endcase
                end
            end

            wire [3:0] lane_oh;
            assign lane_oh[0] = fwd_r | (addr_r[1:0] == 2'd0);
            assign lane_oh[1] = ~fwd_r & (addr_r[1:0] == 2'd1);
            assign lane_oh[2] = ~fwd_r & (addr_r[1:0] == 2'd2);
            assign lane_oh[3] = ~fwd_r & (addr_r[1:0] == 2'd3);

            wire [4*DW-1:0] lane_m;
            for (gi = 0; gi < 4; gi = gi + 1) begin : g_ln
                assign lane_m[gi*DW +: DW] = (data_r >> (8*gi)) & {DW{lane_oh[gi]}};
            end
            wire [DW-1:0] al;
            btree_or_word #(.N(4), .W(DW)) u_lane (.in(lane_m), .out(al));

            wire sel_w  = (f3_r == 3'b010);
            wire sel_bs = (f3_r == 3'b000);
            wire sel_bu = (f3_r == 3'b100);
            wire sel_hs = (f3_r == 3'b001);
            wire sel_hu = (f3_r == 3'b101);

            wire [5*DW-1:0] res_m;
            assign res_m[0*DW +: DW] = al & {DW{sel_w}};
            assign res_m[1*DW +: DW] = {{(DW-8){al[7]}}, al[7:0]} & {DW{sel_bs}};
            assign res_m[2*DW +: DW] = {{(DW-8){1'b0}}, al[7:0]} & {DW{sel_bu}};
            assign res_m[3*DW +: DW] = {{(DW-16){al[15]}}, al[15:0]} & {DW{sel_hs}};
            assign res_m[4*DW +: DW] = {{(DW-16){1'b0}}, al[15:0]} & {DW{sel_hu}};
            wire [DW-1:0] res;
            btree_or_word #(.N(5), .W(DW)) u_res (.in(res_m), .out(res));

            assign ld_ready_o[gp]                = (state == S_IDLE) & ~kill_i;
            assign mem_ren_o[gp]                 = (state == S_REQ) & ~kill_now;
            assign mem_raddr_o[gp*ADRB +: ADRB]  = addr_r[ADRB+1:2];
            assign mem_rready_o[gp]              = (state == S_WAIT);
            assign wb_valid_o[gp]                = (state == S_OUT) & ~kill_now;
            assign wb_tag_o[gp*TAGW +: TAGW]     = tag_r;
            assign wb_data_o[gp*DW +: DW]        = res;
        end

        for (gp = 0; gp < SP; gp = gp + 1) begin : g_st
            reg [1:0]      state;
            reg [DW-1:0]   addr_r;
            reg [DW-1:0]   data_r;
            reg [2:0]      f3_r;
            reg [IDW-1:0]  id_r;

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    state <= S_IDLE;
                end else begin
                    case (state)
                        S_IDLE: begin
                            if (st_valid_i[gp]) begin
                                addr_r <= st_addr_i[gp*DW +: DW];
                                data_r <= st_data_i[gp*DW +: DW];
                                f3_r   <= st_funct3_i[gp*3 +: 3];
                                id_r   <= st_id_i[gp*IDW +: IDW];
                                state  <= S_REQ;
                            end
                        end
                        S_REQ: begin
                            state <= S_WAIT;
                        end
                        S_WAIT: begin
                            if (mem_bvalid_i[gp]) state <= S_DONE;
                        end
                        S_DONE: begin
                            state <= S_IDLE;
                        end
                    endcase
                end
            end

            wire sz_b = (f3_r[1:0] == 2'b00);
            wire sz_h = (f3_r[1:0] == 2'b01);
            wire sz_w = (f3_r[1:0] == 2'b10);
            wire [1:0] o = addr_r[1:0];

            wire [6:0] sel;
            assign sel[0] = sz_w;
            assign sel[1] = sz_h & ~o[1];
            assign sel[2] = sz_h &  o[1];
            assign sel[3] = sz_b & (o == 2'd0);
            assign sel[4] = sz_b & (o == 2'd1);
            assign sel[5] = sz_b & (o == 2'd2);
            assign sel[6] = sz_b & (o == 2'd3);

            wire [7*BEN-1:0] be_m;
            assign be_m[0*BEN +: BEN] = 4'b1111 & {BEN{sel[0]}};
            assign be_m[1*BEN +: BEN] = 4'b0011 & {BEN{sel[1]}};
            assign be_m[2*BEN +: BEN] = 4'b1100 & {BEN{sel[2]}};
            assign be_m[3*BEN +: BEN] = 4'b0001 & {BEN{sel[3]}};
            assign be_m[4*BEN +: BEN] = 4'b0010 & {BEN{sel[4]}};
            assign be_m[5*BEN +: BEN] = 4'b0100 & {BEN{sel[5]}};
            assign be_m[6*BEN +: BEN] = 4'b1000 & {BEN{sel[6]}};
            wire [BEN-1:0] be;
            btree_or_word #(.N(7), .W(BEN)) u_be (.in(be_m), .out(be));

            wire [3*DW-1:0] wd_m;
            assign wd_m[0*DW +: DW] = data_r & {DW{sz_w}};
            assign wd_m[1*DW +: DW] = {data_r[DW/2-1:0], data_r[DW/2-1:0]} & {DW{sz_h}};
            assign wd_m[2*DW +: DW] = {(DW/8){data_r[7:0]}} & {DW{sz_b}};
            wire [DW-1:0] wd;
            btree_or_word #(.N(3), .W(DW)) u_wd (.in(wd_m), .out(wd));

            assign st_ready_o[gp]                = (state == S_IDLE);
            assign mem_wen_o[gp]                 = (state == S_REQ);
            assign mem_waddr_o[gp*ADRB +: ADRB]  = addr_r[ADRB+1:2];
            assign mem_wdata_o[gp*DW +: DW]      = wd;
            assign mem_wstrb_o[gp*BEN +: BEN]    = be;
            assign mem_bready_o[gp]              = (state == S_WAIT);
            assign st_done_valid_o[gp]           = (state == S_DONE);
            assign st_done_id_o[gp*IDW +: IDW]   = id_r;
        end
    endgenerate

endmodule