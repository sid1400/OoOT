module mem_backend #(
    parameter DAT   = 8,
    parameter LINEL = 8,
    parameter ADDRW = 7
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire [ADDRW-1:0]      mem_addr,
    input  wire                  mem_ren,
    input  wire                  mem_wen,
    input  wire [DAT*LINEL-1:0]  mem_wdata,
    output reg  [DAT*LINEL-1:0]  mem_rdata,
    output reg                   mem_ready
);

localparam FETCH_DELAY = 10;
localparam CBITS = 4;

reg [DAT*LINEL-1:0] mem_array [0:(1<<ADDRW)-1];

localparam S_IDLE = 0, S_BUSY = 1;
reg              state;
reg [CBITS-1:0]  cnt;
reg              busy_is_write;
reg [ADDRW-1:0]  busy_addr;
reg [DAT*LINEL-1:0] busy_wdata;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state     <= S_IDLE;
        mem_ready <= 1'b0;
        cnt       <= {CBITS{1'b0}};
    end else begin
        mem_ready <= 1'b0;
        case (state)
            S_IDLE: begin
                if (mem_ren || mem_wen) begin
                    busy_is_write <= mem_wen;
                    busy_addr     <= mem_addr;
                    busy_wdata    <= mem_wdata;
                    cnt           <= FETCH_DELAY - 1;
                    state         <= S_BUSY;
                end
            end
            S_BUSY: begin
                if (cnt == {CBITS{1'b0}}) begin
                    if (busy_is_write)
                        mem_array[busy_addr] <= busy_wdata;
                    else
                        mem_rdata <= mem_array[busy_addr];
                    mem_ready <= 1'b1;
                    state     <= S_IDLE;
                end else begin
                    cnt <= cnt - 1'b1;
                end
            end
        endcase
    end
end

endmodule

module cache_l2 #(
    parameter DAT    = 8,
    parameter ADRB   = 10,
    parameter LINEL  = 8,
    parameter TAGL   = 3
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire [ADRB-1:0]          req_addr,
    input  wire                     req_valid,
    output reg                      req_ready,
    output reg  [DAT*LINEL-1:0]     rsp_data,
    output reg                      rsp_dirty,
    output reg                      rsp_valid,
    input  wire [ADRB-1:0]          evict_addr,
    input  wire [DAT*LINEL-1:0]     evict_data,
    input  wire                     evict_dirty,
    input  wire                     evict_valid,
    output reg                      evict_ready,
    output reg  [TAGL+SETS-1:0]     mem_addr,
    output reg                      mem_ren,
    output reg                      mem_wen,
    output reg  [DAT*LINEL-1:0]     mem_wdata,
    input  wire [DAT*LINEL-1:0]     mem_rdata,
    input  wire                     mem_ready
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
localparam SETS  = ADRB - TAGL - OFFB;
localparam NSETS = (1 << SETS);
localparam L2_LAT = 3;
localparam CBITS = (CLOG2(L2_LAT) > 0) ? CLOG2(L2_LAT) : 1;

reg                 valid [0:NSETS-1][0:1];
reg                 dirty [0:NSETS-1][0:1];
reg                 age   [0:NSETS-1][0:1];
reg  [TAGL-1:0]     tag   [0:NSETS-1][0:1];
reg  [DAT-1:0]      data  [0:NSETS-1][0:1][0:LINEL-1];

wire [TAGL-1:0] req_tag = req_addr[ADRB-1 -: TAGL];
wire [SETS-1:0] req_set = req_addr[ADRB-TAGL-1 -: SETS];
wire [TAGL-1:0] evict_tag = evict_addr[ADRB-1 -: TAGL];
wire [SETS-1:0] evict_set = evict_addr[ADRB-TAGL-1 -: SETS];

wire req_hit0 = valid[req_set][0] && (tag[req_set][0] == req_tag);
wire req_hit1 = valid[req_set][1] && (tag[req_set][1] == req_tag);
wire req_hit  = req_hit0 || req_hit1;
wire req_hit_way = req_hit1;

wire e_free0 = !valid[evict_set][0];
wire e_free1 = !valid[evict_set][1];
wire e_any_free = e_free0 || e_free1;
wire e_pick_free = e_free0 ? 1'b0 : 1'b1;

wire e_clean0 = valid[evict_set][0] && !dirty[evict_set][0];
wire e_clean1 = valid[evict_set][1] && !dirty[evict_set][1];
wire e_both_dirty = valid[evict_set][0] && valid[evict_set][1] &&
                     dirty[evict_set][0] && dirty[evict_set][1];
wire e_pick_clean = e_clean0 ? 1'b0 : e_clean1 ? 1'b1 : 1'b0;
wire e_pick_wb    = (age[evict_set][0] == 1'b0) ? 1'b0 : 1'b1;

localparam S_IDLE       = 3'd0,
           S_HIT_WAIT   = 3'd1,
           S_MEM_FETCH  = 3'd2,
           S_RESPOND    = 3'd3,
           S_EV_WB_WAIT = 3'd4,
           S_EV_INSERT  = 3'd5;

reg [2:0] state;
reg [TAGL-1:0] r_tag;
reg [SETS-1:0] r_set;
reg            r_hit_way;
reg [DAT-1:0]  r_line [0:LINEL-1];
reg            r_dirty;
reg [CBITS-1:0] hit_cnt;

reg [TAGL-1:0] ev_tag;
reg [SETS-1:0] ev_set;
reg [DAT-1:0]  ev_line [0:LINEL-1];
reg            ev_dirty;
reg            ev_target_way;
reg            ev_wb_way;

integer m, p;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state       <= S_IDLE;
        req_ready   <= 1'b0;
        rsp_valid   <= 1'b0;
        evict_ready <= 1'b0;
        mem_ren     <= 1'b0;
        mem_wen     <= 1'b0;
        hit_cnt     <= {CBITS{1'b0}};
        for (p = 0; p < NSETS; p = p + 1) begin
            valid[p][0] <= 1'b0; valid[p][1] <= 1'b0;
            dirty[p][0] <= 1'b0; dirty[p][1] <= 1'b0;
            age[p][0]   <= 1'b0; age[p][1]   <= 1'b0;
        end
    end else begin
        req_ready   <= 1'b0;
        rsp_valid   <= 1'b0;
        evict_ready <= 1'b0;
        mem_ren     <= 1'b0;
        mem_wen     <= 1'b0;

        case (state)
        S_IDLE: begin
            if (req_valid) begin
                req_ready <= 1'b1;
                r_tag <= req_tag;
                r_set <= req_set;
                if (req_hit) begin
                    r_hit_way <= req_hit_way;
                    for (m = 0; m < LINEL; m = m + 1)
                        r_line[m] <= data[req_set][req_hit_way][m];
                    r_dirty <= dirty[req_set][req_hit_way];
                    valid[req_set][req_hit_way] <= 1'b0;
                    hit_cnt <= L2_LAT - 1;
                    state <= S_HIT_WAIT;
                end else begin
                    mem_addr <= {req_tag, req_set};
                    mem_ren  <= 1'b1;
                    state    <= S_MEM_FETCH;
                end
            end else if (evict_valid) begin
                evict_ready <= 1'b1;
                ev_tag   <= evict_tag;
                ev_set   <= evict_set;
                ev_dirty <= evict_dirty;
                for (m = 0; m < LINEL; m = m + 1)
                    ev_line[m] <= evict_data[m*DAT +: DAT];

                if (e_any_free) begin
                    ev_target_way <= e_pick_free;
                    state <= S_EV_INSERT;
                end else if (!e_both_dirty) begin
                    ev_target_way <= e_pick_clean;
                    state <= S_EV_INSERT;
                end else begin
                    ev_wb_way <= e_pick_wb;
                    mem_wen   <= 1'b1;
                    mem_addr  <= {tag[evict_set][e_pick_wb], evict_set};
                    for (m = 0; m < LINEL; m = m + 1)
                        mem_wdata[m*DAT +: DAT] <= data[evict_set][e_pick_wb][m];
                    state <= S_EV_WB_WAIT;
                end
            end
        end

        S_HIT_WAIT: begin
            if (hit_cnt == {CBITS{1'b0}}) begin
                for (m = 0; m < LINEL; m = m + 1)
                    rsp_data[m*DAT +: DAT] <= r_line[m];
                rsp_dirty <= r_dirty;
                rsp_valid <= 1'b1;
                state     <= S_IDLE;
            end else begin
                hit_cnt <= hit_cnt - 1'b1;
            end
        end

        S_MEM_FETCH: begin
            if (mem_ready) begin
                rsp_data  <= mem_rdata;
                rsp_dirty <= 1'b0;
                rsp_valid <= 1'b1;
                state     <= S_IDLE;
            end
        end

        S_EV_WB_WAIT: begin
            if (mem_ready) begin
                ev_target_way <= ev_wb_way;
                state <= S_EV_INSERT;
            end
        end

        S_EV_INSERT: begin
            valid[ev_set][ev_target_way] <= 1'b1;
            tag[ev_set][ev_target_way]   <= ev_tag;
            dirty[ev_set][ev_target_way] <= ev_dirty;
            for (m = 0; m < LINEL; m = m + 1)
                data[ev_set][ev_target_way][m] <= ev_line[m];
            age[ev_set][ev_target_way]  <= 1'b1;
            age[ev_set][~ev_target_way] <= 1'b0;
            state <= S_IDLE;
        end
        endcase
    end
end

endmodule

module cache_l1 #(
    parameter DAT   = 8,
    parameter ADRB  = 10,
    parameter LINEL = 8,
    parameter TAGL  = 3,
    parameter BEN   = 1
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire [ADRB-1:0]          addr,
    input  wire [LINEL*BEN-1:0]     wr_mask,
    input  wire [DAT*LINEL-1:0]     wr_data,
    output reg  [DAT*LINEL-1:0]     rd_data,
    input  wire                     en,
    output reg                      done,
    output reg  [ADRB-1:0]          l2_req_addr,
    output reg                      l2_req_valid,
    input  wire                     l2_req_ready,
    input  wire [DAT*LINEL-1:0]     l2_rsp_data,
    input  wire                     l2_rsp_valid,
    input  wire                     l2_rsp_dirty,
    output reg  [ADRB-1:0]          l2_evict_addr,
    output reg  [DAT*LINEL-1:0]     l2_evict_data,
    output reg                      l2_evict_dirty,
    output reg                      l2_evict_valid,
    input  wire                     l2_evict_ready
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
localparam SETS  = ADRB - TAGL - OFFB;
localparam NSETS = (1 << SETS);
localparam BW    = DAT / BEN;

reg                 valid [0:NSETS-1][0:1];
reg                 dirty [0:NSETS-1][0:1];
reg                 age   [0:NSETS-1][0:1];
reg  [TAGL-1:0]     tag   [0:NSETS-1][0:1];
reg  [DAT-1:0]      cdata [0:NSETS-1][0:1][0:LINEL-1];

localparam S_IDLE      = 3'd0,
           S_MISS_WAIT = 3'd1,
           S_EVICT     = 3'd2,
           S_INSERT    = 3'd3,
           S_SERVE     = 3'd4;

reg [2:0] state;
reg [TAGL-1:0]  tgt_tag;
reg [SETS-1:0]  tgt_set;
reg [DAT-1:0]   tgt_line [0:LINEL-1];
reg             tgt_dirty;
reg             tgt_way;

reg [LINEL*BEN-1:0]  op_wr_mask;
reg [DAT*LINEL-1:0]  op_wr_data;

reg             evict_way;
reg [ADRB-1:0]  evict_addr_reg;
reg             req_acked;

integer m, bb;

wire [TAGL-1:0] a_tag = addr[ADRB-1 -: TAGL];
wire [SETS-1:0] a_set = addr[ADRB-TAGL-1 -: SETS];

wire hit0 = valid[a_set][0] && (tag[a_set][0] == a_tag);
wire hit1 = valid[a_set][1] && (tag[a_set][1] == a_tag);
wire l1_hit = hit0 || hit1;
wire hit_way = hit1;

wire free0 = !valid[tgt_set][0];
wire free1 = !valid[tgt_set][1];
wire any_free = free0 || free1;
wire pick_free_way = free0 ? 1'b0 : 1'b1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state          <= S_IDLE;
        done           <= 1'b0;
        l2_req_valid   <= 1'b0;
        l2_evict_valid <= 1'b0;
        req_acked      <= 1'b0;
        for (m = 0; m < NSETS; m = m + 1) begin
            valid[m][0] <= 1'b0; valid[m][1] <= 1'b0;
            dirty[m][0] <= 1'b0; dirty[m][1] <= 1'b0;
            age[m][0]   <= 1'b0; age[m][1]   <= 1'b0;
        end
    end else begin
        done           <= 1'b0;
        l2_evict_valid <= 1'b0;

        case (state)
        S_IDLE: begin
            if (en) begin
                tgt_tag    <= a_tag;
                tgt_set    <= a_set;
                op_wr_mask <= wr_mask;
                op_wr_data <= wr_data;
                if (l1_hit) begin
                    tgt_way <= hit_way;
                    state   <= S_SERVE;
                end else begin
                    l2_req_addr  <= addr;
                    l2_req_valid <= 1'b1;
                    req_acked    <= 1'b0;
                    state        <= S_MISS_WAIT;
                end
            end
        end

        S_MISS_WAIT: begin
            if (!req_acked) begin
                if (l2_req_ready) begin
                    req_acked    <= 1'b1;
                    l2_req_valid <= 1'b0;
                end else begin
                    l2_req_valid <= 1'b1;
                end
            end
            if (l2_rsp_valid) begin
                for (m = 0; m < LINEL; m = m + 1)
                    tgt_line[m] <= l2_rsp_data[m*DAT +: DAT];
                tgt_dirty <= l2_rsp_dirty;

                if (any_free) begin
                    tgt_way <= pick_free_way;
                    state   <= S_INSERT;
                end else begin
                    evict_way      <= (age[tgt_set][0] == 1'b0) ? 1'b0 : 1'b1;
                    evict_addr_reg <= {tag[tgt_set][(age[tgt_set][0]==1'b0)?1'b0:1'b1], tgt_set, {OFFB{1'b0}}};
                    state          <= S_EVICT;
                end
            end
        end

        S_EVICT: begin
            l2_evict_addr  <= evict_addr_reg;
            l2_evict_dirty <= dirty[tgt_set][evict_way];
            for (m = 0; m < LINEL; m = m + 1)
                l2_evict_data[m*DAT +: DAT] <= cdata[tgt_set][evict_way][m];
            l2_evict_valid <= 1'b1;
            if (l2_evict_ready) begin
                tgt_way <= evict_way;
                state   <= S_INSERT;
            end
        end

        S_INSERT: begin
            valid[tgt_set][tgt_way] <= 1'b1;
            tag[tgt_set][tgt_way]   <= tgt_tag;
            dirty[tgt_set][tgt_way] <= tgt_dirty;
            for (m = 0; m < LINEL; m = m + 1)
                cdata[tgt_set][tgt_way][m] <= tgt_line[m];
            age[tgt_set][tgt_way]  <= 1'b1;
            age[tgt_set][~tgt_way] <= 1'b0;
            state <= S_SERVE;
        end

        S_SERVE: begin
            for (m = 0; m < LINEL; m = m + 1) begin
                for (bb = 0; bb < BEN; bb = bb + 1) begin
                    if (op_wr_mask[m*BEN + bb]) begin
                        cdata[tgt_set][tgt_way][m][bb*BW +: BW] <= op_wr_data[m*DAT + bb*BW +: BW];
                        rd_data[m*DAT + bb*BW +: BW]            <= op_wr_data[m*DAT + bb*BW +: BW];
                    end else begin
                        rd_data[m*DAT + bb*BW +: BW]            <= cdata[tgt_set][tgt_way][m][bb*BW +: BW];
                    end
                end
            end
            if (|op_wr_mask)
                dirty[tgt_set][tgt_way] <= 1'b1;
            age[tgt_set][tgt_way]  <= 1'b1;
            age[tgt_set][~tgt_way] <= 1'b0;
            done  <= 1'b1;
            state <= S_IDLE;
        end
        endcase
    end
end

endmodule

module single_access_mem #(
    parameter DAT     = 8,
    parameter ADRB    = 10,
    parameter LINEL   = 8,
    parameter TAGL_L1 = 3,
    parameter TAGL_L2 = 3,
    parameter BEN     = 1
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire [ADRB-1:0]          addr,
    input  wire [LINEL*BEN-1:0]     wr_mask,
    input  wire [DAT*LINEL-1:0]     wr_data,
    output wire [DAT*LINEL-1:0]     rd_data,
    input  wire                     en,
    output wire                     done
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

localparam OFFB    = CLOG2(LINEL);
localparam SETS_L2 = ADRB - TAGL_L2 - OFFB;

wire [ADRB-1:0]      l2_req_addr;
wire                  l2_req_valid;
wire                  l2_req_ready;
wire [DAT*LINEL-1:0]  l2_rsp_data;
wire                  l2_rsp_dirty;
wire                  l2_rsp_valid;

wire [ADRB-1:0]       l2_evict_addr;
wire [DAT*LINEL-1:0]  l2_evict_data;
wire                  l2_evict_dirty;
wire                  l2_evict_valid;
wire                  l2_evict_ready;

wire [TAGL_L2+SETS_L2-1:0] mem_addr;
wire                  mem_ren;
wire                  mem_wen;
wire [DAT*LINEL-1:0]  mem_wdata;
wire [DAT*LINEL-1:0]  mem_rdata;
wire                  mem_ready;

cache_l1 #(
    .DAT(DAT), .ADRB(ADRB), .LINEL(LINEL), .TAGL(TAGL_L1), .BEN(BEN)
) u_l1 (
    .clk(clk), .rst(rst),
    .addr(addr), .wr_mask(wr_mask), .wr_data(wr_data), .rd_data(rd_data), .en(en), .done(done),
    .l2_req_addr(l2_req_addr), .l2_req_valid(l2_req_valid), .l2_req_ready(l2_req_ready),
    .l2_rsp_data(l2_rsp_data), .l2_rsp_valid(l2_rsp_valid), .l2_rsp_dirty(l2_rsp_dirty),
    .l2_evict_addr(l2_evict_addr), .l2_evict_data(l2_evict_data),
    .l2_evict_dirty(l2_evict_dirty), .l2_evict_valid(l2_evict_valid),
    .l2_evict_ready(l2_evict_ready)
);

cache_l2 #(
    .DAT(DAT), .ADRB(ADRB), .LINEL(LINEL), .TAGL(TAGL_L2)
) u_l2 (
    .clk(clk), .rst(rst),
    .req_addr(l2_req_addr), .req_valid(l2_req_valid), .req_ready(l2_req_ready),
    .rsp_data(l2_rsp_data), .rsp_dirty(l2_rsp_dirty), .rsp_valid(l2_rsp_valid),
    .evict_addr(l2_evict_addr), .evict_data(l2_evict_data),
    .evict_dirty(l2_evict_dirty), .evict_valid(l2_evict_valid),
    .evict_ready(l2_evict_ready),
    .mem_addr(mem_addr), .mem_ren(mem_ren), .mem_wen(mem_wen),
    .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(mem_ready)
);

mem_backend #(
    .DAT(DAT), .LINEL(LINEL), .ADDRW(TAGL_L2 + SETS_L2)
) u_mem (
    .clk(clk), .rst(rst),
    .mem_addr(mem_addr), .mem_ren(mem_ren), .mem_wen(mem_wen),
    .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(mem_ready)
);

endmodule