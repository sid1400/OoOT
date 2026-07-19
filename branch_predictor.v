module branch_predictor #(
    parameter DEPTH_BITS      = 8,
    parameter TAG_BITS        = 8,
    parameter INSTRL          = 32,
    parameter NUM_READ_PORTS  = 2,
    parameter NUM_WRITE_PORTS = 2
)(
    input  wire clk,
    input  wire rst_n,

    // addr_r[i] = addr_r_flat[i*INSTRL +: INSTRL]
    input  wire [NUM_READ_PORTS*INSTRL-1:0] addr_r_flat,
    input  wire [NUM_READ_PORTS-1:0]        valid_r,
    output wire [NUM_READ_PORTS-1:0]        pred_o,
    output wire [NUM_READ_PORTS-1:0]        pred_valid_o,

    // addr_w[i] = addr_w_flat[i*INSTRL +: INSTRL]
    input  wire [NUM_WRITE_PORTS*INSTRL-1:0] addr_w_flat,
    input  wire [NUM_WRITE_PORTS-1:0]        valid_w,
    input  wire [NUM_WRITE_PORTS-1:0]        learn_w
);

    localparam DEPTH = 1 << DEPTH_BITS;

    // struct entry_t {tag, cnt} split into two memories
    reg [TAG_BITS-1:0] mem_tag [0:DEPTH-1];
    reg [1:0]          mem_cnt [0:DEPTH-1];

    wire [DEPTH_BITS-1:0] ridx   [0:NUM_READ_PORTS-1];
    wire [TAG_BITS-1:0]   rtag   [0:NUM_READ_PORTS-1];
    wire                  rmatch [0:NUM_READ_PORTS-1];

    wire [DEPTH_BITS-1:0] widx   [0:NUM_WRITE_PORTS-1];
    wire [TAG_BITS-1:0]   wtag   [0:NUM_WRITE_PORTS-1];
    wire                  wmatch [0:NUM_WRITE_PORTS-1];

    reg pred_o_r       [0:NUM_READ_PORTS-1];
    reg pred_valid_o_r [0:NUM_READ_PORTS-1];

    genvar gi;

    generate
        for (gi = 0; gi < NUM_READ_PORTS; gi = gi + 1) begin : RD
            assign ridx[gi]   = addr_r_flat[gi*INSTRL + DEPTH_BITS+1 : gi*INSTRL + 2];
            assign rtag[gi]   = addr_r_flat[gi*INSTRL + DEPTH_BITS+TAG_BITS+1 : gi*INSTRL + DEPTH_BITS+2];
            assign rmatch[gi] = (mem_tag[ridx[gi]] == rtag[gi]);

            assign pred_o[gi]       = pred_o_r[gi];
            assign pred_valid_o[gi] = pred_valid_o_r[gi];

            always @(posedge clk) begin
                if (!rst_n) begin
                    pred_valid_o_r[gi] <= 1'b0;
                    pred_o_r[gi]       <= 1'b0;
                end else begin
                    pred_valid_o_r[gi] <= valid_r[gi];
                    if (valid_r[gi])
                        pred_o_r[gi] <= rmatch[gi] ? mem_cnt[ridx[gi]][1] : 1'b0;
                end
            end
        end

        for (gi = 0; gi < NUM_WRITE_PORTS; gi = gi + 1) begin : WR
            assign widx[gi]   = addr_w_flat[gi*INSTRL + DEPTH_BITS+1 : gi*INSTRL + 2];
            assign wtag[gi]   = addr_w_flat[gi*INSTRL + DEPTH_BITS+TAG_BITS+1 : gi*INSTRL + DEPTH_BITS+2];
            assign wmatch[gi] = (mem_tag[widx[gi]] == wtag[gi]);
        end
    endgenerate

    integer i, idx, r, w;
    reg reset_done;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem_tag[i] <= {TAG_BITS{1'b0}};
                mem_cnt[i] <= 2'b00;
            end
        end else begin
            for (idx = 0; idx < DEPTH; idx = idx + 1) begin
                reset_done = 1'b0;

                for (r = 0; r < NUM_READ_PORTS; r = r + 1) begin
                    if (!reset_done && valid_r[r] && (ridx[r] == idx) && !rmatch[r]) begin
                        mem_tag[idx] <= rtag[r];
                        mem_cnt[idx] <= 2'b00;
                        reset_done = 1'b1;
                    end
                end

                if (!reset_done) begin
                    for (w = 0; w < NUM_WRITE_PORTS; w = w + 1) begin
                        if (!reset_done && valid_w[w] && (widx[w] == idx) && wmatch[w]) begin
                            if (learn_w[w]) begin
                                if (mem_cnt[idx] != 2'b11)
                                    mem_cnt[idx] <= mem_cnt[idx] + 2'b01;
                            end else begin
                                if (mem_cnt[idx] != 2'b00)
                                    mem_cnt[idx] <= mem_cnt[idx] - 2'b01;
                            end
                            reset_done = 1'b1;
                        end
                    end
                end
            end
        end
    end

endmodule