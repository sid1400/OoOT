module branch_predictor #(
    parameter N = 8,
    parameter L = 8,
    parameter NUM_READ_PORTS = 2,
    parameter NUM_WRITE_PORTS = 2
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [31:0] addr_r   [NUM_READ_PORTS],
    input  logic         valid_r [NUM_READ_PORTS],
    output logic         pred_o       [NUM_READ_PORTS],
    output logic         pred_valid_o [NUM_READ_PORTS],

    input  logic [31:0] addr_w   [NUM_WRITE_PORTS],
    input  logic         valid_w [NUM_WRITE_PORTS],
    input  logic         learn_w [NUM_WRITE_PORTS]
);

    localparam DEPTH = 1 << N;

    typedef struct packed {
        logic [L-1:0] tag;
        logic [1:0]   cnt;
    } entry_t;

    entry_t mem [DEPTH];

    logic [N-1:0] ridx [NUM_READ_PORTS];
    logic [L-1:0] rtag [NUM_READ_PORTS];
    logic         rmatch [NUM_READ_PORTS];

    logic [N-1:0] widx [NUM_WRITE_PORTS];
    logic [L-1:0] wtag [NUM_WRITE_PORTS];
    logic         wmatch [NUM_WRITE_PORTS];

    genvar gi;

    generate
        for (gi = 0; gi < NUM_READ_PORTS; gi++) begin : RD
            assign ridx[gi] = addr_r[gi][N+1:2];
            assign rtag[gi] = addr_r[gi][N+L+1:N+2];
            assign rmatch[gi] = (mem[ridx[gi]].tag == rtag[gi]);

            always_ff @(posedge clk) begin
                if (!rst_n) begin
                    pred_valid_o[gi] <= 1'b0;
                    pred_o[gi] <= 1'b0;
                end else begin
                    pred_valid_o[gi] <= valid_r[gi];
                    if (valid_r[gi])
                        pred_o[gi] <= rmatch[gi] ? mem[ridx[gi]].cnt[1] : 1'b0;
                end
            end
        end

        for (gi = 0; gi < NUM_WRITE_PORTS; gi++) begin : WR
            assign widx[gi] = addr_w[gi][N+1:2];
            assign wtag[gi] = addr_w[gi][N+L+1:N+2];
            assign wmatch[gi] = (mem[widx[gi]].tag == wtag[gi]);
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < DEPTH; i++) begin
                mem[i].tag <= '0;
                mem[i].cnt <= 2'b00;
            end
        end else begin
            for (int idx = 0; idx < DEPTH; idx++) begin
                logic reset_done;
                reset_done = 1'b0;

                for (int r = 0; r < NUM_READ_PORTS; r++) begin
                    if (!reset_done && valid_r[r] && (ridx[r] == idx) && !rmatch[r]) begin
                        mem[idx].tag <= rtag[r];
                        mem[idx].cnt <= 2'b00;
                        reset_done = 1'b1;
                    end
                end

                if (!reset_done) begin
                    for (int w = 0; w < NUM_WRITE_PORTS; w++) begin
                        if (!reset_done && valid_w[w] && (widx[w] == idx) && wmatch[w]) begin
                            if (learn_w[w]) begin
                                if (mem[idx].cnt != 2'b11)
                                    mem[idx].cnt <= mem[idx].cnt + 2'b01;
                            end else begin
                                if (mem[idx].cnt != 2'b00)
                                    mem[idx].cnt <= mem[idx].cnt - 2'b01;
                            end
                            reset_done = 1'b1;
                        end
                    end
                end
            end
        end
    end

endmodule