module thermometer #(
    parameter PN = 4,
    parameter CW = $clog2(PN) + 1
)(
    input  [CW-1:0] in,
    output [PN-1:0] out
);
    genvar i;
    generate
        for (i = 0; i < PN; i = i + 1) begin : therm_bit
            assign out[i] = (in > (PN-1-i));
        end
    endgenerate
endmodule

module antitherm #(
    parameter PN = 4,
    parameter CW = $clog2(PN) + 1   // just take this as upper estimate, even though the last bit wont be used for non 2 powers
)(
    input      [PN-1:0] allow,
    input      [PN-1:0] block,
    output     [PN-1:0] out,
    output     [CW-1:0] count
);
    wire [PN-1:0] combined = allow & ~block;
    wire [PN-1:0] chain;
 
    genvar gi;
    generate
        for (gi = PN-1; gi >= 0; gi = gi - 1) begin : g_chain
            if (gi == PN-1)
                assign chain[gi] = combined[gi];
            else
                assign chain[gi] = chain[gi+1] & combined[gi];
        end
    endgenerate
 
    assign out = chain;
 
    integer j;
    reg [CW-1:0] count_r;
    always @(*) begin
        count_r = {CW{1'b0}};
        for (j = 0; j < PN; j = j + 1)
            count_r = count_r + chain[j];// does not use binary tree adding
    end
    assign count = count_r;
endmodule