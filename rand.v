`include "boiling.v"
module rand;

    reg clk;
    reg rst;
    reg [3:0]set;
    reg [3:0]cut;
    wire [3:0]out;
    wire [2:0]num;
    wire [3:0] thermed;
    always #5 clk = ~clk;

    antitherm #(.PN(4)) atherm(set,cut,out,num);
    thermometer #(.PN(4)) therm(num,thermed);

    initial begin
        $dumpfile("tb_rand.vcd");
        $dumpvars(0,rand );

        clk =0;rst = 1;
        #10;
        set = 0; cut = 0;
        #10;
        set = 4'b1100; cut = 4'b0000;
        #10;
        set = 4'b1110; cut = 4'b0000;
        #10;
        set = 4'b1101; cut = 4'b0000;
        #10;
        set = 4'b1110; cut = 4'b0100;
        #10;
        set = 4'b1111; cut = 4'b0100;
        #10;
        set = 4'b0100; cut = 4'b0001;
        #10;
        $finish;
    end

endmodule