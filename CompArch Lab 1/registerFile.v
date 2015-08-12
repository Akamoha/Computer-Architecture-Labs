module registerFile(input clk, input reset, input regWrite, input [3:0] rs, input [3:0] rt, input [3:0] rd1, input [3:0] rd2, input [15:0] writeData, input select, output [15:0] outR0, output [15:0] outR1, output [15:0] outR2);

wire [3:0] m1out;
wire [3:0] m2out;
wire [15:0] decoderOut;
wire [15:0] regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15;

mux2to1 m1(rd2, rd1, select, m1out);
mux2to1 m2(rd1, rd2, select, m2out);
decoder d1(m1out, decoderOut);
registerSet rs1(clk, reset, regWrite, decoderOut, writeData, regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15);

mux16to1 m3(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15, rs, outR0 );
mux16to1 m4(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15, rt, outR1 );
mux16to1 m5(regOut0, regOut1, regOut2, regOut3, regOut4, regOut5, regOut6, regOut7, regOut8, regOut9, regOut10, regOut11, regOut12, regOut13, regOut14, regOut15, m2out, outR2 );

endmodule