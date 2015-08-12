module registerSet( input clk, input reset, input regWrite, input [15:0] decOut, input [15:0] writeData, output [15:0] outR0, output [15:0] outR1, output [15:0] outR2, output [15:0] outR3, output [15:0] outR4, output [15:0] outR5, output [15:0] outR6, output [15:0] outR7, output [15:0] outR8, output [15:0] outR9, output [15:0] outR10, output [15:0] outR11, output [15:0] outR12, output [15:0] outR13, output [15:0] outR14, output [15:0] outR15);

register16bit r00(clk, reset, regWrite, decOut[0], writeData, outR0);
register16bit r01(clk, reset, regWrite, decOut[1], writeData, outR1);
register16bit r02(clk, reset, regWrite, decOut[2], writeData, outR2);
register16bit r03(clk, reset, regWrite, decOut[3], writeData, outR3);
register16bit r04(clk, reset, regWrite, decOut[4], writeData, outR4);
register16bit r05(clk, reset, regWrite, decOut[5], writeData, outR5);
register16bit r06(clk, reset, regWrite, decOut[6], writeData, outR6);
register16bit r07(clk, reset, regWrite, decOut[7], writeData, outR7);
register16bit r08(clk, reset, regWrite, decOut[8], writeData, outR8);
register16bit r09(clk, reset, regWrite, decOut[9], writeData, outR9);
register16bit r10(clk, reset, regWrite, decOut[10], writeData, outR10);
register16bit r11(clk, reset, regWrite, decOut[11], writeData, outR11);
register16bit r12(clk, reset, regWrite, decOut[12], writeData, outR12);
register16bit r13(clk, reset, regWrite, decOut[13], writeData, outR13);
register16bit r14(clk, reset, regWrite, decOut[14], writeData, outR14);
register16bit r15(clk, reset, regWrite, decOut[15], writeData, outR15);

endmodule