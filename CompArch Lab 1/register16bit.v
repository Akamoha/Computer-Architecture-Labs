module register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output [15:0] outR );

D_ff d00(clk, reset, regWrite, decOut1b , writeData[0], outR[0]);
D_ff d01(clk, reset, regWrite, decOut1b , writeData[1], outR[1]);
D_ff d02(clk, reset, regWrite, decOut1b , writeData[2], outR[2]);
D_ff d03(clk, reset, regWrite, decOut1b , writeData[3], outR[3]);
D_ff d04(clk, reset, regWrite, decOut1b , writeData[4], outR[4]);
D_ff d05(clk, reset, regWrite, decOut1b , writeData[5], outR[5]);
D_ff d06(clk, reset, regWrite, decOut1b , writeData[6], outR[6]);
D_ff d07(clk, reset, regWrite, decOut1b , writeData[7], outR[7]);
D_ff d08(clk, reset, regWrite, decOut1b , writeData[8], outR[8]);
D_ff d09(clk, reset, regWrite, decOut1b , writeData[9], outR[9]);
D_ff d10(clk, reset, regWrite, decOut1b , writeData[10], outR[10]);
D_ff d11(clk, reset, regWrite, decOut1b , writeData[11], outR[11]);
D_ff d12(clk, reset, regWrite, decOut1b , writeData[12], outR[12]);
D_ff d13(clk, reset, regWrite, decOut1b , writeData[13], outR[13]);
D_ff d14(clk, reset, regWrite, decOut1b , writeData[14], outR[14]);
D_ff d15(clk, reset, regWrite, decOut1b , writeData[15], outR[15]);

endmodule