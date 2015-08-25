module rShiftReg(input clk, input load, input [3:0] multiplier, input [3:0] adderResult, output [8:0] Q);

  D_ff_Shifter d0(clk, load, 1'b0, Q[1], Q[0]);
  D_ff_Shifter d1(clk, load, multiplier[0], Q[2], Q[1]);
  D_ff_Shifter d2(clk, load, multiplier[1], Q[3], Q[2]);
  D_ff_Shifter d3(clk, load, multiplier[2], Q[4], Q[3]);
  D_ff_Shifter d4(clk, load, multiplier[3], adderResult[0], Q[4]);
  D_ff_Shifter d5(clk, load, 1'b0, adderResult[1], Q[5]);
  D_ff_Shifter d6(clk, load, 1'b0, adderResult[2], Q[6]);
  D_ff_Shifter d7(clk, load, 1'b0, adderResult[3], Q[7]);
  D_ff_Shifter d8(clk, load, 1'b0, adderResult[3], Q[8]);

endmodule