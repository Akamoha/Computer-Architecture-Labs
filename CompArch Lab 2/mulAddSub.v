module mulAddSub(input clk, input load, input select, input [3:0] multiplier, input [3:0] multiplicand, input [3:0] in3,output [7:0] aluOut);
  
  wire [7:0] sExtOut;
  wire [2:0] cnt;
  wire [8:0] Q;
  wire [3:0] adderRes;

  sExt se(in3, sExtOut);

  rShiftReg rsr(clk, load, multiplier, adderRes, Q);

  adder addr(Q[1], Q[0], Q[8:5], multiplicand, adderRes);

  counter ctr(clk, load, cnt);

  alu ari(select, cnt, Q[8:1], sExtOut, aluOut);

endmodule