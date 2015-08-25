module alu(input select, input [2:0] cnt,input [7:0] in1,input [7:0] in2,output reg [7:0] aluOut);

  always @(cnt) begin
    if((~cnt[0])&(~cnt[1])&(~cnt[2])&select) begin
      aluOut = in1 + in2;
    end

    if((~cnt[0])&(~cnt[1])&(~cnt[2])&(~select)) begin
      aluOut = in1 - in2;
    end
  end

endmodule