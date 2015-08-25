module counter(input clk, input load, output [2:0] cnt);

  reg [2:0] dffinput;

  D_ff_cnt d2(dffinput[2], cnt[2]);
  D_ff_cnt d1(dffinput[1], cnt[1]);
  D_ff_cnt d0(dffinput[0], cnt[0]);

  always @ (negedge clk) begin

    if(load) begin
      dffinput = 3'b100;
    end

    if(~load) begin
      dffinput = dffinput - 3'b001;
    end

  end

endmodule