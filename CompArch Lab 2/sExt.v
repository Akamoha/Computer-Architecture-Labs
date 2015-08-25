module sExt(input [3:0] in, output reg [7:0] out);

  always @(in) begin
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    out[3] = in[3];
    out[4] = in[3];
    out[5] = in[3];
    out[6] = in[3];
    out[7] = in[3];
  end

endmodule