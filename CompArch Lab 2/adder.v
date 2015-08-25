module adder(input q1, input q0, input [3:0] A, input [3:0] multiplicand, output reg [3:0] adderOut);

  always @(q1, q0, A, multiplicand) begin
    if(q1&(~q0)) begin
      adderOut = A - multiplicand;
    end

    if(q0&(~q1)) begin
      adderOut = A + multiplicand;
    end

    if(q0&q1) begin
      adderOut = A;
    end

    if((~q0)&(~q1)) begin
      adderOut = A;
    end
  end

endmodule