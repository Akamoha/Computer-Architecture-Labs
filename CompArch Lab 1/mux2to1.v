module mux2to1( input [3:0] rd1, input [3:0] rd2, input Sel, output reg [3:0] desReg );

always @(Sel or rd1 or rd2) begin
case(Sel)
	1'b0: begin
	    desReg = rd1;
	end

	1'b1: begin
	    desReg = rd2;
	end
endcase
end

endmodule