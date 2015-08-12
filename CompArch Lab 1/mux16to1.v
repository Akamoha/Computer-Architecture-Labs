module mux16to1( input [15:0] outR0, input [15:0] outR1, input [15:0] outR2, input [15:0] outR3, input [15:0] outR4, input [15:0] outR5, input [15:0] outR6, input [15:0] outR7, input [15:0] outR8, input [15:0] outR9, input [15:0] outR10, input [15:0] outR11, input [15:0] outR12, input [15:0] outR13, input [15:0] outR14, input [15:0] outR15, input [3:0] Sel, output reg [15:0] outBus );

always @(Sel or outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15) begin
case(Sel)
	4'b0000: begin
		outBus = outR0;
	end
	
	4'b0001: begin
		outBus = outR1;
	end
	
	4'b0010: begin
		outBus = outR2;
	end
	
	4'b0011: begin
		outBus = outR3;
	end
	
	4'b0100: begin
		outBus = outR4;
	end
	
	4'b0101: begin
		outBus = outR5;
	end
	
	4'b0110: begin
		outBus = outR6;
	end
	
	4'b0111: begin
		outBus = outR7;
	end
	
	4'b1000: begin
		outBus = outR8;
	end
	
	4'b1001: begin
		outBus = outR9;
	end
	
	4'b1010: begin
		outBus = outR10;
	end
	
	4'b1011: begin
		outBus = outR11;
	end
	
	4'b1100: begin
		outBus = outR12;
	end
	
	4'b1101: begin
		outBus = outR13;
	end
	
	4'b1110: begin
		outBus = outR14;
	end
	
	4'b1111: begin
		outBus = outR15;
	end
	
endcase
end

endmodule