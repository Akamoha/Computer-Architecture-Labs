module D_ff_Shifter(input clk, input load, input init_d , input shift_d, output reg q);
	always @ (negedge clk)
	begin
	if(load==1)
		q=init_d;
	else
		q=shift_d;
	end
endmodule