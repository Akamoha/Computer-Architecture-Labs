module test_registerFile( );
	//inputs
	reg clk,reset,regWrite,select;
	reg [3:0] rs,rt,rd1,rd2;
	reg [15:0] writeData;
 	//outputs
	wire [15:0] outR0;
	wire [15:0] outR1;
	wire [15:0] outR2;
	registerFile uut(clk,reset,regWrite,rs,rt,rd1,rd2,writeData,select,outR0,outR1,outR2);

 	always begin #5 clk=~clk; end
 
 	initial
 	begin
 		clk=0; reset=1; rs=4'd0; rt=4'd1; rd1=4'd10; rd2=4'd2;
 		#5 reset=0; select=1; regWrite=1; rd1=4'd1; writeData=16'd1;
 		#10 rd1=4'd3; writeData=16'd3;
 		#10 rd1=4'd5; writeData=16'd5;
 		#10 rd1=4'd7; writeData=16'd7;
 		#10 rd1=4'd9; writeData=16'd9;
 		#10 select=0; rd2=4'd2;writeData=16'd2;
 		#10 rd2=4'd4;writeData=16'd4;
		#10 rd2=4'd6;writeData=16'd6;
 		#10 rd2=4'd8;writeData=16'd8;
 		#10 rd2=4'd10;writeData=16'd10;
 		#10 regWrite=0; select=1; rs=4'd0; rd1=4'd1; rd1=4'd3; rd2=4'd2;
 		#10 rs=4'd3;rt=4'd4;rd1=4'd6; rd2=4'd5;
 		#10 select=0; rs=4'd6;rt=4'd7;rd1=4'd8; rd2=4'd9;
 		#10 $finish;
 	end
endmodule 