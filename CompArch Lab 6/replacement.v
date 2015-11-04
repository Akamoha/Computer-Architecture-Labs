module D_FF(input clk,input set,input reset,output reg Q);
	always@(negedge clk) 
		begin
			if(reset==1'b1)
				Q=0;
			else
				begin
					if(set)
						Q=1;
				end
		end
	
endmodule

module decoder2to4(input [1:0] muxOut,output reg[3:0] decOut);
	always@ (muxOut)
	begin
		case (muxOut)
			2'b00 : decOut = 4'b0001;
			2'b01 : decOut = 4'b0010;
			2'b10 : decOut = 4'b0100;
			2'b11 : decOut = 4'b1000;
		endcase
	end
endmodule

module mux(input [1:0] LineIndex,input[1:0] LRUWay,input Hit,output reg [1:0] muxOut);
	always @ (LineIndex or LRUWay or Hit)
	begin
		case (Hit)
			1'b0 : muxOut = LRUWay;
			1'b1 : muxOut = LineIndex;
		endcase
	end
endmodule

module NxN_DFFs(input clk,input reset,input[3:0] decOut,output [3:0] NxNOut);
	
	reg [3:0] reset_sig;
	wire [15:0] q_out;
	
	initial
		reset_sig = 4'b1111;
	always@(decOut or reset)
		begin
		reset_sig[0] = decOut[0] | reset;
		reset_sig[1] = decOut[1] | reset;
		reset_sig[2] = decOut[2] | reset;
		reset_sig[3] = decOut[3] | reset;
		end
	
	D_FF D11(clk,decOut[0],reset_sig[0],q_out[0]);
	D_FF D12(clk,decOut[0],reset_sig[1],q_out[1]);
	D_FF D13(clk,decOut[0],reset_sig[2],q_out[2]);
	D_FF D14(clk,decOut[0],reset_sig[3],q_out[3]);
	
	D_FF D21(clk,decOut[1],reset_sig[0],q_out[4]);
	D_FF D22(clk,decOut[1],reset_sig[1],q_out[5]);
	D_FF D23(clk,decOut[1],reset_sig[2],q_out[6]);
	D_FF D24(clk,decOut[1],reset_sig[3],q_out[7]);
	
	D_FF D31(clk,decOut[2],reset_sig[0],q_out[8]);
	D_FF D32(clk,decOut[2],reset_sig[1],q_out[9]);
	D_FF D33(clk,decOut[2],reset_sig[2],q_out[10]);
	D_FF D34(clk,decOut[2],reset_sig[3],q_out[11]);
	
	D_FF D41(clk,decOut[3],reset_sig[0],q_out[12]);
	D_FF D42(clk,decOut[3],reset_sig[0],q_out[13]);
	D_FF D43(clk,decOut[3],reset_sig[2],q_out[14]);
	D_FF D44(clk,decOut[3],reset_sig[3],q_out[15]);
	
	assign NxNOut[0] = q_out[0] | q_out[1] | q_out[2] | q_out[3];
	assign NxNOut[1] = q_out[4] | q_out[5] | q_out[6] | q_out[7];
	assign NxNOut[2] = q_out[8] | q_out[9] | q_out[10] | q_out[11];
	assign NxNOut[3] = q_out[12] | q_out[13] | q_out[14] | q_out[15];
endmodule

module prio_Enc(input reset, input [3:0]NxNOut,output reg [1:0] LRUWay);
  always@(reset or NxNOut)
  begin
	if (reset == 1'b1)
		LRUWay = 2'b00;
    else if(NxNOut[0] == 1'b0)
      LRUWay = 2'b00;
    else if(NxNOut[1] == 1'b0)
      LRUWay = 2'b01;
    else if(NxNOut[2] == 1'b0)
      LRUWay = 2'b10;
    else if(NxNOut[3] == 1'b0)
      LRUWay = 2'b11;
  end
endmodule

module LRU(input [1:0] LineIndex,input clk,input reset,input Hit,output [1:0] LRUWay,output [1:0] mOut,output [3:0] dOut,output [3:0] nOut);
	mux m1(LineIndex,LRUWay,Hit,mOut);
	decoder2to4 d24(mOut,dOut);
	NxN_DFFs nxn(clk,reset,dOut,nOut);
	prio_Enc penc(reset,nOut,LRUWay);
endmodule
		
module testbench;
	reg [1:0] LineIndex;
	reg clk;
	reg reset;
	reg Hit;
	wire [1:0] LRUWay;
	wire [1:0] mOut;
	wire [3:0] dOut, nOut;
	LRU uut (.LineIndex(LineIndex), .clk(clk), .reset(reset), .Hit(Hit), .LRUWay(LRUWay), .mOut(mOut), .dOut(dOut), .nOut(nOut) );
	always #5 clk=~clk;
	initial
	
	begin 
		LineIndex = 0;
		reset = 1;
		Hit = 0;
		clk = 0;
		$monitor($time," Current_LRUWay=%d Hit=%d LineIndex=%d ",LRUWay,Hit,LineIndex);
		#8 Hit=1;
		#2 reset=0;
		LineIndex=3'd0;
		#10 LineIndex=3'd1;
		#10 LineIndex=3'd2;
		#10 LineIndex=3'd3;
		#10 Hit=0; LineIndex=3'd1;
		#10 LineIndex=3'd0;
		#10 LineIndex=3'd1;
		#10 LineIndex=3'd2;
		#10 LineIndex=3'd3;
		#10 $finish;
	end
endmodule
