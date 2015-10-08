module D_ff(input clk,input reset,input d,output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		q=d;
	end
endmodule

module register16bit(input clk,input reset,input [15:0] numberIn,output [15:0] numberOut);
	D_ff d0(clk, reset, numberIn[0], numberOut[0]);
	D_ff d1(clk, reset, numberIn[1], numberOut[1]);
	D_ff d2(clk, reset, numberIn[2], numberOut[2]);
	D_ff d3(clk, reset, numberIn[3], numberOut[3]);
	D_ff d4(clk, reset, numberIn[4], numberOut[4]);
	D_ff d5(clk, reset, numberIn[5], numberOut[5]);
	D_ff d6(clk, reset, numberIn[6], numberOut[6]);
	D_ff d7(clk, reset, numberIn[7], numberOut[7]);
	D_ff d8(clk, reset, numberIn[8], numberOut[8]);
	D_ff d9(clk, reset, numberIn[9], numberOut[9]);
	D_ff d10(clk, reset, numberIn[10], numberOut[10]);
	D_ff d11(clk, reset, numberIn[11], numberOut[11]);
	D_ff d12(clk, reset, numberIn[12], numberOut[12]);
	D_ff d13(clk, reset, numberIn[13], numberOut[13]);
	D_ff d14(clk, reset, numberIn[14], numberOut[14]);
	D_ff d15(clk, reset, numberIn[15], numberOut[15]);
endmodule

//signDiffExpo finds the diffExpo. diffExpo = number1OutExpo - number2OutExpo
module signDiffExpo(input [4:0] number1OutExpo,input [4:0] number2OutExpo,output reg [5:0] diffExpo);
	reg[5:0] num1, num2;
	
	always@(number1OutExpo, number2OutExpo)
	begin
	  num1 = {1'b0, number1OutExpo};
	  num2 = {1'b0, number2OutExpo};
	  diffExpo = num1 - num2;
	end
endmodule

//signToUnsign6b module finds the absolute value 'absDiff' of signed difference 'diffExpo'.
//If 'diffExpo' is positive sign is set to 0. 
//If 'diffExpo' is negative sign is set to 1.
module signToUnsign6b(input [5:0] diffExpo,output reg sign,output reg [4:0] absDiff);
	reg[5:0] diff6bits;
	
	always@(diffExpo)
	begin
	  if(diffExpo[5])
	    begin
	      sign = 1'b1;
	      diff6bits = (6'b111111 - diffExpo) + 6'b000001;
	      absDiff = diff6bits[4:0];
	    end
	  else
	    begin
	      sign = 1'b0;
	      absDiff = diffExpo[4:0];
	    end
	end
endmodule

module mux2to1_5b(input [4:0] number1OutExpo,input [4:0] number2OutExpo,input sign,output reg[4:0] resultExpo);
	always@(number1OutExpo, number2OutExpo, sign)
	begin
	  case(sign)
	    1'b0: resultExpo = number1OutExpo;
	    1'b1: resultExpo = number2OutExpo;
	  endcase
	end
endmodule

module mux2to1_12b(input [11:0] in1,input [11:0] in2,input selOp,output reg [11:0] Op);
	always@(in1, in2, selOp)
	begin
	  case(selOp)
	    1'b0: Op = in1;
	    1'b1: Op = in2;
	  endcase
	end
endmodule

//shiftRightOp1 module shifts the fraction with smallest exponent to right by factor of 'absDiff'
module shiftRightOp1(input [11:0] tempOp,input [4:0] absDiff,output reg [11:0] shiftOp);
	always@(tempOp, absDiff)
	begin
	  shiftOp = tempOp >> absDiff;
	end
endmodule

//unsignToSign13b module converts unsigned number 'in12bUnsigned' to signed number 'out13bSigned'
module unsignToSign13b(input [11:0] in12bUnsigned,input sign,output reg [12:0] out13bSigned);
  reg[12:0] in13bUnsigned;
  
	always@(in12bUnsigned, sign)
	begin
	  in13bUnsigned = {1'b0, in12bUnsigned};
	  if(sign)
	    begin
	      out13bSigned = (13'b1111111111111 - in13bUnsigned) + 13'b0000000000001;
	    end
	  else
	    begin
	      out13bSigned = in13bUnsigned;
	    end
	end
endmodule

module mux2to1_1b(input s1,input s2,input selSign,output reg resultSign);
	always@(s1, s2, selSign)
	begin
	  case(selSign)
	    1'b0: resultSign = s1;
	    1'b1: resultSign = s2;
	  endcase
	end
endmodule

//bigALU module adds two numbers 'ALUin1' and 'ALUin2'
module bigALU(input [12:0] ALUin1,input [12:0] ALUin2,output reg [12:0] ALUout);
	always@(ALUin1, ALUin2)
	begin
	  ALUout = ALUin1 + ALUin2;
	end
endmodule

//signToUnsign13b module converts signed 'ALUout' to unsigned number 'resultFraction12bits' and finds the sign bit of result
module signToUnsign13b(input [12:0] ALUout,output reg sign,output reg [11:0] resultFraction12bits);
	reg[12:0] diff13bits;
	
	always@(ALUout)
	begin
	  if(ALUout[12])
	    begin
	      sign = 1'b1;
	      diff13bits = (13'b1111111111111 - ALUout) + 13'b0000000000001;
	      resultFraction12bits = diff13bits[11:0];
	    end
	  else
	    begin
	      sign = 1'b0;
	      resultFraction12bits = ALUout[11:0];
	    end
	end
endmodule

//topModule
module floatingPointAdder(input [15:0] in1,input [15:0] in2,input clk,input reset,output resultSign1bit,output [4:0]resultExponet5bits,output [11:0]resultFraction12bits);
	wire[15:0] in1RegOut, in2RegOut;
	wire[5:0] sdeOut;
	wire blueSignLine;
	wire[4:0] absDiff;
	wire[11:0] badaLeftMuxOut, badaRightMuxOut;
	wire[11:0] sro1Out;
	wire beechWaalaMuxOut, rightWaalaMuxOut;
	wire[12:0] bigAluIn1, bigAluIn2, bigAluOut;
	
	register16bit in1Reg(clk, reset, in1, in1RegOut);
	register16bit in2Reg(clk, reset, in2, in2RegOut);
	
	signDiffExpo sde(in1RegOut[14:10], in2RegOut[14:10], sdeOut);
	
	signToUnsign6b stu6b(sdeOut, blueSignLine, absDiff);
	
	mux2to1_5b leftWaalaMux(in1RegOut[14:10], in2RegOut[14:10], blueSignLine, resultExponet5bits);
	
	mux2to1_12b badaLeftMux({2'b01, in2RegOut[9:0]}, {2'b01, in1RegOut}, blueSignLine, badaLeftMuxOut);
	
	mux2to1_12b badaRightMux({2'b01, in1RegOut[9:0]}, {2'b01, in2RegOut}, blueSignLine, badaRightMuxOut);
	
	shiftRightOp1 sro1(badaLeftMuxOut, absDiff, sro1Out);
	
	mux2to1_1b beechWaalaMux(in2RegOut[15], in1RegOut[15], blueSignLine, beechWaalaMuxOut);
	
	unsignToSign13b uts13b1(sro1Out, beechWaalaMuxOut, bigAluIn1);
	
	mux2to1_1b rightWaalaMux(in1RegOut[15], in2RegOut[15], blueSignLine, rightWaalaMuxOut);
	
	unsignToSign13b uts13b2(badaRightMuxOut, rightWaalaMuxOut, bigAluIn2);
	
	bigALU bALU(bigAluIn1, bigAluIn2, bigAluOut);
	
	signToUnsign13b stu13b(bigAluOut, resultSign1bit, resultFraction12bits);
endmodule

module testFloatingPointAdder;
	// Inputs
	reg [15:0] in1;
	reg [15:0] in2;
	reg clk;
	reg reset;

	// Outputs
	wire resultSign1bit;
	wire [4:0] resultExponet5bits;
	wire [11:0] resultFraction12bits;

	// Instantiate the Unit Under Test (UUT)
	floatingPointAdder uut (.in1(in1), .in2(in2), .clk(clk), .reset(reset), .resultSign1bit(resultSign1bit), .resultExponet5bits(resultExponet5bits), .resultFraction12bits(resultFraction12bits));
	always 
	#5 clk=~clk;
	initial begin
	//Initialize Inputs
	in1 = 0; in2 = 0; clk = 0; reset = 0;
	$display("time in1,in2,clk,reset, ResultSign1bit,ResultExponet5bits,ResultFraction12bits" );
	$monitor($time," %b %b %b %b %b ",in1,in2,resultSign1bit,resultExponet5bits,resultFraction12bits);
	
	#20 in1=16'b0_10001_0011_0000_00;in2=16'b0_10000_0000_1000_00;
	//4.5 + 2.0625 resultSign1bit =0, resultExponent5bits =10001, resultFraction12bits = 01_1011_0100_00
	
	#20 in1=16'b0_10001_0011_0000_00;in2=16'b1_10000_0000_1000_00;
	//4.5 - 2.0625 resultSign1bit =0, resultExponent5bits =10001, resultFraction12bits = 00_1010_1100_00 
	
	#20 in1=16'b1_10001_0011_0000_00;in2=16'b0_10000_0000_1000_00;
	//-4.5 + 2.0625 resultSign1bit =1, resultExponent5bits =10001, resultFraction12bits = 00_1010_1100_00
	
	#20 in1=16'b1_10001_0011_0000_00;in2=16'b1_10000_0000_1000_00;
	//-4.5 - 2.0625 resultSign1bit =1, resultExponent5bits =10001, resultFraction12bits = 01_1011_0100_00
	
	#20 $finish;
	end
endmodule