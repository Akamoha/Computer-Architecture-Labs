
module testMulAddSub;
// Inputs
reg clk,load,select;
reg [3:0] multiplier,multiplicand,in3;
// Outputs
wire [7:0] aluOut;
// Instantiate the Unit Under Test (UUT)
mulAddSub uut (
.clk(clk),
.load(load),
.select(select),
.multiplier(multiplier),
.multiplicand(multiplicand),
.in3(in3),
.aluOut(aluOut)
);
always #5 clk=~clk;
initial begin
//$monitor($time," %d * %d select( %b ) %d = %d ",multiplier,multiplicand,select,in3,aluOut);
// Initialize Inputs
//Case 1 : multiplier = 1 mulitplicand = 5 select = 1 (add) in3 = 5, (5 * 1 + 5 = 10)
clk = 0;
load = 1;
multiplier = 1;
multiplicand = 5;
in3 = 5;
select = 1;
#10 load = 0;
#40
//Case 2 : multiplier = -5 mulitplicand = 5 select = 1 (add) in3 = 5, (5 * (-5) + 5 = (-20))
load = 1;
multiplier = -5;
multiplicand = 5;
#7 select = 1; in3=5; //#10 select = 0; load=0;
#3 load=0;
#40
//Case 3 : multiplier = -5 mulitplicand = -7 select = 0 (sub) in3 = 5, ((-7) * (-5) - 5 = 30)
load = 1;
multiplier = -5;
multiplicand = -7;
#7select = 0; in3=5;
#3 load=0;
#40
//Case 4 : multiplier = 6 mulitplicand = -6 select = 1 (add) in3 = -4, (6 * (-6) + (-4) = (-40))
load = 1;
multiplier = 6;
multiplicand = -6;
#7select = 1; in3=-4;
#3 load=0;
#50
$finish;
end
endmodule