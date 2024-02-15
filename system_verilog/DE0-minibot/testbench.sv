`timescale 1ns/1ps     

module testbench();

	reg clk_nano;
	reg A;
	reg B;
	reg [31:0] speed;
	reg dir;




	test dut(
		.clk_nano		(clk_nano),
		.A		(A),
		
		.B			(B),
		.speed		(speed),
		
		.dir			(dir)

	);

	always #20 clk_nano = ~clk_nano;
	always #3000 A = ~A;
	
	initial 
		begin 
		B = 1'b0;
		dir = 0;
		speed = 1'b0;
		#5
		clk_nano   = 1'b0;
		A = 1'b0;
		#30000
		clk_nano = 1'b0;
	end
	
endmodule 