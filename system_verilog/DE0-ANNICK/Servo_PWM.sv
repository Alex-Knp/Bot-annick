module Servo_PWM(
    input logic clk,
	input logic [31:0] control,
	output logic servo
    );

//50 MHz clock onBoard
//20 ms counter.
//		1/50,000,000  Hz 			= 20  ns (every posedge)
//		(20,000,000 ns)/(20 ns) = 1,000,000
//
//    (20 bits) (2^(20)-1) 	= 1,048,575 (from 0 to 1,048,575)
//				Therefore, counter needs 20 bits [19:0]
//				Count up to 999,999 (0 included)

//Assumed Max (180 deg) 2 ms 			= 100,000*20 ns (100,000 clks)
//Assumed Min (0   deg) 1 ms 			=  50,000*20 ns ( 50,000 clks)
//Positions  100,000 - 50,000 		= 50,000
//Resolution (180 degrees)/50,000 	= 0.0036 degrees

//essential registers
logic [19:0] counter;

always_ff @(posedge clk)
begin

//Servo algorithm
	counter <= counter + 1;
	if(counter == 20'd999_999)
			counter <= 0;

	if(counter < (32'd50_000 + control))
			servo	<= 1;
	else
			servo	<= 0;

end

endmodule