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

////////////////////////////////////////////////////////////
//Test Control registers ///////////////////////////////////
////////////////////////////////////////////////////////////

//reg toggle	=	1;
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


always_ff @(posedge clk)
begin

//Servo algorithm
	counter <= counter + 1;
	if(counter == 'd999999)
			counter <= 0;

	if(counter < ('d27000 + control))
			servo	<= 1;
	else
			servo	<= 0;

////////////////////////////////////////////////////////////
//Test control algorithm////////////////////////////////////
////////////////////////////////////////////////////////////
/*
if(control_cut == 'd50000)
	toggle <= 0;
if(control_cut == 0)
	toggle <= 1; 

if(counter == 0)
	begin
		if(toggle == 0)
			control_cut <= control_cut - 1000;
		if(toggle == 1)
			control_cut <= control_cut + 1000;
	end
	*/  
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

end

endmodule