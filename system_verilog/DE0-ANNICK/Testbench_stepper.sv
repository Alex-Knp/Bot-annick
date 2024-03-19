`timescale 1ms/100ns

module Testbench_stepper(); 

    reg clock; 
    reg reset; 
    reg [31:0] control;  
    wire stepper, direction;
    reg homing_enable; 

    stepper stepper_inst (
        .clk        (clock), 
        .reset      (reset),
        .control    (control),
        .homing_enable (homing_enable),
        .step       (stepper), 
        .dir        (direction)
    );

    always #0.5 clock = ~clock; 

    initial begin

        clock = 1'b1; 
        reset = 1'b1;
        homing_enable = 0;

        #5
        reset = 1'b0;

        #5
        control[31:24] = 8'd0; 
        control[23:0] = 24'b0000_0000_0000_0000_0000_0000;

        #5
        control[31:24] = 8'd20; 
        control[23:0] = 24'd3600;

    end

endmodule