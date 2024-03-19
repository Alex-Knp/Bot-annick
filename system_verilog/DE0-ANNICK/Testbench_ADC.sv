`timescale 1ms/100ns

module Testbench_ADC(); 

    reg clock; 
    reg Data_out; 
    reg reset;  
    wire Data_in, ADC_CS, ADC_clk;
    wire [11:0]   ADC_DATA[3:0];

    ADC_interface ADC(
        .clk        (clock), 
        .reset      (reset),
        .ADC_Dout   (Data_out), 
        .ADC_Din    (Data_in), 
        .ADC_CS     (ADC_CS), 
        .ADC_clk    (ADC_clk), 
        .ADC_DATA   (ADC_DATA)
    );

    logic [15:0]    IR_0, IR_1, IR_2, IR_3;
    assign IR_0 = 16'habcd;
    assign IR_1 = 16'h1234;
    assign IR_2 = 16'h5678;
    assign IR_3 = 16'h9abc;

    always #0.5 clock = ~clock; 

    initial begin

        clock = 1'b1; 
        Data_out = 1'b0;
        reset = 1'b1;

        #3.5
        reset = 1'b0;

        #1
        Data_out = IR_0[15];

        #1
        Data_out = IR_0[14];

        #1
        Data_out = IR_0[13];

        #1
        Data_out = IR_0[12];

        #1
        Data_out = IR_0[11];

        #1
        Data_out = IR_0[10];

        #1
        Data_out = IR_0[9];

        #1
        Data_out = IR_0[8];

        #1
        Data_out = IR_0[7];

        #1
        Data_out = IR_0[6];

        #1
        Data_out = IR_0[5];

        #1
        Data_out = IR_0[4];

        #1
        Data_out = IR_0[3];

        #1
        Data_out = IR_0[2];

        #1
        Data_out = IR_0[1];

        #1
        Data_out = IR_0[0];

        #1
        Data_out = IR_1[15];
        #1
        Data_out = IR_1[14];
        #1
        Data_out = IR_1[13];
        #1
        Data_out = IR_1[12];
        #1
        Data_out = IR_1[11];
        #1
        Data_out = IR_1[10];
        #1
        Data_out = IR_1[9];
        #1
        Data_out = IR_1[8];
        #1
        Data_out = IR_1[7];
        #1
        Data_out = IR_1[6];
        #1
        Data_out = IR_1[5];
        #1
        Data_out = IR_1[4];
        #1
        Data_out = IR_1[3];
        #1
        Data_out = IR_1[2];
        #1
        Data_out = IR_1[1];
        #1
        Data_out = IR_1[0];
        #1
        Data_out = IR_2[15];
        #1
        Data_out = IR_2[14];
        #1
        Data_out = IR_2[13];
        #1
        Data_out = IR_2[12];
    end

endmodule