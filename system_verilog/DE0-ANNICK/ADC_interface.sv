module ADC_interface(
    input  logic clk,
    input  logic ADC_Dout, 
    output logic ADC_Din,
    output logic ADC_CS, 
    output logic ADC_clk,
    output logic [11:0] ADC_0, 
    output logic [11:0] ADC_1,
    output logic [11:0] ADC_2,
    output logic [11:0] ADC_3 
);


    logic [6:0]  counter_clk; 
    logic [11:0] REG_DATA;

    assign ADC_clk = ADC_CS ? 1 : clk;

    assign ADC_CS = counter_clk[4];

    always_ff @(posedge clk) begin

        counter_clk <= counter_clk + 1;
        case (counter_clk)
        
            4:      REG_DATA[11] <= ADC_Dout;
            5 :     REG_DATA[10] <= ADC_Dout;
            6 :     REG_DATA[9]  <= ADC_Dout;
            7 :     REG_DATA[8]  <= ADC_Dout;
            8 :     REG_DATA[7]  <= ADC_Dout;
            9 :     REG_DATA[6]  <= ADC_Dout;
            10 :    REG_DATA[5]  <= ADC_Dout;
            11 :    REG_DATA[4]  <= ADC_Dout;
            12:     REG_DATA[3]  <= ADC_Dout;
            13 :    REG_DATA[2]  <= ADC_Dout;
            14 :    REG_DATA[1]  <= ADC_Dout;
            15 :    REG_DATA[0]  <= ADC_Dout;
            16 :    ADC_0 <= REG_DATA;

            36:     REG_DATA[11] <= ADC_Dout;
            37 :    REG_DATA[10] <= ADC_Dout;
            38 :    REG_DATA[9]  <= ADC_Dout;
            39 :    REG_DATA[8]  <= ADC_Dout;
            40 :    REG_DATA[7]  <= ADC_Dout;
            41 :    REG_DATA[6]  <= ADC_Dout;
            42 :    REG_DATA[5]  <= ADC_Dout;
            43 :    REG_DATA[4]  <= ADC_Dout;
            44:     REG_DATA[3]  <= ADC_Dout;
            45 :    REG_DATA[2]  <= ADC_Dout;
            46 :    REG_DATA[1]  <= ADC_Dout;
            47 :    REG_DATA[0]  <= ADC_Dout;
            48 :    ADC_1       <= REG_DATA;

            68:     REG_DATA[11] <= ADC_Dout;
            69 :    REG_DATA[10] <= ADC_Dout;
            70 :    REG_DATA[9]  <= ADC_Dout;
            71 :    REG_DATA[8]  <= ADC_Dout;
            72 :    REG_DATA[7]  <= ADC_Dout;
            73 :    REG_DATA[6]  <= ADC_Dout;
            74 :    REG_DATA[5]  <= ADC_Dout;
            75 :    REG_DATA[4]  <= ADC_Dout;
            76:     REG_DATA[3]  <= ADC_Dout;
            77 :    REG_DATA[2]  <= ADC_Dout;
            78 :    REG_DATA[1]  <= ADC_Dout;
            79 :    REG_DATA[0]  <= ADC_Dout;
            80 :    ADC_2       <= REG_DATA;

            100:    REG_DATA[11] <= ADC_Dout;
            101 :   REG_DATA[10] <= ADC_Dout;
            102 :   REG_DATA[9]  <= ADC_Dout;
            103 :   REG_DATA[8]  <= ADC_Dout;
            104 :   REG_DATA[7]  <= ADC_Dout;
            105 :   REG_DATA[6]  <= ADC_Dout;
            106 :   REG_DATA[5]  <= ADC_Dout;
            107 :   REG_DATA[4]  <= ADC_Dout;
            108:    REG_DATA[3]  <= ADC_Dout;
            109 :   REG_DATA[2]  <= ADC_Dout;
            110 :   REG_DATA[1]  <= ADC_Dout;
            111 :   REG_DATA[0]  <= ADC_Dout;
            112 :   ADC_3       <= REG_DATA;

        endcase
    end


    always_ff @(negedge clk) begin

        case(counter_clk)
            2 :     ADC_Din <= 0;
            3 :     ADC_Din <= 0;  
            4 :     ADC_Din <= 1;

            34 :    ADC_Din <= 1; 
            35 :    ADC_Din <= 0; 
            36 :    ADC_Din <= 0;

            66 :    ADC_Din <= 1;
            67 :    ADC_Din <= 1;
            68 :    ADC_Din <= 0;

            98 :    ADC_Din <= 0;
            99 :    ADC_Din <= 1;
            100:    ADC_Din <= 1;

            default: ADC_Din <= 1'bx;
        endcase
    end

endmodule