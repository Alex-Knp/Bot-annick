
//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module DE0_ANNICK(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// SDRAM //////////
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_DQM,
	DRAM_RAS_N,
	DRAM_WE_N,

	//////////// EPCS //////////
	EPCS_ASDO,
	EPCS_DATA0,
	EPCS_DCLK,
	EPCS_NCSO,

	//////////// EEPROM //////////
	I2C_SCLK,
	I2C_SDAT,

	//////////// ADC //////////
	ADC_CS_N,
	ADC_SADDR,
	ADC_SCLK,
	ADC_SDAT,

	//////////// 2x13 GPIO Header //////////
	GPIO_2,
	GPIO_2_IN,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0,
	GPIO_0_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	GPIO_1,
	GPIO_1_IN 
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// SW //////////
input 		     [3:0]		SW;

//////////// SDRAM //////////
output		    [12:0]		DRAM_ADDR;
output		     [1:0]		DRAM_BA;
output		          		DRAM_CAS_N;
output		          		DRAM_CKE;
output		          		DRAM_CLK;
output		          		DRAM_CS_N;
inout 		    [15:0]		DRAM_DQ;
output		     [1:0]		DRAM_DQM;
output		          		DRAM_RAS_N;
output		          		DRAM_WE_N;

//////////// EPCS //////////
output		          		EPCS_ASDO;
input 		          		EPCS_DATA0;
output		          		EPCS_DCLK;
output		          		EPCS_NCSO;

//////////// EEPROM ////////
output		          		I2C_SCLK;
inout 		          		I2C_SDAT;

//////////// ADC //////////
output		          		ADC_CS_N;
output		          		ADC_SADDR;
output		          		ADC_SCLK;
input 		          		ADC_SDAT;

//////////// 2x13 GPIO Header //////////
inout 		    [12:0]		GPIO_2;
input 		     [2:0]		GPIO_2_IN;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_0;
input 		     [1:0]		GPIO_0_IN;

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_1;
input 		     [1:0]		GPIO_1_IN;


//=======================================================
//  Structural coding
//=======================================================

//////// Reset //////////
logic [31:0] State_control;

logic reset;
logic reset_stepper_R; 		// Reset du stepper droit
logic reset_stepper_L; 		// Reset du stepper gauche

assign reset 			= State_control[0]; 
assign reset_stepper_L 	= State_control[4];		// reset current known position to 0
assign reset_stepper_R 	= State_control[8];		// reset current known position to 0
assign homing_L 		= State_control[12]; 	// activate homing mode of left stepper
assign homing_R 		= State_control[16];	// activate homing mode of left stepper


//////// Stored Data assignement SENSORS //////////

logic [31:0] Odometre_Left, Odometre_Right;

always_comb begin
	case(AddrFromPi[7:4])
		4'hf : DataToPI = 32'h0f0f0f0f;		//TEST
		4'he : DataToPI = 32'h1234abcd;		//TEST

		4'h1 : DataToPI = Odometre_Left;  		// Odomètre gauche 	: 1x
		4'h2 : DataToPI = Odometre_Right; 		// Odomètre droit 	: 2x
		4'h3 : DataToPI = IR1_ext;				// IR1
		4'h4 : DataToPI = IR2_ext;				// IR2
		4'h5 : DataToPI = IR3_ext;				// IR3
		4'h6 : DataToPI = IR4_ext;				// IR4
		4'h7 : DataToPI = micro_switch_data;	// Micro-switches

		default : DataToPI = 32'bx; 		
	endcase
end

//////// Data assignement for ACTUATORS //////////

logic [31:0] Actuators_RAM[15:0];

always_ff @(posedge CLOCK_50) begin
	Actuators_RAM[AddrFromPi[3:0]] = DataFromPI;  
end

assign State_control 	  = Actuators_RAM[0];		// State control

assign Servo_control_LC   = Actuators_RAM[1];		// Servo pince gauche 	: x1 
assign Servo_control_LP   = Actuators_RAM[2];		// Servo pento gauche 	: x2
assign Servo_control_RC   = Actuators_RAM[3];		// Servo pince droite 	: x3
assign Servo_control_RP   = Actuators_RAM[4];		// Servo pince gauche 	: x4
assign stepper_control_L  = Actuators_RAM[5]; 		// Steppers gauche		: x5
assign stepper_control_R  = Actuators_RAM[6]; 		// Steppers droit  		: x6


// ---   SPI module instantiation   ------------------------------------------

logic spi_clk, spi_cs, spi_mosi, spi_miso;
logic [31:0] DataFromPI;
logic [31:0] DataToPI;
logic [7:0]  AddrFromPi;

spi_slave spi_slave_inst (
	.SPI_CLK(spi_clk),
	.SPI_CS(spi_cs),
	.SPI_MOSI(spi_mosi),
	.SPI_MISO(spi_miso),
	.Data_Addr(AddrFromPi),
	.Data_Write(DataToPI),
	.Data_Read(DataFromPI), 
	.Clk(CLOCK_50)
);


// ---   odometers instantiation   -------------------------------------------

logic odoLA, odoLB, odoRA, odoRB;

odometer odoL (
	.reset(reset),
	.A(odoLA),
	.B(odoLB),
	.distance(Odometre_Left)
);
odometer odoR (
	.reset(reset),
	.A(odoRA),
	.B(odoRB),
	.distance(Odometre_Right)
);


// ---   Servo  instantation    ----------------------------------------------

logic [31:0] Servo_control_LC, Servo_control_LP, Servo_control_RC, Servo_control_RP; 
logic servo_LC, servo_LP, servo_RC, servo_RP;

Servo_PWM SERVO_LC (
	.clk(CLOCK_50), 
	.control(Servo_control_LC), 
	.servo(servo_LC) 
); 
Servo_PWM SERVO_LP (
	.clk(CLOCK_50), 
	.control(Servo_control_LP), 
	.servo(servo_LP)
); 
Servo_PWM SERVO_RC (
	.clk(CLOCK_50), 
	.control(Servo_control_RC), 
	.servo(servo_RC)
); 
Servo_PWM SERVO_RP (
	.clk(CLOCK_50), 
	.control(Servo_control_RP), 
	.servo(servo_RP)
); 

// TEST servo
//assign Servo_control_LC = 32'd25000; //0 def;
/*
logic [19:0] counter1;
logic toggle; 
always_ff @(posedge CLOCK_50) begin
	if(Servo_control_RP == 'd90000)
		toggle <= 0;
	if(Servo_control_RP == 0)
		toggle <= 1; 

	counter1 <= counter1 + 1;
	if(counter1 == 0)
	begin
		if(toggle == 0)
			Servo_control_RP <= Servo_control_RP - 1000;
		if(toggle == 1)
			Servo_control_RP <= Servo_control_RP + 1000;
	end

end 
assign Servo_control_RC = Servo_control_RP;  
*/


//TEST STEPPER
/*
logic [26:0] counterstepper; 

always_ff @(posedge CLOCK_50) begin
	counterstepper <= counterstepper + 1;
	if(counterstepper[24]) begin 
		stepper_control_L[31:24] <= 8'd1;
		stepper_control_L[23:0] <= 24'b0000_0000_0000_1111_1010_0000;
	end
	else begin
		stepper_control_L[31:24] <= 8'd1;
		stepper_control_L[23:0] <= 24'b0000_0000_0000_0000_0000_0000;
	end		
end
*/

/*
logic [19:0] counter_delay; 
logic [19:0] counter_step;  

always_ff @(posedge CLOCK_50) begin
	counter_delay <= counter_delay + 1;
end
always_ff @(posedge stepper_L) begin
	counter_step = counter_step + 1; 
end 
assign stepper_L = counter_delay[11];
assign dir_L = counter_step[15];
*/

// ---   IR's   instantation   -------------------------------------------------

// Generating 2.5 MHz ADC clock from 50 MHz clock
logic [4:0] counter_CLOCK_50;
always_ff @(posedge CLOCK_50) begin
		counter_CLOCK_50 <= counter_CLOCK_50 + 1;
	end

assign clk_1_5 = counter_CLOCK_50[4];


// Interfacing ADC. 
logic [11:0]   IR1, IR2, IR3, IR4;	// IR sensors
logic [31:0]   IR1_ext, IR2_ext, IR3_ext, IR4_ext;	// IR sensors extended to 32 bits

ADC_interface ADC_interface_inst (
	.clk(clk_1_5),
	.ADC_Dout(ADC_SDAT),
	.ADC_Din(ADC_SADDR),
	.ADC_CS(ADC_CS_N),
	.ADC_clk(ADC_SCLK),
	.ADC_0(IR1),
	.ADC_1(IR2),
	.ADC_2(IR3),
	.ADC_3(IR4)
);

assign IR1_ext = {20'b0, IR1};
assign IR2_ext = {20'b0, IR2};
assign IR3_ext = {20'b0, IR3};
assign IR4_ext = {20'b0, IR4};



// ---   Micro-swithes instantation   ------------------------------------------

logic micro_switch_1, micro_switch_2, micro_switch_3, micro_switch_4;
logic [31:0] micro_switch_data;

assign micro_switch_data[0]  = micro_switch_1;	// reset left stepper
assign micro_switch_data[8]  = micro_switch_5;	// reset right stepper
assign micro_switch_data[16] = micro_switch_3;
assign micro_switch_data[24] = micro_switch_4;
assign micro_switch_data[31] = micro_switch_2;


// ---   Stepper-motors instantation   -----------------------------------------

logic stepper_L, stepper_R, dir_L, dir_R, homing_L, homing_R;
logic [31:0] stepper_control_L, stepper_control_R;

stepper stepper_inst_L (
	.clk(CLOCK_50),
	.reset(reset_stepper_L),
	.control(stepper_control_L),
	.homing_enable(homing_L),
	.step(stepper_L),
	.dir(dir_L)
);


stepper stepper_inst_R (
	.clk(CLOCK_50),
	.reset(reset_stepper_R),
	.control(stepper_control_R),
	.homing_enable(homing_R),
	.step(stepper_R),
	.dir(dir_R)
);

//assign homing_L = State_control[12];
//assign homing_R = State_control[16];

//////////// Pin assignment for DE0_ANNICK //////////

//--- SPI ---//
assign spi_clk  		= GPIO_0[2];			//  11 (EDS Setup)	Pin : ? GPIO : 2
assign spi_cs   		= GPIO_0[4];			//  9  (EDS Setuo)  Pin : ? GPIO : 4
assign spi_mosi     	= GPIO_0_IN[1];			//  15 (EDS Setup)  Pin : ? GPIO : IN1
assign GPIO_0[0] = spi_cs ? 1'bz : spi_miso ;   //  13 (EDS Setup)  Pin : ? GPIO : 0

//---Odomètre---//
assign odoLA     = GPIO_0[16];	//Pin : 21 GPIO : 16
assign odoLB     = GPIO_0[18];	//Pin : 23 GPIO : 18
assign odoRA     = GPIO_0[17];	//Pin : 22 GPIO : 17
assign odoRB     = GPIO_0[19];	//Pin : 24 GPIO : 19

//--- Servo ---//
assign GPIO_0[28]  = servo_LC; 	//Pin : ? GPIO : ?
assign GPIO_0[26]  = servo_LP; 	//Pin : ? GPIO : ?
assign GPIO_0[30]  = servo_RC; 	//Pin : ? GPIO : ?
assign GPIO_0[32]  = servo_RP; 	//Pin : ? GPIO : ?

//---Stepper-motors---//

assign GPIO_0[27] = stepper_R;	// Pin : 34 GPIO : 27 
assign GPIO_0[23] = stepper_L; 	// Pin : 28 GPIO : 23 
assign GPIO_0[25] = dir_R;		// Pin : 32 GPIO : 25
assign GPIO_0[21] = dir_L;		// Pin : 26 GPIO : 21 

//---Micro-Swich---//

assign micro_switch_1 =  GPIO_0[5]; 	// Pin : 8  GPIO : 5
assign micro_switch_2 =  GPIO_0[7];		// Pin : 10 GPIO : 7
assign micro_switch_3 =  GPIO_0[6];		// Pin : 9  GPIO : 6
assign micro_switch_4 =  GPIO_0[9];		// Pin : 14 GPIO : 9
assign micro_switch_5 =  GPIO_0[8];		// Pin : 13 GPIO : 8


//--- IR ---//
//	IR1 => PIN : 3-25   GPIO : ANALOG_in_1
//  IR2 => PIN : 3-23 	GPIO : ANALOG_in_4	
//  IR3 => PIN : 3-21   GPIO : ANALOG_in_3
//  IR4 => PIN : 3-19   GPIO : ANALOG_in_6

endmodule
