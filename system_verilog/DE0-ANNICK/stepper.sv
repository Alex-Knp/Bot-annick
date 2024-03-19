
module stepper (
    input   logic clk,            // Clock input
    input   logic reset,          // Reset input
    input   logic [31:0] control, // Control input (speed, position)
    input   logic homing_enable,  // homing mode enable
    output  logic step,           // Step output
    output  logic dir             // Direction output
);

// Parameters
logic [23:0] current_position;
logic [23:0] goal_position;
logic [7:0]  speed;
logic [20:0] MIN_DELAY;
logic [20:0] DELAY;
logic [20:0] MIN_DELAY;
logic [20:0] DELAY;


always_comb begin
    case (speed)
        8'd0 : MIN_DELAY = 2_000_000; 
        8'd1 : MIN_DELAY = 15_625; // 1  RPS
        8'd2 : MIN_DELAY = 7_812;  // 2  RPS
        8'd3 : MIN_DELAY = 5_208;  // 3  RPS
        8'd4 : MIN_DELAY = 3_906;  // 4  RPS
        8'd5 : MIN_DELAY = 3_125;  // 5  RPS
        8'd0 : MIN_DELAY = 2_000_000; 
        8'd1 : MIN_DELAY = 15_625; // 1  RPS
        8'd2 : MIN_DELAY = 7_812;  // 2  RPS
        8'd3 : MIN_DELAY = 5_208;  // 3  RPS
        8'd4 : MIN_DELAY = 3_906;  // 4  RPS
        8'd5 : MIN_DELAY = 3_125;  // 5  RPS

        8'd6 : MIN_DELAY = 2_604;  // 6  RPS
        8'd7 : MIN_DELAY = 2_232;  // 7  RPS
        8'd8 : MIN_DELAY = 1_953;  // 8  RPS
        8'd9 : MIN_DELAY = 1_736;  // 9  RPS
        8'd10 : MIN_DELAY = 1_562; // 10 RPS
        8'd6 : MIN_DELAY = 2_604;  // 6  RPS
        8'd7 : MIN_DELAY = 2_232;  // 7  RPS
        8'd8 : MIN_DELAY = 1_953;  // 8  RPS
        8'd9 : MIN_DELAY = 1_736;  // 9  RPS
        8'd10 : MIN_DELAY = 1_562; // 10 RPS

        8'd11 : MIN_DELAY = 1_420; // 11 RPS
        8'd12 : MIN_DELAY = 1_302; // 12 RPS
        8'd13 : MIN_DELAY = 1_201; // 13 RPS
        8'd14 : MIN_DELAY = 1_116; // 14 RPS
        8'd15 : MIN_DELAY = 1_041; // 15 RPS
        8'd11 : MIN_DELAY = 1_420; // 11 RPS
        8'd12 : MIN_DELAY = 1_302; // 12 RPS
        8'd13 : MIN_DELAY = 1_201; // 13 RPS
        8'd14 : MIN_DELAY = 1_116; // 14 RPS
        8'd15 : MIN_DELAY = 1_041; // 15 RPS

        8'd16 : MIN_DELAY = 976;   // 16 RPS
        8'd17 : MIN_DELAY = 919;   // 17 RPS
        8'd18 : MIN_DELAY = 868;   // 18 RPS
        8'd19 : MIN_DELAY = 822;   // 19 RPS
        8'd20 : MIN_DELAY = 781;   // 20 RPS
        8'd16 : MIN_DELAY = 976;   // 16 RPS
        8'd17 : MIN_DELAY = 919;   // 17 RPS
        8'd18 : MIN_DELAY = 868;   // 18 RPS
        8'd19 : MIN_DELAY = 822;   // 19 RPS
        8'd20 : MIN_DELAY = 781;   // 20 RPS


        default: MIN_DELAY = 17'bx;    // Undefined speed
        default: MIN_DELAY = 17'bx;    // Undefined speed
    endcase
end


assign {speed, goal_position} = control;

// Internal variables
logic [17:0] counter;

// Step output control and position update
always_ff @(posedge clk, posedge reset) begin

    // RESET CASE
    if (reset) begin
        counter <= 0;
        current_position <= 0;
        step <= 0;
        DELAY <= 21'd15_625;
    end 

    // HOMING CASE
    // HOMING CASE
    else if (homing_enable) begin
        counter <= counter + 1;
        step <= counter[11];
    end

    // STATIC CASE
    else if(current_position == goal_position) begin
        step <= 0; 
        DELAY <= 21'd15_625;
        counter <= 0;
    end
    
    // DELAY MODE 
    else if (counter < DELAY) begin
    // STATIC CASE
    else if(current_position == goal_position) begin
        step <= 0; 
        DELAY <= 21'd15_625;
        counter <= 0;
    end
    
    // DELAY MODE 
    else if (counter < DELAY) begin
            counter <= counter + 1;
            //Maintained step
            //Maintained step
            if(step == 1'b1) begin
               if(counter < 50) step <= 1'b1;
               if(counter < 50) step <= 1'b1;
                else step <= 1'b0;
            end
            //lower step
            //lower step
            else step <= 1'b0;
        end

    // STARTING STEP MODE
    // STARTING STEP MODE
    else begin
        counter <= 0;
        step <= 1'b1;
        case(dir)
            0: current_position <= current_position + 1;
            1: current_position <= current_position - 1;
        endcase

        
        //accel logic 
        if(DELAY > MIN_DELAY) begin
            DELAY <= DELAY - 21'd5;
        end
    end   
end



// Direction control
always_comb begin
    if(homing_enable) dir = 1; 
    else if (goal_position > current_position)  dir = 0; 
    else dir = 1; 
end

endmodule
