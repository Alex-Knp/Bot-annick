
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
logic [20:0] STEP_DELAY;


always_comb begin
    case (speed)
        8'd0 : STEP_DELAY = 2_000_000; 
        8'd1 : STEP_DELAY = 15_625; // 1  RPS
        8'd2 : STEP_DELAY = 7_812;  // 2  RPS
        8'd3 : STEP_DELAY = 5_208;  // 3  RPS
        8'd4 : STEP_DELAY = 3_906;  // 4  RPS
        8'd5 : STEP_DELAY = 3_125;  // 5  RPS

        8'd6 : STEP_DELAY = 2_604;  // 6  RPS
        8'd7 : STEP_DELAY = 2_232;  // 7  RPS
        8'd8 : STEP_DELAY = 1_953;  // 8  RPS
        8'd9 : STEP_DELAY = 1_736;  // 9  RPS
        8'd10 : STEP_DELAY = 1_562; // 10 RPS

        8'd11 : STEP_DELAY = 1_420; // 11 RPS
        8'd12 : STEP_DELAY = 1_302; // 12 RPS
        8'd13 : STEP_DELAY = 1_201; // 13 RPS
        8'd14 : STEP_DELAY = 1_116; // 14 RPS
        8'd15 : STEP_DELAY = 1_041; // 15 RPS

        8'd16 : STEP_DELAY = 976;   // 16 RPS
        8'd17 : STEP_DELAY = 919;   // 17 RPS
        8'd18 : STEP_DELAY = 868;   // 18 RPS
        8'd19 : STEP_DELAY = 822;   // 19 RPS
        8'd20 : STEP_DELAY = 781;   // 20 RPS

        8'd21 : STEP_DELAY = 744;   // 21 RPS
        8'd22 : STEP_DELAY = 710;   // 22 RPS
        8'd23 : STEP_DELAY = 679;   // 23 RPS
        8'd24 : STEP_DELAY = 651;   // 24 RPS
        8'd25 : STEP_DELAY = 625;   // 25 RPS

        8'd26 : STEP_DELAY = 600;   // 26 RPS
        8'd27 : STEP_DELAY = 578;   // 27 RPS
        8'd28 : STEP_DELAY = 558;   // 28 RPS
        8'd29 : STEP_DELAY = 538;   // 29 RPS
        8'd30 : STEP_DELAY = 520;   // 30 RPS

        8'd31 : STEP_DELAY = 504;   // 31 RPS
        8'd32 : STEP_DELAY = 488;   // 32 RPS
        8'd33 : STEP_DELAY = 473;   // 33 RPS
        8'd34 : STEP_DELAY = 459;   // 34 RPS
        8'd35 : STEP_DELAY = 446;   // 35 RPS

        8'd36 : STEP_DELAY = 434;   // 36 RPS
        8'd37 : STEP_DELAY = 422;   // 37 RPS
        8'd38 : STEP_DELAY = 411;   // 38 RPS
        8'd39 : STEP_DELAY = 400;   // 39 RPS

        8'd40 : STEP_DELAY = 390;   // 40 RPS


        default: STEP_DELAY = 17'bx;    // Undefined speed
    endcase
end


assign {speed, goal_position} = control;

// Internal variables
logic [17:0] counter;

// Step output control and position update
always_ff @(posedge clk, posedge reset) begin
    if (reset) begin
        counter <= 0;
        current_position <= 0;
        step <= 0;
    end 

    else if (homing_enable) begin
        counter <= counter + 1;
        step <= counter[11];
    end

    else if (counter < STEP_DELAY) begin
            counter <= counter + 1;
            if(step == 1'b1) begin
               if(counter < 200) step <= 1'b1;
                else step <= 1'b0;
            end
            else step <= 1'b0;
        end

    else begin
        counter <= 0;
        if (current_position == goal_position) begin
            step <= 1'b0;
        end
        else begin
            step <= 1'b1;
            case(dir)
                0: current_position <= current_position + 1;
                1: current_position <= current_position - 1;
            endcase
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
