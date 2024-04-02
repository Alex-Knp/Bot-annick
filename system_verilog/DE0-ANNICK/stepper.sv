
module stepper (
    input   logic clk,              // Clock input
    input   logic reset,            // Reset input
    input   logic [31:0] control,   // Control input (speed, position)
    input   logic homing_enable,    // homing mode enable
    output  logic step,             // Step output
    output  logic dir,               // Direction output
    output  logic [31:0] feedback_position // Feedback position
);

// Parameters
logic [23:0] current_position;
logic [23:0] goal_position;
logic [7:0]  speed;
logic [20:0] MIN_DELAY;
logic [27:0] DELAY;
logic [14:0] ACCEL_Counter;
logic [14:0] ACCEL_trigger;
logic [31:0] Decel_trigger; // [step]
logic [31:0] Diff_step; 


always_comb begin
    case (speed)
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


        8'd21 : MIN_DELAY = 743;   // 21 RPS
        8'd22 : MIN_DELAY = 709;   // 22 RPS
        8'd23 : MIN_DELAY = 678;   // 23 RPS
        8'd24 : MIN_DELAY = 650;   // 24 RPS
        8'd25 : MIN_DELAY = 625;   // 25 RPS
        
        8'd26 : MIN_DELAY = 602;   // 26 RPS
        8'd27 : MIN_DELAY = 581;   // 27 RPS
        8'd28 : MIN_DELAY = 562;   // 28 RPS
        8'd29 : MIN_DELAY = 544;   // 29 RPS
        8'd30 : MIN_DELAY = 528;   // 30 RPS

        8'd31 : MIN_DELAY = 513;   // 31 RPS
        8'd32 : MIN_DELAY = 500;   // 32 RPS
        8'd33 : MIN_DELAY = 488;   // 33 RPS
        8'd34 : MIN_DELAY = 476;   // 34 RPS
        8'd35 : MIN_DELAY = 465;   // 35 RPS

        8'd36 : MIN_DELAY = 455;   // 36 RPS
        8'd37 : MIN_DELAY = 446;   // 37 RPS
        8'd38 : MIN_DELAY = 437;   // 38 RPS
        8'd39 : MIN_DELAY = 429;   // 39 RPS
        8'd40 : MIN_DELAY = 390;   // 40 RPS
        8'd45 : MIN_DELAY = 294;   // 45 RPS
        8'd50 : MIN_DELAY = 234;   // 50 RPS
        8'd55 : MIN_DELAY = 195;   // 55 RPS

        default: MIN_DELAY = 17'bx;    // Undefined speed
    endcase
end


assign {speed, goal_position} = control;
assign ACCEL_trigger = dir ? 15'd7_000 : 15'd2_000;

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
    
    // DYNAMIC MODE
    else begin

        // DECELERATION LOGIC
        Decel_trigger <= speed * 3200 / 10;         // 1 rotation of decel for 5 rps 
        if(current_position < goal_position)    Diff_step <= goal_position - current_position;
        else                                    Diff_step <= current_position - goal_position;

        // ACCELERATION LOGIC
        ACCEL_Counter <=  ACCEL_Counter + 1;
        if(ACCEL_Counter == ACCEL_trigger & Diff_step > Decel_trigger) begin
            ACCEL_Counter <= 0;
            if(DELAY > MIN_DELAY)   DELAY <= (DELAY*9_999)/10_000;
            else                    DELAY <= MIN_DELAY;
            
        end

        // DELAY MODE 
        if (counter < DELAY) begin
            counter <= counter + 1;
            step <= 1'b0;
        end

        // STARTING STEP MODE
        else begin

            // DECEL LOGIC
            if(Diff_step <= Decel_trigger) begin
                if(DELAY < 5_000) begin
                    DELAY <= DELAY + (5_000 - DELAY) / Diff_step;
                end
            end

            // STEP LOGIC
            counter <= 0;
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
    else if (goal_position >= current_position)  dir = 0; 
    else dir = 1; 
end

assign feedback_position = {8'b0 , current_position};

endmodule
