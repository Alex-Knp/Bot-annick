
module stepper (
    input   logic clk,            // Clock input
    input   logic reset,          // Reset input
    input   logic [31:0] control, // Control input (speed, position)
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
        8'd1 : STEP_DELAY = 800_000; // 1  RPS
        8'd2 : STEP_DELAY = 125_000; // 2  RPS
        8'd3 : STEP_DELAY = 83_333;  // 3  RPS
        8'd4 : STEP_DELAY = 62_500;  // 4  RPS
        8'd5 : STEP_DELAY = 50_000;  // 5  RPS
        8'd6 : STEP_DELAY = 41_666;  // 6  RPS
        8'd7 : STEP_DELAY = 35_714;  // 7  RPS
        8'd8 : STEP_DELAY = 31_250;  // 8  RPS
        8'd9 : STEP_DELAY = 27_777;  // 9  RPS
        8'd10 : STEP_DELAY = 25_000; // 10 RPS
        8'd11 : STEP_DELAY = 22_727; // 11 RPS
        8'd12 : STEP_DELAY = 20_833; // 12 RPS
        8'd13 : STEP_DELAY = 19_230; // 13 RPS
        8'd14 : STEP_DELAY = 17_857; // 14 RPS
        8'd15 : STEP_DELAY = 16_666; // 15 RPS
        8'd16 : STEP_DELAY = 15_625; // 16 RPS
        8'd17 : STEP_DELAY = 14_705; // 17 RPS
        8'd18 : STEP_DELAY = 13_888; // 18 RPS
        8'd19 : STEP_DELAY = 13_157; // 19 RPS
        8'd20 : STEP_DELAY = 12_500; // 20 RPS
        8'd25 : STEP_DELAY = 10_000; // 25 RPS
        8'd30 : STEP_DELAY = 8_333;  // 30 RPS
        8'd35 : STEP_DELAY = 7_142;  // 35 RPS
        8'd40 : STEP_DELAY = 6_250;  // 40 RPS
        8'd45 : STEP_DELAY = 5_555;  // 45 RPS
        8'd50 : STEP_DELAY = 5_000;  // 50 RPS
        8'd55 : STEP_DELAY = 4_545;  // 55 RPS
        8'd60 : STEP_DELAY = 4_166;  // 60 RPS
        8'd65 : STEP_DELAY = 3_846;  // 65 RPS
        8'd70 : STEP_DELAY = 100;    // 70 RPS
        default: STEP_DELAY = 17'bx; // Undefined speed
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

    else if (counter < STEP_DELAY) begin
            counter <= counter + 1;
            if(step == 1'b1) begin
               if(counter < 10) step <= 1'b1;
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
assign dir = (goal_position > current_position) ? 0 : 1;

endmodule
