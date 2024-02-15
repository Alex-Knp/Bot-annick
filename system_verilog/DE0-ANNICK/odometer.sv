
module odometer (
    input logic reset,                       // chaque fois qu'on recoit l'adresse de cet odometre on reset le compteur donc ce reset doit valoir 0
    input logic A,
    input logic B,
    output logic [31:0] distance               // compteur correspondant au nombre de flancs montants de A depuis la dernière fois qu'on a reçu l'adresse de cet odomètre
);                                           // pour obtenir lal distance parcourue, on fait dist/2048 * 2pi*r avec r le rayon de la roue

logic [31:0] count_odo_A_pos, count_odo_B_pos, count_odo_A_neg, count_odo_B_neg;

always_ff @(posedge A, posedge reset) begin
    if(reset) count_odo_A_pos <= 0;
    else if (B) count_odo_A_pos <= count_odo_A_pos - 1;
    else count_odo_A_pos <= count_odo_A_pos + 1;
end

always_ff @(negedge A, posedge reset) begin
    if(reset) count_odo_A_neg <= 0;
    else if (B) count_odo_A_neg <= count_odo_A_neg + 1;
    else count_odo_A_neg <= count_odo_A_neg - 1;
end

always_ff @(posedge B, posedge reset) begin
    if(reset) count_odo_B_pos <= 0;
    else if (A) count_odo_B_pos <= count_odo_B_pos + 1;
    else count_odo_B_pos <= count_odo_B_pos - 1;
end

always_ff @(negedge B, posedge reset) begin
    if(reset) count_odo_B_neg <= 0;
    else if (A) count_odo_B_neg <= count_odo_B_neg - 1;
    else count_odo_B_neg <= count_odo_B_neg + 1;
end

assign distance = (count_odo_A_pos + count_odo_A_neg + count_odo_B_pos + count_odo_B_neg);

endmodule 