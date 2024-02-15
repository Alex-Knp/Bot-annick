module encoder (
  input logic clk,
  input logic A,
  input logic B,
  output logic [31:0] out,
  input logic reset
);

  logic [31:0] counter_clk;
  logic [15:0] counter;
  logic [15:0] speed;
  logic [15:0] direction_out;
  logic dir;
  logic old_A, old_B, new_A, new_B;


always_ff @(posedge clk, posedge reset) begin
  	 if (reset) begin
		counter_clk <= 0;
		counter <= 0;
		old_A <= 0;
		old_B <= 0;
		new_A <= 0;
		new_B <= 0;
	 end
	 else begin 
		old_A <= new_A;
		new_A <= A;
		old_B <= new_B;
		new_B <= B;
		if (counter_clk == 250_000) begin
			speed <= counter;
			direction_out <= dir;                   // nombre de posedge A ou B ou negedge A ou B
			counter_clk <= 0;
			counter <= 0;
		end
		else counter_clk <= counter_clk + 1;

		
		if (~old_A && new_A)begin                // si je rentre sur posedge A 
			counter <= counter +1;
			if (~B) dir <= 1;
			else dir <= 0;
		end
		if (~old_B && new_B) begin              // si je rentre sur un posedge B
			counter <= counter +1;
			if (A) dir <= 1;
			else dir <= 0;
		end
		if (old_A && ~new_A)begin                // si je rentre sur negedge A 
			counter <= counter +1;
			if (B) dir <= 1;
			else dir <= 0;
		end
		if (old_B && ~new_B) begin              // si je rentre sur un negedge B
		    counter <= counter +1;
			if (~A) dir <= 1;
			else dir <= 0;
		end

	end
end

assign out = {direction_out, speed};
  
endmodule

/*

Pour ce code, sur chaque posedge de clock je connais l'ancienne et la nouvelle valeur de A et B.
Avec ça, si A ou B à changé j'augemente le compteur et j'update la direction.
Dès que le compteur de clock vaut 100 je stock dans speed le compteur de posedges et negedges

*/