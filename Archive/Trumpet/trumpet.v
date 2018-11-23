`timescale 1ns / 1ns // `timescale time_unit/time_precision


module trumpet (SW, KEY, HEX0);
	input [3:1]KEY;
	input [1:0]SW;
	
	output [6:0]HEX0;

	wire [4:0]note;
	
	
	note_Select u0 (
		.keys(~KEY[3:1]),
		.airflow(SW[1:0]),
		.note(note)
		);

	
	hex_decoder H0(
        .hex_digit(note[3:0]), 
        .segments(HEX0)
        );
		
endmodule


module note_Select (keys, airflow, note);
	input [2:0] keys;
	input [1:0]airflow;
	
	output reg [4:0] note;
	
	always @(*)	//declare always block
	begin
		case (airflow[1:0])
			//alternate code used for testing
			2'b00: note = 5'b00000;	//case 0: no airflow
			2'b01:
				begin
					if (keys == 3'b000)			//Middle C
						note = 5'b00001;
					else if (keys == 3'b111)		//C#
						note = 5'b00010;
					else if (keys == 3'b101)		//D
						note = 5'b00011;
					else if (keys == 3'b011)		//D#
						note = 5'b00100;
					else if (keys == 3'b110)		//E
						note = 5'b00101;
					else if (keys == 3'b100)		//F
						note = 5'b00110;
					else if (keys == 3'b010)		//F#
						note = 5'b00111;
					else
						note = 5'b00000;
				end
			2'b10:
				begin
					if (keys == 3'b000)			//G
						note = 5'b01000;
					else if (keys == 3'b011)		//G#
						note = 5'b01001;
					else if (keys == 3'b110)		//A
						note = 5'b01010;
					else if (keys == 3'b100)		//A#
						note = 5'b01011;
					else if (keys == 3'b010)		//B
						note = 5'b01100;
					else
						note = 5'b00000;
				end
			2'b11: 
				begin
					if (keys == 3'b000)			//High C
						note = 5'b01101;
					else if (keys == 3'b110)		//C#
						note = 5'b01110;
					else if (keys == 3'b100)		//D
						note = 5'b01111;
					else if (keys == 3'b010)		//D#
						note = 5'b10000;
					else
						note = 5'b00000;
				end
			default: note = 5'b00000;	//default case
		endcase
	end

endmodule



module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule