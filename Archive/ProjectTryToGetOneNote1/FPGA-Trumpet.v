
/*****************************************************************************
 *                            Top Level Entity                               *
 *****************************************************************************/
module FPGATrumpet(
//KEY[0] is reset

	// Inputs
	CLOCK_50,
	KEY,
	LEDR,
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// Audio ports
	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	SW,
	
	
	// VGA ports
	VGA_CLK,   						//	VGA Clock
	VGA_HS,							//	VGA H_SYNC
	VGA_VS,							//	VGA V_SYNC
	VGA_BLANK_N,						//	VGA BLANK
	VGA_SYNC_N,						//	VGA SYNC
	VGA_R,   						//	VGA Red[9:0]
	VGA_G,	 						//	VGA Green[9:0]
	VGA_B   						//	VGA Blue[9:0]
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                            Audio Port Declarations                        *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
input		[9:0]	SW;
output	[1:0] LEDR;
output	[6:0]HEX0;
output	[6:0]HEX1;
output	[6:0]HEX2;
output	[6:0]HEX3;
output	[6:0]HEX4;
output	[6:0]HEX5;

input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;

/*****************************************************************************
 *                            VGA Port Declarations                          *
 *****************************************************************************/
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK_N;				//	VGA BLANK
output			VGA_SYNC_N;				//	VGA SYNC
output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
output	[7:0]	VGA_B;   				//	VGA Blue[7:0]

wire resetn;
assign resetn = KEY[0];
/*****************************************************************************
 *                 			     Trumpet Sound		       	                 *
 *****************************************************************************/
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

wire [9:0] audio_from_ram;
wire write;

note_Select n0 (.clock(CLOCK_50), .keys(~KEY[3:1]), .airflow(air), .audio_out_allowed(audio_out_allowed), .note(audio_from_ram), .write(write));


assign write_audio_out			= write & audio_out_allowed;


wire [31:0] sound;


assign sound = {audio_from_ram, 22'b0};


assign read_audio_in = audio_in_available & audio_out_allowed;



assign left_channel_audio_out	= sound;
assign right_channel_audio_out	= sound;

/*****************************************************************************
 *                         Audio Controller Module 		                    *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);
	
/*****************************************************************************
 *                         VGA Adapter Module 		 		                    *
 *****************************************************************************/

// wires  
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

// instantiate VGA module
vga_adapter VGA(
		.resetn(resetn),
		.clock(CLOCK_50),
		.colour(colour),
		.x(x),
		.y(y),
		.plot(writeEn),
		
		/* Signals for the DAC to drive the monitor. */
		.VGA_R(VGA_R),
		.VGA_G(VGA_G),
		.VGA_B(VGA_B),
		.VGA_HS(VGA_HS),
		.VGA_VS(VGA_VS),
		.VGA_BLANK(VGA_BLANK_N),
		.VGA_SYNC(VGA_SYNC_N),
		.VGA_CLK(VGA_CLK));
defparam VGA.RESOLUTION = "160x120";
defparam VGA.MONOCHROME = "FALSE";
defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
defparam VGA.BACKGROUND_IMAGE = "scalestaff.mif";
/*****************************************************************************
 *                         	FSM and Datapath 			 		                 *
 *****************************************************************************/
 
 
/*****************************************************************************
 *                         Microphone Input Module 		                    *
 *****************************************************************************/
	wire [1:0] air;
	
	
	assign LEDR[0]=air[0];
	assign LEDR[1]=air[1];
	
	wire Enable;
	wire [25:0]RDiv;
	wire [31:0] outp;
	
	assign outp = (left_channel_audio_in[31] ==1)? -left_channel_audio_in: left_channel_audio_in;
	
	
	wire [10:0] lCount;
	wire [10:0] mCount;
	wire [10:0] hCount;
	wire [10:0] rc;
	
	micCheck tm0 (outp, Enable, SW[9], lCount, mCount, hCount, rc, air);
	
	assign Enable = (RDiv == 26'b00000000000000000000000000)?1:0;
	
	RateDivider rd0 (
		.Clock(CLOCK_50),
		.q(RDiv)
		);
	
// Display microphone input on the HEX 

	hex_decoder H0(
        .hex_digit(lCount[3:0]), 
        .segments(HEX0)
        );
	
	hex_decoder H1(
        .hex_digit(lCount[7:4]), 
        .segments(HEX1)
        );
	hex_decoder H2(
        .hex_digit(mCount[3:0]), 
        .segments(HEX2)
        );
	hex_decoder H3(
        .hex_digit(mCount[7:4]), 
        .segments(HEX3)
        );
	hex_decoder H4(
        .hex_digit(hCount[3:0]), 
        .segments(HEX4)
        );
	hex_decoder H5(
        .hex_digit(hCount[7:4]), 
        .segments(HEX5)
        );
	
	

/*****************************************************************************
 *                 Drawing Datapath	 	part of top level	                    *
 *****************************************************************************/
	
	reg [7:0] xselect;
	reg [6:0] yselect;
	reg [2:0] colourselect;	
	
	wire [7:0] gx, gsx, ax, asx, scalex;
	wire [6:0] gy, gsy, ay, asy, scaley;
	
	wire [6:0] gaddress;
//
//	linenote g(
//	.address(gaddress),
//	.clock(CLOCK_50),
//	.data(),
//	.wren(0),
//	.q(gcolour));
//	
//	outlet g0 (
//	.clock(vEnable), 
//	.resetn(resetn), 
//	.xin(8'd34),
//	.yin(7'd99),
//	.width(8'd9),
//	.height(7'd5),
//	.x(gx), 
//	.y(gy), 
//	.wren(), 
//	.address(gaddress)
//);
 

 
wire [14:0] scaleaddress;
	
newscalestaff nss0(
	.address(scaleaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(colour));

BGoutlet sc1 (
	.clock(CLOCK_50), 
	.resetn(~resetn), 
	.xin(0),
	.yin(0),
	.width(159),
	.height(119),
	.x(x), 
	.y(y), 
	.wren(writeEn), 
	.address(scaleaddress)
);
 
//always@(posedge CLOCK_50)begin
//	if(SW[4]) begin
//		xselect <= gx;
//		yselect <= gy;
//		colourselect <= gcolour;
//	end
//	else if(~KEY[2]) begin
//		xselect <= gsx;
//		yselect <= gsy;
//		colourselect <= gscolour;
//	end
//	else if(~KEY[1]) begin
//		xselect <= ax;
//		yselect <= ay;
//		colourselect <= acolour;
//	end
//	else begin
//		xselect <= scalex;
//		yselect <= scaley;
//		colourselect <= scalecolour;	
//	end
//
//end
	
//assign x = xselect;	
//assign y = yselect;
//assign colour = colourselect;	

wire vEnable;
wire [8:0]vRDiv;
assign vEnable = (vRDiv == 9'b000000000)?1:0;
	
vga_RateDivider vrd0 (
	.Clock(CLOCK_50),
	.q(vRDiv)
);	
	
endmodule



/*****************************************************************************
 *                          Note Select Module 				                    *
 *****************************************************************************/
module note_Select (clock, keys, airflow, audio_out_allowed, note, write);
	input clock;
	input [2:0] keys;
	input [1:0] airflow;
	input audio_out_allowed;
	
	output reg [9:0] note;
	output reg write;
	
	wire [15:0] address_count;
	reg [15:0] address_count_reg;
	
	wire [9:0] c4_audio;
	wire [9:0] cs4_audio;
	wire [9:0] d4_audio;
	wire [9:0] ds4_audio;
	wire [9:0] e4_audio;
	wire [9:0] f4_audio;
	wire [9:0] fs4_audio;
	wire [9:0] g4_audio;
	wire [9:0] gs4_audio;
	wire [9:0] a4_audio;
	wire [9:0] as4_audio;
	wire [9:0] b4_audio;
	wire [9:0] c5_audio;

// instantiating ram for each note
	
	c4 c0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(c4_audio));

	cs4 cs0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(cs4_audio));

	d4 d0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(d4_audio));

	ds4 ds0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(ds4_audio));

	e4 e0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(e4_audio));

	f4 f0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(f4_audio));

	newnewfs4 fs0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(fs4_audio));

	g4 g0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(g4_audio));

	gs4 gs0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(gs4_audio));

	a4 a0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(a4_audio));

	as4 as0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(as4_audio));

	b4 b0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(b4_audio));

	c5 hc0(
	.address(address_count),
	.clock(clock),
	.data(),
	.wren(1'b0),
	.q(c5_audio));
	
	
	always @(posedge clock) begin
		if(address_count_reg >= 16'd16383 && audio_out_allowed) begin
				address_count_reg <= 16'b0;
				write <= 0;
		end 
		else if(audio_out_allowed) begin
				address_count_reg <= address_count_reg + 1;
				write <= 1;
		end
		else
			write <= 0;
	end
	
	assign address_count = address_count_reg;

// determining output note	

	always @(posedge clock)	
	begin
		case (airflow[1:0])
			2'b00: note = 10'b00000;	//case 0: no airflow
			2'b01:							//case 1: level 1 airflow
				begin
					if (keys == 3'b000)			//Middle C 523
						note = c4_audio;
					else if (keys == 3'b111)		//C# 554
						note = cs4_audio;
					else if (keys == 3'b101)		//D 587
						note = d4_audio;
					else if (keys == 3'b011)		//D# 622
						note = ds4_audio;
					else if (keys == 3'b110)		//E 659
						note = e4_audio;
					else if (keys == 3'b100)		//F 698
						note = f4_audio;
					else if (keys == 3'b010)		//F# 740	
						note = fs4_audio;
					else
						note = 10'b0;
				end
			2'b10:							//case 2: level 2 airflow
				begin
					if (keys == 3'b000)			//G 784
						note = g4_audio;
					else if (keys == 3'b011)		//G# 831	
						note = gs4_audio;
					else if (keys == 3'b110)		//A 880
						note = a4_audio;
					else if (keys == 3'b100)		//A# 932
						note = as4_audio;
					else if (keys == 3'b010)		//B 988
						note = b4_audio;
					else if (keys == 3'b001)
						note = c5_audio;
					else
						note = 10'b0; 
				end
			default: note = 10'b0;	//default case
		endcase
	end

endmodule

/*****************************************************************************
 *                       Microphone Input/ Airflow Module             		  *
 *****************************************************************************/

module micCheck (audio_in, clk, mute, lCount, mCount, hCount, q, air);
	input [31:0]audio_in;
	input clk;
	input mute;
	output reg [10:0] lCount;
	output reg [10:0] mCount;
	output reg [10:0] hCount;
	output reg [10:0]q;
	output reg [1:0]air;
	
	always @(posedge clk) // triggered every time clock rises
	begin
		if (q == 10'b0000000000) // when q is the min value for the counter
			begin
				//if (hCount> 10'b0000000100 || hCount > lCount && hCount > mCount)
					//air <=2'b10;
				//else if (mCount > lCount || mCount >10'b0000000011)
					//air <=2'b01;
				//else
					//air <=2'b00;
			
				
				if (mute ==1)
					air<=2'b00;
				else if (lCount > mCount && lCount > hCount)
					air<=2'b00;
				else if (mCount > lCount && mCount > hCount)
					air<=2'b01;
				else if (hCount > lCount && hCount > mCount)
					air <=2'b10;
				else
					air <= air;
				
				q <= 26'b0000001111;//something bit something; // q reset to 0
				hCount <= 11'b00000000000;
				mCount <= 11'b00000000000;
				lCount <= 11'b00000000000;
			end
		else
			begin
			q <= q - 1; // decrement q
			if (audio_in > 32'b00000001111111111110000000000000)
				hCount <= hCount + 1;
			else if (audio_in > 32'b00000000000111111111111100000000)
				mCount <= mCount + 1;
			else
				lCount <= lCount + 1;
			end
	end
endmodule

/*****************************************************************************
 *                          Rate Divider Module 			                    *
 *****************************************************************************/
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

module RateDivider (Clock, q);
	input Clock;
	
	output reg [25:0] q; // declare q

	always @(posedge Clock) // triggered every time clock rises
	begin
		if (q == 26'b00000000000000000000000000) // when q is the min value for the counter
			//Real code value
			q <= 26'b00000001111111111111111111;//something bit something; // q reset to 0

		else
			q <= q - 1; // decrement q
	end

endmodule


module outlet(clock, resetn, xin, yin, width, height, x, y, wren, address);
	input clock, resetn;
	input [7:0] xin;
	input [6:0] yin;
	input [7:0] width;
	input [6:0] height;
	output [7:0] x;
	output [6:0] y;
	output [7:0] address;
	output wren;
	
	reg [7:0] xCounter;
	reg [6:0] yCounter;

	wire xCounter_clear;
	wire yCounter_clear;
	/* A counter to scan through a horizontal line. */
	
	
	always @(posedge clock)
	begin
		if (resetn)
			xCounter <= 8'd0;
		else if (xCounter_clear)
			xCounter <= 8'd0;
		else
		begin
			xCounter <= xCounter + 1'b1;
		end
	end
	
	assign xCounter_clear = (xCounter == (width));


	always @(posedge clock)
	begin
		if (resetn)
			yCounter <= 7'd0;
		else if (xCounter_clear && yCounter_clear)
			yCounter <= 7'd0;
		else if (xCounter_clear)		//Increment when x counter resets
			yCounter <= yCounter + 1'b1;
	end
	
	assign yCounter_clear = (yCounter == (height)); 
	
	assign x = xCounter + xin;
	assign y = yCounter + yin;
	assign address = (xCounter + (yCounter*(width + 1)));
	assign wren = (address <= (((height + 1)*(width + 1)) -1));
endmodule
		

module BGoutlet(clock, resetn, xin, yin, width, height, x, y, wren, address);
	input clock, resetn;
	input [7:0] xin;
	input [6:0] yin;
	input [7:0] width;
	input [6:0] height;
	output [7:0] x;
	output [6:0] y;
	output [14:0] address;
	output wren;
	
	reg [7:0] xCounter;
	reg [6:0] yCounter;

	wire xCounter_clear;
	wire yCounter_clear;
	/* A counter to scan through a horizontal line. */
	
	
	always @(posedge clock)
	begin
		if (resetn)
			xCounter <= 8'd0;
		else if (xCounter_clear)
			xCounter <= 8'd0;
		else
		begin
			xCounter <= xCounter + 1'b1;
		end
	end
	
	assign xCounter_clear = (xCounter == (width));


	always @(posedge clock)
	begin
		if (resetn)
			yCounter <= 7'd0;
		else if (xCounter_clear && yCounter_clear)
			yCounter <= 7'd0;
		else if (xCounter_clear)		//Increment when x counter resets
			yCounter <= yCounter + 1'b1;
	end
	
	assign yCounter_clear = (yCounter == (height)); 
	
	assign x = xCounter + xin;
	assign y = yCounter + yin;
	assign address = (xCounter + (yCounter*(width + 1)));
	assign wren = (address <= (((height + 1)*(width + 1)) -1));
endmodule
		
module vga_RateDivider (Clock, q);
	input Clock;
	
	output reg [8:0] q; // declare q

	always @(posedge Clock) // triggered every time clock rises
	begin
		if (q == 9'b000000000) // when q is the min value for the counter
			//Real code value
			q <= 9'b111111111;//something bit something; // q reset to 0

		else
			q <= q - 1; // decrement q
	end

endmodule
