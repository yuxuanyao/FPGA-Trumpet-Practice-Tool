
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
wire [3:0]note_id;

note_Select n0 (
	.clock(CLOCK_50), 
	.keys(~KEY[3:1]), 
	.airflow(air), 
	.audio_out_allowed(audio_out_allowed), 
	.note(audio_from_ram), 
	.write(write), 
	.id (note_id)
	);


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
wire VGAwriteEn;

// instantiate VGA module
vga_adapter VGA(
		.resetn(resetn),
		.clock(CLOCK_50),
		.colour(colour),
		.x(x),
		.y(y),
		.plot(VGAwriteEn),
		
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
defparam VGA.BACKGROUND_IMAGE = "scalestaffshift.mif";

 note_info ni(
	.id(note_id), 
	.clock(CLOCK_50), 
	.air(air),
	.resetn(~KEY[0]),
	.x(x), 
	.y(y), 
	.colour(colour), 
	.writeEn(VGAwriteEn)
);
 
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
	
	
	
	
endmodule

/*****************************************************************************
 *                         	Note Graphics Module		 		                 *
 *****************************************************************************/
 module note_info (id, clock, air, resetn, x, y, colour, writeEn);
	input [3:0]id;
	input clock;
	input [1:0] air;
	input resetn;
	output reg [7:0] x;
	output reg [6:0] y;
	output reg [2:0] colour;
	output writeEn;
	
	wire [7:0] caddress, csaddress, daddress, dsaddress, eaddress, faddress, fsaddress, gaddress, gsaddress, aaddress, asaddress, baddress, hcaddress;
	wire [14:0] scaddress;
	
	
	// wires for x, y, and colour 
	wire [7:0] cx, csx, dx, dsx, ex, fx, fsx, gx, gsx, ax, asx, bx, hcx;
	wire [6:0] cy, csy, dy, dsy, ey, fy, fsy, gy, gsy, ay, asy, by, hcy;

	wire [7:0] scalex;
	wire [6:0] scaley;

	
	wire [2:0] ccolour, cscolour, dcolour, dscolour, ecolour, fcolour, fscolour, gcolour, gscolour, acolour, ascolour, bcolour, hccolour;

	wire [2:0] scalecolour;
	
	
	wire vEnable;
	wire [8:0]vRDiv;
	assign vEnable = (vRDiv == 9'b000000000)?1:0;
		
	vga_RateDivider vrd0 (
		.Clock(clock),
		.q(vRDiv)
	);
	
	reg bg;
	
	
// select which to draw

	always@(posedge clock)begin
	
	if(air != 2'b0)begin
		if(id == 4'd0) begin
			x <= scalex;
			y <= scaley;
			colour <= scalecolour;
		end
		if(id == 4'd1) begin
			x <= cx;
			y <= cy;
			colour <= ccolour;
		end
		else if(id == 4'd2) begin
			x <= csx;
			y <= csy;
			colour <= cscolour;
		end
		else if(id == 4'd3) begin
			x <= dx;
			y <= dy;
			colour <= dcolour;
		end
		else if(id == 4'd4) begin
			x <= dsx;
			y <= dsy;
			colour <= dscolour;
		end
		else if(id == 4'd5) begin
			x <= ex;
			y <= ey;
			colour <= ecolour;
		end
		else if(id == 4'd6) begin
			x <= fx;
			y <= fy;
			colour <= fcolour;
		end
		else if(id == 4'd7) begin
			x <= fsx;
			y <= fsy;
			colour <= fscolour;
		end
		else if(id == 4'd8) begin
			x <= gx;
			y <= gy;
			colour <= gcolour;
		end
		else if(id == 4'd9) begin
			x <= gsx;
			y <= gsy;
			colour <= gscolour;
		end
		else if(id == 4'd10) begin
			x <= ax;
			y <= ay;
			colour <= acolour;
		end
		else if(id == 4'd11) begin
			x <= asx;
			y <= asy;
			colour <= ascolour;
		end
		else if(id == 4'd12) begin
			x <= bx;
			y <= by;
			colour <= bcolour;
		end
		else if(id == 4'd13) begin
			x <= hcx;
			y <= hcy;
			colour <= hccolour;
		end
		else begin
			x <= scalex;
			y <= scaley;
			colour <= scalecolour;
		end
	end
	else begin
			x <= scalex;
			y <= scaley;
			colour <= scalecolour;
		end
	end

// Image RAM instantiations 

// D sharp RAM
	Dsharp dsram(
	.address(dsaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(dscolour));

// Line Note RAM C, E, G, B	
	linenote cram(
	.address(caddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(ccolour));
	
	linenote eram(
	.address(eaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(ecolour));
	
	linenote gram(
	.address(gaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(gcolour));
	
	linenote bram(
	.address(baddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(bcolour));

// line Sharp Note RAM Cs, Gs
	linesharpnote csram(
	.address(csaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(cscolour));
	
	linesharpnote gsram(
	.address(gsaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(gscolour));
	
// Space Note RAM D, F, A, HC
	spacenote dram(
	.address(daddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(dcolour));
	
	spacenote fram(
	.address(faddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(fcolour));
	
	spacenote aram(
	.address(aaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(acolour));
	
	spacenote hcram(
	.address(hcaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(hccolour));

// Space Sharp Note RAM Fs	As
	spacesharpnote fsram(
	.address(fsaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(fscolour));
	
	spacesharpnote asram(
	.address(asaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(ascolour));

// Scale/background RAM 	
	newscale2 sc(
	.address(scaddress),
	.clock(clock),
	.data(),
	.wren(0),
	.q(scalecolour));
	
// outlet/ drawing module instantiation 

// Background outlet
BGoutlet bgg (
	.clock(clock), 
	.resetn(resetn), 
	.xin(0),
	.yin(0),
	.width(8'd159),
	.height(7'd119),
	.x(scalex), 
	.y(scaley), 
	.wren(writeEn), 
	.address(scaddress)
);	
	
// note outlets

// C
outlet notec (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd33),
	.yin(7'd39),
	.width(8'd9),
	.height(7'd5),
	.x(cx), 
	.y(cy), 
	.wren(), 
	.address(caddress)
);	

// C sharp
outlet notecs (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd47),
	.yin(7'd35),
	.width(8'd14),
	.height(7'd12),
	.x(csx), 
	.y(csy), 
	.wren(), 
	.address(csaddress)
);	

// D
outlet noted (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd68),
	.yin(7'd35),
	.width(8'd7),
	.height(7'd5),
	.x(dx), 
	.y(dy), 
	.wren(), 
	.address(daddress)
);	

// D sharp
outlet noteds (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd82),
	.yin(7'd31),
	.width(8'd13),
	.height(7'd12),
	.x(dsx), 
	.y(dsy), 
	.wren(), 
	.address(dsaddress)
);	

// E
outlet notee (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd102),
	.yin(7'd32),
	.width(8'd9),
	.height(7'd5),
	.x(ex), 
	.y(ey), 
	.wren(), 
	.address(eaddress)
);	

// F
outlet notef (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd118),
	.yin(7'd28),
	.width(8'd7),
	.height(7'd5),
	.x(fx), 
	.y(fy), 
	.wren(), 
	.address(faddress)
);	

// F sharp
outlet notefs (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd132),
	.yin(7'd24),
	.width(8'd13),
	.height(7'd12),
	.x(fsx), 
	.y(fsy), 
	.wren(), 
	.address(fsaddress)
);	

// G
outlet noteg (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd33),
	.yin(7'd99),
	.width(8'd9),
	.height(7'd5),
	.x(gx), 
	.y(gy), 
	.wren(), 
	.address(gaddress)
);	

// G sharp
outlet notegs (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd50),
	.yin(7'd95),
	.width(8'd14),
	.height(7'd12),
	.x(gsx), 
	.y(gsy), 
	.wren(), 
	.address(gsaddress)
);	

// A
outlet notea (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd75),
	.yin(7'd95),
	.width(8'd7),
	.height(7'd5),
	.x(ax), 
	.y(ay), 
	.wren(), 
	.address(aaddress)
);	

// A sharp
outlet noteas (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd93),
	.yin(7'd91),
	.width(8'd13),
	.height(7'd12),
	.x(asx), 
	.y(asy), 
	.wren(), 
	.address(asaddress)
);	

// B
outlet noteb (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd116),
	.yin(7'd92),
	.width(8'd9),
	.height(7'd5),
	.x(bx), 
	.y(by), 
	.wren(), 
	.address(baddress)
);	

// C
outlet notehc (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd135),
	.yin(7'd88),
	.width(8'd7),
	.height(7'd5),
	.x(hcx), 
	.y(hcy), 
	.wren(), 
	.address(hcaddress)
);	


 
endmodule 
/*****************************************************************************
 *                          Note Select Module 				                    *
 *****************************************************************************/
module note_Select (clock, keys, airflow, audio_out_allowed, note, write, id);
	input clock;
	input [2:0] keys;
	input [1:0] airflow;
	input audio_out_allowed;
	
	output reg [9:0] note;
	output reg write;
	output reg [3:0] id;
	
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
					if (keys == 3'b000)begin			//Middle C 523
						note = c4_audio;
						id = 4'd1;
					end
					else if (keys == 3'b111)begin		//C# 554
						note = cs4_audio;
						id = 4'd2;
					end
					else if (keys == 3'b101)begin		//D 587
						note = d4_audio;
						id = 4'd3;
					end
					else if (keys == 3'b011)begin		//D# 622
						note = ds4_audio;
						id = 4'd4;
					end
					else if (keys == 3'b110)begin		//E 659
						note = e4_audio;
						id = 4'd5;
					end
					else if (keys == 3'b100)begin		//F 698
						note = f4_audio;
						id = 4'd6;
					end
					else if (keys == 3'b010)begin		//F# 740	
						note = fs4_audio;
						id = 4'd7;
					end
					else begin
						note = 10'b0;
						id = 4'd0;
					end
				end
			2'b10:							//case 2: level 2 airflow
				begin
					if (keys == 3'b000)begin			//G 784
						note = g4_audio;
						id = 4'd8;
					end
					else if (keys == 3'b011)begin		//G# 831	
						note = gs4_audio;
						id = 4'd9;
					end
					else if (keys == 3'b110)begin		//A 880
						note = a4_audio;
						id = 4'd10;
					end
					else if (keys == 3'b100)begin		//A# 932
						note = as4_audio;
						id = 4'd11;
					end
					else if (keys == 3'b010)begin		//B 988
						note = b4_audio;
						id = 4'd12;
					end
					else if (keys == 3'b001)begin
						note = c5_audio;
						id = 4'd13;
					end
					else begin
						note = 10'b0; 
						id = 4'd0;
					end
				end
			default: begin 
				note = 10'b0;
				id = 4'd0;
			end	//default case
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
 *                          Draw Note Module 					                 *
 *****************************************************************************/
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

		
/*****************************************************************************
 *                          Draw Background Module 			                 *
 *****************************************************************************/
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

/*****************************************************************************
 *                          VGA Rate Divider Module 			                 *
 *****************************************************************************/
		
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
