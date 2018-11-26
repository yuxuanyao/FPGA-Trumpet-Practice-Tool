// Part 2 skeleton

module VGAtest
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,							// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;					
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	wire resetn;
	assign resetn = ~KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(~resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x ),
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
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	reg note; 
	reg scalestaff;
	reg writeEnable; 
	
	reg [7:0] xselect;
	reg [6:0] yselect;
	reg [2:0] colourselect;
	
	wire [5:0] noteaddress;
	
	wire [7:0] gx, gsx, ax, asx, scalex;
	wire [6:0] gy, gsy, ay, asy, scaley;



	
	wire [2:0] gcolour;
	wire [2:0] gscolour;
	wire [2:0] acolour;
	wire [2:0] ascolour;
	wire [2:0] scalecolour;

/******************************************************************************/
/*				G works														
/******************************************************************************/
	
wire [6:0] gaddress;

	linenote g(
	.address(gaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(gcolour));
	
outlet g0 (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd35),
	.yin(7'd99),
	.width(8'd9),
	.height(7'd5),
	.x(gx), 
	.y(gy), 
	.wren(), 
	.address(gaddress)
);


/******************************************************************************/
/*				A works																			
/******************************************************************************/
//
wire [6:0] aaddress;

newspacenote a(
	.address(aaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(acolour));

outlet a0 (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd77),
	.yin(7'd95),
	.width(8'd7),
	.height(7'd5),
	.x(ax), 
	.y(ay), 
	.wren(), 
	.address(aaddress)
);

/******************************************************************************/
/*				Gsharp 	works																			
/******************************************************************************/

wire [7:0] gsaddress;

linesharpnote gs(
	.address(gsaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(gscolour));

outlet gso (
	.clock(vEnable), 
	.resetn(resetn), 
	.xin(8'd52),
	.yin(7'd95),
	.width(8'd14),
	.height(7'd12),
	.x(gsx), 
	.y(gsy), 
	.wren(), 
	.address(gsaddress)
);


/******************************************************************************/
/*				Asharp works																			
/******************************************************************************/

wire [7:0] asaddress;

spacesharpnote as(
	.address(asaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(ascolour));

outlet as0 (
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






/******************************************************************************/
/*				Background NEED TO BE CLOCK_50 !!!!! (everything else use enable)																				
/******************************************************************************/
wire [14:0] scaleaddress;
	
newscalestaff n0(
	.address(scaleaddress),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(scalecolour));

BGoutlet sc1 (
	.clock(CLOCK_50), 
	.resetn(resetn), 
	.xin(0),
	.yin(0),
	.width(159),
	.height(119),
	.x(scalex), 
	.y(scaley), 
	.wren(writeEn), 
	.address(scaleaddress)
);


always@(posedge CLOCK_50)begin
	if(~KEY[3]) begin
		xselect <= gx;
		yselect <= gy;
		colourselect <= gcolour;
	end
	else if(~KEY[2]) begin
		xselect <= gsx;
		yselect <= gsy;
		colourselect <= gscolour;
	end
	else if(~KEY[1]) begin
		xselect <= ax;
		yselect <= ay;
		colourselect <= acolour;
	end
	else begin
		xselect <= scalex;
		yselect <= scaley;
		colourselect <= scalecolour;	
	end

end
	
assign x = xselect;	
assign y = yselect;
assign colour = colourselect;	

wire vEnable;
wire [8:0]vRDiv;
assign vEnable = (vRDiv == 9'b000000000)?1:0;
	
vga_RateDivider vrd0 (
	.Clock(CLOCK_50),
	.q(vRDiv)
);
	
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
