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
			.x(x + 7'd34),
			.y(y + 7'd99),
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
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	reg note; 
	reg scalestaff;
	reg writeEnable; 
	reg [7:0] x_counter;
	reg [6:0] y_counter;

	
	wire [5:0] address;
	reg [5:0] address_count;
	wire x_counter_clear;
	wire y_counter_clear;
	
	linenote s0(
	.address(address),
	.clock(CLOCK_50),
	.data(),
	.wren(0),
	.q(colour));
	
//always@(posedge CLOCK_50)begin
//	if(~KEY[3])
//		writeEnable <= 1;
//	else
//		writeEnable <= 0;
//end	


//always@(posedge CLOCK_50) begin
//	if(resetn) begin
//		x_counter <= 0;
//		y_counter <= 0;
//		address_count <= 0;
//	end
//	
//	if(address_count == 14'd47) begin
//		address_count <= 0;
//	end
//	else
//		address_count <= address_count + 1;
		
//		
//	if(x_counter_clear) begin
//		x_counter <= 0;
//		y_counter <= y_counter + 1;
//	end
//	else if(x_counter_clear && y_counter_clear) begin
//		x_counter <= 0;
//		y_counter <= 0;
//	end
//	else begin
//		x_counter <= x_counter + 1;
//	end
//	
//	address_count <= x_counter + y_counter *8;
//	
//end

outlet o1 (.clock(vEnable), .resetn(resetn), .x(x), .y(y), .wren(writeEn), .address(address));



//assign x_counter_clear = (x_counter == 8'd7);
//assign y_counter_clear = (y_counter == 7'd5);
//assign writeEn = address <= 47;//writeEnable;
//assign address = address_count;
//assign x = x_counter;
//assign y = y_counter;
	
	wire vEnable;
	wire [8:0]vRDiv;
	assign vEnable = (vRDiv == 9'b000000000)?1:0;
	
	vga_RateDivider vrd0 (
		.Clock(CLOCK_50),
		.q(vRDiv)
		);
	
endmodule

module outlet(clock, resetn, x, y, wren, address);
	input clock, resetn;
	output [7:0] x;
	output [6:0] y;
	output [5:0] address;
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
	
	assign xCounter_clear = (xCounter == (8'd9));


	always @(posedge clock)
	begin
		if (resetn)
			yCounter <= 7'd0;
		else if (xCounter_clear && yCounter_clear)
			yCounter <= 7'd0;
		else if (xCounter_clear)		//Increment when x counter resets
			yCounter <= yCounter + 1'b1;
	end
	
	assign yCounter_clear = (yCounter == (7'd5)); 
	
	assign x = xCounter;
	assign y = yCounter;
	assign address = (x + (y*10));
	assign wren = (address <= 59);
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