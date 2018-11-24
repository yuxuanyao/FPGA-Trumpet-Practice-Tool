
module C (
	// Inputs
	CLOCK_50,
	KEY,

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
	SW
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
input		[3:0]	SW;

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
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;



// Internal Registers


wire [15:0] audio_from_ram;
wire [15:0] address_count;

reg [15:0] address_count_reg;


last3 memory1(
.address(address_count),
.clock(CLOCK_50),
.data(),
.wren(1'b0),
.q(audio_from_ram));


reg write;

always @(posedge CLOCK_50) begin
		if(address_count_reg >= 16'd7535 && audio_out_allowed) begin
			address_count_reg <= 16'b0;
			write <= 0;
		end 
		else if(audio_out_allowed) 
			address_count_reg <= address_count_reg + 1;
			write <= 1;
	end


assign address_count = address_count_reg;
assign write_audio_out			= write & audio_out_allowed;



wire [31:0] sound;


assign sound = {audio_from_ram, 16'b0};


assign read_audio_in = audio_in_available & audio_out_allowed;



assign left_channel_audio_out	= sound;
assign right_channel_audio_out	= sound;

//assign write_audio_out			= audio_in_available & audio_out_allowed;






 
// Audio controller
Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out ),
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

endmodule


// rate divider
module RateDivider (Clock, q);
	input Clock;
	
	output reg [25:0] q; // declare q

	always @(posedge Clock) // triggered every time clock rises
	begin
		if (q == 26'b0) // when q is the min value for the counter
			//Real code value
			q <= 26'd1042;//something bit something; // q reset to 0

		else
			q <= q - 1; // decrement q
	end

endmodule
