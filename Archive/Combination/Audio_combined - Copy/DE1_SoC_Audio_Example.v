
module DE1_SoC_Audio_Example (
	// Inputs
	CLOCK_50,
	KEY,
	LEDR,

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
input		[7:0] LEDR;

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

reg [18:0] delay_cnt;
wire [18:0] delay;

reg snd;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
	if(delay_cnt == delay) begin
		delay_cnt <= 0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

assign delay = {pitch};

wire [31:0] sound = (delay_cnt == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;


assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= sound;
assign right_channel_audio_out	= sound;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
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



	wire [18:0]pitch;
	
	note_Select u0 (
		.keys(~KEY[3:1]),
		.airflow(SW[1:0]),
		.note(pitch)
		);

	//assign pitch = ~note;
	
	//mic_check m0(left_channel_audio_in, right_channel_audio_in, LEDR[0]);

endmodule



module note_Select (keys, airflow, note);
	input [2:0] keys;
	input [1:0]airflow;
	
	output reg [18:0] note;
	
	always @(*)	//declare always block
	begin
		case (airflow[1:0])
			//alternate code used for testing
			2'b00: note = 5'b00000;	//case 0: no airflow
			2'b01:
				begin
					if (keys == 3'b000)			//Middle C 523
						note = 19'd191204;
					else if (keys == 3'b111)		//C# 554
						note = 19'd180505;
					else if (keys == 3'b101)		//D 587
						note = 19'd170357;
					else if (keys == 3'b011)		//D# 622
						note = 19'd160771;
					else if (keys == 3'b110)		//E 659
						note = 19'd151745;
					else if (keys == 3'b100)		//F 698
						note = 19'd143266;
					else if (keys == 3'b010)		//F# 740	
						note = 19'd135135;
					else
						note = 19'b0;
				end
			2'b10:
				begin
					if (keys == 3'b000)			//G 784
						note = 19'd127551;
					else if (keys == 3'b011)		//G# 831	
						note = 19'd120336;
					else if (keys == 3'b110)		//A 880
						note = 19'd113636;
					else if (keys == 3'b100)		//A# 932
						note = 19'd107296;
					else if (keys == 3'b010)		//B 988
						note = 19'd101214;
					else
						note = 19'b0;
				end
			2'b11: 
				begin
					if (keys == 3'b000)			//High C 1047
						note = 19'd95510;
					else if (keys == 3'b110)		//C# 	1109
						note = 19'd90171;
					else if (keys == 3'b100)		//D 1175
						note = 19'd85106;
					else if (keys == 3'b010)		//D# 1245
						note = 19'd80321;
					else
						note = 19'b0;
				end
			default: note = 19'b0;	//default case
		endcase
	end

endmodule



module mic_check (channel1, channel2, q);
	input [31:0] channel1;
	input [31:0] channel2;
	output reg q;
	
	always @(*)	//declare always block
	begin
		if (channel1 > 32'd10000000)
			q=1;
		else if (channel1 < -32'd10000000)
			q=1;
		else if (channel2 < -32'd10000000)
			q=1;
		else if (channel1 > 32'd10000000)
			q=1;
		else 
			q=0;
	end

endmodule

