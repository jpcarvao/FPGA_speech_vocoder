

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;
assign HEX3 = 7'b1111111;
assign HEX2 = 7'b1111111;
assign HEX1 = 7'b1111111;
assign HEX0 = 7'b1111111;


wire [31:0] vga_out_base_address = 32'h0000_0000;  // vga base addr

//Data to be written to VGA
reg   [7:0] vga_sram_writedata;

//Address to write to
reg  [31:0] vga_sram_address; 

//Should VGA be written to?
reg         vga_sram_write;

wire vga_sram_clken = 1'b1;
wire vga_sram_chipselect = 1'b1;
reg [3:0] vga_state;

reg [9:0] vga_x_cood, vga_y_cood;

reg [9:0] vga_y_audio_center = 10'd120; 
reg [9:0] vga_y_spect_top = 10'd240; 

//=======================================================
// Audio controller for AVALON bus-master
//=======================================================
// computes DDS for sine wave and fills audio FIFO
// reads audio FIFO and loops it back
// MUST configure (in Qsys) Audio Config module:
// -- Line in to ADC
// -- uncheck both bypass options
// The audio_input_ready signal goes high for one
// cycle when there is new audio input data
// --
// 32-bit data is on 
//   right_audio_input, left_audio_input ;
// Every write requires 32-bit data on 
//   right_audio_output, left_audio_output ;

reg [31:0] bus_addr ; // Avalon address
// see 
// ftp://ftp.altera.com/up/pub/Altera_Material/15.1/University_Program_IP_Cores/Audio_Video/Audio.pdf
// for addresses
wire [31:0] audio_base_address = 32'h00003040 ;  // Avalon address
wire [31:0] audio_fifo_address = 32'h00003044 ;  // Avalon address +4 offset
wire [31:0] audio_data_left_address = 32'h00003048 ;  // Avalon address +8
wire [31:0] audio_data_right_address = 32'h0000304c ;  // Avalon address +12
reg [3:0] bus_byte_enable ; // four bit byte read/write mask
reg bus_read  ;       // high when requesting data
reg bus_write ;      //  high when writing data
reg [31:0] bus_write_data ; //  data to send to Avalog bus
wire bus_ack  ;       //  Avalon bus raises this when done
wire [31:0] bus_read_data ; // data from Avalon bus
reg [30:0] timer ;
reg [3:0] state ;

wire state_clock ;
wire reset;

assign reset = ~KEY[0];

// current free words in audio interface
reg [7:0] fifo_space ;
// debug check of space
assign LEDR = fifo_space ;

// audio input/output from audio module FIFO
reg [15:0] right_audio_input, left_audio_input ;
reg audio_input_ready ;
wire [15:0] right_audio_output, left_audio_output ;

reg [9:0] spect_height = 10'd7; 


// For audio loopback, or filtering
assign right_audio_output = SW[9] ? debug_right9 :
									 SW[8] ? debug_right8 :	
									 SW[7] ? debug_right7 :
									 SW[6] ? debug_right6 :
									 SW[5] ? debug_right5 : 
									 SW[4] ? right_filter_output3:
									 SW[0] ? right_filter_output : right_audio_input ;
									 
assign left_audio_output = SW[9] ? debug_left9 :
									SW[8] ? debug_left8 :	
									SW[7] ? debug_left7 :
									SW[6] ? debug_left6 :
									SW[5] ? debug_left5 : 
									SW[4] ? left_filter_output3:
								   SW[0] ? left_filter_output : left_audio_input ;
/*									
// For audio loopback, or filtering
assign right_audio_output = SW[9] ? right_audio_input : right_filter_output;
									 
assign left_audio_output = SW[9] ? left_audio_input : left_filter_output;*/
									
wire [2:0] pitch_shift = SW[3:1];							
									
// DDS update signal for testing
reg [31:0] dds_accum ;
// DDS LUT
wire [15:0] sine_out ;
// update phase accumulator
// sync to audio data rate (48kHz) using audio_input_ready signal
always @(posedge CLOCK_50) begin //CLOCK_50
	// Fout = (sample_rate)/(2^32)*{SW[9:0], 16'b0}
	// then Fout=48000/(2^32)*(2^25) = 375 Hz
	if (audio_input_ready) dds_accum <= dds_accum + {SW[9:0], 16'b0} ;	
end
// DDS sine wave ROM
sync_rom sineTable(CLOCK_50, dds_accum[31:24], sine_out);

// get some signals exposed
// connect bus master signals to i/o for probes
assign GPIO_0[0] = bus_write ;
assign GPIO_0[1] = bus_read ;
assign GPIO_0[2] = bus_ack ;
assign GPIO_0[3] = audio_input_ready ;
assign GPIO_0[4] = sine_out;


assign GPIO_0[5] = right_filter_output3;
assign GPIO_0[7] = left_filter_output3;



// ======================================================
// === Filters ==========================================
// ======================================================


assign right_filter_output = 
 right_filter_output1 + right_filter_output2 + right_filter_output3 + right_filter_output4                               
 + right_filter_output5 + right_filter_output6 + right_filter_output7 + right_filter_output8                               
 + right_filter_output9 + right_filter_output10 + right_filter_output11 + right_filter_output12                               
 + right_filter_output13 + right_filter_output14 + right_filter_output15 + right_filter_output16                               
 + right_filter_output17 + right_filter_output18 + right_filter_output19 + right_filter_output20                               
 + right_filter_output21 + right_filter_output22 + right_filter_output23 + right_filter_output24                               
 + right_filter_output25 + right_filter_output26 + right_filter_output27 + right_filter_output28                               
 + right_filter_output29 + right_filter_output30 + right_filter_output31 + right_filter_output32; 
 
 
assign left_filter_output = 
 left_filter_output1 + left_filter_output2 + left_filter_output3 + left_filter_output4                               
 + left_filter_output5 + left_filter_output6 + left_filter_output7 + left_filter_output8                               
 + left_filter_output9 + left_filter_output10 + left_filter_output11 + left_filter_output12                               
 + left_filter_output13 + left_filter_output14 + left_filter_output15 + left_filter_output16                               
 + left_filter_output17 + left_filter_output18 + left_filter_output19 + left_filter_output20                               
 + left_filter_output21 + left_filter_output22 + left_filter_output23 + left_filter_output24                               
 + left_filter_output25 + left_filter_output26 + left_filter_output27 + left_filter_output28                               
 + left_filter_output29 + left_filter_output30 + left_filter_output31 + left_filter_output32; 



wire [15:0]    right_filter_output , left_filter_output, 
               right_filter_output1 , left_filter_output1, 
               right_filter_output2 , left_filter_output2, 
               right_filter_output3 , left_filter_output3, 
               right_filter_output4 , left_filter_output4, 
               right_filter_output5 , left_filter_output5, 
               right_filter_output6 , left_filter_output6, 
               right_filter_output7 , left_filter_output7, 
               right_filter_output8 , left_filter_output8, 
               right_filter_output9 , left_filter_output9, 
               right_filter_output10 , left_filter_output10, 
               right_filter_output11 , left_filter_output11, 
               right_filter_output12 , left_filter_output12, 
               right_filter_output13 , left_filter_output13, 
               right_filter_output14 , left_filter_output14, 
               right_filter_output15 , left_filter_output15, 
               right_filter_output16 , left_filter_output16, 
               right_filter_output17 , left_filter_output17, 
               right_filter_output18 , left_filter_output18, 
               right_filter_output19 , left_filter_output19, 
               right_filter_output20 , left_filter_output20, 
               right_filter_output21 , left_filter_output21, 
               right_filter_output22 , left_filter_output22, 
               right_filter_output23 , left_filter_output23, 
               right_filter_output24 , left_filter_output24, 
               right_filter_output25 , left_filter_output25, 
               right_filter_output26 , left_filter_output26, 
               right_filter_output27 , left_filter_output27, 
               right_filter_output28 , left_filter_output28, 
               right_filter_output29 , left_filter_output29, 
               right_filter_output30 , left_filter_output30, 
               right_filter_output31 , left_filter_output31, 
               right_filter_output32 , left_filter_output32; 
 
 
 
 
//Filter 1 Right 
//Filter 1: frequency=299.991723 
//Filter 1: BW=0.035018 
//Filter 1: shifted freq=299.991723 
IIR2_18bit_fixed filter1_RIGHT( 
     .audio_out (right_filter_output1), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd26842805 <<< pitch_shift), 
     .b1 (18'sd45), 
     .b2 (18'sd0), 
     .b3 (-18'sd45), 
     .a2 (18'sd130880), 
     .a3 (-18'sd65445), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 1 Left 
//Filter 1: frequency=299.991723 
//Filter 1: BW=0.035018 
//Filter 1: shifted freq=299.991723 
IIR2_18bit_fixed filter1_LEFT( 
     .audio_out (left_filter_output1), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd26842805 <<< pitch_shift), 
     .b1 (18'sd45), 
     .b2 (18'sd0), 
     .b3 (-18'sd45), 
     .a2 (18'sd130880), 
     .a3 (-18'sd65445), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 2 Right 
//Filter 2: frequency=347.381037 
//Filter 2: BW=0.035015 
//Filter 2: shifted freq=347.381037 
IIR2_18bit_fixed filter2_RIGHT( 
     .audio_out (right_filter_output2), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd31083129 <<< pitch_shift), 
     .b1 (18'sd52), 
     .b2 (18'sd0), 
     .b3 (-18'sd52), 
     .a2 (18'sd130832), 
     .a3 (-18'sd65431), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 2 Left 
//Filter 2: frequency=347.381037 
//Filter 2: BW=0.035015 
//Filter 2: shifted freq=347.381037 
IIR2_18bit_fixed filter2_LEFT( 
     .audio_out (left_filter_output2), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd31083129 <<< pitch_shift), 
     .b1 (18'sd52), 
     .b2 (18'sd0), 
     .b3 (-18'sd52), 
     .a2 (18'sd130832), 
     .a3 (-18'sd65431), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 3 Right 
//Filter 3: frequency=397.016117 
//Filter 3: BW=0.035013 
//Filter 3: shifted freq=397.016117 
IIR2_18bit_fixed filter3_RIGHT( 
     .audio_out (right_filter_output3), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd35524401 <<< pitch_shift), 
     .b1 (18'sd59), 
     .b2 (18'sd0), 
     .b3 (-18'sd59), 
     .a2 (18'sd130776), 
     .a3 (-18'sd65416), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset),
	  .voice_in_abs( debug_right9),
	  .voice_low_new(debug_right8),
	  .wave_ex(debug_right7),
	  .modulated_voice(debug_right6),
	  .f1_mac_new(debug_right5)  
) ; //end filter 
 

 wire signed[15:0] debug_right9, debug_left9;
 wire signed[15:0] debug_right8, debug_left8;
 wire 		[15:0] debug_right7, debug_left7;
 wire signed[15:0] debug_right6, debug_left6;
 wire signed[15:0] debug_right5, debug_left5;

 
//Filter 3 Left 
//Filter 3: frequency=397.016117 
//Filter 3: BW=0.035013 
//Filter 3: shifted freq=397.016117 
IIR2_18bit_fixed filter3_LEFT( 
     .audio_out (left_filter_output3), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd35524401 <<< pitch_shift), 
     .b1 (18'sd59), 
     .b2 (18'sd0), 
     .b3 (-18'sd59), 
     .a2 (18'sd130776), 
     .a3 (-18'sd65416), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset),
	  .voice_in_abs( debug_left9),
	  .voice_low_new(debug_left8),
	  .wave_ex(debug_left7),
	  .modulated_voice(debug_left6),
	  .f1_mac_new(debug_left5)
 
 
) ; //end filter 
 
 
//Filter 4 Right 
//Filter 4: frequency=449.003389 
//Filter 4: BW=0.035012 
//Filter 4: shifted freq=449.003389 
IIR2_18bit_fixed filter4_RIGHT( 
     .audio_out (right_filter_output4), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd40176143 <<< pitch_shift), 
     .b1 (18'sd67), 
     .b2 (18'sd0), 
     .b3 (-18'sd67), 
     .a2 (18'sd130711), 
     .a3 (-18'sd65401), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 4 Left 
//Filter 4: frequency=449.003389 
//Filter 4: BW=0.035012 
//Filter 4: shifted freq=449.003389 
IIR2_18bit_fixed filter4_LEFT( 
     .audio_out (left_filter_output4), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd40176143 <<< pitch_shift), 
     .b1 (18'sd67), 
     .b2 (18'sd0), 
     .b3 (-18'sd67), 
     .a2 (18'sd130711), 
     .a3 (-18'sd65401), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 5 Right 
//Filter 5: frequency=503.454323 
//Filter 5: BW=0.035010 
//Filter 5: shifted freq=503.454323 
IIR2_18bit_fixed filter5_RIGHT( 
     .audio_out (right_filter_output5), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd45048330 <<< pitch_shift), 
     .b1 (18'sd75), 
     .b2 (18'sd0), 
     .b3 (-18'sd75), 
     .a2 (18'sd130636), 
     .a3 (-18'sd65384), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 5 Left 
//Filter 5: frequency=503.454323 
//Filter 5: BW=0.035010 
//Filter 5: shifted freq=503.454323 
IIR2_18bit_fixed filter5_LEFT( 
     .audio_out (left_filter_output5), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd45048330 <<< pitch_shift), 
     .b1 (18'sd75), 
     .b2 (18'sd0), 
     .b3 (-18'sd75), 
     .a2 (18'sd130636), 
     .a3 (-18'sd65384), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 6 Right 
//Filter 6: frequency=560.485670 
//Filter 6: BW=0.035009 
//Filter 6: shifted freq=560.485670 
IIR2_18bit_fixed filter6_RIGHT( 
     .audio_out (right_filter_output6), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd50151409 <<< pitch_shift), 
     .b1 (18'sd84), 
     .b2 (18'sd0), 
     .b3 (-18'sd84), 
     .a2 (18'sd130551), 
     .a3 (-18'sd65367), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 6 Left 
//Filter 6: frequency=560.485670 
//Filter 6: BW=0.035009 
//Filter 6: shifted freq=560.485670 
IIR2_18bit_fixed filter6_LEFT( 
     .audio_out (left_filter_output6), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd50151409 <<< pitch_shift), 
     .b1 (18'sd84), 
     .b2 (18'sd0), 
     .b3 (-18'sd84), 
     .a2 (18'sd130551), 
     .a3 (-18'sd65367), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 7 Right 
//Filter 7: frequency=620.219716 
//Filter 7: BW=0.035008 
//Filter 7: shifted freq=620.219716 
IIR2_18bit_fixed filter7_RIGHT( 
     .audio_out (right_filter_output7), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd55496321 <<< pitch_shift), 
     .b1 (18'sd93), 
     .b2 (18'sd0), 
     .b3 (-18'sd93), 
     .a2 (18'sd130455), 
     .a3 (-18'sd65349), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 7 Left 
//Filter 7: frequency=620.219716 
//Filter 7: BW=0.035008 
//Filter 7: shifted freq=620.219716 
IIR2_18bit_fixed filter7_LEFT( 
     .audio_out (left_filter_output7), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd55496321 <<< pitch_shift), 
     .b1 (18'sd93), 
     .b2 (18'sd0), 
     .b3 (-18'sd93), 
     .a2 (18'sd130455), 
     .a3 (-18'sd65349), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 8 Right 
//Filter 8: frequency=682.784541 
//Filter 8: BW=0.035008 
//Filter 8: shifted freq=682.784541 
IIR2_18bit_fixed filter8_RIGHT( 
     .audio_out (right_filter_output8), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd61094527 <<< pitch_shift), 
     .b1 (18'sd102), 
     .b2 (18'sd0), 
     .b3 (-18'sd102), 
     .a2 (18'sd130345), 
     .a3 (-18'sd65331), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 8 Left 
//Filter 8: frequency=682.784541 
//Filter 8: BW=0.035008 
//Filter 8: shifted freq=682.784541 
IIR2_18bit_fixed filter8_LEFT( 
     .audio_out (left_filter_output8), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd61094527 <<< pitch_shift), 
     .b1 (18'sd102), 
     .b2 (18'sd0), 
     .b3 (-18'sd102), 
     .a2 (18'sd130345), 
     .a3 (-18'sd65331), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 9 Right 
//Filter 9: frequency=748.314295 
//Filter 9: BW=0.035007 
//Filter 9: shifted freq=748.314295 
IIR2_18bit_fixed filter9_RIGHT( 
     .audio_out (right_filter_output9), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd66958030 <<< pitch_shift), 
     .b1 (18'sd112), 
     .b2 (18'sd0), 
     .b3 (-18'sd112), 
     .a2 (18'sd130220), 
     .a3 (-18'sd65311), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter 9 Left 
//Filter 9: frequency=748.314295 
//Filter 9: BW=0.035007 
//Filter 9: shifted freq=748.314295 
IIR2_18bit_fixed filter9_LEFT( 
     .audio_out (left_filter_output9), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd66958030 <<< pitch_shift), 
     .b1 (18'sd112), 
     .b2 (18'sd0), 
     .b3 (-18'sd112), 
     .a2 (18'sd130220), 
     .a3 (-18'sd65311), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter10 Right 
//Filter10: frequency=816.949484 
//Filter10: BW=0.035006 
//Filter10: shifted freq=816.949484 
IIR2_18bit_fixed filter10_RIGHT( 
     .audio_out (right_filter_output10), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd73099402 <<< pitch_shift), 
     .b1 (18'sd122), 
     .b2 (18'sd0), 
     .b3 (-18'sd122), 
     .a2 (18'sd130080), 
     .a3 (-18'sd65291), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter10 Left 
//Filter10: frequency=816.949484 
//Filter10: BW=0.035006 
//Filter10: shifted freq=816.949484 
IIR2_18bit_fixed filter10_LEFT( 
     .audio_out (left_filter_output10), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd73099402 <<< pitch_shift), 
     .b1 (18'sd122), 
     .b2 (18'sd0), 
     .b3 (-18'sd122), 
     .a2 (18'sd130080), 
     .a3 (-18'sd65291), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter11 Right 
//Filter11: frequency=888.837275 
//Filter11: BW=0.035006 
//Filter11: shifted freq=888.837275 
IIR2_18bit_fixed filter11_RIGHT( 
     .audio_out (right_filter_output11), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd79531813 <<< pitch_shift), 
     .b1 (18'sd133), 
     .b2 (18'sd0), 
     .b3 (-18'sd133), 
     .a2 (18'sd129921), 
     .a3 (-18'sd65269), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter11 Left 
//Filter11: frequency=888.837275 
//Filter11: BW=0.035006 
//Filter11: shifted freq=888.837275 
IIR2_18bit_fixed filter11_LEFT( 
     .audio_out (left_filter_output11), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd79531813 <<< pitch_shift), 
     .b1 (18'sd133), 
     .b2 (18'sd0), 
     .b3 (-18'sd133), 
     .a2 (18'sd129921), 
     .a3 (-18'sd65269), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter12 Right 
//Filter12: frequency=964.131808 
//Filter12: BW=0.035005 
//Filter12: shifted freq=964.131808 
IIR2_18bit_fixed filter12_RIGHT( 
     .audio_out (right_filter_output12), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd86269054 <<< pitch_shift), 
     .b1 (18'sd144), 
     .b2 (18'sd0), 
     .b3 (-18'sd144), 
     .a2 (18'sd129743), 
     .a3 (-18'sd65247), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter12 Left 
//Filter12: frequency=964.131808 
//Filter12: BW=0.035005 
//Filter12: shifted freq=964.131808 
IIR2_18bit_fixed filter12_LEFT( 
     .audio_out (left_filter_output12), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd86269054 <<< pitch_shift), 
     .b1 (18'sd144), 
     .b2 (18'sd0), 
     .b3 (-18'sd144), 
     .a2 (18'sd129743), 
     .a3 (-18'sd65247), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter13 Right 
//Filter13: frequency=1042.994526 
//Filter13: BW=0.035005 
//Filter13: shifted freq=1042.994526 
IIR2_18bit_fixed filter13_RIGHT( 
     .audio_out (right_filter_output13), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd93325570 <<< pitch_shift), 
     .b1 (18'sd156), 
     .b2 (18'sd0), 
     .b3 (-18'sd156), 
     .a2 (18'sd129543), 
     .a3 (-18'sd65223), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter13 Left 
//Filter13: frequency=1042.994526 
//Filter13: BW=0.035005 
//Filter13: shifted freq=1042.994526 
IIR2_18bit_fixed filter13_LEFT( 
     .audio_out (left_filter_output13), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd93325570 <<< pitch_shift), 
     .b1 (18'sd156), 
     .b2 (18'sd0), 
     .b3 (-18'sd156), 
     .a2 (18'sd129543), 
     .a3 (-18'sd65223), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter14 Right 
//Filter14: frequency=1125.594525 
//Filter14: BW=0.035005 
//Filter14: shifted freq=1125.594525 
IIR2_18bit_fixed filter14_RIGHT( 
     .audio_out (right_filter_output14), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd100716493 <<< pitch_shift), 
     .b1 (18'sd168), 
     .b2 (18'sd0), 
     .b3 (-18'sd168), 
     .a2 (18'sd129318), 
     .a3 (-18'sd65198), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter14 Left 
//Filter14: frequency=1125.594525 
//Filter14: BW=0.035005 
//Filter14: shifted freq=1125.594525 
IIR2_18bit_fixed filter14_LEFT( 
     .audio_out (left_filter_output14), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd100716493 <<< pitch_shift), 
     .b1 (18'sd168), 
     .b2 (18'sd0), 
     .b3 (-18'sd168), 
     .a2 (18'sd129318), 
     .a3 (-18'sd65198), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter15 Right 
//Filter15: frequency=1212.108914 
//Filter15: BW=0.035004 
//Filter15: shifted freq=1212.108914 
IIR2_18bit_fixed filter15_RIGHT( 
     .audio_out (right_filter_output15), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd108457670 <<< pitch_shift), 
     .b1 (18'sd181), 
     .b2 (18'sd0), 
     .b3 (-18'sd181), 
     .a2 (18'sd129067), 
     .a3 (-18'sd65173), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter15 Left 
//Filter15: frequency=1212.108914 
//Filter15: BW=0.035004 
//Filter15: shifted freq=1212.108914 
IIR2_18bit_fixed filter15_LEFT( 
     .audio_out (left_filter_output15), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd108457670 <<< pitch_shift), 
     .b1 (18'sd181), 
     .b2 (18'sd0), 
     .b3 (-18'sd181), 
     .a2 (18'sd129067), 
     .a3 (-18'sd65173), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 

//Filter16 Right 
//Filter16: frequency=1302.723195 
//Filter16: BW=0.035004 
//Filter16: shifted freq=1302.723195 
IIR2_18bit_fixed filter16_RIGHT( 
     .audio_out (right_filter_output16), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd116565698 <<< pitch_shift), 
     .b1 (18'sd195), 
     .b2 (18'sd0), 
     .b3 (-18'sd195), 
     .a2 (18'sd128787), 
     .a3 (-18'sd65145), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter16 Left 
//Filter16: frequency=1302.723195 
//Filter16: BW=0.035004 
//Filter16: shifted freq=1302.723195 
IIR2_18bit_fixed filter16_LEFT( 
     .audio_out (left_filter_output16), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd116565698 <<< pitch_shift), 
     .b1 (18'sd195), 
     .b2 (18'sd0), 
     .b3 (-18'sd195), 
     .a2 (18'sd128787), 
     .a3 (-18'sd65145), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter17 Right 
//Filter17: frequency=1397.631659 
//Filter17: BW=0.035004 
//Filter17: shifted freq=1397.631659 
IIR2_18bit_fixed filter17_RIGHT( 
     .audio_out (right_filter_output17), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd125057964 <<< pitch_shift), 
     .b1 (18'sd209), 
     .b2 (18'sd0), 
     .b3 (-18'sd209), 
     .a2 (18'sd128473), 
     .a3 (-18'sd65117), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter17 Left 
//Filter17: frequency=1397.631659 
//Filter17: BW=0.035004 
//Filter17: shifted freq=1397.631659 
IIR2_18bit_fixed filter17_LEFT( 
     .audio_out (left_filter_output17), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd125057964 <<< pitch_shift), 
     .b1 (18'sd209), 
     .b2 (18'sd0), 
     .b3 (-18'sd209), 
     .a2 (18'sd128473), 
     .a3 (-18'sd65117), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter18 Right 
//Filter18: frequency=1497.037808 
//Filter18: BW=0.035004 
//Filter18: shifted freq=1497.037808 
IIR2_18bit_fixed filter18_RIGHT( 
     .audio_out (right_filter_output18), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd133952676 <<< pitch_shift), 
     .b1 (18'sd223), 
     .b2 (18'sd0), 
     .b3 (-18'sd223), 
     .a2 (18'sd128124), 
     .a3 (-18'sd65088), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter18 Left 
//Filter18: frequency=1497.037808 
//Filter18: BW=0.035004 
//Filter18: shifted freq=1497.037808 
IIR2_18bit_fixed filter18_LEFT( 
     .audio_out (left_filter_output18), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd133952676 <<< pitch_shift), 
     .b1 (18'sd223), 
     .b2 (18'sd0), 
     .b3 (-18'sd223), 
     .a2 (18'sd128124), 
     .a3 (-18'sd65088), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter19 Right 
//Filter19: frequency=1601.154785 
//Filter19: BW=0.035003 
//Filter19: shifted freq=1601.154785 
IIR2_18bit_fixed filter19_RIGHT( 
     .audio_out (right_filter_output19), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd143268905 <<< pitch_shift), 
     .b1 (18'sd239), 
     .b2 (18'sd0), 
     .b3 (-18'sd239), 
     .a2 (18'sd127735), 
     .a3 (-18'sd65056), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter19 Left 
//Filter19: frequency=1601.154785 
//Filter19: BW=0.035003 
//Filter19: shifted freq=1601.154785 
IIR2_18bit_fixed filter19_LEFT( 
     .audio_out (left_filter_output19), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd143268905 <<< pitch_shift), 
     .b1 (18'sd239), 
     .b2 (18'sd0), 
     .b3 (-18'sd239), 
     .a2 (18'sd127735), 
     .a3 (-18'sd65056), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter20 Right 
//Filter20: frequency=1710.205836 
//Filter20: BW=0.035003 
//Filter20: shifted freq=1710.205836 
IIR2_18bit_fixed filter20_RIGHT( 
     .audio_out (right_filter_output20), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd153026628 <<< pitch_shift), 
     .b1 (18'sd255), 
     .b2 (18'sd0), 
     .b3 (-18'sd255), 
     .a2 (18'sd127303), 
     .a3 (-18'sd65024), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter20 Left 
//Filter20: frequency=1710.205836 
//Filter20: BW=0.035003 
//Filter20: shifted freq=1710.205836 
IIR2_18bit_fixed filter20_LEFT( 
     .audio_out (left_filter_output20), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd153026628 <<< pitch_shift), 
     .b1 (18'sd255), 
     .b2 (18'sd0), 
     .b3 (-18'sd255), 
     .a2 (18'sd127303), 
     .a3 (-18'sd65024), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter21 Right 
//Filter21: frequency=1824.424783 
//Filter21: BW=0.035003 
//Filter21: shifted freq=1824.424783 
IIR2_18bit_fixed filter21_RIGHT( 
     .audio_out (right_filter_output21), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd163246766 <<< pitch_shift), 
     .b1 (18'sd272), 
     .b2 (18'sd0), 
     .b3 (-18'sd272), 
     .a2 (18'sd126823), 
     .a3 (-18'sd64990), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter21 Left 
//Filter21: frequency=1824.424783 
//Filter21: BW=0.035003 
//Filter21: shifted freq=1824.424783 
IIR2_18bit_fixed filter21_LEFT( 
     .audio_out (left_filter_output21), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd163246766 <<< pitch_shift), 
     .b1 (18'sd272), 
     .b2 (18'sd0), 
     .b3 (-18'sd272), 
     .a2 (18'sd126823), 
     .a3 (-18'sd64990), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter22 Right 
//Filter22: frequency=1944.056533 
//Filter22: BW=0.035003 
//Filter22: shifted freq=1944.056533 
IIR2_18bit_fixed filter22_RIGHT( 
     .audio_out (right_filter_output22), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd173951234 <<< pitch_shift), 
     .b1 (18'sd290), 
     .b2 (18'sd0), 
     .b3 (-18'sd290), 
     .a2 (18'sd126289), 
     .a3 (-18'sd64954), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter22 Left 
//Filter22: frequency=1944.056533 
//Filter22: BW=0.035003 
//Filter22: shifted freq=1944.056533 
IIR2_18bit_fixed filter22_LEFT( 
     .audio_out (left_filter_output22), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd173951234 <<< pitch_shift), 
     .b1 (18'sd290), 
     .b2 (18'sd0), 
     .b3 (-18'sd290), 
     .a2 (18'sd126289), 
     .a3 (-18'sd64954), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter23 Right 
//Filter23: frequency=2069.357596 
//Filter23: BW=0.035003 
//Filter23: shifted freq=2069.357596 
IIR2_18bit_fixed filter23_RIGHT( 
     .audio_out (right_filter_output23), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd185162983 <<< pitch_shift), 
     .b1 (18'sd309), 
     .b2 (18'sd0), 
     .b3 (-18'sd309), 
     .a2 (18'sd125698), 
     .a3 (-18'sd64917), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter23 Left 
//Filter23: frequency=2069.357596 
//Filter23: BW=0.035003 
//Filter23: shifted freq=2069.357596 
IIR2_18bit_fixed filter23_LEFT( 
     .audio_out (left_filter_output23), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd185162983 <<< pitch_shift), 
     .b1 (18'sd309), 
     .b2 (18'sd0), 
     .b3 (-18'sd309), 
     .a2 (18'sd125698), 
     .a3 (-18'sd64917), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter24 Right 
//Filter24: frequency=2200.596640 
//Filter24: BW=0.035002 
//Filter24: shifted freq=2200.596640 
IIR2_18bit_fixed filter24_RIGHT( 
     .audio_out (right_filter_output24), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd196906054 <<< pitch_shift), 
     .b1 (18'sd328), 
     .b2 (18'sd0), 
     .b3 (-18'sd328), 
     .a2 (18'sd125042), 
     .a3 (-18'sd64878), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter24 Left 
//Filter24: frequency=2200.596640 
//Filter24: BW=0.035002 
//Filter24: shifted freq=2200.596640 
IIR2_18bit_fixed filter24_LEFT( 
     .audio_out (left_filter_output24), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd196906054 <<< pitch_shift), 
     .b1 (18'sd328), 
     .b2 (18'sd0), 
     .b3 (-18'sd328), 
     .a2 (18'sd125042), 
     .a3 (-18'sd64878), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter25 Right 
//Filter25: frequency=2338.055064 
//Filter25: BW=0.035002 
//Filter25: shifted freq=2338.055064 
IIR2_18bit_fixed filter25_RIGHT( 
     .audio_out (right_filter_output25), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd209205626 <<< pitch_shift), 
     .b1 (18'sd349), 
     .b2 (18'sd0), 
     .b3 (-18'sd349), 
     .a2 (18'sd124317), 
     .a3 (-18'sd64837), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter25 Left 
//Filter25: frequency=2338.055064 
//Filter25: BW=0.035002 
//Filter25: shifted freq=2338.055064 
IIR2_18bit_fixed filter25_LEFT( 
     .audio_out (left_filter_output25), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd209205626 <<< pitch_shift), 
     .b1 (18'sd349), 
     .b2 (18'sd0), 
     .b3 (-18'sd349), 
     .a2 (18'sd124317), 
     .a3 (-18'sd64837), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter26 Right 
//Filter26: frequency=2482.027602 
//Filter26: BW=0.035002 
//Filter26: shifted freq=2482.027602 
IIR2_18bit_fixed filter26_RIGHT( 
     .audio_out (right_filter_output26), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd222088070 <<< pitch_shift), 
     .b1 (18'sd370), 
     .b2 (18'sd0), 
     .b3 (-18'sd370), 
     .a2 (18'sd123514), 
     .a3 (-18'sd64794), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter26 Left 
//Filter26: frequency=2482.027602 
//Filter26: BW=0.035002 
//Filter26: shifted freq=2482.027602 
IIR2_18bit_fixed filter26_LEFT( 
     .audio_out (left_filter_output26), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd222088070 <<< pitch_shift), 
     .b1 (18'sd370), 
     .b2 (18'sd0), 
     .b3 (-18'sd370), 
     .a2 (18'sd123514), 
     .a3 (-18'sd64794), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter27 Right 
//Filter27: frequency=2632.822957 
//Filter27: BW=0.035002 
//Filter27: shifted freq=2632.822957 
IIR2_18bit_fixed filter27_RIGHT( 
     .audio_out (right_filter_output27), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd235581010 <<< pitch_shift), 
     .b1 (18'sd392), 
     .b2 (18'sd0), 
     .b3 (-18'sd392), 
     .a2 (18'sd122627), 
     .a3 (-18'sd64750), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter27 Left 
//Filter27: frequency=2632.822957 
//Filter27: BW=0.035002 
//Filter27: shifted freq=2632.822957 
IIR2_18bit_fixed filter27_LEFT( 
     .audio_out (left_filter_output27), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd235581010 <<< pitch_shift), 
     .b1 (18'sd392), 
     .b2 (18'sd0), 
     .b3 (-18'sd392), 
     .a2 (18'sd122627), 
     .a3 (-18'sd64750), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter28 Right 
//Filter28: frequency=2790.764459 
//Filter28: BW=0.035002 
//Filter28: shifted freq=2790.764459 
IIR2_18bit_fixed filter28_RIGHT( 
     .audio_out (right_filter_output28), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd249713377 <<< pitch_shift), 
     .b1 (18'sd416), 
     .b2 (18'sd0), 
     .b3 (-18'sd416), 
     .a2 (18'sd121647), 
     .a3 (-18'sd64703), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter28 Left 
//Filter28: frequency=2790.764459 
//Filter28: BW=0.035002 
//Filter28: shifted freq=2790.764459 
IIR2_18bit_fixed filter28_LEFT( 
     .audio_out (left_filter_output28), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd249713377 <<< pitch_shift), 
     .b1 (18'sd416), 
     .b2 (18'sd0), 
     .b3 (-18'sd416), 
     .a2 (18'sd121647), 
     .a3 (-18'sd64703), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter29 Right 
//Filter29: frequency=2956.190763 
//Filter29: BW=0.035002 
//Filter29: shifted freq=2956.190763 
IIR2_18bit_fixed filter29_RIGHT( 
     .audio_out (right_filter_output29), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd264515472 <<< pitch_shift), 
     .b1 (18'sd440), 
     .b2 (18'sd0), 
     .b3 (-18'sd440), 
     .a2 (18'sd120566), 
     .a3 (-18'sd64654), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter29 Left 
//Filter29: frequency=2956.190763 
//Filter29: BW=0.035002 
//Filter29: shifted freq=2956.190763 
IIR2_18bit_fixed filter29_LEFT( 
     .audio_out (left_filter_output29), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd264515472 <<< pitch_shift), 
     .b1 (18'sd440), 
     .b2 (18'sd0), 
     .b3 (-18'sd440), 
     .a2 (18'sd120566), 
     .a3 (-18'sd64654), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter30 Right 
//Filter30: frequency=3129.456570 
//Filter30: BW=0.035002 
//Filter30: shifted freq=3129.456570 
IIR2_18bit_fixed filter30_RIGHT( 
     .audio_out (right_filter_output30), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd280019034 <<< pitch_shift), 
     .b1 (18'sd466), 
     .b2 (18'sd0), 
     .b3 (-18'sd466), 
     .a2 (18'sd119374), 
     .a3 (-18'sd64603), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter30 Left 
//Filter30: frequency=3129.456570 
//Filter30: BW=0.035002 
//Filter30: shifted freq=3129.456570 
IIR2_18bit_fixed filter30_LEFT( 
     .audio_out (left_filter_output30), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd280019034 <<< pitch_shift), 
     .b1 (18'sd466), 
     .b2 (18'sd0), 
     .b3 (-18'sd466), 
     .a2 (18'sd119374), 
     .a3 (-18'sd64603), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter31 Right 
//Filter31: frequency=3310.933394 
//Filter31: BW=0.035002 
//Filter31: shifted freq=3310.933394 
IIR2_18bit_fixed filter31_RIGHT( 
     .audio_out (right_filter_output31), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd296257305 <<< pitch_shift), 
     .b1 (18'sd493), 
     .b2 (18'sd0), 
     .b3 (-18'sd493), 
     .a2 (18'sd118061), 
     .a3 (-18'sd64549), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter31 Left 
//Filter31: frequency=3310.933394 
//Filter31: BW=0.035002 
//Filter31: shifted freq=3310.933394 
IIR2_18bit_fixed filter31_LEFT( 
     .audio_out (left_filter_output31), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd296257305 <<< pitch_shift), 
     .b1 (18'sd493), 
     .b2 (18'sd0), 
     .b3 (-18'sd493), 
     .a2 (18'sd118061), 
     .a3 (-18'sd64549), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter32 Right 
//Filter32: frequency=3501.010351 
//Filter32: BW=0.035001 
//Filter32: shifted freq=3501.010351 
IIR2_18bit_fixed filter32_RIGHT( 
     .audio_out (right_filter_output32), 
     .audio_in (right_audio_input), 
     .freq_sw (32'd313265103 <<< pitch_shift), 
     .b1 (18'sd521), 
     .b2 (18'sd0), 
     .b3 (-18'sd521), 
     .a2 (18'sd116615), 
     .a3 (-18'sd64493), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 
 
//Filter32 Left 
//Filter32: frequency=3501.010351 
//Filter32: BW=0.035001 
//Filter32: shifted freq=3501.010351 
IIR2_18bit_fixed filter32_LEFT( 
     .audio_out (left_filter_output32), 
     .audio_in (left_audio_input), 
     .freq_sw (32'd313265103 <<< pitch_shift), 
     .b1 (18'sd521), 
     .b2 (18'sd0), 
     .b3 (-18'sd521), 
     .a2 (18'sd116615), 
     .a3 (-18'sd64493), 
     .state_clk(CLOCK_50), 
     .audio_input_ready(audio_input_ready), 
     .reset(reset) 
) ; //end filter 
 

 
// ===============================================
// === Video SRAM bus master state machine ============
// ===============================================
	// writes to VGA, if data
	

reg [9:0] prev_x [99:0], prev_y[99:0];
reg [9:0] out_prev_x [99:0], out_prev_y[99:0];
reg write_prev;
reg [9:0] audio_data;
reg [7:0] count; 
reg [9:0] vga_timer;
reg [9:0] back_x, back_y;
wire [10:0] TIME_STOP = 10'd500;
reg [7:0] spect_data, color;
reg [9:0] index;

always @(posedge CLOCK_50) begin //CLOCK_50
	// reset state machine and read/write controls
	if (reset) begin
		vga_state <=4'd0;
		vga_x_cood <=10'd0;
		vga_y_cood <=vga_y_audio_center;
		vga_sram_write <=1'd0;
		vga_sram_writedata <= 8'd255;
		audio_data <= 10'd0;
		count <= 8'd0;
		write_prev <= 1'b0;
		spect_data <= (right_filter_output1<<<5);
		index <= 10'd1;
	end

	if (count >= 8'd100) begin
		write_prev <= 1'b1;
		count <= 8'd0;
	end

	//WRITE AUDIO INPUT
	if (vga_state == 4'd0) begin
		
		if ((audio_input_ready) && (timer%TIME_STOP==0))begin
			vga_sram_write <= 1'b0;
			audio_data <= bus_read_data;
			vga_state <= 4'd1;
		end
		else begin
			vga_state <=4'd0;

		end
	end
	
	//calculate y coord
	if (vga_state == 4'd1) begin
		vga_sram_write <= 1'b0;	
		vga_y_cood <= vga_y_audio_center + (audio_data>>3);
		vga_state <= 4'd2;
	end
	
	if (vga_state == 4'd2) begin
		vga_sram_write <= 1'b0;
		
		if (vga_y_cood >= 10'd180) begin
			vga_y_cood <= vga_y_cood - 10'd120;
		end
	
		vga_state <= 4'd3;
	end

	//Write to VGA
	if (vga_state == 4'd3) begin

		vga_sram_writedata <= 8'd255;
		vga_sram_write <= 1'b1;
		vga_sram_address <= vga_out_base_address + {22'b0, vga_x_cood} + ({22'b0, vga_y_cood}*640);
		prev_x[count] <= vga_x_cood;
		prev_y[count] <= vga_y_cood;
		vga_state <= 4'd4;
	end
	
	//Write to VGA
	if (vga_state == 4'd4) begin
		vga_sram_writedata <= 8'd255;
		vga_sram_write <= 1'b1;

		vga_state <= 4'd5;	
	end
	
	
	
	if (vga_state == 4'd5) begin
		if (write_prev) begin
			vga_sram_writedata <= 8'd0;
			vga_sram_write <= 1'b1;
			vga_sram_address <= vga_out_base_address + {22'b0, prev_x[(count+1)%100]} + ({22'b0, prev_y[(count+1)%100]}*640);
			vga_state <=4'd6;
		end
		else begin
			vga_state <= 4'd9;
		end
	end
	
	
	
	
	if (vga_state == 4'd6) begin
		vga_sram_write <= 1'b1;
		vga_sram_writedata <= 8'd0;
		vga_state <=4'd10;
		vga_y_cood <=vga_y_spect_top;
		spect_data <= (right_filter_output1);	
			
	end
	
	
	
	//WRITE SPECTROGRAM
	
	if (vga_state == 4'd7) begin
		
		if (vga_y_cood < vga_y_spect_top + index*10'd7) begin
			vga_sram_writedata <= color;
			vga_sram_write <= 1'b1;
			vga_sram_address <= vga_out_base_address + {22'b0, vga_x_cood} + ({22'b0, vga_y_cood}*640);
			vga_state <=4'd8;
		end
		else if (vga_y_cood == vga_y_spect_top + index*10'd7 && index <= 10'd32) begin
			index <= index + 10'd1;
			vga_state <=4'd8;
		end
		else begin
			vga_state <=4'd9;
		end
		
	end
	
	if (vga_state == 4'd8) begin
	
		if (index == 10'd1) begin
			spect_data <=  (right_filter_output1);	
		end
		else if (index == 10'd2) begin
			spect_data <=  (right_filter_output2);	
		end
		else if (index == 10'd3) begin
			spect_data <=  (right_filter_output3);	
		end
		else if (index == 10'd4) begin
			spect_data <=  (right_filter_output4);	
		end
		else if (index == 10'd5) begin
			spect_data <=  (right_filter_output5);	
		end
		else if (index == 10'd6) begin
			spect_data <=  (right_filter_output6);	
		end
		else if (index == 10'd7) begin
			spect_data <=  (right_filter_output7);	
		end
		else if (index == 10'd8) begin
			spect_data <=  (right_filter_output8);	
		end
		else if (index == 10'd9) begin
			spect_data <=  (right_filter_output9);	
		end
		else if (index == 10'd10) begin
			spect_data <=  (right_filter_output10);	
		end
		else if (index == 10'd11) begin
			spect_data <=  (right_filter_output11);	
		end
		else if (index == 10'd12) begin
			spect_data <=  (right_filter_output12);	
		end
		else if (index == 10'd13) begin
			spect_data <=  (right_filter_output13);	
		end
		else if (index == 10'd14) begin
			spect_data <=  (right_filter_output14);	
		end
		else if (index == 10'd15) begin
			spect_data <=  (right_filter_output15);	
		end
		else if (index == 10'd16) begin
			spect_data <=  (right_filter_output16);	
		end
		else if (index == 10'd17) begin
			spect_data <=  (right_filter_output17);	
		end
		else if (index == 10'd18) begin
			spect_data <=  (right_filter_output18);	
		end
		else if (index == 10'd19) begin
			spect_data <=  (right_filter_output19);	
		end
		else if (index == 10'd20) begin
			spect_data <=  (right_filter_output20);	
		end
		else if (index == 10'd21) begin
			spect_data <=  (right_filter_output21);	
		end
		else if (index == 10'd22) begin
			spect_data <=  (right_filter_output22);	
		end
		else if (index == 10'd23) begin
			spect_data <=  (right_filter_output23);	
		end
		else if (index == 10'd24) begin
			spect_data <=  (right_filter_output24);	
		end
		else if (index == 10'd25) begin
			spect_data <=  (right_filter_output25);	
		end
		else if (index == 10'd26) begin
			spect_data <=  (right_filter_output26);	
		end
		else if (index == 10'd27) begin
			spect_data <=  (right_filter_output27);	
		end
		else if (index == 10'd28) begin
			spect_data <=  (right_filter_output28);	
		end
		else if (index == 10'd29) begin
			spect_data <=  (right_filter_output29);	
		end
		else if (index == 10'd30) begin
			spect_data <=  (right_filter_output30);	
		end
		else if (index == 10'd31) begin
			spect_data <=  (right_filter_output31);	
		end
		else begin
			spect_data <=  (right_filter_output32);	
		end
	
		vga_sram_write <= 1'b0;
		vga_y_cood <= vga_y_cood + 10'd1;
		vga_state <=4'd10;
	end
	
	//Next row 
	if (vga_state == 4'd9) begin
		vga_sram_write <= 1'b0;
		count <= count + 8'd1;
		index <= 10'd1;

		if (vga_x_cood >= 10'd640) begin
			vga_x_cood <= 10'd0;
		end
		else begin
			vga_x_cood <= vga_x_cood + 10'd1;
		end
		vga_state <= 4'd0;
		
	end
	
	if (vga_state == 4'd10) begin
		vga_sram_write <= 1'b0;
		if (spect_data < 8'd25) begin
			color <= 8'd10;
		end
		else if (spect_data < 8'd50) begin
			color <= 8'd20;
		end
		else if (spect_data < 8'd75) begin
			color <= 8'd30;
		end
		else if (spect_data < 8'd100) begin
			color <= 8'd40;
		end
		else if (spect_data < 8'd125) begin
			color <= 8'd50;
		end
		else if (spect_data < 8'd150) begin
			color <= 8'd60;
		end
		
		else if (spect_data < 8'd175) begin
			color <= 8'd70;
		end
		else if (spect_data < 8'd200) begin
			color <= 8'd80;
		end
		else if (spect_data < 8'd225) begin
			color <= 8'd90;
		end
		else begin
			color <=8'd100;
		end
		vga_state <= 4'd7;
	end
	
end // always @(posedge state_clock)



// ===============================================
// === Audio bus master state machine ============
// ===============================================
	// writes, then reads the audio interface, if data
	// is ready (FIFO indicates data). Sensing the FIFO
	// effectively syncs data generation to the Audio 
	// rate.
	// 
	// The audio_input_ready signal goes high for one
	// cycle when there is new audio input data
	//
	// 32-bit audio ADC data is on:
	//   right_audio_input, left_audio_input ;
	// Every write requires 32-bit data on: 
	//   right_audio_output, left_audio_output ;
	
always @(posedge CLOCK_50) begin //CLOCK_50
	// reset state machine and read/write controls
	if (reset) begin
		state <= 0 ;
		bus_read <= 0 ; // set to one if a read opeation from bus
		bus_write <= 0 ; // set to one if a write operation to bus
		timer <= 0;
	end
	else begin
		// timer just for deubgging
		timer <= timer + 1;
	end
	
	// === writing stereo to the audio FIFO ==========
	// set up read FIFO available space
	if (state==4'd0) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		state <= 4'd1 ; // wait for read ACK
	end
	
	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (state==4'd1 && bus_ack==1) begin
		state <= 4'd2 ; //4'd2
		// FIFO write space is in high byte
		fifo_space <= (bus_read_data>>24) ;
		// end the read
		bus_read <= 1'b0 ;
	end
	
	// When there is room in the FIFO
	// -- start write to fifo for each channel
	// -- first the left channel
	if (state==4'd2 && fifo_space>8'd2) begin // 
		state <= 4'd3;	
		bus_write_data <= left_audio_output ;
		bus_addr <= audio_data_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_write <= 1'b1 ;
	end	
	// if no space, try again later
	else if (state==4'd2 && fifo_space<=8'd2) begin
		state <= 4'b0 ;
	end
	
	// detect bus-transaction-complete ACK 
	// for left channel write
	// You MUST do this check
	if (state==4'd3 && bus_ack==1) begin
		state <= 4'd4 ; // include right channel
		//state <= 4'd0 ; // left channel only!
		bus_write <= 0;
	end
	
	// -- now the right channel
	if (state==4'd4) begin // 
		state <= 4'd5;	
		// loop back audio input data
		bus_write_data <= right_audio_output ;
		bus_addr <= audio_data_right_address ;
		bus_write <= 1'b1 ;
	end	
	
	// detect bus-transaction-complete ACK
	// for right channel write
	// You MUST do this check
	if (state==4'd5 && bus_ack==1) begin
		// state <= 4'd0 ; // for write only function
		state <= 4'd6 ; // for read/write  function
		bus_write <= 0;
	end
	
	// === reading stereo from the audio FIFO ==========
	// set up read FIFO for available read values
	if (state==4'd6 ) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		state <= 4'd7 ; // wait for read ACK
	end
	
	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (state==4'd7 && bus_ack==1) begin
		state <= 4'd8 ; //4'dxx
		// FIFO read space is in low byte
		// which is zero when empty
		fifo_space <= bus_read_data & 8'hff ;
		// end the read
		bus_read <= 1'b0 ;
	end
	
	// When there is data in the read FIFO
	// -- read it from both channels
	// -- first the left channel
	if (state==4'd8 && fifo_space>8'd0) begin // 
		state <= 4'd9;	
		bus_addr <= audio_data_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_read <= 1'b1 ;
	end	
	// if no data, try again later
	else if (state==4'd8 && fifo_space<=8'd0) begin
		state <= 4'b0 ;
	end
	
	// detect bus-transaction-complete ACK 
	// for left channel read
	// You MUST do this check
	if (state==4'd9 && bus_ack==1) begin
		state <= 4'd10 ; // include right channel
		left_audio_input <= bus_read_data; //((bus_read_data < 0) && SW[2]) ? 
									//(0 - bus_read_data) : bus_read_data;
		bus_read <= 0;
	end
	
	// When there is data in the read FIFO
	// -- read it from both channels
	// -- now right channel
	if (state==4'd10) begin // 
		state <= 4'd11;	
		bus_addr <= audio_data_right_address ;
		bus_byte_enable <= 4'b1111;
		bus_read <= 1'b1 ;
	end	
	
	// detect bus-transaction-complete ACK 
	// for right channel read
	// You MUST do this check
	if (state==4'd11 && bus_ack==1) begin
		state <= 4'd12 ; // back to beginning
		right_audio_input <= bus_read_data;//((bus_read_data < 0) && SW[2]) ? 
									//(0 - bus_read_data) : bus_read_data;
		// set the data-ready strobe
		audio_input_ready <= 1'b1;
		bus_read <= 0;
	end
	
	// wait 1 cycle data-ready strobe
	if (state==4'd12) begin
		state <= 4'd13 ; // back to beginning
		//audio_input_ready <= 1'b0;
	end
	// end data-ready strobe
	if (state==4'd13) begin
		state <= 4'd0 ; // back to beginning
		audio_input_ready <= 1'b0;
	end
	
end // always @(posedge state_clock)
 
 
  










//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),
//	.sdram_clk_clk								(state_clock),

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),

	// Audio Subsystem
	.audio_pll_ref_clk_clk					(CLOCK3_50),
	.audio_pll_ref_reset_reset				(1'b0),
	.audio_clk_clk								(AUD_XCK),
	.audio_ADCDAT								(AUD_ADCDAT),
	.audio_ADCLRCK								(AUD_ADCLRCK),
	.audio_BCLK									(AUD_BCLK),
	.audio_DACDAT								(AUD_DACDAT),
	.audio_DACLRCK								(AUD_DACLRCK),

	// bus-master state machine interface
	.bus_master_audio_external_interface_address     (bus_addr),     
	.bus_master_audio_external_interface_byte_enable (bus_byte_enable), 
	.bus_master_audio_external_interface_read        (bus_read),        
	.bus_master_audio_external_interface_write       (bus_write),       
	.bus_master_audio_external_interface_write_data  (bus_write_data),  
	.bus_master_audio_external_interface_acknowledge (bus_ack),                                  
	.bus_master_audio_external_interface_read_data   (bus_read_data),   
	
	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	
	//  sram to video
	.onchip_vga_buffer_s1_address    (vga_sram_address),    
	.onchip_vga_buffer_s1_clken      (vga_sram_clken),      
	.onchip_vga_buffer_s1_chipselect (vga_sram_chipselect), 
	.onchip_vga_buffer_s1_write      (vga_sram_write),      
	.onchip_vga_buffer_s1_readdata   (),   // never read from vga here
	.onchip_vga_buffer_s1_writedata  (vga_sram_writedata),   
	
	// 50 MHz clock bridge
	.clock_bridge_0_in_clk_clk            (CLOCK_50), //(CLOCK_50), 
	
	
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
	
	//PIO pins
	
);
endmodule

///////////////////////////////////////////////////
//// signed mult of 2.16 format 2'comp ////////////
///////////////////////////////////////////////////
module signed_mult (out, a, b);

	output 		[17:0]	out;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;
	
	wire	signed	[17:0]	out;
	wire 	signed	[35:0]	mult_out;

	assign mult_out = a * b;
	//assign out = mult_out[33:17];
	assign out = {mult_out[35], mult_out[32:16]};
endmodule
//////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
/// Second order IIR filter ///////////////////////////////////////
///////////////////////////////////////////////////////////////////
module IIR2_18bit_fixed (audio_out, audio_in, 
			freq_sw,
			b1, b2, b3, 
			a2, a3, 
			state_clk, audio_input_ready, reset,
			voice_in_abs, voice_low_new, wave_ex,  modulated_voice, f1_mac_new, debug) ;
// The filter is a "Direct Form II Transposed"
// 
//    a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
//                          - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
// 
//    If a(1) is not equal to 1, FILTER normalizes the filter
//    coefficients by a(1). 
//
// one audio sample, 16 bit, 2's complement
output reg signed [15:0] audio_out ;
// one audio sample, 16 bit, 2's complement
input wire signed [15:0] audio_in ;

// sign wave to modulate by
input wire [31:0] freq_sw;

// filter coefficients
input wire signed [17:0] b1, b2, b3, a2, a3 ;
input wire state_clk, audio_input_ready, reset ;

output reg debug;
/// filter vars //////////////////////////////////////////////////
output wire signed [17:0] f1_mac_new;
wire signed [17:0] f1_coeff_x_value ;
reg signed [17:0] f1_coeff, f1_mac_old, f1_value ;

// input to filter
reg signed [17:0] x_n ;
// input history x(n-1), x(n-2)
reg signed [17:0] x_n1, x_n2 ; 

// output history: y_n is the new filter output, BUT it is
// immediately stored in f1_y_n1 for the next loop through 
// the filter state machine
reg signed [17:0] f1_y_n1, f1_y_n2 ; 

signed_mult MULTIPLIER (product,mult_in1,mult_in2);
wire signed [17:0] product;
reg  signed [17:0] mult_in1;
reg  signed [17:0] mult_in2;

assign f1_mac_new =  f1_mac_old +  product ;

// MAC operation
//signed_mult f1_c_x_v (f1_coeff_x_value, f1_coeff, f1_value);
//assign f1_mac_new =  f1_mac_old + f1_coeff_x_value ;

// modulate by sine wave
output reg signed [17:0] voice_in_abs;
reg signed [17:0] voice_low_old;
output reg signed [17:0] voice_low_new;
output wire signed [17:0] modulated_voice;
wire [15:0] wave;
reg [31:0] dds_accum ;
sync_rom sineTable(state_clk, dds_accum[31:24], wave);

output wire signed [17:0] wave_ex = {wave[15],wave[15],wave};
//signed_mult mod_mult(modulated_voice, (voice_low_new <<< 3) , wave_ex);
//assign modulated_voice = voice_in;

wire signed [4:0] log_alpha = 5'sd8;
reg signed [17:0] Ai;

// state variable 
reg [5:0] state ;
//oneshot gen to sync to audio clock
reg last_clk ; 
//////////////////////////////////////////////////////////////////

//Run the filter state machine FAST so that it completes in one 
//audio cycle

/*
always @ (posedge state_clk)
begin
		if (audio_input_ready) 
		begin
			dds_accum <= dds_accum + {freq_sw, 16'b0} ;
		end
end*/

always @ (posedge state_clk) 
begin
	if (reset)
	begin
		state <= 4'd15 ; //turn off the filter state machine	
	end
	
	else begin
		case (state)
	
			1: 
			begin
				// set up b1*x(n)
				f1_mac_old <= 18'd0 ;
				//f1_coeff <= b1 ;
				mult_in1 <= b1 ;
				//f1_value <= {audio_in, 2'b0} ;	
				// sign extend
				//f1_value <= {audio_in[15],audio_in[15], audio_in} ;	
				mult_in2 <= {audio_in[15],audio_in[15], audio_in} ;
				//register input
				//x_n <= {audio_in, 2'b0} ;	
				x_n <= {audio_in[15],audio_in[15], audio_in} ;			
				// next state
				state <= 4'd2;
			end
	
			2: 
			begin
				// set up b2*x(n-1) 
				f1_mac_old <= f1_mac_new ;
				//f1_coeff <= b2 ;
				mult_in1 <= b2 ;
				//f1_value <= x_n1 ;		
				mult_in2 <= x_n1 ;
				// next state
				state <= 6'd3;
			end
			
			3:
			begin
				// set up b3*x(n-2) 
				f1_mac_old <= f1_mac_new ;
				//f1_coeff <= b3 ;
				mult_in1 <= b3 ;
				//f1_value <= x_n2 ;
				mult_in2 <= x_n2 ;
				// next state
				state <= 6'd6;
			end
						
			6: 
			begin
				// set up -a2*y(n-1) 
				f1_mac_old <= f1_mac_new ;
				//f1_coeff <= a2 ;
				mult_in1 <= a2 ;
				//f1_value <= f1_y_n1 ; 
				mult_in2 <= f1_y_n1 ; 
				//next state 
				state <= 6'd7;
			end
			
			7: 
			begin
				// set up -a3*y(n-2) 
				f1_mac_old <= f1_mac_new ;
				//f1_coeff <= a3 ;
				mult_in1 <= a3 ;
				//f1_value <= f1_y_n2 ; 
				mult_in2 <= f1_y_n2 ;
				//next state 
				state <= 6'd10;
			end
			
			10: 
			begin
				// get the output 
				// and put it in the LAST output var
				// for the next pass thru the state machine
				//mult by scale because of coeff scaling
				// to prevent overflow
				f1_y_n1 <= f1_mac_new  ; 
				//audio_out <= f1_y_n1[17:2] ;				
				// update output history
				f1_y_n2 <= f1_y_n1 ;				
				// update input history
				x_n1 <= x_n ;
				x_n2 <= x_n1 ;
				//next state 
				state <= 5'd15;
			end	
			
			15: // take absolute value
			begin
				// wait for the audio_input_ready 
				/*if (freq_sw[3])
				begin
					if (audio_input_ready)
					begin
						state <= 5'd1 ; 
						audio_out <= f1_mac_new ;
					end
					
			   end
				
				else
				begin*/
					if (audio_input_ready)
					begin
						state <= 5'd16 ; 
						// take absolute value of audio out =======================
						if ( (f1_mac_new < 18'sd0))
						begin
							voice_in_abs <= 18'sd0 - f1_mac_new;
						end
						else
						begin
							voice_in_abs <= f1_mac_new ; //f1_y_n1[17:2] ;	
						end
					   // ========================================================
					end
					end
				//end
			//end
			
			16: // LPF-power 
			begin
				
				if (audio_input_ready) 
				begin
					state <= 5'd17 ; 
					debug <= 1;
					
					dds_accum <= dds_accum + freq_sw ;
					
					//mult_in1 <= voice_in_abs <<< 3;
					// voice_low_new <= voice_in_abs;
					mult_in2 <= wave_ex;
					mult_in1 <= ((voice_in_abs - voice_low_old) >>> log_alpha) + voice_low_old;
					
				end

			end
			
			
			
			17: // modulate with new sine wave
			begin
				
				// wait for the audio_input_ready 
				if (audio_input_ready)
				begin
					voice_low_old <= mult_in1;
					//mult_in2 <= wave_ex;
					
					debug <= 0;
					state <= 5'd1 ; 
					audio_out <= product; 
				end
			end
			
			
			default:
			begin
				// default state is end state
				//state <= 4'd15 ;
				state <= 5'd17 ;
			end
		endcase
	end
end	

endmodule
//////////////////////////////////////////////////
////////////	Sin Wave ROM Table	//////////////
//////////////////////////////////////////////////
// produces a 2's comp, 16-bit, approximation
// of a sine wave, given an input phase (address)
module sync_rom (clock, address, sine);
input clock;
input [7:0] address;
output [15:0] sine;
reg signed [15:0] sine;
always@(posedge clock)
begin
    case(address)
    		8'h00: sine = 16'h0000 ;
			8'h01: sine = 16'h0192 ;
			8'h02: sine = 16'h0323 ;
			8'h03: sine = 16'h04b5 ;
			8'h04: sine = 16'h0645 ;
			8'h05: sine = 16'h07d5 ;
			8'h06: sine = 16'h0963 ;
			8'h07: sine = 16'h0af0 ;
			8'h08: sine = 16'h0c7c ;
			8'h09: sine = 16'h0e05 ;
			8'h0a: sine = 16'h0f8c ;
			8'h0b: sine = 16'h1111 ;
			8'h0c: sine = 16'h1293 ;
			8'h0d: sine = 16'h1413 ;
			8'h0e: sine = 16'h158f ;
			8'h0f: sine = 16'h1708 ;
			8'h10: sine = 16'h187d ;
			8'h11: sine = 16'h19ef ;
			8'h12: sine = 16'h1b5c ;
			8'h13: sine = 16'h1cc5 ;
			8'h14: sine = 16'h1e2a ;
			8'h15: sine = 16'h1f8b ;
			8'h16: sine = 16'h20e6 ;
			8'h17: sine = 16'h223c ;
			8'h18: sine = 16'h238d ;
			8'h19: sine = 16'h24d9 ;
			8'h1a: sine = 16'h261f ;
			8'h1b: sine = 16'h275f ;
			8'h1c: sine = 16'h2899 ;
			8'h1d: sine = 16'h29cc ;
			8'h1e: sine = 16'h2afa ;
			8'h1f: sine = 16'h2c20 ;
			8'h20: sine = 16'h2d40 ;
			8'h21: sine = 16'h2e59 ;
			8'h22: sine = 16'h2f6b ;
			8'h23: sine = 16'h3075 ;
			8'h24: sine = 16'h3178 ;
			8'h25: sine = 16'h3273 ;
			8'h26: sine = 16'h3366 ;
			8'h27: sine = 16'h3452 ;
			8'h28: sine = 16'h3535 ;
			8'h29: sine = 16'h3611 ;
			8'h2a: sine = 16'h36e4 ;
			8'h2b: sine = 16'h37ae ;
			8'h2c: sine = 16'h3870 ;
			8'h2d: sine = 16'h3929 ;
			8'h2e: sine = 16'h39da ;
			8'h2f: sine = 16'h3a81 ;
			8'h30: sine = 16'h3b1f ;
			8'h31: sine = 16'h3bb5 ;
			8'h32: sine = 16'h3c41 ;
			8'h33: sine = 16'h3cc4 ;
			8'h34: sine = 16'h3d3d ;
			8'h35: sine = 16'h3dad ;
			8'h36: sine = 16'h3e14 ;
			8'h37: sine = 16'h3e70 ;
			8'h38: sine = 16'h3ec4 ;
			8'h39: sine = 16'h3f0d ;
			8'h3a: sine = 16'h3f4d ;
			8'h3b: sine = 16'h3f83 ;
			8'h3c: sine = 16'h3fb0 ;
			8'h3d: sine = 16'h3fd2 ;
			8'h3e: sine = 16'h3feb ;
			8'h3f: sine = 16'h3ffa ;
			8'h40: sine = 16'h3fff ;
			8'h41: sine = 16'h3ffa ;
			8'h42: sine = 16'h3feb ;
			8'h43: sine = 16'h3fd2 ;
			8'h44: sine = 16'h3fb0 ;
			8'h45: sine = 16'h3f83 ;
			8'h46: sine = 16'h3f4d ;
			8'h47: sine = 16'h3f0d ;
			8'h48: sine = 16'h3ec4 ;
			8'h49: sine = 16'h3e70 ;
			8'h4a: sine = 16'h3e14 ;
			8'h4b: sine = 16'h3dad ;
			8'h4c: sine = 16'h3d3d ;
			8'h4d: sine = 16'h3cc4 ;
			8'h4e: sine = 16'h3c41 ;
			8'h4f: sine = 16'h3bb5 ;
			8'h50: sine = 16'h3b1f ;
			8'h51: sine = 16'h3a81 ;
			8'h52: sine = 16'h39da ;
			8'h53: sine = 16'h3929 ;
			8'h54: sine = 16'h3870 ;
			8'h55: sine = 16'h37ae ;
			8'h56: sine = 16'h36e4 ;
			8'h57: sine = 16'h3611 ;
			8'h58: sine = 16'h3535 ;
			8'h59: sine = 16'h3452 ;
			8'h5a: sine = 16'h3366 ;
			8'h5b: sine = 16'h3273 ;
			8'h5c: sine = 16'h3178 ;
			8'h5d: sine = 16'h3075 ;
			8'h5e: sine = 16'h2f6b ;
			8'h5f: sine = 16'h2e59 ;
			8'h60: sine = 16'h2d40 ;
			8'h61: sine = 16'h2c20 ;
			8'h62: sine = 16'h2afa ;
			8'h63: sine = 16'h29cc ;
			8'h64: sine = 16'h2899 ;
			8'h65: sine = 16'h275f ;
			8'h66: sine = 16'h261f ;
			8'h67: sine = 16'h24d9 ;
			8'h68: sine = 16'h238d ;
			8'h69: sine = 16'h223c ;
			8'h6a: sine = 16'h20e6 ;
			8'h6b: sine = 16'h1f8b ;
			8'h6c: sine = 16'h1e2a ;
			8'h6d: sine = 16'h1cc5 ;
			8'h6e: sine = 16'h1b5c ;
			8'h6f: sine = 16'h19ef ;
			8'h70: sine = 16'h187d ;
			8'h71: sine = 16'h1708 ;
			8'h72: sine = 16'h158f ;
			8'h73: sine = 16'h1413 ;
			8'h74: sine = 16'h1293 ;
			8'h75: sine = 16'h1111 ;
			8'h76: sine = 16'h0f8c ;
			8'h77: sine = 16'h0e05 ;
			8'h78: sine = 16'h0c7c ;
			8'h79: sine = 16'h0af0 ;
			8'h7a: sine = 16'h0963 ;
			8'h7b: sine = 16'h07d5 ;
			8'h7c: sine = 16'h0645 ;
			8'h7d: sine = 16'h04b5 ;
			8'h7e: sine = 16'h0323 ;
			8'h7f: sine = 16'h0192 ;
			8'h80: sine = 16'h0000 ;
			8'h81: sine = 16'hfe6e ;
			8'h82: sine = 16'hfcdd ;
			8'h83: sine = 16'hfb4b ;
			8'h84: sine = 16'hf9bb ;
			8'h85: sine = 16'hf82b ;
			8'h86: sine = 16'hf69d ;
			8'h87: sine = 16'hf510 ;
			8'h88: sine = 16'hf384 ;
			8'h89: sine = 16'hf1fb ;
			8'h8a: sine = 16'hf074 ;
			8'h8b: sine = 16'heeef ;
			8'h8c: sine = 16'hed6d ;
			8'h8d: sine = 16'hebed ;
			8'h8e: sine = 16'hea71 ;
			8'h8f: sine = 16'he8f8 ;
			8'h90: sine = 16'he783 ;
			8'h91: sine = 16'he611 ;
			8'h92: sine = 16'he4a4 ;
			8'h93: sine = 16'he33b ;
			8'h94: sine = 16'he1d6 ;
			8'h95: sine = 16'he075 ;
			8'h96: sine = 16'hdf1a ;
			8'h97: sine = 16'hddc4 ;
			8'h98: sine = 16'hdc73 ;
			8'h99: sine = 16'hdb27 ;
			8'h9a: sine = 16'hd9e1 ;
			8'h9b: sine = 16'hd8a1 ;
			8'h9c: sine = 16'hd767 ;
			8'h9d: sine = 16'hd634 ;
			8'h9e: sine = 16'hd506 ;
			8'h9f: sine = 16'hd3e0 ;
			8'ha0: sine = 16'hd2c0 ;
			8'ha1: sine = 16'hd1a7 ;
			8'ha2: sine = 16'hd095 ;
			8'ha3: sine = 16'hcf8b ;
			8'ha4: sine = 16'hce88 ;
			8'ha5: sine = 16'hcd8d ;
			8'ha6: sine = 16'hcc9a ;
			8'ha7: sine = 16'hcbae ;
			8'ha8: sine = 16'hcacb ;
			8'ha9: sine = 16'hc9ef ;
			8'haa: sine = 16'hc91c ;
			8'hab: sine = 16'hc852 ;
			8'hac: sine = 16'hc790 ;
			8'had: sine = 16'hc6d7 ;
			8'hae: sine = 16'hc626 ;
			8'haf: sine = 16'hc57f ;
			8'hb0: sine = 16'hc4e1 ;
			8'hb1: sine = 16'hc44b ;
			8'hb2: sine = 16'hc3bf ;
			8'hb3: sine = 16'hc33c ;
			8'hb4: sine = 16'hc2c3 ;
			8'hb5: sine = 16'hc253 ;
			8'hb6: sine = 16'hc1ec ;
			8'hb7: sine = 16'hc190 ;
			8'hb8: sine = 16'hc13c ;
			8'hb9: sine = 16'hc0f3 ;
			8'hba: sine = 16'hc0b3 ;
			8'hbb: sine = 16'hc07d ;
			8'hbc: sine = 16'hc050 ;
			8'hbd: sine = 16'hc02e ;
			8'hbe: sine = 16'hc015 ;
			8'hbf: sine = 16'hc006 ;
			8'hc0: sine = 16'hc001 ;
			8'hc1: sine = 16'hc006 ;
			8'hc2: sine = 16'hc015 ;
			8'hc3: sine = 16'hc02e ;
			8'hc4: sine = 16'hc050 ;
			8'hc5: sine = 16'hc07d ;
			8'hc6: sine = 16'hc0b3 ;
			8'hc7: sine = 16'hc0f3 ;
			8'hc8: sine = 16'hc13c ;
			8'hc9: sine = 16'hc190 ;
			8'hca: sine = 16'hc1ec ;
			8'hcb: sine = 16'hc253 ;
			8'hcc: sine = 16'hc2c3 ;
			8'hcd: sine = 16'hc33c ;
			8'hce: sine = 16'hc3bf ;
			8'hcf: sine = 16'hc44b ;
			8'hd0: sine = 16'hc4e1 ;
			8'hd1: sine = 16'hc57f ;
			8'hd2: sine = 16'hc626 ;
			8'hd3: sine = 16'hc6d7 ;
			8'hd4: sine = 16'hc790 ;
			8'hd5: sine = 16'hc852 ;
			8'hd6: sine = 16'hc91c ;
			8'hd7: sine = 16'hc9ef ;
			8'hd8: sine = 16'hcacb ;
			8'hd9: sine = 16'hcbae ;
			8'hda: sine = 16'hcc9a ;
			8'hdb: sine = 16'hcd8d ;
			8'hdc: sine = 16'hce88 ;
			8'hdd: sine = 16'hcf8b ;
			8'hde: sine = 16'hd095 ;
			8'hdf: sine = 16'hd1a7 ;
			8'he0: sine = 16'hd2c0 ;
			8'he1: sine = 16'hd3e0 ;
			8'he2: sine = 16'hd506 ;
			8'he3: sine = 16'hd634 ;
			8'he4: sine = 16'hd767 ;
			8'he5: sine = 16'hd8a1 ;
			8'he6: sine = 16'hd9e1 ;
			8'he7: sine = 16'hdb27 ;
			8'he8: sine = 16'hdc73 ;
			8'he9: sine = 16'hddc4 ;
			8'hea: sine = 16'hdf1a ;
			8'heb: sine = 16'he075 ;
			8'hec: sine = 16'he1d6 ;
			8'hed: sine = 16'he33b ;
			8'hee: sine = 16'he4a4 ;
			8'hef: sine = 16'he611 ;
			8'hf0: sine = 16'he783 ;
			8'hf1: sine = 16'he8f8 ;
			8'hf2: sine = 16'hea71 ;
			8'hf3: sine = 16'hebed ;
			8'hf4: sine = 16'hed6d ;
			8'hf5: sine = 16'heeef ;
			8'hf6: sine = 16'hf074 ;
			8'hf7: sine = 16'hf1fb ;
			8'hf8: sine = 16'hf384 ;
			8'hf9: sine = 16'hf510 ;
			8'hfa: sine = 16'hf69d ;
			8'hfb: sine = 16'hf82b ;
			8'hfc: sine = 16'hf9bb ;
			8'hfd: sine = 16'hfb4b ;
			8'hfe: sine = 16'hfcdd ;
			8'hff: sine = 16'hfe6e ;
	endcase
end
endmodule

//////////////////////////////////////////////////
// === References/Links ======================
//////////////////////////////////////////////////
/*
DSP page: https://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HPS_peripherials/DSP_index.html
Mel-cepstrum wikipedia: https://en.wikipedia.org/wiki/Mel-frequency_cepstrum
Vocoder filter png: https://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/DSP/Audio/vocoder/filters.png
MC Spectrogram: https://sites.google.com/site/autosignlan/result/speech-results/mel-frequency-cepstrum-coefficients-analysis?fbclid=IwAR044DxaqdVpinyY3tHLLfbCUdtM9lr0d45nuEgMG_S5HtuR3oF_lo81SHY
How to make a spectogram: https://www.princeton.edu/~cuff/ele201/files/spectrogram.pdf  
*/

//////////////////////////////////////////////////
// === matlab filter writer ======================
//////////////////////////////////////////////////
/*
% IIR header test pgm

% at Fs=48kHz normalized F=1 at 24kHz
F = 0.2;
BW = 0.15;
[b,a] = butter(1,[F-F*(BW/2), F+F*(BW/2)] );
a = -a ; % makes it easier to use an MAC
disp(' ')
fprintf('//Filter: frequency=%f \n',F(1))
fprintf('//Filter: BW=%f \n',BW(1))
sorder = 2;
scstr = 'IIR2 filter(';

fprintf('%s \n',scstr); 
fprintf('     .audio_out (your_out), \n')
fprintf('     .audio_in (your_in), \n')
for i=1:length(b)
    if b(i)>=0
        fprintf('     .b%1d (18''sd%d), \n', i,fix(2^16*b(i)) ) ;
    else
        fprintf('     .b%1d (-18''sd%d), \n', i, fix(-2^16*b(i)) );
    end
end

for i=2:length(a)
    if a(i)>=0
        fprintf('     .a%1d (18''sd%d), \n', i, fix(2^16*a(i)) )
    else
        fprintf('     .a%1d (-18''sd%d), \n', i, fix(-2^16*a(i)) )
    end
end
fprintf('     .state_clk(CLOCK_50), \n');
fprintf('     .audio_input_ready(audio_input_ready), \n');
fprintf('     .reset(reset) \n');
fprintf(') ; //end filter \n');

disp(' ')
*/
// === end ====================================
