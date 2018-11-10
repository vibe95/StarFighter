module projectv1(
//	Clock Input
  input CLOCK_50,	//	50 MHz
  input CLOCK_27,     //      27 MHz
//	Push Button
  input [3:0] KEY,      //	Pushbutton[3:0]
//	DPDT Switch
  input [17:0] SW,		//	Toggle Switch[17:0]
//	7-SEG Display
  output [6:0]	HEX0, HEX1, HEship_x,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,  // Seven Segment Digits
//	LED
  output [8:0]	LEDG,  //	LED Green[8:0]
  output reg [17:0] LEDR,  //	LED Red[17:0]
//	GPIO
 inout [35:0] GPIO_0,GPIO_1,	//	GPIO Connections
//	TV Decoder
//TD_DATA,    	//	TV Decoder Data bus 8 bits
//TD_HS,		//	TV Decoder H_SYNC
//TD_VS,		//	TV Decoder V_SYNC
  output TD_RESET,	//	TV Decoder Reset
// VGA
  output VGA_CLK,   						//	VGA Clock
  output VGA_HS,							//	VGA H_SYNC
  output VGA_VS,							//	VGA V_SYNC
  output VGA_BLANK,						//	VGA BLANK
  output VGA_SYNC,						//	VGA SYNC
  output [9:0] VGA_R,   						//	VGA Red[9:0]
  output [9:0] VGA_G,	 						//	VGA Green[9:0]
  output [9:0] VGA_B, 						//	VGA Blue[9:0]
  input	PS2_DAT,
  input	PS2_CLK
);

//	All inout port turn to tri-state 	
assign	GPIO_0		=	36'hzzzzzzzzz;
assign	GPIO_1		=	36'hzzzzzzzzz;

wire RST;
assign RST = ~SW[1];

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(	.iCLK(CLOCK_50),.oRESET(DLY_RST) );
wire [6:0] blank = 7'b111_1111;
wire		VGA_CTRL_CLK;
wire		AUD_CTRL_CLK;
wire [59:0]	mVGA_R;
wire [59:0]	mVGA_G;
wire [59:0]	mVGA_B;
wire [9:0]	mCoord_X;
wire [9:0]	mCoord_Y;

assign	TD_RESET = 1'b1; // Enable 27 MHz

VGA_Audio_PLL 	p1 (	
	.areset(~DLY_RST),
	.inclk0(CLOCK_27),
	.c0(VGA_CTRL_CLK),
	.c1(AUD_CTRL_CLK),
	.c2(VGA_CLK)
);

wire [9:0] r, g, b; //Colours on display 

//clock for asteroids
reg [32:0]Q  = 0;
reg change = 0;
always@(posedge CLOCK_27)
	begin
		if (Q >= 250000)
		begin
		Q = 0;
		change = 1;
		end
		else
		begin
		Q = Q + 1;
		change = 0;
		end
	end

//clock for ship	
reg [32:0]Q2  = 0;
reg change2 = 0;
always@(posedge CLOCK_50)
	begin
		if (Q2 >= 150000)
		begin
		Q2 = 0;
		change2 = 1;
		end
		else
		begin
		Q2 = Q2 + 1;
		change2 = 0;
		end
	end

	
ps2In(CLOCK_50, SW[17:0],PS2_DAT,PS2_CLK, GPIO_0[35:0], GPIO_1[35:0],left,right,fire); // keyboard input
bars a1 (mCoord_X, mCoord_Y, r, g, b, ~KEY[1], ~KEY[3], ~KEY[2], change,LEDG[7:0],over,collided, HEX0, HEX1, change2, KEY[0]); // COMBING WITH RANDOM GENRATOR 
music m1 (CLOCK_27,AUD_ADCDAT,~KEY[2],LEDG[8],collided); // Audio output (not working)

//lights for game over 
always @(over)
	LEDR[0] = over;
	LEDR[1] = over;
	LEDR[2] = over;
	LEDR[3] = over;
	LEDR[4] = over;
	LEDR[5] = over;
	LEDR[6] = over;
	LEDR[7] = over;
	LEDR[8] = over;
	LEDR[9] = over;
	LEDR[10] = over;
	LEDR[11] = over;
	LEDR[12] = over;
	LEDR[13] = over;
	LEDR[14] = over;
	LEDR[15] = over;
	LEDR[16] = over;
	LEDR[17] = over;

//More display code
wire [9:0] gray = (mCoord_X<80 || mCoord_X>560? 10'h000:
	(mCoord_Y/15)<<5 | (mCoord_X-80)/15);
wire s = SW[0];
assign mVGA_R = (s? gray: r);
assign mVGA_G = (s? gray: g);
assign mVGA_B = (s? gray: b);

vga_sync u1(
   .iCLK(VGA_CTRL_CLK),
   .iRST_N(DLY_RST&~SW[1]),	
   .iRed(mVGA_R),
   .iGreen(mVGA_G),
   .iBlue(mVGA_B),
   // pixel coordinates
   .px(mCoord_X),
   .py(mCoord_Y),
   // VGA Side
   .VGA_R(VGA_R),
   .VGA_G(VGA_G),
   .VGA_B(VGA_B),
   .VGA_H_SYNC(VGA_HS),
   .VGA_V_SYNC(VGA_VS),
   .VGA_SYNC(VGA_SYNC),
   .VGA_BLANK(VGA_BLANK)
);
endmodule


//Code for regulate audio 
module music(clk, speaker,fire,light,collided);
input clk,fire,collided;
output reg speaker;
output reg light;
reg [16:0]note;
reg sound;
always@(posedge clk)
	begin
	if (fire)
		begin
			light = 1;
			sound = 1;
			note = 38408;
		end
	else
		begin 
			light=0;
			sound = 0;
		end 
	if (collided)
		note = 28408;
	end 

// Binary counter, 16-bits wide
reg [15:0] counter;
always @(posedge clk) 
	begin
	if (sound==1)
		begin
		if(counter==note) 
			counter<=0;
		else 
			counter <= counter+1;		
		end 
	end 
always @(posedge clk) 
	if(counter==note) 
		speaker <= ~speaker;

endmodule


//////////// new CODE FROM BARs
module bars(input [9:0] x, input [9:0] y, output [9:0] red, output [9:0] green, output [9:0] blue, input right, input left, input fire, input clk, output reg [7:0]light, output over, output reg collided, output[6:0] display, output[6:0] display2, input clk2, input reset1);
	
	//creating parameters and initializing variables 
	parameter screen_height = 450;
	parameter screen_width = 640;
	parameter ship_radius = 8;
	reg [2:0] idx; //for colour on display 
	
	//intial points for asteriod 1 
	reg [11:0] asteroid1_x1 = 100;  
	reg [11:0] asteroid1_x2 = 125;
	reg signed [9:0] asteroid1_y1 = -125;
	reg signed [9:0] asteroid1_y2 = - 100;
	
	//intial points for asteriod 2 
	reg [11:0] asteroid2_x1 = 130;
	reg [11:0] asteroid2_x2 = 155;
	reg signed [9:0] asteroid2_y1 = -200;
	reg signed [9:0] asteroid2_y2 = -175;

	//intial points for asteriod 3
	reg [11:0] asteroid3_x1 = 380;
	reg [11:0] asteroid3_x2 = 405;
	reg signed [9:0] asteroid3_y1 = -150 - 100; 
	reg signed [9:0] asteroid3_y2 = -125 - 100;
	
	//intial points for asteriod 4 
	reg [11:0] asteroid4_x1 = 230;
	reg [11:0] asteroid4_x2 = 255;
	reg signed [9:0] asteroid4_y1 = -200 - 100;
	reg signed [9:0] astetroid4_y2 = -175 - 100;
	
	//intial points for bull 
	reg [11:0] half_bullet_dimension = 3;
	reg [11:0] bullet_x1; 
	reg signed [11:0] bullet_y1;
	reg [11:0] bullet_x2;
	reg signed [11:0] bullet_y2;
	
	//ship location and setting fire/collide
	reg is_firing = 0;
	reg has_collided = 0;
	reg [11:0] ship_x = 320;
	reg [11:0] ship_y = screen_height;
	
	//create regs for random 
	reg [8:0] rand1;
	reg [8:0] rand2;
   reg [8:0] rand3;
	reg [8:0] rand4;

	//other 
	reg [6:0] score = 0;
	reg [6:0] tens = 0;
	reg [6:0] ones = 0;
	reg lose = 0;
	
	//checking game over
	always@(posedge clk2)
	begin 
		if(asteroid1_y1 >= 700 | asteroid2_y1 >= 700 | asteroid3_y1 >= 700 | asteroid4_y1 >= 700)
			begin
				lose = 1;
			end
	end
	
	//asteroid collision detection 
	always@(posedge clk)
	begin	
	ones = score % 10;
	tens = (score - ones)/10;
	asteroid1_y1 = asteroid1_y1 + 1;
	asteroid1_y2 = asteroid1_y2 + 1;
	asteroid2_y1 = asteroid2_y1 + 1;
	asteroid2_y2 = asteroid2_y2 + 1;
	asteroid3_y1 = asteroid3_y1 + 1;
	asteroid3_y2 = asteroid3_y2 + 1;
	asteroid4_y1 = asteroid4_y1 + 1;
	asteroid4_y2 = asteroid4_y2 + 1;
	if(bullet_x2 > asteroid1_x1 & bullet_x1 < asteroid1_x2 & bullet_y1 < asteroid1_y2)
		begin
		has_collided = 1;
		asteroid1_y1 = -25;
		asteroid1_y2 = 0;
		rand1 <= {rand1[7:0], rand1[8] ^ rand1[6] ^ rand1[5] ^ rand1[4]}; //random x value after asteroid is destroyed 
		asteroid1_x1 = rand1;
		asteroid1_x2 = rand1 + 25;
		score = score + 1; // get a point for destroying asteroid
		ones = score % 10;    // inorder to display score on hex
		tens = (score - ones)/10; // inorder to display score on hex
		end
	else if(bullet_x2 > asteroid2_x1 & bullet_x1 < asteroid2_x2 & bullet_y1 < asteroid2_y2)
		begin
		has_collided = 1;
		asteroid2_y1 = -25;
		asteroid2_y2 = 0;
		rand2 <= {rand2[7:0], rand2[8] ^ rand2[6] ^ rand2[5] ^ rand2[4]};
		asteroid2_x1 = rand2;
		asteroid2_x2 = rand2 + 25;
		score = score + 1;
		ones = score % 10;
		tens = (score - ones)/10;
		end
	else if(bullet_x2 > asteroid3_x1 & bullet_x1 < asteroid3_x2 & bullet_y1 < asteroid3_y2)
		begin
		has_collided = 1;
		asteroid3_y1 = -25;
		asteroid3_y2 = 0;
		rand3 <= {rand3[7:0], rand3[8] ^ rand3[6] ^ rand3[5] ^ rand3[4]};
		asteroid3_x1 = rand3;
		asteroid3_x2 = rand3 + 25;
		score = score + 1;
		ones = score % 10;
		tens = (score - ones)/10;
		end
	else if(bullet_x2 > asteroid4_x1 & bullet_x1 < asteroid4_x2 & bullet_y1 < asteroid4_y2)
		begin
		has_collided = 1;
		asteroid4_y1 = -25;
		asteroid4_y2 = 0;
		rand4 <= {rand4[7:0], rand4[8] ^ rand4[6] ^ rand4[5] ^ rand4[4]};
		asteroid4_x1 = rand4;
		asteroid4_x2 = rand4 + 25;
		score = score + 1;
		ones = score % 10;
		tens = (score - ones)/10;
		end
	else
		begin
		has_collided = 0;
		end 
	end
	
//
always @(clk)
	begin 
	if (has_collided)
	begin
	 light[0] = 1;
	 light[1] = 1;
	 light[2] = 1;
	 light[3] = 1;
	 light[4] = 1;
	 light[5] = 1;
	 light[6] = 1;
	 light[7] = 1;
	 end
	 else
	 light[0] = 0;
	 light[1] = 0;
	 light[2] = 0;
	 light[3] = 0;
	 light[4] = 0;
	 light[5] = 0;
	 light[6] = 0;
	 light[7] = 0;
	end 
	
	
	always@(posedge clk2)
	begin
	if(right)
		begin
			ship_x = ship_x + 1;
		end	
	else if (left)
			ship_x = ship_x - 1;

	end

	
	always @(posedge clk2)
	begin
	if (has_collided)
	begin
	collided = 1; 
	is_firing = 0;
	bullet_x1 = 800;
	bullet_y1 = 800;
	bullet_x2 = 800;
	bullet_y2 = 800;
	end
	else if(fire & ~is_firing)
	begin
	bullet_x1 = ship_x - half_bullet_dimension;
	bullet_y1 = ship_y - ship_radius - half_bullet_dimension;
	bullet_x2 = ship_x + half_bullet_dimension;
	bullet_y2 = ship_y - ship_radius + half_bullet_dimension;
	is_firing = 1;
	end
	else if(is_firing & ~has_collided)
	begin
	bullet_y1 = bullet_y1 - 1; 
	bullet_y2 = bullet_y2 - 1;
	end
	else 
		collided = 0; 
	if(bullet_y2 <= 0)
		begin
		is_firing = 0;
		bullet_x1 = 800;
		bullet_y1 = 800;
		bullet_x2 = 800;
		bullet_y2 = 800;
		end
	end
	//score = score + 10;
	assign display = (ones == 0)? 7'b100_0000:
			(ones == 1)? 7'b111_1001:
			(ones == 2)? 7'b010_0100:
			(ones == 3)? 7'b011_0000:
			(ones == 4)? 7'b001_1001:
			(ones == 5)? 7'b001_0010:
			(ones == 6)? 7'b000_0010:
			(ones == 7)? 7'b101_1000:
			(ones == 8)? 7'b000_0000: 7'b001_1000;
	assign display2 = (tens == 0)? 7'b100_0000:
			(tens == 1)? 7'b111_1001:
			(tens == 2)? 7'b010_0100:
			(tens == 3)? 7'b011_0000:
			(tens == 4)? 7'b001_1001:
			(tens == 5)? 7'b001_0010:
			(tens == 6)? 7'b000_0010:
			(tens == 7)? 7'b101_1000:
			(tens == 8)? 7'b000_0000: 7'b001_1000;
	
//reg start=0; 
//always @(posedge clk)
//	
//begin
//if(~fire)
//bullet_y1 = bullet_y1 -1;
//bullet_y2 = bullet_y2 - 1;
//end

reg [9:0] b_x1a = 0;
reg [9:0] b_x2a = 50;
reg [9:0] b_y1a = 0;
reg [9:0] b_y2a = 15;
reg [9:0] b_x1b = 0;
reg [9:0] b_x2b = 15;
reg [9:0] b_y1b = 15;
reg [9:0] b_y2b = 85;
reg [9:0] b_x1c = 35;
reg [9:0] b_x2c = 50;
reg [9:0] b_y1c = 15;
reg [9:0] b_y2c = 85;
reg [9:0] b_x1d = 0;
reg [9:0] b_x2d = 50;
reg [9:0] b_y1d = 85;
reg [9:0] b_y2d = 100;
reg [9:0] b_x1e = 15;
reg [9:0] b_x2e = 35;
reg [9:0] b_y1e = 28;
reg [9:0] b_y2e = 42;

reg [9:0] o_x1a = 65;
reg [9:0] o_x2a = 115;
reg [9:0] o_y1a = 0;
reg [9:0] o_y2a = 15;
reg [9:0] o_x1b = 65;
reg [9:0] o_x2b = 80;
reg [9:0] o_y1b = 15;
reg [9:0] o_y2b = 85;
reg [9:0] o_x1c = 100;
reg [9:0] o_x2c = 115;
reg [9:0] o_y1c = 15;
reg [9:0] o_y2c = 85;
reg [9:0] o_x1d = 65;
reg [9:0] o_x2d = 115;
reg [9:0] o_y1d = 85;
reg [9:0] o_y2d = 100;

reg [9:0] o2_x1a = 130;
reg [9:0] o2_x2a = 180;
reg [9:0] o2_y1a = 0;
reg [9:0] o2_y2a = 15;
reg [9:0] o2_x1b = 130;
reg [9:0] o2_x2b = 145;
reg [9:0] o2_y1b = 15;
reg [9:0] o2_y2b = 85;
reg [9:0] o2_x1c = 165;
reg [9:0] o2_x2c = 180;
reg [9:0] o2_y1c = 15;
reg [9:0] o2_y2c = 85;
reg [9:0] o2_x1d = 130;
reg [9:0] o2_x2d = 180;
reg [9:0] o2_y1d = 85;
reg [9:0] o2_y2d = 100;

reg [9:0] m_x1a = 195;
reg [9:0] m_x2a = 265;
reg [9:0] m_y1a = 0;
reg [9:0] m_y2a = 15;
reg [9:0] m_x1b = 195;
reg [9:0] m_x2b = 210 ;
reg [9:0] m_y1b = 15;
reg [9:0] m_y2b = 85;
reg [9:0] m_x1c = 250;
reg [9:0] m_x2c = 265;
reg [9:0] m_y1c = 15;
reg [9:0] m_y2c = 85;
reg [9:0] m_x1d = 222;
reg [9:0] m_x2d = 237;
reg [9:0] m_y1d = 15;
reg [9:0] m_y2d = 85;

always @(x)
	begin
		if (~lose)
		begin
			if((x - ship_x)*(x-ship_x) + (y -ship_y)*(y-ship_y) <= ship_radius*ship_radius)idx <= 3'd2;
			else if (x < bullet_x2 & x >bullet_x1 &y < bullet_y2 & y > bullet_y1) idx <= 3'd1; //bullet
			else if (x < asteroid1_x2 & x > asteroid1_x1 & y < asteroid1_y2 & y > asteroid1_y1) idx <= 3'd3;
		    else if (x < asteroid2_x2 & x > asteroid2_x1 & y < asteroid2_y2 & y > asteroid2_y1) idx <= 3'd4;
			else if (x < asteroid3_x2 & x > asteroid3_x1 & y < asteroid3_y2 & y > asteroid3_y1) idx <= 3'd5;
			else if (x < asteroid4_x2 & x > asteroid4_x1 & y < asteroid4_y2 & y > asteroid4_y1) idx <= 3'd6;
			else 
				idx <= 3'd0; // else its black screeen
		end
		else if(lose&reset1)
		begin
			if(x < b_x2a & x > b_x1a & y < b_y2a & y > b_y1a) idx <= 3'd2;
			 else if (x < b_x2b & x > b_x1b & y < b_y2b & y > b_y1b) idx <= 3'd2;
			 else if (x < b_x2c & x > b_x1c & y < b_y2c & y > b_y1c) idx <= 3'd2;
			 else if (x < b_x2d & x > b_x1d & y < b_y2d & y > b_y1d) idx <= 3'd2;
			 else if (x < b_x2e & x > b_x1e & y < b_y2e & y > b_y1e) idx <= 3'd2;
			 
			 else if (x < o_x2a & x > o_x1a & y < o_y2a & y > o_y1a) idx <= 3'd2;
			 else if (x < o_x2b & x > o_x1b & y < o_y2b & y > o_y1b) idx <= 3'd2;
			 else if (x < o_x2c & x > o_x1c & y < o_y2c & y > o_y1c) idx <= 3'd2;
			 else if (x < o_x2d & x > o_x1d & y < o_y2d & y > o_y1d) idx <= 3'd2;

			 else if (x < o2_x2a & x > o2_x1a & y < o2_y2a & y > o2_y1a) idx <= 3'd2;
			 else if (x < o2_x2b & x > o2_x1b & y < o2_y2b & y > o2_y1b) idx <= 3'd2;
			 else if (x < o2_x2c & x > o2_x1c & y < o2_y2c & y > o2_y1c) idx <= 3'd2;
			 else if (x < o2_x2d & x > o2_x1d & y < o2_y2d & y > o2_y1d) idx <= 3'd2;
			 
			 else if (x < m_x2a & x > m_x1a & y < m_y2a & y > m_y1a) idx <= 3'd2;
			 else if (x < m_x2b & x > m_x1b & y < m_y2b & y > m_y1b) idx <= 3'd2;
			 else if (x < m_x2c & x > m_x1c & y < m_y2c & y > m_y1c) idx <= 3'd2;
			 else if (x < m_x2d & x > m_x1d & y < m_y2d & y > m_y1d) idx <= 3'd2;
			 else
			idx <= 3'd1;
		end
	end
	assign red = (idx[0]? 10'h3ff: 10'h000);
	assign green = (idx[1]? 10'h3ff: 10'h000);
	assign blue = (idx[2]? 10'h3ff: 10'h000);
endmodule

//have to randomly generate ypos of asteroids;
//module lfsr_8_bit( out,  clk)
//	input clk;
//	output out;
//	
//	reg [8:0] d;
//	
//	always @(posedge clk) begin
//	d <= {d[7:0], d[8] ^ d[6] ^ d[5] ^ d[4]};
//	end
//	
//	out = d + 50;
//endmodule



module ps2In(
  // Clock Input (50 MHz)
  input  CLOCK_50,
  //  Push Buttons
  //input   KEY,
  //  DPDT Switches 
  input SW,
  //  LEDs

  //  PS2 data and clock lines		
  input	PS2_DAT,
  input	PS2_CLK,
  //  GPIO Connections
  inout  GPIO_0, GPIO_1,
  output reg left,right,fire
);



wire RST;
//assign RST = KEY[0];



wire reset = 1'b0;

reg [7:0] history[1:4];
wire read, scan_ready;

oneshot pulser(
   .pulse_out(read),
   .trigger_in(scan_ready),
   .clk(CLOCK_50)
);

keyboard kbd(
  .keyboard_clk(PS2_CLK),
  .keyboard_data(PS2_DAT),
  .clock50(CLOCK_50),
  .reset(reset),
  .read(read),
  .scan_ready(scan_ready),
  .scan_code(scan_code)
);

always@(posedge CLOCK_50)
	begin
		if (scan_code == 2929)
			
			fire = 1;
		else if (scan_code == 57446)
			left = 1;
		else if (scan_code == 57460)
			right = 1;
		else
			begin
				fire = 0;
				left = 0;
				right = 0;
			end
	end 
				
			



always @(posedge scan_ready)
begin
	history[4] <= history[3];
	history[3] <= history[2];
	history[2] <= history[1];
	history[1] <= scan_code;
end
	

// blank remaining digits
/*
wire [6:0] blank = 7'h7f;
assign HEX2 = blank;
assign HEX3 = blank;
assign HEX4 = blank;
assign HEX5 = blank;
assign HEX6 = blank;
assign HEX7 = blank;
*/

endmodule