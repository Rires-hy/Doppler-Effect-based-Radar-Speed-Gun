`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    04:05:24 03/21/2021 
// Design Name: 
// Module Name:    SegDecoder 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module SegDecoder(
	input wire [7:0] msg,
	input wire noti,
	input wire clk,
	input wire rst_n,
	output reg [7:0] segL,
	output reg [7:0] segR
    );

	reg [3:0] msgL;
	reg [3:0] msgR;
	
	parameter MSG_0 = 4'b0000;
	parameter MSG_1 = 4'b0001;
	parameter MSG_2 = 4'b0010;
	parameter MSG_3 = 4'b0011;
	parameter MSG_4 = 4'b0100;
	parameter MSG_5 = 4'b0101;
	parameter MSG_6 = 4'b0110;
	parameter MSG_7 = 4'b0111;
	parameter MSG_8 = 4'b1000;
	parameter MSG_9 = 4'b1001;
	parameter MSG_EMPTY = 4'b1010;
	parameter MSG_HIGH = 4'b1011;
	parameter MSG_LOW = 4'b1100;
	parameter MSG_FREQ = 4'b1101;
	parameter MSG_ERROR = 4'b1110;

	// DP G F E D C B A
	//MSB             LSB
	parameter SEG_A = 8'b11111110;	
	parameter SEG_B = 8'b11111101;
	parameter SEG_C = 8'b11111011;
	parameter SEG_D = 8'b11110111;
	parameter SEG_E = 8'b11101111;
	parameter SEG_F = 8'b11011111;
	parameter SEG_G = 8'b10111111;
	parameter SEG_DP = 8'b01111111;

	parameter LED_SYM_0_LR = SEG_A & SEG_B & SEG_C & SEG_D & SEG_E & SEG_F;
	parameter LED_SYM_1_LR = SEG_B & SEG_C;
	parameter LED_SYM_2_LR = SEG_A & SEG_B &  SEG_D & SEG_E & SEG_G;
	parameter LED_SYM_3_LR = SEG_A & SEG_B & SEG_C & SEG_D & SEG_G;
	parameter LED_SYM_4_LR = SEG_B & SEG_C & SEG_F & SEG_G;
	parameter LED_SYM_5_LR = SEG_A & SEG_C & SEG_D & SEG_F & SEG_G;
	parameter LED_SYM_6_LR = SEG_A &  SEG_C & SEG_D & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_7_LR = SEG_A & SEG_B & SEG_C;
	parameter LED_SYM_8_LR = SEG_A & SEG_B & SEG_C & SEG_D & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_9_LR = SEG_A & SEG_B & SEG_C & SEG_D & SEG_F & SEG_G;
	parameter LED_SYM_EMPTY_L = SEG_A & SEG_D & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_EMPTY_R = SEG_A & SEG_B & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_HIGH_L = SEG_B & SEG_C & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_HIGH_R = SEG_B & SEG_C;
	parameter LED_SYM_LOW_L = SEG_D & SEG_E & SEG_F;
	parameter LED_SYM_LOW_R = SEG_A & SEG_B & SEG_C & SEG_D & SEG_E & SEG_F;
	parameter LED_SYM_FREQ_L = SEG_A & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_FREQ_R = SEG_A & SEG_B & SEG_C & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_ERROR_L = SEG_A & SEG_D & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_ERROR_R = SEG_A & SEG_B & SEG_C & SEG_E & SEG_F & SEG_G;
	parameter LED_SYM_CLR = 8'b11111111;
	
	always @(posedge noti or negedge rst_n)
	begin
		if (!rst_n)
		begin
			msgL <= 4'b0000;
			msgR <= 4'b0000;
		end
		else
		begin
			msgL <= msg[7:4];
			msgR <= msg[3:0];
		end
	end

	always @(posedge clk or negedge rst_n)
	begin
		if (!rst_n)
		begin
			segL <= 8'b00000000;
			segR <= 8'b00000000;	
		end
		else
		begin
			case (msgL)
				MSG_0 : segL <= LED_SYM_CLR;
				MSG_1 : segL <= LED_SYM_1_LR;
				MSG_2 : segL <= LED_SYM_2_LR;
				MSG_3 : segL <= LED_SYM_3_LR;
				MSG_4 : segL <= LED_SYM_4_LR;
				MSG_5 : segL <= LED_SYM_5_LR;
				MSG_6 : segL <= LED_SYM_6_LR;
				MSG_7 : segL <= LED_SYM_7_LR;
				MSG_8 : segL <= LED_SYM_8_LR;
				MSG_9 : segL <= LED_SYM_9_LR;
				MSG_EMPTY : segL <= LED_SYM_EMPTY_L;
				MSG_HIGH : segL <= LED_SYM_HIGH_L;
				MSG_LOW : segL <= LED_SYM_LOW_L;
				MSG_FREQ : segL <= LED_SYM_FREQ_L;
				MSG_ERROR : segL <= LED_SYM_ERROR_L;
				default : segL <= 8'b00000000;
			endcase

			if (msgL == MSG_0 & msgR == MSG_0)
			begin
				segR <= LED_SYM_CLR;
			end
			else
			case (msgR)
				MSG_0 : segR <= LED_SYM_0_LR;
				MSG_1 : segR <= LED_SYM_1_LR;
				MSG_2 : segR <= LED_SYM_2_LR;
				MSG_3 : segR <= LED_SYM_3_LR;
				MSG_4 : segR <= LED_SYM_4_LR;
				MSG_5 : segR <= LED_SYM_5_LR;
				MSG_6 : segR <= LED_SYM_6_LR;
				MSG_7 : segR <= LED_SYM_7_LR;
				MSG_8 : segR <= LED_SYM_8_LR;
				MSG_9 : segR <= LED_SYM_9_LR;
				MSG_EMPTY : segR <= LED_SYM_EMPTY_R;
				MSG_HIGH : segR <= LED_SYM_HIGH_R;
				MSG_LOW : segR <= LED_SYM_LOW_R;
				MSG_FREQ : segR <= LED_SYM_FREQ_R;
				MSG_ERROR : segR <= LED_SYM_ERROR_R;
				default : segR <= 8'b00000000;
			endcase
		end
	end


endmodule
