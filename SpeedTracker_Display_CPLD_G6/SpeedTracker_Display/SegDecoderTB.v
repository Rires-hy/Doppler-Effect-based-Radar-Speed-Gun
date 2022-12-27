`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   16:16:40 03/21/2021
// Design Name:   SegDecoder
// Module Name:   E:/Code_Project/SpeedTracker_Display/SegDecoderTB.v
// Project Name:  SpeedTracker_Display
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: SegDecoder
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`define CLK8X_DELAY 16

module SegDecoderTB;

	// Inputs
	reg [7:0] msg;
	reg noti;
	reg rst_n;

	// Outputs
	wire [7:0] segL;
	wire [7:0] segR;

	reg [3:0] msgL;
	reg [3:0] msgR;

	initial begin                                                                                        
		clk = 0;
    forever   
		#(`CLK8X_DELAY/8) clk = !clk;
	end

	// Instantiate the Unit Under Test (UUT)
	SegDecoder uut (
		.msg(msg), 
		.noti(noti),
		.rst_n(rst_n),
		.segL(segL), 
		.segR(segR)
	);

	initial begin
		// Initialize Inputs
		msg = 0;
		noti = 0;
		rst_n = 0;
		msgL = 0;
		msgR = 0;


		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		rst_n = 1;
		forever
		begin
			#1 noti = 1;
			#100
			noti = 0;
			#1
			msgL = msgL + 1;
			msgR = msgR + 1;
			msg = { msgL, msgR };
		end
	end

	initial begin
		#350 rst_n = 0;
		#450 rst_n = 1;
	end
      
endmodule

