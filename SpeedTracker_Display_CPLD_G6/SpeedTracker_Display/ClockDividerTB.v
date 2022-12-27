`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   03:46:21 03/21/2021
// Design Name:   ClockDivider
// Module Name:   E:/Code_Project/SpeedTracker_Display/ClockDividerTB.v
// Project Name:  SpeedTracker_Display
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: ClockDivider
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module ClockDividerTB;

	// Inputs
	reg clk;
	reg rst_n;

	// Outputs
	wire clk_div4;

	// Instantiate the Unit Under Test (UUT)
	ClockDivider uut (
		.clk(clk), 
		.rst_n(rst_n), 
		.clk_div4(clk_div4)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		rst_n = 0;

		// Wait 100 ns for global reset to finish
		//#100;
        
		// Add stimulus here
		#20 rst_n = 1;
		forever
			#1 clk = ~clk;
	end
      
endmodule

