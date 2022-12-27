`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   03:21:24 03/22/2021
// Design Name:   SysRstController
// Module Name:   E:/Code_Project/SpeedTracker_Display/SysRstControllerTB.v
// Project Name:  SpeedTracker_Display
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: SysRstController
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`define CLK8X_DELAY 16
`define FRAME 11

module SysRstControllerTB;

	// Inputs
	reg clk_1M8;
	reg rst_n;

	// Outputs
	wire sys_rst_n;

	// Instantiate the Unit Under Test (UUT)
	SysRstController uut (
		.clk_1M8(clk_1M8), 
		.rst_n(rst_n), 
		.sys_rst_n(sys_rst_n)
	);

	//1.8432MHz clock generation
	initial begin                                                                                        
		clk_1M8 = 0;
    forever   
		#(`CLK8X_DELAY/8) clk_1M8 = !clk_1M8;                                                          
	end 

	initial begin
		// Initialize Inputs
		clk_1M8 = 0;
		rst_n = 1;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
      
endmodule

