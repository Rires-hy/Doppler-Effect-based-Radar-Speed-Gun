`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   00:51:39 03/22/2021
// Design Name:   SerialController
// Module Name:   E:/Code_Project/SpeedTracker_Display/SerialControllerTB.v
// Project Name:  SpeedTracker_Display
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: SerialController
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

module SerialControllerTB;

	// Inputs
	reg serial;
	wire clk;
	reg rst_n;

	// Outputs
	wire [7:0] msg;
	wire noti;

	reg clk1M8;
	reg send_start;
	integer i, j, byte_num;
	reg [7:0] data, data_0;
	reg[10:0] data_frame;

	// Instantiate the Unit Under Test (UUT)
	SerialController uut (
		.serial(serial), 
		.clk(clk), 
		.rst_n(rst_n), 
		.msg(msg), 
		.noti(noti)
	);

	ClockDivider uut2 (
		.clk(clk1M8),
		.rst_n(rst_n),
		.clk_div4(clk)
	);

	//1.8432MHz clock generation
	initial begin                                                                                        
		clk1M8 = 0;
      forever   
			#(`CLK8X_DELAY/8) clk1M8 = !clk1M8;                                                          
		end 
	
	initial begin
		rst_n = 1;
		send_start = 0;
		serial = 1;
		byte_num = 3;
		data = 48;
		data_0 = 0;
		#30
		rst_n = 0;
		#30
		rst_n = 1;
		#20
		i= 0;
		#1
		repeat (10) begin
			#10000
			send_start = 1;
			for (i=0; i< byte_num; i=i+1) begin
				if (data < 57)
						data = data + 1'b1;
				else
						data = 48;
				#(`CLK8X_DELAY * 4)
				if (i == 2)
					data_frame = {1'b1, ^data_0, data_0, 1'b0};	//1-bit stop, 1-bit parity, data, 1-bit start
				else
					data_frame = {1'b1, ^data, data, 1'b0};	//1-bit stop, 1-bit parity, data, 1-bit start
				#(`CLK8X_DELAY * 8)
				serial = data_frame[0];
				for (j=1; j< 11; j=j+1) begin
					#(`CLK8X_DELAY * 8) serial = data_frame[j];				
				end
			end
			#1
			send_start = 0;
		end
		// Wait 10000 ns for global reset to finish
		#10000
		$finish;
        
		// Add stimulus here
	end
      
endmodule

