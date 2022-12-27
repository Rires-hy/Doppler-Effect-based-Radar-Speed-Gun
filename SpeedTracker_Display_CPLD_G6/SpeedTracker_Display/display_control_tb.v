`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   01:02:28 09/13/2018
// Design Name:   display_control
// Module Name:   C:/modules/H62PEP_17_UK/refs_wn/Xilinx_CPLD/projects/display_control_simp/display_control_tb.v
// Project Name:  display_control_simp
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: display_control
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

module display_control_tb;

	// Inputs
	reg rstn;
	reg osc_1p8M;
	reg serial_in;

	// Outputs
	wire [7:0] led1_pin_out;
	wire [7:0] led2_pin_out;

	reg send_start;
	integer i, j, byte_num;
	reg [7:0] data, data_0;
	reg[10:0] data_frame;
	
	// Instantiate the Unit Under Test (UUT)
	DisplayController uut (
		.rst_n(rstn), 
		.clk1M8(osc_1p8M), 
		.serial(serial_in),
		.segL(led1_pin_out),
		.segR(led2_pin_out)
	);
	
	//1.8432MHz clock generation
	initial begin                                                                                        
		osc_1p8M = 0;
      forever   
			#(`CLK8X_DELAY/8) osc_1p8M = !osc_1p8M;                                                          
		end 
	
	initial begin
		rstn = 1;
		send_start = 0;
		serial_in = 1;
		byte_num = 3;
		data = 48;
		data_0 = 0;
		#30
		rstn = 0;
		#30
		rstn = 1;
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
				serial_in = data_frame[0];
				for (j=1; j< 11; j=j+1) begin
					#(`CLK8X_DELAY * 8) serial_in = data_frame[j];				
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


