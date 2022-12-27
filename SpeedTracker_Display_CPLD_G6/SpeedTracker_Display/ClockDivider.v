`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    03:15:04 03/21/2021 
// Design Name: 
// Module Name:    ClockDivider 
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
module ClockDivider(
	input wire clk,
	input wire rst_n,
	output reg clk_div4
    );

	reg [1:0] count;
	
	always @(posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			count <= 1'b0;
			clk_div4 <= 1'b0;
		end
		else if(count == 2'b01)	//01
		begin
			count <= 1'b0;
			clk_div4 <= ~clk_div4;
		end
		else
			count <= count + 1'b1;
	end

endmodule
