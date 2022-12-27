`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    03:04:13 03/22/2021 
// Design Name: 
// Module Name:    SysRstController 
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
module SysRstController(
    input wire clk_1M8,
    input wire rst_n,
    output reg sys_rst_n
    );

    //reg [1:0] count;

    initial
    begin
        //count = 0;
    end

    always @(posedge clk_1M8 or negedge rst_n)
    begin
        if (!rst_n)
        begin
            //count <= 0;
            sys_rst_n <= 0;
        end
        else// if (count == 2'b1)    //17'h3ffff
        begin
            sys_rst_n <= 1;
            //count <= 2'b1;         //17'h3ffff
        end
        // else
        // begin
        //     count <= count + 1;
        //     sys_rst_n <= 0;
        // end
    end

endmodule
