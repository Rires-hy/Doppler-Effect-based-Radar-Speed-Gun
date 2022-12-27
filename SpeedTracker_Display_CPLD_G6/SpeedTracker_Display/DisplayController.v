`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    03:38:14 03/22/2021 
// Design Name: 
// Module Name:    DisplayController 
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
module DisplayController(
    input wire clk1M8,
    input wire serial,
    input wire rst_n,
    output wire [7:0] segL,
    output wire [7:0] segR,
    output wire DE
    );

    wire clk_div4;
    wire sys_rst_n;

    wire [7:0] msg;
    wire noti;

    SysRstController rst_control(
        .clk_1M8(clk1M8),
        .rst_n(rst_n),
        .sys_rst_n(sys_rst_n)
    );

    ClockDivider clk_div(
        .clk(clk1M8),
        .rst_n(sys_rst_n),
        .clk_div4(clk_div4)
    );

    SerialController serial_control(
        .serial(serial),
        .clk(clk_div4),
        .rst_n(sys_rst_n),
        .msg(msg),
        .noti(noti)
    );

    SegDecoder seg_decode(
        .msg(msg),
        .noti(noti),
        .clk(clk1M8),
        .rst_n(sys_rst_n),
        .segL(segL),
        .segR(segR)
    );

    assign DE = 0;

endmodule
