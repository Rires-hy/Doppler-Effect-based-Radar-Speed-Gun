`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:29:47 03/21/2021 
// Design Name: 
// Module Name:    SerialController 
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
module SerialController(
    input wire serial,
    input wire clk,
    input wire rst_n,
    output reg [7:0] msg,
    output reg noti
    );

    reg [2:0] sample_index;
    reg [2:0] data_index;
    reg [2:0] serial_state;
    reg [3:0] p_counter;
    reg [3:0] n_counter;

    localparam S_WAIT = 0, S_START = 1, S_DATA = 2, S_PARITY = 3, S_STOP = 4;

    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n)
        begin
            noti <= 1'b0;
            msg <= 8'b00000000;
            sample_index <= 3'b000;
            data_index <= 3'b000;
            p_counter <= 4'b0000;
            n_counter <= 4'b0000;
            serial_state <= S_WAIT;
        end
        else
        begin
            case (serial_state)
                S_WAIT : 
                begin
                    if (!serial)
                    begin
                        serial_state <= S_START;
                        noti <= 1'b0;
                    end
                end

                S_START :
                begin
                    if (serial)
                        p_counter <= p_counter + 1'b1;
                    else
                        n_counter <= n_counter + 1'b1;
                    
                    if(sample_index == 3'b111)
                    begin
                        if (p_counter < n_counter)
                            serial_state <= S_DATA;
                        else
                            serial_state <= S_WAIT;

                        sample_index <= 3'b000;
                        p_counter <= 4'b0000;
                        n_counter <= 4'b0000;
                    end
                    else
                        sample_index <= sample_index + 1'b1;
                end

                S_DATA :
                begin
                    if (serial)
                        p_counter <= p_counter + 1'b1;
                    else
                        n_counter <= n_counter + 1'b1;

                    if(sample_index == 3'b111)
                    begin
                        if (p_counter >= n_counter)
                            msg[data_index] <= 1'b1;
                        else
                            msg[data_index] <= 1'b0;

                        if (data_index == 3'b111)
                        begin
                            serial_state <= S_PARITY;
                            data_index <= 3'b000;
                        end
                        else
                            data_index <= data_index + 1'b1;

                        sample_index <= 3'b000;
                        p_counter <= 4'b0000;
                        n_counter <= 4'b0000;

                    end
                    else
                        sample_index <= sample_index + 1'b1;

                end

                S_PARITY :
                begin
                    if (serial)
                        p_counter <= p_counter + 1'b1;
                    else
                        n_counter <= n_counter + 1'b1;
                    
                    if(sample_index == 3'b111)
                    begin
                        if (p_counter >= n_counter)
                            if (^msg == 1'b1)
                            begin
                                serial_state <= S_STOP;
                            end
                            else
                                serial_state <= S_WAIT;
                        else
                            if (^msg == 1'b0)
                            begin
                                serial_state <= S_STOP;
                            end
                            else
                                serial_state <= S_WAIT;

                        sample_index <= 3'b000;
                        p_counter <= 4'b0000;
                        n_counter <= 4'b0000;
                    end
                    else
                        sample_index <= sample_index + 1'b1;
                end

                S_STOP :
                begin
                    if (serial)
                        p_counter <= p_counter + 1'b1;
                    else
                        n_counter <= n_counter + 1'b1;
                    
                    if(sample_index == 3'b111)
                    begin
                        if (p_counter >= n_counter)
                        begin
                            noti <= 1'b1;
                        end
                        
                        serial_state <= S_WAIT;
                        
                        sample_index <= 3'b000;
                        p_counter <= 4'b0000;
                        n_counter <= 4'b0000;
                    end
                    else
                        sample_index <= sample_index + 1'b1;
                end
                default: 
                begin
                    noti <= 1'b0;
                    msg <= 8'b00000000;
                    sample_index <= 3'b000;
                    data_index <= 3'b000;
                    p_counter <= 4'b0000;
                    n_counter <= 4'b0000;
                    serial_state <= S_WAIT;
                end
            endcase
        end
    end

endmodule
