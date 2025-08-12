`timescale 1ns / 1ps

module tlul_spi_monitor#(
    parameter TL_ADDR_WIDTH     = 64,
    parameter TL_DATA_WIDTH     = 64,
    parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH   = 3,
    parameter TL_SINK_WIDTH     = 3,
    parameter TL_OPCODE_WIDTH   = 3,
    parameter TL_PARAM_WIDTH    = 3,
    parameter TL_SIZE_WIDTH     = 8
) (
    input wire a_valid_tb,
    input wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb,
    input wire [TL_PARAM_WIDTH-1:0]  a_param_tb,
    input wire [TL_ADDR_WIDTH-1:0]   a_address_tb,
    input wire [TL_SIZE_WIDTH-1:0]   a_size_tb,
    input wire [TL_STRB_WIDTH-1:0]   a_mask_tb,
    input wire [TL_DATA_WIDTH-1:0]   a_data_tb,
    input wire [TL_SOURCE_WIDTH-1:0] a_source_tb,
    input wire a_ready_tb,

    input wire d_valid_tb,
    input wire d_ready_tb,
    input wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb,
    input wire [TL_PARAM_WIDTH-1:0]  d_param_tb,
    input wire [TL_SIZE_WIDTH-1:0]   d_size_tb,
    input wire [TL_SINK_WIDTH-1:0]   d_sink_tb,
    input wire [TL_SOURCE_WIDTH-1:0] d_source_tb,
    input wire [TL_DATA_WIDTH-1:0]   d_data_tb,
    input wire d_error_tb );

/*
    input wire clk,
    input wire rst,
// A Channel (Request from Master to Slave)
    input wire a_valid,
    input wire a_ready,
    input wire [2:0] a_opcode,
    input wire [63:0] a_address,
    input wire [63:0] a_data,

// D Channel (Response from Slave to Master)
    input wire d_valid,
    input wire d_ready,
    input wire [2:0] d_opcode,
    input wire [31:0] d_data,

// SPI-specific signals
    input wire spi_start,
    input wire spi_done,
    input wire data_end,
    input wire [7:0] spi_cmd,
    input wire [23:0] spi_addr,
    input wire [31:0] spi_data,
    input wire [31:0] spi_resp*/

/*
// Monitor the A Channel Transactions
always @(posedge clk) begin
    if (rst) begin
        $display("[%0t] MONITOR: Reset Asserted", $time);
    end else begin
        if (a_valid && a_ready) begin
            case (a_opcode)
                3'd0: $display("[%0t] MONITOR: TL-UL WRITE Transaction Initiated | Address: %h | Data: %h", $time, a_address, a_data);
                3'd4: $display("[%0t] MONITOR: TL-UL READ Transaction Initiated | Address: %h", $time, a_address);
                default: $display("[%0t] MONITOR: TL-UL UNKNOWN Transaction | Opcode: %d", $time, a_opcode);
            endcase
        end
    end
end

// Monitor the D Channel Transactions
always @(posedge clk) begin
    if (d_valid && d_ready) begin
        case (d_opcode)
            3'd0: $display("[%0t] MONITOR: TL-UL WRITE ACK received from Slave", $time);
            3'd1: $display("[%0t] MONITOR: TL-UL READ ACK received from Slave | Data: %h", $time, d_data);
            default: $display("[%0t] MONITOR: TL-UL UNKNOWN Response | Opcode: %d", $time, d_opcode);
        endcase
    end
end

// Monitor the SPI Interface Activity
always @(posedge clk) begin
    if (spi_start) begin
        $display("[%0t] MONITOR: SPI Transaction START | CMD: %h | ADDR: %h | DATA: %h", $time, spi_cmd, spi_addr, spi_data);
    end
    if (spi_done) begin
        $display("[%0t] MONITOR: SPI Transaction DONE | Response Data: %h", $time, spi_resp);
    end
    if (data_end) begin
        $display("[%0t] MONITOR: SPI Data Transfer END", $time);
    end
end */
endmodule
