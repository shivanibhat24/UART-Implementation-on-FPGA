`timescale 1ns/1ps
module uart_rx_tb;
    parameter CLK_FREQ = 50000000;
    parameter BAUD_RATE = 115200;
    localparam CLK_PERIOD = 20; // 50 MHz clock
    reg clk;
    reg rst;
    // TX signals
    reg uart_start;
    reg [7:0] uart_data;
    wire tx_pin;
    wire tx_busy;
    // RX signals
    wire [7:0] rx_data;
    wire rx_done;
    wire rx_busy;
    // Clock generation
    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end
    // UART transmitter
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) tx_inst (
        .clk(clk),
        .rst(rst),
        .uart_start(uart_start),
        .uart_data(uart_data),
        .tx_pin(tx_pin),
        .tx_busy(tx_busy)
    );
    // UART receiver
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) rx_inst (
        .clk(clk),
        .rst(rst),
        .rx_pin(tx_pin),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .rx_busy(rx_busy)
    );
    // Test sequence
    initial begin
        clk = 0;
        rst = 1;
        uart_start = 0;
        uart_data = 8'h00;
        // Reset
        #100;
        rst = 0;
        // Wait before transmission
        #100;
        // Send ASCII 'A'
        uart_data = 8'h41;
        uart_start = 1;
        #(CLK_PERIOD);
        uart_start = 0;
        // Wait for receiver
        wait(rx_done == 1'b1);
        // Check received byte
        if(rx_data == 8'h41) begin
            $display("UART RX TEST PASSED");
            $display("Received Data = %h", rx_data);
        end else begin
            $display("UART RX TEST FAILED");
            $display("Expected = 41, Received = %h", rx_data);
        end
        #100;
        $finish;
    end
    // Monitor signals
    initial begin
        $monitor(
            "Time=%0t | TX=%b | TX_BUSY=%b | RX_DATA=%h | RX_DONE=%b",
            $time,
            tx_pin,
            tx_busy,
            rx_data,
            rx_done
        );
    end
endmodule
