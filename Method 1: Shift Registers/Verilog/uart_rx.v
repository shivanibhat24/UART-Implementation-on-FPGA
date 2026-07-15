// Simple UART Receiver using oversampling by bit timing
module uart_rx #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst,
    // UART serial input
    input wire rx_pin,    // Received byte
    output reg [7:0] rx_data,
    // One clock pulse when reception completes
    output reg rx_done,
    // Receiver busy status
    output wire rx_busy
);
    // Number of FPGA clocks per UART bit
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;
    // Counter for baud timing
    reg [$clog2(BAUD_DIV)-1:0] baud_cnt;
    // Counts received bits
    reg [3:0] bit_cnt;
    // Stores incoming bits
    reg [7:0] rx_shift;
    // Receiver active flag
    reg busy;
    assign rx_busy = busy;
    always @(posedge clk) begin
        if(rst) begin
            baud_cnt <= 0;
            bit_cnt <= 0;
            rx_shift <= 0;
            rx_data <= 0;
            rx_done <= 0;
            busy <= 0;
        end else begin
            // Default value
            rx_done <= 0;
            // Waiting for start bit
            if(!busy) begin
                // Detect falling edge
                if(rx_pin == 1'b0) begin
                    busy <= 1'b1;
                    // Wait half a bit before sampling
                    baud_cnt <= BAUD_DIV/2;
                    bit_cnt <= 0;
                end
            end else begin
                if(baud_cnt == BAUD_DIV-1) begin
                    baud_cnt <= 0;
                    // Receive data bits
                    if(bit_cnt < 8) begin
                        rx_shift[bit_cnt] <= rx_pin;
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                    // Stop bit
                    else begin
                        busy <= 1'b0;
                        rx_data <= rx_shift;
                        rx_done <= 1'b1;
                        bit_cnt <= 0;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end
        end
    end
endmodule
