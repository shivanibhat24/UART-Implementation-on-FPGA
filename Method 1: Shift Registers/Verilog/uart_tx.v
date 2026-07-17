// Simple implementation of UART Transmitter using shift register approach
module uart_tx #(
    // Clock frequency of FPGA
    parameter CLK_FREQ = 50000000,
    // UART baud rate
    parameter BAUD_RATE = 115200
)(
    // FPGA clock input
    input wire clk,
    // Reset signal to return transmitter to original state
    input wire rst,    // Signal to start UART transmission
    input wire uart_start,
    // Parallel data that needs to be transmitted
    input wire [7:0] uart_data,
    // UART serial output pin
    output wire tx_pin,
    // Signal indicating transmitter is busy
    output wire tx_busy
);
    // Calculate number of clock cycles required for one UART bit
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;
    // Count FPGA clock cycles until one UART bit is complete
    reg [$clog2(BAUD_DIV)-1:0] baud_cnt;
    // Count transmitted bits
    // UART frame contains:
    // 1 start bit + 8 data bits + 1 stop bit
    reg [3:0] bit_cnt;
    // Shift register containing complete UART frame
    reg [9:0] shift_reg;
    // Internal busy signal
    reg busy;
    // Connecting internal signals to output ports
    assign tx_busy = busy;
    // The first bit of the shift register is always sent
    assign tx_pin = shift_reg[0];
    // Main sequential process that runs on every rising edge of clock
    always @(posedge clk) begin
        // Checking reset condition
        if(rst) begin
            // Transmitter becomes inactive
            busy <= 1'b0;
            // UART idle state is logic high
            // Fill shift register with ones
            shift_reg <= 10'b1111111111;
            // Reset counters
            baud_cnt <= 0;
            bit_cnt <= 0;
        end else begin
            // Check if transmitter is currently idle
            if(!busy) begin
                // Check if a transmission request has arrived
                if(uart_start) begin
                    // Mark transmitter as active
                    busy <= 1'b1;
                    // Create UART frame
                    //
                    // Bit 0  = Start bit
                    // Bits 1-8 = Data bits
                    // Bit 9  = Stop bit
                    //
                    // Data is sent LSB first
                    shift_reg <= {1'b1, uart_data, 1'b0};
                    // Start counting from first bit
                    baud_cnt <= 0;
                    bit_cnt <= 0;
                end
            end else begin
                // Count FPGA clock cycles for current bit
                if(baud_cnt == BAUD_DIV-1) begin
                    // Restart baud counter
                    baud_cnt <= 0;
                    // Shift next bit onto TX pin
                    shift_reg <= {1'b1, shift_reg[9:1]};
                    // Check if complete UART frame has been transmitted
                    if(bit_cnt == 4'd9) begin
                        // Transmission complete
                        busy <= 1'b0;
                        // Reset bit counter
                        bit_cnt <= 0;
                    end else begin
                        // Move to next UART bit
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end else begin
                    // Increase clock counter
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end
        end
    end
endmodule
