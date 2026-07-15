// Testbench for UART Transmitter
module uart_tx_tb;
    // Defining the clock frequency of the transmitter
    parameter CLK_FREQ = 50000000;
    // Defining the UART baud rate
    parameter BAUD_RATE = 115200;
    // Creating clock signal
    reg clk;
    // Reset signal to return transmitter to original state
    reg rst;
    // Signal to start UART transmission
    reg uart_start;
    // Data to be transmitted
    reg [7:0] uart_data;
    // UART serial output
    wire tx_pin;
    // Signal indicating transmitter is active
    wire tx_busy;
    // Clock period calculation
    // 50 MHz clock = 20 ns period
    localparam CLK_PERIOD = 20;
    // Generate FPGA clock
    // This process continuously toggles the clock
    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end
    // Instantiate UART transmitter
    // This is the hardware module being tested
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    )
    dut (
        .clk(clk),
        .rst(rst),
        .uart_start(uart_start),
        .uart_data(uart_data),
        .tx_pin(tx_pin),
        .tx_busy(tx_busy)
    );
    // Main test process
    initial begin
        // Initial values
        clk = 1'b0;
        rst = 1'b1;
        uart_start = 1'b0;
        uart_data = 8'b0;
        // Apply reset
        #100;
        rst = 1'b0;
        // Allow transmitter to settle
        #100;
        // Sending byte 0x41
        //
        // Binary:
        // 01000001
        //
        // UART sends:
        //
        // Start bit = 0
        // Data bits = 1 0 0 0 0 0 1 0
        // Stop bit  = 1
        uart_data = 8'h41;
        // Request transmission
        uart_start = 1'b1;
        // Start signal is only one clock pulse
        #(CLK_PERIOD);
        uart_start = 1'b0;
        // Wait until transmission is complete
        wait(tx_busy == 1'b0);
        // Small delay before ending simulation
        #100;
        // End simulation
        $display("UART TX Test Completed");
        $finish;
    end
    // Monitor important signals during simulation
   initial begin
    $monitor(
        "Time=%0t | Start=%b | Data=%h | TX=%b | Busy=%b",
        $time, uart_start, uart_data, tx_pin, tx_busy
    );
  end
endmodule
