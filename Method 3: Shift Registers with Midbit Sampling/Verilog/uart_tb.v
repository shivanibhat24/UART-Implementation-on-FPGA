// =====================================================================
// tb_uart_m1.v -- loopback + fault-injection testbench for Method 1
//
//   iverilog -g2005 -Wall -o sim_m1 tb_uart_m1.v uart_m1.v && vvp sim_m1
//
//   Uses a reduced CLKS_PER_BIT so the sim finishes quickly. The timing
//   logic is identical -- only the divisor changes.
// =====================================================================

`timescale 1ns / 1ps

module tb_uart_m1;

    localparam integer CPB = 16;      // clocks per bit (small, for sim speed)

    reg        clk   = 1'b0;
    reg        rst_n = 1'b0;
    always #5 clk = ~clk;             // 100 MHz

    reg  [7:0] tx_data  = 8'h00;
    reg        tx_start = 1'b0;
    wire       tx_busy;
    wire       txd;

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       frame_err;

    // Loopback with an injectable disturbance.
    reg        force_glitch = 1'b0;
    wire       rxd = force_glitch ? 1'b0 : txd;

    uart_m1 #(.CLKS_PER_BIT(CPB)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data   (tx_data),
        .tx_start  (tx_start),
        .tx_busy   (tx_busy),
        .txd       (txd),
        .rxd       (rxd),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .frame_err (frame_err)
    );

    integer errors = 0;
    integer n_rx   = 0;

    // ---- capture every received byte ------------------------------------
    reg [7:0] last_rx;
    always @(posedge clk) begin
        if (rx_valid) begin
            last_rx = rx_data;
            n_rx    = n_rx + 1;
            $display("  [%0t] RX = 0x%02h  frame_err=%b", $time, rx_data, frame_err);
        end
    end

    // ---- send one byte and wait for it -----------------------------------
    task send_and_check(input [7:0] b);
        begin
            @(negedge clk);
            tx_data  = b;
            tx_start = 1'b1;
            @(negedge clk);
            tx_start = 1'b0;

            wait (tx_busy == 1'b1);
            wait (tx_busy == 1'b0);

            // stop bit still has to be sampled by the RX
            repeat (CPB * 3) @(posedge clk);

            if (last_rx !== b) begin
                $display("  FAIL: sent 0x%02h, got 0x%02h", b, last_rx);
                errors = errors + 1;
            end else if (frame_err !== 1'b0) begin
                $display("  FAIL: sent 0x%02h, frame_err asserted", b);
                errors = errors + 1;
            end else begin
                $display("  ok  : 0x%02h", b);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_uart_m1.vcd");
        $dumpvars(0, tb_uart_m1);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("--- loopback, corner patterns ---");
        send_and_check(8'h00);   // all zeros: 8 data bits look like start bits
        send_and_check(8'hFF);   // all ones: data indistinguishable from idle
        send_and_check(8'hA5);
        send_and_check(8'h5A);
        send_and_check(8'h01);   // LSB set only -> catches bit-order errors
        send_and_check(8'h80);   // MSB set only

        $display("--- back-to-back (tx_busy drops at end of stop bit) ---");
        send_and_check(8'h55);
        send_and_check(8'hAA);

        // ------------------------------------------------------------------
        // Glitch rejection: pull the line low for a few clocks while idle.
        // Shorter than HALF_BIT, so S_START must re-check at mid-bit, see
        // the line back high, and return to IDLE without emitting rx_valid.
        // ------------------------------------------------------------------
        $display("--- short glitch on idle line (must NOT produce a byte) ---");
        n_rx = 0;
        @(negedge clk);
        force_glitch = 1'b1;
        repeat (CPB/4) @(posedge clk);
        force_glitch = 1'b0;
        repeat (CPB * 12) @(posedge clk);
        if (n_rx != 0) begin
            $display("  FAIL: glitch produced %0d phantom byte(s)", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : glitch rejected, no phantom frame");
        end

        // ------------------------------------------------------------------
        // Long glitch: hold low past mid-bit so S_START accepts it. This one
        // SHOULD get through as a bogus frame -- Method 1 cannot tell it from
        // real traffic. Demonstrates exactly the hole that 16x oversampling
        // plus majority voting closes in Methods 3/4.
        // ------------------------------------------------------------------
        $display("--- long glitch (Method 1 is EXPECTED to be fooled) ---");
        n_rx = 0;
        @(negedge clk);
        force_glitch = 1'b1;
        repeat (CPB) @(posedge clk);
        force_glitch = 1'b0;
        repeat (CPB * 14) @(posedge clk);
        $display("  -> %0d byte(s) received, frame_err=%b  (this is the weakness)",
                 n_rx, frame_err);

        $display("");
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d FAILURE(S) ===", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
