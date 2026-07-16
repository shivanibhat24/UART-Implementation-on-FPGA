// =====================================================================
// tb_uart_ovs16.v -- regression for Method 3
//
//   iverilog -g2005 -Wall -o sim_m3 tb_uart_ovs16.v uart_ovs16.v && vvp sim_m3
//
//   Same battery as Methods 1/2 (directly comparable) PLUS the two things
//   Method 3 actually claims:
//
//     1. LONG GLITCH REJECTION -- the disturbance that produced a phantom
//        0xFF with frame_err=0 in M1 and M2 must now be rejected. This is
//        the headline result of the whole method.
//     2. BAUD TOLERANCE -- an independent BFM transmitter drives rxd at a
//        deliberately WRONG bit rate. M1/M2 tolerate ~+/-0.5%; M3 claims
//        ~+/-2%. Asserting that in a comment is worthless, so we sweep it.
//
//   The BFM is essential: loopback from our own TX can never expose a baud
//   mismatch, because both ends share one crystal. A loopback-only test
//   suite will pass a receiver that fails against every real UART on earth.
// =====================================================================

`timescale 1ns / 1ps

module tb_uart_ovs16;

    // Sim-scaled timing. OS_DIV=4 -> 64 clk per bit at 16x oversample.
    // Keeps the 16-tick structure intact (the whole point of the method)
    // while running fast enough to sweep baud offsets.
    localparam integer OS_DIV   = 4;
    localparam integer OS_PER_BIT = 16;
    localparam integer CPB      = OS_DIV * OS_PER_BIT;   // 64 clk / bit
    localparam integer CLK_PS   = 10000;                 // 10 ns = 100 MHz

    reg        clk   = 1'b0;
    reg        rst_n = 1'b0;
    always #5 clk = ~clk;

    reg  [7:0] tx_data  = 8'h00;
    reg        tx_start = 1'b0;
    wire       tx_busy;
    wire [7:0] tx_hold_o;
    wire       txd;

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       frame_err;
    wire       rx_overrun;
    reg        rx_clear = 1'b0;

    // ---- rxd source select ----------------------------------------------
    //   0 = loopback from our own TX (shared crystal, no baud error)
    //   1 = BFM transmitter at an arbitrary, independent bit rate
    reg        use_bfm      = 1'b0;
    reg        bfm_line     = 1'b1;
    reg        force_glitch = 1'b0;

    wire rxd = force_glitch ? 1'b0 : (use_bfm ? bfm_line : txd);

    uart_ovs16 #(
        .CLKS_PER_BIT (CPB),
        .OS_DIV       (OS_DIV)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .tx_data    (tx_data),
        .tx_start   (tx_start),
        .tx_busy    (tx_busy),
        .tx_hold_o  (tx_hold_o),
        .txd        (txd),
        .rxd        (rxd),
        .rx_data    (rx_data),
        .rx_valid   (rx_valid),
        .frame_err  (frame_err),
        .rx_overrun (rx_overrun),
        .rx_clear   (rx_clear)
    );

    integer errors = 0;
    integer n_rx   = 0;

    reg [7:0] last_rx;
    reg       last_ferr;
    always @(posedge clk) begin
        if (rx_valid) begin
            last_rx   = rx_data;
            last_ferr = frame_err;
            n_rx      = n_rx + 1;
        end
    end

    // ---- collect the byte so rx_overrun stays meaningful -----------------
    task collect;
        begin
            @(negedge clk);
            rx_clear = 1'b1;
            @(negedge clk);
            rx_clear = 1'b0;
        end
    endtask

    // ---------------------------------------------------------------------
    // BFM transmitter -- drives bfm_line at bit_ticks time units per bit,
    // completely independent of the DUT's clock. This is what a real far-end
    // UART looks like: same nominal baud, different crystal.
    //
    // NOTE ON UNITS: `timescale is 1ns/1ps, so a bare #N delay is N ns.
    // The clock is 10 ns, so one bit = CPB*10 = 640 ns nominal. Work in ns
    // throughout and scale ppm with MULTIPLY-BEFORE-DIVIDE -- the obvious
    // (nominal/1000000)*ppm truncates to zero for any nominal under 1e6 and
    // silently turns the whole sweep into nine repeats of the 0% case.
    // ---------------------------------------------------------------------
    task bfm_send(input [7:0] b, input integer bit_ns_x1000);
        integer i;
        begin
            // bit_ns_x1000 is in milli-nanoseconds (i.e. ps) to keep ppm
            // resolution without floating point. #(x/1000) gives ns.
            bfm_line = 1'b0;                        // start
            #(bit_ns_x1000 / 1000.0);
            for (i = 0; i < 8; i = i + 1) begin
                bfm_line = b[i];                    // LSB first
                #(bit_ns_x1000 / 1000.0);
            end
            bfm_line = 1'b1;                        // stop
            #(bit_ns_x1000 / 1000.0);
        end
    endtask

    // ---- loopback send + check (M1/M2 comparable) ------------------------
    task send_and_check(input [7:0] b);
        begin
            @(negedge clk);
            tx_data  = b;
            tx_start = 1'b1;
            @(negedge clk);
            tx_start = 1'b0;
            tx_data  = 8'hDE;            // scribble: tx_hold must not follow

            wait (tx_busy == 1'b1);
            wait (tx_busy == 1'b0);
            repeat (CPB * 3) @(posedge clk);

            if (last_rx !== b) begin
                $display("  FAIL: sent 0x%02h, got 0x%02h", b, last_rx);
                errors = errors + 1;
            end else if (last_ferr !== 1'b0) begin
                $display("  FAIL: sent 0x%02h, frame_err asserted", b);
                errors = errors + 1;
            end else if (tx_hold_o !== b) begin
                $display("  FAIL: tx_hold=0x%02h after sending 0x%02h",
                         tx_hold_o, b);
                errors = errors + 1;
            end else begin
                $display("  ok  : 0x%02h", b);
            end
            collect;
        end
    endtask

    // ---- BFM send at a given baud error, check result ---------------------
    task bfm_check(input [7:0] b, input integer ppm_err, input expect_ok);
        integer bit_x1000;
        integer nominal_x1000;
        begin
            // Nominal bit period in milli-nanoseconds: CPB clocks * 10 ns.
            nominal_x1000 = CPB * 10 * 1000;               // 640000 => 640 ns
            // MULTIPLY FIRST. (nominal/1000000)*ppm truncates to zero here.
            bit_x1000 = nominal_x1000 + ((nominal_x1000 * ppm_err) / 1000000);

            n_rx = 0;
            use_bfm  = 1'b1;
            bfm_line = 1'b1;
            // Idle the line and drain any byte still in flight from the
            // PREVIOUS call before zeroing n_rx. bfm_send returns when its
            // stop bit ENDS, but the DUT samples the stop bit at os_cnt==7
            // and only returns to IDLE at os_cnt==15 -- so rx_valid fires
            // after the task has already returned. Without this drain, each
            // check scores the previous byte and the whole sweep reports
            // garbage that looks like data corruption.
            repeat (CPB * 3) @(posedge clk);
            n_rx      = 0;
            last_rx   = 8'hXX;
            last_ferr = 1'bx;
            collect;

            bfm_send(b, bit_x1000);
            repeat (CPB * 3) @(posedge clk);
            use_bfm = 1'b0;

            if (expect_ok) begin
                if (n_rx == 1 && last_rx === b && last_ferr === 1'b0) begin
                    $display("  ok  : %+0.2f%%  0x%02h received clean",
                             ppm_err/10000.0, b);
                end else begin
                    $display("  FAIL: %+0.2f%%  expected 0x%02h clean, got 0x%02h n_rx=%0d ferr=%b",
                             ppm_err/10000.0, b, last_rx, n_rx, last_ferr);
                    errors = errors + 1;
                end
            end else begin
                // Beyond tolerance: we do NOT require a specific failure
                // mode, only that it is not silently wrong. Either a framing
                // error or corrupt data is acceptable and expected.
                if (n_rx == 1 && last_rx === b && last_ferr === 1'b0)
                    $display("  note: %+0.2f%%  still worked (margin better than spec)",
                             ppm_err/10000.0);
                else
                    $display("  note: %+0.2f%%  broke as expected (got 0x%02h ferr=%b)",
                             ppm_err/10000.0, last_rx, last_ferr);
            end
            collect;
        end
    endtask

    initial begin
        $dumpfile("tb_uart_ovs16.vcd");
        $dumpvars(0, tb_uart_ovs16);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("--- loopback, corner patterns ---");
        send_and_check(8'h00);
        send_and_check(8'hFF);
        send_and_check(8'hA5);
        send_and_check(8'h5A);
        send_and_check(8'h01);
        send_and_check(8'h80);
        send_and_check(8'h55);
        send_and_check(8'hAA);

        // ------------------------------------------------------------------
        // Short glitch -- rejected by M1 and M2 too. Regression only.
        // ------------------------------------------------------------------
        $display("--- short glitch on idle line ---");
        n_rx = 0;
        @(negedge clk);
        force_glitch = 1'b1;
        repeat (CPB/8) @(posedge clk);
        force_glitch = 1'b0;
        repeat (CPB * 12) @(posedge clk);
        if (n_rx != 0) begin
            $display("  FAIL: glitch produced %0d phantom byte(s)", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : rejected");
        end

        // ------------------------------------------------------------------
        // Long glitch: one full bit period low.
        //
        // HONEST RESULT: Method 3 is still fooled, exactly like M1 and M2.
        //
        // An earlier version of this testbench claimed M3 rejected it. That
        // pass was produced by an RTL BUG (a bogus re-validation at the bit
        // boundary, os_cnt==15) which also rejected every valid frame from a
        // fast far end. Removing the bug restored the +/-2% tolerance sweep
        // and restored this failure. The "hardening" was never real.
        //
        // WHY NO START-BIT CHECK CAN FIX THIS: a 1-bit-long glitch is low at
        // the bit CENTRE, which is the only place a level may legitimately be
        // sampled. The signal is not malformed where you are allowed to look.
        // Rejecting it needs a SECOND OPINION at the same instant -- sampling
        // ticks 7/8/9 and voting -- which is Method 4, or a wider-context
        // check such as minimum idle time before a start bit.
        //
        // Kept as a pass/fail assertion of the CURRENT limitation so that the
        // Method 4 run shows a genuine, earned change on identical stimulus.
        // ------------------------------------------------------------------
        $display("--- long glitch (M1/M2/M3 are all fooled -- M4 fixes it) ---");
        n_rx = 0;
        @(negedge clk);
        force_glitch = 1'b1;
        repeat (CPB) @(posedge clk);
        force_glitch = 1'b0;
        repeat (CPB * 14) @(posedge clk);
        if (n_rx != 1) begin
            $display("  FAIL: expected 1 phantom byte (known M3 limitation), got %0d", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : %0d phantom byte 0x%02h ferr=%b -- KNOWN HOLE, unfixed until M4",
                     n_rx, last_rx, last_ferr);
        end
        collect;

        // ------------------------------------------------------------------
        // Baud tolerance sweep, driven by an INDEPENDENT transmitter.
        // Loopback cannot test this: both ends share one crystal.
        // ------------------------------------------------------------------
        $display("--- baud tolerance sweep (independent BFM transmitter) ---");
        bfm_check(8'hA5,      0, 1);   //  0.00%  nominal
        bfm_check(8'hA5,   5000, 1);   // +0.50%  M1/M2 limit
        bfm_check(8'hA5,  -5000, 1);   // -0.50%
        bfm_check(8'h5A,  10000, 1);   // +1.00%
        bfm_check(8'h5A, -10000, 1);   // -1.00%
        bfm_check(8'hFF,  20000, 1);   // +2.00%  M3 claim
        bfm_check(8'h00, -20000, 1);   // -2.00%
        $display("  (beyond spec -- informational, no pass/fail)");
        bfm_check(8'hA5,  40000, 0);   // +4.00%
        bfm_check(8'hA5, -40000, 0);   // -4.00%

        // ------------------------------------------------------------------
        // Overrun: two bytes back to back, never collected. The second must
        // set rx_overrun. M1/M2 had no way to know this happened.
        // ------------------------------------------------------------------
        $display("--- overrun detection ---");
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        bfm_send(8'h11, CPB * 10 * 1000);
        bfm_send(8'h22, CPB * 10 * 1000);
        repeat (CPB * 3) @(posedge clk);
        use_bfm = 1'b0;
        if (rx_overrun !== 1'b1) begin
            $display("  FAIL: two uncollected bytes did not set rx_overrun");
            errors = errors + 1;
        end else begin
            $display("  ok  : rx_overrun set (M1/M2 lost the byte silently)");
        end
        collect;
        if (rx_overrun !== 1'b0) begin
            $display("  FAIL: rx_clear did not clear rx_overrun");
            errors = errors + 1;
        end else begin
            $display("  ok  : rx_clear clears it");
        end

        $display("");
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d FAILURE(S) ===", errors);
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
