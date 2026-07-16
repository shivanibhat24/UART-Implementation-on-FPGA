// =====================================================================
// tb_uart_mvote_fifo.v -- regression for Method 4
//
//   iverilog -g2005 -Wall -o sim_m4 tb_uart_mvote_fifo.v uart_mvote_fifo.v
//   vvp sim_m4
//
//   Carries forward the M1/M2/M3 battery (directly comparable) and adds the
//   tests for what Method 4 actually claims:
//
//     1. TARGETED GLITCH. A disturbance placed on exactly ONE of the three
//        vote points (ticks 7/8/9). This is THE test. M3 samples only at
//        tick 7, so a glitch there corrupts the bit silently. M4 outvotes
//        it 2-to-1 and also raises rx_noise. Run against M3 it fails; that
//        is the point.
//     2. rx_noise as a link-quality measure -- must be clean on a clean
//        link and set on a disturbed one.
//     3. FIFO depth: burst DEPTH bytes with no consumer, then drain.
//     4. FIFO full -> overrun flagged, not silent.
//     5. Framing errors keep the bad byte OUT of the FIFO.
//     6. Baud tolerance sweep against an INDEPENDENT time base.
//
//   Reduced OS_DIV for sim speed; the 16-tick structure is intact, which
//   is what the vote-point tests depend on.
// =====================================================================

`timescale 1ns / 1ps

module tb_uart_mvote_fifo;

    // OS_DIV=4, 16 ticks/bit -> 64 clk/bit. 10ns clk -> 640 ns/bit.
    localparam integer OS_DIV     = 4;
    localparam integer OS_PER_BIT = 16;
    localparam integer CPB        = OS_DIV * OS_PER_BIT;   // 64 clk / bit
    localparam integer BIT_NS     = CPB * 10;              // 640 ns
    localparam integer FIFO_DEPTH = 16;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;                                   // 100 MHz

    // TX producer
    reg  [7:0] tx_data = 8'h00;
    reg        tx_we   = 1'b0;
    wire       tx_full;
    wire [4:0] tx_count;
    wire       tx_active;
    wire       txd;

    // RX consumer
    wire [7:0] rx_data;
    reg        rx_re = 1'b0;
    wire       rx_empty;
    wire [4:0] rx_count;

    wire       frame_err;
    wire       rx_overrun;
    wire       rx_noise;
    reg        err_clear = 1'b0;

    // ---- rxd source select ------------------------------------------
    //   0 = loopback from our own TX (shared crystal -- no baud error)
    //   1 = BFM line (independent time base, or deliberately malformed)
    reg  use_bfm  = 1'b0;
    reg  bfm_line = 1'b1;
    reg  glitch   = 1'b0;

    wire rxd = glitch ? ~(use_bfm ? bfm_line : txd)   // invert = disturb
                      :  (use_bfm ? bfm_line : txd);

    uart_mvote_fifo #(
        .CLKS_PER_BIT (CPB),
        .OS_DIV       (OS_DIV),
        .FIFO_DEPTH   (FIFO_DEPTH)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .tx_data    (tx_data),
        .tx_we      (tx_we),
        .tx_full    (tx_full),
        .tx_count   (tx_count),
        .tx_active  (tx_active),
        .txd        (txd),
        .rxd        (rxd),
        .rx_data    (rx_data),
        .rx_re      (rx_re),
        .rx_empty   (rx_empty),
        .rx_count   (rx_count),
        .frame_err  (frame_err),
        .rx_overrun (rx_overrun),
        .rx_noise   (rx_noise),
        .err_clear  (err_clear)
    );

    integer errors = 0;

    // -----------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------
    task push_tx(input [7:0] b);
        begin
            @(negedge clk);
            tx_data = b;
            tx_we   = 1'b1;
            @(negedge clk);
            tx_we   = 1'b0;
        end
    endtask

    task pop_rx(output [7:0] b);
        begin
            @(negedge clk);
            b     = rx_data;      // async read -- head is already present
            rx_re = 1'b1;
            @(negedge clk);
            rx_re = 1'b0;
        end
    endtask

    task clear_errs;
        begin
            @(negedge clk);
            err_clear = 1'b1;
            @(negedge clk);
            err_clear = 1'b0;
        end
    endtask

    // Drain the RX FIFO. Deliberately does NOT clear the error flags: a
    // draining helper that also wipes frame_err/rx_overrun will silently
    // destroy the very evidence the next check is about to look for. Call
    // clear_errs explicitly when you want the flags reset.
    task drain_rx;
        reg [7:0] junk;
        begin
            while (!rx_empty) pop_rx(junk);
        end
    endtask

    // Drain AND reset the flags -- for use between unrelated tests.
    task reset_rx;
        begin
            drain_rx;
            clear_errs;
        end
    endtask

    // -----------------------------------------------------------------
    // BFM transmitter -- independent time base. Loopback CANNOT test baud
    // tolerance because both ends share one crystal; a loopback-only suite
    // will pass a receiver that fails against every real UART.
    //
    // bit_x1000 is in picoseconds so ppm offsets are expressible without
    // floating point in the caller.
    // -----------------------------------------------------------------
    task bfm_send(input [7:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;                        // start
            #(bit_x1000 / 1000.0);
            for (i = 0; i < 8; i = i + 1) begin
                bfm_line = b[i];                    // LSB first
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b1;                        // stop
            #(bit_x1000 / 1000.0);
        end
    endtask

    // BFM frame with a deliberately broken stop bit
    task bfm_send_badstop(input [7:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;
            #(bit_x1000 / 1000.0);
            for (i = 0; i < 8; i = i + 1) begin
                bfm_line = b[i];
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b0;                        // stop held SPACE
            #(bit_x1000 / 1000.0);
            bfm_line = 1'b1;
            #(bit_x1000 / 1000.0);
        end
    endtask

    // -----------------------------------------------------------------
    // Loopback round-trip
    // -----------------------------------------------------------------
    task loopback_check(input [7:0] b);
        reg [7:0] got;
        begin
            reset_rx;
            use_bfm = 1'b0;
            push_tx(b);
            wait (tx_active == 1'b1);
            wait (tx_active == 1'b0);
            repeat (CPB * 3) @(posedge clk);

            if (rx_empty) begin
                $display("  FAIL: sent 0x%02h, RX FIFO empty", b);
                errors = errors + 1;
            end else begin
                pop_rx(got);
                if (got !== b) begin
                    $display("  FAIL: sent 0x%02h, got 0x%02h", b, got);
                    errors = errors + 1;
                end else if (frame_err) begin
                    $display("  FAIL: sent 0x%02h, frame_err on a good frame", b);
                    errors = errors + 1;
                end else if (rx_noise) begin
                    // A clean loopback link must NEVER raise rx_noise: all
                    // three samples land in the settled region of the bit.
                    // If this fires, the vote points are misplaced.
                    $display("  FAIL: sent 0x%02h, rx_noise on a CLEAN link", b);
                    errors = errors + 1;
                end else begin
                    $display("  ok  : 0x%02h  (noise=0, ferr=0)", b);
                end
            end
        end
    endtask

    // =================================================================
    integer i;
    reg [7:0] got;
    reg [7:0] expect_b;
    integer   nominal_x1000;
    integer   bit_x1000;
    integer   n_got;

    initial begin
        $dumpfile("tb_uart_mvote_fifo.vcd");
        $dumpvars(0, tb_uart_mvote_fifo);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // -------------------------------------------------------------
        $display("--- 1. loopback, corner patterns ---");
        // 0x00: every data bit is space -- looks like start bits
        // 0xFF: every data bit is mark  -- looks like idle
        // 0x01/0x80: single bits at each end -- catches bit-order errors
        loopback_check(8'h00);
        loopback_check(8'hFF);
        loopback_check(8'hA5);
        loopback_check(8'h5A);
        loopback_check(8'h01);
        loopback_check(8'h80);
        loopback_check(8'h55);
        loopback_check(8'hAA);

        // -------------------------------------------------------------
        // THE HEADLINE TEST.
        //
        // A one-oversample-tick disturbance placed on exactly ONE vote
        // point. Method 3 samples only at tick 7, so a glitch there IS the
        // bit -- corrupted, silently, frame_err=0. Method 4 has two other
        // opinions from the same settled region and outvotes it 2-to-1.
        //
        // Timing: the DUT restarts its oversample phase at the start edge,
        // so tick N of data bit K lands at
        //     (start_edge) + (K+1)*16*OS_DIV + N*OS_DIV  clocks.
        // We aim at data bit 0 (K=0), tick 7.
        // -------------------------------------------------------------
        $display("--- 2. TARGETED GLITCH on ONE vote point (M3 corrupts here) ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);

        fork
            // 0xFF: every data bit is mark, so a glitch inverting one
            // sample to space would flip that bit to 0 if unvoted.
            bfm_send(8'hFF, BIT_NS * 1000);
            begin
                // Wait for the start edge, then walk to data bit 0, tick 7.
                @(negedge bfm_line);              // start edge
                repeat (OS_PER_BIT * OS_DIV) @(posedge clk);  // end of start
                repeat (7 * OS_DIV) @(posedge clk);           // to tick 7
                glitch = 1'b1;                    // invert for ONE tick
                repeat (OS_DIV) @(posedge clk);
                glitch = 1'b0;
            end
        join

        repeat (CPB * 3) @(posedge clk);
        if (rx_empty) begin
            $display("  FAIL: glitched frame vanished entirely");
            errors = errors + 1;
        end else begin
            pop_rx(got);
            if (got !== 8'hFF) begin
                $display("  FAIL: got 0x%02h, expected 0xFF -- VOTE DID NOT OUTVOTE THE GLITCH",
                         got);
                errors = errors + 1;
            end else begin
                $display("  ok  : 0xFF SURVIVED a glitch on vote point 7");
                $display("        (M3 samples only tick 7 -> would return 0xFE silently)");
            end
        end
        // rx_noise must have caught it: the three samples disagreed.
        if (!rx_noise) begin
            $display("  FAIL: rx_noise did NOT flag the disagreement");
            errors = errors + 1;
        end else begin
            $display("  ok  : rx_noise raised -- link degradation is visible");
        end
        use_bfm = 1'b0;
        reset_rx;

        // -------------------------------------------------------------
        // Same glitch on vote point 8 (the middle one). Must also survive.
        // -------------------------------------------------------------
        $display("--- 3. targeted glitch on vote point 8 ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        fork
            bfm_send(8'hFF, BIT_NS * 1000);
            begin
                @(negedge bfm_line);
                repeat (OS_PER_BIT * OS_DIV) @(posedge clk);
                repeat (8 * OS_DIV) @(posedge clk);
                glitch = 1'b1;
                repeat (OS_DIV) @(posedge clk);
                glitch = 1'b0;
            end
        join
        repeat (CPB * 3) @(posedge clk);
        if (rx_empty) begin
            $display("  FAIL: frame vanished");
            errors = errors + 1;
        end else begin
            pop_rx(got);
            if (got !== 8'hFF) begin
                $display("  FAIL: got 0x%02h, expected 0xFF", got);
                errors = errors + 1;
            end else begin
                $display("  ok  : 0xFF survived a glitch on vote point 8");
            end
        end
        use_bfm = 1'b0;
        reset_rx;

        // -------------------------------------------------------------
        // Clean link: rx_noise must stay LOW. A noise flag that is always
        // set is worse than no flag -- it trains the consumer to ignore it.
        // -------------------------------------------------------------
        $display("--- 4. rx_noise stays clean on a clean link ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        for (i = 0; i < 4; i = i + 1)
            bfm_send(8'hA5, BIT_NS * 1000);
        repeat (CPB * 3) @(posedge clk);
        use_bfm = 1'b0;
        if (rx_noise) begin
            $display("  FAIL: rx_noise set on 4 clean frames -- vote points misplaced");
            errors = errors + 1;
        end else if (rx_count != 4) begin
            $display("  FAIL: expected 4 bytes in FIFO, got %0d", rx_count);
            errors = errors + 1;
        end else begin
            $display("  ok  : 4 clean frames, rx_noise=0, rx_count=4");
        end
        reset_rx;

        // -------------------------------------------------------------
        // FIFO depth. Burst FIFO_DEPTH bytes as fast as the producer can
        // push. M3's single holding register forced the producer to wait
        // for each frame (~87us at 115200); the FIFO lets it dump and walk
        // away.
        //
        // NOTE: we do NOT assert tx_full here. The TX starts draining the
        // FIFO onto the wire the instant the first byte lands, so by the
        // time the 16th push happens several have already left -- the FIFO
        // never actually reaches 16. An earlier version of this test
        // asserted tx_full and failed on a design that was working
        // correctly. Testing FIFO fullness requires a consumer that is
        // stalled, which is what the RX-side test below does.
        // -------------------------------------------------------------
        $display("--- 5. TX FIFO burst: push %0d bytes, walk away ---", FIFO_DEPTH);
        reset_rx;
        use_bfm = 1'b0;
        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            push_tx(8'h40 + i[7:0]);
        end
        $display("  ok  : %0d bytes pushed without blocking (count=%0d in flight)",
                 FIFO_DEPTH, tx_count);
        // Let them all drain through the wire and back via loopback.
        wait (tx_active == 1'b1);
        while (tx_active || tx_count != 0) @(posedge clk);
        repeat (CPB * 4) @(posedge clk);

        n_got = 0;
        while (!rx_empty) begin
            pop_rx(got);
            expect_b = 8'h40 + n_got[7:0];
            if (got !== expect_b) begin
                $display("  FAIL: byte %0d expected 0x%02h got 0x%02h",
                         n_got, expect_b, got);
                errors = errors + 1;
            end
            n_got = n_got + 1;
        end
        if (n_got != FIFO_DEPTH) begin
            $display("  FAIL: expected %0d bytes back, got %0d", FIFO_DEPTH, n_got);
            errors = errors + 1;
        end else begin
            $display("  ok  : all %0d bytes round-tripped in order", FIFO_DEPTH);
        end
        clear_errs;

        // -------------------------------------------------------------
        // RX FIFO overrun: send DEPTH+2 with no consumer. The FIFO fills,
        // the extras are DROPPED and rx_overrun records the fact. Silent
        // loss is the thing being engineered out.
        // -------------------------------------------------------------
        $display("--- 6. RX FIFO overrun (DEPTH+2 frames, no consumer) ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        for (i = 0; i < FIFO_DEPTH + 2; i = i + 1)
            bfm_send(8'h50 + i[7:0], BIT_NS * 1000);
        repeat (CPB * 3) @(posedge clk);
        use_bfm = 1'b0;
        if (!rx_overrun) begin
            $display("  FAIL: %0d frames into a %0d-deep FIFO, no overrun flag",
                     FIFO_DEPTH + 2, FIFO_DEPTH);
            errors = errors + 1;
        end else if (rx_count != FIFO_DEPTH) begin
            $display("  FAIL: rx_count=%0d, expected %0d (full)", rx_count, FIFO_DEPTH);
            errors = errors + 1;
        end else begin
            $display("  ok  : rx_overrun set, FIFO holds %0d, extras dropped",
                     rx_count);
        end
        reset_rx;

        // -------------------------------------------------------------
        // Framing error: the bad byte must NOT enter the FIFO. M1-M3 pushed
        // it anyway with a flag beside it, forcing every consumer to
        // remember to check. Keeping it out is the safer contract.
        // -------------------------------------------------------------
        $display("--- 7. framing error keeps the bad byte OUT of the FIFO ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB * 2) @(posedge clk);
        bfm_send_badstop(8'h96, BIT_NS * 1000);
        repeat (CPB * 4) @(posedge clk);
        use_bfm = 1'b0;
        if (!frame_err) begin
            $display("  FAIL: bad stop bit did not raise frame_err");
            errors = errors + 1;
        end else if (!rx_empty) begin
            $display("  FAIL: frame_err raised but a byte still entered the FIFO");
            errors = errors + 1;
        end else begin
            $display("  ok  : frame_err set, FIFO empty -- bad byte rejected");
        end
        reset_rx;

        // -------------------------------------------------------------
        // Short glitch on an idle line -- rejected since M1. Regression.
        // -------------------------------------------------------------
        $display("--- 8. short glitch on idle line ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CPB / 8) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CPB * 12) @(posedge clk);
        use_bfm = 1'b0;
        if (!rx_empty) begin
            $display("  FAIL: short glitch produced a phantom byte");
            errors = errors + 1;
        end else begin
            $display("  ok  : rejected");
        end
        reset_rx;

        // -------------------------------------------------------------
        // Long glitch: one full bit period low.
        //
        // HONEST RESULT: still accepted. Voting NARROWS this hole; it does
        // not close it. A disturbance low across ticks 7, 8 AND 9 is
        // indistinguishable from a real start bit at every instant a
        // receiver is permitted to look. Nothing local to the start bit
        // fixes it -- it needs wider context (a minimum idle-time qualifier
        // before accepting a start edge) or a protocol-level frame check.
        //
        // Asserted as a KNOWN limitation rather than hidden, so that if a
        // later method genuinely closes it the change is visible on
        // identical stimulus.
        // -------------------------------------------------------------
        $display("--- 9. long glitch (KNOWN hole -- voting narrows, not closes) ---");
        reset_rx;
        use_bfm  = 1'b1;
        bfm_line = 1'b1;
        repeat (CPB) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CPB) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CPB * 14) @(posedge clk);
        use_bfm = 1'b0;
        if (rx_empty && !frame_err) begin
            $display("  FAIL: expected either a phantom byte or frame_err");
            errors = errors + 1;
        end else begin
            $display("  ok  : accepted as a start bit -> ferr=%b empty=%b",
                     frame_err, rx_empty);
            $display("        KNOWN LIMITATION: low at all 3 vote points ==");
            $display("        indistinguishable from a real start bit.");
        end
        reset_rx;

        // -------------------------------------------------------------
        // Baud tolerance, independent time base.
        // -------------------------------------------------------------
        $display("--- 10. baud tolerance sweep (independent BFM) ---");
        nominal_x1000 = BIT_NS * 1000;
        for (i = 0; i < 5; i = i + 1) begin : sweep
            integer ppm;
            case (i)
                0: ppm =      0;
                1: ppm =  10000;   // +1.0%
                2: ppm = -10000;   // -1.0%
                3: ppm =  20000;   // +2.0%
                4: ppm = -20000;   // -2.0%
                default: ppm = 0;
            endcase
            // MULTIPLY BEFORE DIVIDE. (nominal/1000000)*ppm truncates to
            // zero for any nominal under 1e6 and silently turns the whole
            // sweep into N repeats of the 0% case -- passing, meaningless.
            bit_x1000 = nominal_x1000 + ((nominal_x1000 * ppm) / 1000000);

            drain_rx;
            use_bfm  = 1'b1;
            bfm_line = 1'b1;
            repeat (CPB * 2) @(posedge clk);
            bfm_send(8'hA5, bit_x1000);
            repeat (CPB * 3) @(posedge clk);
            use_bfm = 1'b0;

            if (rx_empty) begin
                $display("  FAIL: %+0.2f%% -- nothing received", ppm/10000.0);
                errors = errors + 1;
            end else begin
                pop_rx(got);
                if (got !== 8'hA5 || frame_err) begin
                    $display("  FAIL: %+0.2f%% -- got 0x%02h ferr=%b",
                             ppm/10000.0, got, frame_err);
                    errors = errors + 1;
                end else begin
                    $display("  ok  : %+0.2f%%  0x%02h clean", ppm/10000.0, got);
                end
            end
        end
        reset_rx;

        // -------------------------------------------------------------
        // FIFO pointer-width regression.
        //
        // Fill, drain, fill again -- crossing the wrap boundary. Pointers
        // are $clog2(DEPTH)+1 bits; the extra MSB is the wrap bit. Size
        // them by address bits alone and full becomes indistinguishable
        // from empty (both have wr_ptr == rd_ptr): the FIFO reports empty
        // when full, the producer overwrites unread data, and the bug only
        // shows under sustained back-pressure. Most common FIFO defect
        // there is, and invisible in a lightly loaded test.
        // -------------------------------------------------------------
        $display("--- 11. FIFO wrap: fill/drain/fill across the boundary ---");
        reset_rx;
        use_bfm = 1'b0;
        begin : wrap_test
            integer pass;
            for (pass = 0; pass < 2; pass = pass + 1) begin
                for (i = 0; i < FIFO_DEPTH; i = i + 1)
                    push_tx(8'h80 + (pass*16) + i[7:0]);
                if (!tx_full) begin
                    $display("  FAIL: pass %0d -- tx_full not set at %0d bytes",
                             pass, FIFO_DEPTH);
                    errors = errors + 1;
                end
                wait (tx_active == 1'b1);
                while (tx_active || tx_count != 0) @(posedge clk);
                repeat (CPB * 4) @(posedge clk);

                n_got = 0;
                while (!rx_empty) begin
                    pop_rx(got);
                    expect_b = 8'h80 + (pass*16) + n_got[7:0];
                    if (got !== expect_b) begin
                        $display("  FAIL: pass %0d byte %0d expected 0x%02h got 0x%02h",
                                 pass, n_got, expect_b, got);
                        errors = errors + 1;
                    end
                    n_got = n_got + 1;
                end
                if (n_got != FIFO_DEPTH) begin
                    $display("  FAIL: pass %0d -- expected %0d bytes, got %0d",
                             pass, FIFO_DEPTH, n_got);
                    errors = errors + 1;
                end
                clear_errs;
            end
            $display("  ok  : two full fill/drain cycles, pointers wrapped cleanly");
        end

        // -------------------------------------------------------------
        $display("");
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d FAILURE(S) ===", errors);
        $finish;
    end

    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
