--============================================================================
-- SIMPLE UART FOR FPGA - ALL-IN-ONE FILE (single-file edition)
-- Source: jakubcabal/uart-for-fpga (MIT). Original per-unit headers retained.
--
-- This one file contains all SIX design units in dependency order, so it
-- analyzes in a single pass in Vivado/xsim/GHDL with no other design files:
--   1. UART_CLK_DIV    2. UART_DEBOUNCER    3. UART_PARITY
--   4. UART_RX         5. UART_TX           6. UART (top level)
-- Add this file as a design source and set UART as top (or instantiate it).
-- The testbench (uart_tb.vhd) stays separate on purpose: sim-only code does
-- not belong in a synthesis source. It works against this file unchanged.
--============================================================================

--============================================================================
-- UNIT 1/6 : UART_CLK_DIV - programmable divider / tick generator
-- WHAT: Counts CLK cycles and pulses DIV_MARK for one cycle at DIV_MARK_POS; CLEAR re-phases the count.
-- WHY : The single timing primitive reused three times: the ~16x oversampler in the top, and the per-bit timers inside RX and TX (RX re-phases it on each start bit - that is how bit-center sampling stays aligned).
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity UART_CLK_DIV is
    Generic (
        DIV_MAX_VAL  : integer := 16;
        DIV_MARK_POS : integer := 1
    );
    Port (
        CLK      : in  std_logic; -- system clock
        RST      : in  std_logic; -- high active synchronous reset
        -- USER INTERFACE
        CLEAR    : in  std_logic; -- clock divider counter clear
        ENABLE   : in  std_logic; -- clock divider counter enable
        DIV_MARK : out std_logic  -- output divider mark (divided clock enable)
    );
end entity;

architecture RTL of UART_CLK_DIV is

    constant CLK_DIV_WIDTH  : integer := integer(ceil(log2(real(DIV_MAX_VAL))));

    signal clk_div_cnt      : unsigned(CLK_DIV_WIDTH-1 downto 0);
    signal clk_div_cnt_mark : std_logic;

begin

    clk_div_cnt_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (CLEAR = '1') then
                clk_div_cnt <= (others => '0');
            elsif (ENABLE = '1') then
                if (clk_div_cnt = DIV_MAX_VAL-1) then
                    clk_div_cnt <= (others => '0');
                else
                    clk_div_cnt <= clk_div_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    clk_div_cnt_mark <= '1' when (clk_div_cnt = DIV_MARK_POS) else '0';

    div_mark_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            DIV_MARK <= ENABLE and clk_div_cnt_mark;
        end if;
    end process;

end architecture;

--============================================================================
-- UNIT 2/6 : UART_DEBOUNCER - glitch filter
-- WHAT: Output follows input only after it has been stable for LATENCY consecutive CLK cycles.
-- WHY : A sub-bit-period noise spike on an idle line would otherwise be taken as a start bit and desynchronize a whole frame.
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UART_DEBOUNCER is
    Generic (
        -- latency of debouncer in clock cycles, minimum value is 2,
        -- value also corresponds to the number of bits compared
        LATENCY : natural := 4
    );
    Port (
        CLK     : in  std_logic; -- system clock
        DEB_IN  : in  std_logic; -- input of signal from outside FPGA
        DEB_OUT : out std_logic  -- output of debounced (filtered) signal
    );
end entity;

architecture RTL of UART_DEBOUNCER is

    constant SHREG_DEPTH : natural := LATENCY-1;

    signal input_shreg    : std_logic_vector(SHREG_DEPTH-1 downto 0);
    signal output_reg_rst : std_logic;
    signal output_reg_set : std_logic;

begin

    -- parameterized input shift register
    input_shreg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            input_shreg <= input_shreg(SHREG_DEPTH-2 downto 0) & DEB_IN;
        end if;
    end process;

    -- output register will be reset when all compared bits are low
    output_reg_rst_p : process (DEB_IN, input_shreg)
        variable or_var : std_logic;
    begin
        or_var := DEB_IN;
        all_bits_or_l : for i in 0 to SHREG_DEPTH-1 loop
            or_var := or_var or input_shreg(i);
        end loop;
        output_reg_rst <= not or_var;
    end process;

    -- output register will be set when all compared bits are high
    output_reg_set_p : process (DEB_IN, input_shreg)
        variable and_var : std_logic;
    begin
        and_var := DEB_IN;
        all_bits_and_l : for i in 0 to SHREG_DEPTH-1 loop
            and_var := and_var and input_shreg(i);
        end loop;
        output_reg_set <= and_var;
    end process;

    -- output register
    output_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (output_reg_rst = '1') then
                DEB_OUT <= '0';
            elsif (output_reg_set = '1') then
                DEB_OUT <= '1';
            end if;
        end if;
    end process;

end architecture;

--============================================================================
-- UNIT 3/6 : UART_PARITY - combinational parity generator
-- WHAT: Reduces the 8 data bits to one parity bit; mode (even/odd/mark/space) selected by generic.
-- WHY : Shared by RX (to check) and TX (to generate) so both ends compute parity identically; costs nothing when PARITY_BIT=none because it is left unconnected.
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UART_PARITY is
    Generic (
        DATA_WIDTH  : integer := 8;
        PARITY_TYPE : string  := "none" -- legal values: "none", "even", "odd", "mark", "space"
    );
    Port (
        DATA_IN     : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        PARITY_OUT  : out std_logic
    );
end entity;

architecture RTL of UART_PARITY is

begin

    -- -------------------------------------------------------------------------
    -- PARITY BIT GENERATOR
    -- -------------------------------------------------------------------------

    even_parity_g : if (PARITY_TYPE = "even") generate
        process (DATA_IN)
        	variable parity_temp : std_logic;
        begin
            parity_temp := '0';
            for i in DATA_IN'range loop
                parity_temp := parity_temp XOR DATA_IN(i);
            end loop;
            PARITY_OUT <= parity_temp;
        end process;
    end generate;

    odd_parity_g : if (PARITY_TYPE = "odd") generate
        process (DATA_IN)
        	variable parity_temp : std_logic;
        begin
            parity_temp := '1';
            for i in DATA_IN'range loop
                parity_temp := parity_temp XOR DATA_IN(i);
            end loop;
            PARITY_OUT <= parity_temp;
        end process;
    end generate;

    mark_parity_g : if (PARITY_TYPE = "mark") generate
        PARITY_OUT <= '1';
    end generate;

    space_parity_g : if (PARITY_TYPE = "space") generate
        PARITY_OUT <= '0';
    end generate;

end architecture;

--============================================================================
-- UNIT 4/6 : UART_RX - receiver engine
-- WHAT: FSM detects the start edge, re-phases its bit timer to it, center-samples 8 data bits LSB-first, then checks parity and stop.
-- WHY : The deserializer half. Contract: DOUT_VLD pulses one cycle ONLY for a clean frame; a low stop bit pulses FRAME_ERROR instead.
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UART_RX is
    Generic (
        CLK_DIV_VAL : integer := 16;
        PARITY_BIT  : string  := "none" -- type of parity: "none", "even", "odd", "mark", "space"
    );
    Port (
        CLK          : in  std_logic; -- system clock
        RST          : in  std_logic; -- high active synchronous reset
        -- UART INTERFACE
        UART_CLK_EN  : in  std_logic; -- oversampling (16x) UART clock enable
        UART_RXD     : in  std_logic; -- serial receive data
        -- USER DATA OUTPUT INTERFACE
        DOUT         : out std_logic_vector(7 downto 0); -- output data received via UART
        DOUT_VLD     : out std_logic; -- when DOUT_VLD = 1, output data (DOUT) are valid without errors (is assert only for one clock cycle)
        FRAME_ERROR  : out std_logic; -- when FRAME_ERROR = 1, stop bit was invalid (is assert only for one clock cycle)
        PARITY_ERROR : out std_logic  -- when PARITY_ERROR = 1, parity bit was invalid (is assert only for one clock cycle)
    );
end entity;

architecture RTL of UART_RX is

    signal rx_clk_en          : std_logic;
    signal rx_data            : std_logic_vector(7 downto 0);
    signal rx_bit_count       : unsigned(2 downto 0);
    signal rx_parity_bit      : std_logic;
    signal rx_parity_error    : std_logic;
    signal rx_parity_check_en : std_logic;
    signal rx_done            : std_logic;
    signal fsm_idle           : std_logic;
    signal fsm_databits       : std_logic;
    signal fsm_stopbit        : std_logic;

    type state is (idle, startbit, databits, paritybit, stopbit);
    signal fsm_pstate : state;
    signal fsm_nstate : state;

begin

    -- -------------------------------------------------------------------------
    -- UART RECEIVER CLOCK DIVIDER AND CLOCK ENABLE FLAG
    -- -------------------------------------------------------------------------

    rx_clk_divider_i : entity work.UART_CLK_DIV
    generic map(
        DIV_MAX_VAL  => CLK_DIV_VAL,
        DIV_MARK_POS => 3
    )
    port map (
        CLK      => CLK,
        RST      => RST,
        CLEAR    => fsm_idle,
        ENABLE   => UART_CLK_EN,
        DIV_MARK => rx_clk_en
    );

    -- -------------------------------------------------------------------------
    -- UART RECEIVER BIT COUNTER
    -- -------------------------------------------------------------------------

    uart_rx_bit_counter_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                rx_bit_count <= (others => '0');
            elsif (rx_clk_en = '1' AND fsm_databits = '1') then
                if (rx_bit_count = "111") then
                    rx_bit_count <= (others => '0');
                else
                    rx_bit_count <= rx_bit_count + 1;
                end if;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- UART RECEIVER DATA SHIFT REGISTER
    -- -------------------------------------------------------------------------

    uart_rx_data_shift_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (rx_clk_en = '1' AND fsm_databits = '1') then
                rx_data <= UART_RXD & rx_data(7 downto 1);
            end if;
        end if;
    end process;

    DOUT <= rx_data;

    -- -------------------------------------------------------------------------
    -- UART RECEIVER PARITY GENERATOR AND CHECK
    -- -------------------------------------------------------------------------

    uart_rx_parity_g : if (PARITY_BIT /= "none") generate
        uart_rx_parity_gen_i: entity work.UART_PARITY
        generic map (
            DATA_WIDTH  => 8,
            PARITY_TYPE => PARITY_BIT
        )
        port map (
            DATA_IN     => rx_data,
            PARITY_OUT  => rx_parity_bit
        );

        uart_rx_parity_check_reg_p : process (CLK)
        begin
            if (rising_edge(CLK)) then
                if (rx_clk_en = '1') then
                    rx_parity_error <= rx_parity_bit XOR UART_RXD;
                end if;
            end if;
        end process;
    end generate;

    uart_rx_noparity_g : if (PARITY_BIT = "none") generate
        rx_parity_error <= '0';
    end generate;

    -- -------------------------------------------------------------------------
    -- UART RECEIVER OUTPUT REGISTER
    -- -------------------------------------------------------------------------

    rx_done <= rx_clk_en and fsm_stopbit;

    uart_rx_output_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                DOUT_VLD     <= '0';
                FRAME_ERROR  <= '0';
                PARITY_ERROR <= '0';
            else
                DOUT_VLD     <= rx_done and not rx_parity_error and UART_RXD;
                FRAME_ERROR  <= rx_done and not UART_RXD;
                PARITY_ERROR <= rx_done and rx_parity_error;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- UART RECEIVER FSM
    -- -------------------------------------------------------------------------

    -- PRESENT STATE REGISTER
    process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                fsm_pstate <= idle;
            else
                fsm_pstate <= fsm_nstate;
            end if;
        end if;
    end process;

    -- NEXT STATE AND OUTPUTS LOGIC
    process (fsm_pstate, UART_RXD, rx_clk_en, rx_bit_count)
    begin
        case fsm_pstate is

            when idle =>
                fsm_stopbit  <= '0';
                fsm_databits <= '0';
                fsm_idle     <= '1';

                if (UART_RXD = '0') then
                    fsm_nstate <= startbit;
                else
                    fsm_nstate <= idle;
                end if;

            when startbit =>
                fsm_stopbit  <= '0';
                fsm_databits <= '0';
                fsm_idle     <= '0';

                if (rx_clk_en = '1') then
                    fsm_nstate <= databits;
                else
                    fsm_nstate <= startbit;
                end if;

            when databits =>
                fsm_stopbit  <= '0';
                fsm_databits <= '1';
                fsm_idle     <= '0';

                if ((rx_clk_en = '1') AND (rx_bit_count = "111")) then
                    if (PARITY_BIT = "none") then
                        fsm_nstate <= stopbit;
                    else
                        fsm_nstate <= paritybit;
                    end if ;
                else
                    fsm_nstate <= databits;
                end if;

            when paritybit =>
                fsm_stopbit  <= '0';
                fsm_databits <= '0';
                fsm_idle     <= '0';

                if (rx_clk_en = '1') then
                    fsm_nstate <= stopbit;
                else
                    fsm_nstate <= paritybit;
                end if;

            when stopbit =>
                fsm_stopbit  <= '1';
                fsm_databits <= '0';
                fsm_idle     <= '0';

                if (rx_clk_en = '1') then
                    fsm_nstate <= idle;
                else
                    fsm_nstate <= stopbit;
                end if;

            when others =>
                fsm_stopbit  <= '0';
                fsm_databits <= '0';
                fsm_idle     <= '0';
                fsm_nstate   <= idle;

        end case;
    end process;

end architecture;

--============================================================================
-- UNIT 5/6 : UART_TX - transmitter engine
-- WHAT: Latches DIN on the DIN_VLD-and-DIN_RDY handshake, then shifts start + 8 data bits LSB-first + optional parity + stop.
-- WHY : The serializer half. No FIFO: DIN_RDY low back-pressures the producer until the frame completes.
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UART_TX is
    Generic (
        CLK_DIV_VAL : integer := 16;
        PARITY_BIT  : string  := "none" -- type of parity: "none", "even", "odd", "mark", "space"
    );
    Port (
        CLK         : in  std_logic; -- system clock
        RST         : in  std_logic; -- high active synchronous reset
        -- UART INTERFACE
        UART_CLK_EN : in  std_logic; -- oversampling (16x) UART clock enable
        UART_TXD    : out std_logic; -- serial transmit data
        -- USER DATA INPUT INTERFACE
        DIN         : in  std_logic_vector(7 downto 0); -- input data to be transmitted over UART
        DIN_VLD     : in  std_logic; -- when DIN_VLD = 1, input data (DIN) are valid
        DIN_RDY     : out std_logic  -- when DIN_RDY = 1, transmitter is ready and valid input data will be accepted for transmiting
    );
end entity;

architecture RTL of UART_TX is

    signal tx_clk_en       : std_logic;
    signal tx_clk_div_clr  : std_logic;
    signal tx_data         : std_logic_vector(7 downto 0);
    signal tx_bit_count    : unsigned(2 downto 0);
    signal tx_bit_count_en : std_logic;
    signal tx_ready        : std_logic;
    signal tx_parity_bit   : std_logic;
    signal tx_data_out_sel : std_logic_vector(1 downto 0);

    type state is (idle, txsync, startbit, databits, paritybit, stopbit);
    signal tx_pstate : state;
    signal tx_nstate : state;

begin

    DIN_RDY <= tx_ready;

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER CLOCK DIVIDER AND CLOCK ENABLE FLAG
    -- -------------------------------------------------------------------------

    tx_clk_divider_i : entity work.UART_CLK_DIV
    generic map(
        DIV_MAX_VAL  => CLK_DIV_VAL,
        DIV_MARK_POS => 1
    )
    port map (
        CLK      => CLK,
        RST      => RST,
        CLEAR    => tx_clk_div_clr,
        ENABLE   => UART_CLK_EN,
        DIV_MARK => tx_clk_en
    );

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER INPUT DATA REGISTER
    -- -------------------------------------------------------------------------

    uart_tx_input_data_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (DIN_VLD = '1' AND tx_ready = '1') then
                tx_data <= DIN;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER BIT COUNTER
    -- -------------------------------------------------------------------------

    uart_tx_bit_counter_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                tx_bit_count <= (others => '0');
            elsif (tx_bit_count_en = '1' AND tx_clk_en = '1') then
                if (tx_bit_count = "111") then
                    tx_bit_count <= (others => '0');
                else
                    tx_bit_count <= tx_bit_count + 1;
                end if;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER PARITY GENERATOR
    -- -------------------------------------------------------------------------

    uart_tx_parity_g : if (PARITY_BIT /= "none") generate
        uart_tx_parity_gen_i: entity work.UART_PARITY
        generic map (
            DATA_WIDTH  => 8,
            PARITY_TYPE => PARITY_BIT
        )
        port map (
            DATA_IN     => tx_data,
            PARITY_OUT  => tx_parity_bit
        );
    end generate;

    uart_tx_noparity_g : if (PARITY_BIT = "none") generate
        tx_parity_bit <= '0';
    end generate;

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER OUTPUT DATA REGISTER
    -- -------------------------------------------------------------------------

    uart_tx_output_data_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                UART_TXD <= '1';
            else
                case tx_data_out_sel is
                    when "01" => -- START BIT
                        UART_TXD <= '0';
                    when "10" => -- DATA BITS
                        UART_TXD <= tx_data(to_integer(tx_bit_count));
                    when "11" => -- PARITY BIT
                        UART_TXD <= tx_parity_bit;
                    when others => -- STOP BIT OR IDLE
                        UART_TXD <= '1';
                end case;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER FSM
    -- -------------------------------------------------------------------------

    -- PRESENT STATE REGISTER
    process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                tx_pstate <= idle;
            else
                tx_pstate <= tx_nstate;
            end if;
        end if;
    end process;

    -- NEXT STATE AND OUTPUTS LOGIC
    process (tx_pstate, DIN_VLD, tx_clk_en, tx_bit_count)
    begin

        case tx_pstate is

            when idle =>
                tx_ready <= '1';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '1';

                if (DIN_VLD = '1') then
                    tx_nstate <= txsync;
                else
                    tx_nstate <= idle;
                end if;

            when txsync =>
                tx_ready <= '0';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '0';

                if (tx_clk_en = '1') then
                    tx_nstate <= startbit;
                else
                    tx_nstate <= txsync;
                end if;

            when startbit =>
                tx_ready <= '0';
                tx_data_out_sel <= "01";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '0';

                if (tx_clk_en = '1') then
                    tx_nstate <= databits;
                else
                    tx_nstate <= startbit;
                end if;

            when databits =>
                tx_ready <= '0';
                tx_data_out_sel <= "10";
                tx_bit_count_en <= '1';
                tx_clk_div_clr <= '0';

                if ((tx_clk_en = '1') AND (tx_bit_count = "111")) then
                    if (PARITY_BIT = "none") then
                        tx_nstate <= stopbit;
                    else
                        tx_nstate <= paritybit;
                    end if ;
                else
                    tx_nstate <= databits;
                end if;

            when paritybit =>
                tx_ready <= '0';
                tx_data_out_sel <= "11";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '0';

                if (tx_clk_en = '1') then
                    tx_nstate <= stopbit;
                else
                    tx_nstate <= paritybit;
                end if;

            when stopbit =>
                tx_ready <= '1';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '0';

                if (DIN_VLD = '1') then
                    tx_nstate <= txsync;
                elsif (tx_clk_en = '1') then
                    tx_nstate <= idle;
                else
                    tx_nstate <= stopbit;
                end if;

            when others =>
                tx_ready <= '0';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_div_clr <= '0';
                tx_nstate <= idle;

        end case;
    end process;

end architecture;

--============================================================================
-- UNIT 6/6 : UART - top level (fully commented edition)
-- WHAT: Wires divider + synchronizer + debouncer + RX + TX into one entity with a byte-stream user interface.
-- WHY : This is the unit you instantiate; everything above is internal plumbing.
--============================================================================

--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/uart-for-fpga
--------------------------------------------------------------------------------
-- COMMENTED EDITION (functionally identical to upstream master).
--
-- In this all-in-one file the five sub-entities this top level instantiates
-- (UART_CLK_DIV, UART_DEBOUNCER, UART_PARITY, UART_RX, UART_TX) are defined
-- ABOVE, in dependency order, so the file is fully self-contained.
--
-- Frame format is fixed: 1 start bit, 8 data bits (LSB first), optional
-- parity bit, 1 stop bit. Everything else is set through the generics.
--------------------------------------------------------------------------------

-- ==== LIBRARY / PACKAGE IMPORTS ==============================================
-- WHAT: Make the IEEE standard packages visible to this file.
-- WHY : VHDL has no built-in logic types; std_logic and vector arithmetic
--       come from these packages.
library IEEE;                      -- the IEEE standard library container
use IEEE.STD_LOGIC_1164.ALL;       -- std_logic / std_logic_vector types and operators
use IEEE.NUMERIC_STD.ALL;          -- unsigned/signed arithmetic (used in sub-modules)
use IEEE.MATH_REAL.ALL;            -- real-number math for the divider constants below

-- SIMPLE UART FOR FPGA
-- ====================
-- UART FOR FPGA REQUIRES: 1 START BIT, 8 DATA BITS, 1 STOP BIT!!!
-- OTHER PARAMETERS CAN BE SET USING GENERICS.

-- ==== ENTITY DECLARATION =====================================================
-- WHAT: The external interface (generics + ports) of the whole UART.
-- WHY : This is the only part the rest of your SoC sees. The user side is a
--       simple valid/ready byte stream in each direction, so the UART drops
--       into any design without knowing anything about serial timing.
entity UART is
    Generic (
        CLK_FREQ      : integer := 50e6;   -- system clock frequency in Hz; MUST match the real clock driving CLK or the baud rate will be wrong and the link produces garbage
        BAUD_RATE     : integer := 115200; -- baud rate value; keep CLK_FREQ/BAUD_RATE >= ~16 so the 16x oversampler still has whole clock ticks to work with
        PARITY_BIT    : string  := "none"; -- type of parity: "none", "even", "odd", "mark", "space"; adds a 9th bit to the frame when not "none"
        USE_DEBOUNCER : boolean := True    -- enable/disable debouncer; True filters sub-4-cycle glitches on RXD (keep True on real cables, False saves a few LUTs in clean simulation-only setups)
    );
    Port (
        -- CLOCK AND RESET
        CLK          : in  std_logic; -- system clock; every flop in the design runs on this single domain
        RST          : in  std_logic; -- high active synchronous reset; hold >= 2 cycles at startup
        -- UART INTERFACE (the physical serial pins)
        UART_TXD     : out std_logic; -- serial transmit data; idles high per RS-232 convention
        UART_RXD     : in  std_logic; -- serial receive data; ASYNCHRONOUS input, synchronized internally before use
        -- USER DATA INPUT INTERFACE (parallel side, TX direction)
        DIN          : in  std_logic_vector(7 downto 0); -- input data to be transmitted over UART
        DIN_VLD      : in  std_logic; -- when DIN_VLD = 1, input data (DIN) are valid; byte is accepted on the clock edge where DIN_VLD and DIN_RDY are both 1
        DIN_RDY      : out std_logic; -- when DIN_RDY = 1, transmitter is ready and valid input data will be accepted for transmiting; there is no FIFO, so wait for this before each byte
        -- USER DATA OUTPUT INTERFACE (parallel side, RX direction)
        DOUT         : out std_logic_vector(7 downto 0); -- output data received via UART; stable until the next byte finishes
        DOUT_VLD     : out std_logic; -- when DOUT_VLD = 1, output data (DOUT) are valid (is assert only for one clock cycle); only fires on an error-free frame, so capture DOUT on this pulse
        FRAME_ERROR  : out std_logic; -- when FRAME_ERROR = 1, stop bit was invalid (is assert only for one clock cycle); fires INSTEAD of DOUT_VLD for that frame
        PARITY_ERROR : out std_logic  -- when PARITY_ERROR = 1, parity bit was invalid (is assert only for one clock cycle); can only occur when PARITY_BIT /= "none"
    );
end entity;

-- ==== ARCHITECTURE ===========================================================
-- WHAT: The implementation — two compile-time constants, five internal wires,
--       and five functional units (divider, synchronizer, debouncer, RX, TX).
architecture RTL of UART is

    -- ==== BAUD-RATE DIVIDER CONSTANTS ========================================
    -- WHAT: Two divider values computed at elaboration time from the generics.
    -- WHY : The receiver oversamples the line at ~16x the baud rate to find
    --       bit centers. OS_CLK_DIV_VAL turns CLK into that ~16x tick;
    --       UART_CLK_DIV_VAL then counts 16 (nominally) of those ticks to make
    --       one bit period. Doing this with integer(real(...)) rounds to the
    --       NEAREST divider, halving the worst-case baud error vs truncation.
    --       At 50 MHz / 115200: 50e6/(16*115200)=27.13 -> 27, then
    --       50e6/(27*115200)=16.08 -> 16, giving an effective 115740 baud
    --       (+0.47% — well inside the ~2% UART tolerance).
    constant OS_CLK_DIV_VAL   : integer := integer(real(CLK_FREQ)/real(16*BAUD_RATE));         -- CLK cycles per oversampling tick
    constant UART_CLK_DIV_VAL : integer := integer(real(CLK_FREQ)/real(OS_CLK_DIV_VAL*BAUD_RATE)); -- oversampling ticks per bit (nominally 16)

    -- ==== INTERNAL SIGNALS ===================================================
    -- WHY the "_n" (inverted) naming: the RXD path is deliberately carried
    -- INVERTED through the synchronizer/debouncer flops. FPGA registers wake
    -- up as '0' after configuration, and inverted-idle ('0' = line idle-high)
    -- means the receiver sees a clean idle line at power-up instead of a
    -- phantom start bit.
    signal os_clk_en            : std_logic; -- single-cycle enable pulse at ~16x baud rate; distributed to RX and TX instead of a derived clock (keeps the design single-clock — no CDC, clean timing analysis)
    signal uart_rxd_meta_n      : std_logic; -- first synchronizer flop; may go metastable, nothing is allowed to read it except the second flop
    signal uart_rxd_synced_n    : std_logic; -- second synchronizer flop; safe, stable, still inverted
    signal uart_rxd_debounced_n : std_logic; -- after glitch filtering, still inverted
    signal uart_rxd_debounced   : std_logic; -- final clean, true-polarity RXD used by the receiver

begin

    -- -------------------------------------------------------------------------
    --  UART OVERSAMPLING (~16X) CLOCK DIVIDER AND CLOCK ENABLE FLAG
    -- -------------------------------------------------------------------------
    -- WHAT: Free-running divider producing os_clk_en, a one-CLK-cycle pulse
    --       every OS_CLK_DIV_VAL cycles (~16x the baud rate).
    -- WHY : It is the timebase for everything serial. Generating an ENABLE
    --       instead of a divided clock is the correct FPGA idiom: all logic
    --       stays on CLK, so there is no clock-domain crossing and no manual
    --       constraint work. CLEAR is tied to RST so the phase is known
    --       after reset.

    os_clk_divider_i : entity work.UART_CLK_DIV -- instantiate the shared divider component (file: comp/uart_clk_div.vhd)
    generic map(
        DIV_MAX_VAL  => OS_CLK_DIV_VAL,   -- count this many CLK cycles per tick
        DIV_MARK_POS => OS_CLK_DIV_VAL-1  -- emit the tick on the last count value
    )
    port map (
        CLK      => CLK,       -- same system clock as everything else
        RST      => RST,       -- synchronous reset
        CLEAR    => RST,       -- restart the count on reset so tick phase is deterministic
        ENABLE   => '1',       -- this divider never pauses (RX/TX gate themselves)
        DIV_MARK => os_clk_en  -- the ~16x-baud enable pulse consumed by RX and TX
    );

    -- -------------------------------------------------------------------------
    --  UART RXD CROSS DOMAIN CROSSING
    -- -------------------------------------------------------------------------
    -- WHAT: Classic two-flip-flop synchronizer on the RXD pin (stored inverted,
    --       see signal comments above).
    -- WHY : REQUIRED, not optional. UART_RXD comes from another device with an
    --       unrelated clock; sampling it directly would feed metastable values
    --       into the receiver state machine and corrupt it in rare,
    --       unreproducible ways. Two back-to-back flops reduce that
    --       probability to negligible. (For synthesis QoR, an ASYNC_REG
    --       property on these two flops in the XDC keeps them packed together.)

    uart_rxd_cdc_reg_p : process (CLK)          -- sensitive to the clock only: a synchronous process
    begin
        if (rising_edge(CLK)) then              -- act on the rising edge of the system clock
            uart_rxd_meta_n   <= not UART_RXD;        -- flop 1: capture the async pin (inverted); THIS one may go metastable
            uart_rxd_synced_n <= uart_rxd_meta_n;     -- flop 2: re-sample flop 1; output is now safe to use
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  UART RXD DEBAUNCER
    -- -------------------------------------------------------------------------
    -- WHAT: Optional glitch filter — the debouncer only passes a new RXD value
    --       once it has been stable for LATENCY (4) consecutive CLK cycles.
    -- WHY : Real cables pick up noise spikes shorter than a bit period; without
    --       filtering, a single 1-cycle glitch during idle can be mistaken for
    --       a start bit and desynchronize a whole frame. A generate pair is
    --       used so the False case costs zero hardware.

    use_debouncer_g : if (USE_DEBOUNCER = True) generate  -- elaborate this branch only when the generic asks for it
        debouncer_i : entity work.UART_DEBOUNCER          -- instantiate the filter (file: comp/uart_debouncer.vhd)
        generic map(
            LATENCY => 4                                  -- input must be stable 4 CLK cycles to propagate (80 ns at 50 MHz — far below a bit period, so no timing impact)
        )
        port map (
            CLK     => CLK,                    -- same single clock domain
            DEB_IN  => uart_rxd_synced_n,      -- input: synchronized (inverted) RXD
            DEB_OUT => uart_rxd_debounced_n    -- output: filtered (still inverted) RXD
        );
    end generate;

    not_use_debouncer_g : if (USE_DEBOUNCER = False) generate -- the bypass branch
        uart_rxd_debounced_n <= uart_rxd_synced_n;            -- no filter: just wire through
    end generate;

    uart_rxd_debounced <= not uart_rxd_debounced_n; -- undo the storage inversion: from here on, '1' = idle, '0' = start bit, as on the wire

    -- -------------------------------------------------------------------------
    --  UART RECEIVER
    -- -------------------------------------------------------------------------
    -- WHAT: The RX engine: detects the start-bit edge, re-synchronizes its bit
    --       counter to it, samples each data bit at its center using os_clk_en
    --       ticks, checks parity and the stop bit, and presents the byte.
    -- WHY : This is the deserializer half of the UART's purpose. Center
    --       sampling is what gives the receiver tolerance to baud-rate
    --       mismatch between the two ends. Note the output contract it
    --       implements: DOUT_VLD pulses only for a clean frame; a bad stop
    --       bit pulses FRAME_ERROR instead.

    uart_rx_i: entity work.UART_RX      -- instantiate the receiver (file: comp/uart_rx.vhd)
    generic map (
        CLK_DIV_VAL => UART_CLK_DIV_VAL, -- oversampling ticks per bit (how many os_clk_en pulses make one bit)
        PARITY_BIT  => PARITY_BIT        -- pass the parity mode straight through
    )
    port map (
        CLK          => CLK,                 -- system clock
        RST          => RST,                 -- synchronous reset
        -- UART INTERFACE
        UART_CLK_EN  => os_clk_en,           -- the shared ~16x-baud timebase
        UART_RXD     => uart_rxd_debounced,  -- the cleaned, true-polarity serial input
        -- USER DATA OUTPUT INTERFACE
        DOUT         => DOUT,                -- received byte to the user
        DOUT_VLD     => DOUT_VLD,            -- one-cycle "byte is good" strobe
        FRAME_ERROR  => FRAME_ERROR,         -- one-cycle "stop bit was low" strobe
        PARITY_ERROR => PARITY_ERROR         -- one-cycle "parity mismatch" strobe
    );

    -- -------------------------------------------------------------------------
    --  UART TRANSMITTER
    -- -------------------------------------------------------------------------
    -- WHAT: The TX engine: latches DIN on a DIN_VLD & DIN_RDY handshake, then
    --       shifts out start bit, 8 data bits LSB-first, optional parity, and
    --       the stop bit, one bit per UART_CLK_DIV_VAL os_clk_en ticks.
    -- WHY : The serializer half. The valid/ready handshake is what lets any
    --       host logic drive it safely: there is no FIFO, so DIN_RDY low
    --       simply back-pressures the producer until the frame finishes.

    uart_tx_i: entity work.UART_TX      -- instantiate the transmitter (file: comp/uart_tx.vhd)
    generic map (
        CLK_DIV_VAL => UART_CLK_DIV_VAL, -- same ticks-per-bit as the receiver, so both ends of this entity agree on the baud rate
        PARITY_BIT  => PARITY_BIT        -- same parity mode as the receiver
    )
    port map (
        CLK         => CLK,        -- system clock
        RST         => RST,        -- synchronous reset
        -- UART INTERFACE
        UART_CLK_EN => os_clk_en,  -- the shared ~16x-baud timebase
        UART_TXD    => UART_TXD,   -- serial output pin (idles high)
        -- USER DATA INPUT INTERFACE
        DIN         => DIN,        -- byte from the user
        DIN_VLD     => DIN_VLD,    -- user asserts: "DIN is valid now"
        DIN_RDY     => DIN_RDY     -- core asserts: "I will accept a byte this cycle"
    );

end architecture; -- RTL of UART
