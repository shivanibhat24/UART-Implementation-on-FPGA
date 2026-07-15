library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity uart_rx_tb is
end uart_rx_tb;
architecture sim of uart_rx_tb is
    constant CLK_FREQ : integer := 50000000;
    constant BAUD_RATE : integer := 115200;
    -- Clock signal used to drive the receiver
    signal clk : std_logic := '0';
    -- Reset signal to return receiver to original state
    signal rst : std_logic := '0';
    -- Serial input signal connected to receiver
    signal rx_pin : std_logic := '1';
    -- Received parallel data from UART receiver
    signal rx_data : std_logic_vector(7 downto 0);
    -- Signal indicating received byte is ready
    signal data_valid : std_logic;
    -- Clock period calculation
    -- 50 MHz clock = 20 ns period
    constant CLK_PERIOD : time := 20 ns;
    -- UART bit period calculation
    -- 1 / 115200 = 8.68 us
    constant BIT_PERIOD : time := 1 sec / BAUD_RATE;
begin
    -- Clock generation process
    -- This continuously generates the FPGA clock
    clk_process : process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2;
        clk <= '1';
        wait for CLK_PERIOD/2;
    end process;
    -- Instantiating the UART receiver module
    -- This is the actual hardware block being tested
    uart_rx_inst : entity work.uart_rx
        generic map (
            CLK_FREQ  => CLK_FREQ,
            BAUD_RATE => BAUD_RATE
        )
        port map (
            clk        => clk,
            rst        => rst,
            rx_pin     => rx_pin,
            rx_data    => rx_data,
            data_valid => data_valid

        );
    -- UART transmission process
    -- This manually creates a UART frame on rx_pin
    stimulus_process : process
    begin
        -- Applying reset to initialize receiver
        rst <= '1';
        wait for 100 ns;
        -- Removing reset
        rst <= '0';
        wait for 100 ns;
        -- Sending byte 0x41
        -- Binary representation:
        -- 01000001
        --
        -- UART sends bits LSB first:
        -- Start bit = 0
        -- Data bits = 1 0 0 0 0 0 1 0
        -- Stop bit  = 1
        -- Sending start bit
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 0
        rx_pin <= '1';
        wait for BIT_PERIOD;
        -- Sending bit 1
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 2
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 3
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 4
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 5
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending bit 6
        rx_pin <= '1';
        wait for BIT_PERIOD;
        -- Sending bit 7
        rx_pin <= '0';
        wait for BIT_PERIOD;
        -- Sending stop bit
        rx_pin <= '1';
        wait for BIT_PERIOD;
        -- Wait for receiver to indicate data is ready
        wait until data_valid = '1';
        -- Check received byte
        assert rx_data = "01000001"
        report "UART RX received incorrect data"
        severity error;
        -- Indicate successful test completion
        assert false
        report "UART RX test completed successfully"
        severity failure;
    end process;
  end sim;
