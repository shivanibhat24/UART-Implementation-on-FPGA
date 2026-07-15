library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity uart_rx is
    generic (
        CLK_FREQ  : integer := 50000000; -- Hz
        BAUD_RATE : integer := 115200
    );
    port (

        clk : in std_logic;
        rst : in std_logic;
        rx_pin : in std_logic;
        rx_data : out std_logic_vector(7 downto 0);
        data_valid : out std_logic
    );
end uart_rx;
architecture rtl of uart_rx is
    constant BAUD_DIV : integer := CLK_FREQ / BAUD_RATE;
    signal baud_cnt : integer range 0 to BAUD_DIV-1 := 0;
    signal bit_cnt : integer range 0 to 7 := 0;
    signal shift_reg : std_logic_vector(7 downto 0) := (others => '0');
    signal receiving : std_logic := '0';
begin
    process(clk)
    begin
        if rising_edge(clk) then
            -- Check if reset is active
            if rst = '1' then
                -- Receiver becomes inactive
                receiving <= '0';
                -- Reset baud counter
                baud_cnt <= 0;
                -- Reset bit counter
                bit_cnt <= 0;
                -- Clear shift register
                shift_reg <= (others => '0');
                -- No data is available after reset
                data_valid <= '0';
                rx_data <= (others => '0');
            else
                -- Data valid is only high for one clock cycle
                data_valid <= '0';
                -- Check if receiver is currently idle
                if receiving = '0' then
                    -- UART line normally stays high
                    -- A falling edge indicates the start bit
                    if rx_pin = '0' then
                        -- Receiver is now active
                        receiving <= '1';
                        -- Wait half a bit period
                        -- This places sampling point in the middle of the bit
                        baud_cnt <= BAUD_DIV/2;
                        -- Start receiving from the first data bit
                        bit_cnt <= 0;
                    end if;
                else
                    -- Counting clock cycles until next bit arrives
                    if baud_cnt = BAUD_DIV-1 then
                        -- Restart baud counter
                        baud_cnt <= 0;
                        -- Shift received serial bit into register
                        -- UART transmits LSB first
                        shift_reg <= rx_pin & shift_reg(7 downto 1);
                        -- Check if all 8 data bits are received
                        if bit_cnt = 7 then
                            -- Copy received byte to output
                            rx_data <= shift_reg;
                            -- Indicate that new data is available
                            data_valid <= '1';
                            -- Return receiver to idle state
                            receiving <= '0';
                        else
                            -- Move to next data bit
                            bit_cnt <= bit_cnt + 1;
                        end if;
                    else
                        -- Increase clock counter
                        baud_cnt <= baud_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;
end rtl;
