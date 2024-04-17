library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity spi is
  generic ( data_width : integer := 8 );
  port (i_clk       : in  std_logic;
        i_rst       : in  std_logic;
        i_cs        : in  std_logic;
        i_sclk      : in  std_logic;
        i_mosi      : in  std_logic;
        --o_miso      : out std_logic;
        o_busy      : out std_logic;
        o_rx_data   : out  std_logic_vector(data_width-1 downto 0)
        --i_tx_data   : in std_logic_vector(data_width-1 downto 0)
      );
end spi;

architecture behavioral of spi is
    signal rx_data_internal : std_logic_vector(data_width-1 downto 0) := (others => '0');
    --signal tx_data_internal : std_logic_vector(data_width-1 downto 0) := (others => '0');
    signal sclk_int : std_logic := '0';
begin
    -- spi slave process
    spi_slave_process : process(i_clk, i_rst)
    begin
        if i_rst = '1' then
            o_rx_data <= (others => '0');
            sclk_int <= '0';
            o_busy <= '1';
        elsif rising_edge(i_clk) then
          if i_cs = '0' then
            o_busy <= '1';
            sclk_int <= i_sclk;
          else
            sclk_int <= '0';
            o_busy <= '0';
          end if;
        end if;
        o_rx_data <= rx_data_internal;
    end process spi_slave_process;

    spi_slave_clocked : process(sclk_int)
    begin
      if sclk_int = '1' then
          rx_data_internal <= rx_data_internal(data_width-2 downto 0) & i_mosi;
      end if;
    end process spi_slave_clocked;
end behavioral;
