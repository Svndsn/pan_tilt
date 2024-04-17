library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity spi_test is
  generic ( data_width : integer := 8 );
  port (clk       : in  std_logic;
        ck_cs        : in  std_logic;
        ck_sclk      : in  std_logic;
        ck_mosi      : in  std_logic;
        led0      : out std_logic
      );
end spi_test;

architecture behavioral of spi_test is
    signal rst              :  std_logic := '0';
    signal busy             :  std_logic := '0';
    signal rx_data          :  std_logic_vector(data_width-1 downto 0) := (others => '0');
begin
  spi : entity work.spi
    generic map (
      data_width => data_width
    )
    port map (
      i_clk => clk,
      i_rst => rst,
      i_cs => ck_cs,
      i_sclk => ck_sclk,
      i_mosi => ck_mosi,
      o_busy => busy,
      o_rx_data => rx_data
    );
  process(clk)
  begin
    if rx_data = "01010101" then
      led0 <='1';
    else
      led0 <= '0';
    end if;
  end process;
end behavioral;

