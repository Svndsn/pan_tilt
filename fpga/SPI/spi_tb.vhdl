library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.finish;

entity spi_tb is
end spi_tb;

architecture behave of spi_tb is
  constant data_width : integer := 8;
  signal clk       :  std_logic := '0';
  signal rst       :  std_logic := '0';
  signal cs        :  std_logic := '1';
  signal sclk      :  std_logic := '0';
  signal mosi      :  std_logic := '0';
  signal miso      :  std_logic := '0';
  signal busy      :  std_logic := '0';
  signal rx_data   :  std_logic_vector(data_width-1 downto 0) := (others => '0');
  signal tx_data   :  std_logic_vector(data_width-1 downto 0) := (others => '0');
begin
  spi : entity work.spi
    generic map (
      data_width => data_width
    )
    port map (
    i_clk => clk,
    i_rst => rst,
    i_cs => cs,
    i_sclk => sclk,
    i_mosi => mosi,
    --o_miso => miso,
    o_busy => busy,
    o_rx_data => rx_data
    --i_tx_data => tx_data
    );

  clk <= not clk after 1 ns;
  sclk <= not sclk after 6 ns;

  process is
  begin
    wait for 3 ns;
    cs <= '0';
    mosi <= '1';
    wait for 12 ns;
    mosi <= '0';
    wait for 12 ns;
    mosi <= '1';
    wait for 12 ns;
    mosi <= '0';
    wait for 12 ns;
    mosi <= '1';
    wait for 12 ns;
    mosi <= '1';
    wait for 12 ns;
    mosi <= '0';
    wait for 12 ns;
    mosi <= '1';
    wait for 12 ns;
    cs <= '1';
    finish;
  end process;
end behave;
