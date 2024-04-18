library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity variable_counter is
  generic (
    max_count_value : natural := 9; -- Default maximum count value
    n_bits : integer := 4  -- Default width is 4 bits
  );
  port (
    Clk : in STD_LOGIC;
    rst : in STD_LOGIC := '0';
    en  : in STD_LOGIC := '1';
    Q   : out STD_LOGIC_VECTOR(n_bits-1 downto 0) := (others => '0');
    overflow : out STD_LOGIC := '0'
  );
end entity variable_counter;

architecture Behavioral of variable_counter is

  signal temp : unsigned(n_bits - 1 downto 0) := (others => '0');
  signal temp_overflow : std_logic := '0';
  
begin
  process(Clk)
  begin
    if rising_edge(Clk) then
      if rst = '1' then
        temp <= (others => '0');
      elsif en = '1' then
        if temp < max_count_value then
          temp <= temp + 1;
          temp_overflow <= '0';
        else
          temp <= (others => '0');
          temp_overflow <= '1';
        end if;
        Q <= std_logic_vector(temp);
        overflow <= temp_overflow;
      end if;
    end if;
  end process;
end architecture Behavioral;
