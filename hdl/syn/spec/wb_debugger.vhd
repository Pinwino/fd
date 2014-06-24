----------------------------------------------------------------------------------
-- Company: FREE INDEPENDENT ALLIANCE OF MAKERS
-- Engineer: Jose Jimenez Montañez
-- 
-- Create Date:    21:56:34 06/08/2014 
-- Design Name: 
-- Module Name:    wb_debugger - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

use work.gn4124_core_pkg.all;
use work.gencores_pkg.all;
use work.wrcore_pkg.all;
use work.wr_fabric_pkg.all;
use work.wishbone_pkg.all;
use work.fine_delay_pkg.all;
--use work.etherbone_pkg.all;
use work.wr_xilinx_pkg.all;
use work.genram_pkg.all;
use work.wb_irq_pkg.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity wb_debugger is
	generic(	g_dbg_dpram_size	: integer := 40960/4;
				g_dbg_init_file: string;
				g_reset_vector	:  t_wishbone_address := x"00000000";
				g_msi_queues 	: natural := 1;
				g_profile		: string := "medium_icache_debug";
				g_timers			: integer := 1);
    Port ( clk_sys 		: in  STD_LOGIC;
           reset_n 		: in  STD_LOGIC;
           master_i 		: in  t_wishbone_master_in;
           master_o 		: out t_wishbone_master_out;
			  slave_ram_i	: in  t_wishbone_slave_in;
			  slave_ram_o 	: out t_wishbone_slave_out;			  
			  wrpc_uart_rxd_i: inout std_logic;
			  wrpc_uart_txd_o: inout std_logic;
           uart_rxd_i 	: in  STD_LOGIC;
           uart_txd_o 	: out STD_LOGIC;
			  running_indicator : out STD_LOGIC;
			  control_button : in std_logic);
end wb_debugger;

architecture Behavioral of wb_debugger is

function f_xwb_dpram_dbg(g_size : natural) return t_sdb_device
  is
    variable result : t_sdb_device;
  begin
    result.abi_class     := x"0001"; -- RAM device
    result.abi_ver_major := x"01";
    result.abi_ver_minor := x"00";
    result.wbd_width     := x"7"; -- 32/16/8-bit supported
    result.wbd_endian    := c_sdb_endian_big;
    
    result.sdb_component.addr_first := (others => '0');
    result.sdb_component.addr_last  := std_logic_vector(to_unsigned(g_size*4-1, 64));
    
    result.sdb_component.product.vendor_id := x"000000000000CE42"; -- CERN
    result.sdb_component.product.device_id := x"deaf0bee";
    result.sdb_component.product.version   := x"00000001";
    result.sdb_component.product.date      := x"20120305";
    result.sdb_component.product.name      := "BlockRAM-Debugger  ";
    
    return result;
  end f_xwb_dpram_dbg;
  
  
  constant c_NUM_TIMERS		 : natural range 1 to 3 := 1;
  
  constant c_NUM_WB_MASTERS : integer := 6;
  constant c_NUM_WB_SLAVES  : integer := 2;

  constant c_MASTER_LM32	: integer := 0; ---has two
  --constant c_MASTER_OUT_PORT : integer := 2;
  
  constant c_EXT_BRIDGE			: integer := 0;
  constant c_SLAVE_DPRAM	 	: integer := 1;
  constant c_SLAVE_UART		 	: integer := 2;
  constant c_SLAVE_TICS		 	: integer := 3;
  constant c_SLAVE_TIMER_IRQ	: integer := 4;
  constant c_SLAVE_IRQ_CTRL	: integer := 5;

  constant c_EXT_BRIDGE_SDB : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"003fffff", x"00300000");
	 
  --constant init_lm32_addr : t_wishbone_address := x"00040000";

  --constant g_dpram_size		: integer := 114688/4;  --in 32-bit words
  constant c_FREQ_DIVIDER	: integer := 62500; -- LM32 clk = 62.5 Mhz
  
  
  
  constant c_uart_sdb_dbg : t_sdb_device := (
    abi_class     => x"0000",              -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7",                 -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"00000000000000ff",
      product     => (
        vendor_id => x"000000000000CE42",  -- CERN
        device_id => x"dead0fee",
        version   => x"00000001",
        date      => x"20120305",
        name      => "WB-UART-debugger   "))); 

  constant c_xwb_tics_sdb_dbg : t_sdb_device := (
    abi_class     => x"0000",              -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"00",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7",                 -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"0000000000000000",
      product     => (
        vendor_id => x"000000000000CE42",  -- GSI
        device_id => x"adabadaa",
        version   => x"00000001",
        date      => x"20111004",
        name      => "WB-Tics-Debugger   ")));
		  

  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array(c_NUM_WB_MASTERS-1 downto 0) :=
    (c_EXT_BRIDGE		 => f_sdb_embed_bridge(c_EXT_BRIDGE_SDB, x"00c00000"), 
	  c_SLAVE_DPRAM	 => f_sdb_embed_device(f_xwb_dpram_dbg(g_dbg_dpram_size), g_reset_vector),
	  c_SLAVE_UART	    => f_sdb_embed_device(c_uart_sdb_dbg, x"00600000"), -- UART
	  c_SLAVE_TICS	    => f_sdb_embed_device(c_xwb_tics_sdb_dbg, x"00700000"),
	  c_SLAVE_TIMER_IRQ=> f_sdb_embed_device(c_irq_timer_sdb, x"00750000"),
	  c_SLAVE_IRQ_CTRL	=> f_sdb_embed_device(c_irq_ctrl_sdb, x"00770000"));  

  constant c_SDB_ADDRESS : t_wishbone_address := x"00400000";
  
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_MASTERS-1 downto 0);

  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);
  
  signal debugger_ram_wbb_i : t_wishbone_slave_in;
  signal debugger_ram_wbb_o : t_wishbone_slave_out;
  
  signal periph_slave_i : t_wishbone_slave_in_array(0 to 2);
  signal periph_slave_o : t_wishbone_slave_out_array(0 to 2);
  signal periph_dummy	: std_logic_vector (9 downto 0);
  signal wrpc_dummy		: std_logic_vector (2 downto 0);
  signal forced_lm32_reset_n : std_logic := '0';
  signal button2_synced_n : std_logic;
  
  signal irq_slave_i : t_wishbone_slave_in_array(g_msi_queues-1 to 0);
  signal irq_slave_o : t_wishbone_slave_out_array(g_msi_queues-1 to 0);

  signal local_counter : unsigned (63 downto 0);
  
  signal uart_dummy_i 	: std_logic;
  signal uart_dummy_o 	: std_logic;

  signal dbg_uart_rxd_i	: std_logic;
  signal dbg_uart_txd_o	: std_logic;
  
  signal use_dbg_uart	: std_logic := '1';
  signal state_control 	: unsigned (39 downto 0) := x"0000000000";

begin
  running_indicator <= forced_lm32_reset_n;
  
  master_o <= cnx_master_out(c_EXT_BRIDGE);
  cnx_master_in(c_EXT_BRIDGE) <= master_i;
  
  controller : process (clk_sys)
	begin 
		if (rising_edge(clk_sys)) then
			if (control_button = '0' and (state_control /= x"ffffffffff")) then
				state_control <= state_control + 1;
			else
				if ((state_control /= x"0000000000") and (state_control <= x"3B9ACA0")) then -- 62500000 -> 0.5s
					forced_lm32_reset_n <= not forced_lm32_reset_n;
				elsif (state_control > x"3B9ACA0") then
					use_dbg_uart <= not use_dbg_uart;
				end if;
				state_control <= x"0000000000";
			end if;
		end if;
	end process;
	
	uart_txd_o <= dbg_uart_txd_o when use_dbg_uart ='1' else wrpc_uart_txd_o;
	dbg_uart_rxd_i <= uart_rxd_i when use_dbg_uart ='1' else '1';
	wrpc_uart_rxd_i <= uart_rxd_i when use_dbg_uart ='0' else '1';

--------------------------------------
-- UART
--------------------------------------
  UART : xwb_simple_uart
    generic map(
      g_with_virtual_uart   => false,
      g_with_physical_uart  => true,
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE
      )
    port map(
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,

      -- Wishbone
		slave_i => cnx_master_out(c_SLAVE_UART),
      slave_o => cnx_master_in(c_SLAVE_UART),
      desc_o  => open,

		uart_rxd_i => dbg_uart_rxd_i,
      uart_txd_o => dbg_uart_txd_o
      );

--------------------------------------
-- Tics Counter
--------------------------------------
	tic_cnt : xwb_tics 
	generic map(
		g_period => c_FREQ_DIVIDER
		)
	port map(
		clk_sys_i => clk_sys,
		rst_n_i   => reset_n,

		-- Wishbone
		slave_i => cnx_master_out(c_SLAVE_TICS),
      slave_o => cnx_master_in(c_SLAVE_TICS),
      desc_o  => open
    );

-----------------------------------------------------------------------------
-- LM32 with MSI interface
----------------------------------------------------------------------------- 

	U_LM32_CORE : wb_irq_lm32
	generic map(
			  g_msi_queues => g_msi_queues, 
			  g_profile => g_profile)
			  --g_reset_vector=> init_lm32_addr)
	port map(
		clk_sys_i => clk_sys,
		rst_n_i => forced_lm32_reset_n,

	   dwb_o => cnx_slave_in(c_MASTER_LM32),
      dwb_i => cnx_slave_out(c_MASTER_LM32),
      iwb_o => cnx_slave_in(c_MASTER_LM32+1),
      iwb_i => cnx_slave_out(c_MASTER_LM32+1),

		irq_slave_o  => irq_slave_o,  -- wb msi interface
		irq_slave_i  => irq_slave_i,
				
		ctrl_slave_o => cnx_master_in(c_SLAVE_IRQ_CTRL),                -- ctrl interface for LM32 irq processing
		ctrl_slave_i => cnx_master_out(c_SLAVE_IRQ_CTRL)
	);

---------------------------------------------------------------------------
-- Dual-port RAM
-----------------------------------------------------------------------------  
  U_DPRAM : xwb_dpram
    generic map(
      g_size                  => g_dbg_dpram_size,  --in 32-bit words
      g_init_file             => g_dbg_init_file,
      g_must_have_init_file   => true,  --> OJO <--
      g_slave1_interface_mode => PIPELINED,
      g_slave2_interface_mode => PIPELINED,
      g_slave1_granularity    => BYTE,
      g_slave2_granularity    => WORD)  
    port map(
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,

      slave1_i => cnx_master_out(c_SLAVE_DPRAM),
      slave1_o => cnx_master_in(c_SLAVE_DPRAM),
      slave2_i => debugger_ram_wbb_i,
      slave2_o => debugger_ram_wbb_o
      );

---------------------------------------------------------------------------
-- IRQ - Timer
---------------------------------------------------------------------------
  process(clk_sys)
	begin
	if (clk_sys'event and clk_sys = '1') then
		if (reset_n = '0') then
			local_counter <= (others => '0');
		else
			local_counter <= local_counter + 1;
		end if;
	end if;
  end process;
		
  U_Timer : wb_irq_timer 
  generic map( g_timers =>  g_timers)
  port map(clk_sys_i     => clk_sys,           
           rst_sys_n_i   => reset_n,             
         
           tm_tai8ns_i   => std_logic_vector(local_counter),

           ctrl_slave_o  => cnx_master_in(c_SLAVE_TIMER_IRQ), 				 -- ctrl interface for LM32 irq processing
           ctrl_slave_i  => cnx_master_out(c_SLAVE_TIMER_IRQ),
           
           irq_master_o  => irq_slave_i(0),                             -- wb msi interface 
           irq_master_i  => irq_slave_o(0)
   );

  U_Intercon : xwb_sdb_crossbar
    generic map (
      g_num_masters => c_NUM_WB_SLAVES,
      g_num_slaves  => c_NUM_WB_MASTERS,
      g_registered  => true,
      g_wraparound  => true,
      g_layout      => c_INTERCONNECT_LAYOUT,
      g_sdb_addr    => c_SDB_ADDRESS)
    port map (
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,
      slave_i   => cnx_slave_in,
      slave_o   => cnx_slave_out,
      master_i  => cnx_master_in,
      master_o  => cnx_master_out);
		
end Behavioral;

