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
use work.debugger_pkg.all;

use work.synthesis_descriptor.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity wb_debugger is
	generic(
    g_dbg_dpram_size	    : integer;
    g_dbg_init_file       : string;
    g_reset_vector	      : t_wishbone_address := x"00000000"; -- if wb_irq_lm32 from general-cores::proposed-master
    g_msi_queues 	        : natural := 1;
    g_profile		          : string  := "medium_icache_debug";
    g_internal_time_ref   : boolean := true;
    g_timers			        : integer := 1;
    g_slave_interface_mode: t_wishbone_interface_mode := PIPELINED;
    g_slave_granularity   : t_wishbone_address_granularity := BYTE);
  port(
    clk_sys   : in  std_logic;
    reset_n   : in  std_logic;
    master_i  : in  t_wishbone_master_in;
    master_o  : out t_wishbone_master_out;
    slave_i   : in  t_wishbone_slave_in;
    slave_o   : out t_wishbone_slave_out;
    
    wrpc_uart_rxd_i : inout std_logic;
    wrpc_uart_txd_o : inout std_logic;
    uart_rxd_i 	    : in  std_logic;
    uart_txd_o 	    : out std_logic;
    
    dbg_indicator       : out std_logic;
		dbg_control_select  : in std_logic);
end wb_debugger;

architecture Behavioral of wb_debugger is

  function f_check_if_lm32_firmware_necessary return boolean is
  begin
    if(g_dbg_init_file /= "") then
      return true;
    else
      return false;
    end if;
  end function;
  
  function f_generate_irq_timer return integer is
  begin
    if(g_timers /= 0) then
      return 1;
    else
      return 0;
    end if;
  end function;
  
  function f_generate_time_ref return integer is
  begin
    if(g_internal_time_ref) then
      return 1;
    else
      return 0;
    end if;
  end function;
  
  function f_choose_lm32_firmware_file return string is
  begin
    if(g_dbg_init_file = "debugger") then
      report "[Dbg Core] Using debugging firmware." severity note;
      return "dbg_code.ram";
    elsif (g_dbg_init_file = "FD_node") then
      report "[Dbg Core] Using FMC Delay stand alone node firmware." severity note;
      return "fd_std.ram";
    else
      report "[Dbg Core] Using user provided firmware."  severity note;
      return g_dbg_init_file;
    end if;
  end function;
  
  function f_select_dpram_size return integer is
  begin
    if(g_dbg_init_file = "debugger") then
      report "[Dbg Core] Using a 40960 Bytes size RAM." severity note;
      return 40960;
    elsif (g_dbg_init_file = "FD_node") then
      report "[Dbg Core] Using a 114740 Bytes RAM." severity note;
      return 114740;
    else
      report "[Dbg Core] Using user specifie size RAM size." severity note;
      return g_dbg_dpram_size;
    end if;
  end function;
-- constant c_NUM_WB_MASTERS : integer := 6 + f_generate_irq_timer + f_generate_time_ref;
  constant c_NUM_WB_MASTERS : integer := 4 + f_generate_irq_timer + f_generate_time_ref;
  constant c_NUM_WB_SLAVES  : integer := 3;

  constant c_MASTER_LM32    : integer := 0;
  constant c_MASTER_ADAPT   : integer := 2;
  
  constant c_EXT_BRIDGE			: integer := 0;
  constant c_SLAVE_DPRAM		: integer := 1;
  constant c_SLAVE_UART     : integer := 2;
  constant c_SLAVE_IRQ_CTRL	: integer := 3;
  constant c_SLAVE_TICS		 	: integer := c_SLAVE_IRQ_CTRL + f_generate_time_ref;
  constant c_SLAVE_TIMER_IRQ: integer := c_SLAVE_TICS + f_generate_irq_timer;

  constant c_EXT_BRIDGE_SDB : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"000effff", x"00000000");
  
  constant c_FREQ_DIVIDER	: integer := 62500; -- LM32 clk = 62.5 Mhz
  
  function f_generate_c_interconection_layout(
    num_wb_masters       : integer;
    last_mandatory_slave : integer
    )
  return t_sdb_record_array is
    variable interconnect_layout : t_sdb_record_array(NUM_WB_MASTERS-1 downto 0);
    variable offset : integer range last_mandatory_slave to NUM_WB_MASTERS-1 := last_mandatory_slave;
    variable adr_off: unsigned (c_wishbone_address_width-1 downto 0);
    begin
      -- Vader is Coming Look Busy
      interconnect_layout (offset downto 0):=
      (c_EXT_BRIDGE     => f_sdb_embed_bridge(c_EXT_BRIDGE_SDB,     x"00100000"),
       c_SLAVE_DPRAM    => f_sdb_embed_device(f_xwb_dbg_dpram(g_dbg_dpram_size), x"00000000"),
       c_SLAVE_UART     => f_sdb_embed_device(c_dbg_uart_sdb,       x"00020100"),
       c_SLAVE_IRQ_CTRL => f_sdb_embed_device(c_dbg_irq_ctrl_sdb,   x"00020200"));
      
      adr_off := x"00020300";
    
      if (f_generate_time_ref /= 0) then
        offset := offset + f_generate_time_ref;
        interconnect_layout (offset) := f_sdb_embed_device(c_xwb_dbg_tics_sdb, t_wishbone_address(adr_off));
        adr_off := adr_off + x"100";
      end if;
      
      if (f_generate_irq_timer /= 0) then
        offset := offset + f_generate_irq_timer;
        interconnect_layout (offset) := f_sdb_embed_device(c_dbg_irq_timer_sdb, t_wishbone_address(adr_off));
        adr_off := adr_off + x"100";
      end if;
      
      --interconnect_layout (offset+1) := f_sdb_embed_synthesis(c_sdb_synthesis_info);
      --interconnect_layout (offset+2) := f_sdb_embed_repo_url(c_sdb_repo_url);
      
      return interconnect_layout;
    end function;
  
  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array := f_generate_c_interconection_layout (c_NUM_WB_MASTERS, c_SLAVE_IRQ_CTRL);

  constant c_SDB_ADDRESS : t_wishbone_address := x"00020800";
  
 
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array (c_NUM_WB_MASTERS-1 downto 0);

  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array (c_NUM_WB_SLAVES-1 downto 0);  
   
  signal dummy_debugger_ram_wbb_i : t_wishbone_slave_in;

  signal forced_lm32_reset_n : std_logic := '0';
  
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
  dbg_indicator <= forced_lm32_reset_n;
  
  master_o <= cnx_master_out(c_EXT_BRIDGE);
  cnx_master_in(c_EXT_BRIDGE) <= master_i;
  
--------------------------------------
-- UART Selector & Reset controller
--------------------------------------  
  controller : process (clk_sys)
  begin 
    if (rising_edge(clk_sys)) then
      if (dbg_control_select = '0' and (state_control /= x"ffffffffff")) then
        state_control <= state_control + 1;
	  else
        if ((state_control /= x"0000000000") and (state_control <= x"3B9ACA0")) then --0.5s
          forced_lm32_reset_n <= not forced_lm32_reset_n;
        elsif (state_control > x"3B9ACA0") then
          use_dbg_uart <= not use_dbg_uart;
        end if;
        state_control <= x"0000000000";
      end if;
    end if;
  end process;
	
--------------------------------------
-- UART
--------------------------------------
	uart_txd_o <= dbg_uart_txd_o when use_dbg_uart ='1' else wrpc_uart_txd_o;
	dbg_uart_rxd_i <= uart_rxd_i when use_dbg_uart ='1' else '1';
	wrpc_uart_rxd_i <= uart_rxd_i when use_dbg_uart ='0' else '1';

  DBG_UART : xwb_simple_uart
    generic map(
      g_with_virtual_uart   => true,
      g_with_physical_uart  => true,
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE
      )
    port map(
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,

      slave_i => cnx_master_out(c_SLAVE_UART),
      slave_o => cnx_master_in(c_SLAVE_UART),
      desc_o  => open,

      uart_rxd_i => dbg_uart_rxd_i,
      uart_txd_o => dbg_uart_txd_o
      );

-----------------------------------------------------------------------------
-- LM32, with MSI interface
----------------------------------------------------------------------------- 
	DBG_IRQ_LM32_CORE : wb_irq_lm32
    generic map(
      g_msi_queues => g_msi_queues, 
      g_profile    => g_profile
      )
    port map(
      clk_sys_i => clk_sys,
      rst_n_i   => forced_lm32_reset_n,

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
  DBG_DPRAM : xwb_dpram
    generic map(
      g_size                  => f_select_dpram_size/4,  --in 32-bit words
--      g_size                  => g_dbg_dpram_size,  --in 32-bit words
      g_init_file             => f_choose_lm32_firmware_file,
      g_must_have_init_file   => f_check_if_lm32_firmware_necessary,
      g_slave1_interface_mode => PIPELINED,
      g_slave2_interface_mode => PIPELINED,
      g_slave1_granularity    => BYTE,
      g_slave2_granularity    => WORD
      )
    port map(
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,
    
      slave1_i => cnx_master_out(c_SLAVE_DPRAM),
      slave1_o => cnx_master_in(c_SLAVE_DPRAM),
      slave2_i => dummy_debugger_ram_wbb_i,
      slave2_o => open
      );

---------------------------------------------------------------------------
-- TIMER
---------------------------------------------------------------------------
  gen_time_ref : if (f_generate_time_ref /= 0) generate
  begin 
    DBG_TIME_REF : xwb_tics 
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
    end generate gen_time_ref;
    
  gen_timer : if (g_timers > 0) generate
  begin
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
    
    DBG_IRQ_TIMER : wb_irq_timer 
      generic map( 
        g_timers => g_timers
        )
      port map(
        clk_sys_i     => clk_sys,           
        rst_sys_n_i   => forced_lm32_reset_n,             
           
        tm_tai8ns_i   => std_logic_vector(local_counter),

        ctrl_slave_o  => cnx_master_in(c_SLAVE_TIMER_IRQ), 		 -- ctrl interface for LM32 irq processing
        ctrl_slave_i  => cnx_master_out(c_SLAVE_TIMER_IRQ),
        
        irq_master_o  => irq_slave_i(g_timers-1),              -- wb msi interface 
        irq_master_i  => irq_slave_o(g_timers-1)
        );
    end generate gen_timer;

---------------------------------------------------------------------------
-- Crossbar
---------------------------------------------------------------------------
  DBG_MAIN_INTERCON : xwb_sdb_crossbar
    generic map (
      g_num_masters => c_NUM_WB_SLAVES,
      g_num_slaves  => c_NUM_WB_MASTERS,
      g_registered  => true,
      g_wraparound  => true,
      g_layout      => c_INTERCONNECT_LAYOUT,
      g_sdb_addr    => c_SDB_ADDRESS
      )
    port map (
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,
      slave_i   => cnx_slave_in,
      slave_o   => cnx_slave_out,
      master_i  => cnx_master_in,
      master_o  => cnx_master_out
      );

---------------------------------------------------------------------------
-- Adatper
---------------------------------------------------------------------------
  DBG_SALVE_ADAPTER : wb_slave_adapter
    generic map (
      g_master_use_struct  => true,
      g_master_mode        => g_slave_interface_mode,
      g_master_granularity => BYTE,
      g_slave_use_struct   => false,
      g_slave_mode         => g_slave_interface_mode,
      g_slave_granularity  => g_slave_granularity)
    port map (
      clk_sys_i => clk_sys,
      rst_n_i   => reset_n,
      
      master_i  => cnx_slave_out(c_MASTER_ADAPT),
      master_o  => cnx_slave_in(c_MASTER_ADAPT),
      
      sl_adr_i(c_wishbone_address_width-1 downto 16)  => (others => '0'),
      sl_adr_i(15 downto 0)   => slave_i.adr(15 downto 0),
      sl_dat_i   => slave_i.dat,
      sl_sel_i   => slave_i.sel,
      sl_cyc_i   => slave_i.cyc,
      sl_stb_i   => slave_i.stb,
      sl_we_i    => slave_i.we,
      sl_dat_o   => slave_o.dat,
      sl_ack_o   => slave_o.ack,
      sl_err_o   => slave_o.err,
      sl_rty_o   => slave_o.rty,
      sl_stall_o => slave_o.stall
      );
end Behavioral;
