--
--	Package File Template
--
--	Purpose: This package defines supplemental types, subtypes, 
--		 constants, and functions 
--
--   To use any of the example code shown below, uncomment the lines and modify as necessary
--

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

package debugger_pkg is
------------------------------------------------------------------------------
-- Constants
-------------------------------------------------------------------------------
  constant c_dbg_uart_sdb : t_sdb_device := (
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
        device_id => x"0deafbee",          -- she didn't listen & cames & goes
        version   => x"00000001",
        date      => x"20120305",
        name      => "WB-UART-Debugger   ")));

  constant c_dbg_irq_ctrl_sdb : t_sdb_device := (
    abi_class     => x"0000", -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7", -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first    => x"0000000000000000",
      addr_last     => x"00000000000000ff",
      product => (
        vendor_id     => x"0000000000000651", -- GSI
        device_id     => x"e1fb1ade",         -- balanced, perfect grip, absolute control
        version       => x"00000001",
        date          => x"20120308",
        name          => "IRQ_CTRL-Debugger  ")));

  constant c_xwb_dbg_tics_sdb : t_sdb_device := (
    abi_class     => x"0000",              -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"00",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7",                 -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"0000000000000000",
      product     => (
        vendor_id => x"000000000000CE42",   -- GSIx
        device_id => x"fade1eaf",           -- Time is always ticking!  
        version   => x"00000001",
        date      => x"20111004",
        name      => "WB-Tics-Debugger   ")));

  constant c_dbg_irq_timer_sdb : t_sdb_device := (
    abi_class     => x"0000", -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7", -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first    => x"0000000000000000",
      addr_last     => x"00000000000000ff",
      product => (
        vendor_id     => x"0000000000000651", -- GSI
        device_id     => x"deadface",         -- eventully "the dead line" is going to arrive
        version       => x"00000001",
        date          => x"20120308",
        name          => "IRQ_TIMER-Debugger ")));

  constant c_xwb_dbg_slave_sdb : t_sdb_device := (
    abi_class     => x"0000",              -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"00",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7",                 -- 8/16/32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"00000000000201ff",
      product     => (
        vendor_id => x"a1eBEEFc0ffeeBED",  -- Jose Jimenez Motel. Open 24/7. Next exit.
        device_id => x"c0a110de",          -- obvious (sadly)
        version   => x"00000001",
        date      => x"20140704",
        name      => "Debugger-Slave     ")));
        
------------------------------------------------------------------------------
-- Functions
-------------------------------------------------------------------------------
  function f_xwb_dbg_dpram(g_size : natural) return t_sdb_device
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
      result.sdb_component.product.device_id := x"deafbeef"; -- she didn't listen & is as essential as protein
      result.sdb_component.product.version   := x"00000001";
      result.sdb_component.product.date      := x"20120305";
      result.sdb_component.product.name      := "BlockRAM-Debugger  ";
    
    return result;
  end f_xwb_dbg_dpram;


------------------------------------------------------------------------------
-- Components declaration
-------------------------------------------------------------------------------
  component  wb_debugger is
    generic (	
      g_dbg_dpram_size        : integer := 40960/4;
      g_dbg_init_file         : string;
      g_reset_vector	        : t_wishbone_address := x"00000000";
      g_msi_queues 	          : natural := 1;
      g_profile		            : string  := "medium_icache_debug";
      g_internal_time_ref     : boolean := true;
      g_timers			          : integer := 1;				
      g_slave_interface_mode  : t_wishbone_interface_mode := PIPELINED;
      g_slave_granularity     : t_wishbone_address_granularity := BYTE
      );
    port (
      clk_sys 	: in std_logic;
      reset_n 	: in std_logic;
      
      master_i 	: in  t_wishbone_master_in;
      master_o 	: out t_wishbone_master_out;
      slave_i   : in  t_wishbone_slave_in;
      slave_o 	: out t_wishbone_slave_out;
      
      wrpc_uart_rxd_i : inout std_logic;
      wrpc_uart_txd_o : inout std_logic;
      uart_rxd_i 	    : in  std_logic;
      uart_txd_o 	    : out std_logic;
      
      dbg_indicator       : out std_logic;
      dbg_control_select  : in std_logic
      );
  end component;
  
end debugger_pkg;

package body debugger_pkg is

-- Notihg to include right now!!!

end debugger_pkg;
