target = "xilinx"
action = "synthesis"

fetchto = "../../ip_cores"

syn_device = "xc6slx45t"
syn_grade = "-3"
syn_package = "fgg484"
syn_top = "spec_top"
syn_project = "spec_fine_delay.xise"

modules = { "local" : [ "../../top/spec", "../../platform" ] }

files = ["wb_debugger.vhd", "debugger_pkg.vhd", "wrc-ethb.ram", "dbg_code.ram"]
