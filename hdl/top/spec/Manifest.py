files = ["synthesis_descriptor.vhd", "spec_top.vhd", "spec_top.ucf", "spec_reset_gen.vhd"]

fetchto = "../../ip_cores"

modules = {
    "local" : ["../../rtl", "../../platform" ],
    "git" : [ "git://ohwr.org/hdl-core-lib/wr-cores.git",
    					"git://ohwr.org/hdl-core-lib/etherbone-core.git",
					    "git://ohwr.org/hdl-core-lib/gn4124-core.git" ]
    }
