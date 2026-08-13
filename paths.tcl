project_open Atari2600
create_timing_netlist -model slow
read_sdc
update_timing_netlist
puts "===CLKVID_WORST==="
report_timing -setup -npaths 3 -detail summary -stdout \
  -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]
delete_timing_netlist
project_close
