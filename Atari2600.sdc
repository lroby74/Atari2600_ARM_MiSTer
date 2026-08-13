# Core-specific timing constraints for Atari2600.
#
# clk_sys/clk_vid/clk_tia (rtl/pll/pll_0002.v) get their base create_clock via
# sys/sys_top.sdc's derive_pll_clocks (matches any *|pll|pll_inst|... instance,
# which covers this core's pll_0002 instantiated as "pll" in Atari2600.sv).
# No duplicate create_clock/derive_clocks needed here.
#
# sys_top.sdc however puts ALL THREE pll outputs (clk_sys/clk_vid/clk_tia) in
# ONE exclusive clock group together with several unrelated system clocks:
#   set_clock_groups -exclusive -group [get_clocks {*|pll|pll_inst|...|divclk}] ...
# "-exclusive" means Quartus does NOT time paths between clocks in different
# groups, but WOULD normally still time paths between clk_sys and clk_vid
# since -exclusive treats everything INSIDE one group as mutually synchronous/
# related, not asynchronous. That is wrong for this core once the ARM Thumb
# core moves to clk_vid: clk_sys and clk_vid are independent altera_pll
# outputs and Quartus does not guarantee a known phase relationship between
# them (even though clk_vid is an exact 4x multiple of clk_sys), so paths
# between the two domains must be constrained as genuinely asynchronous, not
# left unconstrained/mis-related.
#
# Atari2600.sdc is read after sys_top.sdc, so this narrower group wins for
# the clk_sys<->clk_vid pair specifically (verify in Quartus: Timing Analyzer
# > Report > Clock Groups, once this file is in use with clk_vid actually
# driving a register - Step B2).
#
# Livello 1 (fix jitter): il crossing clk_sys<->clk_vid ora avviene in DUE
# forme, ENTRAMBE coperte da questo stesso -asynchronous:
#   1) i sincronizzatori RTL cdc_sync2 dei soli segnali di controllo/stato
#      (core_reset, callfn, halted) e dell'handshake callback musicale;
#   2) le M10K true dual-clock dual-port (harmony_m10k_tdp, cdf_rom_m10k):
#      porta A su clk_vid, porta B su clk_sys. Il crossing e' interno alla
#      primitiva M10K, che il Cyclone V gestisce in hardware; -asynchronous
#      dice a Quartus di non temporizzare tra i due domini attraverso di essa.
# Nessun percorso dati ARM<->bridge attraversa piu' un handshake RTL a 4 fasi
# (rimosso in Livello 1): il bus ARM e' interamente su clk_vid.
set_clock_groups -asynchronous \
   -group [get_clocks {*|pll|pll_inst|altera_pll_i|*[0].*|divclk}] \
   -group [get_clocks {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}] \
   -group [get_clocks {*|pll|pll_inst|altera_pll_i|*[3].*|divclk}]
