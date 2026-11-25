set nskip 0; set nsat 0; set ns2m 0
when {/tb/clk == 1} {
  if {[examine /tb/u_hub/_tsd/go_tw_skip] == 1} { incr nskip }
  if {[examine /tb/u_hub/_tsd/pick] == 2 && [examine /tb/u_hub/_tsd/s_match] == 1} { incr ns2m }
}
when {/tb/u_hub/_tsd/tw_sat == 1} { incr nsat }
run -all
echo "COUNTS skip_clocks=$nskip stm32_writes_overtaken=$nsat saturn_reads_of_pending_copy=$ns2m"
quit -f
