vlib work
vlog C.v
vsim C

log {/*}
add wave {/*}

# input: clock(CLOCK_50), s1(SW[1]), s0(SW[0]), resetn(SW[2])
# output: Q(LEDR)

# Set clock
force {CLOCK_50} 1 0ns, 0 {10ns} -r 20ns
