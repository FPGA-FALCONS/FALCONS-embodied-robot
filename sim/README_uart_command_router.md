# Shared UART MOT1 / ASCII isolation

`rtl/debug/uart_command_router.v` sits after the UART byte receiver. Connect its
ASCII outputs to `uart_cmd_rx`, and its motion outputs to `voice_motion_bridge`.
Connect `o_ascii_abort` to the new `uart_cmd_rx.i_abort` input. A binary boundary
clears an incomplete ASCII command without changing already committed PID values.

Only the full four-byte `MOT1` magic claims a binary frame. All 12 bytes are then
isolated from ASCII, even for unknown commands or invalid CRC. CRC and command
validation remain the motion bridge's responsibility. Unmatched partial magic is
replayed in order as ordinary ASCII. `M`, `MO`, and `MOT` at end of input are
released after the inter-byte timeout (default 20 ms).

A claimed but truncated binary frame expires after the same timeout. Its received
bytes remain binary and are never replayed into the ASCII parser. The motion
bridge must independently expire its partial parser after 20 ms as well; timeout
does not revoke an active command or refresh its motion watchdog.

The tagged output queue handles one input byte every clock, including back-to-back
frames and prefix mismatches. Abort is aligned to the first forwarded motion byte,
after all preceding ASCII has drained. It cannot discard a preceding complete PID
command's final byte.

Run from the repository root:

```text
iverilog -g2012 -Wall -s tb_uart_command_router -o router_tb.vvp rtl/debug/uart_command_router.v rtl/debug/uart_cmd_rx.v sim/tb_uart_command_router.sv
vvp router_tb.vvp
```

The self-checking test routes all 256 sequence values for each of nine legal MOT1
commands, including STOP seq32 `4d4f5431200000000000cc78`. It also covers bad CRC,
unknown commands, ASCII-looking binary bytes, back-to-back frames, partial magic,
overlapping magic candidates, prefix timeout, a half-written PID command followed
by MOT1, truncated-frame timeout, and subsequent PID/immediate command recovery.
This is simulation evidence, not a physical UART or vehicle test.
