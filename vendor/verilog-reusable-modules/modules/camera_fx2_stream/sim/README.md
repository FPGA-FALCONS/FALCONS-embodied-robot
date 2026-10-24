# Camera to FX2 transport simulation

Run `python run_sim.py --out <evidence-directory>` with Python 3 and Icarus
Verilog (`iverilog`, `vvp`) on PATH. The runner also recognizes the installed
Windows tools under `C:/iverilog/bin`. No physical board is accessed.

Three parameter sets cover downsampling, transfers crossing multiple USB
packets, and a frame that fills exactly one 512-byte packet:

| Name | Source | Downsample | Output | Complete frame bytes | PKTEND |
|---|---|---|---|---|---|
| small | 8 x 6 | 2 | 4 x 3 | 48 | Once per completed frame |
| multi_packet | 32 x 16 | 1 | 32 x 16 | 1048 | Once per completed frame |
| exact_packet | 244 x 1 | 1 | 244 x 1 | 512 | Never |

Each run checks byte-exact FIMG header, little-endian RGB565 payload and CRC;
startup pixels before the first frame boundary; random FLAGC stalls; a long
full-FIFO interval; held FD and inactive SLWR while a send is stalled; frozen
RAM ownership during transport; whole-frame busy drops; truncated and excess
camera frames; coincident VSYNC/valid pixels ignored on frame boundaries;
common reset during capture and during transmission; and single-cycle PKTEND
following a complete guard cycle. Short-frame scenarios explicitly lower
FLAGC after entry to PACKET_END and hold it full for seven cycles, checking
that commit waits until writable again.

Artifacts per scenario:

- `wire.bin`: actual FD bus bytes sampled only on accepted SLWR cycles.
- `wire.hex`: identical bytes in printable form; the runner verifies equality.
- `expected_frames.json`: the three complete frames expected from an
  independently implemented host FIMG parser.
- `expected_frames.txt`: the same records emitted directly by the testbench.
- `compile.log`, `simulation.log`, `commands.json`: execution evidence.

Input pixel value is `(tag * 256 + y * source_width + x) & 0xffff`.
Downsampling retains coordinates divisible by the downsample factor. A run
deliberately includes a ten-byte prefix of an interrupted frame between the
second and third complete frames. This prefix is not an expected complete
frame; a stream receiver must resynchronize after it. Complete frames have
tags 1, 3, 8 and sequence numbers 0, 1, 0 respectively. The final sequence
returns to zero because common reset begins a new transport session.

This is a digital transport simulation. It does not establish camera timing,
board pin assignments, FX2 firmware or endpoint configuration, USB electrical
behavior, synthesis RAM inference, or physical USB throughput.
