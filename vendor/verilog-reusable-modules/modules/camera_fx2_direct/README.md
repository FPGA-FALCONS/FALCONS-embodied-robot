# Camera → FX2 direct stream (FIMG v2)

This package is a **no-full-frame-buffer** transport for the OV5640 DVP stream.
It is intentionally separate from `camera_fx2_stream` (the frozen-frame 512×300
transport).  The application channel mapping and display path are unchanged.

```
OV5640 → existing ov5640_capture ─┬→ existing DDR3 → HDMI/UI
                                  └→ 5-of-8 X/Y decimator
                                       → small dual-clock FIFO
                                       → IFCLK prefetch FIFO
                                       → FX2 EP6 (FIMG v2)
```

## Why decimation is mandatory for a direct path

A DVP camera cannot be backpressured.  `DVP_Capture` is an 8-bit interface and
forms one RGB565 pixel every **2 PCLK** (`DataValid` on odd Hcount), so 42 MHz
PCLK is 21 Mpx/s (42 MB/s), not 42 Mpx/s.  At 30 fps the 1024×600 average is
18.4 Mpx/s (36.9 MB/s), with a 21 Mpx/s active burst.  The FX2 high-speed bulk
path is slower and bursty (roughly 12–24 Mword/s depending on firmware/USB host).
A small FIFO can absorb line jitter, but not a full active frame; a 1024×600
1:1 direct stream needs a 1.23 MB elastic buffer or a separate DDR3 read port
unless FX2 can sustain more than the 36.9 MB/s average.

The board configuration therefore emits **5 of every 8 columns and rows**:
1024×600 → **640×375** (same aspect ratio, 240,000 pixels/frame, 30 fps source).
The active output rate is 21 Mpx/s × (5/8)² ≈ **8.2 Mpx/s (16.4 MB/s)**, comfortably below the prefetch path's ≈24 Mword/s at 48 MHz IFCLK.  The 128k-word (256 KB)
elastic FIFO absorbs the active-line burst while USB drains during blanking; it
is still not a full-frame buffer.  The decimator is a pair of phase counters.

The `DECIM_NUM`/`DECIM_DEN` parameters allow 1/1, 2/2, 5/8, 3/4, etc., provided
the source dimensions divide `DECIM_DEN`.  1/1 is useful for simulation and for
experiments where the camera PCLK is reduced; it is not sustainable at the
default 42 MHz PCLK over FX2.

## FIMG v2 wire format

| Field | Value |
|---|---|
| magic | `FIMG` |
| version / format | 2 / 1 (RGB565 little-endian) |
| header length | 24 |
| width / height | decimated output geometry |
| sequence | uint32, incremented for every admitted source frame |
| payload bytes | width × height × 2 |
| header field 10 | CRC32 of header bytes 0..19 |
| trailer | CRC32 of header bytes 0..19 + payload |

The trailer follows the payload; it is not included in its own CRC.  A frame
that overflows the FIFO or is truncated still sends the declared payload
length, zero-padded, with a deliberately non-matching trailer.  The host
parser discards it, so byte framing and recovery are preserved.

`camera_fx2_stream` (v1) places the CRC in header field 10 and has no trailer.
The host parser accepts both versions.

## Parameters and ports

| Parameter | Board value | Meaning |
|---|---|---|
| `SOURCE_WIDTH/HEIGHT` | 1024/600 | OV5640 raster |
| `DECIM_NUM/DECIM_DEN` | 5/8 | keep 5 of every 8 columns/rows |
| `FRAME_DECIMATION` | 2 | additionally consider every 2nd source frame |
| `BACKOFF_FRAMES` | 3 | frames skipped after an overflow |
| `START_WAIT_CYCLES` | 4096 | wait for the previous trailer to finish |
| `FIFO_DEPTH` | 131072 | 16-bit async elastic FIFO (128 BSRAM, 256 KB) |

`cam_valid` is one RGB565 pixel in `cam_clk`; a rising `cam_vsync` closes the
previous frame.  FX2 contract is identical to `camera_fx2_stream`: external
48 MHz IFCLK, 16-bit AUTO-IN EP6 at FIFO address `2'b10`, FLAGC active high,
512-byte high-speed packets, guarded PKTEND on short frames.  SLWR/PKTEND are
driven from registered write windows, so the pad-to-pad path is only
FLAGC → one AND → pad rather than a FLAGC → state-decode → pad path.

Status outputs: `frames_sent` (CRC-valid frames), `frames_aborted` (bad-trailer
frames), `frames_dropped` (not admitted), `frames_malformed` (admitted but
source-raster/overflow failure), and sticky `fault`.

## Verification (2026-10-24)

- Icarus Verilog-2012 compile and Verilator 5.053 `--lint-only -Wall`: clean.
- Eight self-checking simulations: clean frames, frame decimation, overflow →
  backoff → recovery, truncated/extra-pixel frames, packet-aligned total,
  mid-frame reset, small 5/8 decimation, and a full 1024×600 → 640×375 frame at
  42 MHz camera / 50 MHz IFCLK.
- The full-size frame is 480,028 bytes including the FIMG v2 trailer; the Python
  host parser reports `crc_ok=1`, `header_errors=0`, `sequence_errors=0`.
- The full-size frame also passes with FLAGC duty 35% (~17.5 MB/s of write
  opportunities); a 64k-word FIFO passed only at 50% duty and overflowed at 40%.
  These are rate models, not physical USB measurements.  The testbench
  `pixels()` drives `cam_valid` almost every cam_clk, which is 2× the real DVP
  cadence (one pixel per 2 PCLK); it is deliberately conservative, not a
  fixed-FPS throughput proof.
- Standalone Gowin synthesis (1024×600, 5/8, FIFO 131072): **128 BSRAM, 1,836 LUT,
  1,196 registers**, no synthesis warnings.
- Integrated application synthesis: **251/340 BSRAM**, 31,340 LUT, 18,520
  registers; the direct transport contributes 128 BSRAM, 1,487 LUT and 863
  registers.  Inherited DDR3/IP dangling-net warnings remain; the direct module
  itself is warning-free.
- Full application PnR: **272/340 BSRAM**, 38,785 logic, 18,520 registers.
  `camera_pclk`, `i_clk_50m` and `clk_pixel` have zero setup/hold violations.
  `fx2_ifclk` does **not** close under the provisional 5 ns I/O budgets: worst
  Fmax 40.804 MHz vs 48 MHz, TNS -6.141 ns on two endpoints (worst
  `fx2_flagc -> fx2_pktend_n`, -3.674 ns).  Do not flash this bitstream without
  board-level FX2 timing measurement or a registered-output variant.

Evidence: `D:/HITSZ_FALCONS/ai部分/camera_direct_20261024/` (simulation,
crosscheck, synthesis) and `D:/HITSZ_FALCONS/camera_direct_candidate_20261024_r1/`
(full application candidate).

## Integration example

```verilog
camera_fx2_direct #(
    .SOURCE_WIDTH(1024), .SOURCE_HEIGHT(600),
    .FRAME_DECIMATION(2), .BACKOFF_FRAMES(3),
    .START_WAIT_CYCLES(4096), .FIFO_DEPTH(131072),
    .DECIM_NUM(5), .DECIM_DEN(8)
) u_camera_fx2_direct (
    .cam_clk(stream_clk), .ifclk(fx2_ifclk), .reset_n(sys_rst_n),
    .cam_vsync(stream_vsync), .cam_valid(stream_valid),
    .cam_rgb565(stream_rgb565),
    .fx2_flagc(fx2_flagc), .fx2_fd(fx2_fd), .fx2_faddr(fx2_faddr),
    .fx2_slwr_n(fx2_slwr_n), .fx2_sloe_n(fx2_sloe_n),
    .fx2_slrd_n(fx2_slrd_n), .fx2_pktend_n(fx2_pktend_n),
    .capture_busy(),
    .frames_sent(), .frames_dropped(), .frames_malformed(),
    .frames_aborted(), .fault()
);
```

The async FIFO uses gray-coded pointers and two-stage synchronizers.  Constrain
`camera_pclk` and `fx2_ifclk` as asynchronous clocks; keep the 5 ns I/O budget
as a provisional starting point, not a timing sign-off.

## Gates not yet closed

- PnR/timing closure and electrical measurement of FX2 IFCLK/FLAGC.
- FX2 firmware endpoint/flag identity and long-running USB CRC soak.
- Real OV5640 signal integrity and unchanged HDMI/display behavior.
- YOLO accuracy at 640×375 versus the 512×300 baseline.
- True 1024×600 1:1 needs a second DDR3 read port or a lower camera PCLK; it is
  not claimed by this direct path.

No FPGA image was flashed and no board model/firmware was loaded by this work.
