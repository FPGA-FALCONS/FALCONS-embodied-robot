# Camera → FX2 EP6 stream

This reusable transport taps the existing OV5640 RGB565 capture stream. It does not replace camera configuration, the DDR3 display buffer, UI, or motion control.

```
OV5640 → existing ov5640_capture ─┬→ existing DDR3 → HDMI/UI
                                └→ camera_fx2_stream RAM → FX2 EP6 → RK3588 USB host
```

## Contract

`cam_valid` is one complete RGB565 pixel in `cam_clk`; a rising `cam_vsync` closes the preceding frame and starts the next opportunity. Boundary pixels are deliberately ignored. Exactly `SOURCE_WIDTH*SOURCE_HEIGHT` valid pixels must appear between boundaries; incomplete and extra-pixel frames are rejected. Linear valid counts establish coordinates, independent of the existing DVP module's one-based x/y outputs. Dimensions must divide `DOWNSAMPLE` exactly.

The default 1024×600 source is sampled at even x/even y to 512×300. This is nearest-neighbor sampling, **not** an image-quality guarantee: small screwdrivers can disappear after reduction, and real camera/YOLO acceptance is still required. A single complete frozen frame is used. Capture never backpressures the camera or its display path; occupied-buffer frames are dropped whole. The held request/ack toggles cross through two-stage synchronizers; RAM and sequence remain stable until acknowledge. Both domains require the same reset assertion; reset release is synchronized separately.

### FIMG v1 wire format

24-byte little-endian header (`<4sBBHHHIII`): magic `FIMG`, version 1, format 1 (RGB565 LE), header length 24, width, height, uint32 sequence, payload bytes, uint32 CRC32. CRC32 is zlib/IEEE over the first 20 header bytes concatenated with the complete payload. Each RGB565 word is sent low byte first. Sequence starts at zero after reset; a receiver must treat reset as a new session rather than ordinary monotonic traffic. CRC is calculated in an IFCLK prepass over frozen RAM before transmitting the header.

### Mandatory FX2 firmware contract

- The FX2 firmware supplies synchronous 48 MHz IFCLK to FPGA; FPGA does not generate IFCLK.
- EP6 is a 16-bit Bulk IN AUTO-IN endpoint, FIFO address `2'b10`, 512-byte high-speed packets.
- FLAGC is configured active high when EP6 can accept a write. Flag mapping/polarity must be verified from firmware; USB descriptors alone do not prove them.
- FD is a write-only FPGA output; SLOE and SLRD stay inactive, SLCS is active. Do not run firmware that makes FX2 drive FD while this design is active.
- SLWR only asserts when writable. Data is prepared at least one IFCLK cycle before each accepted word and is held across full-FIFO stalls.
- A short final packet uses one accepted PKTEND pulse after a guard cycle; packet-aligned frame lengths use no PKTEND. PKTEND also waits while FLAGC is low.
- No FX2 firmware download, reset signal, EEPROM write, or `fx2_clear` manipulation is performed by this module. L13 and unused FLAGB remain unassigned.

Primary reference: [Infineon AN61345](https://www.infineon.com/assets/row/public/documents/24/42/infineon-an61345-designing-with-ez-usb-fx2lp-slave-fifo-interface-applicationnotes-en.pdf), [FX2 Technical Reference Manual](https://www.infineon.com/assets/row/public/documents/24/44/infineon-ez-usb-fx2-trm-version2-2-usermanual-en.pdf). Board pins are transcribed from manufacturer ACG720 chapter 70 `fx2_stream_in_ov5640_rgb.cst`; **do not copy that example's different camera pin mapping**.

## Safe isolated integration

The dirty local production checkout is older than upstream. `tools/build_integration_candidate.py` exports exactly upstream commit `ec759b161f378fcf545a836fe8bed63295efe1a2` into a **new directory outside the checkout**, copies this package, and adds capture taps, the streamer instance, dependencies and constraint fragments. It refuses overwrites and checks every added FX2 pin against upstream CST. It preserves existing camera pins and continuous DDR `.rfifo_rden(pixel_req)`.

```powershell
python vendor/verilog-reusable-modules/modules/camera_fx2_stream/tools/build_integration_candidate.py --repo . --out D:/FALCONS_workspace/camera_fx2_candidate_new
```

Use an ASCII output path for the installed Gowin build, which fails on non-ASCII working paths. Generation is **not flashing**. Firmware contract, external interface timing, PnR, physical USB reception, frame identity and detection quality remain separate acceptance gates. SDC's 5 ns I/O budgets are provisional, not derived electrical limits or timing sign-off; actual FX2 specifications, phase and board timing must determine accepted constraints. Synthesis is not timing closure.

## Verification (2026-10-24)

- Verilog-2001 Icarus compilation and Verilator `--lint-only -Wall`: clean.
- Three standalone self-checking simulations: sampled output 4×3, multi-packet 32×16, exactly 512-byte total 244×1. Each checks headers, CRC and pixels, repeated frames, random/long stalls, frozen RAM, busy-frame drops, truncated/extra frames, boundary valid pixels, capture/transfer resets, short-packet guard/full stall, and packet-aligned no-PKTEND.
- Actual sampled FD `wire.bin`/`wire.hex` vectors are independently decoded by the Python host receiver; all nine complete frames match. A deliberately interrupted ten-byte prefix remains in each stream.
- Default standalone Gowin synthesis: 160/340 BSRAM, 711 LUT + 124 ALU, 410 registers. Transport read/CRC/write state cycles take about 22.4 ms at 48 MHz excluding capture intervals and USB stalls; this is **not measured video FPS**.
- Isolated upstream candidate synthesis: 283/340 BSRAM, 30,348 LUT + 7,091 ALU, 17,770 registers. Inherited application/IP synthesis warnings remain (including `nav_safety` swept); this is not a warning-free production sign-off. The final targeted CDC SDC replaces broad clock cuts; its hierarchy names are checked against the netlist, while PnR parsing/closure and external timing remain pending.
- Local application regression at local HEAD a3cfcd8: all 13 tests pass (does not verify the separate upstream candidate).

Evidence is under `D:/FALCONS_workspace/ai部分/camera_fx2_20261024/`. No hardware transfer or board update is claimed.
