# TOF200C / VL53L0X verification

Executed 2026-10-17 with Icarus Verilog; production RTL is Verilog-2001, behavioral sensor model/testbenches use SystemVerilog.

| Check | Result |
|---|---|
| Production RTL `iverilog -g2001 -s tof_vl53l0x` | PASS |
| Verilator `--lint-only -Wall --top-module tof_vl53l0x` | PASS, no warnings or errors |
| Gowin EDA V1.9.12.03 synthesis, GW5AT-LV138PG484AC1/I0 | PASS; 50 MHz synthesis target; 1546 LUT + 14 ALU, 442 FF, 0 latches |
| I²C register master at 4 MHz / 100 kHz target | 85 checks, 0 failures |
| I²C register master at **50 MHz** / 100 kHz target | 85 checks, 0 failures |
| End-to-end controller at 4 MHz / 400 kHz target | 15 scenario groups, 104 test checks + 4001 sensor-model checks, 0 failures |
| Stale sample/recovery at 4 MHz | 4 test checks + 543 sensor-model checks, 0 failures |

Total: **4 simulations, 278 test checks + 4544 protocol-model checks, zero failures**. `sim/run_test.ps1` checks each compiler and simulator exit status; all test failures and watchdog expirations call `$fatal(1)` and return nonzero. The `.do` entry point is supplied but was not executed because this session used Icarus.

The register master test drives physical `tri1` open-drain SCL/SDA. It checks write and repeated-START read, final master NACK, missing acknowledgements at all four relevant byte phases, successful clock stretching, stuck SCL/SDA, in-flight excessive stretching, one response per command, and reset cancellation/recovery.

The controller test also drives physical `tri1` SCL/SDA/SHUT; it **does not replace, force, or stub internal command/response wires**. Its separately banked sensor model uses a nontrivial factory stop variable (0x6D), a 48-bit good-SPAD map containing holes, aperture and non-aperture cases, and an independent 80-pair reference tuning table. It verifies bank unlock for stop-variable reads/restores, exact tuning order, active-low IRQ read-modify-write preservation, VHV before phase calibration, restored single-shot state, raw range status interpretation, big-endian byte order, final NACKs, IRQ clear after status/high/low, and atomic publication only after clear succeeds.

End-to-end faults cover absent device, wrong model ID, stalled SPAD-ready, stalled calibration-ready, stuck START, stalled range-ready, insufficient factory map, zero factory SPAD count, final IRQ-clear NACK, initial SCL held low, initial SDA held low, retry after a live measurement fault, and external reset halfway through a physical byte. Fatal faults invalidate all data and hold SHUT low. The separate stale test confirms automatic invalidation while acquisition remains initialized, diagnostic retention of last distance, and automatic recovery on a new valid sample.

The core-only Gowin synthesis entry point is `scripts/run_synth.ps1`. It checks tool exit status, completion marker, and generated `global_freq=50.000` setting. The synthesis report estimates Fmax at 114.518 MHz; this is **not** routed timing signoff. Reports/netlist are under `build/synth/impl/gwsynthesis/`. That entry point does not use a CST or perform place-and-route.

## ACG720 SAFE/WARN/STOP board example

The new `examples/acg720_tof_debug` instantiates the complete real core, formatter and physical HC595 display, using verified adapter J7 SDA/E17, SCL/F16 and SHUT/D20 assignments. Its production Verilog-2001 compile passed. Formatter testing passed **8 groups / 656507 checks**, including all 65536 distances. The physical I²C + HC595 test passed **14 groups / 121301 test checks / 1768 sensor-model checks / 1376 latch words / 172 complete frames**. Final SAFE version tests cover 99/100/199/200 mm thresholds, invalid status, staleness/recovery, device NACK, reset, LEDs and heartbeat. Tests use a 4 MHz simulation clock; production remains 50 MHz.

The example's actual Gowin build passed synthesis and place-and-route at 50 MHz: **Fmax 101.482 MHz**, setup/hold violated endpoints both **0**, worst reported recovery/removal slack **+14.535 ns / +0.873 ns**. Recovery/removal global endpoint counts are not published; these are worst-path-table results. The build has a strict timing gate, freshness check and source/report/bitstream SHA256 manifest. `examples/acg720_tof_debug/VERIFICATION.md` records details. Integration lint passes with one narrowly scoped documented SYNCASYNCNET waiver for the intentional core reset synchronizer `sensor.reset_pipe`; no global warning suppression is applied.

These tests verify the digital implementation against a behavioral reference and routed internal timing for the specific example. They do not measure analog I²C rise time, sensor optical accuracy, or module voltage compatibility. The exact example bitstream was subsequently programmed successfully into ACG720 SRAM, and the user confirmed successful real ranging/display and SAFE/WARN/STOP operation. See docs/HARDWARE_TEST.md for evidence and limits. Other main projects must rerun place-and-route.
