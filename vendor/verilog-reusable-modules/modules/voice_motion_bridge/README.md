# Voice motion bridge (MOT1)

Camera-independent bounded motion primitives from RK3588, separate from the
FAM1 high-level mission protocol. This module has no command acknowledgment:
a CRC-valid received frame or host serial write is **not execution evidence**.

```text
Chinese text -> host/semantic_nav -> MOT1 byte router -> voice_motion_bridge
                                                -> voice_nav_arbiter -> chassis_ctrl
physical manual / estop / ToF / planner / RC-loss safety --------^ higher priority
```

## Wire contract and limits

12 bytes: `MOT1 + seq:u8 + cmd:u8 + value:i16BE + speed:u8 + flags:u8 + CRC16BE`.
CRC-16/CCITT-FALSE covers the first 10 bytes. Reserved flags are retained as
telemetry but do not authorize extra behavior; sequence is telemetry, not a
replay-protection or receipt protocol. Host codec: `host/semantic_nav/mot1.py`.

| cmd | Accepted primitive | value |
|---|---|---|
| 0 | Stop and retain zero ownership | ignored |
| 1 / 2 | Forward / backward | 1..5000 ms; rejected outside range |
| 3 / 4 | Left / right | positive 1..360 degrees; zero/negative/outside rejected |
| 5 | Spin | -360..-1 or 1..360 degrees; positive clockwise |
| 6 | Request patrol start and release ownership | accepted only with `i_nav_release_ready` |
| 7 | Patrol stop pulse and retain zero ownership | ignored |
| 8 | Heartbeat | refresh admitted command watchdog only |

Motion commands require nonzero speed. Forward/backward center magnitude is
`min(speed*4, MAX_CENTER)`; turn magnitude is the fixed `TURN_DIFF` parameter.
Host language parsing normally imposes a stricter 5-degree minimum. RTL checks
are independent: malformed binary commands cannot bypass its 360-degree limit.
Unknown opcodes, bad CRC, invalid geometry/duration/speed and stale-IMU turn
requests do not start a new action or refresh the watchdog. They leave an
already admitted action subject to its existing duration, watchdog and safety.
A refused patrol release instead takes/retains ownership and stops motion.

`BYTE_TIMEOUT_MS=20` resets a partial header/body after that many milliseconds
without a byte, reports an error, and admits the next complete MOT1 frame.
Match the upstream byte-router timeout. There is no partial-frame execution.
`o_last_*` records the last CRC-valid wire frame, including semantic rejects;
use it only for diagnostics, never as an execution acknowledgment.

## Ownership and safety

`o_owned` is persistent authority for the voice mux, distinct from `o_active`.
`o_active` means a drive or turn is in progress. Stop, normal completion,
watchdog, IMU fault, timeout, and patrol stop keep ownership with zero center
and diff. The arbiter must select voice based on **owned**, not active, so a
stale navigation command cannot resume after a voice action stops.

By default, only an explicit admitted PATROL_START with `i_nav_release_ready=1` releases
ownership and pulses `o_patrol_start`. Raising ready alone or sending a
heartbeat never releases it. Reset clears ownership. The ready input must
mean navigation can safely accept a fresh start, not merely that a stale
navigation command happens to be valid.

The whole-vehicle application can set `LOCAL_RELEASE_EN=1` and pulse
`i_local_release` for a freshly confirmed local FPGA navigation start. This
handoff requires voice disabled, idle motion, no manual takeover and no estop.
Disconnect or recovery alone must never generate this pulse. With this option,
an offline STOP cannot acquire voice ownership when the bridge does not already
own control; this preserves original navigation after an explicit local handoff.
Default `LOCAL_RELEASE_EN=0` retains the original module contract. The local
handoff is covered by the application integration regression, separate from the
71-check default-module regression and whole-vehicle hardware validation.

`i_enable && !i_estop && !i_manual` masks outputs immediately. At the next
clock the old action is canceled; restoring safety cannot restart it. Keep
RC/manual, physical/key estop, ToF, planner and stall safety outside/above the
voice path. With the default ownership contract, STOP/PATROL_STOP received during a safety override still acquire persistent zero
ownership (no motion, startup pulse or watchdog refresh); unsafe motion requests
are ignored. During a safety override `o_auto_valid=0`; the top-level arbiter
and chassis safety chain retain their independent authority.

A newly admitted action has priority over old-action millisecond completion,
yaw update and timeout in the same clock. A heartbeat refreshes watchdog but
does not extend duration or turn timeout. Default watchdog is 500 ms.

## Real IMU sample contract

Inputs added for integration:

- `i_imu_sample_valid`: one clock pulse for an actual coherent yaw update.
- `i_imu_healthy`: initialization/calibration/decoder health; fail closed.
- `i_yaw_valid`: yaw value usable, in addition to health and sample freshness.
- `i_nav_release_ready`: explicit navigation handoff readiness.
- `o_owned`: persistent voice ownership for the auto-source arbiter.

The bridge stores sample freshness internally. A latched yaw-valid flag cannot
keep a sensor fresh. Turn admission requires a healthy actual sample within
200 ms, yaw within [-180,180], and valid status. Loss of health/valid/range
stops an active turn immediately at the next clock; 200 ms without an actual
sample also stops it. Forward/backward do not require yaw.

`YAW_SIGN=1` expects mathematical yaw: left/CCW increases, right/CW decreases.
`YAW_SIGN=0` reverses the input convention. Accumulation occurs only on actual
sample pulses, unwraps +/-180 crossings, and compares signed angles; a reverse
one-degree disturbance cannot become an unsigned completion. Turn/spin also
stop at `TIMEOUT_MS`. This does not compensate gyro drift or wheel slip.

## Verification and hardware limits

2026-10-24 release regression: Icarus `-g2012 -Wall`, 71 self-checks PASS;
Verilator 5.053 `--lint-only -Wall`, clean. Testbench:
`sim/tb_voice_motion_bridge.sv`. Covers persistent zero ownership, explicit
handoff/refusal, speed/angle/duration limits, CRC/unknown/semantic reject
watchdogs, heartbeat, actual IMU freshness/health, +/-180 wraps, reverse
perturbations, both 360-degree directions, turn timeout, safety cancellation,
new-action/tick collision, and truncated-frame recovery.

These are RTL simulation and lint results, not wheel/motor or whole-vehicle
hardware validation. Existing standalone synthesis counts from earlier
versions do not describe this revised module. Build the whole-vehicle design,
check pin mapping/timing, then perform SRAM-only wheels-up testing with an
independent estop before low-speed floor tests. This module alone is not a
complete navigation stack or proof of exact physical distance/angle.
