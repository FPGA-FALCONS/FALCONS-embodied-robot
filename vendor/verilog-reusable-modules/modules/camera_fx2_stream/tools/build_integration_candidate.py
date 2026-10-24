"""Generate an isolated, pinned upstream camera->FX2 candidate; never edit checkout RTL."""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

BASE_COMMIT = "ec759b161f378fcf545a836fe8bed63295efe1a2"
PACKAGE = Path(__file__).resolve().parents[1]
PINS = {
    "fx2_ifclk": "L19", "fx2_flagc": "J17", "fx2_slcs_n": "J16",
    "fx2_pktend_n": "J14", "fx2_slrd_n": "K19", "fx2_slwr_n": "L20",
    "fx2_sloe_n": "J15", "fx2_faddr[1]": "H13", "fx2_faddr[0]": "K13",
}
PINS.update({f"fx2_fd[{15-i}]": p for i, p in enumerate(
    "H19 J19 K17 K18 L16 K16 L15 L14 G15 K14 G16 G13 J20 G20 G17 G18".split())})


def once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"Pinned integration anchor is not unique: {old!r}")
    return text.replace(old, new, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--mode", choices=("snapshot", "live"), default="snapshot",
                        help="snapshot=FIMG v1 frozen frame; live=FIM2 v2 fullresolution line stream")
    args = parser.parse_args()
    repo, out = args.repo.resolve(), args.out.resolve()
    if out.exists():
        raise SystemExit("Output exists; choose a NEW directory. Nothing overwritten.")
    if out == repo or repo in out.parents:
        raise SystemExit("Output must be outside the working checkout.")
    archive = subprocess.check_output(["git", "-C", str(repo), "archive", BASE_COMMIT])
    out.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tf:
        for member in tf.getmembers():
            target = (out / member.name).resolve()
            if out not in target.parents or member.issym() or member.islnk():
                raise RuntimeError(f"Unsafe archive entry: {member.name}")
        tf.extractall(out, filter="data")
    cst = (out / "constr/falcons.cst").read_text(encoding="utf-8-sig")
    used = {pin: port for port, pin in re.findall(r'IO_LOC\s+"([^"]+)"\s+([A-Z]+\d+)\s*;', cst)}
    conflicts = {pin: used[pin] for pin in PINS.values() if pin in used}
    if conflicts:
        raise RuntimeError(f"FX2 pin collision against pinned production CST: {conflicts}")
    target_pkg = out / "vendor/verilog-reusable-modules/modules/camera_fx2_stream"
    shutil.copytree(PACKAGE, target_pkg, ignore=shutil.ignore_patterns("__pycache__", "*.vvp", "*.bin", "*.hex"))

    cam_path = out / "rtl/cam/cam_overlay.v"
    cam = cam_path.read_text(encoding="utf-8-sig")
    cam = once(cam, "output wire        pll_lock\n", "output wire        pll_lock,\n    // Raw capture tap: PCLK domain, does not alter DDR or display paths.\n    output wire        stream_clk,\n    output wire        stream_vsync,\n    output wire        stream_valid,\n    output wire [15:0] stream_rgb565\n")
    cam = once(cam, "    wire cam_rst_n = sys_rst_n & pll_lock;", "    wire cam_rst_n = sys_rst_n & pll_lock;\n    assign stream_clk = camera_pixel_clk;\n    assign stream_vsync = camera_frame_sync;\n    assign stream_valid = camera_pixel_valid;\n    assign stream_rgb565 = camera_pixel_rgb565;")
    assert ".rfifo_rden          (pixel_req)" in cam
    cam_path.write_text(cam, encoding="utf-8")

    top_path = out / "rtl/top/falcons_top.v"
    top = top_path.read_text(encoding="utf-8-sig")
    ports = """    // FX2 EP6 synchronous Slave FIFO; firmware contract is mandatory.
    input  wire        fx2_ifclk,
    input  wire        fx2_flagc,
    output wire [15:0] fx2_fd,
    output wire [1:0]  fx2_faddr,
    output wire        fx2_slwr_n,
    output wire        fx2_sloe_n,
    output wire        fx2_slrd_n,
    output wire        fx2_pktend_n,
    output wire        fx2_slcs_n,

"""
    top = once(top, "module falcons_top (\n", "module falcons_top (\n" + ports)
    top = once(top, "    cam_overlay #(\n", "    wire stream_clk, stream_vsync, stream_valid;\n    wire [15:0] stream_rgb565;\n    cam_overlay #(\n")
    # Extend only this instance. Never touch camera physical pins or DDR rden.
    start = top.index("    cam_overlay #(\n")
    end = top.index("    );", start)
    instance = top[start:end]
    instance = once(instance, ".pll_lock", ".stream_clk(stream_clk),\n        .stream_vsync(stream_vsync),\n        .stream_valid(stream_valid),\n        .stream_rgb565(stream_rgb565),\n        .pll_lock")
    top = top[:start] + instance + top[end:]
    stream_instance = """
    // One independent downsampled RAM snapshot; transport cannot stall capture.
    assign fx2_slcs_n = 1'b0;
    camera_fx2_stream #(
        .SOURCE_WIDTH(1024), .SOURCE_HEIGHT(600), .DOWNSAMPLE(2)
    ) u_camera_fx2_stream (
        .cam_clk(stream_clk), .ifclk(fx2_ifclk), .reset_n(sys_rst_n),
        .cam_vsync(stream_vsync), .cam_valid(stream_valid), .cam_rgb565(stream_rgb565),
        .fx2_flagc(fx2_flagc), .fx2_fd(fx2_fd), .fx2_faddr(fx2_faddr),
        .fx2_slwr_n(fx2_slwr_n), .fx2_sloe_n(fx2_sloe_n), .fx2_slrd_n(fx2_slrd_n),
        .fx2_pktend_n(fx2_pktend_n), .capture_busy(),
        .frames_sent(), .frames_dropped(), .frames_malformed()
    );
"""
    if args.mode == "live":
        stream_instance = stream_instance.replace(
            "// One independent downsampled RAM snapshot; transport cannot stall capture.",
            "// Fullresolution direct forwarding with four line banks; no resize/frame cache.")
        stream_instance = stream_instance.replace("camera_fx2_stream #(\n        .SOURCE_WIDTH(1024), .SOURCE_HEIGHT(600), .DOWNSAMPLE(2)\n    ) u_camera_fx2_stream", "camera_fx2_live #(\n        .SOURCE_WIDTH(1024), .SOURCE_HEIGHT(600), .LINE_BANKS(4), .DESC_DEPTH(16)\n    ) u_camera_fx2_live")
    top = once(top, "\nendmodule", stream_instance + "\nendmodule")
    top_path.write_text(top, encoding="utf-8")
    deps = out / "deps/modules.list"
    deps.write_text(deps.read_text(encoding="utf-8-sig") + "\ncamera_fx2_stream\n", encoding="utf-8")
    cst_extra = ["// FX2 pins transcribed from manufacturer ch70 CST; original camera map retained."]
    for port, pin in PINS.items():
        cst_extra.extend([f'IO_LOC "{port}" {pin};', f'IO_PORT "{port}" IO_TYPE=LVCMOS33 PULL_MODE=NONE BANK_VCCIO=3.3;'])
    (out / "constr/camera_fx2.cst").write_text("\n".join(cst_extra) + "\n", encoding="utf-8")
    snapshot_cdc = (
        "// Only first synchronizer stages are timing exceptions; bundle remains bounded.\n"
        "set_false_path -to [get_pins {u_camera_fx2_stream/req_sync_0_s0/D u_camera_fx2_stream/ack_sync_0_s0/D}]\n"
        "set_max_delay -from [get_regs {" + " ".join(f"u_camera_fx2_stream/captured_sequence_{i}_s0" for i in range(32)) + "}] -to [get_regs {" + " ".join(f"u_camera_fx2_stream/usb_sequence_{i}_s0" for i in range(32)) + "}] 20.000\n")
    live_cdc = (
        "// Descriptor Gray pointers and bank acknowledgements cross through 2-stage synchronizers.\n"
        "// Targeted exceptions and Gray-bus skew bounds must be resolved against the final netlist at PnR.\n"
        "// No blanket clock grouping cuts the dualclock descriptor/line RAM paths.\n")
    (out / "constr/camera_fx2.sdc").write_text(
        "// Contract: FX2 firmware supplies 48 MHz synchronous IFCLK.\n"
        "create_clock -name fx2_ifclk -period 20.833 [get_ports {fx2_ifclk}]\n"
        "// Capture/USB CDC uses owner toggles and stable bundled RAM; never per-bit live sampling.\n"
        + (snapshot_cdc if args.mode == "snapshot" else live_cdc) +
        "// PROVISIONAL 5 ns I/O budget, not derived timing sign-off: verify FX2 electrical specs/board at PnR.\n"
        "set_input_delay -clock fx2_ifclk -max 5.000 [get_ports {fx2_flagc}]\n"
        "set_input_delay -clock fx2_ifclk -min 0.000 [get_ports {fx2_flagc}]\n"
        "set_output_delay -clock fx2_ifclk -max 5.000 [get_ports {fx2_fd[*] fx2_faddr[*] fx2_slwr_n fx2_pktend_n fx2_slcs_n fx2_sloe_n fx2_slrd_n}]\n"
        "set_output_delay -clock fx2_ifclk -min 0.000 [get_ports {fx2_fd[*] fx2_faddr[*] fx2_slwr_n fx2_pktend_n fx2_slcs_n fx2_sloe_n fx2_slrd_n}]\n", encoding="utf-8")
    files = ["rtl/top/falcons_top.v", "rtl/cam/cam_overlay.v", "deps/modules.list", "constr/camera_fx2.cst", "constr/camera_fx2.sdc"]
    manifest = {"base_commit": BASE_COMMIT, "mode": args.mode, "working_checkout_modified": False,
                "pin_collisions": conflicts, "fx2_pins": PINS,
                "notes": ["L13 fx2_clear not driven; no FX2 reset/firmware manipulation", "No board flashing or physical transfer claimed", "Original camera pin map and DDR continuous pixel_req unchanged"],
                "changed_files_sha256": {p: hashlib.sha256((out/p).read_bytes()).hexdigest() for p in files}}
    (out / "camera_fx2_integration_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"candidate": str(out), **manifest}, indent=2))


if __name__ == "__main__":
    main()
