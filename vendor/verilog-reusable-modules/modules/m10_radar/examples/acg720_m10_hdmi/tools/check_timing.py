"""Require successful Gowin setup/hold/recovery/removal analysis after PnR.

Uses only Python's standard library. The generated-clock and path tables are
checked, so producing an .fs file alone is not treated as a successful build.
"""
from pathlib import Path
import html
import json
import re
import sys


def table(report, anchor):
    start = re.search(r'<a name="' + re.escape(anchor) + r'">', report)
    if start is None:
        raise ValueError(f"Missing timing section: {anchor}")
    remainder = report[start.end():]
    # Do not borrow a table from the next analysis when this section is empty.
    boundary = re.search(r"<h[123]\b[^>]*>", remainder)
    section = remainder[:boundary.start()] if boundary else remainder
    match = re.search(r"<table\b[^>]*>(.*?)</table>", section, re.S)
    if match is None:
        raise ValueError(f"Missing timing table: {anchor}")
    rows = []
    for row in re.findall(r"<tr\b[^>]*>(.*?)</tr>", match.group(1), re.S):
        # Gowin omits </td> after clock frequency; HTML permits implied ends.
        cells = re.findall(r"<td\b[^>]*>(.*?)(?=</td>|<td\b|$)", row, re.S)
        if cells:
            rows.append([html.unescape(re.sub(r"<[^>]*>", "", c)).strip() for c in cells])
    return rows


def main():
    root = Path(__file__).resolve().parent.parent
    report_path = root / "impl/pnr/acg720_m10_hdmi_tr_content.html"
    report = report_path.read_text(encoding="utf-8")
    # Official report contains named tables for the four timing analyses.
    summary = {"report": str(report_path), "min_slack_ns": {}}
    violations = []
    for analysis in ("Setup", "Hold", "Recovery", "Removal"):
        rows = table(report, f"{analysis}_Slack_Table")
        if not rows:
            raise ValueError(f"Empty {analysis} timing table")
        slacks = [float(row[1]) for row in rows]
        summary["min_slack_ns"][analysis.lower()] = min(slacks) if slacks else None
        for row, slack in zip(rows, slacks):
            if slack < 0:
                violations.append(f"{analysis}: {slack:.3f} ns, {row[2]} -> {row[3]}")
    tns = table(report, "Total_Negative_Slack_Report")
    if not tns:
        raise ValueError("Empty setup/hold TNS table")
    for row in tns:
        if float(row[2]) < 0 or int(row[3]) > 0:
            violations.append(f"TNS: {row}")
    # Check that the two PLL clocks were derived and constrained as expected.
    clocks = table(report, "Clock_Report")
    for port, frequency in (("CLKOUT0", 49.0), ("CLKOUT1", 245.0)):
        matches = [row for row in clocks if f"/PLL_inst/{port}.default_gen_clk" in row[1]]
        if len(matches) != 1 or matches[0][2] != "Generated" or abs(float(matches[0][4]) - frequency) > .001 or matches[0][8] != "clk_50m":
            raise ValueError(f"Missing/incorrect PLL {port} clock at {frequency} MHz")
    summary["pass"] = not violations
    (root / "tmp/timing_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if violations:
        print("FAIL Gowin timing:")
        print("\n".join(violations[:8]))
        return 1
    print("PASS Gowin timing: 49/245 MHz clocks, no negative setup/hold/recovery/removal slack")
    print(json.dumps(summary["min_slack_ns"]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, IndexError) as exc:
        print(f"FAIL timing report validation: {exc}")
        sys.exit(1)
