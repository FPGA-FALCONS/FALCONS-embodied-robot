"""Inspect M10_GPS 102-byte UART data without treating every marker as a frame.

Usage: python tools/analyze_m10_dump.py capture.bin
       python tools/analyze_m10_dump.py --hex capture.txt
Only Python's standard library is required.
"""

import argparse
from collections import Counter
from pathlib import Path
import re


HEADER = b"\xa5\x5a"
TAIL = b"\xfa\xfb"
FRAME_BYTES = 102
TAIL_SEARCH_BYTES = 512


def read_capture(path, hex_text=False):
    if not hex_text:
        return path.read_bytes()
    # Raw HEX text accepts whitespace, commas, 0x byte prefixes and # comments.
    # Address columns or ASCII columns from a formatted hexdump are not accepted.
    content = path.read_text(encoding="utf-8-sig")
    content = re.sub(r"(?m)#.*$", "", content)
    content = re.sub(r"0[xX](?=[0-9a-fA-F]{2})", "", content)
    compact = re.sub(r"[\s,]+", "", content)
    if re.search(r"[^0-9a-fA-F]", compact):
        raise ValueError("HEX input must contain raw hex bytes, not address/ASCII columns")
    if len(compact) % 2:
        raise ValueError("HEX input has an odd number of hex digits")
    return bytes.fromhex(compact)


def marker_offsets(data, marker):
    offsets = []
    start = 0
    while True:
        offset = data.find(marker, start)
        if offset < 0:
            return offsets
        offsets.append(offset)
        start = offset + 1


def inspect_capture(data):
    headers = marker_offsets(data, HEADER)
    intervals = Counter(b - a for a, b in zip(headers, headers[1:]))
    nearest_tail_offsets = Counter()
    no_nearby_tail = 0
    complete_102 = valid_102_tails = valid_102_angles = 0
    gps_slot_counts = Counter()
    for header in headers:
        # Search for a tail whose FIRST byte lies at offset 2 through 512.
        # Both bytes must be present in the capture.
        tail = data.find(TAIL, header + 2,
                         min(len(data), header + TAIL_SEARCH_BYTES + len(TAIL)))
        if tail < 0:
            no_nearby_tail += 1
        else:
            nearest_tail_offsets[tail - header] += 1
        if header + FRAME_BYTES <= len(data):
            complete_102 += 1
            frame = data[header:header + FRAME_BYTES]
            if frame[100:102] == TAIL:
                valid_102_tails += 1
                if int.from_bytes(frame[2:4], "big") <= 36000:
                    valid_102_angles += 1
                    distances = [int.from_bytes(frame[i:i+2], "big") for i in range(6, 90, 2)]
                    if distances[39] == 65535 or frame[86] >= 0xF5:
                        distances[40] = 65535
                    gps_slot_counts[sum(d != 65535 for d in distances)] += 1
    return {
        "headers": headers,
        "intervals": intervals,
        "nearest_tail_offsets": nearest_tail_offsets,
        "no_nearby_tail": no_nearby_tail,
        "complete_102": complete_102,
        "valid_102_tails": valid_102_tails,
        "valid_102_angles": valid_102_angles,
        "gps_slot_counts": gps_slot_counts,
    }


def print_histogram(title, histogram):
    print(title)
    if not histogram:
        print("  (none)")
        return
    for offset, count in sorted(histogram.items(), key=lambda item: (-item[1], item[0]))[:10]:
        print(f"  {offset:5d} bytes: {count}")


def print_report(path, data, result):
    headers = result["headers"]
    print(f"Capture: {path}")
    print(f"Total bytes: {len(data)}")
    print(f"A5 5A header candidates: {len(headers)}")
    print_histogram("Adjacent header distances (top 10):", result["intervals"])
    print_histogram("Nearest FA FB start offset after each header, 2..512 (top 10):",
                    result["nearest_tail_offsets"])
    print(f"Headers with no tail in that window: {result['no_nearby_tail']}")
    print(f"102-byte complete candidates: {result['complete_102']}")
    print(f"102-byte candidates with FA FB at offsets 100/101: {result['valid_102_tails']}")
    print(f"102-byte candidates with valid tail and start angle: {result['valid_102_angles']}")
    print(f"102-byte candidates with valid tail but invalid start angle: "
          f"{result['valid_102_tails'] - result['valid_102_angles']}")
    print(f"Header candidates too close to capture end for 102 bytes: "
          f"{len(headers) - result['complete_102']}")
    print("M10_GPS non-padding slot counts (zero distance still occupies a slot):")
    for slots, count in sorted(result['gps_slot_counts'].items()):
        print(f"  {slots} slots: {count} packets")
    print("First three header neighborhoods (at most 128 bytes each; absolute byte offsets):")
    if not headers:
        print("  (none)")
    for number, header in enumerate(headers[:3], 1):
        start = max(0, header - 16)
        end = min(len(data), start + 128)
        print(f"  Header {number}: offset {header} (0x{header:X}), bytes [{start}, {end})")
        for offset in range(start, end, 16):
            row = data[offset:min(end, offset + 16)]
            print(f"    {offset:08X}: {row.hex(' ').upper()}")
    print("NOTE: Payload bytes can coincidentally contain A5 5A or FA FB. These are")
    print("candidate statistics only. Verify several consecutive frames and the")
    print("actual protocol before drawing conclusions; this tool does not change it.")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze M10_GPS raw UART bytes using markers and fixed 102-byte candidates.")
    parser.add_argument("path", type=Path, help="raw binary capture path (default input format)")
    parser.add_argument("--hex", action="store_true",
                        help="read raw HEX text instead; whitespace, commas, 0x prefixes and # comments allowed")
    args = parser.parse_args()
    try:
        data = read_capture(args.path, args.hex)
    except (OSError, UnicodeError, ValueError) as error:
        parser.exit(2, f"error: {error}\n")
    print_report(args.path, data, inspect_capture(data))


if __name__ == "__main__":
    main()
