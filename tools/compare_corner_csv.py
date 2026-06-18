#!/usr/bin/env python3
"""Compare two Ceres corner CSV files by timestamp and corner id."""

import argparse
import csv
import math
import statistics
from bisect import bisect_left
from collections import defaultdict


def read_corners(path, match_key, target_round_digits):
    corners = {}
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or not row[0] or not row[0][0].isdigit():
                continue
            if len(row) < 4:
                continue
            if match_key == "target":
                if len(row) < 7:
                    continue
                tail = tuple(round(float(value), target_round_digits) for value in row[4:7])
            else:
                tail = (int(row[1]),)
            key = (int(row[0]),) + tail
            corners[key] = (float(row[2]), float(row[3]))
    return corners


def group_timestamps(corners):
    frames = defaultdict(dict)
    for key, pixel in corners.items():
        frames[key[0]][key[1:]] = pixel
    return frames


def nearest_timestamp(timestamp_ns, sorted_timestamps, tolerance_ns):
    index = bisect_left(sorted_timestamps, timestamp_ns)
    candidates = []
    if index < len(sorted_timestamps):
        candidates.append(sorted_timestamps[index])
    if index > 0:
        candidates.append(sorted_timestamps[index - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda value: abs(value - timestamp_ns))
    if abs(best - timestamp_ns) <= tolerance_ns:
        return best
    return None


def percentile(values, ratio):
    if not values:
        return float("nan")
    index = min(len(values) - 1, max(0, int(round((len(values) - 1) * ratio))))
    return sorted(values)[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--label", default="corner_csv")
    parser.add_argument("--timestamp-tolerance-ns", type=int, default=0)
    parser.add_argument("--match-key", choices=["corner-id", "target"], default="corner-id")
    parser.add_argument("--target-round-digits", type=int, default=6)
    args = parser.parse_args()

    reference = read_corners(args.reference, args.match_key, args.target_round_digits)
    candidate = read_corners(args.candidate, args.match_key, args.target_round_digits)
    matched_timestamps = 0
    if args.timestamp_tolerance_ns > 0:
        reference_frames = group_timestamps(reference)
        candidate_frames = group_timestamps(candidate)
        reference_timestamps = sorted(reference_frames.keys())
        common_keys = []
        remapped_candidate = {}
        for candidate_timestamp, frame in candidate_frames.items():
            reference_timestamp = nearest_timestamp(
                candidate_timestamp,
                reference_timestamps,
                args.timestamp_tolerance_ns,
            )
            if reference_timestamp is None:
                continue
            matched_timestamps += 1
            for key_tail, pixel in frame.items():
                key = (reference_timestamp,) + key_tail
                remapped_candidate[key] = pixel
                if key in reference:
                    common_keys.append(key)
        candidate_for_compare = remapped_candidate
    else:
        common_keys = sorted(reference.keys() & candidate.keys())
        candidate_for_compare = candidate
    distances = []
    for key in common_keys:
        ref = reference[key]
        cand = candidate_for_compare[key]
        distances.append(math.hypot(cand[0] - ref[0], cand[1] - ref[1]))

    if distances:
        mean = statistics.fmean(distances)
        rms = math.sqrt(statistics.fmean([value * value for value in distances]))
        median = statistics.median(distances)
        p95 = percentile(distances, 0.95)
        max_value = max(distances)
    else:
        mean = rms = median = p95 = max_value = float("nan")

    print(f"label,{args.label}")
    print(f"match_key,{args.match_key}")
    print(f"reference_rows,{len(reference)}")
    print(f"candidate_rows,{len(candidate)}")
    print(f"common_rows,{len(common_keys)}")
    print(f"matched_timestamps,{matched_timestamps}")
    print(f"reference_only,{len(reference) - len(common_keys)}")
    print(f"candidate_only,{len(candidate) - len(common_keys)}")
    print(f"mean_px,{mean:.9f}")
    print(f"median_px,{median:.9f}")
    print(f"rms_px,{rms:.9f}")
    print(f"p95_px,{p95:.9f}")
    print(f"max_px,{max_value:.9f}")


if __name__ == "__main__":
    main()
