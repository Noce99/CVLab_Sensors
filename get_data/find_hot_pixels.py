"""List the pixels producing a large share of the events of a recording.

Usage: find_hot_pixels.py <file.raw> [--seconds 5] [--min-share 0.01]
Record a few seconds of a static scene, then copy the result into
cvlab/config.py EVENT_HOT_PIXELS for that camera.
"""

import argparse

import numpy as np
from metavision_core.event_io import EventsIterator


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--min-share", type=float, default=0.01, help="Minimum fraction of all events")
    args = parser.parse_args()

    iterator = EventsIterator(args.raw, delta_t=100_000, max_duration=int(args.seconds * 1e6))
    height, width = iterator.get_size()
    counts = np.zeros(width * height, dtype=np.int64)
    total = 0
    for evs in iterator:
        counts += np.bincount(evs["y"].astype(np.int64) * width + evs["x"], minlength=width * height)
        total += len(evs)

    hot = [i for i in np.argsort(counts)[::-1][:64] if counts[i] >= args.min_share * total]
    print(f"{total} events in {args.seconds}s, {len(hot)} hot pixels:")
    for i in hot:
        print(f"  ({i % width}, {i // width})  {100 * counts[i] / total:.1f}% of all events")
    print(f"\nEVENT_HOT_PIXELS entry: {[(int(i % width), int(i // width)) for i in hot]}")


if __name__ == "__main__":
    main()
