"""
Standalone test — no ROS runtime required.

Run with:  python3 src/point_painting/test/test_painting_node.py
"""
import sys
import os
import numpy as np

# Allow importing the package without installing it.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from point_painting.painting_logic import paint_points


def make_fake_points(n=10):
    """Return (n, 3) float32 array of [x, y, z] values."""
    rng = np.random.default_rng(42)
    xyz = rng.uniform(low=0.5, high=5.0, size=(n, 3)).astype(np.float32)
    return xyz


def make_fake_seg_image(height=200, width=200, fill_class=7):
    return np.full((height, width), fill_class, dtype=np.uint8)


def test_no_projector_skips_all_points():
    """Without a calibration file, paint_points should skip every point."""
    points = make_fake_points(10)
    seg = make_fake_seg_image()

    painted, skipped, class_ids = paint_points(points, seg)

    assert painted == 0, f"Expected 0 painted (no projector), got {painted}"
    assert skipped == 10, f"Expected 10 skipped, got {skipped}"
    assert all(c == -1 for c in class_ids), "All class_ids should be -1 without projector"

    print(f"  painted={painted}, skipped={skipped} (no projector — expected)")


def test_empty_point_cloud():
    """Empty point cloud should return immediately with zero counts."""
    points = np.zeros((0, 3), dtype=np.float32)
    seg = make_fake_seg_image()

    painted, skipped, class_ids = paint_points(points, seg)

    assert painted == 0
    assert skipped == 0
    assert class_ids == []

    print(f"  painted={painted}, skipped={skipped}, class_ids={class_ids}")


def test_class_ids_length_matches_input():
    """class_ids list must always have exactly N entries (one per input point)."""
    points = make_fake_points(15)
    seg = make_fake_seg_image()

    _, _, class_ids = paint_points(points, seg)

    assert len(class_ids) == 15, f"Expected 15 class_ids, got {len(class_ids)}"
    print(f"  len(class_ids)={len(class_ids)} (matches input)")


if __name__ == '__main__':
    tests = [
        test_no_projector_skips_all_points,
        test_empty_point_cloud,
        test_class_ids_length_matches_input,
    ]

    failures = []
    for t in tests:
        print(f"Running {t.__name__} ...")
        try:
            t()
            print(f"  PASSED\n")
        except Exception as e:
            print(f"  FAILED: {e}\n")
            failures.append((t.__name__, e))

    if failures:
        print("SOME TESTS FAILED:")
        for name, err in failures:
            print(f"  {name}: {err}")
        sys.exit(1)
    else:
        print("ALL TESTS PASSED")
