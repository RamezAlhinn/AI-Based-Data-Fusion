"""
Standalone test — no ROS runtime required.

Run with:  python3 src/point_painting/test/test_painting_node.py
"""
import sys
import os
import numpy as np

# Allow importing the package without installing it.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from point_painting.painting_logic import project_point_to_pixel, paint_points


def make_fake_points(n=10):
    """Return (n, 3) float32 array of [x, y, z] values."""
    rng = np.random.default_rng(42)
    xyz = rng.uniform(low=0.5, high=5.0, size=(n, 3)).astype(np.float32)
    return xyz


def make_fake_seg_image(height=200, width=200, fill_class=7):
    """Return a 2-D segmentation label image filled with fill_class.

    Must be at least 101x101 so the stub projection (100,100) is in-bounds.
    """
    return np.full((height, width), fill_class, dtype=np.uint8)


def test_all_points_get_class_id():
    points = make_fake_points(10)
    seg = make_fake_seg_image(fill_class=7)

    painted, skipped, class_ids = paint_points(points, seg)

    assert painted == 10, f"Expected 10 painted, got {painted}"
    assert skipped == 0, f"Expected 0 skipped, got {skipped}"
    assert len(class_ids) == 10, f"Expected 10 class_ids, got {len(class_ids)}"

    for i, cid in enumerate(class_ids):
        assert cid == 7, f"Point {i}: expected class_id=7, got {cid}"

    print(f"  painted={painted}, skipped={skipped}")
    print(f"  class_ids={class_ids}")


def test_stub_projection_returns_valid_pixel():
    u, v = project_point_to_pixel(1.0, 2.0, 5.0)
    assert isinstance(u, int) and isinstance(v, int), "pixel coords must be ints"
    print(f"  project_point_to_pixel(1.0, 2.0, 5.0) -> ({u}, {v})")


def test_out_of_bounds_points_are_skipped():
    """Force out-of-bounds by using a tiny 50x50 image; stub returns (100,100)."""
    points = make_fake_points(5)
    tiny_seg = np.full((50, 50), 3, dtype=np.uint8)
    painted, skipped, class_ids = paint_points(points, tiny_seg)

    assert skipped == 5, f"Expected 5 skipped (out-of-bounds), got {skipped}"
    assert painted == 0, f"Expected 0 painted, got {painted}"
    assert all(c == -1 for c in class_ids), "Out-of-bounds points should have class_id=-1"
    print(f"  painted={painted}, skipped={skipped} (all out of 50x50 bounds as expected)")


if __name__ == '__main__':
    tests = [
        test_stub_projection_returns_valid_pixel,
        test_all_points_get_class_id,
        test_out_of_bounds_points_are_skipped,
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
