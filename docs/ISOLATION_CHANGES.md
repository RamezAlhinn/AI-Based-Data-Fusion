# Pipeline Isolation Script — Improvements Summary

Changes made to `test_pipeline_isolation.py` and `.devcontainer/Dockerfile` to fix YOLO model loading and LiDAR point painting quality.

---

## 1. Correct YOLO Model Path

**Problem:** `YOLO('yolo11m-seg.pt')` resolved to the ultralytics download cache (`~/.config/Ultralytics/`), ignoring the manually downloaded model in the repo root. The Dockerfile was also pre-downloading a separate copy into the cache at build time.

**Fix (`test_pipeline_isolation.py`):** Resolve the model path relative to the script's own location:
```python
_script_dir = os.path.dirname(os.path.abspath(__file__))
YOLO_MODEL = os.path.join(_script_dir, 'yolo11m-seg.pt')
```

**Fix (`Dockerfile`):** Copy the downloaded model to `/workspace/yolo11m-seg.pt` at build time so the cache and workspace file are the same:
```dockerfile
RUN python3 -c "from ultralytics import YOLO; import shutil, os; m = YOLO('yolo11m-seg.pt'); \
    shutil.copy2(m.ckpt_path, '/workspace/yolo11m-seg.pt') \
    if os.path.abspath(m.ckpt_path) != '/workspace/yolo11m-seg.pt' else None"
```

---

## 2. Print Model Identity at Runtime

**Problem:** No way to confirm which model file was actually loaded.

**Fix:** Print the resolved path and model class immediately after loading:
```python
print(f'  Model file : {os.path.abspath(YOLO_MODEL)}')
print(f'  Model type : {type(model.model).__name__}')
```

---

## 3. Prevent YOLO Letterboxing

**Problem:** YOLO auto-resizes and pads input images internally, which offsets the output mask coordinates relative to the original image pixels, causing mask/point misalignment.

**Fix:** Pass the exact image dimensions so YOLO skips letterboxing:
```python
results = model(img_rgb, verbose=False, conf=0.25, imgsz=(h, w))
```

---

## 4. Cleaner Mask Boundaries

**Problem:** `INTER_NEAREST` resize produced blocky mask edges. Threshold `127` was too loose, letting blurry mask edges bleed into background regions.

**Fix:** Bilinear resize with a tighter threshold:
```python
mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_LINEAR)
label_mask[mask_resized > 200] = cls_id
```

---

## 5. Mask Dilation to Close Interior Holes

**Problem:** After the tighter threshold, small holes appeared inside masks (especially on the person), causing LiDAR points inside the object to miss the mask and render as grey (unpainted).

**Fix:** Dilate each class mask with a 7×7 elliptical kernel before point lookup:
```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
dilated = cv2.dilate(binary, kernel, iterations=1)
```

---

## 6. Majority-Vote Neighbourhood Lookup

**Problem:** Each LiDAR point looked up exactly one pixel `label_mask[v, u]`. Sub-pixel projection errors caused points near object edges to hit background pixels and render grey.

**Fix:** Sample a 3px radius neighbourhood and take the most common class:
```python
def majority_class(lm, cy, cx, r=3):
    patch = lm[max(0,cy-r):cy+r+1, max(0,cx-r):cx+r+1]
    valid = patch[patch >= 0]
    if len(valid) == 0:
        return -1
    return int(np.argmax(np.bincount(valid)))
```

---

## 7. Ground Point Filter

**Problem:** LiDAR ground returns (low-elevation points) scattered noisy coloured dots across the bottom of the image, some hitting YOLO car masks incorrectly.

**Fix:** Filter out points below the sensor mounting height before projection:
```python
GROUND_Z_THRESH = -1.5  # metres in Velodyne frame (z-up)
lidar_filtered = lidar_frame[lidar_frame[:, 2] > GROUND_Z_THRESH]
```

---

## 8. Depth-Sorted Rendering with Scaled Radius

**Problem:** All points drawn at a fixed radius regardless of distance. Far points were too small to fill scan-line gaps; near/far points drawn in arbitrary order caused incorrect occlusion.

**Fix:** Sort points back-to-front and scale radius by depth:
```python
order = np.argsort(depths)[::-1]  # far points drawn first
# radius: ~6px at 5m, ~4px at 10m, ~3px at 20m
radius = max(3, int(30.0 / max(depth, 1.0)))
```

---

## 9. Draw on Raw Image (No Double-Blend)

**Problem:** LiDAR points were drawn on top of `overlay` (which was already a blend of the image + mask). This double-applied colour in masked regions, making them look oversaturated.

**Fix:** Draw points directly on a copy of the raw image:
```python
lidar_img = img_frame.copy()  # was: overlay.copy()
```
