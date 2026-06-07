#!/usr/bin/env python3
"""
offline_mapper.py — Offline sparse 3D reconstruction pipeline.

Uses known camera poses from a recording manifest to triangulate 3D points
without estimating relative pose from images.

Commands:
  seed          Pick two frames with a wide baseline, extract SIFT features,
                match and triangulate to create the initial sparse map.
  visualize     Serve an interactive Three.js viewer (point cloud + frustums).
  expand        Add more frames from the recording to grow the map.
                (planned — Phase 2)
  bundle_adjust Jointly optimise all poses and 3D point positions.
                (planned — Phase 3)
  export_osa    Re-extract ORB features and write a .osa atlas file.
                (planned — Phase 4)

See MappingProject.md for full design documentation.
"""

import argparse
import base64
import json
import sys
from pathlib import Path

import cv2
import numpy as np


# ─── Camera ──────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, fx, fy, cx, cy, width, height, dist=None):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.width  = int(width)
        self.height = int(height)
        self.dist   = (np.zeros(4, dtype=np.float64)
                       if dist is None else np.array(dist, dtype=np.float64))

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0,       1      ]], dtype=np.float64)

    def to_dict(self) -> dict:
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
                "width": self.width, "height": self.height,
                "dist": self.dist.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "Camera":
        return cls(d["fx"], d["fy"], d["cx"], d["cy"],
                   d["width"], d["height"], d.get("dist"))

    @classmethod
    def from_yaml(cls, yaml_path) -> "Camera":
        fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise ValueError(f"Cannot open YAML: {yaml_path}")

        def real_or(name, default=0.0):
            n = fs.getNode(name)
            return float(n.real()) if n.isReal() else default

        fx = real_or("Camera1.fx") or real_or("Camera.fx")
        fy = real_or("Camera1.fy") or real_or("Camera.fy")
        cx = real_or("Camera1.cx") or real_or("Camera.cx")
        cy = real_or("Camera1.cy") or real_or("Camera.cy")
        w  = int(real_or("Camera.width",  1280))
        h  = int(real_or("Camera.height",  960))
        k1 = real_or("Camera1.k1")
        k2 = real_or("Camera1.k2")
        p1 = real_or("Camera1.p1")
        p2 = real_or("Camera1.p2")
        fs.release()
        return cls(fx, fy, cx, cy, w, h, [k1, k2, p1, p2])


# ─── Keyframe ────────────────────────────────────────────────────────────────

def _arr_to_b64(arr: np.ndarray) -> str:
    return base64.b64encode(arr.tobytes()).decode()

def _b64_to_arr(s: str, dtype, shape) -> np.ndarray:
    return np.frombuffer(base64.b64decode(s), dtype=dtype).reshape(shape)

def _kp_to_dict(kp: cv2.KeyPoint) -> dict:
    return {"x": kp.pt[0], "y": kp.pt[1], "size": kp.size,
            "angle": kp.angle, "response": kp.response, "octave": kp.octave}

def _dict_to_kp(d: dict) -> cv2.KeyPoint:
    return cv2.KeyPoint(d["x"], d["y"], d["size"], d["angle"],
                        d["response"], d["octave"])


class Keyframe:
    def __init__(self, id, recording_index, file, ts_ms, Twc, keypoints, descriptors):
        self.id               = int(id)
        self.recording_index  = int(recording_index)
        self.file             = file          # relative path e.g. "frames/000013.jpg"
        self.ts_ms            = float(ts_ms)
        self.Twc              = np.array(Twc, dtype=np.float64)   # 4×4 camera-to-world
        self.keypoints        = keypoints     # list of cv2.KeyPoint
        self.descriptors      = descriptors   # (N, 128) float32 SIFT

    @property
    def Tcw(self) -> np.ndarray:
        return np.linalg.inv(self.Twc)

    @property
    def camera_center(self) -> np.ndarray:
        return self.Twc[:3, 3]

    def to_dict(self) -> dict:
        return {
            "id":                 self.id,
            "recording_index":    self.recording_index,
            "file":               self.file,
            "ts_ms":              self.ts_ms,
            "Twc":                self.Twc.tolist(),
            "keypoints":          [_kp_to_dict(kp) for kp in self.keypoints],
            "descriptors_b64":    _arr_to_b64(self.descriptors),
            "descriptors_shape":  list(self.descriptors.shape),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Keyframe":
        kps  = [_dict_to_kp(k) for k in d["keypoints"]]
        desc = _b64_to_arr(d["descriptors_b64"], np.float32, d["descriptors_shape"])
        return cls(d["id"], d["recording_index"], d["file"], d["ts_ms"],
                   d["Twc"], kps, desc)


# ─── MapPoint ────────────────────────────────────────────────────────────────

class MapPoint:
    def __init__(self, id, pos, observations=None):
        self.id           = int(id)
        self.pos          = np.array(pos, dtype=np.float64)  # (3,) world position
        # observations: {kf_id (int) -> keypoint_index (int)}
        self.observations = dict(observations) if observations else {}

    def to_dict(self) -> dict:
        return {
            "id":           self.id,
            "pos":          self.pos.tolist(),
            "observations": {str(k): v for k, v in self.observations.items()},
        }

    @classmethod
    def from_dict(cls, d: dict) -> "MapPoint":
        obs = {int(k): v for k, v in d["observations"].items()}
        return cls(d["id"], d["pos"], obs)


# ─── SparseMap ───────────────────────────────────────────────────────────────

class SparseMap:
    def __init__(self, camera: Camera, recording_dir=None):
        self.camera        = camera
        self.recording_dir = Path(recording_dir) if recording_dir else None
        self.keyframes:  list[Keyframe]  = []
        self.mappoints:  list[MapPoint]  = []
        self._next_kf_id = 0
        self._next_mp_id = 0

    def add_keyframe(self, recording_index, file, ts_ms, Twc,
                     keypoints, descriptors) -> Keyframe:
        kf = Keyframe(self._next_kf_id, recording_index, file, ts_ms,
                      Twc, keypoints, descriptors)
        self._next_kf_id += 1
        self.keyframes.append(kf)
        return kf

    def add_mappoint(self, pos, observations: dict) -> MapPoint:
        mp = MapPoint(self._next_mp_id, pos, observations)
        self._next_mp_id += 1
        self.mappoints.append(mp)
        return mp

    def keyframe_by_id(self, kf_id: int):
        for kf in self.keyframes:
            if kf.id == kf_id:
                return kf
        return None

    def save(self, path):
        doc = {
            "version":       1,
            "recording_dir": str(self.recording_dir) if self.recording_dir else None,
            "camera":        self.camera.to_dict(),
            "next_kf_id":    self._next_kf_id,
            "next_mp_id":    self._next_mp_id,
            "keyframes":     [kf.to_dict() for kf in self.keyframes],
            "mappoints":     [mp.to_dict() for mp in self.mappoints],
        }
        with open(path, "w") as f:
            json.dump(doc, f, indent=2)
        print(f"Saved: {len(self.keyframes)} KFs, "
              f"{len(self.mappoints)} MPs → {path}")

    @classmethod
    def load(cls, path) -> "SparseMap":
        with open(path) as f:
            doc = json.load(f)
        cam = Camera.from_dict(doc["camera"])
        m   = cls(cam, doc.get("recording_dir"))
        m._next_kf_id = doc["next_kf_id"]
        m._next_mp_id = doc["next_mp_id"]
        m.keyframes   = [Keyframe.from_dict(d)  for d in doc["keyframes"]]
        m.mappoints   = [MapPoint.from_dict(d)  for d in doc["mappoints"]]
        return m

    def print_stats(self):
        obs_counts = [len(mp.observations) for mp in self.mappoints]
        print(f"  Keyframes : {len(self.keyframes)}")
        print(f"  MapPoints : {len(self.mappoints)}")
        if obs_counts:
            print(f"  Obs/point : min={min(obs_counts)} "
                  f"max={max(obs_counts)} "
                  f"mean={sum(obs_counts)/len(obs_counts):.1f}")


# ─── Utility: poses ──────────────────────────────────────────────────────────

def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion (scalar-last) → 3×3 rotation matrix."""
    x, y, z, w = float(qx), float(qy), float(qz), float(qw)
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),    1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),   1-2*(x*x+y*y)],
    ], dtype=np.float64)


def pose_to_Twc(pose: dict, unity_coords: bool = False) -> np.ndarray:
    """
    Convert manifest pose dict to 4×4 Twc (camera-to-world).

    When unity_coords=True, apply the Unity→ORB-SLAM3 axis conversion:
      t_orb = (x, -y, z),  q_orb = (-qx, qy, -qz, qw)
    """
    qx, qy, qz, qw = pose["qx"], pose["qy"], pose["qz"], pose["qw"]
    tx, ty, tz      = pose["x"],  pose["y"],  pose["z"]

    if unity_coords:
        ty  = -ty
        qx  = -qx
        qz  = -qz

    R   = quat_to_rot(qx, qy, qz, qw)
    t   = np.array([tx, ty, tz], dtype=np.float64)
    Twc = np.eye(4, dtype=np.float64)
    Twc[:3, :3] = R
    Twc[:3, 3]  = t
    return Twc


# ─── Utility: recording ──────────────────────────────────────────────────────

def load_recording(recording_dir) -> list:
    """Load manifest, return valid-pose frames sorted by timestamp."""
    manifest_path = Path(recording_dir) / "manifest.json"
    with open(manifest_path) as f:
        manifest = json.load(f)
    frames = [fr for fr in manifest["frames"] if fr.get("pose", {}).get("valid")]
    frames.sort(key=lambda fr: fr["ts_ms"])
    return frames


# ─── Utility: features ───────────────────────────────────────────────────────

def extract_sift(image_path, nfeatures: int = 5000):
    """Extract SIFT keypoints and descriptors from a JPEG file."""
    img = cv2.imread(str(image_path))
    if img is None:
        raise IOError(f"Cannot read image: {image_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create(nfeatures=nfeatures)
    kps, des = sift.detectAndCompute(gray, None)
    return kps, des.astype(np.float32)


def match_features(des1: np.ndarray, des2: np.ndarray,
                   ratio: float = 0.75):
    """
    FLANN-based kNN matching with Lowe's ratio test.
    Returns (idx1, idx2) arrays of matched descriptor indices.
    """
    index_params  = dict(algorithm=1, trees=5)   # FLANN_INDEX_KDTREE
    search_params = dict(checks=50)
    flann   = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    idx1, idx2 = [], []
    for pair in matches:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            idx1.append(m.queryIdx)
            idx2.append(m.trainIdx)
    return np.array(idx1, dtype=np.int32), np.array(idx2, dtype=np.int32)


def epipolar_filter(kps1, kps2, idx1, idx2,
                    K: np.ndarray, Tcw1: np.ndarray, Tcw2: np.ndarray,
                    epi_threshold: float = 1.0):
    """
    Filter matches using the epipolar constraint derived from known poses.
    Computes F = K^{-T} E K^{-1} where E = [t21]× R21, then keeps only
    matches where |pt2^T F pt1| < epi_threshold pixels.
    Returns filtered (idx1, idx2).
    """
    # Relative pose: Tcw2 * Twc1
    Twc1 = np.linalg.inv(Tcw1)
    T21  = Tcw2 @ Twc1                         # relative camera-to-camera
    R21  = T21[:3, :3]
    t21  = T21[:3, 3]

    # Skew-symmetric matrix [t21]×
    tx = np.array([[     0, -t21[2],  t21[1]],
                   [ t21[2],      0, -t21[0]],
                   [-t21[1],  t21[0],      0]])
    E = tx @ R21
    Ki = np.linalg.inv(K)
    F  = Ki.T @ E @ Ki

    pts1 = np.float64([kps1[i].pt for i in idx1])
    pts2 = np.float64([kps2[i].pt for i in idx2])

    # Sampson distance ≈ |x2^T F x1|^2 / (sum of first two squared entries of Fx1 and F^T x2)
    keep = []
    for k, (p1, p2) in enumerate(zip(pts1, pts2)):
        h1 = np.array([p1[0], p1[1], 1.0])
        h2 = np.array([p2[0], p2[1], 1.0])
        Fh1 = F @ h1
        Fth2 = F.T @ h2
        num  = (h2 @ Fh1) ** 2
        denom = Fh1[0]**2 + Fh1[1]**2 + Fth2[0]**2 + Fth2[1]**2
        sampson = num / (denom + 1e-10)
        if np.sqrt(abs(sampson)) < epi_threshold:
            keep.append(k)

    keep = np.array(keep, dtype=np.int32)
    return idx1[keep], idx2[keep]


# ─── Utility: geometry ───────────────────────────────────────────────────────

def build_projection(K: np.ndarray, Tcw: np.ndarray) -> np.ndarray:
    """3×4 projection matrix P = K [R|t] from world-to-camera Tcw."""
    return K @ Tcw[:3, :]


def triangulate_points(kps1, kps2, idx1, idx2,
                        K: np.ndarray, Tcw1: np.ndarray, Tcw2: np.ndarray
                       ) -> np.ndarray:
    """
    Triangulate matched keypoints.
    Returns pts3d of shape (N, 3) in world coordinates.
    """
    pts1 = np.float32([kps1[i].pt for i in idx1])
    pts2 = np.float32([kps2[i].pt for i in idx2])
    P1   = build_projection(K, Tcw1)
    P2   = build_projection(K, Tcw2)
    pts4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
    pts3d = (pts4d[:3] / pts4d[3]).T   # (N, 3)
    return pts3d


def reprojection_error(pt3d: np.ndarray, kp: cv2.KeyPoint,
                       K: np.ndarray, Tcw: np.ndarray) -> float:
    """Pixel reprojection error for one 3D point in one view."""
    p = Tcw[:3, :3] @ pt3d + Tcw[:3, 3]
    if p[2] <= 0:
        return np.inf
    px = K[0, 0] * p[0] / p[2] + K[0, 2]
    py = K[1, 1] * p[1] / p[2] + K[1, 2]
    return float(np.sqrt((px - kp.pt[0])**2 + (py - kp.pt[1])**2))


def filter_points(pts3d: np.ndarray, kps1, kps2, idx1, idx2,
                  K: np.ndarray, Tcw1: np.ndarray, Tcw2: np.ndarray,
                  max_reproj: float = 2.0,
                  min_depth:  float = 0.01,
                  max_depth:  float = 100.0) -> np.ndarray:
    """
    Return boolean mask over triangulated points.
    Keeps points that have positive bounded depth and low reprojection error
    in both views.
    """
    mask = np.zeros(len(pts3d), dtype=bool)
    for i, pt3d in enumerate(pts3d):
        # Depth in camera 1
        pc1 = Tcw1[:3, :3] @ pt3d + Tcw1[:3, 3]
        if pc1[2] < min_depth or pc1[2] > max_depth:
            continue
        # Depth in camera 2
        pc2 = Tcw2[:3, :3] @ pt3d + Tcw2[:3, 3]
        if pc2[2] < min_depth or pc2[2] > max_depth:
            continue
        # Reprojection error
        e1 = reprojection_error(pt3d, kps1[idx1[i]], K, Tcw1)
        e2 = reprojection_error(pt3d, kps2[idx2[i]], K, Tcw2)
        if e1 > max_reproj or e2 > max_reproj:
            continue
        mask[i] = True
    return mask


# ─── Command: seed ───────────────────────────────────────────────────────────

def cmd_seed(args):
    recording_dir = Path(args.recording_dir)
    frames = load_recording(recording_dir)
    if len(frames) < 2:
        print("ERROR: need at least 2 frames with valid poses", file=sys.stderr)
        sys.exit(1)
    print(f"Recording : {recording_dir}")
    print(f"Frames    : {len(frames)} with valid poses")

    camera = Camera.from_yaml(args.yaml)
    print(f"Camera    : fx={camera.fx:.1f} fy={camera.fy:.1f} "
          f"cx={camera.cx:.1f} cy={camera.cy:.1f} "
          f"{camera.width}×{camera.height}")
    K = camera.K

    unity = (args.coordsys == "unity")

    # ── Select seed pair ──────────────────────────────────────────────────────
    # Criteria (in priority order):
    #   1. Both cameras point within max_view_angle of each other
    #   2. Translation baseline in [min_baseline, max_baseline]
    #   3. Among qualifying pairs, maximise baseline
    Twcs = [pose_to_Twc(fr["pose"], unity) for fr in frames]

    def optical_axis(Twc):
        # Camera +Z in world frame = third column of Rwc
        return Twc[:3, 2] / np.linalg.norm(Twc[:3, 2])

    def view_angle_deg(Twc_a, Twc_b):
        cos_a = np.clip(optical_axis(Twc_a) @ optical_axis(Twc_b), -1.0, 1.0)
        return float(np.degrees(np.arccos(cos_a)))

    positions = [T[:3, 3] for T in Twcs]

    best_i, best_j, best_b = None, None, 0.0
    fallback_i, fallback_j, fallback_b = 0, 1, 0.0  # widest regardless of angle

    for i in range(len(frames)):
        for j in range(i + 1, len(frames)):
            b = float(np.linalg.norm(positions[j] - positions[i]))
            if b > fallback_b:
                fallback_b = b
                fallback_i, fallback_j = i, j
            if b < args.min_baseline or b > args.max_baseline:
                continue
            ang = view_angle_deg(Twcs[i], Twcs[j])
            if ang > args.max_view_angle:
                continue
            if b > best_b:
                best_b = b
                best_i, best_j = i, j

    if best_i is None:
        print(f"WARNING: no pair satisfies baseline [{args.min_baseline}, {args.max_baseline}] m "
              f"and view angle ≤ {args.max_view_angle}°.")
        print(f"  Falling back to widest pair ({fallback_b:.3f} m) — "
              f"view angle: {view_angle_deg(Twcs[fallback_i], Twcs[fallback_j]):.1f}°")
        best_i, best_j, best_b = fallback_i, fallback_j, fallback_b

    fr1, fr2 = frames[best_i], frames[best_j]
    Twc1, Twc2 = Twcs[best_i], Twcs[best_j]
    ang = view_angle_deg(Twc1, Twc2)
    print(f"\nSeed pair : frame #{fr1['index']} (t={fr1['ts_ms']:.0f} ms) "
          f"and frame #{fr2['index']} (t={fr2['ts_ms']:.0f} ms)")
    print(f"Baseline  : {best_b:.3f} m   view angle: {ang:.1f}°")
    Tcw1 = np.linalg.inv(Twc1)
    Tcw2 = np.linalg.inv(Twc2)

    # ── Extract SIFT ─────────────────────────────────────────────────────────
    print("\nExtracting SIFT features...")
    kps1, des1 = extract_sift(recording_dir / fr1["file"], args.nfeatures)
    kps2, des2 = extract_sift(recording_dir / fr2["file"], args.nfeatures)
    print(f"  Frame 1  : {len(kps1)} keypoints")
    print(f"  Frame 2  : {len(kps2)} keypoints")

    # ── Match ─────────────────────────────────────────────────────────────────
    print("\nMatching features (FLANN ratio={:.2f})...".format(args.ratio))
    idx1, idx2 = match_features(des1, des2, args.ratio)
    print(f"  Matches  : {len(idx1)} after ratio test")
    idx1, idx2 = epipolar_filter(kps1, kps2, idx1, idx2, K, Tcw1, Tcw2,
                                 args.epi_threshold)
    print(f"  Matches  : {len(idx1)} after epipolar filter (threshold={args.epi_threshold:.1f}px)")

    if len(idx1) < 8:
        print("ERROR: too few matches for triangulation (need ≥ 8)", file=sys.stderr)
        sys.exit(1)

    # ── Triangulate ───────────────────────────────────────────────────────────
    print("\nTriangulating...")
    pts3d = triangulate_points(kps1, kps2, idx1, idx2, K, Tcw1, Tcw2)
    mask  = filter_points(pts3d, kps1, kps2, idx1, idx2, K, Tcw1, Tcw2,
                          args.max_reproj, args.min_depth, args.max_depth)
    n_good = int(mask.sum())
    print(f"  Valid    : {n_good} / {len(mask)} points "
          f"({100*n_good/max(len(mask),1):.0f}%)")

    if n_good == 0:
        print("ERROR: no valid 3D points after filtering", file=sys.stderr)
        sys.exit(1)

    # Compute and report mean reprojection error on kept points
    errs = []
    for i, pt3d in enumerate(pts3d[mask]):
        e1 = reprojection_error(pt3d, kps1[idx1[mask][i]], K, Tcw1)
        e2 = reprojection_error(pt3d, kps2[idx2[mask][i]], K, Tcw2)
        errs.append((e1 + e2) / 2)
    print(f"  Reproj   : mean={np.mean(errs):.2f} px  "
          f"max={np.max(errs):.2f} px")

    # ── Build map ─────────────────────────────────────────────────────────────
    smap = SparseMap(camera, recording_dir)
    kf1  = smap.add_keyframe(best_i, fr1["file"], fr1["ts_ms"], Twc1, kps1, des1)
    kf2  = smap.add_keyframe(best_j, fr2["file"], fr2["ts_ms"], Twc2, kps2, des2)

    pts3d_good = pts3d[mask]
    idx1_good  = idx1[mask]
    idx2_good  = idx2[mask]
    for pt3d, i1, i2 in zip(pts3d_good, idx1_good, idx2_good):
        smap.add_mappoint(pt3d, {kf1.id: int(i1), kf2.id: int(i2)})

    # ── Save ──────────────────────────────────────────────────────────────────
    out_path = Path(args.output)
    smap.save(out_path)
    print()
    smap.print_stats()


# ─── Command: expand (Phase 2 stub) ──────────────────────────────────────────

def cmd_expand(args):
    print("expand: Phase 2 — not yet implemented")
    print("See MappingProject.md for the planned algorithm.")
    sys.exit(1)


# ─── Command: bundle_adjust (Phase 3 stub) ───────────────────────────────────

def cmd_bundle_adjust(args):
    print("bundle_adjust: Phase 3 — not yet implemented")
    print("See MappingProject.md for the planned algorithm.")
    sys.exit(1)


# ─── Command: export_osa (Phase 4 stub) ──────────────────────────────────────

def cmd_export_osa(args):
    print("export_osa: Phase 4 — not yet implemented")
    print("See MappingProject.md for the planned algorithm.")
    sys.exit(1)


# ─── Command: visualize ──────────────────────────────────────────────────────

from map_viewer import serve as _viewer_serve


def cmd_visualize(args):
    smap = SparseMap.load(args.map)
    smap.print_stats()
    html_dir = Path(__file__).parent.parent / "html"
    map_data = {
        "camera":    smap.camera.to_dict(),
        "keyframes": [
            {"id": kf.id, "file": kf.file, "ts_ms": kf.ts_ms,
             "Twc": kf.Twc.tolist()}
            for kf in smap.keyframes
        ],
        "mappoints": [
            {"id": mp.id, "pos": mp.pos.tolist(),
             "n_obs": len(mp.observations)}
            for mp in smap.mappoints
        ],
    }
    _viewer_serve(map_data, args.port, html_dir)


# ─── CLI ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Offline sparse 3D reconstruction pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # ── seed ──────────────────────────────────────────────────────────────────
    p = sub.add_parser("seed",
                       help="Two-frame seed reconstruction (Phase 1)")
    p.add_argument("recording_dir",
                   help="Recording directory containing manifest.json and frames/")
    p.add_argument("--yaml", required=True,
                   help="ORB-SLAM3 camera YAML (for intrinsics)")
    p.add_argument("--output", default="sparse_map.json",
                   help="Output map file (default: sparse_map.json)")
    p.add_argument("--coordsys", choices=["orbslam", "unity"], default="orbslam",
                   help="Coordinate system of recorded poses (default: orbslam); "
                        "use 'unity' for recordings made via Unity app")
    p.add_argument("--nfeatures", type=int, default=5000,
                   help="Max SIFT features per image (default: 5000)")
    p.add_argument("--ratio", type=float, default=0.75,
                   help="Lowe ratio test threshold (default: 0.75)")
    p.add_argument("--min-baseline", type=float, default=0.1, dest="min_baseline",
                   metavar="M",
                   help="Min camera baseline for seed pair in metres (default: 0.1)")
    p.add_argument("--max-baseline", type=float, default=5.0, dest="max_baseline",
                   metavar="M",
                   help="Max camera baseline for seed pair in metres (default: 5.0)")
    p.add_argument("--epi-threshold", type=float, default=2.0, dest="epi_threshold",
                   metavar="PX",
                   help="Sampson epipolar distance cutoff for match filtering (default: 2.0 px); "
                        "uses known poses to reject geometrically inconsistent matches")
    p.add_argument("--max-view-angle", type=float, default=2.0, dest="max_view_angle",
                   metavar="DEG",
                   help="Max angle between camera optical axes (default: 2°); "
                        "keeps only pairs pointing in nearly the same direction")
    p.add_argument("--max-reproj", type=float, default=2.0, dest="max_reproj",
                   metavar="PX",
                   help="Max reprojection error to keep a point (default: 2.0 px)")
    p.add_argument("--min-depth", type=float, default=0.01, dest="min_depth",
                   metavar="M",
                   help="Min triangulated point depth (default: 0.01 m)")
    p.add_argument("--max-depth", type=float, default=100.0, dest="max_depth",
                   metavar="M",
                   help="Max triangulated point depth (default: 100.0 m)")
    p.set_defaults(func=cmd_seed)

    # ── visualize ─────────────────────────────────────────────────────────────
    p = sub.add_parser("visualize",
                       help="Serve an interactive Three.js viewer for a sparse_map.json")
    p.add_argument("map", help="sparse_map.json to display")
    p.add_argument("--port", type=int, default=8765,
                   help="HTTP port to serve the viewer on (default: 8765)")
    p.set_defaults(func=cmd_visualize)

    # ── expand ────────────────────────────────────────────────────────────────
    p = sub.add_parser("expand",
                       help="Add more frames to the map (Phase 2 — planned)")
    p.add_argument("recording_dir", help="Recording directory")
    p.add_argument("--map", required=True, help="Existing sparse_map.json to extend")
    p.set_defaults(func=cmd_expand)

    # ── bundle_adjust ─────────────────────────────────────────────────────────
    p = sub.add_parser("bundle_adjust",
                       help="Optimise poses and 3D points (Phase 3 — planned)")
    p.add_argument("--map", required=True, help="sparse_map.json to optimise")
    p.set_defaults(func=cmd_bundle_adjust)

    # ── export_osa ────────────────────────────────────────────────────────────
    p = sub.add_parser("export_osa",
                       help="Write .osa atlas file (Phase 4 — planned)")
    p.add_argument("--map",   required=True, help="sparse_map.json")
    p.add_argument("--vocab", required=True, help="ORBvoc.txt vocabulary file")
    p.add_argument("--output", required=True, help="Output .osa path")
    p.set_defaults(func=cmd_export_osa)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
