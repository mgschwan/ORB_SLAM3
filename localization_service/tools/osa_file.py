"""
osa_file.py — Read and write ORB-SLAM3 Atlas (.osa) files from Python.

The module uses the ``osa_convert`` C++ tool (built alongside the project)
to serialise/deserialise the Boost binary archive format that ORB-SLAM3 uses.

Quick start
-----------
Load an existing atlas::

    from osa_file import OsaAtlas
    atlas = OsaAtlas.load("my_map.osa")
    for m in atlas.maps:
        print(f"Map {m.id}: {len(m.keyframes)} KFs, {len(m.mappoints)} MPs")
        for mp in m.mappoints:
            print(mp.world_pos)  # numpy array [x, y, z]

Round-trip (load, optionally modify, save)::

    atlas = OsaAtlas.load("my_map.osa")
    atlas.save("copy.osa")

Build a new atlas from scratch::

    atlas = OsaAtlas.new("ORBvoc.txt", "abc123checksum")
    m = OsaMap(id=0)
    kf = OsaKeyFrame(id=0, timestamp=1.0, tcw=np.eye(4), ...)
    m.keyframes.append(kf)
    atlas.maps.append(m)
    atlas.save("new_map.osa")

Binary data
-----------
Descriptors are ``numpy.ndarray`` of dtype ``uint8`` with shape
``(N, 32)`` for ORB descriptors.  KeyPoint coordinates, world positions,
and normal vectors are ``float32`` numpy arrays.
"""

from __future__ import annotations

import base64
import json
import os
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# ─── locate the osa_convert binary ────────────────────────────────────────────

def _find_osa_convert() -> str:
    """Return path to the osa_convert binary."""
    candidates = [
        Path(__file__).parent / "osa_convert",
        Path(__file__).parent.parent.parent / "build" / "osa_convert",
    ]
    for p in candidates:
        if p.exists() and os.access(str(p), os.X_OK):
            return str(p)
    raise FileNotFoundError(
        "osa_convert binary not found. Build the project first:\n"
        "  cd <ORB_SLAM3_root> && mkdir -p build && cd build && cmake .. && make -j osa_convert"
    )


# ─── Low-level JSON ↔ numpy helpers ───────────────────────────────────────────

def _mat_from_json(j: dict) -> np.ndarray:
    """Decode a cv::Mat encoded as {cols, rows, type, data(base64)}."""
    raw = base64.b64decode(j["data"])
    rows, cols = j["rows"], j["cols"]
    # OpenCV type: lower 3 bits are depth, upper bits are channels
    depth = j["type"] & 7
    chans = (j["type"] >> 3) + 1
    dtype_map = {0: np.uint8, 1: np.int8, 2: np.uint16, 3: np.int16,
                 4: np.int32, 5: np.float32, 6: np.float64}
    dtype = dtype_map.get(depth, np.uint8)
    arr = np.frombuffer(raw, dtype=dtype)
    if chans == 1:
        arr = arr.reshape(rows, cols)
    else:
        arr = arr.reshape(rows, cols, chans)
    return arr


def _mat_to_json(arr: np.ndarray) -> dict:
    """Encode a numpy array as a cv::Mat JSON blob."""
    dtype_to_depth = {
        np.dtype("uint8"):   0, np.dtype("int8"):    1,
        np.dtype("uint16"):  2, np.dtype("int16"):   3,
        np.dtype("int32"):   4, np.dtype("float32"): 5,
        np.dtype("float64"): 6,
    }
    arr = np.ascontiguousarray(arr)
    if arr.ndim == 2:
        rows, cols = arr.shape
        chans = 1
    else:
        rows, cols, chans = arr.shape
    depth = dtype_to_depth.get(arr.dtype, 0)
    cv_type = depth | ((chans - 1) << 3)
    return {
        "cols": cols,
        "rows": rows,
        "type": cv_type,
        "data": base64.b64encode(arr.tobytes()).decode(),
    }


def _se3_from_json(j: dict) -> np.ndarray:
    """Return a 4×4 float32 transformation matrix from {qw,qx,qy,qz,tx,ty,tz}."""
    qw, qx, qy, qz = j["qw"], j["qx"], j["qy"], j["qz"]
    tx, ty, tz = j["tx"], j["ty"], j["tz"]
    T = np.eye(4, dtype=np.float32)
    # Rotation from quaternion
    T[0, 0] = 1 - 2*(qy*qy + qz*qz)
    T[0, 1] = 2*(qx*qy - qz*qw)
    T[0, 2] = 2*(qx*qz + qy*qw)
    T[1, 0] = 2*(qx*qy + qz*qw)
    T[1, 1] = 1 - 2*(qx*qx + qz*qz)
    T[1, 2] = 2*(qy*qz - qx*qw)
    T[2, 0] = 2*(qx*qz - qy*qw)
    T[2, 1] = 2*(qy*qz + qx*qw)
    T[2, 2] = 1 - 2*(qx*qx + qy*qy)
    T[0, 3] = tx; T[1, 3] = ty; T[2, 3] = tz
    return T


def _se3_to_json(T: np.ndarray) -> dict:
    """Convert a 4×4 matrix to {qw,qx,qy,qz,tx,ty,tz}."""
    R = T[:3, :3].astype(float)
    t = T[:3, 3].astype(float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return {"qw": float(qw), "qx": float(qx), "qy": float(qy), "qz": float(qz),
            "tx": float(t[0]), "ty": float(t[1]), "tz": float(t[2])}


def _kp_from_json(j: dict) -> dict:
    return {"x": j["x"], "y": j["y"], "size": j["size"], "angle": j["angle"],
            "response": j["response"], "octave": j["octave"], "class_id": j["class_id"]}


def _kp_to_json(kp: dict) -> dict:
    return {"x": kp["x"], "y": kp["y"], "size": kp["size"], "angle": kp["angle"],
            "response": kp["response"], "octave": kp["octave"], "class_id": kp["class_id"]}


# ─── Data classes ─────────────────────────────────────────────────────────────

@dataclass
class OsaCamera:
    """Camera intrinsics model stored in the atlas."""
    id: int = 0
    type: int = 0          # 0 = Pinhole, 1 = KannalaBrandt8 (fisheye)
    params: List[float] = field(default_factory=list)
    # Pinhole: [fx, fy, cx, cy]
    # KannalaBrandt8: [fx, fy, cx, cy, k1, k2, k3, k4]

    @classmethod
    def pinhole(cls, fx: float, fy: float, cx: float, cy: float, cam_id: int = 0) -> "OsaCamera":
        return cls(id=cam_id, type=0, params=[fx, fy, cx, cy])

    @classmethod
    def _from_json(cls, j: dict) -> "OsaCamera":
        return cls(id=j["id"], type=j["type"], params=list(j["params"]))

    def _to_json(self) -> dict:
        return {"id": self.id, "type": self.type, "params": list(self.params)}


@dataclass
class OsaMapPoint:
    """A 3-D map point."""
    id: int = 0
    first_kf_id: int = 0
    first_frame: int = 0
    n_obs: int = 0
    world_pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    normal: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    # observations: {kf_id: feature_index} (stored as two separate maps for stereo)
    obs1: Dict[int, int] = field(default_factory=dict)
    obs2: Dict[int, int] = field(default_factory=dict)
    # Best descriptor (32 bytes, uint8, shape (1,32))
    descriptor: np.ndarray = field(default_factory=lambda: np.zeros((1, 32), dtype=np.uint8))
    ref_kf_id: int = 0
    bad: bool = False
    replaced_id: int = -1
    min_dist: float = 0.0
    max_dist: float = 0.0

    @classmethod
    def _from_json(cls, j: dict) -> "OsaMapPoint":
        obj = cls()
        obj.id          = j["id"]
        obj.first_kf_id = j["first_kf_id"]
        obj.first_frame = j["first_frame"]
        obj.n_obs       = j["n_obs"]
        obj.world_pos   = np.array(j["world_pos"], dtype=np.float32)
        obj.normal      = np.array(j["normal"],    dtype=np.float32)
        obj.obs1        = {int(k): v for k, v in j["obs1"].items()}
        obj.obs2        = {int(k): v for k, v in j["obs2"].items()}
        obj.descriptor  = _mat_from_json(j["descriptor"])
        obj.ref_kf_id   = j["ref_kf_id"]
        obj.bad         = j["bad"]
        obj.replaced_id = j["replaced_id"]
        obj.min_dist    = j["min_dist"]
        obj.max_dist    = j["max_dist"]
        return obj

    def _to_json(self) -> dict:
        return {
            "id": self.id, "first_kf_id": self.first_kf_id,
            "first_frame": self.first_frame, "n_obs": self.n_obs,
            "world_pos": [float(v) for v in self.world_pos],
            "normal":    [float(v) for v in self.normal],
            "obs1": {str(k): v for k, v in self.obs1.items()},
            "obs2": {str(k): v for k, v in self.obs2.items()},
            "descriptor": _mat_to_json(np.asarray(self.descriptor, dtype=np.uint8)),
            "ref_kf_id": self.ref_kf_id, "bad": self.bad,
            "replaced_id": self.replaced_id,
            "min_dist": self.min_dist, "max_dist": self.max_dist,
        }


@dataclass
class OsaKeyFrame:
    """A key frame: camera pose + ORB features + covisibility metadata."""
    id: int = 0
    frame_id: int = 0
    timestamp: float = 0.0
    # Camera pose Tcw (camera-from-world), 4×4 float32
    tcw: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))

    # Intrinsics
    fx: float = 0.0; fy: float = 0.0; cx: float = 0.0; cy: float = 0.0
    invfx: float = 0.0; invfy: float = 0.0
    mbf: float = 0.0; mb: float = 0.0; th_depth: float = 40.0

    # Image dimensions
    min_x: int = 0; min_y: int = 0; max_x: int = 640; max_y: int = 480

    # Feature count
    n: int = 0

    # ORB features — shape (N, 32) uint8
    descriptors: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 32), dtype=np.uint8))
    # Keypoints list of dicts {x,y,size,angle,response,octave,class_id}
    keypoints:    List[dict] = field(default_factory=list)
    keypoints_un: List[dict] = field(default_factory=list)
    u_right:      List[float] = field(default_factory=list)
    depth:        List[float] = field(default_factory=list)

    # BoW  {word_id: weight}  and feature vector {node_id: [feat_idx,...]}
    bow_vec:  Dict[int, float] = field(default_factory=dict)
    feat_vec: Dict[int, List[int]] = field(default_factory=dict)

    # Pose relative to parent in spanning tree
    tcp: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))

    # Scale pyramid
    scale_levels: int = 8
    scale_factor: float = 1.2
    log_scale_factor: float = 0.0
    scale_factors:    List[float] = field(default_factory=list)
    level_sigma2:     List[float] = field(default_factory=list)
    inv_level_sigma2: List[float] = field(default_factory=list)

    # Camera matrix K (3×3 row-major)
    K: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=np.float32))

    # MapPoint associations: mappoint_ids[i] = mp_id or -1
    mappoint_ids: List[int] = field(default_factory=list)

    # Feature grid (for fast lookup) — list of lists of lists of ints
    grid:       List = field(default_factory=list)
    grid_right: List = field(default_factory=list)

    # Covisibility
    connected_kf_weights: Dict[int, int] = field(default_factory=dict)

    # Spanning tree
    first_connection: bool = True
    parent_id: int = -1
    children_ids:   List[int] = field(default_factory=list)
    loop_edge_ids:  List[int] = field(default_factory=list)
    merge_edge_ids: List[int] = field(default_factory=list)

    # Flags
    not_erase:    bool = False
    to_be_erased: bool = False
    bad:          bool = False

    half_baseline:  float = 0.0
    origin_map_id:  int = 0
    camera_id:      int = 0
    camera2_id:     int = 0

    left_to_right:  List[int] = field(default_factory=list)
    right_to_left:  List[int] = field(default_factory=list)
    n_left:  int = -1
    n_right: int = -1
    keypoints_right: List[dict] = field(default_factory=list)
    tlr: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))

    # IMU (mostly zeros for monocular non-inertial)
    imu_bias:  dict = field(default_factory=lambda: {"bax":0,"bay":0,"baz":0,"bwx":0,"bwy":0,"bwz":0})
    imu_preint: dict = field(default_factory=dict)
    imu_calib:  dict = field(default_factory=dict)
    prev_kf_id: int = -1
    next_kf_id: int = -1
    b_imu: bool = False
    vw:  List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    owb: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    has_velocity: bool = False

    # Misc
    dist_coef: np.ndarray = field(default_factory=lambda: np.zeros((0, 1), dtype=np.float32))
    scale: float = 1.0
    grid_cols: int = 64; grid_rows: int = 48
    grid_width_inv: float = 0.0; grid_height_inv: float = 0.0

    @classmethod
    def _from_json(cls, j: dict) -> "OsaKeyFrame":
        obj = cls()
        obj.id            = j["id"]
        obj.frame_id      = j["frame_id"]
        obj.timestamp     = j["timestamp"]
        obj.tcw           = _se3_from_json(j["tcw"])
        obj.fx            = j["fx"];   obj.fy    = j["fy"]
        obj.cx            = j["cx"];   obj.cy    = j["cy"]
        obj.invfx         = j["invfx"];obj.invfy = j["invfy"]
        obj.mbf           = j["mbf"];  obj.mb    = j["mb"]
        obj.th_depth      = j["th_depth"]
        obj.dist_coef     = _mat_from_json(j["dist_coef"])
        obj.min_x         = j["min_x"]; obj.min_y = j["min_y"]
        obj.max_x         = j["max_x"]; obj.max_y = j["max_y"]
        obj.n             = j["n"]
        obj.descriptors   = _mat_from_json(j["descriptors"])
        obj.keypoints     = [_kp_from_json(k) for k in j["keypoints"]]
        obj.keypoints_un  = [_kp_from_json(k) for k in j["keypoints_un"]]
        obj.u_right       = list(j["u_right"])
        obj.depth         = list(j["depth"])
        obj.bow_vec       = {int(k): v for k, v in j["bow_vec"].items()}
        obj.feat_vec      = {int(k): [int(x) for x in v]
                             for k, v in j["feat_vec"].items()}
        obj.tcp           = _se3_from_json(j["tcp"])
        obj.scale_levels  = j["scale_levels"]
        obj.scale_factor  = j["scale_factor"]
        obj.log_scale_factor = j["log_scale_factor"]
        obj.scale_factors    = list(j["scale_factors"])
        obj.level_sigma2     = list(j["level_sigma2"])
        obj.inv_level_sigma2 = list(j["inv_level_sigma2"])
        K = np.array(j["K"], dtype=np.float32).reshape(3, 3)
        obj.K = K
        obj.mappoint_ids  = list(j["mappoint_ids"])
        obj.grid          = j["grid"]
        obj.grid_right    = j["grid_right"]
        obj.connected_kf_weights = {int(k): v for k, v in j["connected_kf_weights"].items()}
        obj.first_connection = j["first_connection"]
        obj.parent_id     = j["parent_id"]
        obj.children_ids  = list(j["children_ids"])
        obj.loop_edge_ids = list(j["loop_edge_ids"])
        obj.merge_edge_ids= list(j["merge_edge_ids"])
        obj.not_erase     = j["not_erase"]
        obj.to_be_erased  = j["to_be_erased"]
        obj.bad           = j["bad"]
        obj.half_baseline = j["half_baseline"]
        obj.origin_map_id = j["origin_map_id"]
        obj.camera_id     = j["camera_id"]
        obj.camera2_id    = j["camera2_id"]
        obj.left_to_right = list(j["left_to_right"])
        obj.right_to_left = list(j["right_to_left"])
        obj.n_left        = j["n_left"]
        obj.n_right       = j["n_right"]
        obj.keypoints_right = [_kp_from_json(k) for k in j["keypoints_right"]]
        obj.tlr           = _se3_from_json(j["tlr"])
        obj.imu_bias      = dict(j["imu_bias"])
        obj.imu_preint    = dict(j["imu_preint"])
        obj.imu_calib     = dict(j["imu_calib"])
        obj.prev_kf_id    = j["prev_kf_id"]
        obj.next_kf_id    = j["next_kf_id"]
        obj.b_imu         = j["b_imu"]
        obj.vw            = list(j["vw"])
        obj.owb           = list(j["owb"])
        obj.has_velocity  = j["has_velocity"]
        obj.scale         = j.get("scale", 1.0)
        obj.grid_cols     = j.get("grid_cols", 64)
        obj.grid_rows     = j.get("grid_rows", 48)
        obj.grid_width_inv  = j.get("grid_width_inv", 0.0)
        obj.grid_height_inv = j.get("grid_height_inv", 0.0)
        return obj

    def _to_json(self) -> dict:
        return {
            "id": self.id, "frame_id": self.frame_id, "timestamp": self.timestamp,
            "tcw": _se3_to_json(self.tcw),
            "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
            "invfx": self.invfx, "invfy": self.invfy,
            "mbf": self.mbf, "mb": self.mb, "th_depth": self.th_depth,
            "dist_coef": _mat_to_json(np.asarray(self.dist_coef, dtype=np.float32)),
            "min_x": self.min_x, "min_y": self.min_y,
            "max_x": self.max_x, "max_y": self.max_y,
            "n": self.n,
            "descriptors": _mat_to_json(np.asarray(self.descriptors, dtype=np.uint8)),
            "keypoints":    [_kp_to_json(k) for k in self.keypoints],
            "keypoints_un": [_kp_to_json(k) for k in self.keypoints_un],
            "u_right": [float(v) for v in self.u_right],
            "depth":   [float(v) for v in self.depth],
            "bow_vec":  {str(k): v for k, v in self.bow_vec.items()},
            "feat_vec": {str(k): [int(x) for x in v] for k, v in self.feat_vec.items()},
            "tcp": _se3_to_json(self.tcp),
            "scale_levels": self.scale_levels,
            "scale_factor": self.scale_factor,
            "log_scale_factor": self.log_scale_factor,
            "scale_factors":    [float(v) for v in self.scale_factors],
            "level_sigma2":     [float(v) for v in self.level_sigma2],
            "inv_level_sigma2": [float(v) for v in self.inv_level_sigma2],
            "K": [float(v) for v in np.asarray(self.K).ravel()],
            "mappoint_ids": [int(v) for v in self.mappoint_ids],
            "grid": self.grid, "grid_right": self.grid_right,
            "connected_kf_weights": {str(k): v for k, v in self.connected_kf_weights.items()},
            "first_connection": self.first_connection,
            "parent_id":   self.parent_id,
            "children_ids":   [int(v) for v in self.children_ids],
            "loop_edge_ids":  [int(v) for v in self.loop_edge_ids],
            "merge_edge_ids": [int(v) for v in self.merge_edge_ids],
            "not_erase": self.not_erase, "to_be_erased": self.to_be_erased,
            "bad": self.bad, "half_baseline": self.half_baseline,
            "origin_map_id": self.origin_map_id,
            "camera_id": self.camera_id, "camera2_id": self.camera2_id,
            "left_to_right": [int(v) for v in self.left_to_right],
            "right_to_left": [int(v) for v in self.right_to_left],
            "n_left": self.n_left, "n_right": self.n_right,
            "keypoints_right": [_kp_to_json(k) for k in self.keypoints_right],
            "tlr": _se3_to_json(self.tlr),
            "imu_bias": dict(self.imu_bias),
            "imu_preint": _default_imu_preint() if not self.imu_preint else dict(self.imu_preint),
            "imu_calib":  _default_imu_calib()  if not self.imu_calib  else dict(self.imu_calib),
            "prev_kf_id": self.prev_kf_id, "next_kf_id": self.next_kf_id,
            "b_imu": self.b_imu,
            "vw": [float(v) for v in self.vw],
            "owb": [float(v) for v in self.owb],
            "has_velocity": self.has_velocity,
            "scale": self.scale,
            "grid_cols": self.grid_cols, "grid_rows": self.grid_rows,
            "grid_width_inv": self.grid_width_inv,
            "grid_height_inv": self.grid_height_inv,
        }

    # ── Convenience accessors ──────────────────────────────────────────────

    def camera_center(self) -> np.ndarray:
        """World position of the camera (Ow = R^T * -t)."""
        R = self.tcw[:3, :3]
        t = self.tcw[:3, 3]
        return (-R.T @ t).astype(np.float32)

    def keypoints_array(self) -> np.ndarray:
        """Return keypoints as (N,2) float32 array of (x,y) pixel coordinates."""
        if not self.keypoints:
            return np.zeros((0, 2), dtype=np.float32)
        return np.array([[k["x"], k["y"]] for k in self.keypoints], dtype=np.float32)


@dataclass
class OsaMap:
    """One map in the atlas (multiple maps possible after loop/merge events)."""
    id: int = 0
    init_kf_id: int = 0
    max_kf_id:  int = 0
    big_change_idx: int = 0
    imu_initialized: bool = False
    is_inertial:     bool = False
    imu_ba1: bool = False
    imu_ba2: bool = False
    kf_origins_ids: List[int] = field(default_factory=list)
    keyframes: List[OsaKeyFrame] = field(default_factory=list)
    mappoints: List[OsaMapPoint] = field(default_factory=list)

    @classmethod
    def _from_json(cls, j: dict) -> "OsaMap":
        obj = cls()
        obj.id              = j["id"]
        obj.init_kf_id      = j["init_kf_id"]
        obj.max_kf_id       = j["max_kf_id"]
        obj.big_change_idx  = j["big_change_idx"]
        obj.imu_initialized = j["imu_initialized"]
        obj.is_inertial     = j["is_inertial"]
        obj.imu_ba1         = j["imu_ba1"]
        obj.imu_ba2         = j["imu_ba2"]
        obj.kf_origins_ids  = list(j.get("kf_origins_ids", []))
        obj.keyframes = [OsaKeyFrame._from_json(kf) for kf in j["keyframes"]]
        obj.mappoints = [OsaMapPoint._from_json(mp) for mp in j["mappoints"]]
        return obj

    def _to_json(self) -> dict:
        return {
            "id": self.id, "init_kf_id": self.init_kf_id,
            "max_kf_id": self.max_kf_id, "big_change_idx": self.big_change_idx,
            "imu_initialized": self.imu_initialized, "is_inertial": self.is_inertial,
            "imu_ba1": self.imu_ba1, "imu_ba2": self.imu_ba2,
            "kf_origins_ids": list(self.kf_origins_ids),
            "keyframes": [kf._to_json() for kf in self.keyframes],
            "mappoints": [mp._to_json() for mp in self.mappoints],
        }

    # ── Convenience ───────────────────────────────────────────────────────

    def world_points(self) -> np.ndarray:
        """Return all non-bad map point positions as (M,3) float32 array."""
        pts = [mp.world_pos for mp in self.mappoints if not mp.bad]
        return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)

    def camera_centers(self) -> np.ndarray:
        """Return all keyframe camera centres as (N,3) float32 array."""
        return np.array([kf.camera_center() for kf in self.keyframes], dtype=np.float32)

    def keyframe_by_id(self, kf_id: int) -> Optional[OsaKeyFrame]:
        for kf in self.keyframes:
            if kf.id == kf_id:
                return kf
        return None

    def mappoint_by_id(self, mp_id: int) -> Optional[OsaMapPoint]:
        for mp in self.mappoints:
            if mp.id == mp_id:
                return mp
        return None


@dataclass
class OsaAtlas:
    """Top-level atlas container: metadata + list of maps + cameras."""
    vocab_name:     str = "ORBvoc.txt"
    vocab_checksum: str = ""
    map_next_id:    int = 1
    frame_next_id:  int = 0
    kf_next_id:     int = 0
    mp_next_id:     int = 0
    cam_next_id:    int = 1
    last_init_kfid: int = 0
    cameras: List[OsaCamera] = field(default_factory=list)
    maps:    List[OsaMap]    = field(default_factory=list)

    # ── I/O ───────────────────────────────────────────────────────────────

    @classmethod
    def load(cls, osa_path: str) -> "OsaAtlas":
        """Load an .osa file into Python objects.

        The path may or may not include the `.osa` extension.
        """
        osa_path = _ensure_osa_ext(osa_path)
        binary = _find_osa_convert()
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            json_path = tf.name
        try:
            result = subprocess.run(
                [binary, "dump", osa_path, json_path],
                capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(f"osa_convert dump failed:\n{result.stderr}")
            with open(json_path) as f:
                root = json.load(f)
        finally:
            os.unlink(json_path)

        obj = cls()
        obj.vocab_name     = root["vocab_name"]
        obj.vocab_checksum = root["vocab_checksum"]
        obj.map_next_id    = root["map_next_id"]
        obj.frame_next_id  = root["frame_next_id"]
        obj.kf_next_id     = root["kf_next_id"]
        obj.mp_next_id     = root["mp_next_id"]
        obj.cam_next_id    = root["cam_next_id"]
        obj.last_init_kfid = root["last_init_kfid"]
        obj.cameras = [OsaCamera._from_json(c) for c in root["cameras"]]
        obj.maps    = [OsaMap._from_json(m)    for m in root["maps"]]
        return obj

    def save(self, osa_path: str) -> None:
        """Write the atlas to an .osa file readable by the localization service.

        The `.osa` extension is appended automatically if absent.
        """
        osa_path = _ensure_osa_ext(osa_path)
        binary = _find_osa_convert()
        root = self._to_json()
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".json", delete=False) as tf:
            json.dump(root, tf)
            json_path = tf.name
        try:
            result = subprocess.run(
                [binary, "pack", json_path, osa_path],
                capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(f"osa_convert pack failed:\n{result.stderr}")
        finally:
            os.unlink(json_path)

    # ── Factory ───────────────────────────────────────────────────────────

    @classmethod
    def new(cls, vocab_name: str = "ORBvoc.txt",
            vocab_checksum: str = "") -> "OsaAtlas":
        """Create an empty atlas (no maps, no cameras yet)."""
        return cls(vocab_name=vocab_name, vocab_checksum=vocab_checksum)

    # ── Serialisation ─────────────────────────────────────────────────────

    def _to_json(self) -> dict:
        return {
            "vocab_name":     self.vocab_name,
            "vocab_checksum": self.vocab_checksum,
            "map_next_id":    self.map_next_id,
            "frame_next_id":  self.frame_next_id,
            "kf_next_id":     self.kf_next_id,
            "mp_next_id":     self.mp_next_id,
            "cam_next_id":    self.cam_next_id,
            "last_init_kfid": self.last_init_kfid,
            "cameras": [c._to_json() for c in self.cameras],
            "maps":    [m._to_json() for m in self.maps],
        }

    def to_json_file(self, path: str) -> None:
        """Write the intermediate JSON representation to a file (for inspection)."""
        with open(path, "w") as f:
            json.dump(self._to_json(), f, indent=2)

    # ── Convenience ───────────────────────────────────────────────────────

    def __repr__(self) -> str:
        maps_info = ", ".join(
            f"Map#{m.id}({len(m.keyframes)}KF/{len(m.mappoints)}MP)"
            for m in self.maps)
        return f"OsaAtlas(vocab='{self.vocab_name}', maps=[{maps_info}])"


# ─── IMU defaults (for non-inertial atlases) ──────────────────────────────────

def _default_imu_preint() -> dict:
    zeros15x15 = [0.0] * 225
    zeros3     = [0.0, 0.0, 0.0]
    zeros9     = [0.0] * 9
    zeros6     = [0.0] * 6
    return {
        "dT": 0.0, "C": zeros15x15, "Info": zeros15x15,
        "Nga": zeros6, "NgaWalk": zeros6,
        "b": {"bax":0,"bay":0,"baz":0,"bwx":0,"bwy":0,"bwz":0},
        "dR": zeros9, "dV": zeros3, "dP": zeros3,
        "JRg": zeros9, "JVg": zeros9, "JVa": zeros9,
        "JPg": zeros9, "JPa": zeros9,
        "avgA": zeros3, "avgW": zeros3,
        "bu": {"bax":0,"bay":0,"baz":0,"bwx":0,"bwy":0,"bwz":0},
        "db": zeros6, "measurements": [],
    }

def _default_imu_calib() -> dict:
    identity = {"qw":1.0,"qx":0.0,"qy":0.0,"qz":0.0,"tx":0.0,"ty":0.0,"tz":0.0}
    return {
        "tcb": identity, "tbc": identity,
        "cov": [0.0]*6, "cov_walk": [0.0]*6,
        "is_set": False,
    }


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _ensure_osa_ext(path: str) -> str:
    if not path.endswith(".osa"):
        path += ".osa"
    return path


# ─── Builder helpers for creating new atlases ─────────────────────────────────

def build_feature_grid(keypoints: List[dict], width: int, height: int,
                       grid_cols: int = 64, grid_rows: int = 48
                       ) -> List[List[List[int]]]:
    """Assign keypoints to a grid cell, returning a 3-D list [col][row][feat_idx]."""
    cell_w = width  / grid_cols
    cell_h = height / grid_rows
    grid = [[[] for _ in range(grid_rows)] for _ in range(grid_cols)]
    for i, kp in enumerate(keypoints):
        col = min(int(kp["x"] / cell_w), grid_cols - 1)
        row = min(int(kp["y"] / cell_h), grid_rows - 1)
        grid[col][row].append(i)
    return grid


def make_scale_pyramid(scale_levels: int = 8, scale_factor: float = 1.2
                       ) -> tuple:
    """Return (scale_factors, level_sigma2, inv_level_sigma2, log_scale_factor)."""
    import math
    factors = [scale_factor ** i for i in range(scale_levels)]
    sigma2  = [f * f for f in factors]
    inv_s2  = [1.0 / s for s in sigma2]
    return factors, sigma2, inv_s2, math.log(scale_factor)
