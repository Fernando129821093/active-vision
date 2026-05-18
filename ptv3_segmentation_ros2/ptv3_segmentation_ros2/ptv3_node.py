#!/usr/bin/env python3
"""
PTv3 segmentation bridge node (Python 3.12 + rclpy).

Spawns ptv3_inference.py under the pointcept conda Python (3.8 + torch + CUDA 11.8)
as a persistent subprocess.  Communicates via binary stdin/stdout:
  send: 4B uint32 N  +  N*24B float32[N,6] xyzrgb  (rgb normalized to [0,1])
  recv: 4B uint32 N  +  N*4B  int32[N]    labels

Subscribe:  /nbv/plant_pointcloud   (sensor_msgs/PointCloud2, xyzrgb packed, world frame)
Publish:    /nbv/semantic_pointcloud (sensor_msgs/PointCloud2, xyz + semantic_color uint32)
"""
import os
import struct
import subprocess
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

# Default path to the pointcept conda Python that has torch 2.1 + CUDA 11.8
_DEFAULT_CONDA_PYTHON = "/home/fondecyt/anaconda3/envs/pointcept/bin/python3"

# PTv3 6-class colour map: Soil, Stem, Leaf, Tassel, Corn, Pot
CLASS_COLORS_RGB = np.array([
    [128,  64,   0],
    [  0, 128,   0],
    [  0, 255,   0],
    [255, 215,   0],
    [255, 165,   0],
    [128,   0, 128],
], dtype=np.uint8)


def _parse_xyzrgb(msg: PointCloud2):
    """
    Extract (N, 3) float32 xyz and (N, 3) float32 rgb∈[0,1] from a PointCloud2.

    Expects fields: x, y, z (float32) and rgb (float32 = packed uint32 0x00RRGGBB).
    Falls back to zeros for rgb if the field is absent (backward compatible).
    """
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    field_map = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    idx  = np.arange(n) * step

    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off  = field_map[name]
        cols = np.stack([raw[idx + off + b] for b in range(4)], axis=1)
        xyz[:, i] = np.frombuffer(cols.tobytes(), dtype=np.float32)

    rgb = np.zeros((n, 3), dtype=np.float32)
    if 'rgb' in field_map:
        off  = field_map['rgb']
        cols = np.stack([raw[idx + off + b] for b in range(4)], axis=1)
        packed = np.frombuffer(cols.tobytes(), dtype=np.uint32)
        rgb[:, 0] = ((packed >> 16) & 0xFF).astype(np.float32) / 255.0  # R
        rgb[:, 1] = ((packed >>  8) & 0xFF).astype(np.float32) / 255.0  # G
        rgb[:, 2] = ( packed        & 0xFF).astype(np.float32) / 255.0  # B

    return xyz, rgb


def _build_semantic_cloud(orig: PointCloud2,
                          pts: np.ndarray,
                          labels: np.ndarray) -> PointCloud2:
    """
    Output fields:
      x, y, z           — position
      rgb                — PCL-style packed RGB (uint32 as float32) for ColorOcTree + RViz
      semantic_color     — same bits, read by octomap_generator for class extraction
    """
    n = len(pts)
    colors_u32 = np.zeros(n, dtype=np.uint32)
    for cls_idx, (r, g, b) in enumerate(CLASS_COLORS_RGB.tolist()):
        mask = labels == cls_idx
        if mask.any():
            colors_u32[mask] = (r << 16) | (g << 8) | b

    colors_f32 = colors_u32.view(np.float32)

    dt = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32),
                   ('rgb', np.float32), ('semantic_color', np.float32)])
    arr = np.zeros(n, dtype=dt)
    arr['x'] = pts[:, 0]
    arr['y'] = pts[:, 1]
    arr['z'] = pts[:, 2]
    arr['rgb']            = colors_f32
    arr['semantic_color'] = colors_f32

    out = PointCloud2()
    out.header = orig.header
    out.height = 1
    out.width  = n
    out.is_dense    = False
    out.is_bigendian = False
    out.fields = [
        PointField(name='x',              offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',              offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',              offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb',            offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='semantic_color', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    out.point_step = 20
    out.row_step   = 20 * n
    out.data       = arr.tobytes()
    return out


def _recv_exact(stream, nbytes: int) -> bytes:
    buf = b''
    while len(buf) < nbytes:
        chunk = stream.read(nbytes - len(buf))
        if not chunk:
            raise EOFError("worker stdout closed")
        buf += chunk
    return buf


class PTv3BridgeNode(Node):
    def __init__(self):
        super().__init__("ptv3_segmentation_node")

        self.declare_parameter("conda_python", _DEFAULT_CONDA_PYTHON)
        self._conda_python = self.get_parameter("conda_python").value

        # Find the inference worker next to this module
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self._worker_script = os.path.join(script_dir, "ptv3_inference.py")
        if not os.path.exists(self._worker_script):
            self.get_logger().fatal(f"Inference script not found: {self._worker_script}")
            raise RuntimeError(f"Missing {self._worker_script}")

        self._lock = threading.Lock()
        self._worker = None
        self._spawn_worker()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(PointCloud2, "/nbv/semantic_pointcloud", qos)
        self.sub = self.create_subscription(
            PointCloud2, "/nbv/plant_pointcloud", self._cloud_cb, qos)

        self.get_logger().info("PTv3 bridge node ready.")

    def _spawn_worker(self):
        if self._worker is not None:
            try:
                self._worker.kill()
            except Exception:
                pass
        self.get_logger().info(
            f"Starting PTv3 worker: {self._conda_python} {self._worker_script}")
        self._worker = subprocess.Popen(
            [self._conda_python, self._worker_script],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        threading.Thread(target=self._pipe_stderr,
                         args=(self._worker,), daemon=True).start()

    def _pipe_stderr(self, proc):
        for line in proc.stderr:
            self.get_logger().info(line.decode(errors='replace').rstrip())

    def _cloud_cb(self, msg: PointCloud2):
        self.get_logger().info(
            f"[PTv3] cloud received: {msg.width}x{msg.height} step={msg.point_step}")
        try:
            pts, rgb = _parse_xyzrgb(msg)
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")
            return

        valid = np.isfinite(pts).all(axis=1)
        pts_v = pts[valid]
        rgb_v = rgb[valid]
        self.get_logger().info(f"[PTv3] valid pts: {len(pts_v)}/{len(pts)}")
        if len(pts_v) < 10:
            self.get_logger().warn(f"[PTv3] too few valid points ({len(pts_v)}), skipping")
            return

        try:
            labels_v = self._call_worker(pts_v, rgb_v)
        except Exception as e:
            self.get_logger().warn(f"Worker error: {e}")
            return

        labels_full = np.zeros(len(pts), dtype=np.int32)
        labels_full[valid] = labels_v

        try:
            cloud_out = _build_semantic_cloud(msg, pts, labels_full)
            self.pub.publish(cloud_out)
            unique, counts = np.unique(labels_full, return_counts=True)
            self.get_logger().info(
                f"[PTv3] published {len(pts)} pts, classes: "
                + ", ".join(f"{u}:{c}" for u, c in zip(unique, counts)))
        except Exception as e:
            self.get_logger().error(f"[PTv3] publish error: {e}")

    def _call_worker(self, pts: np.ndarray, rgb: np.ndarray) -> np.ndarray:
        N = len(pts)
        xyzrgb = np.concatenate([pts.astype(np.float32), rgb.astype(np.float32)], axis=1)  # (N,6)
        payload = struct.pack('!I', N) + xyzrgb.tobytes()
        with self._lock:
            # Check if worker is alive; restart if not
            if self._worker.poll() is not None:
                self.get_logger().warn(
                    f"Worker exited (code={self._worker.returncode}), restarting...")
                self._spawn_worker()
                raise RuntimeError("Worker restarted, skipping this cloud")
            try:
                self._worker.stdin.write(payload)
                self._worker.stdin.flush()
                header = _recv_exact(self._worker.stdout, 4)
                N_resp = struct.unpack('!I', header)[0]
                raw = _recv_exact(self._worker.stdout, N_resp * 4)
            except (BrokenPipeError, EOFError, OSError) as e:
                self.get_logger().warn(
                    f"Worker pipe error ({e}), restarting...")
                self._spawn_worker()
                raise RuntimeError("Worker restarted, skipping this cloud")
        return np.frombuffer(raw, dtype=np.int32)

    def destroy_node(self):
        if self._worker is not None:
            self._worker.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PTv3BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
