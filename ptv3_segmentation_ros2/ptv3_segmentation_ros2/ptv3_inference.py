#!/usr/bin/env python3
"""
PTv3 inference worker (Python 3.8 + torch + pointcept, NO rclpy).

Run with the pointcept conda Python:
    /home/fondecyt/anaconda3/envs/pointcept/bin/python3 ptv3_inference.py

Binary protocol over stdin/stdout (big-endian):
  RX: 4B uint32 N  +  N*24B float32[N,6] xyzrgb  (rgb normalized to [0,1])
  TX: 4B uint32 N  +  N*4B  int32[N]    class labels
Repeats until stdin EOF.  All log messages go to stderr.
"""
import sys
import os
import struct
import numpy as np

POINTCEPT_ROOT = os.path.expanduser("~/Documents/pointtransform/Pointcept")
if POINTCEPT_ROOT not in sys.path:
    sys.path.insert(0, POINTCEPT_ROOT)

import torch  # noqa: E402

_PTv3_BACKBONE_CFG = dict(
    type="PT-v3m1",
    in_channels=6,
    order=("z", "z-trans"),
    stride=(2, 2, 2, 2),
    enc_depths=(2, 2, 2, 6, 2),
    enc_channels=(32, 64, 128, 256, 512),
    enc_num_head=(2, 4, 8, 16, 32),
    enc_patch_size=(128, 128, 128, 128, 128),
    dec_depths=(2, 2, 2, 2),
    dec_channels=(64, 64, 128, 256),
    dec_num_head=(4, 4, 8, 16),
    dec_patch_size=(128, 128, 128, 128),
    mlp_ratio=4,
    qkv_bias=True,
    qk_scale=None,
    attn_drop=0.0,
    proj_drop=0.0,
    drop_path=0.3,
    shuffle_orders=True,
    pre_norm=True,
    enable_rpe=False,
    enable_flash=False,
    upcast_attention=False,
    upcast_softmax=False,
)

NUM_CLASSES = 6


def _log(msg: str):
    sys.stderr.write(f"[PTv3-worker] {msg}\n")
    sys.stderr.flush()


def _normalize_xyz(pts: np.ndarray) -> np.ndarray:
    cmin = pts.min(axis=0, keepdims=True)
    cmax = pts.max(axis=0, keepdims=True)
    denom = cmax - cmin
    denom[denom == 0.0] = 1.0
    return (pts - cmin) / denom


def _load_model(weights_path: str, device: torch.device):
    from pointcept.models import build_model

    model_cfg = dict(
        type="DefaultSegmentorV2",
        num_classes=NUM_CLASSES,
        backbone_out_channels=64,
        backbone=_PTv3_BACKBONE_CFG,
        criteria=[dict(type="CrossEntropyLoss", loss_weight=1.0, ignore_index=-1)],
    )
    model = build_model(model_cfg).to(device)

    ckpt = torch.load(weights_path, map_location=device, weights_only=False)
    state = ckpt.get("state_dict", ckpt)
    state = {k.replace("module.", "", 1): v for k, v in state.items()}
    missing, _ = model.load_state_dict(state, strict=False)
    if missing:
        _log(f"Missing keys ({len(missing)}): {missing[:3]}...")
    model.eval()
    return model


GRID_SIZE = 0.01  # voxel size for PTv3 serialization (matches training config)


def _infer(model, pts: np.ndarray, rgb: np.ndarray, device: torch.device) -> np.ndarray:
    """
    pts: (N, 3) float32 world coordinates
    rgb: (N, 3) float32 normalized [0, 1]
    feat: (N, 6) = [xyz_norm, rgb] — uses all 6 in_channels of PTv3
    """
    coord_norm = _normalize_xyz(pts)
    feat = np.concatenate([coord_norm, rgb], axis=1)  # (N, 6)

    data_dict = {
        "coord":     torch.from_numpy(pts.astype(np.float32)).to(device),
        "feat":      torch.from_numpy(feat.astype(np.float32)).to(device),
        "offset":    torch.tensor([len(pts)], dtype=torch.int32).to(device),
        "grid_size": torch.tensor(GRID_SIZE).to(device).float(),
    }
    with torch.no_grad():
        out    = model(data_dict)
        labels = out["seg_logits"].argmax(dim=1).cpu().numpy().astype(np.int32)
    return labels


def _recv_exact(stream, nbytes: int) -> bytes:
    buf = b''
    while len(buf) < nbytes:
        chunk = stream.read(nbytes - len(buf))
        if not chunk:
            raise EOFError("stdin closed")
        buf += chunk
    return buf


def main():
    weights_path = os.path.expanduser(
        "~/Documents/pointtransform/Pointcept/exp/"
        "crops3d_maize/semseg-pt-v3/model/model_best.pth"
    )
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    _log(f"Loading PTv3 from {weights_path} on {device}...")
    model = _load_model(weights_path, device)
    _log("PTv3 inference worker ready")

    stdin  = sys.stdin.buffer
    stdout = sys.stdout.buffer

    while True:
        try:
            header = _recv_exact(stdin, 4)
        except EOFError:
            break
        N = struct.unpack('!I', header)[0]

        try:
            raw = _recv_exact(stdin, N * 24)
        except EOFError:
            break
        data   = np.frombuffer(raw, dtype=np.float32).reshape(N, 6)
        pts    = data[:, :3]
        rgb    = data[:, 3:]

        try:
            labels = _infer(model, pts, rgb, device)
        except Exception as exc:
            import traceback
            _log(f"INFERENCE ERROR (N={N}): {exc}")
            _log(traceback.format_exc())
            # Return zeros so the pipeline keeps running
            labels = np.zeros(N, dtype=np.int32)

        stdout.write(struct.pack('!I', N))
        stdout.write(labels.tobytes())
        stdout.flush()


if __name__ == "__main__":
    main()
