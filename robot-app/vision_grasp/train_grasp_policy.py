#!/usr/bin/env python3
"""
抓取策略训练（train_grasp_policy.py）—— 行为克隆（BC）

数据：collect_grasp_data.py 产出的 episode 目录（默认只使用 label=success 的片段）。
观测：当前相机图像（缩放到 train.image_size）+ 6 关节角。
动作：horizon 帧后的关节角增量 dq（裁剪到 ±train.max_delta）+ 夹爪目标位置。
模型：轻量 CNN 视觉编码 + 关节状态分支 → 7 维动作头。

用法（建议在有 GPU 的机器上跑，WSL/Windows 均可）：
    python robot-app/vision_grasp/train_grasp_policy.py [--data 数据集目录] [--epochs 60]

产出：
    vision_grasp/checkpoints/grasp_policy_best.pt   权重 + 执行端所需的全部超参
    vision_grasp/checkpoints/train_log.jsonl        每个 epoch 的训练/验证损失

执行端用 run_grasp.py --mode policy --checkpoint <ckpt> 加载。
依赖：torch（未安装时脚本会给出安装提示而不是堆栈）。
"""
import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np

from common import GraspAbort, cfg_path, load_config

try:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, Dataset
except ImportError:
    raise SystemExit(
        "[训练] 未安装 PyTorch。请先安装，例如：\n"
        "  pip install torch torchvision   （GPU 版按其官方指引选择 CUDA 版本）\n"
        "采集脚本与执行脚本（mock/dry-run）不依赖 torch。"
    )


# ---------------- 数据集 ----------------
def scan_episodes(data_dir, include_fail=False):
    """返回可用 episode 目录列表（默认仅 success）。"""
    eps = []
    for meta_file in sorted(Path(data_dir).glob("episode_*/meta.json")):
        with open(meta_file, "r", encoding="utf-8") as f:
            meta = json.load(f)
        label = meta.get("label", "unlabeled")
        if label == "success" or (include_fail and label == "fail"):
            eps.append(meta_file.parent)
    return eps


class GraspDemoDataset(Dataset):
    """样本 = (图像, 关节角, dq前瞻增量, 夹爪目标)。帧在取用时按需读盘。"""

    def __init__(self, episodes, image_size, horizon, max_delta):
        self.image_size = int(image_size)
        self.horizon = int(horizon)
        self.max_delta = float(max_delta)
        self.samples = []          # (episode_dir, t, num_frames)
        for ep in episodes:
            states_file = ep / "states.jsonl"
            if not states_file.exists():
                continue
            with open(states_file, "r", encoding="utf-8") as f:
                rows = [json.loads(line) for line in f if line.strip()]
            # 末帧作为动作目标，因此起点最多到 len-horizon-1
            for t in range(len(rows) - self.horizon):
                self.samples.append((ep, t, rows))
        if not self.samples:
            raise GraspAbort(
                f"数据集为空：{data_dir_hint(episodes)}。"
                "请先用 collect_grasp_data.py 采集并标注至少一条 success 片段。"
            )

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        ep, t, rows = self.samples[idx]
        now, future = rows[t], rows[t + self.horizon]

        img = cv2.imread(str(ep / "frames" / now["frame"]))
        if img is None:
            raise RuntimeError(f"帧缺失: {ep / 'frames' / now['frame']}")
        img = cv2.resize(img, (self.image_size, self.image_size))
        img = torch.from_numpy(img.astype(np.float32) / 255.0).permute(2, 0, 1)

        q = np.array(now["q"], dtype=np.float32)
        dq = np.array(future["q"], dtype=np.float32) - q
        dq = np.clip(dq, -self.max_delta, self.max_delta)
        gripper = np.float32(future["gripper_q"])
        return img, q, np.concatenate([dq, [gripper]]).astype(np.float32)


def data_dir_hint(episodes):
    return "（未发现任何可用 episode）" if not episodes else str(episodes[0].parent)


# ---------------- 模型 ----------------
class GraspPolicy(nn.Module):
    """轻量 CNN + 关节状态分支 → 7 维动作（6 关节增量 + 夹爪目标）。"""

    def __init__(self):
        super().__init__()
        self.vision = nn.Sequential(
            nn.Conv2d(3, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(128, 256, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(),
        )
        self.state = nn.Sequential(nn.Linear(6, 32), nn.ReLU())
        self.head = nn.Sequential(
            nn.Linear(256 + 32, 256), nn.ReLU(), nn.Linear(256, 7),
        )

    def forward(self, img, q):
        return self.head(torch.cat([self.vision(img), self.state(q)], dim=1))


# ---------------- 训练 ----------------
def run_epoch(model, loader, device, optim=None):
    total, n = 0.0, 0
    model.train(optim is not None)
    for img, q, act in loader:
        img, q, act = img.to(device), q.to(device), act.to(device)
        pred = model(img, q)
        loss = nn.functional.mse_loss(pred, act)
        if optim is not None:
            optim.zero_grad()
            loss.backward()
            optim.step()
        total += loss.item() * len(img)
        n += len(img)
    return total / max(n, 1)


def save_checkpoint(path, model, meta):
    payload = {"model_state": model.state_dict()}
    payload.update(meta)
    torch.save(payload, path)
    print(f"[train] 已保存 {path}")


def main():
    ap = argparse.ArgumentParser(description="视觉抓取策略训练（行为克隆）")
    ap.add_argument("--config", default=None, help="配置文件路径（默认同目录 config.yaml）")
    ap.add_argument("--data", default=None, help="数据集目录（默认取配置 record.output_dir）")
    ap.add_argument("--out", default=None, help="checkpoint 输出目录（默认取配置 train.checkpoint_dir）")
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--batch-size", type=int, default=None)
    ap.add_argument("--lr", type=float, default=None)
    ap.add_argument("--horizon", type=int, default=None)
    ap.add_argument("--image-size", type=int, default=None)
    ap.add_argument("--include-fail", action="store_true", help="同时把 fail 片段纳入训练（默认仅 success）")
    ap.add_argument("--val-ratio", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg, cfg_dir = load_config(args.config)
    tr = cfg["train"]
    epochs = args.epochs or tr["epochs"]
    batch_size = args.batch_size or tr["batch_size"]
    lr = args.lr or tr["lr"]
    horizon = args.horizon or tr["horizon"]
    image_size = args.image_size or tr["image_size"]
    max_delta = float(tr["max_delta"])

    data_dir = Path(args.data) if args.data else cfg_path(cfg_dir, cfg["record"]["output_dir"])
    out_dir = Path(args.out) if args.out else cfg_path(cfg_dir, tr["checkpoint_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    episodes = scan_episodes(data_dir, include_fail=args.include_fail)
    print(f"[train] 数据集: {data_dir}，可用 episode {len(episodes)} 条")
    if not episodes:
        raise GraspAbort(f"数据集目录中没有可用 episode: {data_dir}")

    # 按 episode 划分训练/验证，避免同一演示泄漏到验证集
    random.shuffle(episodes)
    n_val = max(1, int(len(episodes) * args.val_ratio)) if len(episodes) >= 2 else 0
    val_eps, train_eps = episodes[:n_val], episodes[n_val:]
    train_ds = GraspDemoDataset(train_eps, image_size, horizon, max_delta)
    val_ds = (GraspDemoDataset(val_eps, image_size, horizon, max_delta)
              if val_eps else None)
    print(f"[train] 样本数: 训练 {len(train_ds)}" +
          (f" / 验证 {len(val_ds)}" if val_ds else "（数据太少，跳过验证集）"))

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[train] 设备: {device}")
    model = GraspPolicy().to(device)
    optim = torch.optim.Adam(model.parameters(), lr=lr)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=0, drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=batch_size) if val_ds else None

    log_file = out_dir / "train_log.jsonl"
    best_val = float("inf")
    ckpt_meta = {
        "image_size": image_size, "horizon": horizon, "max_delta": max_delta,
        "record_rate_hz": cfg["record"]["rate_hz"],
        "policy_rate_hz": cfg["policy"]["rate_hz"],
        "joint_lower": cfg["robot"]["joint_lower"],
        "joint_upper": cfg["robot"]["joint_upper"],
        "gripper_open": cfg["grasp"]["gripper_open"],
        "gripper_close": cfg["grasp"]["gripper_close"],
    }
    with open(log_file, "a", encoding="utf-8") as lf:
        for epoch in range(1, epochs + 1):
            train_loss = run_epoch(model, train_loader, device, optim)
            val_loss = run_epoch(model, val_loader, device) if val_loader else None
            line = {"epoch": epoch, "train_loss": round(train_loss, 6),
                    "val_loss": None if val_loss is None else round(val_loss, 6)}
            lf.write(json.dumps(line) + "\n")
            lf.flush()
            msg = f"[train] epoch {epoch:3d}/{epochs}  train={train_loss:.6f}"
            if val_loss is not None:
                msg += f"  val={val_loss:.6f}"
                if val_loss < best_val:
                    best_val = val_loss
                    ckpt_meta.update({"epoch": epoch, "val_loss": best_val})
                    save_checkpoint(out_dir / "grasp_policy_best.pt", model, ckpt_meta)
            else:
                # 无验证集时每轮覆盖保存 latest
                ckpt_meta.update({"epoch": epoch})
                save_checkpoint(out_dir / "grasp_policy_latest.pt", model, ckpt_meta)
            print(msg)

    if val_loader is not None:
        print(f"[train] 完成，最佳验证损失 {best_val:.6f} → {out_dir / 'grasp_policy_best.pt'}")
    else:
        print(f"[train] 完成 → {out_dir / 'grasp_policy_latest.pt'}")
    print("[train] 下一步：run_grasp.py --mode policy --checkpoint <ckpt> --dry-run 先验证")


if __name__ == "__main__":
    try:
        main()
    except GraspAbort as e:
        print(f"[中止] {e}")
