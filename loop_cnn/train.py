"""Train the leorover CNN policy."""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import time
from datetime import datetime
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, WeightedRandomSampler
from tqdm.auto import tqdm

from . import (
    DEFAULT_DATA_ROOT,
    DEFAULT_FRAME_HISTORY,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    LEGACY_DATA_ROOT,
)
from .dataset import build_datasets, build_experiment_datasets
from .model import LoopPolicyConfig, build_model, save_checkpoint


def resolve_run_dir(base_dir: Path) -> Path:
    """Create a unique timestamped run directory beneath the requested base path."""
    base_dir = Path(base_dir)
    base_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    candidate = base_dir / timestamp
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = base_dir / f"{timestamp}_{suffix:02d}"
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def write_training_summary(
    path: Path,
    *,
    device: torch.device,
    args,
    model_config: LoopPolicyConfig,
    train_sessions: list[str],
    val_sessions: list[str],
    history: list[dict[str, float]],
    best_epoch: int,
    best_metric: float,
    interrupted: bool,
) -> None:
    """Persist the current training state so interrupted runs still keep history."""
    summary = {
        "device": str(device),
        "epochs_requested": args.epochs,
        "epochs_completed": len(history),
        "best_epoch": best_epoch,
        "best_metric": best_metric,
        "train_sessions": train_sessions,
        "val_sessions": val_sessions,
        "model_config": asdict(model_config),
        "history": history,
        "interrupted": interrupted,
    }
    if getattr(args, "max_white_episodes", None) is not None:
        summary["max_white_episodes"] = args.max_white_episodes
    if getattr(args, "finetune_from", None) is not None:
        summary["finetune_from"] = str(args.finetune_from)
    if getattr(args, "freeze_encoder_blocks", 0):
        summary["freeze_encoder_blocks"] = args.freeze_encoder_blocks
    with path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)


def resolve_episodes_dir(path: Path) -> Path:
    """Prefer the new generic dataset root but transparently support the legacy one."""
    if path.exists():
        return path

    default_path = Path(DEFAULT_DATA_ROOT)
    legacy_path = Path(LEGACY_DATA_ROOT)
    if path == default_path and legacy_path.exists():
        print(f"[train] NOTE: Using legacy CNN dataset root at {legacy_path}")
        return legacy_path
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train the leorover CNN policy")
    parser.add_argument("--episodes-dir", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--run-dir", default="runs/cnn_v1",
                        help="Base directory for training runs; each launch creates a timestamped child run")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--frame-history", type=int, default=DEFAULT_FRAME_HISTORY)
    parser.add_argument("--image-width", type=int, default=DEFAULT_IMAGE_WIDTH)
    parser.add_argument("--image-height", type=int, default=DEFAULT_IMAGE_HEIGHT)
    parser.add_argument("--huber-delta", type=float, default=1.0)
    parser.add_argument("--no-progress", action="store_true",
                        help="Disable tqdm progress bars")
    parser.add_argument("--tape-color",
                        choices=["red", "blue", "green", "white"],
                        default=None,
                        help="Only train on episodes collected with this tape color "
                             "(reads tape_color from episode_info.json). "
                             "Omit to use all episodes in --episodes-dir.")
    parser.add_argument("--use-masked-video", action="store_true",
                        help="Load video_masked.mp4 instead of video.mp4 for each episode. "
                             "Requires running `python -m loop_cnn.segment` first.")
    parser.add_argument("--finetune-from", default=None, metavar="CHECKPOINT",
                        help="Load weights from this .pt checkpoint before training "
                             "(fine-tuning from a pre-trained policy).")
    parser.add_argument("--max-white-episodes", type=int, default=None, metavar="N",
                        help="Cap white tape training episodes at N and enable experiment "
                             "mode: trains on all red + N white episodes and reports "
                             "per-color (val_loss_red / val_loss_white) each epoch.")
    parser.add_argument("--freeze-encoder-blocks", type=int, default=0, metavar="N",
                        choices=[0, 1, 2, 3, 4],
                        help="Freeze the first N encoder ConvBlocks during fine-tuning "
                             "(0=none, 4=all). Frozen blocks stay in eval mode so BatchNorm "
                             "uses running statistics rather than batch statistics. "
                             "Useful when --finetune-from is set: higher values preserve "
                             "geometry features at the cost of reduced color adaptation.")
    return parser


def resolve_device(requested: str) -> torch.device:
    if requested != "auto":
        return torch.device(requested)
    if torch.cuda.is_available():
        return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():  # pragma: no cover
        return torch.device("mps")
    return torch.device("cpu")


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def build_loaders(
    episodes_dir: Path,
    *,
    val_ratio: float,
    seed: int,
    batch_size: int,
    num_workers: int,
    frame_history: int,
    image_width: int,
    image_height: int,
    tape_color: str | None = None,
    use_masked_video: bool = False,
) -> tuple[DataLoader, DataLoader | None, list[str], list[str]]:
    train_dataset, val_dataset = build_datasets(
        episodes_dir=episodes_dir,
        image_size=(image_width, image_height),
        history=frame_history,
        val_ratio=val_ratio,
        seed=seed,
        tape_color=tape_color,
        use_masked_video=use_masked_video,
    )
    if len(train_dataset) == 0:
        raise RuntimeError(f"No CNN episodes found under {episodes_dir}")

    preload_threshold_frames = 25000
    preload_threshold_records = 64
    if train_dataset.records and len(train_dataset.records) <= preload_threshold_records and train_dataset.total_frames <= preload_threshold_frames:
        estimated_gb = train_dataset.estimated_cache_bytes / (1024 ** 3)
        print(
            f"[train] Preloading {len(train_dataset.records)} train episodes into RAM "
            f"(~{estimated_gb:.2f} GB resized frames) to avoid repeated video decode."
        )
        train_dataset.preload_all()
        if len(val_dataset.records) > 0:
            val_dataset.preload_all()

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        sampler=WeightedRandomSampler(
            weights=torch.as_tensor(train_dataset.sample_weights, dtype=torch.double),
            num_samples=len(train_dataset.sample_weights),
            replacement=True,
        ),
        shuffle=False,
        num_workers=num_workers,
        pin_memory=torch.cuda.is_available(),
        persistent_workers=num_workers > 0,
    )
    val_loader = None
    if len(val_dataset) > 0:
        val_loader = DataLoader(
            val_dataset,
            batch_size=batch_size,
            shuffle=False,
            num_workers=num_workers,
            pin_memory=torch.cuda.is_available(),
            persistent_workers=num_workers > 0,
        )
    train_sessions = sorted({record.session_name for record in train_dataset.records})
    val_sessions = sorted({record.session_name for record in val_dataset.records})
    return train_loader, val_loader, train_sessions, val_sessions


def build_experiment_loaders(
    episodes_dir: Path,
    *,
    max_white_episodes: int,
    val_ratio: float,
    seed: int,
    batch_size: int,
    num_workers: int,
    frame_history: int,
    image_width: int,
    image_height: int,
) -> tuple[DataLoader, DataLoader | None, DataLoader | None, DataLoader | None, list[str], list[str]]:
    """Build loaders for mixed red+white experiment.

    Returns:
        (train_loader, val_loader_all, val_loader_red, val_loader_white, train_sessions, val_sessions)
    """
    train_ds, val_ds_all, val_ds_red, val_ds_white = build_experiment_datasets(
        episodes_dir=episodes_dir,
        max_white_train_episodes=max_white_episodes,
        image_size=(image_width, image_height),
        history=frame_history,
        val_ratio=val_ratio,
        seed=seed,
    )
    if len(train_ds) == 0:
        raise RuntimeError(f"No training episodes found under {episodes_dir}")

    n_red = sum(1 for r in train_ds.records if r.tape_color == "red")
    n_white = sum(1 for r in train_ds.records if r.tape_color == "white")
    print(f"[train] Experiment mode: {n_red} red + {n_white} white train episodes "
          f"({max_white_episodes} white cap)")

    preload_threshold = 25000
    preload_recs = 64
    if len(train_ds.records) <= preload_recs and train_ds.total_frames <= preload_threshold:
        print(f"[train] Preloading {len(train_ds.records)} train episodes into RAM.")
        train_ds.preload_all()
        for ds in (val_ds_all, val_ds_red, val_ds_white):
            if ds.records:
                ds.preload_all()

    def _loader(dataset, *, weighted: bool = False) -> DataLoader | None:
        if len(dataset) == 0:
            return None
        return DataLoader(
            dataset,
            batch_size=batch_size,
            sampler=(
                WeightedRandomSampler(
                    weights=torch.as_tensor(dataset.sample_weights, dtype=torch.double),
                    num_samples=len(dataset.sample_weights),
                    replacement=True,
                )
                if weighted else None
            ),
            shuffle=False,
            num_workers=num_workers,
            pin_memory=torch.cuda.is_available(),
            persistent_workers=num_workers > 0,
        )

    train_loader = _loader(train_ds, weighted=True)
    val_loader_all = _loader(val_ds_all)
    val_loader_red = _loader(val_ds_red)
    val_loader_white = _loader(val_ds_white)

    train_sessions = sorted({r.session_name for r in train_ds.records})
    val_sessions = sorted({r.session_name for r in val_ds_all.records})
    return train_loader, val_loader_all, val_loader_red, val_loader_white, train_sessions, val_sessions


@torch.no_grad()
def evaluate_model(model: nn.Module, loader: DataLoader | None, criterion: nn.Module, device: torch.device) -> dict[str, float]:
    if loader is None:
        return {"loss": math.nan, "mae_vx": math.nan, "mae_vy": math.nan, "mae_omega": math.nan}

    model.eval()
    total_loss = 0.0
    total_examples = 0
    abs_error = torch.zeros(3, dtype=torch.float64)

    for batch in loader:
        images = batch["image"].to(device)
        targets = batch["action"].to(device)
        preds = model(images)
        loss = criterion(preds, targets)
        batch_size = images.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_examples += batch_size
        abs_error += torch.abs(preds - targets).sum(dim=0).float().cpu()

    if total_examples == 0:
        return {"loss": math.nan, "mae_vx": math.nan, "mae_vy": math.nan, "mae_omega": math.nan}

    return {
        "loss": total_loss / total_examples,
        "mae_vx": float(abs_error[0].item() / total_examples),
        "mae_vy": float(abs_error[1].item() / total_examples),
        "mae_omega": float(abs_error[2].item() / total_examples),
    }


@torch.no_grad()
def evaluate_model_with_progress(
    model: nn.Module,
    loader: DataLoader | None,
    criterion: nn.Module,
    device: torch.device,
    *,
    epoch: int,
    epochs: int,
    show_progress: bool,
) -> dict[str, float]:
    if loader is None:
        return {"loss": math.nan, "mae_vx": math.nan, "mae_vy": math.nan, "mae_omega": math.nan}

    if not show_progress:
        return evaluate_model(model, loader, criterion, device)

    model.eval()
    total_loss = 0.0
    total_examples = 0
    abs_error = torch.zeros(3, dtype=torch.float64)
    start_time = time.perf_counter()

    bar = tqdm(
        total=len(loader.dataset),
        desc=f"Epoch {epoch:03d}/{epochs:03d} val",
        unit="sample",
        dynamic_ncols=True,
        leave=False,
    )

    try:
        for batch in loader:
            images = batch["image"].to(device)
            targets = batch["action"].to(device)
            preds = model(images)
            loss = criterion(preds, targets)

            batch_size = images.shape[0]
            total_loss += float(loss.item()) * batch_size
            total_examples += batch_size
            abs_error += torch.abs(preds - targets).sum(dim=0).float().cpu()

            elapsed = max(time.perf_counter() - start_time, 1e-6)
            bar.update(batch_size)
            bar.set_postfix(
                batch_loss=f"{float(loss.item()):.4f}",
                avg_loss=f"{total_loss / total_examples:.4f}",
                samples_per_s=f"{total_examples / elapsed:.1f}",
            )
    finally:
        bar.close()

    if total_examples == 0:
        return {"loss": math.nan, "mae_vx": math.nan, "mae_vy": math.nan, "mae_omega": math.nan}

    return {
        "loss": total_loss / total_examples,
        "mae_vx": float(abs_error[0].item() / total_examples),
        "mae_vy": float(abs_error[1].item() / total_examples),
        "mae_omega": float(abs_error[2].item() / total_examples),
    }


def train_epoch(
    model: nn.Module,
    loader: DataLoader,
    criterion: nn.Module,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    *,
    epoch: int,
    epochs: int,
    lr: float,
    show_progress: bool,
    freeze_encoder_blocks: int = 0,
) -> dict[str, float]:
    model.train()
    # Re-apply eval mode to frozen encoder blocks so their BatchNorm layers keep
    # using running statistics (fixed during fine-tuning) rather than batch statistics.
    if freeze_encoder_blocks > 0 and hasattr(model, "encoder"):
        for i in range(min(freeze_encoder_blocks, 4)):
            model.encoder[i].eval()
    total_loss = 0.0
    total_examples = 0
    abs_error = torch.zeros(3, dtype=torch.float64)
    start_time = time.perf_counter()

    bar = None
    if show_progress:
        bar = tqdm(
            total=len(loader.dataset),
            desc=f"Epoch {epoch:03d}/{epochs:03d} train",
            unit="sample",
            dynamic_ncols=True,
            leave=False,
        )

    try:
        for batch in loader:
            images = batch["image"].to(device)
            targets = batch["action"].to(device)
            preds = model(images)
            loss = criterion(preds, targets)

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()

            batch_size = images.shape[0]
            total_loss += float(loss.item()) * batch_size
            total_examples += batch_size
            abs_error += torch.abs(preds.detach() - targets).sum(dim=0).float().cpu()

            if bar is not None:
                elapsed = max(time.perf_counter() - start_time, 1e-6)
                bar.update(batch_size)
                bar.set_postfix(
                    batch_loss=f"{float(loss.item()):.4f}",
                    avg_loss=f"{total_loss / total_examples:.4f}",
                    lr=f"{lr:.2e}",
                    samples_per_s=f"{total_examples / elapsed:.1f}",
                )
    finally:
        if bar is not None:
            bar.close()

    denom = max(1, total_examples)
    return {
        "loss": total_loss / denom,
        "mae_vx": float(abs_error[0].item() / denom),
        "mae_vy": float(abs_error[1].item() / denom),
        "mae_omega": float(abs_error[2].item() / denom),
    }


def main() -> None:
    args = build_parser().parse_args()
    set_seed(args.seed)
    device = resolve_device(args.device)

    episodes_dir = resolve_episodes_dir(Path(args.episodes_dir))
    run_base_dir = Path(args.run_dir)
    run_dir = resolve_run_dir(run_base_dir)
    checkpoint_dir = run_dir / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    summary_path = run_dir / "training_summary.json"

    experiment_mode = args.max_white_episodes is not None
    val_loader_red: DataLoader | None = None
    val_loader_white: DataLoader | None = None

    if experiment_mode:
        train_loader, val_loader, val_loader_red, val_loader_white, train_sessions, val_sessions = (
            build_experiment_loaders(
                episodes_dir,
                max_white_episodes=args.max_white_episodes,
                val_ratio=args.val_ratio,
                seed=args.seed,
                batch_size=args.batch_size,
                num_workers=args.num_workers,
                frame_history=args.frame_history,
                image_width=args.image_width,
                image_height=args.image_height,
            )
        )
    else:
        train_loader, val_loader, train_sessions, val_sessions = build_loaders(
            episodes_dir,
            val_ratio=args.val_ratio,
            seed=args.seed,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
            frame_history=args.frame_history,
            image_width=args.image_width,
            tape_color=args.tape_color,
            image_height=args.image_height,
            use_masked_video=args.use_masked_video,
        )

    model_config = LoopPolicyConfig(
        image_width=args.image_width,
        image_height=args.image_height,
        frame_history=args.frame_history,
    )
    model = build_model(model_config).to(device)

    if args.finetune_from:
        _payload = torch.load(Path(args.finetune_from), map_location=device, weights_only=False)
        model.load_state_dict(_payload["model_state_dict"])
        print(f"[train] Fine-tuning from {args.finetune_from} (source epoch {_payload.get('epoch', '?')})")

    n_freeze = args.freeze_encoder_blocks
    if n_freeze > 0:
        _n = min(n_freeze, 4)
        for i in range(_n):
            for param in model.encoder[i].parameters():
                param.requires_grad = False
        n_frozen = sum(not p.requires_grad for p in model.parameters())
        n_total = sum(1 for _ in model.parameters())
        print(f"[train] Frozen encoder blocks 0–{_n - 1}  ({n_frozen}/{n_total} parameters frozen)")

    criterion = nn.HuberLoss(delta=args.huber_delta)
    trainable_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.AdamW(trainable_params, lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(1, args.epochs))

    if val_loader is None:
        print("[train] WARNING: Only one session available. Validation will be skipped.")

    print("[train] Sessions:", ", ".join(train_sessions) or "(none)")
    if val_sessions:
        print("[train] Validation sessions:", ", ".join(val_sessions))
    print(f"[train] Device: {device}")
    print(f"[train] Run base dir: {run_base_dir}")
    print(f"[train] Run dir: {run_dir}")
    print(f"[train] Train samples: {len(train_loader.dataset)}")
    if val_loader is not None:
        print(f"[train] Validation samples: {len(val_loader.dataset)}")

    history: list[dict[str, float]] = []
    best_metric = float("inf")
    best_epoch = -1
    interrupted = False

    try:
        for epoch in range(1, args.epochs + 1):
            current_lr = float(optimizer.param_groups[0]["lr"])
            train_metrics = train_epoch(
                model,
                train_loader,
                criterion,
                optimizer,
                device,
                epoch=epoch,
                epochs=args.epochs,
                lr=current_lr,
                show_progress=not args.no_progress,
                freeze_encoder_blocks=args.freeze_encoder_blocks,
            )
            val_metrics = (
                evaluate_model_with_progress(
                    model,
                    val_loader,
                    criterion,
                    device,
                    epoch=epoch,
                    epochs=args.epochs,
                    show_progress=not args.no_progress,
                )
                if val_loader is not None else train_metrics
            )
            val_red_metrics = (
                evaluate_model(model, val_loader_red, criterion, device)
                if val_loader_red is not None else None
            )
            val_white_metrics = (
                evaluate_model(model, val_loader_white, criterion, device)
                if val_loader_white is not None else None
            )
            scheduler.step()

            record: dict[str, float] = {
                "epoch": epoch,
                "train_loss": train_metrics["loss"],
                "train_mae_vx": train_metrics["mae_vx"],
                "train_mae_vy": train_metrics["mae_vy"],
                "train_mae_omega": train_metrics["mae_omega"],
                "val_loss": val_metrics["loss"],
                "val_mae_vx": val_metrics["mae_vx"],
                "val_mae_vy": val_metrics["mae_vy"],
                "val_mae_omega": val_metrics["mae_omega"],
                "lr": float(optimizer.param_groups[0]["lr"]),
            }
            if val_red_metrics is not None:
                record.update({
                    "val_loss_red": val_red_metrics["loss"],
                    "val_mae_red_vx": val_red_metrics["mae_vx"],
                    "val_mae_red_omega": val_red_metrics["mae_omega"],
                })
            if val_white_metrics is not None:
                record.update({
                    "val_loss_white": val_white_metrics["loss"],
                    "val_mae_white_vx": val_white_metrics["mae_vx"],
                    "val_mae_white_omega": val_white_metrics["mae_omega"],
                })
            history.append(record)

            msg = (
                f"[train] epoch {epoch:03d} "
                f"train_loss={record['train_loss']:.4f} "
                f"val_loss={record['val_loss']:.4f} "
                f"val_mae=[{record['val_mae_vx']:.4f}, {record['val_mae_vy']:.4f}, {record['val_mae_omega']:.4f}]"
            )
            if experiment_mode:
                msg += (
                    f" | red={record.get('val_loss_red', float('nan')):.4f}"
                    f" white={record.get('val_loss_white', float('nan')):.4f}"
                )
            print(msg)

            checkpoint_extra = {
                "train_sessions": train_sessions,
                "val_sessions": val_sessions,
                "history_length": args.frame_history,
                "image_size": [args.image_width, args.image_height],
            }

            save_checkpoint(
                checkpoint_dir / "last.pt",
                model,
                epoch=epoch,
                metrics=record,
                extra=checkpoint_extra,
            )

            metric = record["val_loss"] if not math.isnan(record["val_loss"]) else record["train_loss"]
            if metric < best_metric:
                best_metric = metric
                best_epoch = epoch
                save_checkpoint(
                    checkpoint_dir / "best.pt",
                    model,
                    epoch=epoch,
                    metrics=record,
                    extra=checkpoint_extra,
                )

            epoch_dir = checkpoint_dir / f"epoch_{epoch:03d}"
            epoch_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(checkpoint_dir / "last.pt", epoch_dir / "last.pt")
            shutil.copy2(checkpoint_dir / "best.pt", epoch_dir / "best.pt")

            with (epoch_dir / "metrics.json").open("w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "epoch": epoch,
                        "metrics": record,
                        "best_epoch_so_far": best_epoch,
                        "best_metric_so_far": best_metric,
                    },
                    handle,
                    indent=2,
                )

            write_training_summary(
                summary_path,
                device=device,
                args=args,
                model_config=model_config,
                train_sessions=train_sessions,
                val_sessions=val_sessions,
                history=history,
                best_epoch=best_epoch,
                best_metric=best_metric,
                interrupted=False,
            )
    except KeyboardInterrupt:
        interrupted = True
        print("\n[train] Interrupted by user. Partial checkpoints and history were saved.")
    finally:
        write_training_summary(
            summary_path,
            device=device,
            args=args,
            model_config=model_config,
            train_sessions=train_sessions,
            val_sessions=val_sessions,
            history=history,
            best_epoch=best_epoch,
            best_metric=best_metric,
            interrupted=interrupted,
        )

    print(f"[train] Saved checkpoints to {checkpoint_dir}")
    print(f"[train] Best epoch: {best_epoch}")


if __name__ == "__main__":
    main()
