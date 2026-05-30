"""Grid sweep: white episodes × epochs × freeze strategy for color generalization.

Each cell fine-tunes a red-tape checkpoint on all-red + N white episodes for E epochs
with F encoder blocks frozen, then reports val_loss_red (forgetting) and
val_loss_white (generalization).

Freeze strategies
-----------------
f0 – no freezing: all parameters fine-tuned (highest plasticity, most forgetting risk)
f2 – freeze encoder blocks 0–1: preserves early color/edge detectors, adapts geometry
f4 – freeze all encoder blocks: only the action head is trained (fastest, least forgetting)

Usage:
    python -m loop_cnn.sweep \\
        --checkpoint runs/cnn_v2/.../checkpoints/best.pt \\
        --episodes-dir data/leorover_cnn_red/episodes \\
        --white-episodes 2 4 8 16 32 \\
        --epochs 5 10 20 40 \\
        --freeze-strategies 0 2 4

The sweep creates runs/sweep/sweep_TIMESTAMP/ with one subdirectory per cell and a
sweep_results.json summary.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from itertools import product
from pathlib import Path


def run_cell(
    *,
    white_episodes: int,
    epochs: int,
    freeze_encoder_blocks: int,
    checkpoint: str,
    episodes_dir: str,
    cell_dir: Path,
    seed: int,
    device: str,
    batch_size: int,
    lr: float,
    val_ratio: float,
) -> dict:
    """Run one sweep cell and return its result dict."""
    cell_dir.mkdir(parents=True, exist_ok=True)
    log_path = cell_dir / "stdout.txt"

    cmd = [
        sys.executable, "-m", "loop_cnn.train",
        "--episodes-dir", episodes_dir,
        "--run-dir", str(cell_dir),
        "--epochs", str(epochs),
        "--max-white-episodes", str(white_episodes),
        "--finetune-from", checkpoint,
        "--freeze-encoder-blocks", str(freeze_encoder_blocks),
        "--seed", str(seed),
        "--device", device,
        "--batch-size", str(batch_size),
        "--lr", str(lr),
        "--val-ratio", str(val_ratio),
        "--no-progress",
    ]

    cell_name = cell_dir.name
    print(
        f"[sweep] {cell_name}: {white_episodes} white eps, {epochs} epochs, "
        f"{freeze_encoder_blocks} frozen blocks ...",
        flush=True,
    )

    with log_path.open("w") as log_fh:
        proc = subprocess.run(cmd, stdout=log_fh, stderr=subprocess.STDOUT, text=True)

    # train.py creates a timestamped subdir inside cell_dir
    subdirs = sorted(cell_dir.glob("run_*"))
    if not subdirs:
        return _failed(white_episodes, epochs, freeze_encoder_blocks, cell_name,
                       proc.returncode, "no run subdir created")

    actual_run_dir = subdirs[-1]
    summary_path = actual_run_dir / "training_summary.json"
    if not summary_path.exists():
        return _failed(white_episodes, epochs, freeze_encoder_blocks, cell_name,
                       proc.returncode, "training_summary.json missing")

    with summary_path.open() as f:
        summary = json.load(f)

    history = summary.get("history", [])
    final = history[-1] if history else {}

    return {
        "white_episodes": white_episodes,
        "epochs": epochs,
        "freeze_encoder_blocks": freeze_encoder_blocks,
        "cell_name": cell_name,
        "status": "interrupted" if summary.get("interrupted") else "ok",
        "returncode": proc.returncode,
        "run_dir": str(actual_run_dir),
        "epochs_completed": summary.get("epochs_completed", 0),
        "best_epoch": summary.get("best_epoch", -1),
        "best_val_loss": summary.get("best_metric"),
        "final_train_loss": final.get("train_loss"),
        "final_val_loss": final.get("val_loss"),
        "final_val_loss_red": final.get("val_loss_red"),
        "final_val_loss_white": final.get("val_loss_white"),
        "final_val_mae_omega": final.get("val_mae_omega"),
        "final_val_mae_red_omega": final.get("val_mae_red_omega"),
        "final_val_mae_white_omega": final.get("val_mae_white_omega"),
        "history": history,
    }


def _failed(white_episodes, epochs, freeze_blocks, cell_name, returncode, reason) -> dict:
    return {
        "white_episodes": white_episodes,
        "epochs": epochs,
        "freeze_encoder_blocks": freeze_blocks,
        "cell_name": cell_name,
        "status": "failed",
        "returncode": returncode,
        "error": reason,
    }


def _fmt(v) -> str:
    if v is None:
        return "   n/a  "
    try:
        return f"{float(v):8.4f}"
    except (TypeError, ValueError):
        return "   n/a  "


def print_table(results: list[dict]) -> None:
    ok = [r for r in results if r.get("status") == "ok"]
    if not ok:
        print("[sweep] No successful runs to display.")
        for r in results:
            if r.get("status") != "ok":
                print(f"  FAILED {r.get('cell_name')}: {r.get('error', r.get('returncode'))}")
        return

    header = (
        f"{'white_eps':>9} {'epochs':>6} {'freeze':>6}"
        f" {'val_all':>8} {'val_red':>8} {'val_white':>9}"
        f" {'mae_ω':>7} {'best_ep':>7}"
    )
    sep = "=" * len(header)
    print(f"\n{sep}")
    print(header)
    print(sep)
    for r in sorted(ok, key=lambda x: (x["white_episodes"], x["epochs"], x.get("freeze_encoder_blocks", 0))):
        print(
            f"{r['white_episodes']:>9} {r['epochs']:>6} {r.get('freeze_encoder_blocks', 0):>6}"
            f" {_fmt(r.get('final_val_loss'))}"
            f" {_fmt(r.get('final_val_loss_red'))}"
            f" {_fmt(r.get('final_val_loss_white')):>9}"
            f" {_fmt(r.get('final_val_mae_omega')):>7}"
            f" {r.get('best_epoch', -1):>7}"
        )
    print(sep)

    best = min(
        (r for r in ok if r.get("final_val_loss_white") is not None),
        key=lambda r: r["final_val_loss_white"],
        default=None,
    )
    if best:
        print(
            f"\n[sweep] Best white generalisation: {best['cell_name']} "
            f"(val_white={best['final_val_loss_white']:.4f}, "
            f"val_red={best.get('final_val_loss_red') or float('nan'):.4f}, "
            f"freeze={best.get('freeze_encoder_blocks', 0)})"
        )
    print()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sweep white episodes × epochs × freeze strategy for color generalisation"
    )
    parser.add_argument(
        "--checkpoint", required=True,
        help="Pre-trained red-tape checkpoint (.pt) to fine-tune from",
    )
    parser.add_argument(
        "--episodes-dir", default="data/leorover_cnn_red/episodes",
        help="Episodes root; must contain both red and white tape episodes",
    )
    parser.add_argument(
        "--white-episodes", type=int, nargs="+", default=[2, 4, 8, 16, 32],
        metavar="N",
        help="White episode counts to sweep (default: 2 4 8 16 32)",
    )
    parser.add_argument(
        "--epochs", type=int, nargs="+", default=[5, 10, 20, 40],
        metavar="E",
        help="Epoch counts to sweep (default: 5 10 20 40)",
    )
    parser.add_argument(
        "--freeze-strategies", type=int, nargs="+", default=[0],
        metavar="F", choices=[0, 1, 2, 3, 4],
        help=(
            "Encoder blocks to freeze during fine-tuning (default: 0 = no freezing). "
            "Pass multiple values to sweep: e.g. --freeze-strategies 0 2 4. "
            "0=none  2=freeze blocks 0-1 (early color layers)  4=freeze all encoder"
        ),
    )
    parser.add_argument(
        "--run-dir", default="runs/sweep",
        help="Parent for the timestamped sweep output directory",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    return parser


def main() -> None:
    args = build_parser().parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_dir = Path(args.run_dir) / f"sweep_{timestamp}"
    sweep_dir.mkdir(parents=True, exist_ok=True)

    white_eps_sorted = sorted(set(args.white_episodes))
    epochs_sorted = sorted(set(args.epochs))
    freeze_sorted = sorted(set(args.freeze_strategies))
    grid = list(product(white_eps_sorted, epochs_sorted, freeze_sorted))

    print(
        f"[sweep] {len(grid)} cells — "
        f"white_episodes={white_eps_sorted}, epochs={epochs_sorted}, freeze={freeze_sorted}"
    )
    print(f"[sweep] Checkpoint : {args.checkpoint}")
    print(f"[sweep] Episodes   : {args.episodes_dir}")
    print(f"[sweep] Output dir : {sweep_dir}")
    print()

    results: list[dict] = []
    for white_eps, n_epochs, freeze_blocks in grid:
        cell_name = f"w{white_eps:03d}_e{n_epochs:03d}_f{freeze_blocks}"
        cell_dir = sweep_dir / cell_name
        result = run_cell(
            white_episodes=white_eps,
            epochs=n_epochs,
            freeze_encoder_blocks=freeze_blocks,
            checkpoint=args.checkpoint,
            episodes_dir=args.episodes_dir,
            cell_dir=cell_dir,
            seed=args.seed,
            device=args.device,
            batch_size=args.batch_size,
            lr=args.lr,
            val_ratio=args.val_ratio,
        )
        results.append(result)
        v_all = result.get("final_val_loss") or float("nan")
        v_red = result.get("final_val_loss_red") or float("nan")
        v_white = result.get("final_val_loss_white") or float("nan")
        print(
            f"[sweep] {cell_name} {result['status']:>11}: "
            f"val_all={v_all:.4f}  val_red={v_red:.4f}  val_white={v_white:.4f}"
        )

    results_path = sweep_dir / "sweep_results.json"
    sweep_meta = {
        "sweep_dir": str(sweep_dir),
        "checkpoint": args.checkpoint,
        "episodes_dir": args.episodes_dir,
        "white_episodes_grid": white_eps_sorted,
        "epochs_grid": epochs_sorted,
        "freeze_strategies_grid": freeze_sorted,
        "seed": args.seed,
        "timestamp": timestamp,
        "num_cells": len(grid),
        "results": results,
    }
    with results_path.open("w") as f:
        json.dump(sweep_meta, f, indent=2)

    print(f"\n[sweep] Results saved to {results_path}")
    print_table(results)

    print(
        f"  To deploy a checkpoint to the robot:\n"
        f"    python -m loop_cnn.sweep_deploy --results {results_path}\n"
        f"  Add --auto-best to skip the selection prompt, or --dry-run to preview the drive command.\n"
    )


if __name__ == "__main__":
    main()
