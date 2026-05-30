"""Select a sweep checkpoint and deploy it on the physical robot.

Reads a sweep_results.json produced by loop_cnn.sweep, shows the results table,
lets you pick a cell interactively (or auto-select the best), optionally exports
the checkpoint to a named path, then launches loop_cnn.drive.

Usage:
    # Interactive selection
    python -m loop_cnn.sweep_deploy \\
        --results runs/sweep/sweep_20260528_120000/sweep_results.json

    # Auto-select best white generalisation, export to models/, then drive
    python -m loop_cnn.sweep_deploy \\
        --results runs/sweep/sweep_20260528_120000/sweep_results.json \\
        --auto-best \\
        --export-dir models

    # Just print the drive command without launching
    python -m loop_cnn.sweep_deploy --results ... --dry-run
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def _fmt(v, width: int = 8) -> str:
    if v is None:
        return " " * (width - 3) + "n/a"
    try:
        return f"{float(v):{width}.4f}"
    except (TypeError, ValueError):
        return " " * (width - 3) + "n/a"


def _ok_cells(results: list[dict]) -> list[dict]:
    return sorted(
        (r for r in results if r.get("status") == "ok"),
        key=lambda r: (r["white_episodes"], r["epochs"]),
    )


def print_table(ok: list[dict]) -> None:
    if not ok:
        print("[deploy] No successful sweep cells found.")
        return

    header = (
        f"{'#':>3}  {'white_eps':>9}  {'epochs':>6}  {'freeze':>6}  {'val_all':>8}  "
        f"{'val_red':>8}  {'val_white':>9}  {'mae_ω':>7}  {'best_ep':>7}"
    )
    sep = "─" * len(header)
    print(f"\n{sep}")
    print(header)
    print(sep)
    for i, r in enumerate(ok):
        marker = " *" if i == _best_idx(ok) else "  "
        print(
            f"{i:>3}{marker} {r['white_episodes']:>9}  {r['epochs']:>6}"
            f"  {r.get('freeze_encoder_blocks', 0):>6}"
            f"  {_fmt(r.get('final_val_loss'))}"
            f"  {_fmt(r.get('final_val_loss_red'))}"
            f"  {_fmt(r.get('final_val_loss_white'), 9)}"
            f"  {_fmt(r.get('final_val_mae_omega'), 7)}"
            f"  {r.get('best_epoch', -1):>7}"
        )
    print(sep)
    print("  * = lowest val_white (best generalisation)\n")


def _best_idx(ok: list[dict]) -> int:
    candidates = [(i, r) for i, r in enumerate(ok) if r.get("final_val_loss_white") is not None]
    if not candidates:
        return 0
    return min(candidates, key=lambda t: t[1]["final_val_loss_white"])[0]


def _find_checkpoint(run_dir: str, prefer: str) -> Path | None:
    base = Path(run_dir) / "checkpoints"
    order = ["best.pt", "last.pt"] if prefer == "best" else ["last.pt", "best.pt"]
    for name in order:
        p = base / name
        if p.exists():
            return p
    return None


def _select_interactive(ok: list[dict]) -> dict | None:
    best_i = _best_idx(ok)
    while True:
        try:
            raw = input(
                f"  Select # (0–{len(ok) - 1}), Enter for best [{best_i}], or q to quit: "
            ).strip()
        except (EOFError, KeyboardInterrupt):
            return None

        if raw.lower() in {"q", "quit"}:
            return None
        if raw == "":
            return ok[best_i]
        try:
            idx = int(raw)
            if 0 <= idx < len(ok):
                return ok[idx]
            print(f"  Out of range. Choose 0–{len(ok) - 1}.")
        except ValueError:
            print("  Enter a number, Enter, or q.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Deploy a sweep checkpoint to the physical robot"
    )
    parser.add_argument(
        "--results", required=True, metavar="SWEEP_RESULTS_JSON",
        help="Path to sweep_results.json from a completed loop_cnn.sweep run",
    )
    parser.add_argument(
        "--cell", default=None, metavar="CELL_NAME",
        help="Directly select a sweep cell by name (e.g. w008_e020); skips interactive prompt",
    )
    parser.add_argument(
        "--auto-best", action="store_true",
        help="Automatically select the cell with the lowest val_loss_white",
    )
    parser.add_argument(
        "--checkpoint-type", choices=["best", "last"], default="best",
        help="Use best.pt (lowest val loss during training) or last.pt (final epoch). Default: best",
    )
    parser.add_argument(
        "--export-dir", default=None, metavar="DIR",
        help="Copy the checkpoint here with a descriptive filename before driving "
             "(e.g. models/). Useful for keeping a named copy of the deployed policy.",
    )
    # Drive options (passed through to loop_cnn.drive)
    parser.add_argument("--robot-ip",   default="192.168.2.12")
    parser.add_argument("--robot-port", type=int, default=8081)
    parser.add_argument("--loop-hz",    type=float, default=10.0)
    parser.add_argument("--smoothing",  type=float, default=0.65,
                        help="EMA smoothing factor for action (0 = no smoothing)")
    parser.add_argument("--vx-cap",    type=float, default=35.0)
    parser.add_argument("--vy-cap",    type=float, default=35.0)
    parser.add_argument("--omega-cap", type=float, default=25.0)
    parser.add_argument("--device",    default="auto")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the drive command but do not launch it",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    results_path = Path(args.results)
    if not results_path.exists():
        print(f"[deploy] ERROR: results file not found: {results_path}")
        sys.exit(1)

    sweep = _load(results_path)
    ok = _ok_cells(sweep.get("results", []))

    print()
    print("=" * 60)
    print("  CNN Sweep — Deploy to Robot")
    print("=" * 60)
    print(f"  Sweep timestamp : {sweep.get('timestamp', '?')}")
    print(f"  Base checkpoint : {sweep.get('checkpoint', '?')}")
    print(f"  Grid            : {sweep.get('white_episodes_grid')} white eps "
          f"× {sweep.get('epochs_grid')} epochs")
    print_table(ok)

    if not ok:
        sys.exit(1)

    # Select cell
    if args.cell:
        selected = next((r for r in ok if r.get("cell_name") == args.cell), None)
        if selected is None:
            print(f"[deploy] Cell '{args.cell}' not found. Available: {[r['cell_name'] for r in ok]}")
            sys.exit(1)
    elif args.auto_best:
        selected = ok[_best_idx(ok)]
        print(f"[deploy] Auto-selected best: {selected['cell_name']}")
    else:
        selected = _select_interactive(ok)
        if selected is None:
            print("[deploy] No cell selected. Exiting.")
            sys.exit(0)

    run_dir = selected.get("run_dir")
    if not run_dir:
        print(f"[deploy] Cell '{selected['cell_name']}' has no run_dir (may have failed).")
        sys.exit(1)

    checkpoint = _find_checkpoint(run_dir, prefer=args.checkpoint_type)
    if checkpoint is None:
        print(f"[deploy] No checkpoint found under {run_dir}/checkpoints/")
        sys.exit(1)

    # Optional export
    deploy_checkpoint = checkpoint
    if args.export_dir:
        export_dir = Path(args.export_dir)
        export_dir.mkdir(parents=True, exist_ok=True)
        w = selected["white_episodes"]
        e = selected["epochs"]
        f = selected.get("freeze_encoder_blocks", 0)
        fname = f"generalize_w{w:03d}_e{e:03d}_f{f}_{args.checkpoint_type}.pt"
        dest = export_dir / fname
        shutil.copy2(checkpoint, dest)
        print(f"[deploy] Checkpoint exported to {dest}")
        deploy_checkpoint = dest

    # Summary
    print()
    print(f"  Cell              : {selected['cell_name']}")
    print(f"  White episodes    : {selected['white_episodes']}")
    print(f"  Epochs trained    : {selected['epochs']}")
    print(f"  Frozen enc blocks : {selected.get('freeze_encoder_blocks', 0)}")
    print(f"  val_loss_red      : {selected.get('final_val_loss_red', 'n/a')}")
    print(f"  val_loss_white    : {selected.get('final_val_loss_white', 'n/a')}")
    print(f"  best_epoch        : {selected.get('best_epoch', 'n/a')}")
    print(f"  Checkpoint        : {deploy_checkpoint}")
    print(f"  Robot             : http://{args.robot_ip}:{args.robot_port}")
    print()

    cmd = [
        sys.executable, "-m", "loop_cnn.drive",
        "--checkpoint", str(deploy_checkpoint),
        "--robot-ip",   args.robot_ip,
        "--robot-port", str(args.robot_port),
        "--loop-hz",    str(args.loop_hz),
        "--smoothing",  str(args.smoothing),
        "--vx-cap",     str(args.vx_cap),
        "--vy-cap",     str(args.vy_cap),
        "--omega-cap",  str(args.omega_cap),
        "--device",     args.device,
    ]
    print(f"  Drive command:\n    {' '.join(cmd)}\n")

    if args.dry_run:
        print("[deploy] --dry-run: not launching. Copy the command above to run manually.")
        sys.exit(0)

    try:
        input("  Press Enter to launch drive session, or Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print("\n[deploy] Aborted.")
        sys.exit(0)

    print()
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
