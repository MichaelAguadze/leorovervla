"""Tiny CNN policy for image-based path following."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import torch
from torch import nn

from . import DEFAULT_FRAME_HISTORY, DEFAULT_IMAGE_HEIGHT, DEFAULT_IMAGE_WIDTH


@dataclass(frozen=True)
class LoopPolicyConfig:
    """Architecture and input-shape settings for the CNN policy."""

    image_width: int = DEFAULT_IMAGE_WIDTH
    image_height: int = DEFAULT_IMAGE_HEIGHT
    frame_history: int = DEFAULT_FRAME_HISTORY
    action_dim: int = 3
    hidden_dim: int = 64  # kept for API compatibility; included in head_hidden_sizes
    dropout: float = 0.1
    # Full encoder channel widths — one entry per conv block.
    encoder_channels: tuple = (32, 64, 128, 128)
    # Hidden layer sizes in the MLP head (before the final action projection).
    head_hidden_sizes: tuple = (64, 32)
    # Whether Conv2d layers carry a bias term (False is correct when BN follows).
    conv_bias: bool = False

    def __post_init__(self) -> None:
        # Coerce to tuple so JSON-round-tripped lists still compare equal.
        object.__setattr__(self, "encoder_channels", tuple(self.encoder_channels))
        object.__setattr__(self, "head_hidden_sizes", tuple(self.head_hidden_sizes))

    @property
    def input_channels(self) -> int:
        return self.frame_history * 3


class ConvBlock(nn.Module):
    """Conv-BN-ReLU helper block."""

    def __init__(self, in_channels: int, out_channels: int, kernel_size: int, stride: int, padding: int, bias: bool = False):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=kernel_size, stride=stride,
                      padding=padding, bias=bias),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class LoopPolicyNet(nn.Module):
    """Tiny CNN that regresses normalized [vx, vy, omega]."""

    def __init__(self, config: LoopPolicyConfig | None = None):
        super().__init__()
        self.config = config or LoopPolicyConfig()

        # Build encoder from config so the architecture is fully captured.
        enc_blocks: list[nn.Module] = []
        c_in = self.config.input_channels
        for i, c_out in enumerate(self.config.encoder_channels):
            kernel = 5 if i == 0 else 3
            enc_blocks.append(ConvBlock(c_in, c_out, kernel_size=kernel, stride=2,
                                        padding=kernel // 2, bias=self.config.conv_bias))
            c_in = c_out
        enc_blocks.append(nn.AdaptiveAvgPool2d((1, 1)))
        self.encoder = nn.Sequential(*enc_blocks)

        # Build MLP head: Flatten → [hidden, ReLU, (Dropout after first)]... → action → Tanh
        head_layers: list[nn.Module] = [nn.Flatten()]
        fc_in = self.config.encoder_channels[-1]
        for i, h in enumerate(self.config.head_hidden_sizes):
            head_layers.append(nn.Linear(fc_in, h))
            head_layers.append(nn.ReLU(inplace=True))
            if i == 0:
                head_layers.append(nn.Dropout(self.config.dropout))
            fc_in = h
        head_layers.append(nn.Linear(fc_in, self.config.action_dim))
        head_layers.append(nn.Tanh())
        self.head = nn.Sequential(*head_layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.ndim != 4:
            raise ValueError(f"Expected a 4D tensor [B,C,H,W], got shape {tuple(x.shape)}")
        if x.shape[1] != self.config.input_channels:
            raise ValueError(
                f"Expected {self.config.input_channels} input channels, got {x.shape[1]}. "
                f"Frame history={self.config.frame_history}"
            )
        return self.head(self.encoder(x))


LoopCNNModel = LoopPolicyNet


def build_model(config: LoopPolicyConfig | None = None) -> LoopPolicyNet:
    """Create a fresh CNN policy model."""
    return LoopPolicyNet(config=config)


def save_checkpoint(
    path: Path,
    model: LoopPolicyNet,
    *,
    epoch: int,
    metrics: dict[str, float],
    extra: dict[str, object] | None = None,
) -> None:
    """Persist a checkpoint with model and metadata."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "epoch": epoch,
        "metrics": metrics,
        "model_config": asdict(model.config),
        "model_state_dict": model.state_dict(),
        "extra": extra or {},
    }
    torch.save(payload, path)


def _infer_config_from_state_dict(state_dict: dict, base_config: dict) -> dict:
    """Reconstruct architecture fields by inspecting state dict shapes.

    State dict is the ground truth for structural parameters; always override
    encoder_channels, head_hidden_sizes, and frame_history from it so that old
    checkpoints (saved before these fields existed) load correctly.
    """
    cfg = dict(base_config)

    # Encoder: collect output channels of each ConvBlock.
    enc_channels: list[int] = []
    i = 0
    while f"encoder.{i}.net.0.weight" in state_dict:
        enc_channels.append(int(state_dict[f"encoder.{i}.net.0.weight"].shape[0]))
        i += 1
    if enc_channels:
        cfg["encoder_channels"] = tuple(enc_channels)
        # Infer frame_history from input channel count of the first conv.
        in_ch = int(state_dict["encoder.0.net.0.weight"].shape[1])
        cfg["frame_history"] = in_ch // 3
        # Detect whether Conv layers were saved with bias.
        cfg["conv_bias"] = "encoder.0.net.0.bias" in state_dict

    # Head: all Linear layers except the last one are hidden layers.
    linear_keys = sorted(
        k for k in state_dict if k.startswith("head.") and k.endswith(".weight")
    )
    if len(linear_keys) >= 2:
        cfg["head_hidden_sizes"] = tuple(
            int(state_dict[k].shape[0]) for k in linear_keys[:-1]
        )

    return cfg


def load_checkpoint(
    path: Path, map_location: str | torch.device | None = None
) -> tuple[LoopPolicyNet, dict[str, object]]:
    """Load a checkpoint and return the instantiated model plus raw payload."""
    payload = torch.load(Path(path), map_location=map_location)
    state_dict = payload["model_state_dict"]

    # Infer full architecture from state dict so old checkpoints load correctly.
    config_dict = _infer_config_from_state_dict(state_dict, payload.get("model_config", {}))
    config = LoopPolicyConfig(**config_dict)
    model = LoopPolicyNet(config=config)
    model.load_state_dict(state_dict)
    return model, payload
