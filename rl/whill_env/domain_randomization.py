"""
Domain Randomization for Camera Images

カメラ画像に対してランダムな視覚変換を適用し、
Sim-to-Real転移性能を向上させる。

変換一覧:
  1. Brightness  (明るさ)     : 0.8 ~ 1.3倍
  2. Contrast    (コントラスト) : 0.8 ~ 1.2倍
  3. Hue         (色相)       : -0.1 ~ 0.1
  4. Saturation  (彩度)       : 0.8 ~ 1.2倍
  5. Gaussian Noise (ノイズ)   : std 0 ~ 5
"""

import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DRConfig:
    """Domain Randomization パラメータ設定"""
    # Brightness
    brightness_min: float = 0.8
    brightness_max: float = 1.3
    # Contrast
    contrast_min: float = 0.8
    contrast_max: float = 1.2
    # Hue (HSV色相シフト, -0.1~0.1 は -18~18 度に相当)
    hue_min: float = -0.1
    hue_max: float = 0.1
    # Saturation
    saturation_min: float = 0.8
    saturation_max: float = 1.2
    # Gaussian Noise
    noise_std_min: float = 0.0
    noise_std_max: float = 5.0


class DomainRandomization:
    """
    カメラ画像に対するDomain Randomization。

    毎ステップ呼び出され、ランダムなパラメータで画像を変換する。
    入力・出力ともに uint8 (0-255) のRGB画像。
    """

    def __init__(self, config: Optional[DRConfig] = None):
        self.config = config or DRConfig()
        self._rng = np.random.default_rng()

    def apply(self, image: np.ndarray) -> np.ndarray:
        """
        画像にDomain Randomizationを適用する。

        Args:
            image: uint8 RGB画像 (H, W, 3)

        Returns:
            変換後の uint8 RGB画像 (H, W, 3)
        """
        cfg = self.config

        # float32に変換 (0-255)
        img = image.astype(np.float32)

        # 1. Brightness (明るさ)
        brightness = self._rng.uniform(cfg.brightness_min, cfg.brightness_max)
        img = img * brightness

        # 2. Contrast (コントラスト)
        contrast = self._rng.uniform(cfg.contrast_min, cfg.contrast_max)
        mean = img.mean()
        img = (img - mean) * contrast + mean

        # 3. Hue / Saturation (色相・彩度)
        hue_shift = self._rng.uniform(cfg.hue_min, cfg.hue_max)
        sat_factor = self._rng.uniform(cfg.saturation_min, cfg.saturation_max)

        if abs(hue_shift) > 1e-4 or abs(sat_factor - 1.0) > 1e-4:
            # クリップしてからHSVに変換
            img_uint8 = np.clip(img, 0, 255).astype(np.uint8)
            hsv = cv2.cvtColor(img_uint8, cv2.COLOR_RGB2HSV).astype(np.float32)

            # Hue: OpenCVでは0-180の範囲
            hsv[:, :, 0] = (hsv[:, :, 0] + hue_shift * 180) % 180
            # Saturation
            hsv[:, :, 1] = hsv[:, :, 1] * sat_factor

            hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
            hsv = hsv.astype(np.uint8)
            img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).astype(np.float32)

        # 4. Gaussian Noise (ガウシアンノイズ)
        noise_std = self._rng.uniform(cfg.noise_std_min, cfg.noise_std_max)
        if noise_std > 0.1:
            noise = self._rng.normal(0, noise_std, img.shape).astype(np.float32)
            img = img + noise

        # クリップして uint8 に戻す
        img = np.clip(img, 0, 255).astype(np.uint8)

        return img
