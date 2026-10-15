"""
データと設定の置き場所の既定値 (リポジトリルート基準なので、どこから実行しても同じ場所を指す)。

実機 WHILL の rosbag2 はリポジトリに含めていない (公開権限の確認が必要なため)。
置き場所は環境変数 WHILL_ROSBAG_DIR か各コマンドの --rosbag-path で指定する。
"""

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_ROSBAG_DIR = os.environ.get("WHILL_ROSBAG_DIR", str(REPO_ROOT / "data" / "real_whill"))
DEFAULT_SEARCH_SPACE = str(REPO_ROOT / "config" / "optimizer" / "search_space.yaml")
