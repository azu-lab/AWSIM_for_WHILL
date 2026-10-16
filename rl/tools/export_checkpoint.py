#!/usr/bin/env python3
"""
Make a publishable package of a trained checkpoint (e.g. for a GitHub Release).

    python3 rl/tools/export_checkpoint.py --logdir logs/rl/shinjuku_box_30k --checkpoint latest \
        --name shinjuku_box_dreamerv3 --eval-dir logs/rl/shinjuku_box_30k/eval --archive

The package is a small run directory, so that the usual commands work on it:

    <out>/run.yaml, task.yaml, progress.json
    <out>/checkpoints/<step>/dreamerv3.pkl     without the optimizer state (opt/*)
    <out>/MODEL_CARD.md, LICENSE.txt
    <out>.tar.gz + <out>.tar.gz.sha256        with --archive

    python3 rl/evaluate.py --logdir <out>                  # evaluate
    python3 -u rl/train.py --logdir <out> --steps 60000    # continue training (fresh optimizer,
                                                           # empty replay buffer)

Only the DreamerV3 agent is supported (its checkpoint layout).
"""

import argparse
import hashlib
import json
import pickle
import shutil
import subprocess
import sys
import tarfile
from datetime import date
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

LICENSE_TEXT = """\
The trained model in this directory (checkpoints/*/dreamerv3.pkl) is licensed under the
Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0):
https://creativecommons.org/licenses/by-nc/4.0/legalcode

It was trained on camera images rendered from the West Shinjuku map of AWSIM
(TIER IV, Inc. / Autoware Foundation, CC BY-NC), with the code of AWSIM for WHILL
(https://github.com/shoumexi-03/AWSIM_for_WHILL, Apache License 2.0) and DreamerV3
(https://github.com/danijar/dreamerv3, MIT License).

Attribution: "AWSIM for WHILL - DreamerV3 policy, shoumexi-03, CC BY-NC 4.0".
The file is a Python pickle: load it only from a source you trust.
"""


def git_commit() -> str:
    try:
        rev = subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(REPO_ROOT), "status", "--porcelain"],
                               capture_output=True, text=True).stdout.strip()
        return rev + (" (with uncommitted changes)" if dirty else "")
    except Exception:  # noqa: BLE001
        return "unknown"


def eval_summary(eval_dir: Path, checkpoint: str):
    import csv
    path = eval_dir / "results.csv"
    if not path.exists():
        return None
    rows = [r for r in csv.DictReader(open(path)) if r["checkpoint"] in (checkpoint, "")]
    if not rows:
        return None
    n = len(rows)
    rate = lambda o: sum(r["outcome"] == o for r in rows) / n  # noqa: E731
    return {"episodes": n, "success": rate("success"), "collision": rate("collision"), "timeout": rate("timeout"),
            "mean_reward": sum(float(r["reward"]) for r in rows) / n,
            "mean_length": sum(int(r["length"]) for r in rows) / n}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--logdir", required=True, help="run directory of rl/train.py")
    p.add_argument("--checkpoint", default="latest")
    p.add_argument("--name", required=True, help="package name, e.g. shinjuku_box_dreamerv3")
    p.add_argument("--out", default=None, help="output directory (default: pretrained/<name>)")
    p.add_argument("--eval-dir", default=None, help="evaluate.py output with results.csv, summarized in the model card")
    p.add_argument("--archive", action="store_true", help="also write <out>.tar.gz and its SHA-256")
    p.add_argument("--note", default="", help="text of a 'Notes' section of the model card (limitations, ...)")
    args = p.parse_args()

    logdir = Path(args.logdir)
    run = yaml.safe_load((logdir / "run.yaml").read_text())
    if run["agent"] != "dreamerv3":
        sys.exit(f"only DreamerV3 checkpoints are supported (agent of {logdir}: {run['agent']})")
    names = sorted(d.name for d in (logdir / "checkpoints").iterdir() if d.is_dir())
    ckpt = names[-1] if args.checkpoint == "latest" else args.checkpoint
    if ckpt not in names:
        sys.exit(f"checkpoint {ckpt} not found; available: {', '.join(names)}")
    out = Path(args.out or REPO_ROOT / "pretrained" / args.name)
    if out.exists():
        shutil.rmtree(out)
    (out / "checkpoints" / ckpt).mkdir(parents=True)

    src = logdir / "checkpoints" / ckpt
    with open(src / "dreamerv3.pkl", "rb") as f:
        data = pickle.load(f)
    before = sum(getattr(v, "nbytes", 0) for v in data["params"].values())
    data["params"] = {k: v for k, v in data["params"].items() if not k.startswith("opt/")}
    after = sum(getattr(v, "nbytes", 0) for v in data["params"].values())
    with open(out / "checkpoints" / ckpt / "dreamerv3.pkl", "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    shutil.copyfile(src / "dreamerv3_state.json", out / "checkpoints" / ckpt / "dreamerv3_state.json")
    print(f"parameters: {before / 1e6:.0f} MB -> {after / 1e6:.0f} MB (optimizer state removed)")

    steps = json.loads((src / "dreamerv3_state.json").read_text())["step"]
    episodes = sum(1 for line in open(logdir / "episodes.jsonl") if json.loads(line)["step"] <= steps)
    shutil.copyfile(logdir / "task.yaml", out / "task.yaml")
    (out / "run.yaml").write_text(yaml.safe_dump({
        "agent": run["agent"], "agent_config": run["agent_config"],
        "command": run.get("command", ""),
        "exported_from": {"logdir": str(logdir), "checkpoint": ckpt, "steps": steps, "episodes": episodes,
                          "commit": git_commit(), "date": date.today().isoformat()},
    }, sort_keys=False))
    (out / "progress.json").write_text(json.dumps({"steps": steps, "episodes": episodes, "checkpoint": ckpt}, indent=2))
    (out / "LICENSE.txt").write_text(LICENSE_TEXT)

    task = yaml.safe_load((logdir / "task.yaml").read_text())
    ev = eval_summary(Path(args.eval_dir), ckpt) if args.eval_dir else None
    ev_text = ("not evaluated" if ev is None else
               f"{ev['episodes']} episodes without learning (`rl/evaluate.py`): success {ev['success']:.0%}, "
               f"collision {ev['collision']:.0%}, timeout {ev['timeout']:.0%}, mean reward {ev['mean_reward']:.1f}, "
               f"mean length {ev['mean_length']:.0f} steps")
    (out / "MODEL_CARD.md").write_text(f"""# {args.name}

DreamerV3 policy for the WHILL Model CR in AWSIM for WHILL: front camera (64x64 RGB) -> (v, w).

| | |
|---|---|
| Task | `{task.get('name')}` ({task.get('description', '')}); `task.yaml` |
| Scene | `{task.get('scene', '')}` (default player of AWSIM for WHILL) |
| Training | {steps} environment steps, {episodes} episodes, `rl/train.py --agent dreamerv3` |
| Configuration | `run.yaml` (`agent_config`) |
| Evaluation | {ev_text} |
| Code | AWSIM for WHILL, commit {git_commit()} |
| File | `checkpoints/{ckpt}/dreamerv3.pkl` — parameters without the optimizer state ({after / 1e6:.0f} MB) |
| License | CC BY-NC 4.0 (`LICENSE.txt`) |

Use (from the repository root, AWSIM running with the default player):

```bash
D="env IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh"
$D python3 rl/evaluate.py --logdir pretrained/{args.name} --episodes 10
$D python3 -u rl/train.py --logdir pretrained/{args.name} --steps {steps * 2}   # continue training
```

Continuing the training starts with a fresh optimizer and an empty replay buffer. Results depend
on the vehicle parameters, the scene (obstacle placement) and the task file; they are those of
the repository at the commit above.
""" + (f"\n## Notes\n\n{args.note.strip()}\n" if args.note.strip() else ""))
    print(f"package: {out}")

    if args.archive:
        archive = out.parent / f"{out.name}.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            tar.add(out, arcname=out.name)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        (out.parent / f"{archive.name}.sha256").write_text(f"{digest}  {archive.name}\n")
        print(f"archive: {archive} ({archive.stat().st_size / 1e6:.0f} MB), sha256 {digest}")


if __name__ == "__main__":
    main()
