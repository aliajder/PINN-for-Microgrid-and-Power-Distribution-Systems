"""Estimate H and D for labeled synchronous-machine traces via the swing-equation PINN."""

import argparse
import glob
import os
import sys

import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from SYNC_MAIN import REPO_ROOT, SYNC, parse_true_hd  # noqa: E402
import deepxde as dde  # noqa: E402


def list_machine_csvs(data_dir, cases):
    paths = sorted(glob.glob(os.path.join(data_dir, "1208_*.csv")))
    if not paths:
        raise FileNotFoundError("No 1208_*.csv files in %s" % data_dir)
    if cases:
        selected = []
        for path in paths:
            name = os.path.basename(path)
            if any(token in name for token in cases):
                selected.append(path)
        if not selected:
            raise FileNotFoundError("No cases matched %s" % cases)
        return selected
    return paths


def relative_error(pred, true):
    if true is None:
        return None
    if abs(true) < 1e-12:
        return abs(pred - true)
    return abs(pred - true) / abs(true)


def estimate_one(path, args):
    index = os.path.splitext(os.path.basename(path))[0]
    model = SYNC(
        path=path,
        st=args.st,
        et=args.et,
        index=index,
        prompt_on_dat_conflict=False,
        period=args.period,
    )
    model.build(
        transform="all",
        net=dde.nn.FNN([6] + [50] * 3 + [3], "swish", "Glorot uniform"),
        lr=args.lr,
        iterations=args.iterations,
    )
    model.train()
    pred_H, pred_D = model.latest_variables()
    true_H, true_D = model.true_H, model.true_D
    if true_H is None:
        true_H, true_D = parse_true_hd(path)
    return {
        "file": os.path.basename(path),
        "true_H": true_H,
        "true_D": true_D,
        "pred_H": pred_H,
        "pred_D": pred_D,
        "rel_error_D": relative_error(pred_D, true_D),
        "variable_log": os.path.relpath(model.fnamevar_path, REPO_ROOT),
    }


def parse_args():
    parser = argparse.ArgumentParser(description="PINN swing-equation D estimation")
    parser.add_argument("--iterations", type=int, default=4000)
    parser.add_argument("--period", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--st", type=float, default=0.0)
    parser.add_argument("--et", type=float, default=1.0)
    parser.add_argument("--cases", nargs="*", default=None, help="Substring filters, e.g. SyncOn_H1.0D0.0")
    parser.add_argument(
        "--data-dir",
        default=os.path.join(REPO_ROOT, "data", "synchronous-machine"),
    )
    parser.add_argument(
        "--output",
        default=os.path.join(REPO_ROOT, "results", "D_estimates.csv"),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    rows = []
    for path in list_machine_csvs(args.data_dir, args.cases):
        print("Estimating D for", os.path.basename(path))
        rows.append(estimate_one(path, args))
        pd.DataFrame(rows).to_csv(args.output, index=False)
    print(pd.DataFrame(rows).to_string(index=False))
    print("Wrote", args.output)


if __name__ == "__main__":
    main()
