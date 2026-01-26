#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# running command:
# python3 plot_reward_curve.py --csv merged_red_points_ordinates.csv --window 25 --out united_plot.png


import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(
        description="Plot reward and 25-episode moving average from merged_red_points_ordinates.csv"
    )
    parser.add_argument(
        "--csv",
        default="merged_red_points_ordinates.csv",
        help="Path to merged CSV (default: merged_red_points_ordinates.csv)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=25,
        help="Rolling window size for average reward (default: 25)",
    )
    parser.add_argument(
        "--out",
        default="reward_with_avg25.png",
        help="Output image path (default: reward_with_avg25.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive window (optional)",
    )
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    if "reward" not in df.columns:
        raise ValueError(f"CSV must contain column 'reward'. Found columns: {list(df.columns)}")

    y = df["reward"].to_numpy()
    x = range(1, len(y) + 1)

    avg = (
        pd.Series(y)
        .rolling(window=args.window, min_periods=1)
        .mean()
        .to_numpy()
    )

    plt.figure(figsize=(10, 4.5))
    plt.plot(x, y, marker="o", markersize=2.5, linewidth=1, label="Reward")
    plt.plot(x, avg, linewidth=2, label=f"Average reward ({args.window})")

    plt.xlabel("Index (concatenated points)")
    plt.ylabel("Reward")
    plt.title("Reward and rolling average")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(args.out, dpi=200)
    print(f"Saved plot to: {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()

