"""outputs/data/*.csv ve generator_HD_estimates.csv dosyalarindan sekil uretir."""
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "outputs")

fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
for j, bus in enumerate([1, 3, 5]):
    d = pd.read_csv(os.path.join(out, "data", f"PmStep20_Bus{bus}.csv"))
    axes[0, j].plot(d.t, d.Tm, label="Tm")
    axes[0, j].plot(d.t, d.Te, label="Te")
    axes[0, j].set_title(f"Bus {bus}: Pm basamagi (t=1 s)")
    axes[0, j].legend()
    axes[0, j].grid(alpha=0.3)
    axes[1, j].plot(d.t, d.w - 1)
    axes[1, j].set_xlabel("t [s]")
    axes[1, j].grid(alpha=0.3)
axes[0, 0].set_ylabel("pu")
axes[1, 0].set_ylabel("w - 1 [pu]")
fig.tight_layout()
fig.savefig(os.path.join(out, "generator_responses.png"), dpi=130)

est = pd.read_csv(os.path.join(out, "generator_HD_estimates.csv"))
fig, ax = plt.subplots(1, 2, figsize=(11, 4))
for name, col, tv in [("H", "est_H", 5.5), ("D", "est_D", 1.5)]:
    a = ax[0 if name == "H" else 1]
    for m, g in est.groupby("method"):
        a.scatter(range(len(g)), g[col], label=m, s=45)
    a.axhline(tv, color="k", ls="--", label="gercek")
    a.set_title(f"{name} tahmini")
    a.set_xlabel("senaryo / makine indeksi")
    a.legend()
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(out, "estimates_vs_true.png"), dpi=130)
