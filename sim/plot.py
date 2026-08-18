#!/usr/bin/env python3
"""
Turn the CSVs written by the PIDX simulations into figures.

Not part of the library. Reads only what sim/*.c produced; it never
recomputes a control result in Python, so a figure cannot disagree with the
C study it illustrates.

Usage:
    make plots            # from sim/
    python3 plot.py       # same thing

Inputs (whichever exist):
    results/rules.csv        one row per (plant, rule)
    results/rules_trace.csv  per-sample traces for one plant
    results/robust.csv       one row per (plant, rule, axis, factor)
    results/autotune.csv     one row per (plant, method, noise)

Outputs: results/*.png

Dependencies are matplotlib only. numpy is used where it is already
required by matplotlib anyway. Missing input files are skipped with a
message rather than treated as an error, so this works after running any
single simulation.
"""

import csv
import os
import sys
from collections import defaultdict, OrderedDict

RESULTS = "results"

try:
    import matplotlib
    matplotlib.use("Agg")          # headless: no display in a build sandbox
    import matplotlib.pyplot as plt
except ImportError:
    sys.stderr.write("matplotlib not installed - no plots produced\n")
    sys.exit(0)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def load(name):
    """Read a CSV into a list of dicts, or return None if it is absent."""
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        print("  skip %-18s (not generated yet)" % name)
        return None
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def fnum(row, key):
    """Float from a CSV cell, or None when the cell is blank.

    Blank is meaningful in these files: a failed run writes empty metric
    columns rather than a zero, because zero IAE would be an excellent
    score and would quietly poison every average.
    """
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def save(fig, name):
    path = os.path.join(RESULTS, name)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print("  wrote %s" % path)


# --------------------------------------------------------------------------
# 1. step responses of every rule on one plant
# --------------------------------------------------------------------------

def plot_traces(rows, plant_name=None):
    """The picture behind the rule table: what the loops actually did.

    The traced plant is whichever one sim_rules.c chose to record; it is
    passed in rather than hardcoded here. An earlier version of this file
    captioned the figure "easy L/T=0.1" while the data was actually the
    typical plant - the mismatch was visible only because the x-axis ran to
    25 s and the easy plant's horizon is 20 s.
    """
    series = OrderedDict()
    for r in rows:
        series.setdefault(r["series"], ([], [], []))
        t = fnum(r, "t")
        y = fnum(r, "y")
        u = fnum(r, "u")
        if t is None:
            continue
        series[r["series"]][0].append(t)
        series[r["series"]][1].append(y)
        series[r["series"]][2].append(u)

    if not series:
        return

    sp = fnum(rows[0], "setpoint") or 1.0

    fig, (ax_y, ax_u) = plt.subplots(
        2, 1, figsize=(10, 8), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]})

    for name, (t, y, u) in series.items():
        ax_y.plot(t, y, linewidth=1.3, label=name)
        ax_u.plot(t, u, linewidth=1.0)

    ax_y.axhline(sp, color="black", linestyle="--", linewidth=0.8)
    # The +/-2% band is the settling criterion the C code measures against,
    # so drawing it makes the settle_s column checkable by eye.
    ax_y.axhspan(sp * 0.98, sp * 1.02, color="grey", alpha=0.15)
    ax_y.set_ylabel("process value")
    ax_y.set_title("Step response by tuning rule (plant: %s)"
                   % (plant_name if plant_name else "as traced"))
    ax_y.legend(fontsize=8, ncol=3)
    ax_y.grid(alpha=0.3)

    ax_u.set_ylabel("controller output")
    ax_u.set_xlabel("time [s]")
    ax_u.grid(alpha=0.3)
    # Control effort is half the story: a rule that wins on IAE by slamming
    # the actuator is not a rule you can ship.
    ax_u.set_title("Control effort (the cost of the response above)",
                   fontsize=9)

    save(fig, "fig1_step_responses.png")


# --------------------------------------------------------------------------
# 2. normalised IAE per rule per plant
# --------------------------------------------------------------------------

def plot_rules(rows):
    """Grouped bars: IAE normalised by K*T so plants can share an axis."""
    plants = []
    rules = []
    scale = {}
    data = defaultdict(dict)

    for r in rows:
        p = r["plant"]
        rule = r["rule"]
        if p not in plants:
            plants.append(p)
        if rule not in rules:
            rules.append(rule)
        iae = fnum(r, "iae")
        if iae is None:
            continue
        # Normalise by K*T: without it the furnace (IAE ~ 18) would set the
        # scale and every fast plant would be an invisible sliver.
        kp_scale = scale.get(p)
        data[p][rule] = iae

    if not data:
        return

    # K*T per plant, taken from the plant names used by sim_common.c.
    kt = {"easy L/T=0.1": 2.0 * 1.0,
          "typical L/T=0.3": 2.0 * 1.0,
          "hard L/T=1.0": 1.0 * 1.0,
          "slow L/T=0.1": 1.0 * 10.0,
          "furnace L/T=0.2": 3.0 * 20.0}

    n_rule = len(rules)
    width = 0.8 / n_rule
    fig, ax = plt.subplots(figsize=(12, 5.5))

    xs = range(len(plants))
    for i, rule in enumerate(rules):
        vals = []
        for p in plants:
            v = data[p].get(rule)
            vals.append((v / kt.get(p, 1.0)) if v is not None else 0.0)
        ax.bar([x + (i * width) for x in xs], vals, width, label=rule)

    ax.set_xticks([x + 0.4 - (width / 2) for x in xs])
    ax.set_xticklabels(plants, fontsize=9)
    ax.set_ylabel("IAE / (K*T)   (lower is better)")
    ax.set_title("Tuning rules on an EXACT model - performance only, "
                 "not robustness")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(alpha=0.3, axis="y")
    # A missing bar is a rule the library refused for that plant (e.g.
    # Cohen-Coon outside L/T in [0.1, 1]); that refusal is a feature.
    save(fig, "fig2_rule_iae.png")


# --------------------------------------------------------------------------
# 3. robustness: survival heat map
# --------------------------------------------------------------------------

def plot_robust(rows):
    """Where each rule breaks when the plant moves under it."""
    rules = []
    factors = []
    axes = []
    for r in rows:
        if r["rule"] not in rules:
            rules.append(r["rule"])
        if r["factor"] not in factors:
            factors.append(r["factor"])
        if r["axis"] not in axes:
            axes.append(r["axis"])

    factors.sort(key=float)

    # survived fraction per (rule, axis, factor) across the plants
    cnt = defaultdict(lambda: [0, 0])
    for r in rows:
        key = (r["rule"], r["axis"], r["factor"])
        cnt[key][1] += 1
        if r["survived"] == "1":
            cnt[key][0] += 1

    fig, axs = plt.subplots(1, len(axes), figsize=(15, 5), sharey=True)
    if len(axes) == 1:
        axs = [axs]

    for ai, axis_name in enumerate(axes):
        grid = []
        for rule in rules:
            row = []
            for f in factors:
                ok, tot = cnt[(rule, axis_name, f)]
                row.append((ok / tot) if tot else 0.0)
            grid.append(row)

        ax = axs[ai]
        im = ax.imshow(grid, aspect="auto", cmap="RdYlGn",
                       vmin=0.0, vmax=1.0)
        ax.set_xticks(range(len(factors)))
        ax.set_xticklabels(factors, fontsize=8)
        ax.set_xlabel("%s multiplier" % axis_name)
        ax.set_title("plant %s changed" % axis_name, fontsize=10)
        if ai == 0:
            ax.set_yticks(range(len(rules)))
            ax.set_yticklabels(rules, fontsize=8)
        # Annotate so the figure is readable in greyscale too.
        for i in range(len(rules)):
            for j in range(len(factors)):
                v = grid[i][j]
                if v < 0.999:
                    ax.text(j, i, "%d%%" % round(v * 100), ha="center",
                            va="center", fontsize=6)

    fig.suptitle("Fraction of plants surviving, per rule "
                 "(green = always stable, red = always failed)")
    fig.colorbar(im, ax=axs, fraction=0.02, pad=0.02)
    save(fig, "fig3_robustness.png")


# --------------------------------------------------------------------------
# 4. the headline: performance rank vs robustness rank
# --------------------------------------------------------------------------

def plot_rank_inversion(rules_rows, robust_rows):
    """Slope chart of the two rankings. This is the finding of phase 18."""
    kt = {"easy L/T=0.1": 2.0, "typical L/T=0.3": 2.0, "hard L/T=1.0": 1.0,
          "slow L/T=0.1": 10.0, "furnace L/T=0.2": 60.0}

    perf = defaultdict(list)
    for r in rules_rows:
        iae = fnum(r, "iae")
        if iae is not None:
            perf[r["rule"]].append(iae / kt.get(r["plant"], 1.0))

    surv = defaultdict(lambda: [0, 0])
    for r in robust_rows:
        surv[r["rule"]][1] += 1
        if r["survived"] == "1":
            surv[r["rule"]][0] += 1

    common = [r for r in perf if r in surv and perf[r]]
    if len(common) < 3:
        return

    by_perf = sorted(common, key=lambda r: sum(perf[r]) / len(perf[r]))
    by_rob = sorted(common, key=lambda r: -surv[r][0] / surv[r][1])

    fig, ax = plt.subplots(figsize=(9, 6.5))
    for rule in common:
        a = by_perf.index(rule) + 1
        b = by_rob.index(rule) + 1
        moved = abs(a - b) >= 4
        ax.plot([0, 1], [a, b],
                color=("crimson" if moved else "steelblue"),
                linewidth=(2.2 if moved else 1.0),
                alpha=(1.0 if moved else 0.5),
                marker="o", markersize=5)
        ax.text(-0.03, a, rule, ha="right", va="center", fontsize=9)
        ax.text(1.03, b, rule, ha="left", va="center", fontsize=9)

    ax.set_xlim(-0.55, 1.55)
    ax.set_ylim(len(common) + 0.5, 0.5)      # rank 1 at the top
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["rank on an\nEXACT model",
                        "rank on\nROBUSTNESS"], fontsize=10)
    ax.set_ylabel("rank (1 = best)")
    ax.set_title("Low IAE on a perfect model does not survive a wrong model\n"
                 "red = moved four or more places", fontsize=11)
    ax.grid(alpha=0.25, axis="y")
    save(fig, "fig4_rank_inversion.png")


# --------------------------------------------------------------------------
# 5. auto-tune accuracy and what it costs
# --------------------------------------------------------------------------

def plot_autotune(rows):
    """Identification error next to the IAE penalty it actually caused."""
    ok = [r for r in rows if r.get("ok") == "1"]
    if not ok:
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # Left: identification error vs noise. Plotting ONLY the gain error here
    # would badly mislead: the step test nails K to ~0.3% while getting the
    # dead time wrong by up to +82%, and it is the dead time that costs the
    # IAE in the right-hand panel. So both are drawn - gain dashed, worst
    # parameter solid.
    for method, colour in (("relay", "tab:red"), ("step", "tab:blue")):
        gain_pts = defaultdict(list)
        worst_pts = defaultdict(list)
        for r in ok:
            if r["method"] != method:
                continue
            sigma = fnum(r, "noise_sigma")
            if sigma is None:
                continue
            if method == "relay":
                errs = [fnum(r, "ku_err_pct"), fnum(r, "pu_err_pct")]
                gain = fnum(r, "ku_err_pct")
            else:
                errs = [fnum(r, "k_err_pct"), fnum(r, "t_err_pct"),
                        fnum(r, "l_err_pct")]
                gain = fnum(r, "k_err_pct")
            errs = [abs(e) for e in errs if e is not None]
            if gain is not None:
                gain_pts[sigma].append(abs(gain))
            if errs:
                worst_pts[sigma].append(max(errs))

        if gain_pts:
            xs = sorted(gain_pts)
            ys = [sum(gain_pts[x]) / len(gain_pts[x]) for x in xs]
            ax1.plot(xs, ys, marker="o", linestyle="--", color=colour,
                     alpha=0.6,
                     label="%s: gain only (%s)"
                           % (method, "Ku" if method == "relay" else "K"))
        if worst_pts:
            xs = sorted(worst_pts)
            ys = [sum(worst_pts[x]) / len(worst_pts[x]) for x in xs]
            ax1.plot(xs, ys, marker="o", color=colour,
                     label="%s: worst parameter" % method)

    ax1.set_xlabel("measurement noise sigma [process units]")
    ax1.set_ylabel("mean identification error [%]")
    ax1.set_title("How wrong is the identified model?\n"
                  "dashed = gain only, solid = worst parameter", fontsize=11)
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # right: the penalty that error actually caused
    for method, colour in (("relay", "tab:red"), ("step", "tab:blue")):
        pts = defaultdict(list)
        for r in ok:
            if r["method"] != method:
                continue
            sigma = fnum(r, "noise_sigma")
            pen = fnum(r, "penalty_pct")
            if sigma is not None and pen is not None:
                pts[sigma].append(pen)
        if not pts:
            continue
        xs = sorted(pts)
        ys = [sum(pts[x]) / len(pts[x]) for x in xs]
        ax2.plot(xs, ys, marker="s", color=colour, label=method)

    ax2.axhline(0.0, color="black", linewidth=0.8)
    ax2.set_xlabel("measurement noise sigma [process units]")
    ax2.set_ylabel("extra closed-loop IAE vs an exact model [%]")
    ax2.set_title("What that error COST\n(same rule both times)", fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3)

    save(fig, "fig5_autotune.png")


# --------------------------------------------------------------------------

def main():
    if not os.path.isdir(RESULTS):
        sys.stderr.write("no %s/ directory - run the simulations first\n"
                         % RESULTS)
        return 1

    print("plotting from %s/" % RESULTS)

    rules = load("rules.csv")
    trace = load("rules_trace.csv")
    robust = load("robust.csv")
    auto = load("autotune.csv")

    if trace:
        # sim_rules.c traces exactly one plant. Identify it by matching the
        # trace horizon against the plant table, so the caption cannot drift
        # away from the data.
        t_max = max((fnum(r, "t") or 0.0) for r in trace)
        horizons = {20.0: "easy L/T=0.1", 25.0: "typical L/T=0.3",
                    40.0: "hard L/T=1.0", 150.0: "slow L/T=0.1",
                    400.0: "furnace L/T=0.2"}
        traced = None
        for h, nm in horizons.items():
            if abs(t_max - h) < (0.05 * h):
                traced = nm
                break
        plot_traces(trace, traced)
    if rules:
        plot_rules(rules)
    if robust:
        plot_robust(robust)
    if rules and robust:
        plot_rank_inversion(rules, robust)
    if auto:
        plot_autotune(auto)

    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
