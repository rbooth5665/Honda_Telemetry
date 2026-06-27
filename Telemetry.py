import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

RED, BLUE, GREEN = "#E03A2F", "#1C7293", "#2C5F2D"
KMH_TO_MPH = 0.621371
GEAR_EDGES = [(114, 1), (87, 2), (74, 3), (66, 4), (59, 5), (0, 6)]

class Ride:
    def __init__(self, csv_path, out_dir="analysis"):
        self.path = csv_path
        self.name = os.path.splitext(os.path.basename(csv_path))[0]
        self.out_dir = out_dir
        df = pd.read_csv(csv_path)
        df = df[df["rpm"] >= 0].reset_index(drop=True)   # drop no-data rows
        self.df = self._ensure_channels(df)

    @staticmethod
    def _ensure_channels(df):
        if "speed_kmh" not in df and "t11_b13" in df:
            df["speed_kmh"] = df["t11_b13"]
        if "speed_mph" not in df and "speed_kmh" in df:
            df["speed_mph"] = df["speed_kmh"] * KMH_TO_MPH
        derive = {"tps": ("t11_b3", lambda v: v / 16 * 10),
                  "ect_c": ("t11_b5", lambda v: v - 40),
                  "iat_c": ("t11_b7", lambda v: v - 40),
                  "map_kpa": ("t11_b9", lambda v: v),
                  "batt": ("t11_b12", lambda v: v / 10)}
        for col, (raw, fn) in derive.items():
            if col not in df and raw in df:
                df[col] = fn(df[raw])
        if "gear" not in df:
            df["gear"] = df.apply(lambda r: Ride._gear(r["rpm"], r["speed_kmh"]), axis=1)
        return df
    
    @staticmethod
    def _gear(rpm, speed_kmh):
        if speed_kmh < 3 or rpm < 1000:
            return 0
        ratio = rpm / speed_kmh
        return next(g for edge, g in GEAR_EDGES if ratio > edge)
    
    def _has(self, *cols):
        return all(c in self.df for c in cols)
    
    def _save(self, fig, suffix):
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, f"{self.name}_{suffix}.png")
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        return path
    
    def summary(self):
        d = self.df
        moving = d[d["speed_kmh"] > 3]
        return {
            "duration_s": round(d["elapsed_s"].max(), 1),
            "samples": len(d),
            "avg_hz": round(len(d) / d["elapsed_s"].max(), 1),
            "top_speed_mph": round(d["speed_mph"].max(), 1),
            "peak_rpm": int(d["rpm"].max()),
            "avg_moving_mph": round(moving["speed_mph"].mean(), 1) if len(moving) else 0,
        }

    def find_pull(self, min_gain_kmh=40):
        d = self.df.reset_index(drop=True)
        sp, t = d["speed_kmh"].values, d["elapsed_s"].values
        best = None
        for i in range(len(d)):
            if sp[i] < 5:                                
                peak, pj, j = sp[i], i, i
                while j < len(d) and t[j] - t[i] < 30:
                    if sp[j] > peak:
                        peak, pj = sp[j], j
                    if peak - sp[j] > 15 and peak > min_gain_kmh:
                        break
                    j += 1
                if best is None or peak - sp[i] > best["gain"]:
                    best = {"start": i, "peak": pj, "gain": peak - sp[i]}
        return best

    def accel_stats(self):
        pull = self.find_pull()
        if not pull:
            return None
        d = self.df.reset_index(drop=True)
        run = d.iloc[pull["start"]:].reset_index(drop=True)
        launch = run[run["speed_kmh"] > 0]
        if launch.empty:
            return None
        run = run.iloc[launch.index[0]:].reset_index(drop=True)
        run["tr"] = run["elapsed_s"] - run["elapsed_s"].iloc[0]
        out = {}
        for mph in [30, 40, 50, 60, 70, 80]:
            hit = run[run["speed_kmh"] >= mph / KMH_TO_MPH]
            out[f"0-{mph}mph_s"] = round(hit["tr"].iloc[0], 1) if len(hit) else None
        a = (run["speed_kmh"].diff() / 3.6) / run["elapsed_s"].diff()
        out["peak_g"] = round(a.max() / 9.81, 2)
        out["top_mph"] = round(run["speed_mph"].max(), 0)
        out["peak_rpm"] = int(run["rpm"].max())
        return out

    def plot_overview(self):
        d = self.df
        fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
        ax[0].plot(d["elapsed_s"], d["speed_mph"], color=RED, lw=0.8)
        ax[0].set_ylabel("Speed (mph)"); ax[0].set_title(f"{self.name} — overview", fontweight="bold")
        ax[1].plot(d["elapsed_s"], d["rpm"], color=BLUE, lw=0.7); ax[1].set_ylabel("RPM")
        ax[2].plot(d["elapsed_s"], d["gear"], color=GREEN, lw=0, marker=".", ms=2)
        ax[2].set_ylabel("Gear"); ax[2].set_yticks(range(7)); ax[2].set_xlabel("elapsed (s)")
        for a in ax: a.grid(alpha=0.3)
        return self._save(fig, "overview")
    
    def plot_pull(self):
        pull = self.find_pull()
        if not pull:
            return None
        d = self.df.reset_index(drop=True)
        run = d.iloc[pull["start"]:pull["peak"] + 1].reset_index(drop=True)
        run["tr"] = run["elapsed_s"] - run["elapsed_s"].iloc[0]
        fig, ax = plt.subplots(figsize=(13, 6))
        ax.plot(run["tr"], run["speed_mph"], color=RED, lw=2.5, label="Speed (mph)")
        ax.set_xlabel("Time from launch (s)"); ax.set_ylabel("Speed (mph)", color=RED)
        ax.tick_params(axis="y", labelcolor=RED); ax.grid(alpha=0.3)
        ax2 = ax.twinx()
        ax2.plot(run["tr"], run["rpm"], color=BLUE, lw=1.5, alpha=0.7)
        ax2.set_ylabel("RPM", color=BLUE); ax2.tick_params(axis="y", labelcolor=BLUE)
        prev = None
        for _, r in run.iterrows():
            g = int(r["gear"])
            if prev is not None and g != prev:
                ax.axvline(r["tr"], color="gray", ls="--", lw=1, alpha=0.5)
            prev = g
        st = self.accel_stats()
        if st and st.get("0-60mph_s"):
            ax.axvline(st["0-60mph_s"], color="black", ls=":", lw=1.5)
            ax.text(st["0-60mph_s"] + 0.1, run["speed_mph"].max() * 0.8,
                    f"0-60: {st['0-60mph_s']}s", fontweight="bold")
        ax.set_title(f"{self.name} — the pull", fontweight="bold")
        return self._save(fig, "pull")
    
    def plot_accel_milestones(self):
        st = self.accel_stats()
        if not st:
            return None
        mph = [m for m in [30, 40, 50, 60, 70, 80] if st.get(f"0-{m}mph_s")]
        times = [st[f"0-{m}mph_s"] for m in mph]
        fig, ax = plt.subplots(figsize=(11, 5))
        bars = ax.bar([f"0-{m}" for m in mph], times, color=RED)
        for b, t in zip(bars, times):
            ax.text(b.get_x() + b.get_width() / 2, t + 0.05, f"{t}s", ha="center", fontweight="bold")
        ax.set_ylabel("Time (s)"); ax.set_title(f"{self.name} — acceleration (mph)", fontweight="bold")
        ax.grid(alpha=0.3, axis="y")
        return self._save(fig, "accel")

    def plot_warmup(self):
        if not self._has("ect_c", "batt"):
            return None
        d = self.df; t = d["elapsed_s"]
        fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
        ax[0].plot(t, d["ect_c"] * 9 / 5 + 32, color=RED, lw=1.5); ax[0].set_ylabel("Coolant °F")
        ax[0].set_title(f"{self.name} — warmup & health", fontweight="bold")
        if "iat_c" in d:
            ax[1].plot(t, d["iat_c"] * 9 / 5 + 32, color=BLUE, lw=1.5)
        ax[1].set_ylabel("Intake °F")
        ax[2].plot(t, d["batt"], color=GREEN, lw=1.2); ax[2].set_ylabel("Battery V")
        ax[2].set_xlabel("elapsed (s)")
        for a in ax: a.grid(alpha=0.3)
        return self._save(fig, "warmup")

    def plot_engine_map(self):
        if not self._has("tps", "map_kpa"):
            return None
        d = self.df[self.df["rpm"] > 1000]
        fig, ax = plt.subplots(figsize=(11, 7))
        sc = ax.scatter(d["rpm"], d["tps"], c=d["map_kpa"], cmap="plasma", s=12, alpha=0.6)
        ax.set_xlabel("RPM"); ax.set_ylabel("Throttle %")
        ax.set_title(f"{self.name} — engine operating map (color = MAP/load)", fontweight="bold")
        plt.colorbar(sc, label="MAP (kPa)"); ax.grid(alpha=0.3)
        return self._save(fig, "engine_map")
    
    def plot_speed_by_gear(self):
        d = self.df
        t, sp, g = d["elapsed_s"].values, d["speed_mph"].values, d["gear"].values
        pts = np.array([t, sp]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap="viridis", norm=plt.Normalize(0, 6))
        lc.set_array(g[:-1]); lc.set_linewidth(2.5)
        fig, ax = plt.subplots(figsize=(13, 6))
        ax.add_collection(lc); ax.set_xlim(t.min(), t.max()); ax.set_ylim(-2, sp.max() + 5)
        ax.set_xlabel("elapsed (s)"); ax.set_ylabel("Speed (mph)"); ax.grid(alpha=0.3)
        ax.set_title(f"{self.name} — speed colored by gear", fontweight="bold")
        plt.colorbar(lc, label="gear", ticks=range(7))
        return self._save(fig, "speed_by_gear")
    
    def plot_rpm_histogram(self):
        d = self.df[self.df["rpm"] > 1000]
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.hist(d["rpm"], bins=40, color=RED, alpha=0.8, edgecolor="white", linewidth=0.5)
        ax.axvline(d["rpm"].mean(), color="black", ls="--", lw=1.5)
        ax.text(d["rpm"].mean() + 150, ax.get_ylim()[1] * 0.9, f"avg {d['rpm'].mean():.0f}")
        ax.set_xlabel("RPM"); ax.set_ylabel("Samples")
        ax.set_title(f"{self.name} — RPM distribution", fontweight="bold"); ax.grid(alpha=0.3, axis="y")
        return self._save(fig, "rpm_hist")

    def plot_gear_separation(self):
        d = self.df[(self.df["speed_kmh"] > 3) & (self.df["rpm"] > 1000)]
        fig, ax = plt.subplots(figsize=(11, 6))
        sc = ax.scatter(d["speed_kmh"], d["rpm"], c=d["gear"], cmap="viridis", s=8)
        ax.set_xlabel("Speed (km/h)"); ax.set_ylabel("RPM")
        ax.set_title(f"{self.name} — gear separation (each gear = a band)", fontweight="bold")
        plt.colorbar(sc, label="gear"); ax.grid(alpha=0.3)
        return self._save(fig, "gear_sep")
    
    def plot_unknown_bytes(self):
        d = self.df
        known_raw = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13}
        cols = []
        for c in d.columns:
            if c.startswith("b") and c[1:].isdigit():
                cols.append(c)
            elif c.startswith("t11_b") and int(c.split("b")[1]) not in known_raw:
                cols.append(c)
        cols = sorted(set(cols), key=lambda c: int(c.split("b")[-1]))
        if not cols:
            return None
 
        n = len(cols)
        fig, ax = plt.subplots(n, 3, figsize=(15, 2.2 * n), squeeze=False)
        for r, c in enumerate(cols):
            v = pd.to_numeric(d[c], errors="coerce")
            ax[r][0].plot(d["elapsed_s"], v, color=BLUE, lw=0.6)
            ax[r][0].set_ylabel(c, fontweight="bold")
            ax[r][1].scatter(d["rpm"], v, s=4, alpha=0.4, color=RED)
            ax[r][2].scatter(d["speed_kmh"], v, s=4, alpha=0.4, color=GREEN)
            for a in ax[r]:
                a.grid(alpha=0.3)
        ax[0][0].set_title("vs time"); ax[0][1].set_title("vs RPM")
        ax[0][2].set_title("vs speed")
        ax[-1][0].set_xlabel("elapsed (s)"); ax[-1][1].set_xlabel("RPM")
        ax[-1][2].set_xlabel("speed (km/h)")
        fig.suptitle(f"{self.name} — unmapped byte discovery", fontweight="bold", y=1.0)
        fig.tight_layout()
        return self._save(fig, "unknown_bytes")
    
    def report(self):
        plots = [self.plot_overview, self.plot_pull, self.plot_accel_milestones,
                 self.plot_warmup, self.plot_engine_map, self.plot_speed_by_gear,
                 self.plot_rpm_histogram, self.plot_gear_separation,
                 self.plot_unknown_bytes]
        made = []
        for p in plots:
            try:
                path = p()
                if path:
                    made.append(path)
            except Exception as e:
                print(f"  skipped {p.__name__}: {e}")
        return made
    
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("usage: python telemetry.py <ride.csv>")
        sys.exit(1)
    r = Ride(sys.argv[1])
    print("Summary:", r.summary())
    st = r.accel_stats()
    if st:
        print("Acceleration:", st)
    files = r.report()
    print(f"\nGenerated {len(files)} graphs in ./{r.out_dir}/:")
    for f in files:
        print(f"  {f}")










