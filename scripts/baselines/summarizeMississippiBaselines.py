"""Summarize the paired Mississippi replays with native measurement availability."""
import argparse
import csv
from pathlib import Path


METHODS = {
    "proposed": "Proposed",
    "overlap_ekf": "Overlap EKF",
    "overlap_iekf": "Overlap iterated EKF",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=Path, default=Path("output/mississippi_baselines_20261030"))
    folder = parser.parse_args().folder
    rows = []
    for drive in (1, 2):
        with (folder / f"drive{drive}_metrics.csv").open() as src:
            current = list(csv.DictReader(src))
        expected = {(str(drive), "both", method, window) for method in METHODS
                    for window in ("all", "after_2s")}
        keys = {(r["drive"], r["scenario"], r["method"], r["window"]) for r in current}
        if keys != expected or len(current) != len(expected):
            raise ValueError(f"Expected one native-input result per method and window for drive {drive}")
        rows.extend(current)
    with (folder / "combined_metrics.csv").open("w", newline="") as dst:
        writer = csv.DictWriter(dst, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    table = [r"\begin{tabular}{llrrr}\toprule",
             r"Sequence & Method & Position RMSE (m) & Maximum (m) & Heading RMSE (deg) \\ \midrule"]
    for r in rows:
        if r["window"] == "all":
            table.append(f"{r['drive']} & {METHODS[r['method']]} & {float(r['rmseM']):.3f} & "
                         f"{float(r['maximumM']):.3f} & {float(r['headingRmseDeg']):.3f} " + r"\\")
    table.append(r"\bottomrule\end{tabular}")
    (folder / "mississippi_baseline_results.tex").write_text("\n".join(table) + "\n")


if __name__ == "__main__":
    main()
