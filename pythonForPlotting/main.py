"""
@author BaoLongDo
@brief  This program helps to visualize raw vs filtered data from I3G4250D sensor
@date   18 Aug 2025
"""

# main.py
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_log(path: Path) -> pd.DataFrame:
    """Return DataFrame with columns: i, rawRoll, filteredRoll (handles Excel/CSV and single-column CSV-in-Excel)."""
    if path.suffix.lower() in {".xlsx", ".xls"}:
        df = pd.read_excel(path)
    else:
        df = pd.read_csv(path)

    # If it's a single column like "i, rawRoll, filteredRoll", split it
    if df.shape[1] == 1:
        col = df.columns[0]
        df = df[col].astype(str).str.split(",", expand=True).iloc[:, :3]
        df.columns = ["i", "rawRoll", "filteredRoll"]

    # Normalize & coerce types
    df = df.rename(columns=lambda c: "".join(str(c).split()).lower())
    rename = {"i":"i", "index":"i", "sample":"i", "rawroll":"rawRoll", "raw":"rawRoll",
              "rollraw":"rawRoll", "filteredroll":"filteredRoll", "filtered":"filteredRoll",
              "rollfiltered":"filteredRoll", "filtroll":"filteredRoll"}
    df = df.rename(columns={k:v for k,v in rename.items() if k in df.columns})

    # If still ambiguous, take first 3 numeric columns
    need = {"i","rawRoll","filteredRoll"}
    if not need.issubset(df.columns):
        num_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
        if len(num_cols) >= 3:
            df = df[num_cols[:3]].copy()
            df.columns = ["i","rawRoll","filteredRoll"]
        else:
            raise ValueError(f"Columns not found. Got: {list(df.columns)}")

    # Types
    df["i"] = pd.to_numeric(df["i"], errors="coerce").astype("Int64")
    df["rawRoll"] = pd.to_numeric(df["rawRoll"], errors="coerce")
    df["filteredRoll"] = pd.to_numeric(df["filteredRoll"], errors="coerce")
    df = df.dropna().copy()
    df["i"] = df["i"].astype(int)
    return df

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input","-i", default="i3g4250d_log.xlsx", help="Excel or CSV file")
    ap.add_argument("--show", action="store_true", help="Show plots interactively")
    args = ap.parse_args()

    df = load_log(Path(args.input))
    df["residual"] = df["rawRoll"] - df["filteredRoll"]

    # --- stats ---
    raw_std = df["rawRoll"].std(ddof=1)
    filt_std = df["filteredRoll"].std(ddof=1)
    ratio = (raw_std / filt_std) if filt_std else np.nan
    db = 20*np.log10(ratio) if ratio and np.isfinite(ratio) and ratio>0 else np.nan

    print(f"\nSamples: {len(df)}")
    print(f"Raw mean/std:     {df['rawRoll'].mean():.6f} / {raw_std:.6f}")
    print(f"Filtered mean/std:{df['filteredRoll'].mean():.6f} / {filt_std:.6f}")
    print(f"Residual std:     {df['residual'].std(ddof=1):.6f}")
    print(f"Noise reduction:  ratio={ratio:.3f}  ({db:.2f} dB)\n")

    # --- plots ---
    outdir = Path("plots"); outdir.mkdir(exist_ok=True)
    plt.figure(); plt.plot(df["i"], df["rawRoll"], label="raw"); plt.plot(df["i"], df["filteredRoll"], label="filtered")
    plt.xlabel("sample"); plt.ylabel("roll"); plt.title("Raw vs Filtered Roll"); plt.legend(); plt.tight_layout()
    p1 = outdir/"raw_vs_filtered.png"; plt.savefig(p1, dpi=160); 
    if args.show: plt.show(); plt.close()

    plt.figure(); plt.plot(df["i"], df["residual"], label="residual")
    plt.xlabel("sample"); plt.ylabel("raw - filtered"); plt.title("Residual (High-frequency component)")
    plt.legend(); plt.tight_layout()
    p2 = outdir/"residual.png"; plt.savefig(p2, dpi=160);
    if args.show: plt.show(); plt.close()

    # Save a clean CSV
    clean_csv = Path(args.input).with_suffix(".parsed.csv")
    df.to_csv(clean_csv, index=False)
    print(f"Saved: {p1}\n       {p2}\n       {clean_csv}")

if __name__ == "__main__":
    main()
