"""
Interactive Circadian Rhythm Analysis Pipeline for BeeSpectrograms  v3.0
------------------------------------------------------------------------
Usage:
    python claudeHelper_20260315_circadian_prototype.py

CSV format expected:
    spec[0, 0]   = 0 (placeholder)
    spec[0, 1:]  = Unix epoch timestamps (seconds)
    spec[1:, 0]  = frequency bins (Hz)
    spec[1:, 1:] = power values (dB)

Supports both:
  - Spectrogram_1.csv … Spectrogram_6.csv  (standard)
  - YYYYMMDD_HHMMSS_YYYYMMDD_HHMMSS_ch1.csv … ch6.csv  (chunked)
"""

import sys
import os
import re
import datetime
import traceback
import numpy as np
import pandas as pd
from scipy.signal import lombscargle

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QComboBox, QCheckBox, QScrollArea, QSplitter, QGroupBox,
    QSpinBox, QDoubleSpinBox, QGridLayout, QLineEdit, QMessageBox, QSizePolicy,
    QDialog, QTableWidget, QTableWidgetItem, QHeaderView,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.dates as mdates

# Max time columns / frequency rows to display (downsampled for speed); does not affect computation
MAX_DISPLAY_COLS = 1500
MAX_DISPLAY_ROWS = 512

# ── regex for chunked filenames ───────────────────────────────────────────────
_CHUNK_RE = re.compile(r'^(\d{8}_\d{6})_(\d{8}_\d{6})_ch(\d+)\.csv$')


# ══════════════════════════════════════════════════════════════════════════════
# Data loading helpers
# ══════════════════════════════════════════════════════════════════════════════

def _csv_to_float32(path):
    """Load a spectrogram CSV into a float32 array.

    Reads as float64 with the C engine (pandas is faster that way) then casts.
    Falls back to a pure read_csv if something goes wrong.
    """
    mb = os.path.getsize(path) / 1_048_576
    print(f"    reading {os.path.basename(path)}  ({mb:.1f} MB) …", flush=True)
    t0 = _now()
    try:
        arr = pd.read_csv(path, header=None, dtype=np.float64,
                          engine='c', low_memory=False).values.astype(np.float32)
    except Exception:
        arr = pd.read_csv(path, header=None).values.astype(np.float32)
    print(f"    → {arr.shape}  loaded in {_now()-t0:.1f}s", flush=True)
    return arr


def _now():
    import time
    return time.perf_counter()


def load_standard_spectrograms(folder):
    import time
    print(f"\n[load_standard] folder: {folder}", flush=True)
    t_total = time.perf_counter()
    specs = []
    for c in range(1, 7):
        path = os.path.join(folder, f"Spectrogram_{c}.csv")
        if os.path.exists(path):
            print(f"  ch{c}: loading …", flush=True)
            t0 = time.perf_counter()
            arr = _csv_to_float32(path)
            print(f"  ch{c}: done ({time.perf_counter()-t0:.1f}s)", flush=True)
            specs.append((c, arr))
        else:
            print(f"  ch{c}: not found, skipping", flush=True)
            specs.append((c, None))
    print(f"[load_standard] total: {time.perf_counter()-t_total:.1f}s\n", flush=True)
    return specs


def load_chunked_spectrograms(folder):
    import time
    print(f"\n[load_chunked] folder: {folder}", flush=True)
    t_total = time.perf_counter()

    chunks = {}
    for fname in sorted(os.listdir(folder)):
        m = _CHUNK_RE.match(fname)
        if not m:
            continue
        ch = int(m.group(3))
        try:
            s = datetime.datetime.strptime(m.group(1), '%Y%m%d_%H%M%S')
            e = datetime.datetime.strptime(m.group(2), '%Y%m%d_%H%M%S')
        except ValueError:
            continue
        chunks.setdefault(ch, []).append((s, e, os.path.join(folder, fname)))

    if not chunks:
        print("[load_chunked] no matching files found", flush=True)
        return []

    results = []
    for ch in range(1, 7):
        if ch not in chunks:
            results.append((ch, None))
            continue
        pieces = sorted(chunks[ch], key=lambda x: x[0])
        print(f"  ch{ch}: {len(pieces)} chunk(s)", flush=True)
        t0 = time.perf_counter()

        # Collect all piece arrays then concatenate once (avoids O(n²) copies)
        arrays = []
        for i, (_, _, path) in enumerate(pieces):
            print(f"    chunk {i+1}/{len(pieces)} …", flush=True)
            try:
                arr = _csv_to_float32(path)
            except Exception as exc:
                print(f"    WARNING: could not read {path}: {exc}", flush=True)
                continue
            if arrays:
                arrays.append(arr[:, 1:])   # drop the repeated freq column
            else:
                arrays.append(arr)

        if not arrays:
            results.append((ch, None))
            continue

        print(f"  ch{ch}: concatenating {len(arrays)} array(s) …", flush=True)
        combined = np.concatenate(arrays, axis=1)
        print(f"  ch{ch}: shape {combined.shape}  ({time.perf_counter()-t0:.1f}s)", flush=True)
        results.append((ch, combined))

    print(f"[load_chunked] total: {time.perf_counter()-t_total:.1f}s\n", flush=True)
    return results


# ══════════════════════════════════════════════════════════════════════════════
# Spec array accessors
# ══════════════════════════════════════════════════════════════════════════════

def spec_freqs(spec):  return spec[1:, 0]
def spec_times(spec):  return spec[0, 1:]
def spec_power(spec):  return spec[1:, 1:]


def rebuild_spec(freqs, times, power):
    """Reconstruct the packed spec array, preserving the power dtype (float32 or float64)."""
    dt  = power.dtype
    out = np.empty((len(freqs) + 1, len(times) + 1), dtype=dt)
    out[0, 0] = 0
    out[0, 1:] = times
    out[1:, 0] = freqs
    out[1:, 1:] = power
    return out


# ══════════════════════════════════════════════════════════════════════════════
# Pipeline processing functions
# ══════════════════════════════════════════════════════════════════════════════

def apply_freq_filter(spec, min_hz, max_hz):
    freqs = spec_freqs(spec)
    mask = (freqs >= min_hz) & (freqs <= max_hz)
    if mask.all():
        return spec
    return rebuild_spec(freqs[mask], spec_times(spec), spec_power(spec)[mask, :])


def detect_noise_bands(spec, high_power_pct=95, low_cv_threshold=0.05):
    power = spec_power(spec)
    row_means = np.nanmean(power, axis=1)
    row_stds  = np.nanstd(power,  axis=1)
    cv        = row_stds / (np.abs(row_means) + 1e-7)
    high_power_mask = row_means > np.nanpercentile(row_means, high_power_pct)
    low_cv_mask     = cv < low_cv_threshold
    return high_power_mask | low_cv_mask, row_means, cv


def parse_manual_bands(bands_str):
    bands = []
    for token in bands_str.split():
        token = token.strip()
        if '-' in token:
            parts = token.split('-', 1)
            try:
                lo, hi = float(parts[0]), float(parts[1])
                bands.append((min(lo, hi), max(lo, hi)))
            except ValueError:
                pass
        else:
            try:
                v = float(token)
                bands.append((v, v))
            except ValueError:
                pass
    return bands


def apply_band_removal(spec, manual_bands_str='', use_auto=False,
                       high_power_pct=95, low_cv_threshold=0.05):
    freqs  = spec_freqs(spec)
    power  = spec_power(spec)
    remove = np.zeros(len(freqs), dtype=bool)
    for lo, hi in parse_manual_bands(manual_bands_str):
        remove |= (freqs >= lo) & (freqs <= hi)
    if use_auto:
        noise_mask, _, _ = detect_noise_bands(spec, high_power_pct, low_cv_threshold)
        remove |= noise_mask
    keep = ~remove
    if keep.all():
        return spec
    return rebuild_spec(freqs[keep], spec_times(spec), power[keep, :])


def apply_normalisation(spec, method):
    power = spec_power(spec).copy()
    eps   = np.float32(1e-7)
    if method == 'Subtract per-freq median':
        power -= np.nanmedian(power, axis=1, keepdims=True)
    elif method == 'Z-score per time column':
        mu, sigma = np.nanmean(power, axis=0), np.nanstd(power, axis=0)
        power = (power - mu) / (sigma + eps)
    elif method == 'Z-score per freq row':
        mu    = np.nanmean(power, axis=1, keepdims=True)
        sigma = np.nanstd(power,  axis=1, keepdims=True)
        power = (power - mu) / (sigma + eps)
    elif method == 'Global Z-score':
        power = (power - np.nanmean(power)) / (np.nanstd(power) + eps)
    elif method == 'Min-Max (0–1) per column':
        mn, mx = np.nanmin(power, axis=0), np.nanmax(power, axis=0)
        power  = (power - mn) / (mx - mn + eps)
    return rebuild_spec(spec_freqs(spec), spec_times(spec), power)


def apply_percentile_filter(spec, percentile):
    power = spec_power(spec).copy()
    threshold = np.nanpercentile(power, percentile)
    power[power < threshold] = np.nan
    return rebuild_spec(spec_freqs(spec), spec_times(spec), power)


def znorm_spec(spec):
    power = spec_power(spec)
    mu    = np.nanmean(power)
    sigma = np.nanstd(power)
    return rebuild_spec(spec_freqs(spec), spec_times(spec),
                        ((power - mu) / (sigma + np.float32(1e-7))).astype(power.dtype))


def combine_spectrograms(specs, method='Mean', znorm=False):
    if znorm:
        specs = [znorm_spec(s) for s in specs]
    min_t = min(spec_power(s).shape[1] for s in specs)
    dt    = spec_power(specs[0]).dtype

    if method == 'Median':
        # Median requires the full stack; unavoidable peak memory
        stack = np.stack([spec_power(s)[:, :min_t] for s in specs], axis=0)
        power = np.nanmedian(stack, axis=0).astype(dt)
        del stack
    else:
        # Accumulate channel by channel — avoids holding all channels at once
        acc   = spec_power(specs[0])[:, :min_t].astype(dt)
        count = (~np.isnan(acc)).astype(np.int16)
        nan_mask = np.isnan(acc)
        acc[nan_mask] = 0

        for s in specs[1:]:
            p = spec_power(s)[:, :min_t].astype(dt)
            valid = ~np.isnan(p)
            if method == 'Max':
                better = valid & (p > acc)
                acc[better] = p[better]
                # for cells where acc was NaN, adopt p's value
                acc[nan_mask & valid] = p[nan_mask & valid]
            else:  # Mean or Sum
                p_safe = np.where(valid, p, np.float32(0))
                acc += p_safe
                count += valid.astype(np.int16)
            nan_mask &= ~valid  # cell is only NaN if ALL channels are NaN

        if method == 'Mean':
            with np.errstate(invalid='ignore'):
                acc = np.where(count > 0, acc / count.astype(dt), np.float32(np.nan))
        acc[nan_mask] = np.nan
        power = acc

    return rebuild_spec(spec_freqs(specs[0]), spec_times(specs[0])[:min_t], power)


def aggregate_to_timeseries(spec, method):
    power = spec_power(spec)
    if   method == 'Mean':          values = np.nanmean(power, axis=0)
    elif method == 'Sum':           values = np.nansum(power, axis=0)
    elif method == 'Max':           values = np.nanmax(power, axis=0)
    elif method == 'Median':        values = np.nanmedian(power, axis=0)
    elif method == 'Non-NaN count': values = np.sum(~np.isnan(power), axis=0).astype(float)
    else:                           values = np.nanmean(power, axis=0)
    return spec_times(spec), values


def apply_binning(timestamps, values, bin_size_min, bin_step_min, func='Mean'):
    """Bin a time series using a sliding window.

    bin_size_min  : width of each bin in minutes
    bin_step_min  : step between bin centres in minutes
    func          : 'Min' | 'Max' | 'Mean'
    Returns (bin_timestamps, bin_values) where bin_timestamps are bin-centre Unix seconds.
    """
    if len(timestamps) < 2:
        return timestamps, values

    bin_size_s = bin_size_min * 60.0
    bin_step_s = bin_step_min * 60.0
    if bin_step_s <= 0 or bin_size_s <= 0:
        return timestamps, values

    t_start = timestamps[0]
    t_end   = timestamps[-1]

    centres = np.arange(t_start + bin_size_s / 2.0,
                        t_end   - bin_size_s / 2.0 + bin_step_s,
                        bin_step_s)

    bin_vals = []
    for c in centres:
        mask = (timestamps >= c - bin_size_s / 2.0) & (timestamps < c + bin_size_s / 2.0)
        v = values[mask]
        finite = v[np.isfinite(v)]
        if len(finite) == 0:
            bin_vals.append(np.nan)
        elif func == 'Min':
            bin_vals.append(np.min(finite))
        elif func == 'Max':
            bin_vals.append(np.max(finite))
        else:  # Mean
            bin_vals.append(np.mean(finite))

    return centres, np.array(bin_vals)


def apply_smoothing(timestamps, values, window, method='Rolling mean',
                    step=1, sg_polyorder=3, loess_frac=0.1):
    """Smooth a time series.

    method        : 'Rolling mean' | 'Savitzky-Golay' | 'LOESS'
    window        : window size in *points* (must be odd for SG; forced odd internally)
    step          : output every Nth point (down-sample after smoothing)
    sg_polyorder  : polynomial order for Savitzky-Golay
    loess_frac    : fraction of data used for each LOESS fit
    """
    n = len(values)
    if n < 2 or window < 2:
        return timestamps[::max(1, step)], values[::max(1, step)]

    if method == 'Rolling mean':
        smoothed = pd.Series(values).rolling(window, center=True, min_periods=1).mean().values

    elif method == 'Savitzky-Golay':
        from scipy.signal import savgol_filter
        w = window if window % 2 == 1 else window + 1   # must be odd
        poly = min(sg_polyorder, w - 1)
        finite = np.isfinite(values)
        smoothed = values.copy().astype(float)
        if finite.sum() > w:
            tmp = values.copy().astype(float)
            # fill NaNs with linear interpolation before SG
            tmp_series = pd.Series(tmp)
            tmp_filled = tmp_series.interpolate(limit_direction='both').values
            smoothed = savgol_filter(tmp_filled, w, poly)
            smoothed[~finite] = np.nan

    elif method == 'LOESS':
        try:
            from statsmodels.nonparametric.smoothers_lowess import lowess
            finite = np.isfinite(values)
            if finite.sum() > 4:
                t_norm = timestamps.astype(float)
                result = lowess(values[finite], t_norm[finite],
                                frac=loess_frac, return_sorted=True)
                # interpolate back onto original timestamps
                smoothed = np.interp(t_norm, result[:, 0], result[:, 1])
                smoothed[~finite] = np.nan
            else:
                smoothed = values.copy().astype(float)
        except ImportError:
            # Fall back to rolling mean if statsmodels not installed
            smoothed = pd.Series(values).rolling(window, center=True, min_periods=1).mean().values
    else:
        smoothed = pd.Series(values).rolling(window, center=True, min_periods=1).mean().values

    step = max(1, step)
    return timestamps[::step], smoothed[::step]


def apply_detrending(timestamps, values, method, window=48):
    v = values.copy()
    if method == 'Subtract rolling mean':
        trend = pd.Series(v).rolling(window, center=True, min_periods=1).mean().values
        return timestamps, v - trend
    elif method == 'Subtract daily mean':
        dts  = pd.to_datetime(timestamps, unit='s', utc=True)
        days = dts.normalize()
        for day in days.unique():
            mask = (days == day).values
            v[mask] -= np.nanmean(v[mask])
        return timestamps, v
    elif method == 'Linear detrend':
        x      = np.arange(len(v))
        finite = np.isfinite(v)
        if finite.sum() > 1:
            coeffs = np.polyfit(x[finite], v[finite], 1)
            v -= np.polyval(coeffs, x)
    return timestamps, v


def build_actogram(timestamps, values, period_hours=24.0):
    """Build a double-plotted actogram grid.

    Returns (grid, day_labels, period_hours) where:
      grid        — (n_days, 2*n_bins) float array, double-plotted (day repeated in cols)
      day_labels  — list of date strings for the y-axis
      period_hours — the period used (passed through for the x-axis)
    """
    dts      = pd.to_datetime(timestamps, unit='s', utc=True)
    day_zero = dts.normalize().min()

    # number of time steps per period
    diffs   = np.diff(timestamps)
    dt_step = float(np.median(diffs[diffs > 0])) if len(diffs) > 0 else 3600.0
    n_bins  = max(1, round(period_hours * 3600.0 / dt_step))

    # which period-length window each sample falls in (0-based day index)
    elapsed_h = (timestamps - float(timestamps[0])) / 3600.0
    day_idx   = np.floor(elapsed_h / period_hours).astype(int)
    n_days    = int(day_idx.max()) + 1 if len(day_idx) else 1

    # bin within the period
    frac_in_period = (elapsed_h % period_hours) / period_hours
    bin_idx = np.floor(frac_in_period * n_bins).astype(int).clip(0, n_bins - 1)

    counts   = np.zeros((n_days, n_bins))
    grid_sum = np.zeros((n_days, n_bins))
    finite_mask = np.isfinite(values)
    np.add.at(grid_sum, (day_idx[finite_mask], bin_idx[finite_mask]), values[finite_mask])
    np.add.at(counts,   (day_idx[finite_mask], bin_idx[finite_mask]), 1)

    with np.errstate(invalid='ignore'):
        single = np.where(counts > 0, grid_sum / counts, np.nan)

    # Double-plot: repeat each row side by side
    grid = np.concatenate([single, single], axis=1)

    # Day labels from the start timestamp of each row
    day_starts = [
        pd.to_datetime(float(timestamps[0]) + r * period_hours * 3600, unit='s', utc=True)
        for r in range(n_days)
    ]
    day_labels = [d.strftime('%d %b') for d in day_starts]

    return grid, day_labels, period_hours


def compute_lomb_scargle(timestamps, values, n_freqs=3000):
    """Compute a Lomb-Scargle periodogram over the 15–30 h period range.

    Returns (periods_hours, power, peak_period_hours, peak_power).
    Uses a slightly wider internal range (10–36 h) to avoid edge artefacts,
    but only the 15–30 h slice is returned.
    """
    finite = np.isfinite(values)
    if finite.sum() < 4:
        return None, None, None, None

    t = (timestamps[finite].astype(float) - timestamps[finite][0]) / 3600.0  # hours
    y = values[finite].astype(float)
    y -= np.mean(y)

    # Angular frequencies: slightly wider than display range
    freq_lo  = 1.0 / 36.0
    freq_hi  = 1.0 / 10.0
    freqs    = np.linspace(freq_lo, freq_hi, n_freqs)   # cycles / hour
    ang_freq = 2.0 * np.pi * freqs

    power   = lombscargle(t, y, ang_freq, normalize=True)
    periods = 1.0 / freqs

    # Clip to display range 15–30 h
    mask    = (periods >= 15.0) & (periods <= 30.0)
    periods = periods[mask]
    power   = power[mask]

    if len(power) == 0:
        return None, None, None, None

    peak_idx = int(np.argmax(power))
    return periods, power, periods[peak_idx], float(power[peak_idx])


def compute_band_stats(spec, bands):
    """Return summary statistics for each (lo_hz, hi_hz) frequency band.

    bands  : list of (lo, hi) tuples in Hz
    Returns: list of dicts — one per band — with keys:
             band, n_freqs, min, mean, median, max, sd
    """
    freqs  = spec_freqs(spec)
    power  = spec_power(spec)
    results = []
    for lo, hi in bands:
        mask = (freqs >= lo) & (freqs <= hi)
        label = f"{lo:.0f}–{hi:.0f} Hz"
        n_freqs = int(mask.sum())
        if n_freqs == 0:
            results.append(dict(band=label, n_freqs=0,
                                min=np.nan, mean=np.nan, median=np.nan,
                                max=np.nan, sd=np.nan))
            continue
        vals = power[mask, :].ravel()
        finite = vals[np.isfinite(vals)]
        if len(finite) == 0:
            results.append(dict(band=label, n_freqs=n_freqs,
                                min=np.nan, mean=np.nan, median=np.nan,
                                max=np.nan, sd=np.nan))
        else:
            results.append(dict(
                band=label, n_freqs=n_freqs,
                min=float(np.min(finite)),
                mean=float(np.mean(finite)),
                median=float(np.median(finite)),
                max=float(np.max(finite)),
                sd=float(np.std(finite)),
            ))
    return results


# ══════════════════════════════════════════════════════════════════════════════
# Noise preview dialog
# ══════════════════════════════════════════════════════════════════════════════

class NoiseBandPreviewDialog(QDialog):
    def __init__(self, spec, high_power_pct, low_cv_threshold, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Noise Band Diagnostics")
        self.resize(700, 500)
        layout = QVBoxLayout(self)

        fig = Figure(facecolor='#1e1e2e')
        canvas = FigureCanvas(fig)
        layout.addWidget(canvas)

        freqs     = spec_freqs(spec)
        power     = spec_power(spec).astype(float)
        row_means = np.nanmean(power, axis=1)
        row_stds  = np.nanstd(power, axis=1)
        cv        = row_stds / (np.abs(row_means) + 1e-12)
        noise_mask, _, _ = detect_noise_bands(spec, high_power_pct, low_cv_threshold)

        ax1 = fig.add_subplot(2, 1, 1)
        ax1.set_facecolor('#2a2a3e')
        ax1.plot(freqs, row_means, color='#88aaff', lw=0.8)
        thresh_power = np.nanpercentile(row_means, high_power_pct)
        ax1.axhline(thresh_power, color='#ff6666', lw=1, linestyle='--',
                    label=f'{high_power_pct}th pct threshold')
        ax1.fill_between(freqs, row_means.min(), row_means.max(),
                         where=noise_mask, color='#ff4444', alpha=0.25, label='detected noise')
        ax1.set_ylabel("Mean power (dB)", color='#cccccc', fontsize=8)
        ax1.set_title("Per-frequency mean power  (red = detected as noise)", color='#e0e0f0', fontsize=9)
        ax1.legend(fontsize=7, facecolor='#2a2a3e', labelcolor='#cccccc')
        for sp in ax1.spines.values(): sp.set_edgecolor('#555577')
        ax1.tick_params(colors='#cccccc', labelsize=7)

        ax2 = fig.add_subplot(2, 1, 2)
        ax2.set_facecolor('#2a2a3e')
        ax2.plot(freqs, cv, color='#aaffaa', lw=0.8)
        ax2.axhline(low_cv_threshold, color='#ffaa44', lw=1, linestyle='--',
                    label=f'CV threshold = {low_cv_threshold:.3f}')
        ax2.fill_between(freqs, 0, cv.max(),
                         where=noise_mask, color='#ff4444', alpha=0.25, label='detected noise')
        ax2.set_xlabel("Frequency (Hz)", color='#cccccc', fontsize=8)
        ax2.set_ylabel("Coefficient of variation", color='#cccccc', fontsize=8)
        ax2.set_title("Per-frequency temporal variation (CV = std/|mean|)", color='#e0e0f0', fontsize=9)
        ax2.legend(fontsize=7, facecolor='#2a2a3e', labelcolor='#cccccc')
        for sp in ax2.spines.values(): sp.set_edgecolor('#555577')
        ax2.tick_params(colors='#cccccc', labelsize=7)

        n_noise  = noise_mask.sum()
        noise_hz = freqs[noise_mask]
        ranges   = []
        if len(noise_hz):
            gaps   = np.where(np.diff(noise_hz) > (freqs[1] - freqs[0]) * 1.5)[0]
            starts = np.concatenate([[0], gaps + 1])
            ends   = np.concatenate([gaps, [len(noise_hz) - 1]])
            for s, e in zip(starts, ends):
                lo, hi = noise_hz[s], noise_hz[e]
                ranges.append(f"{lo:.0f}–{hi:.0f} Hz" if lo != hi else f"{lo:.0f} Hz")

        summary = QLabel(
            f"Detected {n_noise} noisy frequency row(s).  "
            + ("Bands: " + ", ".join(ranges) if ranges else "None."))
        summary.setWordWrap(True)
        summary.setStyleSheet("color: #cccccc; font-size: 10px;")
        layout.addWidget(summary)

        fig.tight_layout(pad=1.5)
        canvas.draw()


# ══════════════════════════════════════════════════════════════════════════════
# Frequency band statistics dialog
# ══════════════════════════════════════════════════════════════════════════════

class FreqBandStatsDialog(QDialog):
    _COLS = ['Band', 'N freq bins', 'Min', 'Mean', 'Median', 'Max', 'SD']

    def __init__(self, rows, parent=None):
        """rows: list of dicts from compute_band_stats()."""
        super().__init__(parent)
        self.setWindowTitle("Frequency Band Statistics")
        self.resize(640, max(200, 80 + len(rows) * 30))
        layout = QVBoxLayout(self)

        tbl = QTableWidget(len(rows), len(self._COLS), self)
        tbl.setHorizontalHeaderLabels(self._COLS)
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setAlternatingRowColors(True)

        for r, row in enumerate(rows):
            vals = [
                row['band'],
                str(row['n_freqs']),
                f"{row['min']:.4f}"    if np.isfinite(row['min'])    else '–',
                f"{row['mean']:.4f}"   if np.isfinite(row['mean'])   else '–',
                f"{row['median']:.4f}" if np.isfinite(row['median']) else '–',
                f"{row['max']:.4f}"    if np.isfinite(row['max'])    else '–',
                f"{row['sd']:.4f}"     if np.isfinite(row['sd'])     else '–',
            ]
            for c, v in enumerate(vals):
                item = QTableWidgetItem(v)
                item.setTextAlignment(Qt.AlignCenter)
                tbl.setItem(r, c, item)

        layout.addWidget(tbl)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn)


# ══════════════════════════════════════════════════════════════════════════════
# Main application window
# ══════════════════════════════════════════════════════════════════════════════

class CircadianPipelineApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Circadian Rhythm Pipeline  v3.0")
        self.resize(1450, 920)

        self.spectrograms          = []
        self._last_ts              = None   # (timestamps, values) after full TS pipeline
        self._last_processed_spec  = None   # combined+post-pipeline spec, pre-TS-agg

        # Ordered list of spectrogram step IDs (s0=combine, s1-s4=spec transforms).
        # Steps before s0 in this list run per-channel; steps after run on the
        # combined spectrogram.
        self._spec_step_order = ['s0', 's1', 's2', 's3', 's4']
        self._spec_groups = {}   # step_id → QGroupBox (reorderable)
        self._ts_groups   = {}   # step_id → QGroupBox (fixed time-series steps)

        self._refresh_timer = QTimer()
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.timeout.connect(self._replot)

        self._build_ui()
        self._update_smooth_ui()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        splitter = QSplitter(Qt.Horizontal)

        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setFixedWidth(330)

        ctrl_widget = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_widget)
        ctrl_layout.setAlignment(Qt.AlignTop)
        ctrl_layout.setSpacing(5)

        # ── Load buttons ──────────────────────────────────────────────────────
        load_group = QGroupBox("Load Data")
        ll = QVBoxLayout(load_group)
        self.load_std_btn = QPushButton("Load Spectrogram_x.csv folder")
        self.load_std_btn.clicked.connect(self.load_standard)
        self.load_chunked_btn = QPushButton("Load chunked CSV folder")
        self.load_chunked_btn.clicked.connect(self.load_chunked)
        self.load_label = QLabel("No data loaded")
        self.load_label.setWordWrap(True)
        self.load_label.setStyleSheet("color: grey; font-size: 10px;")
        ll.addWidget(self.load_std_btn)
        ll.addWidget(self.load_chunked_btn)
        ll.addWidget(self.load_label)
        ctrl_layout.addWidget(load_group)

        # ── Channel selection (always at top, non-reorderable) ────────────────
        ch_group = QGroupBox("Channel Selection")
        ch_outer = QVBoxLayout(ch_group)
        ch_outer.setSpacing(4)

        sel_row = QHBoxLayout()
        self.sel_all_btn  = QPushButton("All");  self.sel_all_btn.setFixedWidth(38)
        self.sel_all_btn.clicked.connect(self._select_all_channels)
        self.sel_none_btn = QPushButton("None"); self.sel_none_btn.setFixedWidth(42)
        self.sel_none_btn.clicked.connect(self._clear_channels)
        self.show_raw_cb = QCheckBox("Show raw")
        self.show_raw_cb.setChecked(True)
        self.show_raw_cb.stateChanged.connect(self._schedule_replot)
        sel_row.addWidget(QLabel("Select:"))
        sel_row.addWidget(self.sel_all_btn)
        sel_row.addWidget(self.sel_none_btn)
        sel_row.addStretch()
        sel_row.addWidget(self.show_raw_cb)
        ch_outer.addLayout(sel_row)

        self.ch_cb_widget = QWidget()
        self.ch_cb_layout = QVBoxLayout(self.ch_cb_widget)
        self.ch_cb_layout.setContentsMargins(0, 0, 0, 0)
        self.ch_cb_layout.setSpacing(2)
        ch_outer.addWidget(self.ch_cb_widget)
        self.ch_checkboxes = []
        ctrl_layout.addWidget(ch_group)

        # ── Reorderable spectrogram steps (s0–s4) ─────────────────────────────
        reorder_label = QLabel("Spectrogram steps  (↑/↓ to reorder, steps before Combine run per-channel):")
        reorder_label.setWordWrap(True)
        reorder_label.setStyleSheet("color: #aaaacc; font-size: 10px;")
        ctrl_layout.addWidget(reorder_label)

        self._steps_container = QWidget()
        self._steps_layout    = QVBoxLayout(self._steps_container)
        self._steps_layout.setContentsMargins(0, 0, 0, 0)
        self._steps_layout.setSpacing(3)
        ctrl_layout.addWidget(self._steps_container)

        # s0: Combine channels ─────────────────────────────────────────────────
        s0 = self._make_spec_group('s0', "Combine Channels", enabled=True)
        comb_row = QHBoxLayout()
        comb_row.addWidget(QLabel("Method:"))
        self.combine_combo = QComboBox()
        self.combine_combo.addItems(['Mean', 'Sum', 'Max', 'Median'])
        self.combine_combo.currentIndexChanged.connect(lambda *_, g=s0: self._on_step_param_changed(g))
        comb_row.addWidget(self.combine_combo)
        s0.layout().addLayout(comb_row)
        self.znorm_before_combine_cb = QCheckBox("Z-norm each channel before combining")
        self.znorm_before_combine_cb.setChecked(True)
        self.znorm_before_combine_cb.stateChanged.connect(lambda *_, g=s0: self._on_step_param_changed(g))
        s0.layout().addWidget(self.znorm_before_combine_cb)
        self._spec_groups['s0'] = s0

        # s1: Frequency filter ─────────────────────────────────────────────────
        s1 = self._make_spec_group('s1', "Frequency Filter", enabled=True)
        fg = QGridLayout()
        fg.addWidget(QLabel("Min Hz:"), 0, 0)
        self.freq_min = QSpinBox(); self.freq_min.setRange(0, 20000); self.freq_min.setValue(0)
        self.freq_min.valueChanged.connect(lambda *_, g=s1: self._on_step_param_changed(g))
        fg.addWidget(self.freq_min, 0, 1)
        fg.addWidget(QLabel("Max Hz:"), 1, 0)
        self.freq_max = QSpinBox(); self.freq_max.setRange(0, 20000); self.freq_max.setValue(500)
        self.freq_max.valueChanged.connect(lambda *_, g=s1: self._on_step_param_changed(g))
        fg.addWidget(self.freq_max, 1, 1)
        s1.layout().addLayout(fg)
        self._spec_groups['s1'] = s1

        # s2: Noise band removal ───────────────────────────────────────────────
        s2 = self._make_spec_group('s2', "Noise Band Removal", enabled=False)
        ng = QGridLayout()
        ng.addWidget(QLabel("Manual bands\n(e.g. 48-52 98-102):"), 0, 0, 1, 2)
        self.noise_manual = QLineEdit()
        self.noise_manual.setPlaceholderText("e.g. 48-52 100")
        self.noise_manual.textChanged.connect(lambda *_, g=s2: self._on_step_param_changed(g))
        ng.addWidget(self.noise_manual, 1, 0, 1, 2)
        self.noise_auto_cb = QCheckBox("Auto-detect noise bands")
        self.noise_auto_cb.stateChanged.connect(lambda *_, g=s2: self._on_step_param_changed(g))
        ng.addWidget(self.noise_auto_cb, 2, 0, 1, 2)
        ng.addWidget(QLabel("High-power pct:"), 3, 0)
        self.noise_power_pct = QSpinBox()
        self.noise_power_pct.setRange(50, 100); self.noise_power_pct.setValue(95)
        self.noise_power_pct.valueChanged.connect(lambda *_, g=s2: self._on_step_param_changed(g))
        ng.addWidget(self.noise_power_pct, 3, 1)
        ng.addWidget(QLabel("Low-CV threshold:"), 4, 0)
        self.noise_cv = QDoubleSpinBox()
        self.noise_cv.setRange(0.001, 1.0); self.noise_cv.setValue(0.05)
        self.noise_cv.setSingleStep(0.005); self.noise_cv.setDecimals(3)
        self.noise_cv.valueChanged.connect(lambda *_, g=s2: self._on_step_param_changed(g))
        ng.addWidget(self.noise_cv, 4, 1)
        self.noise_preview_btn = QPushButton("Preview detected bands…")
        self.noise_preview_btn.clicked.connect(self._show_noise_preview)
        ng.addWidget(self.noise_preview_btn, 5, 0, 1, 2)
        s2.layout().addLayout(ng)
        self._spec_groups['s2'] = s2

        # s3: Normalisation ────────────────────────────────────────────────────
        s3 = self._make_spec_group('s3', "Normalisation", enabled=False)
        nl = QGridLayout()
        nl.addWidget(QLabel("Method:"), 0, 0)
        self.norm_combo = QComboBox()
        self.norm_combo.addItems([
            'Subtract per-freq median',
            'Z-score per time column',
            'Z-score per freq row',
            'Global Z-score',
            'Min-Max (0–1) per column',
        ])
        self.norm_combo.currentIndexChanged.connect(lambda *_, g=s3: self._on_step_param_changed(g))
        nl.addWidget(self.norm_combo, 0, 1)
        s3.layout().addLayout(nl)
        self._spec_groups['s3'] = s3

        # s4: Percentile filter ────────────────────────────────────────────────
        s4 = self._make_spec_group('s4', "Percentile Filter", enabled=True, showing=True)
        pl = QGridLayout()
        pl.addWidget(QLabel("Keep cells ≥ Nth pct:"), 0, 0)
        self.pct_spin = QSpinBox(); self.pct_spin.setRange(0, 99); self.pct_spin.setValue(50)
        self.pct_spin.valueChanged.connect(lambda *_, g=s4: self._on_step_param_changed(g))
        pl.addWidget(self.pct_spin, 0, 1)
        s4.layout().addLayout(pl)
        self._spec_groups['s4'] = s4

        # Populate container in default order
        for sid in self._spec_step_order:
            self._steps_layout.addWidget(self._spec_groups[sid])
        self._update_step_buttons()

        # ── Time-series steps (fixed order, Show checkbox only) ───────────────
        ts_label = QLabel("Time-series steps  (fixed order):")
        ts_label.setStyleSheet("color: #aaaacc; font-size: 10px;")
        ctrl_layout.addWidget(ts_label)

        # s5: Time aggregation
        s5 = self._make_ts_group('s5', "Time Series Aggregation", enabled=True)
        al = QGridLayout()
        al.addWidget(QLabel("Aggregate freq by:"), 0, 0)
        self.agg_combo = QComboBox()
        self.agg_combo.addItems(['Mean', 'Median', 'Max', 'Sum', 'Non-NaN count'])
        self.agg_combo.currentIndexChanged.connect(lambda *_, g=s5: self._on_step_param_changed(g))
        al.addWidget(self.agg_combo, 0, 1)
        s5.layout().addLayout(al)
        self._ts_groups['s5'] = s5
        ctrl_layout.addWidget(s5)

        # s5b: Binning
        s5b = self._make_ts_group('s5b', "Binning", enabled=False)
        bl = QGridLayout()
        bl.addWidget(QLabel("Bin size (min):"), 0, 0)
        self.bin_size_spin = QDoubleSpinBox()
        self.bin_size_spin.setRange(0.1, 1440.0); self.bin_size_spin.setValue(5.0)
        self.bin_size_spin.setSingleStep(1.0); self.bin_size_spin.setDecimals(1)
        self.bin_size_spin.valueChanged.connect(lambda *_, g=s5b: self._on_step_param_changed(g))
        bl.addWidget(self.bin_size_spin, 0, 1)
        bl.addWidget(QLabel("Bin step (min):"), 1, 0)
        self.bin_step_spin = QDoubleSpinBox()
        self.bin_step_spin.setRange(0.1, 1440.0); self.bin_step_spin.setValue(5.0)
        self.bin_step_spin.setSingleStep(1.0); self.bin_step_spin.setDecimals(1)
        self.bin_step_spin.valueChanged.connect(lambda *_, g=s5b: self._on_step_param_changed(g))
        bl.addWidget(self.bin_step_spin, 1, 1)
        bl.addWidget(QLabel("Function:"), 2, 0)
        self.bin_func_combo = QComboBox()
        self.bin_func_combo.addItems(['Mean', 'Min', 'Max'])
        self.bin_func_combo.currentIndexChanged.connect(lambda *_, g=s5b: self._on_step_param_changed(g))
        bl.addWidget(self.bin_func_combo, 2, 1)
        s5b.layout().addLayout(bl)
        self._ts_groups['s5b'] = s5b
        ctrl_layout.addWidget(s5b)

        # s6: Smoothing
        s6 = self._make_ts_group('s6', "Smoothing", enabled=True, showing=True)
        sl = QGridLayout()
        sl.addWidget(QLabel("Method:"), 0, 0)
        self.smooth_method = QComboBox()
        self.smooth_method.addItems(['Rolling mean', 'Savitzky-Golay', 'LOESS'])
        self.smooth_method.currentIndexChanged.connect(lambda *_, g=s6: self._on_step_param_changed(g))
        self.smooth_method.currentIndexChanged.connect(self._update_smooth_ui)
        sl.addWidget(self.smooth_method, 0, 1)
        sl.addWidget(QLabel("Window (points):"), 1, 0)
        self.smooth_spin = QSpinBox(); self.smooth_spin.setRange(2, 5000); self.smooth_spin.setValue(2500)
        self.smooth_spin.valueChanged.connect(lambda *_, g=s6: self._on_step_param_changed(g))
        sl.addWidget(self.smooth_spin, 1, 1)
        sl.addWidget(QLabel("Output step (points):"), 2, 0)
        self.smooth_step = QSpinBox(); self.smooth_step.setRange(1, 5000); self.smooth_step.setValue(500)
        self.smooth_step.valueChanged.connect(lambda *_, g=s6: self._on_step_param_changed(g))
        sl.addWidget(self.smooth_step, 2, 1)
        # SG-specific
        self.smooth_sg_label = QLabel("SG poly order:")
        sl.addWidget(self.smooth_sg_label, 3, 0)
        self.smooth_sg_poly = QSpinBox(); self.smooth_sg_poly.setRange(1, 10); self.smooth_sg_poly.setValue(3)
        self.smooth_sg_poly.valueChanged.connect(lambda *_, g=s6: self._on_step_param_changed(g))
        sl.addWidget(self.smooth_sg_poly, 3, 1)
        # LOESS-specific
        self.smooth_loess_label = QLabel("LOESS frac (0–1):")
        sl.addWidget(self.smooth_loess_label, 4, 0)
        self.smooth_loess_frac = QDoubleSpinBox()
        self.smooth_loess_frac.setRange(0.01, 1.0); self.smooth_loess_frac.setValue(0.1)
        self.smooth_loess_frac.setSingleStep(0.05); self.smooth_loess_frac.setDecimals(2)
        self.smooth_loess_frac.valueChanged.connect(lambda *_, g=s6: self._on_step_param_changed(g))
        sl.addWidget(self.smooth_loess_frac, 4, 1)
        s6.layout().addLayout(sl)
        self._ts_groups['s6'] = s6
        ctrl_layout.addWidget(s6)

        # s7: Detrending
        s7 = self._make_ts_group('s7', "Detrending", enabled=False)
        dl = QGridLayout()
        dl.addWidget(QLabel("Method:"), 0, 0)
        self.detrend_combo = QComboBox()
        self.detrend_combo.addItems([
            'Subtract rolling mean', 'Subtract daily mean', 'Linear detrend'])
        self.detrend_combo.currentIndexChanged.connect(lambda *_, g=s7: self._on_step_param_changed(g))
        dl.addWidget(self.detrend_combo, 0, 1)
        dl.addWidget(QLabel("Rolling window (pts):"), 1, 0)
        self.detrend_window = QSpinBox()
        self.detrend_window.setRange(2, 5000); self.detrend_window.setValue(48)
        self.detrend_window.valueChanged.connect(lambda *_, g=s7: self._on_step_param_changed(g))
        dl.addWidget(self.detrend_window, 1, 1)
        s7.layout().addLayout(dl)
        self._ts_groups['s7'] = s7
        ctrl_layout.addWidget(s7)

        # s8: Actogram
        s8 = self._make_ts_group('s8', "Actogram", enabled=False)
        cl = QGridLayout()
        cl.addWidget(QLabel("Period (hours):"), 0, 0)
        self.circ_period = QDoubleSpinBox()
        self.circ_period.setRange(1.0, 48.0); self.circ_period.setValue(24.0)
        self.circ_period.setSingleStep(0.5)
        self.circ_period.valueChanged.connect(lambda *_, g=s8: self._on_step_param_changed(g))
        cl.addWidget(self.circ_period, 0, 1)
        s8.layout().addLayout(cl)
        self._ts_groups['s8'] = s8
        ctrl_layout.addWidget(s8)

        # ── Plot settings ─────────────────────────────────────────────────────
        cm_group = QGroupBox("Plot settings")
        cm_layout = QGridLayout(cm_group)
        cm_layout.addWidget(QLabel("Colormap:"), 0, 0)
        self.cmap_combo = QComboBox()
        self.cmap_combo.addItems(['viridis', 'plasma', 'inferno', 'magma', 'cividis',
                                  'RdYlBu_r', 'coolwarm', 'seismic', 'hot'])
        self.cmap_combo.currentIndexChanged.connect(self._schedule_replot)
        cm_layout.addWidget(self.cmap_combo, 0, 1)
        cm_layout.addWidget(QLabel("Clip pct (each end):"), 1, 0)
        self.clip_spin = QSpinBox(); self.clip_spin.setRange(0, 49); self.clip_spin.setValue(2)
        self.clip_spin.valueChanged.connect(self._schedule_replot)
        cm_layout.addWidget(self.clip_spin, 1, 1)
        ctrl_layout.addWidget(cm_group)

        # s9: Lomb-Scargle periodogram
        s9 = self._make_ts_group('s9', "Lomb-Scargle Periodogram", enabled=True, showing=True )
        pl9 = QGridLayout()
        pl9.addWidget(QLabel("Period range (h): 12 – 30"), 0, 0, 1, 2)
        pl9.addWidget(QLabel("(uses freq bands if set above)"), 1, 0, 1, 2)
        s9.layout().addLayout(pl9)
        self._ts_groups['s9'] = s9
        ctrl_layout.addWidget(s9)

        # ── Export time series ────────────────────────────────────────────────
        export_group = QGroupBox("Export")
        eg = QVBoxLayout(export_group)
        self.export_btn = QPushButton("Export time series as CSV…")
        self.export_btn.clicked.connect(self._export_timeseries)
        eg.addWidget(self.export_btn)
        ctrl_layout.addWidget(export_group)

        # ── Frequency band statistics ─────────────────────────────────────────
        stats_group = QGroupBox("Frequency Band Statistics")
        sg = QVBoxLayout(stats_group)
        sg.addWidget(QLabel("Bands (e.g. 200-400 800-1200):"))
        self.stats_bands_edit = QLineEdit()
        self.stats_bands_edit.setPlaceholderText("e.g. 0-200 200-800 800-2500")
        self.stats_bands_edit.setText("0-40 40-80 80-120 120-160 160-200")
        sg.addWidget(self.stats_bands_edit)
        self.stats_btn = QPushButton("Compute band statistics…")
        self.stats_btn.clicked.connect(self._show_band_stats)
        sg.addWidget(self.stats_btn)
        ctrl_layout.addWidget(stats_group)

        ctrl_scroll.setWidget(ctrl_widget)

        # ── Right matplotlib panel ────────────────────────────────────────────
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.plot_scroll = QScrollArea()
        self.plot_scroll.setWidgetResizable(True)
        self.fig    = Figure(facecolor='#1e1e2e')
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.plot_scroll.setWidget(self.canvas)
        right_layout.addWidget(self.plot_scroll)

        splitter.addWidget(ctrl_scroll)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter)

    # ── Group factory helpers ─────────────────────────────────────────────────

    def _make_spec_group(self, step_id, title, enabled=True, showing=False):
        """Create a reorderable spectrogram-step group with ↑/↓, Use and Show checkboxes."""
        group = QGroupBox()
        outer = QVBoxLayout(group)
        outer.setContentsMargins(4, 3, 4, 3)
        outer.setSpacing(2)

        hdr = QHBoxLayout()
        hdr.setSpacing(2)

        up_btn = QPushButton("↑"); up_btn.setFixedSize(20, 18)
        dn_btn = QPushButton("↓"); dn_btn.setFixedSize(20, 18)
        up_btn.clicked.connect(lambda: self._move_step(step_id, -1))
        dn_btn.clicked.connect(lambda: self._move_step(step_id, +1))

        lbl = QLabel(title)
        lbl.setStyleSheet("font-weight: bold;")

        use_cb = QCheckBox("Use")
        use_cb.setChecked(enabled)
        use_cb.stateChanged.connect(self._schedule_replot)  # always fires

        show_cb = QCheckBox("Show")
        show_cb.setChecked(showing)
        show_cb.stateChanged.connect(lambda *_, g=group: self._on_step_param_changed(g))

        hdr.addWidget(up_btn)
        hdr.addWidget(dn_btn)
        hdr.addWidget(lbl, 1)
        hdr.addWidget(use_cb)
        hdr.addWidget(show_cb)
        outer.addLayout(hdr)

        group._step_id      = step_id
        group._checkbox     = use_cb
        group._show_plot_cb = show_cb
        group._up_btn       = up_btn
        group._dn_btn       = dn_btn
        return group

    def _make_ts_group(self, step_id, title, enabled=True, showing=False):
        """Create a time-series-step group (Use and Show checkboxes, no reorder buttons)."""
        group = QGroupBox()
        outer = QVBoxLayout(group)
        outer.setContentsMargins(4, 3, 4, 3)
        outer.setSpacing(2)

        hdr = QHBoxLayout()
        hdr.setSpacing(2)

        lbl = QLabel(title)
        lbl.setStyleSheet("font-weight: bold;")

        use_cb = QCheckBox("Use")
        use_cb.setChecked(enabled)
        use_cb.stateChanged.connect(self._schedule_replot)  # always fires

        show_cb = QCheckBox("Show")
        show_cb.setChecked(showing)
        show_cb.stateChanged.connect(lambda *_, g=group: self._on_step_param_changed(g))

        hdr.addWidget(lbl, 1)
        hdr.addWidget(use_cb)
        hdr.addWidget(show_cb)
        outer.addLayout(hdr)

        group._step_id      = step_id
        group._checkbox     = use_cb
        group._show_plot_cb = show_cb
        return group

    def _update_smooth_ui(self):
        method = self.smooth_method.currentText()
        is_sg    = method == 'Savitzky-Golay'
        is_loess = method == 'LOESS'
        for w in (self.smooth_sg_label, self.smooth_sg_poly):
            w.setVisible(is_sg)
        for w in (self.smooth_loess_label, self.smooth_loess_frac):
            w.setVisible(is_loess)

    def _on_step_param_changed(self, group):
        """Schedule a replot only if the owning step is currently enabled."""
        if self._step_enabled(group):
            self._schedule_replot()

    # ── Step reordering ───────────────────────────────────────────────────────

    def _move_step(self, step_id, direction):
        idx = self._spec_step_order.index(step_id)
        new_idx = idx + direction
        if new_idx < 0 or new_idx >= len(self._spec_step_order):
            return
        order = self._spec_step_order
        order[idx], order[new_idx] = order[new_idx], order[idx]
        self._rebuild_steps_layout()
        self._schedule_replot()

    def _rebuild_steps_layout(self):
        """Remove all spec-step groups from the container, then re-add in current order."""
        while self._steps_layout.count():
            self._steps_layout.takeAt(0)
        for sid in self._spec_step_order:
            self._steps_layout.addWidget(self._spec_groups[sid])
        self._update_step_buttons()

    def _update_step_buttons(self):
        n = len(self._spec_step_order)
        for i, sid in enumerate(self._spec_step_order):
            g = self._spec_groups[sid]
            g._up_btn.setEnabled(i > 0)
            g._dn_btn.setEnabled(i < n - 1)

    def _step_enabled(self, group):
        return group._checkbox.isChecked()

    # ── Data loading ──────────────────────────────────────────────────────────

    def load_standard(self):
        try:
            folder = QFileDialog.getExistingDirectory(
                self, "Select folder with Spectrogram_x.csv files")
            if not folder:
                return
            specs  = load_standard_spectrograms(folder)
            loaded = [(c, s) for c, s in specs if s is not None]
            if not loaded:
                QMessageBox.warning(self, "No files", "No Spectrogram_x.csv files found.")
                return
            self.spectrograms = specs
            self._update_channel_combo()
            self._set_freq_range_from_data()
            self.load_label.setText(f"Loaded {len(loaded)} ch from:\n{os.path.basename(folder)}")
            self._replot()
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Load error", str(e))

    def load_chunked(self):
        try:
            folder = QFileDialog.getExistingDirectory(
                self, "Select folder with chunked CSV files")
            if not folder:
                return
            specs  = load_chunked_spectrograms(folder)
            loaded = [(c, s) for c, s in specs if s is not None]
            if not loaded:
                QMessageBox.warning(self, "No files", "No chunked CSV files found.")
                return
            self.spectrograms = specs
            self._update_channel_combo()
            self._set_freq_range_from_data()
            self.load_label.setText(
                f"Loaded {len(loaded)} ch (chunked) from:\n{os.path.basename(folder)}")
            self._replot()
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Load error", str(e))

    def _update_channel_combo(self):
        for _, cb in self.ch_checkboxes:
            cb.setParent(None)
        self.ch_checkboxes.clear()
        for c, s in self.spectrograms:
            cb = QCheckBox(f"Channel {c}" + (" (no data)" if s is None else ""))
            cb.setChecked(s is not None)
            cb.setEnabled(s is not None)
            cb.stateChanged.connect(self._schedule_replot)
            self.ch_cb_layout.addWidget(cb)
            self.ch_checkboxes.append((c, cb))

    def _select_all_channels(self):
        for _, cb in self.ch_checkboxes:
            cb.blockSignals(True)
            if cb.isEnabled(): cb.setChecked(True)
            cb.blockSignals(False)
        self._schedule_replot()

    def _clear_channels(self):
        for _, cb in self.ch_checkboxes:
            cb.blockSignals(True); cb.setChecked(False); cb.blockSignals(False)
        self._schedule_replot()

    def _selected_specs(self):
        result = []
        for (c, cb), (_, s) in zip(self.ch_checkboxes, self.spectrograms):
            if cb.isChecked() and s is not None:
                result.append((c, s))
        return result

    def _set_freq_range_from_data(self):
        spec = next((s for _, s in self.spectrograms if s is not None), None)
        if spec is None:
            return
        freqs = spec_freqs(spec)
        for w in (self.freq_min, self.freq_max):
            w.blockSignals(True)
        self.freq_min.setValue(int(freqs.min()))
        self.freq_max.setValue(int(freqs.max()))
        self.freq_max.setMaximum(int(freqs.max()) + 100)
        for w in (self.freq_min, self.freq_max):
            w.blockSignals(False)

    # ── Spec-step dispatch ────────────────────────────────────────────────────

    def _apply_spec_step(self, step_id, spec):
        """Apply one enabled spec step to a single spectrogram array."""
        if step_id == 's1':
            return apply_freq_filter(spec, self.freq_min.value(), self.freq_max.value())
        elif step_id == 's2':
            return apply_band_removal(
                spec,
                self.noise_manual.text().strip(),
                self.noise_auto_cb.isChecked(),
                self.noise_power_pct.value(),
                self.noise_cv.value())
        elif step_id == 's3':
            return apply_normalisation(spec, self.norm_combo.currentText())
        elif step_id == 's4':
            return apply_percentile_filter(spec, self.pct_spin.value())
        return spec

    def _step_label(self, step_id):
        if step_id == 's0': return "Combine"
        if step_id == 's1':
            return f"Freq filter {self.freq_min.value()}–{self.freq_max.value()} Hz"
        if step_id == 's2':
            auto = " [auto]" if self.noise_auto_cb.isChecked() else ""
            return f"Noise removal{auto}"
        if step_id == 's3': return f"Normalised ({self.norm_combo.currentText()})"
        if step_id == 's4': return f"Percentile filter (≥{self.pct_spin.value()}th)"
        return step_id

    # ── Noise preview ─────────────────────────────────────────────────────────

    def _show_noise_preview(self):
        try:
            spec = self._compute_pre_noise_spec()
            if spec is None:
                QMessageBox.information(self, "No data", "Load a spectrogram first.")
                return
            dlg = NoiseBandPreviewDialog(
                spec, self.noise_power_pct.value(), self.noise_cv.value(), parent=self)
            dlg.exec_()
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Preview error", str(e))

    def _compute_pre_noise_spec(self):
        """Return the spectrogram as it would look just before s2 (noise removal)."""
        selected = self._selected_specs()
        if not selected:
            return None
        s2_pos    = self._spec_step_order.index('s2')
        pre_ids   = self._spec_step_order[:s2_pos]
        post_ids  = [sid for sid in self._spec_step_order[s2_pos+1:]
                     if sid != 's2']  # steps after s2 (not applied here)
        s0_pos_in_pre = next(
            (i for i, sid in enumerate(pre_ids) if sid == 's0'), None)

        per_ch = list(selected)

        if s0_pos_in_pre is not None:
            # s0 is before s2: apply steps before s0 per-channel, then combine
            for sid in pre_ids[:s0_pos_in_pre]:
                if self._step_enabled(self._spec_groups[sid]):
                    per_ch = [(c, self._apply_spec_step(sid, s)) for c, s in per_ch]
            specs = [s for _, s in per_ch]
            znorm = self.znorm_before_combine_cb.isChecked()
            method = self.combine_combo.currentText()
            current = (combine_spectrograms(specs, method, znorm=znorm)
                       if len(specs) > 1 else
                       (znorm_spec(specs[0]) if znorm else specs[0]))
            for sid in pre_ids[s0_pos_in_pre+1:]:
                if self._step_enabled(self._spec_groups[sid]):
                    current = self._apply_spec_step(sid, current)
        else:
            # s0 comes after s2: apply all pre_ids steps per-channel, then combine
            for sid in pre_ids:
                if self._step_enabled(self._spec_groups[sid]):
                    per_ch = [(c, self._apply_spec_step(sid, s)) for c, s in per_ch]
            specs = [s for _, s in per_ch]
            znorm  = self.znorm_before_combine_cb.isChecked()
            method = self.combine_combo.currentText()
            current = (combine_spectrograms(specs, method, znorm=znorm)
                       if len(specs) > 1 else
                       (znorm_spec(specs[0]) if znorm else specs[0]))
        return current

    def _run_ts_pipeline(self, spec):
        """Apply the enabled time-series steps (s5–s7, not s8/s9) to a spec and return (timestamps, values)."""
        s5 = self._ts_groups['s5']
        if not self._step_enabled(s5):
            return None
        ts = aggregate_to_timeseries(spec, self.agg_combo.currentText())
        s5b = self._ts_groups['s5b']
        if self._step_enabled(s5b):
            ts = apply_binning(ts[0], ts[1],
                               self.bin_size_spin.value(),
                               self.bin_step_spin.value(),
                               self.bin_func_combo.currentText())
        s6 = self._ts_groups['s6']
        if self._step_enabled(s6):
            ts = apply_smoothing(ts[0], ts[1],
                                 self.smooth_spin.value(),
                                 method=self.smooth_method.currentText(),
                                 step=self.smooth_step.value(),
                                 sg_polyorder=self.smooth_sg_poly.value(),
                                 loess_frac=self.smooth_loess_frac.value())
        s7 = self._ts_groups['s7']
        if self._step_enabled(s7):
            ts = apply_detrending(ts[0], ts[1],
                                  self.detrend_combo.currentText(),
                                  self.detrend_window.value())
        return ts

    def _export_timeseries(self):
        try:
            if self._last_ts is None:
                QMessageBox.information(self, "Nothing to export",
                                        "Run the pipeline first (Time Series Aggregation must be enabled).")
                return

            timestamps, values = self._last_ts
            dts = pd.to_datetime(timestamps, unit='s', utc=True).tz_convert(None)

            df = pd.DataFrame({
                'datetime_utc':   dts.strftime('%Y-%m-%d %H:%M:%S'),
                'timestamp_unix': timestamps,
                'value':          values,
            })

            # Per-band columns — only if bands are entered and spec is available
            if self._last_processed_spec is not None:
                bands = parse_manual_bands(self.stats_bands_edit.text().strip())
                for lo, hi in bands:
                    band_spec = apply_freq_filter(self._last_processed_spec, lo, hi)
                    band_ts   = self._run_ts_pipeline(band_spec)
                    if band_ts is not None:
                        col = f"{lo:.0f}-{hi:.0f}Hz"
                        # Align by position — band_ts has the same timestamps as main ts
                        df[col] = band_ts[1]

            path, _ = QFileDialog.getSaveFileName(
                self, "Save time series CSV", "", "CSV files (*.csv)")
            if not path:
                return
            if not path.endswith('.csv'):
                path += '.csv'
            df.to_csv(path, index=False)
            print(f"[export] wrote {len(df)} rows to {path}", flush=True)
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Export error", str(e))

    def _show_band_stats(self):
        try:
            spec = self._current_processed_spec()
            if spec is None:
                QMessageBox.information(self, "No data",
                                        "Load a spectrogram first.")
                return
            bands = parse_manual_bands(self.stats_bands_edit.text().strip())
            if not bands:
                QMessageBox.information(self, "No bands",
                                        "Enter at least one frequency band "
                                        "(e.g. 200-400 800-1200).")
                return
            rows = compute_band_stats(spec, bands)
            dlg  = FreqBandStatsDialog(rows, parent=self)
            dlg.exec_()
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Stats error", str(e))

    def _current_processed_spec(self):
        """Return the fully-processed combined spectrogram (all enabled spec steps applied)."""
        selected = self._selected_specs()
        if not selected:
            return None
        s0_pos   = self._spec_step_order.index('s0')
        pre_ids  = self._spec_step_order[:s0_pos]
        post_ids = self._spec_step_order[s0_pos + 1:]

        per_ch = list(selected)
        for sid in pre_ids:
            if self._step_enabled(self._spec_groups[sid]):
                per_ch = [(c, self._apply_spec_step(sid, s)) for c, s in per_ch]

        specs = [s for _, s in per_ch]
        znorm  = self.znorm_before_combine_cb.isChecked()
        method = self.combine_combo.currentText()
        current = (combine_spectrograms(specs, method, znorm=znorm)
                   if len(specs) > 1 else
                   (znorm_spec(specs[0]) if znorm else specs[0]))

        for sid in post_ids:
            if self._step_enabled(self._spec_groups[sid]):
                current = self._apply_spec_step(sid, current)
        return current

    # ── Pipeline & plotting ───────────────────────────────────────────────────

    def _schedule_replot(self):
        self._refresh_timer.start(80)

    def _replot(self):
        try:
            self._replot_inner()
        except Exception as e:
            traceback.print_exc()
            self.fig.clear()
            ax = self.fig.add_subplot(1, 1, 1)
            ax.set_facecolor('#1e1e2e')
            ax.text(0.5, 0.5, f"Error during plotting:\n{e}",
                    ha='center', va='center', transform=ax.transAxes,
                    color='#ff6666', fontsize=10, wrap=True)
            self.canvas.draw()

    def _replot_inner(self):
        import time
        t_plot = time.perf_counter()
        print("\n[replot] starting …", flush=True)

        selected = self._selected_specs()
        self.fig.clear()

        if not selected:
            ax = self.fig.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5,
                    "No data loaded (or no channels selected).\n"
                    "Use the buttons on the left to load Spectrogram CSV files.",
                    ha='center', va='center', transform=ax.transAxes,
                    color='white', fontsize=12)
            ax.set_facecolor('#1e1e2e')
            self.canvas.draw()
            return

        cmap   = self.cmap_combo.currentText()
        clip   = self.clip_spin.value()
        stages = []   # list of (title, data, plot_type) to draw

        combine_method = self.combine_combo.currentText()
        znorm          = self.znorm_before_combine_cb.isChecked()

        # ── Spectrogram phase ─────────────────────────────────────────────────
        s0_pos   = self._spec_step_order.index('s0')
        pre_ids  = self._spec_step_order[:s0_pos]
        post_ids = self._spec_step_order[s0_pos+1:]

        # 1. Apply pre-combine steps to each channel individually
        per_ch = list(selected)   # [(channel_num, spec_array), ...]

        for sid in pre_ids:
            group = self._spec_groups[sid]
            if not self._step_enabled(group):
                continue
            t0 = time.perf_counter()
            per_ch = [(c, self._apply_spec_step(sid, s)) for c, s in per_ch]
            print(f"  [spec pre-combine] {self._step_label(sid)}: {time.perf_counter()-t0:.2f}s", flush=True)
            if group._show_plot_cb.isChecked():
                display_specs = [s for _, s in per_ch]
                disp = (combine_spectrograms(display_specs, combine_method, znorm=znorm)
                        if len(display_specs) > 1 else
                        (znorm_spec(display_specs[0]) if znorm else display_specs[0]))
                n_ch = len(per_ch)
                ch_tag = (f"Ch {per_ch[0][0]}" if n_ch == 1
                          else f"Ch {','.join(str(c) for c, _ in per_ch)} ({combine_method})")
                stages.append((
                    f"[per-channel] {self._step_label(sid)} — {ch_tag}",
                    disp, 'heatmap'))

        # 2. Combine (s0)
        t0 = time.perf_counter()
        specs_to_combine = [s for _, s in per_ch]
        if len(specs_to_combine) == 1:
            combined = znorm_spec(specs_to_combine[0]) if znorm else specs_to_combine[0]
        else:
            combined = combine_spectrograms(specs_to_combine, combine_method, znorm=znorm)
        print(f"  [combine s0] {combine_method}: {time.perf_counter()-t0:.2f}s  shape={spec_power(combined).shape}", flush=True)

        s0_group = self._spec_groups['s0']
        if s0_group._show_plot_cb.isChecked():
            ch_str = ", ".join(str(c) for c, _ in selected)
            if len(selected) == 1:
                label = f"Ch {ch_str}" + (" [Z-norm]" if znorm else "")
            else:
                label = (f"Combined ({combine_method}): Ch {ch_str}"
                         + (" [Z-norm]" if znorm else ""))
            stages.append((label, combined, 'heatmap'))

        current = combined

        # 3. Apply post-combine spec steps
        for sid in post_ids:
            group = self._spec_groups[sid]
            if not self._step_enabled(group):
                continue
            t0 = time.perf_counter()
            current = self._apply_spec_step(sid, current)
            print(f"  [spec post-combine] {self._step_label(sid)}: {time.perf_counter()-t0:.2f}s", flush=True)
            if group._show_plot_cb.isChecked():
                stages.append((self._step_label(sid), current, 'heatmap'))

        # ── Time-series phase ─────────────────────────────────────────────────
        ts = None

        s5 = self._ts_groups['s5']
        if self._step_enabled(s5):
            t0 = time.perf_counter()
            method = self.agg_combo.currentText()
            ts = aggregate_to_timeseries(current, method)
            print(f"  [s5 time-agg] {method}: {time.perf_counter()-t0:.2f}s", flush=True)
            if s5._show_plot_cb.isChecked():
                ts_series = [('All freqs', ts[0], ts[1])]
                bands = parse_manual_bands(self.stats_bands_edit.text().strip())
                for lo, hi in bands:
                    band_spec = apply_freq_filter(current, lo, hi)
                    band_ts   = aggregate_to_timeseries(band_spec, method)
                    ts_series.append((f'{lo:.0f}–{hi:.0f} Hz', band_ts[0], band_ts[1]))
                stages.append((f"Time series ({method} across freq)", ts_series, 'timeseries'))

        if ts is not None:
            s5b = self._ts_groups['s5b']
            if self._step_enabled(s5b):
                t0 = time.perf_counter()
                ts = apply_binning(ts[0], ts[1],
                                   self.bin_size_spin.value(),
                                   self.bin_step_spin.value(),
                                   self.bin_func_combo.currentText())
                print(f"  [s5b binning] size={self.bin_size_spin.value()}min "
                      f"step={self.bin_step_spin.value()}min "
                      f"func={self.bin_func_combo.currentText()}: "
                      f"{time.perf_counter()-t0:.2f}s", flush=True)
                if s5b._show_plot_cb.isChecked():
                    ts_series = [('All freqs', ts[0], ts[1])]
                    bands = parse_manual_bands(self.stats_bands_edit.text().strip())
                    for lo, hi in bands:
                        band_spec   = apply_freq_filter(current, lo, hi)
                        band_ts_agg = aggregate_to_timeseries(band_spec, self.agg_combo.currentText())
                        band_ts_bin = apply_binning(band_ts_agg[0], band_ts_agg[1],
                                                    self.bin_size_spin.value(),
                                                    self.bin_step_spin.value(),
                                                    self.bin_func_combo.currentText())
                        ts_series.append((f'{lo:.0f}–{hi:.0f} Hz', band_ts_bin[0], band_ts_bin[1]))
                    stages.append((
                        f"Binned ({self.bin_func_combo.currentText()}, "
                        f"{self.bin_size_spin.value():.0f} min bins, "
                        f"{self.bin_step_spin.value():.0f} min step)",
                        ts_series, 'timeseries'))

            s6 = self._ts_groups['s6']
            if self._step_enabled(s6):
                t0     = time.perf_counter()
                w      = self.smooth_spin.value()
                sm     = self.smooth_method.currentText()
                step   = self.smooth_step.value()
                ts = apply_smoothing(ts[0], ts[1], w,
                                     method=sm, step=step,
                                     sg_polyorder=self.smooth_sg_poly.value(),
                                     loess_frac=self.smooth_loess_frac.value())
                print(f"  [s6 smooth] {sm} window={w} step={step}: {time.perf_counter()-t0:.2f}s", flush=True)
                if s6._show_plot_cb.isChecked():
                    ts_series = [('All freqs', ts[0], ts[1])]
                    bands = parse_manual_bands(self.stats_bands_edit.text().strip())
                    for lo, hi in bands:
                        band_spec = apply_freq_filter(current, lo, hi)
                        b = self._run_ts_pipeline(band_spec)
                        if b is not None:
                            ts_series.append((f'{lo:.0f}–{hi:.0f} Hz', b[0], b[1]))
                    stages.append((f"Smoothed ({sm}, w={w}, step={step})", ts_series, 'timeseries'))

            s7 = self._ts_groups['s7']
            if self._step_enabled(s7):
                t0 = time.perf_counter()
                method = self.detrend_combo.currentText()
                dw     = self.detrend_window.value()
                ts     = apply_detrending(ts[0], ts[1], method, dw)
                print(f"  [s7 detrend] {method}: {time.perf_counter()-t0:.2f}s", flush=True)
                if s7._show_plot_cb.isChecked():
                    ts_series = [('All freqs', ts[0], ts[1])]
                    bands = parse_manual_bands(self.stats_bands_edit.text().strip())
                    for lo, hi in bands:
                        band_spec = apply_freq_filter(current, lo, hi)
                        b = self._run_ts_pipeline(band_spec)
                        if b is not None:
                            ts_series.append((f'{lo:.0f}–{hi:.0f} Hz', b[0], b[1]))
                    stages.append((f"Detrended ({method})", ts_series, 'timeseries'))

            s8 = self._ts_groups['s8']
            if self._step_enabled(s8):
                t0 = time.perf_counter()
                period = self.circ_period.value()
                try:
                    acto = build_actogram(ts[0], ts[1], period)
                    print(f"  [s8 actogram] {period}h: {time.perf_counter()-t0:.2f}s", flush=True)
                    if s8._show_plot_cb.isChecked():
                        stages.append((f"Actogram ({period}h period)", acto, 'actogram'))
                except Exception as e:
                    stages.append((f"Actogram error: {e}", None, 'error'))

        # ── Periodogram stage (s9) ────────────────────────────────────────────
        s9 = self._ts_groups['s9']
        if ts is not None and self._step_enabled(s9):
            t0 = time.perf_counter()
            pgram_series = [('All freqs', ts[0], ts[1])]
            bands = parse_manual_bands(self.stats_bands_edit.text().strip())
            for lo, hi in bands:
                band_spec = apply_freq_filter(current, lo, hi)
                band_ts   = self._run_ts_pipeline(band_spec)
                if band_ts is not None:
                    pgram_series.append((f'{lo:.0f}–{hi:.0f} Hz', band_ts[0], band_ts[1]))
            print(f"  [periodogram] {len(pgram_series)} series built: {time.perf_counter()-t0:.2f}s", flush=True)
            if s9._show_plot_cb.isChecked():
                stages.append(('Lomb-Scargle Periodogram (15–30 h)', pgram_series, 'periodogram'))

        # Cache for export (updated every replot so export always reflects current state)
        self._last_processed_spec = current
        self._last_ts             = ts

        # ── Raw channel panels (2-column grid, shown before pipeline stages) ────
        raw_channels = []
        if self.show_raw_cb.isChecked():
            raw_channels = [(c, s) for c, s in self.spectrograms if s is not None]

        # ── Draw ──────────────────────────────────────────────────────────────
        if not stages and not raw_channels:
            ax = self.fig.add_subplot(1, 1, 1)
            ax.set_facecolor('#1e1e2e')
            ax.text(0.5, 0.5,
                    "No steps have 'Show' ticked.\n"
                    "Check 'Show' on any step to display its output here.",
                    ha='center', va='center', transform=ax.transAxes,
                    color='#aaaacc', fontsize=12)
            self.canvas.draw()
            return

        import math
        from matplotlib.gridspec import GridSpec

        n_raw      = len(raw_channels)
        n_raw_rows = math.ceil(n_raw / 3)   # 3 columns for raw panels
        n_pipe     = len(stages)
        total_rows = n_raw_rows + n_pipe

        self.canvas.setMinimumHeight(max(600, n_raw_rows * 260 + n_pipe * 280))

        # GridSpec: 3 columns; pipeline stages span all cols
        gs = GridSpec(total_rows, 3, figure=self.fig,
                      hspace=0.45, wspace=0.15,
                      left=0.06, right=0.97, top=0.97, bottom=0.03)

        def _style_ax(ax):
            ax.set_facecolor('#2a2a3e')
            ax.tick_params(colors='#cccccc', labelsize=7)
            for sp in ax.spines.values():
                sp.set_edgecolor('#555577')

        # Raw channels — 3 per row
        for i, (ch, spec) in enumerate(raw_channels):
            row, col = divmod(i, 3)
            ax = self.fig.add_subplot(gs[row, col])
            _style_ax(ax)
            t0 = time.perf_counter()
            self._draw_heatmap(ax, spec, cmap, clip)
            print(f"  [draw raw] Ch{ch}: {time.perf_counter()-t0:.2f}s", flush=True)
            ax.set_title(f"Ch {ch}  (raw)", color='#e0e0f0', fontsize=9, pad=3)

        # Pipeline stages — full width
        for i, (title, data, ptype) in enumerate(stages):
            row = n_raw_rows + i
            ax  = self.fig.add_subplot(gs[row, :])
            _style_ax(ax)

            t0 = time.perf_counter()
            if ptype == 'heatmap':
                self._draw_heatmap(ax, data, cmap, clip)
            elif ptype == 'timeseries':
                self._draw_timeseries(ax, data)
            elif ptype == 'actogram':
                self._draw_actogram(ax, data, cmap)
            elif ptype == 'periodogram':
                self._draw_periodogram(ax, data)
            elif ptype == 'error':
                ax.text(0.5, 0.5, title, ha='center', va='center',
                        transform=ax.transAxes, color='red', fontsize=9)
                continue
            print(f"  [draw] '{title[:50]}' ({ptype}): {time.perf_counter()-t0:.2f}s", flush=True)
            ax.set_title(title, color='#e0e0f0', fontsize=9, pad=3)

        t0 = time.perf_counter()
        self.fig.set_facecolor('#1e1e2e')
        self.canvas.draw()
        print(f"  [draw] canvas.draw(): {time.perf_counter()-t0:.2f}s", flush=True)
        print(f"[replot] total: {time.perf_counter()-t_plot:.2f}s\n", flush=True)

    # ── Drawing helpers ───────────────────────────────────────────────────────

    def _draw_heatmap(self, ax, spec, cmap, clip):
        power = spec_power(spec)
        times = spec_times(spec)
        freqs = spec_freqs(spec)

        n_cols = power.shape[1]
        if n_cols > MAX_DISPLAY_COLS:
            step  = n_cols // MAX_DISPLAY_COLS
            power = power[:, ::step]
            times = times[::step]

        n_rows = power.shape[0]
        if n_rows > MAX_DISPLAY_ROWS:
            step  = n_rows // MAX_DISPLAY_ROWS
            power = power[::step, :]
            freqs = freqs[::step]

        try:
            # mdates.date2num accepts numpy datetime64 directly — avoids slow
            # per-element Python datetime object conversion via to_pydatetime()
            dts    = pd.to_datetime(times, unit='s', utc=True).tz_convert(None)
            t_nums = mdates.date2num(dts.values.astype('datetime64[ms]'))
            use_dates = True
        except Exception:
            t_nums    = times
            use_dates = False

        finite = power[np.isfinite(power)]
        if len(finite) == 0:
            return
        vmin = np.percentile(finite, clip)       if clip > 0 else finite.min()
        vmax = np.percentile(finite, 100 - clip) if clip > 0 else finite.max()

        extent = [t_nums[0], t_nums[-1], freqs[0], freqs[-1]]
        im = ax.imshow(power, aspect='auto', origin='lower', cmap=cmap,
                       vmin=vmin, vmax=vmax, extent=extent)
        cb = self.fig.colorbar(im, ax=ax, fraction=0.02, pad=0.01)
        cb.ax.yaxis.label.set_color('#cccccc')
        cb.ax.tick_params(colors='#cccccc', labelsize=6)
        ax.set_ylabel("Frequency (Hz)", color='#cccccc', fontsize=8)
        if use_dates:
            ax.xaxis_date()
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%d %b\n%H:%M'))
            ax.xaxis.set_major_locator(mdates.AutoDateLocator())

    def _draw_timeseries(self, ax, ts_data):
        """ts_data may be (timestamps, values) or [(label, timestamps, values), ...]."""
        _COLOURS = ['#88aaff', '#ff8877', '#77ee99', '#ffbb44',
                    '#cc88ff', '#44eedd', '#ffff66', '#ff88cc']

        # Normalise to list format
        if isinstance(ts_data, (list, tuple)) and len(ts_data) and isinstance(ts_data[0], tuple) and isinstance(ts_data[0][0], str):
            series_list = ts_data
        else:
            series_list = [('', ts_data[0], ts_data[1])]

        all_finite = []
        for i, (label, timestamps, values) in enumerate(series_list):
            colour = _COLOURS[i % len(_COLOURS)]
            lw     = 0.8 if i == 0 else 0.7
            alpha  = 0.9 if i == 0 else 0.75
            try:
                dts = pd.to_datetime(timestamps, unit='s', utc=True).tz_convert(None)
                x   = dts.values.astype('datetime64[ms]')
                ax.plot(x, values, color=colour, lw=lw, alpha=alpha,
                        label=label if label else None)
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%d %b\n%H:%M'))
                ax.xaxis.set_major_locator(mdates.AutoDateLocator())
            except Exception:
                ax.plot(timestamps, values, color=colour, lw=lw, alpha=alpha,
                        label=label if label else None)
            fin = values[np.isfinite(values)]
            if len(fin):
                all_finite.extend(fin.tolist())

        if len(series_list) > 1:
            ax.legend(fontsize=7, facecolor='#2a2a3e', labelcolor='#cccccc',
                      framealpha=0.8, edgecolor='#555577')

        ax.set_ylabel("Power", color='#cccccc', fontsize=8)
        ax.axhline(0, color='#555577', lw=0.5)
        ax.margins(x=0)
        if all_finite:
            lo, hi = min(all_finite), max(all_finite)
            pad = (hi - lo) * 0.05 if hi != lo else max(abs(lo) * 0.05, 0.1)
            ax.set_ylim(lo - pad, hi + pad)

    def _draw_actogram(self, ax, acto_data, cmap):
        grid, day_labels, period_hours = acto_data
        n_days, n_cols = grid.shape   # n_cols = 2 * n_bins (double-plotted)

        finite = grid[np.isfinite(grid)]
        vmin = np.percentile(finite, 2)  if len(finite) else 0
        vmax = np.percentile(finite, 98) if len(finite) else 1

        # extent: x spans 0 → 2*period, y spans 0 → n_days
        im = ax.imshow(grid, aspect='auto', origin='upper', cmap=cmap,
                       vmin=vmin, vmax=vmax,
                       extent=[0, 2 * period_hours, n_days, 0],
                       interpolation='nearest')
        cb = self.fig.colorbar(im, ax=ax, fraction=0.02, pad=0.01)
        cb.ax.yaxis.label.set_color('#cccccc')
        cb.ax.tick_params(colors='#cccccc', labelsize=6)

        # Vertical divider between the two plot-periods
        ax.axvline(period_hours, color='#ffffff', lw=0.6, alpha=0.5)

        # X-axis: hour ticks every 6 h across both periods
        tick_step = 6
        xticks = np.arange(0, 2 * period_hours + 1, tick_step)
        ax.set_xticks(xticks)
        ax.set_xticklabels([f"{int(t % period_hours):02d}h" for t in xticks],
                           color='#cccccc', fontsize=6)
        ax.set_xlim(0, 2 * period_hours)

        # Y-axis: one label per day
        ax.set_yticks(np.arange(n_days) + 0.5)
        ax.set_yticklabels(day_labels, color='#cccccc', fontsize=6)
        ax.set_xlabel(f"Time (double-plotted, period = {period_hours:.1f} h)",
                      color='#cccccc', fontsize=8)
        ax.set_ylabel("Day", color='#cccccc', fontsize=8)

    def _draw_periodogram(self, ax, pgram_series):
        """Draw overlaid LS periodograms for each (label, timestamps, values) in pgram_series."""
        _COLOURS = ['#88aaff', '#ff8877', '#77ee99', '#ffbb44',
                    '#cc88ff', '#44eedd', '#ffff66', '#ff88cc']

        peak_lines = []   # (label, peak_period, peak_power) for summary box

        for i, (label, timestamps, values) in enumerate(pgram_series):
            colour = _COLOURS[i % len(_COLOURS)]
            periods, power, peak_p, peak_pw = compute_lomb_scargle(timestamps, values)
            if periods is None:
                continue
            ax.plot(periods, power, color=colour, lw=1.0, alpha=0.85, label=label)
            ax.axvline(peak_p, color=colour, lw=0.7, linestyle='--', alpha=0.55)
            peak_lines.append((label, peak_p, peak_pw))

        ax.set_xlim(15, 30)
        ax.set_ylim(bottom=0)
        ## Reference line at 24 h
        #ax.axvline(24, color='#555577', lw=0.9, linestyle=':', alpha=0.8)
        ax.set_xlabel("Period (hours)", color='#cccccc', fontsize=8)
        ax.set_ylabel("LS Power",       color='#cccccc', fontsize=8)
        ax.set_xticks(np.arange(15, 31, 2))

        if len(pgram_series) > 1:
            legend = ax.legend(fontsize=7, facecolor='#2a2a3e', labelcolor='#cccccc',
                               framealpha=0.8, edgecolor='#555577')

        # Peak summary box
        if peak_lines:
            summary = "\n".join(
                f"{lbl}: {pp:.2f} h  (power {pw:.4f})"
                for lbl, pp, pw in peak_lines
            )
            ax.text(0.98, 0.97, summary,
                    transform=ax.transAxes, ha='right', va='top',
                    color='#e0e0f0', fontsize=7.5,
                    bbox=dict(boxstyle='round,pad=0.35', facecolor='#1e1e2e', alpha=0.75,
                              edgecolor='#555577'))


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    def _qt_exception_hook(exc_type, exc_value, exc_tb):
        traceback.print_exception(exc_type, exc_value, exc_tb)
    sys.excepthook = _qt_exception_hook

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    palette = QPalette()
    palette.setColor(QPalette.Window,          QColor(30, 30, 46))
    palette.setColor(QPalette.WindowText,      QColor(220, 220, 240))
    palette.setColor(QPalette.Base,            QColor(40, 40, 60))
    palette.setColor(QPalette.AlternateBase,   QColor(50, 50, 70))
    palette.setColor(QPalette.ToolTipBase,     QColor(30, 30, 46))
    palette.setColor(QPalette.ToolTipText,     QColor(220, 220, 240))
    palette.setColor(QPalette.Text,            QColor(220, 220, 240))
    palette.setColor(QPalette.Button,          QColor(55, 55, 80))
    palette.setColor(QPalette.ButtonText,      QColor(220, 220, 240))
    palette.setColor(QPalette.Highlight,       QColor(100, 100, 200))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = CircadianPipelineApp()
    win.show()
    sys.exit(app.exec_())
