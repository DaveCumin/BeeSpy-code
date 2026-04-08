"""
Batch Daily Spectrogram + Activity Pipeline
-------------------------------------------
Points to a folder of .bin files (local or Dropbox) and processes them
day by day over a user-selected date range.

For each calendar day it produces two outputs in the output folder:
  • YYYYMMDD_ch1.csv … YYYYMMDD_ch6.csv   — per-channel spectrogram
  • YYYYMMDD_activity.csv                 — activity time series

The activity pipeline for each day:
  1. Z-normalise each channel spectrogram globally
  2. Sum channels into a single spectrogram
  3. Apply 80th-percentile cutoff (cells below threshold → NaN)
  4. Sum across frequency axis → 1-D time series

Days whose output files already exist are skipped automatically (resume safe).

Usage:
    python claudeHelper_20260402_batch_daily_pipeline.py
"""

import os
import sys
import datetime
import traceback

import numpy as np
import pandas as pd

from PyQt5.QtCore import Qt, QThread, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QPlainTextEdit, QSpinBox, QDoubleSpinBox,
    QGroupBox, QGridLayout, QProgressBar, QMessageBox, QDateTimeEdit,
    QSplitter, QCheckBox,
)

# Import batch-processing machinery from the existing localApp
# (safe to import — the Qt app only starts inside its __main__ block)
from localApp import (
    process_bin_files,
    DropboxAuthDialog,
    DropboxFolderBrowser,
)
from utils import utils
from utils import dropbox_helper


# ══════════════════════════════════════════════════════════════════════════════
# stdout → log widget (same pattern as localApp)
# ══════════════════════════════════════════════════════════════════════════════

class _LogEmitter(QObject):
    """Emits log messages as a Qt signal so background threads can update the UI safely."""
    message = pyqtSignal(str)


class _StdoutRedirector:
    """Replaces sys.stdout so every print() appears in the footer log widget."""
    def __init__(self, emitter: _LogEmitter):
        self._emitter = emitter

    def write(self, msg):
        if msg.strip():
            self._emitter.message.emit(msg.rstrip())

    def flush(self):
        pass


# ══════════════════════════════════════════════════════════════════════════════
# Spec array helpers (mirrors claudeHelper_20260315_circadian_prototype.py)
# ══════════════════════════════════════════════════════════════════════════════

def spec_freqs(spec):  return spec[1:, 0]
def spec_times(spec):  return spec[0, 1:]
def spec_power(spec):  return spec[1:, 1:]


def rebuild_spec(freqs, times, power):
    """Reconstruct the packed spec array from components."""
    dt = power.dtype
    out = np.empty((len(freqs) + 1, len(times) + 1), dtype=dt)
    out[0, 0] = 0
    out[0, 1:] = times
    out[1:, 0] = freqs
    out[1:, 1:] = power
    return out


# ══════════════════════════════════════════════════════════════════════════════
# Activity pipeline functions
# ══════════════════════════════════════════════════════════════════════════════

# Frequency bands reported in the output CSV
FREQ_BANDS = [(0, 40), (40, 80), (80, 120), (120, 160), (160, 200), (200, 300)]


def _znorm_spec(spec):
    """Global Z-normalise a single spectrogram."""
    power  = spec_power(spec).astype(np.float32)
    mu     = np.nanmean(power)
    sigma  = np.nanstd(power)
    normed = (power - mu) / (sigma + np.float32(1e-7))
    return rebuild_spec(spec_freqs(spec), spec_times(spec), normed)


def _combine_znorm_sum(specs):
    """Z-normalise each channel then sum into one combined spectrogram."""
    znormed  = [_znorm_spec(s) for s in specs]
    min_t    = min(spec_power(s).shape[1] for s in znormed)
    acc      = spec_power(znormed[0])[:, :min_t].copy()
    nan_mask = np.isnan(acc)
    acc      = np.where(nan_mask, np.float32(0), acc)
    for s in znormed[1:]:
        p     = spec_power(s)[:, :min_t]
        valid = ~np.isnan(p)
        acc  += np.where(valid, p, np.float32(0))
        nan_mask &= ~valid
    acc[nan_mask] = np.nan
    return rebuild_spec(spec_freqs(znormed[0]), spec_times(znormed[0])[:min_t], acc)


def _percentile_cutoff(spec, percentile=75):
    """Set cells below the Nth percentile to NaN."""
    power     = spec_power(spec).copy()
    threshold = np.nanpercentile(power, percentile)
    power[power < threshold] = np.nan
    return rebuild_spec(spec_freqs(spec), spec_times(spec), power)


def _sum_to_timeseries(spec):
    """Sum across frequency axis per time column (NaN-safe; all-NaN columns → NaN)."""
    power   = spec_power(spec)
    values  = np.nansum(power, axis=0)
    all_nan = np.all(np.isnan(power), axis=0)
    values[all_nan] = np.nan
    return spec_times(spec), values


def _apply_freq_filter(spec, lo, hi):
    """Return a spectrogram slice restricted to [lo, hi] Hz, or None if no rows match."""
    freqs = spec_freqs(spec)
    mask  = (freqs >= lo) & (freqs <= hi)
    if not mask.any():
        return None
    return rebuild_spec(freqs[mask], spec_times(spec), spec_power(spec)[mask, :])


def _bin_timeseries(timestamps, values, bin_size_min=10, bin_step_min=10):
    """Average a time series into non-overlapping bins (bin_size == bin_step → no overlap).

    Returns (bin_centre_timestamps, bin_mean_values).
    """
    if len(timestamps) < 2:
        return timestamps, values

    bin_size_s = bin_size_min * 60.0
    bin_step_s = bin_step_min * 60.0

    t_start = timestamps[0]
    t_end   = timestamps[-1]

    centres = np.arange(t_start + bin_size_s / 2.0,
                        t_end   - bin_size_s / 2.0 + bin_step_s,
                        bin_step_s)

    bin_vals = []
    for c in centres:
        mask   = (timestamps >= c - bin_size_s / 2.0) & (timestamps < c + bin_size_s / 2.0)
        finite = values[mask]
        finite = finite[np.isfinite(finite)]
        bin_vals.append(np.mean(finite) if len(finite) > 0 else np.nan)

    return centres, np.array(bin_vals)


def run_activity_pipeline(spectrograms, percentile=75):
    """Full activity pipeline for one day's spectrograms.

    Pipeline:
      1. Z-normalise each channel
      2. Sum channels into a combined spectrogram
      3. Apply Nth-percentile cutoff (cells below → NaN)
      4. Sum across frequency axis → all-data time series
      5. Sum across each frequency band → per-band time series
      6. Bin everything into 10-min intervals

    Returns (bin_timestamps, bin_activity, band_dict) where band_dict maps
    "lo-hi" → binned values array, or (None, None, None) if no usable data.
    """
    valid = [s for s in spectrograms
             if s is not None and not np.all(np.isnan(spec_power(s)))]
    if not valid:
        return None, None, None

    combined = _combine_znorm_sum(valid)
    filtered = _percentile_cutoff(combined, percentile)

    # All-data time series
    times, activity = _sum_to_timeseries(filtered)

    # Per-frequency-band time series
    raw_bands = {}
    for lo, hi in FREQ_BANDS:
        band_spec = _apply_freq_filter(filtered, lo, hi)
        if band_spec is not None:
            _, bvals = _sum_to_timeseries(band_spec)
        else:
            bvals = np.full(len(activity), np.nan)
        raw_bands[f"{lo}-{hi}"] = bvals

    # Bin to 10-min intervals
    bin_times, bin_activity = _bin_timeseries(times, activity)
    bin_bands = {}
    for key, bvals in raw_bands.items():
        _, binned = _bin_timeseries(times, bvals)
        bin_bands[key] = binned

    return bin_times, bin_activity, bin_bands


# ══════════════════════════════════════════════════════════════════════════════
# Output file helpers
# ══════════════════════════════════════════════════════════════════════════════

def _spec_path(output_dir, window_start, window_end, channel):
    s = window_start.strftime('%Y%m%d_%H%M%S')
    e = window_end.strftime('%Y%m%d_%H%M%S')
    return os.path.join(output_dir, f"{s}_{e}_ch{channel}.csv")


def _activity_path(output_dir, window_start, window_end):
    s = window_start.strftime('%Y%m%d_%H%M%S')
    e = window_end.strftime('%Y%m%d_%H%M%S')
    return os.path.join(output_dir, f"{s}_{e}_activity.csv")


def _day_is_complete(output_dir, window_start, window_end, save_specs=True):
    """True if the expected output files already exist.

    When save_specs is False only the activity CSV is checked (spectrogram
    files are never written in that mode so their absence is expected).
    """
    if not os.path.isfile(_activity_path(output_dir, window_start, window_end)):
        return False
    if save_specs:
        return all(os.path.isfile(_spec_path(output_dir, window_start, window_end, c))
                   for c in range(1, 7))
    return True


def _save_day(output_dir, window_start, window_end, spectrograms, timestamps, activity, band_dict,
              save_specs=True):
    """Write spectrogram CSVs and activity CSV for one window atomically."""
    os.makedirs(output_dir, exist_ok=True)
    if save_specs:
        for c, spec in enumerate(spectrograms, start=1):
            path = _spec_path(output_dir, window_start, window_end, c)
            tmp  = path + '.tmp'
            np.savetxt(tmp, spec, delimiter=',')
            os.replace(tmp, path)
    act_path = _activity_path(output_dir, window_start, window_end)
    dts = pd.to_datetime(timestamps, unit='s', utc=True)
    df  = pd.DataFrame({'timestamp_unix': timestamps,
                        'datetime_utc':   dts.strftime('%Y-%m-%d %H:%M:%S'),
                        'all_data':       activity})
    for key, vals in band_dict.items():
        lo, hi = key.split('-')
        df[f"band_{lo}_{hi}Hz"] = vals
    tmp = act_path + '.tmp'
    df.to_csv(tmp, index=False)
    os.replace(tmp, act_path)


# ══════════════════════════════════════════════════════════════════════════════
# QDateTime helpers
# ══════════════════════════════════════════════════════════════════════════════

def _to_qdt(dt):
    from PyQt5.QtCore import QDateTime
    return QDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)


def _qdt_to_dt(qdt):
    d = qdt.date(); t = qdt.time()
    return datetime.datetime(d.year(), d.month(), d.day(),
                             t.hour(), t.minute(), t.second())


# ══════════════════════════════════════════════════════════════════════════════
# Background worker
# ══════════════════════════════════════════════════════════════════════════════

class _BatchWorker(QThread):
    progress = pyqtSignal(int, int)   # (current, total)
    done     = pyqtSignal(int, int)   # (days_processed, days_skipped)
    error    = pyqtSignal(str)

    def __init__(self, days, bin_folder, bin_files, output_dir,
                 sampFreq, window, minFreq, maxFreq,
                 percentile,
                 dbx, dbx_folder,
                 continue_batch=True, save_specs=True,
                 parent=None):
        super().__init__(parent)
        self._days            = days
        self._bin_folder      = bin_folder
        self._bin_files       = bin_files
        self._output_dir      = output_dir
        self._sampFreq        = sampFreq
        self._window          = window
        self._minFreq         = minFreq
        self._maxFreq         = maxFreq
        self._percentile      = percentile
        self._dbx             = dbx
        self._dbx_folder      = dbx_folder
        self._continue_batch  = continue_batch
        self._save_specs      = save_specs

    def run(self):
        try:
            total     = len(self._days)
            processed = 0
            skipped   = 0

            for i, (day, window_start, window_end) in enumerate(self._days, start=1):
                self.progress.emit(i, total)

                if self._continue_batch and _day_is_complete(
                        self._output_dir, window_start, window_end, self._save_specs):
                    print(f"[{day}] Already complete — skipping.")
                    skipped += 1
                    continue

                print(f"\n[{day}] Processing bin files…")

                try:
                    specs = process_bin_files(
                        self._bin_folder or self._output_dir,
                        self._bin_files,
                        _to_qdt(window_start),
                        _to_qdt(window_end),
                        self._sampFreq,
                        self._window,
                        self._window,
                        self._minFreq,
                        self._maxFreq,
                        dbx=self._dbx,
                        dbx_folder=self._dbx_folder,
                        local_bin_folder=self._bin_folder,
                    )
                except Exception as e:
                    print(f"[{day}] ERROR in process_bin_files: {e}")
                    traceback.print_exc()
                    continue

                print(f"[{day}] Running activity pipeline…")
                try:
                    times, activity, band_dict = run_activity_pipeline(
                        specs, self._percentile)
                except Exception as e:
                    print(f"[{day}] ERROR in pipeline: {e}")
                    traceback.print_exc()
                    continue

                if times is None:
                    print(f"[{day}] No valid data — skipping output.")
                    skipped += 1
                    continue

                _save_day(self._output_dir, window_start, window_end, specs, times, activity, band_dict,
                          save_specs=self._save_specs)
                print(f"[{day}] Done — saved {len(times)} time points.")
                processed += 1

            self.done.emit(processed, skipped)
        except Exception as e:
            self.error.emit(traceback.format_exc())


# ══════════════════════════════════════════════════════════════════════════════
# Main GUI
# ══════════════════════════════════════════════════════════════════════════════

class BatchDailyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Batch Daily Spectrogram + Activity Pipeline v4.1")
        self.setMinimumWidth(720)

        self._bin_folder = None
        self._bin_files  = []
        self._output_dir = None
        self._dbx        = None
        self._dbx_folder = None
        self._worker     = None

        self._build_ui()

        # Redirect stdout so every print() (including from process_bin_files)
        # appears in the log widget — same as localApp
        self._log_emitter = _LogEmitter()
        self._log_emitter.message.connect(self._append_log)
        sys.stdout = _StdoutRedirector(self._log_emitter)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Inner layout for all controls — will sit in top half of splitter
        inner = QWidget()
        main  = QVBoxLayout(inner)
        main.setSpacing(8)

        # ── 1. Source selection ───────────────────────────────────────────────
        src_box = QGroupBox("1. Select .bin source")
        src_lay = QHBoxLayout(src_box)
        self._local_btn = QPushButton("Local folder…")
        self._local_btn.clicked.connect(self._pick_local_folder)
        src_lay.addWidget(self._local_btn)
        self._dbx_btn = QPushButton("Dropbox folder…")
        self._dbx_btn.clicked.connect(self._pick_dropbox_folder)
        src_lay.addWidget(self._dbx_btn)
        self._src_label = QLabel("No source selected")
        self._src_label.setWordWrap(True)
        self._src_label.setStyleSheet("color: grey; font-size: 10px;")
        src_lay.addWidget(self._src_label, 1)
        main.addWidget(src_box)

        # ── 2. Output folder ──────────────────────────────────────────────────
        out_box = QGroupBox("2. Select output folder")
        out_lay = QHBoxLayout(out_box)
        self._out_btn = QPushButton("Choose output folder…")
        self._out_btn.clicked.connect(self._pick_output_folder)
        out_lay.addWidget(self._out_btn)
        self._out_label = QLabel("No output folder selected")
        self._out_label.setWordWrap(True)
        self._out_label.setStyleSheet("color: grey; font-size: 10px;")
        out_lay.addWidget(self._out_label, 1)
        main.addWidget(out_box)

        # ── 3. Date range ─────────────────────────────────────────────────────
        date_box = QGroupBox("3. Date range")
        date_grid = QGridLayout(date_box)

        date_grid.addWidget(QLabel("Start:"), 0, 0)
        self._start_dt = QDateTimeEdit()
        self._start_dt.setCalendarPopup(True)
        self._start_dt.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self._start_dt.setEnabled(False)
        date_grid.addWidget(self._start_dt, 0, 1)

        date_grid.addWidget(QLabel("End:"), 1, 0)
        self._end_dt = QDateTimeEdit()
        self._end_dt.setCalendarPopup(True)
        self._end_dt.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self._end_dt.setEnabled(False)
        date_grid.addWidget(self._end_dt, 1, 1)

        main.addWidget(date_box)

        # ── 4. Settings ───────────────────────────────────────────────────────
        set_box  = QGroupBox("4. Settings")
        set_grid = QGridLayout(set_box)

        def _ispin(lo, hi, val):
            w = QSpinBox(); w.setRange(lo, hi); w.setValue(val); return w

        def _dspin(lo, hi, val, step, dec):
            w = QDoubleSpinBox()
            w.setRange(lo, hi); w.setValue(val)
            w.setSingleStep(step); w.setDecimals(dec)
            return w

        self._samp_freq  = _ispin(100, 100000, 5000)
        self._window     = _dspin(0.01, 10.0, 0.2, 0.05, 3)
        self._min_freq   = _ispin(0, 20000, 0)
        self._max_freq   = _ispin(0, 20000, 500)
        self._chunk_hrs  = _ispin(0, 240, 0)   # 0 = whole day as one chunk (localApp default)
        self._percentile = _ispin(1, 99, 75)

        rows = [
            ("Sampling freq (Hz):",       self._samp_freq),
            ("Window duration (s):",      self._window),
            ("Min freq (Hz):",            self._min_freq),
            ("Max freq (Hz):",            self._max_freq),
            ("Chunk size (hrs, 0=day):",  self._chunk_hrs),
            ("Percentile cutoff (%):",    self._percentile),
        ]
        for r, (lbl, w) in enumerate(rows):
            set_grid.addWidget(QLabel(lbl), r, 0)
            set_grid.addWidget(w, r, 1)

        n = len(rows)
        self._continue_cb = QCheckBox("Continue batch (skip windows whose activity file already exists)")
        self._continue_cb.setChecked(True)
        set_grid.addWidget(self._continue_cb, n, 0, 1, 2)

        self._skip_specs_cb = QCheckBox("Don't save spectrogram CSVs (activity file only)")
        self._skip_specs_cb.setChecked(False)
        set_grid.addWidget(self._skip_specs_cb, n + 1, 0, 1, 2)

        main.addWidget(set_box)

        # ── Run / Stop ────────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        self._run_btn = QPushButton("Run batch processing")
        self._run_btn.setEnabled(False)
        self._run_btn.clicked.connect(self._run)
        btn_row.addWidget(self._run_btn)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop)
        btn_row.addWidget(self._stop_btn)
        main.addLayout(btn_row)

        # ── Progress bar ──────────────────────────────────────────────────────
        self._progress = QProgressBar()
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        main.addWidget(self._progress)

        # ── Log (bottom pane of splitter) ─────────────────────────────────────
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(5000)
        self._log.setStyleSheet("font-family: monospace; font-size: 11px;")
        self._log.setPlaceholderText("Progress log…")

        # Splitter: controls on top, log on bottom (user-resizable, same as localApp)
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(inner)
        splitter.addWidget(self._log)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([600, 120])

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(splitter)

    def _append_log(self, msg: str):
        self._log.appendPlainText(msg)
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Folder pickers ────────────────────────────────────────────────────────

    def _pick_local_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select folder containing .bin files")
        if not folder:
            return
        files = sorted(f for f in os.listdir(folder) if f.endswith('.bin'))
        if not files:
            QMessageBox.warning(self, "No .bin files", "No .bin files found in that folder.")
            return
        self._bin_folder = folder
        self._bin_files  = files
        self._dbx        = None
        self._dbx_folder = None
        self._src_label.setText(f"{len(files)} .bin files in:\n{folder}")
        self._src_label.setStyleSheet("color: #226622; font-size: 10px;")
        self._populate_date_range()
        self._update_run_btn()

    def _pick_dropbox_folder(self):
        try:
            import dropbox  # noqa
        except ImportError:
            QMessageBox.critical(self, "Dropbox not installed",
                                 "Please install the Dropbox SDK:\n\n    pip install dropbox")
            return

        from PyQt5.QtWidgets import QDialog
        dbx = dropbox_helper.get_saved_client()
        if dbx is None:
            auth_dlg = DropboxAuthDialog(self)
            if auth_dlg.exec_() != QDialog.Accepted:
                return
            dbx = auth_dlg.dbx
            if dbx is None:
                return

        dbx = dropbox_helper.scope_to_root_namespace(dbx)

        browser = DropboxFolderBrowser(dbx, self)
        if browser.exec_() != browser.Accepted:
            return
        bin_entries = browser.selected_bin_files
        if not bin_entries:
            QMessageBox.warning(self, "No .bin files", "No .bin files found in that Dropbox folder.")
            return
        self._bin_folder = None
        self._bin_files  = [e.name for e in bin_entries]
        self._dbx        = dbx
        self._dbx_folder = browser.selected_path
        self._src_label.setText(
            f"{len(self._bin_files)} .bin files in Dropbox:\n{self._dbx_folder}")
        self._src_label.setStyleSheet("color: #226622; font-size: 10px;")
        self._populate_date_range()
        self._update_run_btn()

    def _pick_output_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select output folder")
        if folder:
            self._output_dir = folder
            self._out_label.setText(folder)
            self._out_label.setStyleSheet("color: #226622; font-size: 10px;")
            self._update_run_btn()

    # ── Date range auto-population ────────────────────────────────────────────

    def _populate_date_range(self):
        """Auto-set start/end date pickers from the bin file timestamps."""
        if not self._bin_files:
            return
        times = []
        for f in self._bin_files:
            try:
                times.append(utils.extract_start_time(f))
            except ValueError:
                continue
        if not times:
            return

        min_time = min(times).replace(hour=0,  minute=0,  second=0)
        max_time = max(times).replace(hour=23, minute=59, second=59)

        for w in (self._start_dt, self._end_dt):
            w.blockSignals(True)

        self._start_dt.setMinimumDateTime(_to_qdt(min_time))
        self._start_dt.setMaximumDateTime(_to_qdt(max_time))
        self._start_dt.setDateTime(_to_qdt(min_time))
        self._start_dt.setEnabled(True)

        self._end_dt.setMinimumDateTime(_to_qdt(min_time))
        self._end_dt.setMaximumDateTime(_to_qdt(max_time))
        self._end_dt.setDateTime(_to_qdt(max_time))
        self._end_dt.setEnabled(True)

        for w in (self._start_dt, self._end_dt):
            w.blockSignals(False)

        n_days = (max_time.date() - min_time.date()).days + 1
        self._src_label.setText(
            self._src_label.text()
            + f"\n{min_time.strftime('%d-%b-%Y')} → {max_time.strftime('%d-%b-%Y')}  ({n_days} day(s))")

    # ── Run / Stop ────────────────────────────────────────────────────────────

    def _update_run_btn(self):
        self._run_btn.setEnabled(bool(self._bin_files) and self._output_dir is not None)

    def _run(self):
        if not self._bin_files or self._output_dir is None:
            return

        start_dt = _qdt_to_dt(self._start_dt.dateTime())
        end_dt   = _qdt_to_dt(self._end_dt.dateTime())
        if start_dt > end_dt:
            QMessageBox.warning(self, "Bad date range", "Start date must be before end date.")
            return

        chunk_hrs = self._chunk_hrs.value()

        # Build list of processing windows.
        # chunk_hrs == 0: one window per calendar day in the selected range.
        # chunk_hrs >  0: fixed-hour windows spanning the selected range.
        if chunk_hrs == 0:
            # One entry per calendar day
            windows = []
            current = start_dt.date()
            while current <= end_dt.date():
                day_start = datetime.datetime(current.year, current.month, current.day, 0, 0, 0)
                day_end   = datetime.datetime(current.year, current.month, current.day, 23, 59, 59)
                # Clip to user-selected range
                day_start = max(day_start, start_dt)
                day_end   = min(day_end,   end_dt)
                windows.append((current, day_start, day_end))
                current  += datetime.timedelta(days=1)
        else:
            # Sub-day or multi-hour chunks; label each by the start date
            windows = []
            cur = start_dt
            delta = datetime.timedelta(hours=chunk_hrs)
            while cur < end_dt:
                chunk_end = min(cur + delta, end_dt)
                windows.append((cur.date(), cur, chunk_end))
                cur = chunk_end

        if not windows:
            QMessageBox.warning(self, "Empty range", "No days to process in the selected range.")
            return

        self._progress.setRange(0, len(windows))
        self._progress.setValue(0)
        self._log.clear()
        print(f"Starting batch: {len(windows)} window(s) | output → {self._output_dir}")
        print(f"Range: {start_dt.strftime('%d-%b-%Y %H:%M:%S')} → {end_dt.strftime('%d-%b-%Y %H:%M:%S')}")

        self._worker = _BatchWorker(
            days            = windows,
            bin_folder      = self._bin_folder,
            bin_files       = self._bin_files,
            output_dir      = self._output_dir,
            sampFreq        = self._samp_freq.value(),
            window          = self._window.value(),
            minFreq         = self._min_freq.value(),
            maxFreq         = self._max_freq.value(),
            percentile      = self._percentile.value(),
            dbx             = self._dbx,
            dbx_folder      = self._dbx_folder,
            continue_batch  = self._continue_cb.isChecked(),
            save_specs      = not self._skip_specs_cb.isChecked(),
            parent          = self,
        )
        self._worker.progress.connect(lambda cur, tot: self._progress.setValue(cur))
        self._worker.done.connect(self._on_done)
        self._worker.error.connect(self._on_error)

        self._run_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._worker.start()

    def _stop(self):
        if self._worker and self._worker.isRunning():
            self._worker.terminate()
            print("--- Stopped by user ---")
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)

    def _on_done(self, processed, skipped):
        print(f"\nAll done.  Processed: {processed}  Skipped/no-data: {skipped}")
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)

    def _on_error(self, msg):
        print(f"\nFATAL ERROR:\n{msg}")
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)


# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = BatchDailyApp()
    win.show()
    sys.exit(app.exec_())
