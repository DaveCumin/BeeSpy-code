import os
import re
import json
import pickle
import shutil
import datetime
import numpy as np
import pandas as pd
import sys
import math
import time
import threading
from utils import utils
from utils import binaryConvert as bc
from utils import denoiseSignal as denoise
from utils import spect
from utils import QThelpers as QThelpers
from utils import dropbox_helper
from PyQt5.QtCore import QObject, pyqtSignal, Qt, QThread, QDateTime
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QDateTimeEdit, QMessageBox, QLineEdit, QPlainTextEdit,
    QDialog, QListWidget, QListWidgetItem, QComboBox, QSplitter, QCheckBox,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


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


class DropboxAuthDialog(QDialog):
    """Two-step PKCE OAuth2 dialog for connecting a Dropbox app.

    The user supplies their App key (from dropbox.com/developers/apps) and the
    dialog walks them through the browser-based authorization code flow.
    The resulting tokens are persisted to ~/.beespy_dropbox.json so subsequent
    launches connect automatically.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to Dropbox")
        self.setMinimumWidth(500)
        self.dbx = None        # populated on success
        self._flow = None
        self._app_key = ""

        layout = QVBoxLayout(self)

        # ── Step 1: App key ──────────────────────────────────────────────────
        step1 = QLabel(
            "<b>Step 1.</b>  Create a Dropbox app at "
            "<a href='https://www.dropbox.com/developers/apps'>"
            "dropbox.com/developers/apps</a> "
            "(Scoped access → Full Dropbox → <i>files.content.read</i> permission), "
            "then paste your <b>App key</b> below.", self)
        step1.setWordWrap(True)
        step1.setOpenExternalLinks(True)
        layout.addWidget(step1)

        self._app_key_input = QLineEdit(self)
        self._app_key_input.setPlaceholderText("App key  (e.g. abc123xyz456)")
        saved_key = dropbox_helper.load_config().get("app_key", "")
        if saved_key:
            self._app_key_input.setText(saved_key)
        layout.addWidget(self._app_key_input)

        self._open_btn = QPushButton("Open Dropbox in browser →", self)
        self._open_btn.clicked.connect(self._open_browser)
        layout.addWidget(self._open_btn)

        # ── Step 2: Auth code ────────────────────────────────────────────────
        self._step2_label = QLabel(
            "<b>Step 2.</b>  After authorizing in the browser, paste the code here:", self)
        self._step2_label.setWordWrap(True)
        self._step2_label.hide()
        layout.addWidget(self._step2_label)

        self._code_input = QLineEdit(self)
        self._code_input.setPlaceholderText("Authorization code")
        self._code_input.hide()
        layout.addWidget(self._code_input)

        btn_row = QHBoxLayout()
        self._connect_btn = QPushButton("Connect", self)
        self._connect_btn.clicked.connect(self._finish)
        self._connect_btn.hide()
        btn_row.addWidget(self._connect_btn)
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)

        self._status = QLabel("", self)
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

    def _open_browser(self):
        import webbrowser
        self._app_key = self._app_key_input.text().strip()
        if not self._app_key:
            self._status.setText("Please enter your App key first.")
            return
        try:
            self._flow, url = dropbox_helper.start_oauth(self._app_key)
            webbrowser.open(url)
            self._step2_label.show()
            self._code_input.show()
            self._connect_btn.show()
            self._status.setText("Browser opened. Authorize, then paste the code above and click Connect.")
        except Exception as e:
            self._status.setText(f"Error starting auth: {e}")

    def _finish(self):
        code = self._code_input.text().strip()
        if not code:
            self._status.setText("Please paste the authorization code.")
            return
        try:
            self.dbx = dropbox_helper.finish_oauth(self._flow, code, self._app_key)
            self.accept()
        except Exception as e:
            self._status.setText(f"Connection failed: {e}")


class DropboxFolderBrowser(QDialog):
    """Navigate a Dropbox account tree and select the folder containing .bin files."""

    def __init__(self, dbx, parent=None):
        super().__init__(parent)
        self.dbx = dbx
        self.setWindowTitle("Select Dropbox folder")
        self.setMinimumSize(540, 480)
        self.selected_path = None        # Dropbox folder path on accept
        self.selected_bin_files = []     # list of FileMetadata for .bin files
        self._current_path = ""

        layout = QVBoxLayout(self)

        # Manual path entry bar
        path_row = QHBoxLayout()
        path_row.addWidget(QLabel("Path:", self))
        self._path_edit = QLineEdit("/", self)
        self._path_edit.returnPressed.connect(self._go_to_path)
        path_row.addWidget(self._path_edit)
        go_btn = QPushButton("Go", self)
        go_btn.clicked.connect(self._go_to_path)
        path_row.addWidget(go_btn)
        layout.addLayout(path_row)

        self._list = QListWidget(self)
        self._list.itemDoubleClicked.connect(self._on_double_click)
        layout.addWidget(self._list)

        btn_row = QHBoxLayout()
        up_btn = QPushButton("↑  Up", self)
        up_btn.clicked.connect(self._go_up)
        btn_row.addWidget(up_btn)
        btn_row.addStretch()
        self._select_btn = QPushButton("Select this folder", self)
        self._select_btn.clicked.connect(self._select)
        btn_row.addWidget(self._select_btn)
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)

        self._status = QLabel("Loading…", self)
        layout.addWidget(self._status)

        self._load("")

    def _go_to_path(self):
        text = self._path_edit.text().strip()
        path = "" if text in ("/", "") else text
        self._load(path)

    def _load(self, path):
        self._list.clear()
        self._status.setText("Loading…")
        QApplication.processEvents()
        try:
            import dropbox as _dbx_mod
            entries = dropbox_helper.list_folder(self.dbx, path)
        except Exception as e:
            self._status.setText(f"Error: {e}")
            return

        self._current_path = path
        self._path_edit.setText(path or "/")

        folders = sorted(
            [e for e in entries if isinstance(e, _dbx_mod.files.FolderMetadata)],
            key=lambda e: e.name.lower())
        files = sorted(
            [e for e in entries if isinstance(e, _dbx_mod.files.FileMetadata)],
            key=lambda e: e.name.lower())

        # At root, also show shared folders not already in the listing
        if not path:
            existing_names = {e.name.lower() for e in folders}
            shared = dropbox_helper.list_shared_folders(self.dbx)
            extra = sorted(
                [s for s in shared if s.name.lower() not in existing_names],
                key=lambda s: s.name.lower())
            for s in extra:
                item = QListWidgetItem(f"📁  {s.name}  (shared)")
                item.setData(Qt.UserRole, ("folder", s.path_lower))
                self._list.addItem(item)

        for entry in folders:
            item = QListWidgetItem(f"📁  {entry.name}")
            item.setData(Qt.UserRole, ("folder", entry.path_display))
            self._list.addItem(item)
        for entry in files:
            icon = "🔵" if entry.name.endswith(".bin") else "📄"
            item = QListWidgetItem(f"{icon}  {entry.name}")
            item.setData(Qt.UserRole, ("file", entry.path_display, entry.size))
            self._list.addItem(item)

        bin_count = sum(1 for f in files if f.name.endswith(".bin"))
        self._status.setText(f"{len(folders)} subfolder(s), {bin_count} .bin file(s) in this folder")

    def _on_double_click(self, item):
        data = item.data(Qt.UserRole)
        if data[0] == "folder":
            self._load(data[1])

    def _go_up(self):
        if self._current_path:
            parent = self._current_path.rsplit("/", 1)[0]
            self._load(parent)

    def _select(self):
        import dropbox as _dbx_mod
        try:
            entries = dropbox_helper.list_folder(self.dbx, self._current_path)
        except Exception as e:
            self._status.setText(f"Error: {e}")
            return
        bin_files = sorted(
            [e for e in entries
             if isinstance(e, _dbx_mod.files.FileMetadata) and e.name.endswith(".bin")],
            key=lambda e: e.name)
        if not bin_files:
            self._status.setText("No .bin files found here — please choose another folder.")
            return
        self.selected_path = self._current_path
        self.selected_bin_files = bin_files
        self.accept()


_CHUNKED_META_FILE = '_beespy_chunked_meta.json'


def _safe_to_delete(path, local_bin_folder):
    """Return True only if it is safe to delete `path`.

    Never returns True for a .bin file that lives inside the user's locally-
    selected source folder.  Dropbox-downloaded files have local_bin_folder=None
    (they are in-memory BytesIO objects anyway, so nothing to delete on disk).
    """
    if local_bin_folder is None:
        return True   # Dropbox mode — no local source to protect
    real_path   = os.path.realpath(path)
    real_source = os.path.realpath(local_bin_folder)
    if real_path.startswith(real_source + os.sep) and real_path.lower().endswith('.bin'):
        print(f"  [SAFETY] Skipping deletion of local bin file: {path}")
        return False
    return True


def process_all_chunks(folder, bin_files, start_time_qdt, end_time_qdt,
                       chunk_hours, sampFreq, defaultWindows, calcWindows,
                       minFreq, maxFreq, dbx, dbx_folder, agg,
                       local_bin_folder=None):
    """Process a date range in fixed-size chunks, saving date-stamped CSVs.

    Each completed chunk is written as 6 CSV files named:
        YYYYMMDD_HHMMSS_YYYYMMDD_HHMMSS_ch1.csv … ch6.csv

    If all 6 CSVs already exist for a chunk, that chunk is skipped (acts as
    the chunk-level checkpoint).  Within each chunk, the existing per-file
    checkpoint system is used.

    Returns a list of (chunk_start_dt, chunk_end_dt) covering the full range.
    """
    # Convert QDateTime boundaries to Python datetime (UTC)
    start_epoch = start_time_qdt.toSecsSinceEpoch() + start_time_qdt.offsetFromUtc()
    end_epoch   = end_time_qdt.toSecsSinceEpoch()   + end_time_qdt.offsetFromUtc()
    start_dt = datetime.datetime.utcfromtimestamp(start_epoch)
    end_dt   = datetime.datetime.utcfromtimestamp(end_epoch)

    delta = datetime.timedelta(hours=chunk_hours)
    chunk_list = []
    current = start_dt
    while current < end_dt:
        chunk_end = min(current + delta, end_dt)
        chunk_list.append((current, chunk_end))
        current = chunk_end

    n_chunks = len(chunk_list)
    print(f"Chunked mode: {n_chunks} chunk(s) of {chunk_hours}h each")

    # Write a meta file so _continue_session can detect an in-progress chunked run
    chunked_meta_path = os.path.join(folder, _CHUNKED_META_FILE)
    chunked_meta = {
        'chunk_hours':    chunk_hours,
        'start_epoch':    start_epoch,
        'end_epoch':      end_epoch,
        'sampFreq':       sampFreq,
        'defaultWindows': defaultWindows,
        'calcWindows':    calcWindows,
        'minFreq':        minFreq,
        'maxFreq':        maxFreq,
        'agg':            agg,
        'source':         str(dbx_folder or folder),
    }
    with open(chunked_meta_path, 'w') as _f:
        json.dump(chunked_meta, _f, indent=2)

    for i, (chunk_start, chunk_end) in enumerate(chunk_list, start=1):
        if _chunk_is_complete(folder, chunk_start, chunk_end):
            print(f"  Chunk {i}/{n_chunks} [{chunk_start} → {chunk_end}]: already complete — skipping")
            continue

        print(f"  Chunk {i}/{n_chunks} [{chunk_start} → {chunk_end}]: processing…")

        # Build QDateTime equivalents for this chunk
        from PyQt5.QtCore import QDateTime as _QDT
        def _dt_to_qdt(dt):
            return _QDT(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)

        chunk_start_qdt = _dt_to_qdt(chunk_start)
        chunk_end_qdt   = _dt_to_qdt(chunk_end)

        chk_dir = os.path.join(
            folder,
            _CHK_SUBDIR,
            f'chunk_{chunk_start.strftime("%Y%m%d_%H%M%S")}')
        chk_key = _checkpoint_settings_key(
            chunk_start_qdt, chunk_end_qdt,
            sampFreq, defaultWindows, calcWindows, minFreq, maxFreq, agg,
            dbx_folder or folder)

        # Check for a within-chunk checkpoint to resume
        resume_files = None
        chk_meta_path = os.path.join(chk_dir, 'meta.json')
        if os.path.isfile(chk_meta_path):
            try:
                with open(chk_meta_path) as _f:
                    _meta = json.load(_f)
                if _checkpoint_meta_matches(_meta, chk_key):
                    resume_files = set(_meta.get('processed_files', []))
                    print(f"    [checkpoint] Resuming chunk ({len(resume_files)} file(s) already done)")
                else:
                    shutil.rmtree(chk_dir, ignore_errors=True)
            except Exception:
                shutil.rmtree(chk_dir, ignore_errors=True)

        specs = process_bin_files(
            folder, bin_files,
            chunk_start_qdt, chunk_end_qdt,
            sampFreq, defaultWindows, calcWindows, minFreq, maxFreq,
            dbx=dbx, dbx_folder=dbx_folder, agg=agg,
            checkpoint_dir=chk_dir, checkpoint_key=chk_key,
            resume_files=resume_files,
            local_bin_folder=local_bin_folder)

        write_chunked_spectrograms(folder, specs, chunk_start, chunk_end)
        print(f"  Chunk {i}/{n_chunks}: written.")

    # All chunks complete — remove the meta file
    if os.path.isfile(chunked_meta_path):
        os.remove(chunked_meta_path)
        print("  [chunked checkpoint] All chunks complete — meta file removed")

    return chunk_list


class _ChunkedProcessingThread(QThread):
    """Runs process_all_chunks on a background thread."""
    finished = pyqtSignal(object)   # emits list of (start_dt, end_dt) tuples
    error    = pyqtSignal(str)

    def __init__(self, folder, bin_files, start_time, end_time,
                 chunk_hours, sampFreq, defaultWindows, calcWindows,
                 minFreq, maxFreq, dbx, dbx_folder, agg,
                 local_bin_folder=None, parent=None):
        super().__init__(parent)
        self._folder          = folder
        self._bin_files       = bin_files
        self._start_time      = start_time
        self._end_time        = end_time
        self._chunk_hours     = chunk_hours
        self._sampFreq        = sampFreq
        self._defaultWindows  = defaultWindows
        self._calcWindows     = calcWindows
        self._minFreq         = minFreq
        self._maxFreq         = maxFreq
        self._dbx             = dbx
        self._dbx_folder      = dbx_folder
        self._agg             = agg
        self._local_bin_folder = local_bin_folder

    def run(self):
        try:
            result = process_all_chunks(
                self._folder, self._bin_files,
                self._start_time, self._end_time,
                self._chunk_hours,
                self._sampFreq, self._defaultWindows, self._calcWindows,
                self._minFreq, self._maxFreq,
                self._dbx, self._dbx_folder, self._agg,
                local_bin_folder=self._local_bin_folder)
            self.finished.emit(result)
        except Exception as e:
            self.error.emit(str(e))


class _ChunkDateRangeDialog(QDialog):
    """Small dialog to let the user pick start/end dates for loading chunked spectrograms."""

    def __init__(self, min_dt, max_dt, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select date range for chunked spectrograms")
        self.setMinimumWidth(400)
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Select the date range to load and concatenate:"))

        from PyQt5.QtCore import QDateTime as _QDT
        def _dt_to_qdt(dt):
            return _QDT(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)

        row_start = QHBoxLayout()
        row_start.addWidget(QLabel("Start:"))
        self._start = QDateTimeEdit(self)
        self._start.setCalendarPopup(True)
        self._start.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self._start.setDateTime(_dt_to_qdt(min_dt))
        row_start.addWidget(self._start)
        layout.addLayout(row_start)

        row_end = QHBoxLayout()
        row_end.addWidget(QLabel("End:  "))
        self._end = QDateTimeEdit(self)
        self._end.setCalendarPopup(True)
        self._end.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self._end.setDateTime(_dt_to_qdt(max_dt))
        row_end.addWidget(self._end)
        layout.addLayout(row_end)

        btn_row = QHBoxLayout()
        ok_btn = QPushButton("Load", self)
        ok_btn.clicked.connect(self.accept)
        btn_row.addWidget(ok_btn)
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)

    def selected_range(self):
        """Return (start_dt, end_dt) as Python datetime objects."""
        s = self._start.dateTime()
        e = self._end.dateTime()
        return (
            datetime.datetime(s.date().year(), s.date().month(), s.date().day(),
                              s.time().hour(), s.time().minute(), s.time().second()),
            datetime.datetime(e.date().year(), e.date().month(), e.date().day(),
                              e.time().hour(), e.time().minute(), e.time().second()),
        )


class _BatchDayDateRangeDialog(QDialog):
    """Small dialog to let the user pick a date range for loading batch-pipeline spectrograms.

    Scans the chosen folder for files matching YYYYMMDD_ch1.csv and populates
    min/max dates automatically.
    """

    def __init__(self, min_date, max_date, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select date range for batch spectrograms")
        self.setMinimumWidth(400)
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Select the date range to load:"))

        def _date_to_qdt(d):
            return QDateTime(d.year, d.month, d.day, 0, 0, 0)

        row_start = QHBoxLayout()
        row_start.addWidget(QLabel("Start:"))
        self._start = QDateTimeEdit(self)
        self._start.setCalendarPopup(True)
        self._start.setDisplayFormat("dd-MMM-yyyy")
        self._start.setDateTime(_date_to_qdt(min_date))
        row_start.addWidget(self._start)
        layout.addLayout(row_start)

        row_end = QHBoxLayout()
        row_end.addWidget(QLabel("End:  "))
        self._end = QDateTimeEdit(self)
        self._end.setCalendarPopup(True)
        self._end.setDisplayFormat("dd-MMM-yyyy")
        self._end.setDateTime(_date_to_qdt(max_date))
        row_end.addWidget(self._end)
        layout.addLayout(row_end)

        btn_row = QHBoxLayout()
        ok_btn = QPushButton("Load", self)
        ok_btn.clicked.connect(self.accept)
        btn_row.addWidget(ok_btn)
        cancel_btn = QPushButton("Cancel", self)
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)

    def selected_range(self):
        """Return (start_date, end_date) as Python date objects."""
        s = self._start.dateTime().date()
        e = self._end.dateTime().date()
        return (
            datetime.date(s.year(), s.month(), s.day()),
            datetime.date(e.year(), e.month(), e.day()),
        )


class _ProcessingThread(QThread):
    """Runs process_bin_files on a background thread so the Qt event loop
    (and therefore the log widget) stays responsive throughout."""
    finished = pyqtSignal(object)   # emits the spectrograms list on success
    error    = pyqtSignal(str)      # emits an error message on failure

    def __init__(self, folder, bin_files, start_time, end_time,
                 sampFreq, defaultWindows, calcWindows, minFreq, maxFreq,
                 dbx, dbx_folder, agg="Average",
                 checkpoint_dir=None, checkpoint_key=None, resume_files=None,
                 local_bin_folder=None, parent=None):
        super().__init__(parent)
        self._args   = (folder, bin_files, start_time, end_time,
                        sampFreq, defaultWindows, calcWindows, minFreq, maxFreq)
        self._kwargs = dict(dbx=dbx, dbx_folder=dbx_folder, agg=agg,
                            checkpoint_dir=checkpoint_dir,
                            checkpoint_key=checkpoint_key,
                            resume_files=resume_files,
                            local_bin_folder=local_bin_folder)

    def run(self):
        try:
            result = process_bin_files(*self._args, **self._kwargs)
            self.finished.emit(result)
        except Exception as e:
            self.error.emit(str(e))


class SpectrogramApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BeeSpy ESF visualiser v15.0")
        self.setGeometry(100, 100, 800, 100)
        
        self.folder = None
        self.bin_files = []
        self.spectrograms = []
        self.file_size_map = {}    # {filename: size_bytes} — populated by both local and Dropbox paths
        self.dbx = None            # Dropbox client (None = local mode)
        self.dbx_folder = None     # Dropbox folder path used as input
        # When the user picks a local folder with bin files, this is set to that
        # folder path so that processing code knows never to delete those files.
        # It is None in Dropbox mode (downloaded copies may be cleaned up).
        self._local_bin_folder = None

        # Zoom state (None = no zoom applied on that axis)
        self._zoom_t_start = None
        self._zoom_t_end = None
        self._zoom_f_min = None
        self._zoom_f_max = None
        self._current_view = None  # ('channel', idx) | ('all',) | ('average',)

        # Main layout
        self.layout = QVBoxLayout()

        ############################
        # Entry-point buttons — two columns (Select | Load) + Continue centred below
        self.folder_button = QPushButton("Select Folder with .bin files", self)
        self.folder_button.clicked.connect(self.select_folder)

        self.load_csv_button = QPushButton("Load existing spectrograms", self)
        self.load_csv_button.clicked.connect(self.load_existing_spectrograms)

        self.load_chunked_button = QPushButton("Load chunked spectrograms", self)
        self.load_chunked_button.clicked.connect(self.load_chunked_spectrograms)

        self.load_batch_button = QPushButton("Load batch spectrograms", self)
        self.load_batch_button.clicked.connect(self.load_batch_spectrograms)

        self.dropbox_button = QPushButton("Select Dropbox Folder", self)
        self.dropbox_button.clicked.connect(self._connect_dropbox)

        self.continue_button = QPushButton("Continue session", self)
        self.continue_button.clicked.connect(self._continue_session)

        # Two-column row: Select buttons on the left, Load buttons on the right
        _col_row = QHBoxLayout()
        _select_col = QVBoxLayout()
        _select_col.addWidget(self.folder_button)
        _select_col.addWidget(self.dropbox_button)
        _load_col = QVBoxLayout()
        _load_col.addWidget(self.load_csv_button)
        _load_col.addWidget(self.load_chunked_button)
        _load_col.addWidget(self.load_batch_button)
        _col_row.addLayout(_select_col)
        _col_row.addLayout(_load_col)
        self.layout.addLayout(_col_row)

        # Continue session centred below the two columns
        _continue_row = QHBoxLayout()
        _continue_row.addStretch()
        _continue_row.addWidget(self.continue_button)
        _continue_row.addStretch()
        self.layout.addLayout(_continue_row)

        # Summary label (to show the files and date range)
        self.summary_label = QLabel("", self)
        self.layout.addWidget(self.summary_label)
        self.diff_folder_button = QPushButton("Start again", self)
        self.diff_folder_button.clicked.connect(self.select_diff_folder)
        self.layout.addWidget(self.diff_folder_button)
        self.diff_folder_button.hide()


        ############################
        # QDateTimeEdit widgets for start and end dates
        self.start_date_edit = QDateTimeEdit(self)
        self.start_date_edit.setCalendarPopup(True)
        self.start_date_edit.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self.start_date_edit.dateTimeChanged.connect(self.updateOutputSummary)

        self.end_date_edit = QDateTimeEdit(self)
        self.end_date_edit.setCalendarPopup(True)
        self.end_date_edit.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self.end_date_edit.dateTimeChanged.connect(self.updateOutputSummary)


        # Labels for the dates
        self.startTimeLabel = QLabel("Start Date:")
        self.layout.addWidget(self.startTimeLabel)
        self.layout.addWidget(self.start_date_edit)
        self.endTimeLabel = QLabel("End Date:")
        self.layout.addWidget(self.endTimeLabel)
        self.layout.addWidget(self.end_date_edit)

        # Chunk size field (0 = process as single file, >0 = chunked mode)
        self.chunkSizeHours = QThelpers.LabelledIntField('Chunk size (hours, 0 = single file):', 0)
        self.chunkSizeHours.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.chunkSizeHours)

        # Hide date selectors until files are selected
        self.startTimeLabel.hide()
        self.start_date_edit.hide()
        self.endTimeLabel.hide()
        self.end_date_edit.hide()
        self.chunkSizeHours.hide()



        ############################
        # "Edit details" button (and settings; initially hidden)
        self.sampFreq = QThelpers.LabelledIntField('Sampling Freq (Hz):', 5000)
        self.sampFreq.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.sampFreq)
        self.sampFreq.hide()

        self.defaultWindows = QThelpers.LabelledDoubleField('Default window (s):', 0.2)
        self.defaultWindows.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.defaultWindows)
        self.defaultWindows.hide()

        self.calcWindows = QThelpers.LabelledDoubleField('Output window (s):', 0.2)
        self.calcWindows.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.calcWindows)
        self.calcWindows.hide()

        self.minFreq = QThelpers.LabelledIntField('Min freq (Hz):', 0)
        self.minFreq.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.minFreq)
        self.minFreq.hide()

        self.maxFreq = QThelpers.LabelledIntField('Max freq (Hz):', 500)
        self.maxFreq.on_value_changed(self.updateOutputSummary)
        self.layout.addWidget(self.maxFreq)
        self.maxFreq.hide()

        # Aggregation method selector (hidden with other settings)
        self._agg_row = QWidget(self)
        _agg_layout = QHBoxLayout(self._agg_row)
        _agg_layout.setContentsMargins(0, 0, 0, 0)
        _agg_layout.addWidget(QLabel("Aggregation method:"))
        self.aggMethod = QComboBox(self)
        self.aggMethod.addItems(["Average", "Maximum", "75th percentile", "90th percentile", "95th percentile"])
        _agg_layout.addWidget(self.aggMethod)
        self.layout.addWidget(self._agg_row)
        self._agg_row.hide()

        self.detail_settings_button = QPushButton("Edit settings", self)
        self.detail_settings_button.clicked.connect(self.detail_settings)
        self.layout.addWidget(self.detail_settings_button)
        self.detail_settings_button.hide()
        

        self.outputSummaryLabel = QLabel("", self)
        self.layout.addWidget(self.outputSummaryLabel)
        self.outputSummaryLabel.hide()

        ############################
        # "Make Graphs" button (initially hidden)
        self.make_graphs_button = QPushButton("Make Graphs", self)
        self.make_graphs_button.clicked.connect(self.make_graphs)
        self.make_graphs_button.hide()
        self.layout.addWidget(self.make_graphs_button)

        ############################
        # Spectrogram plot area (canvas)
        self.plot_canvas = PlotCanvas(self)
        self.layout.addWidget(self.plot_canvas)
        self.plot_canvas.hide()
       
        # Buttons to toggle between individual channels or all channels
        self.toggle_buttons_layout = QHBoxLayout()
        self.toggle_buttons = []
        for i in range(6):
            btn = QPushButton(f"Channel {i + 1}", self)
            btn.clicked.connect(lambda _, idx=i: self.plot_channel(idx))
            self.toggle_buttons.append(btn)
            self.toggle_buttons_layout.addWidget(btn)

        # Add "Show All" button to show all channels
        self.show_all_button = QPushButton("Show All Channels", self)
        self.show_all_button.clicked.connect(self.show_all_channels)
        self.toggle_buttons_layout.addWidget(self.show_all_button)

        # Save current view as JPEG
        self.save_jpeg_button = QPushButton("Save as JPEG", self)
        self.save_jpeg_button.clicked.connect(self.save_as_jpeg)
        self.save_jpeg_button.hide()
        self.toggle_buttons_layout.addWidget(self.save_jpeg_button)

        # Add the buttons to the layout
        self.layout.addLayout(self.toggle_buttons_layout)

        # ── Average channels row ─────────────────────────────────────────────
        self.avg_row_layout = QHBoxLayout()
        self.avg_row_layout.setSpacing(6)

        self.avg_ch_label = QLabel("Average:", self)
        self.avg_row_layout.addWidget(self.avg_ch_label)

        self.avg_checkboxes = []
        for i in range(6):
            cb = QCheckBox(f"Ch {i+1}", self)
            cb.setChecked(True)
            self.avg_checkboxes.append(cb)
            self.avg_row_layout.addWidget(cb)

        self.avg_button = QPushButton("Show Average", self)
        self.avg_button.clicked.connect(self.show_average_channels)
        self.avg_row_layout.addWidget(self.avg_button)

        self.export_avg_csv_button = QPushButton("Export Average CSV", self)
        self.export_avg_csv_button.clicked.connect(self.export_average_csv)
        self.avg_row_layout.addWidget(self.export_avg_csv_button)

        self.avg_row_layout.addStretch()
        self.layout.addLayout(self.avg_row_layout)
        self._avg_row_widgets = [self.avg_ch_label, self.avg_button,
                                  self.export_avg_csv_button] + self.avg_checkboxes

        # ── Zoom row ─────────────────────────────────────────────────────────
        self.zoom_row_layout = QHBoxLayout()
        self.zoom_row_layout.setSpacing(6)

        self.zoom_label = QLabel("Zoom:", self)
        self.zoom_row_layout.addWidget(self.zoom_label)

        self.zoom_start_dt = QDateTimeEdit(self)
        self.zoom_start_dt.setCalendarPopup(True)
        self.zoom_start_dt.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self.zoom_row_layout.addWidget(self.zoom_start_dt)

        self.zoom_to_label = QLabel("to", self)
        self.zoom_row_layout.addWidget(self.zoom_to_label)

        self.zoom_end_dt = QDateTimeEdit(self)
        self.zoom_end_dt.setCalendarPopup(True)
        self.zoom_end_dt.setDisplayFormat("dd-MMM-yyyy HH:mm:ss")
        self.zoom_row_layout.addWidget(self.zoom_end_dt)

        self.zoom_freq_label = QLabel("  Freq (Hz):", self)
        self.zoom_row_layout.addWidget(self.zoom_freq_label)

        self.zoom_freq_min_input = QLineEdit(self)
        self.zoom_freq_min_input.setFixedWidth(70)
        self.zoom_freq_min_input.setPlaceholderText("min")
        self.zoom_row_layout.addWidget(self.zoom_freq_min_input)

        self.zoom_dash_label = QLabel("\u2013", self)
        self.zoom_row_layout.addWidget(self.zoom_dash_label)

        self.zoom_freq_max_input = QLineEdit(self)
        self.zoom_freq_max_input.setFixedWidth(70)
        self.zoom_freq_max_input.setPlaceholderText("max")
        self.zoom_row_layout.addWidget(self.zoom_freq_max_input)

        self.zoom_apply_button = QPushButton("Apply Zoom", self)
        self.zoom_apply_button.clicked.connect(self.apply_zoom)
        self.zoom_row_layout.addWidget(self.zoom_apply_button)

        self.zoom_reset_button = QPushButton("Reset Zoom", self)
        self.zoom_reset_button.clicked.connect(self.reset_zoom)
        self.zoom_row_layout.addWidget(self.zoom_reset_button)

        self.zoom_row_layout.addStretch()
        self.layout.addLayout(self.zoom_row_layout)
        self._zoom_row_widgets = [
            self.zoom_label, self.zoom_start_dt, self.zoom_to_label,
            self.zoom_end_dt, self.zoom_freq_label, self.zoom_freq_min_input,
            self.zoom_dash_label, self.zoom_freq_max_input,
            self.zoom_apply_button, self.zoom_reset_button,
        ]

        # Frequency band summary export row (hidden until spectrograms are loaded)
        self.freq_export_layout = QHBoxLayout()
        self.freq_bands_label = QLabel("Freq bands for summary (e.g. 10-50 100-200 300-500):")
        self.freq_bands_input = QLineEdit("10-50 100-200 300-500")
        self.export_band_button = QPushButton("Export Band Summary")
        self.export_band_button.clicked.connect(self.export_band_summary_clicked)
        self.freq_export_layout.addWidget(self.freq_bands_label)
        self.freq_export_layout.addWidget(self.freq_bands_input, stretch=1)
        self.freq_export_layout.addWidget(self.export_band_button)
        self.layout.addLayout(self.freq_export_layout)
        self._freq_export_widgets = [self.freq_bands_label, self.freq_bands_input, self.export_band_button]

        # Initially hide all the toggle buttons
        self.toggle_buttons_visibility(False)

        # Footer log widget — shows progress messages instead of printing to terminal
        self.log_widget = QPlainTextEdit(self)
        self.log_widget.setReadOnly(True)
        self.log_widget.setMinimumHeight(30)
        self.log_widget.setPlaceholderText("Progress log…")

        # Redirect stdout so every print() goes to the footer
        self._log_emitter = _LogEmitter()
        self._log_emitter.message.connect(self._append_log)
        sys.stdout = _StdoutRedirector(self._log_emitter)

        # Wrap the main content in a container widget so it can go into a splitter
        _top_container = QWidget()
        _top_container.setLayout(self.layout)

        # Vertical splitter: main content on top, log on bottom — both user-resizable
        self._splitter = QSplitter(Qt.Vertical)
        self._splitter.addWidget(_top_container)
        self._splitter.addWidget(self.log_widget)
        self._splitter.setStretchFactor(0, 1)   # top section takes all extra space
        self._splitter.setStretchFactor(1, 0)
        self._splitter.setSizes([800, 80])       # default: log starts small

        outer_layout = QVBoxLayout()
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(self._splitter)
        self.setLayout(outer_layout)

    def _append_log(self, msg: str):
        self.log_widget.appendPlainText(msg)
        sb = self.log_widget.verticalScrollBar()
        sb.setValue(sb.maximum())

    def detail_settings(self):
        if(self.detail_settings_button.text() == "Edit settings"):
            # Show the settings and change the button text
            self.detail_settings_button.setText("Hide settings")
            self.sampFreq.show() # TODO: MOVE THE SAMP FREQ AND DEFAULT WINDOW SIZE TO AN 'ADMIN' TAB.
            self.defaultWindows.show()
            self.calcWindows.show()
            self.minFreq.show()
            self.maxFreq.show()
            self._agg_row.show()
            # TODO: ADD IN BUTTONS TO SELECT CHANNELS FOR ANALYSIS (default to 'all')
        else:
            # Hide the settings and change the button text
            self.detail_settings_button.setText("Edit settings")
            self.sampFreq.hide()
            self.defaultWindows.hide()
            self.calcWindows.hide()
            self.minFreq.hide()
            self.maxFreq.hide()
            self._agg_row.hide()



    # Folder selection
    def select_diff_folder(self):
        # Clear everything, including the local-bin-folder guard
        self._local_bin_folder = None
        self.summary_label.setText("")
        self.spectrograms = []

        # Hide date selectors until files are selected
        self.startTimeLabel.hide()
        self.start_date_edit.hide()
        self.endTimeLabel.hide()
        self.end_date_edit.hide()
        self.chunkSizeHours.hide()

        # Hide the plots and buttons
        self.make_graphs_button.hide()
        self.plot_canvas.hide()
        self.toggle_buttons_visibility(False)
        self.diff_folder_button.hide()

        # Hide settings
        self.detail_settings_button.hide()
        self.outputSummaryLabel.hide()

        # Reset Dropbox state
        self.dbx_folder = None
        self.file_size_map = {}

        # Restore all entry-point buttons
        self.folder_button.show()
        self.load_csv_button.show()
        self.load_chunked_button.show()
        self.load_batch_button.show()
        self.dropbox_button.show()
        self.continue_button.show()

        # Resize the window
        self.setGeometry(100, 100, 800, 100)
        QApplication.processEvents()



    def select_folder(self):
        self.folder = QFileDialog.getExistingDirectory(self, "Select folder with the .bin files")
        if self.folder:    
            self.bin_files = sorted([f for f in os.listdir(self.folder) if f.endswith(".bin")])

            if self.bin_files:
                self.dbx = None
                self.dbx_folder = None
                # Mark this as a local-origin session — bin files must never be deleted
                self._local_bin_folder = self.folder
                self.file_size_map = {
                    f: os.path.getsize(os.path.join(self.folder, f))
                    for f in self.bin_files
                }
                self.summary_label.setText(f"Folder selected: {self.folder}\n{len(self.bin_files)} bin files found.")

                ## hide the entry buttons
                self.folder_button.hide()
                self.load_csv_button.hide()
                self.load_chunked_button.hide()
                self.load_batch_button.hide()
                self.dropbox_button.hide()
                self.continue_button.hide()
                self.diff_folder_button.show()

                # Show date range selectors and make graphs button
                self.startTimeLabel.show()
                self.start_date_edit.show()
                self.endTimeLabel.show()
                self.end_date_edit.show()
                self.chunkSizeHours.show()
                self.make_graphs_button.show()

                # show the settings button
                self.detail_settings_button.show()
                
                # Set default date range (based on file modification times)
                self.set_default_date_range()

                ## update the output summary and show
                self.updateOutputSummary()
                self.outputSummaryLabel.show()

            else:
                self.summary_label.setText(f"No bin files found in the selected folder. Please try another folder")

    # ── Continue session entry-point ─────────────────────────────────────────

    def _continue_session(self):
        """Restore a previous run from a saved checkpoint and resume processing."""
        folder = QFileDialog.getExistingDirectory(
            self, "Select the output folder that contains a checkpoint")
        if not folder:
            return

        # ── Check for an in-progress chunked run first ────────────────────────
        chunked_meta_path = os.path.join(folder, _CHUNKED_META_FILE)
        if os.path.isfile(chunked_meta_path):
            try:
                with open(chunked_meta_path) as _f:
                    chunked_meta = json.load(_f)
            except Exception as e:
                QMessageBox.critical(self, "Error reading chunked checkpoint", str(e))
                return

            source = chunked_meta.get('source', '')

            # Collect bin files
            if os.path.isdir(source):
                bin_files = sorted(f for f in os.listdir(source) if f.endswith('.bin'))
                if not bin_files:
                    QMessageBox.warning(self, "No .bin files",
                                        f"No .bin files found in:\n{source}")
                    return
                self.folder      = folder
                self.dbx         = None
                self.dbx_folder  = None
                self.bin_files   = bin_files
                self.file_size_map = {f: os.path.getsize(os.path.join(source, f))
                                      for f in bin_files}
            else:
                # Dropbox mode
                self.folder     = folder
                self.dbx_folder = source
                dbx = dropbox_helper.get_saved_client()
                if dbx is None:
                    QMessageBox.warning(self, "Dropbox reconnection needed",
                                        "Please reconnect Dropbox via 'Select Dropbox Folder'.")
                    return
                self.dbx = dropbox_helper.scope_to_root_namespace(dbx)
                try:
                    import dropbox as _dbx_mod
                    entries = dropbox_helper.list_folder(self.dbx, source)
                    bin_entries = sorted(
                        [e for e in entries
                         if isinstance(e, _dbx_mod.files.FileMetadata)
                         and e.name.endswith('.bin')],
                        key=lambda e: e.name)
                except Exception as e:
                    QMessageBox.critical(self, "Dropbox error", str(e))
                    return
                self.bin_files     = [e.name for e in bin_entries]
                self.file_size_map = {e.name: e.size for e in bin_entries}

            # Restore settings
            from PyQt5.QtCore import QDateTime as _QDT
            self.sampFreq.lineEdit.setText(str(int(chunked_meta['sampFreq'])))
            self.defaultWindows.lineEdit.setText(str(chunked_meta['defaultWindows']))
            self.calcWindows.lineEdit.setText(str(chunked_meta['calcWindows']))
            self.minFreq.lineEdit.setText(str(chunked_meta['minFreq']))
            self.maxFreq.lineEdit.setText(str(chunked_meta['maxFreq']))
            self.aggMethod.setCurrentText(chunked_meta.get('agg', 'Average'))
            self.chunkSizeHours.lineEdit.setText(str(chunked_meta['chunk_hours']))

            _MIN = _QDT(2000, 1, 1, 0, 0, 0)
            _MAX = _QDT(2100, 1, 1, 0, 0, 0)
            for edit in (self.start_date_edit, self.end_date_edit):
                edit.setMinimumDateTime(_MIN)
                edit.setMaximumDateTime(_MAX)

            def _epoch_to_qdt(epoch):
                d = datetime.datetime.utcfromtimestamp(epoch)
                return _QDT(d.year, d.month, d.day, d.hour, d.minute, d.second)

            self.start_date_edit.setDateTime(_epoch_to_qdt(chunked_meta['start_epoch']))
            self.end_date_edit.setDateTime(_epoch_to_qdt(chunked_meta['end_epoch']))

            self.summary_label.setText(
                f"Resuming chunked run\n"
                f"Source: {source}\n"
                f"Chunk size: {chunked_meta['chunk_hours']}h")

            self.folder_button.hide()
            self.load_csv_button.hide()
            self.load_chunked_button.hide()
            self.load_batch_button.hide()
            self.dropbox_button.hide()
            self.continue_button.hide()
            self.diff_folder_button.show()
            self.startTimeLabel.show()
            self.start_date_edit.show()
            self.endTimeLabel.show()
            self.end_date_edit.show()
            self.chunkSizeHours.show()
            self.make_graphs_button.show()
            self.detail_settings_button.show()
            self.updateOutputSummary()
            self.outputSummaryLabel.show()
            return

        # ── Fall back to original single-file checkpoint ──────────────────────
        chk_meta_path = os.path.join(folder, _CHK_SUBDIR, 'meta.json')
        if not os.path.isfile(chk_meta_path):
            QMessageBox.warning(self, "No checkpoint found",
                                f"No checkpoint was found in:\n{folder}\n\n"
                                f"(Looking for: {os.path.join(_CHK_SUBDIR, 'meta.json')} "
                                f"or {_CHUNKED_META_FILE})")
            return

        try:
            with open(chk_meta_path) as f:
                meta = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Error reading checkpoint", str(e))
            return

        source   = meta.get('source', '')
        n_done   = len(meta.get('processed_files', []))
        saved_at = meta.get('saved_at', 'unknown')

        # ── Collect bin files (local or Dropbox) ─────────────────────────────
        if os.path.isdir(source):
            # Local mode: source is the folder containing .bin files
            bin_files = sorted(f for f in os.listdir(source) if f.endswith('.bin'))
            if not bin_files:
                QMessageBox.warning(self, "No .bin files",
                                    f"No .bin files found in:\n{source}")
                return
            self.folder      = source
            self.dbx         = None
            self.dbx_folder  = None
            self.bin_files   = bin_files
            self.file_size_map = {f: os.path.getsize(os.path.join(source, f))
                                  for f in bin_files}
        else:
            # Dropbox mode: source is a Dropbox path; output lives in `folder`
            self.folder     = folder
            self.dbx_folder = source
            dbx = dropbox_helper.get_saved_client()
            if dbx is None:
                QMessageBox.warning(
                    self, "Dropbox reconnection needed",
                    "This checkpoint was created from a Dropbox folder.\n\n"
                    "Please use 'Select Dropbox Folder' to reconnect — the "
                    "checkpoint will be offered automatically when you click "
                    "'Make Graphs'.")
                return
            self.dbx = dropbox_helper.scope_to_root_namespace(dbx)
            self.summary_label.setText("Reconnecting to Dropbox and listing files…")
            QApplication.processEvents()
            try:
                import dropbox as _dbx_mod
                entries = dropbox_helper.list_folder(self.dbx, source)
                bin_entries = sorted(
                    [e for e in entries
                     if isinstance(e, _dbx_mod.files.FileMetadata)
                     and e.name.endswith('.bin')],
                    key=lambda e: e.name)
            except Exception as e:
                QMessageBox.critical(self, "Dropbox error", str(e))
                return
            self.bin_files     = [e.name for e in bin_entries]
            self.file_size_map = {e.name: e.size for e in bin_entries}

        # ── Restore settings into UI fields ──────────────────────────────────
        self.sampFreq.lineEdit.setText(str(int(meta['sampFreq'])))
        self.defaultWindows.lineEdit.setText(str(meta['defaultWindows']))
        self.calcWindows.lineEdit.setText(str(meta['calcWindows']))
        self.minFreq.lineEdit.setText(str(meta['minFreq']))
        self.maxFreq.lineEdit.setText(str(meta['maxFreq']))
        self.aggMethod.setCurrentText(meta.get('agg', 'Average'))

        # Reconstruct QDateTime from stored epoch values.
        # start_epoch = toSecsSinceEpoch() + offsetFromUtc(), so
        # utcfromtimestamp(start_epoch) recovers the original local datetime.
        from PyQt5.QtCore import QDateTime as _QDT
        _MIN = _QDT(2000, 1, 1, 0, 0, 0)
        _MAX = _QDT(2100, 1, 1, 0, 0, 0)
        for edit in (self.start_date_edit, self.end_date_edit):
            edit.setMinimumDateTime(_MIN)
            edit.setMaximumDateTime(_MAX)

        def _epoch_to_qdt(epoch):
            d = datetime.datetime.utcfromtimestamp(epoch)
            return _QDT(d.year, d.month, d.day, d.hour, d.minute, d.second)

        self.start_date_edit.setDateTime(_epoch_to_qdt(meta['start_epoch']))
        self.end_date_edit.setDateTime(_epoch_to_qdt(meta['end_epoch']))

        # ── Show the normal processing UI ─────────────────────────────────────
        n_total = len(self.bin_files)
        self.summary_label.setText(
            f"Resuming checkpoint\n"
            f"Source: {source}\n"
            f"Progress saved: {n_done} of {n_total} file(s) | saved at {saved_at}")

        self.folder_button.hide()
        self.load_csv_button.hide()
        self.load_chunked_button.hide()
        self.load_batch_button.hide()
        self.dropbox_button.hide()
        self.continue_button.hide()
        self.diff_folder_button.show()
        self.startTimeLabel.show()
        self.start_date_edit.show()
        self.endTimeLabel.show()
        self.end_date_edit.show()
        self.chunkSizeHours.show()
        self.make_graphs_button.show()
        self.detail_settings_button.show()
        self.updateOutputSummary()
        self.outputSummaryLabel.show()

    # ── Dropbox entry-point ──────────────────────────────────────────────────

    def _connect_dropbox(self):
        """Authenticate with Dropbox (reusing saved tokens when available), then browse."""
        try:
            import dropbox  # noqa – just to confirm the package is present
        except ImportError:
            QMessageBox.critical(self, "Dropbox not installed",
                                 "Please install the Dropbox SDK:\n\n    pip install dropbox")
            return

        # Re-use saved tokens if they still work
        dbx = dropbox_helper.get_saved_client()
        if dbx is None:
            dlg = DropboxAuthDialog(self)
            if dlg.exec_() != QDialog.Accepted:
                return
            dbx = dlg.dbx

        self.dbx = dropbox_helper.scope_to_root_namespace(dbx)
        self._select_dropbox_folder()

    def _select_dropbox_folder(self):
        """Open the folder browser, then set up the UI exactly like select_folder() does."""
        browser = DropboxFolderBrowser(self.dbx, self)
        if browser.exec_() != QDialog.Accepted:
            return

        self.dbx_folder = browser.selected_path
        bin_entries = browser.selected_bin_files
        self.bin_files = [e.name for e in bin_entries]
        self.file_size_map = {e.name: e.size for e in bin_entries}
        # Dropbox mode — files are downloaded on-demand; no local bin folder to protect
        self._local_bin_folder = None

        # Ask for a local folder to save output CSVs/JPEGs
        out_folder = QFileDialog.getExistingDirectory(
            self, "Select a local folder for output files (CSVs, JPEGs)")
        if not out_folder:
            return
        self.folder = out_folder

        self.folder_button.hide()
        self.load_csv_button.hide()
        self.load_chunked_button.hide()
        self.load_batch_button.hide()
        self.dropbox_button.hide()
        self.continue_button.hide()
        self.diff_folder_button.show()
        self.summary_label.setText(
            f"Dropbox: {self.dbx_folder}\n"
            f"{len(self.bin_files)} .bin file(s) found.\n"
            f"Output: {self.folder}")

        self.startTimeLabel.show()
        self.start_date_edit.show()
        self.endTimeLabel.show()
        self.end_date_edit.show()
        self.chunkSizeHours.show()
        self.make_graphs_button.show()
        self.detail_settings_button.show()
        self.set_default_date_range()
        self.updateOutputSummary()
        self.outputSummaryLabel.show()

    def _get_settings(self):
        """Read all numeric settings fields, returning (sampFreq, defaultWindows, calcWindows, minFreq, maxFreq)."""
        try:
            sampFreq = self.sampFreq.getValue()
        except ValueError:
            sampFreq = 5000

        try:
            defaultWindows = self.defaultWindows.getValue()
        except ValueError:
            defaultWindows = 0.2

        try:
            calcWindows = self.calcWindows.getValue()
        except ValueError:
            calcWindows = 0.2

        try:
            minFreq = self.minFreq.getValue()
        except ValueError:
            minFreq = 0

        try:
            maxFreq = self.maxFreq.getValue()
        except ValueError:
            maxFreq = 0

        return sampFreq, defaultWindows, calcWindows, minFreq, maxFreq

    def _validate_settings(self, sampFreq, defaultWindows, calcWindows, minFreq, maxFreq):
        """Return a list of human-readable error strings for any invalid (zero/negative) settings."""
        errors = []
        if sampFreq <= 0:
            errors.append("Sampling Freq must be greater than 0 Hz.")
        if defaultWindows <= 0:
            errors.append("Default window must be greater than 0 s.")
        if calcWindows <= 0:
            errors.append("Output window (time step) must be greater than 0 s.")
        if maxFreq <= minFreq:
            errors.append("Max freq must be greater than Min freq.")
        return errors

    def updateOutputSummary(self):
        bytePerValue = 25.99 ## the number of bytes per value (estimated from output; may change)

        sampFreq, defaultWindows, calcWindows, minFreq, maxFreq = self._get_settings()

        errors = self._validate_settings(sampFreq, defaultWindows, calcWindows, minFreq, maxFreq)
        if errors:
            self.outputSummaryLabel.setText("Invalid settings: " + " | ".join(errors))
            return

        freq_steps = (sampFreq   /2) / (defaultWindows * sampFreq / 2) ## the step size of frequency axis, estimated from the default spectrogram settings

        fbands = np.arange(minFreq, maxFreq + freq_steps, freq_steps).shape[0] + 1


        tsteps = np.arange(self.start_date_edit.dateTime().toSecsSinceEpoch() + self.start_date_edit.dateTime().offsetFromUtc(), self.end_date_edit.dateTime().toSecsSinceEpoch() + self.end_date_edit.dateTime().offsetFromUtc() + calcWindows, calcWindows).shape[0]

        rows = 1 + (fbands) # the number of frequency values
        cols = 1 + (tsteps) # the number of time steps
        expected__bytes = rows * cols * bytePerValue
        start_str = self.start_date_edit.dateTime().toString('dd-MMM-yyyy HH:mm:ss')
        end_str   = self.end_date_edit.dateTime().toString('dd-MMM-yyyy HH:mm:ss')
        self.outputSummaryLabel.setText(
            f"Selected range: {start_str} to {end_str}\n"
            f"Expected output of approximately {convert_size(expected__bytes)} for each channel."
        )


    # Set default date range for bin files based on file modification times
    def set_default_date_range(self):

        try:
            sampFreq = self.sampFreq.getValue()
        except ValueError:
            sampFreq = 5000

        if sampFreq <= 0:
            sampFreq = 5000  # fallback to avoid division by zero in file-duration estimate
    
        startTimes = []
        for file in self.bin_files:
            startTimes.append(utils.extract_start_time(file))

        # Set min and max dates in the QDateTimeEdit widgets
        minTime = min(startTimes).replace(hour=0, minute=0, second=0)
        maxTime = max(startTimes).replace(hour=23, minute=59, second=59)
        self.start_date_edit.setDateTime(minTime)
        self.start_date_edit.setMinimumDateTime(minTime)
        self.start_date_edit.setMaximumDateTime(maxTime)
        self.end_date_edit.setDateTime(maxTime)
        self.end_date_edit.setMinimumDateTime(minTime)
        self.end_date_edit.setMaximumDateTime(maxTime)
        
        last_size = self.file_size_map.get(self.bin_files[-1], 0)
        endTime = max(startTimes) + pd.to_timedelta(round(last_size / (sampFreq * 6 * 2.1333), 0), unit='s')
        self.summary_label.setText(f"{self.summary_label.text()}\nTime range: {min(startTimes).strftime("%d-%b-%Y %H:%M:%S")} to {endTime.strftime("%d-%b-%Y %H:%M:%S")}")

    # Generate graphs (show progress)
    def make_graphs(self):

        sampFreq, defaultWindows, calcWindows, minFreq, maxFreq = self._get_settings()

        errors = self._validate_settings(sampFreq, defaultWindows, calcWindows, minFreq, maxFreq)
        if errors:
            QMessageBox.warning(self, "Invalid settings",
                                "Please fix the following before generating graphs:\n\n• " +
                                "\n• ".join(errors))
            return

        agg = self.aggMethod.currentText()
        start_time = self.start_date_edit.dateTime()
        end_time = self.end_date_edit.dateTime()

        try:
            chunk_hours = self.chunkSizeHours.getValue()
        except ValueError:
            chunk_hours = 0

        # Hide plot and buttons until processing is done
        self.plot_canvas.hide()
        self.toggle_buttons_visibility(False)

        # Hide the settings and change the button text
        self.detail_settings_button.setText("Edit settings")
        self.sampFreq.hide()
        self.defaultWindows.hide()
        self.calcWindows.hide()
        self.minFreq.hide()
        self.maxFreq.hide()
        self._agg_row.hide()

        ## update the info with the start and end dates (do this now, before
        ## the thread starts, so the QDateTime widgets are still visible)
        chunk_note = f" in {chunk_hours}h chunks" if chunk_hours > 0 else ""
        summary_suffix = (
            f"\nStart time: {start_time.toString('dd-MMM-yyyy HH:mm:ss')}"
            f"\nEnd time: {end_time.toString('dd-MMM-yyyy HH:mm:ss')}{chunk_note}"
            f"\nData were sampled at {sampFreq} Hz, spectrograms created with a"
            f" window of {defaultWindows} seconds, outputted every {calcWindows}"
            f" seconds in the frequency range {minFreq} to {maxFreq}.")
        self._pending_summary_suffix = summary_suffix

        ## hide date controls
        self.startTimeLabel.hide()
        self.start_date_edit.hide()
        self.endTimeLabel.hide()
        self.end_date_edit.hide()
        self.chunkSizeHours.hide()
        self.make_graphs_button.hide()

        if chunk_hours > 0:
            # ── Chunked mode ──────────────────────────────────────────────────
            self._proc_thread = _ChunkedProcessingThread(
                self.folder, self.bin_files, start_time, end_time,
                chunk_hours, sampFreq, defaultWindows, calcWindows,
                minFreq, maxFreq,
                dbx=self.dbx, dbx_folder=self.dbx_folder, agg=agg,
                local_bin_folder=self._local_bin_folder)
            self._proc_thread.finished.connect(self._on_chunked_done)
            self._proc_thread.error.connect(self._on_processing_error)
            self._proc_thread.start()
        else:
            # ── Single-file mode (original behaviour) ─────────────────────────
            chk_dir = os.path.join(self.folder, _CHK_SUBDIR)
            chk_key = _checkpoint_settings_key(start_time, end_time, sampFreq, defaultWindows,
                                               calcWindows, minFreq, maxFreq, agg,
                                               self.dbx_folder or self.folder)
            resume_files = None
            chk_meta_path = os.path.join(chk_dir, 'meta.json')
            if os.path.isfile(chk_meta_path):
                try:
                    with open(chk_meta_path) as _f:
                        _meta = json.load(_f)
                    if _checkpoint_meta_matches(_meta, chk_key):
                        n_done = len(_meta.get('processed_files', []))
                        n_total = len(self.bin_files)
                        saved_at = _meta.get('saved_at', 'unknown')
                        msg = QMessageBox(self)
                        msg.setWindowTitle("Resume from checkpoint?")
                        msg.setText(
                            f"A checkpoint was found for these exact settings.\n\n"
                            f"Progress saved: {n_done} of {n_total} file(s) processed\n"
                            f"Saved at: {saved_at}")
                        msg.setInformativeText("Resume from where it left off, or start fresh?")
                        resume_btn = msg.addButton("Resume", QMessageBox.YesRole)
                        fresh_btn  = msg.addButton("Start Fresh", QMessageBox.NoRole)
                        cancel_btn = msg.addButton("Cancel", QMessageBox.RejectRole)
                        msg.setDefaultButton(resume_btn)
                        msg.exec_()
                        clicked = msg.clickedButton()
                        if clicked == cancel_btn:
                            # Restore UI and bail out
                            self.startTimeLabel.show(); self.start_date_edit.show()
                            self.endTimeLabel.show();   self.end_date_edit.show()
                            self.chunkSizeHours.show()
                            self.make_graphs_button.show()
                            return
                        elif clicked == fresh_btn:
                            shutil.rmtree(chk_dir, ignore_errors=True)
                        else:  # Resume
                            resume_files = set(_meta.get('processed_files', []))
                    else:
                        shutil.rmtree(chk_dir, ignore_errors=True)
                except Exception:
                    shutil.rmtree(chk_dir, ignore_errors=True)

            self._proc_thread = _ProcessingThread(
                self.folder, self.bin_files, start_time, end_time,
                sampFreq, defaultWindows, calcWindows, minFreq, maxFreq,
                dbx=self.dbx, dbx_folder=self.dbx_folder, agg=agg,
                checkpoint_dir=chk_dir, checkpoint_key=chk_key,
                resume_files=resume_files,
                local_bin_folder=self._local_bin_folder)
            self._proc_thread.finished.connect(self._on_processing_done)
            self._proc_thread.error.connect(self._on_processing_error)
            self._proc_thread.start()

    def _on_processing_done(self, spectrograms):
        self.spectrograms = spectrograms

        self.summary_label.setText(self.summary_label.text() + self._pending_summary_suffix)

        self._init_zoom_controls()
        # Show plots
        self.plot_canvas.show()
        self.plot_channel(0)
        self.plot_canvas.draw()
        self.toggle_buttons_visibility(True)
        self.setGeometry(100, 100, 800, 600)

        # Write CSVs in a separate daemon thread
        threading.Thread(
            target=write_spectrograms_to_disk,
            args=(self.folder, self.spectrograms),
            daemon=True
        ).start()

    def _on_chunked_done(self, chunk_list):
        """Called when chunked processing completes. Load and display the stitched result."""
        self.summary_label.setText(self.summary_label.text() + self._pending_summary_suffix)

        spectrograms = _load_and_concat_chunks(self.folder, chunk_list)
        if spectrograms is None:
            QMessageBox.warning(self, "No data",
                                "Chunked processing completed but no CSV files could be loaded.")
            self.make_graphs_button.show()
            self.startTimeLabel.show(); self.start_date_edit.show()
            self.endTimeLabel.show();   self.end_date_edit.show()
            self.chunkSizeHours.show()
            return

        self.spectrograms = spectrograms
        self._init_zoom_controls()
        self.plot_canvas.show()
        self.plot_channel(0)
        self.plot_canvas.draw()
        self.toggle_buttons_visibility(True)
        self.setGeometry(100, 100, 800, 600)

    def _on_processing_error(self, msg):
        QMessageBox.critical(self, "Processing error", msg)
        self.make_graphs_button.show()
        self.startTimeLabel.show(); self.start_date_edit.show()
        self.endTimeLabel.show();   self.end_date_edit.show()
        self.chunkSizeHours.show()


    # Plot a specific channel
    def plot_channel(self, channel_index):
        if self.spectrograms:
            self._current_view = ('channel', channel_index)
            self._do_plot_channel(channel_index)

    def _do_plot_channel(self, channel_index):
        spec = self._apply_zoom_to_spec(self.spectrograms[channel_index])
        self.plot_canvas.clear_figure()
        self.plot_canvas.plot_spectrogram(spec, channel_index)
        self.plot_canvas.draw()

    # Show all channels
    def show_all_channels(self):
        if self.spectrograms:
            self._current_view = ('all',)
            self._do_show_all()

    def _do_show_all(self):
        zoomed = [self._apply_zoom_to_spec(s) for s in self.spectrograms]
        self.plot_canvas.clear_figure()
        self.plot_canvas.plot_all_spectrograms(zoomed)
        self.plot_canvas.draw()

    def _get_average_spectrogram(self):
        """Return (avg_spec, label) for the currently checked channels, or (None, None)."""
        selected = [i for i, cb in enumerate(self.avg_checkboxes) if cb.isChecked()]
        if not selected:
            QMessageBox.warning(self, "No channels selected",
                                "Please check at least one channel to average.")
            return None, None
        if not self.spectrograms:
            return None, None

        # Use the first selected channel as the reference for timestamps/freqs
        ref = self.spectrograms[selected[0]]
        stacks = [self.spectrograms[i][1:, 1:] for i in selected]
        avg_power = np.nanmean(np.stack(stacks, axis=0), axis=0)

        # Rebuild the spectrogram array with the averaged power block
        avg_spec = ref.copy()
        avg_spec[1:, 1:] = avg_power

        label = "Average (Ch " + ", ".join(str(i + 1) for i in selected) + ")"
        return avg_spec, label

    def show_average_channels(self):
        avg_spec, label = self._get_average_spectrogram()
        if avg_spec is None:
            return
        self._current_view = ('average',)
        self._do_show_average(avg_spec, label)

    def _do_show_average(self, avg_spec=None, label=None):
        if avg_spec is None:
            avg_spec, label = self._get_average_spectrogram()
            if avg_spec is None:
                return
        zoomed = self._apply_zoom_to_spec(avg_spec)
        self.plot_canvas.clear_figure()
        self.plot_canvas.plot_spectrogram(zoomed, title=label)
        self.plot_canvas.draw()

    def export_average_csv(self):
        avg_spec, label = self._get_average_spectrogram()
        if avg_spec is None:
            return
        default_path = os.path.join(self.folder or "", "Spectrogram_Average.csv")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save average spectrogram as CSV",
            default_path, "CSV files (*.csv)")
        if not path:
            return
        if not path.lower().endswith('.csv'):
            path += '.csv'
        np.savetxt(path, avg_spec, delimiter=',')
        self.summary_label.setText(f"{self.summary_label.text()}\nAverage CSV saved: {path}")

    # ── Zoom helpers ─────────────────────────────────────────────────────────

    def _init_zoom_controls(self):
        """Populate the zoom inputs from the full extent of the loaded spectrograms."""
        ref = next((s for s in self.spectrograms if s is not None), None)
        if ref is None:
            return
        timestamps = ref[0, 1:]
        freqs = ref[1:, 0]
        t0, t1 = float(timestamps[0]), float(timestamps[-1])
        f0, f1 = float(freqs[0]), float(freqs[-1])

        self.zoom_start_dt.setDateTime(QDateTime.fromSecsSinceEpoch(int(t0), Qt.UTC))
        self.zoom_end_dt.setDateTime(QDateTime.fromSecsSinceEpoch(int(t1), Qt.UTC))
        self.zoom_freq_min_input.setText(f"{f0:.1f}")
        self.zoom_freq_max_input.setText(f"{f1:.1f}")

        # Clear any previously applied zoom
        self._zoom_t_start = None
        self._zoom_t_end = None
        self._zoom_f_min = None
        self._zoom_f_max = None

    def _apply_zoom_to_spec(self, spec):
        """Return a sliced copy of spec restricted to the current zoom window."""
        if spec is None:
            return spec
        timestamps = spec[0, 1:]
        freqs = spec[1:, 0]

        t_start = self._zoom_t_start if self._zoom_t_start is not None else timestamps[0]
        t_end   = self._zoom_t_end   if self._zoom_t_end   is not None else timestamps[-1]
        f_min   = self._zoom_f_min   if self._zoom_f_min   is not None else freqs[0]
        f_max   = self._zoom_f_max   if self._zoom_f_max   is not None else freqs[-1]

        t_mask = (timestamps >= t_start) & (timestamps <= t_end)
        f_mask = (freqs >= f_min) & (freqs <= f_max)

        col_idx = np.where(t_mask)[0] + 1   # +1: col 0 is freq axis
        row_idx = np.where(f_mask)[0] + 1   # +1: row 0 is time axis

        if col_idx.size == 0 or row_idx.size == 0:
            return spec   # nothing matched — return full data unchanged

        rows = np.concatenate([[0], row_idx])
        cols = np.concatenate([[0], col_idx])
        return spec[np.ix_(rows, cols)]

    def apply_zoom(self):
        try:
            f_min = float(self.zoom_freq_min_input.text())
            f_max = float(self.zoom_freq_max_input.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid input", "Frequency values must be numbers.")
            return

        # Reconstruct as UTC — the widget displays UTC to match the spectrogram
        # axis, but Qt may have reset the internal timespec to local after user
        # interaction, which would make toSecsSinceEpoch() return a wrong epoch.
        ds = self.zoom_start_dt.dateTime()
        de = self.zoom_end_dt.dateTime()
        t_start = float(QDateTime(ds.date(), ds.time(), Qt.UTC).toSecsSinceEpoch())
        t_end   = float(QDateTime(de.date(), de.time(), Qt.UTC).toSecsSinceEpoch())

        if t_end <= t_start:
            QMessageBox.warning(self, "Invalid range", "End time must be after start time.")
            return
        if f_max <= f_min:
            QMessageBox.warning(self, "Invalid range", "Max frequency must be greater than min.")
            return

        self._zoom_t_start = t_start
        self._zoom_t_end   = t_end
        self._zoom_f_min   = f_min
        self._zoom_f_max   = f_max
        self._replot_current()

    def reset_zoom(self):
        self._zoom_t_start = None
        self._zoom_t_end   = None
        self._zoom_f_min   = None
        self._zoom_f_max   = None
        self._init_zoom_controls()
        self._replot_current()

    def _replot_current(self):
        if self._current_view is None or not self.spectrograms:
            return
        kind = self._current_view[0]
        if kind == 'channel':
            self._do_plot_channel(self._current_view[1])
        elif kind == 'all':
            self._do_show_all()
        elif kind == 'average':
            self._do_show_average()

    # Show or hide toggle buttons
    def toggle_buttons_visibility(self, visible):
        for btn in self.toggle_buttons:
            btn.setVisible(visible)
        self.show_all_button.setVisible(visible)
        self.save_jpeg_button.setVisible(visible)
        for w in self._avg_row_widgets:
            w.setVisible(visible)
        for w in self._zoom_row_widgets:
            w.setVisible(visible)
        for w in self._freq_export_widgets:
            w.setVisible(visible)

    def load_existing_spectrograms(self):
        """Load previously saved Spectrogram_x.csv files and display them directly."""
        folder = QFileDialog.getExistingDirectory(self, "Select folder containing Spectrogram CSV files")
        if not folder:
            return

        spectrograms = []
        for c in range(1, 7):
            path = os.path.join(folder, f"Spectrogram_{c}.csv")
            if os.path.exists(path):
                spectrograms.append(np.loadtxt(path, delimiter=','))
            else:
                spectrograms.append(None)

        loaded = [s for s in spectrograms if s is not None]
        if not loaded:
            QMessageBox.warning(self, "No files found",
                                f"No Spectrogram_x.csv files found in:\n{folder}")
            return

        # Replace missing channels with a blank array matching the first loaded shape
        ref = loaded[0]
        blank = np.full_like(ref, np.nan)
        blank[0, 1:] = ref[0, 1:]  # copy timestamps
        blank[1:, 0] = ref[1:, 0]  # copy frequencies
        self.spectrograms = [s if s is not None else blank.copy() for s in spectrograms]

        self.folder = folder
        self.folder_button.hide()
        self.load_csv_button.hide()
        self.load_chunked_button.hide()
        self.load_batch_button.hide()
        self.diff_folder_button.show()
        self.summary_label.setText(
            f"Loaded {len(loaded)} spectrogram(s) from:\n{folder}")

        self._init_zoom_controls()
        self.plot_canvas.show()
        self.plot_channel(0)
        self.plot_canvas.draw()
        self.toggle_buttons_visibility(True)
        self.setGeometry(100, 100, 800, 600)

    def load_chunked_spectrograms(self):
        """Let the user pick a folder + date range and load/concatenate chunked CSVs."""
        folder = QFileDialog.getExistingDirectory(
            self, "Select folder containing chunked spectrogram CSV files")
        if not folder:
            return

        # Discover all ch1 files and extract their date ranges
        all_chunks = []
        for fname in sorted(os.listdir(folder)):
            m = _CHUNK_FILENAME_RE.match(fname)
            if m and m.group(3) == '1':
                try:
                    s = datetime.datetime.strptime(m.group(1), '%Y%m%d_%H%M%S')
                    e = datetime.datetime.strptime(m.group(2), '%Y%m%d_%H%M%S')
                    all_chunks.append((s, e))
                except ValueError:
                    continue

        if not all_chunks:
            QMessageBox.warning(self, "No chunked files found",
                                f"No chunked spectrogram files (YYYYMMDD_HHMMSS_YYYYMMDD_HHMMSS_ch1.csv) "
                                f"found in:\n{folder}")
            return

        all_chunks.sort()
        min_dt = all_chunks[0][0]
        max_dt = all_chunks[-1][1]

        # Ask the user to confirm / narrow the date range
        dlg = _ChunkDateRangeDialog(min_dt, max_dt, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        sel_start, sel_end = dlg.selected_range()

        # Filter chunks that fall within the selected range
        selected_chunks = [
            (s, e) for s, e in all_chunks
            if s >= sel_start and e <= sel_end
        ]
        if not selected_chunks:
            QMessageBox.warning(self, "No chunks in range",
                                "No chunked CSV files found in the selected date range.")
            return

        spectrograms = _load_and_concat_chunks(folder, selected_chunks)
        if spectrograms is None:
            QMessageBox.warning(self, "Load failed",
                                "Could not load any spectrogram data from the selected chunks.")
            return

        self.folder = folder
        self.folder_button.hide()
        self.load_csv_button.hide()
        self.load_chunked_button.hide()
        self.load_batch_button.hide()
        self.dropbox_button.hide()
        self.continue_button.hide()
        self.diff_folder_button.show()
        n = len(selected_chunks)
        self.summary_label.setText(
            f"Loaded {n} chunk(s) from:\n{folder}\n"
            f"Range: {sel_start.strftime('%d-%b-%Y %H:%M:%S')} to "
            f"{sel_end.strftime('%d-%b-%Y %H:%M:%S')}")

        self.spectrograms = spectrograms
        self._init_zoom_controls()
        self.plot_canvas.show()
        self.plot_channel(0)
        self.plot_canvas.draw()
        self.toggle_buttons_visibility(True)
        self.setGeometry(100, 100, 800, 600)

    def load_batch_spectrograms(self):
        """Load batch-pipeline daily spectrogram CSVs (YYYYMMDD_chX.csv) over a date range."""
        folder = QFileDialog.getExistingDirectory(
            self, "Select folder containing batch spectrogram CSV files")
        if not folder:
            return

        # Discover available dates by scanning for ch1 files
        available_dates = []
        for fname in sorted(os.listdir(folder)):
            m = _BATCH_FILENAME_RE.match(fname)
            if m and m.group(2) == '1':
                try:
                    available_dates.append(
                        datetime.datetime.strptime(m.group(1), '%Y%m%d').date())
                except ValueError:
                    continue

        if not available_dates:
            QMessageBox.warning(self, "No batch files found",
                                f"No YYYYMMDD_ch1.csv files found in:\n{folder}")
            return

        available_dates.sort()
        min_date = available_dates[0]
        max_date = available_dates[-1]

        dlg = _BatchDayDateRangeDialog(min_date, max_date, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        sel_start, sel_end = dlg.selected_range()

        if sel_start > sel_end:
            QMessageBox.warning(self, "Bad date range", "Start date must be before end date.")
            return

        selected_dates = [d for d in available_dates if sel_start <= d <= sel_end]
        if not selected_dates:
            QMessageBox.warning(self, "No files in range",
                                "No batch CSV files found in the selected date range.")
            return

        spectrograms = _load_and_concat_batch_days(folder, selected_dates)
        if spectrograms is None:
            QMessageBox.warning(self, "Load failed",
                                "Could not load any spectrogram data from the selected dates.")
            return

        self.folder = folder
        self.folder_button.hide()
        self.load_csv_button.hide()
        self.load_chunked_button.hide()
        self.load_batch_button.hide()
        self.dropbox_button.hide()
        self.continue_button.hide()
        self.diff_folder_button.show()
        n = len(selected_dates)
        self.summary_label.setText(
            f"Loaded {n} day(s) from:\n{folder}\n"
            f"Range: {sel_start.strftime('%d-%b-%Y')} to {sel_end.strftime('%d-%b-%Y')}")

        self.spectrograms = spectrograms
        self._init_zoom_controls()
        self.plot_canvas.show()
        self.plot_channel(0)
        self.plot_canvas.draw()
        self.toggle_buttons_visibility(True)
        self.setGeometry(100, 100, 800, 600)

    def save_as_jpeg(self):
        """Save the currently displayed figure as a JPEG."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save spectrogram as JPEG",
            os.path.join(self.folder or "", "spectrogram.jpg"),
            "JPEG images (*.jpg *.jpeg)")
        if not path:
            return
        if not path.lower().endswith(('.jpg', '.jpeg')):
            path += '.jpg'
        self.plot_canvas.fig.savefig(path, format='jpeg', dpi=150, bbox_inches='tight')
        self.summary_label.setText(f"{self.summary_label.text()}\nSaved: {path}")

    def export_band_summary_clicked(self):
        """Parse frequency bands from the input field, compute per-band averages, and save CSV."""
        bands = parse_freq_bands(self.freq_bands_input.text())
        if not bands:
            QMessageBox.warning(self, "Invalid input",
                                "No valid frequency bands found.\n"
                                "Use format like: 10-50 100-200 300-500")
            return
        if not self.spectrograms:
            QMessageBox.warning(self, "No data", "No spectrograms loaded.")
            return
        path = export_band_summary(self.spectrograms, bands, self.folder)
        self.summary_label.setText(f"{self.summary_label.text()}\nBand summary saved: {path}")

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()
        self.axes = self.fig.add_subplot(111)
        self._im = None   # reused AxesImage handle
        super().__init__(self.fig)
        self.setParent(parent)

    @staticmethod
    def _to_rgb(data):
        """Convert a float spectrogram slice to a uint8 RGB image via a
        pre-built jet LUT.  NaN/inf pixels are rendered as white."""
        nan_mask = ~np.isfinite(data) ##This is different to Manu's - to make nan pixels white
        d = data.astype(np.float32)
        # Normalise using only finite values so NaN doesn't skew the range
        finite = d[~nan_mask]
        if finite.size > 0:
            vmin, vmax = float(finite.min()), float(finite.max())
        else:
            vmin, vmax = 0.0, 1.0
        if vmax > vmin:
            d = (d - vmin) / (vmax - vmin)
        else:
            d = np.zeros_like(d)
        # Build jet LUT once via matplotlib, then apply via integer indexing
        import matplotlib.cm as cm
        lut = (cm.jet(np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
        idx = np.where(nan_mask, 0, (d * 255).clip(0, 255)).astype(np.uint8)
        rgb = lut[idx]
        rgb[nan_mask] = 255  # white for missing data
        return rgb  # (freq, time, 3) uint8

    def _apply_ticks(self, axis, spectrogram, n_x=5, n_y=5):
        time_stamps = pd.to_datetime(spectrogram[0, 1:], unit='s')
        tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=n_x, dtype=int)
        axis.set_xticks(tick_indices_x)
        axis.set_xticklabels(time_stamps[tick_indices_x].strftime('%Y-%m-%d %H:%M:%S'), ha='center')
        freqVals = spectrogram[1:, 0]
        tick_indices_y = np.linspace(0, len(freqVals) - 1, num=n_y, dtype=int)
        axis.set_yticks(tick_indices_y)
        axis.set_yticklabels(freqVals[tick_indices_y], ha='center')

    # Plot a single spectrogram — reuses the existing AxesImage when possible
    def plot_spectrogram(self, spectrogram, ID=None, title=None):
        resolved_title = title if title is not None else (f"Channel {ID+1}" if ID is not None else "")
        data = spectrogram[1:, 1:]
        if data.size == 0:
            self.axes.cla()
            self.axes.set_title(f"{resolved_title} — no data in selected range")
            self._im = None
            return
        self.axes.cla()
        self._im = self.axes.imshow(data, aspect='auto', origin='lower',
                                    cmap='jet', interpolation='nearest')
        self._apply_ticks(self.axes, spectrogram, n_x=5, n_y=5)
        self.axes.set_title(resolved_title)

    # Plot all 6 spectrograms together
    def plot_all_spectrograms(self, spectrograms):
        self.fig.clear()
        self._im = None
        ax = self.fig.subplots(2, 3)
        for i in range(6):
            axis = ax[i // 3, i % 3]
            data = spectrograms[i][1:, 1:]
            if data.size == 0:
                axis.set_title(f"Channel {i+1} — no data")
                continue
            axis.imshow(data, aspect='auto', origin='lower', cmap='jet', interpolation='nearest')
            axis.set_title(f"Channel {i + 1}")
            self._apply_ticks(axis, spectrograms[i], n_x=2, n_y=5)
            if i % 3 != 0:
                axis.yaxis.set_visible(False)
            if i < 3:
                axis.xaxis.set_visible(False)

    # Clear the entire figure (whether single or subplots)
    def clear_figure(self):
        self.fig.clear()
        self._im = None
        self.axes = self.fig.add_subplot(111)


# ── Checkpoint helpers ────────────────────────────────────────────────────────

_CHK_SUBDIR = '_beespy_checkpoint'


def _checkpoint_settings_key(start_time, end_time, sampFreq, defaultWindows,
                              calcWindows, minFreq, maxFreq, agg, source):
    """Return a dict of settings that uniquely identify a processing run."""
    return {
        'start_epoch':    round(start_time.toSecsSinceEpoch() + start_time.offsetFromUtc(), 1),
        'end_epoch':      round(end_time.toSecsSinceEpoch()   + end_time.offsetFromUtc(),   1),
        'sampFreq':       sampFreq,
        'defaultWindows': round(defaultWindows, 6),
        'calcWindows':    round(calcWindows, 6),
        'minFreq':        round(minFreq, 6),
        'maxFreq':        round(maxFreq, 6),
        'agg':            agg,
        'source':         str(source),
    }


def _checkpoint_meta_matches(meta, key):
    """Return True when the stored meta matches all fields in key."""
    return all(meta.get(k) == v for k, v in key.items())


def _write_checkpoint(chk_dir, key, processed_files, accumulators, mode):
    """Atomically write checkpoint data.  meta.json is written last so it
    serves as the 'committed' marker — an incomplete write leaves the previous
    meta.json intact."""
    os.makedirs(chk_dir, exist_ok=True)
    for c in range(6):
        if mode == 'avg':
            np.save(os.path.join(chk_dir, f'ch{c}_sum.npy'),   accumulators[0][c])
            np.save(os.path.join(chk_dir, f'ch{c}_count.npy'), accumulators[1][c])
        elif mode == 'max':
            np.save(os.path.join(chk_dir, f'ch{c}_max.npy'), accumulators[c])
        else:  # pct — defaultdict(list) serialised with pickle
            with open(os.path.join(chk_dir, f'ch{c}_bins.pkl'), 'wb') as f:
                pickle.dump(dict(accumulators[c]), f)
    meta = dict(key)
    meta['processed_files'] = list(processed_files)
    meta['saved_at'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    tmp = os.path.join(chk_dir, '_meta_tmp.json')
    with open(tmp, 'w') as f:
        json.dump(meta, f, indent=2)
    os.replace(tmp, os.path.join(chk_dir, 'meta.json'))
    print(f"  [checkpoint] Saved ({len(processed_files)} files processed)")


def _load_checkpoint(chk_dir, mode, n_freq, n_time):
    """Load accumulators from a checkpoint.

    Returns (accumulators, processed_files_set) or (None, None) on failure.
    """
    try:
        if mode == 'avg':
            accum_sum, accum_count = [], []
            for c in range(6):
                accum_sum.append(np.load(os.path.join(chk_dir, f'ch{c}_sum.npy')))
                accum_count.append(np.load(os.path.join(chk_dir, f'ch{c}_count.npy')))
            accumulators = (accum_sum, accum_count)
        elif mode == 'max':
            accumulators = [np.load(os.path.join(chk_dir, f'ch{c}_max.npy')) for c in range(6)]
        else:  # pct
            from collections import defaultdict
            accumulators = []
            for c in range(6):
                with open(os.path.join(chk_dir, f'ch{c}_bins.pkl'), 'rb') as f:
                    d = pickle.load(f)
                accumulators.append(defaultdict(list, {int(k): v for k, v in d.items()}))
        with open(os.path.join(chk_dir, 'meta.json')) as f:
            meta = json.load(f)
        return accumulators, set(meta.get('processed_files', []))
    except Exception as _e:
        print(f"  WARNING: could not load checkpoint ({_e}) — starting fresh")
        return None, None


# Do the hard work here
def process_bin_files(folder, bin_files, start_time, end_time, sampFreq, defaultWindows, calcWindows, minFreq, maxFreq,
                      dbx=None, dbx_folder=None, agg="Average",
                      checkpoint_dir=None, checkpoint_key=None,
                      resume_files=None, checkpoint_every=10,
                      local_bin_folder=None):
    """Process .bin files into spectrograms, aggregated per output time bin.

    agg: "Average", "Maximum", "75th percentile", "90th percentile", "95th percentile"
    Pass dbx + dbx_folder to read files from Dropbox on-demand (one at a time,
    no bulk download).  Leave both as None to read from the local filesystem.

    checkpoint_dir / checkpoint_key: when set, the function saves progress to disk
    every checkpoint_every files so it can be resumed after a crash or lost connection.
    resume_files: set of filenames already processed on a previous run (skip them).

    local_bin_folder: when set (local mode), _safe_to_delete() will refuse to delete
    any .bin file inside this folder, protecting the user's raw data.
    """
    # Safety check: confirm we will not accidentally delete files from the local source
    if local_bin_folder is not None:
        print(f"  [SAFETY] Local bin folder protected from deletion: {local_bin_folder}")
    elif dbx is None:
        # Local mode but no protection path set — warn so this is auditable
        print("  [WARNING] Local mode but local_bin_folder not set — bin files unprotected")
    # Determine percentile level if needed
    _pct_map = {"75th percentile": 75, "90th percentile": 90, "95th percentile": 95}
    pct_level = _pct_map.get(agg, None)
    use_max = (agg == "Maximum")
    use_pct = pct_level is not None

    # Calculate the frequency axis using the exact scipy bins (same as dospectrogram produces)
    nps_out = max(2, int(round(defaultWindows * sampFreq)))
    fq_full = np.fft.rfftfreq(nps_out, 1.0 / sampFreq)
    freq_mask = (fq_full >= minFreq) & (fq_full <= maxFreq)
    freqs = fq_full[freq_mask]
    _first_freq_idx = int(np.where(freq_mask)[0][0]) if freq_mask.any() else 0
    times = np.arange(start_time.toSecsSinceEpoch()+ start_time.offsetFromUtc(), end_time.toSecsSinceEpoch()+ end_time.offsetFromUtc()+calcWindows, calcWindows)

    ## set up the output - 6 channels with the above dimensions
    if use_max:
        spectrogram_max = [np.full((freqs.shape[0], times.shape[0]), -np.inf) for _ in range(6)]
    elif use_pct:
        from collections import defaultdict
        spectrogram_bins = [defaultdict(list) for _ in range(6)]
    else:  # average
        spectrogram_sum = [np.zeros((freqs.shape[0], times.shape[0])) for _ in range(6)]
        spectrogram_count = [np.zeros((freqs.shape[0], times.shape[0])) for _ in range(6)]

    # ── Checkpoint resume: load accumulators from previous run if applicable ──
    _chk_mode = 'max' if use_max else ('pct' if use_pct else 'avg')
    _files_already_done = set(resume_files) if resume_files else set()
    if checkpoint_dir and resume_files:
        _chk_accum, _chk_done = _load_checkpoint(checkpoint_dir, _chk_mode,
                                                   freqs.shape[0], times.shape[0])
        if _chk_accum is not None:
            _files_already_done = _chk_done
            if use_max:
                spectrogram_max = _chk_accum
            elif use_pct:
                spectrogram_bins = _chk_accum
            else:
                spectrogram_sum, spectrogram_count = _chk_accum
            print(f"  [checkpoint] Resumed — {len(_files_already_done)} file(s) already processed")

    ## now slot in the data
    _DBX_MAX_RETRIES = 3
    _DBX_RETRY_DELAYS = [10, 30, 60]   # seconds to wait before each retry
    files_failed = []                  # files skipped due to download/processing errors

    N = len(bin_files)
    start_ts_dbg = pd.Timestamp(start_time.toSecsSinceEpoch() + start_time.offsetFromUtc(), unit='s')
    end_ts_dbg   = pd.Timestamp(end_time.toSecsSinceEpoch()   + end_time.offsetFromUtc(),   unit='s')
    print(f"MODE: {'Dropbox' if dbx is not None else 'local'} | {N} file(s) | range {start_ts_dbg} → {end_ts_dbg}")
    n = 0
    for file in bin_files:
        n = n+1
        # Skip files already processed in a previous run
        if file in _files_already_done:
            continue
        ## get the start time and estimated end time of each bin file
        fstart = utils.extract_start_time(file)

        if dbx is not None:
            # Dropbox mode: get file size from the API metadata already fetched
            # (we don't have it here, so use a rough fixed estimate; overlap check
            # is conservative — worst case we download a file and skip it quickly)
            fend = fstart + pd.to_timedelta(3600, unit='s')   # 1-hour upper bound
        else:
            filepath = os.path.join(folder, file)
            fend = fstart + pd.to_timedelta(
                round(os.path.getsize(filepath) / (sampFreq * 6 * 2.1333), 0), unit='s')

        overlaps = check_overlap(start_time, end_time, fstart, fend)

        ##if the time is in the requested range then make the spectrogram from the data
        if overlaps:
            if dbx is not None:
                dbx_path = f"{dbx_folder.rstrip('/')}/{file}"
                print(f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | File {n} of {N} | {file} | Downloading from Dropbox…")
                file_obj = None
                for _attempt in range(_DBX_MAX_RETRIES):
                    try:
                        file_obj = dropbox_helper.download_to_bytes(dbx, dbx_path)
                        _sz = file_obj.seek(0, 2); file_obj.seek(0)
                        print(f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Downloaded {_sz} bytes from {dbx_path}")
                        break
                    except Exception as _dl_err:
                        if _attempt < _DBX_MAX_RETRIES - 1:
                            _wait = _DBX_RETRY_DELAYS[_attempt]
                            print(f"  WARNING: download failed ({_dl_err}). "
                                  f"Retrying in {_wait}s… (attempt {_attempt + 1}/{_DBX_MAX_RETRIES})")
                            time.sleep(_wait)
                        else:
                            print(f"  ERROR: download failed after {_DBX_MAX_RETRIES} attempts — skipping {file}")
                            files_failed.append(file)
                if file_obj is None:
                    continue
            else:
                filepath = os.path.join(folder, file)
                _fsz = os.path.getsize(filepath)
                print(f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | File {n} of {N} | {file} | {_fsz} bytes | Calculating spectrograms, etc.")
                file_obj = filepath

            # ── Process the file (guard against corrupt/partial data) ──────────
            try:
                fileStart = utils.timestamp_to_secs_since_epoch(fstart)
                x = bc.beespy_arduino_reader(file_obj)  # accepts both path str and BytesIO
                print(f"  Data shape: {x.shape}")
                if x.shape[0] == 0:
                    print(f"  WARNING: no data in {file} (file may be an online-only Dropbox placeholder) — skipping")
                    continue
                for c in range(x.shape[1]): #for each channel
                    # Denoise
                    denoised = denoise.umw_denoise(x[:, c], 5, 5) # denoise the signal
                    # Get the spectrogram
                    fq, ts, tempSpec = spect.dospectrogram(denoised, sampFreq, window_duration=defaultWindows, window_overlap=0)
                    ts = fileStart + ts
                    # Map spectrogram time steps and frequency bins to output grid indices
                    tsIndicies = np.digitize(ts, np.array(np.append(times, end_time.toSecsSinceEpoch() + end_time.offsetFromUtc()+calcWindows+calcWindows)) - (calcWindows/2)) - 1
                    freqIndicies = np.arange(len(fq), dtype=int) - _first_freq_idx
                    # Vectorised scatter-add: replaces the nested Python loop with np.bincount.
                    # Build valid-index masks, then accumulate via flat linear indices so the
                    # entire inner loop runs in a single C-level pass.
                    n_freq_out, n_time_out = freqs.shape[0], times.shape[0]
                    t_valid = (tsIndicies >= 0) & (tsIndicies < n_time_out)    # (n_time_in,)
                    f_valid = (freqIndicies >= 0) & (freqIndicies < n_freq_out) # (n_freq_in,)
                    t_idx = np.where(t_valid)[0]
                    f_idx = np.where(f_valid)[0]
                    if t_idx.size and f_idx.size:
                        tOut = tsIndicies[t_idx]
                        fOut = freqIndicies[f_idx]
                        lin_idx  = (fOut[:, np.newaxis] * n_time_out + tOut[np.newaxis, :]).ravel()
                        vals     = tempSpec[np.ix_(f_idx, t_idx)].ravel()
                        flat_size = n_freq_out * n_time_out
                        if use_max:
                            np.maximum.at(spectrogram_max[c].ravel(), lin_idx, vals)
                        elif use_pct:
                            for idx, v in zip(lin_idx, vals):
                                spectrogram_bins[c][idx].append(v)
                        else:
                            spectrogram_sum[c]   += np.bincount(lin_idx, weights=vals,                minlength=flat_size).reshape(n_freq_out, n_time_out)
                            spectrogram_count[c] += np.bincount(lin_idx, weights=np.ones_like(vals), minlength=flat_size).reshape(n_freq_out, n_time_out)
            except Exception as _proc_err:
                print(f"  ERROR: failed to process {file} ({_proc_err}) — skipping, accumulated data preserved")
                files_failed.append(file)
            # Track and checkpoint only for overlapping files (non-overlapping files
            # are trivially cheap to re-check on resume, so no need to record them)
            _files_already_done.add(file)
            if checkpoint_dir and checkpoint_key and len(_files_already_done) % checkpoint_every == 0:
                _accum_to_save = (spectrogram_sum, spectrogram_count) if not (use_max or use_pct) else (spectrogram_max if use_max else spectrogram_bins)
                _write_checkpoint(checkpoint_dir, checkpoint_key, _files_already_done, _accum_to_save, _chk_mode)
            print(f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Done.")

    if files_failed:
        print(f"\nWARNING: {len(files_failed)} file(s) could not be processed and were skipped:")
        for _f in files_failed:
            print(f"  - {_f}")
        print("Results shown are based on the remaining files only.\n")

    # ── Delete checkpoint on successful completion ─────────────────────────────
    if checkpoint_dir and os.path.isdir(checkpoint_dir):
        shutil.rmtree(checkpoint_dir, ignore_errors=True)
        print("  [checkpoint] Deleted after successful completion")

    # Build the final output arrays (with header row/column for freqs and times)
    n_freq_out = freqs.shape[0]
    n_time_out = times.shape[0]
    spectrogram_average_out = [np.zeros((n_freq_out + 1, n_time_out + 1)) for _ in range(6)]
    for c in range(6):
        spectrogram_average_out[c][0, 0] = f"{99}{sampFreq}99{int(1000*defaultWindows)}{99}"
        spectrogram_average_out[c][0, 1:] = times
        spectrogram_average_out[c][1:, 0] = freqs
        if use_max:
            data = spectrogram_max[c]
            spectrogram_average_out[c][1:, 1:] = np.where(np.isfinite(data), data, np.nan)
        elif use_pct:
            data = np.full(n_freq_out * n_time_out, np.nan)
            for idx, vals in spectrogram_bins[c].items():
                data[idx] = np.percentile(vals, pct_level)
            spectrogram_average_out[c][1:, 1:] = data.reshape(n_freq_out, n_time_out)
        else:
            cnt = spectrogram_count[c]
            avg = np.where(cnt > 0, spectrogram_sum[c] / np.where(cnt > 0, cnt, 1.0), np.nan)
            spectrogram_average_out[c][1:, 1:] = avg

    return spectrogram_average_out


def write_spectrograms_to_disk(folder, spectrograms):
    """Write spectrogram CSVs in a background thread so the UI isn't blocked."""
    print("Writing output to file.")
    for c, spec in enumerate(spectrograms):
        np.savetxt(f"{folder}/Spectrogram_{c+1}.csv", spec, delimiter=',')
    print("Writing complete.")


# ── Chunked spectrogram helpers ───────────────────────────────────────────────

_CHUNK_FILENAME_RE = re.compile(
    r'^(\d{8}_\d{6})_(\d{8}_\d{6})_ch(\d+)\.csv$'
)


def _chunk_csv_path(folder, chunk_start_dt, chunk_end_dt, channel):
    """Return the path for a single channel's chunked CSV.

    e.g. /out/20260101_000000_20260101_235959_ch1.csv
    """
    s = chunk_start_dt.strftime('%Y%m%d_%H%M%S')
    e = chunk_end_dt.strftime('%Y%m%d_%H%M%S')
    return os.path.join(folder, f'{s}_{e}_ch{channel}.csv')


def _chunk_is_complete(folder, chunk_start_dt, chunk_end_dt):
    """Return True if all 6 channel CSV files exist for this chunk."""
    return all(
        os.path.isfile(_chunk_csv_path(folder, chunk_start_dt, chunk_end_dt, c))
        for c in range(1, 7)
    )


def write_chunked_spectrograms(folder, spectrograms, chunk_start_dt, chunk_end_dt):
    """Atomically write 6 channel CSVs for a single time chunk."""
    for c, spec in enumerate(spectrograms, start=1):
        path = _chunk_csv_path(folder, chunk_start_dt, chunk_end_dt, c)
        tmp = path + '.tmp'
        np.savetxt(tmp, spec, delimiter=',')
        os.replace(tmp, path)


def _load_and_concat_chunks(folder, chunk_list):
    """Load and concatenate chunked spectrogram CSVs along the time axis.

    chunk_list: list of (start_dt, end_dt) datetime pairs, sorted by start_dt.
    Returns a 6-element list of combined spectrogram arrays (same format as
    process_bin_files output), or None if no data could be loaded.
    """
    if not chunk_list:
        return None

    combined = [None] * 6
    for chunk_start_dt, chunk_end_dt in chunk_list:
        for c in range(1, 7):
            path = _chunk_csv_path(folder, chunk_start_dt, chunk_end_dt, c)
            if not os.path.isfile(path):
                continue
            try:
                arr = np.loadtxt(path, delimiter=',')
            except Exception as e:
                print(f"  WARNING: could not load {path} ({e}) — skipping")
                continue
            idx = c - 1
            if combined[idx] is None:
                combined[idx] = arr
            else:
                # Concatenate along time axis: keep col-0 (freq axis) from the
                # existing data, append all columns except col-0 from new chunk.
                combined[idx] = np.concatenate([combined[idx], arr[:, 1:]], axis=1)

    loaded = [s for s in combined if s is not None]
    if not loaded:
        return None

    # Fill missing channels with NaN arrays matching the first loaded shape
    ref = loaded[0]
    blank = np.full_like(ref, np.nan)
    blank[0, 1:] = ref[0, 1:]
    blank[1:, 0] = ref[1:, 0]
    return [s if s is not None else blank.copy() for s in combined]


# Regex matching batch-pipeline daily spectrogram filenames: YYYYMMDD_chN.csv
_BATCH_FILENAME_RE = re.compile(r'^(\d{8})_ch(\d+)\.csv$')


def _load_and_concat_batch_days(folder, dates):
    """Load and concatenate batch-pipeline per-day spectrogram CSVs along the time axis.

    dates: sorted list of datetime.date objects.
    Files are expected to be named YYYYMMDD_ch1.csv … YYYYMMDD_ch6.csv.
    Returns a 6-element list of combined spectrogram arrays (packed format:
    row-0 = timestamps, col-0 = frequencies), or None if no data could be loaded.
    """
    if not dates:
        return None

    combined = [None] * 6
    for day in dates:
        date_str = day.strftime('%Y%m%d')
        for c in range(1, 7):
            path = os.path.join(folder, f"{date_str}_ch{c}.csv")
            if not os.path.isfile(path):
                continue
            try:
                arr = np.loadtxt(path, delimiter=',')
            except Exception as e:
                print(f"  WARNING: could not load {path} ({e}) — skipping")
                continue
            idx = c - 1
            if combined[idx] is None:
                combined[idx] = arr
            else:
                combined[idx] = np.concatenate([combined[idx], arr[:, 1:]], axis=1)

    loaded = [s for s in combined if s is not None]
    if not loaded:
        return None

    ref = loaded[0]
    blank = np.full_like(ref, np.nan)
    blank[0, 1:] = ref[0, 1:]
    blank[1:, 0] = ref[1:, 0]
    return [s if s is not None else blank.copy() for s in combined]


def convert_size(size_bytes):
   if size_bytes == 0:
       return "0B"
   size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
   i = int(math.floor(math.log(size_bytes, 1024)))
   p = math.pow(1024, i)
   s = round(size_bytes / p, 2)
   return "%s %s" % (s, size_name[i])

def check_overlap(start, end, fstart, fend):
    # Convert QDateTime to pd.Timestamp
    start_ts = pd.Timestamp(start.toSecsSinceEpoch()+ start.offsetFromUtc(), unit='s')
    end_ts = pd.Timestamp(end.toSecsSinceEpoch()+ end.offsetFromUtc(), unit='s')
    # Check if there's an overlap
    if start_ts <= fend and fstart <= end_ts:
        return True  # Overlap exists
    else:
        return False  # No overlap


def parse_freq_bands(text):
    """Parse '10-50 100-200' or '10-50,100-200' into [(10.0,50.0),(100.0,200.0)]."""
    bands = []
    for m in re.finditer(r'(\d+(?:\.\d+)?)\s*[-\u2013]\s*(\d+(?:\.\d+)?)', text):
        lo, hi = float(m.group(1)), float(m.group(2))
        if lo < hi:
            bands.append((lo, hi))
    return bands


def export_band_summary(spectrograms, freq_bands, folder):
    """For each (time, channel), compute average power in each user-defined frequency
    band plus an overall average.  Writes FrequencyBandSummary.csv and returns the path.

    CSV columns: time, channel, band_<lo>_<hi>_Hz, ..., average_power
    """
    ref = next((s for s in spectrograms if s is not None), None)
    if ref is None:
        return None

    timestamps = ref[0, 1:]   # unix epoch seconds
    freqs      = ref[1:, 0]   # Hz values

    band_names = [f"band_{int(lo)}_{int(hi)}_Hz" for lo, hi in freq_bands]
    columns    = ['time', 'channel'] + band_names + ['average_power']

    rows = []
    for c, spec in enumerate(spectrograms):
        if spec is None:
            continue
        power = spec[1:, 1:]   # shape (n_freq, n_time)
        for t_idx, ts in enumerate(timestamps):
            col = power[:, t_idx]
            row = [
                pd.Timestamp(ts, unit='s').strftime('%Y-%m-%d %H:%M:%S'),
                c + 1,
            ]
            for lo, hi in freq_bands:
                mask  = (freqs >= lo) & (freqs <= hi)
                valid = col[mask & np.isfinite(col)]
                row.append(float(valid.mean()) if valid.size > 0 else np.nan)
            valid_all = col[np.isfinite(col)]
            row.append(float(valid_all.mean()) if valid_all.size > 0 else np.nan)
            rows.append(row)

    df = pd.DataFrame(rows, columns=columns)
    out_path = os.path.join(folder, 'FrequencyBandSummary.csv')
    df.to_csv(out_path, index=False)
    return out_path


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SpectrogramApp()
    window.show()
    sys.exit(app.exec_())


##########################################################################################################################
##########################################################################################################################
##########################################################################################################################
##########################################################################################################################
'''
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy

### offline stuff
spectrograms = []
for i in range(6):
    print(i)
    print(f"/Users/dcum007/Documents/Circadian/BeeSpy/Auckland Data/20250224/Spectrogram_{i+1}.csv")
    spectrograms.append(pd.read_csv(f"/Users/dcum007/Documents/Circadian/BeeSpy/Auckland Data/20250224/Spectrogram_{i+1}.csv", delimiter=',', header=None))

    
#do biunary
binary_by_freq = copy.deepcopy(spectrograms)
for c in range(len(binary_by_freq)):
    for f in range(binary_by_freq[c].shape[1]-1):
        binary_by_freq[c].loc[1:,f+1] = (spectrograms[c].loc[1:,f+1] > np.percentile(spectrograms[c].loc[1:, f+1], 95)).astype(float)
    
for c in range(len(binary_by_freq)):
    binary_by_freq[c] = (spectrograms[c] > -5.0).astype(float)

################################################################################
## PLOT
fig, axs = plt.subplots(2, 3, figsize=(20, 10))
axs = axs.flatten()
i = 0
for ax, spectrogram in zip(axs, binary_by_freq):
    #ax.imshow(spectrogram.loc[1:101, 1:], aspect='auto', cmap='jet', origin='lower')
    ax.imshow(pd.DataFrame(spectrogram.loc[1:, 1:]).fillna(0), aspect='auto', origin='lower')
    ax.set_title(f"Channel {i + 1}")

    time_stamps = pd.to_datetime(spectrogram.iloc[0, 1:], unit='s')
    num_ticks_x = 2 
    tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=num_ticks_x, dtype=int)
    tick_labels_x = time_stamps[tick_indices_x+1]
    # Apply tick positions and labels to the x-axis
    ax.set_xticks(tick_indices_x)
    ax.set_xticklabels(tick_labels_x, ha='center') 

    # Apply tick positions and labels to the y-axis
    freqVals = spectrogram.loc[1:101:,0]
    num_ticks_y = 5
    tick_indices_y = np.linspace(0, len(freqVals) - 1, num=num_ticks_y, dtype=int)
    tick_labels_y = freqVals[tick_indices_y+1]
    ax.set_yticks(tick_indices_y)
    ax.set_yticklabels(tick_labels_y, ha='center')
    
    # Remove y-axis labels/ticks for the second and third columns
    if i % 3 != 0:  # Second and third columns
        ax.yaxis.set_visible(False)

    # Remove x-axis labels/ticks for all but the last row (bottom row)
    if i < 3:  # Top row
        ax.xaxis.set_visible(False)

    i += 1

plt.show()


################################################################################
################################################################################
## do the rough calc for high freq < 40Hz
fig, axs = plt.subplots(2, 3, figsize=(20, 10))
axs = axs.flatten()
i = 0
for ax, spectrogram in zip(axs, spectrograms):
    freqofinterest = np.sum(spectrogram.loc[1:60, 1:], axis=0)
    ax.plot(freqofinterest)
    
    ax.set_title(f"Channel {i + 1}")

    time_stamps = pd.to_datetime(spectrogram.iloc[0, 1:], unit='s')
    num_ticks_x = 2 
    tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=num_ticks_x, dtype=int)
    tick_labels_x = time_stamps[tick_indices_x+1]
    # Apply tick positions and labels to the x-axis
    ax.set_xticks(tick_indices_x)
    ax.set_xticklabels(tick_labels_x, ha='center') 

    # Apply tick positions and labels to the y-axis
    freqVals = spectrogram.loc[1:101:,0]
    num_ticks_y = 5
    tick_indices_y = np.linspace(0, len(freqVals) - 1, num=num_ticks_y, dtype=int)
    tick_labels_y = freqVals[tick_indices_y+1]
    ax.set_yticks(tick_indices_y)
    ax.set_yticklabels(tick_labels_y, ha='center')
    
    # Remove y-axis labels/ticks for the second and third columns
    if i % 3 != 0:  # Second and third columns
        ax.yaxis.set_visible(False)

    # Remove x-axis labels/ticks for all but the last row (bottom row)
    if i < 3:  # Top row
        ax.xaxis.set_visible(False)

    i += 1

plt.show()

##############################################################################
##############################################################################
## do a plot over time of just the high intensity freqs

#make an empty array the length of spectrograms[0].shape[1]

combined = np.zeros(spectrograms[0].shape[1]-1)
allbyfreq = np.zeros((spectrograms[0].shape[0]-1, spectrograms[0].shape[1]-1))
for i in range(6):
    combined = combined + np.sum(binary_by_freq[i].loc[1:, 1:], axis=0)
    allbyfreq = allbyfreq + binary_by_freq[i].loc[1:, 1:].values

#plot
plt.subplot(3,1,1)
#set upp the x-axis times
time_stamps = pd.to_datetime(spectrograms[0].iloc[0, 1:], unit='s')
num_ticks_x = 7
tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=num_ticks_x, dtype=int)
tick_labels_x = time_stamps[tick_indices_x+1]


plt.imshow(allbyfreq, aspect='auto', origin='lower')
plt.xticks(tick_indices_x, tick_labels_x, ha='center')

plt.subplot(3,1,2)
plt.plot(combined)
plt.xticks(tick_indices_x, tick_labels_x, ha='center')

plt.subplot(3,1,3)
## now add a moving average filter over combined
combined_ma = np.convolve(combined, np.ones(5)/5, mode='same')
plt.plot(combined_ma)
plt.xticks(tick_indices_x, tick_labels_x, ha='center')

plt.show()
 
'''