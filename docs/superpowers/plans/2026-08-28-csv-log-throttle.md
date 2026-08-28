# CSV Log Throttle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator running `pc_app` choose how often incoming packets get written to the CSV log (every packet / 1s / 2s / 5s / 10s), without touching firmware or restructuring the app.

**Architecture:** All changes live in `pc_app/main.py`, a single-file PySide6 desktop app with no existing test harness. We add one pure helper function that decides whether a given packet should be written (easy to verify standalone), one new `QComboBox` in the toolbar, and two small edits to the existing `_start_logging` / `_stop_logging` / `_on_packet` methods to read the chosen interval and gate the `csv_writer.writerow(...)` call. Live graphs, the data table, and the packets/s counter are untouched — they keep updating on every packet.

**Tech Stack:** Python 3.12, PySide6 (Qt widgets), existing venv at `pc_app/venv`.

## Global Constraints

- Only CSV row writing is throttled; graphs, table, and packets/s counter update on every packet regardless of the chosen interval. (spec: "Live view ... is unaffected")
- The interval combo box is enabled only while logging is stopped, and disabled the moment `Start Logging` is pressed. (spec: "the interval cannot change mid-log")
- Default selection on app start is `1 с` (1 second). (spec default)
- Combo box items, in order: `Каждый пакет`(0s) / `1 с`(1) / `2 с`(2) / `5 с`(5) / `10 с`(10). (spec table)
- No persistence of the chosen interval across app restarts, no config file, no env var. (spec: "Out of scope")
- No new third-party dependency — `pc_app/requirements.txt` stays as-is.

---

## File Structure

Single file touched:
- `pc_app/main.py` — add a constants list (`LOG_INTERVALS`), a pure helper function (`_should_write_csv_row`), a new `QComboBox` in `_build_ui`, new instance state in `__init__`, and edits to `_start_logging`, `_stop_logging`, `_on_packet`.

No new files. This app has no `tests/` directory and no test framework in `requirements.txt`; verification below uses plain `assert` scripts run through the project's own venv interpreter (`pc_app/venv/bin/python`), matching the manual-verification approach the spec calls for. Headless imports of `main.py` need `QT_QPA_PLATFORM=offscreen` since there's no display in this environment — Qt widget classes can be *defined* without it, but the env var keeps `python -c "import main"` safe everywhere.

---

### Task 1: Add throttle-decision constants and pure helper function

**Files:**
- Modify: `pc_app/main.py:28-40` (constants block)
- Modify: `pc_app/main.py:82-84` (insert helper function between `SerialReader` class and the `# Main Window` header comment)

**Interfaces:**
- Produces: `LOG_INTERVALS: list[tuple[str, int]]` — `(label, seconds)` pairs, `0` meaning "every packet". Consumed by Task 2 (combo box population) and Task 3 (default index lookup).
- Produces: `_should_write_csv_row(interval_s: float, last_write: float | None, rx: float) -> bool`. Consumed by Task 3 (`_on_packet` gating).

- [ ] **Step 1: Write a standalone verification script for the not-yet-existing helper**

Create `/tmp/claude-1000/-home-maxat-Projects-NU-Temp-monitoring/5207a054-fbc0-481b-95ef-713ce7329528/scratchpad/test_throttle.py`:

```python
import sys
sys.path.insert(0, "/home/maxat/Projects/NU/Temp_monitoring/pc_app")
import main

# interval_s == 0 -> always write
assert main._should_write_csv_row(0, None, 100.0) is True
assert main._should_write_csv_row(0, 99.9, 100.0) is True

# first row after (re)start -> always write, regardless of interval
assert main._should_write_csv_row(5, None, 100.0) is True

# elapsed < interval -> don't write
assert main._should_write_csv_row(5, 100.0, 102.0) is False

# elapsed == interval -> write (boundary is inclusive)
assert main._should_write_csv_row(5, 100.0, 105.0) is True

# elapsed > interval -> write
assert main._should_write_csv_row(5, 100.0, 108.0) is True

print("ALL PASS")
```

- [ ] **Step 2: Run it, confirm it fails because the function doesn't exist yet**

Run:
```bash
cd /home/maxat/Projects/NU/Temp_monitoring/pc_app && QT_QPA_PLATFORM=offscreen venv/bin/python /tmp/claude-1000/-home-maxat-Projects-NU-Temp-monitoring/5207a054-fbc0-481b-95ef-713ce7329528/scratchpad/test_throttle.py
```
Expected: `AttributeError: module 'main' has no attribute '_should_write_csv_row'`

- [ ] **Step 3: Add `LOG_INTERVALS` next to the existing constants**

In `pc_app/main.py`, the constants block currently reads (lines 28-40):

```python
# ─────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────
BAUD_RATE      = 115200
MAX_POINTS     = 300        # number of samples visible on graphs
SENSOR_LABELS  = ["T1 (Top)", "T2 (Top)", "T3 (Top)", "T4 (Top)", "T5 (Bot)"]
COLORS = [
    (255, 80,  80),   # T1 – red
    (80,  200, 80),   # T2 – green
    (80,  150, 255),  # T3 – blue
    (255, 200, 60),   # T4 – yellow
    (200, 80,  255),  # T5 – purple
]
```

Replace it with (adds `LOG_INTERVALS` at the end, everything above unchanged):

```python
# ─────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────
BAUD_RATE      = 115200
MAX_POINTS     = 300        # number of samples visible on graphs
SENSOR_LABELS  = ["T1 (Top)", "T2 (Top)", "T3 (Top)", "T4 (Top)", "T5 (Bot)"]
COLORS = [
    (255, 80,  80),   # T1 – red
    (80,  200, 80),   # T2 – green
    (80,  150, 255),  # T3 – blue
    (255, 200, 60),   # T4 – yellow
    (200, 80,  255),  # T5 – purple
]

# CSV log write interval: (label shown in the combo box, seconds between
# writes). 0 seconds means "write every received packet".
LOG_INTERVALS = [
    ("Каждый пакет", 0),
    ("1 с", 1),
    ("2 с", 2),
    ("5 с", 5),
    ("10 с", 10),
]
```

- [ ] **Step 4: Add the pure helper function after `SerialReader`**

In `pc_app/main.py`, immediately before the `# Main Window` section header (currently lines 86-89):

```python
# ─────────────────────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
```

Insert a new block above it, so the file reads:

```python
def _should_write_csv_row(interval_s: float, last_write: float | None, rx: float) -> bool:
    """Whether a packet received at time `rx` should be written to the CSV
    log, given the configured `interval_s` and the timestamp of the last
    row actually written (`last_write`, None if none written yet)."""
    if interval_s == 0:
        return True
    if last_write is None:
        return True
    return (rx - last_write) >= interval_s


# ─────────────────────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
```

- [ ] **Step 5: Run the verification script again, confirm it passes**

Run:
```bash
cd /home/maxat/Projects/NU/Temp_monitoring/pc_app && QT_QPA_PLATFORM=offscreen venv/bin/python /tmp/claude-1000/-home-maxat-Projects-NU-Temp-monitoring/5207a054-fbc0-481b-95ef-713ce7329528/scratchpad/test_throttle.py
```
Expected: `ALL PASS`

- [ ] **Step 6: Commit**

```bash
cd /home/maxat/Projects/NU/Temp_monitoring
git add pc_app/main.py
git commit -m "Add CSV log throttle helper and interval constants"
```

---

### Task 2: Add the interval combo box to the toolbar

**Files:**
- Modify: `pc_app/main.py:136-143` (toolbar construction in `_build_ui`)

**Interfaces:**
- Consumes: `LOG_INTERVALS` from Task 1.
- Produces: `self._combo_log_interval: QComboBox` — instance attribute consumed by Task 3 (`_start_logging` reads `currentData()`; `_stop_logging` re-enables it).

- [ ] **Step 1: Locate the insertion point**

In `pc_app/main.py`, the toolbar currently has (lines 136-146):

```python
        toolbar.addSpacing(20)

        self._btn_log = QPushButton("Start Logging")
        self._btn_log.setEnabled(False)
        self._btn_log.clicked.connect(self._toggle_logging)
        self._btn_log.setCheckable(True)
        toolbar.addWidget(self._btn_log)

        self._btn_clear = QPushButton("Clear")
        self._btn_clear.clicked.connect(self._clear_data)
        toolbar.addWidget(self._btn_clear)
```

- [ ] **Step 2: Insert the combo box between the spacing and the Start Logging button**

Replace the block above with:

```python
        toolbar.addSpacing(20)

        toolbar.addWidget(QLabel("Запись:"))
        self._combo_log_interval = QComboBox()
        for label, seconds in LOG_INTERVALS:
            self._combo_log_interval.addItem(label, seconds)
        self._combo_log_interval.setCurrentIndex(1)  # default: "1 с"
        toolbar.addWidget(self._combo_log_interval)

        self._btn_log = QPushButton("Start Logging")
        self._btn_log.setEnabled(False)
        self._btn_log.clicked.connect(self._toggle_logging)
        self._btn_log.setCheckable(True)
        toolbar.addWidget(self._btn_log)

        self._btn_clear = QPushButton("Clear")
        self._btn_clear.clicked.connect(self._clear_data)
        toolbar.addWidget(self._btn_clear)
```

Index `1` in `LOG_INTERVALS` is `("1 с", 1)`, matching the spec's required default.

- [ ] **Step 3: Verify the app still launches headless without errors**

Run:
```bash
cd /home/maxat/Projects/NU/Temp_monitoring/pc_app && QT_QPA_PLATFORM=offscreen venv/bin/python -c "
from PySide6.QtWidgets import QApplication
import main
app = QApplication([])
win = main.MainWindow()
print('combo items:', [win._combo_log_interval.itemText(i) for i in range(win._combo_log_interval.count())])
print('default index:', win._combo_log_interval.currentIndex(), win._combo_log_interval.currentText())
"
```
Expected:
```
combo items: ['Каждый пакет', '1 с', '2 с', '5 с', '10 с']
default index: 1 1 с
```

- [ ] **Step 4: Commit**

```bash
cd /home/maxat/Projects/NU/Temp_monitoring
git add pc_app/main.py
git commit -m "Add CSV log interval combo box to toolbar"
```

---

### Task 3: Wire the combo box into logging start/stop and gate CSV writes

**Files:**
- Modify: `pc_app/main.py:100-105` (instance state in `__init__`)
- Modify: `pc_app/main.py:340-365` (`_start_logging`, `_stop_logging`)
- Modify: `pc_app/main.py:309-318` (CSV-write block inside `_on_packet`)

**Interfaces:**
- Consumes: `self._combo_log_interval` (Task 2), `_should_write_csv_row` (Task 1).
- Produces: `self._log_interval_s: float`, `self._last_csv_write: float | None` — internal state, not consumed elsewhere.

- [ ] **Step 1: Add new instance state in `__init__`**

Current (lines 100-105):

```python
        # Serial / logging state
        self._reader    : SerialReader | None = None
        self._csv_file  = None
        self._csv_writer = None
        self._logging   = False
        self._pkt_count = 0
```

Replace with:

```python
        # Serial / logging state
        self._reader    : SerialReader | None = None
        self._csv_file  = None
        self._csv_writer = None
        self._logging   = False
        self._pkt_count = 0
        self._log_interval_s : float = 0
        self._last_csv_write : float | None = None
```

- [ ] **Step 2: Read the interval and reset throttle state in `_start_logging`; re-enable combo in `_stop_logging`**

Current (lines 340-365):

```python
    def _start_logging(self):
        default_name = datetime.now().strftime("temp_log_%Y%m%d_%H%M%S.csv")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save CSV log", str(Path.home() / default_name),
            "CSV files (*.csv)")
        if not path:
            self._btn_log.setChecked(False)
            return

        self._csv_file   = open(path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ["Timestamp", "T1", "T2", "T3", "T4", "T5", "Status"])
        self._logging = True
        self._btn_log.setText("Stop Logging")
        self.statusBar().showMessage(f"Logging to {path}")

    def _stop_logging(self):
        self._logging = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
        self._btn_log.setText("Start Logging")
        self._btn_log.setChecked(False)
        self.statusBar().showMessage("Logging stopped", 3000)
```

Replace with:

```python
    def _start_logging(self):
        default_name = datetime.now().strftime("temp_log_%Y%m%d_%H%M%S.csv")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save CSV log", str(Path.home() / default_name),
            "CSV files (*.csv)")
        if not path:
            self._btn_log.setChecked(False)
            return

        self._csv_file   = open(path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ["Timestamp", "T1", "T2", "T3", "T4", "T5", "Status"])
        self._log_interval_s = self._combo_log_interval.currentData()
        self._last_csv_write = None
        self._combo_log_interval.setEnabled(False)
        self._logging = True
        self._btn_log.setText("Stop Logging")
        self.statusBar().showMessage(f"Logging to {path}")

    def _stop_logging(self):
        self._logging = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
        self._combo_log_interval.setEnabled(True)
        self._btn_log.setText("Start Logging")
        self._btn_log.setChecked(False)
        self.statusBar().showMessage("Logging stopped", 3000)
```

(`_stop_serial()` already calls `_stop_logging()` when logging is active — see `pc_app/main.py:247-248` — so disconnect and serial-error paths get the combo box re-enabled for free.)

- [ ] **Step 3: Gate the CSV write in `_on_packet`**

Current (lines 309-318):

```python
        # CSV logging
        if self._logging and self._csv_writer:
            ts_str = datetime.fromtimestamp(rx).isoformat(timespec="milliseconds")
            def _cv(v): return "" if v is None else v
            self._csv_writer.writerow([
                ts_str,
                _cv(top[0]), _cv(top[1]), _cv(top[2]), _cv(top[3]),
                _cv(bottom[0]),
                status,
            ])
```

Replace with:

```python
        # CSV logging (throttled by the operator-selected interval)
        if self._logging and self._csv_writer:
            if _should_write_csv_row(self._log_interval_s, self._last_csv_write, rx):
                ts_str = datetime.fromtimestamp(rx).isoformat(timespec="milliseconds")
                def _cv(v): return "" if v is None else v
                self._csv_writer.writerow([
                    ts_str,
                    _cv(top[0]), _cv(top[1]), _cv(top[2]), _cv(top[3]),
                    _cv(bottom[0]),
                    status,
                ])
                self._last_csv_write = rx
```

- [ ] **Step 4: Write a headless script exercising the full gated path**

This simulates packet arrival by calling `_on_packet` directly (no real serial port needed), checking that with a 2-second interval only every-other 1-second-spaced packet gets written.

Run:
```bash
cd /home/maxat/Projects/NU/Temp_monitoring/pc_app && QT_QPA_PLATFORM=offscreen venv/bin/python -c "
from PySide6.QtWidgets import QApplication
import main

app = QApplication([])
win = main.MainWindow()

# Pick '2 с' (index 2 -> ('2 с', 2)) and start logging into a temp file.
win._combo_log_interval.setCurrentIndex(2)
import tempfile, os
fd, path = tempfile.mkstemp(suffix='.csv')
os.close(fd)
win._csv_file = open(path, 'w', newline='', encoding='utf-8')
import csv as csv_mod
win._csv_writer = csv_mod.writer(win._csv_file)
win._csv_writer.writerow(['Timestamp', 'T1', 'T2', 'T3', 'T4', 'T5', 'Status'])
win._log_interval_s = win._combo_log_interval.currentData()
win._last_csv_write = None
win._logging = True
assert win._combo_log_interval.isEnabled(), 'combo should still be enabled here (test bypasses _start_logging UI disable)'

base = 1000.0
for i in range(6):  # rx = 1000, 1001, ..., 1005 (1s apart)
    win._on_packet({'top': [20+i, 21+i, 22+i, 23+i], 'bottom': [24+i], 'status': 'OK', '_rx_time': base + i})

win._csv_file.close()
with open(path) as f:
    rows = f.read().splitlines()
print('row count (incl header):', len(rows))
for r in rows:
    print(r)
os.remove(path)
"
```
Expected: header + rows for `rx = 1000, 1002, 1004` only (3 data rows, 4 lines total including header) — packets at `1001, 1003, 1005` are skipped because less than 2s elapsed since the last write.

- [ ] **Step 5: Run it and confirm actual output matches expected**

(Same command as Step 4.) If the row count or timestamps don't match, re-check Task 1's `_should_write_csv_row` and the `_on_packet` edit in Step 3 before proceeding.

- [ ] **Step 6: Commit**

```bash
cd /home/maxat/Projects/NU/Temp_monitoring
git add pc_app/main.py
git commit -m "Throttle CSV log writes by operator-selected interval"
```

---

### Task 4: Manual end-to-end verification with real hardware (or simulated serial input)

**Files:** none (verification-only task, no code changes)

- [ ] **Step 1: Launch the app normally (not headless) and connect to the gateway**

```bash
cd /home/maxat/Projects/NU/Temp_monitoring/pc_app && venv/bin/python main.py
```
Select the correct COM port, click `Connect`. Confirm the interval combo box shows `1 с` selected by default and is enabled.

- [ ] **Step 2: Verify the combo box locks during logging**

Click `Start Logging`, choose a save path. Confirm the interval combo box becomes disabled (greyed out) immediately. Confirm graphs, table, and `Packets/s` keep updating normally.

- [ ] **Step 3: Verify throttled row count**

Before starting, set the combo to `2 с`. Start logging, let it run for ~10 seconds, click `Stop Logging`. Confirm the combo box re-enables. Open the CSV file and confirm timestamps are roughly 2 seconds apart and the row count is roughly `duration / 2`.

- [ ] **Step 4: Verify "every packet" mode**

Set the combo to `Каждый пакет`, start logging for ~5 seconds, stop. Confirm the row count roughly matches `Packets/s` (shown in the table during the run) × duration — i.e. every packet was written, matching current (pre-change) behavior.

- [ ] **Step 5: Verify combo re-enables on disconnect while logging**

Start logging with any interval, then click `Disconnect` (not `Stop Logging`) while still logging. Confirm logging stops (file closes, `Start Logging` button resets) and the interval combo box re-enables.

No commit for this task — it's manual verification only, confirming Tasks 1-3's changes behave correctly with the real app.

---

## Self-Review Notes

- **Spec coverage:** combo box + items + default (Task 2) ✓; throttle-only-CSV, live view untouched (Task 3 Step 3, only touches the CSV block) ✓; disable/enable tied to start/stop (Task 3 Step 2) ✓; disconnect-while-logging path covered via existing `_stop_serial` → `_stop_logging` call, verified in Task 4 Step 5 ✓; no persistence — confirmed, `_log_interval_s` is a plain instance attribute, never written to disk ✓.
- **Placeholder scan:** none found — every step has literal code/commands and expected output.
- **Type consistency:** `_should_write_csv_row(interval_s, last_write, rx)` signature and `self._log_interval_s` / `self._last_csv_write` names are used identically across Tasks 1 and 3.
