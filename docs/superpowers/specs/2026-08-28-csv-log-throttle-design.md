# CSV log throttle — design spec

Date: 2026-08-28
Component: `pc_app/main.py` (Windows desktop app, PySide6)

## Problem

`pc_app` writes one CSV row per received serial packet. The write rate is
therefore fixed by the gateway firmware's `JSON_PERIOD_MS` (200 ms / 5 Hz),
and cannot be changed without touching and reflashing firmware. The operator
running the app needs to be able to reduce the CSV write rate themselves,
without any code change, while the app is running on a given device.

## Goal

Add an operator-facing control in `pc_app` that sets how often incoming
packets get written to the CSV log. Live view (graphs, table, packets/s)
is unaffected — it always updates on every packet. Only CSV logging is
throttled.

## UI

A `QComboBox` labeled "Запись:" is added to the toolbar, next to the
`Start Logging` button. Items (label → stored interval in seconds):

| Label          | Value (s) |
|----------------|-----------|
| Каждый пакет   | 0         |
| 1 с            | 1         |
| 2 с            | 2         |
| 5 с            | 5         |
| 10 с           | 10        |

Default selection on app start: `1 с`.

The combo box is enabled only while logging is stopped. It is disabled
(`setEnabled(False)`) as soon as `Start Logging` is pressed, and
re-enabled when logging stops (`Stop Logging`, disconnect, or serial
error). This matches the decision that the interval cannot change mid-log
— it's read once when logging starts and fixed for that CSV file.

## Data flow / throttle logic

New instance state:
- `self._log_interval_s: float` — read from the combo box's `currentData()`
  when `Start Logging` is pressed; `0` means "every packet".
- `self._last_csv_write: float | None` — timestamp (`rx` time, same clock
  as used for CSV rows today) of the last row actually written; reset to
  `None` in `_start_logging()`.

In `_on_packet`, the existing CSV-logging block gets a gating condition:

```
if self._logging and self._csv_writer:
    if (self._log_interval_s == 0
            or self._last_csv_write is None
            or rx - self._last_csv_write >= self._log_interval_s):
        # ... existing writerow(...) ...
        self._last_csv_write = rx
```

Everything above this block (buffers, graphs, table, packet counter) is
unchanged and still runs for every packet.

## Interaction with existing controls

- `_start_logging()`: read combo box value into `self._log_interval_s`,
  reset `self._last_csv_write = None`, disable the combo box, then proceed
  with existing file-open logic.
- `_stop_logging()`: re-enable the combo box (mirrors existing enable/
  disable handling for `_btn_log`).
- `_stop_serial()`: already calls `_stop_logging()` when logging is active,
  so the combo box re-enable is covered by the same path — no extra work
  needed there.

## Out of scope

- No persistence of the chosen interval across app restarts (session-only
  setting, per operator's confirmed requirement).
- No config file or environment variable.
- No change to firmware or to the live graph/table update rate.

## Testing

Manual verification (no automated test harness exists for `pc_app`):
1. Connect, select `2 с`, start logging, let it run ~10 s, stop, open CSV
   — rows should be ~5, spaced ~2 s apart by timestamp column.
2. Select `Каждый пакет`, start logging — row count should match packet
   count shown by `Packets/s` × duration.
3. Confirm combo box is disabled while `Start Logging` is active and
   re-enabled after `Stop Logging` and after a serial disconnect while
   logging.
