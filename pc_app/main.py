"""
Temperature Monitoring Desktop App
Stack : Python 3.12, PySide6, pyqtgraph, pyserial
Input : Serial JSON @ 115200 baud
Format: {"timestamp":12345,"top":[T1,T2,T3,T4],"bottom":[T5],"status":"OK"}
"""

import sys
import json
import time
import csv
import threading
from datetime import datetime
from pathlib import Path
from collections import deque

import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTableWidget, QTableWidgetItem,
    QGroupBox, QSplitter, QStatusBar, QFileDialog, QHeaderView,
)
from PySide6.QtGui import QColor, QFont

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


# ─────────────────────────────────────────────────────────────
# Serial reader thread – emits parsed packets via Qt signal
# ─────────────────────────────────────────────────────────────
class SerialSignals(QObject):
    packet  = Signal(dict)
    error   = Signal(str)
    stopped = Signal()


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int):
        super().__init__(daemon=True)
        self.port    = port
        self.baud    = baud
        self.signals = SerialSignals()
        self._stop   = threading.Event()

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            self.signals.error.emit(str(e))
            return

        try:
            while not self._stop.is_set():
                line = ser.readline()
                if not line:
                    continue
                try:
                    data = json.loads(line.decode("utf-8", errors="replace").strip())
                    data["_rx_time"] = time.time()
                    self.signals.packet.emit(data)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass
        finally:
            ser.close()
            self.signals.stopped.emit()

    def stop(self):
        self._stop.set()


# ─────────────────────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Temperature Monitor — ESP32-C6")
        self.resize(1280, 800)

        # Data buffers  (deque, length MAX_POINTS)
        self._times  = deque(maxlen=MAX_POINTS)
        self._temps  = [deque(maxlen=MAX_POINTS) for _ in range(5)]
        self._t0     = None          # first packet rx time for x-axis

        # Serial / logging state
        self._reader    : SerialReader | None = None
        self._csv_file  = None
        self._csv_writer = None
        self._logging   = False
        self._pkt_count = 0

        self._build_ui()
        self._refresh_ports()

    # ── UI construction ──────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(4)

        # ── Top toolbar ──────────────────────────────────────
        toolbar = QHBoxLayout()
        root.addLayout(toolbar)

        toolbar.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        toolbar.addWidget(self._port_combo)

        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.clicked.connect(self._refresh_ports)
        toolbar.addWidget(self._btn_refresh)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.clicked.connect(self._toggle_connect)
        self._btn_connect.setCheckable(True)
        toolbar.addWidget(self._btn_connect)

        toolbar.addSpacing(20)

        self._btn_log = QPushButton("Start Logging")
        self._btn_log.setEnabled(False)
        self._btn_log.clicked.connect(self._toggle_logging)
        self._btn_log.setCheckable(True)
        toolbar.addWidget(self._btn_log)

        toolbar.addStretch()

        self._lbl_status = QLabel("Disconnected")
        self._lbl_status.setFont(QFont("Monospace", 10))
        toolbar.addWidget(self._lbl_status)

        # ── Splitter: graphs (top) / table (bottom) ──────────
        splitter = QSplitter(Qt.Vertical)
        root.addWidget(splitter, stretch=1)

        # ── Graphs ───────────────────────────────────────────
        graph_widget = QWidget()
        graph_layout = QVBoxLayout(graph_widget)
        graph_layout.setContentsMargins(0, 0, 0, 0)
        graph_layout.setSpacing(2)
        splitter.addWidget(graph_widget)

        pg.setConfigOption("background", "#1e1e1e")
        pg.setConfigOption("foreground", "#cccccc")

        self._plots : list[pg.PlotItem]   = []
        self._curves: list[pg.PlotDataItem] = []

        for i, (label, color) in enumerate(zip(SENSOR_LABELS, COLORS)):
            pw = pg.PlotWidget()
            pw.setMinimumHeight(100)
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setLabel("left", label, units="°C")
            if i == len(SENSOR_LABELS) - 1:
                pw.setLabel("bottom", "Time", units="s")
            pw.setYRange(-10, 150, padding=0.05)

            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            self._plots.append(pw)
            self._curves.append(curve)
            graph_layout.addWidget(pw)

        # ── Data table ───────────────────────────────────────
        table_group = QGroupBox("Current Values")
        table_layout = QVBoxLayout(table_group)
        splitter.addWidget(table_group)

        self._table = QTableWidget(1, 8)
        self._table.setHorizontalHeaderLabels(
            ["Timestamp (s)", "T1 (°C)", "T2 (°C)", "T3 (°C)", "T4 (°C)",
             "T5 (°C)", "Status", "Packets/s"])
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.setFixedHeight(80)
        self._table.verticalHeader().setVisible(False)
        table_layout.addWidget(self._table)

        splitter.setSizes([650, 150])

        # ── Status bar ───────────────────────────────────────
        self.setStatusBar(QStatusBar())

        # ── Packet rate timer ────────────────────────────────
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._calc_rate)
        self._rate_timer.start(1000)
        self._rate_count_last = 0

    # ── Port management ──────────────────────────────────────
    def _refresh_ports(self):
        self._port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports):
            self._port_combo.addItem(f"{p.device}  —  {p.description}", p.device)
        if not ports:
            self._port_combo.addItem("No ports found")

    def _toggle_connect(self, checked: bool):
        if checked:
            self._start_serial()
        else:
            self._stop_serial()

    def _start_serial(self):
        idx  = self._port_combo.currentIndex()
        port = self._port_combo.itemData(idx)
        if not port:
            self._btn_connect.setChecked(False)
            return

        self._reader = SerialReader(port, BAUD_RATE)
        self._reader.signals.packet.connect(self._on_packet)
        self._reader.signals.error.connect(self._on_serial_error)
        self._reader.signals.stopped.connect(self._on_serial_stopped)
        self._reader.start()

        self._btn_connect.setText("Disconnect")
        self._btn_log.setEnabled(True)
        self._lbl_status.setText(f"Connected: {port}")
        self.statusBar().showMessage(f"Opened {port} @ {BAUD_RATE}")

    def _stop_serial(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        if self._logging:
            self._stop_logging()
        self._btn_connect.setText("Connect")
        self._btn_connect.setChecked(False)
        self._btn_log.setEnabled(False)
        self._lbl_status.setText("Disconnected")

    def _on_serial_error(self, msg: str):
        self._stop_serial()
        self.statusBar().showMessage(f"Serial error: {msg}", 5000)

    def _on_serial_stopped(self):
        self._stop_serial()

    # ── Packet processing ────────────────────────────────────
    def _on_packet(self, data: dict):
        rx = data.get("_rx_time", time.time())
        if self._t0 is None:
            self._t0 = rx
        t = rx - self._t0

        top    = data.get("top",    [None, None, None, None])
        bottom = data.get("bottom", [None])
        status = data.get("status", "?")

        # Ensure lists are long enough
        while len(top)    < 4: top.append(None)
        while len(bottom) < 1: bottom.append(None)

        raw_vals = [top[0], top[1], top[2], top[3], bottom[0]]

        # Append to buffers
        self._times.append(t)
        for i, v in enumerate(raw_vals):
            self._temps[i].append(float(v) if v is not None else float("nan"))

        # Update graphs
        xs = list(self._times)
        for i, curve in enumerate(self._curves):
            curve.setData(xs, list(self._temps[i]))

        # Update table
        def fmt(v): return f"{v:.2f}" if v is not None else "—"
        row_data = [
            f"{t:.1f}",
            fmt(top[0]), fmt(top[1]), fmt(top[2]), fmt(top[3]),
            fmt(bottom[0]),
            status,
            "",
        ]
        for col, txt in enumerate(row_data):
            item = QTableWidgetItem(txt)
            item.setTextAlignment(Qt.AlignCenter)
            if col == 6:  # status cell
                item.setForeground(QColor("lime") if status == "OK"
                                   else QColor("tomato"))
            self._table.setItem(0, col, item)

        # CSV logging
        if self._logging and self._csv_writer:
            ts_str = datetime.fromtimestamp(rx).isoformat(timespec="milliseconds")
            self._csv_writer.writerow([
                ts_str,
                top[0], top[1], top[2], top[3],
                bottom[0],
                status,
            ])

        self._pkt_count += 1

    # ── Logging ──────────────────────────────────────────────
    def _toggle_logging(self, checked: bool):
        if checked:
            self._start_logging()
        else:
            self._stop_logging()

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

    # ── Packet rate ──────────────────────────────────────────
    def _calc_rate(self):
        rate = self._pkt_count - self._rate_count_last
        self._rate_count_last = self._pkt_count
        if self._table.item(0, 7) is not None:
            self._table.item(0, 7).setText(f"{rate} pkt/s")
        else:
            item = QTableWidgetItem(f"{rate} pkt/s")
            item.setTextAlignment(Qt.AlignCenter)
            self._table.setItem(0, 7, item)

    # ── Clean up on close ────────────────────────────────────
    def closeEvent(self, event):
        self._stop_serial()
        super().closeEvent(event)


# ─────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    from PySide6.QtGui import QPalette
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(30, 30, 30))
    pal.setColor(QPalette.WindowText,      QColor(220, 220, 220))
    pal.setColor(QPalette.Base,            QColor(20, 20, 20))
    pal.setColor(QPalette.AlternateBase,   QColor(40, 40, 40))
    pal.setColor(QPalette.Text,            QColor(220, 220, 220))
    pal.setColor(QPalette.Button,          QColor(50, 50, 50))
    pal.setColor(QPalette.ButtonText,      QColor(220, 220, 220))
    pal.setColor(QPalette.Highlight,       QColor(42, 130, 218))
    pal.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(pal)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())
