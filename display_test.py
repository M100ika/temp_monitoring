"""
Nextion HMI Simulator - Thermal Conductivity Measurement System
Имитация данных ESP32 для тестирования дисплея Nextion

Использование:
  python nextion_simulator.py
"""

import serial
import serial.tools.list_ports
import time
import random
import threading
import sys

# ============================================================
# НАСТРОЙКИ
# ============================================================
COM_PORT = "/dev/ttyUSB0"        # <<< ИЗМЕНИТЕ НА СВОЙ COM ПОРТ
BAUD_RATE = 115200
REFRESH_MS = 500

# ============================================================
# NEXTION PROTOCOL
# ============================================================
TERMINATOR = b'\xff\xff\xff'

def ncmd(ser, cmd):
    """Send command to Nextion"""
    try:
        data = cmd.encode('utf-8', errors='replace') + TERMINATOR
        ser.write(data)
    except Exception as e:
        pass  # Silent fail to avoid spam

def nset_var(ser, var, val):
    """Глобальная переменная — БЕЗ .val"""
    ncmd(ser, f"{var}={val}")

def nset_int(ser, comp, val):
    """Числовой компонент — С .val"""
    ncmd(ser, f"{comp}.val={val}")

def nset_txt(ser, comp, txt):
    """Set text component"""
    escaped = txt.replace('"', '\\"')
    ncmd(ser, f'{comp}.txt="{escaped}"')

def nset_both(ser, var_name, txt_comp, val):
    """Set variable AND text component directly"""
    nset_int(ser, var_name, val)
    nset_txt(ser, txt_comp, str(val))

# ============================================================
# SENSOR SIMULATOR
# ============================================================

class SensorSimulator:
    def __init__(self):
        self.temps = [25.0, 30.0, 22.0, 28.0, 45.0]
        self.targets = [450.0, 320.0, 280.0, 510.0, 890.0]
        self.tmin = [1200, 1200, 1200, 1200, 1200]
        self.tmax = [0, 0, 0, 0, 0]
        self.rates = [0, 0, 0, 0, 0]
        self.prev_temps = [int(round(t)) for t in self.temps]
        self.heater = 0
        self.phase = 0
        self.tick = 0

    def update(self):
        self.tick += 1
        self.prev_temps = [int(round(t)) for t in self.temps]

        if self.tick < 30:
            self.phase = 0
            self.heater = 1
        elif self.tick < 60:
            self.phase = 1
            self.heater = 1
        elif self.tick == 60:
            self.phase = 2
            self.heater = 2
        elif self.tick == 65:
            self.phase = 3
            self.heater = 0
        elif self.tick > 95:
            self.tick = 0
            self.phase = 0
            self.heater = 1
            for i in range(5):
                self.temps[i] = random.uniform(20, 35)
                self.tmin[i] = 1200
                self.tmax[i] = 0

        for i in range(5):
            if self.phase == 0:
                diff = self.targets[i] - self.temps[i]
                self.temps[i] += diff * 0.05 + random.uniform(-2, 2)
            elif self.phase == 1:
                self.temps[i] += random.uniform(-5, 5)
            elif self.phase == 2:
                if i == 4:
                    self.temps[i] += random.uniform(5, 15)
                else:
                    self.temps[i] += random.uniform(-3, 3)
            elif self.phase == 3:
                self.temps[i] -= random.uniform(2, 8)

            self.temps[i] = max(0, min(1200, self.temps[i]))
            val = int(round(self.temps[i]))

            if val < self.tmin[i]:
                self.tmin[i] = val
            if val > self.tmax[i]:
                self.tmax[i] = val

            self.rates[i] = (val - self.prev_temps[i]) * 2


class ErrorLogSimulator:
    ERROR_CODES = [
        ("ERR-01", "Over threshold T1"),
        ("ERR-01", "Over threshold T2"),
        ("ERR-02", "Sensor T3 fault"),
        ("ERR-03", "Comm timeout"),
        ("ERR-04", "Heater overload"),
        ("ERR-05", "ADC overflow"),
    ]

    def __init__(self):
        self.entries = [""] * 10
        self.count = 0

    def add_entry(self):
        code, desc = random.choice(self.ERROR_CODES)
        hour = random.randint(0, 23)
        minute = random.randint(0, 59)
        second = random.randint(0, 59)
        timestamp = f"{hour:02d}:{minute:02d}:{second:02d}"
        entry = f"{code}|{timestamp}|{desc}"

        for i in range(9, 0, -1):
            self.entries[i] = self.entries[i - 1]
        self.entries[0] = entry
        self.count += 1
        return self.entries.copy()


# ============================================================
# SEND DATA TO NEXTION
# ============================================================

def send_sensor_data(ser, sim):
    for i in range(5):
        val = int(round(sim.temps[i]))
        n = i + 1
        nset_var(ser, f"t{n}_val", val)      # глобальная переменная
        nset_var(ser, f"t{n}_min", sim.tmin[i])
        nset_var(ser, f"t{n}_max", sim.tmax[i])
        nset_var(ser, f"t{n}_rate", sim.rates[i])
    nset_var(ser, "heater_status", sim.heater)

def send_system_info(ser):
    """Send system info for Page 4"""
    nset_txt(ser, "espStatus", "Connected")
    nset_txt(ser, "wifiStatus", "Connected")
    nset_txt(ser, "ssidVal", "Nazarbayev_Uni")
    nset_txt(ser, "ipAddr", "IP:192.168.1.42")
    nset_txt(ser, "macAddr", "MAC:A4:CF:12:34:56:78")


def send_error_log(ser, log_sim):
    """Send error log for Page 5"""
    entries = log_sim.add_entry()
    for i in range(10):
        nset_txt(ser, f"log{i+1}", entries[i])
    nset_int(ser, "errCount", log_sim.count)
    ncmd(ser, "cov errCount,errCnt.txt,0")
    print(f"  [LOG] Error #{log_sim.count}: {entries[0]}")


# ============================================================
# LISTEN FOR NEXTION COMMANDS
# ============================================================

def listen_nextion(ser):
    """Thread to read commands from Nextion"""
    buffer = ""
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                buffer += data
                while '\xff\xff\xff' in buffer:
                    cmd, _, buffer = buffer.partition('\xff\xff\xff')
                    cmd = cmd.strip('\x00').strip()
                    if cmd:
                        handle_command(ser, cmd)
            time.sleep(0.01)
        except:
            time.sleep(0.1)


def handle_command(ser, cmd):
    """Handle command from Nextion"""
    print(f"\n  [RECV] '{cmd}'")
    if cmd == "WIFI_CFG":
        nset_txt(ser, "espStatus", "Config...")
        nset_txt(ser, "wifiStatus", "Config...")
        time.sleep(5)
        nset_txt(ser, "espStatus", "Connected")
        nset_txt(ser, "wifiStatus", "Connected")
        nset_txt(ser, "ssidVal", "New_WiFi")
        nset_txt(ser, "ipAddr", "IP:192.168.1.100")
    elif cmd == "LOG_CLEAR":
        print("  Log cleared")
    elif cmd.startswith("REFRESH_"):
        print(f"  Refresh rate: {cmd[8:]}ms")


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("  Nextion HMI Simulator - Thermal Conductivity System")
    print("  Nazarbayev University")
    print("=" * 60)
    print(f"  COM Port: {COM_PORT}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Refresh: {REFRESH_MS}ms")
    print("=" * 60)

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"\n  [OK] Connected to {COM_PORT}")
    except Exception as e:
        print(f"\n  [ERROR] Cannot open {COM_PORT}: {e}")
        print(f"  Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"    - {p.device}: {p.description}")
        sys.exit(1)

    time.sleep(1)

    sim = SensorSimulator()
    log_sim = ErrorLogSimulator()

    # Start listener thread
    listener = threading.Thread(target=listen_nextion, args=(ser,), daemon=True)
    listener.start()

    # Send system info
    send_system_info(ser)

    # Go to Page 1
    time.sleep(0.5)
    ncmd(ser, "page 1")
    print("  [NAV] Switched to Page 1")

    tick_count = 0
    error_interval = 40
    sysinfo_interval = 100

    print("\n  Simulation started! Press Ctrl+C to stop.\n")
    print("  " + "-" * 56)
    print(f"  {'Tick':>4} | {'T1':>5} | {'T2':>5} | {'T3':>5} | {'T4':>5} | {'T5':>5} | Heater")
    print("  " + "-" * 56)

    try:
        while True:
            tick_count += 1
            sim.update()
            send_sensor_data(ser, sim)

            t = [int(round(v)) for v in sim.temps]
            h_names = {0: "IDLE  ", 1: "ACTIVE", 2: "ERROR "}
            p_names = {0: "HEATING", 1: "STABLE ", 2: "OVERHEAT", 3: "COOLING"}
            print(f"  {tick_count:>4} | {t[0]:>5} | {t[1]:>5} | {t[2]:>5} | {t[3]:>5} | {t[4]:>5} | {h_names[sim.heater]} | {p_names[sim.phase]}")

            if tick_count % error_interval == 0:
                send_error_log(ser, log_sim)

            if tick_count % sysinfo_interval == 0:
                send_system_info(ser)

            time.sleep(REFRESH_MS / 1000.0)

    except KeyboardInterrupt:
        print("\n\n  [STOP] Simulation stopped")
    finally:
        ser.close()
        print("  [OK] Serial port closed")


if __name__ == "__main__":
    main()