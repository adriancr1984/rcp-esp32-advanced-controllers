"""
RCP telemetry / auto-test logger for the ESP32-S3 standalone controllers.

Three decoupled threads (matches the paper's telemetry architecture):
  1) Acquisition thread  -> reads the UART in bulk and decodes each batch with
                            VECTORIZED NumPy parsing; pushes samples to a circular
                            buffer (live plot) and to a logging queue.
  2) Logging thread      -> drains the queue and writes to the CSV CONTINUOUSLY
                            (background disk I/O, decoupled from acquisition/render).
  3) Main thread         -> runs the step-reference trajectory (control logic) and
                            the real-time Matplotlib visualization (decimated).

Serial line format expected from the firmware:  ref,pos,ctrl,time_us
"""

import serial
import threading
import queue
import time
import csv
import io
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

# ================= CONFIGURATION =================
SERIAL_PORT   = 'COM13'        # <--- set your port
BAUD_RATE     = 921600
DURATION      = 10.0           # total run time [s] (matches the paper's 10 s)
PLOT_DECIMATION = 20           # plot 1 of every 20 samples (screen only; CSV keeps all)
PLOT_BUFFER_LEN = 2000         # circular buffer length for the live view

# Step-reference trajectory: (time_s, angle_deg)  -> 0 -> 60 -> 100 -> 180
TRAYECTORIA = [
    (0.0,   0.0),
    (1.0,  60.0),
    (4.0, 100.0),
    (7.0, 180.0),
]

CSV_FILENAME = f"Prueba_Control_{datetime.now().strftime('%H%M%S')}.csv"


class RCPTelemetry:
    def __init__(self, port, baudrate):
        self.running   = False      # threads alive
        self.capturing = False      # gate: only log/measure after ENTER
        self.start_time = 0.0
        self.step_index = 0

        self.plot_buffer = deque(maxlen=PLOT_BUFFER_LEN)   # circular buffer (live plot)
        self.log_queue   = queue.Queue()                   # acquisition -> logger
        self.plot_lock   = threading.Lock()
        self._rx = b""                                     # partial-line byte buffer
        self.samples_logged = 0

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            print(f"[OK] Connected to {port}.")
            self.send_setpoint(0.0)                        # safety: hold 0 deg
        except Exception as e:
            print(f"[ERROR] Could not open {port}: {e}")
            raise SystemExit(1)

    # ---------------- serial TX ----------------
    def send_setpoint(self, angle):
        try:
            self.ser.write(f"{angle:.2f}\n".encode())
        except Exception:
            pass

    # ---------------- THREAD 1: acquisition + vectorized batch decode ----------------
    def acquisition_thread(self):
        while self.running:
            n = self.ser.in_waiting
            if not n:
                time.sleep(0.0005)
                continue

            self._rx += self.ser.read(n)
            if b"\n" not in self._rx:
                continue

            chunks = self._rx.split(b"\n")
            self._rx = chunks[-1]                    # keep the incomplete tail
            block = b"\n".join(chunks[:-1]).decode('utf-8', errors='ignore').strip()
            if not block:
                continue

            # ---- vectorized batch decoding with NumPy ----
            try:
                arr = np.genfromtxt(io.StringIO(block), delimiter=',')
            except Exception:
                continue
            if arr.size == 0:
                continue
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[1] != 4:
                continue
            arr = arr[np.isfinite(arr).all(axis=1)]   # drop malformed rows

            for r in arr:
                row = (float(r[0]), float(r[1]), float(r[2]), int(r[3]))
                with self.plot_lock:
                    self.plot_buffer.append(row)          # -> live plot (always)
                if self.capturing:
                    self.log_queue.put(row)               # -> background logger

    # ---------------- THREAD 2: background CSV logger (continuous disk I/O) ----------------
    def logging_thread(self):
        with open(CSV_FILENAME, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Referencia', 'Posicion', 'Control', 'Timestamp_us'])
            last_flush = time.time()
            # keep draining while running OR while samples remain in the queue
            while self.running or not self.log_queue.empty():
                try:
                    row = self.log_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                writer.writerow(row)
                self.samples_logged += 1
                if time.time() - last_flush > 0.5:        # periodic flush to disk
                    f.flush()
                    last_flush = time.time()
            f.flush()

    # ---------------- control logic (step-reference sequencer) ----------------
    def control_logic(self):
        t = time.time() - self.start_time
        if self.step_index < len(TRAYECTORIA):
            t_next, angle = TRAYECTORIA[self.step_index]
            if t >= t_next:
                print(f"  t={t:5.2f}s  ->  setpoint {angle:.0f} deg")
                self.send_setpoint(angle)
                self.step_index += 1
        return t >= DURATION

    # ---------------- main: threads + trajectory + live plot ----------------
    def run_test(self):
        self.running = True
        threading.Thread(target=self.acquisition_thread, daemon=True).start()
        threading.Thread(target=self.logging_thread,    daemon=True).start()

        print("\n" + "=" * 52)
        print("  AUTOMATIC STEP-REFERENCE TEST  (0 -> 60 -> 100 -> 180 deg)")
        print(f"  Duration: {DURATION:.0f} s   |   CSV: {CSV_FILENAME}")
        print("=" * 52)
        input(">> Press [ENTER] to start the capture... ")

        # clean start at t0
        with self.plot_lock:
            self.plot_buffer.clear()
        with self.log_queue.mutex:
            self.log_queue.queue.clear()
        self.step_index = 0
        self.start_time = time.time()
        self.capturing  = True
        print("[RUN] capturing...")

        # live plot
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        l_ref, = ax1.plot([], [], 'g--', lw=1.2, label='Reference')
        l_pos, = ax1.plot([], [], 'b-',  lw=1.5, label='Position')
        l_u,   = ax2.plot([], [], 'r-',  lw=1.0, label='Control')
        ax1.set_ylabel("Position [deg]"); ax1.legend(loc='lower right'); ax1.grid(True)
        ax2.set_ylabel("Control [PWM, 10-bit]"); ax2.set_xlabel("Time [s]")
        ax2.legend(loc='upper right'); ax2.grid(True)

        finished = False
        try:
            while self.running and plt.fignum_exists(fig.number):
                if not finished:
                    finished = self.control_logic()
                    if finished:
                        print("[DONE] trajectory finished; returning to 0 deg.")
                        self.send_setpoint(0.0)
                        self.capturing = False
                        time.sleep(0.8)             # capture the tail
                        self.running = False
                        break

                # ---- render (decimated) ----
                with self.plot_lock:
                    m = len(self.plot_buffer)
                    data = np.array(self.plot_buffer) if m > PLOT_DECIMATION else None
                if data is not None:
                    t_axis = (data[:, 3] - data[0, 3]) / 1e6
                    l_ref.set_data(t_axis[::PLOT_DECIMATION], data[::PLOT_DECIMATION, 0])
                    l_pos.set_data(t_axis[::PLOT_DECIMATION], data[::PLOT_DECIMATION, 1])
                    l_u.set_data(  t_axis[::PLOT_DECIMATION], data[::PLOT_DECIMATION, 2])
                    ax1.set_xlim(0, max(DURATION, t_axis[-1]))
                    ax1.set_ylim(-10, 200)
                    cmin, cmax = float(data[:, 2].min()), float(data[:, 2].max())
                    ax2.set_ylim(cmin - 10, cmax + 10)
                plt.pause(0.05)                     # ~20 FPS

        except KeyboardInterrupt:
            print("\n[STOP] user interrupt.")
        finally:
            self.shutdown()

    def shutdown(self):
        self.capturing = False
        self.running = False
        self.send_setpoint(0.0)                     # safety
        time.sleep(0.3)
        # wait for the logger to drain the queue
        while not self.log_queue.empty():
            time.sleep(0.05)
        time.sleep(0.2)
        if self.ser.is_open:
            self.ser.close()
        print(f"[SAVED] {self.samples_logged} samples -> {CSV_FILENAME}")


if __name__ == "__main__":
    RCPTelemetry(SERIAL_PORT, BAUD_RATE).run_test()
