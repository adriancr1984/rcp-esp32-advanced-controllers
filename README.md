# Standalone RCP of Advanced Controllers on the ESP32-S3

Reproducibility package for the paper **"A Model-Based Design Methodology for Standalone Rapid Control Prototyping of Advanced Controllers on Low-Cost Hardware."**

This repository contains the Simulink models, the ESP32-S3 integration/firmware layer, the Python telemetry and analysis scripts, and the raw experimental data needed to reproduce the three test-case controllers (PID, Super-Twisting SMC — STSMC, and MRAC) on a Quanser SRV02 DC servo driven by an ESP32-S3 and an L298N driver.

> **Note.** The auto-generated C/C++ produced by MATLAB Embedded Coder carries a MathWorks
> academic-use license header and is **not** redistributed here. Instead, the Simulink models
> (`.slx`) and the code-generation settings below let you regenerate that code yourself.

---

## 1. Repository structure

```
.
├── README.md
├── LICENSE
├── simulink/                   # Simulink models (source of truth)
│   ├── pid.slx
│   ├── stsmc.slx
│   └── mrac.slx
├── firmware/                   # ESP32-S3 integration layer (Arduino IDE)
│   ├── main.ino                # extern "C" wrapper + FreeRTOS task + HAL mapping
│   └── README.md               # pin mapping and build notes
├── telemetry/                  # Python host tools
│   ├── serial_logger.py        # PySerial reader + circular buffer + CSV logging
│   └── analyze_metrics.py      # segments each run and computes e_ss, t_s, M_p, Var u
└── data/                       # raw experimental runs (reproduce Table 5)
    ├── pid_run.csv
    ├── stsmc_run.csv
    └── mrac_run.csv
```

---

## 2. Hardware

| Item | Detail |
| :-- | :-- |
| Microcontroller | ESP32-S3 (Xtensa LX7 dual-core, 240 MHz, hardware FPU) |
| Plant | Quanser SRV02 DC servo, 70:1 gear reduction |
| Position sensor | Incremental optical quadrature encoder, 4096 CPR (decoded in software) |
| Motor driver | L298N, supply limited to ±6 V |
| Host | PC with MATLAB/Simulink, Arduino IDE, and Python |

**Pin mapping:** encoder A/B on GPIO 15/16, PWM (L298N ENA) on GPIO 4, direction (IN1/IN2)
on GPIO 5/6, telemetry over USB at 921600 bps. Full wiring and build notes in
[`firmware/README.md`](firmware/README.md).

---

## 3. Software requirements

- **MATLAB/Simulink** with **Embedded Coder** (models generated with R2023a).
- **Arduino IDE** with the **ESP32 board package** (Espressif core).
- **Python 3** with `pyserial`, `numpy`, `matplotlib`, `pandas`, `scipy`:

```
pip install pyserial numpy matplotlib pandas scipy
```

---

## 4. Code generation settings (Simulink → Embedded Coder)

Reproduces Table 1 of the paper:

| Category | Parameter | Value |
| :-- | :-- | :-- |
| Solver | Type | Fixed-step |
| Solver | Solver | Discrete (no continuous states) |
| Solver | Sample time `Ts` | `1e-3` s (1 kHz) |
| Code Generation | System Target File | `ert.tlc` (Embedded Real-Time) |
| Code Generation | Language | C++ (Arduino-compatible) |
| Code Generation | Generate code only | On |
| Optimization | Default data type | `single` (32-bit float) |

Generate the code for each model, copy the generated `*.c/*.h` into `firmware/`, and wrap them
as shown in `firmware/main.ino`.

---

## 5. Controller parameters (as used in the paper)

Identified plant model (voltage-to-angular-velocity, from system identification):

```
G(s) = 4.673 / (s + 4.676)      ->  a1 = 4.676 s^-1,  b0 = 4.673,  tau = 0.214 s
```

| Controller | Gains | Discretization | Ts |
| :-- | :-- | :-- | :-- |
| PID | Kp = 5, Ki = 4, Kd = 0.65 | Tustin | 1 ms |
| STSMC | C = 12, λ1 = 15, λ2 = 100 | Forward Euler | 1 ms |
| MRAC | γ = 10, κ0 = 1.5, θ̂(0) = 0 | Forward Euler | 1 ms |

Gains were tuned heuristically; see the paper for the tuning discussion and caveats.

---

## 6. Build, flash, and run

1. **Generate** the C++ code for a controller from its `.slx` (Section 4 settings).
2. **Integrate**: place the generated files in `firmware/`, open `main.ino` in the Arduino IDE, select the ESP32-S3 board, and flash.
3. **Log telemetry**:
   ```
   python telemetry/serial_logger.py --port COMx --baud 921600 --out data/run.csv
   ```
4. **Compute metrics / plot**:
   ```
   python telemetry/analyze_metrics.py data/run.csv
   ```
   Segments the multi-step trajectory (0→60→100→180°) and reports steady-state error, settling
   time, overshoot, and control-signal variation (`Var u = Σ|u[k]−u[k−1]|`).
---

## 7. Data

`data/` holds one representative run per controller. Running `analyze_metrics.py` on these files reproduces the metrics in Table 5 of the paper exactly.

---

## 8. Citation

```
[AUTHOR TO COMPLETE — full paper citation once published: authors, title, venue, year, DOI.]
```

## 9. License

MIT (see `LICENSE`) for the authors' own code, models, and scripts. The MATLAB Embedded Coder auto-generated C/C++ is **not** redistributed — regenerate it from the provided Simulink models under your own MATLAB license.
