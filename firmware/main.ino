#include <Arduino.h>

// =====================================================================
//  CONTROLLER-SPECIFIC ADAPTER
//  This is the ONLY block that changes when porting a different
//  controller (PID / STSMC / MRAC / ...). Point it to your Embedded
//  Coder-generated model and to its input/output port names.
//  Everything below this block is reused unchanged.
// =====================================================================
extern "C" {
  #include "ST_matlab.h"     // <-- your generated model header (e.g. pid.h, MRCA.h)
  #include "rtwtypes.h"
}
#define CTRL_initialize  ST_matlab_initialize
#define CTRL_step        ST_matlab_step
#define CTRL_U           ST_matlab_U     // generated input struct
#define CTRL_Y           ST_matlab_Y     // generated output struct
#define IN_REFERENCE     Referencia      // model input port: reference
#define IN_ENCODER       Encoder         // model input port: measured position
#define OUT_CONTROL      Salida          // model output port: control command
// =====================================================================

// ---------------- Pins (ESP32-S3) ----------------
const int PIN_ENCODER_A = 15;
const int PIN_ENCODER_B = 16;
const int PIN_PWM  = 4;
const int PIN_DIR1 = 5;
const int PIN_DIR2 = 6;

// ---------------- PWM: LEDC, 20 kHz, 10-bit ----------------
// 10-bit is the maximum usable resolution at 20 kHz on the LEDC clock.
const int PWM_FREQ     = 20000;
const int PWM_RES_BITS = 10;
const int MAX_PWM      = (1 << PWM_RES_BITS) - 1;   // 1023

// ---------------- Encoder: software x2 decoding (edges of A; B for direction) --------
const float ENCODER_COUNTS_PER_REV = 4096.0 / 2.0;

volatile long  encoderCount = 0;
volatile float setpoint_deg = 0.0;

struct TelemetryData { float ref; float pos; float ctrl; unsigned long time; };

hw_timer_t*        timer = NULL;
SemaphoreHandle_t  timerSemaphore;
QueueHandle_t      telemetryQueue;

// ---------------- ISRs ----------------
void IRAM_ATTR onEncoderChange() {
  if (digitalRead(PIN_ENCODER_B) == digitalRead(PIN_ENCODER_A)) encoderCount++;
  else                                                          encoderCount--;
}
void IRAM_ATTR onTimerTrigger() { xSemaphoreGiveFromISR(timerSemaphore, NULL); }

// ---------------- Control task (Core 1, high priority) ----------------
void ControlLoopTask(void* pv) {
  TelemetryData d;
  for (;;) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
      float pos_counts = (float)encoderCount;
      float ref_counts = (setpoint_deg / 360.0f) * ENCODER_COUNTS_PER_REV;

      // ---- inject to the model, step, read control ----
      CTRL_U.IN_REFERENCE = ref_counts;
      CTRL_U.IN_ENCODER   = pos_counts;
      CTRL_step();
      float u = (float)CTRL_Y.OUT_CONTROL;

      // ---- actuate ----
      int pwm = abs((int)u);
      if (pwm > MAX_PWM) pwm = MAX_PWM;
      digitalWrite(PIN_DIR1, u > 0 ? HIGH : LOW);
      digitalWrite(PIN_DIR2, u > 0 ? LOW  : HIGH);
      ledcWrite(PIN_PWM, pwm);

      // ---- telemetry (position reported in degrees) ----
      d.ref  = setpoint_deg;
      d.pos  = (pos_counts / ENCODER_COUNTS_PER_REV) * 360.0f;
      d.ctrl = u;
      d.time = micros();
      xQueueSend(telemetryQueue, &d, 0);
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  Serial.setTimeout(5);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), onEncoderChange, CHANGE);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES_BITS);

  CTRL_initialize();

  timerSemaphore = xSemaphoreCreateBinary();
  telemetryQueue = xQueueCreate(200, sizeof(TelemetryData));
  xTaskCreatePinnedToCore(ControlLoopTask, "ControlTask", 8192, NULL, 24, NULL, 1);

  timer = timerBegin(1000000);            // 1 MHz time base
  timerAttachInterrupt(timer, &onTimerTrigger);
  timerAlarm(timer, 1000, true, 0);       // 1000 us -> 1 kHz
}

// ---------------- Loop (Core 0): setpoint RX + telemetry TX ----------------
void loop() {
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    if (s.length() > 0) setpoint_deg = s.toFloat();
  }
  TelemetryData r;
  while (xQueueReceive(telemetryQueue, &r, 0) == pdTRUE) {
    Serial.print(r.ref);  Serial.print(",");
    Serial.print(r.pos);  Serial.print(",");
    Serial.print(r.ctrl); Serial.print(",");
    Serial.println(r.time);
  }
}
