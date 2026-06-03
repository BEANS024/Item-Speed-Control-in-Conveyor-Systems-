#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// LCD SETUP
// ============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
// IR SENSOR PINS
// ============================================================
const int IR_SENSOR_1 = 2;   // ENTRY SENSOR
const int IR_SENSOR_2 = 3;   // EXIT SENSOR

// ============================================================
// MOTOR DRIVER (L298N)
// ============================================================
const int motor1pin1 = 6;
const int motor1pin2 = 7;
const int ENA        = 5;

// ============================================================
// PUSH BUTTONS
// ============================================================
const int GREEN_BTN = 8;
const int RED_BTN   = 9;

// ============================================================
// SPEED SETTINGS
// ============================================================
const int DEFAULT_SPEED = 110;
const int MIN_SPEED     = 90;
const int MAX_SPEED     = 200;

// ============================================================
// TARGET GAP
// 900 ms limits belt occupancy to ~2 items at a time,
// giving IR2 corrections time to propagate before the next
// item commits to the belt.
// ============================================================
const unsigned long TARGET_GAP = 900; // milliseconds

// ============================================================
// FEED-FORWARD SCHEDULE
//
// When IR1 detects a dangerously small gap it schedules a
// soft pre-emptive speed reduction to fire midway through
// the belt travel (~1 second later). IR2 always overrides
// this with PI output when the item actually exits.
//
// FF_TRIGGER_GAP  — gap below this arms a feed-forward event
// FF_DELAY_MS     — how long after IR1 to fire (~half travel)
// FF_TARGET_PWM   — soft preliminary target PWM
// ============================================================
const unsigned long FF_TRIGGER_GAP = 700;  // ms
const unsigned long FF_DELAY_MS    = 1000; // ms
const int           FF_TARGET_PWM  = 120;  // soft pre-emptive target

// ============================================================
// PI CONTROLLER — ASYMMETRIC GAINS
//
// Reduced Ki (was 0.025 / 0.010) to prevent integral windup
// on this short 2-second transport delay belt.
// Max integral contribution = Ki * clamp = 0.005*1000 = ±5 PWM.
// Kp does the heavy lifting; I only removes residual bias.
// ============================================================
float Kp_slow = 0.18;
float Ki_slow = 0.005;

float Kp_fast = 0.07;
float Ki_fast = 0.002;

float integral = 0;

// ============================================================
// SYSTEM VARIABLES
// ============================================================
bool systemActive = false;

int currentPWM = DEFAULT_SPEED;
int targetPWM  = DEFAULT_SPEED;

int materialCount = 0;

// ============================================================
// SENSOR STATES
// ============================================================
bool lastIR1State   = HIGH;
bool lastIR2State   = HIGH;
bool lastGreenState = HIGH;
bool lastRedState   = HIGH;

// ============================================================
// TIMERS
// ============================================================
unsigned long previousEntryTime = 0;   // timestamp of last IR1 trigger
unsigned long entryGap          = 0;   // gap measured at IR1
unsigned long measuredGap       = 0;   // gap dequeued at IR2 for display
unsigned long lastLCDUpdate     = 0;

// ============================================================
// DEBOUNCE
// ============================================================
unsigned long lastIR1Debounce = 0;
unsigned long lastIR2Debounce = 0;
const unsigned long DEBOUNCE_DELAY = 120;

// ============================================================
// ENTRY GAP FIFO QUEUE
//
// IR1 enqueues each measured inter-item gap.
// IR2 dequeues the matching gap when an item exits.
//
// SEMANTICS NOTE: The queue holds N-1 gaps for N items per
// batch. The last item to exit will always dequeue the
// TARGET_GAP fallback, causing zero error — correct behaviour.
// ============================================================
const int QUEUE_SIZE = 10;
unsigned long gapQueue[QUEUE_SIZE];
int queueHead  = 0;
int queueTail  = 0;
int queueCount = 0;

void enqueueGap(unsigned long gap) {
  if (queueCount < QUEUE_SIZE) {
    gapQueue[queueTail] = gap;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount++;
  }
}

unsigned long dequeueGap() {
  if (queueCount > 0) {
    unsigned long gap = gapQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount--;
    return gap;
  }
  return TARGET_GAP; // fallback — last item of batch, no correction needed
}

// ============================================================
// FEED-FORWARD EVENT QUEUE
//
// Each entry records:
//   fireAt  — millis() timestamp when the event should execute
//   pwm     — the preliminary targetPWM to apply
//   active  — whether this slot is occupied
//
// IR1 populates slots; loop() fires them on schedule.
// IR2 cancels the matching slot before applying PI, ensuring
// PI always has final authority over feed-forward.
// ============================================================
const int FF_QUEUE_SIZE = 10;

struct FFEvent {
  unsigned long fireAt;
  int           pwm;
  bool          active;
};

FFEvent ffQueue[FF_QUEUE_SIZE];
int ffCount = 0;

void scheduleFFEvent(unsigned long delayMs, int pwm) {
  for (int i = 0; i < FF_QUEUE_SIZE; i++) {
    if (!ffQueue[i].active) {
      ffQueue[i].fireAt = millis() + delayMs;
      ffQueue[i].pwm    = pwm;
      ffQueue[i].active = true;
      ffCount++;
      return;
    }
  }
  // All slots full — drop silently (belt saturated)
}

// Called every loop() iteration — fires any due events.
// Only lowers speed, never raises it.
void processFFEvents() {
  unsigned long now = millis();
  for (int i = 0; i < FF_QUEUE_SIZE; i++) {
    if (ffQueue[i].active && now >= ffQueue[i].fireAt) {
      if (ffQueue[i].pwm < targetPWM) {
        targetPWM = ffQueue[i].pwm;
      }
      ffQueue[i].active = false;
      ffCount--;
    }
  }
}

// IR2 cancels the oldest pending FF event before applying PI,
// so PI output is never immediately overwritten by a late FF.
void cancelNextFFEvent() {
  unsigned long earliest = 0xFFFFFFFF;
  int           slot     = -1;

  for (int i = 0; i < FF_QUEUE_SIZE; i++) {
    if (ffQueue[i].active && ffQueue[i].fireAt < earliest) {
      earliest = ffQueue[i].fireAt;
      slot     = i;
    }
  }

  if (slot >= 0) {
    ffQueue[slot].active = false;
    ffCount--;
  }
}

void clearAllFFEvents() {
  for (int i = 0; i < FF_QUEUE_SIZE; i++) {
    ffQueue[i].active = false;
  }
  ffCount = 0;
}

// ============================================================
// MOTOR FUNCTIONS
// ============================================================
void motorRun(int pwmValue) {
  pwmValue = constrain(pwmValue, MIN_SPEED, MAX_SPEED);

  // Asymmetric ramp: faster slowdown, gentler speedup
  if (currentPWM > pwmValue)      currentPWM -= 5;
  else if (currentPWM < pwmValue) currentPWM += 2;

  currentPWM = constrain(currentPWM, MIN_SPEED, MAX_SPEED);

  analogWrite(ENA, currentPWM);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
}

void motorStop() {
  analogWrite(ENA, 0);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
}

// ============================================================
// LCD FUNCTIONS
// ============================================================
void updateLCDReady() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PI CONVEYOR");
  lcd.setCursor(0, 1); lcd.print("PRESS GREEN");
}

void updateLCDStopped() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("SYSTEM STOP");
  lcd.setCursor(0, 1); lcd.print("PRESS GREEN");
}

void updateLCDLive() {
  long errorMs    = (long)measuredGap - (long)TARGET_GAP;
  int  errorTenth = (int)(errorMs / 100);
  char errorSign  = (errorTenth >= 0) ? '+' : '-';
  int  errorAbs   = abs(errorTenth);

  char row0[17];
  snprintf(row0, sizeof(row0),
    "PWM:%-3d E:%c%d.%d",
    currentPWM, errorSign, errorAbs / 10, errorAbs % 10
  );
  lcd.setCursor(0, 0);
  lcd.print(row0);

  unsigned int gapTenth = (unsigned int)(measuredGap / 100);
  char row1[17];
  snprintf(row1, sizeof(row1),
    "ON:%-2d GAP:%d.%d  ",
    materialCount, gapTenth / 10, gapTenth % 10
  );
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

// ============================================================
// RESET SYSTEM
// ============================================================
void resetSystem() {
  materialCount     = 0;
  previousEntryTime = 0;   // FIX: cleared so inter-batch gaps are not measured
  entryGap          = 0;
  measuredGap       = 0;
  targetPWM         = DEFAULT_SPEED;
  currentPWM        = DEFAULT_SPEED;
  integral          = 0;

  queueHead  = 0;
  queueTail  = 0;
  queueCount = 0;

  clearAllFFEvents();
}

// ============================================================
// PI SPEED CONTROL — EVENT-DRIVEN, ANTI-WINDUP
//
// Called ONLY from IR2 block (once per item exit).
//
// Key properties:
//   - Integral updates once per item exit, NOT every loop tick
//   - Conditional integration: skips accumulation when output
//     is saturated AND error would push it further into sat.
//   - Integral clamped to ±1000 → max I contribution ±5 PWM
//   - Asymmetric gains: harder brake, gentler accelerate
//   - Kp does the heavy lifting on a 2-sec transport delay belt
//   - Ki only removes residual steady-state bias
// ============================================================
void applyPIControl() {
  float error = (float)measuredGap - (float)TARGET_GAP;

  // Select asymmetric gains
  // Negative error → items too close → slow down harder
  // Positive error → items too far   → speed up gently
  float Kp = (error < 0) ? Kp_slow : Kp_fast;
  float Ki = (error < 0) ? Ki_slow : Ki_fast;

  // Compute candidate output using CURRENT integral (before update)
  // so we can inspect saturation before touching the integral state.
  float pTerm     = Kp * error;
  float iTerm     = Ki * integral;
  int   candidate = DEFAULT_SPEED + (int)(pTerm + iTerm);

  // Conditional integration — anti-windup back-calculation guard:
  // Do NOT accumulate if the output is already railed AND the new
  // error would push it further into saturation.
  //   atMin + negative error → integrating would keep us floored
  //   atMax + positive error → integrating would keep us ceilinged
  bool atMin       = (candidate <= MIN_SPEED);
  bool atMax       = (candidate >= MAX_SPEED);
  bool windingWorse = (error < 0 && atMin) || (error > 0 && atMax);

  if (!windingWorse) {
    integral += error;
    integral  = constrain(integral, -1000.0f, 1000.0f); // tight clamp
  }

  // Recompute final output with updated integral
  iTerm     = Ki * integral;
  targetPWM = DEFAULT_SPEED + (int)(pTerm + iTerm);
  targetPWM = constrain(targetPWM, MIN_SPEED, MAX_SPEED);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(GREEN_BTN, INPUT_PULLUP);
  pinMode(RED_BTN,   INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  motorStop();
  updateLCDReady();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ==========================================================
  // GREEN BUTTON (START)
  // ==========================================================
  bool greenState = digitalRead(GREEN_BTN);
  if (lastGreenState == HIGH && greenState == LOW) {
    delay(50);
    if (!systemActive) {
      systemActive = true;
      resetSystem();
      lcd.clear();
    }
  }
  lastGreenState = greenState;

  // ==========================================================
  // RED BUTTON (STOP)
  // ==========================================================
  bool redState = digitalRead(RED_BTN);
  if (lastRedState == HIGH && redState == LOW) {
    delay(50);
    systemActive = false;
    motorStop();
    resetSystem();
    updateLCDStopped();
  }
  lastRedState = redState;

  // ==========================================================
  // MAIN SYSTEM
  // ==========================================================
  if (systemActive) {

    // ========================================================
    // IR1 — ENTRY DETECTION
    //
    // Responsibilities:
    //   1. Increment materialCount
    //   2. Measure entryGap (only if a preceding item is still
    //      on the belt — materialCount > 1 after increment,
    //      AND previousEntryTime is valid)
    //   3. Enqueue entryGap (for IR2 PI use)
    //   4. If gap is dangerously small, SCHEDULE a FF event
    //
    // FIX: Gap is only measured and enqueued when materialCount
    // was already >= 1 before this item arrived (i.e. there is
    // a real preceding item on the belt). This prevents phantom
    // inter-batch gaps from polluting the queue when the belt
    // was empty between batches.
    //
    // IR1 never writes targetPWM, never calls applyPIControl().
    // ========================================================
    bool ir1State = digitalRead(IR_SENSOR_1);

    if (lastIR1State == HIGH &&
        ir1State == LOW &&
        now - lastIR1Debounce > DEBOUNCE_DELAY) {

      lastIR1Debounce = now;

      // Capture count BEFORE incrementing to check if a
      // preceding item is genuinely present on the belt.
      int countBeforeEntry = materialCount;
      materialCount++;

      // Only measure and act on a gap if:
      //   (a) previousEntryTime is set (not the very first item ever), AND
      //   (b) there was already at least 1 item on the belt
      //       (countBeforeEntry >= 1), meaning the gap is a real
      //       intra-batch gap, not a cross-batch measurement.
      if (previousEntryTime > 0 && countBeforeEntry >= 1) {
        entryGap = now - previousEntryTime;
        enqueueGap(entryGap);

        // If the gap is dangerously small, schedule a soft
        // pre-emptive slowdown to fire midway through belt travel.
        // IR2 will cancel this and replace with PI output.
        if (entryGap < FF_TRIGGER_GAP) {
          scheduleFFEvent(FF_DELAY_MS, FF_TARGET_PWM);
        }
      }

      previousEntryTime = now;
    }

    lastIR1State = ir1State;

    // ========================================================
    // TIMED FEED-FORWARD EVENTS
    //
    // Fires any FF events whose scheduled time has arrived.
    // Only lowers speed, never raises it.
    // IR2 always cancels the matching event before applying PI.
    // ========================================================
    processFFEvents();

    // ========================================================
    // IR2 — EXIT DETECTION
    //
    // Responsibilities:
    //   1. Decrement materialCount
    //   2. Cancel the feed-forward event for this item
    //   3. Dequeue the stored gap for this item
    //   4. Run PI control → update targetPWM
    //
    // IR2 is the ONLY place that calls applyPIControl().
    // PI output always overrides any prior feed-forward target.
    // ========================================================
    bool ir2State = digitalRead(IR_SENSOR_2);

    if (lastIR2State == HIGH &&
        ir2State == LOW &&
        now - lastIR2Debounce > DEBOUNCE_DELAY) {

      lastIR2Debounce = now;

      materialCount--;
      if (materialCount < 0) materialCount = 0;

      // Cancel matching scheduled FF before PI runs, so a
      // late-firing FF cannot overwrite fresh PI output.
      cancelNextFFEvent();

      // Retrieve the gap measured at IR1 for this item.
      // If queue is empty (last item of batch), returns TARGET_GAP
      // → error = 0 → no spurious correction.
      measuredGap = dequeueGap();
      applyPIControl();
    }

    lastIR2State = ir2State;

    // ========================================================
    // NO ITEMS ON BELT → RETURN TO DEFAULT
    //
    // FIX: previousEntryTime is now also cleared here so that
    // the gap between the last item of one batch and the first
    // item of the next batch is NOT measured as an intra-batch
    // gap. Without this, a pause between batches would produce
    // a tiny phantom entryGap on the first IR1 of the new
    // batch, triggering a spurious FF slowdown.
    //
    // Fires only after IR2 confirms the belt is empty.
    // ========================================================
    if (materialCount == 0) {
      targetPWM         = DEFAULT_SPEED;
      integral          = 0;
      previousEntryTime = 0;   // FIX: prevents inter-batch gap measurement
      entryGap          = 0;
      clearAllFFEvents();
    }

    // ========================================================
    // RUN MOTOR
    // ========================================================
    motorRun(targetPWM);

    // ========================================================
    // LCD UPDATE (every 250 ms)
    // ========================================================
    if (now - lastLCDUpdate >= 250) {
      lastLCDUpdate = now;
      updateLCDLive();
    }
  }
}
