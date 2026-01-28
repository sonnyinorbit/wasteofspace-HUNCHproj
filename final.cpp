#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <Adafruit_VL53L1X.h>
#include <Stepper.h>


// =====================
// Hardware Configuration
// =====================


// Stepper
#define STEPS_PER_REV 400
Stepper stepper(STEPS_PER_REV, 9, 10, 11, 12);
const int STEPPER_RPM = 30;


// OLED 128x32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const uint8_t OLED_ADDR = 0x3C;


// NeoPixels
const int NEOPIXEL_PIN = 6;
const int NUM_PIXELS = 6;
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const uint8_t LED_BRIGHTNESS = 50;


// ToF
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
const uint8_t TOF_ADDR = 0x29;
const uint16_t TOF_TIMING_BUDGET_MS = 50;


// Servo
Servo hatchServo;
const int SERVO_PIN = 8;
const int SERVO_STOP = 90;
const int SERVO_CW   = 0;
const int SERVO_CCW  = 180;


// IO Pins
const int BTN_PIN = 2;
const int LIMIT_SWITCH_PIN = 4; // INPUT_PULLUP, LOW = closed


// Distance thresholds (mm)
const int EMPTY_DISTANCE_MM = 90;
const int CAN_PRESENT_THRESHOLD_MM = 60;


// Filtering
const int NUM_READINGS = 5;
int distanceReadings[NUM_READINGS];
int distanceIndex = 0;


// =====================
// State
// =====================
int lastBtn = LOW;


bool oledWorking = false;
bool tofWorking  = false;


bool trashPresent = false;
bool lastTrashState = false;


bool cycleAborted = false;


// =====================
// Colors (NeoPixel uses GRB order internally)
// =====================
inline uint32_t C(uint8_t r, uint8_t g, uint8_t b) { return strip.Color(r, g, b); }
const uint32_t LED_OFF     = 0;
const uint32_t LED_WAITING = /*blue*/ 0; // set in setup after strip.begin
const uint32_t LED_READY   = /*cyan*/ 0; // set in setup after strip.begin
const uint32_t LED_OK      = /*green*/ 0; // set in setup after strip.begin
const uint32_t LED_ERR     = /*red*/ 0; // set in setup after strip.begin


// =====================
// Cycle Timings (ms)
// =====================
const unsigned long T_DEPRESSURIZE = 2750;
const unsigned long T_OPEN_PULSE   = 560;
const unsigned long T_OPEN_WAIT    = 7440;
const unsigned long T_EJECT_WAIT   = 560;
const unsigned long T_CLOSE_PULSE  = 610;
const unsigned long T_CLOSE_WAIT   = 2000;
const unsigned long T_REPRESSURIZE = 2750;


// =====================
// Forward Decls
// =====================
void initPins();
void initDistanceBuffer();
void initNeoPixels();
void initI2C();
void initServo();
void initOLED();
void initToF();


void setAllLEDs(uint32_t color);
void pulseLED(uint32_t color, int times, unsigned long onMs, unsigned long offMs);


void oledSplash();
void oledIdle();
void oledError(const __FlashStringHelper* title, const __FlashStringHelper* msg);
void oledPhase(const __FlashStringHelper* phase, const __FlashStringHelper* msg);


bool isInnerHatchClosed();
bool readButtonPressedEdge();
bool canStartCycle();


bool checkForCanFiltered();
int  getAveragedDistanceMM(); // returns -1 if no fresh reading


bool checkLimitSwitchAbort();        // true = ok, false = aborted
bool safeDelay(unsigned long ms);    // monitors limit switch during wait


void runCycle();
void runEjection();


// Optional: faster “refresh can state” helper used post-eject
void resetDistanceFilterToEmpty();
void primeDistanceFilter(int samples, unsigned long spacingMs);


// =====================
// Setup / Loop
// =====================
void setup() {
 // Serial removed (as requested)
 delay(200);


 initPins();
 initDistanceBuffer();


 initNeoPixels();
 initI2C();


 initToF();
 initOLED();
 initServo();


 if (oledWorking) oledSplash();


 // Ready pulse
 pulseLED(C(0, 0, 255), 3, 250, 250); // blue pulse


 oledIdle();
}


void loop() {
 hatchServo.write(SERVO_STOP);


 // Update sensors/state
 if (tofWorking) {
   trashPresent = checkForCanFiltered();
 } else {
   trashPresent = false;
 }


 // Update idle UI only on state change
 if (trashPresent != lastTrashState) {
   oledIdle();
   lastTrashState = trashPresent;
 }


 // Button edge -> attempt cycle
 if (readButtonPressedEdge()) {
   if (!canStartCycle()) {
     // canStartCycle already shows the correct error + returns to idle
   } else {
     runCycle();
   }
 }


 delay(30);
}


// =====================
// Init Helpers
// =====================
void initPins() {
 pinMode(BTN_PIN, INPUT);
 pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
}


void initDistanceBuffer() {
 for (int i = 0; i < NUM_READINGS; i++) distanceReadings[i] = EMPTY_DISTANCE_MM;
 distanceIndex = 0;
}


void initNeoPixels() {
 strip.begin();
 strip.setBrightness(LED_BRIGHTNESS);
 strip.show();


 // quick white test
 setAllLEDs(C(255, 255, 255));
 delay(250);
 setAllLEDs(LED_OFF);


 // assign const “colors” that depend on strip.Color()
 // (keeps them in one place, avoids global init order problems)
 // NOTE: we’ll just call C(...) directly elsewhere, but keeping these is fine.
}


void initI2C() {
 Wire.begin();
 Wire.setClock(400000L);
 delay(50);
}


void initServo() {
 hatchServo.attach(SERVO_PIN);
 hatchServo.write(SERVO_STOP);
 delay(50);
}


void initOLED() {
 delay(50);
 oledWorking = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
 if (!oledWorking) {
   // orange-ish flash to indicate OLED fail
   pulseLED(C(255, 100, 0), 5, 180, 180);
 }
}


void initToF() {
 tofWorking = false;


 if (!vl53.begin(TOF_ADDR, &Wire)) {
   pulseLED(C(255, 0, 0), 5, 200, 200); // red flash: sensor missing
   tofWorking = false;
   return;
 }


 if (!vl53.startRanging()) {
   pulseLED(C(255, 0, 0), 5, 200, 200); // red flash: ranging failed
   tofWorking = false;
   return;
 }


 vl53.setTimingBudget(TOF_TIMING_BUDGET_MS);
 tofWorking = true;
}


// =====================
// UI Helpers
// =====================
void setAllLEDs(uint32_t color) {
 for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, color);
 strip.show();
}


void pulseLED(uint32_t color, int times, unsigned long onMs, unsigned long offMs) {
 for (int i = 0; i < times; i++) {
   setAllLEDs(color);
   delay(onMs);
   setAllLEDs(LED_OFF);
   delay(offMs);
 }
}


void oledSplash() {
 display.clearDisplay();
 display.setTextColor(WHITE);


 display.setTextSize(2);
 display.setCursor(10, 8);
 display.print(F("TRASH"));
 display.display();
 delay(700);


 display.clearDisplay();
 display.setCursor(5, 8);
 display.print(F("EJECTOR"));
 display.display();
 delay(700);


 display.setTextSize(1);
}


void oledIdle() {
 if (trashPresent) {
   setAllLEDs(C(0, 255, 255)); // cyan: ready
   if (!oledWorking) return;


   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0, 0);
   display.println(F("TRASH DETECTED"));
   display.println(F("Ready to eject"));
   display.println(F("Press button"));
   display.display();
 } else {
   setAllLEDs(C(0, 100, 255)); // blue: waiting
   if (!oledWorking) return;


   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0, 0);
   display.println(F("WAITING"));
   display.println(F("Insert trash"));
   display.display();
 }
}


void oledError(const __FlashStringHelper* title, const __FlashStringHelper* msg) {
 pulseLED(C(255, 0, 0), 3, 160, 160);
 setAllLEDs(C(255, 0, 0));


 if (!oledWorking) return;


 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0, 0);
 display.println(F("ERROR"));
 display.println(title);
 display.println(msg);
 display.display();
}


void oledPhase(const __FlashStringHelper* phase, const __FlashStringHelper* msg) {
 if (!oledWorking) return;


 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0, 0);
 display.println(phase);
 display.println(msg);
 display.println(F("DO NOT OPEN HATCH"));
 display.display();
}


// =====================
// Input / Sensor Helpers
// =====================
bool isInnerHatchClosed() {
 // limit switch wired to pullup: pressed/closed -> LOW
 return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}


bool readButtonPressedEdge() {
 int btn = digitalRead(BTN_PIN);
 bool pressedEdge = (btn == HIGH && lastBtn == LOW);
 lastBtn = btn;


 if (!pressedEdge) return false;


 // debounce confirm
 delay(40);
 return digitalRead(BTN_PIN) == HIGH;
}


int getAveragedDistanceMM() {
 if (!tofWorking) return -1;
 if (!vl53.dataReady()) return -1;


 int16_t d = vl53.distance();
 vl53.clearInterrupt();


 if (d == -1) return -1;


 distanceReadings[distanceIndex] = d;
 distanceIndex = (distanceIndex + 1) % NUM_READINGS;


 long sum = 0;
 for (int i = 0; i < NUM_READINGS; i++) sum += distanceReadings[i];
 return (int)(sum / NUM_READINGS);
}


bool checkForCanFiltered() {
 int avg = getAveragedDistanceMM();
 if (avg == -1) return trashPresent; // keep last known state
 return (avg < CAN_PRESENT_THRESHOLD_MM);
}


void resetDistanceFilterToEmpty() {
 for (int i = 0; i < NUM_READINGS; i++) distanceReadings[i] = EMPTY_DISTANCE_MM;
 distanceIndex = 0;
}


void primeDistanceFilter(int samples, unsigned long spacingMs) {
 for (int i = 0; i < samples; i++) {
   (void)checkForCanFiltered();
   delay(spacingMs);
 }
}


// =====================
// Safety Helpers
// =====================
bool checkLimitSwitchAbort() {
 if (isInnerHatchClosed()) return true;


 cycleAborted = true;


 // emergency stop servo
 hatchServo.write(SERVO_STOP);


 oledError(F("CYCLE ABORTED"), F("Hatch opened!"));
 delay(2000);


 oledIdle();
 return false;
}


bool safeDelay(unsigned long ms) {
 unsigned long start = millis();
 while (millis() - start < ms) {
   if (!checkLimitSwitchAbort()) return false;
   delay(10);
 }
 return true;
}


// =====================
// Cycle Logic
// =====================
bool canStartCycle() {
 if (!tofWorking) {
   oledError(F("SENSOR ERROR"), F("ToF not working"));
   delay(2000);
   oledIdle();
   return false;
 }


 if (!trashPresent) {
   oledError(F("NO TRASH"), F("Insert trash"));
   delay(2000);
   oledIdle();
   return false;
 }


 if (!isInnerHatchClosed()) {
   oledError(F("HATCH OPEN"), F("Close inner hatch"));
   delay(2000);
   oledIdle();
   return false;
 }


 return true;
}


void runEjection() {
 const unsigned long EJECTION_DURATION_MS = 60000;


 stepper.setSpeed(STEPPER_RPM);


 unsigned long start = millis();
 const int CHUNK_STEPS = 10;   // smaller = more responsive
 const int CHUNK_DELAY = 2;    // keep tiny; can be 0


 while (millis() - start < EJECTION_DURATION_MS) {
   // safety abort if hatch opened mid-cycle
   if (!checkLimitSwitchAbort()) return;


   // run the stepper in small chunks for the full 60 seconds
   stepper.step(1125000);
   delay(CHUNK_DELAY);
 }
}




void runCycle() {
 cycleAborted = false;


 // final safety gate
 if (!checkLimitSwitchAbort()) return;
 if (!trashPresent) {
   oledError(F("NO TRASH"), F("Can not detected"));
   delay(2000);
   oledIdle();
   return;
 }


 // PHASE 1 - Depressurize
 setAllLEDs(C(255, 165, 0)); // orange
 oledPhase(F("PHASE 1"), F("DEPRESSURIZING"));
 if (!safeDelay(T_DEPRESSURIZE)) return;


 // PHASE 2 - Opening hatch
 setAllLEDs(C(255, 255, 0)); // yellow
 oledPhase(F("PHASE 2"), F("OPENING HATCH"));


 hatchServo.write(SERVO_CCW);
 if (!safeDelay(T_OPEN_PULSE)) { hatchServo.write(SERVO_STOP); return; }
 hatchServo.write(SERVO_STOP);


 if (!safeDelay(T_OPEN_WAIT)) return;


 // PHASE 3 - Eject
 setAllLEDs(C(255, 100, 0)); // orange-red
 oledPhase(F("PHASE 3"), F("EJECTING TRASH"));


 runEjection();
 if (!safeDelay(T_EJECT_WAIT)) return;


 // Post-eject check (refresh filter & re-check)
 if (!safeDelay(250)) return;


 bool canStillPresent = false;
 if (tofWorking) {
   resetDistanceFilterToEmpty();
   primeDistanceFilter(NUM_READINGS * 2, 80);
   canStillPresent = checkForCanFiltered();


   if (canStillPresent) {
     oledError(F("EJECT FAILED"), F("Can still there"));
     if (!safeDelay(1500)) return;
     // still proceed to close hatch for safety
   }
 }


 // PHASE 4 - Close hatch
 setAllLEDs(C(255, 255, 0)); // yellow
 oledPhase(F("PHASE 4"), F("CLOSING HATCH"));


 hatchServo.write(SERVO_CW);
 if (!safeDelay(T_CLOSE_PULSE)) { hatchServo.write(SERVO_STOP); return; }
 hatchServo.write(SERVO_STOP);


 if (!safeDelay(T_CLOSE_WAIT)) return;


 // Verify hatch is closed (with timeout)
 int tries = 0;
 while (!isInnerHatchClosed() && tries < 50) {
   if (!safeDelay(100)) return;
   tries++;
 }


 if (!isInnerHatchClosed()) {
   oledError(F("HATCH ERROR"), F("Not closed"));
   delay(2500);
   oledIdle();
   return;
 }


 // PHASE 5 - Repressurize
 setAllLEDs(C(0, 255, 255)); // cyan
 oledPhase(F("PHASE 5"), F("REPRESSURIZING"));
 if (!safeDelay(T_REPRESSURIZE)) return;


 // COMPLETE
 setAllLEDs(C(0, 255, 0)); // green
 if (oledWorking) {
   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0, 0);
   display.println(F("CYCLE COMPLETE"));
   display.println(F("Trash ejected"));
   display.println(F("Safe to open"));
   display.display();
 }


 delay(2500);
 hatchServo.write(SERVO_STOP);


 // Reset state for next cycle
 resetDistanceFilterToEmpty();
 trashPresent = false;
 lastTrashState = false;
 oledIdle();
}



