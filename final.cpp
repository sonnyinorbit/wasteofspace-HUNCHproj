#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <Stepper.h>

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledWorking = false;

// ================= PINS =================
const int NEOPIXEL_PIN     = 2;
const int LIMIT_SWITCH_PIN = 3;  // INPUT_PULLUP, pressed/closed = LOW
const int BUTTON_PIN       = 4;  // pressed = HIGH (your wiring)
const int SERVO_PIN        = 5;

// Stepper pins
const int STEPPER_PIN_1 = 9;
const int STEPPER_PIN_2 = 10;
const int STEPPER_PIN_3 = 11;
const int STEPPER_PIN_4 = 12;

// ================= NEOPIXEL =================
const int NUM_PIXELS = 11;
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ================= SERVO =================
Servo hatchServo;
const int CLOSED_ANGLE = 80;
const int OPEN_ANGLE   = 180;
const unsigned long SERVO_SETTLE_MS = 600;

// ================= STEPPER =================
const int MOTOR_STEPS_PER_REV = 200;
Stepper ejectStepper(200, 9, 10, 11, 12);
const int STEPPER_RPM = 30;

// Your ~360° eject output (with 125:1) -> ~25000 motor steps (as you’ve been using)
const long EJECT_STEPS = 25000;

// ================= TIMING =================
const unsigned long T_DEPRESSURIZE = 2000;
const unsigned long T_OPEN_HATCH   = 2000;
const unsigned long T_EJECT_PAUSE  = 1000;
const unsigned long T_CLOSE_HATCH  = 2000;
const unsigned long T_REPRESSURIZE = 8000;

// ================= BUTTON DEBOUNCE / EDGE =================
int btnStable = LOW;
int btnLastReading = LOW;
unsigned long btnLastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 35;

bool btnRisingEdge = false; // set true for ONE loop tick when a clean rising edge happens

void updateButton() {
int reading = digitalRead(BUTTON_PIN);

if (reading != btnLastReading) {
 btnLastChangeMs = millis();
 btnLastReading = reading;
}

// if stable long enough, accept new state
if ((millis() - btnLastChangeMs) > DEBOUNCE_MS) {
 if (btnStable != reading) {
   btnStable = reading;
   if (btnStable == HIGH) {
     btnRisingEdge = true; // latch a clean press
   }
 }
}
}

bool consumeButtonPress() {
if (btnRisingEdge) {
 btnRisingEdge = false;
 return true;
}
return false;
}

// ================= HELPERS =================
bool hatchClosed() {
// INPUT_PULLUP => pressed/closed == LOW
return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}

void setAllLEDs(uint32_t color) {
for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, color);
strip.show();
}

uint32_t wheel(byte pos) {
pos = 255 - pos;
if (pos < 85)   return strip.Color(255 - pos * 3, 0, pos * 3);
if (pos < 170)  { pos -= 85;  return strip.Color(0, pos * 3, 255 - pos * 3); }
pos -= 170;
return strip.Color(pos * 3, 255 - pos * 3, 0);
}

void rainbowFor(unsigned long ms, int waitMs = 12) {
unsigned long start = millis();
byte j = 0;
while (millis() - start < ms) {
 for (int i = 0; i < NUM_PIXELS; i++) {
   strip.setPixelColor(i, wheel((j + i * (256 / NUM_PIXELS)) & 255));
 }
 strip.show();
 j++;
 delay(waitMs);
}
}

void showText(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr) {
if (!oledWorking) return;
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0, 0);
if (line1) display.println(line1);
if (line2) display.println(line2);
if (line3) display.println(line3);
display.display();
}

// Abort rules DURING cycle:
// - hatch opened (limit not pressed) OR
// - user presses button again (a NEW press)
bool abortRequestedDuringCycle() {
if (!hatchClosed()) return true;
if (consumeButtonPress()) return true;
return false;
}

// Delay that keeps checking abort + button debounce
bool safeDelay(unsigned long ms) {
unsigned long start = millis();
while (millis() - start < ms) {
 updateButton();
 if (abortRequestedDuringCycle()) return false;
 delay(10);
}
return true;
}
// Chunked stepping so we can check abort in between
bool safeStepperMove(long steps) {
const int CHUNK = 50; // smaller chunk = more responsive abort
long remaining = steps;

while (remaining > 0) {
 updateButton();
 if (abortRequestedDuringCycle()) return false;

 int thisChunk = (remaining > CHUNK) ? CHUNK : (int)remaining;
 ejectStepper.step(thisChunk);
 remaining -= thisChunk;
}
return true;
}

// ================= IDLE (READY) ANIMATION =================
unsigned long idleLastUiMs = 0;
unsigned long idleLastLedMs = 0;
bool idleFlip = false;
byte idleRainbowJ = 0;

void idleTick() {
// LED idle animation
if (millis() - idleLastLedMs > 35) {
 idleLastLedMs = millis();

 if (!hatchClosed()) {
   // warning: orange/red pulse-ish
   byte b = (byte)( (millis() / 6) % 255 );
   uint32_t c = strip.Color(255, (b < 128 ? 80 : 20), 0);
   setAllLEDs(c);
 } else {
   // ready: subtle moving blue-cyan gradient
   for (int i = 0; i < NUM_PIXELS; i++) {
     // mix a little rainbow wheel but keep it "cool"
     uint32_t w = wheel((idleRainbowJ + i * (256 / NUM_PIXELS)) & 255);
     // "cool it down" by zeroing red a bit
     uint8_t r = (uint8_t)(w >> 16);
     uint8_t g = (uint8_t)(w >> 8);
     uint8_t b = (uint8_t)(w);
     r = r / 6;           // reduce red a lot
     g = (g * 2) / 3;     // moderate green
     // keep blue
     strip.setPixelColor(i, strip.Color(r, g, b));
   }
   strip.show();
   idleRainbowJ++;
 }
}

// OLED idle “cycle”
if (millis() - idleLastUiMs > 700) {
 idleLastUiMs = millis();
 idleFlip = !idleFlip;

 if (!hatchClosed()) {
   showText("CLOSE HATCH", "Limit not pressed", "Then press button");
 } else {
   if (idleFlip) showText("READY", "Press button to eject", "");
   else          showText("READY", "Hatch closed OK", "Waiting...");
 }
}
}
// ================= CYCLE UI =================
void abortCycleUI() {
setAllLEDs(strip.Color(255, 0, 0));
showText("CYCLE ABORTED", "Hatch opened or", "button pressed");
delay(1200);
}
// One-shot startup splash
void startupSplash() {
if (oledWorking) {
 display.clearDisplay();
 display.setTextSize(2);
 display.setTextColor(WHITE);
 display.setCursor(10, 8);
 display.print("TRASH");
 display.display();
 rainbowFor(900);

 display.clearDisplay();
 display.setTextSize(2);
 display.setCursor(5, 8);
 display.print("EJECTOR");
 display.display();
 rainbowFor(900);
} else {
 rainbowFor(1200);
}
}

// ================= MAIN CYCLE =================
void runCycle() {
if (!hatchClosed()) return;

// Clear any “start press” so holding it doesn’t abort immediately
btnRisingEdge = false;
delay(50);

// PHASE 1
setAllLEDs(strip.Color(255, 165, 0));
showText("PHASE 1", "DEPRESSURIZING", "Do not open hatch");
if (!safeDelay(T_DEPRESSURIZE)) { abortCycleUI(); return; }

// PHASE 2 - OPEN
setAllLEDs(strip.Color(255, 255, 0));
showText("PHASE 2", "OPENING HATCH", "Do not open hatch");
hatchServo.write(OPEN_ANGLE);
if (!safeDelay(SERVO_SETTLE_MS)) { abortCycleUI(); return; }
if (!safeDelay(T_OPEN_HATCH))    { abortCycleUI(); return; }

// PHASE 3 - EJECT
setAllLEDs(strip.Color(255, 120, 0));
showText("PHASE 3", "EJECTING TRASH", "Do not open hatch");
{
 unsigned long ejectStart = millis();
 while (millis() - ejectStart < 15000) {
   updateButton();
   if (!hatchClosed()) { abortCycleUI(); return; }
   if (consumeButtonPress()) { abortCycleUI(); return; }
   ejectStepper.step(10);
 }
}
if (!safeDelay(T_EJECT_PAUSE)) { abortCycleUI(); return; }

delay(1000);
// PHASE 4 - CLOSE
setAllLEDs(strip.Color(255, 255, 0));
showText("PHASE 4", "CLOSING HATCH", "");
hatchServo.write(CLOSED_ANGLE);
if (!safeDelayButtonOnly(SERVO_SETTLE_MS)) { abortCycleUI(); return; }
if (!safeDelayButtonOnly(T_CLOSE_HATCH))   { abortCycleUI(); return; }

delay(2000);
// PHASE 5
setAllLEDs(strip.Color(0, 255, 255));
showText("PHASE 5", "REPRESSURIZING", "");
if (!safeDelayButtonOnly(T_DEPRESSURIZE)) { abortCycleUI(); return; }
delay(2000);

// COMPLETE
setAllLEDs(strip.Color(0, 255, 0));
showText("CYCLE COMPLETE", "Safe to open", "");
delay(2000);
}

bool safeDelayButtonOnly(unsigned long ms) {
 unsigned long start = millis();
 while (millis() - start < ms) {
   updateButton();
   if (consumeButtonPress()) {
     Serial.println("ABORT: button in safeDelayButtonOnly");
     return false;
   }
   delay(10);
 }
 return true;
}

// ================= SETUP/LOOP =================
void setup() {
 Serial.begin(9600);
pinMode(BUTTON_PIN, INPUT);              // pressed = HIGH (external wiring)
pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP); // pressed = LOW

strip.begin();
strip.setBrightness(60);
strip.show();

Wire.begin();
Wire.setClock(400000L);

oledWorking = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

hatchServo.attach(SERVO_PIN);
hatchServo.write(CLOSED_ANGLE);

ejectStepper.setSpeed(STEPPER_RPM);

// Initialize button state cleanly
btnLastReading = digitalRead(BUTTON_PIN);
btnStable = btnLastReading;
btnLastChangeMs = millis();
btnRisingEdge = false;

startupSplash();
}

void loop() {
updateButton();

// Always run READY animation
idleTick();

// Start cycle only on a clean press AND hatch closed
if (consumeButtonPress()) {
 if (hatchClosed()) {
   runCycle();
 }
}

delay(5);
}
