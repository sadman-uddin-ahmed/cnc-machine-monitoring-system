#include <Arduino.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
// Pin mapping for working setup
static const int DHT_PIN = 16;
static const int DHT_TYPE = DHT11;
static const int LDR_PIN = 34;
static const int ENC_A_PIN = 32;
static const int ENC_B_PIN = 33;
static const int ENC_BTN_PIN = 13;
static const int RESET_BTN_PIN = 27;
static const int BUZZER_PIN = 4;
static const int LED_R_PIN = 25;
static const int LED_G_PIN = 26;
static const int LED_B_PIN = 14;
static const int TFT_SCK_PIN = 18;
static const int TFT_MOSI_PIN = 23;
static const int TFT_CS_PIN = 21;
static const int TFT_RST_PIN = 17;
static const int TFT_DC_PIN = 22;
// Main hardware objects
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
// System timing constants
static const unsigned long SENSOR_INTERVAL_MS = 2000;
static const unsigned long DISPLAY_INTERVAL_MS = 300;
static const unsigned long SERIAL_INTERVAL_MS = 5000;
static const unsigned long LOG_INTERVAL_MS = 30000;
static const unsigned long BLINK_INTERVAL_MS = 500;
static const unsigned long RESET_HOLD_MS = 5000;
static const unsigned long PEAK_WINDOW_MS = 2UL * 60UL * 60UL * 1000UL;
static const unsigned long LOCK_WINDOW_MS = 15UL * 60UL * 1000UL;
static const unsigned long LOCK_UNSAFE_TOTAL_MS = 5UL * 60UL * 1000UL;
// Demo timing constants
static const unsigned long INSPECTION_DEMO_MS = 15000;
static const unsigned long SAFE_HOLD_MS = 5000;
static const unsigned long RECOVERY_DELAY_MS = 2000;
// Buzzer timing constants
static const uint16_t BUZZER_UNSAFE_FREQ = 2000;
static const uint16_t BUZZER_INSPECT_FREQ = 1800;
static const uint16_t BUZZER_LOCKOUT_FREQ = 1000;
static const unsigned long UNSAFE_BEEP_PERIOD_MS = 2000;
static const unsigned long INSPECT_BEEP_PERIOD_MS = 3000;
static const unsigned long LOCKOUT_BEEP_PERIOD_MS = 30000;
static const unsigned long BEEP_SHORT_MS = 120;
static const unsigned long BEEP_CHIRP_MS = 70;
static const unsigned long BEEP_LOCKOUT_MS = 250;
// Modes and states
enum BaseMode{
  MODE_STANDARD = 0,
  MODE_HIGH_SENSITIVITY,
  MODE_USER_EDITABLE
};
enum SystemState{
  STATE_SAFE = 0,
  STATE_UNSAFE,
  STATE_INSPECTION,
  STATE_LOCKOUT
};
enum EditField{
  EDIT_NONE = 0,
  EDIT_TEMP_MIN,
  EDIT_TEMP_MAX,
  EDIT_HUM_MIN,
  EDIT_HUM_MAX
};
// Demo phase selection
enum DemoPhase{
  DEMO_PHASE_INSPECTION = 0,
  DEMO_PHASE_SAFE_HOLD,
  DEMO_PHASE_UNSAFE_HOLD,
  DEMO_PHASE_RECOVERY_WAIT
};
// Threshold structure
struct Thresholds{
  float tMin;
  float tMax;
  float hMin;
  float hMax;
};
// RAM log structure
struct LogEntry{
  uint32_t ms;
  float temp;
  float hum;
  uint16_t light;
  uint8_t mode;
  uint8_t state;
};
// Threshold presets
Thresholds standardThr = {20.0f,30.0f,30.0f,70.0f};
Thresholds highThr = {22.0f,28.0f,35.0f,65.0f};
Thresholds userThr = {21.0f,29.0f,40.0f,60.0f};
// Main state variables
BaseMode selectedMode = MODE_STANDARD;
SystemState currentState = STATE_SAFE;
EditField editField = EDIT_NONE;
float currentTemp = NAN;
float currentHum = NAN;
uint16_t currentLight = 0;
int ldrThreshold = 1800;
bool blinkPhase = false;
bool screenDrawn = false;
int lastEncA = HIGH;
bool unsafeActive = false;
unsigned long unsafeStartMs = 0;
unsigned long unsafeAccumulatedMs = 0;
int unsafeEpisodes = 0;
bool resetHoldActive = false;
unsigned long resetHoldStartMs = 0;
unsigned long lastBlinkMs = 0;
unsigned long lastSensorMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;
unsigned long lastLogMs = 0;
// Demo cycle variables
DemoPhase currentDemoPhase = DEMO_PHASE_INSPECTION;
unsigned long demoPhaseStartMs = 0;
// Buzzer scheduler variables
bool buzzerPulseActive = false;
unsigned long buzzerPulseEndMs = 0;
unsigned long lastUnsafeBuzzMs = 0;
unsigned long lastInspectBuzzMs = 0;
unsigned long lastLockoutBuzzMs = 0;
bool inspectSecondChirpPending = false;
unsigned long inspectSecondChirpAtMs = 0;
// RAM log storage
static const int LOG_CAPACITY = 2880;
LogEntry logs[LOG_CAPACITY];
int logWriteIndex = 0;
bool logWrapped = false;
// Buzzer PWM config
static const int BUZZER_CHANNEL = 0;
static const int BUZZER_RESOLUTION = 8;
static const int BUZZER_BASE_FREQ = 2000;
// Text helpers
const char* baseModeName(BaseMode m){
  switch(m){
    case MODE_STANDARD: return "STANDARD";
    case MODE_HIGH_SENSITIVITY: return "HIGH-SENS";
    case MODE_USER_EDITABLE: return "USER";
    default: return "UNKNOWN";
  }
}
const char* stateName(SystemState s){
  switch(s){
    case STATE_SAFE: return "NORMAL";
    case STATE_UNSAFE: return "UNSAFE";
    case STATE_INSPECTION: return "INSPECTION";
    case STATE_LOCKOUT: return "LOCK OUT";
    default: return "UNKNOWN";
  }
}
const char* editFieldName(EditField e){
  switch(e){
    case EDIT_TEMP_MIN: return "T_MIN";
    case EDIT_TEMP_MAX: return "T_MAX";
    case EDIT_HUM_MIN: return "H_MIN";
    case EDIT_HUM_MAX: return "H_MAX";
    default: return "-";
  }
}
const char* demoPhaseName(DemoPhase p){
  switch(p){
    case DEMO_PHASE_INSPECTION: return "INSPECT";
    case DEMO_PHASE_SAFE_HOLD: return "SAFE HOLD";
    case DEMO_PHASE_UNSAFE_HOLD: return "UNSAFE";
    case DEMO_PHASE_RECOVERY_WAIT: return "RECOVER";
    default: return "UNKNOWN";
  }
}
// Return active thresholds
Thresholds getActiveThresholds(){
  switch(selectedMode){
    case MODE_STANDARD: return standardThr;
    case MODE_HIGH_SENSITIVITY: return highThr;
    case MODE_USER_EDITABLE: return userThr;
    default: return standardThr;
  }
}
// RGB LED control (This assumes common cathode.)
void setRgb(bool r,bool g,bool b){
  digitalWrite(LED_R_PIN,r ? HIGH : LOW);
  digitalWrite(LED_G_PIN,g ? HIGH : LOW);
  digitalWrite(LED_B_PIN,b ? HIGH : LOW);
}
// Buzzer low-level control
void buzzerOn(uint16_t freq){
  ledcWriteTone(BUZZER_CHANNEL,freq);
}
void buzzerOff(){
  ledcWriteTone(BUZZER_CHANNEL,0);
}
void startBuzzerPulse(uint16_t freq,unsigned long durationMs){
  buzzerOn(freq);
  buzzerPulseActive = true;
  buzzerPulseEndMs = millis() + durationMs;
}
// Sensor meaning (Dark/covered = machine stopped)
bool isMachineStoppedByLdr(){
  return currentLight < ldrThreshold;
}
// Threshold comparison
bool isWithinThresholds(float temp,float hum,const Thresholds& thr){
  if(isnan(temp) || isnan(hum)) return false;
  return (temp >= thr.tMin && temp <= thr.tMax && hum >= thr.hMin && hum <= thr.hMax);
}
// Add a log entry to RAM
void addLogEntry(){
  logs[logWriteIndex].ms = millis();
  logs[logWriteIndex].temp = currentTemp;
  logs[logWriteIndex].hum = currentHum;
  logs[logWriteIndex].light = currentLight;
  logs[logWriteIndex].mode = static_cast<uint8_t>(selectedMode);
  logs[logWriteIndex].state = static_cast<uint8_t>(currentState);
  logWriteIndex++;
  if(logWriteIndex >= LOG_CAPACITY){
    logWriteIndex = 0;
    logWrapped = true;
  }
}
// Calculate 2-hour peaks
void calculatePeaks2Hours(float& peakTemp,float& peakHum){
  peakTemp = -1000.0f;
  peakHum = -1000.0f;
  unsigned long now = millis();
  int count = logWrapped ? LOG_CAPACITY : logWriteIndex;
  for(int i=0;i<count;i++){
    if((now - logs[i].ms) <= PEAK_WINDOW_MS){
      if(!isnan(logs[i].temp) && logs[i].temp > peakTemp) peakTemp = logs[i].temp;
      if(!isnan(logs[i].hum) && logs[i].hum > peakHum) peakHum = logs[i].hum;
    }
  }
  if(peakTemp < -999.0f) peakTemp = currentTemp;
  if(peakHum < -999.0f) peakHum = currentHum;
}
// Serial status report
void printSerialStatus(){
  Serial.println("--------------------------------------------------");
  Serial.print("Mode: "); Serial.println(baseModeName(selectedMode));
  Serial.print("State: "); Serial.println(stateName(currentState));
  Serial.print("Demo Phase: "); Serial.println(demoPhaseName(currentDemoPhase));
  Serial.print("Temp: "); Serial.print(currentTemp); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(currentHum); Serial.println(" %");
  Serial.print("Light ADC: "); Serial.println(currentLight);
  Serial.print("LDR threshold: "); Serial.println(ldrThreshold);
  Serial.print("Machine stopped?: "); Serial.println(isMachineStoppedByLdr() ? "YES" : "NO");
  Serial.print("Unsafe episodes: "); Serial.println(unsafeEpisodes);
  Serial.print("Unsafe accumulated ms: "); Serial.println(unsafeAccumulatedMs);
  Serial.print("User thresholds: T["); Serial.print(userThr.tMin); Serial.print(","); Serial.print(userThr.tMax); Serial.print("] H["); Serial.print(userThr.hMin); Serial.print(","); Serial.print(userThr.hMax); Serial.println("]");
}
// Blink phase update
void updateBlink(){
  unsigned long now = millis();
  if(now - lastBlinkMs >= BLINK_INTERVAL_MS){
    lastBlinkMs = now;
    blinkPhase = !blinkPhase;
  }
}
// Read DHT and LDR
void updateSensors(){
  unsigned long now = millis();
  if(now - lastSensorMs >= SENSOR_INTERVAL_MS){
    lastSensorMs = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if(!isnan(t)) currentTemp = t;
    if(!isnan(h)) currentHum = h;
  }
  currentLight = analogRead(LDR_PIN);
}
// Update unsafe tracking (For lockout condition)
void updateUnsafeTracking(bool unsafeNow){
  unsigned long now = millis();
  static unsigned long lastUnsafeSeenMs = 0;
  if(unsafeNow) lastUnsafeSeenMs = now;
  if((now - lastUnsafeSeenMs) > LOCK_WINDOW_MS){
    unsafeAccumulatedMs = 0;
    unsafeEpisodes = 0;
    unsafeActive = false;
  }
  if(unsafeNow && !unsafeActive){
    unsafeActive = true;
    unsafeStartMs = now;
    unsafeEpisodes++;
  } else if(!unsafeNow && unsafeActive){
    unsafeActive = false;
    unsafeAccumulatedMs += (now - unsafeStartMs);
  }
  if(unsafeActive){
    unsigned long liveAccum = unsafeAccumulatedMs + (now - unsafeStartMs);
    if(unsafeEpisodes >= 2 && liveAccum >= LOCK_UNSAFE_TOTAL_MS){
      currentState = STATE_LOCKOUT;
    }
  }
}
// Main demo state machine
void updateStateMachine(){
  if(currentState == STATE_LOCKOUT) return;
  unsigned long now = millis();
  Thresholds thr = getActiveThresholds();
  bool safeNow = isWithinThresholds(currentTemp,currentHum,thr);
  bool unsafeNow = !safeNow;
  updateUnsafeTracking(unsafeNow);
  if(currentState == STATE_LOCKOUT) return;
  switch(currentDemoPhase){
    case DEMO_PHASE_INSPECTION:
      currentState = STATE_INSPECTION;
      if(unsafeNow){
        currentDemoPhase = DEMO_PHASE_UNSAFE_HOLD;
        demoPhaseStartMs = now;
        currentState = STATE_UNSAFE;
      } else if(now - demoPhaseStartMs >= INSPECTION_DEMO_MS){
        currentDemoPhase = DEMO_PHASE_SAFE_HOLD;
        demoPhaseStartMs = now;
        currentState = STATE_SAFE;
      }
      break;
    case DEMO_PHASE_SAFE_HOLD:
      if(unsafeNow){
        currentDemoPhase = DEMO_PHASE_UNSAFE_HOLD;
        demoPhaseStartMs = now;
        currentState = STATE_UNSAFE;
      } else {
        currentState = STATE_SAFE;
        if(now - demoPhaseStartMs >= SAFE_HOLD_MS){
          currentDemoPhase = DEMO_PHASE_INSPECTION;
          demoPhaseStartMs = now;
          currentState = STATE_INSPECTION;
        }
      }
      break;
    case DEMO_PHASE_UNSAFE_HOLD:
      currentState = STATE_UNSAFE;
      if(safeNow){
        currentDemoPhase = DEMO_PHASE_RECOVERY_WAIT;
        demoPhaseStartMs = now;
      }
      break;
    case DEMO_PHASE_RECOVERY_WAIT:
      if(unsafeNow){
        currentDemoPhase = DEMO_PHASE_UNSAFE_HOLD;
        demoPhaseStartMs = now;
        currentState = STATE_UNSAFE;
      } else {
        currentState = STATE_SAFE;
        if(now - demoPhaseStartMs >= RECOVERY_DELAY_MS){
          currentDemoPhase = DEMO_PHASE_INSPECTION;
          demoPhaseStartMs = now;
          currentState = STATE_INSPECTION;
        }
      }
      break;
  }
}
// External reset button logic (Only clears LOCKOUT after hold)
void updateResetButton(){
  bool pressed = (digitalRead(RESET_BTN_PIN) == LOW);
  unsigned long now = millis();
  if(currentState == STATE_LOCKOUT){
    if(pressed){
      if(!resetHoldActive){
        resetHoldActive = true;
        resetHoldStartMs = now;
      } else if((now - resetHoldStartMs) >= RESET_HOLD_MS){
        currentState = STATE_SAFE;
        unsafeAccumulatedMs = 0;
        unsafeEpisodes = 0;
        unsafeActive = false;
        resetHoldActive = false;
        currentDemoPhase = DEMO_PHASE_INSPECTION;
        demoPhaseStartMs = now;
      }
    } else {
      resetHoldActive = false;
    }
  } else {
    resetHoldActive = false;
  }
}
// LED behavior by state
void updateRgbByState(){
  switch(currentState){
    case STATE_SAFE:
      setRgb(false,true,false);
      break;
    case STATE_UNSAFE:
      setRgb(true,false,false);
      break;
    case STATE_INSPECTION:
      setRgb(false,false,blinkPhase);
      break;
    case STATE_LOCKOUT:
      if(blinkPhase) setRgb(true,false,false);
      else setRgb(false,false,true);
      break;
  }
}
// Non-blocking buzzer behavior
void updateBuzzerByState(){
  unsigned long now = millis();
  if(buzzerPulseActive && now >= buzzerPulseEndMs){
    buzzerOff();
    buzzerPulseActive = false;
  }
  if(inspectSecondChirpPending && now >= inspectSecondChirpAtMs){
    startBuzzerPulse(BUZZER_INSPECT_FREQ,BEEP_CHIRP_MS);
    inspectSecondChirpPending = false;
  }
  switch(currentState){
    case STATE_SAFE:
      if(!buzzerPulseActive) buzzerOff();
      break;
    case STATE_UNSAFE:
      if((now - lastUnsafeBuzzMs) >= UNSAFE_BEEP_PERIOD_MS && !buzzerPulseActive){
        lastUnsafeBuzzMs = now;
        startBuzzerPulse(BUZZER_UNSAFE_FREQ,BEEP_SHORT_MS);
      }
      break;
    case STATE_INSPECTION:
      if((now - lastInspectBuzzMs) >= INSPECT_BEEP_PERIOD_MS && !buzzerPulseActive && !inspectSecondChirpPending){
        lastInspectBuzzMs = now;
        startBuzzerPulse(BUZZER_INSPECT_FREQ,BEEP_CHIRP_MS);
        inspectSecondChirpPending = true;
        inspectSecondChirpAtMs = now + 2 * BEEP_CHIRP_MS;
      }
      break;
    case STATE_LOCKOUT:
      if((now - lastLockoutBuzzMs) >= LOCKOUT_BEEP_PERIOD_MS && !buzzerPulseActive){
        lastLockoutBuzzMs = now;
        startBuzzerPulse(BUZZER_LOCKOUT_FREQ,BEEP_LOCKOUT_MS);
      }
      break;
  }
}
// Draw static TFT layout once (This avoids whole-screen flicker)
void drawStaticScreen(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(2,2);
  tft.println("CNC MONITOR");
  tft.drawLine(0,12,127,12,ST77XX_WHITE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2,18); tft.println("State:");
  tft.setCursor(2,30); tft.println("Mode :");
  tft.setCursor(2,42); tft.println("Temp :");
  tft.setCursor(2,54); tft.println("Hum  :");
  tft.setCursor(2,66); tft.println("LDR  :");
  tft.setCursor(2,78); tft.println("Mach :");
  tft.setCursor(2,90); tft.println("PkT2h:");
  tft.setCursor(66,90); tft.println("PkH:");
  tft.setCursor(2,104); tft.println("Light:");
  tft.setCursor(2,116); tft.println("User :");
  screenDrawn = true;
}
// Helper to clear text area
void clearTextArea(int x,int y,int w,int h){
  tft.fillRect(x,y,w,h,ST77XX_BLACK);
}
// Update only dynamic TFT fields (This keeps display stable)
void updateDisplay(){
  unsigned long now = millis();
  if(now - lastDisplayMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayMs = now;
  if(!screenDrawn) drawStaticScreen();
  float peakT,peakH;
  calculatePeaks2Hours(peakT,peakH);
  clearTextArea(44,18,82,10);
  tft.setCursor(44,18);
  if(currentState == STATE_SAFE) tft.setTextColor(ST77XX_GREEN);
  else if(currentState == STATE_UNSAFE) tft.setTextColor(ST77XX_RED);
  else if(currentState == STATE_INSPECTION) tft.setTextColor(ST77XX_BLUE);
  else tft.setTextColor(ST77XX_MAGENTA);
  tft.print(stateName(currentState));
  clearTextArea(44,30,82,10);
  tft.setCursor(44,30);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(baseModeName(selectedMode));
  clearTextArea(44,42,82,10);
  tft.setCursor(44,42);
  if(isnan(currentTemp)) tft.print("ERR");
  else { tft.print(currentTemp,1); tft.print(" C"); }
  clearTextArea(44,54,82,10);
  tft.setCursor(44,54);
  if(isnan(currentHum)) tft.print("ERR");
  else { tft.print(currentHum,1); tft.print(" %"); }
  clearTextArea(44,66,82,10);
  tft.setCursor(44,66);
  tft.print(currentLight);
  clearTextArea(44,78,82,10);
  tft.setCursor(44,78);
  if(currentState == STATE_INSPECTION) tft.print("STOPPED");
  else tft.print("RUNNING");
  clearTextArea(44,90,20,10);
  tft.setCursor(44,90);
  tft.print(peakT,1);
  clearTextArea(94,90,30,10);
  tft.setCursor(94,90);
  tft.print(peakH,1);
  clearTextArea(44,104,82,10);
  tft.setCursor(44,104);
  if(currentState == STATE_LOCKOUT){
    tft.setTextColor(ST77XX_RED);
    tft.print("LOCK OUT");
  } else if(currentState == STATE_INSPECTION){
    tft.setTextColor(ST77XX_BLUE);
    tft.print("BLUE FLASH");
  } else if(currentState == STATE_UNSAFE){
    tft.setTextColor(ST77XX_RED);
    tft.print("RED ALERT");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("GREEN OK");
  }
  clearTextArea(44,116,82,10);
  tft.setCursor(44,116);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print((int)userThr.tMin); tft.print("/");
  tft.print((int)userThr.tMax); tft.print(" ");
  tft.print((int)userThr.hMin); tft.print("/");
  tft.print((int)userThr.hMax);
}
// Log data every 30 seconds
void logDataIfDue(){
  unsigned long now = millis();
  if(now - lastLogMs >= LOG_INTERVAL_MS){
    lastLogMs = now;
    addLogEntry();
  }
}
// Serial print every 5 seconds
void serialIfDue(){
  unsigned long now = millis();
  if(now - lastSerialMs >= SERIAL_INTERVAL_MS){
    lastSerialMs = now;
    printSerialStatus();
  }
}
// Change mode or edit values (Rotation is used for both)
void handleEncoderTurn(int dir){
  if(currentState == STATE_LOCKOUT) return;
  if(editField == EDIT_NONE){
    if(dir > 0) selectedMode = static_cast<BaseMode>((selectedMode + 1) % 3);
    else if(dir < 0) selectedMode = static_cast<BaseMode>((selectedMode + 2) % 3);
  } else {
    float step = 1.0f;
    switch(editField){
      case EDIT_TEMP_MIN: userThr.tMin += dir * step; break;
      case EDIT_TEMP_MAX: userThr.tMax += dir * step; break;
      case EDIT_HUM_MIN: userThr.hMin += dir * step; break;
      case EDIT_HUM_MAX: userThr.hMax += dir * step; break;
      default: break;
    }
    userThr.tMin = constrain(userThr.tMin,0.0f,60.0f);
    userThr.tMax = constrain(userThr.tMax,0.0f,60.0f);
    userThr.hMin = constrain(userThr.hMin,0.0f,100.0f);
    userThr.hMax = constrain(userThr.hMax,0.0f,100.0f);
    if(userThr.tMin >= userThr.tMax){
      if(editField == EDIT_TEMP_MIN) userThr.tMin = userThr.tMax - 1.0f;
      else userThr.tMax = userThr.tMin + 1.0f;
    }
    if(userThr.hMin >= userThr.hMax){
      if(editField == EDIT_HUM_MIN) userThr.hMin = userThr.hMax - 1.0f;
      else userThr.hMax = userThr.hMin + 1.0f;
    }
  }
}
// Encoder push-button control (Short press = next edit field, Long press = enter/exit USER edit)
void handleEncoderButton(){
  static unsigned long pressStart = 0;
  static bool prevPressed = false;
  bool pressed = (digitalRead(ENC_BTN_PIN) == LOW);
  if(pressed && !prevPressed) pressStart = millis();
  if(!pressed && prevPressed){
    unsigned long held = millis() - pressStart;
    if(held >= 800){
      if(selectedMode == MODE_USER_EDITABLE){
        if(editField == EDIT_NONE) editField = EDIT_TEMP_MIN;
        else editField = EDIT_NONE;
      }
    } else {
      if(editField != EDIT_NONE){
        switch(editField){
          case EDIT_TEMP_MIN: editField = EDIT_TEMP_MAX; break;
          case EDIT_TEMP_MAX: editField = EDIT_HUM_MIN; break;
          case EDIT_HUM_MIN: editField = EDIT_HUM_MAX; break;
          case EDIT_HUM_MAX: editField = EDIT_NONE; break;
          default: editField = EDIT_NONE; break;
        }
      }
    }
  }
  prevPressed = pressed;
}
// Read encoder A/B
void handleEncoderRotation(){
  int a = digitalRead(ENC_A_PIN);
  int b = digitalRead(ENC_B_PIN);
  if(a != lastEncA){
    if(a == LOW){
      int dir = (b == HIGH) ? 1 : -1;
      handleEncoderTurn(dir);
    }
  }
  lastEncA = a;
}
// Set all pin directions
void setupPins(){
  pinMode(ENC_A_PIN,INPUT_PULLUP);
  pinMode(ENC_B_PIN,INPUT_PULLUP);
  pinMode(ENC_BTN_PIN,INPUT_PULLUP);
  pinMode(RESET_BTN_PIN,INPUT_PULLUP);
  pinMode(LED_R_PIN,OUTPUT);
  pinMode(LED_G_PIN,OUTPUT);
  pinMode(LED_B_PIN,OUTPUT);
  analogReadResolution(12);
}
// Setup buzzer PWM
void setupBuzzer(){
  ledcSetup(BUZZER_CHANNEL,BUZZER_BASE_FREQ,BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN,BUZZER_CHANNEL);
  buzzerOff();
}
// Setup TFT using working config (TFT 3.3V -> ESP32 3.3V, TFT Lite -> ESP32 3.3V)
void setupDisplay(){
  SPI.begin(TFT_SCK_PIN,-1,TFT_MOSI_PIN,TFT_CS_PIN);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawStaticScreen();
}
// setup(Runs once)
void setup(){
  Serial.begin(115200);
  delay(500);
  setupPins();
  setupBuzzer();
  dht.begin();
  setupDisplay();
  setRgb(false,false,false);
  currentLight = analogRead(LDR_PIN);
  currentTemp = dht.readTemperature();
  currentHum = dht.readHumidity();
  lastEncA = digitalRead(ENC_A_PIN);
  demoPhaseStartMs = millis();
  currentDemoPhase = DEMO_PHASE_INSPECTION;
  Serial.println("ESP32 CNC Monitoring System Started");
}
// loop(Main repeating program)
void loop(){
  updateBlink();
  updateSensors();
  handleEncoderRotation();
  handleEncoderButton();
  updateStateMachine();
  updateResetButton();
  updateRgbByState();
  updateBuzzerByState();
  updateDisplay();
  logDataIfDue();
  serialIfDue();
}