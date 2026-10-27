/*
  ESP32-C3 SuperMini + TF Luna (UART) + BLE Gate Timer
  ====================================================

  Passage sequence:
    1st detection -> START  : LED GREEN
    2nd detection -> STOP   : LED RED
    3rd detection -> RESET  : LED BLUE
    4th detection -> START  : repeats...

  Why TF Luna is the right part for this job
  ------------------------------------------
  100 Hz by default, configurable to 250 Hz. At 250 Hz the frame period is
  4 ms, so timing quantisation is about +/- 2 ms. A child running at 5 m/s
  covers 1 cm in that time. The TOF250 at 20 Hz gave +/- 25 ms, or 12 cm.

  Its UART logic is 3.3 V while the supply is 5 V, so no level shifter and
  no divider are needed on an ESP32-C3. This removes the entire class of
  problem that made the TOF250 wiring fragile.

  Every frame carries a checksum and a signal-strength value, so corrupt
  and unreliable readings can be rejected outright instead of being
  mistaken for a passage.

  WIRING
  ------
    TF Luna Red    +5V  -> ESP32-C3 5V      (USB must be connected)
    TF Luna Black  GND  -> ESP32-C3 GND
    TF Luna Green  TXD  -> ESP32-C3 GPIO5   (sensor output -> ESP input)
    TF Luna White  RXD  -> ESP32-C3 GPIO6   (ESP output -> sensor input)

    NeoPixel     VCC  -> 5V  (ring's own supply if you have one)
    NeoPixel     GND  -> ESP32-C3 GND        (shared ground is mandatory)
    NeoPixel     DIN  -> ESP32-C3 GPIO4      (a 330 ohm series resistor on
                                              this line suppresses ringing)

  Green is the wire that carries data out of the sensor. If nothing is
  received, swapping green and white is the first thing to try; wire colour
  conventions vary between suppliers.

  A 16-pixel ring can draw close to 1 A in a white flash, which is why
  LED_BRIGHTNESS is held at 80 - see the note beside it. If you give the ring
  a separate 5 V supply, its ground still has to meet the ESP32's or the data
  line has no reference and the colours scramble.

  WS2812B wants a 5 V data line and the ESP32-C3 drives 3.3 V, which is
  marginal on paper but reliable over the few centimetres of wire inside an
  enclosure. If the colours ever glitch, a level shifter on DIN settles it.

  BOARD SETTINGS (Tools menu) - these matter more than the code
  ------------------------------------------------------------
    Board           : ESP32C3 Dev Module
    USB CDC On Boot : Enabled     <-- REQUIRED for the USB cable connection
                                      and for the serial monitor. Bluetooth
                                      works either way, which is why a wrong
                                      setting here looks like a cable fault.
                                      On core 3.x there is no way around it:
                                      with this off, the core declares no USB
                                      serial object at all.
    JTAG Adapter    : Disabled

  BLE
  ---
  UUIDs unchanged. DISTANCE (...0001) still notifies a 2-byte little-endian
  millimetre value, so the existing web page works untouched. The EVENT
  characteristic (...0004) adds a device-side timestamp for accurate timing.
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

// ========================================================
// Options
// ========================================================

#define TARGET_FRAME_RATE   250     // 1..250 Hz
// Boot diagnostic window. It prints live distances and, if nothing arrives,
// names the wire to check. 1200 ms at 250 Hz is 300 frames - plenty to
// measure the rate and prove the sensor is alive. It was 6000 ms, which
// added six seconds to every power-on for no extra information. Raise it
// again only while hunting a wiring fault.
#define BOOT_LIVE_STREAM_MS 1200    // 0 to skip
#define HEARTBEAT_MS        700
#define AUTO_BASELINE       1       // 0 = always use FALLBACK_THRESHOLD_MM

// Used when auto-baseline is off, and whenever the background is too far or
// too weak to measure. Matched to the web page's own default so the
// threshold does not jump the moment a browser connects.
constexpr uint16_t FALLBACK_THRESHOLD_MM = 1700;

// Signal strength below this means the reading is not trustworthy.
// TF Luna reports amplitude alongside every distance.
constexpr uint16_t MIN_AMPLITUDE = 100;

// TF Luna is specified from 0.2 m to 8 m, but a gate never needs anything
// like that. Capping the accepted range at 4 m means distant walls, windows
// and passers-by beyond the lane are rejected before they reach the state
// machine at all. Readings outside this band are treated exactly like a weak
// return: ignored, changing nothing. Raise this if the background behind the
// gate sits further away than 4 m and you want it measured.
constexpr uint16_t TOF_MIN_MM = 200;
constexpr uint16_t TOF_MAX_MM = 4000;

// Consecutive agreeing frames required before acting.
//
// CONFIRM_IN is 1 because the frames are already filtered three ways before
// they get here: checksum, amplitude, and range. Requiring two CONSECUTIVE
// close frames is fragile - a runner's arm gap, or amplitude dipping under
// MIN_AMPLITUDE for a single frame, resets the counter and the whole passage
// is missed. Raise it to 2 only if false triggers appear.
constexpr uint8_t CONFIRM_IN  = 1;
constexpr uint8_t CONFIRM_OUT = 4;

// Must exceed the time a body takes to clear the beam, or one person
// counts twice when a gap between arm and torso shows through.
constexpr uint32_t LOCKOUT_MS = 400;

// ========================================================
// BLE
// ========================================================

static const char* BLE_DEVICE_NAME    = "GATE01";
static const char* SERVICE_UUID       = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char* DISTANCE_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char* THRESHOLD_UUID     = "19B10002-E8F2-537E-4F6C-D104768A1214";
static const char* RESET_UUID         = "19B10003-E8F2-537E-4F6C-D104768A1214";
static const char* EVENT_UUID         = "19B10004-E8F2-537E-4F6C-D104768A1214";

// ========================================================
// UART
// ========================================================

constexpr int      LUNA_RX_PIN = 5;        // ESP input  <- green
constexpr int      LUNA_TX_PIN = 6;        // ESP output -> white
constexpr uint32_t LUNA_BAUD   = 115200;   // TF Luna factory default

HardwareSerial luna(1);

// ========================================================
// State
// ========================================================

uint16_t currentThreshold = FALLBACK_THRESHOLD_MM;
uint16_t baselineMm       = 0;

uint8_t  passageState  = 0;   // 0 await START, 1 await STOP, 2 await RESET
bool     gateArmed     = true;
uint32_t lastTriggerMs = 0;
uint8_t  eventSeq      = 0;

uint8_t  inCount  = 0;
uint8_t  outCount = 0;

uint16_t lastMm    = 0;
uint16_t lastAmp   = 0;
bool     lastValid = false;

uint32_t frameCount = 0;
uint32_t badChecksum = 0;

// Per-heartbeat diagnostics. windowMinMm is the closest valid reading seen
// since the last status line, and windowNearFrames how many of those were
// inside the threshold. Together they separate the two failure modes: a miss
// with a high minimum means the sensor never saw the runner, while a miss with
// frames already inside the threshold means the gate logic swallowed it.
uint16_t windowMinMm      = 0xFFFF;
uint32_t windowNearFrames = 0;
uint32_t passageCount     = 0;

// ========================================================
// NeoPixel - driven outside the detection path
// ========================================================

// 16-pixel WS2812B ring.
//
// Brightness is capped at 80 of 255 on purpose. Sixteen pixels at full white
// would ask for roughly 16 x 60 mA = 960 mA, far more than the board's 5 V pin
// should source; at 80 the worst case is about 300 mA, which USB power carries
// comfortably. Raise this only if the ring has its own 5 V supply, and if you
// do, keep that supply's ground tied to the ESP32's.
constexpr int     LED_PIN        = 4;
constexpr int     LED_COUNT      = 16;
constexpr uint8_t LED_BRIGHTNESS = 80;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile uint32_t pendingColor = 0;
volatile bool     ledDirty     = false;

void requestLed(uint8_t r, uint8_t g, uint8_t b) {
  pendingColor = strip.Color(r, g, b);
  ledDirty = true;
}

void serviceLed() {
  if (!ledDirty) return;
  ledDirty = false;
  uint32_t c = pendingColor;
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

void ledNow(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

// ========================================================
// BLE
// ========================================================

NimBLECharacteristic* distanceCharacteristic  = nullptr;
NimBLECharacteristic* thresholdCharacteristic = nullptr;
NimBLECharacteristic* resetCharacteristic     = nullptr;
NimBLECharacteristic* eventCharacteristic     = nullptr;

volatile bool deviceConnected = false;

class GateServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    deviceConnected = true;
    // 1.25 ms units: 6 -> 7.5 ms. Timeout unit 10 ms: 200 -> 2 s.
    server->updateConnParams(info.getConnHandle(), 6, 12, 0, 200);
    Serial.println("[BLE] connected");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
    Serial.printf("[BLE] disconnected (%d), advertising again\n", reason);
  }
};
GateServerCallbacks serverCallbacks;

class ThresholdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    if (v.size() >= 2) {
      uint16_t t = static_cast<uint8_t>(v[0]) |
                   (static_cast<uint16_t>(static_cast<uint8_t>(v[1])) << 8);
      if (t > 0) {
        currentThreshold = t;
        Serial.printf("[Threshold] %u mm\n", currentThreshold);
      }
    }
  }
};
ThresholdCallbacks thresholdCallbacks;

class ResetCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) override {
    passageState  = 0;
    gateArmed     = true;
    lastTriggerMs = 0;
    inCount = outCount = 0;
    requestLed(0, 0, 255);
    Serial.println("[RESET] waiting for START");
  }
};
ResetCallbacks resetCallbacks;

// ========================================================
// TF Luna command helper
//
// Command frame: 0x5A, length, id, payload..., checksum
// Checksum is the low byte of the sum of all preceding bytes.
// ========================================================

void lunaSend(const uint8_t* cmd, uint8_t len) {
  luna.write(cmd, len);
  luna.flush();
  delay(120);
  while (luna.available()) luna.read();   // discard the acknowledgement
}

void lunaSetFrameRate(uint16_t hz) {
  uint8_t cmd[6] = { 0x5A, 0x06, 0x03,
                     static_cast<uint8_t>(hz & 0xFF),
                     static_cast<uint8_t>(hz >> 8), 0x00 };
  uint16_t sum = 0;
  for (int i = 0; i < 5; i++) sum += cmd[i];
  cmd[5] = sum & 0xFF;
  lunaSend(cmd, 6);
  Serial.printf("[TF Luna] frame rate request: %u Hz\n", hz);
}

void lunaSetOutputFormat9ByteCm() {
  // id 0x05, payload 0x01 = the standard nine byte centimetre frame
  uint8_t cmd[5] = { 0x5A, 0x05, 0x05, 0x01, 0x00 };
  uint16_t sum = 0;
  for (int i = 0; i < 4; i++) sum += cmd[i];
  cmd[4] = sum & 0xFF;
  lunaSend(cmd, 5);
}

void lunaEnableOutput() {
  uint8_t cmd[5] = { 0x5A, 0x05, 0x07, 0x01, 0x00 };
  uint16_t sum = 0;
  for (int i = 0; i < 4; i++) sum += cmd[i];
  cmd[4] = sum & 0xFF;
  lunaSend(cmd, 5);
}

// ========================================================
// Frame parser
//
// 0x59 0x59 | dist_L dist_H | amp_L amp_H | temp_L temp_H | checksum
//
// The timestamp is captured on the first header byte. That instant is the
// earliest software can know a new measurement exists. The residual offset
// is identical for START and STOP, so it cancels in the difference.
//
// Plain reference out-parameters, no struct: the Arduino IDE injects
// auto-generated prototypes above any user-defined type, which breaks
// structs used in function signatures.
// ========================================================

static uint8_t  fbuf[9];
static uint8_t  fidx = 0;
static uint32_t fFirstUs = 0;

bool lunaReadFrame(uint16_t &outMm, uint16_t &outAmp, bool &outValid,
                   uint32_t &outUs) {
  while (luna.available()) {
    uint32_t nowUs = micros();
    uint8_t b = luna.read();

    if (fidx == 0) {
      if (b != 0x59) continue;
      fFirstUs = nowUs;
      fbuf[fidx++] = b;
      continue;
    }

    if (fidx == 1) {
      if (b != 0x59) {          // false header, restart
        fidx = 0;
        if (b == 0x59) { fFirstUs = nowUs; fbuf[fidx++] = b; }
        continue;
      }
      fbuf[fidx++] = b;
      continue;
    }

    fbuf[fidx++] = b;
    if (fidx < 9) continue;

    fidx = 0;

    uint16_t sum = 0;
    for (int i = 0; i < 8; i++) sum += fbuf[i];

    if ((sum & 0xFF) != fbuf[8]) {
      badChecksum++;
      continue;
    }

    uint16_t distCm = fbuf[2] | (static_cast<uint16_t>(fbuf[3]) << 8);
    uint16_t amp    = fbuf[4] | (static_cast<uint16_t>(fbuf[5]) << 8);

    outUs  = fFirstUs;
    outAmp = amp;
    outMm  = distCm * 10;

    // Amplitude is how TF Luna reports confidence. Too low means no usable
    // return; 0xFFFF means the receiver is saturated. Either way the
    // distance figure is meaningless.
    bool ampOk = (amp >= MIN_AMPLITUDE) && (amp != 0xFFFF);
    bool rangeOk = (outMm >= TOF_MIN_MM) && (outMm <= TOF_MAX_MM);

    outValid = ampOk && rangeOk;
    return true;
  }
  return false;
}

// ========================================================
// Baseline calibration
// ========================================================

void calibrateBaseline() {
#if AUTO_BASELINE
  Serial.println("\n---- BASELINE ----");
  Serial.println("Keep the gate clear...");

  // 700 ms of settling, then 900 ms of averaging. The sensor is already
  // streaming by the time we get here, so the only thing this pause buys is
  // a moment for whoever pressed reset to step out of the beam.
  delay(700);

  uint32_t sum = 0;
  uint32_t n = 0;
  uint32_t t0 = millis();

  uint16_t mm, amp, dummy16;
  bool valid;
  uint32_t tUs;

  // 900 ms at 250 Hz is about 225 samples, ten times the 20 the test below
  // demands. Averaging longer than this does not make the number better.
  while (millis() - t0 < 900) {
    if (lunaReadFrame(mm, amp, valid, tUs)) {
      if (valid) { sum += mm; n++; }
    }
  }
  (void)dummy16;

  if (n >= 20) {
    baselineMm = sum / n;
    uint32_t th = (baselineMm * 7UL) / 10UL;     // 70 percent of background
    if (th < 300) th = 300;
    currentThreshold = static_cast<uint16_t>(th);
    Serial.printf("background %u mm over %lu frames -> threshold %u mm\n",
                  baselineMm, (unsigned long)n, currentThreshold);
  } else {
    baselineMm = 0;
    currentThreshold = FALLBACK_THRESHOLD_MM;
    Serial.printf("open space, no stable return -> threshold %u mm\n",
                  currentThreshold);
  }
  Serial.println("------------------\n");
#endif
}

// ========================================================
// Boot live stream
// ========================================================

float measuredHz = 0.0f;

void liveStream(uint32_t windowMs) {
  Serial.println("---- LIVE STREAM ----");
  Serial.println("Wave a hand in front of the sensor.\n");

  uint32_t t0 = millis();
  uint32_t frames = 0;
  uint32_t printAt = 0;

  uint16_t mm = 0, amp = 0;
  bool valid = false;
  uint32_t tUs;

  while (millis() - t0 < windowMs) {
    if (lunaReadFrame(mm, amp, valid, tUs)) frames++;

    uint32_t now = millis();
    if (now - printAt >= 250) {
      printAt = now;
      if (valid) Serial.printf("   %5u mm   amp %u\n", mm, amp);
      else       Serial.printf("   ----       amp %u  (weak or out of range)\n", amp);
    }
  }

  uint32_t elapsed = millis() - t0;
  measuredHz = (frames * 1000.0f) / elapsed;

  Serial.printf("\n  %.1f frames/s, %lu checksum errors\n",
                measuredHz, (unsigned long)badChecksum);

  if (frames == 0) {
    Serial.println("\n  NO FRAMES AT ALL. Check in this order:");
    Serial.println("   1. green wire on GPIO5, white on GPIO6 - try swapping");
    Serial.println("   2. red wire on 5V, USB plugged in");
    Serial.println("   3. black wire on GND");
  }
  Serial.println("---------------------\n");
}

// ========================================================
// BLE notify
// ========================================================

void notifyDistance(uint16_t mm) {
  if (!deviceConnected || !distanceCharacteristic) return;
  uint8_t d[2] = { static_cast<uint8_t>(mm & 0xFF),
                   static_cast<uint8_t>((mm >> 8) & 0xFF) };
  distanceCharacteristic->setValue(d, 2);
  distanceCharacteristic->notify();
}

/*
  EVENT payload, 8 bytes little-endian:
    [0..1] mm uint16 | [2] state 1=START 2=STOP 3=RESET
    [3] seq uint8    | [4..7] micros() uint32

  Web side, unsigned so the 71 minute wrap is handled:
    const dt = ((tStop - tStart) >>> 0) / 1e6;
*/
void notifyEvent(uint16_t mm, uint8_t state, uint8_t seq, uint32_t tUs) {
  if (!deviceConnected || !eventCharacteristic) return;

  uint8_t d[8];
  d[0] = mm & 0xFF;
  d[1] = (mm >> 8) & 0xFF;
  d[2] = state;
  d[3] = seq;
  d[4] = tUs & 0xFF;
  d[5] = (tUs >> 8) & 0xFF;
  d[6] = (tUs >> 16) & 0xFF;
  d[7] = (tUs >> 24) & 0xFF;

  eventCharacteristic->setValue(d, 8);
  eventCharacteristic->notify();
}

// ========================================================
// Passage event - keep this path short
// ========================================================

void handlePassage(uint16_t mm, uint32_t tUs) {
  uint8_t state;
  if      (passageState == 0) { passageState = 1; state = 1; }
  else if (passageState == 1) { passageState = 2; state = 2; }
  else                        { passageState = 0; state = 3; }

  passageCount++;
  const uint8_t seq = eventSeq++;

  // Two transports carry the same event. BLE goes first because it is the one
  // that can be delayed; the serial line below is for a browser connected by
  // USB cable through the Web Serial API. A page reads whichever it opened,
  // and both carry the device timestamp, so the measured time is identical.
  notifyEvent(mm, state, seq, tUs);
  Serial.printf("$E,%u,%u,%u,%lu\n", mm, state, seq, (unsigned long)tUs);
  notifyDistance(mm);

  if      (state == 1) requestLed(0, 255, 0);
  else if (state == 2) requestLed(255, 0, 0);
  else                 requestLed(0, 0, 255);

  Serial.printf("[%s] %u mm\n",
                state == 1 ? "START" : (state == 2 ? "STOP " : "RESET"), mm);
}

// ========================================================
// Commands arriving over USB
//
// The browser sends the same two things it sends over BLE:
//   T<mm>   set the trigger distance, e.g. T1700
//   R       reset to standby
// Anything else is ignored, so an accidental keystroke in a serial monitor
// cannot put the gate into a strange state.
// ========================================================

void serviceSerialCommands() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;

      if (buf[0] == 'T' || buf[0] == 't') {
        long v = atol(buf + 1);
        if (v >= 100 && v <= 8000) {
          currentThreshold = (uint16_t)v;
          Serial.printf("$T,%u\n", currentThreshold);
        }
      } else if (buf[0] == 'R' || buf[0] == 'r') {
        passageState  = 0;
        gateArmed     = true;
        lastTriggerMs = 0;
        inCount = outCount = 0;
        requestLed(0, 0, 255);
        Serial.println("$R");
      } else if (buf[0] == '?') {
        // handshake: lets the page confirm it opened the right port
        Serial.printf("$I,%s,%u,%u\n", BLE_DEVICE_NAME, currentThreshold, TOF_MAX_MM);
      }
      continue;
    }

    if (idx < sizeof(buf) - 1) buf[idx++] = c;
    else idx = 0;
  }
}

// ========================================================
// BLE setup
// ========================================================

void setupBLE() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(64);

  // Maximum transmit power. On USB supply there is no reason not to.
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);

  NimBLEService* service = server->createService(SERVICE_UUID);

  distanceCharacteristic = service->createCharacteristic(
      DISTANCE_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  eventCharacteristic = service->createCharacteristic(
      EVENT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  thresholdCharacteristic = service->createCharacteristic(
      THRESHOLD_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  thresholdCharacteristic->setCallbacks(&thresholdCallbacks);

  uint8_t th[2] = { static_cast<uint8_t>(currentThreshold & 0xFF),
                    static_cast<uint8_t>((currentThreshold >> 8) & 0xFF) };
  thresholdCharacteristic->setValue(th, 2);

  resetCharacteristic = service->createCharacteristic(
      RESET_UUID, NIMBLE_PROPERTY::WRITE);
  resetCharacteristic->setCallbacks(&resetCallbacks);

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_DEVICE_NAME);
  adv->addServiceUUID(SERVICE_UUID);
  adv->enableScanResponse(true);

  // Advertising interval, in units of 0.625 ms. The NimBLE default is a few
  // hundred milliseconds, which is polite for a battery device but makes the
  // browser's scan dialog feel slow. 32 units is 20 ms, the floor the BLE
  // spec allows for connectable advertising, so the gate shows up almost
  // immediately. The extra power draw is irrelevant on USB.
  adv->setMinInterval(32);   // 20 ms
  adv->setMaxInterval(48);   // 30 ms

  adv->start();
}

// ========================================================
// setup
// ========================================================

void setup() {
  Serial.begin(115200);

  // Native USB needs a moment for the host to open the port. Nobody is
  // usually watching, so this waits 1200 ms rather than 2500: long enough
  // for a monitor that is already open, short enough not to delay the gate
  // for a session that never opens one.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1200) delay(10);
  delay(150);

  // setTxTimeoutMs exists only on the USB CDC object. Its absence at compile
  // time means Serial is bound to HardwareSerial, which on this board routes
  // to the UART0 pins rather than the USB connector - nothing typed here will
  // ever reach the serial monitor. The guard keeps the sketch compiling
  // either way, and the warning below says so out loud.
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   // never block waiting for a reader
#else
#warning "Tools > USB CDC On Boot is Disabled. Bluetooth will work, the USB cable will NOT, and the serial monitor stays blank. Set it to Enabled and upload again."
#endif

  Serial.println("\n\n======================================");
  Serial.println("  TF LUNA GATE TIMER");
  Serial.println("======================================");
  Serial.println("  TF Luna : red 5V | black GND | green GPIO5 | white GPIO6");
  Serial.println("  NeoPixel: VCC 5V | GND (shared) | DIN GPIO4 | 16 px");

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);

  // Colour sweep at boot: red, green, blue, off. A dead strip, a wrong
  // LED_COUNT or a data line that never got connected all show up here,
  // before anything else can mask them.
  ledNow(255, 0, 0); delay(200);
  ledNow(0, 255, 0); delay(200);
  ledNow(0, 0, 255); delay(200);
  ledNow(0, 0, 0);   delay(120);

#if !ARDUINO_USB_CDC_ON_BOOT
  // This block cannot reach the serial monitor - that is the whole point of
  // the warning - so the LED says it instead: a slow amber blink that no
  // other state uses. Nothing else in this sketch blinks. It has to sit
  // after strip.begin(), or ledNow writes to a pin that was never set up
  // and the warning silently does nothing.
  Serial.println("  *** USB CDC On Boot is DISABLED ***");
  for (int i = 0; i < 6; i++) {
    ledNow(255, 150, 0); delay(220);
    ledNow(0, 0, 0);     delay(220);
  }
#endif

  // Purple means warming up: the sensor is streaming but the gate is NOT
  // armed yet and a passage now would be ignored. It turns blue the instant
  // the gate goes live, so there is no guessing when to start running.
  ledNow(70, 0, 90);

  luna.setRxBufferSize(1024);
  luna.begin(LUNA_BAUD, SERIAL_8N1, LUNA_RX_PIN, LUNA_TX_PIN);
  delay(300);

  lunaEnableOutput();
  lunaSetOutputFormat9ByteCm();
  lunaSetFrameRate(TARGET_FRAME_RATE);
  delay(200);
  while (luna.available()) luna.read();
  fidx = 0;

#if BOOT_LIVE_STREAM_MS > 0
  liveStream(BOOT_LIVE_STREAM_MS);
#endif

  calibrateBaseline();
  setupBLE();

  ledNow(0, 0, 255);   // armed: blue standby, purple is over

  Serial.println("======================================");
  Serial.printf ("BLE name   : %s\n", BLE_DEVICE_NAME);
  Serial.printf ("Frame rate : %.1f Hz -> jitter about +/- %.1f ms\n",
                 measuredHz, measuredHz > 0 ? 500.0f / measuredHz : 0.0f);
  Serial.printf ("Threshold  : %u mm\n", currentThreshold);
  Serial.printf ("Confirm    : in %u / out %u frames\n", CONFIRM_IN, CONFIRM_OUT);
  Serial.println("1st=START(G) 2nd=STOP(R) 3rd=RESET(B)");
  Serial.println("======================================\n");

  // Opening the USB port from a browser reboots this chip, so the web page
  // has no way to know whether it is talking to an armed gate or one still
  // in the middle of setup. This single line is that answer: the gate is
  // live from here on. The page waits for it, shows the wait, and re-sends
  // its threshold once it arrives - without it, settings sent during the
  // reboot are swallowed and silently lost.
  Serial.println("$B");
}

// ========================================================
// loop
// ========================================================

uint32_t heartbeatAt = 0;

void loop() {
  uint16_t mm, amp;
  bool     valid;
  uint32_t tUs;

  // Drain every buffered frame so a backlog can never age into a stale
  // detection. This is the whole detection path.
  while (lunaReadFrame(mm, amp, valid, tUs)) {
    frameCount++;
    lastMm    = mm;
    lastAmp   = amp;
    lastValid = valid;

    uint32_t nowMs = millis();

    // An unreliable reading means no trustworthy target in the beam, which for
    // a gate is the same thing as nothing being there: it counts as OUTSIDE.
    //
    // Earlier this branch did nothing at all unless baselining had failed. That
    // stalled the gate: once a passage had disarmed it, re-arming needed
    // CONFIRM_OUT valid far readings, so if the background itself read as
    // unreliable - beyond TOF_MAX_MM, dark, or angled away - outCount never
    // advanced and no second passage was ever detected. Counting it as outside
    // is still safe, because re-arming requires CONFIRM_OUT consecutive frames
    // and firing requires the gate to be armed.
    if (!valid)                     { outCount++; inCount  = 0; }
    else if (mm < currentThreshold) { inCount++;  outCount = 0; }
    else                            { outCount++; inCount  = 0; }

    if (valid && mm < windowMinMm) windowMinMm = mm;
    if (valid && mm < currentThreshold) windowNearFrames++;

    if (inCount >= CONFIRM_IN && gateArmed &&
        nowMs - lastTriggerMs >= LOCKOUT_MS) {
      gateArmed     = false;
      lastTriggerMs = nowMs;
      handlePassage(mm, tUs);
    }

    if (outCount >= CONFIRM_OUT && nowMs - lastTriggerMs >= LOCKOUT_MS) {
      gateArmed = true;
    }
  }

  serviceLed();
  serviceSerialCommands();

  // Heartbeat prints unconditionally, whether or not a frame arrived.
  // Silence here would be indistinguishable from a dead board.
  uint32_t now = millis();
  if (now - heartbeatAt >= HEARTBEAT_MS) {
    heartbeatAt = now;

    // closest: nearest valid reading this window. near: how many frames were
    // already inside the threshold. If near is above zero but no passage fired,
    // the problem is the gate logic or the lockout, not the sensor. If closest
    // stays far while someone walks through, the beam is missing them.
    Serial.printf("[%s %5u mm amp %5u] th=%u armed=%s state=%u fps=%lu "
                  "closest=%u near=%lu trig=%lu cksum_err=%lu\n",
                  lastValid ? "OK " : "BAD",
                  lastMm, lastAmp, currentThreshold,
                  gateArmed ? "Y" : "N", passageState,
                  (unsigned long)(frameCount * 1000 / HEARTBEAT_MS),
                  windowMinMm == 0xFFFF ? 0 : windowMinMm,
                  (unsigned long)windowNearFrames,
                  (unsigned long)passageCount,
                  (unsigned long)badChecksum);

    frameCount       = 0;
    windowMinMm      = 0xFFFF;
    windowNearFrames = 0;

    if (lastAmp == 0 && lastMm == 0) {
      Serial.println("  no frames arriving - check green/white wiring");
    }
  }
}
