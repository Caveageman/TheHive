#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <Preferences.h>
#include <FastLED.h> 
#include "SwarmProtocol.h" 

// ======================
// 🛠️ HARDWARE PINS (QUEEN NODE)
// ======================
// No OLED, No SHT31. Just Lights.
#define LED_PIN_RING    5   // WS2812B Data
#define LED_PIN_CORE    21  // PWM White LED (The "Heart")
#define NUM_LEDS        37  
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define BRIGHTNESS      140 // Master Brightness limit

// ======================
// ⚙️ CONFIGURATION
// ======================
#define CPU_MHZ 80 

// Timing
#define FAST_TICK_MS    20   // Faster tick for smooth LEDs (50fps)
#define MED_TICK_MS     100  
#define SLOW_TICK_MS    4000 

// Personality: The Queen is Stable. She observes.
#define BASE_CHANCE_CRASH     1      // Very low chance to crash herself
#define BASE_CHANCE_SATELLITE 20     // Good at spotting satellites

// ======================
// GLOBAL STATE
// ======================
uint32_t bootToken;
uint16_t swarmId = 0; 
Preferences prefs;

// Flags
bool isIsolated = false;
unsigned long lastHeardMs = 0;
volatile int rawPacketCount = 0; 

// Swarm Data
float swarmAvgVal = 0.0;     
float swarmEnergy = 255.0; 
float currentBrightness = 255.0; 

// LED Objects
CRGB leds[NUM_LEDS];

// Animation State
enum VisualMode { VIS_TRAFFIC, VIS_BIO, VIS_SAT, VIS_CRASH };
VisualMode currentMode = VIS_TRAFFIC;
unsigned long modeStartTime = 0;
unsigned long modeDuration = 0;

// Vars for Satellites
const int SAT_START = 9; 
const int SAT_END = 29; 

// Comms
SwarmMessage incomingMsg;
volatile bool msgReceived = false;

// Timing
unsigned long lastFastTick = 0;
unsigned long lastMedTick  = 0;
unsigned long lastSlowTick = 0;

// Identity
enum IdState { ID_UNASSIGNED, ID_CLAIMING, ID_ASSIGNED };
IdState nodeState = ID_UNASSIGNED;
uint16_t candidateId = 0;
unsigned long stateTimer = 0;
const int CLAIM_WINDOW_MS = 1500;

// ======================
// RING RENDERER CLASS
// ======================
class RingRenderer {
public:
  void begin() {
    FastLED.addLeds<LED_TYPE, LED_PIN_RING, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    pinMode(LED_PIN_CORE, OUTPUT);
  }

  void render(unsigned long now) {
    // 1. Handle Overrides (Crash/Bio/Sat)
    if (currentMode != VIS_TRAFFIC) {
       unsigned long elapsed = now - modeStartTime;
       
       if (elapsed > modeDuration) {
           currentMode = VIS_TRAFFIC; // Reset to default
       } else {
           switch(currentMode) {
               case VIS_BIO:   runBioEffect(elapsed, modeDuration); break;
               case VIS_SAT:   runSatEffect(elapsed, modeDuration); break;
               case VIS_CRASH: runCrashEffect(elapsed, modeDuration); break;
               default: break;
           }
           FastLED.show();
           return; // Skip traffic logic while in an event
       }
    }

    // 2. Default: Traffic / Swarm Energy
    // Fade trail
    fadeToBlackBy(leds, NUM_LEDS, 20); 

    // Base color based on Swarm Average (Humidity/Mood)
    // Map 0-100 to Hue (Blue->Green->Red)
    uint8_t baseHue = map((int)swarmAvgVal, 0, 100, 160, 0);
    
    // Sparkle Probability based on Energy + Packet Traffic
    int activity = map((int)swarmEnergy, 0, 255, 2, 20);
    activity += (rawPacketCount * 2); 
    
    if (random8() < activity) {
       int pos = random(NUM_LEDS);
       leds[pos] += CHSV(baseHue, 255, 255);
    }
    
    // Core LED: Alien Breathing (Same math as Nodes)
    float breath = (exp(sin(now/2000.0*PI)) - 0.36787944)*108.0;
    // Scale by Swarm Energy (dim if stealth mode)
    int coreBri = map((int)breath, 0, 255, 0, (int)currentBrightness);
    analogWrite(LED_PIN_CORE, coreBri);

    FastLED.show();
  }

private:
  void runBioEffect(unsigned long elapsed, unsigned long duration) {
    // Fill up the ring like a gauge
    float p = (float)elapsed / (float)duration; 
    int h = p * (NUM_LEDS/2); 
    
    // Flash Core
    analogWrite(LED_PIN_CORE, (sin(elapsed * 0.01) + 1) * 127);
    
    for (int i=0; i<=h; i++) { 
        CRGB c = CHSV(96+(i*2), 255, 180); // Greenish fill
        if(i < NUM_LEDS) leds[i] = c; 
        if(NUM_LEDS-1-i >= 0) leds[NUM_LEDS-1-i] = c; 
    }
  }

  void runSatEffect(unsigned long elapsed, unsigned long duration) {
    fadeToBlackBy(leds, NUM_LEDS, 60); 
    float p = (float)elapsed / (float)duration;
    // Move pixel from Start to End
    int pos = (int)(SAT_START + (p * (SAT_END - SAT_START)));
    if (pos >= 0 && pos < NUM_LEDS) leds[pos] = CRGB::Cyan;
  }

  void runCrashEffect(unsigned long elapsed, unsigned long duration) {
    float progress = (float)elapsed / (float)duration;
    
    if (progress < 0.40) {
      // Phase 1: Chaos
      fadeToBlackBy(leds, NUM_LEDS, 100);
      leds[random(NUM_LEDS)] = (random(2)) ? CRGB::Red : CRGB::White;
      analogWrite(LED_PIN_CORE, (random(2) ? 255 : 0));
    } 
    else if (progress < 0.50) {
      // Phase 2: Blackout
      fill_solid(leds, NUM_LEDS, CRGB::Black); 
      analogWrite(LED_PIN_CORE, 0);
    } 
    else {
      // Phase 3: Spin Reboot
      float spin = (progress - 0.50) * 2.0;
      float revs = 20.0 * (spin * spin * spin); // Accelerating spin
      long total = (long)(revs * NUM_LEDS);
      int head = total % NUM_LEDS;
      int tailLen = 8;
      
      fadeToBlackBy(leds, NUM_LEDS, 120);
      for (int i = 0; i < tailLen; i++) {
        int px = (head - i + NUM_LEDS) % NUM_LEDS;
        leds[px] += CHSV(160, 255, 255 - (i*30));
      }
      analogWrite(LED_PIN_CORE, (head < 5) ? 255 : 0);
    }
  }
};

RingRenderer renderer;

// ======================
// CALLBACKS
// ======================
void IRAM_ATTR wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  rawPacketCount++;
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(SwarmMessage)) return;
  SwarmMessage in;
  memcpy(&in, incomingData, sizeof(in));
  if (in.header.version != SWARM_PROTO_VERSION) return;

  msgReceived = true; 
  lastHeardMs = millis(); 

  // IDENTITY
  if (in.header.msgType == MSG_ID_CLAIM) {
    if (nodeState == ID_ASSIGNED && in.data.id.targetId == swarmId) {
      // Defense: Queen claims her ID strongly
      SwarmMessage conflict;
      conflict.header.version = SWARM_PROTO_VERSION;
      conflict.header.msgType = MSG_ID_CONFLICT;
      conflict.header.senderId = swarmId;
      conflict.header.bootToken = bootToken;
      conflict.data.id.targetId = swarmId;
      esp_now_send(BROADCAST_ADDR, (uint8_t *) &conflict, sizeof(conflict));
    }
    if (nodeState == ID_CLAIMING && in.data.id.targetId == candidateId && in.header.bootToken < bootToken) {
      nodeState = ID_UNASSIGNED; 
      stateTimer = millis() + random(100, 500); 
    }
  }

  // EVENTS: Trigger Visual Modes
  if (in.header.msgType == MSG_EVENT) {
    // 1=Crash, 2=Sat, 4=Dark, 5=Bio
    switch(in.data.event.eventId) {
      case 1: // CRASH
        currentMode = VIS_CRASH; 
        modeStartTime = millis(); 
        modeDuration = in.data.event.durationMs + 2000; // Extra time for spin
        break;
      case 2: // SATELLITE
        currentMode = VIS_SAT; 
        modeStartTime = millis(); 
        modeDuration = in.data.event.durationMs;
        break;
      case 5: // BIO (Breath)
        currentMode = VIS_BIO;
        modeStartTime = millis();
        modeDuration = 2000;
        break;
      case 4: // DARKNESS
        swarmEnergy = 20.0;
        break;
    }
  }

  // STATE: Aggregate Data
  if (in.header.msgType == MSG_STATE) {
    float incomingVal = in.data.state.humidity;
    // Update Swarm Average (Color)
    if (swarmAvgVal == 0) swarmAvgVal = incomingVal;
    else swarmAvgVal = (swarmAvgVal * 0.95) + (incomingVal * 0.05);
    
    // Update Swarm Energy (Activity)
    float incEnergy = in.data.state.energy;
    swarmEnergy = (swarmEnergy * 0.9) + (incEnergy * 0.1);
  }
}

// ======================
// HELPER FUNCTIONS
// ======================
void runBootSequence() {
  // Step 1: "BOOT" equivalent (Red Quadrant)
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for(int i=0; i<NUM_LEDS/4; i++) leds[i] = CRGB::Red;
  FastLED.show();
  delay(200); 
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  // Step 2: "MEM OK" equivalent (Yellow Half)
  for(int i=0; i<NUM_LEDS/2; i++) leds[i] = CRGB::Yellow;
  FastLED.show();
  delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  // Step 3: "RADIO" equivalent (Blue 3/4)
  for(int i=0; i<(NUM_LEDS*3)/4; i++) leds[i] = CRGB::Blue;
  FastLED.show();
  delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  // Step 4: "HIVE" equivalent (Full White + Core Flash)
  fill_solid(leds, NUM_LEDS, CRGB::White);
  analogWrite(LED_PIN_CORE, 255);
  FastLED.show();
  delay(500); // Longer pause like the OLED nodes

  // Cleanup
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  analogWrite(LED_PIN_CORE, 0);
  FastLED.show();
}

void manageIdentity() {
  unsigned long now = millis();
  switch (nodeState) {
    case ID_UNASSIGNED:
      if (now > stateTimer) { 
        candidateId = random(1, 65);
        SwarmMessage msg;
        msg.header.version = SWARM_PROTO_VERSION;
        msg.header.msgType = MSG_ID_CLAIM;
        msg.header.senderId = 0; 
        msg.header.bootToken = bootToken;
        msg.data.id.targetId = candidateId;
        esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
        nodeState = ID_CLAIMING;
        stateTimer = now;
      }
      break;
    case ID_CLAIMING:
      if (now - stateTimer > CLAIM_WINDOW_MS) {
        swarmId = candidateId;
        nodeState = ID_ASSIGNED;
        prefs.putUShort("swarmId", swarmId);
        // Visual confirmation on Ring
        fill_solid(leds, NUM_LEDS, CRGB::White);
        FastLED.show();
        delay(50);
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
      }
      break;
    case ID_ASSIGNED: break;
  }
}

// ======================
// TICKS
// ======================
void fastTick() {
  unsigned long now = millis();
  
  // Smooth Brightness
  currentBrightness = (currentBrightness * 0.95) + (swarmEnergy * 0.05);
  
  // Decay packet count for visualizer
  if (rawPacketCount > 0) rawPacketCount--;

  // RENDER
  renderer.render(now);
}

void slowTick() {
  // --- AUTONOMOUS TRIGGERS ---
  // The Queen is less glitchy, but she observes deep space.
  if (nodeState == ID_ASSIGNED) {
      long r = random(0, 10000);
      
      long crashThresh = BASE_CHANCE_CRASH;
      long satThresh   = BASE_CHANCE_SATELLITE;

      if (currentBrightness > 200) satThresh *= 2; 
      else if (currentBrightness < 50) { satThresh /= 10; crashThresh /= 10; }

      // 1. CRASH 
      if (r < crashThresh) {
          SwarmMessage msg;
          msg.header.version = SWARM_PROTO_VERSION;
          msg.header.msgType = MSG_EVENT;
          msg.header.senderId = swarmId;
          msg.header.bootToken = bootToken;
          msg.data.event.eventId = 1; 
          msg.data.event.intensity = 200;
          msg.data.event.durationMs = 3000;
          esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
      }
      // 2. SATELLITE 
      else if (r < (crashThresh + satThresh)) {
          SwarmMessage msg;
          msg.header.version = SWARM_PROTO_VERSION;
          msg.header.msgType = MSG_EVENT;
          msg.header.senderId = swarmId;
          msg.header.bootToken = bootToken;
          msg.data.event.eventId = 2; 
          msg.data.event.intensity = 150;
          msg.data.event.durationMs = 6000;
          esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
      }
  }

  // Announce Presence
  if (nodeState == ID_ASSIGNED) {
    SwarmMessage msg;
    msg.header.version = SWARM_PROTO_VERSION;
    msg.header.msgType = MSG_ANNOUNCE;
    msg.header.senderId = swarmId;
    msg.header.bootToken = bootToken;
    // Capabilites: 4 = High Power LED / Queen
    msg.data.announce.capabilities = 4; 
    esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
  }
}

// ======================
// SETUP & LOOP
// ======================
void setup() {
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);

  // Init LEDs
  renderer.begin();

  // Data
  bootToken = esp_random();
  prefs.begin("swarm", false);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Error"); } 
  else {
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDR, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb);
  }

  // Trigger Visual Boot Sequence to match standard nodes
  runBootSequence();
}

void loop() {
  unsigned long now = millis();
  manageIdentity(); 
  
  if (now - lastFastTick >= FAST_TICK_MS) { lastFastTick = now; fastTick(); }
  if (now - lastMedTick >= MED_TICK_MS) { lastMedTick = now; mediumTick(); }
  if (now - lastSlowTick >= SLOW_TICK_MS) { lastSlowTick = now; slowTick(); }
}