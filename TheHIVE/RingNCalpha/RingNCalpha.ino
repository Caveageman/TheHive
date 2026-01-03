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
#define FAST_TICK_MS    20   // 50 FPS for smooth LEDs
#define SLOW_TICK_MS    4000 // 4s for Swarm Decisions

// Personality (Stable Observer)
#define BASE_CHANCE_CRASH     1      
#define BASE_CHANCE_SATELLITE 20     

// ======================
// GLOBAL STATE
// ======================
uint32_t bootToken;
uint16_t swarmId = 0; 
Preferences prefs;

// Swarm Data
float swarmAvgVal = 0.0;     
float swarmEnergy = 255.0; 
float currentBrightness = 255.0; 

// LED Objects
CRGB leds[NUM_LEDS];

// Pulse State (From ringgate2)
int trfPulseLoc = -1; // Clockwise Pulse
int synPulseLoc = -1; // Counter-Clockwise Pulse

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
volatile int rawPacketCount = 0; 

// Timing
unsigned long lastFastTick = 0;
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
    fadeToBlackBy(leds, NUM_LEDS, 20); 

    // Base Sparkles
    uint8_t baseHue = map((int)swarmAvgVal, 0, 100, 160, 0);
    int activity = map((int)swarmEnergy, 0, 255, 2, 20);
    activity += (rawPacketCount * 2); 
    
    if (random8() < activity) {
       int pos = random(NUM_LEDS);
       leds[pos] += CHSV(baseHue, 255, 255);
    }
    
    // --- DIRECTIONAL PULSES (Steal from ringgate2) ---
    
    // Traffic Pulse (Clockwise - Teal)
    if (trfPulseLoc >= 0) {
        leds[trfPulseLoc] += CHSV(140, 255, 255); 
        // Trail
        if(trfPulseLoc > 0) leds[trfPulseLoc-1] += CHSV(140, 255, 100);
        
        trfPulseLoc++;
        if (trfPulseLoc >= NUM_LEDS) trfPulseLoc = -1;
    }

    // Sync Pulse (Counter-Clockwise - White)
    if (synPulseLoc >= 0) {
        leds[synPulseLoc] += CHSV(0, 0, 255);
        // Trail
        if(synPulseLoc < NUM_LEDS-1) leds[synPulseLoc+1] += CHSV(0, 0, 100);
        
        synPulseLoc--;
        if (synPulseLoc < 0) synPulseLoc = -1;
    }

    // Core LED: Alien Breathing
    float breath = (exp(sin(now/2000.0*PI)) - 0.36787944)*108.0;
    int coreBri = map((int)breath, 0, 255, 0, (int)currentBrightness);
    analogWrite(LED_PIN_CORE, coreBri);

    FastLED.show();
  }

private:
  void runBioEffect(unsigned long elapsed, unsigned long duration) {
    float p = (float)elapsed / (float)duration; 
    int h = p * (NUM_LEDS/2); 
    analogWrite(LED_PIN_CORE, (sin(elapsed * 0.01) + 1) * 127);
    for (int i=0; i<=h; i++) { 
        CRGB c = CHSV(96+(i*2), 255, 180); 
        if(i < NUM_LEDS) leds[i] = c; 
        if(NUM_LEDS-1-i >= 0) leds[NUM_LEDS-1-i] = c; 
    }
  }

  void runSatEffect(unsigned long elapsed, unsigned long duration) {
    fadeToBlackBy(leds, NUM_LEDS, 60); 
    float p = (float)elapsed / (float)duration;
    int pos = (int)(SAT_START + (p * (SAT_END - SAT_START)));
    
    if (pos >= 0 && pos < NUM_LEDS) {
        // Glitch Logic (Steal from ringgate2)
        if (random8() > 220) {
             leds[pos] = CRGB::White; // Glitch flash
        } else {
             leds[pos] = CRGB::Cyan;  // Normal signal
        }
        // Trail
        if(pos > 0) leds[pos-1] += CHSV(128, 255, 60); 
    }
  }

  void runCrashEffect(unsigned long elapsed, unsigned long duration) {
    float progress = (float)elapsed / (float)duration;
    if (progress < 0.40) {
      fadeToBlackBy(leds, NUM_LEDS, 100);
      leds[random(NUM_LEDS)] = (random(2)) ? CRGB::Red : CRGB::White;
      analogWrite(LED_PIN_CORE, (random(2) ? 255 : 0));
    } else if (progress < 0.50) {
      fill_solid(leds, NUM_LEDS, CRGB::Black); 
      analogWrite(LED_PIN_CORE, 0);
    } else {
      float spin = (progress - 0.50) * 2.0;
      float revs = 20.0 * (spin * spin * spin); 
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

  // IDENTITY
  if (in.header.msgType == MSG_ID_CLAIM) {
    if (nodeState == ID_ASSIGNED && in.data.id.targetId == swarmId) {
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

  // EVENTS
  if (in.header.msgType == MSG_EVENT) {
    switch(in.data.event.eventId) {
      case 1: // CRASH
        currentMode = VIS_CRASH; 
        modeStartTime = millis(); 
        modeDuration = in.data.event.durationMs + 2000; 
        break;
      case 2: // SATELLITE
        currentMode = VIS_SAT; 
        modeStartTime = millis(); 
        modeDuration = in.data.event.durationMs;
        break;
      case 5: // BIO
        currentMode = VIS_BIO;
        modeStartTime = millis();
        modeDuration = 2000;
        break;
      case 4: // DARKNESS
        swarmEnergy = 20.0;
        break;
    }
  }

  // STATE & SYNC
  if (in.header.msgType == MSG_STATE) {
    // Trigger Clockwise Pulse
    if (trfPulseLoc == -1) trfPulseLoc = 0; 
    
    float incomingVal = in.data.state.humidity;
    if (swarmAvgVal == 0) swarmAvgVal = incomingVal;
    else swarmAvgVal = (swarmAvgVal * 0.95) + (incomingVal * 0.05);
    
    float incEnergy = in.data.state.energy;
    swarmEnergy = (swarmEnergy * 0.9) + (incEnergy * 0.1);
  }
  if (in.header.msgType == MSG_SYNC) {
      // Trigger Counter-Clockwise Pulse
      if (synPulseLoc == -1) synPulseLoc = NUM_LEDS - 1; 
  }
}

// ======================
// BOOT SEQUENCE
// ======================
void runBootSequence() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for(int i=0; i<NUM_LEDS/4; i++) leds[i] = CRGB::Red;
  FastLED.show(); delay(200); 
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  for(int i=0; i<NUM_LEDS/2; i++) leds[i] = CRGB::Yellow;
  FastLED.show(); delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  for(int i=0; i<(NUM_LEDS*3)/4; i++) leds[i] = CRGB::Blue;
  FastLED.show(); delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show(); delay(50);

  fill_solid(leds, NUM_LEDS, CRGB::White);
  analogWrite(LED_PIN_CORE, 255);
  FastLED.show(); delay(500); 

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
        fill_solid(leds, NUM_LEDS, CRGB::White); FastLED.show(); delay(50);
        fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();
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
  currentBrightness = (currentBrightness * 0.95) + (swarmEnergy * 0.05);
  if (rawPacketCount > 0) rawPacketCount--;
  renderer.render(now);
}

void slowTick() {
  if (nodeState == ID_ASSIGNED) {
      long r = random(0, 10000);
      long crashThresh = BASE_CHANCE_CRASH;
      long satThresh = BASE_CHANCE_SATELLITE;
      
      if (currentBrightness > 200) satThresh *= 2; 
      else if (currentBrightness < 50) { satThresh /= 10; crashThresh /= 10; }

      if (r < crashThresh) {
          SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_EVENT; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
          msg.data.event.eventId = 1; msg.data.event.intensity = 200; msg.data.event.durationMs = 3000;
          esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
      } else if (r < (crashThresh + satThresh)) {
          SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_EVENT; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
          msg.data.event.eventId = 2; msg.data.event.intensity = 150; msg.data.event.durationMs = 6000;
          esp_now_send(BROADCAST_ADDR, (uint8_t *) &msg, sizeof(msg));
      }

      SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_ANNOUNCE; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
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

  renderer.begin();
  bootToken = esp_random();
  prefs.begin("swarm", false);

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

  runBootSequence();
}

void loop() {
  unsigned long now = millis();
  manageIdentity(); 
  
  if (now - lastFastTick >= FAST_TICK_MS) { lastFastTick = now; fastTick(); }
  if (now - lastSlowTick >= SLOW_TICK_MS) { lastSlowTick = now; slowTick(); }
}