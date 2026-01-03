#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <Preferences.h>
#include <FastLED.h> 
#include "SwarmProtocol.h" 

// Compile-time sanity check for ESP-NOW payload size
static_assert(sizeof(SwarmMessage) <= 250, "SwarmMessage exceeds ESP-NOW payload size");

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

// Traffic -> Energy Logic
#define TRAFFIC_BASELINE      10   // Packets per tick to maintain energy
#define MIN_ENERGY            50.0 // Never go below this (Idling)
#define MAX_ENERGY            255.0

// ======================
// GLOBAL STATE
// ======================
uint32_t bootToken;
uint16_t swarmId = 0; 
Preferences prefs;

// Swarm Data
float swarmAvgVal = 0.0;     
float swarmEnergy = 100.0; // Start at medium energy
float currentBrightness = 100.0; 

// Traffic Data
volatile int rawPacketCount = 0;        // Local packets (ISR)
volatile int activityPackets = 0;       // Snapshot for logic
volatile int remoteSwarmTraffic = 0;    // Sum of traffic reported by probes

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

// Timing
unsigned long lastFastTick = 0;
unsigned long lastSlowTick = 0;

// Identity
enum IdState { ID_UNASSIGNED, ID_CLAIMING, ID_ASSIGNED };
IdState nodeState = ID_UNASSIGNED;
uint16_t candidateId = 0;
unsigned long stateTimer = 0;
const int CLAIM_WINDOW_MS = 1500;

// 🛰️ SCANNER VARS
volatile int currentChannel = 1; 
unsigned long lastChannelEval = 0;
const bool ENABLE_SWEEP_REPORT = true; 
const unsigned long SWEEP_SEND_DELAY_MS = 15;

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
               case VIS_BIO:    runBioEffect(elapsed, modeDuration); break;
               case VIS_SAT:    runSatEffect(elapsed, modeDuration); break;
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

    // Combine Local + Remote Traffic for EXTRA Sparkle density if busy
    int totalTraffic = activityPackets + (remoteSwarmTraffic / 5);
    if (totalTraffic > 20) activity += 5;
    if (totalTraffic > 50) activity += 10;
    
    if (random8() < activity) {
       int pos = random(NUM_LEDS);
       leds[pos] += CHSV(baseHue, 255, 255);
    }
    
    // --- DIRECTIONAL PULSES ---
    // Traffic Pulse (Clockwise - Teal)
    if (trfPulseLoc >= 0) {
        leds[trfPulseLoc] += CHSV(140, 255, 255); 
        if(trfPulseLoc > 0) leds[trfPulseLoc-1] += CHSV(140, 255, 100);
        trfPulseLoc++;
        if (trfPulseLoc >= NUM_LEDS) trfPulseLoc = -1;
    }

    // Sync Pulse (Counter-Clockwise - White)
    if (synPulseLoc >= 0) {
        leds[synPulseLoc] += CHSV(0, 0, 255);
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
        if (random8() > 220) leds[pos] = CRGB::White; // Glitch flash
        else leds[pos] = CRGB::Cyan;  // Normal signal
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
// 🛰️ SCANNER & BURST HELPERS
// ======================
void scanAndHop() {
  unsigned long now = millis();
  if (now - lastChannelEval > 150) { 
    lastChannelEval = now;
    currentChannel++;
    if (currentChannel > 13) currentChannel = 1;
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  }
}

void sendMultiChannelBurst(uint8_t* data, size_t len) {
    if (!ENABLE_SWEEP_REPORT) {
        esp_now_send(BROADCAST_ADDR, data, len);
        return;
    }
    uint8_t originalCh = currentChannel;
    uint8_t baddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, baddr, 6);
    peer.encrypt = false;

    for (int ch = 1; ch <= 13; ch++) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        peer.channel = ch;
        esp_now_add_peer(&peer); 
        esp_now_send(baddr, data, len);
        delay(SWEEP_SEND_DELAY_MS);
    }
    esp_wifi_set_channel(originalCh, WIFI_SECOND_CHAN_NONE);
}

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

  incomingMsg = in;
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
    if (trfPulseLoc == -1) trfPulseLoc = 0; 
    
    // 🧠 HIVE MIND: Eat the remote traffic data
    remoteSwarmTraffic += in.data.state.traffic;

    float incomingVal = in.data.state.humidity;
    if (swarmAvgVal == 0) swarmAvgVal = incomingVal;
    else swarmAvgVal = (swarmAvgVal * 0.95) + (incomingVal * 0.05);
    
    // We still blend probe energy slightly so if THEY go dark, we feel it
    float incEnergy = in.data.state.energy;
    swarmEnergy = (swarmEnergy * 0.95) + (incEnergy * 0.05);
  }
  if (in.header.msgType == MSG_SYNC) {
      if (synPulseLoc == -1) synPulseLoc = NUM_LEDS - 1; 
  }
}

// ======================
// TICKS
// ======================
void fastTick() {
  unsigned long now = millis();
  
  // Decay remote traffic memory (so sparkles fade if probes stop reporting)
  if (remoteSwarmTraffic > 0) remoteSwarmTraffic -= 5;
  if (remoteSwarmTraffic < 0) remoteSwarmTraffic = 0;

  // Atomically capture local packets
  noInterrupts();
  int pc = rawPacketCount;
  rawPacketCount = 0;
  interrupts();
  activityPackets = pc; 

  // 🧠 TRAFFIC -> ENERGY LOGIC
  // Total = Local Packets + Scaled Remote Packets
  int totalActivity = activityPackets + (remoteSwarmTraffic / 4);

  // Adjust Energy based on Baseline
  if (totalActivity > TRAFFIC_BASELINE) {
      swarmEnergy += 0.5; // Slow rise when busy
  } else {
      swarmEnergy -= 0.1; // Very slow decay when quiet
  }
  // Clamp Energy (Never fully sleep, never explode)
  if (swarmEnergy < MIN_ENERGY) swarmEnergy = MIN_ENERGY;
  if (swarmEnergy > MAX_ENERGY) swarmEnergy = MAX_ENERGY;

  // Smooth visual brightness
  currentBrightness = (currentBrightness * 0.9) + (swarmEnergy * 0.1);

  renderer.render(now);
}

void slowTick() {
  scanAndHop(); // Keep looking for data

  if (nodeState == ID_ASSIGNED) {
      long r = random(0, 10000);
      long crashThresh = BASE_CHANCE_CRASH;
      long satThresh = BASE_CHANCE_SATELLITE;
      
      if (currentBrightness > 200) satThresh *= 2; 
      else if (currentBrightness < 50) { satThresh /= 10; crashThresh /= 10; }

      // Autonomous Events (Crash/Sat)
      if (r < crashThresh) {
          SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_EVENT; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
          msg.data.event.eventId = 1; msg.data.event.intensity = 200; msg.data.event.durationMs = 3000;
          sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
      } else if (r < (crashThresh + satThresh)) {
          SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_EVENT; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
          msg.data.event.eventId = 2; msg.data.event.intensity = 150; msg.data.event.durationMs = 6000;
          sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
      }

      // Announce Presence + Current Energy (So probes know the hive mood)
      SwarmMessage msg; msg.header.version = SWARM_PROTO_VERSION; msg.header.msgType = MSG_STATE; msg.header.senderId = swarmId; msg.header.bootToken = bootToken;
      msg.data.state.energy = (uint8_t)currentBrightness; // Share the computed energy!
      msg.data.state.traffic = activityPackets; // Share our local view too
      sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
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

  swarmId = prefs.getUShort("swarmId", 0);
  if (swarmId != 0) nodeState = ID_ASSIGNED;

  randomSeed((unsigned long)esp_random());

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

  currentChannel = 1;
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);

  runBootSequence();
}

void loop() {
  unsigned long now = millis();
  if (nodeState == ID_UNASSIGNED || nodeState == ID_CLAIMING) manageIdentity();
  
  if (now - lastFastTick >= FAST_TICK_MS) { lastFastTick = now; fastTick(); }
  if (now - lastSlowTick >= SLOW_TICK_MS) { lastSlowTick = now; slowTick(); }
}
