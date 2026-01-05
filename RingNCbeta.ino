#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAdvertising.h>

// ======================
// 🛠️ HARDWARE CONFIG
// ======================
#define HIVE_MANUFACTURER_ID 0x00FF 

#define LED_PIN_RING 5
#define LED_PIN_CORE 21
#define NUM_LEDS 37
#define BRIGHTNESS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
Preferences preferences;

// ======================
// 📜 SWARM PROTOCOL V4 (Same as GNC)
// ======================
#define SWARM_PROTO_VERSION 4

enum MsgType : uint8_t {
    MSG_HEARTBEAT   = 0,
    MSG_ID_CLAIM    = 1,
    MSG_ID_CONFLICT = 2,
    MSG_SYNC        = 3,
    MSG_EVENT       = 4,
    MSG_STATE       = 5,
    MSG_ANNOUNCE    = 6,
};

struct __attribute__((packed)) SwarmHeader {
    uint8_t  version;       
    uint8_t  msgType;       
    uint16_t senderId;      
    uint32_t bootToken;     
    uint16_t sequenceId;    
    uint8_t  groupId;       
    uint8_t  context;       
};

struct __attribute__((packed)) MsgId { uint16_t targetId; };

struct __attribute__((packed)) MsgEvent {
    uint8_t  eventId;       
    uint8_t  intensity;     
    uint16_t durationMs;    
};

struct __attribute__((packed)) MsgState {
    int16_t  temp_x10;      
    uint16_t humidity_x10;  
    uint16_t traffic;       
    uint8_t  energy;        
    uint8_t  mood;          
    uint8_t  cohesion;      
    uint8_t  tempo;         
    uint8_t  drift;         
};

struct __attribute__((packed)) MsgAnnounce {
    uint8_t posX; uint8_t posY;
    uint8_t role; uint8_t influence;      
};

struct __attribute__((packed)) SwarmMessage {
    SwarmHeader header;
    union {
        MsgId       id;
        MsgEvent    event;
        MsgState    state;
        MsgAnnounce announce;
    } data;
};

// ======================
// 🧠 GLOBAL STATE
// ======================
BLEAdvertising *pAdvertising;
BLEScan *pBLEScan;

uint32_t bootToken;
uint16_t swarmId = 0;
uint16_t mySequenceCounter = 0;

int myX = 128;
int myY = 128;

// 🔥 NEW: Ring's Own Autonomic State
float myEnergy = 180.0;
float myMood = 128.0;
float myTempo = 120.0;
float myDrift = 10.0;

// Swarm State Tracking (for influence)
float avgEnergy = 128.0;
float avgMood = 128.0;
float avgTempo = 128.0;
float avgDrift = 10.0;
float avgHum = 50.0;
long globalTraffic = 0;

unsigned long lastHeardMs = 0;
bool isBroadcasting = false;
unsigned long broadcastStartTime = 0;
unsigned long broadcastDuration = 150;

// Visual Modes
enum State { MODE_NORMAL, MODE_BIO, MODE_CRASH };
State currentState = MODE_NORMAL;
unsigned long modeStartTime = 0;
unsigned long modeDuration = 0;

bool satActive = false;
unsigned long satStartTime = 0;
unsigned long satDuration = 10000;
int satStartPos = 9;
int satEndPos = 29;

bool syncActive = false;
unsigned long syncStartTime = 0;
const int SYNC_DURATION = 600;

// Interaction Tracking
int interactionCount = 0;
unsigned long lastInteraction = 0;

// ======================
// 📡 BLE TRANSMITTER
// ======================
void triggerBroadcast(MsgType type, SwarmMessage* msgOverride = nullptr) {
    pBLEScan->stop(); 
    pAdvertising->stop(); 

    SwarmMessage outMsg;
    
    if (msgOverride) {
        outMsg = *msgOverride; 
    } else {
        memset(&outMsg, 0, sizeof(SwarmMessage));
    }

    // Header stamping
    outMsg.header.version = SWARM_PROTO_VERSION;
    outMsg.header.msgType = type;
    outMsg.header.senderId = swarmId;
    outMsg.header.bootToken = bootToken;
    outMsg.header.sequenceId = ++mySequenceCounter;
    outMsg.header.groupId = 0;
    outMsg.header.context = 0;

    // Fill in Ring's STATE
    if (type == MSG_STATE && !msgOverride) {
        outMsg.data.state.humidity_x10 = 0; // Ring has no sensor
        outMsg.data.state.traffic = (uint16_t)interactionCount;
        outMsg.data.state.energy = (uint8_t)myEnergy;
        outMsg.data.state.mood = (uint8_t)myMood;
        outMsg.data.state.cohesion = 128; // Ring doesn't track this
        outMsg.data.state.tempo = (uint8_t)myTempo;
        outMsg.data.state.drift = (uint8_t)myDrift;
    }

    // Calculate Payload Size
    size_t payloadSize = 0;
    switch(type) {
        case MSG_STATE: payloadSize = sizeof(MsgState); break;
        case MSG_EVENT: payloadSize = sizeof(MsgEvent); break;
        case MSG_ANNOUNCE: payloadSize = sizeof(MsgAnnounce); break;
        default: payloadSize = 0; break;
    }

    // Serialize to BLE
    size_t totalLen = sizeof(SwarmHeader) + payloadSize;
    uint8_t buffer[50];
    uint16_t id = HIVE_MANUFACTURER_ID;
    buffer[0] = (uint8_t)(id & 0xFF);
    buffer[1] = (uint8_t)((id >> 8) & 0xFF);
    memcpy(&buffer[2], &outMsg, totalLen);
    
    String mfgData = "";
    for(size_t i=0; i < 2 + totalLen; i++) mfgData += (char)buffer[i];

    BLEAdvertisementData oData = BLEAdvertisementData();
    oData.setFlags(0x04);
    oData.setManufacturerData(mfgData);
    pAdvertising->setAdvertisementData(oData);
    pAdvertising->setMinPreferred(0x20); 
    pAdvertising->setMaxPreferred(0x20); 
    
    pAdvertising->start();
    isBroadcasting = true;
    broadcastStartTime = millis();

    // Priority duration for events
    if (type == MSG_EVENT) {
        broadcastDuration = 800;
    } else {
        broadcastDuration = 150;
    }
}

// ======================
// 📡 BLE PARSER
// ======================
void parseSwarmPacket(const uint8_t* rawData, int len, int rssi) {
    SwarmMessage in;
    if (len > sizeof(SwarmMessage)) len = sizeof(SwarmMessage);
    memcpy(&in, rawData, len);

    if (in.header.version != SWARM_PROTO_VERSION) return;
    if (in.header.senderId == swarmId && in.header.bootToken == bootToken) return;
    if (in.header.senderId >= 256) return;

    lastHeardMs = millis();

    // --- STATE UPDATES ---
    if (in.header.msgType == MSG_STATE) {
        Serial.printf("[RX] #%d STATE | E:%d M:%d T:%d D:%d\n", 
            in.header.senderId, 
            in.data.state.energy, 
            in.data.state.mood,
            in.data.state.tempo,
            in.data.state.drift);
        
        // Blend incoming state into averages
        float influence = 0.08;
        avgEnergy = (avgEnergy * (1.0-influence)) + ((float)in.data.state.energy * influence);
        avgMood   = (avgMood   * (1.0-influence)) + ((float)in.data.state.mood * influence);
        avgTempo  = (avgTempo  * (1.0-influence)) + ((float)in.data.state.tempo * influence);
        avgDrift  = (avgDrift  * (1.0-influence)) + ((float)in.data.state.drift * influence);
        
        float hum = in.data.state.humidity_x10 / 10.0;
        avgHum = (avgHum * 0.95) + (hum * 0.05);
        
        globalTraffic = in.data.state.traffic;
        
        // 🔥 NEW: Ring's autonomic state influenced by swarm
        float myInfluence = 0.02; // Slower blend for Ring's own state
        myEnergy = (myEnergy * (1.0-myInfluence)) + (avgEnergy * myInfluence);
        myMood   = (myMood   * (1.0-myInfluence)) + (avgMood * myInfluence);
        myTempo  = (myTempo  * (1.0-myInfluence)) + (avgTempo * myInfluence);
        myDrift  = (myDrift  * (1.0-myInfluence)) + (avgDrift * myInfluence);
    }
    
    // --- EVENTS ---
    else if (in.header.msgType == MSG_EVENT) {
        uint8_t evId = in.data.event.eventId;
        Serial.printf("[RX] #%d EVENT | ID:%d Intensity:%d\n", 
            in.header.senderId, evId, in.data.event.intensity);

        if (evId == 5) { // BIO HAZARD
            currentState = MODE_BIO;
            modeStartTime = millis();
            modeDuration = in.data.event.durationMs;
            
            // 🔥 NEW: Bio spikes mood
            myMood += 30.0;
            if (myMood > 255) myMood = 255;
        }
        else if (evId == 1) { // CRASH
            currentState = MODE_CRASH;
            modeStartTime = millis();
            modeDuration = in.data.event.durationMs;
            
            // 🔥 NEW: Crash drains energy, spikes drift
            myEnergy -= 50.0;
            if (myEnergy < 0) myEnergy = 0;
            myDrift += 40.0;
            if (myDrift > 255) myDrift = 255;
        }
        else if (evId == 2) { // SATELLITE
            satActive = true;
            satStartTime = millis();
            satDuration = in.data.event.durationMs;
            satStartPos = (in.header.senderId % 2 == 0) ? 9 : 29;
            satEndPos = (satStartPos == 9) ? 29 : 9;
            
            // 🔥 NEW: Satellite heals drift
            myDrift -= 30.0;
            if (myDrift < 0) myDrift = 0;
        }
    }
    
    // --- SYNC ---
    else if (in.header.msgType == MSG_SYNC) {
        Serial.printf("[RX] #%d SYNC\n", in.header.senderId);
        syncActive = true;
        syncStartTime = millis();
        
        // 🔥 NEW: Sync stabilizes tempo
        myTempo = (myTempo * 0.7) + (avgTempo * 0.3);
    }
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.haveManufacturerData()) {
            String strData = advertisedDevice.getManufacturerData();
            if (strData.length() >= 2 + sizeof(SwarmHeader)) {
                const uint8_t *data = (const uint8_t*)strData.c_str();
                uint16_t mfgID = data[0] | (data[1] << 8);
                if (mfgID == HIVE_MANUFACTURER_ID) {
                    parseSwarmPacket(&data[2], strData.length() - 2, advertisedDevice.getRSSI());
                }
            }
        }
    }
};

// ======================
// 🎨 VISUAL EFFECTS
// ======================
void runBootSequence() {
  for(int speed=50; speed>0; speed-=5){
    for(int i=0;i<NUM_LEDS;i++){ 
      fill_rainbow(leds, NUM_LEDS, i*5, 7); 
      FastLED.show(); 
      delay(speed); 
    }
  }
  fill_solid(leds, NUM_LEDS, CRGB::White); 
  digitalWrite(LED_PIN_CORE, HIGH); 
  FastLED.show(); 
  delay(300);
  digitalWrite(LED_PIN_CORE, LOW);
  for(int i=255; i>0; i-=10){ 
    fadeToBlackBy(leds, NUM_LEDS, 20); 
    FastLED.show(); 
    delay(10); 
  }
}

void runCrashEffect(unsigned long elapsed, unsigned long duration) {
  float progress = (float)elapsed / (float)duration;
  
  if (progress < 0.40) {
    // Glitch phase
    CRGB glitchColor;
    int r = random8();
    if (r < 60) glitchColor = CRGB::Red;
    else if (r < 110) glitchColor = CRGB::Cyan;
    else if (r < 160) glitchColor = CRGB::Magenta;
    else if (r < 210) glitchColor = CRGB::Yellow;
    else glitchColor = CRGB::White;
    
    int numHits = random8(2, 6);
    for (int k = 0; k < numHits; k++) leds[random(NUM_LEDS)] = glitchColor;
    if (random8() > 100) analogWrite(LED_PIN_CORE, 255); 
    else analogWrite(LED_PIN_CORE, 0);
    fadeToBlackBy(leds, NUM_LEDS, 100);
  } 
  else if (progress < 0.50) {
    // Dead phase
    fill_solid(leds, NUM_LEDS, CRGB::Black); 
    analogWrite(LED_PIN_CORE, 0);
  } 
  else {
    // Reboot phase
    float spinProgress = (progress - 0.50) * 2.0;
    float revolutions = 20.0 * (spinProgress * spinProgress * spinProgress);
    long totalPixels = (long)(revolutions * NUM_LEDS);
    int pos = totalPixels % NUM_LEDS;
    uint8_t hue = (totalPixels / 2) % 255;
    fadeToBlackBy(leds, NUM_LEDS, 50); 
    leds[pos] = CHSV(hue, 255, 255);
    if (pos < 5) analogWrite(LED_PIN_CORE, 255); 
    else analogWrite(LED_PIN_CORE, 0);
  }
}

void runTrafficEffect() {
  // 🔥 MOOD: Affects base color palette
  // Low mood (0) = Blue/Cyan (sad)
  // Mid mood (128) = Green/Yellow (neutral)
  // High mood (255) = Orange/Red (excited)
  uint8_t baseHue = map((long)myMood, 0, 255, 160, 30); // 160=Blue, 30=Orange
  
  // 🔥 ENERGY: Controls metabolism/activity rate AND brightness ceiling
  int metabolism = map((long)myEnergy, 0, 255, 2, 40);
  metabolism = constrain(metabolism, 2, 50);
  
  // Energy-based brightness ceiling (0=very dim, 255=full bright)
  uint8_t brightnessCeiling = map((long)myEnergy, 0, 255, 20, 255);
  
  // 🔥 DRIFT: Adds chaos/randomness
  int chaos = map((long)myDrift, 0, 255, 0, 60);
  
  // Spawn new pixels based on metabolism + chaos
  if (random8() < (metabolism + chaos)) {
    int pos = random(NUM_LEDS);
    
    // Color variation based on drift
    int hueJitter = map((long)myDrift, 0, 255, 5, 40);
    
    // 🔥 NEW: Brightness constrained by energy level
    CHSV color = CHSV(baseHue + random(-hueJitter, hueJitter), 255, brightnessCeiling);
    leds[pos] += color;
    
    // Spread based on mood (happier = more spread)
    int spread = map((long)myMood, 0, 255, 0, 2);
    if (spread > 0) { 
      CHSV dim = color; 
      dim.val = brightnessCeiling / 3; // Dimmer spread
      leds[(pos + 1) % NUM_LEDS] += dim; 
      leds[(pos - 1 + NUM_LEDS) % NUM_LEDS] += dim;
    }
  }
  
  // 🔥 TEMPO: Controls fade speed (heartbeat)
  // Low tempo (0) = slow fade (4s period)
  // High tempo (255) = fast fade (0.5s period)
  int fadeSpeed = map((long)myTempo, 0, 255, 5, 70);
  fadeToBlackBy(leds, NUM_LEDS, fadeSpeed);
}

void runBioEffect(unsigned long elapsed, unsigned long duration) {
  float instability = (float)elapsed / (float)duration;
  int baseHeight = instability * 18;
  int jitter = (sin(millis() / 40.0) * 3) + (cos(millis() / 15.0) * 3);
  int h = constrain(baseHeight + jitter, 2, 18);
  
  for (int i = 0; i < 19; i++) {
    CRGB color = CRGB::Black;
    if (i <= h) { 
      int hue = 96, sat = 255, val = 200;
      if (i >= h - 2) { hue = 60; val = 255; sat = 100; }
      if (random(100) < (20 * instability)) { sat = 0; val = 255; }
      color = CHSV(hue, sat, val);
    }
    leds[i] = color; 
    leds[NUM_LEDS - 1 - i] = color;
  }
  
  if (random(255) < (instability * 255)) digitalWrite(LED_PIN_CORE, HIGH); 
  else digitalWrite(LED_PIN_CORE, LOW);
}

void runSatEffect(unsigned long elapsed, unsigned long duration) {
  float p = (float)elapsed / (float)duration; 
  float pos = satStartPos + (p * (satEndPos - satStartPos));
  int i = (int)pos; 
  float f = abs(pos - i);
  
  if (i >= 0 && i < NUM_LEDS) leds[i] += CHSV(10, 255, 255 * (1 - f));
  if (i == 18) { 
    leds[18] = CRGB::White; 
    if (random(2)) leds[17] = CRGB::SkyBlue;
  }
}

void runSyncEffect(unsigned long elapsed) {
  float p = (float)elapsed / (float)SYNC_DURATION;
  if (currentState != MODE_BIO) analogWrite(LED_PIN_CORE, 255 * (1.0 - p));
  
  int h = p * 20;
  for (int i = 0; i < 19; i++) {
    if (i >= h - 4 && i <= h) {
      int hue = map(i, 0, 18, 190, 250); 
      int sat = map(i, 0, 18, 255, 120); 
      int bri = 255 - ((h - i) * 60); 
      if (bri < 0) bri = 0;
      CRGB c = CHSV(hue, sat, bri); 
      leds[i] += c; 
      leds[NUM_LEDS - 1 - i] += c;
    }
  }
}

// ======================
// 🧠 AUTONOMIC DRIFT (Ring's Biological Clock)
// ======================
void updateRingAutonomics() {
    bool isolated = (millis() - lastHeardMs) > 5000;
    
    // --- 1. Energy Logic (Mass & Gravity) ---
    // If isolated, we decay FAST (-2.0). If connected, we decay slow (-0.5).
    // This matches the GNC "Active Decay" logic.
    float decay = isolated ? 2.0 : 0.5;
    myEnergy -= decay;

    // SAFETY CUTOFF (The Coma): 
    // If we are "sick" (High Drift), force Energy down so we can sleep.
    if (myDrift > 220) {
        myEnergy -= 20.0; 
    }
    myEnergy = constrain(myEnergy, 0.0, 255.0);
    
    // --- 2. Drift Logic (Entropy) ---
    // Isolation = Panic (+1.2). Connection = Safety (0.0).
    myDrift += isolated ? 1.2 : 0.0;
    
    // Natural Aging (Entropy)
    myDrift += 0.01; 
    
    // HEALING SLEEP:
    // If we are calm (Low Energy) and safe (Not Isolated), we heal.
    if (myEnergy < 80 && !isolated) {
        myDrift -= 1.5; 
    }
    myDrift = constrain(myDrift, 0.0, 255.0);
    
    // --- 3. Mood Logic ---
    // Random walk, but biased by Stress (Drift)
    myMood += random(-2, 3) * (0.5 + myDrift / 255.0);
    myMood = constrain(myMood, 0.0, 255.0);
    
    // --- 4. Tempo Logic ---
    // Linked to Energy (Healthy) vs Chaotic (Sick)
    float targetTempo;
    if (myDrift > 180) {
        targetTempo = random(40, 220); // Arrhythmia
    } else {
        targetTempo = map((long)myEnergy, 0, 255, 50, 230);
    }
    
    // Inertia (Rubber Band effect)
    myTempo += (targetTempo - myTempo) * 0.1;
    myTempo += sin(millis() * 0.0003) * 2.0;
    myTempo = constrain(myTempo, 0.0, 255.0);
    
    // Decay interaction count for Traffic density
    if (interactionCount > 0) interactionCount--;
}

// ======================
// 🚀 SETUP
// ======================
void setup() {
  Serial.begin(115200);
  FastLED.addLeds<LED_TYPE, LED_PIN_RING, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  pinMode(LED_PIN_CORE, OUTPUT);
  
  preferences.begin("hive", false);
  myX = preferences.getInt("x", 128); 
  myY = preferences.getInt("y", 128);
  swarmId = preferences.getUShort("swarmId", 0);
  
  if (swarmId == 0) {
    swarmId = (uint16_t)random(1, 254);
    preferences.putUShort("swarmId", swarmId);
  }
  preferences.end();
  
  bootToken = esp_random();
  
  runBootSequence();
  
  // Initialize BLE
  BLEDevice::init("");
  pAdvertising = BLEDevice::getAdvertising();
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); 
  pBLEScan->setInterval(40);
  pBLEScan->setWindow(40);   
  pBLEScan->start(0, nullptr); 
  
  Serial.printf("🔵 Ring Node #%d Ready\n", swarmId);
  
  fill_solid(leds, NUM_LEDS, CRGB::Green); 
  FastLED.show(); 
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black); 
  FastLED.show();
}

// ======================
// 🔄 LOOP
// ======================
void loop() {
  unsigned long now = millis();
  
  // 1. BROADCAST TIMING
  if (isBroadcasting) {
    if (now - broadcastStartTime > broadcastDuration) { 
      isBroadcasting = false;
      pAdvertising->stop();
      pBLEScan->clearResults();
      pBLEScan->start(0, nullptr); 
    }
  }
  
  // 2. MODE TIMERS
  if (currentState != MODE_NORMAL && (now - modeStartTime > modeDuration)) {
    currentState = MODE_NORMAL; 
    digitalWrite(LED_PIN_CORE, LOW);
  }
  if (satActive && (now - satStartTime > satDuration)) satActive = false;
  if (syncActive && (now - syncStartTime > SYNC_DURATION)) { 
    syncActive = false; 
    if (currentState != MODE_BIO) digitalWrite(LED_PIN_CORE, LOW);
  }
  
  // 3. STATE MACHINE
  switch (currentState) {
    case MODE_NORMAL: 
      runTrafficEffect(); 
      break;
    case MODE_BIO: 
      runBioEffect(now - modeStartTime, modeDuration); 
      break;
    case MODE_CRASH: 
      runCrashEffect(now - modeStartTime, modeDuration); 
      break;
  }
  
  // 4. OVERLAYS
  if (satActive) runSatEffect(now - satStartTime, satDuration);
  if (syncActive) runSyncEffect(now - syncStartTime);
  
  FastLED.show(); 
  delay(15);
  
  // 5. AUTONOMIC UPDATE (every 500ms)
  static unsigned long lastAutonomic = 0;
  if (now - lastAutonomic > 500) {
    updateRingAutonomics();
    lastAutonomic = now;
  }
  
  // 6. HEARTBEAT BROADCAST (tempo-driven)
  static unsigned long lastBroadcast = 0;
  long broadcastInterval = map((long)myTempo, 0, 255, 4000, 250);
  if (now - lastBroadcast > broadcastInterval) {
    triggerBroadcast(MSG_STATE);
    lastBroadcast = now;
    Serial.printf("[TX] E:%d M:%d T:%d D:%d -> Int:%dms\n", 
      (int)myEnergy, (int)myMood, (int)myTempo, (int)myDrift, (int)broadcastInterval);
  }
  
  // 7. SERIAL COMMANDS
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '?') {
      Serial.println("\n--- RING STATUS ---");
      Serial.printf("ID: %d | State: %d\n", swarmId, currentState);
      Serial.printf("MY: E:%.1f M:%.1f T:%.1f D:%.1f\n", 
        myEnergy, myMood, myTempo, myDrift);
      Serial.printf("AVG: E:%.1f M:%.1f T:%.1f D:%.1f\n", 
        avgEnergy, avgMood, avgTempo, avgDrift);
      Serial.printf("LastMsg: %ldms ago\n", now - lastHeardMs);
    }
    else if (c == 'c') {
      // Manual crash
      currentState = MODE_CRASH;
      modeStartTime = millis();
      modeDuration = 6000;
      myEnergy -= 50;
      myDrift += 40;
      
      SwarmMessage msg;
      msg.header.msgType = MSG_EVENT;
      msg.data.event.eventId = 1;
      msg.data.event.intensity = 200;
      msg.data.event.durationMs = 6000;
      triggerBroadcast(MSG_EVENT, &msg);
    }
    else if (c == 's') {
      // Manual satellite
      satActive = true;
      satStartTime = millis();
      myDrift -= 30;
      
      SwarmMessage msg;
      msg.header.msgType = MSG_EVENT;
      msg.data.event.eventId = 2;
      msg.data.event.intensity = 255;
      msg.data.event.durationMs = 10000;
      triggerBroadcast(MSG_EVENT, &msg);
    }
  }
}