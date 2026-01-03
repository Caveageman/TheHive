#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include "SwarmProtocol.h" 

// Compile-time sanity check for ESP-NOW payload size
static_assert(sizeof(SwarmMessage) <= 250, "SwarmMessage exceeds ESP-NOW payload size");

// ======================
// 🛠️ HARDWARE PINS (C3 SUPER MINI)
// ======================
#define I2C_SDA 2   
#define I2C_SCL 3   

// LED Pinout
#define LED_PIN_1  5   // Sync Blink 1 (5s) / Boot Step 1
#define LED_PIN_2  7   // Sync Blink 2 (3s) / Boot Step 2
#define LED_PIN_3  20  // Heartbeat / Boot Step 3
#define LED_PIN_4  1   // Data Activity
#define LED_PIN_5  0   // Satellite / Event Fade

#define BOOT_BUTTON 9 

// ======================
// 🤖 NODE PERSONALITY & CONFIG
// ======================
// Base probabilities per 10,000 ticks (~11 hours)
#define BASE_CHANCE_CRASH     15      // Default: 5 (Rare)
#define BASE_CHANCE_SATELLITE 115     // Default: 15 (Uncommon)

// Visual Settings
uint8_t currentGraphWidth = 2; 

// Timing
#define FAST_TICK_MS    50   
#define MED_TICK_MS     100  
#define SLOW_TICK_MS    4000 

// ======================
// CONFIGURATION (SYS)
// ======================
#define CPU_MHZ 80 
#define SCREEN_HW_WIDTH  128
#define SCREEN_HW_HEIGHT 32
#define OLED_ADDR   0x3C
#define VISUAL_WIDTH  32
#define VISUAL_HEIGHT 128

// ======================
// GLOBAL STATE
// ======================
uint32_t bootToken;
uint16_t swarmId = 0; 
uint8_t posX = 255;
uint8_t posY = 255;
Preferences prefs;

// Hardware Flags
bool hasOLED = false;
bool hasSensor = false;
bool isIsolated = false;
unsigned long lastHeardMs = 0;

//Sweep
const bool ENABLE_SWEEP_REPORT = true;       // Set TRUE to sweep all channels
const unsigned long SWEEP_SEND_DELAY_MS = 15; // Slight increase to prevent packet choke

// Data State
float localPrimaryVal = 0.0; 
float swarmAvgVal = 0.0;     
volatile int rawPacketCount = 0; 
int lastNoiseLevel = 0; // Stored noise level for crash logic

// Bio-Sensing State
float avgHum = 0.0;        
float deltaHum = 0.0;      
bool isBioEvent = false;   
unsigned long bioEventEnd = 0;

// LED Timers 
unsigned long lastBlink1 = 0;
unsigned long lastBlink2 = 0;

// Swarm Energy
float swarmEnergy = 255.0; 
float currentBrightness = 255.0; 

// Signal Routing
enum SignalSource : uint8_t {
  SRC_SILENCE = 0,
  SRC_LOCAL_BREATH = 1, 
  SRC_SWARM_TRAFFIC = 2, 
  SRC_SWARM_AVG = 3,     
  SRC_SINE_WAVE = 4      
};

SignalSource graphSourceA = SRC_LOCAL_BREATH;
SignalSource graphSourceB = SRC_SWARM_TRAFFIC;

// Comms
SwarmMessage incomingMsg;
volatile bool msgReceived = false;

// Timing
unsigned long lastFastTick = 0;
unsigned long lastMedTick  = 0;
unsigned long lastSlowTick = 0;

// Peripherals
Adafruit_SSD1306 display(SCREEN_HW_WIDTH, SCREEN_HW_HEIGHT, &Wire, -1);
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ======================
// GRAPHICS & IDENTITY
// ======================
enum IdState { ID_UNASSIGNED, ID_CLAIMING, ID_ASSIGNED };
IdState nodeState = ID_UNASSIGNED;
uint16_t candidateId = 0;
unsigned long stateTimer = 0;
const int CLAIM_WINDOW_MS = 1500;

struct VerticalGraph {
  int8_t values[VISUAL_HEIGHT]; 
  uint8_t head; 
};
VerticalGraph graphDataA; 
VerticalGraph graphDataB; 

enum OverlayType : uint8_t { OVERLAY_NONE, OVERLAY_JITTER, OVERLAY_DROPOUT, OVERLAY_PHASE_SHEAR };
struct Overlay {
  OverlayType type;
  uint8_t intensity; 
  uint16_t ttl;      
};
Overlay activeOverlays[2];

// Crash State
bool wasCrashing = false; 

// ======================
// 🛰️ CHANNEL / TRF SNIFER (ADDED)
// ======================
// Per-channel aggregation for RingNC compatibility & auto-lock
volatile uint16_t channelCounts[14];   // index 1..13 used
volatile int32_t channelRSSI[14];      // sum of rssi per channel
volatile int reportPacketCount = 0;    // aggregated counter for reporting to RingNC
volatile int totalRSSI = 0;            // aggregated RSSI for reporting
volatile int currentChannel = 1;       // channel where the radio currently listens
unsigned long lastChannelEval = 0;
unsigned long channelLockUntil = 0;

//Commented out due to conflict with burst comms
//const unsigned long CHANNEL_EVAL_INTERVAL = 10000;  // evaluate every 10s
//const unsigned long CHANNEL_LOCK_DURATION  = 60000; // stay locked 60s after choosing
//const bool ENABLE_SWEEP_REPORT = false;             // set true to also sweep and send on all channels
//const unsigned long SWEEP_SEND_DELAY_MS = 10;       // delay between sends when sweeping

// ======================
// 🎨 RENDERER CLASS
// ======================
class OLEDRenderer {
public:
  OLEDRenderer(Adafruit_SSD1306* disp) : display(disp) {}

  void begin() {
    if(!display->begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return;
    display->setRotation(1); 
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->clearDisplay();
    display->display();
  }

  void setGraphs(VerticalGraph* gA, VerticalGraph* gB) {
    graphA = gA; graphB = gB;
    memset(graphA->values, 0, sizeof(graphA->values));
    memset(graphB->values, 0, sizeof(graphB->values));
    graphA->head = 0;
    graphB->head = 0;
  }

  void setOverlays(Overlay* ov, uint8_t count) {
    overlays = ov; overlayCount = count;
  }

  void pushValue(VerticalGraph* g, int8_t val) {
    if (!g) return;
    g->head = (g->head + 1) % VISUAL_HEIGHT;
    g->values[g->head] = val;
  }

  void render() {
    display->clearDisplay();

    // Use runtime dimensions so rotation / different SSD sizes are handled safely
    int w = display->width();
    int h = display->height();

    // --- UI CONFIG (runtime-based) ---
    const int HEADER_LINE_Y = 8;
    const int GRAPH_TOP     = HEADER_LINE_Y + 2;
    const int GRAPH_BOT     = h - 16; 
    const int FOOTER_LINE_Y = h - 16; 
    const int FOOTER_TEXT_Y = h - 13; 

    // --- initialize scanline on first call (must be runtime-aware) ---
    static int scanY = 0;
    static bool scanInit = false;
    if (!scanInit) { scanY = GRAPH_TOP; scanInit = true; }

    // --- 1. HEADER (#01) ---
    display->setCursor(0, 0);
    if (swarmId == 0) {
        display->print("??");
    } else {
        display->print("#");
        if (swarmId < 10) display->print("0");
        display->print(swarmId);
    }
    
    // Slow Blink Activity Dot
    if ((millis() / 500) % 2 == 0) {
        // clip the dot inside current width/height
        if (w > 26 && h > 3) display->fillCircle(26, 3, 2, SSD1306_WHITE);
    }

    display->drawLine(0, HEADER_LINE_Y, w - 1, HEADER_LINE_Y, SSD1306_WHITE);

    // --- 2. GRAPHS (CENTERED & WINDOWED) ---
    // Graph A moved from 6 -> 8 (Centers it in the left lane)
    renderGraph(graphA, 8, GRAPH_TOP, GRAPH_BOT);  
    
    // Graph B at 24 (Centers it in the right lane)
    renderGraph(graphB, 24, GRAPH_TOP, GRAPH_BOT); 

    // --- 3. CRT SCANLINE ---
    static int scanDir = 1;
    scanY += scanDir;
    if (scanY >= GRAPH_BOT) { scanY = GRAPH_BOT; scanDir = -1; }
    else if (scanY <= GRAPH_TOP) { scanY = GRAPH_TOP; scanDir = 1; }
    // draw entire width of scanline safely
    display->drawFastHLine(0, scanY, w, SSD1306_INVERSE);

    // --- 4. FOOTER (Status) ---
    display->drawLine(0, FOOTER_LINE_Y, w - 1, FOOTER_LINE_Y, SSD1306_WHITE);
    display->setCursor(0, FOOTER_TEXT_Y);
    
    if (isIsolated) {
        display->print("!ISO!");
    } else {
        int p = (int)map((long)swarmEnergy, 0, 255, 0, 100);
        if (p < 100) display->print(" "); 
        if (p < 10)  display->print(" ");
        display->print(p); 
        display->print("%");
    }

    display->display();
  }

private:
  Adafruit_SSD1306* display;
  VerticalGraph* graphA = nullptr;
  VerticalGraph* graphB = nullptr;
  Overlay* overlays = nullptr;
  uint8_t overlayCount = 0;

  void applyOverlays(int8_t &val, int row) {
    if (!overlays) return;
    for (uint8_t i=0; i<overlayCount; i++) {
      Overlay &o = overlays[i];
      if (o.ttl == 0) continue;
      switch(o.type) {
        case OVERLAY_JITTER: val += random(-o.intensity, o.intensity+1); break;
        case OVERLAY_DROPOUT: if (random(0,255) < o.intensity) val = -128; break;
        case OVERLAY_PHASE_SHEAR: val += (row % 2) ? o.intensity/2 : -o.intensity/2; break;
        default: break;
      }
    }
  }

  void renderGraph(VerticalGraph* graph, uint8_t xCenter, int yMin, int yMax) {
    if (!graph) return;
    
    int w = display->width();
    // We loop through the SCREEN Y coordinates from Top to Bottom
    for (int y = yMin; y < yMax; y++) {
      
      // Calculate how far back in the history buffer we need to look.
      // yMax is the "newest" data, yMin is the "oldest" visible data.
      int bufferOffset = (yMax - 1) - y;
      
      int16_t idx = graph->head - bufferOffset;
      // Handle buffer wrap-around (circular buffer)
      if (idx < 0) idx += VISUAL_HEIGHT; 
      while(idx >= VISUAL_HEIGHT) idx -= VISUAL_HEIGHT;

      int8_t val = graph->values[idx];
      
      // Apply Glitch Effects
      applyOverlays(val, y);

      // Draw if value is valid (not a "dropout" value)
      if (val != -128) {
        int x = xCenter + val;
        // Clamp X to actual display width so it doesn't attempt to draw outside framebuffer
        if (x >= 0 && x < w) {
          display->drawPixel(x, y, SSD1306_WHITE);
          // Thicker line option
          if (currentGraphWidth > 1 && (x + 1) < w) {
            display->drawPixel(x + 1, y, SSD1306_WHITE);
          }
        }
      }
    }
  }
};

OLEDRenderer renderer(&display);

// ======================
// CALLBACKS
// ======================

void IRAM_ATTR wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  // existing visual noise counter
  rawPacketCount++;

  // aggregation for channel/trf logic
  reportPacketCount++;

  // Try to extract RSSI if packet struct is available
  wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t*)buf;
  int rssi = 0;
  if (p) {
    rssi = p->rx_ctrl.rssi;
    if (rssi < -95) rssi = -95;
    if (rssi > -20) rssi = -20;
    totalRSSI += rssi;
  }

  // bucket by current channel
  int ch = currentChannel;
  if (ch >= 1 && ch <= 13) {
    channelCounts[ch]++;
    channelRSSI[ch] += rssi;
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(SwarmMessage)) return;
  SwarmMessage in;
  memcpy(&in, incomingData, sizeof(in));

  if (in.header.version != SWARM_PROTO_VERSION) return;

  // COPY into the global message BEFORE signaling
  incomingMsg = in;
  msgReceived = true; 
  lastHeardMs = millis(); 

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
      
      activeOverlays[0].type = OVERLAY_JITTER;
      activeOverlays[0].intensity = 4;
      activeOverlays[0].ttl = 5;
    }
    if (nodeState == ID_CLAIMING && in.data.id.targetId == candidateId && in.header.bootToken < bootToken) {
      nodeState = ID_UNASSIGNED; 
      stateTimer = millis() + random(100, 500); 
    }
  }

  // SYNC
  if (in.header.msgType == MSG_SYNC) {
      lastBlink1 = millis();
      lastBlink2 = millis();
      renderer.pushValue(&graphDataB, 12);
  }

  // EVENTS
  if (in.header.msgType == MSG_EVENT) {
    uint16_t frames = in.data.event.durationMs / FAST_TICK_MS;
    if (frames < 1) frames = 1;

    if (in.data.event.eventId == 5) { // Bio
       renderer.pushValue(&graphDataA, in.data.event.intensity / 4);
    }
    
    if (in.data.event.eventId == 4) { // Darkness
       swarmEnergy = 20.0; 
    }

    switch(in.data.event.eventId) {
      case 1: // CRASH
        activeOverlays[0].type = OVERLAY_DROPOUT;
        activeOverlays[0].intensity = in.data.event.intensity;
        activeOverlays[0].ttl = frames;
        wasCrashing = true; 
        break;
      case 2: // SATELLITE
        activeOverlays[0].type = OVERLAY_PHASE_SHEAR;
        activeOverlays[0].intensity = in.data.event.intensity; 
        activeOverlays[0].ttl = frames;
        break;
    }
  }

  // STATE
  if (in.header.msgType == MSG_STATE) {
    float incomingVal = in.data.state.humidity;
    if (swarmAvgVal == 0) swarmAvgVal = incomingVal;
    else swarmAvgVal = (swarmAvgVal * 0.95) + (incomingVal * 0.05);
    
    float incEnergy = in.data.state.energy;
    swarmEnergy = (swarmEnergy * 0.9) + (incEnergy * 0.1);

    renderer.pushValue(&graphDataB, 12); 
    analogWrite(LED_PIN_4, (int)currentBrightness);
  }
}

// ======================
// 🛰️ NEW: ACTIVE SCANNER & HOPPER
// ======================
void scanAndHop() {
  unsigned long now = millis();
  
  // 1. HOP LOGIC: Switch channels rapidly to "scan" the air
  // We hop every 100-200ms to gather visual data from the whole spectrum
  if (now - lastChannelEval > 150) { 
    lastChannelEval = now;

    // Move to next channel (Round Robin 1 -> 13)
    currentChannel++;
    if (currentChannel > 13) currentChannel = 1;
    
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    
    // Reset counters for the new channel so visuals are fresh
    reportPacketCount = 0; 
  }
}

// Helper to blast a message across all channels (The "Old Way")
void sendMultiChannelBurst(uint8_t* data, size_t len) {
    if (!ENABLE_SWEEP_REPORT) {
        esp_now_send(BROADCAST_ADDR, data, len);
        return;
    }

    // Save current channel to return to it later
    uint8_t originalCh = currentChannel;

    // Temporary Peer for broadcasting
    uint8_t baddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, baddr, 6);
    peer.encrypt = false;

    for (int ch = 1; ch <= 13; ch++) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        // We have to re-add the peer on some ESP versions when changing channels
        // forcing the peer channel to 0 (any) or current helps
        peer.channel = ch;
        esp_now_add_peer(&peer); 
        
        esp_now_send(baddr, data, len);
        delay(SWEEP_SEND_DELAY_MS);
    }
    
    // Return to scanning
    esp_wifi_set_channel(originalCh, WIFI_SECOND_CHAN_NONE);
}

// ======================
// HELPER FUNCTIONS
// ======================

void runBootSequence() {
  if (!hasOLED) return;
  display.clearDisplay(); display.display(); delay(200);

  display.setCursor(0, 10); display.println("BOOT..");
  display.display(); 
  analogWrite(LED_PIN_1, 255); delay(150); analogWrite(LED_PIN_1, 0);

  display.setCursor(0, 30); display.println("MEM OK");
  display.display(); 
  analogWrite(LED_PIN_2, 255); delay(150); analogWrite(LED_PIN_2, 0);

  display.setCursor(0, 50); display.println("RADIO");
  display.display(); 
  analogWrite(LED_PIN_3, 255); delay(150); analogWrite(LED_PIN_3, 0);

  display.setCursor(0, 70); display.println("HIVE..");
  display.display(); 
  delay(400);

  display.clearDisplay();
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
        sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
        nodeState = ID_CLAIMING;
        stateTimer = now;
      }
      break;
    case ID_CLAIMING:
      if (now - stateTimer > CLAIM_WINDOW_MS) {
        swarmId = candidateId;
        nodeState = ID_ASSIGNED;
        prefs.putUShort("swarmId", swarmId);
        display.invertDisplay(true);
        delay(50);
        display.invertDisplay(false);
      }
      break;
    case ID_ASSIGNED: break;
  }
}

void updateSensors() {
  if (hasSensor) {
    float hum = sht31.readHumidity();
    if (avgHum == 0) avgHum = hum;
    avgHum = (avgHum * 0.999) + (hum * 0.001);
    
    deltaHum = hum - avgHum;
    if (deltaHum < 0) deltaHum = 0; 
    
    if (deltaHum > 15.0) { 
        isBioEvent = true;
        bioEventEnd = millis() + 5000;
    }
    
    if (isBioEvent && millis() > bioEventEnd) isBioEvent = false;

    localPrimaryVal = deltaHum; 
  } else {
    // Simulation
    if (random(0,1000) > 995) localPrimaryVal = 20.0;
    else localPrimaryVal = localPrimaryVal * 0.9;
  }

  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > 4000 + random(0, 1000)) {
    lastBroadcast = millis();
    if (nodeState == ID_ASSIGNED) {
      SwarmMessage msg;
      msg.header.version = SWARM_PROTO_VERSION;
      msg.header.msgType = MSG_STATE;
      msg.header.senderId = swarmId;
      msg.header.bootToken = bootToken;
      msg.data.state.temp = 0; 
      msg.data.state.humidity = hasSensor ? sht31.readHumidity() : swarmAvgVal;
      msg.data.state.energy = (uint8_t)currentBrightness; 
      msg.data.state.traffic = reportPacketCount; 
      msg.data.state.traffic = msg.data.state.traffic; // explicit
      msg.data.state.traffic = constrain(msg.data.state.traffic, 0, 65535);
      msg.data.state.traffic = msg.data.state.traffic;

      msg.data.state.traffic = reportPacketCount;
      msg.data.state.traffic = (uint16_t)reportPacketCount;

      sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));

      if (isBioEvent) {
          SwarmMessage bioMsg;
          bioMsg.header.version = SWARM_PROTO_VERSION;
          bioMsg.header.msgType = MSG_EVENT;
          bioMsg.header.senderId = swarmId;
          bioMsg.data.event.eventId = 5; 
          bioMsg.data.event.intensity = (uint8_t)constrain(localPrimaryVal * 5, 0, 255);
          bioMsg.data.event.durationMs = 1000;
          sendMultiChannelBurst((uint8_t *) &bioMsg, sizeof(bioMsg));
      }
    }
  }
}

int8_t getSignalValue(SignalSource src) {
  int8_t out = 0;
  switch (src) {
    case SRC_SILENCE: out = 0; break;
    case SRC_LOCAL_BREATH: 
      {
        // Smoother, cleaner wave
        float idleWave = sin(millis() / 1500.0) * 3.0; // Increased amplitude slightly
        float breathSpike = map((long)localPrimaryVal, 0, 30, 0, 12);
        out = (int8_t)(idleWave + breathSpike);
        // NO JITTER ADDED
      } break;
    case SRC_SWARM_TRAFFIC: {
        int8_t noise = map(rawPacketCount, 0, 50, 0, 8);
        if (noise > 8) noise = 8;
        lastNoiseLevel = noise; // Save for crash logic
        rawPacketCount = 0; 
        out = random(-noise, noise + 1);
      } break;
    case SRC_SWARM_AVG:
      out = (int8_t)((swarmAvgVal - 50.0) / 10.0);
      out += (int8_t)(sin(millis() / 2000.0) * 1.5); 
      // NO JITTER ADDED
      break;
    case SRC_SINE_WAVE:
      out = (int8_t)(sin(millis() / 500.0) * 6.0);
      break;
  }
  return out;
}

// ======================
// ANIMATION TICKS
// ======================

void fastTick() {
  unsigned long now = millis();

  // 1. CRASH OVERRIDE
  if (wasCrashing) {
     if (activeOverlays[0].ttl > 0) {
        analogWrite(LED_PIN_1, random(0, 255));
        analogWrite(LED_PIN_2, random(0, 255));
        analogWrite(LED_PIN_3, random(0, 255));
        analogWrite(LED_PIN_4, random(0, 255));
        analogWrite(LED_PIN_5, random(0, 255));
     } else {
        wasCrashing = false;
        analogWrite(LED_PIN_1, 0); analogWrite(LED_PIN_2, 0);
        analogWrite(LED_PIN_3, 0); analogWrite(LED_PIN_4, 0); analogWrite(LED_PIN_5, 0);
        runBootSequence();
     }
  } 
  else {
      // 2. NORMAL OPERATION
      if (isBioEvent) currentGraphWidth = 2; else currentGraphWidth = 1;

      currentBrightness = (currentBrightness * 0.95) + (swarmEnergy * 0.05);
      int brightInt = (int)currentBrightness;

      if (msgReceived) {
        msgReceived = false; 
        lastHeardMs = now; 
        if (incomingMsg.header.msgType != MSG_STATE) {
           renderer.pushValue(&graphDataB, 3);
           analogWrite(LED_PIN_4, brightInt);
        }
      } else {
        static int led4Val = 0;
        if (led4Val > 0) led4Val -= 10;
        if (led4Val < 0) led4Val = 0;
        analogWrite(LED_PIN_4, led4Val);
      }

      float breath = (exp(sin(now/2000.0*PI)) - 0.36787944)*108.0;
      int breathVal = map((int)breath, 0, 255, 0, brightInt);
      analogWrite(LED_PIN_3, breathVal);

      if (now - lastBlink1 > 5000) {
          analogWrite(LED_PIN_1, brightInt); 
          if (now - lastBlink1 > 5020) {     
              analogWrite(LED_PIN_1, 0);
              lastBlink1 = now;
          }
      } else if (now - lastBlink1 > 20) analogWrite(LED_PIN_1, 0);

      if (now - lastBlink2 > 3000) {
          analogWrite(LED_PIN_2, brightInt);
          if (now - lastBlink2 > 3020) {
              analogWrite(LED_PIN_2, 0);
              lastBlink2 = now;
          }
      } else if (now - lastBlink2 > 20) analogWrite(LED_PIN_2, 0);

      if (activeOverlays[0].type == OVERLAY_PHASE_SHEAR) {
         int satVal = (sin(now / 200.0) + 1.0) * 127.0; 
         analogWrite(LED_PIN_5, map(satVal, 0, 255, 0, brightInt));
      } else {
         analogWrite(LED_PIN_5, 0);
      }
  }

  for(int i=0; i<2; i++) {
    if(activeOverlays[i].ttl > 0) activeOverlays[i].ttl--;
  }
  renderer.render();
}

void mediumTick() {
  int8_t valA = getSignalValue(graphSourceA);
  int8_t valB = getSignalValue(graphSourceB);
  renderer.pushValue(&graphDataA, valA);
  renderer.pushValue(&graphDataB, valB);
}

void slowTick() {
  // Evaluate channels early so locking & reporting happens before other announcements
    scanAndHop();

  isIsolated = (millis() - lastHeardMs) > 15000;
  
  // Random Glitch
  if (random(0, 10) > 8) {
    activeOverlays[0].type = OVERLAY_JITTER;
    activeOverlays[0].intensity = 2;
    activeOverlays[0].ttl = 10; 
  }
  
  // --- AUTONOMOUS SWARM TRIGGERS ---
  if (nodeState == ID_ASSIGNED) {
      long r = random(0, 10000);
      
      // Calculate Dynamic Thresholds based on Swarm State
      long crashThresh = BASE_CHANCE_CRASH;
      long satThresh   = BASE_CHANCE_SATELLITE;

      // 1. Swarm Energy Impact
      // High Energy (>200) -> 2x Satellite Chance
      // Stealth Mode (<50) -> 10x Less Likely to emit anything
      if (currentBrightness > 200) {
          satThresh *= 2; 
      } else if (currentBrightness < 50) {
          satThresh /= 10;
          crashThresh /= 10;
      }

      // 2. Radio Interference Impact (Noise -> Crash)
      // If noise on Graph B is high, increase crash chance
      if (lastNoiseLevel > 5) {
          crashThresh *= 4; // High interference = Unstable
      }

      // 1. CRASH 
      if (r < crashThresh) {
          SwarmMessage msg;
          msg.header.version = SWARM_PROTO_VERSION;
          msg.header.msgType = MSG_EVENT;
          msg.header.senderId = swarmId;
          msg.header.bootToken = bootToken;
          msg.data.event.eventId = 1; // CRASH
          msg.data.event.intensity = 200;
          msg.data.event.durationMs = 3000;
          sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
          Serial.println("Auto-Trigger: CRASH");
      }
      // 2. SATELLITE 
      else if (r < (crashThresh + satThresh)) {
          SwarmMessage msg;
          msg.header.version = SWARM_PROTO_VERSION;
          msg.header.msgType = MSG_EVENT;
          msg.header.senderId = swarmId;
          msg.header.bootToken = bootToken;
          msg.data.event.eventId = 2; // SATELLITE
          msg.data.event.intensity = 150;
          msg.data.event.durationMs = 6000;
          sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
          Serial.println("Auto-Trigger: SATELLITE");
      }
  }

  // Occasional SYNC Broadcast (ID #1 helps keep rhythm)
  if (nodeState == ID_ASSIGNED && swarmId == 1 && random(0,10) > 8) {
      SwarmMessage syncMsg;
      syncMsg.header.version = SWARM_PROTO_VERSION;
      syncMsg.header.msgType = MSG_SYNC;
      syncMsg.header.senderId = swarmId;
      sendMultiChannelBurst((uint8_t *) &syncMsg, sizeof(syncMsg));
  }

  if (nodeState == ID_ASSIGNED) {
    SwarmMessage msg;
    msg.header.version = SWARM_PROTO_VERSION;
    msg.header.msgType = MSG_ANNOUNCE;
    msg.header.senderId = swarmId;
    msg.header.bootToken = bootToken;
    msg.data.announce.posX = posX;
    msg.data.announce.posY = posY;
    msg.data.announce.capabilities = (hasOLED ? 1 : 0); 
    sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
  }
}

// ======================
// SETUP
// ======================

void setup() {
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);
  
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  pinMode(LED_PIN_4, OUTPUT);
  pinMode(LED_PIN_5, OUTPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  
  bootToken = esp_random();
  // seed PRNG for non-deterministic random() usage
  randomSeed((unsigned long)esp_random());

  prefs.begin("swarm", false);
  // Restore persisted state
  swarmId = prefs.getUShort("swarmId", 0);
  if (swarmId != 0) nodeState = ID_ASSIGNED;

  posX = prefs.getUChar("posX", 255);
  posY = prefs.getUChar("posY", 255);
  if (posX == 255) {
    posX = esp_random() & 0xFF;
    posY = esp_random() & 0xFF;
    prefs.putUChar("posX", posX);
    prefs.putUChar("posY", posY);
  }

  Wire.begin(I2C_SDA, I2C_SCL); 
  renderer.begin();
  if (display.width() > 0) hasOLED = true; 
  renderer.setGraphs(&graphDataA, &graphDataB);
  renderer.setOverlays(activeOverlays, 2);
  
  if (sht31.begin(0x44)) { 
    hasSensor = true;
    graphSourceA = SRC_LOCAL_BREATH; 
    Serial.println("Sensor Found -> Local Breath");
  } else {
    hasSensor = false;
    graphSourceA = SRC_SWARM_AVG;    
    Serial.println("No Sensor -> Swarm Avg");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Error");
  } else {
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDR, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb);
  }
  lastHeardMs = millis();

  // Initialize sniffer channel to default 1
  currentChannel = 1;
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);

  runBootSequence();
  
  lastBlink1 = millis();
  lastBlink2 = millis();
}

void loop() {
  unsigned long now = millis();
  manageIdentity(); 
  updateSensors();  
  if (now - lastFastTick >= FAST_TICK_MS) { lastFastTick = now; fastTick(); }
  if (now - lastMedTick >= MED_TICK_MS) { lastMedTick = now; mediumTick(); }
  if (now - lastSlowTick >= SLOW_TICK_MS) { lastSlowTick = now; slowTick(); }
  
  // Debug
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') { 
        SwarmMessage msg;
        msg.header.version = SWARM_PROTO_VERSION;
        msg.header.msgType = MSG_EVENT;
        msg.header.senderId = swarmId;
        msg.header.bootToken = bootToken;
        msg.data.event.eventId = 1; // CRASH
        msg.data.event.intensity = 120;
        msg.data.event.durationMs = 2000;
        sendMultiChannelBurst((uint8_t *) &msg, sizeof(msg));
    }
  }
}

