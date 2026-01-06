#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
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

// C3 SuperMini Pinout
#define I2C_SDA 6   
#define I2C_SCL 5   
#define LED_PIN_1  4   // Sync Blink 1 
#define LED_PIN_2  3   // Sync Blink 2 
#define LED_PIN_3  2   // HEARTBEAT
#define LED_PIN_4  0   // Data Activity
#define LED_PIN_5  1   // BEACON / Bio Fade
#define BOOT_BUTTON 9 

#define SCREEN_HW_WIDTH  128
#define SCREEN_HW_HEIGHT 32
#define OLED_ADDR   0x3C
#define VISUAL_WIDTH  32
#define VISUAL_HEIGHT 128

// 🧬 GENETIC PARAMETERS
struct GeneticParams {
    // Graph Sources (20-29)
    uint8_t graphSourceA = 1;      // 1=Sensor, 2=Noise, 3=Energy, 4=Drift, 5=Mood, 6=Tempo
    uint8_t graphSourceB = 2;
    
    // LED Assignments (30-39)
    uint8_t pinHeartbeat = 2;      // Physical pin index (0-4)
    uint8_t pinBio = 4;
    uint8_t pinSync = 0;
    uint8_t pinData = 3;
    
    // Visual Tuning (40-49)
    uint8_t masterBrightness = 255;
    uint8_t graphWidthMin = 1;
    uint8_t graphWidthMax = 4;
    
    // Behavior Tuning (50-59)
    uint8_t driftSensitivity = 100;  // 0-255: How fast drift accumulates
    uint8_t energyDecay = 100;        // 0-255: How fast energy drains
    uint8_t confusionThreshold = 150; // 0-255: Chaos mode trigger
} genes;

#define MASTER_BRIGHTNESS genes.masterBrightness

// ======================
// 📜 SWARM PROTOCOL V4 (Packed)
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
    MSG_PARAM       = 7,  // 🧬 NEW: Parameter updates
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

struct __attribute__((packed)) MsgParam {
    uint16_t targetId;  // 0 = All Nodes, >0 = Specific Node
    uint8_t paramId;
    uint8_t value;
};

struct __attribute__((packed)) SwarmMessage {
    SwarmHeader header;
    union {
        MsgId       id;
        MsgEvent    event;
        MsgState    state;
        MsgAnnounce announce;
        MsgParam    param;
    } data;
};


// ======================
// 🧠 GLOBAL STATE
// ======================
Adafruit_SSD1306 display(SCREEN_HW_WIDTH, SCREEN_HW_HEIGHT, &Wire, -1);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Preferences prefs;

BLEAdvertising *pAdvertising;
BLEScan *pBLEScan;

// Sensor Averaging
float smoothTemp = 0;
float smoothHum = 0;
float humBaseline = 0;

// Add global variables
bool settingsChanged = false;
unsigned long lastSettingsChange = 0;

// Swarm Logic
uint32_t bootToken;
uint16_t swarmId = 0; 
uint16_t mySequenceCounter = 0;
uint16_t lastSeenSeqId[256] = {0}; 
float confusionLevel = 0.0; 

// Radio State
bool isBroadcasting = false;
unsigned long broadcastStartTime = 0;
unsigned long broadcastDuration = 150;
volatile int alienPacketRate = 0; 
volatile int rawPacketCount = 0;  

// Identity State
enum IdState { ID_UNASSIGNED, ID_CLAIMING, ID_ASSIGNED };
IdState nodeState = ID_UNASSIGNED;
uint16_t candidateId = 0;
unsigned long stateTimer = 0;
unsigned long lastDataRx = 0;

// Sensor / Autonomic State
bool hasOLED = false;
bool hasSensor = false;
bool isIsolated = false;
unsigned long lastHeardMs = 0;

// Events
float avgHum = 0.0;        
bool isBioEvent = false;   
unsigned long bioEventEnd = 0;
unsigned long lastBlink1 = 0;

float axisEnergy   = 180.0;
float axisMood     = 128.0; 
float axisCohesion = 160.0; 
float axisTempo    = 120.0;
float axisDrift    = 10.0;  
float swarmAvgVal  = 0.0;
float currentBrightness = (float)MASTER_BRIGHTNESS;

uint8_t currentGraphWidth = 2; 

// Swarm Field
float swarmTraffic = 0.0;
float swarmEnergy  = 0.0;

uint16_t advertsThisWindow = 0;
int32_t  rssiAccum = 0;
uint16_t rssiSamples = 0;

// Map logical index (0-4) to physical pins
const int LED_PINS[5] = { LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5 };

// Helper function to safely get pin
int getSafePin(uint8_t geneIndex) {
    if (geneIndex >= 5) return LED_PINS[0]; // Default to safe pin
    return LED_PINS[geneIndex];
}

// Forward Declarations
void triggerCrash(uint8_t intensity);
void triggerSatellite();

// ======================
// 🎨 RENDERER
// ======================
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

class OLEDRenderer {
public:
  OLEDRenderer(Adafruit_SSD1306* disp) : display(disp) {}
  
  bool chaosMode = false;
  bool bioMode = false;      
  bool crashMode = false;     
  bool satelliteMode = false; 
  
  uint8_t bioIntensity = 0;  

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
    graphA->head = 0; graphB->head = 0;
  }

  void setOverlays(Overlay* ov, uint8_t count) { overlays = ov; overlayCount = count; }

  void pushValue(VerticalGraph* g, int8_t val) {
    if (!g) return;
    g->head = (g->head + 1) % VISUAL_HEIGHT;
    g->values[g->head] = val;
  }

  void render() {
    display->clearDisplay();

    if (crashMode) {
        renderCrashVisuals();
        display->display();
        return;
    }

    if (satelliteMode) {
        renderSatelliteVisuals();
        display->display();
        return;
    }

    display->invertDisplay(false);
    int w = display->width();
    int h = display->height();

    const int HEADER_LINE_Y = 8;
    const int GRAPH_TOP     = HEADER_LINE_Y + 2;
    const int GRAPH_BOT     = h - 16; 
    const int FOOTER_LINE_Y = h - 16;
    const int FOOTER_TEXT_Y = h - 13; 

    // Header
    display->setCursor(0, 0);
    if (swarmId == 0) display->print("??");
    else {
        display->print("#");
        if (swarmId < 10) display->print("0");
        display->print(swarmId);
    }
    
    if (confusionLevel > (genes.confusionThreshold / 255.0)) display->drawChar(24, 0, '!', SSD1306_WHITE, SSD1306_BLACK, 1);
    else if ((millis() / 200) % 2 == 0) display->fillCircle(26, 3, 2, SSD1306_WHITE);
    
    display->drawLine(0, HEADER_LINE_Y, w - 1, HEADER_LINE_Y, SSD1306_WHITE);

    // Graphs
    renderGraph(graphA, 8, GRAPH_TOP, GRAPH_BOT);
    renderGraph(graphB, 24, GRAPH_TOP, GRAPH_BOT); 

    // Scanline
    static int scanY = GRAPH_TOP;
    static int scanDir = 1;
    scanY += scanDir;
    if (scanY >= GRAPH_BOT) { scanY = GRAPH_BOT; scanDir = -1; }
    else if (scanY <= GRAPH_TOP) { scanY = GRAPH_TOP; scanDir = 1; }
    display->drawFastHLine(0, scanY, w, SSD1306_INVERSE);

    // Footer
    display->drawLine(0, FOOTER_LINE_Y, w - 1, FOOTER_LINE_Y, SSD1306_WHITE);
    display->setCursor(0, FOOTER_TEXT_Y);
    
    if (isIsolated) {
        display->print("!ISO!");
    } else {
        unsigned long page = (millis() / 3000) % 3;
        if (page == 0) { 
            int p = (int)map((long)axisEnergy, 0, 255, 0, 100);
            if (p < 100) display->print(" "); 
            if (p < 10)  display->print(" ");
            display->print(p); display->print("%");
        } 
        else if (page == 1 && hasSensor) { 
            display->print((int)sht31.readTemperature()); display->print("C");
        } 
        else if (page == 2 && hasSensor) { 
            display->print((int)sht31.readHumidity()); display->print("%");
        }
        else { display->print("---"); }
    }

    // Chaos Pass
    if (chaosMode || confusionLevel > (genes.confusionThreshold / 255.0)) {
        if (random(0,10) > (8 - (confusionLevel*4))) { 
             int y = random(0, h);
             display->drawFastHLine(0, y, w, SSD1306_INVERSE);
        }
    }

    // Bio Takeover
    if (bioMode && bioIntensity > 0) renderBioHazards();

    display->display();
  }

private:
  Adafruit_SSD1306* display;
  VerticalGraph* graphA = nullptr;
  VerticalGraph* graphB = nullptr;
  Overlay* overlays = nullptr;
  uint8_t overlayCount = 0;

  void renderCrashVisuals() {
      if ((millis() / 50) % 2 == 0) display->invertDisplay(true);
      else display->invertDisplay(false);

      int w = display->width();
      int h = display->height();
      for(int i=0; i<5; i++) {
          display->drawLine(random(0, w), random(0, h), random(0, w), random(0, h), SSD1306_WHITE);
      }
      display->fillRect(random(0, w), random(0, h), random(2, 10), random(2, 10), SSD1306_INVERSE);
      
      display->setCursor(2, h/2 - 4);
      display->setTextColor(SSD1306_INVERSE);
      display->print("ERR_");
      display->print(random(100, 999));
  }

  void renderSatelliteVisuals() {
      display->invertDisplay(false);
      int w = display->width();
      int h = display->height();
      
      static int scanX = 0;
      scanX = (millis() / 20) % w; 
      display->drawFastVLine(scanX, 0, h, SSD1306_WHITE);
      
      if (random(0, 10) > 6) {
          display->drawPixel(random(0, w), random(0, h), SSD1306_WHITE);
      }
      if (scanX > 0 && scanX < w) {
          display->drawLine(scanX, random(0, h), scanX-4, random(0, h), SSD1306_WHITE);
      }
      if ((millis() / 200) % 2 == 0) {
          display->fillRect(0, 0, w, 9, SSD1306_BLACK); 
          display->setCursor(2, 1);
          display->print("SAT_LK");
      }
  }

  void renderBioHazards() {
      int w = display->width();
      int h = display->height();
      int numVines = map(bioIntensity, 0, 255, 1, 8);
      for(int i=0; i<numVines; i++) {
          int x = random(0, w);
          int y = h - 1;
          int targetY = h - map(bioIntensity, 0, 255, 5, h);
          if(targetY < 0) targetY = 0;
          while(y > targetY) {
              display->drawPixel(x, y, SSD1306_INVERSE);
              y -= random(1, 3);
              x += random(-2, 3);
              if(x < 0) x = w - 1; if(x >= w) x = 0;
          }
      }
  }

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
    for (int y = yMin; y < yMax; y++) {
      int bufferOffset = (yMax - 1) - y;
      int16_t idx = graph->head - bufferOffset;
      if (idx < 0) idx += VISUAL_HEIGHT; 
      while(idx >= VISUAL_HEIGHT) idx -= VISUAL_HEIGHT;
      int8_t val = graph->values[idx];
      applyOverlays(val, y);

      if (val != -128) {
        int jitter = 0;
        if (axisDrift > 40) jitter = random(-axisDrift / 40, axisDrift / 40 + 1);
        int moodBias = map(axisMood, 0, 255, -2, 2);
        int x = xCenter + val + jitter + moodBias;
        int dynamicSpan = map(axisEnergy, 0, 255, 4, 7);
        int minX = xCenter - dynamicSpan;
        int maxX = xCenter + dynamicSpan;

        if (x < minX) x = minX;
        if (x > maxX) x = maxX;
        display->drawPixel(x, y, SSD1306_WHITE);
        if (currentGraphWidth > 1 && (x + 1) <= maxX) display->drawPixel(x + 1, y, SSD1306_WHITE);
      }
    }
  }
};
OLEDRenderer renderer(&display);

// ======================
// 🧬 DATA SOURCE ROUTER
// ======================
int8_t readSource(uint8_t srcId) {
    switch(srcId) {
        case 1: { // Sensor (Humidity)
            float idle = sin(millis()/1500.0) * 4.0;
            float sensorSpike = 0;
            if (hasSensor) {
                float diff = sht31.readHumidity() - humBaseline;
                if (diff > 2.0) sensorSpike = diff * 4.0; 
                if (sensorSpike > 8) sensorSpike = 8;
                if (sensorSpike < 0) sensorSpike = 0;
                humBaseline = (humBaseline * 0.99) + (sht31.readHumidity() * 0.01);
            }
            if (isBioEvent) sensorSpike += 8; 
            return (int8_t)(idle + sensorSpike);
        }
        case 2: { // Radio Noise
            int8_t noise = map(alienPacketRate, 0, 5, 0, 8); 
            if (isBroadcasting) noise += 4;
            int jitter = random(0, 3); 
            if (rawPacketCount > 0) { 
                noise += rawPacketCount;
                rawPacketCount = 0;
            }
            return random(0, noise + jitter);
        }
        case 3: // Energy
            return (int8_t)map((long)axisEnergy, 0, 255, -16, 16);
        case 4: // Drift
            return (int8_t)map((long)axisDrift, 0, 255, -16, 16);
        case 5: // Mood
            return (int8_t)map((long)axisMood, 0, 255, -16, 16);
        case 6: // Tempo
            return (int8_t)map((long)axisTempo, 0, 255, -16, 16);
        default:
            return 0;
    }
}

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

    outMsg.header.version = SWARM_PROTO_VERSION;
    outMsg.header.msgType = type;
    outMsg.header.senderId = swarmId;
    outMsg.header.bootToken = bootToken;
    outMsg.header.sequenceId = ++mySequenceCounter;
    outMsg.header.groupId = 0;
    outMsg.header.context = (uint8_t)(confusionLevel * 255);

    if (type == MSG_STATE && !msgOverride) {
        float humVal = hasSensor ? sht31.readHumidity() : swarmAvgVal;
        outMsg.data.state.humidity_x10 = (uint16_t)(humVal * 10);
        outMsg.data.state.traffic = (uint16_t)alienPacketRate;
        outMsg.data.state.energy = (uint8_t)axisEnergy;
        outMsg.data.state.mood = (uint8_t)axisMood;
        outMsg.data.state.cohesion = (uint8_t)axisCohesion;
        outMsg.data.state.tempo = (uint8_t)axisTempo;
        outMsg.data.state.drift = (uint8_t)axisDrift;
    } 
    else if (type == MSG_ID_CLAIM && !msgOverride) {
        outMsg.data.id.targetId = candidateId;
    }

    size_t payloadSize = 0;
    switch(type) {
        case MSG_STATE: payloadSize = sizeof(MsgState); break;
        case MSG_EVENT: payloadSize = sizeof(MsgEvent); break;
        case MSG_ID_CLAIM: payloadSize = sizeof(MsgId); break;
        case MSG_ANNOUNCE: payloadSize = sizeof(MsgAnnounce); break;
        case MSG_ID_CONFLICT: payloadSize = sizeof(MsgId); break; 
        case MSG_PARAM: payloadSize = sizeof(MsgParam); break;
        default: payloadSize = 0; break;
    }

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

    if (type == MSG_EVENT || type == MSG_ID_CONFLICT || type == MSG_PARAM) {
        broadcastDuration = 800;
    } else {
        broadcastDuration = 150;
    }

    // FIXED: Changed array access [] to function call ()
    analogWrite(getSafePin(genes.pinSync), (int)currentBrightness);
}

// ======================
// 🧬 PARAMETER HANDLER
// ======================
void setParameter(uint8_t paramId, uint8_t value) {
    switch(paramId) {
        // Graph Sources (20-29)
        case 20: genes.graphSourceA = constrain(value, 1, 6); break;
        case 21: genes.graphSourceB = constrain(value, 1, 6); break;
        
        // LED Assignments (30-39)
        case 30: genes.pinHeartbeat = constrain(value, 0, 4); break;
        case 31: genes.pinBio = constrain(value, 0, 4); break;
        case 32: genes.pinSync = constrain(value, 0, 4); break;
        case 33: genes.pinData = constrain(value, 0, 4); break;
        
        // Visual Tuning (40-49)
        case 40: genes.masterBrightness = value; currentBrightness = value; break;
        case 41: genes.graphWidthMin = constrain(value, 1, 4); break;
        case 42: genes.graphWidthMax = constrain(value, 1, 4); break;
        
        // Behavior Tuning (50-59)
        case 50: genes.driftSensitivity = value; break;
        case 51: genes.energyDecay = value; break;
        case 52: genes.confusionThreshold = value; break;
        
        default: 
            Serial.printf("⚠️ Unknown param: %d\n", paramId);
            return;
    }
    
    Serial.printf("🧬 SET Param %d = %d\n", paramId, value);
    
    settingsChanged = true;
    lastSettingsChange = millis();
}

// ======================
// 👂 BLE PARSER
// ======================
void parseSwarmPacket(const uint8_t* rawData, int len, int rssi) {
    advertsThisWindow++;
    rssiAccum += rssi;
    rssiSamples++;
    lastHeardMs = millis();

    SwarmMessage in;
    if (len > sizeof(SwarmMessage)) len = sizeof(SwarmMessage);
    memcpy(&in, rawData, len);

    if (in.header.version != SWARM_PROTO_VERSION) return;
    if (in.header.senderId == swarmId && in.header.bootToken == bootToken) return;
    if (in.header.senderId >= 256) return; 

    uint16_t last = lastSeenSeqId[in.header.senderId];
    uint16_t current = in.header.sequenceId;
    
    if (last != 0 && current > last + 1) {
        int missed = current - last - 1;
        confusionLevel += (missed * 0.05); 
        if (confusionLevel > 1.0) confusionLevel = 1.0;
        Serial.printf("⚠️ Loss: %d | Conf: %.2f\n", missed, confusionLevel);
    } else {
        confusionLevel -= 0.05; 
        if (confusionLevel < 0.0) confusionLevel = 0.0;
    }
    lastSeenSeqId[in.header.senderId] = current;
    lastHeardMs = millis();
    
    if (in.header.msgType == MSG_STATE) {
        Serial.printf("[RX] #%d STATE | E:%d M:%d\n", in.header.senderId, in.data.state.energy, in.data.state.mood);
        
        float influence = 0.02;
        axisEnergy = (axisEnergy * (1.0-influence)) + ((float)in.data.state.energy * influence);
        axisMood   = (axisMood   * (1.0-influence)) + ((float)in.data.state.mood * influence);
        axisTempo  = (axisTempo  * (1.0-influence)) + ((float)in.data.state.tempo * influence);

        renderer.pushValue(&graphDataB, 12);
        
        // FIXED: Changed array access [] to function call ()
        analogWrite(getSafePin(genes.pinData), (int)currentBrightness);
        lastDataRx = millis(); 
    }
    
    else if (in.header.msgType == MSG_EVENT) {
        uint8_t evId = in.data.event.eventId;
        Serial.printf("[RX] #%d EVENT | ID:%d Intensity:%d\n", in.header.senderId, evId, in.data.event.intensity);

        if (evId == 5) {
             isBioEvent = true;
             bioEventEnd = millis() + in.data.event.durationMs;
             renderer.bioIntensity = 255; 
             renderer.bioMode = true;     
             renderer.pushValue(&graphDataA, 127); 
        }
        else if (evId == 1) {
            triggerCrash(in.data.event.intensity);
        }
        else if (evId == 2) {
             triggerSatellite();
             axisDrift -= 50.0f;
             if(axisDrift < 0) axisDrift = 0;
             Serial.printf("🛰️ SAT SYNC: Healed to %.1f\n", axisDrift);
        }
    }
    
    else if (in.header.msgType == MSG_SYNC) {
        Serial.printf("[RX] #%d SYNC\n", in.header.senderId);
        lastBlink1 = millis(); 
        renderer.pushValue(&graphDataB, 20); 
    }
    
    else if (in.header.msgType == MSG_PARAM) {
        uint16_t target = in.data.param.targetId;
        
        // Check: Is it for EVERYONE (0) or specifically ME (swarmId)?
        if (target == 0 || target == swarmId) {
            Serial.printf("[RX] #%d PARAM | Target:%d | %d=%d\n", 
                in.header.senderId, target, in.data.param.paramId, in.data.param.value);
                
            setParameter(in.data.param.paramId, in.data.param.value);
            
            // Optional: Visual confirmation that *I* accepted the command
            // FIXED: Changed array access [] to function call () for LED_PINS access or just use getSafePin
            analogWrite(LED_PINS[genes.pinData], 255); 
            delay(50);
            analogWrite(LED_PINS[genes.pinData], 0);
        } else {
            Serial.printf("[RX] Ignored Param for Node #%d (I am #%d)\n", target, swarmId);
        }
    }
    // FIXED: Removed extra closing brace here that was closing the function early

    if (in.header.msgType == MSG_ID_CLAIM) {
        if (nodeState == ID_ASSIGNED && in.data.id.targetId == swarmId) {
             SwarmMessage conflict;
             conflict.header = in.header;
             conflict.header.msgType = MSG_ID_CONFLICT;
             conflict.header.senderId = swarmId;
             conflict.data.id.targetId = swarmId;
             triggerBroadcast(MSG_ID_CONFLICT, &conflict);
             activeOverlays[0].type = OVERLAY_JITTER;
             activeOverlays[0].ttl = 10;
        }
    }
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        bool isSwarm = false;
        if (advertisedDevice.haveManufacturerData()) {
            String strData = advertisedDevice.getManufacturerData();
            if (strData.length() >= 2 + sizeof(SwarmHeader)) {
                const uint8_t *data = (const uint8_t*)strData.c_str();
                uint16_t mfgID = data[0] | (data[1] << 8);
                if (mfgID == HIVE_MANUFACTURER_ID) {
                    isSwarm = true;
                    parseSwarmPacket(&data[2], strData.length() - 2, advertisedDevice.getRSSI());
                }
            }
        }
        if (!isSwarm) {
            alienPacketRate++;
            axisDrift += 0.05f * (genes.driftSensitivity / 100.0f);
            if (axisDrift > 255) axisDrift = 255;
        }
    }
};

// ======================
// 🔄 LOGIC TICKS
// ======================
void fastTick() {
    renderer.chaosMode = (confusionLevel > (genes.confusionThreshold / 255.0)) || (axisMood < 50 && random(0,100) > 95);
    
    if (isBioEvent) {
       renderer.bioMode = true;
       long timeLeft = bioEventEnd - millis();
       if (timeLeft > 0) {
           renderer.bioIntensity = map(timeLeft, 0, 5000, 50, (int)currentBrightness);
           float pulse = (exp(sin(millis()/500.0*PI)) - 0.36787944)*108.0;
           // FIXED: Changed array access [] to function call ()
           analogWrite(getSafePin(genes.pinBio), (int)pulse);
       } else isBioEvent = false;
    } else {
        renderer.bioMode = false;
        // FIXED: Changed array access [] to function call ()
        if (!isBroadcasting) analogWrite(getSafePin(genes.pinBio), 0);
    }

    // Heartbeat Breath
    float tempoSpeed = map(axisTempo, 0, 255, 3000, 500);
    float rawBreath = (exp(sin(millis()/tempoSpeed*PI)) - 0.36787944);
    float breath = (rawBreath / 2.35) * currentBrightness; 
    // FIXED: Changed array access [] to function call () (This one was actually already correct in source but good to check)
    analogWrite(getSafePin(genes.pinHeartbeat), (int)breath);
    
    // Sync Blink Logic
    if (!isBroadcasting) {
        // FIXED: Changed array access [] to function call ()
        if (millis() - lastBlink1 < 100) analogWrite(getSafePin(genes.pinSync), (int)currentBrightness);
        else analogWrite(getSafePin(genes.pinSync), 0);
    }

    // Data Blink Logic
    // FIXED: Changed array access [] to function call ()
    if (millis() - lastDataRx > 50) analogWrite(getSafePin(genes.pinData), 0);

    for(int i=0; i<2; i++) if(activeOverlays[i].ttl > 0) activeOverlays[i].ttl--;
    renderer.render();
}

void mediumTick() {
    renderer.pushValue(&graphDataA, readSource(genes.graphSourceA)); 
    renderer.pushValue(&graphDataB, readSource(genes.graphSourceB)); 
    
    if (hasSensor) {
        float h = sht31.readHumidity();
        if ((h - humBaseline) > 4.0) {
             isBioEvent = true; 
             bioEventEnd = millis() + 5000;
             SwarmMessage bioMsg;
             bioMsg.header.msgType = MSG_EVENT;
             bioMsg.data.event.eventId = 5; 
             bioMsg.data.event.intensity = 200; 
             bioMsg.data.event.durationMs = 1000;
             triggerBroadcast(MSG_EVENT, &bioMsg);
        }
    }
}

void slowTick() {
  bool isolated = (millis() - lastHeardMs) > 4000;
  isIsolated = isolated;

  // Energy Logic (with genetic scaling)
  axisEnergy += (swarmEnergy * 255.0f - axisEnergy) * 0.01f;
  float decayFactor = (genes.energyDecay / 100.0f);
  axisEnergy -= isolated ? (2.0f * decayFactor) : (0.5f * decayFactor);
  
  if (axisDrift > 220) axisEnergy -= 20.0f;
  axisEnergy = constrain(axisEnergy, 0.0f, 255.0f);

  // Brightness Mapping
  float energyFactor = map(axisEnergy, 0, 255, 10, (int)genes.masterBrightness) / 255.0;
  currentBrightness = genes.masterBrightness * energyFactor;

  // Dynamic Tempo
  float targetTempo;
  if (axisDrift > 180) {
      targetTempo = random(40, 220);
  } else {
      targetTempo = map(axisEnergy, 0, 255, 40, 240);
  }
  axisTempo += (targetTempo - axisTempo) * 0.15f;
  axisTempo += sin(millis() * 0.00025f) * 4.0f;
  axisTempo = constrain(axisTempo, 0.0f, 255.0f);

  // Cohesion Logic
  axisCohesion += (swarmTraffic * 255.0f - axisCohesion) * 0.04f;
  if (isolated) axisCohesion -= 0.8f;
  axisCohesion = constrain(axisCohesion, 0.0f, 255.0f);

  // Drift Logic (with genetic scaling)
  float driftFactor = (genes.driftSensitivity / 100.0f);
  axisDrift += isolated ? (1.2f * driftFactor) : 0.0f;
  axisDrift += (0.01f * driftFactor);

  if (axisEnergy < 80 && !isolated) {
      axisDrift -= 1.5f; 
  }

  if (axisCohesion > 200) axisDrift += (0.2f * driftFactor);
  axisDrift += confusionLevel * 1.5f;
  axisDrift = constrain(axisDrift, 0.0f, 255.0f);

  // Mood Logic
  int volatility = 8;
  if (axisEnergy > 200) volatility = 16;
  if (axisDrift > 150) volatility = 24;
  axisMood += random(-volatility, volatility + 1);
  axisMood += sin(millis() * 0.0002) * 5.0;
  axisMood = constrain(axisMood, 0.0f, 255.0f);

  // Events
  if (axisDrift > 180 && random(5000) < axisDrift) {
      triggerCrash(100 + axisDrift / 2);
      
      SwarmMessage crashMsg;
      crashMsg.header.msgType = MSG_EVENT;
      crashMsg.data.event.eventId = 1; 
      crashMsg.data.event.intensity = 100 + axisDrift / 2;
      crashMsg.data.event.durationMs = 3000;
      triggerBroadcast(MSG_EVENT, &crashMsg);
  }
  
  if (axisEnergy < 150 && random(1500) < 4) {
      triggerSatellite();
      
      SwarmMessage satMsg;
      satMsg.header.msgType = MSG_EVENT;
      satMsg.data.event.eventId = 2; 
      satMsg.data.event.intensity = 255;
      satMsg.data.event.durationMs = 5000;
      triggerBroadcast(MSG_EVENT, &satMsg);
  }

  currentGraphWidth = map(axisCohesion, 0, 255, genes.graphWidthMin, genes.graphWidthMax);
  currentGraphWidth = constrain(currentGraphWidth, genes.graphWidthMin, genes.graphWidthMax);

  // In loop() (slowTick or standalone):
  if (settingsChanged && (millis() - lastSettingsChange > 3000)) {
    Serial.println("💾 Committing settings to Flash...");
    prefs.begin("genes", false);
    prefs.putBytes("params", &genes, sizeof(genes));
    prefs.end();
    settingsChanged = false;
  }
}

void runBootSequence() {
  if (!hasOLED) return;
  display.clearDisplay(); display.display(); delay(200);

  display.setCursor(0, 10); display.println("BOOT.."); display.display(); 
  // FIXED: Changed array access [] to function call ()
  analogWrite(getSafePin(0), (int)currentBrightness); delay(150); analogWrite(getSafePin(0), 0);

  display.setCursor(0, 30); display.println("MEM OK"); display.display(); 
  // FIXED: Changed array access [] to function call ()
  analogWrite(getSafePin(1), (int)currentBrightness); delay(150); analogWrite(getSafePin(1), 0);

  display.setCursor(0, 50); display.println("RADIO"); display.display(); 
  // FIXED: Changed array access [] to function call ()
  analogWrite(getSafePin(2), (int)currentBrightness); delay(150); analogWrite(getSafePin(2), 0);

  display.setCursor(0, 70); display.println("HIVE.."); display.display(); 
  // FIXED: Changed array access [] to function call ()
  analogWrite(getSafePin(3), (int)currentBrightness); delay(150); analogWrite(getSafePin(3), 0);
  analogWrite(getSafePin(4), (int)currentBrightness); delay(150); analogWrite(getSafePin(4), 0); 

  delay(400);
  display.clearDisplay(); display.display();
}

struct CrashState {
    bool active = false;
    unsigned long startTime = 0;
    uint8_t intensity = 0;       
};
CrashState crash;

struct SatelliteState {
  bool active = false;
  unsigned long startTime = 0;
  uint8_t step = 0;
  unsigned long progress = 0;
};
SatelliteState satellite;

void triggerCrash(uint8_t intensity) {
    crash.active = true;
    crash.startTime = millis();
    crash.intensity = intensity;
    renderer.crashMode = true; 
    Serial.printf("💥 Crash: intensity=%d\n", intensity);
}

void handleCrash() {
    if (!crash.active) return;
    unsigned long elapsed = millis() - crash.startTime;

    if (elapsed < crash.intensity * 20) {
        for (int i = 0; i < 5; i++) {
            // FIXED: Changed array access [] to function call ()
            analogWrite(getSafePin(i), random(0, 2) ? (int)currentBrightness : 0);
        }
    } else {
        crash.active = false;
        renderer.crashMode = false; 
        // FIXED: Changed array access [] to function call ()
        for (int i = 0; i < 5; i++) analogWrite(getSafePin(i), 0);
        
        axisEnergy = 0; 
        currentBrightness = genes.masterBrightness * 0.05; 
        runBootSequence();
        axisDrift = 0; 
        confusionLevel = 0;
    }
}

void triggerSatellite() {
  if (satellite.active) return; 
  satellite.active = true;
  satellite.startTime = millis();
  satellite.step = 0;
  satellite.progress = 0;
}

void handleSatellite() {
  if (!satellite.active) return;
  unsigned long now = millis();
  satellite.progress = now - satellite.startTime;

  const unsigned long fadeIn  = 3500;
  const unsigned long hold    = 4000; 
  const unsigned long fadeOut = 3500;
  const unsigned long total   = fadeIn + hold + fadeOut;

  int brightness = 0;
  
  if (satellite.progress < total) {
      if (satellite.progress < fadeIn) {
          float t = (float)satellite.progress / (float)fadeIn;
          int val = (int)(t * currentBrightness);
          brightness = (val * val) / max(1, (int)currentBrightness); 
      }
      else if (satellite.progress < fadeIn + hold) {
          brightness = (int)currentBrightness;
      }
      else {
          float t = (float)(satellite.progress - (fadeIn + hold)) / (float)fadeOut;
          t = 1.0 - t; 
          int val = (int)(t * currentBrightness);
          brightness = (val * val) / max(1, (int)currentBrightness); 
      }
      
      // FIXED: Changed array access [] to function call ()
      analogWrite(getSafePin(genes.pinBio), brightness);

      if (satellite.progress > fadeIn && satellite.progress < (fadeIn + hold)) {
          if (!renderer.satelliteMode) {
              renderer.satelliteMode = true; 
              // FIXED: Changed array access [] to function call ()
              analogWrite(getSafePin(genes.pinData), (int)currentBrightness); 
          }
      } else {
          if (renderer.satelliteMode) {
              renderer.satelliteMode = false; 
              // FIXED: Changed array access [] to function call ()
              analogWrite(getSafePin(genes.pinData), 0); 
          }
      }
  } else {
    satellite.active = false;
    renderer.satelliteMode = false; 
    // FIXED: Changed array access [] to function call ()
    analogWrite(getSafePin(genes.pinBio), 0);
    analogWrite(getSafePin(genes.pinData), 0);
  }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    pinMode(LED_PIN_1, OUTPUT); pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT); pinMode(LED_PIN_4, OUTPUT);
    pinMode(LED_PIN_5, OUTPUT); pinMode(BOOT_BUTTON, INPUT_PULLUP);

    bootToken = esp_random();

    // Load genetic parameters
    prefs.begin("genes", false);
    size_t loadedSize = prefs.getBytes("params", &genes, sizeof(genes));
    if (loadedSize == sizeof(genes)) {
        Serial.println("🧬 Loaded genetic parameters from flash");
    } else {
        Serial.println("🧬 Using default genetic parameters");
    }
    prefs.end();

    renderer.begin(); 
    hasOLED = true; 
    renderer.setGraphs(&graphDataA, &graphDataB);
    renderer.setOverlays(activeOverlays, 2);

    hasSensor = sht31.begin(0x44);
    if (hasSensor) humBaseline = sht31.readHumidity(); 

    runBootSequence(); 

    BLEDevice::init("");
    pAdvertising = BLEDevice::getAdvertising();
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); 
    pBLEScan->setInterval(40);
    pBLEScan->setWindow(40);   
    pBLEScan->start(0, nullptr); 

    prefs.begin("swarm", false);
    swarmId = prefs.getUShort("swarmId", 0);
    
    if (swarmId != 0) {
        nodeState = ID_ASSIGNED;
        Serial.printf("♻️ Restored Memory: I am Node %d\n", swarmId);
    } else {
        swarmId = (uint16_t)random(1, 254);
        prefs.putUShort("swarmId", swarmId);
        nodeState = ID_ASSIGNED;
        Serial.printf("🆕 Identity: Node %d\n", swarmId);
    }
    
    // FIXED: Changed array access [] to function call ()
    analogWrite(getSafePin(genes.pinSync), (int)currentBrightness); 
    delay(100); 
    analogWrite(getSafePin(genes.pinSync), 0);
}

void updateSwarmField() {
  swarmTraffic = constrain(advertsThisWindow / 3.0f, 0.0f, 1.0f);
  if (rssiSamples > 0) {
    float avgRssi = rssiAccum / (float)rssiSamples;
    swarmEnergy = constrain(map(avgRssi, -95, -50, 0, 100) / 100.0f, 0.0f, 1.0f);
  } else {
      swarmEnergy *= 0.95; 
  }
  confusionLevel += (swarmTraffic * (1.0f - swarmEnergy) - confusionLevel) * 0.15f;
  confusionLevel = constrain(confusionLevel, 0.0f, 1.0f);

  advertsThisWindow = 0;
  rssiAccum = 0;
  rssiSamples = 0;
}

void loop() {
    unsigned long now = millis();

   if (isBroadcasting) {
        if (now - broadcastStartTime > broadcastDuration) { 
            isBroadcasting = false;
            pAdvertising->stop();
            pBLEScan->clearResults();
            pBLEScan->start(0, nullptr); 
            // FIXED: Changed array access [] to function call ()
            analogWrite(getSafePin(genes.pinSync), 0); 
        }
    }

    if (alienPacketRate > 0) {
        rawPacketCount += alienPacketRate; 
        alienPacketRate = 0; 
    }

    static unsigned long lastFast=0, lastMed=0, lastSlow=0;
    if (now - lastFast >= 50)  { lastFast = now; fastTick(); }
    if (now - lastMed  >= 80)  { lastMed  = now; mediumTick(); }
    if (now - lastSlow >= 400) { lastSlow = now; slowTick(); }

    handleCrash();       
    handleSatellite();   

    static unsigned long lastBroadcastTime = 0;
    long broadcastInterval = map(axisTempo, 0, 255, 4000, 250);

    if (now - lastBroadcastTime > broadcastInterval) {
        triggerBroadcast(MSG_STATE);
        lastBroadcastTime = now;
        Serial.printf("[TX] E:%d D:%d T:%d -> Int:%dms\n", (int)axisEnergy, (int)axisDrift, (int)axisTempo, (int)broadcastInterval);
        renderer.pushValue(&graphDataB, 24); 
    }

    // Serial Commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c=='c') { 
            Serial.println("CMD: CRASH");
            triggerCrash(200); 
            SwarmMessage msg; msg.header.msgType = MSG_EVENT; msg.data.event.eventId = 1; msg.data.event.intensity = 200; msg.data.event.durationMs = 3000;
            triggerBroadcast(MSG_EVENT, &msg);
        }
        if (c=='s') { 
            Serial.println("CMD: SATELLITE");
            triggerSatellite(); 
            SwarmMessage msg; msg.header.msgType = MSG_EVENT; msg.data.event.eventId = 2; msg.data.event.intensity = 255; msg.data.event.durationMs = 5000;
            triggerBroadcast(MSG_EVENT, &msg);
        }
        if (c=='b') {
             Serial.println("CMD: BIO");
             isBioEvent = true; bioEventEnd = millis() + 5000;
             SwarmMessage msg; msg.header.msgType = MSG_EVENT; msg.data.event.eventId = 5; msg.data.event.intensity = 200; msg.data.event.durationMs = 5000;
             triggerBroadcast(MSG_EVENT, &msg);
        }
        if (c=='r') { 
            runBootSequence();
        }
        if (c=='?') {
            Serial.println("\n--- STATUS ---");
            Serial.printf("ID: %d | Role: %s\n", swarmId, nodeState==ID_ASSIGNED?"Active":"Search");
            Serial.printf("Energy: %.1f | Mood: %.1f | Drift: %.1f\n", axisEnergy, axisMood, axisDrift);
            Serial.printf("Field: Traffic=%.2f Energy=%.2f\n", swarmTraffic, swarmEnergy);
            Serial.println("\n--- GENETICS ---");
            Serial.printf("GraphA=%d GraphB=%d | Bright=%d\n", genes.graphSourceA, genes.graphSourceB, genes.masterBrightness);
            Serial.printf("Pins: HB=%d Bio=%d Sync=%d Data=%d\n", genes.pinHeartbeat, genes.pinBio, genes.pinSync, genes.pinData);
        }
        if (c == 'p') {
            Serial.println("Enter: TargetID,ParamID,Value (e.g. 101,40,255 or 0,40,255 for all)");
            while (!Serial.available()) delay(10);
            
            String input = Serial.readStringUntil('\n');
            
            // Parse 3 values: Target, Param, Value
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);

            if (firstComma > 0 && secondComma > 0) {
                uint16_t targetId = input.substring(0, firstComma).toInt();
                uint8_t paramId = input.substring(firstComma + 1, secondComma).toInt();
                uint8_t value = input.substring(secondComma + 1).toInt();

                // If I am the target (or if it's a broadcast), apply it locally first
                if (targetId == 0 || targetId == swarmId) {
                    setParameter(paramId, value);
                }

                // Broadcast to the swarm
                SwarmMessage msg;
                msg.header.msgType = MSG_PARAM;
                msg.data.param.targetId = targetId; // Set the target
                msg.data.param.paramId = paramId;
                msg.data.param.value = value;
                triggerBroadcast(MSG_PARAM, &msg);
                
                Serial.printf("📡 SENT Param: Target=%d ID=%d Val=%d\n", targetId, paramId, value);
            } else {
                Serial.println("❌ Invalid format. Use: Target,Param,Value");
            }
        }
    }

    static uint32_t lastFieldUpdate=0;
    if (millis() - lastFieldUpdate > 1000) {
        updateSwarmField();
        lastFieldUpdate = millis();
    }
// FIXED: Moved closing brace to here, so updateSwarmField logic is INSIDE loop()
}
