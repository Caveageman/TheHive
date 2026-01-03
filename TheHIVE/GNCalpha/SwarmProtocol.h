#ifndef SWARM_PROTO_H
#define SWARM_PROTO_H

#include <Arduino.h>

// Change this if you change the struct logic to ignore old nodes
#define SWARM_PROTO_VERSION 1

// The "Broadcast" address (send to everyone)
static uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 1. MESSAGE TYPES
enum SwarmMsgType : uint8_t {
  MSG_ANNOUNCE = 1,   // "I am here" (Presence + Position)
  MSG_ID_CLAIM = 2,   // "Can I have this ID?" (Consensus)
  MSG_ID_CONFLICT = 3,// "No, that ID is taken!"
  MSG_STATE    = 4,   // "Here is my sensor data"
  MSG_EVENT    = 5,    // "Something happened!" (Crash, Satellite, etc.)
  MSG_SYNC     = 6   //Led sync
};

// 2. FIXED HEADER (Every message has this)
struct SwarmHeader {
  uint8_t  version;     // Safety check
  uint8_t  msgType;     // What follows?
  uint16_t senderId;    // Who sent this? (0 if requesting ID)
  uint32_t bootToken;   // Unique session token (for ID conflict resolution)
};

// 3. PAYLOADS (The actual data)

// MSG_ANNOUNCE
struct MsgAnnounce {
  uint8_t posX;         // 0-255
  uint8_t posY;         // 0-255
  uint32_t capabilities;// Bitmask (Has OLED? Has Sensor?)
};

// MSG_ID_CLAIM / MSG_ID_CONFLICT
struct MsgId {
  uint16_t targetId;    // The ID being discussed
};

// MSG_STATE (Continuous data)
struct MsgState {
  float temp;           // Temperature
  float humidity;       // Humidity
  uint8_t energy;       // Swarm Energy Level (0-255)
  uint8_t traffic;      // Local traffic density
};

// MSG_EVENT (Discrete triggers)
struct MsgEvent {
  uint8_t eventId;      // 1=Crash, 2=Satellite, 3=Glitch
  uint8_t intensity;    // How hard to glitch (0-255)
  uint16_t durationMs;  // How long it lasts
};

// 4. THE MASTER WRAPPER
// This union allows us to send any message type as a single packet
typedef struct struct_message {
  SwarmHeader header;
  union {
    MsgAnnounce announce;
    MsgId       id;
    MsgState    state;
    MsgEvent    event;
  } data;
} SwarmMessage;

#endif