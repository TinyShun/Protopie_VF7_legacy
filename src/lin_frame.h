#pragma once
#include <stdint.h>
#include <Arduino.h>

#define LIN_FRAME_SIZE 10
#define SYN_FIELD 0x55
#define Left_Stalk_ID 0x61
// #define MFS_L 0x24
// #define MFS_R 0x25
// #define UNPRESS     0b00
// #define SHORT_PRESS 0x01
// #define LONG_PRESS  0x02
// #define UP          0x01
// #define UP_HOLD     0x02
// #define DOWN        0x03
// #define DOWN_HOLD   0x04
// #define MFS_R_SCROLL 3
// #define MFS_R_SCROLL_BUTTON 4
struct MfsSignal {
  uint8_t bytePos;
  uint8_t shiftRight;
  String messName;
};

extern const MfsSignal leftSignals[], rightSignals[];
extern const size_t LEFT_COUNT, RIGHT_COUNT;

typedef union {
  struct {
    uint8_t id;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;
    uint8_t checkSum;
  } frame;
  uint8_t array[LIN_FRAME_SIZE];
} uLIN_MSG;

size_t writeBreak();
void sendRequest(uint8_t ident);
uint8_t calculateLinChecksum(const uLIN_MSG& msg, bool enhanced);
bool linChecksumValid(const uLIN_MSG& msg, bool enhanced);
void handleLinMess(const uLIN_MSG& msg);
void printLinMess(const uLIN_MSG& msg);
void printToProtopie(String message, String value="");
bool isShortPress(uint8_t button, size_t i, uint8_t mfsPos);
bool isLongPress(uint8_t button, size_t i, uint8_t mfsPos);
// #pragma once
// #include <stdint.h>
// #include <Arduino.h>

// #define LIN_FRAME_SIZE 10
// #define SYN_FIELD 0x55
// #define MFS_L 0x24
// #define MFS_R 0x25
// #define UNPRESS     0b00
// #define SHORT_PRESS 0x01
// #define LONG_PRESS  0x02
// #define UP          0x01
// #define UP_HOLD     0x02
// #define DOWN        0x03
// #define DOWN_HOLD   0x04
// #define MFS_R_SCROLL 3
// #define MFS_R_SCROLL_BUTTON 4
// struct MfsSignal {
//   uint8_t bytePos;
//   uint8_t shiftRight;
//   String messName;
// };

// extern const MfsSignal leftSignals[], rightSignals[];
// extern const size_t LEFT_COUNT, RIGHT_COUNT;

// typedef union {
//   struct {
//     uint8_t id;
//     uint8_t data0;
//     uint8_t data1;
//     uint8_t data2;
//     uint8_t data3;
//     uint8_t data4;
//     uint8_t data5;
//     uint8_t data6;
//     uint8_t data7;
//     uint8_t checkSum;
//   } frame;
//   uint8_t array[LIN_FRAME_SIZE];
// } uLIN_MSG;

// size_t writeBreak();
// void sendRequest(uint8_t ident);
// uint8_t calculateLinChecksum(const uLIN_MSG& msg, bool enhanced);
// bool linChecksumValid(const uLIN_MSG& msg, bool enhanced);
// void handleLinMess(const uLIN_MSG& msg);
// void printLinMess(const uLIN_MSG& msg);
// void printToProtopie(String message, String value="");
// bool isShortPress(uint8_t button, size_t i, uint8_t mfsPos);
// bool isLongPress(uint8_t button, size_t i, uint8_t mfsPos);