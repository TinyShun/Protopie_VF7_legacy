#include "lin_frame.h"

// static uint8_t mfsLeftCam, mfsLeftFav, mfsLeftLeft, mfsLeftRight, mfsLever, mfsLeverButton;           // MFS Left
// static uint8_t mfsRightVoice, mfsRightMenu, mfsRightLeft, mfsRightRight, mfsScroll, mfsScrollButton;  // MFS Right

static uint8_t Left_Stalk_Up, Left_Stalk_Down, Left_Stalk_Inward, Left_Stalk_Outward;
// const MfsSignal leftSignals[] = {
//   { 0, 0, "mfs_l_lever"},
//   { 0, 4, "mfs_l_lever_button"},
//   { 1, 0, "mfs_l_camera"},
//   { 1, 4, "mfs_l_favourites"},
//   { 2, 0, "mfs_l_left"},
//   { 2, 4, "mfs_l_right"},
// };
// const size_t LEFT_COUNT = sizeof(leftSignals) / sizeof(leftSignals[0]);

// const MfsSignal rightSignals[] = {
//   { 1, 4, "mfs_r_menu"},
//   { 2, 0, "mfs_r_left"},
//   { 2, 4, "mfs_r_right"},
//   { 4, 0, "mfs_r_scroll"},
//   { 4, 3, "mfs_r_scroll_button"},
// };
// const size_t RIGHT_COUNT = sizeof(rightSignals) / sizeof(rightSignals[0]);
// static uint8_t signals[LEFT_COUNT + RIGHT_COUNT] = {UNPRESS};


void printLinMess(const uLIN_MSG& msg) {
  if (msg.frame.id == 0x61 && msg.frame.data4 != 0x00) {
    for (uint8_t i = 0; i < LIN_FRAME_SIZE; i++) {
      Serial.print(msg.array[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
}

uint8_t calculateLinChecksum(const uLIN_MSG& msg, bool enhanced) {
  uint16_t sum = 0;
  int startIndex = enhanced ? 0 : 1; // ID included in enhanced checksum

  for (int i = startIndex; i < 9; ++i) {
      sum += msg.array[i];
  }

  return (uint8_t)(255 - (sum % 256));
}
 
bool linChecksumValid(const uLIN_MSG& msg, bool enhanced) {
  uint8_t expected = calculateLinChecksum(msg, enhanced);
  return (expected == msg.frame.checkSum);
}

size_t writeBreak() {
  Serial2.flush();
  Serial2.begin(19200 >> 1);
  size_t ret = Serial2.write(uint8_t(0x00));
  Serial2.flush();
  // restore normal speed
  Serial2.begin(19200);
  return ret;
}

void sendRequest(uint8_t ident) {
  writeBreak();
  Serial2.write(uint8_t(0x55));
  Serial2.write(ident);
  Serial2.flush();
}


// void handleLinMess(const uLIN_MSG& msg) {
//   static unsigned long leftTimestamp, rightTimestamp;
//   static uint8_t leftTriggered[LEFT_COUNT], rightTriggered[RIGHT_COUNT];
//   uint8_t temp;
//   if (msg.frame.id == MFS_L) {
//     for (size_t i = 0; i < LEFT_COUNT; i++) {
//       temp = (msg.array[leftSignals[i].bytePos + 1] >> leftSignals[i].shiftRight) & 0b0111;
//       unsigned long now = millis();
//       if (leftTriggered[i] != UNPRESS) {
//         if (now - leftTimestamp > 600) {
//           if (temp == UNPRESS) {
//             if (isShortPress(leftTriggered[i], i, MFS_L))
//               printToProtopie(leftSignals[i].messName, String(leftTriggered[i]));
//             printToProtopie(leftSignals[i].messName, String(temp));
//             leftTriggered[i] = UNPRESS;
//           } else {
//             if (isLongPress(temp, i, MFS_L)) {
//               if (!isLongPress(leftTriggered[i], i, MFS_L))
//                 printToProtopie(leftSignals[i].messName, String(temp));
//               leftTriggered[i] = temp;
//             }
//           }
//         }
//       }
//       if (signals[i] != temp) {
//         if (isShortPress(temp, i, MFS_L)) {
//           leftTimestamp = millis();
//           leftTriggered[i] = temp;
//         }
//         signals[i] = temp;
//       }
//     }
//   } else if (msg.frame.id == MFS_R) {
//     for (size_t i = 0; i < RIGHT_COUNT; i++) {
//       temp = (msg.array[rightSignals[i].bytePos + 1] >> rightSignals[i].shiftRight) & 0b0111;
//       unsigned long now = millis();
//       if (rightTriggered[i] != UNPRESS) {
//         if (now - rightTimestamp > 600) {
//           if (temp == UNPRESS) {
//             if (isShortPress(rightTriggered[i], i, MFS_R))
//               printToProtopie(rightSignals[i].messName, String(rightTriggered[i]));
//             printToProtopie(rightSignals[i].messName, String(temp));
//             rightTriggered[i] = UNPRESS;
//           } else {
//             if (i == MFS_R_SCROLL_BUTTON) 
//               temp++;
//             if (isLongPress(temp, i, MFS_R)) {
//               if (!isLongPress(rightTriggered[i], i, MFS_R))
//                 printToProtopie(rightSignals[i].messName, String(temp));
//               rightTriggered[i] = temp;
//             }
//           }
//         }
//       }
//       if (signals[LEFT_COUNT + i] != temp) {
//         if (isShortPress(temp, i, MFS_R)) {
//           // printToProtopie(rightSignals[i].messName, String(temp));
//           rightTimestamp = millis();
//           rightTriggered[i] = temp;
//         }
//         signals[LEFT_COUNT + i] = temp;
//       }
//     }
//   }
// }

// bool isShortPress(uint8_t button, size_t i, uint8_t mfsPos) {
//   if (i == MFS_R_SCROLL && mfsPos == MFS_R) {
//     return button == 1 || button == 2;
//   }
//   return button == SHORT_PRESS || button == UP || button == DOWN;
// }

// bool isLongPress(uint8_t button, size_t i, uint8_t mfsPos) {
//   // if (i == MFS_R_SCROLL_BUTTON && mfsPos == MFS_R) {
//   //   return button == 1;
//   // }
//   return button == LONG_PRESS || button == UP_HOLD || button == DOWN_HOLD;
// }
// #include "lin_frame.h"

// static uint8_t mfsLeftCam, mfsLeftFav, mfsLeftLeft, mfsLeftRight, mfsLever, mfsLeverButton;           // MFS Left
// static uint8_t mfsRightVoice, mfsRightMenu, mfsRightLeft, mfsRightRight, mfsScroll, mfsScrollButton;  // MFS Right

// const MfsSignal leftSignals[] = {
//   { 0, 0, "mfs_l_lever"},
//   { 0, 4, "mfs_l_lever_button"},
//   { 1, 0, "mfs_l_camera"},
//   { 1, 4, "mfs_l_favourites"},
//   { 2, 0, "mfs_l_left"},
//   { 2, 4, "mfs_l_right"},
// };
// const size_t LEFT_COUNT = sizeof(leftSignals) / sizeof(leftSignals[0]);

// const MfsSignal rightSignals[] = {
//   { 1, 4, "mfs_r_menu"},
//   { 2, 0, "mfs_r_left"},
//   { 2, 4, "mfs_r_right"},
//   { 4, 0, "mfs_r_scroll"},
//   { 4, 3, "mfs_r_scroll_button"},
// };
// const size_t RIGHT_COUNT = sizeof(rightSignals) / sizeof(rightSignals[0]);
// static uint8_t signals[LEFT_COUNT + RIGHT_COUNT] = {UNPRESS};

// void printLinMess(const uLIN_MSG& msg) {
//   if (msg.frame.id == 0x61 && msg.frame.data4 != 0x00) {
//     for (uint8_t i = 0; i < LIN_FRAME_SIZE; i++) {
//       Serial.print(msg.array[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.print("\n");
//   }
// }

// uint8_t calculateLinChecksum(const uLIN_MSG& msg, bool enhanced) {
//   uint16_t sum = 0;
//   int startIndex = enhanced ? 0 : 1; // ID included in enhanced checksum

//   for (int i = startIndex; i < 9; ++i) {
//       sum += msg.array[i];
//   }

//   return (uint8_t)(255 - (sum % 256));
// }
 
// bool linChecksumValid(const uLIN_MSG& msg, bool enhanced) {
//   uint8_t expected = calculateLinChecksum(msg, enhanced);
//   return (expected == msg.frame.checkSum);
// }

// size_t writeBreak() {
//   Serial2.flush();
//   Serial2.begin(19200 >> 1);
//   size_t ret = Serial2.write(uint8_t(0x00));
//   Serial2.flush();
//   // restore normal speed
//   Serial2.begin(19200);
//   return ret;
// }

// void sendRequest(uint8_t ident) {
//   writeBreak();
//   Serial2.write(uint8_t(0x55));
//   Serial2.write(ident);
//   Serial2.flush();
// }

// void handleLinMess(const uLIN_MSG& msg) {
//   static unsigned long leftTimestamp, rightTimestamp;
//   static uint8_t leftTriggered[LEFT_COUNT], rightTriggered[RIGHT_COUNT];
//   uint8_t temp;
//   if (msg.frame.id == MFS_L) {
//     for (size_t i = 0; i < LEFT_COUNT; i++) {
//       temp = (msg.array[leftSignals[i].bytePos + 1] >> leftSignals[i].shiftRight) & 0b0111;
//       unsigned long now = millis();
//       if (leftTriggered[i] != UNPRESS) {
//         if (now - leftTimestamp > 600) {
//           if (temp == UNPRESS) {
//             if (isShortPress(leftTriggered[i], i, MFS_L))
//               printToProtopie(leftSignals[i].messName, String(leftTriggered[i]));
//             printToProtopie(leftSignals[i].messName, String(temp));
//             leftTriggered[i] = UNPRESS;
//           } else {
//             if (isLongPress(temp, i, MFS_L)) {
//               if (!isLongPress(leftTriggered[i], i, MFS_L))
//                 printToProtopie(leftSignals[i].messName, String(temp));
//               leftTriggered[i] = temp;
//             }
//           }
//         }
//       }
//       if (signals[i] != temp) {
//         if (isShortPress(temp, i, MFS_L)) {
//           leftTimestamp = millis();
//           leftTriggered[i] = temp;
//         }
//         signals[i] = temp;
//       }
//     }
//   } else if (msg.frame.id == MFS_R) {
//     for (size_t i = 0; i < RIGHT_COUNT; i++) {
//       temp = (msg.array[rightSignals[i].bytePos + 1] >> rightSignals[i].shiftRight) & 0b0111;
//       unsigned long now = millis();
//       if (rightTriggered[i] != UNPRESS) {
//         if (now - rightTimestamp > 600) {
//           if (temp == UNPRESS) {
//             if (isShortPress(rightTriggered[i], i, MFS_R))
//               printToProtopie(rightSignals[i].messName, String(rightTriggered[i]));
//             printToProtopie(rightSignals[i].messName, String(temp));
//             rightTriggered[i] = UNPRESS;
//           } else {
//             if (i == MFS_R_SCROLL_BUTTON) 
//               temp++;
//             if (isLongPress(temp, i, MFS_R)) {
//               if (!isLongPress(rightTriggered[i], i, MFS_R))
//                 printToProtopie(rightSignals[i].messName, String(temp));
//               rightTriggered[i] = temp;
//             }
//           }
//         }
//       }
//       if (signals[LEFT_COUNT + i] != temp) {
//         if (isShortPress(temp, i, MFS_R)) {
//           // printToProtopie(rightSignals[i].messName, String(temp));
//           rightTimestamp = millis();
//           rightTriggered[i] = temp;
//         }
//         signals[LEFT_COUNT + i] = temp;
//       }
//     }
//   }
// }

// bool isShortPress(uint8_t button, size_t i, uint8_t mfsPos) {
//   if (i == MFS_R_SCROLL && mfsPos == MFS_R) {
//     return button == 1 || button == 2;
//   }
//   return button == SHORT_PRESS || button == UP || button == DOWN;
// }

// bool isLongPress(uint8_t button, size_t i, uint8_t mfsPos) {
//   // if (i == MFS_R_SCROLL_BUTTON && mfsPos == MFS_R) {
//   //   return button == 1;
//   // }
//   return button == LONG_PRESS || button == UP_HOLD || button == DOWN_HOLD;
// }