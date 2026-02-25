#include <can_defs.h>

extern volatile uint32_t gCanTxCount;


CanTxTask txTasks[] = {

  //IDB_Status  ID:0x20D  Cycle:20ms
  {
    {0x20D, false, CANFDMessage::CAN_DATA, 0, 8,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    20, 0, true, true
  },
  //BCM_Clamp_Stat  ID:0x112  Cycle:100ms
  {
    {0x112, false, CANFDMessage::CAN_DATA, 0, 8,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    100, 0, true, true
  },
  //VCU_HV_Status  ID:0x269  Cycle:40ms
  {
    {0x269, false, CANFDMessage::CAN_DATA, 0, 8,
     {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    40, 0, false, false
  },
  //XGW_NM  ID:0x502  Cycle:640ms
  {
    {0x502, false, CANFDMessage::CAN_DATA, 0, 8,
     {0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    640, 0, true, true
  },
  //VCU_HV_DrvSys_status  ID:0x0D9  Cycle:40ms
  {
    {0x0D9, false, CANFDMessage::CAN_DATA, 0, 8,
     {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    40, 0, true, true
  }
};

constexpr size_t TX_TASK_COUNT = sizeof(txTasks) / sizeof(txTasks[0]);

namespace {
constexpr uint8_t kVcuHvStatusAliveMax = 14; // 0..14
constexpr uint8_t kVcuVehicleHvStatusValue = 1; // HV On
static uint8_t gVcuHvStatusAlive = 0;

void packVcuHvStatus(CANFDMessage &msg, uint8_t aliveCounter, uint8_t hvStatusValue) {
  // ALV: bits 8-12 -> byte1 bits0-4
  msg.data[1] = (msg.data[1] & 0xE0) | (aliveCounter & 0x1F);

  // VehicleHVStatus: bits 14-16 -> byte1 bits6-7, byte2 bit0
  msg.data[1] &= ~0xC0;
  msg.data[2] &= ~0x01;

  const uint8_t hv = hvStatusValue & 0x07;
  if (hv & 0x01) msg.data[1] |= 0x40; // bit14
  if (hv & 0x02) msg.data[1] |= 0x80; // bit15
  if (hv & 0x04) msg.data[2] |= 0x01; // bit16

  // Checksum: bits 0-7 -> byte0, CRC over bytes 1..7
  uint8_t buf[7];
  for (uint8_t i = 0; i < 7; i++) {
    buf[i] = msg.data[i + 1];
  }
  msg.data[0] = getCRC8(buf, 7);
}
} // namespace

void processTxTasks(ACAN2517FD &acan) {
  uint32_t now = millis();
  gCanTxCount++;
  
  for (size_t i = 0; i < TX_TASK_COUNT; i++) {
    CanTxTask &task = txTasks[i];

    if ((int32_t)(now - task.next_send_ms) >= 0) {
      if (i == TX_VCU_HV_STATUS) {
        packVcuHvStatus(task.canMess, gVcuHvStatusAlive, kVcuVehicleHvStatusValue);
      }
      // Try to send (non-blocking)
      if (acan.tryToSend(task.canMess)) {
        task.next_send_ms = now + task.period_ms;
        if (i == TX_VCU_HV_STATUS) {
          if (gVcuHvStatusAlive >= kVcuHvStatusAliveMax) {
            gVcuHvStatusAlive = 0;
          } else {
            gVcuHvStatusAlive++;
          }
          continue;
        }
        if (task.useAliveCounter) {
          task.canMess.data[1] = (task.canMess.data[1] + 1) % 16;
        }
        if (task.useChecksum) {
          uint8_t dlc = task.canMess.len - 1;
          uint8_t buf[64];
          for (uint8_t i = 0; i < dlc; i++) {
            buf[i] = task.canMess.data[i+1];
          }
          task.canMess.data[0] = getCRC8(buf, dlc);
        }
      }
      // else: TX FIFO full → retry next loop
    }
  }
}

uint8_t getCRC8(uint8_t* buf, uint16_t len) {
  uint8_t idx;
  uint8_t crc = UINT8_MAX; // 0xFF
  /* calculate CRC8 conform to SAE J1850 */
  for (idx = 0 ; idx < len; idx ++ )
  {
    crc = CRC8_J1850_TABLE[crc ^ (buf[idx])];
  }
  /* the calculate CRC value shall be XOR with TUINT8_MAX ! */
  crc ^= UINT8_MAX;
  return crc; 
}
