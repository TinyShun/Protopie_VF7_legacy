#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

#include <ACAN2517FD.h>
#include <SPI.h>
#include <string.h>
#include <lin_frame.h>
#include <can_defs.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── BLE UUID ─────────────────────────────────────────────────────────
#define SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_CMD  "abcd1234-5678-90ab-cdef-1234567890ab"

// BLE GLOBAL - Server mode
static BLEServer         *bleServer       = nullptr;
static BLECharacteristic *bleCharCmd      = nullptr;
static bool               bleConnected    = false;
static bool               bleWasConnected = false;

// ── CAN ──────────────────────────────────────────────────────────────
// Map pin for MCP2517
static const byte MCP2517_SCK  = 18;
static const byte MCP2517_MOSI = 23;
static const byte MCP2517_MISO = 19;
static const byte MCP2517_CS   = 5;
static const byte MCP2517_INT  = 21;
// CANFDMessage g_canMess;

// ACAN2517FD Driver object
ACAN2517FD acan(MCP2517_CS, SPI, MCP2517_INT);

CANFDMessage      g_rxCanMsg;
volatile uint32_t gCanRxCount = 0;
volatile uint32_t gCanTxCount = 0;

unsigned long gLastCanPrintMs = 0;
#define CAN_PRINT_INTERVAL_MS  1000

// ── IGN STATE ────────────────────────────────────────────────────────
enum IgnState {
    IGN_OFF = 0,
    IGN_ACC = 1,
    IGN_ON  = 2
};

IgnState gIgnState = IGN_OFF;

// ── BLE KEY COMMANDS ─────────────────────────────────────────────────
enum KeyCommand {
    KEY_NONE   = 0,
    KEY_LOCK   = 1,
    KEY_UNLOCK = 2,
    KEY_TRUNK  = 3
};

KeyCommand gLastKeyCommand    = KEY_NONE;
bool       gKeyCommandProcessed = true;

// ── PIN DEFINES ──────────────────────────────────────────────────────
#define PIN_DOOR_SWITCH  14
// Physical key pins
#define PIN_KEY_LOCK     25  
#define PIN_KEY_UNLOCK   33  
#define KEY_DEBOUNCE_MS  50
#define UART1_RX_PIN     26
#define UART1_TX_PIN     27

bool gDoorOpened = false;

// ── PEDALS / GAS ADC ─────────────────────────────────────────────────
#define PIN_BRAKE_ADC         34
#define PIN_GAS_ADC           35
#define BRAKE_RAW_MIN       2250  // 2000
#define BRAKE_RAW_MAX       3650  // 3900
#define GAS_RAW_MIN          440  // 190
#define GAS_RAW_MAX         3650  // 3900
#define PEDAL_PRINT_THRESHOLD  1
#define ADC_MAX             4095.0f
#define ADC_INTERVAL_MS       50

uint8_t brakePercent     = 0;
uint8_t gasPercent       = 0;
uint8_t prevBrakePercent = 0xFF;
uint8_t prevGasPercent   = 0xFF;
uint8_t gSpeed           = 0;

// ── GEAR ─────────────────────────────────────────────────────────────
uint8_t gGearMapped = 0x00;  // 0:P, 1:R, 2:N, 3:D

// ── MOVING AVERAGE ───────────────────────────────────────────────────
#define MA_WINDOW_SIZE  20

uint16_t brakeBuf[MA_WINDOW_SIZE] = {0};
uint16_t gasBuf[MA_WINDOW_SIZE]   = {0};
uint32_t brakeSum   = 0;
uint32_t gasSum     = 0;
uint8_t  brakeIndex = 0, gasIndex = 0;
bool     brakeFilled = false, gasFilled = false;

// ── WHEEL TURN ───────────────────────────────────────────────────────
uint8_t prevWheelTurn = 0xFF;

// ── LIN ──────────────────────────────────────────────────────────────
#define LIN_TARGET_ID  0x61

unsigned long lastAdcRead    = 0;
//const int PIN_VR = 4;
unsigned long vrPressedStart = 0;
bool          vrButtonPressed = false;
int           vrState         = 1;
uint8_t       vrPressType     = 0;

enum TurnState : uint8_t {
    TURN_NONE    = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_OUTWARD,
    TURN_INWARD
};

unsigned long gLastReq      = 0;
uLIN_MSG      gLinMsg;
TurnState     gCurrentState = TURN_NONE;
TurnState     gLastState    = TURN_NONE;

// ─────────────────────────────────────────────────────────────────────

void printToProtopie(String message, String value)
{
    if (value.length() == 0) {
        Serial.println(message);
    } else {
        Serial.println(message + "||" + value);
    }
}

// Moving average filter
uint16_t movingAverage(uint16_t *buf, uint32_t &sum,
                       uint8_t &index, bool &filled,
                       uint16_t newVal)
{
    sum -= buf[index];
    buf[index] = newVal;
    sum += newVal;

    uint16_t avg = sum / (filled ? MA_WINDOW_SIZE : (index + 1));

    index++;
    if (index >= MA_WINDOW_SIZE) {
        index = 0;
        filled = true;
    }

    return avg;
}

// Handle ADC Pedals
void handleAdcPedals()
{
    unsigned long now = millis();
    if (now - lastAdcRead < ADC_INTERVAL_MS)
        return;

    lastAdcRead = now;

    int rawBrake = analogRead(PIN_BRAKE_ADC);
    int rawGas   = analogRead(PIN_GAS_ADC);

    if (rawBrake < BRAKE_RAW_MIN) rawBrake = BRAKE_RAW_MIN;
    if (rawBrake > BRAKE_RAW_MAX) rawBrake = BRAKE_RAW_MAX;
    if (rawGas   < GAS_RAW_MIN)   rawGas   = GAS_RAW_MIN;
    if (rawGas   > GAS_RAW_MAX)   rawGas   = GAS_RAW_MAX;

    uint16_t brakeFilt = movingAverage(brakeBuf, brakeSum, brakeIndex, brakeFilled, rawBrake);
    uint16_t gasFilt   = movingAverage(gasBuf,   gasSum,   gasIndex,   gasFilled,   rawGas);

    // Normalize filtered values
    float brakeNorm = (float)(BRAKE_RAW_MAX - brakeFilt) / (float)(BRAKE_RAW_MAX - BRAKE_RAW_MIN);
    float gasNorm   = (float)(GAS_RAW_MAX   - gasFilt)   / (float)(GAS_RAW_MAX   - GAS_RAW_MIN);

    // Convert to percentage
    brakePercent = (uint8_t)(brakeNorm * 100.0f + 0.5f);
    gasPercent   = (uint8_t)(gasNorm   * 100.0f + 0.5f);

    // Print only if change >= threshold
    if (abs((int)brakePercent - (int)prevBrakePercent) >= PEDAL_PRINT_THRESHOLD) {
        //Serial.print("BRAKE_PERCENT||");
        //Serial.println(brakePercent);
        prevBrakePercent = brakePercent;
    }

    if (abs((int)gasPercent - (int)prevGasPercent) >= PEDAL_PRINT_THRESHOLD) {
        //Serial.print("GAS_PERCENT||");
        //Serial.println(gasPercent);
        prevGasPercent = gasPercent;
    }
}

// Handle brake pressed
bool brakePressed()
{
    return brakePercent > 20;
}

// Handle Door
void handleDoor()
{
    bool doorNow = (digitalRead(PIN_DOOR_SWITCH) == LOW);

    if (doorNow != gDoorOpened) {
        gDoorOpened = doorNow;
        Serial.print("DOOR||");
        Serial.println(gDoorOpened ? "OPENED" : "CLOSED");
    }
}

// Handle IGN state → CAN
void setIgnStateToCan(uint8_t ign)
{
    /* BCM_Clamp_Stat – ID 0x112 – BYTE 4 */
    txTasks[TX_BCM_CLAMP_STAT].canMess.data[4] = ign;
}

// Handle Physical Key (LOCK pin 25, UNLOCK pin 33)
// Dùng chung gLastKeyCommand → handleKeyAction() xử lý giống BLE
void handlePhysicalKey()
{
    static bool          lockState         = false;
    static bool          lastLockReading   = false;
    static unsigned long lockDebounceMs    = 0;

    static bool          unlockState       = false;
    static bool          lastUnlockReading = false;
    static unsigned long unlockDebounceMs  = 0;

    unsigned long now = millis();

    // --- LOCK key (pin 25, HIGH = pressed) ---
    bool currentLockReading = (digitalRead(PIN_KEY_LOCK) == HIGH);
    if (currentLockReading != lastLockReading) {
        lockDebounceMs = now;
    }
    if ((now - lockDebounceMs) >= KEY_DEBOUNCE_MS) {
        if (currentLockReading != lockState) {
            lockState = currentLockReading;
            if (lockState) {
                if (gKeyCommandProcessed) {
                    gLastKeyCommand      = KEY_LOCK;
                    gKeyCommandProcessed = false;
                    Serial.println("KEY||LOCK");
                    Serial.flush();
                }
            }
        }
    }
    lastLockReading = currentLockReading;

    // --- UNLOCK key (pin 33, HIGH = pressed) ---
    bool currentUnlockReading = (digitalRead(PIN_KEY_UNLOCK) == HIGH);
    if (currentUnlockReading != lastUnlockReading) {
        unlockDebounceMs = now;
    }
    if ((now - unlockDebounceMs) >= KEY_DEBOUNCE_MS) {
        if (currentUnlockReading != unlockState) {
            unlockState = currentUnlockReading;
            if (unlockState) {
                if (gKeyCommandProcessed) {
                    gLastKeyCommand      = KEY_UNLOCK;
                    gKeyCommandProcessed = false;
                    Serial.println("KEY||UNLOCK");
                    Serial.flush();
                }
            }
        }
    }
    lastUnlockReading = currentUnlockReading;
}

// Handle IGN state from BLE Key Commands
void handleKeyAction()
{
    // Process BLE key commands
    if (!gKeyCommandProcessed) {
        gKeyCommandProcessed = true;

        switch (gLastKeyCommand) {
            case KEY_UNLOCK:
                // UNLOCK: OFF -> ACC
                if (gIgnState == IGN_OFF) {
                    gIgnState = IGN_ACC;
                    setIgnStateToCan(IGN_ACC);
                    Serial.println("IGN||ACC");
                    Serial.flush();
                }
                break;

            case KEY_LOCK:
                // LOCK: ACC/ON -> OFF
                if (gIgnState == IGN_ACC || gIgnState == IGN_ON) {
                    setIgnStateToCan(IGN_OFF);
                    gIgnState = IGN_OFF;

                    // Auto shift to Gear P when locking
                    txTasks[TX_VCU_HV_DRVSYS_STATUS].canMess.data[4] = 0x00;  // Gear P
                    gGearMapped = 0x00;
                    Serial.println("GEAR||P");
                    delay(20);
                    Serial.println("IGN||OFF");
                    Serial.flush();
                }
                break;

            default:
                break;
        }
        gLastKeyCommand = KEY_NONE;
    }

    // Brake press: ACC -> ON
    if (gIgnState == IGN_ACC && brakePressed()) {
        gIgnState = IGN_ON;
        setIgnStateToCan(IGN_ON);
        Serial.println("IGN||ON");
        Serial.flush();
    }
}

// Handle CAN RX
void handleCanRx()
{
    while (acan.receive(g_rxCanMsg)) {
        gCanRxCount++;

        switch (g_rxCanMsg.id) {

            case 0x108: {  // Gear selector
                if (brakePressed() && !gDoorOpened) {
                    static uint8_t prevGearMapped = 0xFF;  // lưu gear đã gửi trước đó

                    uint8_t gearRaw    = g_rxCanMsg.data[2];
                    uint8_t gearMapped = 0xFF;

                    switch (gearRaw) {
                        case 0x00:
                            Serial.println("GEAR||P");
                            gearMapped = 00;
                            break;
                        case 0x20:
                            Serial.println("GEAR||R");
                            gearMapped = 01;
                            break;
                        case 0x40:
                            Serial.println("GEAR||N");
                            gearMapped = 02;
                            break;
                        case 0x60:
                            Serial.println("GEAR||D");
                            gearMapped = 03;
                            break;
                        default:
                            break;
                    }

                    if (gearMapped != 0xFF && gearMapped != prevGearMapped) {
                        txTasks[TX_VCU_HV_DRVSYS_STATUS].canMess.data[4] = gearMapped;
                        prevGearMapped = gearMapped;
                        gGearMapped    = gearMapped;
                    }
                }
                break;
            }

            case 0x17E: {  // Steering angle sensor
                if (g_rxCanMsg.len < 7) break;

                uint16_t rawAngle  = ((uint16_t)g_rxCanMsg.data[5] << 8) | g_rxCanMsg.data[6];
                uint8_t  wheelTurn = 100 - ((uint32_t)rawAngle * 100 / 65535);

                if (wheelTurn != prevWheelTurn) {
                    Serial.print("WHEEL_TURN||");
                    Serial.println(wheelTurn);
                    prevWheelTurn = wheelTurn;
                }
                break;
            }

            default:
                break;
        }
    }
}

// Handle CAN debug
void candebug()
{
    unsigned long now = millis();
    if (now - gLastCanPrintMs >= CAN_PRINT_INTERVAL_MS) {
        gLastCanPrintMs = now;
        // Serial.print("CAN RX Count: ");
        // Serial.print(gCanRxCount);
        // Serial.print(" | CAN TX Count: ");
        // Serial.println(gCanTxCount);
    }
}

// ── LIN ──────────────────────────────────────────────────────────────

// Slave response: ID + 8 data + checksum
bool tryReceiveLin(uLIN_MSG &msg)
{
    static uint8_t idx = 0;

    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        if (idx == 0) {
            msg.frame.id = b;
        } else if (idx >= 1 && idx <= 8) {
            msg.array[idx] = b;
        } else if (idx == 9) {
            msg.frame.checkSum = b;
            idx = 0;
            return true;
        }

        idx++;
    }

    return false;
}

// Byte cần decode = msg.array[3]
TurnState decodeTurnState(uint8_t val)
{
    if (val & 0x04 || val & 0x08) return TURN_RIGHT;
    if (val & 0x02 || val & 0x01) return TURN_LEFT;
    if (val & 0x10)                return TURN_OUTWARD;
    if (val & 0x20)                return TURN_INWARD;
    return TURN_NONE;
}

// Handle turn state print
void printTurnState(TurnState state)
{
    switch (state) {
        case TURN_LEFT:    Serial.println("STATE: TURN_LEFT");  break;
        case TURN_RIGHT:   Serial.println("STATE: TURN_RIGHT"); break;
        case TURN_OUTWARD: Serial.println("STATE: OUTWARD");    break;
        case TURN_INWARD:  Serial.println("STATE: INWARD");     break;
        default:           Serial.println("STATE: NONE");       break;
    }
}

// Handle turn state machine
void handleTurnStateMachine(const uLIN_MSG& msg)
{
    // Byte sau 0x7C, trước ALV
    uint8_t val = msg.array[3];

    gCurrentState = decodeTurnState(val);

    if (gCurrentState != gLastState) {
        printTurnState(gCurrentState);
        gLastState = gCurrentState;
    }
}

// ── BLE SERVER CALLBACKS ─────────────────────────────────────────────

// Theo dõi phone kết nối / ngắt kết nối
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        bleConnected = true;
        Serial.println("[BLE] ✓ PHONE CONNECTED");
        Serial.println("[BLE] Waiting for command...");
    }
    void onDisconnect(BLEServer *pServer) override {
        bleConnected    = false;
        bleWasConnected = true;  // trigger restart advertising ở loop()
        Serial.println("[BLE] ✗ PHONE DISCONNECTED");
        Serial.println("[BLE] Restarting advertising...");
    }
};

// Nhận lệnh WRITE từ phone
// Giao thức:
//   Phone App gửi "10" → OFF (tắt xe) → KEY_LOCK
//   Phone App gửi "11" → ON  (bật xe) → KEY_UNLOCK
//   Key vật lý gửi "1" → LOCK, "2" → UNLOCK, "3" → TRUNK
class CmdWriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        std::string val = pChar->getValue();
        if (val.empty()) {
            Serial.println("[BLE] WRITE received but empty!");
            return;
        }

        // Parse chuỗi ASCII → số nguyên: "10" → 10, "11" → 11
        int cmd = atoi(val.c_str());

        switch (cmd) {
            // ── Phone App ───────────────────────────────────
            case 10:  // APP: OFF → tắt nguồn
                gLastKeyCommand      = KEY_LOCK;
                gKeyCommandProcessed = false;
                // Serial.println("[BLE] CMD: APP OFF (10) → KEY_LOCK");
                Serial.println("KEY||LOCK");
                delay(20);
                break;
            case 11:  // APP: ON → bật ACC
                gLastKeyCommand      = KEY_UNLOCK;
                gKeyCommandProcessed = false;
                // Serial.println("[BLE] CMD: APP ON (11) → KEY_UNLOCK");
                Serial.println("KEY||UNLOCK");
                delay(20);
                break;

            // ── Key vật lý (giữ tương thích) ────────────────
            case 1:  // KEY: LOCK → tắt nguồn
                gLastKeyCommand      = KEY_LOCK;
                gKeyCommandProcessed = false;
                // Serial.println("[BLE] CMD: KEY LOCK (1) → KEY_LOCK");
                Serial.println("KEY||LOCK");
                break;
            case 2:  // KEY: UNLOCK → bật ACC
                gLastKeyCommand      = KEY_UNLOCK;
                gKeyCommandProcessed = false;
                // Serial.println("[BLE] CMD: KEY UNLOCK (2) → KEY_UNLOCK");
                Serial.println("KEY||UNLOCK");
                break;
            case 3:  // KEY: TRUNK → mở cốp
                gLastKeyCommand      = KEY_TRUNK;
                gKeyCommandProcessed = false;
                // Serial.println("[BLE] CMD: KEY TRUNK (3)");
                Serial.println("KEY||TRUNK");
                break;

            default:
                Serial.print("[BLE] UNKNOWN CMD: ");
                Serial.println(cmd);
                break;
        }
        Serial.flush();
    }
};

// Khởi tạo BLE Server
void initBleServer()
{
    BLEDevice::init("VF7_ECU");  // Tên hiển thị trên phone khi scan
    Serial.println("[BLE] Device name: VF7_ECU");

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    // Tạo Service
    BLEService *service = bleServer->createService(SERVICE_UUID);
    Serial.print("[BLE] Service UUID: ");
    Serial.println(SERVICE_UUID);

    // Tạo Characteristic với WRITE + WRITE_NO_RESPONSE
    bleCharCmd = service->createCharacteristic(
        CHAR_UUID_CMD,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    bleCharCmd->setCallbacks(new CmdWriteCallbacks());
    bleCharCmd->addDescriptor(new BLE2902());
    Serial.print("[BLE] Char UUID: ");
    Serial.println(CHAR_UUID_CMD);

    service->start();

    // Bắt đầu quảng bá để phone scan thấy
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] ★ SERVER STARTED - Advertising, waiting for phone...");
}

// Handle UART1 send to Protopie
void handleUart1Debug()
{
    static uint8_t prevGear = 0xFF;
    if (gGearMapped != prevGear) {
        prevGear = gGearMapped;
        Serial1.print("GEAR||");
        Serial1.println(gGearMapped);
        Serial1.flush();
    }
}

// Handle speed from Serial USB
void handleSpeed()
{
    static char    buffer[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buffer[idx] = '\0';

                uint16_t speedValue = atoi(buffer);
                Serial1.print("SPEED||");
                Serial1.println(speedValue);
                gSpeed = (uint8_t)(speedValue & 0xFF);

                // Set speed to IDB_Status (0x20D) - bits 27-40 (14 bits)
                // Bit 27-40: byte 3 (bits 3-7), byte 4 (bits 0-7), byte 5 (bits 0-4)
                // speedValue is 14-bit value (0-16383)
                uint16_t speed14bit = speedValue & 0x3FFF;  // Mask to 14 bits
                // Distribute 14 bits across bytes 3, 4, 5
                txTasks[TX_IDB_STATUS].canMess.data[3] = (txTasks[TX_IDB_STATUS].canMess.data[3] & 0x07) | ((speed14bit & 0x1F) << 3);
                txTasks[TX_IDB_STATUS].canMess.data[4] = (speed14bit >> 5) & 0xFF;
                txTasks[TX_IDB_STATUS].canMess.data[5] = (txTasks[TX_IDB_STATUS].canMess.data[5] & 0xE0) | ((speed14bit >> 13) & 0x1F);
            }
            idx = 0;
        } else if (idx < sizeof(buffer) - 1) {
            buffer[idx++] = c;
        }
    }
}

// ── SETUP ────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    Serial2.begin(19200, SERIAL_8N1, 16, 17);                        // LIN
    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);  // Protopie

    // BLE init - ESP32 làm Server, phone scan và ghi lệnh
    initBleServer();

    // Pin init
    pinMode(PIN_DOOR_SWITCH, INPUT_PULLUP);
    pinMode(PIN_KEY_LOCK,   INPUT);  // Physical key LOCK
    pinMode(PIN_KEY_UNLOCK, INPUT);  // Physical key UNLOCK
    //pinMode(PIN_VR, INPUT_PULLUP);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(PIN_BRAKE_ADC, INPUT);
    pinMode(PIN_GAS_ADC, INPUT);

    Serial.println("ESP32 main + CAN scheduler ready");

    // Begin SPI
    SPI.begin(MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI);

    // Configure ACAN2517FD
    // //--- For version >= 2.1.0
    ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 500UL * 1000UL, DataBitRateFactor::x4);
    // //--- For version < 2.1.0
    // ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz10xPLL, 125 * 1000, ACAN2517FDSettings::DATA_BITRATE_x1);
    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;  // Select loopback mode

    // //--- RAM Usage
    // Serial.print("MCP2517FD RAM Usage: ");
    // Serial.print(settings.ramUsage());
    // Serial.println(" bytes");

    const uint32_t errorCode = acan.begin(settings, [] { acan.isr(); });

    // if (errorCode == 0) {
    //     Serial.print("Bit Rate prescaler: ");
    //     Serial.println(settings.mBitRatePrescaler);
    //     Serial.print("Arbitration Phase segment 1: ");
    //     Serial.println(settings.mArbitrationPhaseSegment1);
    //     Serial.print("Arbitration Phase segment 2: ");
    //     Serial.println(settings.mArbitrationPhaseSegment2);
    //     Serial.print("Arbitration SJW:");
    //     Serial.println(settings.mArbitrationSJW);
    //     Serial.print("Actual Arbitration Bit Rate: ");
    //     Serial.print(settings.actualArbitrationBitRate());
    //     Serial.println(" bit/s");
    //     Serial.print("Exact Arbitration Bit Rate ? ");
    //     Serial.println(settings.exactArbitrationBitRate() ? "yes" : "no");
    //     Serial.print("Arbitration Sample point: ");
    //     Serial.print(settings.arbitrationSamplePointFromBitStart());
    //     Serial.println("%");
    // } else {
    //     Serial.print("Configuration error 0x");
    //     Serial.println(errorCode, HEX);
    // }
}

// ── LOOP ─────────────────────────────────────────────────────────────

void loop()
{
    // BLE SERVER - Restart advertising sau khi phone ngắt kết nối
    if (bleWasConnected && !bleConnected) {
        bleWasConnected = false;
        delay(300);
        BLEDevice::startAdvertising();
        Serial.println("[BLE] ★ Advertising restarted, waiting for phone...");
    }

    handleAdcPedals();
    handlePhysicalKey();
    handleKeyAction();
    handleDoor();
    // Handle speed
    handleSpeed();
    // CAN TX TASKS
    processTxTasks(acan);
    // CAN RX HANDLER
    handleCanRx();
    // CAN DEBUG PRINT
    candebug();
    // UART1 DEBUG
    handleUart1Debug();

    // LIN: gửi request mỗi 100ms
    unsigned long now = millis();
    if (now - gLastReq >= 100) {
        gLastReq = now;
        sendRequest(LIN_TARGET_ID);
    }

    // LIN: nhận response (non-blocking)
    if (tryReceiveLin(gLinMsg)) {
        if (gLinMsg.frame.id == LIN_TARGET_ID) {
            handleTurnStateMachine(gLinMsg);
        }
    }

    // uint8_t count = 0;
    // bool syncDetected = false;

    // while ((count < LIN_FRAME_SIZE + 1) && Serial2.available()) {
    //     uint8_t linByte = Serial2.read();
    //     if (linByte == SYN_FIELD)
    //         syncDetected = true;

    //     if (syncDetected) {
    //         delay(1);
    //         if (count == 1) {
    //             gLinMess.frame.id = linByte;
    //         } else if (count == 10) {
    //             gLinMess.frame.checkSum = linByte;
    //         } else if (count > 1) {
    //             gLinMess.array[count - 1] = linByte;
    //         }
    //         count++;
    //     }
    // }

    // handleLinMess(gLinMess);
    // gReqID = (gReqID == MFS_L) ? MFS_R : MFS_L;

    // VR BUTTON
    // int temp = digitalRead(PIN_VR);

    // if (vrButtonPressed) {
    //     unsigned long t = millis();
    //     if (t - vrPressedStart > 500) {
    //         if (vrPressType != 2) {
    //             vrPressType = 2;
    //             printToProtopie("MFS_R_VR", String(vrPressType));
    //         }
    //     }
    // }

    // if (temp != vrState) {
    //     if (temp == LOW) {
    //         vrButtonPressed = true;
    //         vrPressedStart = millis();
    //     } else {
    //         if (millis() - vrPressedStart < 500) {
    //             vrPressType = 1;
    //             printToProtopie("MFS_R_VR", String(vrPressType));
    //         }
    //         vrButtonPressed = false;
    //         vrPressType = 0;
    //         printToProtopie("MFS_R_VR", String(vrPressType));
    //     }
    //     vrState = temp;
    // }
}
