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

// BLE UUID - ESP32 quảng bá, Phone scan và ghi lệnh vào
#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_CMD     "abcd1234-5678-90ab-cdef-1234567890ab"

// BLE GLOBAL - Server mode
static BLEServer         *bleServer       = nullptr;
static BLECharacteristic *bleCharCmd      = nullptr;
static bool               bleConnected    = false;
static bool               bleWasConnected = false;

// Map pin for MCP2517
static const byte MCP2517_SCK  = 18;
static const byte MCP2517_MOSI = 23;
static const byte MCP2517_MISO = 19;
static const byte MCP2517_CS   = 5;
static const byte MCP2517_INT  = 21;

// ACAN2517FD Driver object
ACAN2517FD acan(MCP2517_CS, SPI, MCP2517_INT);

CANFDMessage g_rxCanMsg;

// IGN State
enum IgnState {
    IGN_OFF = 0,
    IGN_ACC = 1,
    IGN_ON  = 2
};

IgnState gIgnState = IGN_OFF;

// BLE Key Commands
enum KeyCommand {
    KEY_NONE   = 0,
    KEY_LOCK   = 1,
    KEY_UNLOCK = 2,
    KEY_TRUNK  = 3
};

KeyCommand gLastKeyCommand    = KEY_NONE;
bool       gKeyCommandProcessed = true;

// Pin defines
#define PIN_DOOR_SWITCH  14
#define PIN_BUTTON_A     25
#define UART1_RX_PIN     26
#define UART1_TX_PIN     27

bool gDoorOpened = false;

// Pedals ADC
#define PIN_BRAKE_ADC        34
#define PIN_GAS_ADC          35
#define BRAKE_RAW_MIN      2250
#define BRAKE_RAW_MAX      3650
#define GAS_RAW_MIN         440
#define GAS_RAW_MAX        3650
#define PEDAL_PRINT_THRESHOLD  1
#define ADC_INTERVAL_MS       50

uint8_t brakePercent     = 0;
uint8_t gasPercent       = 0;
uint8_t prevBrakePercent = 0xFF;
uint8_t prevGasPercent   = 0xFF;

// Moving average filter
#define MA_WINDOW_SIZE  20
uint16_t brakeBuf[MA_WINDOW_SIZE] = {0};
uint16_t gasBuf[MA_WINDOW_SIZE]   = {0};
uint32_t brakeSum = 0;
uint32_t gasSum   = 0;
uint8_t  brakeIndex = 0, gasIndex = 0;
bool     brakeFilled = false, gasFilled = false;

// Gear
uint8_t gGearMapped = 0x00; // 0:P, 1:R, 2:N, 3:D

// Wheel turn
uint8_t prevWheelTurn = 0xFF;

// LIN
#define LIN_TARGET_ID  0x61
unsigned long lastAdcRead = 0;
unsigned long gLastReq    = 0;
uLIN_MSG      gLinMsg;

enum TurnState : uint8_t {
    TURN_NONE    = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_OUTWARD,
    TURN_INWARD
};

TurnState gCurrentState = TURN_NONE;
TurnState gLastState    = TURN_NONE;

// Speed
uint8_t gSpeed = 0;

// ── MOVING AVERAGE ───────────────────────────────────────────────────

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

// ── ADC PEDALS ───────────────────────────────────────────────────────

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

    float brakeNorm = (float)(BRAKE_RAW_MAX - brakeFilt) / (float)(BRAKE_RAW_MAX - BRAKE_RAW_MIN);
    float gasNorm   = (float)(GAS_RAW_MAX   - gasFilt)   / (float)(GAS_RAW_MAX   - GAS_RAW_MIN);

    brakePercent = (uint8_t)(brakeNorm * 100.0f + 0.5f);
    gasPercent   = (uint8_t)(gasNorm   * 100.0f + 0.5f);

    if (abs((int)brakePercent - (int)prevBrakePercent) >= PEDAL_PRINT_THRESHOLD) {
        Serial.print("BRAKE_PERCENT||");
        Serial.println(brakePercent);
        prevBrakePercent = brakePercent;
    }

    if (abs((int)gasPercent - (int)prevGasPercent) >= PEDAL_PRINT_THRESHOLD) {
        Serial.print("GAS_PERCENT||");
        Serial.println(gasPercent);
        prevGasPercent = gasPercent;
    }
}

bool brakePressed()
{
    return brakePercent > 20;
}

// ── DOOR ─────────────────────────────────────────────────────────────

void handleDoor()
{
    bool doorNow = (digitalRead(PIN_DOOR_SWITCH) == LOW);

    if (doorNow != gDoorOpened) {
        gDoorOpened = doorNow;
        Serial.print("DOOR||");
        Serial.println(gDoorOpened ? "OPENED" : "CLOSED");
    }
}

// ── IGN STATE ────────────────────────────────────────────────────────

void setIgnStateToCan(uint8_t ign)
{
    /* BCM_Clamp_Stat – ID 0x112 – BYTE 4 */
    txTasks[TX_BCM_CLAMP_STAT].canMess.data[4] = ign;
}

void handleKeyAction()
{
    if (!gKeyCommandProcessed) {
        gKeyCommandProcessed = true;

        switch (gLastKeyCommand) {
            case KEY_UNLOCK:
                if (gIgnState == IGN_OFF) {
                    gIgnState = IGN_ACC;
                    setIgnStateToCan(IGN_ACC);
                    Serial.println("IGN||ACC");
                    Serial.flush();
                }
                break;

            case KEY_LOCK:
                if (gIgnState == IGN_ACC || gIgnState == IGN_ON) {
                    setIgnStateToCan(IGN_OFF);
                    gIgnState = IGN_OFF;

                    // Auto shift to Gear P when locking
                    txTasks[TX_VCU_HV_DRVSYS_STATUS].canMess.data[4] = 0x00;
                    gGearMapped = 0x00;
                    Serial.println("GEAR||P");
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

// ── CAN RX ───────────────────────────────────────────────────────────

void handleCanRx()
{
    while (acan.receive(g_rxCanMsg)) {
        switch (g_rxCanMsg.id) {

            case 0x108: {  // Gear selector
                if (brakePressed() && !gDoorOpened) {
                    static uint8_t prevGearMapped = 0xFF;

                    uint8_t gearRaw    = g_rxCanMsg.data[2];
                    uint8_t gearMapped = 0xFF;

                    switch (gearRaw) {
                        case 0x00: Serial.println("GEAR||P"); gearMapped = 0; break;
                        case 0x20: Serial.println("GEAR||R"); gearMapped = 1; break;
                        case 0x40: Serial.println("GEAR||N"); gearMapped = 2; break;
                        case 0x60: Serial.println("GEAR||D"); gearMapped = 3; break;
                        default: break;
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

                uint16_t rawAngle = ((uint16_t)g_rxCanMsg.data[5] << 8) | g_rxCanMsg.data[6];
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

// ── LIN ──────────────────────────────────────────────────────────────

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

TurnState decodeTurnState(uint8_t val)
{
    if (val & 0x04 || val & 0x08) return TURN_RIGHT;
    if (val & 0x02 || val & 0x01) return TURN_LEFT;
    if (val & 0x10)                return TURN_OUTWARD;
    if (val & 0x20)                return TURN_INWARD;
    return TURN_NONE;
}

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

void handleTurnStateMachine(const uLIN_MSG& msg)
{
    gCurrentState = decodeTurnState(msg.array[3]);

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
        Serial.println("[BLE] Phone connected");
    }
    void onDisconnect(BLEServer *pServer) override {
        bleConnected    = false;
        bleWasConnected = true;  // trigger restart advertising ở loop()
        Serial.println("[BLE] Phone disconnected");
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
        if (val.empty()) return;

        int cmd = atoi(val.c_str());

        switch (cmd) {
            // Phone App
            case 10:
                gLastKeyCommand      = KEY_LOCK;
                gKeyCommandProcessed = false;
                Serial.println("KEY||LOCK");
                break;
            case 11:
                gLastKeyCommand      = KEY_UNLOCK;
                gKeyCommandProcessed = false;
                Serial.println("KEY||UNLOCK");
                break;

            // Key vật lý
            case 1:
                gLastKeyCommand      = KEY_LOCK;
                gKeyCommandProcessed = false;
                Serial.println("KEY||LOCK");
                break;
            case 2:
                gLastKeyCommand      = KEY_UNLOCK;
                gKeyCommandProcessed = false;
                Serial.println("KEY||UNLOCK");
                break;
            case 3:
                gLastKeyCommand      = KEY_TRUNK;
                gKeyCommandProcessed = false;
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
    BLEDevice::init("VF7_ECU");
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    BLEService *service = bleServer->createService(SERVICE_UUID);

    bleCharCmd = service->createCharacteristic(
        CHAR_UUID_CMD,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    bleCharCmd->setCallbacks(new CmdWriteCallbacks());
    bleCharCmd->addDescriptor(new BLE2902());

    service->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Server started, waiting for phone...");
}

// ── UART1 GEAR NOTIFY ────────────────────────────────────────────────

void handleGearNotify()
{
    static uint8_t prevGear = 0xFF;
    if (gGearMapped != prevGear) {
        prevGear = gGearMapped;
        Serial1.print("GEAR||");
        Serial1.println(gGearMapped);
        Serial1.flush();
    }
}

// ── SPEED ────────────────────────────────────────────────────────────

void handleSpeed()
{
    static char    buffer[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buffer[idx] = '\0';

                uint16_t speedValue  = atoi(buffer);
                uint16_t speed14bit  = speedValue & 0x3FFF;

                Serial1.print("SPEED||");
                Serial1.println(speedValue);

                gSpeed = (uint8_t)(speedValue & 0xFF);

                // Distribute 14-bit speed across bytes 3, 4, 5 of IDB_STATUS (0x20D)
                // bits 27-40: byte3[7:3], byte4[7:0], byte5[0]
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
    Serial2.begin(19200, SERIAL_8N1, 16, 17);                          // LIN
    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);    // Protopie

    initBleServer();

    pinMode(PIN_DOOR_SWITCH, INPUT_PULLUP);
    pinMode(PIN_BUTTON_A, INPUT);
    pinMode(PIN_BRAKE_ADC, INPUT);
    pinMode(PIN_GAS_ADC, INPUT);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    SPI.begin(MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI);

    ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 500UL * 1000UL, DataBitRateFactor::x4);
    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
    acan.begin(settings, [] { acan.isr(); });

    Serial.println("ESP32 ready");
}

// ── LOOP ─────────────────────────────────────────────────────────────

void loop()
{
    // BLE: Restart advertising sau khi phone ngắt kết nối
    if (bleWasConnected && !bleConnected) {
        bleWasConnected = false;
        delay(300);
        BLEDevice::startAdvertising();
        Serial.println("[BLE] Advertising restarted");
    }

    handleAdcPedals();
    handleKeyAction();
    handleDoor();

    if (digitalRead(PIN_BUTTON_A)) {
        Serial.println("Button A");
    }

    handleSpeed();
    processTxTasks(acan);
    handleCanRx();
    handleGearNotify();

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
}
