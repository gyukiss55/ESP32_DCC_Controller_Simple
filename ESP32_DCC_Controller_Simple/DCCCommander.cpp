// DCCCommander.cpp


#define _DCCSimpleWebServer_ 1

#include "Arduino.h"
#include "DCCCommander.h"

#define PARAMNAME_CHANNEL "ch"
#define PARAMNAME_DCCVALUE "dcc"


 // Stop button is attached to PIN 0 (IO0)
#define BTN_STOP_ALARM      0

// 4+1 bit control

#define DCC_RAIL1_OUT_PIN  18
#define DCC_RAIL2_OUT_PIN   5
#define DCC_RAIL3_OUT_PIN  17
#define DCC_RAIL4_OUT_PIN  16

#define DCC_RAILEN_OUT_PIN  4
#define DCC_POCKET_OUT_PIN  2
#define DCC_RAIL_IN_PIN    15

#define DCC_1HALFBITTIME    58
#define DCC_0HALFBITTIME    100
#define MEASURED_TIMES_BUFFER_LEN    256

#define MAX_NR_OF_HEXA_BYTES 6

#define MAX_NR_OF_CHANNEL 4
#define CHANNEL_MASK 0x03

hw_timer_t* timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE webCommandMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;

const uint8_t channelMask = CHANNEL_MASK;

volatile uint8_t isrChannel = 0;
volatile uint16_t isrStatus = 0;        // 0 - preamble ('1'x12), 1,3,5 - leading ('0'x1), 2 - address, 4 - command, 6 - CRC, 8 - close ('1'x1)
volatile uint16_t isrPhase = 0;
volatile uint8_t isrPhaseLimit = 12 * 2;
volatile uint8_t isrMask = 0;
volatile bool isrBit = true;

volatile bool isrFirst = false;
volatile bool isrDisableControl = false;

volatile uint8_t isrPacket[MAX_NR_OF_CHANNEL][7]; // 4 channel x max 1 + 6 byte packet (1. byte packet length (0,3...6))  ( 1 - 2 addr + 1 - 2 - 3 command + 1 CRC)
volatile uint8_t isrPacketRequest[MAX_NR_OF_CHANNEL][7]; // 4 channel x max 1 + 6 byte packet (1. byte packet length (0,3...6))  ( 1 - 2 addr + 1 - 2 - 3 command + 1 CRC)

//volatile uint8_t isrAddr1 = 0x00;
//volatile uint8_t isrComm1 = 0x00;
//volatile uint8_t isrCRC1 = 0;

//volatile uint8_t isrAddr2 = 0xff;
//volatile uint8_t isrComm2 = 0x00;
//volatile uint8_t isrCRC2 = 0xff;

volatile uint32_t isrLoop = 0;
volatile uint32_t lastIsrAt = 0;
volatile int64_t isrBootTime = 0;


void IRAM_ATTR onTimer() {

    portENTER_CRITICAL_ISR(&timerMux);
    isrCounter++;

    isrBootTime = esp_timer_get_time();
    isrPhase++;
    if (isrPhase == isrPhaseLimit) {
        if (!(isrStatus & 1)) {
            isrStatus++;
            isrPhase = 0;
            isrPhaseLimit = 2;
            if ((isrStatus >> 1) <= isrPacket[isrChannel][0])
                isrBit = false;
            else {
                isrBit = true;
                if (isrDisableControl)
                    isrDisableControl = false;
            }
        }
        else  if ((isrStatus >> 1) <= isrPacket[isrChannel][0]) {
            isrStatus++;
            isrPhase = 0;
            isrMask = 0x80;
            isrPhaseLimit = 2 * 8;
        }
        else if (isrDisableControl) {  //read back CV
            isrStatus++;
            isrPhase = 0;
            isrPhaseLimit = 2 * 32;
        }
        else {
            digitalWrite(DCC_RAIL2_OUT_PIN, true);//debug
            
            isrStatus = 0;
            isrPhaseLimit = 2 * 18;
            isrBit = true;

            for (uint8_t i = 0; i < MAX_NR_OF_CHANNEL; ++i) {
                if (isrPacketRequest[i][0] == 1) {// delete channel packet
                    isrPacket[i][0] = 0;
                    isrPacketRequest[i][0] = 0;
                } if (isrPacketRequest[i][0] >= 2 && isrPacketRequest[i][0] <= 5) {
                    uint8_t crc = 0;
                    for (uint8_t j = 0; j <= isrPacketRequest[i][0]; ++j) {
                        isrPacket[i][j] = isrPacketRequest[i][j];
                        if (j > 0)
                            crc = crc ^ isrPacket[i][j];
                    }
                    isrPacket[i][isrPacket[i][0] + 1] = crc;
                    isrPacketRequest[i][0] = 0;
                }
            }

            uint8_t tch = isrChannel;
            for (uint8_t i = 0; i < MAX_NR_OF_CHANNEL; ++i) {
                ++isrChannel;
                isrChannel = isrChannel & channelMask;
                if (isrPacket[isrChannel][0] != 0)
                    break;
            }
            if (tch == isrChannel) {
                if (isrPacket[isrChannel][0] == 0) {
                    isrPacket[isrChannel][0] = 2;
                    isrPacket[isrChannel][1] = 0xff;
                    isrPacket[isrChannel][2] = 0x00;
                    isrPacket[isrChannel][3] = 0xff;
                }
                ++isrChannel;
                isrChannel = isrChannel & channelMask;
                isrPacket[isrChannel][0] = 2;
                isrPacket[isrChannel][1] = 0xff;
                isrPacket[isrChannel][2] = 0x00;
                isrPacket[isrChannel][3] = 0xff;
            }

        }
    }

    if ((!(isrStatus & 1)) && (isrStatus != 0) && ((isrStatus >> 1) <= isrPacket[isrChannel][0] + 1)) {
         if (!(isrPhase & 1)) {
            isrBit = isrPacket[isrChannel][(isrStatus >> 1)] & isrMask;
            isrMask = isrMask >> 1;
         }
    }

    portEXIT_CRITICAL_ISR(&timerMux);

    if (isrStatus == 0){
        digitalWrite(DCC_POCKET_OUT_PIN, false);
    }
    else {
        digitalWrite(DCC_POCKET_OUT_PIN, true);
    }

    if ((!(isrStatus & 1)) && ((isrStatus  >> 1) > isrPacket[isrChannel][0] + 1))
    {
        digitalWrite(DCC_RAILEN_OUT_PIN, true);
    }
    else
    {
        digitalWrite(DCC_RAILEN_OUT_PIN, false);
    }

    isrLoop++;
    if (isrLoop == 120000)
        isrLoop = 0;
    if (isrLoop == 0)
        xSemaphoreGiveFromISR(timerSemaphore, NULL);

    if ((!(isrStatus & 1)) && ((isrStatus >> 1) > isrPacket[isrChannel][0] + 1)) {
        digitalWrite(DCC_RAIL1_OUT_PIN, true);
    }
    else {
        if (isrPhase & 1) {
            digitalWrite(DCC_RAILEN_OUT_PIN, false);
            digitalWrite(DCC_RAIL1_OUT_PIN, false);
            digitalWrite(DCC_RAIL2_OUT_PIN, false);
            digitalWrite(DCC_RAIL3_OUT_PIN, true);
            digitalWrite(DCC_RAIL4_OUT_PIN, true);
            digitalWrite(DCC_RAILEN_OUT_PIN, true);
        }
        else {
            digitalWrite(DCC_RAILEN_OUT_PIN, false);
            digitalWrite(DCC_RAIL3_OUT_PIN, false);
            digitalWrite(DCC_RAIL4_OUT_PIN, false);
            digitalWrite(DCC_RAIL1_OUT_PIN, true);
            digitalWrite(DCC_RAIL2_OUT_PIN, true);
            digitalWrite(DCC_RAILEN_OUT_PIN, true);
        }
    }
    if (isrBit)
        timerAlarmWrite(timer, DCC_1HALFBITTIME, true);
    else
        timerAlarmWrite(timer, DCC_0HALFBITTIME, true);

   digitalWrite(DCC_RAIL2_OUT_PIN, false);//debug

}

static std::string asyncCommand;
static bool changedAsyncCommand = false;

void CallBackAsyncWebServer(const std::string& com)
{
    changedAsyncCommand = true;
    asyncCommand = com;
}

void setupDCCCommander()
{
    // Set BTN_STOP_ALARM to input mode
    pinMode(BTN_STOP_ALARM, INPUT);

    //pinMode (DCC_RAIL_IN_PIN, INPUT);
    //attachInterrupt(DCC_RAIL_IN_PIN, Ext_INT1_ISR, CHANGE);

    for (uint8_t i = 0; i < MAX_NR_OF_CHANNEL;++i) {
        isrPacket[i][0] = 0;
        isrPacketRequest[i][0] = 0;
    }
    isrChannel = 0;
    isrPacket[isrChannel][0] = 2;
    isrPacket[isrChannel][1] = 0x02;
    isrPacket[isrChannel][2] = 0x04;
    isrPacket[isrChannel][3] = isrPacket[isrChannel][1]  ^ isrPacket[isrChannel][2];
    ++isrChannel;
    isrChannel = isrChannel & channelMask;
    isrPacket[isrChannel][0] = 2;
    isrPacket[isrChannel][1] = 0xff;
    isrPacket[isrChannel][2] = 0x00;
    isrPacket[isrChannel][3] = isrPacket[isrChannel][1]  ^ isrPacket[isrChannel][2];
    isrChannel = 0;

    pinMode(DCC_RAIL1_OUT_PIN, OUTPUT);
    digitalWrite(DCC_RAIL1_OUT_PIN, false);
    pinMode(DCC_RAIL2_OUT_PIN, OUTPUT);
    digitalWrite(DCC_RAIL2_OUT_PIN, false);
    pinMode(DCC_RAILEN_OUT_PIN, OUTPUT);
    digitalWrite(DCC_RAILEN_OUT_PIN, false);

    pinMode(DCC_POCKET_OUT_PIN, OUTPUT);
    digitalWrite(DCC_POCKET_OUT_PIN, false);

    //isrCRC1 = isrAddr1 ^ isrComm1;
    //isrCRC2 = isrAddr2 ^ isrComm2;

    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 1000000, true);

    // Start an alarm
    timerAlarmEnable(timer);
    setupDCCWebServer();

}

void setAddrCommand(uint8_t chan, const uint8_t *result, uint8_t result_nr)
{
    if (chan < 4 && result_nr >= 1 && result_nr <= 5) {
        portENTER_CRITICAL(&timerMux);

        isrPacketRequest[chan][0] = result_nr;
        for (uint8_t j = 1; j <= result_nr; ++j) {
            isrPacketRequest[chan][j] = result[j - 1];
        }
        portEXIT_CRITICAL(&timerMux);
    }
}


inline bool isHexa(char ch) {
    if (ch >= '0' && ch <= '9' ||
        ch >= 'A' && ch <= 'F' ||
        ch >= 'a' && ch <= 'f')
        return true;
    return false;
}

inline uint8_t hexaCharToValue(char ch) {
    if (ch >= '0' && ch <= '9')
        return (ch - '0');
    if (ch >= 'A' && ch <= 'F')
        return (ch - 'A' + 10);
    if (ch >= 'a' && ch <= 'f')
        return (ch - 'a' + 10);
    return 0xff;
}

bool StringToArrayOfBytes(const std::string& str, size_t pos, uint8_t* bytes, uint8_t& numberOfBytes)
{
    bool ret = false;
    uint8_t tmp = 0;
    numberOfBytes = 0;
    for (size_t i = pos; i < str.size() && isHexa(str[i]); i += 2) {
        tmp = hexaCharToValue(str[i]);
        bool breakFlag = true;
        if ((i + 1) < str.size() && isHexa(str[i + 1])) {
            tmp = hexaCharToValue(str[i + 1]) + (tmp << 4);
            breakFlag = false;
        }
        bytes[numberOfBytes] = tmp;
        numberOfBytes++;
        ret = true;
        if (breakFlag || numberOfBytes == MAX_NR_OF_HEXA_BYTES)
            break;
    }
    return ret;
}


void loopDCCCommander()
{
    std::string command;
    loopDCCWebServer(command);
    
    if (command[0] == 'e') {
      Serial.println(command.c_str());
        uint8_t result[MAX_NR_OF_HEXA_BYTES];
        uint8_t result_nr = 0;
        uint8_t chan = 0;

        StringToArrayOfBytes(command, command.find("ch=") + 3, result, result_nr);
        chan = result[0];
        StringToArrayOfBytes(command, command.find("dcc=") + 4, result, result_nr);
        Serial.print("result_nr:");
        Serial.print(result_nr);
        Serial.print("result:");
        Serial.print(result[0], HEX);
        Serial.print(",");
        Serial.print(result[1], HEX);
        if (chan  < 4 && result_nr >= 1 && result_nr <= 5) {
            Serial.print("ch:");
            Serial.print(chan, HEX);
            for (uint8_t j = 0; j < result_nr; ++j) {
                Serial.print(",");
                Serial.print(result[j], HEX);
            }
            setAddrCommand(chan, result, result_nr);
        }
        Serial.println(".");
    }
//    if (check) {
//        Serial.println(command.c_str());
//    }
    // If Timer has fired
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
        uint32_t isrCount = 0, isrTime = 0;
        int64_t  bootTime = 0;
        // Read the interrupt count and time
        portENTER_CRITICAL(&timerMux);
        isrCount = isrCounter;
        isrTime = lastIsrAt;
        bootTime = isrBootTime;
        portEXIT_CRITICAL(&timerMux);
        // Print it
        Serial.print("onTimer no. ");
        Serial.print(isrCount);
        Serial.print(" at ");
        Serial.print(isrTime);
        Serial.print(" ms ");
        Serial.print((int32_t)bootTime);
        Serial.println(" us");
    }

}
