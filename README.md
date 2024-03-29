# ESP32_DCC_Control_Simple
ESP32_DCC_Control_Simple

Booster - ESP32 connection:
DCC_RAIL1_OUT_PIN  16
DCC_RAIL2_OUT_PIN  17
DCC_RAILEN_OUT_PIN  4
3.3 V
GND

Simple WiFi Server

- 001 - V1.0.01. convert ESP32_DCC_Control source
- 002 - V1.1.01. convert ESP32_DCC_Control source
	// 4+1 bit control
	#define DCC_RAIL1_OUT_PIN  18
	#define DCC_RAIL2_OUT_PIN   5
	#define DCC_RAIL3_OUT_PIN  17
	#define DCC_RAIL4_OUT_PIN  16
	#define DCC_RAILEN_OUT_PIN  4
- 003 - V1.1.02. fixing DCC_RAIL1_OUT_PIN, DCC_RAIL2_OUT_PIN 
