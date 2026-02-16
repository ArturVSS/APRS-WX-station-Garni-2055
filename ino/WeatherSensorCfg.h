#ifndef WEATHERSENSORCFG_H
#define WEATHERSENSORCFG_H
#include <Arduino.h>

// --- KONFIGURACJA OGÓLNA ---
#define WIND_DATA_FLOATINGPOINT 1 
#define USE_CC1101
#define RADIO_CHIP CC1101
#define RECEIVER_CHIP "CC1101"

// --- DEKODERY ---
#define DECODE_WEATHER1    true  
#define DECODE_WEATHER2    false 
#define DECODE_CH_SOIL     false
#define DECODE_CH_FLOAT    false
#define DECODE_DIS_TEMP    false
#define DECODE_LIGHTNING   false 
#define DECODE_LEAKAGE     false

// --- PINY DLA ESP32 ---
#if defined(ESP32)
    #define PIN_RECEIVER_CS   5   // GPIO 5 (Chip Select)
    #define PIN_RECEIVER_IRQ  4   // GPIO 4 (GDO0 - Przerwanie)
    
    // Te definicje są wymagane przez bibliotekę, nawet jeśli ich nie używamy bezpośrednio
    #define PIN_RECEIVER_GPIO 4   
    #define PIN_RECEIVER_ISR  4
    #define LED_PIN           2   
    #define DEBUG_PORT Serial
    
    // Logi
    #define log_d(...) { Serial.printf("D: "); Serial.printf(__VA_ARGS__); Serial.println(); }
    #define log_e(...) { Serial.printf("E: "); Serial.printf(__VA_ARGS__); Serial.println(); }
    #define log_w(...) { Serial.printf("W: "); Serial.printf(__VA_ARGS__); Serial.println(); }
    #define log_i(...) { Serial.printf("I: "); Serial.printf(__VA_ARGS__); Serial.println(); }
    #define log_v(...) {}
    
// --- PINY DLA ESP8266 (Dla kompatybilności wstecznej) ---
#elif defined(ESP8266)
    #define PIN_RECEIVER_CS   15 
    #define PIN_RECEIVER_IRQ  4   
    #define PIN_RECEIVER_GPIO 4   
    #define PIN_RECEIVER_ISR  4   
    #define LED_PIN           2 
    #define DEBUG_PORT Serial
#endif

#define NUM_SENSORS 1
#define SENSOR_IDS_INC { } 
#define SENSOR_IDS_EXC { } 

#endif // WEATHERSENSORCFG_H