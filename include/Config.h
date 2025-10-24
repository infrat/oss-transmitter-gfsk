#ifndef CONFIG_H
#define CONFIG_H

#include "Secrets.h"

// ==================== PIN DEFINITIONS ====================

// TTGO LoRa32 v2.1 / T-Beam pin definitions
#define SS 18   // GPIO18 -- SX1276's CS
#define RST 14  // GPIO14 -- SX1276's RESET
#define DI0 26  // GPIO26 -- SX1276's IRQ(Interrupt Request)
#define DIO1 33 // GPIO33 -- SX1276's DIO1

// ==================== RADIO CONFIGURATION ====================

#define RADIO_FREQUENCY 433.0                   // MHz
#define RADIO_BITRATE 9.6                       // kbps
#define RADIO_FREQ_DEVIATION 10.0               // kHz frequency deviation
#define RADIO_RX_BANDWIDTH 39.0                 // kHz
#define RADIO_OUTPUT_POWER 20  
#define RADIO_PREAMBLE_LENGTH 48                // bits
#define RADIO_DATA_SHAPING RADIOLIB_SHAPING_0_3 // bits

// ==================== WiFi Configuration ====================

#define WIFI_TIMEOUT_MS 20000 // 20 seconds timeout for WiFi connection

// ==================== NTRIP Configuration ====================

#define NTRIP_TIMEOUT_MS 10000
#define NTRIP_RECONNECT_INTERVAL_MS 30000
// ==================== Location Configuration ====================
// Katowice, Poland coordinates (example location for GGA)
#define LOCATION_LAT 50.2346 // Latitude (decimal degrees)
#define LOCATION_LON 19.2083 // Longitude (decimal degrees)
#define LOCATION_ALT 278.0   // Altitude in meters

// ==================== Timing Configuration ====================
#define GGA_SEND_INTERVAL 10000    // Send GGA every 10 seconds (ms)
#define GATHERING_DURATION 1000    // 1 second for data gathering (ms)
#define TRANSMISSION_DURATION 1500 // 1 second for radio transmission (ms)

// ==================== Radio Transmission Configuration ====================
#define FIXED_PACKET_SIZE 2000 // Always transmit exactly 2000 bytes
#define PADDING_BYTE 0xFF      // Byte used for padding (0xFF or 0x00)

// ==================== RTCM Configuration ====================
// RTCM message types to filter and buffer
// Format: {messageType, maxMessages}
#define RTCM_BUFFER_CONFIG {                               \
    {1019, 1}, /* GPS ephemeris - keep 2 messages */       \
    {1020, 1}, /* GLONASS ephemeris - keep 2 messages */   \
    {1075, 1}, /* GPS MSM5 - keep 3 messages */            \
    {1085, 1}, /* GLONASS MSM5 - keep 3 messages */        \
    {1095, 1}, /* Galileo MSM5 - keep 3 messages */        \
    {1125, 2}, /* BeiDou MSM5 - keep 2 messages */         \
    {1005, 1}, /* Station ARP - keep 1 message (static) */ \
    {1007, 1}, /* Antenna descriptor - keep 1 message */   \
    {1033, 1}, /* Receiver descriptor - keep 1 message */  \
    {1230, 1}  /* GLONASS biases - keep 2 messages */      \
}

// RTCM message priorities for RTK Fix - lower number = higher priority
// Critical for proper RTK positioning - messages must arrive in correct order
#define RTCM_MESSAGE_PRIORITIES {                                  \
    {1005, 1}, /* Station position (ARP) without antenna height */ \
    {1006, 1}, /* Station position (ARP) with antenna height */    \
    {1007, 2}, /* Antenna descriptor */                            \
    {1008, 2}, /* Antenna descriptor and serial number */          \
    {1033, 3}, /* Receiver and antenna descriptors */              \
    {1230, 4}, /* GLONASS code-phase biases */                     \
                                                                   \
    {1074, 10}, /* GPS MSM4 */                                     \
    {1075, 10}, /* GPS MSM5 */                                     \
    {1076, 10}, /* GPS MSM6 */                                     \
    {1077, 10}, /* GPS MSM7 */                                     \
                                                                   \
    {1084, 20}, /* GLONASS MSM4 */                                 \
    {1085, 20}, /* GLONASS MSM5 */                                 \
    {1086, 20}, /* GLONASS MSM6 */                                 \
    {1087, 20}, /* GLONASS MSM7 */                                 \
                                                                   \
    {1094, 30}, /* Galileo MSM4 */                                 \
    {1095, 30}, /* Galileo MSM5 */                                 \
    {1096, 30}, /* Galileo MSM6 */                                 \
    {1097, 30}, /* Galileo MSM7 */                                 \
                                                                   \
    {1124, 40}, /* BeiDou MSM4 */                                  \
    {1125, 40}, /* BeiDou MSM5 */                                  \
    {1126, 40}, /* BeiDou MSM6 */                                  \
    {1127, 40}, /* BeiDou MSM7 */                                  \
                                                                   \
    {1114, 50}, /* QZSS MSM4 */                                    \
    {1115, 50}, /* QZSS MSM5 */                                    \
    {1116, 50}, /* QZSS MSM6 */                                    \
    {1117, 50}, /* QZSS MSM7 */                                    \
                                                                   \
    {1019, 60}, /* GPS ephemeris */                                \
    {1020, 61}, /* GLONASS ephemeris */                            \
    {1045, 62}, /* Galileo F/NAV ephemeris */                      \
    {1046, 63}  /* Galileo I/NAV ephemeris */                      \
}

// Maximum size of a single RTCM message (bytes)
#define RTCM_MAX_MESSAGE_SIZE 512

// ==================== Debug Configuration ====================
#define ENABLE_HEX_DUMP false  // Enable hex dump of transmitted packets
#define HEX_DUMP_MAX_BYTES 200 // Maximum bytes to display in hex dump

// ==================== OLED Display Configuration ====================
#define OLED_SDA 21          // I2C SDA pin for TTGO T-Beam
#define OLED_SCL 22          // I2C SCL pin for TTGO T-Beam
#define OLED_RESET -1        // Reset pin (-1 if not used)
#define OLED_ADDRESS 0x3C    // I2C address for SSD1306
#define OLED_WIDTH 128       // OLED display width
#define OLED_HEIGHT 64       // OLED display height
#define SPLASH_DURATION 2000 // Splash screen duration (ms)

#endif // CONFIG_H