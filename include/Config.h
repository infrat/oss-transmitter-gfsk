#ifndef CONFIG_H
#define CONFIG_H

#include "Secrets.h"

// ==================== WiFi Configuration ====================

#define WIFI_TIMEOUT_MS 20000 // 20 seconds timeout for WiFi connection

// ==================== NTRIP Configuration ====================

#define NTRIP_TIMEOUT_MS 10000
#define NTRIP_RECONNECT_INTERVAL_MS 30000
// ==================== Location Configuration ====================
// Katowice, Poland coordinates (example location for GGA)
#define LOCATION_LAT 50.2649 // Latitude (decimal degrees)
#define LOCATION_LON 19.0238 // Longitude (decimal degrees)
#define LOCATION_ALT 278.0   // Altitude in meters

// ==================== Timing Configuration ====================
#define GGA_SEND_INTERVAL 10000    // Send GGA every 10 seconds (ms)
#define GATHERING_DURATION 1000     // 1 second for data gathering (ms)
#define TRANSMISSION_DURATION 1000 // 1 second for radio transmission (ms)

// ==================== Radio Transmission Configuration ====================
#define FIXED_PACKET_SIZE 1800 // Always transmit exactly 2000 bytes
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

// Maximum size of a single RTCM message (bytes)
#define RTCM_MAX_MESSAGE_SIZE 512

// ==================== Debug Configuration ====================
#define ENABLE_HEX_DUMP true    // Enable hex dump of transmitted packets
#define HEX_DUMP_MAX_BYTES 2000 // Maximum bytes to display in hex dump

// ==================== OLED Display Configuration ====================
#define OLED_SDA 21           // I2C SDA pin for TTGO T-Beam
#define OLED_SCL 22           // I2C SCL pin for TTGO T-Beam
#define OLED_RESET -1         // Reset pin (-1 if not used)
#define OLED_ADDRESS 0x3C     // I2C address for SSD1306
#define OLED_WIDTH 128        // OLED display width
#define OLED_HEIGHT 64        // OLED display height
#define SPLASH_DURATION 2000  // Splash screen duration (ms)

#endif // CONFIG_H