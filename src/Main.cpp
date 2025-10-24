/*
  Enhanced RadioLib Stream Transmit with NTRIP Client

  Features:
  - WiFi connectivity
  - NTRIP client for RTCM corrections
  - RTCM message filtering (1075, 1085, 1095)
  - State machine: 2s GATHERING + 1s TRANSMISSION cycle
  - GGA position transmission every 10s
  - FSK radio transmission of RTCM data
  - OLED SSD1306 display support

  Hardware: TTGO T-Beam v2
  Radio: SX1276 (FSK mode)
  Display: SSD1306 128x64 OLED
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <SSD1306Wire.h>

// IMPORTANT: config.h must be in the same directory as main.cpp (src/)
#include "Config.h"
#include "SplashBitmap.h"

// ==================== PIN DEFINITIONS ====================

// TTGO LoRa32 v2.1 / T-Beam pin definitions
#define SS 18   // GPIO18 -- SX1276's CS
#define RST 14  // GPIO14 -- SX1276's RESET
#define DI0 26  // GPIO26 -- SX1276's IRQ(Interrupt Request)
#define DIO1 33 // GPIO33 -- SX1276's DIO1

// ==================== RADIO CONFIGURATION ====================

#define FREQUENCY 433.0    // MHz
#define BITRATE 9.6        // kbps
#define FREQ_DEV 5.0       // kHz frequency deviation
#define RX_BANDWIDTH 25    // kHz
#define OUTPUT_POWER 20    // dBm
#define PREAMBLE_LENGTH 48 // bits

// ==================== STATE MACHINE ====================

enum SystemState
{
  STATE_INIT,
  STATE_WAITING_FOR_FIRST_RTCM,
  STATE_GATHERING,
  STATE_TRANSMISSION,
  STATE_ERROR
};

SystemState currentState = STATE_INIT;
unsigned long stateStartTime = 0;
unsigned long lastGGASendTime = 0;
bool firstRTCMReceived = false;

// ==================== RTCM BUFFER ====================

// RTCM buffer configuration structure
struct RTCMBufferEntry
{
  uint16_t messageType;
  uint8_t maxMessages;
};

// Parse configuration from config.h
const RTCMBufferEntry rtcmBufferConfig[] = RTCM_BUFFER_CONFIG;
const uint8_t rtcmBufferConfigSize = sizeof(rtcmBufferConfig) / sizeof(RTCMBufferEntry);

// RTCM message priority structure
struct RTCMPriorityEntry
{
  uint16_t messageType;
  int priority;
};

// Parse RTCM priorities from config.h
const RTCMPriorityEntry rtcmPriorities[] = RTCM_MESSAGE_PRIORITIES;
const uint8_t rtcmPrioritiesSize = sizeof(rtcmPriorities) / sizeof(RTCMPriorityEntry);

// Function to get message priority (lower number = higher priority)
int getRTCMPriority(uint16_t messageType)
{
  for (uint8_t i = 0; i < rtcmPrioritiesSize; i++)
  {
    if (rtcmPriorities[i].messageType == messageType)
    {
      return rtcmPriorities[i].priority;
    }
  }
  return 9999; // Unknown message types get lowest priority
}

// Single RTCM message structure
struct RTCMMessage
{
  uint16_t messageType;
  uint16_t length;
  uint8_t data[RTCM_MAX_MESSAGE_SIZE];
  uint32_t timestamp; // millis() when received
  bool valid;

  RTCMMessage() : messageType(0), length(0), timestamp(0), valid(false)
  {
    memset(data, 0, RTCM_MAX_MESSAGE_SIZE);
  }
};

// Circular FIFO buffer for one message type
class RTCMTypeBuffer
{
private:
  RTCMMessage *messages;
  uint8_t maxSize;
  uint8_t writeIndex;
  uint8_t count;

public:
  RTCMTypeBuffer() : messages(nullptr), maxSize(0), writeIndex(0), count(0) {}

  ~RTCMTypeBuffer()
  {
    if (messages != nullptr)
    {
      delete[] messages;
    }
  }

  void init(uint8_t size)
  {
    maxSize = size;
    messages = new RTCMMessage[maxSize];
    writeIndex = 0;
    count = 0;
  }

  // Add new message (overwrites oldest if full)
  bool add(const uint8_t *data, uint16_t length, uint16_t msgType)
  {
    if (messages == nullptr || length > RTCM_MAX_MESSAGE_SIZE)
    {
      return false;
    }

    // Add to circular buffer
    messages[writeIndex].messageType = msgType;
    messages[writeIndex].length = length;
    memcpy(messages[writeIndex].data, data, length);
    messages[writeIndex].timestamp = millis();
    messages[writeIndex].valid = true;

    // Move write pointer
    writeIndex = (writeIndex + 1) % maxSize;

    // Update count (max = maxSize)
    if (count < maxSize)
    {
      count++;
    }

    return true;
  }

  // Get all valid messages (up to maxSize)
  uint8_t getAll(RTCMMessage *output, uint8_t maxOutput)
  {
    if (messages == nullptr || count == 0)
    {
      return 0;
    }

    uint8_t copied = 0;
    uint8_t readIndex;

    // Calculate start index (oldest message)
    if (count < maxSize)
    {
      readIndex = 0; // Buffer not full yet
    }
    else
    {
      readIndex = writeIndex; // Start from oldest (will be overwritten next)
    }

    // Copy messages in order from oldest to newest
    for (uint8_t i = 0; i < count && copied < maxOutput; i++)
    {
      uint8_t idx = (readIndex + i) % maxSize;
      if (messages[idx].valid)
      {
        output[copied] = messages[idx];
        copied++;
      }
    }

    return copied;
  }

  void clear()
  {
    for (uint8_t i = 0; i < maxSize; i++)
    {
      messages[i].valid = false;
    }
    writeIndex = 0;
    count = 0;
  }

  uint8_t getCount() const
  {
    return count;
  }

  uint8_t getMaxSize() const
  {
    return maxSize;
  }
};

// Main RTCM Buffer Manager
class RTCMBufferManager
{
private:
  RTCMTypeBuffer *typeBuffers;
  uint16_t *messageTypes;
  uint8_t numTypes;

public:
  RTCMBufferManager() : typeBuffers(nullptr), messageTypes(nullptr), numTypes(0) {}

  ~RTCMBufferManager()
  {
    if (typeBuffers != nullptr)
    {
      delete[] typeBuffers;
    }
    if (messageTypes != nullptr)
    {
      delete[] messageTypes;
    }
  }

  void init()
  {
    numTypes = rtcmBufferConfigSize;

    // Allocate arrays
    typeBuffers = new RTCMTypeBuffer[numTypes];
    messageTypes = new uint16_t[numTypes];

    // Initialize each type buffer
    for (uint8_t i = 0; i < numTypes; i++)
    {
      messageTypes[i] = rtcmBufferConfig[i].messageType;
      typeBuffers[i].init(rtcmBufferConfig[i].maxMessages);

      int priority = getRTCMPriority(rtcmBufferConfig[i].messageType);
      Serial.printf("[RTCM] Initialized buffer for type %d (max %d messages, priority %d)\n",
                    rtcmBufferConfig[i].messageType,
                    rtcmBufferConfig[i].maxMessages,
                    priority);
    }
  }

  // Add message to appropriate type buffer
  bool addMessage(uint16_t msgType, const uint8_t *data, uint16_t length)
  {
    // Find buffer for this message type
    for (uint8_t i = 0; i < numTypes; i++)
    {
      if (messageTypes[i] == msgType)
      {
        return typeBuffers[i].add(data, length, msgType);
      }
    }
    return false; // Message type not in config
  }

  // Check if message type should be filtered
  bool isFilteredType(uint16_t msgType)
  {
    for (uint8_t i = 0; i < numTypes; i++)
    {
      if (messageTypes[i] == msgType)
      {
        return true;
      }
    }
    return false;
  }

  // Get all messages sorted by priority first, then by timestamp
  uint16_t getAllMessagesSorted(RTCMMessage *output, uint16_t maxOutput)
  {
    // First, collect all messages from all type buffers
    uint16_t totalMessages = 0;

    for (uint8_t i = 0; i < numTypes; i++)
    {
      uint8_t count = typeBuffers[i].getCount();
      if (totalMessages + count <= maxOutput)
      {
        uint8_t copied = typeBuffers[i].getAll(output + totalMessages, maxOutput - totalMessages);
        totalMessages += copied;
      }
    }

    // Sort by priority first, then by timestamp (bubble sort - good enough for small arrays)
    for (uint16_t i = 0; i < totalMessages - 1; i++)
    {
      for (uint16_t j = 0; j < totalMessages - i - 1; j++)
      {
        int priority_j = getRTCMPriority(output[j].messageType);
        int priority_j1 = getRTCMPriority(output[j + 1].messageType);
        
        bool shouldSwap = false;
        
        // First compare by priority (lower number = higher priority)
        if (priority_j > priority_j1)
        {
          shouldSwap = true;
        }
        // If same priority, compare by timestamp (older first)
        else if (priority_j == priority_j1 && output[j].timestamp > output[j + 1].timestamp)
        {
          shouldSwap = true;
        }
        
        if (shouldSwap)
        {
          // Swap
          RTCMMessage temp = output[j];
          output[j] = output[j + 1];
          output[j + 1] = temp;
        }
      }
    }

    return totalMessages;
  }

  // Clear all buffers
  void clearAll()
  {
    for (uint8_t i = 0; i < numTypes; i++)
    {
      typeBuffers[i].clear();
    }
  }

  // Get total message count across all types
  uint16_t getTotalCount()
  {
    uint16_t total = 0;
    for (uint8_t i = 0; i < numTypes; i++)
    {
      total += typeBuffers[i].getCount();
    }
    return total;
  }

  // Get buffer status for printing
  void printStatus()
  {
    Serial.println(F("[RTCM] Buffer status:"));
    for (uint8_t i = 0; i < numTypes; i++)
    {
      Serial.printf("  Type %d: %d/%d messages\n",
                    messageTypes[i],
                    typeBuffers[i].getCount(),
                    typeBuffers[i].getMaxSize());
    }
  }
};

RTCMBufferManager rtcmBuffer;

// ==================== RTCM PARSER ====================

struct RTCMHeader
{
  uint16_t messageType;
  uint16_t messageLength;
  bool isValid;
};

// Parse RTCM3 message header to extract message type
RTCMHeader parseRTCMHeader(const uint8_t *data, size_t length)
{
  RTCMHeader msg;
  msg.isValid = false;

  // RTCM3 messages start with 0xD3 preamble
  if (length < 6 || data[0] != 0xD3)
  {
    return msg;
  }

  // Extract message length (10 bits after preamble)
  msg.messageLength = ((data[1] & 0x03) << 8) | data[2];

  // Extract message type (12 bits)
  msg.messageType = (data[3] << 4) | ((data[4] >> 4) & 0x0F);

  msg.isValid = true;
  return msg;
}

// Check if message type should be filtered (using buffer manager)
bool isFilteredMessageType(uint16_t messageType)
{
  return rtcmBuffer.isFilteredType(messageType);
}

// ==================== OLED DISPLAY ====================

SSD1306Wire display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

// Display update tracking
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_UPDATE_INTERVAL 500 // Update display every 500ms

// RTCM messages per second tracking (incoming from NTRIP)
uint32_t rtcmRxLastSecond = 0;
uint32_t rtcmRxCurrentSecond = 0;
unsigned long lastSecondTimestamp = 0;

// RTCM messages per second tracking (outgoing to radio, after filtering)
uint32_t rtcmTxLastSecond = 0;
uint32_t rtcmTxCurrentSecond = 0;

// ==================== STATISTICS ====================

uint32_t packetsTransmitted = 0;
uint32_t transmissionErrors = 0;
uint32_t rtcmMessagesReceived = 0;

// ==================== NTRIP CLIENT ====================

WiFiClient ntripClient;
bool ntripConnected = false;

// Generate NMEA GGA sentence for Katowice, Poland
String generateGGA()
{
  // Convert decimal degrees to NMEA format (DDMM.MMMM)
  int latDeg = abs((int)LOCATION_LAT);
  double latMin = (abs(LOCATION_LAT) - latDeg) * 60.0;
  char latDir = LOCATION_LAT >= 0 ? 'N' : 'S';

  int lonDeg = abs((int)LOCATION_LON);
  double lonMin = (abs(LOCATION_LON) - lonDeg) * 60.0;
  char lonDir = LOCATION_LON >= 0 ? 'E' : 'W';

  // Get current time (simplified - using millis)
  unsigned long currentMillis = millis();
  unsigned long seconds = (currentMillis / 1000) % 86400;
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;

  // Build GGA string (without checksum first)
  char gga[120];
  snprintf(gga, sizeof(gga),
           "GPGGA,%02d%02d%02d.00,%02d%08.5f,%c,%03d%08.5f,%c,1,08,1.0,%.1f,M,0.0,M,,",
           hours, minutes, secs,
           latDeg, latMin, latDir,
           lonDeg, lonMin, lonDir,
           LOCATION_ALT);

  // Calculate NMEA checksum
  uint8_t checksum = 0;
  for (int i = 0; gga[i] != '\0'; i++)
  {
    checksum ^= gga[i];
  }

  // Return complete GGA sentence with $ prefix and checksum
  String ggaString = "$" + String(gga) + "*" + String(checksum, HEX);
  ggaString.toUpperCase(); // Ensure checksum is uppercase
  ggaString += "\r\n";

  return ggaString;
}

// Simple Base64 encoding function
String base64Encode(const String &input)
{
  const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String output = "";
  int val = 0;
  int valb = -6;

  for (unsigned char c : input)
  {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0)
    {
      output += base64_chars[(val >> valb) & 0x3F];
      valb -= 6;
    }
  }

  if (valb > -6)
  {
    output += base64_chars[((val << 8) >> (valb + 8)) & 0x3F];
  }

  while (output.length() % 4)
  {
    output += '=';
  }

  return output;
}

// Connect to NTRIP caster
bool connectNTRIP()
{
  Serial.println(F("\n[NTRIP] Connecting to NTRIP caster..."));

  if (!ntripClient.connect(NTRIP_SERVER, NTRIP_PORT))
  {
    Serial.println(F("[NTRIP] Connection failed!"));
    return false;
  }

  Serial.println(F("[NTRIP] Connected to server"));

  // Send NTRIP request
  String request = "GET /" + String(NTRIP_MOUNTPOINT) + " HTTP/1.0\r\n";
  request += "User-Agent: NTRIP ESP32Client/1.0\r\n";
  request += "Accept: */*\r\n";
  request += "Connection: close\r\n";

  // Add authorization if credentials provided
  if (strlen(NTRIP_USER) > 0)
  {
    String auth = String(NTRIP_USER) + ":" + String(NTRIP_PASSWORD);
    String authEncoded = base64Encode(auth);
    request += "Authorization: Basic " + authEncoded + "\r\n";
  }

  request += "\r\n";

  ntripClient.print(request);
  Serial.println(F("[NTRIP] Request sent, waiting for response..."));

  // Wait for response
  unsigned long timeout = millis();
  while (ntripClient.available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(F("[NTRIP] Response timeout!"));
      ntripClient.stop();
      return false;
    }
    delay(10);
  }

  // Read response header
  String response = "";
  while (ntripClient.available())
  {
    String line = ntripClient.readStringUntil('\n');
    response += line + "\n";
    if (line == "\r")
      break; // End of headers
    if (line.indexOf("ICY 200 OK") >= 0 || line.indexOf("HTTP/1.") >= 0)
    {
      Serial.println(F("[NTRIP] Connection accepted!"));
    }
  }

  Serial.println(F("[NTRIP] Response:"));
  Serial.println(response);

  ntripConnected = true;
  return true;
}

// Send GGA position to NTRIP server
void sendGGA()
{
  if (!ntripConnected || !ntripClient.connected())
  {
    Serial.println(F("[NTRIP] Not connected, cannot send GGA"));
    return;
  }

  String gga = generateGGA();
  ntripClient.print(gga);

  Serial.print(F("[NTRIP] Sent GGA: "));
  Serial.print(gga);
}

// Process incoming NTRIP data
void processNTRIPData()
{
  if (!ntripConnected || !ntripClient.connected())
  {
    ntripConnected = false;
    return;
  }

  static uint8_t rtcmParseBuffer[RTCM_MAX_MESSAGE_SIZE];
  static size_t rtcmParseIndex = 0;

  while (ntripClient.available())
  {
    uint8_t byte = ntripClient.read();

    // Look for RTCM3 preamble (0xD3)
    if (rtcmParseIndex == 0 && byte != 0xD3)
    {
      continue; // Skip until we find preamble
    }

    rtcmParseBuffer[rtcmParseIndex++] = byte;

    // Check if we have enough bytes to parse header
    if (rtcmParseIndex >= 6)
    {
      RTCMHeader msg = parseRTCMHeader(rtcmParseBuffer, rtcmParseIndex);

      if (msg.isValid)
      {
        // Calculate total message size (header + payload + CRC)
        size_t totalSize = 3 + msg.messageLength + 3;

        // Check if we have complete message
        if (rtcmParseIndex >= totalSize)
        {
          // Check if this is a filtered message type
          if (isFilteredMessageType(msg.messageType))
          {
            // Add to appropriate type buffer
            if (rtcmBuffer.addMessage(msg.messageType, rtcmParseBuffer, totalSize))
            {
              Serial.printf("[RTCM] Buffered message type %d, length %d bytes\n",
                            msg.messageType, totalSize);
              rtcmMessagesReceived++;
              rtcmRxCurrentSecond++; // Track incoming messages for display

              // Mark that we received first RTCM message
              if (!firstRTCMReceived)
              {
                firstRTCMReceived = true;
                Serial.println(F("[RTCM] First RTCM message received - ready to start transmission cycles!"));
              }
            }
            else
            {
              Serial.printf("[RTCM] Failed to buffer message type %d\n", msg.messageType);
            }
          }
          else
          {
            Serial.printf("[RTCM] Skipped message type %d\n", msg.messageType);
          }

          // Reset parser for next message
          rtcmParseIndex = 0;
        }
        else if (rtcmParseIndex >= RTCM_MAX_MESSAGE_SIZE)
        {
          // Message too large, reset parser
          Serial.println(F("[RTCM] Message too large, resetting parser"));
          rtcmParseIndex = 0;
        }
      }
      else if (rtcmParseIndex > RTCM_MAX_MESSAGE_SIZE)
      {
        // Buffer overflow protection
        Serial.println(F("[RTCM] Parse buffer overflow, resetting"));
        rtcmParseIndex = 0;
      }
    }
  }
}

// ==================== RADIO SETUP ====================

SX1276 radio = new Module(SS, DI0, RST, DIO1);

// Transmission state for radio operations
int transmissionState;
volatile bool fifoEmpty = false;
bool transmittedFlag = false;

// Radio transmission buffer
uint8_t *txPacket = nullptr;
int totalLength = 0;
int remLength = 0;

// ==================== ISR FUNCTION ====================

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void fifoAdd(void)
{
  fifoEmpty = true;
}

// ==================== HELPER FUNCTIONS ====================

void printHexDump(const uint8_t *data, size_t totalSize, size_t maxBytes)
{
  size_t bytesToPrint = min(totalSize, maxBytes);
  Serial.printf("[TX] HEX dump (first %u of %u bytes):\n", bytesToPrint, totalSize);

  for (size_t i = 0; i < bytesToPrint; i++)
  {
    if (i % 16 == 0)
    {
      Serial.printf("%04X: ", i);
    }

    Serial.printf("%02X ", data[i]);

    if ((i + 1) % 16 == 0 || i == bytesToPrint - 1)
    {
      // Print ASCII representation
      size_t lineStart = i - (i % 16);
      size_t lineEnd = min(i + 1, bytesToPrint);

      // Pad with spaces if not a full line
      for (size_t j = (i % 16) + 1; j < 16; j++)
      {
        Serial.print("   ");
      }

      Serial.print(" |");
      for (size_t j = lineStart; j < lineEnd; j++)
      {
        char c = (data[j] >= 32 && data[j] <= 126) ? data[j] : '.';
        Serial.print(c);
      }
      Serial.println("|");
    }
  }

  if (totalSize > maxBytes)
  {
    Serial.printf("[TX] ... (%u bytes truncated)\n", totalSize - maxBytes);
  }
  Serial.println();
}

void printStats()
{
  Serial.println(F("\n=== Statistics ==="));
  Serial.printf("Packets transmitted: %u\n", packetsTransmitted);
  Serial.printf("Transmission errors: %u\n", transmissionErrors);
  Serial.printf("RTCM messages received: %u\n", rtcmMessagesReceived);
  Serial.printf("Total buffered messages: %u\n", rtcmBuffer.getTotalCount());

  // Show per-type buffer status
  rtcmBuffer.printStatus();

  if (packetsTransmitted > 0 || transmissionErrors > 0)
  {
    float successRate = (100.0 * packetsTransmitted) / (packetsTransmitted + transmissionErrors);
    Serial.printf("Success rate: %.1f%%\n", successRate);
  }
  Serial.println(F("==================\n"));
}

// ==================== OLED DISPLAY FUNCTIONS ====================

// Initialize OLED display
void initDisplay()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}

// Show splash screen
void showSplashScreen()
{
  display.clear();

  // Draw bitmap full screen (128x64 bitmap on 128x64 screen)
  display.drawXbm(0, 0, splash_width, splash_height, splashBitmap);

  display.display();
}

// Show error message
void showError(const char *title, const char *message)
{
  display.clear();

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 10, title);

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 35, message);

  display.display();
}

// Show status message during initialization
void showStatus(const char *message)
{
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "OSS RTCM Transmitter");
  display.drawString(0, 15, "Initializing...");
  display.drawString(0, 35, message);

  display.display();
}

// Update statistics display
void updateStatsDisplay()
{
  // Calculate RTCM messages per second (both RX and TX)
  if (millis() - lastSecondTimestamp >= 1000)
  {
    rtcmRxLastSecond = rtcmRxCurrentSecond;
    rtcmRxCurrentSecond = 0;
    rtcmTxLastSecond = rtcmTxCurrentSecond;
    rtcmTxCurrentSecond = 0;
    lastSecondTimestamp = millis();
  }

  display.clear();

  // Title
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Top line: Header
  display.drawString(0, 0, "OSS RTCM Transmitter");
  display.drawLine(0, 12, 128, 12);

  // Status line
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  const char *stateStr = "UNKNOWN";
  if (currentState == STATE_WAITING_FOR_FIRST_RTCM)
    stateStr = "WAITING";
  else if (currentState == STATE_GATHERING)
    stateStr = "GATHERING";
  else if (currentState == STATE_TRANSMISSION)
    stateStr = "TRANSMIT";
  else if (currentState == STATE_ERROR)
    stateStr = "ERROR";

  display.drawString(0, 15, "Status:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 15, stateStr);

  // RTCM RX (incoming from NTRIP)
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 28, "RX:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  char rxRate[16];
  snprintf(rxRate, sizeof(rxRate), "%u msg/s", rtcmRxLastSecond);
  display.drawString(128, 28, rxRate);

  // RTCM TX (outgoing to radio, filtered)
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 38, "TX:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  char txRate[16];
  snprintf(txRate, sizeof(txRate), "%u msg/s", rtcmTxLastSecond);
  display.drawString(128, 38, txRate);

  // WiFi status indicator
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 51, "WiFi:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(50, 51, WiFi.status() == WL_CONNECTED ? "OK" : "ERR");

  // NTRIP status indicator
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(60, 51, "NTRIP:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 51, ntripConnected ? "OK" : "ERR");

  display.display();
}

// ==================== WIFI SETUP ====================

bool connectWiFi()
{
  Serial.println(F("\n[WiFi] Connecting to WiFi..."));
  Serial.printf("[WiFi] SSID: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttemptTime < WIFI_TIMEOUT_MS)
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("\n[WiFi] Connection failed!"));
    return false;
  }

  Serial.println(F("\n[WiFi] Connected!"));
  Serial.print(F("[WiFi] IP address: "));
  Serial.println(WiFi.localIP());

  return true;
}

// ==================== STATE MACHINE ====================

void enterState(SystemState newState)
{
  currentState = newState;
  stateStartTime = millis();

  switch (newState)
  {
  case STATE_WAITING_FOR_FIRST_RTCM:
    Serial.println(F("\n>>> STATE: WAITING FOR FIRST RTCM"));
    break;
  case STATE_GATHERING:
    Serial.println(F("\n>>> STATE: GATHERING (2s)"));
    break;
  case STATE_TRANSMISSION:
    Serial.println(F("\n>>> STATE: TRANSMISSION (1s)"));
    break;
  case STATE_ERROR:
    Serial.println(F("\n>>> STATE: ERROR"));
    break;
  default:
    break;
  }
}

void handleWaitingForFirstRTCMState()
{
  // Check if we need to send GGA
  if (millis() - lastGGASendTime >= GGA_SEND_INTERVAL)
  {
    sendGGA();
    lastGGASendTime = millis();
  }

  // Process incoming NTRIP data
  processNTRIPData();

  // Check if we received first RTCM message
  if (firstRTCMReceived)
  {
    Serial.println(F("[STATE] First RTCM received, starting transmission cycles!"));
    enterState(STATE_GATHERING);
  }
}

void handleGatheringState()
{
  // Check if we need to send GGA
  if (millis() - lastGGASendTime >= GGA_SEND_INTERVAL)
  {
    sendGGA();
    lastGGASendTime = millis();
  }

  // Process incoming NTRIP data
  processNTRIPData();

  // Check if gathering time is up
  if (millis() - stateStartTime >= GATHERING_DURATION)
  {
    enterState(STATE_TRANSMISSION);
  }
}

void handleTransmissionState()
{
  static bool transmissionStarted = false;
  static RTCMMessage *messagesToSend = nullptr;
  static uint16_t messageCount = 0;

  // Start transmission at the beginning of this state (ALWAYS transmit, even if buffer empty)
  if (!transmissionStarted)
  {
    // Allocate temporary array for sorted messages (max 50 messages should be enough)
    const uint16_t maxMessages = 50;
    messagesToSend = new RTCMMessage[maxMessages];

    if (messagesToSend == nullptr)
    {
      Serial.println(F("[ERROR] Failed to allocate message array!"));
      enterState(STATE_GATHERING);
      return;
    }

    // Get all messages sorted by priority (RTK-optimized order), then by timestamp
    messageCount = rtcmBuffer.getAllMessagesSorted(messagesToSend, maxMessages);

    Serial.printf("[TX] Preparing %d RTCM messages for transmission\n", messageCount);

    // Track outgoing messages for display statistics
    rtcmTxCurrentSecond += messageCount;

    // Calculate total RTCM data size
    uint32_t rtcmDataSize = 0;
    for (uint16_t i = 0; i < messageCount; i++)
    {
      rtcmDataSize += messagesToSend[i].length;
    }

    Serial.printf("[TX] Total RTCM data: %u bytes\n", rtcmDataSize);

    // Always allocate FIXED_PACKET_SIZE (from config.h)
    if (txPacket != nullptr)
    {
      free(txPacket);
    }

    txPacket = (uint8_t *)malloc(FIXED_PACKET_SIZE);

    if (txPacket == nullptr)
    {
      Serial.println(F("[ERROR] Failed to allocate transmission buffer!"));
      delete[] messagesToSend;
      messagesToSend = nullptr;
      enterState(STATE_GATHERING);
      return;
    }

    // Copy all RTCM messages into transmission buffer (in priority order)
    uint32_t offset = 0;
    for (uint16_t i = 0; i < messageCount; i++)
    {
      memcpy(txPacket + offset, messagesToSend[i].data, messagesToSend[i].length);
      int priority = getRTCMPriority(messagesToSend[i].messageType);
      Serial.printf("[TX] Message %d/%d: type %d, priority %d, %d bytes, timestamp %lu ms\n",
                    i + 1, messageCount,
                    messagesToSend[i].messageType,
                    priority,
                    messagesToSend[i].length,
                    messagesToSend[i].timestamp);
      offset += messagesToSend[i].length;
    }

    // Add padding to reach exactly FIXED_PACKET_SIZE
    uint32_t paddingSize = FIXED_PACKET_SIZE - rtcmDataSize;
    if (paddingSize > 0)
    {
      memset(txPacket + offset, PADDING_BYTE, paddingSize); // Padding from config.h
      Serial.printf("[TX] Added %u bytes of padding (0x%02X)\n", paddingSize, PADDING_BYTE);
    }

    totalLength = FIXED_PACKET_SIZE; // Always fixed size from config
    remLength = totalLength;

    Serial.printf("[TX] Starting transmission of %d bytes (RTCM: %u, Padding: %u)\n",
                  totalLength, rtcmDataSize, paddingSize);

// Display hex dump of transmitted data if enabled
#ifdef ENABLE_HEX_DUMP
#if ENABLE_HEX_DUMP
    printHexDump(txPacket, totalLength, HEX_DUMP_MAX_BYTES);
#endif
#endif

    transmissionState = radio.startTransmit(txPacket, totalLength);
    transmissionStarted = true;

    // Clean up message array
    delete[] messagesToSend;
    messagesToSend = nullptr;
  }

  // Handle FIFO refill during transmission
  if (fifoEmpty)
  {
    fifoEmpty = false;
    transmittedFlag = radio.fifoAdd(txPacket, totalLength, &remLength);
  }

  // Check if transmission finished
  if (transmittedFlag)
  {
    transmittedFlag = false;
    transmissionStarted = false;
    remLength = totalLength;

    if (transmissionState == RADIOLIB_ERR_NONE)
    {
      Serial.println(F("[TX] Transmission finished!"));
      packetsTransmitted++;

      // Clear all buffers after successful transmission
      Serial.println(F("[TX] Clearing RTCM buffers"));
      rtcmBuffer.clearAll();
    }
    else
    {
      Serial.printf("[TX] Transmission failed, code %d\n", transmissionState);
      transmissionErrors++;

      // Also clear buffers on error (fresh start next cycle)
      rtcmBuffer.clearAll();
    }

    radio.standby();

    // Free transmission buffer
    if (txPacket != nullptr)
    {
      free(txPacket);
      txPacket = nullptr;
    }
  }

  // Check if transmission time is up
  if (millis() - stateStartTime >= TRANSMISSION_DURATION)
  {
    // If still transmitting, let it finish
    if (!transmissionStarted)
    {
      enterState(STATE_GATHERING);
    }
  }
}

// ==================== ARDUINO SETUP AND LOOP ====================

void setupRadio()
{
  Serial.print(F("[SX1276] Setting FSK parameters ... "));

  radio.setBitRate(BITRATE);
  radio.setFrequencyDeviation(FREQ_DEV);
  radio.setRxBandwidth(RX_BANDWIDTH);
  radio.setOutputPower(OUTPUT_POWER);
  radio.setPreambleLength(PREAMBLE_LENGTH);
  radio.setDataShaping(RADIOLIB_SHAPING_0_5);

  // Set sync word for packet synchronization
  radio.setFifoEmptyAction(fifoAdd);
  radio.fixedPacketLengthMode(0);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  NTRIP + FSK Radio Transmitter"));
  Serial.println(F("========================================"));

  // Initialize OLED display
  initDisplay();

  // Show splash screen
  showSplashScreen();
  delay(SPLASH_DURATION); // Show splash for configured duration

  // Initialize RTCM buffer
  rtcmBuffer.init();

  // Connect to WiFi
  showStatus("Connecting WiFi...");
  if (!connectWiFi())
  {
    Serial.println(F("[ERROR] WiFi connection failed! Halting."));
    showError("ERROR", "WiFi Failed!");
    currentState = STATE_ERROR;
    while (true)
    {
      delay(1000);
    }
  }

  // Connect to NTRIP
  showStatus("Connecting NTRIP...");
  if (!connectNTRIP())
  {
    Serial.println(F("[ERROR] NTRIP connection failed! Halting."));
    showError("ERROR", "NTRIP Failed!");
    currentState = STATE_ERROR;
    while (true)
    {
      delay(1000);
    }
  }

  // Send initial GGA immediately to start receiving RTCM data
  Serial.println(F("[NTRIP] Sending initial GGA position..."));
  sendGGA();
  lastGGASendTime = millis();

  // Initialize radio
  showStatus("Initializing Radio...");
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.beginFSK(FREQUENCY);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.printf("failed, code %d\n", state);
    showError("ERROR", "Radio Failed!");
    currentState = STATE_ERROR;
    while (true)
    {
      delay(1000);
    }
  }

  // Configure radio
  setupRadio();

  Serial.println(F("\n[INFO] System initialized successfully!"));
  Serial.printf("[INFO] Cycle: %ds GATHERING + %ds TRANSMISSION\n",
                GATHERING_DURATION / 1000, TRANSMISSION_DURATION / 1000);
  Serial.printf("[INFO] GGA send interval: %ds\n", GGA_SEND_INTERVAL / 1000);
  Serial.println();
  Serial.println(F("[INFO] Waiting for first RTCM message before starting transmission cycles..."));

  // Start state machine - wait for first RTCM message
  enterState(STATE_WAITING_FOR_FIRST_RTCM);
  lastSecondTimestamp = millis();

  // Show ready message briefly
  showStatus("Waiting for RTCM...");
  delay(1000);
}

void loop()
{
  // Check WiFi and NTRIP connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("[ERROR] WiFi disconnected!"));
    showStatus("WiFi reconnecting...");
    connectWiFi();
  }

  if (!ntripClient.connected() && ntripConnected)
  {
    Serial.println(F("[ERROR] NTRIP disconnected!"));
    ntripConnected = false;
    showStatus("NTRIP reconnecting...");
    connectNTRIP();
  }

  // Run state machine
  switch (currentState)
  {
  case STATE_WAITING_FOR_FIRST_RTCM:
    handleWaitingForFirstRTCMState();
    break;

  case STATE_GATHERING:
    handleGatheringState();
    break;

  case STATE_TRANSMISSION:
    handleTransmissionState();
    break;

  case STATE_ERROR:
    // Stay in error state
    delay(1000);
    break;

  default:
    break;
  }

  // Update OLED display periodically
  if (currentState != STATE_ERROR && millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
  {
    updateStatsDisplay();
    lastDisplayUpdate = millis();
  }

  // Print stats every 30 seconds
  static unsigned long lastStatsTime = 0;
  if (millis() - lastStatsTime >= 30000)
  {
    printStats();
    lastStatsTime = millis();
  }

  yield(); // Let ESP32 handle background tasks
}