/*
 *  A Combo of VirtualKeypad-Web and HomeAssistant-MQTT 
 *  https://github.com/taligentx/dscKeybusInterface
 *  Provides a virtual keypad web interface using the esp8266 as a standalone web server
 *  and a mqtt client for integration into HomeAssistant
 * 
 *  To upload the files for the web interface, run in the PlatformIO Terminal
 *  `platformio run --target uploadfs`
 * 
 *  Add to HomeAssistant https://www.home-assistant.io/components/alarm_control_panel.mqtt/
 *
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <dscKeybusInterface.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <FS.h>
#include <SPIFFSEditor.h>
#include <ArduinoJson.h>
#include <Chrono.h>
#include <PubSubClient.h>
#include <Ticker.h>

// Configures the Keybus interface with the specified pins
// dscWritePin is optional, leaving it out disables the virtual keypad
// esp8266: D1, D2, D8 (GPIO 5, 4, 15)
#define dscClockPin D1  
#define dscReadPin D2
#define dscWritePin D8

#include <secrets.h>

// Server settings
char dnsHostname[] = "dsc";  // Sets the domain name - if set to "dsc", access via: http://dsc.local

// MQTT settings
const char* mqttClientName = "dscKeybusInterface";
const char* mqttPartitionTopic = "dsc/Get/Partition"; // Sends armed and alarm status per partition: dsc/Get/Partition1 ... dsc/Get/Partition8
const char* mqttZoneTopic = "dsc/Get/Zone"; // Sends zone status per zone: dsc/Get/Zone1 ... dsc/Get/Zone64
const char* mqttTroubleTopic = "dsc/Get/Trouble"; // Sends trouble status
const char* mqttStatusTopic = "dsc/Status"; // Sends the status
const char* mqttSubscribeTopic = "dsc/Set"; // Receives messages to write to the panel
const char* mqttBirthMessage = "online";
const char* mqttLwtMessage = "offline";

dscKeybusInterface dsc(dscClockPin, dscReadPin, dscWritePin);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Chrono ws_ping_pong(Chrono::SECONDS);
bool partitionChanged;
byte viewPartition = 1;
byte lights_sent = 0x00;
byte last_open_zones[8];
byte partition = 0;
bool force_send_status_for_new_client = false;
bool force_send_mqtt = true;
const char* lcdPartition = "Partition ";
WiFiClient wifiClient;
PubSubClient mqtt(mqttServer, mqttPort, wifiClient);
unsigned long mqttPreviousTime;
Ticker secondTick;
volatile long watchdog;

void setLights(byte partition) {
  if ((dsc.lights[partition] != lights_sent || force_send_status_for_new_client) && ws.count()) {
    char outas[128];
    StaticJsonDocument<200> doc;
    JsonObject root = doc.to<JsonObject>();
    root["status_packet"] = dsc.lights[partition];
    serializeJson(root, outas);
    ws.textAll(outas);
    lights_sent = dsc.lights[partition];
  }
}

void setStatus(byte partition) {
  static byte lastStatus[8];
  if (!partitionChanged && dsc.status[partition] == lastStatus[partition] && !force_send_status_for_new_client) return;
  lastStatus[partition] = dsc.status[partition];

  if (ws.count()) {
    char outas[128];
    StaticJsonDocument<200> doc;
    JsonObject root = doc.to<JsonObject>();

    switch (dsc.status[partition]) {
      case 0x01: root["lcd_lower"] = "Ready"; break;
      case 0x02: root["lcd_lower"] = "Stay zones open"; break;
      case 0x03: root["lcd_lower"] = "Zones open"; break;
      case 0x04: root["lcd_lower"] = "Armed stay"; break;
      case 0x05: root["lcd_lower"] = "Armed away"; break;
      case 0x07: root["lcd_lower"] = "Failed to arm"; break;
      case 0x08: root["lcd_lower"] = "Exit delay"; break;
      case 0x09: root["lcd_lower"] = "No entry delay"; break;
      case 0x0B: root["lcd_lower"] = "Quick exit"; break;
      case 0x0C: root["lcd_lower"] = "Entry delay"; break;
      case 0x0D: root["lcd_lower"] = "Alarm memory"; break;
      case 0x10: root["lcd_lower"] = "Keypad lockout"; break;
      case 0x11: root["lcd_lower"] = "Alarm"; break;
      case 0x14: root["lcd_lower"] = "Auto-arm"; break;
      case 0x16: root["lcd_lower"] = "No entry delay"; break;
      case 0x22: root["lcd_lower"] = "Alarm memory"; break;
      case 0x33: root["lcd_lower"] = "Busy"; break;
      case 0x3D: root["lcd_lower"] = "Disarmed"; break;
      case 0x3E: root["lcd_lower"] = "Disarmed"; break;
      case 0x40: root["lcd_lower"] = "Keypad blanked"; break;
      case 0x8A: root["lcd_lower"] = "Activate zones"; break;
      case 0x8B: root["lcd_lower"] = "Quick exit"; break;
      case 0x8E: root["lcd_lower"] = "Invalid option"; break;
      case 0x8F: root["lcd_lower"] = "Invalid code"; break;
      case 0x9E: root["lcd_lower"] = "Enter * code"; break;
      case 0x9F: root["lcd_lower"] = "Access code"; break;
      case 0xA0: root["lcd_lower"] = "Zone bypass"; break;
      case 0xA1: root["lcd_lower"] = "Trouble menu"; break;
      case 0xA2: root["lcd_lower"] = "Alarm memory"; break;
      case 0xA3: root["lcd_lower"] = "Door chime on"; break;
      case 0xA4: root["lcd_lower"] = "Door chime off"; break;
      case 0xA5: root["lcd_lower"] = "Master code"; break;
      case 0xA6: root["lcd_lower"] = "Access codes"; break;
      case 0xA7: root["lcd_lower"] = "Enter new code"; break;
      case 0xA9: root["lcd_lower"] = "User function"; break;
      case 0xAA: root["lcd_lower"] = "Time and Date"; break;
      case 0xAB: root["lcd_lower"] = "Auto-arm time"; break;
      case 0xAC: root["lcd_lower"] = "Auto-arm on"; break;
      case 0xAD: root["lcd_lower"] = "Auto-arm off"; break;
      case 0xAF: root["lcd_lower"] = "System test"; break;
      case 0xB0: root["lcd_lower"] = "Enable DLS"; break;
      case 0xB2: root["lcd_lower"] = "Command output"; break;
      case 0xB7: root["lcd_lower"] = "Installer code"; break;
      case 0xB8: root["lcd_lower"] = "Enter * code"; break;
      case 0xB9: root["lcd_lower"] = "Zone tamper"; break;
      case 0xBA: root["lcd_lower"] = "Zones low batt."; break;
      case 0xC6: root["lcd_lower"] = "Zone fault menu"; break;
      case 0xC8: root["lcd_lower"] = "Service required"; break;
      case 0xD0: root["lcd_lower"] = "Keypads low batt"; break;
      case 0xD1: root["lcd_lower"] = "Wireless low bat"; break;
      case 0xE4: root["lcd_lower"] = "Installer menu"; break;
      case 0xE5: root["lcd_lower"] = "Keypad slot"; break;
      case 0xE6: root["lcd_lower"] = "Input: 2 digits"; break;
      case 0xE7: root["lcd_lower"] = "Input: 3 digits"; break;
      case 0xE8: root["lcd_lower"] = "Input: 4 digits"; break;
      case 0xEA: root["lcd_lower"] = "Code: 2 digits"; break;
      case 0xEB: root["lcd_lower"] = "Code: 4 digits"; break;
      case 0xEC: root["lcd_lower"] = "Input: 6 digits"; break;
      case 0xED: root["lcd_lower"] = "Input: 32 digits"; break;
      case 0xEE: root["lcd_lower"] = "Input: option"; break;
      case 0xF0: root["lcd_lower"] = "Function key 1"; break;
      case 0xF1: root["lcd_lower"] = "Function key 2"; break;
      case 0xF2: root["lcd_lower"] = "Function key 3"; break;
      case 0xF3: root["lcd_lower"] = "Function key 4"; break;
      case 0xF4: root["lcd_lower"] = "Function key 5"; break;
      case 0xF8: root["lcd_lower"] = "Keypad program"; break;
      default: root["lcd_lower"] = dsc.status[partition];
    }
    root["millis"] = millis();
    serializeJson(root, outas);
    ws.textAll(outas);
  }
}

void changedPartition(byte partition) {
  partitionChanged = true;
  setStatus(partition);
  setLights(partition);
  partitionChanged = false;
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->printf("{\"connected_id\": %u}", client->id());
    force_send_status_for_new_client = true;

    client->ping();
    ws_ping_pong.restart();

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
    if (ws.count() <= 0) {
      ws_ping_pong.stop();
    }

  } else if (type == WS_EVT_ERROR) {
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);

  } else if (type == WS_EVT_PONG) {
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if (info->final && info->index == 0 && info->len == len) {
      //the whole message is in a single frame and we got all of it's data
      //Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

      if (info->opcode == WS_TEXT) {
        for (size_t i = 0; i < info->len; i++) {
          msg += (char) data[i];
        }
      }
      Serial.printf("%s\n", msg.c_str());

      if (info->opcode == WS_TEXT) {
        StaticJsonDocument<200> doc;
        auto err = deserializeJson(doc, msg);
        if (!err) {
          JsonObject root = doc.as<JsonObject>();
          if (root.containsKey("btn_single_click")) {
            char *tmp = (char *)root["btn_single_click"].as<char*>();
            char * const sep_at = strchr(tmp, '_');
            if (sep_at != NULL)            {
              *sep_at = '\0';
              dsc.write(sep_at + 1);
            }
          }
        }
      }

    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if (info->index == 0) {
        if (info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

      if (info->opcode == WS_TEXT) {
        for (size_t i = 0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for (size_t i = 0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n", msg.c_str());

      if ((info->index + len) == info->len) {
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if (info->final) {
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
          if (info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}

bool mqttConnect() {
  Serial.print(F("MQTT connect... "));
  Serial.println(mqttServer);
  if (mqtt.connect(mqttClientName, mqttUsername, mqttPassword, mqttStatusTopic, 0, true, mqttLwtMessage)) {
    Serial.println(F("MQTT connected"));
    mqtt.publish(mqttStatusTopic, mqttBirthMessage, true);
    force_send_mqtt = true;
    mqtt.subscribe(mqttSubscribeTopic);
  } else {
    Serial.println(F("MQTT connection failed"));
  }
  return mqtt.connected();
}

void mqttHandle() {
  if (!mqtt.connected()) {
    unsigned long mqttCurrentTime = millis();
    if (mqttCurrentTime - mqttPreviousTime > 5000) {
      mqttPreviousTime = mqttCurrentTime;
      if (mqttConnect()) {
        Serial.println(F("MQTT successfully reconnected."));
        mqttPreviousTime = 0;
      }
      else Serial.println(F("MQTT failed to reconnect."));
    }
  }
  else mqtt.loop();
}

// Handles messages received in the mqttSubscribeTopic
void mqttCallback(char* topic, byte* payload, unsigned int length) {

  // Handles unused parameters
  (void)topic;
  (void)length;

  byte partition = 0;
  byte payloadIndex = 0;

  // Checks if a partition number 1-8 has been sent and sets the second character as the payload
  if (payload[0] >= 0x31 && payload[0] <= 0x38) {
    partition = payload[0] - 49;
    payloadIndex = 1;
  }

  // Arm stay
  if (payload[payloadIndex] == 'S' && !dsc.armed[partition] && !dsc.exitDelay[partition]) {
    while (!dsc.writeReady) dsc.handlePanel();  // Continues processing Keybus data until ready to write
    dsc.writePartition = partition + 1;         // Sets writes to the partition number
    dsc.write('s');                             // Virtual keypad arm stay
  }

  // Arm away
  else if (payload[payloadIndex] == 'A' && !dsc.armed[partition] && !dsc.exitDelay[partition]) {
    while (!dsc.writeReady) dsc.handlePanel();  // Continues processing Keybus data until ready to write
    dsc.writePartition = partition + 1;         // Sets writes to the partition number
    dsc.write('w');                             // Virtual keypad arm away
  }

  // Disarm
  else if (payload[payloadIndex] == 'D' && (dsc.armed[partition] || dsc.exitDelay[partition])) {
    while (!dsc.writeReady) dsc.handlePanel();  // Continues processing Keybus data until ready to write
    dsc.writePartition = partition + 1;         // Sets writes to the partition number
    dsc.write(accessCode);
  }
}

void handleDscChanges() {
if (dsc.handlePanel() && (dsc.statusChanged || force_send_status_for_new_client || force_send_mqtt)) {  // Processes data only when a valid Keybus command has been read
    dsc.statusChanged = false;  // Resets the status flag

    // If the Keybus data buffer is exceeded, the sketch is too busy to process all Keybus commands.  Call
    // handlePanel() more often, or increase dscBufferSize in the library: src/dscKeybusInterface.h
    if (dsc.bufferOverflow) {
      Serial.println("Keybus buffer overflow");
    }
    dsc.bufferOverflow = false;

    // Checks status for the currently viewed partition
    partition = viewPartition - 1;

    setLights(partition);
    setStatus(partition);

    // Publishes status per partition
    for (byte partition = 0; partition < dscPartitions; partition++) {

      // Appends the mqttPartitionTopic with the partition number
      char publishTopic[strlen(mqttPartitionTopic) + 1];
      char partitionNumber[2];
      strcpy(publishTopic, mqttPartitionTopic);
      itoa(partition + 1, partitionNumber, 10);
      strcat(publishTopic, partitionNumber);

      // Publishes exit delay status
      if (dsc.exitDelayChanged[partition] || force_send_mqtt) {
        dsc.exitDelayChanged[partition] = false;  // Resets the exit delay status flag
        if (dsc.exitDelay[partition]) {
          mqtt.publish(publishTopic, "pending", true);
        } else if (!dsc.exitDelay[partition] && !dsc.armed[partition]) {
          mqtt.publish(publishTopic, "disarmed", true);
        }
      }

      // Publishes armed/disarmed status
      if (dsc.armedChanged[partition] || force_send_mqtt) {
        dsc.armedChanged[partition] = false;  // Resets the partition armed status flag
        if (dsc.armed[partition] && dsc.armedAway[partition]) {
          mqtt.publish(publishTopic, "armed_away", true);
        } else if (dsc.armed[partition] && dsc.armedStay[partition]) {
          mqtt.publish(publishTopic, "armed_home", true);
        } else {
          mqtt.publish(publishTopic, "disarmed", true);
        }
      }

      // Publishes alarm status
      if (dsc.alarmChanged[partition] || force_send_mqtt) {
        dsc.alarmChanged[partition] = false;  // Resets the partition alarm status flag
        if (dsc.alarm[partition]) {
          mqtt.publish(publishTopic, "triggered", true);  // Alarm tripped
        }
      }

    }

    // Zone status is stored in the openZones[] and openZonesChanged[] arrays using 1 bit per zone, up to 64 zones
    //   openZones[0] and openZonesChanged[0]: Bit 0 = Zone 1 ... Bit 7 = Zone 8
    //   openZones[1] and openZonesChanged[1]: Bit 0 = Zone 9 ... Bit 7 = Zone 16
    //   ...
    //   openZones[7] and openZonesChanged[7]: Bit 0 = Zone 57 ... Bit 7 = Zone 64
    // only reading from group 0 as I only have 8 zones
    if (dsc.openZonesStatusChanged || force_send_status_for_new_client || force_send_mqtt) {
      dsc.openZonesStatusChanged = false;  // Resets the open zones status flag
      
      if (ws.count()) {
        char outas[512];
        StaticJsonDocument<200> doc;
        JsonObject root = doc.to<JsonObject>();
        root["open_zone_0"] = dsc.openZones[0];
        serializeJson(root, outas);
        ws.textAll(outas);
      }

      for (byte zoneBit = 0; zoneBit < 8; zoneBit++) {
        // Checks an individual open zone status flag
        if (bitRead(dsc.openZonesChanged[0], zoneBit) || force_send_mqtt) {
          // Resets the individual open zone status flag
          bitWrite(dsc.openZonesChanged[0], zoneBit, 0);
          // Appends the mqttZoneTopic with the zone number
          char zonePublishTopic[strlen(mqttZoneTopic) + 2];
          char zone[3];
          strcpy(zonePublishTopic, mqttZoneTopic);
          itoa(zoneBit + 1, zone, 10);
          strcat(zonePublishTopic, zone);
          Serial.print("Zone change publishing to ");
          Serial.println(zonePublishTopic);
          if (bitRead(dsc.openZones[0], zoneBit)) {
            mqtt.publish(zonePublishTopic, "1", true); // Zone open
          } else {
            mqtt.publish(zonePublishTopic, "0", true); // Zone closed
          }
        }
      }

    }

    if (dsc.alarmZonesStatusChanged || force_send_status_for_new_client) {
      dsc.alarmZonesStatusChanged = false;  // Resets the alarm zones status flag

      if (ws.count()) {
        char outas[512];
        StaticJsonDocument<200> doc;
        JsonObject root = doc.to<JsonObject>();
        root["alarm_zone_0"] = dsc.alarmZones[0];
        serializeJson(root, outas);
        ws.textAll(outas);
      }
    }

    if (dsc.troubleChanged || force_send_mqtt) {
      dsc.troubleChanged = false;  // Resets the trouble status flag
      if (dsc.trouble) {
        Serial.println("DSC trouble: 1");
        mqtt.publish(mqttTroubleTopic, "1", true);
      } else {
        Serial.println("DSC trouble: 0");
        mqtt.publish(mqttTroubleTopic, "0", true);
      }
    }

    if (dsc.powerChanged || force_send_mqtt) {
      dsc.powerChanged = false;  // Resets the power trouble status flag
      if (dsc.powerTrouble) {
        Serial.println("Power trouble");
        mqtt.publish(mqttTroubleTopic, "1", true);
      } else {
        Serial.println("Power restored");
        mqtt.publish(mqttTroubleTopic, "0", true);
      }
    }

    if (dsc.batteryChanged || force_send_mqtt) {
      dsc.batteryChanged = false;  // Resets the battery trouble status flag
      if (dsc.batteryTrouble) {
        Serial.println("Battery trouble");
        mqtt.publish(mqttTroubleTopic, "1", true);
      } else {
        Serial.println("Battery restored");
        mqtt.publish(mqttTroubleTopic, "0", true);
      }
    }

    if (force_send_status_for_new_client) {
      force_send_status_for_new_client = false;
    }

    if (force_send_mqtt) {
      force_send_mqtt = false;
    }

  }
}

void ISRwatchdog() {
  if (millis() - watchdog > 30000) {
    Serial.println("Watch Dog RESET!");
    ESP.reset();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  WiFi.hostname(dnsHostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) delay(100);
  Serial.print(F("WiFi connected: "));
  Serial.println(WiFi.localIP());

  if (!MDNS.begin(dnsHostname)) {
    Serial.println("Error setting up MDNS responder.");
    while (1) {
      delay(1000);
    }
  }

  mqtt.setCallback(mqttCallback);
  if (mqttConnect()) {
    mqttPreviousTime = millis();
  } else {
    mqttPreviousTime = 0;
  }

  SPIFFS.begin();
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setAuthentication(http_username, http_password);
  server.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.print(F("Web server started: http://"));
  Serial.print(dnsHostname);
  Serial.println(F(".local"));

  dsc.begin();
  ws_ping_pong.stop();

  watchdog = millis();
  secondTick.attach(1, ISRwatchdog);

  Serial.println(F("DSC Keybus Interface is online."));
}

void loop() {

  yield(); // For ESP8266 to not crash

  MDNS.update();
  mqttHandle();

  //ping-pong WebSocket to keep connection open
  if (ws_ping_pong.isRunning() && ws_ping_pong.elapsed() > 5 * 60) {
    ws.pingAll();
    ws_ping_pong.restart();
  }

  handleDscChanges();

  watchdog = millis();

}
