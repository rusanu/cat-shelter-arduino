#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include "common.h"
#include "aws_iot.h"
#include "secrets.h"
#include "live_photo.h"

// MQTT configuration
#define MQTT_BUFFER_SIZE 1024
#define MQTT_KEEPALIVE_SEC 60
#define IOT_RECONNECT_MAX_DELAY 120000   // 2 minutes max backoff
#define IOT_RECONNECT_MIN_DELAY 5000     // 5 seconds initial
#define IOT_STATUS_PUBLISH_INTERVAL 300000  // 5 minutes

// Topic strings (built in setup)
static String topicCommands;

static String clientId;

static LivePhoto livePhoto;

// Module state
static WiFiClientSecure tlsClient;
static MQTTClient mqttClient(MQTT_BUFFER_SIZE);
static BackOffRetry mqttRetry(IOT_RECONNECT_MAX_DELAY, IOT_RECONNECT_MIN_DELAY);
static bool iotInitialized = false;
static unsigned long lastStatusPublish = 0;

// Forward declarations
static void onMqttMessage(String &topic, String &payload);
static bool mqttConnect();
static void handleIotCommand(const String &payload);

String buildTopicName(const String& topic) {
  return String("cat-shelter/") + deviceName + "/" + topic;
}

// ===== Public API =====

void setupAwsIot() {
  topicCommands = buildTopicName("commands");

  tlsClient.setCACert(AWS_IOT_ROOT_CA);
  tlsClient.setCertificate(AWS_IOT_DEVICE_CERT);
  tlsClient.setPrivateKey(AWS_IOT_DEVICE_PRIVATE_KEY);

  mqttClient.begin(AWS_IOT_ENDPOINT, 8883, tlsClient);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.onMessage(onMqttMessage);

  iotInitialized = true;
  logPrintf(LOG_INFO, "AWS IoT initialized (endpoint: %s, device: %s)", AWS_IOT_ENDPOINT, deviceName);
}

void loopAwsIot() {
  if (!iotInitialized || !IsWiFiConnected()) return;

  if (!mqttClient.connected()) {
    if (mqttRetry.CanRetry()) {
      mqttConnect();
    }
    return;
  }

  
  mqttClient.loop();
  livePhoto.Loop();
}

bool isIotConnected() {
  return iotInitialized && mqttClient.connected();
}

// ===== Connection =====

static bool mqttConnect() {
  logPrintf(LOG_INFO, "MQTT connecting to %s...", AWS_IOT_ENDPOINT);

  auto deviceName = WiFi.getHostname();

  bool ok = mqttClient.connect(deviceName);
  if (!ok) {
    logPrintf(LOG_WARNING, "MQTT connect failed (error: %d)", mqttClient.lastError());
    return false;
  }

  logPrint(LOG_INFO, "MQTT connected");
  mqttRetry.Reset();

  mqttClient.subscribe(topicCommands, 1);
  logPrintf(LOG_INFO, "MQTT subscribed to %s", topicCommands.c_str());
  return true;
}

// ===== Incoming messages =====

static void onMqttMessage(String &topic, String &payload) {
  logPrintf(LOG_INFO, "MQTT message on %s: %s", topic.c_str(), payload.c_str());

  if (topic == topicCommands) {
    handleIotCommand(payload);
  }
}

static void handleIotCommand(const String &payload) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    logPrintf(LOG_ERROR, "IoT command JSON parse error: %s", err.c_str());
    return;
  }

  const char *command = doc["command"];
  const char *commandId = doc["id"] | "unknown";

  if (!command) {
    logPrint(LOG_ERROR, "IoT command missing 'command' field");
    return;
  }

  logPrintf(LOG_INFO, "IoT command: %s (id: %s)", command, commandId);

  if (strcmp(command, "snapshot") == 0) {
    if (cameraAvailable) {
      takeAndUploadPhoto("iot-command");
    }
  }
  else if (strcmp(command, "live-photo") == 0) {
    livePhoto.Start();
  }
  else if (strcmp(command, "reboot") == 0) {
    rebootSystem("IoT reboot command");
  }
  else {
    logPrintf(LOG_WARNING, "Unknown IoT command: %s", command);
  }
}


bool IoTPublish(const String& topic, const String& payload, bool retained, int qos) {
  return  mqttClient.connected() &&
    mqttClient.publish(topic, payload, retained, qos);
}