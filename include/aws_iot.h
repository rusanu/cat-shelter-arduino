#pragma once

void setupAwsIot();
void loopAwsIot();
bool isIotConnected();
bool IoTPublish(const String& topic, const String& payload, bool retained, int qos);
bool IoTPublish(const String& topic, const JsonDocument& payload, bool retained, int qos);

String buildTopicName(const String& topic);
