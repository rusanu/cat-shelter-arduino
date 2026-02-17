#include <ArduinoJson.h>
#include"common.h"
#include "live_photo.h"
#include "aws_iot.h"
#include "secrets.h"
#include "json_config.h"

#define LIVEPHOTO_STREAM_MAX_MS 5*60*1000
#define LIVEPHOTO_STREAM_FPS_MS 1000

void LivePhoto::Start() {
    _isStreaming = true;
    _streamStartedMs = millis();
}

void LivePhoto::Stop() {
    _isStreaming = false;
}
    
void LivePhoto::Loop() {
    if (!_isStreaming) {
        return;
    }

    unsigned long now = millis();

    if (now - _streamStartedMs > LIVEPHOTO_STREAM_MAX_MS) {
        Stop();
        return;
    }

    if (cameraAvailable && 
        (_lastPhotoMs == 0 || now - _lastPhotoMs >= LIVEPHOTO_STREAM_FPS_MS)) {
        flashOn();

        camera_fb_t *fb = esp_camera_fb_get();
        flashOff();
        if (fb) {
            // Generate base filename with timestamp (without extension)
            String baseFilename = "cat_" + getTimestamp();
            String photoFilename = baseFilename + ".jpg";

            if (uploadPhotoToS3(fb, photoFilename, S3_LIVE_PHOTO)) {
                _lastPhotoMs = millis();
                auto topicName = buildTopicName("live-photo");
                JsonDocument doc;
                doc["id"] = String(S3_LIVE_PHOTO) + "/" + photoFilename;
                doc["status"] = JsonCameraConfig::config.BuildStatus();
                IoTPublish(topicName, doc, false, 0);
            }

            esp_camera_fb_return(fb);
        } 
    }
}
