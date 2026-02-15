#include <WebServer.h>
#include "common.h"
#include "app_httpd.h"


static int portNumber = 80;
bool isStreaming = false;
static loopCallback_t externalLoop = nullptr;

static WebServer webServer(portNumber);


void WiFiHttpdIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    webServer.begin();
    isStreaming = false;
    logPrintf(LOG_INFO, "HTTPD started http://%s:%i", WiFi.localIP().toString().c_str(), portNumber);
}

void WiFiHttpdDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    webServer.stop();
    isStreaming = false;
    logPrintf(LOG_INFO, "HTTPD stopped");
}

void HttpdSendSnapshot() {
    camera_fb_t * fb = NULL;

    fb = capturePhoto();
    if (!fb) {
        webServer.send(500, "text/text", "Camera capture failed");
        return;
    }

    if (fb->format == PIXFORMAT_JPEG) {
        webServer.sendHeader("Content-Disposition", "inline; filename=snapshot.jpg");
        webServer.send_P(200, "image/jpeg", (const char*) fb->buf, fb->len);
    }

    logPrintf(LOG_INFO, "HTTPD sent: %i", fb->len);

    esp_camera_fb_return(fb);

}

#define MJPEG_BOUNDARY "123456789000000000000987654321"
#define MAX_FPS 24

void sendStreamFrame() {
    auto client = webServer.client();
    if (!client.connected()) {
        logPrintf(LOG_INFO, "HTTPD stopped streaming");
        isStreaming = false;
    }
    else
    {
        sensor_t *s = esp_camera_sensor_get();
        s->set_framesize(s, FRAMESIZE_VGA);
        s->set_pixformat(s, PIXFORMAT_JPEG);
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb) {
            client.printf("--" MJPEG_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
            client.write(fb->buf, fb->len);
            client.println("\r\n");            
            esp_camera_fb_return(fb);
        }
    }
}

void HttpdSendStream() {
    flashOn();

    auto client = webServer.client();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY);
    client.println("Connection: keep-alive");
    client.println();    

    isStreaming = true;

    logPrintf(LOG_INFO, "HTTPD started streaming");

    const long msPerFrame = 1000/MAX_FPS;

    while (isStreaming) {
        unsigned long msStart = millis();
        sendStreamFrame();
        externalLoop();
        long msThisFrame = millis() - msStart;
        if (msThisFrame > 0 && msThisFrame < msPerFrame) {
            delay(msPerFrame - msThisFrame);
        }
    }
    flashOff();
}

void HttpdRoot() {
String response = R"EOF(
    <!DOCTYPE html><html>
      <head>
        <title>Cat shelter: live</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
      </head>
            
      <body>
        <h1>Web Server</h1>
          <a href="/stream">Stream</a>
          <a href="/snapshot">Snapshot</a>
        </div>
      </body>
    </html>
  )EOF";
  webServer.send(200, "text/html", response);
}

void InitHttpd() {
    WiFi.onEvent(WiFiHttpdDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(WiFiHttpdIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

    webServer.on("/stream", HTTP_GET, HttpdSendStream);
    webServer.on("/snapshot", HTTP_GET, HttpdSendSnapshot);
    webServer.on("/", HTTP_GET, HttpdRoot);

    logPrintf(LOG_INFO, "HTTPD ready to start");
}

void LoopHttpd(loopCallback_t loopCallback) {
    externalLoop = loopCallback;
    externalLoop();

    webServer.handleClient();
}
