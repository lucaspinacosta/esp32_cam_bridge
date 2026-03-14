#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "esp_camera.h"
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =========================
// Wi-Fi
// =========================
const char* ssid = "2_4_ghz_NETWORK_SSID";
const char* password = "PASSWORD";

// =========================
// Camera pin map (Freenove ESP32-S3 camera board)
// =========================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5

#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13

// =========================
// Onboard RGB LED
// =========================
#define LED_PIN       48
#define LED_COUNT     1
#define LED_CHANNEL   0

Freenove_ESP32_WS2812 led(LED_COUNT, LED_PIN, LED_CHANNEL, TYPE_GRB);

// =========================
// HTTP servers
// =========================
WebServer controlServer(80);
WiFiServer streamServer(81);

// =========================
// LED state
// =========================
int currentR = 0;
int currentG = 0;
int currentB = 0;
int currentBrightness = 20;

// =========================
// Shared camera lock
// =========================
SemaphoreHandle_t cameraMutex = nullptr;

// =========================
// LED helpers
// =========================
void setLedColor(int r, int g, int b, int brightness) {
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  brightness = constrain(brightness, 0, 255);

  currentR = r;
  currentG = g;
  currentB = b;
  currentBrightness = brightness;

  led.setBrightness(currentBrightness);
  led.setLedColorData(0, currentR, currentG, currentB);
  led.show();
}

// =========================
// Camera init
// =========================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_count = 2;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("INIT_FAIL:%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    // s->set_hmirror(s, 1);
    // s->set_vflip(s, 1);
  }

  return true;
}

// =========================
// HTTP helpers
// =========================
bool writeAll(WiFiClient &client, const uint8_t *buf, size_t len) {
  size_t sent = 0;

  while (sent < len) {
    if (!client.connected()) {
      return false;
    }

    size_t written = client.write(buf + sent, len - sent);
    if (written == 0) {
      delay(1);
      continue;
    }

    sent += written;
  }

  return true;
}

bool captureFrame(camera_fb_t **fb) {
  if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  *fb = esp_camera_fb_get();
  xSemaphoreGive(cameraMutex);

  return *fb != nullptr;
}

// =========================
// HTTP handlers (control server)
// =========================
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32-S3 Camera</title></head><body>"
    "<h2>ESP32-S3 Camera + LED Control</h2>"
    "<p><a href='/jpg'>Single JPEG</a></p>"
    "<p><a href='/led?r=255&g=255&b=255&brightness=50'>Set LED White</a></p>"
    "<p><a href='/led?r=255&g=0&b=0&brightness=80'>Set LED Red</a></p>"
    "<p>MJPEG stream: <a href='http://" + WiFi.localIP().toString() + ":81/stream'>open</a></p>"
    "<img src='http://" + WiFi.localIP().toString() + ":81/stream' style='max-width:100%;height:auto;' />"
    "</body></html>";

  controlServer.send(200, "text/html", html);
}

void handleJpg() {
  camera_fb_t *fb = nullptr;
  if (!captureFrame(&fb)) {
    controlServer.send(500, "text/plain", "Camera capture failed");
    return;
  }

  WiFiClient client = controlServer.client();

  client.printf(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n\r\n",
    fb->len
  );

  writeAll(client, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  client.stop();
}

void handleLed() {
  int r = currentR;
  int g = currentG;
  int b = currentB;
  int brightness = currentBrightness;

  if (controlServer.hasArg("r")) r = controlServer.arg("r").toInt();
  if (controlServer.hasArg("g")) g = controlServer.arg("g").toInt();
  if (controlServer.hasArg("b")) b = controlServer.arg("b").toInt();
  if (controlServer.hasArg("brightness")) brightness = controlServer.arg("brightness").toInt();

  setLedColor(r, g, b, brightness);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"r\":" + String(currentR) + ",";
  json += "\"g\":" + String(currentG) + ",";
  json += "\"b\":" + String(currentB) + ",";
  json += "\"brightness\":" + String(currentBrightness);
  json += "}";

  controlServer.send(200, "application/json", json);
}

void handleLedOff() {
  setLedColor(0, 0, 0, 0);
  controlServer.send(200, "application/json", "{\"ok\":true,\"off\":true}");
}

void handleStatus() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"psram\":\"" + String(psramFound() ? "YES" : "NO") + "\",";
  json += "\"stream_port\":81,";
  json += "\"r\":" + String(currentR) + ",";
  json += "\"g\":" + String(currentG) + ",";
  json += "\"b\":" + String(currentB) + ",";
  json += "\"brightness\":" + String(currentBrightness);
  json += "}";

  controlServer.send(200, "application/json", json);
}

void handleStreamInfo() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"stream_url\":\"http://" + WiFi.localIP().toString() + ":81/stream\"";
  json += "}";

  controlServer.send(200, "application/json", json);
}

// =========================
// MJPEG stream task
// =========================
void streamTask(void *pvParameters) {
  while (true) {
    WiFiClient client = streamServer.available();

    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    Serial.println("STREAM_CLIENT_CONNECTED");

    client.setTimeout(5);
    client.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Pragma: no-cache\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "\r\n"
    );

    while (client.connected()) {
      camera_fb_t *fb = nullptr;
      if (!captureFrame(&fb)) {
        vTaskDelay(pdMS_TO_TICKS(30));
        continue;
      }

      client.printf(
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %u\r\n\r\n",
        fb->len
      );

      bool ok = writeAll(client, fb->buf, fb->len);
      ok = ok && client.print("\r\n");

      esp_camera_fb_return(fb);

      if (!ok) {
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
    }

    client.stop();
    Serial.println("STREAM_CLIENT_DISCONNECTED");
  }
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1500);

  led.begin();
  setLedColor(0, 0, 0, 0);

  Serial.println("VISION_BOOT");
  Serial.printf("PSRAM:%s\n", psramFound() ? "YES" : "NO");

  cameraMutex = xSemaphoreCreateMutex();
  if (cameraMutex == nullptr) {
    Serial.println("MUTEX_INIT_FAIL");
    setLedColor(255, 0, 0, 120);
    while (true) {
      delay(1000);
    }
  }

  if (!initCamera()) {
    Serial.println("VISION_INIT_FAIL");
    setLedColor(255, 0, 0, 120);
    while (true) {
      delay(1000);
    }
  }

  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WIFI_READY");
  Serial.print("IP:");
  Serial.println(WiFi.localIP());

  controlServer.on("/", HTTP_GET, handleRoot);
  controlServer.on("/jpg", HTTP_GET, handleJpg);
  controlServer.on("/led", HTTP_GET, handleLed);
  controlServer.on("/led/off", HTTP_GET, handleLedOff);
  controlServer.on("/status", HTTP_GET, handleStatus);
  controlServer.on("/stream", HTTP_GET, handleStreamInfo);

  controlServer.begin();
  streamServer.begin();

  xTaskCreatePinnedToCore(
    streamTask,
    "streamTask",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );

  setLedColor(255, 255, 255, 20);
  Serial.println("VISION_READY");
  Serial.printf("STREAM_URL:http://%s:81/stream\n", WiFi.localIP().toString().c_str());
}

// =========================
// Main loop
// =========================
void loop() {
  controlServer.handleClient();
  delay(2);
}
