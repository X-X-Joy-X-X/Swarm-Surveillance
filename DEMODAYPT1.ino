#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_camera.h>
#include <Swarm_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <WebServer.h>
#include <HardwareSerial.h>
#include <AsyncUDP.h>

/* =====================================================================
   ESP32-CAM → Flask server uploader (MERGED VERSION)
   - New model (classification, better training)
   - Old functionality (WiFi, heartbeat, upload)
   ===================================================================== */

// ------------------------------- Configs ------------------------------
#define DEBUG_LOG(msg)        Serial.printf("[%lu] %s\n", millis(), msg)
#define DEBUG_FMT(fmt, ...)   Serial.printf("[%lu] " fmt "\n", millis(), ##__VA_ARGS__)

#define DEBUG_HEAP        true

// Camera pins (AI Thinker ESP32-CAM)
#define PWDN_GPIO 32
#define RESET_GPIO -1
#define XCLK_GPIO 0
#define SIOD_GPIO 26
#define SIOC_GPIO 27
#define Y9_GPIO 35
#define Y8_GPIO 34
#define Y7_GPIO 39
#define Y6_GPIO 36
#define Y5_GPIO 21
#define Y4_GPIO 19
#define Y3_GPIO 18
#define Y2_GPIO 5
#define VSYNC_GPIO 25
#define HREF_GPIO 23
#define PCLK_GPIO 22

// Image settings
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

// Wi-Fi settings
const char* ssid = "By the eye of Agamotto";
const char* password = "Barno1234";
const String serverURL = "http://192.168.114.214:5500";

// Global Buffers
static uint8_t *snapshot_buf = nullptr;
static uint8_t *jpeg_buffer = nullptr;
static size_t jpeg_length = 0;

static SemaphoreHandle_t camMux;
typedef struct { uint8_t *data; size_t len; } ImageBuf;
static QueueHandle_t imgQ;

static camera_config_t camCfg = {
  .pin_pwdn = PWDN_GPIO, .pin_reset = RESET_GPIO, .pin_xclk = XCLK_GPIO,
  .pin_sscb_sda = SIOD_GPIO, .pin_sscb_scl = SIOC_GPIO,
  .pin_d7 = Y9_GPIO, .pin_d6 = Y8_GPIO, .pin_d5 = Y7_GPIO, .pin_d4 = Y6_GPIO,
  .pin_d3 = Y5_GPIO, .pin_d2 = Y4_GPIO, .pin_d1 = Y3_GPIO, .pin_d0 = Y2_GPIO,
  .pin_vsync = VSYNC_GPIO, .pin_href = HREF_GPIO, .pin_pclk = PCLK_GPIO,
  .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA,
  .jpeg_quality = 12, .fb_count = 1, .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_LATEST
};

// ------------------------------- Declarations ------------------------------
static int ei_camera_get_data(size_t, size_t, float*);
bool ei_camera_capture(uint32_t, uint32_t, uint8_t*);
uint32_t crc32(const uint8_t*, size_t);
void sendImage(const uint8_t*, size_t, const String&);
void registerDrone(const String&);

// ------------------------------- Setup -------------------------------------
void setup() {
  Serial.begin(115200);
  delay(400);
  DEBUG_LOG("Booting...");

  if (!psramFound()) {
    DEBUG_LOG("❌ PSRAM missing");
    while (true);
  }
  snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  jpeg_buffer = (uint8_t*)ps_malloc(200000);

  if (esp_camera_init(&camCfg) != ESP_OK) {
    DEBUG_LOG("CAM INIT FAIL");
    while (true);
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  DEBUG_FMT("IP: %s", WiFi.localIP().toString().c_str());

  const String myID = "DRONE-" + WiFi.macAddress();
  registerDrone(myID);

  camMux = xSemaphoreCreateMutex();
  imgQ = xQueueCreate(4, sizeof(ImageBuf));

  xTaskCreatePinnedToCore(camTask, "CAM", 16384, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sendTask, "SEND", 8192, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

// ------------------------------- Camera Task -------------------------------
void camTask(void*) {
  uint32_t frameID = 0;
  for (;;) {
    if (xSemaphoreTake(camMux, portMAX_DELAY)) {
      if (ei_camera_capture(96, 96, snapshot_buf)) {
        frameID++;

        ei::signal_t sig;
        sig.total_length = 96 * 96;
        sig.get_data = &ei_camera_get_data;
        ei_impulse_result_t res = {0};
        EI_IMPULSE_ERROR e = run_classifier(&sig, &res, false);

        bool detect = false;
        if (e == EI_IMPULSE_OK) {
          for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            float score = res.classification[i].value;
            if ((i == 0 || i == 1 || i == 2) && score > 0.5) {
              detect = true;
            }
          }
        }

        DEBUG_FMT("Frame %u detect=%d", frameID, detect);

        if (detect || frameID % 30 == 0) { 
          camera_fb_t *fb = esp_camera_fb_get();
          if (fb) {
            jpeg_length = fb->len;
            memcpy(jpeg_buffer, fb->buf, jpeg_length);
            esp_camera_fb_return(fb);

            uint8_t *cpy = (uint8_t*)ps_malloc(jpeg_length);
            memcpy(cpy, jpeg_buffer, jpeg_length);
            ImageBuf ib = {cpy, jpeg_length};
            if (xQueueSend(imgQ, &ib, 0) != pdTRUE) {
              free(cpy);
            }
          }
        }
      }
      xSemaphoreGive(camMux);
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ------------------------------- Sender Task -------------------------------
void sendTask(void *) {
  const String myID = "DRONE-" + WiFi.macAddress();
  DEBUG_LOG("SEND task running");

  TickType_t lastReg = xTaskGetTickCount();
  const TickType_t regInterval = pdMS_TO_TICKS(10000);

  for (;;) {
    ImageBuf ib;
    if (xQueueReceive(imgQ, &ib, pdMS_TO_TICKS(1000)) == pdTRUE) {
      DEBUG_FMT("Dequeued image %uB", ib.len);
      sendImage(ib.data, ib.len, myID);
      free(ib.data);
    }

    if (xTaskGetTickCount() - lastReg > regInterval) {
      registerDrone(myID);
      lastReg = xTaskGetTickCount();
    }
  }
}

// ------------------------------- Camera Utilities --------------------------
bool ei_camera_capture(uint32_t w, uint32_t h, uint8_t *out) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out);
  esp_camera_fb_return(fb);
  return ok;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) +
                          (snapshot_buf[pixel_ix + 1] << 8) +
                          snapshot_buf[pixel_ix];
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

uint32_t crc32(const uint8_t *d, size_t l) {
  uint32_t c = 0xFFFFFFFF;
  while (l--) {
    c ^= *d++;
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320 & -(c & 1));
  }
  return ~c;
}

void sendImage(const uint8_t *img, size_t sz, const String &id) {
  HTTPClient http;
  http.begin(serverURL + "/upload_image");
  http.addHeader("X-Checksum", String(crc32(img, sz)));
  http.addHeader("Drone-ID", id);
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST((uint8_t*)img, sz);
  if (code == HTTP_CODE_OK) DEBUG_LOG("✅ Upload OK");
  else {
    DEBUG_FMT("❌ Upload failed (%d)", code);
    DEBUG_LOG(http.getString().c_str());
  }
  http.end();
}

void registerDrone(const String& id) {
  HTTPClient http;
  http.begin(serverURL + "/register");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"drone_id\":\"" + id + "\"}");
  if (code == HTTP_CODE_OK) DEBUG_LOG("✅ Registered with server");
  else DEBUG_FMT("❌ Register failed (%d)", code);
  http.end();
}
