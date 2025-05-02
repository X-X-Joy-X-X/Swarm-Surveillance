#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_camera.h>
#include <demoday_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <HardwareSerial.h>
#include <AsyncUDP.h>
#include <esp_crc.h> // ESP32 CRC library

// Add this at the top for debug formatting
#define DEBUG_TAG "[DRONE]"
#define DEBUG_TIME() millis()
#define DEBUG_LOG(fmt, ...) Serial.printf("%s [%lu] " fmt "\n", DEBUG_TAG, DEBUG_TIME(), ##__VA_ARGS__)

// ======================= Configuration ======================= 
// Camera pins (AI-Thinker)
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

// WiFi
const char* ssid = "By the eye of Agamotto";
const char* password = "Barno1234";
const String serverURL = "http://192.168.0.214:5500";
const char* googleApiKey = "AIzaSyATU6D1iIkLr32xEzLEFDmKAT-gh4VD2eg";

// Movement
const int ledPin = 4;
//HardwareSerial Serial1(1);  // UART1 (RX1=16, TX1=17)
const float metersPerMove = 0.1;
const float metersPerDegree = 111111.0;
volatile bool personDetected = false;
portMUX_TYPE detectionMux = portMUX_INITIALIZER_UNLOCKED;

// Geolocation
double currentLat = 0.0;
double currentLng = 0.0;
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

// Camera config
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3

// RTOS
TaskHandle_t movementTaskHandle = NULL;

SemaphoreHandle_t camSemaphore;
SemaphoreHandle_t sendSemaphore;
SemaphoreHandle_t moveSemaphore;
SemaphoreHandle_t camMux;
SemaphoreHandle_t moveTrigger = NULL;
QueueHandle_t imgQ;
static uint8_t *snapshot_buf = nullptr;
static uint8_t *jpeg_buffer  = nullptr;

// Camera structure
typedef struct { 
  uint8_t *data; 
  size_t len; 
  double lat;
  double lng;
} ImageBuf;
ImageBuf ib;
camera_config_t camCfg = {
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

// ======================= Setup ======================= 
void setup() {
  Serial.begin(115200);
 // Serial1.begin(9600, SERIAL_8N1, 16, 17);
  
  // Initialize camera
    DEBUG_LOG("Initializing camera subsystem");

  if(!psramFound()) { 
    Serial.println("PSRAM not available!");
    while(true);
  }
  
  snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS*EI_CAMERA_RAW_FRAME_BUFFER_ROWS*EI_CAMERA_FRAME_BYTE_SIZE);
  jpeg_buffer  = (uint8_t*)ps_malloc(200000);
  
  if(esp_camera_init(&camCfg) != ESP_OK) {
    Serial.println("Camera init failed!");
    while(true);
  }

  // Connect to WiFi
  DEBUG_LOG("Connecting to wifi");

  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    ESP.restart();
  }
  DEBUG_LOG("✅ Wifi connected");
  digitalWrite(ledPin, HIGH);

  // Get initial location
  getInitialLocation();
   //ImageBuf ib;
  // Create RTOS resources
  camMux = xSemaphoreCreateMutex();
  moveTrigger = xSemaphoreCreateBinary();
   camSemaphore = xSemaphoreCreateBinary();
  sendSemaphore = xSemaphoreCreateBinary();
  moveSemaphore = xSemaphoreCreateBinary();
  imgQ = xQueueCreate(4, sizeof(ImageBuf));
  xSemaphoreGive(camSemaphore);
  // Create tasks
  xTaskCreatePinnedToCore(cameraTask, "CAM", 16384, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(sendTask, "SEND", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(movementTask, "MOVE", 4096, NULL, 1, NULL, 1);
}

void loop() { 
  vTaskDelete(NULL); // Free up loop() since we're using RTOS tasks
}

// ======================= Movement Task ======================= 
void movementTask(void *pvParams) {
  while (1) {
    pinMode(4,OUTPUT);
    // Wait until sendTask gives the go-ahead
    if (xSemaphoreTake(moveSemaphore, portMAX_DELAY)) {
            DEBUG_LOG("🚚 Movement task started");

      bool shouldMove = false;
      portENTER_CRITICAL(&detectionMux);
      shouldMove = !personDetected;
      portEXIT_CRITICAL(&detectionMux);
      uint32_t randomValue = esp_random();
      char signal = (randomValue % 10 < 7) ? 'F' : 'L';
      if (shouldMove) {

                if (signal == 'L'){
        Serial1.print('L');
        DEBUG_LOG("✅ Move Sent -> L");
       
        if (waitForAck()) {
          portENTER_CRITICAL(&gpsMux);
          currentLat += metersPerMove / metersPerDegree;
          portEXIT_CRITICAL(&gpsMux);
          DEBUG_LOG("✅ New position: %.6f, %.6f", currentLat, currentLng);
        }}  
        if (signal == 'F'){
        Serial1.print('F');
        DEBUG_LOG("✅ Move Sent -> F");
       
        if (waitForAck()) {
          portENTER_CRITICAL(&gpsMux);
          currentLat += metersPerMove / metersPerDegree;
          portEXIT_CRITICAL(&gpsMux);
          DEBUG_LOG("✅ New position: %.6f, %.6f", currentLat, currentLng);
        }}
         DEBUG_LOG("🔄 Restarting camera cycle");
      xSemaphoreGive(camSemaphore);
      }else{
        signal = 'S';
         if (signal == 'S'){
        Serial1.print('S');
        DEBUG_LOG("✅ Move Sent -> S");
       
        if (waitForAck()) {
          portENTER_CRITICAL(&gpsMux);
          currentLat += metersPerMove / metersPerDegree;
          portEXIT_CRITICAL(&gpsMux);
          DEBUG_LOG("✅ New position: %.6f, %.6f", currentLat, currentLng);
        }}
      DEBUG_LOG("🔄 Restarting camera cycle");
      xSemaphoreGive(camSemaphore);}
    }
        vTaskDelay(10 / portTICK_PERIOD_MS);

  }
}

/* ======================= Camera Task ======================= 
void cameraTask(void *pvParams) {
  uint32_t frameID = 0;
  //DEBUG_LOG("🔃 Grabbing Frame %lu", frameID );

  while(1) {
    if(xSemaphoreTake(camSemaphore, portMAX_DELAY)) {
      DEBUG_LOG("📸 Camera task started");
              camera_fb_t *fb = esp_camera_fb_get();

      if(xSemaphoreTake(camMux, portMAX_DELAY)) {
        if(fb) {
      if(fb) {
        bool detect;
        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT*EI_CAMERA_FRAME_BYTE_SIZE;
        signal.get_data = &ei_camera_get_data;

        // Run inference
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
        
        if(err == EI_IMPULSE_OK) {
          for(size_t i = 0; i < result.bounding_boxes_count; i++) {
            if(result.bounding_boxes[i].value > 0.2) {
              Serial.println(result.bounding_boxes[i].value);
              detect = true;
              break;
            }
          }
        }

        // Handle detection
        portENTER_CRITICAL(&detectionMux);
        personDetected = detect;
        portEXIT_CRITICAL(&detectionMux);

        if(detect) {
   
         // ImageBuf ib;
          ib.data = (uint8_t*)ps_malloc(fb->len);
          memcpy(ib.data, fb->buf, fb->len);
          ib.len = fb->len;
          portENTER_CRITICAL(&gpsMux);
          ib.lat = currentLat;
          ib.lng = currentLng;
          portEXIT_CRITICAL(&gpsMux);
          
          if(xQueueSend(imgQ, &ib, 0) != pdTRUE) {
            free(ib.data);
          }
                 //DEBUG_LOG("✅ Person Detected");
          DEBUG_LOG("✅ Person detected, triggering send task");
          xSemaphoreGive(sendSemaphore);
        }else{ DEBUG_LOG("❌ Person NOT detected, triggering send task");
          xSemaphoreGive(sendSemaphore);
        }
        }
      }
        esp_camera_fb_return(fb);
      }
      xSemaphoreGive(camMux);
    }}
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
*/

void cameraTask(void *pvParams) {
  uint32_t frameID = 0;
  while(1) {
    if(xSemaphoreTake(camSemaphore, portMAX_DELAY)) {
      camera_fb_t *fb = NULL;
      if(xSemaphoreTake(camMux, portMAX_DELAY)) {
        fb = esp_camera_fb_get();
        if(fb) {
          // Decode JPEG to RGB
          bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
          if (!converted) {
            Serial.println("JPEG to RGB conversion failed");
            esp_camera_fb_return(fb);
            xSemaphoreGive(camMux);
            continue;
          }

          // Run inference
          ei::signal_t signal;
          signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * EI_CAMERA_FRAME_BYTE_SIZE;
          signal.get_data = &ei_camera_get_data;

          ei_impulse_result_t result = {0};
          EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
          bool detect = true;

          if(err == EI_IMPULSE_OK) {
            for(size_t i = 0; i < result.bounding_boxes_count; i++) {
              if(result.bounding_boxes[i].value > 0.2) {
                Serial.println(result.bounding_boxes[i].value);
                detect = true;
                break;
              }
            }
          }

          // Handle detection
          portENTER_CRITICAL(&detectionMux);
          personDetected = detect;
          portEXIT_CRITICAL(&detectionMux);

          if(detect) {
            ib.data = (uint8_t*)ps_malloc(fb->len);
            if(ib.data == NULL) {
              Serial.println("Failed to allocate image buffer");
            } else {
              memcpy(ib.data, fb->buf, fb->len);
              ib.len = fb->len;
              portENTER_CRITICAL(&gpsMux);
              ib.lat = currentLat;
              ib.lng = currentLng;
              portEXIT_CRITICAL(&gpsMux);
              
              if(xQueueSend(imgQ, &ib, 0) != pdTRUE) {
                free(ib.data);
              }
              DEBUG_LOG("✅ Person detected, triggering send task");
            }
          } else { 
            DEBUG_LOG("❌ Person NOT detected, triggering send task");
          }
          
          // Always return frame buffer and give semaphore
          esp_camera_fb_return(fb);
          xSemaphoreGive(sendSemaphore);
        }
        xSemaphoreGive(camMux);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
// ======================= Send Task ======================= 
  
  
  
  
  void sendTask(void *pvParams) {
    const String myID = "DRONE-" + WiFi.macAddress();
  TickType_t lastReg = xTaskGetTickCount();
  
  while(1) {
    if(xSemaphoreTake(sendSemaphore, portMAX_DELAY)) {
      DEBUG_LOG("📡 Send task started");
      if(xTaskGetTickCount() - lastReg > pdMS_TO_TICKS(10000)) {
            registerDrone(myID);
            lastReg = xTaskGetTickCount();
          }
     
      if(xQueueReceive(imgQ, &ib, pdMS_TO_TICKS(1000))) {
        // ... existing upload code ...
            uint32_t checksum = esp_crc32_le(0, ib.data, ib.len);

        HTTPClient http;
    http.begin(serverURL + "/upload_image");
    http.addHeader("Content-Type", "image/jpeg");
    http.addHeader("Drone-ID", myID);
    http.addHeader("X-Latitude", String(ib.lat, 6));
    http.addHeader("X-Longitude", String(ib.lng, 6));
    http.addHeader("X-Checksum", String(checksum)); // Add this line
      
      int httpCode = http.POST(ib.data, ib.len);
        if(httpCode == HTTP_CODE_OK) {          
          DEBUG_LOG("✅ Upload success, triggering movement");
          xSemaphoreGive(moveSemaphore);
        }
      }}DEBUG_LOG("❌ Upload Failed, triggering movement");
          xSemaphoreGive(moveSemaphore);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}



// ======================= Helper Functions ======================= 
bool waitForAck() {
  unsigned long start = millis();
  while(millis() - start < 5000) {
    if(Serial1.available()) {
      String response = Serial1.readStringUntil('\n');
      if(response.startsWith("ACKF")||response.startsWith("ACKL")) return true;
    }
  }
  return false;
}

void getInitialLocation() {
  String payload = createScanPayload();
  if(payload.length() > 0) {
    HTTPClient http;
    http.begin("https://www.googleapis.com/geolocation/v1/geolocate?key=" + String(googleApiKey));
    http.addHeader("Content-Type", "application/json");
    
    int httpCode = http.POST(payload);
    if(httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, response);
      currentLat = doc["location"]["lat"];
      currentLng = doc["location"]["lng"];
      Serial.printf("Initial location: %.6f,%.6f\n", currentLat, currentLng);
    }
    http.end();
  }
}

String createScanPayload() {
  int scanResult = WiFi.scanNetworks(false, false, false, 300);
  if(scanResult == WIFI_SCAN_FAILED) return "";

  DynamicJsonDocument doc(2048);
  doc["considerIp"] = false;
  JsonArray wifiAccessPoints = doc.createNestedArray("wifiAccessPoints");

  for(int i = 0; i < scanResult && i < 20; i++) {
    JsonObject ap = wifiAccessPoints.createNestedObject();
    ap["macAddress"] = WiFi.BSSIDstr(i);
    ap["signalStrength"] = WiFi.RSSI(i);
    if(WiFi.channel(i) > 0) ap["channel"] = WiFi.channel(i);
  }

  String payload;
  serializeJson(doc, payload);
  WiFi.scanDelete();
  return payload;
}

void registerDrone(const String& id) {
  HTTPClient http;
  http.begin(serverURL + "/register");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"drone_id\":\"" + id + "\"}");
  if(code == HTTP_CODE_OK) {
    Serial.println("Registration successful");
    DEBUG_LOG("✅ Registration Successfull");

  }else{Serial.println("Registration Failed");
    DEBUG_LOG("❌ Registration Failed");}
  http.end();
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  
  while(pixels_left != 0) {
    out_ptr[0] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];
    out_ptr++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}
