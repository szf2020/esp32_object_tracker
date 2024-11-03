// refer to https://shawnhymel.com/1675/arduino-websocket-server-using-an-esp32/
// To make sure WebSocketServer.h is available:
//    Sketch > Include Library > Manage Libraries
//    Search for "WebSockets Sattler' and install the library named "WebSockets by Markus Sattler"
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h" // This must come after camera model has been selected

const char* ssid     = "bruhmoment";     // input your wifi name
const char* password = "D0gshit!";   // input your wifi passwords
const char* server_ip = "192.168.188.198"; // input your laptop IP
const int port = 8765; // input port used

// Declare GPIO pin numbers
const int gpioPin1 = 12;
const int gpioPin2 = 13;
const int gpioPin3 = 14;
const int gpioPin4 = 15;

// Variables to store pin states (0: LOW, 1: HIGH)
//int pin1State = 0;
//int pin2State = 0;
//int pin3State = 0;
//int pin4State = 0;
int pin1State = 1;
int pin2State = 1;
int pin3State = 1;
int pin4State = 1;

void setupLedFlash(int pin);

// Globals
WebSocketsClient webSocket;

// WebSocket event handler
void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      break;
    case WStype_TEXT:
      {
        String command = (char*)payload;
        Serial.printf("Movement command: %s\n", command.c_str());
        // Check if command has expected format
        if (command.length() == 2) {
          // Extract speed and direction from command
          char speed = command[0];
          char direction = command[1];

          Serial.printf("Speed: %c, Direction: %c\n", speed, direction);

          // Set pin1 and pin2 based on speed (0-3)
          switch (speed) {
//            case '0':
//              pin1State = 0;
//              pin2State = 0;
//              break;
//            case '1':
//              pin1State = 1;
//              pin2State = 0;
//              break;
//            case '2':
//              pin1State = 0;
//              pin2State = 1;
//              break;
//            case '3':
//              pin1State = 1;
//              pin2State = 1;
//              break;
                        case '0':
                          pin1State = 1;
                          pin2State = 1;
                          break;
                        case '1':
                          pin1State = 0;
                          pin2State = 1;
                          break;
                        case '2':
                          pin1State = 1;
                          pin2State = 0;
                          break;
                        case '3':
                          pin1State = 0;
                          pin2State = 0;
                          break;
            default:
              Serial.println("Invalid speed value.");
          }

          // Set pin3 and pin4 based on direction (L, S, R)
//          if (direction == 'L') {
//            pin3State = 1;
//            pin4State = 0;
//          } else if (direction == 'R') {
//            pin3State = 0;
//            pin4State = 1;
//          } else if (direction == 'S') {
//            pin3State = 0;
//            pin4State = 0;
                      if (direction == 'L') {
                        pin3State = 0;
                        pin4State = 1;
                      } else if (direction == 'R') {
                        pin3State = 1;
                        pin4State = 0;
                      } else if (direction == 'S') {
                        pin3State = 1;
                        pin4State = 1;
          } else {
            Serial.println("Invalid direction value.");
          }
        } else {
          Serial.println("Invalid command format.");
        }
      }
      break;
    case WStype_BIN:
      Serial.println("Binary data received");
      break;
    case WStype_ERROR:
      Serial.println("WebSocket Error");
      break;
    case WStype_PING:
      Serial.println("Ping received");
      break;
    case WStype_PONG:
      Serial.println("Pong received");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Setup Begin");

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
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Msg: Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  // For OV2640, mirror the image and adjust color settings
  s->set_saturation(s, 1);  // Adjust saturation if colors seem off
  s->set_brightness(s, 0);  // Set brightness to neutral (adjust as needed)

  //  // initial sensors are flipped vertically and colors are a bit saturated
  //  if (s->id.PID == OV3660_PID) {
  //    s->set_vflip(s, 1);        // flip it back
  //    s->set_brightness(s, 1);   // up the brightness just a bit
  //    s->set_saturation(s, -2);  // lower the saturation
  //  }
  //

  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  //#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  //  s->set_vflip(s, 1);
  //  s->set_hmirror(s, 1);
  //#endif
  //
  //#if defined(CAMERA_MODEL_ESP32S3_EYE)
  //  s->set_vflip(s, 1);
  //#endif
  //
  //// Setup LED FLash if LED pin is defined in camera_pins.h
  //#if defined(LED_GPIO_NUM)
  //  setupLedFlash(LED_GPIO_NUM);
  //#endif

  Serial.println("Msg: Camera Ready!");

  Serial.print("Connecting WiFi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start WebSocket client and assign callback
  webSocket.begin(server_ip, port, "/");
  webSocket.onEvent(onWebSocketEvent);

  // Setup GPIO pins
  pinMode(gpioPin1, OUTPUT);
  pinMode(gpioPin2, OUTPUT);
  pinMode(gpioPin3, OUTPUT);
  pinMode(gpioPin4, OUTPUT);
  Serial.println("End Setup");
}

void loop() {
  // Look for and handle WebSocket data
  webSocket.loop();

  // Update GPIO pins based on pin states
  digitalWrite(gpioPin1, pin1State);
  digitalWrite(gpioPin2, pin2State);
  digitalWrite(gpioPin3, pin3State);
  digitalWrite(gpioPin4, pin4State);

  // Capture image from cam
  camera_fb_t* fb = esp_camera_fb_get(); // Capture an image
  if (fb) {
    size_t fbsize = fb->len; // Get the image size
    Serial.printf("Captured image with size: %d bytes\n", fbsize);
    webSocket.sendBIN(fb->buf, fbsize); // send image data over WebSocket
    esp_camera_fb_return(fb); // return frame buffer to free up memory
  } else {
    Serial.println("Camera capture failed");
  }

  delay(50); // delay in milliseconds
}
