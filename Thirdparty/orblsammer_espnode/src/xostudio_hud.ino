// ESP32-C3 WROOM 1 board
// Arduino IDE 2.3.4

#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <basicMPU6050.h>
#include <accIntegral.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <SD_MMC.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
// Note: Wifi can be unstable which causes tcp/udp errors


#define SD_MMC_CMD 38 //Please do not modify it.
#define SD_MMC_CLK 39 //Please do not modify it.
#define SD_MMC_D0 40 //Please do not modify it.

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
#include "camera_pins.h"

// ===========================
// WiFi credentials
// ===========================
// Will be read from SD card if available, otherwise use these defaults
String ssid = "";
String password = "";


#define TCP_PORT 11212
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

int mdns_started = 0;


// Packet Type Indicators
typedef enum {
  PACKET_TYPE_IMAGE   = 0x01,  // ESP32 → host: JPEG frame
  PACKET_TYPE_IMU     = 0x02,  // ESP32 → host: accumulated IMU frames
  PACKET_TYPE_TRIGGER = 0x03,  // host → ESP32: request a frame capture (1 byte, no header)
} packet_type_t;

// TCP Packet Header Structure (Similar to UDP for consistency, offset/data_len less critical for TCP stream)
typedef struct __attribute__((packed)) {
  uint8_t  packet_type; // Type of data in the packet
  uint32_t frame_time; // Time of frame capture
  uint32_t total_size;   // Total size of the complete frame data
} tcp_packet_header_t;

// I2C pins for SeedStudio ESP32C3
const int SDA_PIN = 21;  // Default SDA pin
const int SCL_PIN = 20;  // Default SCL pin


// Time tracking for integration
unsigned long previousTime = 0;
float deltaTime = 0.0;

bool isCalibrated = false;

// Constants
const float DEG_TO_RADIAN = 0.0174533;  // π/180
const float RAD_TO_DEGREE = 57.2958;  // 180/π

uint32_t loopIndex = 0;

#define MAX_IMU_FRAMES 128
#define IMU_FRAME_SIZE 6 // roll, pitch, yaw, vx, vy, vz
float imuFramesBuffer[MAX_IMU_FRAMES*IMU_FRAME_SIZE]; // Buffer to store IMU frames
int currentIMUFrameIndex = 0;

// How often to flush accumulated IMU frames to the host (milliseconds).
#define IMU_SEND_INTERVAL_MS 50
unsigned long lastImuSendTime = 0;

#define VELOCITY_MIN 100

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 4500;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 2100;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 13536;

//-- Set template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;

// =========== Settings ===========
accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

#define UDP_DISCOVERY_PORT 11211
WiFiUDP udp;
          

void quaternionToEuler();

// ---- TCP helpers -----------------------------------------------------------

// Send all buffered IMU frames to the connected client, then reset the buffer.
// Payload: N × 6 × float32, little-endian (same layout as imuFramesBuffer).
void sendImuFrames() {
  if (currentIMUFrameIndex == 0) return;

  uint32_t payloadSize = currentIMUFrameIndex * IMU_FRAME_SIZE * sizeof(float);
  tcp_packet_header_t hdr;
  hdr.packet_type = PACKET_TYPE_IMU;
  hdr.frame_time  = millis();
  hdr.total_size  = payloadSize;

  if (tcpClient.write((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr)) {
    Serial.println("IMU header send error");
    tcpClient.stop();
    return;
  }
  if (tcpClient.write((uint8_t*)imuFramesBuffer, payloadSize) != payloadSize) {
    Serial.println("IMU data send error");
    tcpClient.stop();
    return;
  }
  Serial.printf("Sent %d IMU frames\n", currentIMUFrameIndex);
  currentIMUFrameIndex = 0;
  lastImuSendTime = millis();
}

// Capture one JPEG frame and send it to the connected client.
void sendCameraFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  tcp_packet_header_t hdr;
  hdr.packet_type = PACKET_TYPE_IMAGE;
  hdr.frame_time  = millis();
  hdr.total_size  = fb->len;

  bool ok = (tcpClient.write((uint8_t*)&hdr, sizeof(hdr)) == sizeof(hdr));
  if (ok) {
    const size_t CHUNK = 1024;
    uint8_t* ptr = fb->buf;
    size_t remaining = fb->len;
    while (ok && remaining > 0) {
      size_t n = (remaining < CHUNK) ? remaining : CHUNK;
      if (tcpClient.write(ptr, n) != n) {
        ok = false;
      } else {
        ptr += n;
        remaining -= n;
        if (remaining > 0) delay(5);
      }
    }
  }
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("Image send error");
    tcpClient.stop();
  } else {
    Serial.printf("Sent frame (%u bytes)\n", fb->len);
  }
}

void printVector( vec3_t r ) {
  Serial.print( r.x, 2 );
  Serial.print( "," );
  Serial.print( r.y, 2 );
  Serial.print( "," );
  Serial.print( r.z, 2 );
}

uint8_t localAddressBuffer[256];

void loop() {

  // Check for new TCP clients
  if (tcpServer.hasClient()) {
    // If a client is already connected, disconnect it first
    if (tcpClient && tcpClient.connected()) {
      Serial.println("Disconnecting existing TCP client.");
      tcpClient.stop();
    }
    tcpClient = tcpServer.available();
    if (tcpClient) {
      Serial.println("New TCP client connected.");
    }
  }

  // Calculate delta time in seconds
  unsigned long currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0;  // Convert microseconds to seconds
  previousTime = currentTime;
  
  // known measured velocity (target state). Estimate will be forced towards this vector
  vec3_t vel_t = {0,0,0};


  // Calibrate if needed
  if (!isCalibrated) {
      imu.setBias();
      isCalibrated=true;
  } else {
    imu.updateBias();

    loopIndex++;

    // Measure state:  
    vec3_t accel = { imu.ax(), imu.ay(), imu.az() };    // g-unit
    vec3_t gyro = { imu.gx(), imu.gy(), imu.gz() };     // radians/second
  
    // Update heading and velocity estimate:
    
    vel_t /= GRAVITY;                         // must have unit: g-force * second
    
        /* note: all coefficients are optional and have default values */
    fusion.update( gyro, accel, vel_t, SD_ACC, SD_VEL, ALPHA ); 

        // obtain velocity estimate
    vec3_t vel = fusion.getVel() * GRAVITY;   // scale by gravity for desired units

    if ( currentIMUFrameIndex > MAX_IMU_FRAMES -1 ) {
        currentIMUFrameIndex = 0;
        Serial.println("IMU buffer overflow, resetting index.");
    }

    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE] = fusion.roll();
    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE+1] = fusion.pitch();
    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE+2] = fusion.yaw();
    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE+3] = vel.x;
    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE+4] = vel.y;
    imuFramesBuffer[currentIMUFrameIndex*IMU_FRAME_SIZE+5] = vel.z;    
    currentIMUFrameIndex++;


    if ( loopIndex % 20 == 0 ) 
    {
        Serial.print("Roll: ");
        Serial.print(fusion.roll());
        Serial.print(" Pitch: ");
        Serial.print(fusion.pitch());
        Serial.print(" Yaw: ");
        Serial.print(fusion.yaw());

        // Display vectors:
        Serial.print( " vel = " );
        printVector( vel );
        Serial.print(" raw ");
        Serial.print(imu.rawAx());
        Serial.print(",");
        Serial.print(imu.rawAy());
        Serial.print(",");
        Serial.print(imu.rawAz());
        Serial.println();

    } 

    if ( loopIndex % 100 == 0 ) {
      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.localIP());
      Serial.print("' to connect");
      Serial.print("   mdns: ");
      Serial.println(mdns_started);
      
      
      udp.beginPacket(WiFi.broadcastIP(), UDP_DISCOVERY_PORT);
      udp.write((uint8_t *)WiFi.localIP().toString().c_str(), WiFi.localIP().toString().length());
      udp.endPacket();
    }
  }

  // --- Persistent TCP session -------------------------------------------------
  if (tcpClient && tcpClient.connected()) {

    // 1. Check for a trigger byte from the host.
    //    The host sends PACKET_TYPE_TRIGGER (0x03) to request one JPEG frame.
    while (tcpClient.available()) {
      uint8_t cmd = tcpClient.read();
      if (cmd == PACKET_TYPE_TRIGGER) {
        sendCameraFrame();
      }
    }

    // 2. Flush accumulated IMU frames at the configured interval.
    if (isCalibrated && (millis() - lastImuSendTime >= IMU_SEND_INTERVAL_MS)) {
      sendImuFrames();
    }

  } else {
    if (tcpClient) {
      Serial.println("TCP client disconnected.");
      tcpClient.stop();
    }
  }
  // --- End TCP session -------------------------------------------------------

  delay(5);
}

#define CAM_MODULE_OV5640 1

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.println("Start Gyro");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C clock
  
  delay(100);  // Short delay for sensors to power up

  imu.setup();
  fusion.setup();

  Serial.println("Initialization complete");
  Serial.println("Starting calibration. Keep device still...");
  delay(5000);  // Allow time for the user to place the device still
  
  // Initialize time tracking
  previousTime = micros();


  uint8_t cardType = CARD_NONE;

  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD,SD_MMC_D0);
  if ( !SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)){
    Serial.println("Could not mount sdcard");
  } else {
    cardType = SD_MMC.cardType();
  }

  Serial.print("SD card is: ");
  Serial.println(cardType);

  if ( cardType != CARD_NONE ) {
    // List files on SD card root
    Serial.println("Listing files on SD card root:");
    File root = SD_MMC.open("/");
    if(!root){
        Serial.println("Failed to open directory");
    } else {
        Serial.println("Listing files in /");
        File file = root.openNextFile();
        while(file){
            if(!file.isDirectory()){
                Serial.print("  FILE: ");
                Serial.print(file.name());
                Serial.print("\tSIZE: ");
                Serial.println(file.size());
            } else {
                Serial.print("  DIR : ");
                Serial.println(file.name());
            }
            file.close(); // Close the file handle
            file = root.openNextFile();
        }
        root.close(); // Close the root directory handle
    }
    Serial.println("---------------------------");


    // Now try to read the config file
    File configFile = SD_MMC.open("/config.txt", FILE_READ);
    if ( configFile ) {
      Serial.println("Reading WiFi config from /config.txt");
      if (configFile.available()) {
        ssid = configFile.readStringUntil('\n');
        ssid.trim(); // Remove potential trailing newline/whitespace
        Serial.print("SSID read: ");
        Serial.println(ssid);
      }
      if (configFile.available()) {
        password = configFile.readStringUntil('\n');
        password.trim(); // Remove potential trailing newline/whitespace
        Serial.println("Password read."); // Don't print password to serial
      }
      configFile.close();

    } else {
      Serial.println("Failed to open /config.txt for reading.");
    }
  } else {
    Serial.println("No SD card or config file found, using default WiFi credentials.");
  }





  camera_config_t config;

  #if defined(CAM_MODULE_OV5640)
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
  config.xclk_freq_hz = 20000000;   //20000000;
  config.frame_size = FRAMESIZE_QVGA;   //FRAMESIZE_XGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  #endif

  #if defined(CAM_MODULE_OV7725)
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
  config.xclk_freq_hz = 20000000;   //20000000;
  config.frame_size = FRAMESIZE_QVGA;   //FRAMESIZE_XGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; 
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 30;
  config.fb_count = 1;
  #endif




  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } 
  
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  s->set_brightness(s,2);
  s->set_saturation(s,-2);
  s->set_awb_gain(s,0);
  s->set_gain_ctrl(s,1);
  s->set_gainceiling(s,(gainceiling_t)64);
  s->set_aec2(s,0);
  s->set_aec_value(s,0);

  s->set_vflip(s,1);
  //s->set_hmirror(s,1);

  WiFi.begin(ssid.c_str(), password.c_str()); // Use .c_str() for String objects
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  //Serial.print("Camera Ready!");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Serial.println("Connected to the WiFi network");

  // Start TCP server
  tcpServer.begin();
  Serial.printf("TCP server started on port %d\n", TCP_PORT);

}
