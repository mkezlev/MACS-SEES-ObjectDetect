#include <Arduino.h>


// ── Edge Impulse library ─────────────────────────────────────────────────────
#include <SEES-Project_inferencing.h>

// ── Camera & preprocessing ───────────────────────────────────────────────────
#include "esp_camera.h"
#include "camera_preprocess.h"   // crop_and_resize(), ei_camera_get_data()

// ── LCD Library ───────────────────────────────────────────────────
#include <LiquidCrystal.h>

#include "ble_preprocess.h"




// Camera pins
//(CAMERA_MODEL_ESP32S3_EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#define LED_GPIO 2

// LCD PINS for Adafruit 1528-399-ND 
// Add resistor between ESP32-S3 5V and LCD +5V
// LCD pin name RS EN   Vss/0V   Vdd/+5V
// LCD pin #    4   6     1        2
// LCD pin name 	RS 	EN 	DB4 DB5	DB6	DB7
// LCD pin # 	    4 	6 	9 	10 	11 	12
#define LCD_RS_GPIO 41  
#define LCD_EN_GPIO 42  
#define LCD_D4_GPIO 19  
#define LCD_D5_GPIO 20  
#define LCD_D6_GPIO 21  
#define LCD_D7_GPIO 47  

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define HEAP_SIZE  EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS



enum camera_type_t {
    S_TYPE_OV3660,
    S_TYPE_OV5640
 } ;


// global variables
static camera_type_t _cameraType = S_TYPE_OV3660;  // Set Sensor type
static bool                _initialized = false;
camera_fb_t* fb       = nullptr;
sensor_t* sensor      = nullptr;
camera_config_t* config = nullptr;

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
LiquidCrystal lcd(LCD_RS_GPIO, LCD_EN_GPIO, LCD_D4_GPIO, LCD_D5_GPIO, LCD_D6_GPIO, LCD_D7_GPIO);


static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    // OV3660 and OV5640 camera sensor, especially when used with ESP32-S3 or similar AI 
    // modules, the recommended xclk_freq_hz is typically 20,000,000 Hz (20 MHz)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //Do not use sizes above QVGA when not JPEG
     /*
      *  FRAMESIZE_96X96,    // 96x96
      *  FRAMESIZE_QQVGA,    // 160x120
      *  FRAMESIZE_QVGA     // 320x240     
      */
    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


void setup() {
    Serial.begin(115200);
    // set up the LCD's number of columns and rows:
     lcd.begin(16, 2);

    Serial.setDebugOutput(false);

    // Configure LED and LCD pin
    pinMode(LED_GPIO, OUTPUT);
    pinMode(LCD_RS_GPIO, OUTPUT);
    pinMode(LCD_EN_GPIO, OUTPUT);
    pinMode(LCD_D4_GPIO, OUTPUT);
    pinMode(LCD_D5_GPIO, OUTPUT);
    pinMode(LCD_D6_GPIO, OUTPUT);
    pinMode(LCD_D7_GPIO, OUTPUT);

    digitalWrite(LED_GPIO, LOW); 
    Serial.println();

    if (!init_camera())  {
        print_status("Camera Init failed!");
        while (1) {
            delay(1000);
        }
    }
    
    
    print_status("Camera Ready!");
    init_ble();

    // Configure deep sleep to wake up after 10 seconds
    esp_sleep_enable_timer_wakeup(10ULL * 1000000ULL);


}


void loop() {
    
    unsigned long startTime = millis();
    // Capture and detect will run 60sec and system will go deep sleep 10 seconds
    // deep sleep wake-up is 10 sec for demo purpose
    // can be configured for longer period or triggered by motion detection sensor
    while (millis() - startTime < 60000) {  // 60 seconds = 30000ms
        capture_and_infer();
        delay(100); // Delay 100ms before next reading
    }

    // After capture and dedect for 60 seconds, go to Deep Sleep
    print_status("Entering Deep Sleep now...");
    delay(100); // Give time for last serial messages to send

    esp_deep_sleep_start(); // Deep Sleep (will reset on wake-up)
 
}


/**
 * @brief Orchestrates the complete EdgeAI execution cycle: 
 *        flashes an indicator LED, 
 *        captures a raw grayscale image from the camera framebuffer, 
 *        preprocesses the pixel array,
 *        triggers local Edge Impulse machine learning inference.
 *        The execution includes safety validation checks for formats, 
 *        releases critical buffers back to the hardware pool as fast as possible 
 */

void capture_and_infer(){
    // Turn Led On
    digitalWrite(LED_GPIO, HIGH);
  
    // 1. Capture frame
    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {
        print_status("Failed to get frame");
        return;
    }

    /*
    if (fb->format != PIXFORMAT_GRAYSCALE ||
        fb->width  != CAM_W              ||
        fb->height != CAM_H) 
    */
    if (fb->format != PIXFORMAT_GRAYSCALE ) {
        Serial.printf("Unexpected frame: %u×%u fmt=%u\n",
                      fb->width, fb->height, fb->format);
        print_status(String("Unexpected frame :") + String(fb->width,4) + String(fb->height,4) + String(fb->format));
                  
        esp_camera_fb_return(fb);
        delay(100);
        return;
    }

    // Capture success
        /*
    Serial.printf("[CAM] Frame size: %d bytes (%dx%d) fmt=%u\n",
                      fb->len,
                      fb->width,
                      fb->height,
                      fb->format
                    );
    sensor = esp_camera_sensor_get();
    Serial.printf("[CAM] Sensor Type: %d )\n",sensor->id);
        */

        print_status("Image Frame Captured");

        // 2. Crop 320×240 → center-crop 240×240 → resize to 96×96
    
    uint8_t* model_input = crop_and_resize(fb->buf, fb->len);
    esp_camera_fb_return(fb);   // return ASAP to free DMA buffer

    if (!model_input) {
        //Serial.println("Preprocessing failed.");
        print_status("Preprocessing failed.");
        delay(100);
        return;
    }        

    // 3. Run Edge Impulse inference
    run_inference(model_input);

       
    // Release camera resource
    if (fb) {
            esp_camera_fb_return(fb);
            fb = nullptr;
    }

    
    // Wait before next capture
    delay(2000);
    // Turn Led Off
    digitalWrite(LED_GPIO, LOW); 
}

/**
 * @brief Initializes the camera sensor config, validates the sensor handle, 
 * and applies hardware-specific defaults (like vertical flip or grayscale optimization).
 * @return true if the camera was successfully initialized or was already active.
 * @return false if the initialization failed or the sensor pointer could not be retrieved.
 */
bool init_camera() {
    if (_initialized) {
        return true;
    }

 
    // Initialize camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        print_status(String("Init failed with error ") + String(":") + String(err, 2));
        return false;
    }

    // Get sensor handle
    sensor = esp_camera_sensor_get();
    if (!sensor) {
        print_status("Failed to get sensor");
        return false;
    }

    // Apply sensor-specific defaults
    switch (_cameraType) {
        case S_TYPE_OV5640:
            // OV5640 typically needs vflip
            sensor->set_vflip(sensor, 1);
            break;

        case S_TYPE_OV3660:
            // OV3660 needs vflip and color adjustments
            //sensor->set_vflip(sensor, 1);
            sensor->set_brightness(sensor, 1);
            sensor->set_saturation(sensor, -2);
            sensor->set_whitebal(sensor, 0);       // disable AWB (grayscale; saves CPU)
            break;

        default:
            break;
    }

    _initialized = true;
    return true;
}



/*************************  Edge Impulse inference *************************************
 * @brief Executes local machine learning inference using the Edge Impulse SDK.
 *        Wraps the preprocessed camera frame buffer into an Edge Impulse signal,
 *        invokes the classifier, and prints runtime performance and raw model outputs 
 *        (bounding boxes/centroids for object detection or class probabilities).
 * @param model_input Pointer to the constant uint8_t array containing the preprocessed 
 *        image feature data (e.g., a 96x96 grayscale buffer).
 */
static void run_inference(const uint8_t* model_input)
{
    // Point the global callback pointer at the preprocessed buffer
    _ei_buf_ptr = model_input;

    // Build EI signal
    signal_t signal;
    signal.total_length = MODEL_W * MODEL_H;
    signal.get_data     = &ei_camera_get_data;

    // Run classifier
    ei_impulse_result_t result = {};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false /* debug */);

    if (err != EI_IMPULSE_OK) {
        //Serial.printf("run_classifier() failed: %d\n", err);
        print_status(String("Failed to run classifier ") + String(":") + String(err, 2));
        return;
    }

    // ── Print timing ─────────────────────────────────────────────────────────
   //  Serial.printf("Inference: %u ms  DSP: %u ms\n", result.timing.classification, result.timing.dsp);
    print_status(String("Predictions (DSP and Classification ") + String(result.timing.dsp)+ String(result.timing.classification) + String("ms)\n"));               

#if EI_CLASSIFIER_OBJECT_DETECTION
    // ── Object detection output (bounding boxes) ─────────────────────────────
    Serial.println("Detections:");
    bool found = false;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t& bb = result.bounding_boxes[i];
      //  if (bb.value < 0.5f) continue;   // confidence threshold
        found = true;
        //Serial.printf("  %-12s  conf=%.2f  x=%u y=%u w=%u h=%u\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        print_status(String(bb.label) + String(" ") + String(bb.value, 2));
    }
    if (!found) Serial.println("  (nothing above threshold)");

#else
    // ── Classification output ────────────────────────────────────────────────
    Serial.println("Classification:");
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        //Serial.printf("  %-12s %.4f\n",  result.classification[i].label, result.classification[i].value);
        print_status(String(result.classification[i].label) + String(" ") + String(result.classification[i].value, 2));
    }
    Serial.println();
#endif
}

 * @brief Initializes the Bluetooth Low Energy (BLE) stack and configures the GATT server.
 *        Sets up a custom device name, establishes the server callbacks, provisions a 
 *        primary service with read/notify characteristics, and begins broadcasting the 
 *        advertising packet so external client applications can discover and connect.
 */
void init_ble(){

  // 1. Initialize BLE
  // Please please change this name to your own!!!
  BLEDevice::init("ESP32_BLE_SEES_1");

  // 2. Create a BLE server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. Create a BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 4. Create a characteristic, supporting Notify (automatic data push)
  //      for Read and Notify  
  //     write not enabled BLECharacteristic::PROPERTY_WRITE  |
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   | 
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // 5. Set initial value
  pCharacteristic->setValue("Hello from ESP32!");

  // 6. Start the service
  pService->start();

  // 7. Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE server is up and advertising, waiting for a client to connect...");

}

/**
 * @brief Routes status messages simultaneously to the hardware Serial monitor, 
 *        a 16x2 character LCD, and any connected BLE client.
 *        The function automatically handles basic string truncation and 2-line wrapping 
 *        for the character display, and checks the global connection state before pushing 
 *        BLE notifications.
 * @param message The Arduino String payload containing the text to broadcast.
 */
void print_status(String message) {
    // Print to Serial Monitor
    Serial.println(message);

    // Print to LCD
    lcd.clear();           // Clear previous screen content
    lcd.setCursor(0, 0);   // Start at the top-left corner
    
    // If the message is long wrap it
    if (message.length() > 16) {
        lcd.print(message.substring(0, 16)); // First line
        lcd.setCursor(0, 1);
        lcd.print(message.substring(16, 32)); // Second line
    } else {
        lcd.print(message);
    }
    
    // Print to BLE
    if(deviceConnected) { 
        pCharacteristic->setValue(message.c_str()); 
        pCharacteristic->notify();
    }
}

