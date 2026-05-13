#include <Arduino.h>
#include <SEES-Project_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"


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

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3



enum camera_type_t {
    S_TYPE_OV3660,
    S_TYPE_OV5640
 } ;


// global variables
static camera_type_t _cameraType = S_TYPE_OV3660;  // Set Camera type
static bool                _initialized = false;
camera_fb_t* fb       = nullptr;
sensor_t* sensor      = nullptr;
camera_config_t* config = nullptr;
uint8_t *snapshot_buf = nullptr ; //points to the output of the capture
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal


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
      *  FRAMESIZE_SXGA (1280 × 1024)
      *  FRAMESIZE_XGA (1024 × 768)
      *  FRAMESIZE_SVGA (800 × 600)
      *  FRAMESIZE_VGA (640 × 480)
      *  FRAMESIZE_QVGA (320 × 240)
      */
    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};





void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(false);

    // Configure LED pin
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, HIGH); 
    Serial.println();

    if (!camera_begin())  {
        Serial.println("[CAM] Init failed!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("[CAM] Ready!");

    // Configure deep sleep to wake up after 10 seconds
    esp_sleep_enable_timer_wakeup(10ULL * 1000000ULL);


}

//task loop uses core 1.
void loop() {
    
    unsigned long startTime = millis();
    // Captue and detect will run 60sec and system will go deep sleep 10 seconds
    // deep sleep wake-up is 10 sec for demo purpose
    // can be configured for longer period or triggered by motion detection sensor
    while (millis() - startTime < 60000) {  // 60 seconds = 30000ms
        capture_and_detect();
        delay(100); // Delay 100ms before next reading
    }

    // After capture and dedect for 60 seconds, go to Deep Sleep
    Serial.println("Entering Deep Sleep now...");
    delay(100); // Give time for last serial messages to send

    esp_deep_sleep_start(); // Deep Sleep (will reset on wake-up)
 
}


void capture_and_detect(){
    // Turn Led On
    digitalWrite(LED_GPIO, HIGH);
  
    // Capture a frame
    if (capture_image()) {
        /*
        Serial.printf("[CAM] Frame size: %d bytes (%dx%d)\n",
                      fb->len,
                      fb->width,
                      fb->height);
        */
        Serial.println("[CAM] Image Frame Captured");
        // Access the raw image data
        uint8_t* imageData = fb->buf;
        size_t imageLen = fb->len;
        
        // Allocate memory in PSRAM for ML Image
        snapshot_buf = (uint8_t*)ps_malloc(imageLen);
        
        //snapshot_buf = (uint8_t*)ps_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);
        

         // check if allocation was successful
        if(snapshot_buf == nullptr) {
            Serial.println("ERR: Failed to allocate snapshot buffer!\n");
            return;
        }

       // Resize image to fit model image size
        if ((fb->width > EI_CLASSIFIER_INPUT_WIDTH) || (fb->height > EI_CLASSIFIER_INPUT_HEIGHT)){
            int res = ei::image::processing::resize_image_using_mode(   
                imageData,               // Source buffer (native camera image)
                fb->width,            // Original width
                fb->height,           // Original height
                snapshot_buf,              // Destination buffer (can be the same as source)
                EI_CLASSIFIER_INPUT_WIDTH,  // Target width (96 for FOMO)
                EI_CLASSIFIER_INPUT_HEIGHT, // Target height (96 for FOMO)
                1,                       // Number of channels (3 for RGB888)
                EI_CLASSIFIER_RESIZE_MODE   // Mode: Squash, Fit shortest, or Fit longest
            );
        }
       
       // Run Classifier from Edge Impulse SDK
       object_detect();
       
        // Free the frame buffer in PSRAM
        if (snapshot_buf != nullptr) {
            free(snapshot_buf);
            snapshot_buf = nullptr; // Important: Set to nullptr to avoid "dangling pointer" errors
        }

        // Release camera resource
        if (fb) {
            esp_camera_fb_return(fb);
            fb = nullptr;
        }

    } else {
        Serial.println("[CAM] Failed to get frame");
    }
    
    // Wait before next capture
    delay(2000);
    // Turn Led Off
    digitalWrite(LED_GPIO, LOW); 
}

bool camera_begin() {
    if (_initialized) {
        return true;
    }

    // Adjust fb_count for non-JPEG formats
    if (camera_config.pixel_format != PIXFORMAT_JPEG) {
        camera_config.fb_count    = 1;
        camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    }

    config = &camera_config;

    // Initialize camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed with error 0x%x\n", err);
        return false;
    }

    // Get sensor handle
    sensor = esp_camera_sensor_get();
    if (!sensor) {
        Serial.println("[CAM] Failed to get sensor");
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
            sensor->set_vflip(sensor, 1);
            sensor->set_brightness(sensor, 1);
            sensor->set_saturation(sensor, -2);
            break;

        default:
            break;
    }

    _initialized = true;
    return true;
}

bool capture_image() {
    if (!_initialized) return false;

    fb = esp_camera_fb_get();
    if (!fb) {
        return false;
    }
    return true;
}


void object_detect(){
    
    ei::signal_t ei_signal;
    ei_signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    ei_signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&ei_signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    Serial.printf("[INF] Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);


    Serial.printf("[INF] Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        Serial.printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }  
}


/**
 * @brief      callback function used by the Edge Impulse library to retrieve pixel data from the buffer during inference.
 *
 * @param[in]  offset    starting index of the pixel data requested by the inference engine.
 * @param[in]  length    total number of samples (pixels) to retrieve in the current call.
 * @param[in]  out_ptr   pointer to the destination buffer where the processed pixel data will be stored.

        signal.get_data = [&](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                // Normalizing 0-255 grayscale value to 0.0-1.0 float
                out_ptr[i] = (float)snapshot_buf[offset + i] / 255.0f;
            }
        return 0;
       };

 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
        
    for (size_t i = 0; i < length; i++) {
        // Normalizing 0-255 grayscale value to 0.0-1.0 float
        out_ptr[i] = (float)fb->buf[offset + i] / 255.0f;
    }
    return 0;
 }