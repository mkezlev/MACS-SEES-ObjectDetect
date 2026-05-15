/**
 * camera_preprocess.h
 *
 * Crop and resize OV3660 grayscale 320x240 → 96x96
 * matching Edge Impulse "Fix short axis" preprocessing EXACTLY.
 *
 * EI "Fix short axis" logic for 320x240 → 96x96:
 *   1. Scale so the SHORT axis (height=240) fits 96  →  scale = 96/240 = 0.4
 *   2. Scaled size: width = floor(320 * 0.4) = 128,  height = 96
 *   3. Center-crop the LONG axis (width 128 → 96):   skip 16px on each side
 *
 * Mapping back to source pixel space:
 *   16 scaled pixels × (240/96) = 40 source pixels skipped on left/right
 *   → sample a 240×240 window centered in the 320×240 frame
 *   → downsample 240×240 → 96×96  (factor 2.5× on both axes)
 */
 
#ifndef CAMERA_PREPROCESS_H
#define CAMERA_PREPROCESS_H
 
#include <stdint.h>
#include <stddef.h>
 
// ── Source frame (OV3660 QVGA grayscale) ─────────────────────────────────────
#define CAM_W   320
#define CAM_H   240
 
// ── Model input ───────────────────────────────────────────────────────────────
#define MODEL_W  96
#define MODEL_H  96
 
// ── "Fix short axis" window in source pixel space ────────────────────────────
//
//  Short axis = CAM_H = 240  → fits MODEL_H=96  (scale 0.4)
//  Long  axis = CAM_W = 320  → scaled = 128, cropped to 96
//  Crop offset in scaled space = (128-96)/2 = 16 scaled pixels
//  In source space: 16 * (240/96) = 40 source pixels
//
#define SRC_X_START   40    // skip 40px on left  (= (320 - 240) / 2)
#define SRC_Y_START    0
#define SRC_REGION_W  240   // 240px wide  (= MODEL_W * CAM_H / MODEL_H)
#define SRC_REGION_H  240   // 240px tall  (= CAM_H)
 
// ── Fixed-point scale factors (×256, right-shift 8 to apply) ─────────────────
// 1 output pixel covers (SRC_REGION / MODEL) = 2.5 source pixels
// Encoded: 2.5 × 256 = 640
#define SCALE_X  ((SRC_REGION_W * 256) / MODEL_W)   // = 640
#define SCALE_Y  ((SRC_REGION_H * 256) / MODEL_H)   // = 640
 
// ── Output buffer (reused every frame, no heap allocation) ───────────────────
static uint8_t ei_input_buf[MODEL_W * MODEL_H];
 
/**
 * crop_and_resize()
 *
 * Applies EI "fix short axis" crop+resize to a raw grayscale camera frame.
 *
 * @param src      Grayscale row-major buffer, CAM_W × CAM_H bytes.
 * @param src_len  Must be >= CAM_W * CAM_H.
 * @return         Pointer to 96×96 output in ei_input_buf[], or NULL on error.
 */
static inline uint8_t* crop_and_resize(const uint8_t* src, size_t src_len)
{
    if (src_len < (size_t)(CAM_W * CAM_H)) {
        return NULL;
    }
 
    for (int dy = 0; dy < MODEL_H; dy++) {
        int sy = SRC_Y_START + (int)(((uint32_t)dy * SCALE_Y) >> 8);
 
        for (int dx = 0; dx < MODEL_W; dx++) {
            int sx = SRC_X_START + (int)(((uint32_t)dx * SCALE_X) >> 8);
 
            ei_input_buf[dy * MODEL_W + dx] = src[sy * CAM_W + sx];
        }
    }
 
    return ei_input_buf;
}
 
// ── Edge Impulse get_data callback ───────────────────────────────────────────
//
// Normalization: uint8 [0,255] → float [0.0, 1.0]
//
// EI's SDK re-quantizes the float internally via the model's baked-in
// scale + zero_point. 
//
#include "edge-impulse-sdk/dsp/numpy_types.h"
 
static const uint8_t* _ei_buf_ptr = NULL;
 
static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)_ei_buf_ptr[offset + i] / 255.0f;
    }
    return 0;  // EI_IMPULSE_OK
}
 
#endif // CAMERA_PREPROCESS_H
 
