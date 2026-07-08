/*
 * File Name          : inference.cpp
 * Description        : Edge Impulse TFLite direct inference wrapper for V5F
 *                      (C++ file, called from C main.c via extern "C")
 */

/* Force C-linkage for Edge Impulse porting symbols (match model expectation) */
#define EI_C_LINKAGE 1

extern "C" {
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
}

#include "tensorflow/lite/c/common.h"
#include "tflite-model/tflite_learn_1016276_3_compiled.h"
#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
extern ei_dsp_config_spectral_analysis_t ei_dsp_config_1016276_2;
/* ---------- simple bump allocator (avoids heap dependency) ---------- */
#define ALLOC_BUF_SIZE  4096
static uint8_t  alloc_buf[ALLOC_BUF_SIZE] __attribute__((aligned(16)));
static size_t   alloc_offset = 0;

static void* bump_alloc(size_t size, size_t align) {
    if (alloc_offset & (align - 1)) {
        alloc_offset = (alloc_offset + align - 1) & ~(align - 1);
    }
    if (alloc_offset + size > ALLOC_BUF_SIZE) return nullptr;
    void* p = &alloc_buf[alloc_offset];
    alloc_offset += size;
    return p;
}

static void bump_free(void* ptr) { (void)ptr; }

extern "C" void ei_printf(const char* format, ...) {
    va_list args; va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
extern "C" void* ei_malloc(size_t size) { return bump_alloc(size, 16); }
extern "C" void* ei_calloc(size_t n, size_t s) {
    size_t t=n*s; if(alloc_offset&15)alloc_offset=(alloc_offset+15)&~15;
    if(alloc_offset+t>ALLOC_BUF_SIZE)return nullptr;
    void* p=&alloc_buf[alloc_offset];for(size_t i=0;i<t;i++)alloc_buf[alloc_offset+i]=0;
    alloc_offset+=t;return p;
}
extern "C" void ei_free(void* p){(void)p;}
extern "C" EI_IMPULSE_ERROR ei_run_impulse_check_canceled(void){return (EI_IMPULSE_ERROR)0;}
extern "C" unsigned long long ei_read_timer_us(void){return 0;}
int extract_spectral_analysis_features(void*, void*, void*, float);
/* ---------- dequantization ---------- */
static const float nn_scale     = 0.00390625f;
static const int   nn_zero_point = -128;

static float dequant_i8(int8_t val) {
    return ((float)(val - nn_zero_point)) * nn_scale;
}

/* ---------- label map: 3 output classes ---------- */
static const char* class_labels[] = { "Normal", "Drive Fault", "Fan Fault" };

/* ---------- C-callable inference API ---------- */
extern "C" {

int ei_inference_init(void) {
    alloc_offset = 0;
    TfLiteStatus s = tflite_learn_1016276_3_init(bump_alloc);
    return (s == kTfLiteOk) ? 0 : -1;
}

int ei_inference_run(const int8_t* features, int feature_count,
                     int* class_idx, float* confidence) {
    TfLiteStatus s;
    TfLiteTensor input, output;

    alloc_offset = 0;
    s = tflite_learn_1016276_3_init(bump_alloc);
    if (s != kTfLiteOk) return -1;

    s = tflite_learn_1016276_3_input(0, &input);
    if (s != kTfLiteOk) return -1;
    {
        int8_t* dst = input.data.int8;
        size_t  len = input.bytes;
        if (len > (size_t)feature_count) len = feature_count;
        memcpy(dst, features, len);
        ei_printf("IN: ");
        for (size_t i = 0; i < len && i < 13; i++) ei_printf("%d ", dst[i]);
        ei_printf("\r\n");
    }

    s = tflite_learn_1016276_3_invoke();
    if (s != kTfLiteOk) return -1;

    s = tflite_learn_1016276_3_output(0, &output);
    if (s != kTfLiteOk) return -1;

    int8_t* odata = output.data.int8;
    size_t  olen  = output.bytes;
    float   max_v = -1.0f;
    int     max_i = 0;
    for (size_t i = 0; i < olen && i < EI_CLASSIFIER_NN_OUTPUT_COUNT; i++) {
        float v = dequant_i8(odata[i]);
        if (v > max_v) { max_v = v; max_i = (int)i; }
    }

    if (class_idx)  *class_idx  = max_i;
    if (confidence) *confidence = max_v;

    tflite_learn_1016276_3_reset(bump_free);
    return 0;
}

int ei_inference_scores(const int8_t* features, int feature_count, float* scores) {
    TfLiteStatus s;
    TfLiteTensor input, output;

    alloc_offset = 0;
    s = tflite_learn_1016276_3_init(bump_alloc);
    if (s != kTfLiteOk) return -1;

    s = tflite_learn_1016276_3_input(0, &input);
    if (s != kTfLiteOk) return -1;
    {
        int8_t* dst = input.data.int8;
        size_t  len = input.bytes;
        if (len > (size_t)feature_count) len = feature_count;
        memcpy(dst, features, len);
    }

    s = tflite_learn_1016276_3_invoke();
    if (s != kTfLiteOk) return -1;

    s = tflite_learn_1016276_3_output(0, &output);
    if (s != kTfLiteOk) return -1;

    int8_t* odata = output.data.int8;
    size_t  olen  = output.bytes;
    for (size_t i = 0; i < olen && i < EI_CLASSIFIER_NN_OUTPUT_COUNT; i++) {
        scores[i] = dequant_i8(odata[i]);
    }
    ei_printf("OUT: ");
    for (size_t i = 0; i < olen && i < 3; i++) ei_printf("%d ", odata[i]);
    ei_printf("\r\n");

    tflite_learn_1016276_3_reset(bump_free);
    return 0;
}

const char* ei_class_label(int idx) {
    if (idx < 0 || idx >= (int)(sizeof(class_labels)/sizeof(class_labels[0])))
        return "?";
    return class_labels[idx];
}

int ei_class_count(void) {
    return (int)(sizeof(class_labels)/sizeof(class_labels[0]));
}

} /* extern "C" */
