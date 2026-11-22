#ifndef __VISION_H
#define __VISION_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_CIRCLE_POINT_COUNT 64

#ifndef VISION_ENABLE_BINARIZATION
#define VISION_ENABLE_BINARIZATION 1
#endif

#ifndef VISION_ENABLE_CANNY
#define VISION_ENABLE_CANNY 1
#endif

#ifndef VISION_ENABLE_CONTOUR
#define VISION_ENABLE_CONTOUR 1
#endif

#ifndef VISION_ENABLE_TARGET_SELECTION
#define VISION_ENABLE_TARGET_SELECTION 1
#endif

#ifndef VISION_ENABLE_PERSPECTIVE
#define VISION_ENABLE_PERSPECTIVE 1
#endif

#ifndef VISION_DETECT_INTERVAL_FRAMES
#define VISION_DETECT_INTERVAL_FRAMES 2
#endif

#if VISION_DETECT_INTERVAL_FRAMES < 1
#error "VISION_DETECT_INTERVAL_FRAMES must be at least 1"
#endif

#if VISION_ENABLE_CANNY && !VISION_ENABLE_BINARIZATION
#error "VISION_ENABLE_CANNY requires VISION_ENABLE_BINARIZATION"
#endif

#if VISION_ENABLE_CONTOUR && !VISION_ENABLE_CANNY
#error "VISION_ENABLE_CONTOUR requires VISION_ENABLE_CANNY"
#endif

#if VISION_ENABLE_TARGET_SELECTION && !VISION_ENABLE_CONTOUR
#error "VISION_ENABLE_TARGET_SELECTION requires VISION_ENABLE_CONTOUR"
#endif

#if VISION_ENABLE_PERSPECTIVE && !VISION_ENABLE_TARGET_SELECTION
#error "VISION_ENABLE_PERSPECTIVE requires VISION_ENABLE_TARGET_SELECTION"
#endif

typedef struct {
    int16_t x;
    int16_t y;
} VISION_POINT_T;

typedef struct {
    bool target_found;
    bool projection_valid;
    uint8_t threshold;
    uint32_t edge_count;
    uint32_t area;
    int32_t score_x100;
    VISION_POINT_T corners[4];
    VISION_POINT_T center;
    VISION_POINT_T circle[VISION_CIRCLE_POINT_COUNT];
} VISION_RESULT_T;

typedef struct {
    int16_t threshold; /* <0: auto Otsu, 0~255: manual */
    uint16_t canny_low;
    uint16_t canny_high;
    uint32_t min_area;
    uint16_t border_margin;
    uint16_t morph;     /* 1: 3x3 open+close on binary image */
    uint16_t eps_x100;  /* polygon approx epsilon, percent x100 */
    uint16_t proc;      /* 0: bypass all processing, keep raw gray only */
    uint16_t cmode;     /* 1: chroma window classifier instead of luma threshold */
    uint16_t u_lo;
    uint16_t v_lo;
    uint16_t u_hi;
    uint16_t v_hi;
    uint16_t y_lo;
    uint16_t y_hi;
    uint16_t cphase;    /* chroma valid row phase mod 4 (sensor rows) */
} VISION_PARAMS_T;

bool vision_init(uint16_t width, uint16_t height);
bool vision_process(const uint8_t *yuyv, uint32_t data_len, VISION_RESULT_T *result);
const uint8_t *vision_get_binary(void);
const uint8_t *vision_get_edges(void);
const uint8_t *vision_get_gray(void);
const VISION_PARAMS_T *vision_get_params(void);
bool vision_set_param(const char *name, int32_t value);
void vision_dump_params(void);

#endif
