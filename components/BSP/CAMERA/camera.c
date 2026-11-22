/******************************************************************
 * @file        camera.c
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2025-07-25
 * @brief       CAMERA driver code
 ******************************************************************
 * @attention
 *
 * platform     : ALIENTEK T5 MINI Board
 * website      : www.alientek.com
 * forum        : www.openedv.com/forum.php
 *
 * change logs  :
 * version      data         notes
 * V1.0         20250725     the first version
 *
 ******************************************************************
 */

#include "camera.h"
#include "vision.h"
#include "servo.h"
#include "tftlcd.h"
#include "tkl_i2c.h"
#include "tkl_gpio.h"



#if defined(ATK_T5AI_MINI_BOARD_LCD_MD0700R_RGB)
#define CAMERA_FPS                20
#define CAMERA_WIDTH              480
#define CAMERA_HEIGHT             480
#else
#define CAMERA_FPS                40
#define CAMERA_WIDTH              240
#define CAMERA_HEIGHT             240
#endif

#define STAT_THUMB_W               16
#define STAT_THUMB_H               12
#define STAT_SAMPLE_LEN            16
#define STAT_PRINT_INTERVAL_MS     5000

#define IMG_WIDTH                  CAMERA_WIDTH
#define IMG_HEIGHT                 CAMERA_HEIGHT
#define IMGPROC_ENABLE             1

/* Binary image dump over log uart: IMGSTART / IMGDATA / IMGEND, hex encoded GRAY8 */
#define IMG_DUMP_ENABLE            0
#define IMG_DUMP_START_FRAME       100
#define IMG_DUMP_PERIOD_FRAMES     200
#define IMG_DUMP_WIDTH             120u
#define IMG_DUMP_HEIGHT            120u

extern void bk_printf(const char *fmt, ...);

extern TDL_DISP_HANDLE_T sg_tdl_disp_hdl;
extern TDL_DISP_FRAME_BUFF_T *sg_p_display_fb;
extern TDL_DISP_DEV_INFO_T sg_display_info;
static TDL_CAMERA_HANDLE_T sg_tdl_camera_hdl = NULL;

static volatile uint32_t sg_frame_cnt = 0;
static uint32_t sg_last_print_cnt = 0;
static TDL_CAMERA_FRAME_T sg_last_frame_info = {0};
static uint8_t sg_thumb[STAT_THUMB_H][STAT_THUMB_W] = {0};
static uint8_t sg_raw_sample[STAT_SAMPLE_LEN] = {0};
static uint8_t sg_luma_avg = 0;
static uint8_t sg_luma_min = 0;
static uint8_t sg_luma_max = 0;

static VISION_RESULT_T sg_vision_result;
static VISION_RESULT_T sg_vision_work;
static volatile bool sg_dump_request = false;
static volatile bool sg_raw_dump_request = false;
static uint8_t sg_dump_buf[IMG_DUMP_WIDTH * IMG_DUMP_HEIGHT * 4];
static volatile bool sg_dump_pending = false;
static volatile bool sg_dump_is_raw = false;

/**
 * @brief       read one sensor register via i2c
 * @param[in]   addr:7bit i2c address
 * @param[in]   reg:register address
 * @param[in]   is_16_reg:register address is 16bit
 * @param[out]  val:register value
 * @return      0:success  -1:failed
 */
static int __camera_i2c_read_reg(uint8_t addr, uint16_t reg, uint8_t is_16_reg, uint8_t *val)
{
    uint8_t reg_buf[2];
    uint8_t reg_len = is_16_reg ? 2 : 1;

    reg_buf[0] = is_16_reg ? (uint8_t)(reg >> 8) : (uint8_t)(reg & 0xFF);
    reg_buf[1] = (uint8_t)(reg & 0xFF);

    if (OPRT_OK != tkl_i2c_master_send(TUYA_I2C_NUM_0, addr, reg_buf, reg_len, 1)) {
        return -1;
    }

    if (OPRT_OK != tkl_i2c_master_receive(TUYA_I2C_NUM_0, addr, val, 1, 0)) {
        return -1;
    }

    return 0;
}

/**
 * @brief       probe camera i2c address and sensor id
 * @param[in]   none
 * @return      none
 */
static void __camera_i2c_probe(void)
{
    static const uint8_t addr_list[] = {0x30, 0x3C, 0x21};
    uint8_t dummy = 0x00;

    PR_NOTICE("---- camera i2c probe ----");

    for (uint32_t i = 0; i < sizeof(addr_list); i++) {
        uint8_t addr = addr_list[i];
        uint8_t id_hi = 0, id_lo = 0;

        if (OPRT_OK != tkl_i2c_master_send(TUYA_I2C_NUM_0, addr, &dummy, 1, 0)) {
            PR_NOTICE("i2c 0x%02X: no ack", addr);
            continue;
        }

        if (addr == 0x3C) {
            if ((0 == __camera_i2c_read_reg(addr, 0x300A, 1, &id_hi))
                && (0 == __camera_i2c_read_reg(addr, 0x300B, 1, &id_lo))) {
                PR_NOTICE("i2c 0x%02X: ack, pid=0x%02X%02X (OV5640=0x5640)", addr, id_hi, id_lo);
            }
            if ((0 == __camera_i2c_read_reg(addr, 0xF0, 0, &id_hi))
                && (0 == __camera_i2c_read_reg(addr, 0xF1, 0, &id_lo))) {
                PR_NOTICE("i2c 0x%02X: ack, pid=0x%02X%02X (GC2145=0x2145)", addr, id_hi, id_lo);
            }
        } else {
            if ((0 == __camera_i2c_read_reg(addr, 0x0A, 0, &id_hi))
                && (0 == __camera_i2c_read_reg(addr, 0x0B, 0, &id_lo))) {
                PR_NOTICE("i2c 0x%02X: ack, pid=0x%02X%02X (OV2640=0x2642)", addr, id_hi, id_lo);
            } else {
                PR_NOTICE("i2c 0x%02X: ack", addr);
            }
        }
    }

    PR_NOTICE("--------------------------");
}

/**
 * @brief       update camera frame statistics and sampled thumbnail
 * @param[in]   frame:camera frame
 * @return      none
 */
static void __camera_stat_update(TDL_CAMERA_FRAME_T *frame)
{
    uint32_t sum = 0;
    uint32_t cnt = 0;
    uint8_t vmin = 255;
    uint8_t vmax = 0;
    uint16_t step_x, step_y, x, y;

    sg_frame_cnt++;

    sg_last_frame_info.id = frame->id;
    sg_last_frame_info.fmt = frame->fmt;
    sg_last_frame_info.width = frame->width;
    sg_last_frame_info.height = frame->height;
    sg_last_frame_info.data_len = frame->data_len;
    sg_last_frame_info.total_frame_len = frame->total_frame_len;

    if ((NULL == frame->data) || (0 == frame->width) || (0 == frame->height)) {
        return;
    }

    memcpy(sg_raw_sample, frame->data,
           frame->data_len < STAT_SAMPLE_LEN ? frame->data_len : STAT_SAMPLE_LEN);

    step_x = frame->width / STAT_THUMB_W;
    step_y = frame->height / STAT_THUMB_H;
    step_x = step_x ? step_x : 1;
    step_y = step_y ? step_y : 1;

    for (y = 0; y < STAT_THUMB_H; y++) {
        for (x = 0; x < STAT_THUMB_W; x++) {
            uint32_t px = (uint32_t)x * step_x + step_x / 2;
            uint32_t py = (uint32_t)y * step_y + step_y / 2;
            uint32_t idx;
            uint8_t luma;

            if ((px >= frame->width) || (py >= frame->height)) {
                sg_thumb[y][x] = 0;
                continue;
            }

            idx = ((uint32_t)py * frame->width + px) * 2; /* YUYV: Y component at even offset */
            if ((idx + 1) >= frame->data_len) {
                sg_thumb[y][x] = 0;
                continue;
            }

            luma = frame->data[idx];
            sg_thumb[y][x] = luma;
            sum += luma;
            cnt++;
            if (luma < vmin) {
                vmin = luma;
            }
            if (luma > vmax) {
                vmax = luma;
            }
        }
    }

    if (cnt) {
        sg_luma_avg = (uint8_t)(sum / cnt);
        sg_luma_min = vmin;
        sg_luma_max = vmax;
    }
}

/**
 * @brief       get rgb565 pixel from display framebuffer
 * @param[in]   pix:frame buffer
 * @param[in]   width:frame width
 * @param[in]   x:coordinate x
 * @param[in]   y:coordinate y
 * @param[in]   swap:rgb565 byte order is swapped
 * @return      rgb565 pixel value in normal byte order
 */
static uint16_t __disp_get_pixel(uint16_t *pix, uint16_t width, uint32_t x, uint32_t y, bool swap)
{
    uint16_t c = pix[y * width + x];

    if (swap) {
        c = (uint16_t)((c >> 8) | (c << 8));
    }

    return c;
}

/**
 * @brief       print camera frame info, fps and a sampled thumbnail
 * @param[in]   none
 * @return      none
 */
void camera_print_stat(void)
{
    static const char *luma_chars = " .:-=+*#%@";
    uint32_t cnt = sg_frame_cnt;
    uint32_t delta = cnt - sg_last_print_cnt;
    uint32_t fps_x10 = delta * 10000 / STAT_PRINT_INTERVAL_MS;
    uint16_t x, y;

    sg_last_print_cnt = cnt;

    PR_NOTICE("========== CAMERA STAT ==========");
    PR_NOTICE("frame id:%u fmt:%u size:%ux%u len:%u total:%u",
              sg_last_frame_info.id, sg_last_frame_info.fmt,
              sg_last_frame_info.width, sg_last_frame_info.height,
              sg_last_frame_info.data_len, sg_last_frame_info.total_frame_len);
    PR_NOTICE("frames:%u fps:%u.%u luma avg:%u min:%u max:%u",
              delta, fps_x10 / 10, fps_x10 % 10,
              sg_luma_avg, sg_luma_min, sg_luma_max);
    PR_NOTICE("vision otsu:%u edge:%u target:%d", sg_vision_result.threshold,
              sg_vision_result.edge_count, sg_vision_result.target_found);
    {
        int16_t pan, tilt;

        servo_get_angles(&pan, &tilt);
        PR_NOTICE("servo pan:%d tilt:%d", pan, tilt);
    }
    if (sg_vision_result.target_found) {
        PR_NOTICE("rect score:%d.%02d area:%u center:(%d,%d)",
                  sg_vision_result.score_x100 / 100, sg_vision_result.score_x100 % 100,
                  sg_vision_result.area, sg_vision_result.center.x, sg_vision_result.center.y);
        for (uint32_t i = 0; i < 4; i++) {
            PR_NOTICE("corner %u:(%d,%d)", i, sg_vision_result.corners[i].x,
                      sg_vision_result.corners[i].y);
        }
        if (sg_vision_result.projection_valid) {
            PR_NOTICE("circle 64 points, first:(%d,%d) opposite:(%d,%d)",
                      sg_vision_result.circle[0].x, sg_vision_result.circle[0].y,
                      sg_vision_result.circle[VISION_CIRCLE_POINT_COUNT / 2].x,
                      sg_vision_result.circle[VISION_CIRCLE_POINT_COUNT / 2].y);
        }
    }
    PR_NOTICE("raw yuv[0..15]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
              sg_raw_sample[0], sg_raw_sample[1], sg_raw_sample[2], sg_raw_sample[3],
              sg_raw_sample[4], sg_raw_sample[5], sg_raw_sample[6], sg_raw_sample[7],
              sg_raw_sample[8], sg_raw_sample[9], sg_raw_sample[10], sg_raw_sample[11],
              sg_raw_sample[12], sg_raw_sample[13], sg_raw_sample[14], sg_raw_sample[15]);
    PR_NOTICE("camera thumbnail %dx%d:", STAT_THUMB_W, STAT_THUMB_H);

    for (y = 0; y < STAT_THUMB_H; y++) {
        char line[STAT_THUMB_W + 1];

        for (x = 0; x < STAT_THUMB_W; x++) {
            line[x] = luma_chars[sg_thumb[y][x] * 9 / 255];
        }
        line[STAT_THUMB_W] = '\0';
        PR_NOTICE("  |%s|", line);
    }

    if (sg_p_display_fb && sg_p_display_fb->frame
        && sg_p_display_fb->width && sg_p_display_fb->height
        && sg_p_display_fb->len >= (uint32_t)sg_p_display_fb->width * sg_p_display_fb->height * 2) {
        TDL_DISP_FRAME_BUFF_T *fb = sg_p_display_fb;
        uint16_t *pix = (uint16_t *)fb->frame;
        uint16_t step_x = fb->width / STAT_THUMB_W;
        uint16_t step_y = fb->height / STAT_THUMB_H;
        uint16_t cx = fb->width / 2;
        uint16_t cy = fb->height / 2;
        uint16_t sx = (cx > STAT_SAMPLE_LEN / 2) ? (cx - STAT_SAMPLE_LEN / 2) : 0;
        uint16_t sample[STAT_SAMPLE_LEN];

        step_x = step_x ? step_x : 1;
        step_y = step_y ? step_y : 1;

        for (x = 0; x < STAT_SAMPLE_LEN; x++) {
            sample[x] = __disp_get_pixel(pix, fb->width, sx + x, cy, sg_display_info.is_swap);
        }

        PR_NOTICE("display fb %ux%u len:%u fmt:%u swap:%d",
                  fb->width, fb->height, fb->len, fb->fmt, sg_display_info.is_swap);
        PR_NOTICE("display rgb565 center row: %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X",
                  sample[0], sample[1], sample[2], sample[3], sample[4], sample[5], sample[6], sample[7],
                  sample[8], sample[9], sample[10], sample[11], sample[12], sample[13], sample[14], sample[15]);
        PR_NOTICE("display thumbnail %dx%d:", STAT_THUMB_W, STAT_THUMB_H);

        for (y = 0; y < STAT_THUMB_H; y++) {
            char line[STAT_THUMB_W + 1];

            for (x = 0; x < STAT_THUMB_W; x++) {
                uint32_t px = (uint32_t)x * step_x + step_x / 2;
                uint32_t py = (uint32_t)y * step_y + step_y / 2;
                uint16_t c;

                if ((px >= fb->width) || (py >= fb->height)) {
                    line[x] = ' ';
                    continue;
                }

                c = __disp_get_pixel(pix, fb->width, px, py, sg_display_info.is_swap);
                {
                    uint32_t r = (c >> 11) & 0x1F;
                    uint32_t g = (c >> 5) & 0x3F;
                    uint32_t b = c & 0x1F;
                    uint32_t luma = (r * 255 / 31 * 77 + g * 255 / 63 * 150 + b * 255 / 31 * 29) >> 8;

                    line[x] = luma_chars[luma * 9 / 255];
                }
            }
            line[STAT_THUMB_W] = '\0';
            PR_NOTICE("  |%s|", line);
        }
    } else {
        PR_NOTICE("display fb not ready");
    }

    PR_NOTICE("=================================");
}

static inline uint16_t __rgb565_swap(uint16_t c)
{
    return (uint16_t)((c >> 8) | (c << 8));
}

#if !IMGPROC_ENABLE
/**
 * @brief       get Y component of one pixel from YUYV frame
 * @param[in]   data:frame data
 * @param[in]   len:frame data length
 * @param[in]   idx:pixel index
 * @return      Y component
 */
static inline int __yuv_get_y(const uint8_t *data, uint32_t len, uint32_t idx)
{
    uint32_t off = idx * 2; /* YUYV: Y at even offset */

    return (off < len) ? data[off] : 0;
}

/**
 * @brief       convert gray value to RGB565
 * @param[in]   y:gray value
 * @return      RGB565 value
 */
static inline uint16_t __gray_to_rgb565(uint8_t y)
{
    return (uint16_t)(((y & 0xF8) << 8) | ((y & 0xFC) << 3) | (y >> 3));
}
#endif /* !IMGPROC_ENABLE */

static OPERATE_RET __imgproc_init(void)
{
#if IMGPROC_ENABLE
    return vision_init(IMG_WIDTH, IMG_HEIGHT) ? OPRT_OK : OPRT_MALLOC_FAILED;
#else
    return OPRT_OK;
#endif
}

#if IMGPROC_ENABLE
static void __imgproc_compose_gray(void);

static void __imgproc_compose(void)
{
    TDL_DISP_FRAME_BUFF_T *fb = sg_p_display_fb;
    const uint8_t *binary = vision_get_binary();
    uint16_t *pix;
    uint16_t dw, dh;
    int x0, y0;
    uint16_t x, y;
    bool swap;

    if (!vision_get_params()->proc) {
        __imgproc_compose_gray();
        return;
    }
    if ((NULL == fb) || (NULL == fb->frame) || (NULL == binary)) {
        return;
    }

    dw = fb->width;
    dh = fb->height;
    pix = (uint16_t *)fb->frame;
    swap = sg_display_info.is_swap;
    x0 = ((int)dw - IMG_WIDTH) / 2;
    y0 = ((int)dh - IMG_HEIGHT) / 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;

    if ((dw != IMG_WIDTH) || (dh != IMG_HEIGHT)) {
        memset(pix, 0xFF, (uint32_t)dw * dh * sizeof(uint16_t));
    }

    for (y = 0; y < IMG_HEIGHT; y++) {
        uint32_t row = (uint32_t)y * IMG_WIDTH;
        uint16_t *dst;
        const uint8_t *bin = binary + row;

        if ((uint32_t)y0 + y >= dh) {
            break;
        }
        dst = pix + ((uint32_t)y0 + y) * dw + x0;

        for (x = 0; x < IMG_WIDTH; x++) {
            uint16_t color;

            if ((uint32_t)x0 + x >= dw) {
                break;
            }
            color = bin[x] ? 0xFFFF : 0x0000;
            dst[x] = swap ? __rgb565_swap(color) : color;
        }
    }
}

static void __imgproc_compose_gray(void)
{
    TDL_DISP_FRAME_BUFF_T *fb = sg_p_display_fb;
    const uint8_t *gray = vision_get_gray();
    uint16_t *pix;
    uint16_t dw, dh;
    int x0, y0;
    uint16_t x, y;
    bool swap;

    if ((NULL == fb) || (NULL == fb->frame) || (NULL == gray)) {
        return;
    }

    dw = fb->width;
    dh = fb->height;
    pix = (uint16_t *)fb->frame;
    swap = sg_display_info.is_swap;
    x0 = ((int)dw - IMG_WIDTH) / 2;
    y0 = ((int)dh - IMG_HEIGHT) / 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;

    for (y = 0; y < IMG_HEIGHT; y++) {
        uint32_t row = (uint32_t)y * IMG_WIDTH;
        uint16_t *dst;
        const uint8_t *src = gray + row;

        if ((uint32_t)y0 + y >= dh) {
            break;
        }
        dst = pix + ((uint32_t)y0 + y) * dw + x0;

        for (x = 0; x < IMG_WIDTH; x++) {
            uint16_t c;

            if ((uint32_t)x0 + x >= dw) {
                break;
            }
            c = (uint16_t)(((src[x] & 0xF8) << 8) | ((src[x] & 0xFC) << 3) | (src[x] >> 3));
            dst[x] = swap ? __rgb565_swap(c) : c;
        }
    }
}

static OPERATE_RET __camera_imgproc_frame(const TDL_CAMERA_FRAME_T *frame)
{
    if (!vision_process(frame->data, frame->data_len, &sg_vision_work)) {
        return OPRT_INVALID_PARM;
    }
    sg_vision_result = sg_vision_work;
    servo_track_update(&sg_vision_work, IMG_WIDTH, IMG_HEIGHT);
    __imgproc_compose();

    return OPRT_OK;
}
#endif /* IMGPROC_ENABLE */

#if !IMGPROC_ENABLE
/**
 * @brief       convert raw YUV frame luma to gray RGB565 and show on lcd
 * @param[in]   frame:camera frame
 * @return      none
 */
static void __imgproc_raw_to_display(const TDL_CAMERA_FRAME_T *frame)
{
    TDL_DISP_FRAME_BUFF_T *fb = sg_p_display_fb;
    uint16_t *pix;
    uint16_t dw, dh;
    int x0, y0;
    uint16_t x, y;
    bool swap;

    if ((NULL == fb) || (NULL == fb->frame) || (NULL == frame->data)) {
        return;
    }

    dw = fb->width;
    dh = fb->height;
    pix = (uint16_t *)fb->frame;
    swap = sg_display_info.is_swap;
    x0 = ((int)dw - IMG_WIDTH) / 2;
    y0 = ((int)dh - IMG_HEIGHT) / 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;

    if ((dw != IMG_WIDTH) || (dh != IMG_HEIGHT)) {
        memset(pix, 0xFF, (uint32_t)dw * dh * sizeof(uint16_t));
    }

    for (y = 0; y < IMG_HEIGHT; y++) {
        uint32_t row = (uint32_t)y * IMG_WIDTH;
        uint16_t *dst;

        if ((uint32_t)y0 + y >= dh) {
            break;
        }
        dst = pix + ((uint32_t)y0 + y) * dw + x0;

        for (x = 0; x < IMG_WIDTH; x++) {
            uint32_t idx = row + x;
            uint8_t yy;
            uint16_t c;

            if ((uint32_t)x0 + x >= dw) {
                break;
            }

            yy = (uint8_t)__yuv_get_y(frame->data, frame->data_len, idx);
            c = __gray_to_rgb565(yy);
            dst[x] = swap ? __rgb565_swap(c) : c;
        }
    }
}
#endif /* !IMGPROC_ENABLE */

/**
 * @brief       copy subsampled image into static dump buffer (fast, in frame cb)
 * @param[in]   frame:camera frame
 * @param[in]   raw:true=YUYV from sensor, false=GRAY8 from binary image
 * @return      none
 */
static void __camera_capture_dump(const TDL_CAMERA_FRAME_T *frame, bool raw)
{
    const uint8_t *binary = vision_get_binary();
    uint16_t x, y;
    uint32_t pos = 0;

    if (NULL == frame->data) {
        return;
    }
    if (!raw && (NULL == binary)) {
        return;
    }

    for (y = 0; y < IMG_DUMP_HEIGHT; y++) {
        uint32_t sy = (uint32_t)y * IMG_HEIGHT / IMG_DUMP_HEIGHT;

        for (x = 0; x < IMG_DUMP_WIDTH; x++) {
            uint32_t sx = (uint32_t)x * IMG_WIDTH / IMG_DUMP_WIDTH;

            if (raw) {
                uint32_t idx = (sy * IMG_WIDTH + sx) * 2;

                if ((idx + 3) >= frame->data_len) {
                    return;
                }
                sg_dump_buf[pos++] = frame->data[idx];
                sg_dump_buf[pos++] = frame->data[idx + 1];
                sg_dump_buf[pos++] = frame->data[idx];
                sg_dump_buf[pos++] = frame->data[idx + 3];
            } else {
                sg_dump_buf[pos++] = binary[sy * IMG_WIDTH + sx];
            }
        }
    }

    sg_dump_is_raw = raw;
    sg_dump_pending = true;
}

/**
 * @brief       print pending dump buffer over log uart (call from main task)
 * @param[in]   none
 * @return      none
 */
void camera_dump_service(void)
{
    static const char hex[] = "0123456789ABCDEF";
    char line[128];
    uint32_t pos = 0;
    uint32_t i, total;
    uint8_t step;

    if (!sg_dump_pending) {
        return;
    }
    sg_dump_pending = false;

    step = sg_dump_is_raw ? 4 : 1;
    total = (uint32_t)IMG_DUMP_WIDTH * IMG_DUMP_HEIGHT * step;
    bk_printf("IMGSTART %u %u %s\n", IMG_DUMP_WIDTH, IMG_DUMP_HEIGHT, sg_dump_is_raw ? "YUYV" : "GRAY8");

    for (i = 0; i < total; i++) {
        line[pos++] = hex[(sg_dump_buf[i] >> 4) & 0x0F];
        line[pos++] = hex[sg_dump_buf[i] & 0x0F];

        if ((pos + 2) >= sizeof(line)) {
            line[pos] = '\0';
            bk_printf("IMGDATA %s\n", line);
            pos = 0;
        }
    }

    if (pos) {
        line[pos] = '\0';
        bk_printf("IMGDATA %s\n", line);
    }

    bk_printf("IMGEND\n");
}

/**
 * @brief       camera frame callback
 * @param[in]   hdl:camera handle
 * @param[in]   frame:camera frame
 * @return      OPERATE_RET
 */
OPERATE_RET __get_camera_raw_frame_cb(TDL_CAMERA_HANDLE_T hdl,  TDL_CAMERA_FRAME_T *frame)
{
    (void)hdl;

    __camera_stat_update(frame);

#if IMGPROC_ENABLE
    if (OPRT_OK == __camera_imgproc_frame(frame)) {
        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);    /* display frame */
    }
#else
    __imgproc_raw_to_display(frame);                             /* raw image on lcd */
    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
#endif

#if IMG_DUMP_ENABLE
    if ((sg_frame_cnt >= IMG_DUMP_START_FRAME)
        && (0 == ((sg_frame_cnt - IMG_DUMP_START_FRAME) % IMG_DUMP_PERIOD_FRAMES))) {
        __camera_capture_dump(frame, false);
    }
#endif

    if (sg_dump_request) {
        sg_dump_request = false;
        __camera_capture_dump(frame, false);
    }

    if (sg_raw_dump_request) {
        sg_raw_dump_request = false;
        __camera_capture_dump(frame, true);
    }

    return OPRT_OK;
}

void camera_request_dump(void)
{
    sg_dump_request = true;
}

void camera_request_raw_dump(void)
{
    sg_raw_dump_request = true;
}

const VISION_RESULT_T *camera_get_vision_result(void)
{
    return &sg_vision_result;
}

void camera_get_luma(uint8_t *luma_min, uint8_t *luma_max)
{
    if (NULL != luma_min) {
        *luma_min = sg_luma_min;
    }
    if (NULL != luma_max) {
        *luma_max = sg_luma_max;
    }
}

static OPERATE_RET __camera_open(void)
{
    TDL_CAMERA_CFG_T cfg;

    cfg.fps     = CAMERA_FPS;
    cfg.width   = CAMERA_WIDTH;
    cfg.height  = CAMERA_HEIGHT;
    cfg.out_fmt = TDL_CAMERA_FMT_YUV422;
    cfg.get_frame_cb = __get_camera_raw_frame_cb;

    return tdl_camera_dev_open(sg_tdl_camera_hdl, &cfg);
}

static bool __camera_sensor_id_ok(void)
{
    uint8_t hi = 0, lo = 0;
    uint8_t bank_sel[2] = {0xFF, 0x00};

    if (OPRT_OK != tkl_i2c_master_send(TUYA_I2C_NUM_0, 0x30, bank_sel, 2, 1)) {
        return false;
    }
    if ((0 != __camera_i2c_read_reg(0x30, 0x0A, 0, &hi))
        || (0 != __camera_i2c_read_reg(0x30, 0x0B, 0, &lo))) {
        return false;
    }

    return (0x26 == hi) && (0x42 == lo);
}

static void __camera_sensor_power_cycle(void)
{
    TUYA_GPIO_BASE_CFG_T gpio_cfg = {0};

    gpio_cfg.direct = TUYA_GPIO_OUTPUT;
    gpio_cfg.mode = TUYA_GPIO_PUSH_PULL;
    gpio_cfg.level = TUYA_GPIO_LEVEL_HIGH;
    tkl_gpio_init(TUYA_GPIO_NUM_7, &gpio_cfg);
    tkl_gpio_init(TUYA_GPIO_NUM_8, &gpio_cfg);
    tkl_gpio_write(TUYA_GPIO_NUM_7, TUYA_GPIO_LEVEL_HIGH);
    tkl_gpio_write(TUYA_GPIO_NUM_8, TUYA_GPIO_LEVEL_HIGH);
    tal_system_sleep(200);
    tkl_gpio_write(TUYA_GPIO_NUM_7, TUYA_GPIO_LEVEL_LOW);
    tal_system_sleep(300);
    tkl_gpio_write(TUYA_GPIO_NUM_8, TUYA_GPIO_LEVEL_LOW);
    tal_system_sleep(50);
    tkl_gpio_write(TUYA_GPIO_NUM_8, TUYA_GPIO_LEVEL_HIGH);
    tal_system_sleep(300);
}

/**
 * @brief       close and reopen camera device until sensor id reads back
 * @param[in]   none
 * @retval      OPRT_OK:success
 */
OPERATE_RET camera_restart(void)
{
    OPERATE_RET rt = OPRT_COM_ERROR;

    if (NULL == sg_tdl_camera_hdl) {
        return OPRT_NOT_FOUND;
    }

    for (int attempt = 0; attempt < 5; attempt++) {
        __camera_sensor_power_cycle();
        tdl_camera_dev_close(sg_tdl_camera_hdl);
        rt = __camera_open();
        tal_system_sleep(300);
        if ((OPRT_OK == rt) && __camera_sensor_id_ok()) {
            PR_NOTICE("camera restart ok, attempt:%d", attempt);
            return OPRT_OK;
        }
        PR_NOTICE("camera sensor not ready, attempt:%d rt:%d", attempt, rt);
    }

    PR_NOTICE("camera restart failed rt:%d", rt);
    __camera_i2c_probe();

    return rt;
}

/**
 * @brief       camera init
 * @param[in]   None
 * @retval      OPRT_OK:success
 */
OPERATE_RET camera_init(void)
{
    OPERATE_RET rt = OPRT_OK;

    TUYA_CALL_ERR_RETURN(__imgproc_init());

    TUYA_CALL_ERR_LOG(servo_init());                        /* Init pan-tilt servo uart */

    sg_tdl_camera_hdl = tdl_camera_find_dev(CAMERA_NAME);   /* Find camera device */
    if(NULL == sg_tdl_camera_hdl) {
        PR_ERR("camera dev %s not found", CAMERA_NAME);
        return OPRT_NOT_FOUND;
    }

    TUYA_CALL_ERR_RETURN(__camera_open());                  /* Open camera device */

    if (!__camera_sensor_id_ok()) {
        TUYA_CALL_ERR_LOG(camera_restart());
    }

    __camera_i2c_probe(); /* Probe camera sensor i2c address/id */

    return OPRT_OK;
}

