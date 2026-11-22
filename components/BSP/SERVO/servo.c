/******************************************************************
 * @file        servo.c
 * @brief       UART servo controller driver for pan-tilt tracking
 *
 * Frame format (6 bytes, 115200 8N1):
 *   0x24 | servonum + 64 | angle ASCII hundreds | tens | units | 0x23
 ******************************************************************
 */

#include "servo.h"

#include <string.h>

static bool       sg_ready         = false;
static int16_t    sg_pan_angle     = SERVO_PAN_INIT;
static int16_t    sg_tilt_angle    = SERVO_TILT_INIT;
static SYS_TIME_T sg_last_track_ms = 0;

static bool    sg_track_enable = true;
static int32_t sg_interval_ms  = SERVO_TRACK_INTERVAL_MS;
static int32_t sg_deadzone     = SERVO_TRACK_DEADZONE_PIX;
static int32_t sg_kp_x100      = SERVO_TRACK_KP_NUM * 100 / SERVO_TRACK_KP_DEN;
static int32_t sg_max_step     = SERVO_TRACK_MAX_STEP_DEG;
static int32_t sg_pan_dir      = SERVO_PAN_DIR;
static int32_t sg_tilt_dir     = SERVO_TILT_DIR;

static int16_t servo_clamp_angle(int32_t angle)
{
    if (angle < SERVO_ANGLE_MIN) {
        return SERVO_ANGLE_MIN;
    }
    if (angle > SERVO_ANGLE_MAX) {
        return SERVO_ANGLE_MAX;
    }

    return (int16_t)angle;
}

static int32_t servo_abs(int32_t value)
{
    return value < 0 ? -value : value;
}

/**
 * @brief       send one angle command frame to the servo controller
 * @param[in]   servonum: servo index (2:pan 3:tilt)
 * @param[in]   angle: target angle in degrees (0~180)
 * @return      OPRT_OK:success
 */
OPERATE_RET servo_set_angle(uint8_t servonum, uint16_t angle)
{
    uint8_t frame[6];
    int     ret;

    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    frame[0] = 0x24;
    frame[1] = (uint8_t)(servonum + 64);
    frame[2] = (uint8_t)('0' + angle / 100);
    frame[3] = (uint8_t)('0' + (angle / 10) % 10);
    frame[4] = (uint8_t)('0' + angle % 10);
    frame[5] = 0x23;

    ret = tal_uart_write(SERVO_UART_PORT, frame, sizeof(frame));

    return ((int)sizeof(frame) == ret) ? OPRT_OK : OPRT_COM_ERROR;
}

/**
 * @brief       sweep one servo back and forth to verify the link
 * @param[in]   servonum: servo index (2:pan 3:tilt)
 * @param[in]   init_angle: center angle of the sweep
 * @return      none
 */
static void servo_selftest_axis(uint8_t servonum, int16_t init_angle)
{
    servo_set_angle(servonum, servo_clamp_angle(init_angle - SERVO_SELFTEST_SWEEP_DEG));
    tal_system_sleep(SERVO_SELFTEST_DELAY_MS);
    servo_set_angle(servonum, servo_clamp_angle(init_angle + SERVO_SELFTEST_SWEEP_DEG));
    tal_system_sleep(SERVO_SELFTEST_DELAY_MS);
    servo_set_angle(servonum, init_angle);
    tal_system_sleep(SERVO_SELFTEST_DELAY_MS);
}

/**
 * @brief       init servo uart and center both servos
 * @param[in]   none
 * @return      OPRT_OK:success
 */
OPERATE_RET servo_init(void)
{
    TAL_UART_CFG_T cfg = {0};
    OPERATE_RET    rt;

    cfg.base_cfg.baudrate = SERVO_UART_BAUDRATE;
    cfg.base_cfg.databits = TUYA_UART_DATA_LEN_8BIT;
    cfg.base_cfg.stopbits = TUYA_UART_STOP_LEN_1BIT;
    cfg.base_cfg.parity   = TUYA_UART_PARITY_TYPE_NONE;
    cfg.rx_buffer_size    = SERVO_UART_RX_BUF_SIZE;
    cfg.open_mode         = O_BLOCK;

    rt = tal_uart_init(SERVO_UART_PORT, &cfg);
    if (OPRT_OK != rt) {
        PR_ERR("servo uart init failed:%d", rt);
        return rt;
    }

    sg_pan_angle  = SERVO_PAN_INIT;
    sg_tilt_angle = SERVO_TILT_INIT;
    servo_set_angle(SERVO_PAN_NUM, SERVO_PAN_INIT);
    servo_set_angle(SERVO_TILT_NUM, SERVO_TILT_INIT);

#if SERVO_SELFTEST_ENABLE
    PR_NOTICE("servo selftest: pan/tilt sweep +/-%d deg", SERVO_SELFTEST_SWEEP_DEG);
    tal_system_sleep(SERVO_SELFTEST_DELAY_MS);
    servo_selftest_axis(SERVO_PAN_NUM, SERVO_PAN_INIT);
    servo_selftest_axis(SERVO_TILT_NUM, SERVO_TILT_INIT);
#endif

    sg_ready = true;

    PR_NOTICE("servo init ok, uart%d %d baud, pan:%d tilt:%d", SERVO_UART_PORT, SERVO_UART_BAUDRATE, sg_pan_angle,
              sg_tilt_angle);

    return OPRT_OK;
}

static int32_t servo_track_step(int32_t error)
{
    int32_t step;

    if (servo_abs(error) <= sg_deadzone) {
        return 0;
    }

    step = error * sg_kp_x100 / 100;
    if (0 == step) {
        step = (error > 0) ? 1 : -1;
    }
    if (step > sg_max_step) {
        step = sg_max_step;
    }
    if (step < -sg_max_step) {
        step = -sg_max_step;
    }

    return step;
}

/**
 * @brief       incremental P control, drive gimbal toward target center
 * @param[in]   result: vision result of current frame
 * @param[in]   img_width: image width in pixels
 * @param[in]   img_height: image height in pixels
 * @return      none
 */
void servo_track_update(const VISION_RESULT_T *result, uint16_t img_width, uint16_t img_height)
{
    SYS_TIME_T now;
    int32_t    err_x, err_y;
    int32_t    step;
    int16_t    angle;

    if (!sg_ready || !sg_track_enable || (NULL == result) || !result->target_found) {
        return;
    }
    if ((result->center.x < 0) || (result->center.y < 0)) {
        return;
    }

    now = tal_system_get_millisecond();
    if ((now - sg_last_track_ms) < (SYS_TIME_T)sg_interval_ms) {
        return;
    }
    sg_last_track_ms = now;

    err_x = (int32_t)result->center.x - img_width / 2;
    err_y = (int32_t)result->center.y - img_height / 2;

    step = servo_track_step(err_x);
    if (0 != step) {
        angle = servo_clamp_angle(sg_pan_angle + sg_pan_dir * step);
        if (angle != sg_pan_angle) {
            sg_pan_angle = angle;
            servo_set_angle(SERVO_PAN_NUM, (uint16_t)angle);
        }
    }

    step = servo_track_step(err_y);
    if (0 != step) {
        angle = servo_clamp_angle(sg_tilt_angle + sg_tilt_dir * step);
        if (angle != sg_tilt_angle) {
            sg_tilt_angle = angle;
            servo_set_angle(SERVO_TILT_NUM, (uint16_t)angle);
        }
    }
}

/**
 * @brief       get current gimbal angles
 * @param[out]  pan: pan servo angle
 * @param[out]  tilt: tilt servo angle
 * @return      none
 */
void servo_get_angles(int16_t *pan, int16_t *tilt)
{
    if (NULL != pan) {
        *pan = sg_pan_angle;
    }
    if (NULL != tilt) {
        *tilt = sg_tilt_angle;
    }
}

void servo_sync_angle(uint8_t servonum, uint16_t angle)
{
    if (SERVO_PAN_NUM == servonum) {
        sg_pan_angle = servo_clamp_angle(angle);
    } else if (SERVO_TILT_NUM == servonum) {
        sg_tilt_angle = servo_clamp_angle(angle);
    }
}

bool servo_set_param(const char *name, int32_t value)
{
    if (0 == strcmp(name, "track")) {
        sg_track_enable = (0 != value);
    } else if (0 == strcmp(name, "interval")) {
        if ((value < 10) || (value > 1000)) {
            return false;
        }
        sg_interval_ms = value;
    } else if (0 == strcmp(name, "deadzone")) {
        if ((value < 0) || (value > 100)) {
            return false;
        }
        sg_deadzone = value;
    } else if (0 == strcmp(name, "kp")) {
        if ((value < 0) || (value > 200)) {
            return false;
        }
        sg_kp_x100 = value;
    } else if (0 == strcmp(name, "step")) {
        if ((value < 1) || (value > 90)) {
            return false;
        }
        sg_max_step = value;
    } else if (0 == strcmp(name, "pandir")) {
        sg_pan_dir = (value < 0) ? -1 : 1;
    } else if (0 == strcmp(name, "tiltdir")) {
        sg_tilt_dir = (value < 0) ? -1 : 1;
    } else {
        return false;
    }

    return true;
}

void servo_dump_params(void)
{
    PR_NOTICE("servo params track:%d interval:%d deadzone:%d kp:%d step:%d pandir:%d tiltdir:%d", sg_track_enable,
              sg_interval_ms, sg_deadzone, sg_kp_x100, sg_max_step, sg_pan_dir, sg_tilt_dir);
    PR_NOTICE("servo angles pan:%d tilt:%d", sg_pan_angle, sg_tilt_angle);
}
