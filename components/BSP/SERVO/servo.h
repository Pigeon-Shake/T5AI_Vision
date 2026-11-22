/******************************************************************
 * @file        servo.h
 * @brief       UART servo controller driver for pan-tilt tracking
 ******************************************************************
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "tal_api.h"
#include "vision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART1 is the default log port, use UART0 (TX:GPIO11 RX:GPIO10) instead */
#define SERVO_UART_PORT        TUYA_UART_NUM_0
#define SERVO_UART_BAUDRATE    9600
#define SERVO_UART_RX_BUF_SIZE 64

#define SERVO_PAN_NUM  3 /* horizontal servo */
#define SERVO_TILT_NUM 2 /* vertical servo */

#define SERVO_ANGLE_MIN  0
#define SERVO_ANGLE_MAX  180
#define SERVO_PAN_INIT   90
#define SERVO_TILT_INIT  170

#define SERVO_SELFTEST_ENABLE    1
#define SERVO_SELFTEST_SWEEP_DEG 30
#define SERVO_SELFTEST_DELAY_MS  400

/* Flip the sign if the gimbal moves away from the target */
#define SERVO_PAN_DIR  (-1)
#define SERVO_TILT_DIR (1)

#define SERVO_TRACK_INTERVAL_MS  300 /* min interval between commands */
#define SERVO_TRACK_DEADZONE_PIX 12  /* ignore error within this range */
#define SERVO_TRACK_KP_NUM       2   /* gain = KP_NUM / KP_DEN deg/pixel */
#define SERVO_TRACK_KP_DEN       100
#define SERVO_TRACK_MAX_STEP_DEG 1   /* max angle change per update */

OPERATE_RET servo_init(void);
OPERATE_RET servo_set_angle(uint8_t servonum, uint16_t angle);
void        servo_track_update(const VISION_RESULT_T *result, uint16_t img_width, uint16_t img_height);
void        servo_get_angles(int16_t *pan, int16_t *tilt);
void        servo_sync_angle(uint8_t servonum, uint16_t angle);
bool        servo_set_param(const char *name, int32_t value);
void        servo_dump_params(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
