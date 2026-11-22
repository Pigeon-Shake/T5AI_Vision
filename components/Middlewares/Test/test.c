/******************************************************************
 * @file        test.c
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2025-07-25
 * @brief       Runtime tuning commands over the log uart (BK shell)
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

#include "test.h"

#include <stdlib.h>
#include <string.h>

#include "tal_api.h"
#include "camera.h"
#include "servo.h"
#include "vision.h"

#if ENABLE_BK_CLI_TUNING
#include "bk_private/bk_cli.h"

static void cli_vget_cb(char *buf, int len, int argc, char **argv)
{
    const VISION_RESULT_T *result = camera_get_vision_result();

    (void)buf;
    (void)len;
    (void)argc;
    (void)argv;

    vision_dump_params();
    servo_dump_params();
    PR_NOTICE("last result target:%d thr:%u edge:%u center:(%d,%d)", result->target_found, result->threshold,
              result->edge_count, result->center.x, result->center.y);
}

static void cli_vset_cb(char *buf, int len, int argc, char **argv)
{
    int32_t value;

    (void)buf;
    (void)len;

    if (argc < 3) {
        PR_NOTICE("usage: vset <name> <value>");
        PR_NOTICE("vision: thr canny_lo canny_hi min_area margin morph eps proc");
        PR_NOTICE("chroma: cmode u_lo u_hi v_lo v_hi y_lo y_hi cphase");
        PR_NOTICE("servo: track interval deadzone kp step pandir tiltdir");
        return;
    }

    value = (int32_t)strtol(argv[2], NULL, 0);
    if (vision_set_param(argv[1], value) || servo_set_param(argv[1], value)) {
        PR_NOTICE("vset %s=%d ok", argv[1], value);
    } else {
        PR_NOTICE("vset %s=%d failed, unknown name or out of range", argv[1], value);
    }
}

static void cli_vservo_cb(char *buf, int len, int argc, char **argv)
{
    (void)buf;
    (void)len;

    if (argc < 3) {
        PR_NOTICE("usage: vservo <num> <angle>");
        return;
    }

    servo_set_angle((uint8_t)atoi(argv[1]), (uint16_t)atoi(argv[2]));
    servo_sync_angle((uint8_t)atoi(argv[1]), (uint16_t)atoi(argv[2]));
    PR_NOTICE("vservo num:%s angle:%s sent", argv[1], argv[2]);
}

static void cli_vimg_cb(char *buf, int len, int argc, char **argv)
{
    (void)buf;
    (void)len;
    (void)argc;
    (void)argv;

    camera_request_dump();
    PR_NOTICE("binary image dump requested");
}

static void cli_vraw_cb(char *buf, int len, int argc, char **argv)
{
    (void)buf;
    (void)len;
    (void)argc;
    (void)argv;

    camera_request_raw_dump();
    PR_NOTICE("raw yuyv dump requested");
}

static void cli_vcam_cb(char *buf, int len, int argc, char **argv)
{
    (void)buf;
    (void)len;
    (void)argc;
    (void)argv;

    camera_restart();
}

static const struct cli_command sg_tuning_cmds[] = {
    {"vget", "dump vision/servo params and last result", cli_vget_cb},
    {"vset", "vset <name> <value>, tune vision/servo params", cli_vset_cb},
    {"vservo", "vservo <num> <angle>, manual servo command", cli_vservo_cb},
    {"vimg", "dump binary image over log uart", cli_vimg_cb},
    {"vraw", "dump raw yuyv image over log uart", cli_vraw_cb},
    {"vcam", "close and reopen camera device", cli_vcam_cb},
};
#endif /* ENABLE_BK_CLI_TUNING */

/**
 * @brief       register tuning commands on the log uart shell
 * @param[in]   none
 * @return      none
 */
void test_init(void)
{
#if ENABLE_BK_CLI_TUNING
    cli_register_commands(sg_tuning_cmds, sizeof(sg_tuning_cmds) / sizeof(sg_tuning_cmds[0]));
    PR_NOTICE("tuning cli ready: vget vset vservo vimg");
#endif
}
