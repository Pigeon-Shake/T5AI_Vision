/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ALIENTEK
 * @brief       Camera Demo
 * @license     Copyright (C) 2020-2030, ALIENTEK
 ****************************************************************************************************
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
 ****************************************************************************************************
 */

#include "tal_api.h"
#include "tkl_output.h"
#include "board_com_api.h"

#include "tftlcd.h"
#include "camera.h"
#include "test.h"

/**
 * @brief user_main
 *
 * @param[in] param:Task parameters
 * @return none
 */
void user_main(void)
{
    tal_log_init(TAL_LOG_LEVEL_DEBUG, 1024, (TAL_LOG_OUTPUT_CB)tkl_log_output); /* Initialize log output */   
    tdd_disp_atk_tftlcd_register();    /* Register the TFTLCD LCD display */

    tftlcd_show_string(30, 50, 200, 16, 16, "T5 MINI Board", RED);
    tftlcd_show_string(30, 70, 200, 16, 16, "CAMERA TEST", RED);
    tftlcd_show_string(30, 90, 200, 16, 16, "ATOM@ALIENTEK", RED);

    camera_init();
    test_init();

    uint32_t tick = 0;
    uint32_t bad_cnt = 0;
    uint32_t restart_guard = 0;

    while(1) 
    {
        tal_system_sleep(100);
        camera_dump_service();
        if (++tick >= 50) {
            tick = 0;
            camera_print_stat();
            uint8_t lmin = 0, lmax = 0;

            camera_get_luma(&lmin, &lmax);
            if ((lmax - lmin) < 40) {
                bad_cnt++;
            } else {
                bad_cnt = 0;
            }
            if ((bad_cnt >= 3) && (restart_guard == 0)) {
                bad_cnt = 0;
                restart_guard = 6;
                camera_restart();
            }
            if (restart_guard > 0) {
                restart_guard--;
            }
        }
    }

}

/**
 * @brief       main
 *
 * @param       argc
 * @param       argv
 * @return      none
 */
#if OPERATING_SYSTEM == SYSTEM_LINUX
void main(int argc, char *argv[])
{
    user_main();

    while (1) {
        tal_system_sleep(500);
    }
}
#else

/**
 * @brief       The application entry point
 * 
 * @param[in]   none
 * @return      none
 */
void tuya_app_main(void)
{
    user_main();
}
#endif
