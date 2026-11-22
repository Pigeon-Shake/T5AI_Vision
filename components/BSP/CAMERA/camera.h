
/******************************************************************
 * @file        camera.h
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

#ifndef __CAMERA_H
#define __CAMERA_H

#include "tal_api.h"
#include "tkl_output.h"
#include "board_com_api.h"
#include "tdl_camera_manage.h"
#include "tftlcd.h"
#include "vision.h"

#if defined(ENABLE_DMA2D) && (ENABLE_DMA2D == 1)
#include "tkl_dma2d.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif




/* Function Declaration */
OPERATE_RET camera_init(void);
OPERATE_RET camera_restart(void);
void camera_print_stat(void);
void camera_request_dump(void);
void camera_request_raw_dump(void);
void camera_dump_service(void);
const VISION_RESULT_T *camera_get_vision_result(void);
void camera_get_luma(uint8_t *luma_min, uint8_t *luma_max);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
