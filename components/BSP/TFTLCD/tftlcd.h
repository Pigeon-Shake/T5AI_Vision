/******************************************************************
 * @file        tftlcd.h
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2025-07-25
 * @brief       TFTLCD driver code
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

#ifndef __TFTLCD_H
#define __TFTLCD_H

#include "tuya_cloud_types.h"
#include "tal_api.h"
#include "tdd_disp_type.h"
#include "tdl_display_driver.h"
#include "tkl_gpio.h"
#include "board_com_api.h"
#include "tdl_display_manage.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************
 * @brief       TFT LCD backlight control pin
 ******************************************************************/
#define LCD_BL_PIN          (TUYA_GPIO_NUM_15)


/******************************************************************
 * @brief       TFT LCD device structure
 ******************************************************************/
typedef struct  
{
    uint16_t id;                        /* Panel ID */
    uint16_t width;                     /* Display width in pixels */
    uint16_t height;                    /* Display height in pixels */
    TDL_DISP_FRAME_BUFF_T *display_fb;  /* Frame buffer pointer */
    TDL_DISP_HANDLE_T disp_hdl;         /* Display handle */
} _tftlcd_dev; 

/******************************************************************
 * @brief       TFT LCD color definitions
 ******************************************************************/
#define WHITE           0xFFFF
#define BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x001F
#define MAGENTA         0XF81F
#define YELLOW          0XFFE0
#define CYAN            0X07FF
#define BROWN           0XBC40
#define BRRED           0XFC07
#define GRAY            0X8430
#define DARKBLUE        0X01CF
#define LIGHTBLUE       0X7D7C
#define GRAYBLUE        0X5458
#define LIGHTGREEN      0X841F
#define LGRAY           0XC618
#define LGRAYBLUE       0XA651
#define LBBLUE          0X2B12

/******************************************************************
 * @brief       TFT LCD display device handle
 ******************************************************************/
extern _tftlcd_dev tftdev;
extern TDL_DISP_HANDLE_T sg_tdl_disp_hdl;
extern TDL_DISP_FRAME_BUFF_T *sg_p_display_fb;
extern TDL_DISP_DEV_INFO_T sg_display_info;

/* Function Declaration */
OPERATE_RET tdd_disp_atk_tftlcd_register(void);
void tftlcd_draw_point(uint16_t x, uint16_t y, uint16_t color);
void tftlcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void tftlcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color);
void tftlcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color);
void tftlcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void tftlcd_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void tftlcd_clear(uint16_t color);
void tftlcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color);
void tftlcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color, uint8_t flush);
void tftlcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color);
void tftlcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color);
void tftlcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color);
#ifdef __cplusplus
} // extern "C"
#endif

#endif
