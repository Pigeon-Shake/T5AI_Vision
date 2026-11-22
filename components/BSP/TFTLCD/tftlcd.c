/******************************************************************
 * @file        tftlcd.c
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2025-07-25
 * @brief       TFT LCD driver code
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

#include "tftlcd.h"
#include "lcdfont.h"

TDL_DISP_HANDLE_T sg_tdl_disp_hdl = NULL;
TDL_DISP_FRAME_BUFF_T *sg_p_display_fb = NULL;
TDL_DISP_DEV_INFO_T sg_display_info;

_tftlcd_dev tftdev;     /* TFT LCD device structure */


/**
 * @brief Registers an TFT display device with the specified configuration.
 *
 * This function initializes the TFT display device with the provided configuration,
 * registers it, and prepares it for use by setting up the display handle and frame buffer.
 *
 * @param name Name of the display device to register.
 * @param dev_cfg Pointer to the configuration structure for the TFT display device.
 *
 * @return Returns OPRT_OK on success, or an appropriate error code if registration fails.
 */
OPERATE_RET tdd_disp_atk_tftlcd_register(void)
{

    OPERATE_RET rt = OPRT_OK;
    uint32_t frame_len = 0;

    /* hardware register */
    board_register_hardware();

    memset(&sg_display_info, 0, sizeof(TDL_DISP_DEV_INFO_T));

    sg_tdl_disp_hdl = tdl_disp_find_dev(DISPLAY_NAME);                          /* Find the display device by name */

    if(NULL == sg_tdl_disp_hdl) 
    {
        PR_ERR("display dev %s not found", DISPLAY_NAME);
        return OPRT_NOT_FOUND;
    }

    rt = tdl_disp_dev_get_info(sg_tdl_disp_hdl, &sg_display_info);              /* Get display device information */
    if(rt != OPRT_OK) 
    {
        PR_ERR("get display dev info failed, rt: %d", rt);
        return OPRT_NOT_FOUND;
    }

    tftdev.width = sg_display_info.width;
    tftdev.height = sg_display_info.height;

    rt = tdl_disp_dev_open(sg_tdl_disp_hdl);                                    /* Open the display device */
    if(rt != OPRT_OK) 
    {
        PR_ERR("open display dev failed, rt: %d", rt);
        return OPRT_NOT_FOUND;
    }
    tdl_disp_set_brightness(sg_tdl_disp_hdl, 100);                              /* Set brightness to 100% */

    frame_len = sg_display_info.width * sg_display_info.height * 2;             /* Calculate frame buffer length based on width, height, and pixel format */
    PR_DEBUG("frame_len: %d; width: %d; height: %d;", frame_len, sg_display_info.width, sg_display_info.height);
    sg_p_display_fb = tdl_disp_create_frame_buff(DISP_FB_TP_PSRAM, frame_len);  /* Create a frame buffer in PSRAM */

    if(NULL == sg_p_display_fb) 
    {
        PR_ERR("create display frame buff failed");
        return OPRT_MALLOC_FAILED;
    }
    
    sg_p_display_fb->fmt    = sg_display_info.fmt;
    sg_p_display_fb->width  = sg_display_info.width;
    sg_p_display_fb->height = sg_display_info.height;
    tftdev.display_fb = sg_p_display_fb;    /* Set the display frame buffer pointer */
    tftdev.disp_hdl = sg_tdl_disp_hdl;      /* Set the display handle */
    tftlcd_clear(WHITE);                    /* Clear the display to white */
    return rt;
}

/**
 * @brief Color byte order conversion function
 * @note  RGB LCD does not require byte order conversion, SPI LCD requires byte order conversion
 */
static inline uint16_t swap16_u16(uint16_t v)
{
#if defined(ATK_T5AI_MINI_BOARD_LCD_MD0700R_RGB)
    /* RGB LCD: no byte order conversion needed */
    return v; 
#else
    /* SPI LCD: byte order conversion needed */
    return (uint16_t)((v >> 8) | (v << 8));
#endif
}

/**
 * @brief Draws a point on the TFT LCD display at specified coordinates with a given color.
 *
 * @param x X-coordinate of the point to draw.
 * @param y Y-coordinate of the point to draw.
 * @param color Color of the point in RGB565 format.
 */
void tftlcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    if (!sg_p_display_fb) return;
    if (x >= sg_p_display_fb->width || y >= sg_p_display_fb->height) return;

    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;
    // fb[y * sg_p_display_fb->width + x] = color;
    fb[y * sg_p_display_fb->width + x] = swap16_u16(color);

    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Draws a line on the TFT LCD display between two points (x1, y1) and (x2, y2) with a specified color.
 *
 * This function implements Bresenham's line algorithm to draw a line efficiently on the display.
 *
 * @param x1 X-coordinate of the starting point of the line.
 * @param y1 Y-coordinate of the starting point of the line.
 * @param x2 X-coordinate of the ending point of the line.
 * @param y2 Y-coordinate of the ending point of the line.
 * @param color Color of the line in RGB565 format.
 */
void tftlcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (!sg_p_display_fb) return;
    uint16_t width = sg_p_display_fb->width;
    uint16_t height = sg_p_display_fb->height;
    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;

    int dx = abs((int)x2 - (int)x1);
    int dy = abs((int)y2 - (int)y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;
    int e2;

    while (1) {
        if (x1 < width && y1 < height)
            fb[y1 * width + x1] = color;
        if (x1 == x2 && y1 == y2) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x1 += sx; }
        if (e2 < dy)  { err += dx; y1 += sy; }
    }

    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Draws a rectangle on the TFT LCD display defined by two corners (x0, y0) and (x1, y1) with a specified color.
 *
 * This function draws the four edges of the rectangle by calling tftlcd_draw_line for each edge.
 *
 * @param x0 X-coordinate of the first corner of the rectangle.
 * @param y0 Y-coordinate of the first corner of the rectangle.
 * @param x1 X-coordinate of the opposite corner of the rectangle.
 * @param y1 Y-coordinate of the opposite corner of the rectangle.
 * @param color Color of the rectangle in RGB565 format.
 */
void tftlcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color)
{
    tftlcd_draw_line(x0, y0, x1, y0,color);
    tftlcd_draw_line(x0, y0, x0, y1,color);
    tftlcd_draw_line(x0, y1, x1, y1,color);
    tftlcd_draw_line(x1, y0, x1, y1,color);
}

/**
 * @brief Draws a horizontal line on the TFT LCD display starting from (x, y) with a specified length and color.
 *
 * @param x X-coordinate of the starting point of the line.
 * @param y Y-coordinate of the starting point of the line.
 * @param len Length of the line in pixels.
 * @param color Color of the line in RGB565 format.
 */
void tftlcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if (!sg_p_display_fb) return;
    if (y >= sg_p_display_fb->height) return;
    if (x + len > sg_p_display_fb->width) len = sg_p_display_fb->width - x;

    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;
    for (uint16_t i = 0; i < len; i++) {
        fb[y * sg_p_display_fb->width + x + i] = color;
    }
    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Draws a vertical line on the TFT LCD display starting from (x, y) with a specified length and color.
 *
 * @param x X-coordinate of the starting point of the line.
 * @param y Y-coordinate of the starting point of the line.
 * @param len Length of the line in pixels.
 * @param color Color of the line in RGB565 format.
 */
void tftlcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (!sg_p_display_fb) return;
    if (x >= sg_p_display_fb->width || y >= sg_p_display_fb->height) return;
    if (x + w > sg_p_display_fb->width) w = sg_p_display_fb->width - x;
    if (y + h > sg_p_display_fb->height) h = sg_p_display_fb->height - y;

    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;
    for (uint16_t j = 0; j < h; j++) 
    {
        for (uint16_t i = 0; i < w; i++) 
        {
            fb[(y + j) * sg_p_display_fb->width + (x + i)] = color;
        }
    }
    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Draws a circle on the TFT LCD display at specified coordinates with a given radius and color.
 *
 * @param x0 X-coordinate of the center of the circle.
 * @param y0 Y-coordinate of the center of the circle.
 * @param r Radius of the circle.
 * @param color Color of the circle in RGB565 format.
 */
void tftlcd_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    if (!sg_p_display_fb) return;
    int x = 0, y = r;
    int d = 1 - r;
    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;

    while (y >= x) 
    {
        #define PLOT(_x, _y) \
            if ((_x) < sg_p_display_fb->width && (_y) < sg_p_display_fb->height) \
                fb[(_y) * sg_p_display_fb->width + (_x)] = color;
        PLOT(x0 + x, y0 + y);
        PLOT(x0 - x, y0 + y);
        PLOT(x0 + x, y0 - y);
        PLOT(x0 - x, y0 - y);
        PLOT(x0 + y, y0 + x);
        PLOT(x0 - y, y0 + x);
        PLOT(x0 + y, y0 - x);
        PLOT(x0 - y, y0 - x);
        x++;
        if (d < 0) 
        {
            d += 2 * x + 1;
        } 
        else 
        {
            y--;
            d += 2 * (x - y) + 1;
        }
    }
    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Clears the TFT LCD display by filling it with a specified color.
 *
 * @param color Color to fill the display with in RGB565 format.
 */
void tftlcd_clear(uint16_t color)
{
    if (!sg_p_display_fb) return;
    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;
    uint32_t total = sg_p_display_fb->width * sg_p_display_fb->height;

    for (uint32_t i = 0; i < total; i++) 
    {
        fb[i] = swap16_u16(color);
    }
    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
}

/**
 * @brief Fills a circle on the TFT LCD display at specified coordinates with a given radius and color.
 *
 * This function uses Bresenham's circle algorithm to fill the circle efficiently.
 *
 * @param center_x X-coordinate of the center of the circle.
 * @param center_y Y-coordinate of the center of the circle.
 * @param radius Radius of the circle.
 * @param color Color to fill the circle with in RGB565 format.
 */
void tftlcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color)
{
    uint32_t i;
    uint32_t imax = ((uint32_t)radius * 707) / 1000 + 1;
    uint32_t sqmax = (uint32_t)radius * (uint32_t)radius + (uint32_t)radius / 2;
    uint32_t xr = radius;

    tftlcd_draw_hline(center_x - radius, center_y, 2 * radius, color);

    for (i = 1; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            if (xr > imax)
            {
                tftlcd_draw_hline (center_x - i + 1, center_y + xr, 2 * (i - 1), color);
                tftlcd_draw_hline (center_x - i + 1, center_y - xr, 2 * (i - 1), color);
            }

            xr--;
        }

        tftlcd_draw_hline(center_x - xr, center_y + i, 2 * xr, color);
        tftlcd_draw_hline(center_x - xr, center_y - i, 2 * xr, color);
    }
}

/**
 * @brief Displays a character on the TFT LCD display at specified coordinates with a given size, mode, and color.
 *
 * This function uses a font array to render the character pixel by pixel.
 *
 * @param x X-coordinate where the character will be displayed.
 * @param y Y-coordinate where the character will be displayed.
 * @param chr Character to display.
 * @param size Size of the character in pixels.
 * @param mode Display mode (0 for normal, 1 for inverse).
 * @param color Color of the character in RGB565 format.
 * @param flush Whether to flush the display immediately (1 = flush, 0 = no flush).
 */
void tftlcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color, uint8_t flush)
{
    if (!sg_p_display_fb) return;
    if (x >= sg_p_display_fb->width || y >= sg_p_display_fb->height) return;

    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize = 0;
    uint8_t *pfont = 0;
    uint16_t *fb = (uint16_t *)sg_p_display_fb->frame;
    uint16_t width = sg_p_display_fb->width;
    uint16_t height = sg_p_display_fb->height;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
    chr = (char)chr - ' ';

    switch (size)
    {
        case 12:
            pfont = (uint8_t *)asc2_1206[(uint8_t)chr];
            break;
        case 16:
            pfont = (uint8_t *)asc2_1608[(uint8_t)chr];
            break;
        case 24:
            pfont = (uint8_t *)asc2_2412[(uint8_t)chr];
            break;
        case 32:
            pfont = (uint8_t *)asc2_3216[(uint8_t)chr];
            break;
        default:
            return ;
    }

    /* Batch update flag - only refresh once after all pixels are written */
    bool need_update = false;

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t];
        for (t1 = 0; t1 < 8; t1++)
        {
            uint16_t px = x;
            uint16_t py = y;
            if (temp & 0x80)
            {
                if (px < width && py < height) 
                {
                    fb[py * width + px] = swap16_u16(color);
                    need_update = true;
                }
            }
            else if (mode == 0)
            {
                if (px < width && py < height)
                {
                    fb[py * width + px] = 0xFFFF;
                    need_update = true;
                }
            }
            temp <<= 1;
            y++;
            if (y >= height) {
                if(need_update && flush) {
                    tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
                }
                return;
            }


            if ((y - y0) == size)
            {
                y = y0;
                x++;
                if (x >= width) {
                    if(need_update && flush) {
                        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
                    }
                    return;
                }

                break;
            }
        }
    }
    /* Only refresh the display when pixels have been updated and flush is requested */
    if(need_update && flush)
    {
        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
    }
}

/**
 * @brief Calculates the power of a number.
 *
 * This function computes m raised to the power of n (m^n).
 *
 * @param m Base number.
 * @param n Exponent.
 * @return Returns the result of m raised to the power of n.
 */
static uint32_t tftlcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

/**
 * @brief Displays a number on the TFT LCD display at specified coordinates with a given length, size, and color.
 *
 * This function formats the number as a string and displays it character by character.
 *
 * @param x X-coordinate where the number will be displayed.
 * @param y Y-coordinate where the number will be displayed.
 * @param num The number to display.
 * @param len Length of the number (number of digits).
 * @param size Size of the characters in pixels.
 * @param color Color of the number in RGB565 format.
 */
void tftlcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / tftlcd_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                tftlcd_show_char(x + (size / 2) * t, y, ' ', size, 0, color, 0);  // flush = 0
                continue;
            }
            else
            {
                enshow = 1;
            }
        }

        tftlcd_show_char(x + (size / 2) * t, y, temp + '0', size, 0, color, 0);  // flush = 0
    }

    /* Flush the display once after all digits are shown */
    if (sg_tdl_disp_hdl && sg_p_display_fb)
    {
        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
    }
}

/**
 * @brief Displays a number on the TFT LCD display at specified coordinates with a given length, size, mode, and color.
 *
 * This function formats the number as a string and displays it character by character,
 * allowing for leading zeros or spaces based on the mode.
 *
 * @param x X-coordinate where the number will be displayed.
 * @param y Y-coordinate where the number will be displayed.
 * @param num The number to display.
 * @param len Length of the number (number of digits).
 * @param size Size of the characters in pixels.
 * @param mode Display mode (0 for normal, 1 for inverse, 0x80 for leading zero).
 * @param color Color of the number in RGB565 format.
 */
void tftlcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / tftlcd_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode & 0x80)
                {
                    tftlcd_show_char(x + (size / 2) * t, y, '0', size, mode & 0x01, color, 0);  // flush = 0
                }
                else
                {
                    tftlcd_show_char(x + (size / 2) * t, y, ' ', size, mode & 0x01, color, 0);  // flush = 0
                }

                continue;
            }
            else
            {
                enshow = 1;
            }

        }

        tftlcd_show_char(x + (size / 2) * t, y, temp + '0', size, mode & 0x01, color, 0);  // flush = 0
    }

    /* Flush all digits to the display at once after display is complete */
    if (sg_tdl_disp_hdl && sg_p_display_fb)
    {
        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
    }
}

/**
 * @brief Displays a string on the TFT LCD display at specified coordinates with a given width, height, size, and color.
 *
 * This function iterates through each character in the string and displays it on the LCD,
 * wrapping to the next line if necessary.
 *
 * @param x X-coordinate where the string will be displayed.
 * @param y Y-coordinate where the string will be displayed.
 * @param width Width of the area to display the string.
 * @param height Height of the area to display the string.
 * @param size Size of the characters in pixels.
 * @param p Pointer to the string to display.
 * @param color Color of the string in RGB565 format.
 */
void tftlcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color)
{
    uint8_t x0 = x;

    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' '))
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }

        if (y >= height)
        {
            break;
        }

        tftlcd_show_char(x, y, *p, size, 0, color, 0);  /* flush = 0, no immediate refresh */
        x += size / 2;
        p++;
    }

    /* Refresh display once after all characters are shown */
    if (sg_tdl_disp_hdl && sg_p_display_fb)
    {
        tdl_disp_dev_flush(sg_tdl_disp_hdl, sg_p_display_fb);
    }
}
