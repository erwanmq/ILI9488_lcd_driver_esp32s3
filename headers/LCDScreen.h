#pragma once

#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "driver/spi_master.h"


/* ESP32-S3-WROOM-1 pins */

// Define GPIO for LCD screen
#define LCD_DCRS    5
#define LCD_RESET   4
#define LCD_MISO 13
#define LCD_MOSI 11
#define LCD_CLK  12
#define LCD_CS   10

// Define GPIO for Touch Screen
#define TOUCH_IRQ 6
#define TOUCH_MISO LCD_MISO
#define TOUCH_MOSI LCD_MOSI
#define TOUCH_CLK  LCD_CLK
#define TOUCH_CS   39

// Spi Definition
#define BAUDRATE_LCD    20000000 // 20 MHz
#define BAUDRATE_TOUCH  50000 // 50KHz

// Pixels dimension
#define WIDTH   480
#define HEIGHT  320

// Min and Max (experimental) values of the edges of the touch screen (to test and to change based on yours)
#define MAX_TOUCH_X 3940
#define MAX_TOUCH_Y 3890
#define MIN_TOUCH_X 240
#define MIN_TOUCH_Y 200

// Command at the section 5 of the datasheet (http://www.lcdwiki.com/res/MSP3520/ILI9488%20Data%20Sheet.pdf)
enum LCDCommand{
    NOP = 0x0,
    SOFTWARE_RST,
    READ_DISPLAY_IDENTIFICATION_INFO    = 0x04,
    SLEEP_IN                            = 0x10,
    SLEEP_OUT,
    NORMAL_DISPLAY_MODE_ON              = 0x13,
    DISPLAY_INVERSION_OFF               = 0x20,
    ALL_PIXEL_OFF                       = 0x22,
    ALL_PIXEL_ON,
    DISPLAY_OFF                         = 0x28,
    DISPLAY_ON,
    COLUMN_ADDRESS_SET                  = 0x2A,
    PAGE_ADDRESS_SET,
    MEMORY_WRITE                        = 0x2C,
    TEARING_EFFECT_LINE_OFF             = 0x34,
    TEARING_EFFECT_LINE_ON,
    MEMORY_ACCESS_CONTROL               = 0x36,
    IDLE_MODE_OFF                       = 0x38,
    INTERFACE_PIXEL_FORMAT              = 0x3A,
    MEMORY_WRITE_CONTINUE               = 0x3C,
    INTERFACE_MODE_CONTROL              = 0xB0,
    FRAME_RATE_CONTROL,
    DISPLAY_INVERSION_CONTROL           = 0xB4,
    DISPLAY_FUNCTION_CONTROL            = 0xB6,
    POWER_CONTROL_1                     = 0xC0,
    POWER_CONTROL_2,
    VCOM_CONTROL                        = 0XC5,
    PGAMCTRL                            = 0xE0,     // Positive Gamma Control
    NGAMCTRL,                                       // Negative Gamma Control
    SET_IMAGE_FUNCTION                  = 0xE9,
    ADJUST_CONTROL_3                    = 0xF7,
};

enum Level{
    LOW = 0,
    HIGH
};

struct Position{
    int x;
    int y;
};

struct Size{
    int width;
    int height;
};

enum Click{
    ONE_CLICK,
    DOUBLE_CLICK,
    RELEASE,
    NONE
};

enum TouchCommand{
    X_READ_COMMAND = 0xD0,
    Y_READ_COMMAND = 0x90,
};


struct Touch{
    spi_device_handle_t spi;
    bool is_touched;
    enum Click type;
};

// LCD Screen ILI9488 in 4-wire SPI mode
// TODO: Revised the code to send video display without desynchronisation and artefacts
// TODO: Accelerate the display for video and add more fps
struct LCDScreen{
    spi_device_handle_t spi;

    char* draw_buffer;
    
    int SC; // Start column
    int SP; // Start page
    int EC; // End Column
    int EP; // End Page
};


void init_touchscreen(struct Touch* touch);
void close_touchscreen(struct Touch* touch);

bool is_touched(struct Touch* touch);
uint16_t read_data(struct Touch* touch, enum TouchCommand command);
uint16_t read_x_coord(struct Touch* touch);
uint16_t read_y_coord(struct Touch* touch);
void map_to_screen(struct Position* raw, struct Position* screen);

void init_lcd_screen(struct LCDScreen* lcd);
void close_lcd_screen(struct LCDScreen* lcd);
bool touch_screen(struct LCDScreen* lcd, struct Touch* touch, struct Position* screen_pos);
void write_command(struct LCDScreen* lcd, enum LCDCommand command);
void write_data(struct LCDScreen* lcd, char* data, int size);
void set_address(struct LCDScreen* lcd, int x1, int y1, int x2, int y2);
void clear_screen(struct LCDScreen* lcd);
void set_pixel(struct LCDScreen* lcd, int x, int y, unsigned int color);
void draw_frame(struct LCDScreen* lcd, unsigned int size);
void set_value_buffer(struct LCDScreen* lcd, unsigned char* val, unsigned int size);