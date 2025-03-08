#include "LCDScreen.h"



//////////////////////////////////////////////////////
/////////////////// TOUCH METHOD /////////////////////
//////////////////////////////////////////////////////

void init_touchscreen(struct Touch* touch){
    // Initialize gpio pins
    if (gpioSetMode(IRQ_TOUCH, PI_INPUT) != 0){
        //std::cerr << "Failed to set mode on one or more pins" << std::endl;
    }
    memset(touch, 0, sizeof(struct Touch));
    touch->handle = spiOpen(SPICHAN_TOUCH, BAUDRATE_TOUCH, SPIFLAGS);
    if(touch->handle < 0){
        //std::cerr << "Failed to open SPI Port Touch Screen" << std::endl;
    }
}

// MUST be called
void close_touchscreen(struct Touch* touch){
    spiClose(touch->handle);
}

bool is_touched(struct Touch* touch){
    // Read the IRQ pin from the LCD touchscreen
    // If LOW, the screen is touched
    if(gpioRead(IRQ_TOUCH) == LOW){
        if(!touch->is_touched){       // If it was not touch, before, it's press
            touch->is_touched = true; // Used to touch one time
            touch->type = ONE_CLICK; // A single click
            return true;        // Return true for the caller
        }
    // If it's high and m_touched is true, it was a release
    }else if(gpioRead(IRQ_TOUCH) == HIGH && touch->is_touched){
        touch->is_touched = false;    // So touch is false
        touch->type = RELEASE;       // And the type is RELEASE
        
        return true;
    }
    return false;
}


uint16_t read_data(struct Touch* touch, enum TouchCommand command){
    char txBuffer[3] = { command, 0, 0 };
    char rxBuffer[3] = { 0, 0, 0 };

    spiXfer(touch->handle, txBuffer, rxBuffer, 3);

    // The data to decode are in 12 bits size. We store them in a 16 bits variable
    uint16_t result = (uint16_t)(((rxBuffer[1] << 8) | rxBuffer[2]) >> 3);
    return result;
}

uint16_t read_x_coord(struct Touch* touch) {
    // Command for reading X coordinates
    // 0xD0 is for 12-bit X read
    return read_data(touch, X_READ_COMMAND);   
}

uint16_t read_y_coord(struct Touch* touch) {
    // Command for reading Y coordinates
    // 0x90 is for 12-bit Y read
    return read_data(touch, Y_READ_COMMAND);   
}

void map_to_screen(struct Position* raw, struct Position* screen) {
    // The position received by the LCD MCU is the difference of voltage between VREF and GND
    // The boundary limits are written in the variable MIN_TOUCH_x and MAX_TOUCH_x
    screen->x = WIDTH - (raw->x - MIN_TOUCH_X) * WIDTH / (MAX_TOUCH_X - MIN_TOUCH_X);
    screen->y = HEIGHT - (raw->y - MIN_TOUCH_Y) * HEIGHT / (MAX_TOUCH_Y - MIN_TOUCH_Y);
}

////////////////////////////////////////////////////
/////////////////// LCD METHOD /////////////////////
////////////////////////////////////////////////////

void init_lcd_screen(struct LCDScreen* lcd)
{
    // Initialize gpio pins
    if (gpioSetMode(RESET, PI_OUTPUT) != 0 ||
        gpioSetMode(DCRS, PI_OUTPUT) != 0)
    {
        //std::cerr << "Failed to set mode on one or more pins" << std::endl;
    }
    memset(lcd, 0, sizeof(struct LCDScreen));
    lcd->handle = spiOpen(SPICHAN_LCD, BAUDRATE_LCD, SPIFLAGS);
    if(lcd->handle < 0)
    {
        //std::cerr << "Failed to open SPI Port LCD Screen" << std::endl;
    }


    // Initialisation Sequence
    //Reset the LCD Screen
    gpioWrite(RESET, HIGH);
    usleep(120 * 1000); // wait 120 ms
    gpioWrite(RESET, LOW);
    usleep(120 * 1000); // wait 120 ms
    gpioWrite(RESET, HIGH);
    usleep(120 * 1000); // wait 120 ms

    // Software reset
    write_command(lcd, SOFTWARE_RST);
    usleep(120 * 1000);


    // 0x3A Interface Pixel Format (bit depth color space)
    write_command(lcd, INTERFACE_PIXEL_FORMAT);
    write_fixed_data(lcd, 0X66);

    write_command(lcd, TEARING_EFFECT_LINE_ON);
    write_fixed_data(lcd, 0x1);

    write_command(lcd, MEMORY_ACCESS_CONTROL);
    write_fixed_data(lcd, 0x20);

    write_command(lcd, FRAME_RATE_CONTROL);
    write_fixed_data(lcd, 0xA1);
    write_fixed_data(lcd, 0x1F);

    write_command(lcd, ADJUST_CONTROL_3);
    write_fixed_data(lcd, 0xA9);
    write_fixed_data(lcd, 0x51);
    write_fixed_data(lcd, 0x2C);
    write_fixed_data(lcd, 0x02);
    

    // 0x11 Exit Sleep Mode. (Sleep OUT)
    write_command(lcd, SLEEP_OUT);
    usleep(120*1000);

    // 0x29 Display ON.
    write_command(lcd, DISPLAY_ON);

    lcd->draw_buffer = (char*)malloc(3 * (WIDTH) * (HEIGHT) * sizeof(char));
}

// MUST be called at the end of use
void close_lcd_screen(struct LCDScreen* lcd)
{
    spiClose(lcd->handle);

    free(lcd->draw_buffer);
}

bool touch_screen(struct LCDScreen* lcd, struct Touch* touch, struct Position* screen_pos){
    // Check if the screen was touched
    if(is_touched(touch)){
        // Get its type
        enum Click type = touch->type;
        // If it is RELEASE type, we don't calculate the coordinates and keep the coordinates of the first click
        // TODO: Calculate the release coordinates
        if(type != RELEASE){
            // Get the raw touch pos(without mapping)
            struct Position raw_touch_pos = {
                .x = read_y_coord(touch),
                .y = read_x_coord(touch),  
            }; // The value are inverted for the screen (idk why)
            // And map them to the real pixels coordinates
            map_to_screen(&raw_touch_pos, screen_pos);
        }

        usleep(50 * 1000); // sleep for 50 ms
        return true;
    }
    return false;
}

void write_command(struct LCDScreen* lcd, enum LCDCommand command)
{
    gpioWrite(DCRS, LOW); // LOW means we send commands to the LCD
    int success = spiWrite(lcd->handle, (char*)(&command), 1);
    if (success < 0){
        //std::cerr << "Error: " << success << " while sending command" << std::endl;
    }
}


void write_fixed_data(struct LCDScreen* lcd, uint8_t data)
{
    gpioWrite(DCRS, HIGH); // HIGH means we send data to the LCD
    int success = spiWrite(lcd->handle, (char*)(&data), 1);
    if (success < 0){
        //std::cerr << "Error: " << success << " while sending data" << std::endl;
    }
}


void write_data(struct LCDScreen* lcd, char* data, unsigned int size){
    gpioWrite(DCRS, HIGH);
    const int chunkSize = 65536;  // Chunk size (adjust as necessary)
    int totalSent = 0;
    
    // Loop through the data in chunks
    while (totalSent < size) {
        int remaining = (int)(size - totalSent);
        int toSend = (remaining > chunkSize) ? chunkSize : remaining;

        
        int success = spiWrite(lcd->handle, data + totalSent, toSend);
        

        if (success < 0) {
            //std::cerr << "Error: " << success << " while sending data" << std::endl;
            break;  // Exit loop on error
        }
        
        totalSent += toSend;
    }
}


void set_address(struct LCDScreen* lcd, int x1, int y1, int x2, int y2)
{
    write_command(lcd, COLUMN_ADDRESS_SET);
	write_fixed_data(lcd, x1 >> 8);
	write_fixed_data(lcd, x1);
	write_fixed_data(lcd, x2 >> 8);
	write_fixed_data(lcd, x2);

    lcd->SC = x1;
    lcd->EC = x2;

    if(lcd->SC < 0)
        lcd->SC = 0;
    else if(lcd->SC > WIDTH)
        lcd->SC = WIDTH;
    
    if(lcd->EC < 0)
        lcd->EC = 0;
    else if(lcd->EC > WIDTH)
        lcd->EC = WIDTH;
    
    write_command(lcd, PAGE_ADDRESS_SET);
	write_fixed_data(lcd, y1 >> 8);
	write_fixed_data(lcd, y1);
	write_fixed_data(lcd, y2 >> 8);
	write_fixed_data(lcd, y2);	 

    lcd->SP = y1;
    lcd->EP = y2;

    if(lcd->SP < 0)
        lcd->SP = 0;
    else if(lcd->SP > HEIGHT)
        lcd->SP = HEIGHT;

    if(lcd->EP < 0)
        lcd->EP = 0;
    else if(lcd->EP > HEIGHT)
        lcd->EP = HEIGHT;
    
}


void clear_screen(struct LCDScreen* lcd)
{
    set_address(lcd, 0, 0, WIDTH, HEIGHT); // Set the address of the entire screen
    write_command(lcd, MEMORY_WRITE);

    int white = 0xFFFFFF;
    // And write WHITE color to all pixels
    for(int i = 0; i < WIDTH; i++)
    {
        for(int m = 0; m < HEIGHT; m++)
        {
            write_fixed_data(lcd, (white >> 16) & 0xFC);
            write_fixed_data(lcd, (white >> 8) & 0xFC);
            write_fixed_data(lcd, white & 0xFC);
        }
    }
}


void set_pixel(struct LCDScreen* lcd, int x, int y, unsigned int color) {
    // Set Address
    set_address(lcd, x, y, x + 1, y + 1);
    // Write pixel data
    //WriteCommand(MEMORY_WRITE);  // Memory write

    uint8_t red = (uint8_t)((color >> 18) & 0x3F);   // Extract bits 23-18 for red
    uint8_t green = (uint8_t)((color >> 10) & 0x3F); // Extract bits 15-10 for green
    uint8_t blue = (uint8_t)((color >> 2) & 0x3F);   // Extract bits 7-2 for blue

    unsigned char buffer[3] = {red, green, blue};
    set_value_buffer(lcd, buffer, 3);

    // write_data(lcd, red << 2);
    // write_data(lcd, green << 2);
    // write_data(lcd, blue << 2);
}

void draw_fixed_frame(struct LCDScreen* lcd){
    
    set_address(lcd, 0, 0, WIDTH, HEIGHT);
    write_command(lcd, MEMORY_WRITE);

    write_data(lcd,  lcd->draw_buffer, 3 * (WIDTH) * (HEIGHT));
}

void draw_frame(struct LCDScreen* lcd, unsigned int size){
    set_address(lcd, 0, 0, WIDTH, HEIGHT);
    write_command(lcd, MEMORY_WRITE);
    
    write_data(lcd, lcd->draw_buffer, size);
}


void set_value_buffer(struct LCDScreen* lcd, unsigned char* val, unsigned int size){
    int col = lcd->SC * 3;
    int row = lcd->SP;
    for(int i = 0; i < size; i++){
        
        int idx = col + (WIDTH) * 3 * row;
        lcd->draw_buffer[idx] = val[i];
        col++;
        if(col >= lcd->EC * 3){ // 3 for RGB
            col = lcd->SC * 3;
            row++;
        }
        if(row >= lcd->EP){
            break;
        }
    }
}

