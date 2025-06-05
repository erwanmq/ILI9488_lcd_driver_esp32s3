#include "LCDScreen.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

const char* TAG = "LCD Screen";
const char* TAG_TOUCH = "Touch Screen";
//////////////////////////////////////////////////////                                                                  
/////////////////// TOUCH METHOD /////////////////////                                                                  
//////////////////////////////////////////////////////                                                                                                                                                                                          
void open_spi(){                                                                                                            
    spi_bus_config_t buscfg = {                                                                                                 
        .miso_io_num = LCD_MISO,                                                                                                
        .mosi_io_num = LCD_MOSI,                                                                                                
        .sclk_io_num = LCD_CLK,                                                                                                 
        .quadhd_io_num = -1,                                                                                                    
        .quadwp_io_num = -1,                                                                                                    
        .max_transfer_sz = SPI_MAX_CHUNK_SIZE,                                                                              
    };                                                                                                                                                                                                                                              
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));                                          
}                                                                                                                                                                                                                                               

void close_spi(){                                                                                                           
    ESP_ERROR_CHECK(spi_bus_free(1));
}  

void touch_spi_touched_interrupt(void* args)                                                                            
{                                                                                                                           
    struct Touch* touch = (struct Touch*)args;                                                                                                                                                                                                      
    // Falling edge                                                                                                         
    if(!touch->is_touched){                                                                                                     
        touch->is_touched = true;                                                                                           
    } else { // Rising edge                                                                                                     
        touch->is_touched = false;                                                                                          
    }                                                                                                                                                                                                                                               
    //ESP_LOGI(TAG_TOUCH, "Interrupt trigerred");                                                                         
}                                                                                                                                                                                                                                               

void init_touchscreen(struct Touch* touch){                                                                                 
    // Initialize non-spi gpio pins                                                                                         
    gpio_config_t io_conf = {                                                                                                   
        .pin_bit_mask = (1ULL << TOUCH_IRQ),                                                                                    
        .mode = GPIO_MODE_INPUT,                                                                                                
        .intr_type = GPIO_INTR_ANYEDGE, // both edges. So it works for click and release. Change it to "GPIO LOW" to use the permanent click (to scroll for example)                                                                                
    };                                                                                                                      
    ESP_ERROR_CHECK(gpio_config(&io_conf));                                                                                                                                                                                                         
    ESP_ERROR_CHECK(gpio_install_isr_service(0));                                                                                                                                                                                                   
    ESP_ERROR_CHECK(gpio_isr_handler_add(TOUCH_IRQ, touch_spi_touched_interrupt, touch));                                                                                                                                                           
    // Initialize spi gpio pins                                                                                             
    spi_device_handle_t spi;                                                                                                
    spi_device_interface_config_t devcfg = {                                                                                    
        .clock_speed_hz = BAUDRATE_TOUCH,                                                                                       
        .mode = 0,                                                                                                              
        .spics_io_num = TOUCH_CS,                                                                                               
        .queue_size = 1,                                                                                                        
        .pre_cb = NULL,                                                                                                     
    };                                                                                                                      
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));                                                                                                                                                                                  
    memset(touch, 0, sizeof(struct Touch));                                                                                                                                                                                                         
    touch->spi = spi;                                                                                                   
}  

// MUST be called                                                                                                       
void close_touchscreen(struct Touch* touch){                                                                                
    ESP_ERROR_CHECK(gpio_isr_handler_remove(TOUCH_IRQ));                                                                    
    gpio_uninstall_isr_service();                                                                                           
    ESP_ERROR_CHECK(spi_bus_remove_device(touch->spi));                                                                 
}    

bool is_touched(struct Touch* touch){                                                                                       
// // Read the IRQ pin from the LCD touchscreen                                                                         
// // If LOW, the screen is touched                                                                                     
// if(gpioRead(IRQ_TOUCH) == LOW){                                                                                      
//     if(!touch->is_touched){       // If it was not touch, before, it's press                                         
//         touch->is_touched = true; // Used to touch one time                                                          
//         touch->type = ONE_CLICK; // A single click                                                                   
//         return true;        // Return true for the caller                                                            
//     }                                                                                                                
// // If it's high and m_touched is true, it was a release                                                              
// }else if(gpioRead(IRQ_TOUCH) == HIGH && touch->is_touched){                                                          
//     touch->is_touched = false;    // So touch is false                                                               
//     touch->type = RELEASE;       // And the type is RELEASE                                                                                                                                                                                  
//     return true;                                                                                                     
// }                                                                                                                    
// return false;                                                                                                        
return false;                                                                                                       
}                                                                                                                                                                                                                                                                                                                                                                       

uint16_t read_data(struct Touch* touch, enum TouchCommand command){
    char txBuffer[3] = { command, 0, 0 };                                                                                   
    char rxBuffer[3] = { 0, 0, 0 };          

    spi_transaction_t t;                                                                                                    
    memset(&t, 0, sizeof(t));     

    t.length = 3 * 8;                                                                                                       
    t.tx_buffer = txBuffer;                                                                                                 
    t.rxlength = 3 * 8;      
    t.rx_buffer = rxBuffer;

    spi_device_acquire_bus(touch->spi, portMAX_DELAY);                                                                                                                                                                                              
    spi_device_polling_transmit(touch->spi, &t);                                                                                                                                                                                                    
    spi_device_release_bus(touch->spi);          

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

//This function is called (in irq context!) just before a transmission starts. It will                                  
//set the D/C line to the value indicated in the user field.                                                            
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)                                                                
{                                                                                                                           
    int dc = (int)t->user;                                                                                                  
    gpio_set_level(LCD_DCRS, dc);                                                                                       
}      
                                                                                                                                                                                                                                         
void init_lcd_screen(struct LCDScreen* lcd)                                                                             
{                                                                                                                           
    // Initialize non-spi gpio pins                                                                                         
    ESP_LOGI(TAG, "Initialize GPIO");                                                                                       
    gpio_config_t io_conf = {                                                                                                   
        .pin_bit_mask = ((1ULL << LCD_DCRS) | (1ULL << LCD_RESET)),                                                             
        .mode = GPIO_MODE_OUTPUT,                                                                                               
        .pull_up_en = GPIO_PULLUP_DISABLE,                                                                                      
        .pull_down_en = GPIO_PULLDOWN_ENABLE,                                                                                   
        .intr_type = GPIO_INTR_DISABLE,                                                                                     
    };                                                                                                                      
    ESP_ERROR_CHECK(gpio_config(&io_conf));                                                                                                                                                                                                         
    // Initialize spi gpio pins                                                                                             
    spi_device_handle_t spi;                                                                                                
    spi_device_interface_config_t devcfg = {                                                                                    
        .clock_speed_hz = BAUDRATE_LCD,                                                                                         
        .mode = 0,                                                                                                              
        .spics_io_num = LCD_CS,                                                                                                 
        .queue_size = 1,                                                                                                        
        .pre_cb = lcd_spi_pre_transfer_callback,                                                                            
    };          

    ESP_LOGI(TAG, "Initialize devcfg and spi");                                                                             
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));                                                                                                                                                                                  
    memset(lcd, 0, sizeof(struct LCDScreen));                                                                                                                                                                                                       
    lcd->spi = spi;    

    // Initialisation Sequence                                                                                              
    //Reset the LCD Screen                                                                                                  
    ESP_LOGI(TAG, "Write commands");                                                                                        
    gpio_set_level(LCD_RESET, HIGH);                                                                                        
    vTaskDelay(pdMS_TO_TICKS(120)); // wait 120 ms                                                                          
    gpio_set_level(LCD_RESET, LOW);                                                                                         
    vTaskDelay(pdMS_TO_TICKS(120)); // wait 120 ms                                                                          
    gpio_set_level(LCD_RESET, HIGH);                                                                                        
    vTaskDelay(pdMS_TO_TICKS(120)); // wait 120 ms         

    // Software reset                                                                                                       
    write_command(lcd, SOFTWARE_RST);                                                                                       
    vTaskDelay(pdMS_TO_TICKS(120));     

    // 0x3A Interface Pixel Format (bit depth color space)                                                                  
    write_command(lcd, INTERFACE_PIXEL_FORMAT);                                                                             
    write_single_data(lcd, 0X66);                   

    write_command(lcd, TEARING_EFFECT_LINE_ON);                                                                             
    write_single_data(lcd, 0x1);                     

    write_command(lcd, MEMORY_ACCESS_CONTROL);                                                                              
    write_single_data(lcd, 0x20);                  

    write_command(lcd, FRAME_RATE_CONTROL);                                                                                 
    write_single_data(lcd, 0xA1);                                                                                           
    write_single_data(lcd, 0x1F);            

    write_command(lcd, ADJUST_CONTROL_3);                                                                                   
    write_single_data(lcd, 0xA9);                                                                                           
    write_single_data(lcd, 0x51);                                                                                           
    write_single_data(lcd, 0x2C);                                                                                           
    write_single_data(lcd, 0x02);       

    // 0x11 Exit Sleep Mode. (Sleep OUT)                                                                                    
    write_command(lcd, SLEEP_OUT);                                                                                          
    vTaskDelay(pdMS_TO_TICKS(120));   

    // 0x29 Display ON.                                                                                                     
    write_command(lcd, DISPLAY_ON);                                                                                                                                                                                                                 
    ESP_LOGI(TAG, "Allocate memory");    

    lcd->draw_buffer = (char*)heap_caps_malloc(BUFFER_MAX_SIZE, MALLOC_CAP_DMA);                                         
}       

// MUST be called at the end of use                                                                                     
void close_lcd_screen(struct LCDScreen* lcd) {                                                                                                                           
    ESP_ERROR_CHECK(spi_bus_remove_device(lcd->spi));                                                                       
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

void write_command(struct LCDScreen* lcd, enum LCDCommand command) {                                                                                                                           
    //spi_device_acquire_bus(lcd->spi, portMAX_DELAY);     

    spi_transaction_t t;                                                                                                    
    memset(&t, 0, sizeof(t));       

    t.length = 8;                                                                                                           
    t.tx_buffer = &command;                                                                                                 
    t.user = (void*)LOW; // DCRS should be to LOW       

    if(spi_device_polling_transmit(lcd->spi, &t) != ESP_OK){                                                                    
        ESP_LOGE(TAG, "Failed to send the command %d", command);                                                            
    }                                

    //spi_device_release_bus(lcd->spi);                                                                                   
}        

void write_data(struct LCDScreen* lcd, uint8_t* data, int size){         
    size_t bytes_sent = 0;

    if(size == 0){                                                                                                              
        return;                                                                                                             
    }        

    //spi_device_acquire_bus(lcd->spi, portMAX_DELAY);  

    while(bytes_sent < size) {
        size_t chunk_size = size - bytes_sent;
        if(chunk_size > SPI_MAX_CHUNK_SIZE) {
            chunk_size = SPI_MAX_CHUNK_SIZE;
        }
    

        spi_transaction_t t;                                                                                                                                                                                                                            


        memset(&t, 0, sizeof(t));           
                                                                                        
        t.length = chunk_size * 8; // in bits, not bytes                                                                              
        t.tx_buffer = data + bytes_sent;                                                                                                     
        t.user = (void*)HIGH; // DCRS should be HIGH for data       

        if(spi_device_polling_transmit(lcd->spi, &t) != ESP_OK){                                                                    
            ESP_LOGE(TAG, "Failed to send the data of size %d", size);                                                          
        }    
        bytes_sent += chunk_size;                                     
    }

    //spi_device_release_bus(lcd->spi);                                                                                   
}             

void write_single_data(struct LCDScreen* lcd, uint8_t data){                                                                
    write_data(lcd, &data, 1);                                                                                          
}     

void set_address(struct LCDScreen* lcd, int x1, int y1, int x2, int y2)                                                 
{                                                                                                                           
    write_command(lcd, COLUMN_ADDRESS_SET);                                                                                     
    write_single_data(lcd, x1 >> 8);                                                                                        
    write_single_data(lcd, x1);                                                                                             
    write_single_data(lcd, x2 >> 8);                                                                                        
    write_single_data(lcd, x2);   

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
    write_single_data(lcd, y1 >> 8);                                                                                        
    write_single_data(lcd, y1);                                                                                             
    write_single_data(lcd, y2 >> 8);                                                                                        
    write_single_data(lcd, y2);   

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

void clear_screen(struct LCDScreen* lcd) {                                                                                                                           
    set_address(lcd, 0, 0, WIDTH, HEIGHT); // Set the address of the entire screen                                          
    write_command(lcd, MEMORY_WRITE);                                                                                                                                                                                                               
    int white = 0xFFFFFF;                                                                                                   
    // And write WHITE color to all pixels                                                                                  
    for(int i = 0; i < WIDTH; i++) {                                                                                                                           
        for(int m = 0; m < HEIGHT; m++) {                                                                                                                           
            write_single_data(lcd, (white >> 16) & 0xFC);                                                                           
            write_single_data(lcd, (white >> 8) & 0xFC);                                                                            
            write_single_data(lcd, white & 0xFC);                                                                               
        }                                                                                                                   
    }                                                                                                                   
}        

void set_pixel(struct LCDScreen* lcd, int x, int y, unsigned int color) 
{                                                   
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

void draw_frame(struct LCDScreen* lcd, unsigned int size){                                                                  
    set_address(lcd, 0, 0, WIDTH, HEIGHT);                                                                                  
    write_command(lcd, MEMORY_WRITE);                                                                                                                                                                                                               
    write_data(lcd, (uint8_t*)lcd->draw_buffer, size);                                                                  
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
