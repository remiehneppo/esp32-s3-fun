#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "sensor.h"
#include "driver/spi_common.h"
#include "esp_lcd_types.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_st7789.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// init cam
#define CAM_PIN_PWDN 38
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

// init lcd
// Using SPI2 in the example, as it also supports octal modes on some targets
#define LCD_HOST       SPI2_HOST
// To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many.
// More means more memory use, but less overhead for setting up / finishing transfers. Make sure 240
// is dividable by this.
#define PARALLEL_LINES 80
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  0
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0          20  /*!< for 1-line SPI, this also refereed as MOSI */
#define EXAMPLE_PIN_NUM_PCLK           19
#define EXAMPLE_PIN_NUM_CS             2
#define EXAMPLE_PIN_NUM_DC             1
#define EXAMPLE_PIN_NUM_RST            21
#define EXAMPLE_PIN_NUM_BK_LIGHT       48

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              320
#define EXAMPLE_LCD_V_RES              240
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

static const char *TAG = "camera_lcd";

// Double buffering for camera frames
static uint8_t *pixels[2] = {NULL, NULL};
static volatile int write_buffer = 0;  // Buffer being written by camera
static volatile int read_buffer = 0;   // Buffer being read by LCD
static SemaphoreHandle_t frame_ready_sem = NULL;
static SemaphoreHandle_t buffer_mutex = NULL;


static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 0, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST //CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

esp_err_t camera_init(){
    //power up the camera if PWDN pin is defined
    if(CAM_PIN_PWDN != -1){
        gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_DEF_OUTPUT);
        gpio_set_level(CAM_PIN_PWDN, 0);
    }

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

esp_err_t camera_capture(){
    // Acquire a frame from camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }
    
    // Calculate frame size (QVGA = 320x240 pixels, RGB565 = 2 bytes per pixel)
    size_t frame_size = fb->len;
    // Lock mutex to safely swap buffers
    if (xSemaphoreTake(buffer_mutex, portMAX_DELAY) == pdTRUE) {
        // Copy data to our buffer instead of using pointer
        // This is CRITICAL because fb->buf will be freed after esp_camera_fb_return()
        if (pixels[write_buffer] != NULL) {
            memcpy(pixels[write_buffer], fb->buf, frame_size);
        }
        
        // Swap write buffer
        write_buffer = !write_buffer;
        
        xSemaphoreGive(buffer_mutex);
    }
    
    // Return the frame buffer back to the driver for reuse
    // After this, fb->buf is INVALID and should not be accessed
    esp_camera_fb_return(fb);
    
    // Signal that new frame is ready for display
    xSemaphoreGive(frame_ready_sem);
    
    return ESP_OK;
}


void task_capture(void *params) {
    // Add delay to let system stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while(1) {
        esp_err_t ret = camera_capture();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Capture failed, retrying...");
        }
        // Delay ~33ms for ~30 FPS (adjust as needed)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_display_lcd(void *params) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) params;
    ESP_LOGI(TAG, "LCD display task started");
    
    while (1) {
        // Wait for new frame to be ready
        if (xSemaphoreTake(frame_ready_sem, portMAX_DELAY) == pdTRUE) {
            
            // Lock mutex to safely read buffer index
            if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                // Switch to the buffer that was just written
                read_buffer = !write_buffer;
                xSemaphoreGive(buffer_mutex);
            }
            
            // Draw the entire frame to LCD
            // Check if buffer is valid before drawing
            if (pixels[read_buffer] != NULL) {
                // Draw frame line by line for better performance
                for (int y = 0; y < EXAMPLE_LCD_V_RES; y += PARALLEL_LINES) {
                    uint8_t *line_buf = pixels[read_buffer] + (y * EXAMPLE_LCD_H_RES * 2);
                    esp_lcd_panel_draw_bitmap(panel_handle, 0, y, EXAMPLE_LCD_H_RES, y + PARALLEL_LINES, line_buf);
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}



void app_main(void) {
    ESP_LOGI(TAG, "Starting camera-LCD application");
    
    // Create semaphores for synchronization
    frame_ready_sem = xSemaphoreCreateBinary();
    buffer_mutex = xSemaphoreCreateMutex();
    
    if (frame_ready_sem == NULL || buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }
    
    // Allocate memory for double buffering (QVGA RGB565 = 320*240*2 bytes)
    size_t frame_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2;
    pixels[0] = (uint8_t*)heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    pixels[1] = (uint8_t*)heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (pixels[0] == NULL || pixels[1] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers");
        return;
    }
    
    // Initialize buffers to black
    memset(pixels[0], 0, frame_size);
    memset(pixels[1], 0, frame_size);
    
    ESP_LOGI(TAG, "Frame buffers allocated: %d bytes each", frame_size);
    
    camera_init();
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    // Initialize the GPIO of backlight
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_PCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_DATA0,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8
    };
    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    // Initialize the LCD configuration
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL));

    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));

    // Swap x and y axis (Different LCD screens may need different options)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    // Turn on backlight (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));

    // Create tasks with reasonable stack sizes
    BaseType_t ret_capture = xTaskCreate(task_capture, "task_capture", 8192, NULL, 2, NULL);
    if (ret_capture != pdPASS) {
        ESP_LOGE(TAG, "Failed to create capture task!");
        return;
    }
    ESP_LOGI(TAG, "Capture task created successfully");

    BaseType_t ret_display = xTaskCreate(task_display_lcd, "task_display", 4096, panel_handle, 1, NULL);
    if (ret_display != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task!");
        return;
    }
    ESP_LOGI(TAG, "Display task created successfully");

}