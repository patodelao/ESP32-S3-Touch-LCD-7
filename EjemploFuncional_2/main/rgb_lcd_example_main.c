/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"

#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4

static const char *TAG = "example";




// Declaraciones de funciones de inicialización del sistema
void init_nvs();
esp_lcd_panel_handle_t init_lcd_panel();
void init_i2c();
esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle);
lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle);
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp);
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv);






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       -1
#define EXAMPLE_PIN_NUM_HSYNC          46
#define EXAMPLE_PIN_NUM_VSYNC          3
#define EXAMPLE_PIN_NUM_DE             5
#define EXAMPLE_PIN_NUM_PCLK           7
#define EXAMPLE_PIN_NUM_DATA0          14 // B3
#define EXAMPLE_PIN_NUM_DATA1          38 // B4
#define EXAMPLE_PIN_NUM_DATA2          18 // B5
#define EXAMPLE_PIN_NUM_DATA3          17 // B6
#define EXAMPLE_PIN_NUM_DATA4          10 // B7
#define EXAMPLE_PIN_NUM_DATA5          39 // G2
#define EXAMPLE_PIN_NUM_DATA6          0 // G3
#define EXAMPLE_PIN_NUM_DATA7          45 // G4
#define EXAMPLE_PIN_NUM_DATA8          48 // G5
#define EXAMPLE_PIN_NUM_DATA9          47 // G6
#define EXAMPLE_PIN_NUM_DATA10         21 // G7
#define EXAMPLE_PIN_NUM_DATA11         1  // R3
#define EXAMPLE_PIN_NUM_DATA12         2  // R4
#define EXAMPLE_PIN_NUM_DATA13         42 // R5
#define EXAMPLE_PIN_NUM_DATA14         41 // R6
#define EXAMPLE_PIN_NUM_DATA15         40 // R7
#define EXAMPLE_PIN_NUM_DISP_EN        -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              800
#define EXAMPLE_LCD_V_RES              480

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB             2
#else
#define EXAMPLE_LCD_NUM_FB             1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static SemaphoreHandle_t lvgl_mux = NULL;

#define DEFAULT_SCAN_LIST_SIZE 5 // numero de redes a escanear

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static lv_obj_t *ssid_dropdown;




#define MAX_NETWORKS 5
static wifi_ap_record_t top_networks[MAX_NETWORKS];

static void wifi_scan(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t *ap_info = malloc(DEFAULT_SCAN_LIST_SIZE * sizeof(wifi_ap_record_t));
    if (!ap_info) {
        ESP_LOGE(TAG, "No se pudo asignar memoria para ap_info");
        return;
    }
    memset(ap_info, 0, DEFAULT_SCAN_LIST_SIZE * sizeof(wifi_ap_record_t));

    uint16_t ap_count = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));

    ESP_LOGI(TAG, "Escaneando redes Wi-Fi...");
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));

    ESP_LOGI(TAG, "Total de redes detectadas: %u", ap_count);

    // Guardar las 5 redes con mayor intensidad en top_networks
    memset(top_networks, 0, sizeof(top_networks));
    for (int i = 0; i < number; i++) {
        ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_info[i].ssid, ap_info[i].rssi);

        for (int j = 0; j < 5; j++) {
            if (strlen((char *)top_networks[j].ssid) == 0 || ap_info[i].rssi > top_networks[j].rssi) {
                for (int k = 4; k > j; k--) {
                    top_networks[k] = top_networks[k - 1];
                }
                top_networks[j] = ap_info[i];
                break;
            }
        }
    }

    free(ap_info);
}

static void scan_button_event_handler(lv_event_t *event) {
    ESP_LOGI(TAG, "Actualizando opciones del dropdown...");

    // Guardar el índice seleccionado actual
    int selected_index = lv_dropdown_get_selected(ssid_dropdown);

    // Actualizar las opciones del dropdown
    lv_dropdown_clear_options(ssid_dropdown);
    for (int i = 0; i < 5; i++) {
        if (strlen((char *)top_networks[i].ssid) > 0) {
            lv_dropdown_add_option(ssid_dropdown, (const char *)top_networks[i].ssid, LV_DROPDOWN_POS_LAST);
        }
    }

    // Restaurar el índice seleccionado, o establecer al primero si ya no es válido
    if (selected_index >= lv_dropdown_get_option_cnt(ssid_dropdown)) {
        selected_index = 0; // Si el índice es inválido, seleccionar la primera opción
    }
    lv_dropdown_set_selected(ssid_dropdown, selected_index);
}

static void connect_button_event_handler(lv_event_t *event) {
    char selected_ssid[33]; // Buffer para almacenar el SSID seleccionado
    lv_dropdown_get_selected_str(ssid_dropdown, selected_ssid, sizeof(selected_ssid));

    const char *password = lv_textarea_get_text((lv_obj_t *)lv_event_get_user_data(event));

    ESP_LOGI(TAG, "Intentando conectar a SSID: %s con contraseña: %s", selected_ssid, password);
    // Implementa la lógica para conectarte al Wi-Fi aquí
}

void create_wifi_settings_widget(lv_disp_t *disp) {
    // Crear pantalla y fondo
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Contenedor principal
    lv_obj_t *container = lv_obj_create(scr);
    lv_obj_set_size(container, 300, 400);
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    // Dropdown para SSIDs
    ssid_dropdown = lv_dropdown_create(container);
    lv_obj_set_width(ssid_dropdown, 200);
    lv_dropdown_set_options(ssid_dropdown, "Seleccione una red...");

    // Campo de texto para contraseña
    lv_obj_t *password_textarea = lv_textarea_create(container);
    lv_textarea_set_one_line(password_textarea, true);
    lv_textarea_set_password_mode(password_textarea, true);
    lv_textarea_set_placeholder_text(password_textarea, "Ingrese contraseña");
    lv_obj_set_width(password_textarea, 200);

    // Botón de escaneo
    lv_obj_t *scan_btn = lv_btn_create(container);
    lv_obj_set_size(scan_btn, 120, 50);
    lv_obj_t *scan_label = lv_label_create(scan_btn);
    lv_label_set_text(scan_label, "Escanear");
    lv_obj_center(scan_label);
    lv_obj_add_event_cb(scan_btn, scan_button_event_handler, LV_EVENT_CLICKED, NULL);

    // Botón para conectar
    lv_obj_t *connect_btn = lv_btn_create(container);
    lv_obj_set_size(connect_btn, 120, 50);
    lv_obj_t *connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "Conectar");
    lv_obj_center(connect_label);
    lv_obj_add_event_cb(connect_btn, connect_button_event_handler, LV_EVENT_CLICKED, password_textarea);
}

void initialize_wifi_event_loop() {
    static bool initialized = false;
    if (!initialized) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        initialized = true;
    }
}












static lv_obj_t *prev_screen;
static lv_obj_t *keyboard;

static void keyboard_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *keyboard = lv_event_get_target(event);

    if (code == LV_EVENT_CANCEL || code == LV_EVENT_READY) {
        lv_obj_t *textarea = lv_keyboard_get_textarea(keyboard);

        if (textarea) {
            // Procesar el texto si es necesario
            const char *text = lv_textarea_get_text(textarea);
            printf("Texto guardado: %s\n", text);
        }

        // Ocultar teclado sin eliminarlo
        lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void textarea_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *textarea = lv_event_get_target(event);

    if (code == LV_EVENT_FOCUSED) {
        if (!keyboard) {
            // Crear teclado si no existe
            keyboard = lv_keyboard_create(lv_scr_act());
            lv_obj_add_event_cb(keyboard, keyboard_event_handler, LV_EVENT_ALL, NULL);
        }

        // Asociar el teclado al textarea y mostrarlo
        lv_keyboard_set_textarea(keyboard, textarea);
        lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void back_button_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_CLICKED) {
        lv_disp_load_scr(prev_screen);
    }
}






























































extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}



static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void gpio_init(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO6 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    // io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

// extern lv_obj_t *scr;
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
void app_main(void)
{
    // Initialize NVS
    init_nvs();
    
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    // Initialize the LCD panel
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel();


    // Register event callbacks
    register_lcd_event_callbacks(panel_handle, &disp_drv);

    // Initialize I2C
    init_i2c();

    // Initialize GPIO
    gpio_init();    


    // Initialize touch controller
    esp_lcd_touch_handle_t tp = init_touch(panel_handle);

  
    // Initialize LVGL
    lv_disp_t *disp = init_lvgl(panel_handle);

    start_lvgl_task(disp, tp);

    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        // example_lvgl_demo_ui(disp);
        //lv_demo_widgets();
        //create_touch_widget(disp);
        create_wifi_settings_widget(disp);
        // lv_demo_benchmark();
        // lv_demo_music();
        // lv_demo_stress();
        // Release the mutex
        example_lvgl_unlock();
    }
    wifi_scan();

    ESP_LOGI(TAG, "Start LVGL task");
    //xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
}




























































////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_lcd_panel_handle_t init_lcd_panel()
{
    ESP_LOGI(TAG, "Installing RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        // Configuración del panel LCD
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = 40,  // Margen posterior horizontal
        .hsync_front_porch = 20, // Margen anterior horizontal
        .hsync_pulse_width = 4,  // Ancho del pulso HSYNC
        .vsync_back_porch = 8,   // Margen posterior vertical
        .vsync_front_porch = 8,  // Margen anterior vertical
        .vsync_pulse_width = 4,  // Ancho del pulso VSYNC
        .flags.pclk_active_neg = true,
    },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "LCD panel initialized successfully");
    return panel_handle;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_i2c()
{
    ESP_LOGI(TAG, "Initializing I2C");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    //Reset the touch screen. It is recommended that you reset the touch screen before using it.
    write_buf = 0x2C;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(100 * 1000);

    gpio_set_level(GPIO_INPUT_IO_4,0);
    esp_rom_delay_us(100 * 1000);

    write_buf = 0x2E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(200 * 1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Initializing touch controller GT911");
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));
    return tp;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    void *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);

    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, EXAMPLE_LCD_H_RES * 100);
    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    return disp;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp)
{
    
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreatePinnedToCore(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, disp, EXAMPLE_LVGL_TASK_PRIORITY, NULL, 0);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv)
{
    ESP_LOGI(TAG, "Register event callbacks");

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };

    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, disp_drv));
}



