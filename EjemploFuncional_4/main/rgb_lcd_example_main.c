/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"



#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4


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


// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif




static const char *TAG = "wifi_lcd";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static lv_obj_t *status_label;
static lv_obj_t *ssid_field;
static lv_obj_t *password_field;
static lv_obj_t *keyboard;

// Prototipo de funciones
static void textarea_event_handler(lv_event_t *event);
static void keyboard_event_handler(lv_event_t *event);

static char wifi_ssid[32] = "";
static char wifi_password[64] = "";

/* Manejador de eventos Wi-Fi */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying connection to Wi-Fi...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Inicialización de Wi-Fi Station */
void wifi_init_sta(const char *ssid, const char *password) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Wi-Fi initialization completed.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));//portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi SSID:%s", ssid);
        lv_label_set_text(status_label, "Conectado a Wi-Fi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid);
        lv_label_set_text(status_label, "Error al conectar");
    } else {
        ESP_LOGE(TAG, "Unexpected event");
    }

    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    vEventGroupDelete(s_wifi_event_group);
}

/* Callback para conectar desde la interfaz */
static void wifi_task(void *param) {
    char *ssid = ((char **)param)[0];
    char *password = ((char **)param)[1];
    
    ESP_LOGI(TAG, "Conectando a Wi-Fi...");
    wifi_init_sta(ssid, password);
    vTaskDelete(NULL); // Finaliza la tarea después de la conexión
}

static void connect_event_handler(lv_event_t *event) {
    // Usa las referencias globales en lugar de buscar los objetos
    const char *ssid = lv_textarea_get_text(ssid_field);
    const char *password = lv_textarea_get_text(password_field);

    if (strlen(ssid) == 0 || strlen(password) == 0) {
        lv_label_set_text(status_label, "Por favor, complete los campos.");
        return;
    }

    lv_label_set_text(status_label, "Conectando...");
    wifi_init_sta(ssid, password);
}


/* Crear interfaz gráfica */
void create_wifi_interface(lv_disp_t *disp) {
    lv_obj_clean(lv_scr_act()); // Limpia la pantalla activa
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    lv_obj_t *ssid_label = lv_label_create(scr);
    lv_label_set_text(ssid_label, "SSID:");
    lv_obj_align(ssid_label, LV_ALIGN_TOP_LEFT, 10, 10);

    ssid_field = lv_textarea_create(scr); // Guarda referencia global
    lv_obj_set_width(ssid_field, 200);
    lv_textarea_set_placeholder_text(ssid_field, "Enter SSID");
    lv_obj_align_to(ssid_field, ssid_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_add_event_cb(ssid_field, textarea_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t *password_label = lv_label_create(scr);
    lv_label_set_text(password_label, "Password:");
    lv_obj_align_to(password_label, ssid_field, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

    password_field = lv_textarea_create(scr); // Guarda referencia global
    lv_obj_set_width(password_field, 200);
    lv_textarea_set_placeholder_text(password_field, "Enter password");
    lv_textarea_set_password_mode(password_field, true);
    lv_obj_align_to(password_field, password_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_add_event_cb(password_field, textarea_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t *connect_button = lv_btn_create(scr);
    lv_obj_align_to(connect_button, password_field, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_add_event_cb(connect_button, connect_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(connect_button);
    lv_label_set_text(btn_label, "Connect");
    lv_obj_center(btn_label);

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Estado: Listo");
    lv_obj_align_to(status_label, connect_button, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
}

static void button_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *button = lv_event_get_target(event);
    lv_obj_t *label = lv_obj_get_child(button, 0);

    if (code == LV_EVENT_CLICKED) {
        const char *text = lv_label_get_text(label);
        if (strcmp(text, "Presionado") == 0) {
            lv_label_set_text(label, "No presionado");
        } else {
            lv_label_set_text(label, "Presionado");
        }
    }
}

void create_touch_widget(lv_disp_t *disp) { 
    // Crear pantalla y fondo gris
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x808080), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER,0);

    // Crear contenedor principal
    lv_obj_t *container = lv_obj_create(scr);
    lv_obj_set_size(container, 400, 240);
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(container, LV_OPA_50,0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Crear primer panel
    lv_obj_t *panel1 = lv_obj_create(container);
    lv_obj_set_size(panel1, 180, 200);
    lv_obj_set_style_bg_color(panel1, lv_color_hex(0xC0C0C0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel1, LV_OPA_COVER,0);

    lv_obj_t *btn1 = lv_btn_create(panel1);
    lv_obj_set_size(btn1, 120, 50);
    lv_obj_center(btn1);
    lv_obj_add_event_cb(btn1, button_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label1 = lv_label_create(btn1);
    lv_label_set_text(label1, "No presionado");
    lv_obj_center(label1);

    // Crear segundo panel
    lv_obj_t *panel2 = lv_obj_create(container);
    lv_obj_set_size(panel2, 180, 200);
    lv_obj_set_style_bg_color(panel2, lv_color_hex(0xC0C0C0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel2, LV_OPA_COVER,0);

    lv_obj_t *btn2 = lv_btn_create(panel2);
    lv_obj_set_size(btn2, 120, 50);
    lv_obj_center(btn2);
    lv_obj_add_event_cb(btn2, button_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "No presionado");
    lv_obj_center(label2);
}



static void wifi_connect_event_handler(lv_event_t *event) { /// PRUEBA CON INTERFAZ DE WIFI, SE VE BIEN. NO HACE NADA (TODAVIA)
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_CLICKED) {
        lv_obj_t *ssid_field = (lv_obj_t *)lv_event_get_user_data(event);
        const char *ssid = lv_textarea_get_text(ssid_field);

        lv_obj_t *password_field = lv_obj_get_child(ssid_field, NULL);
        const char *password = lv_textarea_get_text(password_field);

        // Aquí puedes integrar la lógica para conectarte al Wi-Fi usando ssid y password
        printf("Conectando a SSID: %s con contraseña: %s\n", ssid, password);
    }
}















static lv_obj_t *prev_screen;
static lv_obj_t *keyboard;
static lv_obj_t *last_textarea;

static void keyboard_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *keyboard = lv_event_get_target(event);

    if (code == LV_EVENT_CANCEL || code == LV_EVENT_READY) {
        lv_obj_t *textarea = lv_keyboard_get_textarea(keyboard);

        if (textarea) {
            // Obtener y procesar el texto del campo si es necesario
            const char *text = lv_textarea_get_text(textarea);
            printf("Texto guardado: %s\n", text);
        }

        // Ocultar el teclado en lugar de eliminarlo
        lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
        last_textarea = textarea; // Mantener referencia al último textarea
    }
}
static void textarea_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *textarea = lv_event_get_target(event);

    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
        if (!keyboard) {
            keyboard = lv_keyboard_create(lv_scr_act());
            lv_obj_add_event_cb(keyboard, keyboard_event_handler, LV_EVENT_ALL, NULL);
        }
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

static void next_screen_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_CLICKED) {
        // Crear nueva pantalla
        lv_obj_t *next_scr = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(next_scr, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(next_scr, LV_OPA_COVER, 0);

        // Botón de atrás
        lv_obj_t *back_btn = lv_btn_create(next_scr);
        lv_obj_set_size(back_btn, 100, 50);
        lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
        lv_obj_t *back_label = lv_label_create(back_btn);
        lv_label_set_text(back_label, "Atrás");
        lv_obj_center(back_label);

        lv_obj_add_event_cb(back_btn, back_button_event_handler, LV_EVENT_CLICKED, NULL);

        // Cargar nueva pantalla
        prev_screen = lv_scr_act();
        lv_disp_load_scr(next_scr);
    }
}

void create_wifi_settings_widget(lv_disp_t *disp) {
    // Crear pantalla y fondo
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Contenedor principal
    lv_obj_t *container = lv_obj_create(scr);
    lv_obj_set_size(container, 300, 300);
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    // Etiqueta SSID
    lv_obj_t *ssid_label = lv_label_create(container);
    lv_label_set_text(ssid_label, "SSID:");

    // Campo de texto SSID
    lv_obj_t *ssid_textarea = lv_textarea_create(container);
    lv_textarea_set_one_line(ssid_textarea, true);
    lv_textarea_set_placeholder_text(ssid_textarea, "Ingrese SSID");
    lv_obj_set_width(ssid_textarea, 200);
    lv_obj_add_event_cb(ssid_textarea, textarea_event_handler, LV_EVENT_ALL, NULL);

    // Etiqueta Contraseña
    lv_obj_t *password_label = lv_label_create(container);
    lv_label_set_text(password_label, "Contraseña:");

    // Campo de texto Contraseña
    lv_obj_t *password_textarea = lv_textarea_create(container);
    lv_textarea_set_one_line(password_textarea, true);
    lv_textarea_set_password_mode(password_textarea, true);
    lv_textarea_set_placeholder_text(password_textarea, "Ingrese contraseña");
    lv_obj_set_width(password_textarea, 200);
    lv_obj_add_event_cb(password_textarea, textarea_event_handler, LV_EVENT_ALL, NULL);

    // Botón de conectar
    lv_obj_t *connect_btn = lv_btn_create(container);
    lv_obj_set_size(connect_btn, 120, 50);
    lv_obj_t *connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "Conectar");
    lv_obj_center(connect_label);

    // Asignar el evento
    lv_obj_add_event_cb(connect_btn, next_screen_event_handler, LV_EVENT_CLICKED, NULL);
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
    uint32_t task_delay_ms = 2000;//EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
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


/**
 * @brief i2c master initialization
 */
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
{   vTaskDelay(pdMS_TO_TICKS(2000));   
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
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
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .hsync_pulse_width = 4,
            .vsync_back_porch = 16,
            .vsync_front_porch = 16,
            .vsync_pulse_width = 4,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    gpio_init();

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
    
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

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
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scatter Chart");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        // example_lvgl_demo_ui(disp);
        //lv_demo_widgets();
        //create_touch_widget(disp);
    create_wifi_interface(disp);
        // lv_demo_benchmark();
        // lv_demo_music();
        // lv_demo_stress();
        // Release the mutex
        example_lvgl_unlock();
    }
}