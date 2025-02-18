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
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"

#include "lvgl.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_timer.h"

// -------------------------
// DEFINES Y CONFIGURACIONES
// -------------------------
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

// Configuración del LCD (ajusta los pines según tu panel)
#define LCD_PIXEL_CLOCK_HZ          (18 * 1000 * 1000)
#define PIN_NUM_BK_LIGHT            -1
#define PIN_NUM_HSYNC               46
#define PIN_NUM_VSYNC               3
#define PIN_NUM_DE                  5
#define PIN_NUM_PCLK                7
#define PIN_NUM_DATA0               14
#define PIN_NUM_DATA1               38
#define PIN_NUM_DATA2               18
#define PIN_NUM_DATA3               17
#define PIN_NUM_DATA4               10
#define PIN_NUM_DATA5               39
#define PIN_NUM_DATA6               0
#define PIN_NUM_DATA7               45
#define PIN_NUM_DATA8               48
#define PIN_NUM_DATA9               47
#define PIN_NUM_DATA10              21
#define PIN_NUM_DATA11              1
#define PIN_NUM_DATA12              2
#define PIN_NUM_DATA13              42
#define PIN_NUM_DATA14              41
#define PIN_NUM_DATA15              40

#define LCD_H_RES                   800
#define LCD_V_RES                   480

#if CONFIG_DOUBLE_FB
#define LCD_NUM_FB                  2
#else
#define LCD_NUM_FB                  1
#endif

#define LVGL_TICK_PERIOD_MS         2
#define LVGL_TASK_MAX_DELAY_MS      500
#define LVGL_TASK_MIN_DELAY_MS      1
#define LVGL_TASK_STACK_SIZE        (8 * 1024)
#define LVGL_TASK_PRIORITY          2

static const char *TAG = "waterMachine";

// -------------------------
// VARIABLES GLOBALES LVGL Y PARA PRODUCTOS
// -------------------------
static SemaphoreHandle_t lvgl_mux = NULL; // Mutex para LVGL

// Variables para la configuración de productos
#define MAX_ITEMS 10
static uint32_t cont_index = 0;
lv_obj_t *cont_arr[MAX_ITEMS] = {0};
static lv_obj_t *menu;
static lv_obj_t *main_page;
static lv_obj_t *float_btn_add;
static lv_obj_t *float_btn_del;
static lv_obj_t *float_btn_del_all;

// -------------------------
// DECLARACIONES DE FUNCIONES USADAS
// -------------------------
void init_nvs(void);
static void init_uart(void);
esp_lcd_panel_handle_t init_lcd_panel(void);
void init_i2c(void);
esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle);
lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle);
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp);
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv);
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);

static void increase_lvgl_tick(void *arg);
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data);
static void lvgl_port_task(void *arg);
static esp_err_t i2c_master_init(void);

// Funciones de la interfaz de configuración de productos
static lv_obj_t *create_textarea(lv_obj_t *parent, const char *placeholder);
static void save_button_event_cb(lv_event_t *e);
static void delete_last_item(lv_event_t *e);
static void delete_all_items(lv_event_t *e);
static void go_to_main_screen(lv_event_t *e);
static void transition_to_main_screen(void *param);
static void create_new_product(lv_event_t *e);
static void product_config_in_tab(lv_obj_t *parent);
void create_general_config_screen(void);
void save_products_to_nvs(void);
void load_products_for_config(void);

// -------------------------
// IMPLEMENTACIONES
// -------------------------

// Inicializa NVS
void init_nvs(void){
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");
}

// Inicialización de UART (actualmente no se usa)
static void init_uart(void)
{
    ESP_LOGI(TAG, "UART initialization (no se utiliza en el flujo actual)");
}

// Inicializa el panel LCD RGB
esp_lcd_panel_handle_t init_lcd_panel(void){
    ESP_LOGI(TAG, "Installing RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = PIN_NUM_BK_LIGHT,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .data_gpio_nums = {
            PIN_NUM_DATA0,
            PIN_NUM_DATA1,
            PIN_NUM_DATA2,
            PIN_NUM_DATA3,
            PIN_NUM_DATA4,
            PIN_NUM_DATA5,
            PIN_NUM_DATA6,
            PIN_NUM_DATA7,
            PIN_NUM_DATA8,
            PIN_NUM_DATA9,
            PIN_NUM_DATA10,
            PIN_NUM_DATA11,
            PIN_NUM_DATA12,
            PIN_NUM_DATA13,
            PIN_NUM_DATA14,
            PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_back_porch = 40,
            .hsync_front_porch = 20,
            .hsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "LCD panel initialized successfully");
    return panel_handle;
}

// Inicializa I2C
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
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void init_i2c(void)
{
    ESP_LOGI(TAG, "Initializing I2C");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    // Aquí podrías inicializar dispositivos I2C adicionales si fuese necesario.
}

// Inicializa el touch (GT911)
esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Initializing touch controller GT911");
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
         .x_max = LCD_V_RES,
         .y_max = LCD_H_RES,
         .rst_gpio_num = -1,
         .int_gpio_num = -1,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));
    ESP_LOGI(TAG, "Touch controller initialized successfully");
    return tp;
}

// Inicializa LVGL
lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
    void *buf1 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LCD_H_RES * 100);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    ESP_LOGI(TAG, "LVGL initialized successfully");
    return disp;
}

// Callback para volcar el buffer de LVGL al panel LCD
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

// Incrementa el tick de LVGL
static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// Bloqueo de LVGL (ya que sus APIs no son thread-safe)
bool lvgl_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

// Callback para el touch de LVGL
static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    esp_lcd_touch_read_data(drv->user_data);
    bool pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
    if (pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// Tarea de LVGL
static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        if (lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

// Inicia la tarea de LVGL (se ejecuta en core 1)
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp)
{
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
         .callback = &increase_lvgl_tick,
         .name = "lvgl_tick"
    };
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, disp, 2, NULL, 1);
}

// Registra callbacks de eventos del LCD (opcionalmente se puede ampliar)
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv)
{
    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
         .on_vsync = NULL, // Se puede implementar si es necesario
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, disp_drv));
}

// -------------------------
// INTERFAZ DE CONFIGURACIÓN DE PRODUCTOS
// -------------------------

// Crea un textarea simple
static lv_obj_t *create_textarea(lv_obj_t *parent, const char *placeholder)
{
    ESP_LOGI(TAG, "Creating textarea with placeholder: %s", placeholder);
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_textarea_set_placeholder_text(ta, placeholder);
    lv_textarea_set_one_line(ta, true);
    return ta;
}

// Guarda el nombre modificado desde la subpágina
static void save_button_event_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Save button pressed");
    lv_obj_t *sub_page = lv_event_get_user_data(e);
    lv_obj_t *name_ta = lv_obj_get_child(sub_page, 0);
    const char *name = lv_textarea_get_text(name_ta);
    if (strlen(name) == 0) {
        ESP_LOGW(TAG, "Name is empty");
        printf("El nombre no puede estar vacío\n");
        return;
    }
    lv_obj_t *cont = lv_obj_get_user_data(sub_page);
    if (cont == NULL) {
        ESP_LOGE(TAG, "No container found for sub_page");
        printf("Error: No se encontró el contenedor vinculado.\n");
        return;
    }
    lv_obj_t *label = lv_obj_get_child(cont, 0);
    lv_label_set_text(label, name);
    ESP_LOGI(TAG, "Product name saved: %s", name);
}

// Elimina el último producto añadido
static void delete_last_item(lv_event_t *e)
{
    ESP_LOGI(TAG, "Deleting last product");
    if (cont_index == 0) {
        ESP_LOGW(TAG, "No products to delete");
        printf("No hay productos para eliminar\n");
        return;
    }
    lv_obj_del(cont_arr[cont_index - 1]);
    cont_arr[cont_index - 1] = NULL;
    cont_index--;
    ESP_LOGI(TAG, "Product deleted. New count: %lu", cont_index);
}

// Elimina todos los productos
static void delete_all_items(lv_event_t *e)
{
    ESP_LOGI(TAG, "Deleting all products");
    while (cont_index > 0) {
        lv_obj_del(cont_arr[cont_index - 1]);
        cont_arr[cont_index - 1] = NULL;
        cont_index--;
    }
    ESP_LOGI(TAG, "All products deleted");
}

// Transición para volver a la pantalla principal (se puede ajustar)
static void transition_to_main_screen(void *param)
{
    ESP_LOGI(TAG, "Transitioning to main screen");
    lv_obj_clean(lv_scr_act());
    create_general_config_screen();
}

// Botón para salir (guarda y vuelve a la pantalla principal)
static void go_to_main_screen(lv_event_t *e)
{
    ESP_LOGI(TAG, "Go to main screen button pressed");
    save_products_to_nvs();
    lv_async_call(transition_to_main_screen, NULL);
}

// Crea un nuevo producto y su correspondiente sub-página
static void create_new_product(lv_event_t *e)
{
    ESP_LOGI(TAG, "Creating new product. Current count: %lu", cont_index);
    if (cont_index >= MAX_ITEMS) {
        ESP_LOGW(TAG, "Maximum products reached");
        printf("No se pueden añadir más productos\n");
        return;
    }
    lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
    ESP_LOGI(TAG, "New sub-page created for product %lu", cont_index + 1);
    lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
    lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
    lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
    lv_obj_t *save_btn = lv_btn_create(new_sub_page);
    lv_obj_t *label = lv_label_create(save_btn);
    lv_label_set_text(label, "Guardar");
    lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
    ESP_LOGI(TAG, "Save button created for new product");
    lv_obj_t *cont = lv_menu_cont_create(main_page);
    lv_obj_t *cont_label = lv_label_create(cont);
    static char default_name[20];
    snprintf(default_name, sizeof(default_name), "Producto %lu", cont_index + 1);
    lv_label_set_text(cont_label, default_name);
    lv_menu_set_load_page_event(menu, cont, new_sub_page);
    lv_obj_set_user_data(cont, new_sub_page);
    lv_obj_set_user_data(new_sub_page, cont);
    lv_obj_scroll_to_view_recursive(cont, LV_ANIM_ON);
    cont_arr[cont_index] = cont;
    cont_index++;
    ESP_LOGI(TAG, "New product created. New count: %lu", cont_index);
}

// Crea la parte de configuración de productos en una pestaña
static void product_config_in_tab(lv_obj_t *parent)
{
    ESP_LOGI(TAG, "Creating product configuration in tab");
    lv_obj_set_size(parent, 800, 480);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *config_container = lv_obj_create(parent);
    ESP_LOGI(TAG, "Config container created");
    lv_obj_align(config_container, LV_ALIGN_CENTER, -35, -15);
    lv_obj_set_size(config_container, 750, 455);
    lv_obj_set_style_bg_color(config_container, lv_color_make(240,240,240), 0);

    // Header
    lv_obj_t *header = lv_obj_create(config_container);
    ESP_LOGI(TAG, "Header created");
    lv_obj_set_size(header, 730, 60);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, -35);
    lv_obj_t *title_label = lv_label_create(header);
    lv_label_set_text(title_label, "Configuración de productos:");
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 0);

    // Menú de productos
    lv_obj_t *menu_container = lv_obj_create(config_container);
    ESP_LOGI(TAG, "Menu container created");
    lv_obj_set_size(menu_container, 730, 350);
    lv_obj_align(menu_container, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_scroll_dir(menu_container, LV_DIR_VER);
    lv_obj_clear_flag(menu_container, LV_OBJ_FLAG_SCROLLABLE);
    menu = lv_menu_create(menu_container);
    ESP_LOGI(TAG, "Menu created for products");
    lv_obj_clear_flag(menu, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(menu, 730, 300);
    lv_obj_align(menu, LV_ALIGN_TOP_MID, 0, 0);
    main_page = lv_menu_page_create(menu, NULL);
    lv_menu_set_page(menu, main_page);
    load_products_for_config();

    // Footer con botones
    lv_obj_t *footer = lv_obj_create(config_container);
    ESP_LOGI(TAG, "Footer created for product configuration");
    lv_obj_set_size(footer, 730, 62);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 22);
    float_btn_add = lv_btn_create(footer);
    lv_obj_set_size(float_btn_add, 50, 50);
    lv_obj_align(float_btn_add, LV_ALIGN_RIGHT_MID, -50, 0);
    lv_obj_add_event_cb(float_btn_add, create_new_product, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_add, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_add, LV_SYMBOL_PLUS, 0);
    float_btn_del = lv_btn_create(footer);
    lv_obj_set_size(float_btn_del, 50, 50);
    lv_obj_align(float_btn_del, LV_ALIGN_RIGHT_MID, -120, 0);
    lv_obj_add_event_cb(float_btn_del, delete_last_item, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_del, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_del, LV_SYMBOL_MINUS, 0);
    float_btn_del_all = lv_btn_create(footer);
    lv_obj_set_size(float_btn_del_all, 50, 50);
    lv_obj_align(float_btn_del_all, LV_ALIGN_RIGHT_MID, -170, 0);
    lv_obj_add_event_cb(float_btn_del_all, delete_all_items, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_del_all, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_del_all, LV_SYMBOL_TRASH, 0);
    lv_obj_t *back_btn = lv_btn_create(footer);
    lv_obj_set_size(back_btn, 80, 40);
    lv_obj_align(back_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *btn_label = lv_label_create(back_btn);
    lv_label_set_text(btn_label, "Menú");
    lv_obj_add_event_cb(back_btn, go_to_main_screen, LV_EVENT_CLICKED, NULL);
    ESP_LOGI(TAG, "Product configuration UI created successfully");
}

// Crea la pantalla principal (con tabview) de la interfaz
void create_general_config_screen(void)
{
    ESP_LOGI(TAG, "Creating general configuration screen");
    lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, 70);
    lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE); // No se puede hacer scroll
    lv_obj_set_style_bg_color(tabview, lv_palette_lighten(LV_PALETTE_BLUE_GREY, 2), 0);
    lv_obj_set_style_pad_top(lv_tabview_get_content(tabview), 30, 0);

    // Se crean 3 pestañas
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Tab 1");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Config.\n  de\nProductos");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Tab 3");

    lv_obj_t *label = lv_label_create(tab1);
    lv_label_set_text(label, "Contenido de Tab 1");

    // En la pestaña 2 se monta la configuración de productos
    product_config_in_tab(tab2);

    label = lv_label_create(tab3);
    lv_label_set_text(label, "Contenido de Tab 3");
    ESP_LOGI(TAG, "General configuration screen created successfully");
}

// Guarda los productos en NVS
void save_products_to_nvs(void)
{
    ESP_LOGI(TAG, "Saving products to NVS. Total products: %lu", cont_index);
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error abriendo NVS\n");
        return;
    }
    err = nvs_set_u32(my_handle, "product_count", cont_index);
    if (err != ESP_OK) {
        printf("Error guardando el número de productos\n");
    }
    for (uint32_t i = 0; i < cont_index; i++) {
        if (cont_arr[i] == NULL) continue;
        lv_obj_t *cont = cont_arr[i];
        lv_obj_t *name_label = lv_obj_get_child(cont, 0);
        const char *name = lv_label_get_text(name_label);
        char key[20];
        snprintf(key, sizeof(key), "product_%lu", i);
        err = nvs_set_str(my_handle, key, name);
        if (err != ESP_OK) {
            printf("Error guardando producto %lu\n", i);
        }
    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Products saved successfully to NVS");
    printf("Productos guardados correctamente en NVS.\n");
}

// Carga los productos guardados desde NVS y los recrea en la UI
void load_products_for_config(void)
{
    ESP_LOGI(TAG, "Loading products from NVS");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("Error abriendo NVS en configuración\n");
        return;
    }
    uint32_t saved_product_count = 0;
    err = nvs_get_u32(my_handle, "product_count", &saved_product_count);
    if (err != ESP_OK) {
        printf("No se encontró el número de productos guardados\n");
        nvs_close(my_handle);
        return;
    }
    ESP_LOGI(TAG, "Found %lu products in NVS", saved_product_count);
    cont_index = 0;
    for (uint32_t i = 0; i < saved_product_count; i++) {
        char key[20];
        char product_name[32];
        size_t len = sizeof(product_name);
        snprintf(key, sizeof(key), "product_%lu", i);
        if (nvs_get_str(my_handle, key, product_name, &len) == ESP_OK) {
            ESP_LOGI(TAG, "Loading product %lu: %s", i, product_name);
            lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
            lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
            lv_textarea_set_text(name_ta, product_name);
            lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
            lv_textarea_set_text(price_ta, "");
            lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
            lv_textarea_set_text(desc_ta, "");
            lv_obj_t *save_btn = lv_btn_create(new_sub_page);
            lv_obj_t *save_label = lv_label_create(save_btn);
            lv_label_set_text(save_label, "Guardar");
            lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
            lv_obj_t *cont = lv_menu_cont_create(main_page);
            lv_obj_t *cont_label = lv_label_create(cont);
            lv_label_set_text(cont_label, product_name);
            lv_menu_set_load_page_event(menu, cont, new_sub_page);
            lv_obj_set_user_data(cont, new_sub_page);
            lv_obj_set_user_data(new_sub_page, cont);
            lv_obj_scroll_to_view_recursive(cont, LV_ANIM_ON);
            cont_arr[cont_index] = cont;
            cont_index++;
        }
    }
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Finished loading products from NVS. Total products: %lu", cont_index);
}

// -------------------------
// FUNCIÓN PRINCIPAL (app_main)
// -------------------------
void app_main(void)
{
    init_nvs();
    init_uart();
    
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
    
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel();
    register_lcd_event_callbacks(panel_handle, &disp_drv);
    init_i2c();
    esp_lcd_touch_handle_t tp = init_touch(panel_handle);
    lv_disp_t *disp = init_lvgl(panel_handle);
    start_lvgl_task(disp, tp);
    
    ESP_LOGI(TAG, "Sistema inicializado. Interfaz lista.");
    
    if(lvgl_lock(-1)) {
        lv_obj_clean(lv_scr_act());
        create_general_config_screen();
        lvgl_unlock();
    }
}
