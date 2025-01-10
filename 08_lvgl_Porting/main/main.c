#include "waveshare_rgb_lcd_port.h"
#include "lvgl.h"

// Paneles
lv_obj_t *panel1 = NULL;
lv_obj_t *panel2 = NULL;
lv_obj_t *panel3 = NULL;

// Variables globales para los campos en panel2
lv_obj_t *ssid_field = NULL;
lv_obj_t *password_field = NULL;

// Función para mostrar un panel
void show_panel(lv_obj_t *panel)
{
    lv_obj_add_flag(panel1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(panel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(panel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

// Callback para el botón "Agregar Red"
static void btn_add_network_cb(lv_event_t *event)
{
    show_panel(panel2);
}

// Callback para el botón "Buscar Red"
static void btn_search_network_cb(lv_event_t *event)
{
    show_panel(panel3);
}

// Callback para el botón "Conectar" en panel2
static void connect_btn_event_cb(lv_event_t *event)
{
    const char *ssid = lv_textarea_get_text(ssid_field);
    const char *password = lv_textarea_get_text(password_field);

    ESP_LOGI("MAIN", "Conectando a la red Wi-Fi...");
    ESP_LOGI("MAIN", "SSID: %s", ssid);
    ESP_LOGI("MAIN", "Contraseña: %s", password);
}







void create_panel1(lv_obj_t *parent)
{

    panel1 = lv_obj_create(parent);
    lv_obj_set_size(panel1, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(panel1, lv_color_white(), 0);
    lv_obj_set_style_border_width(panel1, 0, 0);

    // fixme tittle label centring in panel

    lv_obj_t *label = lv_label_create(panel1);
    lv_label_set_text(label, "Configuración de Red");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    
    lv_obj_t *btn_add = lv_btn_create(panel1);
    lv_obj_set_size(btn_add, 120, 50);
    lv_obj_align(btn_add, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_t *label_add = lv_label_create(btn_add);
    lv_label_set_text(label_add, "Agregar Red");
    lv_obj_add_event_cb(btn_add, btn_add_network_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_search = lv_btn_create(panel1);
    lv_obj_set_size(btn_search, 120, 50);
    lv_obj_align(btn_search, LV_ALIGN_TOP_MID, 0, 120);
    lv_obj_t *label_search = lv_label_create(btn_search);
    lv_label_set_text(label_search, "Buscar Red");
    lv_obj_add_event_cb(btn_search, btn_search_network_cb, LV_EVENT_CLICKED, NULL);
}









void create_panel2(lv_obj_t *parent)
{
    panel2 = lv_obj_create(parent);
    lv_obj_set_size(panel2, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(panel2, lv_color_white(), 0);
    lv_obj_set_style_border_width(panel2, 0, 0);

    lv_obj_t *container = lv_obj_create(panel2);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(container, 10, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *col_container = lv_obj_create(container);
    lv_obj_set_size(col_container, LV_PCT(70), LV_PCT(100));
    lv_obj_set_flex_flow(col_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    ssid_field = lv_textarea_create(col_container);
    lv_obj_set_width(ssid_field, LV_PCT(100));
    lv_textarea_set_placeholder_text(ssid_field, "SSID (Red Wi-Fi)");

    password_field = lv_textarea_create(col_container);
    lv_obj_set_width(password_field, LV_PCT(100));
    lv_textarea_set_placeholder_text(password_field, "Contraseña");
    lv_textarea_set_password_mode(password_field, true);

    lv_obj_t *btn_connect = lv_btn_create(container);
    lv_obj_set_size(btn_connect, 120, 60);
    lv_obj_t *label_connect = lv_label_create(btn_connect);
    lv_label_set_text(label_connect, "Conectar");
    lv_obj_add_event_cb(btn_connect, connect_btn_event_cb, LV_EVENT_CLICKED, NULL);
}







void create_panel3(lv_obj_t *parent)
{
    panel3 = lv_obj_create(parent);
    lv_obj_set_size(panel3, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(panel3, lv_color_white(), 0);
    lv_obj_set_style_border_width(panel3, 0, 0);

    lv_obj_t *container = lv_obj_create(panel3);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(container, 10, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < 5; i++) {
        lv_obj_t *label = lv_label_create(container);
        lv_label_set_text_fmt(label, "Red Wi-Fi %d", i + 1);
    }
}







void app_main()
{
    // Inicializar la pantalla Waveshare
    waveshare_esp32_s3_rgb_lcd_init();
    ESP_LOGI("MAIN", "Aplicación inicializada");

    if (lvgl_port_lock(-1)) {
        lv_obj_t *scr = lv_scr_act();

        create_panel1(scr);
        create_panel2(scr);
        create_panel3(scr);

        // Ocultar todos los paneles excepto panel1 al inicio
        lv_obj_add_flag(panel2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(panel3, LV_OBJ_FLAG_HIDDEN);

        lvgl_port_unlock();
    }

    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
