#include "waveshare_rgb_lcd_port.h"
#include "lvgl.h"

// Variables globales para los campos de texto
lv_obj_t *ssid_field = NULL;
lv_obj_t *password_field = NULL;

// Callback para manejar el clic del botón
static void connect_btn_event_cb(lv_event_t *event)
{
    // Obtener el SSID y la contraseña
    const char *ssid = lv_textarea_get_text(ssid_field);
    const char *password = lv_textarea_get_text(password_field);

    // Conectar a la red Wi-Fi
    ESP_LOGI("MAIN", "Conectando a la red Wi-Fi...");
    ESP_LOGI("MAIN", "SSID: %s", ssid);
    ESP_LOGI("MAIN", "Contraseña: %s", password);
}

void app_main()
{
    // Inicializar la pantalla Waveshare
    waveshare_esp32_s3_rgb_lcd_init(); // Initialize the Waveshare ESP32-S3 RGB LCD
    
    ESP_LOGI("MAIN", "Interfaz Wi-Fi inicializada");

    // Bloquear el puerto para inicializar LVGL
    if (lvgl_port_lock(-1)) {
        // Obtiene la pantalla activa (screen root)
        lv_obj_t *scr = lv_scr_act();

        // Configurar el color de fondo de la pantalla como gris claro
        lv_obj_set_style_bg_color(scr, lv_color_hex(0xEEEEEE), 0);

        // Crear un panel que ocupe todo el fondo
        lv_obj_t *panel = lv_obj_create(scr);
        lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100)); // Panel ocupa todo el fondo
        lv_obj_center(panel); // Centrar el panel (opcional, ya que ocupa todo el fondo)
        lv_obj_set_style_bg_color(panel, lv_color_white(), 0);
        lv_obj_set_style_radius(panel, 0, 0); // Sin bordes redondeados
        lv_obj_set_style_border_width(panel, 0, 0); // Sin borde

        // Crear un contenedor que ocupe todo el tamaño del panel
        lv_obj_t *container = lv_obj_create(panel);
        lv_obj_set_size(container, LV_PCT(100), LV_PCT(100)); // Contenedor ocupa todo el panel
        lv_obj_set_style_pad_all(container, 10, 0); // Margen interno de 10 px
        lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW); // Usar diseño en filas (row)
        lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START); // Espaciado

        // Crear el contenedor para los campos de texto (columna izquierda)
        lv_obj_t *col_container = lv_obj_create(container);
        lv_obj_set_size(col_container, LV_PCT(70), LV_PCT(100)); // Ancho del 70% del contenedor
        lv_obj_set_flex_flow(col_container, LV_FLEX_FLOW_COLUMN); // Diseño en columnas (vertical)
        lv_obj_set_flex_align(col_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START); // Espaciado uniforme

        // Crear el campo de texto para el SSID
        ssid_field = lv_textarea_create(col_container);
        lv_obj_set_width(ssid_field, LV_PCT(100)); // Ancho al 100% del contenedor
        lv_textarea_set_placeholder_text(ssid_field, "SSID (Red Wi-Fi)");
        lv_obj_set_style_pad_top(ssid_field, 10, 0); // Espaciado superior
        lv_obj_set_style_pad_bottom(ssid_field, 10, 0); // Espaciado inferior

        // Crear el campo de texto para la contraseña
        password_field = lv_textarea_create(col_container);
        lv_obj_set_width(password_field, LV_PCT(100)); // Ancho al 100% del contenedor
        lv_textarea_set_placeholder_text(password_field, "Contraseña");
        lv_textarea_set_password_mode(password_field, true); // Ocultar caracteres
        lv_obj_set_style_pad_top(password_field, 10, 0); // Espaciado superior
        lv_obj_set_style_pad_bottom(password_field, 10, 0); // Espaciado inferior

        // Crear el botón para conectar (columna derecha)
        lv_obj_t *btn_connect = lv_btn_create(container);
        lv_obj_set_size(btn_connect, 120, 60); // Tamaño del botón
        lv_obj_align(btn_connect, LV_ALIGN_CENTER, 0, 0); // Alinear en el contenedor
        lv_obj_t *label = lv_label_create(btn_connect); // Etiqueta del botón
        lv_label_set_text(label, "Conectar");

        // Añadir el callback al botón
        lv_obj_add_event_cb(btn_connect, connect_btn_event_cb, LV_EVENT_CLICKED, NULL);

        // Desbloquear el puerto de LVGL
        lvgl_port_unlock();
    }

    // Bucle principal de tareas de LVGL (si estás usando FreeRTOS)
    while (1) {
        lv_task_handler(); // Procesa tareas de LVGL
        vTaskDelay(pdMS_TO_TICKS(10)); // Ajusta según tu sistema operativo
    }
}
