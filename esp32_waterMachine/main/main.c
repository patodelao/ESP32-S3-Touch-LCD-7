/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <time.h> 
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

#include "driver/uart.h"

#include "lvgl.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_timer.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "lwip/err.h"
#include "lwip/sys.h"


// -------------------------
// DEFINES Y CONFIGURACIONES
// -------------------------
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_DISCONNECTED_BIT BIT2

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

#define UART_NUM 2
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define UART_BUFFER_SIZE 1024
#define MAX_RETRIES_ACK 3
#define ACK_TIMEOUT_MS 1000
#define STX 0x02
#define ETX 0x03

static const char *TAG = "waterMachine";

// -------------------------
// VARIABLES GLOBALES LVGL Y PARA PRODUCTOS
// -------------------------
static SemaphoreHandle_t lvgl_mux = NULL; // Mutex para LVGL

static lv_obj_t *global_header = NULL;         // Contenedor del header (barra de notificaciones)
static lv_obj_t *global_content = NULL;        // Contenedor donde se inyectan las pantallas
static lv_obj_t *global_clock_label = NULL;    // Label que muestra la hora en el header

static time_t simulated_epoch = 1739984400;    // 2025-12-31 23:00:00

static lv_obj_t *pwd_change_dialog = NULL;     // Diálogo para cambiar la contraseña
#define CONFIG_PASSWORD "root"              // Contraseña para acceder a la configuración general
// -------------------------
// VARIABLES PARA LA CONEXIÓN WIFI
// -------------------------
#define MAX_NETWORKS 5
#define DEFAULT_SCAN_LIST_SIZE 10



static lv_obj_t *wifi_status_icon = NULL;
static lv_obj_t *floating_msgbox = NULL;
static lv_obj_t *success_msgbox = NULL;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static lv_obj_t *status_label;
static lv_obj_t *ssid_field;
static lv_obj_t *password_field;
static lv_obj_t *keyboard;
static lv_obj_t *ssdropdown;
static TaskHandle_t wifi_task_handle = NULL;
static wifi_ap_record_t top_networks[MAX_NETWORKS];

static bool wifi_manual_disconnect = true;
static bool sntp_initialized = false;

static char new_dropdown_options[256] = "";
static volatile bool update_dropdown_flag = false;



///////////////////////// Variables de uart //////////////////////////
static QueueHandle_t uart_event_queue = NULL; // Cola para eventos de UART
static QueueHandle_t ack_queue = NULL; // Cola para ACK/NAK
static QueueHandle_t command_queue = NULL; // Cola para comandos recibidos
static TaskHandle_t uart_task_handle = NULL; // Handle de la tarea de UART



// --------------------------------------------------------------------









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
void create_transaction_command(const char *monto);
static void product_item_event_cb(lv_event_t * e);
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
static void save_objects_btn(lv_event_t *e);
static void create_new_product(lv_event_t *e);
static void product_config_in_tab(lv_obj_t *parent);
void save_products_to_nvs(void);
void load_products_for_config(void);
void create_general_config_screen_in_content(lv_obj_t *parent);
void create_main_screen(lv_obj_t *parent);
void switch_screen(void (*create_screen)(lv_obj_t *parent));

// -------------------------
// DECLARACIONES DE FUNCIONES USADAS
// -------------------------
static bool load_config_password_from_nvs(char *buffer, size_t size);

// -------------------------
// IMPLEMENTACIONES
// -------------------------












//--------------------------------------- TRANSACCIONES --------------------------------------- 
// Función para extraer el monto del producto seleccionado y generar el comando de transacción
void generate_transaction_from_selected_product(lv_obj_t *exhibitor_panel) {
    uint32_t child_count = lv_obj_get_child_cnt(exhibitor_panel);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *item = lv_obj_get_child(exhibitor_panel, i);
        lv_color_t current_color = lv_obj_get_style_bg_color(item, LV_PART_MAIN);

        // Si el color es verde (producto seleccionado)
        if (current_color.full == lv_color_make(0, 255, 0).full) {
            lv_obj_t *price_label = lv_obj_get_child(item, 1);
            if (price_label != NULL) {
                const char *price_text = lv_label_get_text(price_label);

                // Extraer el valor numérico del precio (eliminando el símbolo '$')
                if (price_text[0] == '$') {
                    price_text++;
                }

                // Llamar a la función para crear el comando de transacción
                create_transaction_command(price_text);
                ESP_LOGI("TRANSACTION", "Comando de transacción generado para el monto: %s", price_text);
                return;
            }
        }
    }
    ESP_LOGW("TRANSACTION", "No se seleccionó ningún producto.");
}


// Callback para el botón de compra
static void buy_button_event_cb(lv_event_t *e) {
    lv_obj_t *exhibitor_panel = lv_event_get_user_data(e);
    generate_transaction_from_selected_product(exhibitor_panel);
}






































#define MAX_RETRIES 5

static void set_wifi_icon_connected(void *arg) {
    lv_label_set_text(wifi_status_icon, LV_SYMBOL_WIFI);
}

static void set_wifi_icon_disconnected(void *arg) {
    lv_label_set_text(wifi_status_icon, LV_SYMBOL_CLOSE);
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi‑Fi iniciado. Esperando acción del usuario para conectar.");
        // Se elimina la llamada a esp_wifi_connect() para evitar el escaneo automático.
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Actualizar icono a "desconectado"
        lv_async_call(set_wifi_icon_disconnected, NULL);
        if (wifi_manual_disconnect) {
            ESP_LOGI(TAG, "DESCONECTADO");
            wifi_manual_disconnect = false;
        } else {
            if (s_retry_num < MAX_RETRIES) {
                esp_wifi_connect();
                s_retry_num++;
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                ESP_LOGI(TAG, "Reintentando conexión al AP");
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG, "Falló la conexión al AP");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Obtuvo IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    
        // Mostrar mensaje de conexión exitosa solo cuando se obtiene la IP (****)
        if (lvgl_lock(-1)) {
            success_msgbox = lv_msgbox_create(NULL, "Conexión Exitosa", "Conexión establecida correctamente.", NULL, true);
            lv_obj_center(success_msgbox);
            lvgl_unlock();
        }
        // Actualizar icono a "conectado"
        lv_async_call(set_wifi_icon_connected, NULL);

        // Iniciar SNTP si es necesario
        if (!sntp_initialized) {
            ESP_LOGI(TAG, "Iniciando SNTP");
            sntp_setoperatingmode(SNTP_OPMODE_POLL);
            sntp_setservername(0, "pool.ntp.org");
            sntp_init();
            sntp_initialized = true;
        }
    }
    
}


// Tarea para manejar la conexiÃ³n Wi-Fi
static void wifi_connect_task(void *param) {
    wifi_config_t *wifi_config = (wifi_config_t *)param;

    // Configurar la red Wi-Fi
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, wifi_config);
    if (err == ESP_OK) {
        err = esp_wifi_connect();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al conectar a la red Wi-Fi: %s", esp_err_to_name(err));
        if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo conectar a la red Wi-Fi.", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }
    }

    // Liberar memoria del parÃ¡metro recibido
    free(wifi_config);

    // Terminar la tarea
    vTaskDelete(NULL);
}
/*
static void update_dropdown_options(void *param) {
    char *options = (char *)param;
    lv_dropdown_set_options(ssdropdown, options);
    free(options); // Se libera la memoria aquí, en el callback
}
*/


static void update_dropdown_options(void *param) {
    char *options = (char *)param;
    // Verificar que ssdropdown sea válido antes de actualizar
    if (ssdropdown != NULL && lv_obj_is_valid(ssdropdown)) {
        lv_dropdown_set_options(ssdropdown, options);
    }
    free(options); // Se libera la memoria aquí, en el callback
}





static void wifi_scan_task(void *param) {
    ESP_LOGI(TAG, "Iniciando escaneo Wi-Fi...");

    uint16_t ap_count = 0;
    wifi_ap_record_t *ap_info = malloc(DEFAULT_SCAN_LIST_SIZE * sizeof(wifi_ap_record_t));
    if (ap_info == NULL) {
        ESP_LOGE(TAG, "No se pudo asignar memoria para ap_info");
        vTaskDelete(NULL);
        return;
    }

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 50,
        .scan_time.active.max = 500,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_config, false);
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGI(TAG, "Wi‑Fi no iniciado, iniciándolo...");
        ESP_ERROR_CHECK(esp_wifi_start());
        err = esp_wifi_scan_start(&scan_config, false);
    }
    ESP_ERROR_CHECK(err);

    vTaskDelay(pdMS_TO_TICKS(500)); // Espera 0.5 segundo

    esp_wifi_scan_stop();
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total de redes detectadas: %u", ap_count);

    uint16_t num_to_fetch = ap_count < DEFAULT_SCAN_LIST_SIZE ? ap_count : DEFAULT_SCAN_LIST_SIZE;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_to_fetch, ap_info));

    memset(top_networks, 0, sizeof(top_networks));
    for (int i = 0; i < num_to_fetch; i++) {
        top_networks[i] = ap_info[i];
        ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_info[i].ssid, ap_info[i].rssi);
    }
    free(ap_info);

    // Construir la cadena con los SSID detectados
    char *dropdown_options = malloc(256);
    if (dropdown_options) {
        dropdown_options[0] = '\0';
        for (int i = 0; i < num_to_fetch; i++) {
            strcat(dropdown_options, (const char *)top_networks[i].ssid);
            if (i < num_to_fetch - 1) {
                strcat(dropdown_options, "\n");
            }
        }
        // Agenda el callback para actualizar el dropdown en el contexto seguro de LVGL.
        lv_async_call(update_dropdown_options, dropdown_options);
        // No se llama a free(dropdown_options) aquí.
    }



    ESP_LOGI(TAG, "Escaneo completado.");
    wifi_task_handle = NULL; // Resetear el handle de la tarea
    vTaskDelete(NULL);
}




// FunciÃ³n para eliminar el mensaje flotante
static void remove_floating_msgbox(lv_event_t *event) {
    if (floating_msgbox != NULL) {
        lv_obj_del(floating_msgbox);
        floating_msgbox = NULL;
    }
}

static void scan_button_event_handler(lv_event_t *event) {
    if (wifi_task_handle == NULL) { // Verificar que no haya un escaneo en curso
        ESP_LOGI(TAG, "Creando tarea de escaneo en Core 0...");
        xTaskCreatePinnedToCore(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, &wifi_task_handle, 0);
    } else {
        ESP_LOGW(TAG, "Escaneo ya en curso...");
    }
}

static void connect_button_event_handler(lv_event_t *event) {
    char selected_ssid[33];
    lv_dropdown_get_selected_str(ssdropdown, selected_ssid, sizeof(selected_ssid));

    const char *password = lv_textarea_get_text((lv_obj_t *)lv_event_get_user_data(event));

    // Validar que se haya seleccionado un SSID vÃ¡lido
    if (strcmp(selected_ssid, "Seleccione una red...") == 0 || strlen(selected_ssid) == 0) {
        ESP_LOGW(TAG, "No se seleccionÃ³ una red vÃ¡lida.");
        if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "Seleccione una red vÃ¡lida.", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }
        return;
    }

    // Validar que se haya ingresado una contraseÃ±a (si es necesario)
    if (strlen(password) == 0 || strlen(password) < 8) {
        ESP_LOGW(TAG, "No se ingresÃ³ una contraseÃ±a vÃ¡lida.");
        if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "Ingrese una contraseÃ±a vÃ¡lida (al menos 8 caracteres).", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }
        return;
    }

    ESP_LOGI(TAG, "Intentando conectar a SSID: %s con contraseÃ±a: %s", selected_ssid, password);

    // if sta is conencceted, disconnect
    if (wifi_task_handle != NULL) {
        vTaskDelete(wifi_task_handle);
        wifi_task_handle = NULL;
    }

    // Configura la red Wi-Fi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, selected_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err == ESP_OK) {
        err = esp_wifi_connect();
        /*if (err == ESP_OK) {
            ESP_LOGI(TAG, "Wifi_set_config OK");
            }*/
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar la conexiÃ³n Wi-Fi: %s", esp_err_to_name(err));
        if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo iniciar la conexiÃ³n Wi-Fi.", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }
    }
}

static void disconnect_button_event_handler(lv_event_t *event) {
    wifi_manual_disconnect = true;  // Indica que la desconexión fue solicitada manualmente
    esp_err_t err = esp_wifi_disconnect();
    if (err == ESP_OK) {
        //ESP_LOGI(TAG, "Desconexión exitosa.");
        /*if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Desconectado", "Se ha desconectado de la red Wi-Fi.", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }*/
    } else {
        ESP_LOGE(TAG, "Error al desconectar: %s", esp_err_to_name(err));
        if (lvgl_lock(-1)) {
            lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo desconectar de la red Wi-Fi.", NULL, true);
            lv_obj_center(msgbox);
            lvgl_unlock();
        }
    }
}


static lv_obj_t *keyboard;

static void keyboard_event_handler(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t *keyboard = lv_event_get_target(event);
    lv_obj_move_foreground(keyboard);
    

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

        // AsegÃºrate de que el teclado no estÃ© oculto
        lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_opa(keyboard, LV_OPA_COVER, LV_PART_MAIN); // Mostrarlo con opacidad completa
    }
}

















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

// Inicialización de UART 
static void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_event_queue, 0);

    // Crear colas
    ack_queue = xQueueCreate(5, sizeof(uint8_t)); 
    command_queue = xQueueCreate(10, UART_BUFFER_SIZE);

    if (!command_queue || !ack_queue) {
        printf("Error: No se pudo crear las colas.\n");
    }
}


void uart_send_command(const uint8_t *command, size_t length) {
    uart_write_bytes(UART_NUM, (const char *)command, length);
}


static void uart_RX_task(void *param) {
    uint8_t data_buffer[UART_BUFFER_SIZE];
    size_t bytes_read = 0;

    while (1) {
        uart_event_t event;
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    bytes_read = uart_read_bytes(UART_NUM, data_buffer, sizeof(data_buffer), pdMS_TO_TICKS(10));
                    if (bytes_read > 0) {
                        for (size_t i = 0; i < bytes_read; i++) {
                            // Verificar ACK o NAK
                            if (data_buffer[i] == 0x06 || data_buffer[i] == 0x15) {
                                xQueueSend(ack_queue, &data_buffer[i], portMAX_DELAY);
                            }
                        }

                        // Enviar el mensaje completo a la cola de procesamiento
                        xQueueSend(command_queue, data_buffer, pdMS_TO_TICKS(100));
                    }
                    break;

                default:
                    printf("Evento UART no manejado: %d\n", event.type);
                    break;
            }
        }
    }
}


static void command_processing_task(void *param) {
    uint8_t command_buffer[UART_BUFFER_SIZE];

    while (1) {
        if (xQueueReceive(command_queue, command_buffer, portMAX_DELAY)) {
            printf("Procesando comando: %s\n", command_buffer);
            // Procesa el comando recibido (puedes agregar lógica según el protocolo que uses)
        }
    }
}



static void wait_for_ack_task(void *param) {
    uint8_t *command = (uint8_t *)param;
    size_t length = strlen((char *)command);

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        printf("Intento %d: Enviando comando...\n", attempt);
        uart_write_bytes(UART_NUM, (const char *)command, length);

        uint8_t received_byte;
        if (xQueueReceive(ack_queue, &received_byte, pdMS_TO_TICKS(ACK_TIMEOUT_MS))) {
            if (received_byte == 0x06) {
                printf("ACK recibido.\n");
                break;
            } else if (received_byte == 0x15) {
                printf("NAK recibido. Reintentando...\n");
            }
        } else {
            printf("Timeout esperando ACK.\n");
        }
    }

    free(command);
    vTaskDelete(NULL);
}


void send_command_with_ack(const char *command) {
    uint8_t *formatted_command = malloc(strlen(command) + 1);
    strcpy((char *)formatted_command, command);

    xTaskCreate(wait_for_ack_task, "wait_for_ack_task", 4096, formatted_command, 2, NULL);
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
// GENERACIÓN DE COMANDOS
// -------------------------


uint8_t calculate_lrc(const uint8_t *data, size_t length) {
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++) {
        lrc ^= data[i];
    }
    return lrc;
}

void create_transaction_command(const char *monto) {
    char monto_formateado[10];
    memset(monto_formateado, '0', 9);
    monto_formateado[9] = '\0';
    int len_monto = strlen(monto);
    memmove(monto_formateado + (9 - len_monto), monto, len_monto);

    const char *codigo_cmd = "0200";
    const char *ticket_number = "2189aaA987321";
    const char *campo_impresion = "1";
    const char *enviar_msj = "1";

    char command[256];
    snprintf(command, sizeof(command), "%s|%s|%s|%s|%s", codigo_cmd, monto_formateado, ticket_number, campo_impresion, enviar_msj);

    size_t command_length = strlen(command) + 3;
    uint8_t *formatted_command = malloc(command_length);

    if (!formatted_command) {
        ESP_LOGE(TAG, "Error: No se pudo asignar memoria para el comando.");
        return;
    }

    size_t index = 0;
    formatted_command[index++] = STX;
    memcpy(&formatted_command[index], command, strlen(command));
    index += strlen(command);
    formatted_command[index++] = ETX;

    uint8_t lrc = calculate_lrc(formatted_command + 1, index - 1);
    formatted_command[index++] = lrc;

    ESP_LOGI(TAG, "Mensaje construido dinámicamente:");
    for (size_t i = 0; i < index; i++) {
        printf("%02X ", formatted_command[i]);
    }
    printf("\nLRC Calculado: %02X\n", lrc);

    uart_send_command(formatted_command, index);

    free(formatted_command);
}












// -------------------------
// INTERFAZ DE CONFIGURACIÓN DE WIFI
// -------------------------


// Variable global para controlar la inicialización del servicio Wi‑Fi

static bool wifi_initialized = false;

void wifi_service_init(void) {
    if (wifi_initialized) {
        ESP_LOGI(TAG, "Servicio Wi‑Fi ya está inicializado.");
        return;
    }

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif != NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Inicia el driver Wi‑Fi en modo estación
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_initialized = true;
    ESP_LOGI(TAG, "Servicio Wi‑Fi inicializado.");
}



void wifi_init_sta(const char *ssid, const char *password) {
    // Asegura que el servicio Wi‑Fi esté iniciado
    if (!wifi_initialized) {
        wifi_service_init();
    }

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    // Actualiza la configuración y conecta
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "Wi‑Fi initialization completed.");

    // Espera hasta 10 segundos para que se establezca la conexión
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));

}


static void connect_event_handler(lv_event_t *event) {
    const char *ssid = lv_textarea_get_text(ssid_field);
    const char *password = lv_textarea_get_text(password_field);
    
    if (strlen(ssid) == 0 || strlen(password) == 0) {
        lv_label_set_text(status_label, "Por favor, complete los campos.");
        return;
    }
    lv_label_set_text(status_label, "Conectando...");
    wifi_init_sta(ssid, password);
}


void create_wifi_settings_widget(lv_obj_t *parent) {
    // Configurar el fondo del widget (opcional)
    lv_obj_set_size(parent, 300, 400);
    lv_obj_center(parent);

    lv_obj_set_style_bg_color(parent, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE); // Deshabilitar scroll
    
    // Contenedor principal para la UI de WiFi
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE); // Deshabilitar scroll 
    lv_obj_set_size(container, 300, 400);
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    // Dropdown para seleccionar SSID
    ssdropdown = lv_dropdown_create(container);
    lv_obj_set_width(ssdropdown, 200);
    lv_dropdown_set_options(ssdropdown, "Seleccione una red...");

    // Campo de texto para ingresar la contraseña
    lv_obj_t *password_textarea = lv_textarea_create(container);
    lv_textarea_set_one_line(password_textarea, true);
    lv_textarea_set_password_mode(password_textarea, true);
    lv_textarea_set_placeholder_text(password_textarea, "Ingrese contraseña");
    lv_obj_set_width(password_textarea, 200);
    lv_obj_add_event_cb(password_textarea, textarea_event_handler, LV_EVENT_FOCUSED, NULL);

    
    // Asignamos un string de prueba  de wifi_password para que aparezca ya escrito
    lv_textarea_set_text(password_textarea, "111111111111111");

    // Botón para escanear redes WiFi
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

    // Botón para desconectar
    lv_obj_t *disconnect_btn = lv_btn_create(container);
    lv_obj_set_size(disconnect_btn, 120, 50);
    lv_obj_t *disconnect_label = lv_label_create(disconnect_btn);
    lv_label_set_text(disconnect_label, "Desconectar");
    lv_obj_center(disconnect_label);
    lv_obj_add_event_cb(disconnect_btn, disconnect_button_event_handler, LV_EVENT_CLICKED, NULL);
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
    // Registra el callback para que al recibir foco se muestre el teclado
    lv_obj_add_event_cb(ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
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



// Botón para salir (guarda y vuelve a la pantalla principal)
static void save_objects_btn(lv_event_t *e)
{
    ESP_LOGI(TAG, "Go to main screen button pressed");
    save_products_to_nvs();
    //lv_async_call(transition_to_main_screen, NULL);
}

// Crea un nuevo producto y su correspondiente sub-página
static void create_new_product(lv_event_t *e) {
    ESP_LOGI(TAG, "Creating new product. Current count: %lu", cont_index);
    if (cont_index >= MAX_ITEMS) {
        ESP_LOGW(TAG, "Maximum products reached");
        printf("No se pueden añadir más productos\n");
        return;
    }
    lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
    ESP_LOGI(TAG, "New sub-page created for product %lu", cont_index + 1);
    
    // Crea el campo "Nombre" y le agrega el evento para mostrar el teclado
    lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
    lv_obj_add_event_cb(name_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
    static char default_name[20];
    snprintf(default_name, sizeof(default_name), "Producto %lu", cont_index + 1);
    lv_textarea_set_text(name_ta, default_name);  // Nombre por defecto

    // Crea el campo "Precio"
    lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
    lv_obj_add_event_cb(price_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
    lv_textarea_set_text(price_ta, "100");  // Precio por defecto

    // Crea el campo "Descripción"
    lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
    lv_obj_add_event_cb(desc_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);

    // Crea el botón de guardar
    lv_obj_t *save_btn = lv_btn_create(new_sub_page);
    lv_obj_t *label = lv_label_create(save_btn);
    lv_label_set_text(label, "Guardar");
    lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
    ESP_LOGI(TAG, "Save button created for new product");

    lv_obj_t *cont = lv_menu_cont_create(main_page);
    lv_obj_t *cont_label = lv_label_create(cont);
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
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *config_container = lv_obj_create(parent);
    lv_obj_clear_flag(config_container, LV_OBJ_FLAG_SCROLLABLE);

    ESP_LOGI(TAG, "Config container created");

    lv_coord_t w = 800;
    lv_coord_t h = 480;



    lv_obj_center(config_container);
    lv_obj_set_size(config_container, w, h);
    lv_obj_set_style_bg_color(config_container, lv_color_make(240,240,240), 0);

    // Header
    lv_obj_t *header = lv_obj_create(config_container);
    ESP_LOGI(TAG, "Header created");
    lv_obj_set_size(header, 730, 60);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, -10);
    lv_obj_t *title_label = lv_label_create(header);
    lv_label_set_text(title_label, "Configuración de productos:");
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 0);

    // Menú de productos
    lv_obj_t *menu_container = lv_obj_create(config_container);
    ESP_LOGI(TAG, "Menu container created");
    lv_obj_set_size(menu_container, 730, 350);
    lv_obj_align(menu_container, LV_ALIGN_TOP_MID, 0, 50);
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
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);
    ESP_LOGI(TAG, "Footer created for product configuration");
    lv_obj_set_size(footer, 730, 62);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);
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
    lv_obj_t *save_btn = lv_btn_create(footer);
    lv_obj_set_size(save_btn, 40, 40);
    lv_obj_align(save_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *btn_label = lv_label_create(save_btn);
    lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(btn_label, LV_SYMBOL_SAVE);
    lv_obj_add_event_cb(save_btn, save_objects_btn, LV_EVENT_CLICKED, NULL);
    ESP_LOGI(TAG, "Product configuration UI created successfully");
}

// Callback para volver al main_screen desde la configuración mediante botón flotante
static void go_to_main_screen_from_config_cb(lv_event_t *e) {
    // Cambia el contenido al main_screen
    switch_screen(create_main_screen);
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
        // Suponemos que el contenedor "cont" guarda como user_data la subpágina con la edición del producto
        lv_obj_t *cont = cont_arr[i];
        lv_obj_t *sub_page = lv_obj_get_user_data(cont);
        if(sub_page == NULL) continue;
        
        // Se asume que en sub_page se crearon tres textareas en el siguiente orden:
        // Índice 0: Nombre, índice 1: Precio, índice 2: Descripción.
        lv_obj_t *name_ta = lv_obj_get_child(sub_page, 0);
        lv_obj_t *price_ta = lv_obj_get_child(sub_page, 1);
        lv_obj_t *desc_ta = lv_obj_get_child(sub_page, 2);
        
        const char *name = lv_textarea_get_text(name_ta);
        const char *price = lv_textarea_get_text(price_ta);
        const char *desc = lv_textarea_get_text(desc_ta);
        
        char key[20];
        snprintf(key, sizeof(key), "product_%lu", i);
        err = nvs_set_str(my_handle, key, name);
        if (err != ESP_OK) {
            printf("Error guardando producto %lu\n", i);
        }
        
        snprintf(key, sizeof(key), "price_%lu", i);
        err = nvs_set_str(my_handle, key, price);
        if (err != ESP_OK) {
            printf("Error guardando precio del producto %lu\n", i);
        }
        
        snprintf(key, sizeof(key), "desc_%lu", i);
        err = nvs_set_str(my_handle, key, desc);
        if (err != ESP_OK) {
            printf("Error guardando descripción del producto %lu\n", i);
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
        size_t name_len = sizeof(product_name);
        snprintf(key, sizeof(key), "product_%lu", i);
        if (nvs_get_str(my_handle, key, product_name, &name_len) == ESP_OK) {
            ESP_LOGI(TAG, "Loading product %lu: %s", i, product_name);
            
            // Cargar precio
            char price_key[20];
            char product_price[16];
            size_t price_len = sizeof(product_price);
            snprintf(price_key, sizeof(price_key), "price_%lu", i);
            if(nvs_get_str(my_handle, price_key, product_price, &price_len) != ESP_OK){
                strcpy(product_price, "50");
            }
            
            // Cargar descripción
            char desc_key[20];
            char product_desc[64];  // Ajusta el tamaño según tus necesidades
            size_t desc_len = sizeof(product_desc);
            snprintf(desc_key, sizeof(desc_key), "desc_%lu", i);
            if(nvs_get_str(my_handle, desc_key, product_desc, &desc_len) != ESP_OK){
                strcpy(product_desc, "");
            }
            
            // Crea la sub-página para editar el producto
            lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
            // Crea el campo de nombre y asigna el valor leído
            lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
            lv_textarea_set_text(name_ta, product_name);
            // Crea el campo de precio y asigna el valor leído
            lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
            lv_textarea_set_text(price_ta, product_price);
            // Crea el campo de descripción y asigna el valor leído
            lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
            lv_textarea_set_text(desc_ta, product_desc);
            
            // Crea el botón de guardar
            lv_obj_t *save_btn = lv_btn_create(new_sub_page);
            lv_obj_t *save_label = lv_label_create(save_btn);
            lv_label_set_text(save_label, "Guardar");
            lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
            
            // Crea el contenedor del ítem en el menú principal
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











void switch_screen(void (*create_screen)(lv_obj_t *parent)) {
    // Limpia el área de contenido eliminando sus hijos
    lv_obj_clean(global_content);
    // Llama a la función que crea la pantalla, usando global_content como padre
    create_screen(global_content);
}



void create_main_structure(void) {
    // Crea un contenedor principal que ocupa toda la pantalla
    lv_obj_t *main_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(main_container, LV_HOR_RES, LV_VER_RES);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

    // --- Header (Barra de notificaciones con reloj) ---
    global_header = lv_obj_create(main_container);
    lv_obj_set_size(global_header, LV_HOR_RES, 30);  // Ajusta la altura según necesites
    lv_obj_align(global_header, LV_ALIGN_TOP_MID, 0, -28);
    lv_obj_set_style_bg_color(global_header, lv_color_hex3(0x333), 0);
    lv_obj_set_style_pad_all(global_header, 0, 0);

    //label para el reloj y lo guarda globalmente
    global_clock_label = lv_label_create(global_header);
    lv_label_set_text(global_clock_label, "00:00");  // Valor inicial (se actualizará luego)+
    lv_obj_set_style_text_color(global_clock_label, lv_color_white(), 0);
    lv_obj_align(global_clock_label, LV_ALIGN_CENTER, 0, 0);

    // Label del icono de Wi‑Fi (lado derecho)
    // Inicialmente muestra LV_SYMBOL_CLOSE (desconectado)
    wifi_status_icon = lv_label_create(global_header);
    lv_label_set_text(wifi_status_icon, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(wifi_status_icon, lv_color_white(), 0);
    lv_obj_align(wifi_status_icon, LV_ALIGN_RIGHT_MID, 0, 0);





    // --- Área de Contenido ---
    global_content = lv_obj_create(main_container);
    lv_obj_set_style_pad_all(global_content, 0, 0);
    lv_obj_set_style_border_width(global_content, 0, 0);
    lv_obj_set_style_border_color(global_content, lv_color_black(), 0);
    lv_obj_set_size(global_content, LV_HOR_RES, LV_VER_RES - 20);
    lv_obj_align(global_content, LV_ALIGN_BOTTOM_MID, 0, 25);
    lv_obj_clear_flag(global_content, LV_OBJ_FLAG_SCROLLABLE);
}













































































///////////////////////// Configuracion de password config /////////////////////////

// Callback para el botón de confirmación en el diálogo de contraseña, usando NVS.
static void confirm_password_event_cb(lv_event_t *e) {
    // "user_data" es nuestro diálogo
    lv_obj_t *dialog = lv_event_get_user_data(e);
    if (!lv_obj_is_valid(dialog)) return;
    
    // Obtenemos el textarea (suponiendo que es el segundo hijo del diálogo)
    lv_obj_t *password_ta = lv_obj_get_child(dialog, 1);
    if (!lv_obj_is_valid(password_ta)) return;
    
    const char *entered_pass = lv_textarea_get_text(password_ta);
    if (entered_pass == NULL) {
        entered_pass = "";
    }
    ESP_LOGI(TAG, "Texto ingresado: '%s'", entered_pass);
    
    char stored_pass[32] = {0};
    if (!load_config_password_from_nvs(stored_pass, sizeof(stored_pass))) {
        // Si no se pudo cargar, usamos la contraseña maestra inicial
        strcpy(stored_pass, CONFIG_PASSWORD);
    }
    ESP_LOGI(TAG, "Contraseña almacenada: '%s'", stored_pass);
    
    // Compara las contraseñas
    if (strcmp(entered_pass, stored_pass) == 0) {
        // Deshabilita el botón para evitar múltiples clics
        lv_obj_t *btn = lv_event_get_target(e);
        if (lv_obj_is_valid(btn)) {
            lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        }
        // Elimina el diálogo y cambia de pantalla
        lv_obj_del(dialog);
        switch_screen(create_general_config_screen_in_content);
    } else {
        lv_obj_t *error_msg = lv_msgbox_create(NULL, "Error", "Contraseña incorrecta.", NULL, true);
        lv_obj_center(error_msg);
    }
}

// Función para mostrar el diálogo de contraseña
static void show_config_password_dialog(void) {
    // Crea un contenedor modal para el diálogo
    lv_obj_t *dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(dialog, 300, 200);
    lv_obj_center(dialog);
    lv_obj_set_style_bg_color(dialog, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(dialog, 2, 0);

    // Título del diálogo
    lv_obj_t *title = lv_label_create(dialog);
    lv_label_set_text(title, "Ingrese contraseña:");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Campo de texto para la contraseña
    lv_obj_t *password_ta = lv_textarea_create(dialog);
    lv_textarea_set_placeholder_text(password_ta, "Contraseña");
    lv_textarea_set_one_line(password_ta, true);
    lv_textarea_set_password_mode(password_ta, true);
    lv_obj_set_width(password_ta, 250);
    lv_obj_align(password_ta, LV_ALIGN_CENTER, 0, -10);
    // Se asocia el manejador de teclado (ya definido en el código)
    lv_obj_add_event_cb(password_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);

    // Botón de confirmación
    lv_obj_t *btn_confirm = lv_btn_create(dialog);
    lv_obj_set_size(btn_confirm, 100, 40);
    lv_obj_align(btn_confirm, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *btn_label = lv_label_create(btn_confirm);
    lv_label_set_text(btn_label, "Confirmar");
    lv_obj_center(btn_label);
    // Se asigna como user_data el diálogo para que el callback pueda eliminarlo en caso de éxito.
    lv_obj_add_event_cb(btn_confirm, confirm_password_event_cb, LV_EVENT_CLICKED, dialog);
}

// Callback del botón de configuración general para mostrar el diálogo
static void btn_to_config_event_cb(lv_event_t *e) {
    show_config_password_dialog();
}






/////////////////////////////////// password general config screen //////////////////////////////////////

// --- Funciones para cargar y guardar la contraseña de configuración en NVS ---

// Intenta cargar la contraseña almacenada en NVS bajo la clave "config_password".
// Si no existe, se retorna false.
static bool load_config_password_from_nvs(char *buffer, size_t size) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if(err != ESP_OK) {
        return false;
    }
    err = nvs_get_str(my_handle, "config_password", buffer, &size);
    ESP_LOGI(TAG, "Cargando contraseña de NVS: %s", buffer);
    nvs_close(my_handle);
    return (err == ESP_OK);
}

// Guarda la nueva contraseña en NVS bajo la clave "config_password".
static void save_config_password_to_nvs(const char *password) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if(err == ESP_OK) {
        ESP_LOGI(TAG, "Guardando nueva contraseña en NVS: %s", password);
        nvs_set_str(my_handle, "config_password", password);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}


// Callback sin usar la estructura personalizada, modificado para imprimir la contraseña guardada
static void password_change_confirm_cb(lv_event_t *e) {
    // Se obtiene el contenedor del menú de cambio (user_data)
    lv_obj_t *container = (lv_obj_t *) lv_event_get_user_data(e);
    if (!lv_obj_is_valid(container)) return;
    
    // Suponiendo que el orden de los hijos es:
    // Índice 0: Título
    // Índice 1: Textarea de la clave actual
    // Índice 2: Textarea de la nueva clave
    // Índice 3: Botón (el que disparó el callback)
    lv_obj_t *current_pass_ta = lv_obj_get_child(container, 1);
    lv_obj_t *new_pass_ta = lv_obj_get_child(container, 2);
    
    const char *current_pass = lv_textarea_get_text(current_pass_ta);
    const char *new_pass = lv_textarea_get_text(new_pass_ta);
    if (current_pass == NULL) current_pass = "";
    if (new_pass == NULL) new_pass = "";
    
    char stored_pass[32] = {0};
    // Intenta cargar la contraseña almacenada; si falla, se usa la contraseña maestra CONFIG_PASSWORD.
    if (!load_config_password_from_nvs(stored_pass, sizeof(stored_pass))) {
        strcpy(stored_pass, CONFIG_PASSWORD);
    }
    ESP_LOGI(TAG, "Clave ingresada: '%s' | Almacenada: '%s'", current_pass, stored_pass);
    
    // Verifica que la clave actual ingresada coincida con la almacenada.
    if (strcmp(current_pass, stored_pass) != 0) {
        lv_obj_t *msg = lv_msgbox_create(NULL, "Error", "Clave actual incorrecta.", NULL, true);
        lv_obj_center(msg);
        return;
    }
    // Verifica que la nueva contraseña no esté vacía.
    if (strlen(new_pass) == 0) {
        lv_obj_t *msg = lv_msgbox_create(NULL, "Error", "Nueva clave inválida.", NULL, true);
        lv_obj_center(msg);
        return;
    }
    
    // Guarda la nueva contraseña en NVS.
    save_config_password_to_nvs(new_pass);
    
    // Carga nuevamente la contraseña almacenada para verificar e imprimir.
    char new_stored[32] = {0};
    if (load_config_password_from_nvs(new_stored, sizeof(new_stored))) {
        ESP_LOGI(TAG, "Nueva contraseña almacenada: '%s'", new_stored);
    } else {
        ESP_LOGI(TAG, "No se pudo cargar la nueva contraseña");
    }
    
    lv_obj_t *msg = lv_msgbox_create(NULL, "Éxito", "Contraseña actualizada.", NULL, true);
    lv_obj_center(msg);
    
    // Limpia los campos para que el contenedor permanezca y pueda usarse de nuevo.
    lv_textarea_set_text(current_pass_ta, "");
    lv_textarea_set_text(new_pass_ta, "");
}


// Función que crea el menú para cambiar la contraseña sin usar una estructura
static void create_password_change_menu(lv_obj_t *parent) {
    // Crea un contenedor para el menú de cambio de contraseña
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, 400, 300);
    lv_obj_center(container);
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(container, 2, 0);
    
    // Agrega el título (índice 0)
    lv_obj_t *title = lv_label_create(container);
    lv_label_set_text(title, "Cambiar Clave de Configuración");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Agrega el textarea para la contraseña actual (índice 1)
    lv_obj_t *current_pass_ta = lv_textarea_create(container);
    lv_textarea_set_placeholder_text(current_pass_ta, "Clave actual");
    lv_textarea_set_one_line(current_pass_ta, true);
    lv_textarea_set_password_mode(current_pass_ta, true);
    lv_obj_set_width(current_pass_ta, 300);
    lv_obj_align(current_pass_ta, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_add_event_cb(current_pass_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
    
    // Agrega el textarea para la nueva contraseña (índice 2)
    lv_obj_t *new_pass_ta = lv_textarea_create(container);
    lv_textarea_set_placeholder_text(new_pass_ta, "Nueva clave");
    lv_textarea_set_one_line(new_pass_ta, true);
    lv_textarea_set_password_mode(new_pass_ta, true);
    lv_obj_set_width(new_pass_ta, 300);
    lv_obj_align(new_pass_ta, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_add_event_cb(new_pass_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
    
    // Agrega el botón de confirmación (índice 3)
    lv_obj_t *confirm_btn = lv_btn_create(container);
    lv_obj_set_size(confirm_btn, 100, 40);
    lv_obj_align(confirm_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *btn_label = lv_label_create(confirm_btn);
    lv_label_set_text(btn_label, "Actualizar");
    lv_obj_center(btn_label);
    
    // Pasa el contenedor (que contiene todos los elementos) como user_data
    lv_obj_add_event_cb(confirm_btn, password_change_confirm_cb, LV_EVENT_CLICKED, container);
}
















































// Función para crear la pantalla de configuración dentro del contenedor "content"
void create_general_config_screen_in_content(lv_obj_t *parent) {
    ESP_LOGI(TAG, "Creating general configuration screen in content");
    // Se crea el tabview dentro del contenedor padre (global_content)
    lv_obj_t *tabview = lv_tabview_create(parent, LV_DIR_LEFT, 70);
    lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(tabview, lv_palette_lighten(LV_PALETTE_BLUE_GREY, 2), 0);

    // Se crean 3 pestañas
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview,"   " LV_SYMBOL_WIFI "\nConfig.\nWiFi");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview,"    " LV_SYMBOL_LIST "\nConfig.\nProducts");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview,"    ****\nCambiar\nClave\nadmin.");

    /* --- TAB 1: UI de Configuración WiFi --- */
    lv_obj_t *container1 = lv_obj_create(tab1);
   // lv_obj_set_size(container1, lv_obj_get_width(tab1), lv_obj_get_height(tab1));
    lv_obj_set_align(container1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(container1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(container1, LV_SCROLLBAR_MODE_OFF);
    // Se llama a la función para crear la UI de WiFi dentro de container1
    create_wifi_settings_widget(container1);

    /* --- TAB 2: Configuración de productos --- */
    product_config_in_tab(tab2);

    // --- TAB 3: Contenedor que ocupa todo el espacio ---

    create_password_change_menu(tab3);
















    /*
    lv_obj_t *container3 = lv_obj_create(tab3);
    lv_obj_set_size(container3, lv_obj_get_width(tab3), lv_obj_get_height(tab3));
    lv_obj_set_align(container3, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(container3, lv_color_hex(0x00FF00), 0); // Fondo verde
    lv_obj_clear_flag(container3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *label3 = lv_label_create(container3);
    lv_label_set_text(label3, "Contenido de Tab 3");
    lv_obj_center(label3); */






    // --- Botón flotante para regresar al main_screen ---
    lv_obj_t *float_btn = lv_btn_create(parent);
    lv_obj_set_size(float_btn, 50, 50);
    // Coloca el botón flotante en la esquina inferior derecha del contenedor padre
    lv_obj_align(float_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_radius(float_btn, LV_RADIUS_CIRCLE, 0);
    // Opcional: asigna un color de fondo o ícono
    lv_obj_set_style_bg_color(float_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_t *icon = lv_label_create(float_btn);
    lv_label_set_text(icon, LV_SYMBOL_LEFT); // Puedes usar LV_SYMBOL_LEFT o un texto como "Main"
    lv_obj_center(icon);
    lv_obj_add_event_cb(float_btn, go_to_main_screen_from_config_cb, LV_EVENT_CLICKED, NULL);
}



// Modifica (o reemplaza) la función create_main_screen en main1.c por algo similar a:
void create_main_screen(lv_obj_t *parent) {
    // --- Panel Exhibidor de Productos ---
    lv_obj_t *exhibitor_panel = lv_obj_create(parent);
    lv_obj_set_size(exhibitor_panel, 800, 250);
    // Ubica el panel en la parte superior del área de contenido
    lv_obj_align(exhibitor_panel, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_pad_all(exhibitor_panel, 10, 0);
    lv_obj_set_style_bg_color(exhibitor_panel, lv_palette_main(LV_PALETTE_GREY), 0);
    // Configura el layout para distribuir los ítems de forma flexible
    lv_obj_set_layout(exhibitor_panel, LV_LAYOUT_FLEX);
    lv_obj_set_style_flex_flow(exhibitor_panel, LV_FLEX_FLOW_ROW, 0);
    lv_obj_set_flex_align(exhibitor_panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // --- Carga y muestra los productos almacenados en NVS ---
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        uint32_t product_count = 0;
        nvs_get_u32(my_handle, "product_count", &product_count);
        if (product_count > 0) {
            for (uint32_t i = 0; i < product_count; i++) {
                char key[20];
                char product_name[32];
                size_t len = sizeof(product_name);
                snprintf(key, sizeof(key), "product_%lu", i);
                if (nvs_get_str(my_handle, key, product_name, &len) == ESP_OK) {
                    // Intenta obtener también el precio; si falla, asigna un valor por defecto
                    char price_key[20];
                    char product_price[16];
                    size_t price_len = sizeof(product_price);
                    snprintf(price_key, sizeof(price_key), "price_%lu", i);
                    if(nvs_get_str(my_handle, price_key, product_price, &price_len) != ESP_OK){
                        strcpy(product_price, "50");
                    }
                    
                    // Crea el "item" para el producto
                    lv_obj_t *item = lv_obj_create(exhibitor_panel);
                    lv_obj_set_size(item, 120, 100);
                    lv_obj_set_style_bg_color(item, lv_palette_main(LV_PALETTE_BLUE), 0);
                    // Agrega el callback para el toque del item
                    lv_obj_add_event_cb(item, product_item_event_cb, LV_EVENT_CLICKED, NULL);
                    
                    // Crea y posiciona la etiqueta con el nombre del producto
                    lv_obj_t *name_label = lv_label_create(item);
                    lv_label_set_text(name_label, product_name);
                    lv_obj_align(name_label, LV_ALIGN_TOP_MID, 0, 5);
                    
                    // Crea y posiciona la etiqueta con el precio formateado
                    char formatted_price[32];
                    snprintf(formatted_price, sizeof(formatted_price), "$%s", product_price);
                    lv_obj_t *price_label = lv_label_create(item);
                    lv_label_set_text(price_label, formatted_price);
                    lv_obj_align(price_label, LV_ALIGN_BOTTOM_MID, 0, -5);
                }
            }
        } else {
            // Si no hay productos, muestra un mensaje
            lv_obj_t *label = lv_label_create(exhibitor_panel);
            lv_label_set_text(label, "No hay productos");
            lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        }
        nvs_close(my_handle);
    }
    
    //add_buy_button(parent, exhibitor_panel);


    // --- Botón de compra ---
    lv_obj_t *buy_btn = lv_btn_create(parent);
    lv_obj_set_size(buy_btn, 100, 50);
    lv_obj_align(buy_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    // Se pasa el exhibitor_panel como user_data
    lv_obj_add_event_cb(buy_btn, buy_button_event_cb, LV_EVENT_CLICKED, exhibitor_panel);
    lv_obj_t *buy_label = lv_label_create(buy_btn);
    lv_label_set_text(buy_label, "Comprar");
    lv_obj_center(buy_label);


    // --- Botón de Configuración ---
    lv_obj_t *btn_config = lv_btn_create(parent);
    lv_obj_set_size(btn_config, 40, 40);
    lv_obj_align(btn_config, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    // Usa el callback que ya tienes para ir a la pantalla de configuración
    lv_obj_add_event_cb(btn_config, btn_to_config_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_config = lv_label_create(btn_config);
    lv_label_set_text(label_config, LV_SYMBOL_SETTINGS); // "Configurar Productos"
    lv_obj_center(label_config);
}





// Guarda el valor epoch actual en la NVS
void save_epoch(time_t epoch) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        // Si el epoch se guarda como 32 bits:
        err = nvs_set_u32(my_handle, "last_epoch", (uint32_t)epoch);
        if (err == ESP_OK) {
            nvs_commit(my_handle);
        }
        nvs_close(my_handle);
    }
}

// Recupera el último valor epoch almacenado en la NVS.
// Si no existe, se puede usar un valor por defecto.
time_t load_epoch(void) {
    nvs_handle_t my_handle;
    uint32_t stored_epoch = 0;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        if (nvs_get_u32(my_handle, "last_epoch", &stored_epoch) != ESP_OK) {
            // No se encontró el valor, se usa un valor por defecto, por ejemplo:
            stored_epoch = 1739984400; // 19/02/2025 17:00:00
        }
        nvs_close(my_handle);
    } else {
        // En caso de error, se usa el valor por defecto
        stored_epoch = 1739984400;
    }
    return (time_t)stored_epoch;
}




static void update_clock_cb(lv_timer_t * timer) {
    time_t now;
    // Si SNTP está inicializado y sincronizado, se usa la hora real
    if (sntp_initialized && sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "SNTP sincronizado");
         time(&now);
         simulated_epoch = now;  // Sincronizamos el epoch simulado
    } else {
         // Si no se tiene sincronización, se incrementa manualmente el epoch
         simulated_epoch++;
         now = simulated_epoch;
    }
    
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char datetime_str[20];  // Asegúrate de que el buffer es lo suficientemente grande
    strftime(datetime_str, sizeof(datetime_str), "%d/%m/%Y %H:%M:%S", &timeinfo);
    
    // Actualiza el label del header con la hora formateada
    lv_label_set_text(global_clock_label, datetime_str);
    
    // Guarda el epoch actualizado en NVS para persistirlo
    save_epoch(simulated_epoch);
}










///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void product_item_event_cb(lv_event_t * e) {
    // Obtén el ítem que disparó el evento
    lv_obj_t * item = lv_event_get_target(e);
    // Obtén el contenedor padre (el panel de productos)
    lv_obj_t * product_panel = lv_obj_get_parent(item);
    
    // Si el ítem ya está seleccionado (verde), se deselecciona (vuelve azul)
    lv_color_t current_color = lv_obj_get_style_bg_color(item, LV_PART_MAIN);
    if(current_color.full == lv_color_make(0, 255, 0).full) {
        lv_obj_set_style_bg_color(item, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    } else {
        // Si no, se deseleccionan todos y se marca este en verde
        uint32_t child_count = lv_obj_get_child_cnt(product_panel);
        for(uint32_t i = 0; i < child_count; i++){
            lv_obj_t * child = lv_obj_get_child(product_panel, i);
            lv_obj_set_style_bg_color(child, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
        }
        lv_obj_set_style_bg_color(item, lv_color_make(0, 255, 0), LV_PART_MAIN);
    }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////























































// -------------------------
// FUNCIÓN PRINCIPAL (app_main)
// -------------------------
void app_main(void)
{
    init_nvs();
    // Configura la zona horaria para Chile
    setenv("TZ", "CLT3CLST,m10.1.0/0,m3.1.0/0", 1);
    tzset();

    wifi_service_init();
    init_uart();

    simulated_epoch = load_epoch();
    
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
    
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel();
    register_lcd_event_callbacks(panel_handle, &disp_drv);
    init_i2c();
    esp_lcd_touch_handle_t tp = init_touch(panel_handle);
    lv_disp_t *disp = init_lvgl(panel_handle);
    start_lvgl_task(disp, tp);
    

    // Crear tareas para recibir y procesar UART
    //xTaskCreate(uart_RX_task, "uart_RX_task", 4096, NULL, 2, NULL);
    //xTaskCreate(command_processing_task, "command_processing_task", 4096, NULL, 1, NULL);

    // Crear tareas para recibir y procesar UART
    xTaskCreatePinnedToCore(uart_RX_task, "uart_RX_task", 4096, NULL, 2, NULL, 0); // Core 0
    xTaskCreate(command_processing_task, "command_processing_task", 4096, NULL, 1, NULL); // Sin núcleo fijo


    ESP_LOGI(TAG, "Sistema inicializado. Interfaz lista.");
    

    if(lvgl_lock(-1)) {
        lv_obj_clean(lv_scr_act());
        // Crea la estructura principal: header + content
        create_main_structure();
        
        
        // Carga la pantalla principal (o la que prefieras)
        switch_screen(create_main_screen);

        lv_timer_create(update_clock_cb, 1000, NULL); // Crea el timer para actualizar el reloj cada segundo

        
        lvgl_unlock();
    }
}