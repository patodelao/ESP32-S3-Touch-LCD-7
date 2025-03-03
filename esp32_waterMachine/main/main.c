/*
Proyecto de Máquina Expendedora de Agua con ESP32 

    main.c contiene la aplicación principal para la máquina expendedora de agua basada en ESP32.

Este proyecto implementa la lógica central del sistema, que integra varias funcionalidades clave:
   - Gestión de productos: Configuración, almacenamiento (usando NVS) y edición de los productos disponibles,
     con una interfaz gráfica desarrollada en LVGL.
   - Conexión WiFi: Escaneo, conexión y manejo de eventos de red para conectar el dispositivo a una red inalámbrica,
     con soporte para reintentos y notificación visual del estado.
   - Comunicación con el POS: Intercambio de comandos mediante UART, incluyendo el manejo de mensajes con
     verificación LRC para garantizar la integridad de la comunicación.
   - Generación de transacciones: Formateo de montos y generación de tickets (en formato "AAAA/MM/NNNN") para identificar
     de forma única cada operación.
   - Menú de configuración: Interfaz para cambiar la contraseña de configuración y otros parámetros generales.
   - Reloj y hora: Visualización de la hora actual y manejo de eventos para actualizar la hora en tiempo real.

La aplicación se ejecuta sobre el ESP32-S3 utilizando FreeRTOS para la gestión de tareas y sincronización, y aprovecha
los drivers de Espressif para interfaces como UART, I2C, LCD y el controlador táctil (GT911).
Para mayor información, consulta las secciones correspondientes en el resto de la documentación.

 Patricio de la O - Práctica profesional de Ing. Civil Electrónico - UTFSM - TVAL - 2025
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
 
 
 /** @brief Pin de reloj (SCL) para el bus I2C maestro. */
 #define I2C_MASTER_SCL_IO           9
 /** @brief Pin de datos (SDA) para el bus I2C maestro. */
 #define I2C_MASTER_SDA_IO           8
 /** @brief Número del bus I2C. */
 #define I2C_MASTER_NUM              0
 /** @brief Frecuencia del bus I2C en Hz. */
 #define I2C_MASTER_FREQ_HZ          400000
 /** @brief Timeout del bus I2C en milisegundos. */
 #define I2C_MASTER_TIMEOUT_MS       1000
 
 /** @brief Bit que indica conexión WiFi. */
 #define WIFI_CONNECTED_BIT          BIT0
 /** @brief Bit que indica fallo en la conexión WiFi. */
 #define WIFI_FAIL_BIT               BIT1
 /** @brief Bit que indica desconexión WiFi. */
 #define WIFI_DISCONNECTED_BIT       BIT2
 /** @brief Número máximo de redes detectadas. */
 #define MAX_NETWORKS                5
 /** @brief Tamaño por defecto de la lista de redes escaneadas. */
 #define DEFAULT_SCAN_LIST_SIZE      10
 /** @brief Número máximo de reintentos para conexión WiFi. */
 #define MAX_RETRIES                 5
 
 // Configuración del LCD (ajusta los pines según tu panel)
 /** @brief Frecuencia del reloj del píxel del LCD en Hz. */
 #define LCD_PIXEL_CLOCK_HZ          (18 * 1000 * 1000)
 /** @brief Pin de control para la luz de fondo del LCD. */
 #define PIN_NUM_BK_LIGHT            -1
 /** @brief Pin de sincronización horizontal (HSYNC) del LCD. */
 #define PIN_NUM_HSYNC               46
 /** @brief Pin de sincronización vertical (VSYNC) del LCD. */
 #define PIN_NUM_VSYNC               3
 /** @brief Pin de habilitación de datos (DE) del LCD. */
 #define PIN_NUM_DE                  5
 /** @brief Pin de reloj de píxel (PCLK) del LCD. */
 #define PIN_NUM_PCLK                7
 /** @brief Pin de datos 0 del LCD. */
 #define PIN_NUM_DATA0               14
 /** @brief Pin de datos 1 del LCD. */
 #define PIN_NUM_DATA1               38
 /** @brief Pin de datos 2 del LCD. */
 #define PIN_NUM_DATA2               18
 /** @brief Pin de datos 3 del LCD. */
 #define PIN_NUM_DATA3               17
 /** @brief Pin de datos 4 del LCD. */
 #define PIN_NUM_DATA4               10
 /** @brief Pin de datos 5 del LCD. */
 #define PIN_NUM_DATA5               39
 /** @brief Pin de datos 6 del LCD. */
 #define PIN_NUM_DATA6               0
 /** @brief Pin de datos 7 del LCD. */
 #define PIN_NUM_DATA7               45
 /** @brief Pin de datos 8 del LCD. */
 #define PIN_NUM_DATA8               48
 /** @brief Pin de datos 9 del LCD. */
 #define PIN_NUM_DATA9               47
 /** @brief Pin de datos 10 del LCD. */
 #define PIN_NUM_DATA10              21
 /** @brief Pin de datos 11 del LCD. */
 #define PIN_NUM_DATA11              1
 /** @brief Pin de datos 12 del LCD. */
 #define PIN_NUM_DATA12              2
 /** @brief Pin de datos 13 del LCD. */
 #define PIN_NUM_DATA13              42
 /** @brief Pin de datos 14 del LCD. */
 #define PIN_NUM_DATA14              41
 /** @brief Pin de datos 15 del LCD. */
 #define PIN_NUM_DATA15              40
 
 /** @brief Resolución horizontal del LCD. */
 #define LCD_H_RES                   800
 /** @brief Resolución vertical del LCD. */
 #define LCD_V_RES                   480
 
 #if CONFIG_DOUBLE_FB
 /** @brief Número de framebuffers si se usa doble buffer. */
 #define LCD_NUM_FB                  2
 #else
 /** @brief Número de framebuffers en modo simple. */
 #define LCD_NUM_FB                  1
 #endif
 
 /** @brief Período del tick de LVGL en milisegundos. */
 #define LVGL_TICK_PERIOD_MS         2
 /** @brief Delay máximo de la tarea LVGL en milisegundos. */
 #define LVGL_TASK_MAX_DELAY_MS      500
 /** @brief Delay mínimo de la tarea LVGL en milisegundos. */
 #define LVGL_TASK_MIN_DELAY_MS      1
 /** @brief Tamaño de la pila para la tarea LVGL. */
 #define LVGL_TASK_STACK_SIZE        (8 * 1024)
 /** @brief Prioridad de la tarea LVGL. */
 #define LVGL_TASK_PRIORITY          2
 
 /** @brief Número del puerto UART a utilizar. */
 #define UART_NUM                    2
 /** @brief Pin de transmisión (TX) del UART. */
 #define UART_TX_PIN                 43
 /** @brief Pin de recepción (RX) del UART. */
 #define UART_RX_PIN                 44
 /** @brief Tamaño del buffer del UART. */
 #define UART_BUFFER_SIZE            2048
 
 /** @brief Número máximo de reintentos para esperar ACK. */
 #define MAX_RETRIES_ACK             3
 /** @brief Timeout en milisegundos para esperar ACK. */
 #define ACK_TIMEOUT_MS              1000
 /** @brief Carácter de inicio de transmisión (STX). */
 #define STX                         0x02
 /** @brief Carácter de fin de transmisión (ETX). */
 #define ETX                         0x03
 
 /** @brief Número máximo de productos. */
 #define MAX_ITEMS                   10
 
 /** @brief Contraseña por defecto para la configuración general. */
 #define CONFIG_PASSWORD             "root"
 
 
 
 
 
 
 
 
 /*----------------------------------------
  * VARIABLES GLOBALES 
  *----------------------------------------*/
 static const char *TAG = "waterMachine";
 
 
 /** 
  * @defgroup lvgl_ui_vars Variables para la interfaz LVGL y UI general
  * @brief Variables para la interfaz gráfica y estructura general de la aplicación.
  * @{
  */
 static SemaphoreHandle_t lvgl_mux = NULL;           /**<    Mutex para LVGL */
 static lv_obj_t *global_header = NULL;              /**<    Contenedor del header (barra de notificaciones) */
 static lv_obj_t *global_content = NULL;             /**<    Contenedor donde se inyectan las pantallas */
 static lv_obj_t *global_clock_label = NULL;         /**<    Label que muestra la hora */
 static time_t simulated_epoch = 1739984400;         /**<    Epoch simulado (2025-12-31 23:00:00) */
 static lv_obj_t *pwd_change_dialog = NULL;          /**<    Diálogo para cambiar la contraseña */
 /** @} */
 
 /** 
  * @defgroup wifi_vars Variables para la conexión WiFi
  * @brief Variables y estructuras para la gestión de la conexión WiFi.
  * @{
  */
 static lv_obj_t *wifi_status_icon = NULL;           /**< Icono de estado de WiFi */
 static lv_obj_t *floating_msgbox = NULL;            /**< Mensaje flotante */
 static lv_obj_t *success_msgbox = NULL;             /**< Mensaje de éxito */
 static EventGroupHandle_t s_wifi_event_group;       /**< Grupo de eventos WiFi */
 static int s_retry_num = 0;                         /**< Contador de reintentos */
 static lv_obj_t *status_label;                      /**< Etiqueta para mostrar estado de conexión */
 static lv_obj_t *ssid_field;                        /**< Campo de texto para SSID */
 static lv_obj_t *password_field;                    /**< Campo de texto para contraseña WiFi */
 static lv_obj_t *keyboard;                          /**< Teclado virtual */
 static lv_obj_t *ssdropdown;                        /**< Dropdown para seleccionar redes WiFi */
 static TaskHandle_t wifi_task_handle = NULL;        /**< Handle para la tarea de conexión WiFi */
 static wifi_ap_record_t top_networks[MAX_NETWORKS]; /**< Arreglo de redes detectadas */
 static bool wifi_manual_disconnect = true;          /**< Indica si la desconexión fue manual */
 static bool sntp_initialized = false;               /**< Indica si SNTP ya fue inicializado */
 static char new_dropdown_options[256] = "";         /**< Buffer para nuevas opciones del dropdown */
 static volatile bool update_dropdown_flag = false;  /**< Flag para actualizar el dropdown */
 static bool wifi_initialized = false;               /**< Indica si el WiFi ya está inicializado */
 /** @} */
 
 /** 
  * @defgroup uart_vars Variables para la comunicación UART}
  * @brief Variables y estructuras para la comunicación UART.
  * @{
  */
 static QueueHandle_t uart_event_queue = NULL;       /**< Cola para eventos de UART */
 static QueueHandle_t ack_queue = NULL;              /**< Cola para ACK/NAK */
 static QueueHandle_t command_queue = NULL;          /**< Cola para comandos recibidos */
 typedef struct {
     size_t length;                                 /**< Longitud del mensaje UART */
     uint8_t data[UART_BUFFER_SIZE];                /**< Datos del mensaje UART */
 } uart_message_t;                                   /**< Estructura para mensajes de UART */
 /** @} */
 
 /** 
  * @defgroup config_trans_vars Variables para la configuración y transacciones
  * @brief Variables para la configuración y transacciones.
  * @{
  */
 static lv_obj_t *config_password_dialog = NULL;     /**< Diálogo para cambiar la contraseña de configuración */
 static unsigned int ticket_counter = 0;             /**< Contador global para tickets */
 /** @} */
 
 /** 
  * @defgroup product_vars Variables para la gestión de productos
  * @brief Variables y estructuras para la gestión de productos.
  * @{
  */
 static uint32_t cont_index = 0;                     /**< Índice de productos actuales */
 lv_obj_t *cont_arr[MAX_ITEMS] = {0};                /**< Arreglo de productos */
 static lv_obj_t *menu;                              /**< Menú de productos */
 static lv_obj_t *main_page;                         /**< Página principal del menú de productos */
 static lv_obj_t *float_btn_add;                     /**< Botón flotante para agregar producto */
 static lv_obj_t *float_btn_del;                     /**< Botón flotante para eliminar el último producto */
 static lv_obj_t *float_btn_del_all;                 /**< Botón flotante para eliminar todos los productos */
 static lv_obj_t *save_btn;                          /**< Botón para guardar cambios */
 static lv_obj_t *main_menu_page = NULL;             /**< Página principal global del menú de productos */
 static lv_obj_t *products_footer = NULL;            /**< Contenedor para los botones del footer en productos */
 /** @} */
 
 
 // ----------------------------------------
 // PROTOTIPOS DE FUNCIONES
 // ----------------------------------------
 
 
 // Funciones generales y de comunicación UART / utilidades
 int process_uart_data(const uint8_t *data, size_t length);
 void uart_send_command(const uint8_t *command, size_t length);
 static void process_received_command(const uint8_t *command, size_t length);
 static bool verify_lrc_rx(const uint8_t *data, size_t length);
 uint8_t calculate_lrc(const uint8_t *data, size_t length);
 void create_transaction_command(const char *monto);
 void generate_ticket_number(char *ticket, size_t size);
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
 void create_general_config_screen(lv_obj_t *parent);
 void create_main_screen(lv_obj_t *parent);
 void switch_screen(void (*create_screen)(lv_obj_t *parent));
 static bool load_config_password_from_nvs(char *buffer, size_t size);
 
 // Funciones adicionales de transacciones y UI de compra
 void generate_transaction_from_selected_product(lv_obj_t *exhibitor_panel);
 static void buy_button_event_cb(lv_event_t *e);
 static void menu_page_changed_event_cb(lv_event_t *e);
 static void product_item_event_cb(lv_event_t *e);
 
 // Funciones para el manejo de Wi‑Fi
 static void set_wifi_icon_connected(void *arg);
 static void set_wifi_icon_disconnected(void *arg);
 static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
 static void wifi_connect_task(void *param);
 static void update_dropdown_options(void *param);
 static void wifi_scan_task(void *param);
 static void remove_floating_msgbox(lv_event_t *event);
 static void scan_button_event_handler(lv_event_t *event);
 static void connect_button_event_handler(lv_event_t *event);
 static void disconnect_button_event_handler(lv_event_t *event);
 
 // Funciones para el manejo del teclado en LVGL
 static void keyboard_event_handler(lv_event_t *event);
 static void textarea_event_handler(lv_event_t *event);
 
 // Funciones de tareas y manejo de colas UART
 static void uart_RX_task(void *param);
 static void command_processing_task(void *param);
 static void wait_for_ack_task(void *param);
 void send_command_with_ack(const uint8_t *command, size_t length);
 
 // Funciones para la configuración de Wi‑Fi y su UI
 void wifi_service_init(void);
 void wifi_init_sta(const char *ssid, const char *password);
 static void connect_event_handler(lv_event_t *event);
 void create_wifi_settings_widget(lv_obj_t *parent);
 
 // Funciones de configuración de contraseña (password config)
 static void confirm_password_event_cb(lv_event_t *e);
 static void close_config_password_dialog_cb(lv_event_t *e);
 static void show_config_password_dialog(void);
 static void btn_to_config_event_cb(lv_event_t *e);
 static void save_config_password_to_nvs(const char *password);
 static void password_change_confirm_cb(lv_event_t *e);
 static void create_password_change_menu(lv_obj_t *parent);
 
 // Funciones para comandos de POS
 static void send_init_command(lv_event_t *e);
 static void send_init_response(lv_event_t *e);
 static void send_polling_command(lv_event_t *e);
 static void send_loadkeys_comand(lv_event_t *e);
 static void create_operation_cmd_pos_tab(lv_obj_t *parent);
 
 // Funciones para manejo de epoch y estructura general
 void create_main_structure(void);
 void save_epoch(time_t epoch);
 time_t load_epoch(void);
 static void update_clock_cb(lv_timer_t *timer);
 
 
 
 
 
 ////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////// IMPLEMENTACIONES ///////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////////
 
 
 // ---------------------------------------------------------
 // Funciones de Utilidades y Comunicación UART
 // ---------------------------------------------------------
 
 /**
  * @brief Procesa los datos recibidos por UART.
  *
  * Esta función analiza el buffer recibido para detectar un comando completo,
  * utilizando como indicadores el carácter de inicio (STX) y el de fin (ETX). Además,
  * se verifica la integridad del mensaje mediante la comparación del LRC calculado
  * con el recibido. Se asume que los caracteres ACK (0x06) y NAK (0x15) se reciben
  * de forma aislada.
  *
  * @param data Puntero al buffer de datos recibidos.
  * @param length Longitud de los datos recibidos.
  * @return int Código de estado:
  *         - 1: Comando procesado correctamente.
  *         - 0: Error en el LRC o comando incompleto.
  *         - 2: Se recibió un ACK aislado.
  *         - 3: Se recibió un NAK aislado.
  *         - 9: Buffer lleno (comando inválido).
  *         - 10: LRC fuera de rango.
  */
 int process_uart_data(const uint8_t *data, size_t length) {
     uint8_t temp_message[UART_BUFFER_SIZE] = {0};   // Buffer temporal para acumular el mensaje.
     size_t temp_index = 0;                          // Índice para el buffer temporal.
     char command[5] = {0};                          // Variable para posibles parseos dependiendo del tipo de respuesta. (actualmente no utilizada)
 
     // Se asume que se reciben ACK/NAK de forma aislada:
     for (size_t i = 0; i < length; i++) {
         if (data[i] == 0x06) 
             return 2; // Se recibió ACK aislado. (solo para pruebas)
         else if (data[i] == 0x15) 
             return 3; // Se recibió NAK aislado. (solo para pruebas)
         
         // Al detectar el carácter de inicio (STX), reiniciamos el buffer.
         if (data[i] == STX) {
             temp_index = 0;
         }
         // Almacena el byte actual en el buffer temporal (incluye STX y, eventualmente, ETX).
         if (temp_index < sizeof(temp_message) - 1) {
             temp_message[temp_index++] = data[i];
         } else {
             return 9; // Se ha llenado el buffer: comando inválido.
         }
         // Cuando se detecta ETX, se espera que el siguiente byte sea el LRC.
         if (data[i] == ETX) {
             if (i + 1 >= length) 
                 return 10; // No se encuentra el LRC: fuera de rango.
             // Calcular el LRC sobre el mensaje acumulado (incluyendo STX y ETX).
             uint8_t lrc_calculated = calculate_lrc(temp_message, temp_index);
             uint8_t lrc_received = data[i + 1];
 
             // Si el LRC calculado no coincide con el recibido, se retorna error.
             if (lrc_calculated != lrc_received) 
                 return 0; // Error en LRC.
             // Se añade un carácter nulo al final del mensaje, útil para parseos futuros.
             temp_message[temp_index] = '\0';
             // Aquí se podrían realizar parseos adicionales, por ejemplo, utilizando strtok.
             return 1; // Comando procesado correctamente.
         }
     }
     return 0; // No se procesó ningún comando completo.
 }
 
 /**
  * @brief Envía un comando a través de UART.
  *
  * Esta función envía el bloque de datos especificado usando la función 
  * uart_write_bytes del driver de UART.
  *
  * @param command Puntero al buffer que contiene el comando a enviar.
  * @param length Longitud del comando.
  */
 void uart_send_command(const uint8_t *command, size_t length) {
     uart_write_bytes(UART_NUM, (const char *)command, length);
 }
 
 /**
  * @brief Procesa un comando recibido y envía una respuesta ACK o NAK.
  *
  * Esta función verifica la integridad del comando recibido mediante LRC.
  * Si el comando es válido, se envía un ACK; en caso contrario, se envía un NAK.
  *
  * @param command Puntero al comando recibido.
  * @param length Longitud del comando.
  */
 static void process_received_command(const uint8_t *command, size_t length) {
     if (verify_lrc_rx(command, length)) {
         ESP_LOGI(TAG, "Comando válido. Enviando ACK.");
         uint8_t ack = 0x06; // Código ACK
         uart_send_command(&ack, 1);
     } else {
         ESP_LOGE(TAG, "Comando inválido. Enviando NAK.");
         uint8_t nak = 0x15; // Código NAK
         uart_send_command(&nak, 1);
     }
 }
 
 /**
  * @brief Verifica la integridad de un mensaje recibido usando LRC.
  *
  * Esta función busca el carácter ETX para determinar la longitud del mensaje
  * completo (incluyendo ETX y LRC) y calcula el LRC de los datos entre STX y ETX.
  * Luego, compara el LRC calculado con el valor LRC recibido.
  *
  * @param data Puntero al mensaje recibido.
  * @param length Longitud total del mensaje recibido.
  * @return true si el LRC es correcto, false en caso contrario.
  */
 static bool verify_lrc_rx(const uint8_t *data, size_t length) {
     if (length < 4) 
         return false; // El mensaje debe tener al menos STX, un dato, ETX y LRC.
 
     ESP_LOGI(TAG, "Mensaje recibido para LRC:");
     for (size_t i = 0; i < length; i++) {
         printf("%02X ", data[i]);
     }
     printf("\n");
 
     // Buscar el carácter ETX para determinar la longitud válida del mensaje.
     size_t valid_length = 0;
     for (size_t i = 1; i < length; i++) {
         if (data[i] == ETX) {
             valid_length = i + 2; // Incluye ETX y el byte LRC que le sigue.
             break;
         }
     }
 
     if (valid_length == 0 || valid_length > length) {
         ESP_LOGE(TAG, "No se encontró ETX en la longitud esperada.");
         return false;
     }
 
     // Calcular el LRC sobre los datos comprendidos entre STX y ETX (excluyendo ambos).
     uint8_t calculated_lrc = calculate_lrc(data + 1, valid_length - 3); 
     uint8_t received_lrc = data[valid_length - 1];
 
     ESP_LOGI(TAG, "LRC Calculado: %02X, LRC Recibido: %02X", calculated_lrc, received_lrc);
 
     return calculated_lrc == received_lrc;
 }
 
 /**
  * @brief Calcula el LRC (Longitud de Redundancia Cruzada) de un bloque de datos.
  *
  * El LRC se calcula realizando una operación XOR sucesiva de cada byte del
  * bloque de datos.
  *
  * @param data Puntero al bloque de datos.
  * @param length Longitud del bloque de datos.
  * @return uint8_t Valor del LRC calculado.
  */
 uint8_t calculate_lrc(const uint8_t *data, size_t length) {
     uint8_t lrc = 0;
     for (size_t i = 0; i < length; i++) {
         lrc ^= data[i];
     }
     return lrc;
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 // ---------------------------------------------------------
 // Tareas y funciones de manejo de colas y ACK
 // ---------------------------------------------------------
 
 /**
  * @brief Tarea para la recepción de datos por UART.
  *
  * Esta tarea se ejecuta en un bucle infinito. Lee bytes individualmente desde el
  * UART y los almacena en un buffer temporal. Cuando se detecta el carácter ETX,
  * intenta leer el siguiente byte (que se supone es el LRC), y luego procesa el
  * comando completo mediante la función process_uart_data. Dependiendo del resultado,
  * envía un ACK (0x06) o un NAK (0x15).
  *
  * Nota: Se reinicia el índice del buffer si se detecta STX o si se supera el tamaño del buffer.
  *
  * @param param Parámetro de tarea (no utilizado).
  */
 static void uart_RX_task(void *param) {
     static uint8_t rx_buffer[UART_BUFFER_SIZE];  // Buffer para almacenar los bytes recibidos.
     static size_t rx_index = 0;                  // Índice actual del buffer.
     uint8_t byte;
     while (1) {
         if (uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(10)) > 0) {
 
             // (Opcional) Eco inmediato del byte recibido:
             // uart_write_bytes(UART_NUM, &byte, 1);
 
             // Al detectar el carácter de inicio (STX), reinicia el buffer.
             if (byte == STX) {
                 rx_index = 0;
             }
             // Almacena el byte recibido en el buffer, siempre y cuando no se exceda el tamaño.
             if (rx_index < UART_BUFFER_SIZE) {
                 rx_buffer[rx_index++] = byte;
             }
             // Al detectar ETX, se espera que el siguiente byte sea el LRC.
             if (byte == ETX && rx_index < UART_BUFFER_SIZE) {
                 // Intenta leer el LRC.
                 if (uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(10)) > 0) {
                     rx_buffer[rx_index++] = byte; // Agrega el LRC al buffer.
                 }
                 // Procesa el comando completo.
                 int result = process_uart_data(rx_buffer, rx_index);
                 if (result == 1) {
                     uint8_t ack = 0x06;
                     uart_send_command(&ack, 1);
                 } else {
                     uint8_t nak = 0x15;
                     uart_send_command(&nak, 1);
                 }
                 // Reinicia el índice del buffer para procesar el siguiente comando.
                 rx_index = 0;
             }
             // Previene desbordamientos del buffer.
             if (rx_index >= UART_BUFFER_SIZE) {
                 rx_index = 0;
             }
         }
     }
 }
 
 /**
  * @brief Tarea para procesar los comandos recibidos a través de UART.
  *
  * Esta tarea extrae mensajes completos desde la cola 'command_queue', los muestra
  * (en formato hexadecimal) y luego llama a process_received_command para procesarlos.
  *
  * @param param Parámetro de tarea (no utilizado).
  */
 static void command_processing_task(void *param) {
     uart_message_t msg;
     // Bloquea hasta que se reciba un mensaje en la cola.
     while (xQueueReceive(command_queue, &msg, portMAX_DELAY)) {
         printf("Procesando comando (longitud: %d): ", msg.length);
         for (size_t i = 0; i < msg.length; i++) {
             printf("%02X ", msg.data[i]);
         }
         printf("\n");
         // Procesa el comando y envía la respuesta correspondiente (ACK/NAK).
         process_received_command(msg.data, msg.length);
     }
 }
 
 /**
  * @brief Tarea para esperar y verificar el ACK de un comando enviado por UART.
  *
  * Esta tarea envía un comando (almacenado en params) y espera, con varios reintentos,
  * a recibir un ACK en la cola 'ack_queue'. Si se recibe ACK, la tarea finaliza; de lo
  * contrario, se limpian las colas para evitar bloqueos y se elimina la tarea.
  *
  * @param param Puntero a una estructura CommandParams que contiene el comando y su longitud.
  */
 static void wait_for_ack_task(void *param) {
     // Definición local de la estructura de parámetros para el comando.
     struct CommandParams {
         uint8_t *command;
         size_t length;
     } *params = (struct CommandParams *)param;
 
     uint8_t received_byte;
     bool ack_received = false;
 
     // Intenta enviar el comando y esperar ACK hasta MAX_RETRIES_ACK veces.
     for (int attempt = 1; attempt <= MAX_RETRIES_ACK; attempt++) {
         ESP_LOGI(TAG, "Intento %d: Enviando comando con ACK", attempt);
         uart_write_bytes(UART_NUM, (const char *)params->command, params->length);
 
         // Espera a recibir una respuesta en la cola de ACK, con un timeout definido.
         if (xQueueReceive(ack_queue, &received_byte, pdMS_TO_TICKS(ACK_TIMEOUT_MS))) {
             if (received_byte == 0x06) { // Se recibió ACK.
                 ESP_LOGI(TAG, "ACK recibido.");
                 ack_received = true;
                 break;
             } else if (received_byte == 0x15) { // Se recibió NAK.
                 ESP_LOGW(TAG, "NAK recibido, reintentando...");
             }
         } else {
             ESP_LOGW(TAG, "Timeout esperando ACK.");
         }
     }
 
     // Libera la memoria asignada para el comando.
     free(params->command);
     free(params);
 
     // Si no se recibió ACK, se reinicia la cola de ACK para evitar bloqueos.
     if (!ack_received) {
         xQueueReset(ack_queue);
     }
 
     // Finaliza la tarea.
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Envía un comando a través de UART y espera su confirmación (ACK).
  *
  * Esta función encapsula el comando y su longitud en una estructura dinámica y 
  * lanza la tarea wait_for_ack_task, la cual se encarga de enviar el comando y esperar
  * la respuesta de confirmación.
  *
  * @param command Puntero al buffer que contiene el comando a enviar.
  * @param length Longitud del comando.
  */
 void send_command_with_ack(const uint8_t *command, size_t length) {
     // Definición de la estructura para encapsular el comando.
     struct CommandParams {
         uint8_t *command;
         size_t length;
     } *params = malloc(sizeof(struct CommandParams));
 
     // Asigna memoria y copia el comando.
     params->command = malloc(length);
     memcpy(params->command, command, length);
     params->length = length;
 
     // Crea la tarea que se encargará de enviar el comando y esperar el ACK.
     xTaskCreate(wait_for_ack_task, "wait_for_ack_task", 4096, params, 2, NULL);
 }
 
 
 
 
 
 
 
 
 
 
 // ------------------------------------------------------------------
 // Funciones de transacciones y generación de números de Tickets
 // ------------------------------------------------------------------
 
 /**
  * @brief Crea y envía un comando de transacción basado en el monto recibido.
  *
  * La función formatea el monto recibido en un formato fijo, genera dinámicamente un
  * número de ticket (formateado como "AAAA/MM/NNNN") y construye un comando compuesto
  * por varios campos separados por el carácter '|' (incluyendo un código de comando, el
  * monto formateado, el número de ticket, etc.). Luego, agrega los caracteres de inicio
  * (STX) y fin (ETX), calcula el LRC (Longitud de Redundancia) sobre el mensaje y lo
  * envía a través de la función send_command_with_ack. Si falla la asignación de
  * memoria para el comando, se registra un error.
  *
  * @param monto Cadena que representa el monto de la transacción (ej. "123").
  */
 void create_transaction_command(const char *monto) {
     // Inicializa un buffer con ceros para el monto formateado (9 dígitos).
     char monto_formateado[10] = "000000000";
     int len_monto = strlen(monto);
     // Copia el monto recibido en la parte derecha del buffer, alineándolo a la derecha.
     memmove(monto_formateado + (9 - len_monto), monto, len_monto);
 
     // Código de comando predefinido para la transacción.
     const char *codigo_cmd = "0200";
 
     // Genera el número de ticket dinámicamente, asegurando el formato "AAAA/MM/NNNN".
     char ticket_number[16];  // Espacio suficiente para el formato de ticket.
     generate_ticket_number(ticket_number, sizeof(ticket_number));
     
     // Valores fijos para otros campos del comando.
     const char *campo_impresion = "1";
     const char *enviar_msj = "1";
 
     // Construye el comando completo separando los campos con el carácter '|'.
     char command[256];
     snprintf(command, sizeof(command), "%s|%s|%s|%s|%s", 
              codigo_cmd, monto_formateado, ticket_number, campo_impresion, enviar_msj);
 
     // Calcula la longitud total del comando, sumando STX, DATA, ETX y LRC.
     size_t command_length = strlen(command) + 3;
     uint8_t *formatted_command = malloc(command_length);
     if (!formatted_command) {
         ESP_LOGE(TAG, "Error: No se pudo asignar memoria para el comando.");
         return;
     }
 
     size_t index = 0;
     // Inserta el carácter de inicio (STX).
     formatted_command[index++] = STX;
     // Copia el comando formateado.
     memcpy(&formatted_command[index], command, strlen(command));
     index += strlen(command);
     // Inserta el carácter de fin (ETX).
     formatted_command[index++] = ETX;
 
     // Calcula el LRC sobre el contenido entre STX y ETX.
     uint8_t lrc = calculate_lrc(formatted_command + 1, index - 1);
     // Agrega el LRC al final del mensaje.
     formatted_command[index++] = lrc;
 
     // Registra en el log el comando final (en formato hexadecimal).
     ESP_LOGI(TAG, "Mensaje Enviado:");
     for (size_t i = 0; i < index; i++) {
         printf("%02X ", formatted_command[i]);
     }
     printf("\n");
 
     // Envía el comando a través de la función que espera la confirmación (ACK).
     send_command_with_ack(formatted_command, index);
     free(formatted_command);
 }
 
 /**
  * @brief Genera un número de ticket en el formato "AAAA/MM/NNNN".
  *
  * La función utiliza el valor del epoch simulado para obtener la fecha actual y
  * formatea el año y mes, junto con un contador global que se incrementa en cada
  * llamada, garantizando que el número de ticket tenga 4 dígitos. Este formato es
  * útil para identificar transacciones de forma única.
  *
  * @param ticket Buffer de salida donde se almacenará el número de ticket formateado.
  * @param size Tamaño del buffer de salida.
  */
 void generate_ticket_number(char *ticket, size_t size) {
     // Obtiene el tiempo actual basado en el epoch simulado.
     time_t now = simulated_epoch;
     struct tm timeinfo;
     localtime_r(&now, &timeinfo);
 
     int year = timeinfo.tm_year + 1900;
     int month = timeinfo.tm_mon + 1;
 
     // Incrementa el contador global y lo limita a 4 dígitos.
     ticket_counter++;
     unsigned int counter = ticket_counter % 10000; // Reinicia si llega a 9999.
 
     // Formatea el ticket con el año (4 dígitos), el mes (2 dígitos) y el contador (4 dígitos).
     snprintf(ticket, size, "%04d/%02d/%04u", year, month, counter);
 }
 
 /**
  * @brief Extrae el monto del producto seleccionado en el panel exhibidor y genera el comando de transacción.
  *
  * La función recorre cada elemento hijo del panel exhibidor. Si encuentra un elemento
  * cuya propiedad de color de fondo sea verde (indicando selección), extrae el texto del
  * precio, elimina el símbolo '$' si está presente y llama a create_transaction_command
  * con el monto numérico. Si no se encuentra ningún producto seleccionado, se registra una advertencia.
  *
  * @param exhibitor_panel Objeto LVGL que contiene la lista de productos.
  */
 void generate_transaction_from_selected_product(lv_obj_t *exhibitor_panel) {
     uint32_t child_count = lv_obj_get_child_cnt(exhibitor_panel);
     for (uint32_t i = 0; i < child_count; i++) {
         lv_obj_t *item = lv_obj_get_child(exhibitor_panel, i);
         // Obtiene el color de fondo actual del elemento.
         lv_color_t current_color = lv_obj_get_style_bg_color(item, LV_PART_MAIN);
 
         // Verifica si el producto está seleccionado (color verde).
         if (current_color.full == lv_color_make(0, 255, 0).full) {
             // Obtiene el objeto que contiene el precio.
             lv_obj_t *price_label = lv_obj_get_child(item, 1);
             if (price_label != NULL) {
                 const char *price_text = lv_label_get_text(price_label);
                 // Si el precio comienza con '$', se omite dicho símbolo.
                 if (price_text[0] == '$') {
                     price_text++;
                 }
                 // Crea y envía el comando de transacción basado en el monto extraído.
                 create_transaction_command(price_text);
                 ESP_LOGI("TRANSACTION", "Comando de transacción generado para el monto: %s", price_text);
                 return;
             }
         }
     }
     ESP_LOGW("TRANSACTION", "No se seleccionó ningún producto.");
 }
 
 
 /**
  * @brief Callback del botón de compra.
  *
  * Esta función se activa cuando se presiona el botón de compra en la interfaz.
  * Recupera el panel exhibidor de productos (pasado como user_data), llama a
  * generate_transaction_from_selected_product para procesar la transacción y luego
  * recorre todos los elementos del panel para deseleccionar (cambiar a azul) aquellos
  * que estaban seleccionados (marcados en verde).
  *
  * @param e Evento LVGL asociado al botón de compra.
  */
 static void buy_button_event_cb(lv_event_t *e) {
     // Obtiene el panel exhibidor de productos del user_data.
     lv_obj_t *exhibitor_panel = lv_event_get_user_data(e);
     // Genera el comando de transacción para el producto seleccionado.
     generate_transaction_from_selected_product(exhibitor_panel);
 
     // Recorre todos los elementos del panel y deselecciona los que estén marcados (en verde).
     uint32_t child_count = lv_obj_get_child_cnt(exhibitor_panel);
     for (uint32_t i = 0; i < child_count; i++) {
          lv_obj_t *item = lv_obj_get_child(exhibitor_panel, i);
          lv_color_t current_color = lv_obj_get_style_bg_color(item, LV_PART_MAIN);
          // Si el elemento está seleccionado (color verde), envía un evento para deseleccionarlo.
          if (current_color.full == lv_color_make(0, 255, 0).full) {
              lv_event_send(item, LV_EVENT_CLICKED, NULL);
          }
     }
 }
 
 
 
 
 
 
 
 
 // ---------------------------------------------------------
 // Funciones de Inicialización de Hardware y LVGL
 // ---------------------------------------------------------
 
 /**
  * @brief Inicializa la memoria no volátil (NVS).
  *
  * Esta función inicializa la flash NVS, necesaria para almacenar datos persistentes
  * como configuraciones o valores de tiempo. En caso de que se detecte que la NVS no tiene
  * páginas libres o se ha actualizado a una versión nueva, se borra la NVS y se reinicializa.
  *
  * Se finaliza con una verificación del resultado y se registra un mensaje informativo.
  */
 void init_nvs(void) {
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     ESP_LOGI(TAG, "NVS initialized successfully");
 }
 
 
 /**
  * @brief Inicializa la comunicación UART.
  *
  * Configura el controlador UART utilizando parámetros predefinidos, tales como la tasa
  * de baudios, bits de datos, paridad y bits de parada. Además, se asignan los pines de TX
  * y RX, se instala el driver UART y se crean las colas para la recepción de ACK/NAK y comandos.
  *
  * En caso de error en la creación de las colas, se imprime un mensaje de error.
  */
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
 
     // Crear colas para recibir ACK/NAK y comandos UART.
     ack_queue = xQueueCreate(5, sizeof(uint8_t)); 
     command_queue = xQueueCreate(10, sizeof(uart_message_t));
 
     if (!command_queue || !ack_queue) {
         printf("Error: No se pudo crear las colas.\n");
     }
 }
 
 
 /**
  * @brief Inicializa el panel LCD RGB.
  *
  * Configura y crea un controlador para el panel LCD RGB utilizando parámetros específicos
  * como el ancho de datos, alineación para la transferencia en PSRAM, número de framebuffers,
  * fuente de reloj, pines de control y datos, y parámetros de temporización para la sincronización.
  *
  * La función reinicia e inicializa el panel, y registra un mensaje informativo al finalizar.
  *
  * @return esp_lcd_panel_handle_t Manejador del panel LCD inicializado.
  */
 esp_lcd_panel_handle_t init_lcd_panel(void) {
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
 
 /**
  * @brief Inicializa el bus I2C en modo maestro.
  *
  * Configura el bus I2C con los parámetros especificados, como pines de datos (SDA) y reloj (SCL),
  * habilitación de resistencias pull-up, y la velocidad del reloj del maestro. Luego, instala el
  * driver I2C en el puerto designado.
  *
  * @return esp_err_t Resultado de la instalación del driver I2C.
  */
 static esp_err_t i2c_master_init(void) {
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
 
 /**
  * @brief Inicializa el bus I2C y registra cualquier dispositivo adicional si es necesario.
  *
  * Llama a la función i2c_master_init() para configurar el bus I2C y, posteriormente, se pueden
  * agregar inicializaciones para dispositivos I2C adicionales según los requerimientos del proyecto.
  */
 void init_i2c(void) {
     ESP_LOGI(TAG, "Initializing I2C");
     ESP_ERROR_CHECK(i2c_master_init());
     ESP_LOGI(TAG, "I2C initialized successfully");
     // Se pueden inicializar otros dispositivos I2C aquí, si es necesario.
 }
 
 /**
  * @brief Inicializa el controlador de pantalla táctil (GT911) para el panel LCD.
  *
  * Configura la interfaz I2C para comunicarse con el controlador táctil GT911 y crea
  * el manejador del touch mediante la función esp_lcd_touch_new_i2c_gt911(). Se definen
  * parámetros como las coordenadas máximas y los pines de reset e interrupción.
  *
  * @param panel_handle Manejador del panel LCD, que se utiliza para configurar la comunicación.
  * @return esp_lcd_touch_handle_t Manejador del controlador táctil inicializado.
  */
 esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle) {
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
 
 /**
  * @brief Inicializa el sistema gráfico LVGL.
  *
  * Inicializa la librería LVGL y configura el controlador de visualización. Se reserva
  * un buffer de dibujo en la memoria PSRAM y se establece la resolución horizontal y vertical
  * del display. Se asigna una función de "flush" para volcar el contenido del buffer al panel LCD.
  *
  * @param panel_handle Manejador del panel LCD que se utiliza para el "flush".
  * @return lv_disp_t* Manejador del display LVGL.
  */
 lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle) {
     ESP_LOGI(TAG, "Initializing LVGL");
     lv_init();
     static lv_disp_draw_buf_t disp_buf;
     static lv_disp_drv_t disp_drv;
     // Reserva un buffer de dibujo en la memoria PSRAM.
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
 
 /**
  * @brief Registra los callbacks de eventos para el panel LCD.
  *
  * Permite la extensión de funcionalidades al registrar funciones callback para eventos
  * específicos del panel LCD, como la sincronización vertical (vsync). Actualmente, la función
  * define el callback 'on_vsync' como NULL, pero se puede modificar para incluir otros eventos.
  *
  * @param panel_handle Manejador del panel LCD.
  * @param disp_drv Puntero a la estructura de controlador de display LVGL.
  */
 void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv) {
     ESP_LOGI(TAG, "Register event callbacks");
     esp_lcd_rgb_panel_event_callbacks_t cbs = {
          .on_vsync = NULL, // Callback para vsync; se puede implementar si es necesario.
     };
     ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, disp_drv));
 }
 
 
 
 
 
 
 
 
 
 // ---------------------------------------------------------
 // Funciones relacionadas con el tick y port de LVGL
 // ---------------------------------------------------------
 
 /**
  * @brief Incrementa el contador de ticks de LVGL.
  *
  * Esta función se utiliza como callback de un temporizador (timer) que se ejecuta de forma periódica.
  * Cada vez que se invoca, incrementa el contador interno de LVGL en un período definido por LVGL_TICK_PERIOD_MS.
  *
  * @param arg Puntero a argumentos (no utilizado en este caso).
  */
 static void increase_lvgl_tick(void *arg) {
     lv_tick_inc(LVGL_TICK_PERIOD_MS);
 }
 
 /**
  * @brief Callback de "flush" para LVGL.
  *
  * Esta función es llamada por LVGL para volcar (flush) el contenido del buffer de dibujo al panel LCD.
  * Utiliza la función esp_lcd_panel_draw_bitmap() para enviar la región definida (area) al hardware LCD.
  * Una vez completado el volcado, se llama a lv_disp_flush_ready() para notificar a LVGL.
  *
  * @param drv Puntero al controlador de display LVGL.
  * @param area Área de la pantalla que se debe actualizar.
  * @param color_map Mapa de colores con los datos a dibujar.
  */
 static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
     esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
     int offsetx1 = area->x1;
     int offsety1 = area->y1;
     int offsetx2 = area->x2;
     int offsety2 = area->y2;
     esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
     lv_disp_flush_ready(drv);
 }
 
 /**
  * @brief Callback para leer datos del dispositivo táctil (touch) en LVGL.
  *
  * Esta función se emplea en el driver de entrada (input device driver) de LVGL para dispositivos
  * táctiles. Se leen las coordenadas actuales del touch mediante las funciones de la API del controlador
  * táctil. Si se detecta que la pantalla está siendo presionada (y se han leído coordenadas válidas),
  * se actualiza la estructura lv_indev_data con el punto y el estado PRESSED; de lo contrario, se marca
  * el estado como RELEASED.
  *
  * @param drv Puntero al driver de entrada LVGL.
  * @param data Estructura de datos que se actualizará con la información del touch.
  */
 static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
     uint16_t touchpad_x[1] = {0};
     uint16_t touchpad_y[1] = {0};
     uint8_t touchpad_cnt = 0;
     
     // Lee los datos del controlador táctil usando el user_data (generalmente el handle del touch)
     esp_lcd_touch_read_data(drv->user_data);
     
     // Obtiene las coordenadas y determina si la pantalla está presionada
     bool pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
     if (pressed && touchpad_cnt > 0) {
         data->point.x = touchpad_x[0];
         data->point.y = touchpad_y[0];
         data->state = LV_INDEV_STATE_PR;
     } else {
         data->state = LV_INDEV_STATE_REL;
     }
 }
 
 /**
  * @brief Tarea principal para el procesamiento de LVGL.
  *
  * Esta función se ejecuta en un bucle infinito y se encarga de llamar a lv_timer_handler()
  * para gestionar las actualizaciones de LVGL (timers, animaciones, etc.). El tiempo de
  * espera entre llamadas se ajusta dinámicamente en función de lo que retorne lv_timer_handler(),
  * garantizando que el sistema responda con la periodicidad adecuada.
  *
  * @param arg Puntero a datos (en este caso, se espera el display, aunque no se utiliza directamente).
  */
 static void lvgl_port_task(void *arg) {
     ESP_LOGI(TAG, "Starting LVGL task");
     uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
     while (1) {
         if (lvgl_lock(-1)) {
             // Ejecuta el manejador de LVGL y obtiene el delay sugerido
             task_delay_ms = lv_timer_handler();
             lvgl_unlock();
         }
         // Se asegura de que el delay se encuentre dentro de los límites establecidos
         if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
             task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
         else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
             task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
         vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
     }
 }
 
 /**
  * @brief Inicia la tarea de LVGL y configura su temporizador y entrada táctil.
  *
  * Esta función configura el sistema LVGL, creando el temporizador que incrementa el tick
  * de LVGL, registrando el driver de entrada táctil y creando la tarea que procesa los timers
  * de LVGL. Se asigna la tarea a un core específico (en este caso, core 1) para mejorar el
  * rendimiento de la interfaz gráfica.
  *
  * @param disp Puntero al display LVGL configurado.
  * @param tp Manejador del controlador táctil.
  */
 void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp) {
     ESP_LOGI(TAG, "Install LVGL tick timer");
     // Configuración de argumentos para el temporizador que incrementa el tick de LVGL.
     const esp_timer_create_args_t lvgl_tick_timer_args = {
          .callback = &increase_lvgl_tick,
          .name = "lvgl_tick"
     };
     
     // Inicializa y registra el driver de entrada para el touch
     static lv_indev_drv_t indev_drv;
     lv_indev_drv_init(&indev_drv);
     indev_drv.type = LV_INDEV_TYPE_POINTER;
     indev_drv.disp = disp;
     indev_drv.read_cb = lvgl_touch_cb;
     indev_drv.user_data = tp;
     lv_indev_drv_register(&indev_drv);
     
     // Crea y arranca el temporizador que actualizará el tick de LVGL de forma periódica.
     esp_timer_handle_t lvgl_tick_timer = NULL;
     ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
     ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
     
     // Crea el mutex recursivo para sincronizar el acceso a las APIs de LVGL
     lvgl_mux = xSemaphoreCreateRecursiveMutex();
     assert(lvgl_mux);
     
     // Crea la tarea de LVGL y la fija en el core 1, para no interferir con otras tareas críticas.
     xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, disp, LVGL_TASK_PRIORITY, NULL, 1);
 }
 
 /**
  * @brief Bloquea el acceso a las funciones de LVGL.
  *
  * Debido a que las APIs de LVGL no son thread-safe, esta función utiliza un mutex
  * recursivo para bloquear el acceso a LVGL durante operaciones críticas. Se puede especificar
  * un tiempo de espera (en milisegundos) para la adquisición del mutex.
  *
  * @param timeout_ms Tiempo en milisegundos para esperar la adquisición del mutex; 
  *                   si es -1 se espera indefinidamente.
  * @return true Si el mutex se adquiere correctamente.
  * @return false Si no se pudo adquirir el mutex en el tiempo especificado.
  */
 bool lvgl_lock(int timeout_ms) {
     const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
     return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
 }
 
 /**
  * @brief Libera el mutex de LVGL.
  *
  * Esta función libera el mutex recursivo utilizado para sincronizar el acceso a las APIs de LVGL.
  */
 void lvgl_unlock(void) {
     xSemaphoreGiveRecursive(lvgl_mux);
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 // ---------------------------------------------------------
 // FUNCIONES Y TAREAS PARA CONEXIÓN WIFI
 // ---------------------------------------------------------
 
 /**
  * @brief Inicializa el servicio Wi‑Fi.
  *
  * Esta función configura el sistema Wi‑Fi para que opere en modo estación (STA).
  * Si el servicio ya ha sido inicializado, se omite la configuración adicional.
  * Se crean los grupos de eventos necesarios, se inicializa la pila de red,
  * se crea la interfaz de red por defecto y se configuran los manejadores de eventos
  * para notificar cambios de estado (inicio, desconexión, obtención de IP).
  *
  * @note Este es un buen punto para agregar funcionalidades adicionales como:
  *       - Configuración de modos adicionales (AP, STA+AP).
  *       - Registro de más eventos personalizados.
  */
 void wifi_service_init(void) {
     if (wifi_initialized) {
         ESP_LOGI(TAG, "Servicio Wi‑Fi ya está inicializado.");
         return;
     }
 
     // Crea el grupo de eventos para sincronización de la conexión Wi‑Fi.
     s_wifi_event_group = xEventGroupCreate();
 
     ESP_ERROR_CHECK(esp_netif_init());
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
     assert(sta_netif != NULL);
 
     // Configuración por defecto del Wi‑Fi.
     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
 
     // Registra los manejadores de eventos para WIFI_EVENT y IP_EVENT.
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
 
     // Configura el modo Wi‑Fi como estación y arranca el driver.
     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
     ESP_ERROR_CHECK(esp_wifi_start());
 
     wifi_initialized = true;
     ESP_LOGI(TAG, "Servicio Wi‑Fi inicializado.");
 }
 
 /**
  * @brief Inicializa la conexión Wi‑Fi en modo estación (STA).
  *
  * Esta función configura la red Wi‑Fi utilizando el SSID y la contraseña proporcionados.
  * Si el servicio Wi‑Fi aún no está inicializado, se llama a wifi_service_init().
  * Luego se actualiza la configuración de la interfaz y se intenta establecer la conexión.
  * Finalmente, se espera (hasta 10 segundos) la obtención de la conexión, lo que permite
  * sincronizar otros módulos que dependan de la red.
  *
  * @param ssid Cadena de caracteres con el SSID de la red a conectar.
  * @param password Cadena de caracteres con la contraseña de la red.
  *
  * @note Este es un punto ideal para agregar características como:
  *       - Implementar reintentos de conexión con lógica personalizada.
  *       - Notificar a otros módulos mediante callbacks o eventos.
  */
 void wifi_init_sta(const char *ssid, const char *password) {
     // Asegura que el servicio Wi‑Fi esté inicializado
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
 
     // Actualiza la configuración y conecta a la red.
     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
     ESP_ERROR_CHECK(esp_wifi_connect());
 
     ESP_LOGI(TAG, "Wi‑Fi initialization completed.");
 
     // Espera hasta 10 segundos para que se establezca la conexión.
     EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            pdMS_TO_TICKS(10000));
 }
 
 /**
  * @brief Manejador de eventos para el Wi‑Fi y la obtención de IP.
  *
  * Esta función gestiona diferentes eventos del sistema Wi‑Fi:
  *  - Cuando el Wi‑Fi se inicia, se informa que está a la espera de conexión.
  *  - En caso de desconexión, se actualiza el estado del ícono y se implementa lógica de reintentos,
  *    a menos que la desconexión haya sido solicitada manualmente.
  *  - Cuando se obtiene una IP, se reinicia el contador de reintentos, se actualiza el ícono,
  *    se muestra un mensaje de éxito y se inicia SNTP si aún no lo ha sido.
  *
  * @param arg Parámetro genérico (no utilizado).
  * @param event_base Base del evento (WIFI_EVENT o IP_EVENT).
  * @param event_id Identificador del evento.
  * @param event_data Puntero a la estructura con información del evento.
  *
  * @note Para extender esta función se puede:
  *       - Agregar manejo de otros eventos, como la pérdida de IP.
  *       - Integrar notificaciones a otros módulos o actualizar UI de manera más elaborada.
  */
 static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
         ESP_LOGI(TAG, "Wi‑Fi iniciado. Esperando acción del usuario para conectar.");
         // Se elimina la llamada a esp_wifi_connect() para evitar el escaneo automático.
     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
         // Actualiza el ícono a "desconectado" mediante LVGL de forma asíncrona.
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
     
         // Muestra un mensaje de éxito mediante la UI de LVGL.
         if (lvgl_lock(-1)) {
             success_msgbox = lv_msgbox_create(NULL, "Conexión Exitosa", "Conexión establecida correctamente.", NULL, true);
             lv_obj_center(success_msgbox);
             lvgl_unlock();
         }
         // Actualiza el ícono a "conectado".
         lv_async_call(set_wifi_icon_connected, NULL);
 
         // Inicializa SNTP para sincronizar el tiempo, si aún no se ha hecho.
         if (!sntp_initialized) {
             ESP_LOGI(TAG, "Iniciando SNTP");
             sntp_setoperatingmode(SNTP_OPMODE_POLL);
             sntp_setservername(0, "pool.ntp.org");
             sntp_init();
             sntp_initialized = true;
         }
     }
 }
 
 /**
  * @brief Tarea que gestiona la conexión Wi‑Fi.
  *
  * Esta tarea se encarga de configurar la red Wi‑Fi a partir de la configuración recibida
  * (estructura wifi_config_t), intentar la conexión y notificar en caso de error.
  * Finalmente, libera la memoria asignada a la configuración y termina la tarea.
  *
  * @param param Puntero a la configuración Wi‑Fi (wifi_config_t *).
  *
  * @note Esta función se puede extender para implementar políticas de reconexión avanzadas.
  */
 static void wifi_connect_task(void *param) {
     wifi_config_t *wifi_config = (wifi_config_t *)param;
 
     // Configura la red Wi‑Fi con la configuración recibida.
     esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, wifi_config);
     if (err == ESP_OK) {
         err = esp_wifi_connect();
     }
 
     // Si ocurre un error durante la conexión, se muestra un mensaje de error en la UI.
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Error al conectar a la red Wi-Fi: %s", esp_err_to_name(err));
         if (lvgl_lock(-1)) {
             lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo conectar a la red Wi-Fi.", NULL, true);
             lv_obj_center(msgbox);
             lvgl_unlock();
         }
     }
 
     // Libera la memoria de la configuración Wi‑Fi.
     free(wifi_config);
 
     // Termina la tarea.
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Tarea para escanear redes Wi‑Fi disponibles.
  *
  * Esta tarea inicia un escaneo activo de redes Wi‑Fi, espera un breve período para
  * la recolección de resultados y obtiene el número y la información de las redes detectadas.
  * Posteriormente, construye una cadena de texto con los SSID de las redes encontradas y
  * la envía a través de un callback de LVGL para actualizar un dropdown en la UI.
  *
  * @param param Puntero a parámetros (no utilizado en este caso).
  *
  * @note Para agregar nuevas funcionalidades, se podría:
  *       - Filtrar redes por intensidad de señal.
  *       - Agregar soporte para seleccionar canales específicos.
  */
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
         .scan_time.active.max = 1000,
     };
 
     // Inicia el escaneo y, si el Wi‑Fi no está iniciado, lo inicia.
     esp_err_t err = esp_wifi_scan_start(&scan_config, false);
     if (err == ESP_ERR_WIFI_NOT_STARTED) {
         ESP_LOGI(TAG, "Wi-Fi no iniciado, iniciándolo...");
         ESP_ERROR_CHECK(esp_wifi_start());
         err = esp_wifi_scan_start(&scan_config, false);
     }
     ESP_ERROR_CHECK(err);
 
     // Espera 1 segundo para que el escaneo se complete.
     vTaskDelay(pdMS_TO_TICKS(1000));
 
     esp_wifi_scan_stop();
     ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
     ESP_LOGI(TAG, "Total de redes detectadas: %u", ap_count);
 
     // Obtiene la información de las redes detectadas, limitando el número a DEFAULT_SCAN_LIST_SIZE.
     uint16_t num_to_fetch = ap_count < DEFAULT_SCAN_LIST_SIZE ? ap_count : DEFAULT_SCAN_LIST_SIZE;
     ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_to_fetch, ap_info));
 
     memset(top_networks, 0, sizeof(top_networks));
     for (int i = 0; i < num_to_fetch; i++) {
         top_networks[i] = ap_info[i];
         ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_info[i].ssid, ap_info[i].rssi);
     }
     free(ap_info);
 
     // Construye una cadena con los SSID detectados separados por saltos de línea.
     char *dropdown_options = malloc(256);
     if (dropdown_options) {
         dropdown_options[0] = '\0';
         for (int i = 0; i < num_to_fetch; i++) {
             strcat(dropdown_options, (const char *)top_networks[i].ssid);
             if (i < num_to_fetch - 1) {
                 strcat(dropdown_options, "\n");
             }
         }
         // Llama asíncronamente a la función para actualizar el dropdown en el contexto de LVGL.
         lv_async_call(update_dropdown_options, dropdown_options);
         // La memoria se libera dentro del callback update_dropdown_options.
     }
 
     ESP_LOGI(TAG, "Escaneo completado.");
     // Reinicia el handle de la tarea y termina la tarea.
     wifi_task_handle = NULL;
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Actualiza las opciones de un dropdown de LVGL.
  *
  * Esta función se llama de forma asíncrona mediante lv_async_call() para actualizar
  * las opciones del dropdown de redes Wi‑Fi. Verifica que el objeto dropdown sea válido
  * y, en caso afirmativo, actualiza su lista de opciones. Finalmente, libera la memoria
  * asignada a la cadena de opciones.
  *
  * @param param Puntero a la cadena de caracteres que contiene las nuevas opciones.
  *
  * @note Se puede ampliar para aplicar filtros o formatear de forma dinámica las opciones.
  */
 static void update_dropdown_options(void *param) {
     char *options = (char *)param;
     // Verificar que el objeto dropdown aún sea válido
     if (ssdropdown != NULL && lv_obj_is_valid(ssdropdown)) {
         lv_dropdown_set_options(ssdropdown, options);
     }
     free(options); // Libera la memoria asignada a la cadena de opciones.
 }
 
 /**
  * @brief Manejador de eventos para el botón de escaneo de redes Wi‑Fi.
  *
  * Esta función se activa cuando se presiona el botón destinado a iniciar el escaneo
  * de redes Wi‑Fi. Antes de crear la tarea, se verifica que no haya un escaneo en curso.
  * Si no hay ninguna tarea de escaneo activa, se crea la tarea `wifi_scan_task` en el Core 0.
  *
  * @param event Puntero al evento LVGL que dispara esta función.
  *
  * @note Se puede ampliar para notificar visualmente el inicio del escaneo en la UI.
  */
 static void scan_button_event_handler(lv_event_t *event) {
     if (wifi_task_handle == NULL) { // Verificar que no haya un escaneo en curso
         ESP_LOGI(TAG, "Creando tarea de escaneo en Core 0...");
         xTaskCreatePinnedToCore(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, &wifi_task_handle, 0);
     } else {
         ESP_LOGW(TAG, "Escaneo ya en curso...");
     }
 }
 
 /**
  * @brief Manejador de eventos para el botón de conexión Wi‑Fi.
  *
  * Esta función se invoca cuando el usuario presiona el botón para conectarse a una red.
  * Obtiene el SSID seleccionado del dropdown y la contraseña del campo de texto asociado.
  * Realiza validaciones básicas: que se haya seleccionado un SSID válido y que la contraseña
  * cumpla con una longitud mínima. Si todo es correcto, configura la red Wi‑Fi y establece la conexión.
  *
  * @param event Puntero al evento LVGL que dispara esta función.
  *
  * @note Es posible ampliar esta función para agregar funcionalidades como:
  *       - Mostrar indicadores de progreso durante la conexión.
  *       - Implementar lógicas de reconexión o gestión de errores más detalladas.
  */
 static void connect_button_event_handler(lv_event_t *event) {
     char selected_ssid[33];
     lv_dropdown_get_selected_str(ssdropdown, selected_ssid, sizeof(selected_ssid));
 
     const char *password = lv_textarea_get_text((lv_obj_t *)lv_event_get_user_data(event));
 
     // Validar que se haya seleccionado un SSID válido
     if (strcmp(selected_ssid, "Seleccione una red...") == 0 || strlen(selected_ssid) == 0) {
         ESP_LOGW(TAG, "No se seleccionó una red válida.");
         if (lvgl_lock(-1)) {
             lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "Seleccione una red válida.", NULL, true);
             lv_obj_center(msgbox);
             lvgl_unlock();
         }
         return;
     }
 
     // Validar que se haya ingresado una contraseña adecuada (al menos 8 caracteres)
     if (strlen(password) == 0 || strlen(password) < 8) {
         ESP_LOGW(TAG, "No se ingresó una contraseña válida.");
         if (lvgl_lock(-1)) {
             lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "Ingrese una contraseña válida (al menos 8 caracteres).", NULL, true);
             lv_obj_center(msgbox);
             lvgl_unlock();
         }
         return;
     }
 
     ESP_LOGI(TAG, "Intentando conectar a SSID: %s con contraseña: %s", selected_ssid, password);
 
     // Si ya existe una tarea de conexión en curso, se elimina para reiniciar la conexión.
     if (wifi_task_handle != NULL) {
         vTaskDelete(wifi_task_handle);
         wifi_task_handle = NULL;
     }
 
     // Configura la red Wi‑Fi con los parámetros proporcionados.
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
     }
 
     // Notifica al usuario si ocurre algún error durante la conexión.
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Error al iniciar la conexión Wi‑Fi: %s", esp_err_to_name(err));
         if (lvgl_lock(-1)) {
             lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo iniciar la conexión Wi‑Fi.", NULL, true);
             lv_obj_center(msgbox);
             lvgl_unlock();
         }
     }
 }
 
 /**
  * @brief Manejador de eventos para el botón de desconexión Wi‑Fi.
  *
  * Esta función se ejecuta al presionar el botón de desconexión. Establece la bandera
  * que indica que la desconexión fue solicitada manualmente y llama a la función
  * esp_wifi_disconnect() para finalizar la conexión. En caso de error, se muestra
  * un mensaje en la UI.
  *
  * @param event Puntero al evento LVGL que dispara esta función.
  *
  * @note Se puede extender para agregar notificaciones o realizar limpieza de recursos
  *       específicos tras la desconexión.
  */
 static void disconnect_button_event_handler(lv_event_t *event) {
     wifi_manual_disconnect = true;  // Indica que la desconexión fue solicitada manualmente
     esp_err_t err = esp_wifi_disconnect();
     if (err == ESP_OK) {
         // La desconexión fue exitosa; se podría notificar al usuario si se desea.
     } else {
         ESP_LOGE(TAG, "Error al desconectar: %s", esp_err_to_name(err));
         if (lvgl_lock(-1)) {
             lv_obj_t *msgbox = lv_msgbox_create(NULL, "Error", "No se pudo desconectar de la red Wi‑Fi.", NULL, true);
             lv_obj_center(msgbox);
             lvgl_unlock();
         }
     }
 }
 
 /**
  * @brief Actualiza el ícono de estado Wi‑Fi para indicar conexión.
  *
  * Esta función cambia el texto del objeto LVGL asociado al ícono de Wi‑Fi,
  * indicando que el dispositivo está conectado (por ejemplo, mostrando el símbolo de Wi‑Fi).
  *
  * @param arg Parámetro no utilizado (se requiere por la firma de callback).
  *
  * @note Se puede ampliar para incluir animaciones o cambiar colores según el estado.
  */
 static void set_wifi_icon_connected(void *arg) {
     lv_label_set_text(wifi_status_icon, LV_SYMBOL_WIFI);
 }
 
 /**
  * @brief Actualiza el ícono de estado Wi‑Fi para indicar desconexión.
  *
  * Esta función cambia el texto del objeto LVGL asociado al ícono de Wi‑Fi,
  * indicando que el dispositivo está desconectado (por ejemplo, mostrando un símbolo de cierre).
  *
  * @param arg Parámetro no utilizado (se requiere por la firma de callback).
  *
  * @note Se puede ampliar para incluir animaciones o cambiar colores según el estado.
  */
 static void set_wifi_icon_disconnected(void *arg) {
     lv_label_set_text(wifi_status_icon, LV_SYMBOL_CLOSE);
 }
 
 /**
  * @brief Elimina el mensaje flotante de la UI.
  *
  * Esta función verifica si existe un objeto de mensaje flotante y, de ser así, lo elimina,
  * liberando así el recurso y actualizando el estado de la UI.
  *
  * @param event Puntero al evento LVGL que dispara esta función.
  *
  * @note Esto es útil para limpiar mensajes temporales en la UI antes de mostrar otros.
  */
 static void remove_floating_msgbox(lv_event_t *event) {
     if (floating_msgbox != NULL) {
         lv_obj_del(floating_msgbox);
         floating_msgbox = NULL;
     }
 }
 
 /**
  * @brief Crea y configura la interfaz de usuario para la configuración de Wi‑Fi.
  *
  * Esta función crea un contenedor principal para la UI de Wi‑Fi y configura los siguientes elementos:
  *  - Un dropdown para seleccionar el SSID de la red.
  *  - Un campo de texto para ingresar la contraseña.
  *  - Botones para escanear redes, conectar y desconectar.
  *
  * La función organiza los elementos en un layout flexible y aplica estilos personalizados.
  *
  * @param parent Objeto LVGL padre donde se crea la UI de Wi‑Fi.
  *
  * @note Es un punto clave para extender la funcionalidad agregando, por ejemplo:
  *       - Validaciones adicionales de entrada.
  *       - Indicadores visuales del estado de conexión.
  */
 void create_wifi_settings_widget(lv_obj_t *parent) {
     // Configurar el tamaño y posición del contenedor principal.
     lv_obj_set_size(parent, 300, 400);
     lv_obj_center(parent);
     lv_obj_set_style_bg_color(parent, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
     lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
     lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE); // Deshabilitar scroll
 
     // Crear un contenedor secundario para organizar los controles.
     lv_obj_t *container = lv_obj_create(parent);
     lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_set_size(container, 300, 400);
     lv_obj_center(container);
     lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
     lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
     lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
     lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
 
     // Crear el dropdown para seleccionar el SSID.
     ssdropdown = lv_dropdown_create(container);
     lv_obj_set_width(ssdropdown, 200);
     lv_dropdown_set_options(ssdropdown, "Seleccione una red...");
 
     // Crear el campo de texto para la contraseña.
     lv_obj_t *password_textarea = lv_textarea_create(container);
     lv_textarea_set_one_line(password_textarea, true);
     lv_textarea_set_password_mode(password_textarea, true);
     lv_textarea_set_placeholder_text(password_textarea, "Ingrese contraseña");
     lv_obj_set_width(password_textarea, 200);
     lv_obj_add_event_cb(password_textarea, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
 
     // (Opcional) Se puede preconfigurar un texto de prueba:
     // lv_textarea_set_text(password_textarea, "111111111111111");
 
     // Crear el botón para escanear redes Wi‑Fi.
     lv_obj_t *scan_btn = lv_btn_create(container);
     lv_obj_set_size(scan_btn, 120, 50);
     lv_obj_t *scan_label = lv_label_create(scan_btn);
     lv_label_set_text(scan_label, "Escanear");
     lv_obj_center(scan_label);
     lv_obj_add_event_cb(scan_btn, scan_button_event_handler, LV_EVENT_CLICKED, NULL);
 
     // Crear el botón para conectar a la red seleccionada.
     lv_obj_t *connect_btn = lv_btn_create(container);
     lv_obj_set_size(connect_btn, 120, 50);
     lv_obj_t *connect_label = lv_label_create(connect_btn);
     lv_label_set_text(connect_label, "Conectar");
     lv_obj_center(connect_label);
     lv_obj_add_event_cb(connect_btn, connect_button_event_handler, LV_EVENT_CLICKED, password_textarea);
 
     // Crear el botón para desconectar.
     lv_obj_t *disconnect_btn = lv_btn_create(container);
     lv_obj_set_size(disconnect_btn, 120, 50);
     lv_obj_t *disconnect_label = lv_label_create(disconnect_btn);
     lv_label_set_text(disconnect_label, "Desconectar");
     lv_obj_center(disconnect_label);
     lv_obj_add_event_cb(disconnect_btn, disconnect_button_event_handler, LV_EVENT_CLICKED, NULL);
 }
 
 
 
 
 
 // ---------------------------------------------------------
 // Funciones de la Interfaz de Usuario (UI) para Productos
 // ---------------------------------------------------------
 
 /**
  * @brief Crea un widget de textarea simple.
  *
  * Esta función genera un widget de tipo textarea en LVGL configurado para mostrar
  * un texto de marcador de posición y operar en modo de una sola línea. Además, se
  * registra un callback para que, al recibir foco, se active el teclado virtual.
  *
  * @param parent Puntero al objeto LVGL padre en el que se insertará el textarea.
  * @param placeholder Cadena de caracteres que se utilizará como texto de marcador de posición.
  * @return Puntero al objeto LVGL creado.
  *
  * @note En futuras versiones se podría extender la funcionalidad para incluir validaciones
  *       de entrada o estilos dinámicos según el contexto de uso.
  * \todo Agregar validaciones y estilos personalizados al textarea.
  */
 static lv_obj_t *create_textarea(lv_obj_t *parent, const char *placeholder) {
     ESP_LOGI(TAG, "Creating textarea with placeholder: %s", placeholder);
     lv_obj_t *ta = lv_textarea_create(parent);
     lv_textarea_set_placeholder_text(ta, placeholder);
     lv_textarea_set_one_line(ta, true);
     // Registra el callback para que al recibir foco se muestre el teclado
     lv_obj_add_event_cb(ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
     return ta;
 }
 
 /**
  * @brief Callback para guardar los cambios de nombre en la subpágina de un producto.
  *
  * Esta función se invoca cuando el usuario presiona el botón de guardar en la
  * subpágina de edición de un producto. Busca el widget de textarea que contiene el
  * nombre (identificado por su placeholder "Nombre") y, si el texto ingresado es válido,
  * actualiza el label del contenedor padre para reflejar el cambio.
  *
  * @param e Puntero al evento LVGL.
  *
  * @note Se recomienda mejorar la robustez añadiendo mensajes de error más detallados
  *       y validaciones adicionales sobre el formato del nombre.
  * \todo Agregar mensajes de error más detallados y validaciones adicionales sobre
  *       el formato del nombre. del producto en la UI.
  */
 static void save_button_event_cb(lv_event_t *e) {
     ESP_LOGI(TAG, "Save button pressed");
     
     // Obtener la subpágina asociada (almacenada en el user_data del botón)
     lv_obj_t *sub_page = lv_event_get_user_data(e);
     if (sub_page == NULL) {
         ESP_LOGE(TAG, "Sub-page no válida");
         return;
     }
 
     // Buscar el textarea cuyo placeholder sea "Nombre"
     lv_obj_t *name_ta = NULL;
     uint32_t count = lv_obj_get_child_cnt(sub_page);
     for (uint32_t i = 0; i < count; i++) {
         lv_obj_t *child = lv_obj_get_child(sub_page, i);
         if (lv_obj_check_type(child, &lv_textarea_class)) {
             const char *ph = lv_textarea_get_placeholder_text(child);
             if (ph && strcmp(ph, "Nombre") == 0) {
                 name_ta = child;
                 break;
             }
         }
     }
     if (name_ta == NULL) {
         ESP_LOGE(TAG, "No se encontró el campo de nombre");
         return;
     }
     
     // Obtener el texto ingresado en el textarea
     const char *name = lv_textarea_get_text(name_ta);
     if (strlen(name) == 0) {
         ESP_LOGW(TAG, "El nombre está vacío");
         printf("El nombre no puede estar vacío\n");
         return;
     }
     
     // Obtener el contenedor vinculado a la subpágina (almacenado en el user_data)
     lv_obj_t *cont = lv_obj_get_user_data(sub_page);
     if (cont == NULL) {
         ESP_LOGE(TAG, "No se encontró el contenedor vinculado a la sub-page");
         printf("Error: No se encontró el contenedor vinculado.\n");
         return;
     }
     
     // Buscar el label que muestra el nombre del producto dentro del contenedor
     lv_obj_t *label = NULL;
     uint32_t child_count = lv_obj_get_child_cnt(cont);
     for (uint32_t i = 0; i < child_count; i++) {
         lv_obj_t *child = lv_obj_get_child(cont, i);
         if (lv_obj_check_type(child, &lv_label_class)) {
             label = child;
             break;
         }
     }
     
     if (label == NULL) {
         ESP_LOGE(TAG, "No se encontró el label dentro del contenedor");
         return;
     }
     
     // Actualizar el label con el nuevo nombre
     lv_label_set_text(label, name);
     ESP_LOGI(TAG, "Product name saved: %s", name);
 }
 
 /**
  * @brief Elimina el último producto agregado.
  *
  * Esta función elimina el último elemento del arreglo de productos (representado por
  * cont_arr) y actualiza el contador global de productos. Se registra un mensaje de advertencia
  * si no hay productos para eliminar.
  *
  * @param e Puntero al evento LVGL que dispara esta acción.
  *
  * @note Se puede extender para incluir confirmaciones de usuario antes de eliminar o
  *       implementar una función de "deshacer".
  * \todo Agregar confirmaciones de usuario antes de eliminar o implementar una función de "deshacer".
  */
 static void delete_last_item(lv_event_t *e) {
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
 
 /**
  * @brief Elimina todos los productos registrados.
  *
  * Esta función recorre el arreglo de productos y elimina cada uno de los elementos,
  * actualizando el contador global de productos a cero.
  *
  * @param e Puntero al evento LVGL que dispara esta acción.
  *
  * @note Podría extenderse para pedir confirmación al usuario antes de borrar todos los datos.
  * \todo Pedir confirmación al usuario antes de borrar todos los datos.
  */
 static void delete_all_items(lv_event_t *e) {
     ESP_LOGI(TAG, "Deleting all products");
     while (cont_index > 0) {
         lv_obj_del(cont_arr[cont_index - 1]);
         cont_arr[cont_index - 1] = NULL;
         cont_index--;
     }
     ESP_LOGI(TAG, "All products deleted");
 }
 
 /**
  * @brief Callback para guardar los cambios.
  *
  * Esta función se invoca cuando el usuario presiona el botón destinado a guardar los
  * cambios realizados en la configuración de productos. Llama a la función save_products_to_nvs()
  * para almacenar los datos de los productos en NVS.
  *
  * @param e Puntero al evento LVGL.
  *
  * @note Se puede mejorar integrando confirmaciones visuales o validaciones previas.
  * \todo Integrar confirmaciones visuales o validaciones previas antes de guardar los datos.
  */
 static void save_objects_btn(lv_event_t *e) {
     ESP_LOGI(TAG, "Guardando productos en NVS");
     save_products_to_nvs();
 }
 
 /**
  * @brief Crea un nuevo producto y su subpágina de edición.
  *
  * Esta función crea dinámicamente un nuevo producto en la interfaz, generando una subpágina
  * donde el usuario puede editar el nombre, precio y descripción del producto. Además, se actualiza
  * el menú principal para incluir el nuevo producto.
  *
  * @param e Puntero al evento LVGL que dispara esta acción.
  *
  * @note Se pueden agregar funcionalidades adicionales como validación de datos o
  *       asignación de imágenes para cada producto.
  * \todo Agregar validación de datos y asignación de imágenes para cada producto.
  */
 static void create_new_product(lv_event_t *e) {
     ESP_LOGI(TAG, "Creating new product. Current count: %lu", cont_index);
     if (cont_index >= MAX_ITEMS) {
         ESP_LOGW(TAG, "Maximum products reached");
         printf("No se pueden añadir más productos\n");
         return;
     }
     // Crear la subpágina para el nuevo producto en el menú.
     lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
     ESP_LOGI(TAG, "New sub-page created for product %lu", cont_index + 1);
     
     // Crear el campo "Nombre" con un valor por defecto y registrar el evento de teclado.
     lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
     lv_obj_add_event_cb(name_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
     static char default_name[20];
     snprintf(default_name, sizeof(default_name), "Producto %lu", cont_index + 1);
     lv_textarea_set_text(name_ta, default_name);  // Asigna un nombre por defecto
 
     // Crear el campo "Precio" con un valor predeterminado.
     lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
     lv_obj_add_event_cb(price_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
     lv_textarea_set_text(price_ta, "100");
 
     // Crear el campo "Descripción".
     lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
     lv_obj_add_event_cb(desc_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
 
     // Crear el botón de guardar en la subpágina y registrar su callback.
     lv_obj_t *save_btn = lv_btn_create(new_sub_page);
     lv_obj_t *label = lv_label_create(save_btn);
     lv_label_set_text(label, "Guardar");
     lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
     ESP_LOGI(TAG, "Save button created for new product");
 
     // Integrar el nuevo producto en el menú principal y actualizar el arreglo global.
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
 
 /**
  * @brief Crea la interfaz de configuración de productos en una pestaña.
  *
  * Esta función genera la parte de la interfaz donde se configuran los productos. Se crea un
  * contenedor principal (config_container) que agrupa un encabezado, un menú para visualizar
  * y editar los productos, y un footer con botones para agregar, eliminar o borrar todos los
  * productos. Además, se registra el callback para detectar cambios de página en el menú.
  *
  * @param parent Puntero al objeto LVGL (la pestaña) en el que se creará la UI de productos.
  *
  * @note Futuras mejoras podrían incluir filtros, búsquedas o la integración con bases de datos
  *       externas para una gestión más avanzada de productos.
  * \todo Agregar filtros, búsquedas o integración con bases de datos externas para una gestión avanzada de productos.
  */
 static void product_config_in_tab(lv_obj_t *parent) {
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
 
     // Creación del encabezado (header) con título.
     lv_obj_t *header = lv_obj_create(config_container);
     ESP_LOGI(TAG, "Header created");
     lv_obj_set_size(header, 730, 60);
     lv_obj_align(header, LV_ALIGN_TOP_MID, 0, -10);
     lv_obj_t *title_label = lv_label_create(header);
     lv_label_set_text(title_label, "Configuración de productos:");
     lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 0);
 
     // Creación del contenedor que alberga el menú de productos.
     lv_obj_t *menu_container = lv_obj_create(config_container);
     ESP_LOGI(TAG, "Menu container created");
     lv_obj_set_size(menu_container, 730, 350);
     lv_obj_align(menu_container, LV_ALIGN_TOP_MID, 0, 50);
     lv_obj_set_scroll_dir(menu_container, LV_DIR_VER);
     lv_obj_clear_flag(menu_container, LV_OBJ_FLAG_SCROLLABLE);
 
     // Creación del menú de productos y asignación de la página principal.
     menu = lv_menu_create(menu_container);
     ESP_LOGI(TAG, "Menu created for products");
     lv_obj_clear_flag(menu, LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_set_size(menu, 730, 300);
     lv_obj_align(menu, LV_ALIGN_TOP_MID, 0, 0);
     main_page = lv_menu_page_create(menu, NULL);
     lv_menu_set_page(menu, main_page);
 
     // Se guarda la página principal globalmente para futuras referencias.
     main_menu_page = main_page;
 
     // Cargar productos previamente guardados (si existen).
     load_products_for_config();
 
     // Registrar el callback para detectar el cambio de página en el menú.
     lv_obj_add_event_cb(menu, menu_page_changed_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
 
     // Creación del footer con botones de acción.
     lv_obj_t *footer = lv_obj_create(config_container);
     lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);
     ESP_LOGI(TAG, "Footer created for product configuration");
     lv_obj_set_size(footer, 730, 62);
     lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);
 
     // Botón para agregar un producto.
     float_btn_add = lv_btn_create(footer);
     lv_obj_set_size(float_btn_add, 50, 50);
     lv_obj_align(float_btn_add, LV_ALIGN_RIGHT_MID, -50, 0);
     lv_obj_add_event_cb(float_btn_add, create_new_product, LV_EVENT_CLICKED, NULL);
     lv_obj_set_style_radius(float_btn_add, LV_RADIUS_CIRCLE, 0);
     lv_obj_set_style_bg_img_src(float_btn_add, LV_SYMBOL_PLUS, 0);
 
     // Botón para eliminar el último producto.
     float_btn_del = lv_btn_create(footer);
     lv_obj_set_size(float_btn_del, 50, 50);
     lv_obj_align(float_btn_del, LV_ALIGN_RIGHT_MID, -120, 0);
     lv_obj_add_event_cb(float_btn_del, delete_last_item, LV_EVENT_CLICKED, NULL);
     lv_obj_set_style_radius(float_btn_del, LV_RADIUS_CIRCLE, 0);
     lv_obj_set_style_bg_img_src(float_btn_del, LV_SYMBOL_MINUS, 0);
 
     // Botón para eliminar todos los productos.
     float_btn_del_all = lv_btn_create(footer);
     lv_obj_set_size(float_btn_del_all, 50, 50);
     lv_obj_align(float_btn_del_all, LV_ALIGN_RIGHT_MID, -170, 0);
     lv_obj_add_event_cb(float_btn_del_all, delete_all_items, LV_EVENT_CLICKED, NULL);
     lv_obj_set_style_radius(float_btn_del_all, LV_RADIUS_CIRCLE, 0);
     lv_obj_set_style_bg_img_src(float_btn_del_all, LV_SYMBOL_TRASH, 0);
 
     // Botón para guardar cambios.
     save_btn = lv_btn_create(footer);
     lv_obj_set_size(save_btn, 40, 40);
     lv_obj_align(save_btn, LV_ALIGN_CENTER, 0, 0);
     lv_obj_t *btn_label = lv_label_create(save_btn);
     lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
     lv_label_set_text(btn_label, LV_SYMBOL_SAVE);
     lv_obj_add_event_cb(save_btn, save_objects_btn, LV_EVENT_CLICKED, NULL);
 
     ESP_LOGI(TAG, "Product configuration UI created successfully");
 }
 
 /**
  * @brief Guarda la configuración de productos en la memoria no volátil (NVS).
  *
  * Esta función recorre el arreglo de productos actualmente configurados y guarda, para cada uno,
  * sus datos (nombre, precio y descripción) en NVS. Se actualiza también el contador global de productos.
  *
  * @note Se puede ampliar para incluir la validación de integridad de datos o la compresión de los mismos.
  * \todo Agregar validación de integridad de datos o compresión de los mismo antes de guardar en NVS. 
  */
 void save_products_to_nvs(void) {
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
         // Se asume que el contenedor del producto almacena la subpágina de edición en su user_data.
         lv_obj_t *cont = cont_arr[i];
         lv_obj_t *sub_page = lv_obj_get_user_data(cont);
         if (sub_page == NULL) continue;
         
         // Se extraen los valores de los campos: Nombre, Precio y Descripción.
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
 
 /**
  * @brief Carga los productos almacenados en NVS y los recrea en la interfaz de configuración.
  *
  * Esta función abre el almacén NVS, lee el número de productos guardados y, para cada uno,
  * recupera su nombre, precio y descripción. Luego, reconstruye la interfaz de edición para cada
  * producto en el menú principal.
  *
  * @note Se pueden añadir mejoras como la migración de datos o la verificación de versiones.
  * \todo Agregar migración de datos desde tarjeta SD o verificación de versiones al cargar los productos.
  */
 void load_products_for_config(void) {
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
             if (nvs_get_str(my_handle, price_key, product_price, &price_len) != ESP_OK) {
                 strcpy(product_price, "50");
             }
             
             // Cargar descripción
             char desc_key[20];
             char product_desc[64];  // Ajusta el tamaño según necesidades
             size_t desc_len = sizeof(product_desc);
             snprintf(desc_key, sizeof(desc_key), "desc_%lu", i);
             if (nvs_get_str(my_handle, desc_key, product_desc, &desc_len) != ESP_OK) {
                 strcpy(product_desc, "");
             }
             
             // Crear la subpágina para editar el producto.
             lv_obj_t *new_sub_page = lv_menu_page_create(menu, NULL);
             // Crear el campo de nombre y asignar el valor leído.
             lv_obj_t *name_ta = create_textarea(new_sub_page, "Nombre");
             lv_textarea_set_text(name_ta, product_name);
             // Crear el campo de precio y asignar el valor leído.
             lv_obj_t *price_ta = create_textarea(new_sub_page, "Precio");
             lv_textarea_set_text(price_ta, product_price);
             // Crear el campo de descripción y asignar el valor leído.
             lv_obj_t *desc_ta = create_textarea(new_sub_page, "Descripción");
             lv_textarea_set_text(desc_ta, product_desc);
             
             // Crear el botón de guardar en la subpágina.
             lv_obj_t *save_btn = lv_btn_create(new_sub_page);
             lv_obj_t *save_label = lv_label_create(save_btn);
             lv_label_set_text(save_label, "Guardar");
             lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);
             
             // Crear el contenedor del producto en el menú principal.
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
 
 /**
  * @brief Callback para volver a la pantalla principal desde la configuración.
  *
  * Esta función se invoca mediante un botón flotante y llama a la función switch_screen() para
  * cambiar el contenido del área principal a la pantalla de inicio.
  *
  * @param e Puntero al evento LVGL que dispara esta acción.
  *
  * @note Este callback se puede ampliar para incluir animaciones o transiciones personalizadas.
  * 
  */
 static void go_to_main_screen_from_config_cb(lv_event_t *e) {
     // Cambia el contenido al main_screen
     switch_screen(create_main_screen);
 }
 
 /**
  * @brief Crea la pantalla de configuración general.
  *
  * Esta función genera la pantalla de configuración usando un tabview de LVGL, que incluye:
  * - Tab 1: Configuración Wi‑Fi.
  * - Tab 2: Configuración de productos.
  * - Tab 3: Menú para cambiar la contraseña de administración.
  * - Tab 4: Comandos de la terminal de punto de venta (POS).
  *
  * Se configuran las opciones visuales y se integran las respectivas funciones de cada pestaña.
  *
  * @param parent Puntero al contenedor LVGL donde se creará la pantalla de configuración.
  *
  * @note En futuras versiones se puede ampliar el tabview para incluir más secciones o
  *       funcionalidades adicionales.
  * \todo Agregar más secciones o funcionalidades al tabview de configuración. Cada pestaña puede
  *       contener una funcionalidad específica o un conjunto de opciones relacionadas.
  */
 void create_general_config_screen(lv_obj_t *parent) {
     ESP_LOGI(TAG, "Creating general configuration screen in content");
     // Se crea el tabview en el contenedor principal.
     lv_obj_t *tabview = lv_tabview_create(parent, LV_DIR_LEFT, 70);
     lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_set_style_bg_color(tabview, lv_palette_lighten(LV_PALETTE_BLUE_GREY, 2), 0);
 
     // Se crean cuatro pestañas para diferentes configuraciones.
     lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "   " LV_SYMBOL_WIFI "\nConfig.\nWiFi");
     lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "    " LV_SYMBOL_LIST "\nConfig.\nProducts");
     lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "    ****\nCambiar\nClave\nadmin.");
     lv_obj_t *tab4 = lv_tabview_add_tab(tabview, "POS cmd\n\n init\npoll\nloadkeys");
 
     /* --- TAB 1: Configuración WiFi --- */
     lv_obj_t *container1 = lv_obj_create(tab1);
     lv_obj_set_align(container1, LV_ALIGN_CENTER);
     lv_obj_clear_flag(container1, LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_clear_flag(container1, LV_SCROLLBAR_MODE_OFF);
     create_wifi_settings_widget(container1);
 
     /* --- TAB 2: Configuración de productos --- */
     product_config_in_tab(tab2);
 
     /* --- TAB 3: Menú para cambiar contraseña --- */
     create_password_change_menu(tab3);
 
     /* --- TAB 4: Comandos POS --- */
     create_operation_cmd_pos_tab(tab4);
 
     // Botón flotante para regresar a la pantalla principal.
     lv_obj_t *float_btn = lv_btn_create(parent);
     lv_obj_set_size(float_btn, 50, 50);
     lv_obj_align(float_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -17);
     lv_obj_set_style_radius(float_btn, LV_RADIUS_CIRCLE, 0);
     lv_obj_set_style_bg_color(float_btn, lv_palette_main(LV_PALETTE_RED), 0);
     lv_obj_t *icon = lv_label_create(float_btn);
     lv_label_set_text(icon, LV_SYMBOL_HOME);
     lv_obj_center(icon);
     lv_obj_add_event_cb(float_btn, go_to_main_screen_from_config_cb, LV_EVENT_CLICKED, NULL);
 }
 
 
 /**
  * @brief Crea la pantalla principal de la aplicación.
  *
  * Esta función construye la pantalla principal en la que se muestran los productos disponibles
  * en un panel exhibidor, junto con botones para realizar acciones de compra y acceder a la
  * configuración de productos.
  *
  * @param parent Puntero al contenedor LVGL donde se desplegará la pantalla principal.
  *
  * @note La función podría ampliarse para incluir filtros, búsqueda o la integración con
  *       un carrito de compras.
  */
 void create_main_screen(lv_obj_t *parent) {
     // Creación del panel exhibidor de productos.
     lv_obj_t *exhibitor_panel = lv_obj_create(parent);
     lv_obj_set_size(exhibitor_panel, 800, 250);
     lv_obj_align(exhibitor_panel, LV_ALIGN_TOP_MID, 0, 10);
     lv_obj_set_style_pad_all(exhibitor_panel, 10, 0);
     lv_obj_set_style_bg_color(exhibitor_panel, lv_palette_main(LV_PALETTE_GREY), 0);
     lv_obj_set_layout(exhibitor_panel, LV_LAYOUT_FLEX);
     lv_obj_set_style_flex_flow(exhibitor_panel, LV_FLEX_FLOW_ROW, 0);
     lv_obj_set_flex_align(exhibitor_panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
     
     // Cargar y mostrar los productos almacenados en NVS.
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
                     // Obtener también el precio del producto, con valor por defecto si falla.
                     char price_key[20];
                     char product_price[16];
                     size_t price_len = sizeof(product_price);
                     snprintf(price_key, sizeof(price_key), "price_%lu", i);
                     if(nvs_get_str(my_handle, price_key, product_price, &price_len) != ESP_OK){
                         strcpy(product_price, "50");
                     }
                     
                     // Crear el widget del producto en el panel exhibidor.
                     lv_obj_t *item = lv_obj_create(exhibitor_panel);
                     lv_obj_clear_flag(item, LV_OBJ_FLAG_SCROLLABLE);
                     lv_obj_set_size(item, 120, 100);
                     lv_obj_set_style_bg_color(item, lv_palette_main(LV_PALETTE_BLUE), 0);
                     // Registrar el callback para la selección del producto.
                     lv_obj_add_event_cb(item, product_item_event_cb, LV_EVENT_CLICKED, NULL);
                     
                     // Agregar el label del nombre del producto.
                     lv_obj_t *name_label = lv_label_create(item);
                     lv_label_set_text(name_label, product_name);
                     lv_obj_align(name_label, LV_ALIGN_TOP_MID, 0, 5);
                     
                     // Agregar el label con el precio formateado.
                     char formatted_price[32];
                     snprintf(formatted_price, sizeof(formatted_price), "$%s", product_price);
                     lv_obj_t *price_label = lv_label_create(item);
                     lv_label_set_text(price_label, formatted_price);
                     lv_obj_align(price_label, LV_ALIGN_BOTTOM_MID, 0, -5);
                 }
             }
         } else {
             // Mostrar mensaje si no hay productos.
             lv_obj_t *label = lv_label_create(exhibitor_panel);
             lv_label_set_text(label, "No hay productos");
             lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
         }
         nvs_close(my_handle);
     }
     
     // Botón de compra.
     lv_obj_t *buy_btn = lv_btn_create(parent);
     lv_obj_set_size(buy_btn, 100, 50);
     lv_obj_align(buy_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
     // Se utiliza el exhibidor como user_data para identificar el producto seleccionado.
     lv_obj_add_event_cb(buy_btn, buy_button_event_cb, LV_EVENT_CLICKED, exhibitor_panel);
     lv_obj_t *buy_label = lv_label_create(buy_btn);
     lv_label_set_text(buy_label, "Comprar");
     lv_obj_center(buy_label);
 
     // Botón para acceder a la configuración de productos.
     lv_obj_t *btn_config = lv_btn_create(parent);
     lv_obj_set_size(btn_config, 40, 40);
     lv_obj_align(btn_config, LV_ALIGN_BOTTOM_LEFT, 0, 0);
     lv_obj_add_event_cb(btn_config, btn_to_config_event_cb, LV_EVENT_CLICKED, NULL);
     lv_obj_t *label_config = lv_label_create(btn_config);
     lv_label_set_text(label_config, LV_SYMBOL_SETTINGS); // Ícono de configuración.
     lv_obj_center(label_config);
 }
 
 /**
  * @brief Cambia la pantalla actual a la especificada.
  *
  * Esta función limpia el contenedor global de contenido y llama a la función
  * pasada como parámetro para generar la nueva pantalla. Permite modularizar
  * la navegación entre distintas secciones de la interfaz.
  *
  * @param create_screen Función que crea la pantalla deseada, recibiendo como parámetro el
  *                      objeto padre (global_content).
  *
  * @note Se podría ampliar para incluir animaciones de transición o guardar el historial
  *       de pantallas.
  *  \todo Agregar animaciones de transición o guardar el historial de pantallas visitadas.
  */
 void switch_screen(void (*create_screen)(lv_obj_t *parent)) {
     lv_obj_clean(global_content);
     create_screen(global_content);
 }
 
 /**
  * @brief Callback para manejar la selección de un producto.
  *
  * Esta función se invoca cuando un usuario selecciona (o deselecciona) un producto
  * en el panel de productos. Cambia el color de fondo del widget del producto para
  * indicar su estado (seleccionado en verde o no seleccionado en azul).
  *
  * @param e Puntero al evento LVGL disparado al hacer clic sobre un producto.
  *
  * @note La lógica de selección podría extenderse para permitir la selección múltiple,
  *       mostrando detalles adicionales o agregándolo a un carrito de compra.
  * \todo Mostrar detalles adicionales como monto total o cantidad seleccionada a pagar.
  */
 static void product_item_event_cb(lv_event_t *e) {
     // Obtiene el widget que disparó el evento.
     lv_obj_t *item = lv_event_get_target(e);
     if (!lv_obj_is_valid(item)) {
         ESP_LOGI(TAG, "Objeto no válido");
         return;
     }
     // Obtiene el contenedor padre (panel de productos).
     lv_obj_t *product_panel = lv_obj_get_parent(item);
     
     // Cambia el color de fondo según el estado actual.
     lv_color_t current_color = lv_obj_get_style_bg_color(item, LV_PART_MAIN);
     if (current_color.full == lv_color_make(0, 255, 0).full) {
         // Si ya está seleccionado (verde), se deselecciona (vuelve a azul).
         lv_obj_set_style_bg_color(item, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
     } else {
         // Deselecta todos los items y marca el actual como seleccionado.
         uint32_t child_count = lv_obj_get_child_cnt(product_panel);
         for (uint32_t i = 0; i < child_count; i++) {
             lv_obj_t *child = lv_obj_get_child(product_panel, i);
             lv_obj_set_style_bg_color(child, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
         }
         lv_obj_set_style_bg_color(item, lv_color_make(0, 255, 0), LV_PART_MAIN);
     }
 }
 
 /**
  * @brief Callback que se ejecuta cuando cambia la página en el menú de productos.
  *
  * Esta función se invoca automáticamente cuando el usuario cambia de página en el menú
  * de productos (por ejemplo, al entrar o salir de la vista de edición de un producto).
  * Su principal función es mostrar u ocultar los botones de acción (agregar, eliminar, borrar
  * todos, guardar) en función de si se está visualizando la pantalla principal o una subpágina
  * de edición.
  *
  * @param e Puntero al evento LVGL que contiene la información del cambio de página.
  *
  * @note Se puede ampliar esta función para incluir animaciones o transiciones al mostrar/ocultar
  *       los botones, mejorando la experiencia de usuario.
  */
 static void menu_page_changed_event_cb(lv_event_t *e) {
     // Obtiene el objeto menú que disparó el evento.
     lv_obj_t *menu_obj = lv_event_get_target(e);
     // Obtiene la página actual que se está visualizando en el menú.
     lv_obj_t *current_page = lv_menu_get_cur_main_page(menu_obj);
     
     // Si la página actual es la pantalla principal, se muestran los botones.
     if(current_page == main_page) {
         if(float_btn_add) lv_obj_clear_flag(float_btn_add, LV_OBJ_FLAG_HIDDEN);
         if(float_btn_del) lv_obj_clear_flag(float_btn_del, LV_OBJ_FLAG_HIDDEN);
         if(float_btn_del_all) lv_obj_clear_flag(float_btn_del_all, LV_OBJ_FLAG_HIDDEN);
         if(save_btn) lv_obj_clear_flag(save_btn, LV_OBJ_FLAG_HIDDEN);
     } else {
         // Si se está visualizando una subpágina, se ocultan los botones para evitar acciones
         // no pertinentes en ese contexto.
         if(float_btn_add) lv_obj_add_flag(float_btn_add, LV_OBJ_FLAG_HIDDEN);
         if(float_btn_del) lv_obj_add_flag(float_btn_del, LV_OBJ_FLAG_HIDDEN);
         if(float_btn_del_all) lv_obj_add_flag(float_btn_del_all, LV_OBJ_FLAG_HIDDEN);
         if(save_btn) lv_obj_add_flag(save_btn, LV_OBJ_FLAG_HIDDEN);
     }
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 // ---------------------------------------------------------
 // Funciones para cargar y guardar la contraseña de configuración en NVS
 // ---------------------------------------------------------
 /**
  * @brief Carga la contraseña de configuración almacenada en NVS.
  *
  * Esta función abre la partición "storage" en modo de solo lectura y busca la clave 
  * "config_password". Si la contraseña está almacenada, se copia en el buffer proporcionado.
  *
  * @param buffer Puntero al buffer donde se almacenará la contraseña.
  * @param size Tamaño del buffer.
  * @return true Si la contraseña fue cargada correctamente.
  * @return false Si no se pudo cargar la contraseña (por ejemplo, si no existe).
  *
  * @note Futuras mejoras pueden incluir la encriptación/desencriptación de la contraseña
  *       antes de almacenarla o cargarla.
  * \todo Agregar encriptación/desencriptación de la contraseña antes de almacenarla o cargarla.
  */
 static bool load_config_password_from_nvs(char *buffer, size_t size) {
     nvs_handle_t my_handle;
     esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
     if (err != ESP_OK) { // Si no se pudo abrir NVS, retorna false.
         return false;
     }
     err = nvs_get_str(my_handle, "config_password", buffer, &size);
     ESP_LOGI(TAG, "Cargando contraseña de NVS: %s", buffer);
     nvs_close(my_handle);
     return (err == ESP_OK);
 }
 
 /**
  * @brief Guarda una nueva contraseña de configuración en NVS.
  *
  * Abre la partición "storage" en modo de lectura-escritura y almacena la contraseña
  * bajo la clave "config_password". Se realiza un commit para asegurar que los datos
  * se escriban de forma permanente.
  *
  * @param password Puntero a la nueva contraseña que se desea almacenar.
  *
  * @note Se puede extender esta función para incluir validaciones adicionales o
  *       la encriptación de la contraseña antes de almacenarla.
  * \todo Agregar validaciones adicionales o encriptación de la contraseña antes de almacenarla.
  */
 static void save_config_password_to_nvs(const char *password) {
     nvs_handle_t my_handle;
     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err == ESP_OK) {
         ESP_LOGI(TAG, "Guardando nueva contraseña en NVS: %s", password);
         nvs_set_str(my_handle, "config_password", password);
         nvs_commit(my_handle);
         nvs_close(my_handle);
     }
 }
 
 /**
  * @brief Callback para confirmar la contraseña en el diálogo de configuración.
  *
  * Esta función se invoca cuando el usuario presiona el botón de confirmación en el diálogo 
  * de contraseña. Compara la contraseña ingresada con la almacenada en NVS (o con la predeterminada 
  * si no existe). Si coinciden, procede a cerrar el diálogo y cambiar a la pantalla de configuración.
  * En caso contrario, muestra un mensaje de error.
  *
  * @param e Puntero al evento LVGL que dispara la acción.
  *
  * @note Se pueden incluir mejoras para limitar el número de intentos o notificar al usuario
  *       mediante otros mecanismos de alerta.
  * \todo Agregar límite de intentos o notificaciones adicionales al usuario y dejar registro de 
  *       intentos en memoria no volátil o SD.
  */
 static void confirm_password_event_cb(lv_event_t *e) {
     // Obtiene el diálogo a partir de user_data.
     lv_obj_t *dialog = lv_event_get_user_data(e);
     if (!lv_obj_is_valid(dialog)) return;
     
     // Se obtiene el textarea que contiene la contraseña ingresada (se asume posición 2).
     lv_obj_t *password_ta = lv_obj_get_child(dialog, 2);
     if (!lv_obj_is_valid(password_ta)) return;
     
     const char *entered_pass = lv_textarea_get_text(password_ta);
     if (entered_pass == NULL) {
         entered_pass = "";
     }
     ESP_LOGI(TAG, "Texto ingresado: '%s'", entered_pass);
     
     char stored_pass[32] = {0};
     // Intenta cargar la contraseña almacenada; si falla, se utiliza la contraseña predeterminada.
     if (!load_config_password_from_nvs(stored_pass, sizeof(stored_pass))) {
         strcpy(stored_pass, CONFIG_PASSWORD);
     }
     ESP_LOGI(TAG, "Contraseña almacenada: '%s'", stored_pass);
     
     // Si la contraseña ingresada coincide con la almacenada:
     if (strcmp(entered_pass, stored_pass) == 0) {
         // Deshabilitar el botón para evitar múltiples clics.
         lv_obj_t *btn = lv_event_get_target(e);
         if (lv_obj_is_valid(btn)) {
             lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
         }
         lv_obj_del(dialog);
         config_password_dialog = NULL;
         // Cambia a la pantalla de configuración general.
         switch_screen(create_general_config_screen);
     } else {
         // Si la contraseña es incorrecta, muestra un mensaje de error.
         lv_obj_t *error_msg = lv_msgbox_create(NULL, "Error", "Contraseña incorrecta.", NULL, true);
         lv_obj_center(error_msg);
     }
 }
 
 /**
  * @brief Callback para cerrar el diálogo de contraseña.
  *
  * Esta función se utiliza para eliminar el diálogo de contraseña cuando el usuario
  * decide cerrarlo (por ejemplo, presionando un botón de "cerrar").
  *
  * @param e Puntero al evento LVGL que dispara el cierre del diálogo.
  *
  * @note Se restablece la variable global config_password_dialog a NULL para indicar que
  *       ya no hay un diálogo activo.
  * 
  */
 static void close_config_password_dialog_cb(lv_event_t *e) {
     lv_obj_t *dialog = lv_event_get_user_data(e);
     if (lv_obj_is_valid(dialog)) {
         lv_obj_del(dialog);
     }
     config_password_dialog = NULL;
 }
 
 /**
  * @brief Muestra el diálogo para ingresar la contraseña de configuración.
  *
  * Esta función se encarga de crear y mostrar el diálogo en pantalla para que el usuario
  * ingrese su contraseña de configuración. Si ya existe un diálogo activo, simplemente lo trae
  * al frente.
  *
  * @note La interfaz del diálogo incluye un botón de cierre, un título, un campo de texto para
  *       la contraseña y un botón de confirmación. Se pueden agregar más elementos o personalizar
  *       el estilo en futuras versiones.
  * \todo Agregar más elementos o personalizar el estilo del diálogo de contraseña.
  */
 static void show_config_password_dialog(void) {
     if (config_password_dialog && lv_obj_is_valid(config_password_dialog)) {
         lv_obj_move_foreground(config_password_dialog);
         return;
     }
     
     // Crear el diálogo y asignarlo a la variable global.
     config_password_dialog = lv_obj_create(lv_scr_act());
     lv_obj_set_size(config_password_dialog, 300, 200);
     lv_obj_center(config_password_dialog);
     lv_obj_set_style_bg_color(config_password_dialog, lv_color_hex(0xFFFFFF), 0);
     lv_obj_set_style_border_width(config_password_dialog, 2, 0);
 
     // Botón de cierre en la esquina superior derecha.
     lv_obj_t *btn_close = lv_btn_create(config_password_dialog);
     lv_obj_set_size(btn_close, 30, 30);
     lv_obj_align(btn_close, LV_ALIGN_TOP_RIGHT, -5, 5);
     lv_obj_t *close_label = lv_label_create(btn_close);
     lv_label_set_text(close_label, LV_SYMBOL_CLOSE);
     lv_obj_center(close_label);
     lv_obj_add_event_cb(btn_close, close_config_password_dialog_cb, LV_EVENT_CLICKED, config_password_dialog);
 
     // Título del diálogo.
     lv_obj_t *title = lv_label_create(config_password_dialog);
     lv_label_set_text(title, "Ingrese contraseña:");
     lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
 
     // Campo de texto para ingresar la contraseña.
     lv_obj_t *password_ta = lv_textarea_create(config_password_dialog);
     lv_textarea_set_placeholder_text(password_ta, "Contraseña");
     lv_textarea_set_one_line(password_ta, true);
     lv_textarea_set_password_mode(password_ta, true);
     lv_obj_set_width(password_ta, 250);
     lv_obj_align(password_ta, LV_ALIGN_CENTER, 0, -10);
     lv_obj_add_event_cb(password_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
 
     // Botón de confirmación.
     lv_obj_t *btn_confirm = lv_btn_create(config_password_dialog);
     lv_obj_set_size(btn_confirm, 100, 40);
     lv_obj_align(btn_confirm, LV_ALIGN_BOTTOM_MID, 0, -10);
     lv_obj_t *btn_label = lv_label_create(btn_confirm);
     lv_label_set_text(btn_label, "Confirmar");
     lv_obj_center(btn_label);
     // Se pasa el diálogo como user_data para que el callback lo elimine.
     lv_obj_add_event_cb(btn_confirm, confirm_password_event_cb, LV_EVENT_CLICKED, config_password_dialog);
 }
 
 /**
  * @brief Callback para mostrar el diálogo de configuración de contraseña.
  *
  * Este callback se asocia al botón de configuración general y, al ser activado,
  * invoca la función para mostrar el diálogo de contraseña.
  *
  * @param e Puntero al evento LVGL disparado al presionar el botón de configuración.
  */
 static void btn_to_config_event_cb(lv_event_t *e) {
     show_config_password_dialog();
 }
 
 /**
  * @brief Callback para confirmar el cambio de contraseña de configuración.
  *
  * Esta función se invoca cuando el usuario desea cambiar la contraseña de configuración.
  * Recupera la contraseña actual y la nueva contraseña ingresadas, verifica que la contraseña
  * actual coincida con la almacenada y, si es correcta, guarda la nueva contraseña en NVS.
  *
  * @param e Puntero al evento LVGL disparado al presionar el botón de actualizar.
  *
  * @note Se puede ampliar para notificar al usuario mediante una interfaz más elaborada o
  *       agregar medidas de seguridad adicionales.
  * \todo Agregar notificaciones al usuario o medidas de seguridad adicionales al cambiar la contraseña.
  */
 static void password_change_confirm_cb(lv_event_t *e) {
     // Obtiene el contenedor que agrupa los elementos del cambio de contraseña.
     lv_obj_t *container = (lv_obj_t *) lv_event_get_user_data(e);
     if (!lv_obj_is_valid(container)) return;
     
     // Se asume el siguiente orden de elementos: título, campo de contraseña actual,
     // campo de nueva contraseña y botón de confirmación.
     lv_obj_t *current_pass_ta = lv_obj_get_child(container, 1);
     lv_obj_t *new_pass_ta = lv_obj_get_child(container, 2);
     
     const char *current_pass = lv_textarea_get_text(current_pass_ta);
     const char *new_pass = lv_textarea_get_text(new_pass_ta);
     if (current_pass == NULL) current_pass = "";
     if (new_pass == NULL) new_pass = "";
     
     char stored_pass[32] = {0};
     // Intenta cargar la contraseña almacenada; si falla, se utiliza la contraseña predeterminada.
     if (!load_config_password_from_nvs(stored_pass, sizeof(stored_pass))) {
         strcpy(stored_pass, CONFIG_PASSWORD);
     }
     ESP_LOGI(TAG, "Clave ingresada: '%s' | Almacenada: '%s'", current_pass, stored_pass);
     
     // Comprueba que la contraseña actual ingresada coincida con la almacenada.
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
     
     // Recarga la contraseña almacenada para confirmar el cambio.
     char new_stored[32] = {0};
     if (load_config_password_from_nvs(new_stored, sizeof(new_stored))) {
         ESP_LOGI(TAG, "Nueva contraseña almacenada: '%s'", new_stored);
     } else {
         ESP_LOGI(TAG, "No se pudo cargar la nueva contraseña");
     }
     
     // Muestra un mensaje de éxito.
     lv_obj_t *msg = lv_msgbox_create(NULL, "Éxito", "Contraseña actualizada.", NULL, true);
     lv_obj_center(msg);
     
     // Limpia los campos para que el contenedor siga siendo utilizable.
     lv_textarea_set_text(current_pass_ta, "");
     lv_textarea_set_text(new_pass_ta, "");
 }
 
 /**
  * @brief Crea el menú de cambio de contraseña.
  *
  * Esta función genera una interfaz que permite al usuario cambiar la contraseña de configuración.
  * Se crea un contenedor con un título, dos campos de texto (para la contraseña actual y la nueva)
  * y un botón de confirmación. El contenedor se utiliza en el tab correspondiente.
  *
  * @param parent Puntero al objeto LVGL (pestaña o contenedor) donde se integrará el menú.
  *
  * @note Se puede personalizar el estilo, agregar validaciones adicionales o incluso integrar
  *       autenticación biométrica en proyectos futuros.
  * \todo Agregar validaciones adicionales, personalizar el estilo o integrar autenticación más robusta.
  */
 static void create_password_change_menu(lv_obj_t *parent) {
     // Crear el contenedor principal del menú.
     lv_obj_t *container = lv_obj_create(parent);
     lv_obj_set_size(container, 400, 300);
     lv_obj_center(container);
     lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), 0);
     lv_obj_set_style_border_width(container, 2, 0);
     
     // Agregar el título.
     lv_obj_t *title = lv_label_create(container);
     lv_label_set_text(title, "Cambiar Clave de Configuración");
     lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
     
     // Crear el campo para la contraseña actual.
     lv_obj_t *current_pass_ta = lv_textarea_create(container);
     lv_textarea_set_placeholder_text(current_pass_ta, "Clave actual");
     lv_textarea_set_one_line(current_pass_ta, true);
     lv_textarea_set_password_mode(current_pass_ta, true);
     lv_obj_set_width(current_pass_ta, 300);
     lv_obj_align(current_pass_ta, LV_ALIGN_TOP_MID, 0, 50);
     lv_obj_add_event_cb(current_pass_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
     
     // Crear el campo para la nueva contraseña.
     lv_obj_t *new_pass_ta = lv_textarea_create(container);
     lv_textarea_set_placeholder_text(new_pass_ta, "Nueva clave");
     lv_textarea_set_one_line(new_pass_ta, true);
     lv_textarea_set_password_mode(new_pass_ta, true);
     lv_obj_set_width(new_pass_ta, 300);
     lv_obj_align(new_pass_ta, LV_ALIGN_TOP_MID, 0, 100);
     lv_obj_add_event_cb(new_pass_ta, textarea_event_handler, LV_EVENT_FOCUSED, NULL);
     
     // Crear el botón de confirmación.
     lv_obj_t *confirm_btn = lv_btn_create(container);
     lv_obj_set_size(confirm_btn, 100, 40);
     lv_obj_align(confirm_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
     lv_obj_t *btn_label = lv_label_create(confirm_btn);
     lv_label_set_text(btn_label, "Actualizar");
     lv_obj_center(btn_label);
     
     // Asocia el contenedor como user_data para usarlo en el callback.
     lv_obj_add_event_cb(confirm_btn, password_change_confirm_cb, LV_EVENT_CLICKED, container);
 }
 
 
 
 
 
 
 // ------------------------------------------------------------------------------
 // Funciones para el Manejo del Teclado en LVGL
 // ------------------------------------------------------------------------------
 
 /**
  * @brief Manejador de eventos del teclado virtual de LVGL.
  *
  * Esta función se invoca cada vez que ocurre un evento en el teclado virtual.
  * Se asegura de que el teclado esté en primer plano. Cuando se recibe un evento de
  * cancelación o de confirmación (READY), se obtiene el texto del campo asociado, se
  * imprime (para fines de depuración) y luego se oculta el teclado sin eliminarlo.
  *
  * @param event Puntero al evento LVGL.
  *
  * @note En futuros desarrollos se podría extender para notificar de forma más
  *       elaborada la finalización de la entrada o para gestionar diferentes tipos de
  *       interacción con el teclado.
  * \todo Extender la función para notificar de forma más elaborada la finalización de la entrada.
  *       Manejar adecuadamente los diferentes tipos de interacción con el teclado.
  */
 static void keyboard_event_handler(lv_event_t *event) {
     lv_event_code_t code = lv_event_get_code(event);
     lv_obj_t *keyboard = lv_event_get_target(event);
     lv_obj_move_foreground(keyboard);
 
     if (code == LV_EVENT_CANCEL || code == LV_EVENT_READY) {
         // Obtiene el campo de texto asociado al teclado.
         lv_obj_t *textarea = lv_keyboard_get_textarea(keyboard);
         if (textarea) {
             const char *text = lv_textarea_get_text(textarea);
             printf("Texto guardado: %s\n", text);
         }
         // Oculta el teclado sin eliminarlo para un posible reutilizado.
         lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
     }
 }
 
 /**
  * @brief Manejador de eventos para los campos de texto (textarea) en LVGL.
  *
  * Cuando un campo de texto recibe el foco, esta función verifica si ya existe
  * un teclado virtual. Si no existe, lo crea y lo asocia al campo. Luego se
  * asegura de que el teclado esté visible y listo para recibir entrada.
  *
  * @param event Puntero al evento LVGL.
  *
  * @note Esta función es fundamental para la entrada de datos en dispositivos sin
  *       teclado físico. Se puede extender para incluir diferentes estilos o métodos
  *       de entrada.
  * \todo Extender la función para incluir diferentes estilos o métodos de entrada.
  */
 static void textarea_event_handler(lv_event_t *event) {
     lv_event_code_t code = lv_event_get_code(event);
     lv_obj_t *textarea = lv_event_get_target(event);
 
     if (code == LV_EVENT_FOCUSED) {
         // Si aún no existe el teclado, se crea.
         if (!keyboard) {
             keyboard = lv_keyboard_create(lv_scr_act());
             lv_obj_add_event_cb(keyboard, keyboard_event_handler, LV_EVENT_ALL, NULL);
         }
         // Se asocia el teclado al textarea activo y se muestra.
         lv_keyboard_set_textarea(keyboard, textarea);
         lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
         lv_obj_set_style_opa(keyboard, LV_OPA_COVER, LV_PART_MAIN); // Se muestra con opacidad completa.
     }
 }
 
 // ------------------------------------------------------------------------------
 // Funciones para Comandos de POS (Operaciones de la terminal de punto de venta)
 // ------------------------------------------------------------------------------
 
 /**
  * @brief Envía el comando de inicialización (INIT) a la terminal de punto de venta.
  *
  * Esta función construye el comando INIT con su formato correspondiente y lo envía
  * mediante la función send_command_with_ack, que gestiona la verificación del ACK.
  *
  * @param e Puntero al evento LVGL que dispara el envío del comando.
  *
  * @note Este comando es el primer paso en la comunicación con el POS. En proyectos futuros,
  *       se podría ampliar la funcionalidad para incluir parámetros de configuración adicionales.
  * \todo Ampliar la funcionalidad para ilustrar su uso en la comunicación con el POS.
  */
 static void send_init_command(lv_event_t *e) {
     uint8_t init_command[] = {0x02, 0x30, 0x30, 0x37, 0x30, 0x03, 0x04}; // Comando INIT
     printf("Enviando comando INIT...\n");
     send_command_with_ack(init_command, sizeof(init_command));
 }
 
 /**
  * @brief Envía la respuesta de inicialización (INIT RESP) a la terminal de punto de venta.
  *
  * Esta función prepara el comando INIT RESP, el cual es la respuesta esperada para
  * confirmar la inicialización. Luego utiliza send_command_with_ack para enviarlo y
  * validar la recepción del ACK.
  *
  * @param e Puntero al evento LVGL que dispara el envío del comando.
  *
  * \todo Ampliar la funcionalidad para ilustrar su uso en la comunicación con el POS.
  */
 static void send_init_response(lv_event_t *e) {
     uint8_t init_response[] = {0x02, 0x30, 0x30, 0x37, 0x30, 0x03, 0x0B}; // Comando INIT RESP
     printf("Enviando comando INIT RESP...\n");
     send_command_with_ack(init_response, sizeof(init_response));
 }
 
 /**
  * @brief Envía el comando de polling al POS para verificar la conexión.
  *
  * Este comando se utiliza para verificar de forma periódica la conectividad con el POS.
  * Se envía mediante send_command_with_ack para asegurar la correcta recepción del comando.
  *
  * @param e Puntero al evento LVGL que dispara el envío del comando.
  *
  * @note En futuras implementaciones, se podría incluir la lógica para manejar diferentes
  *       intervalos de polling o ajustar dinámicamente los parámetros de la comunicación.
  * \todo Implementar lógica para manejar diferentes intervalos de polling o ajustar dinámicamente
  *       los parámetros de la comunicación. Actualmente se envía el comando POLLING, no se
  *       maneja la respuesta.
  */
 static void send_polling_command(lv_event_t *e) {
     uint8_t polling_command[] = {0x02, 0x30, 0x31, 0x30, 0x30, 0x03, 0x02}; // Comando POLLING
     printf("Enviando comando POLLING...\n");
     send_command_with_ack(polling_command, sizeof(polling_command));
 }
 
 /**
  * @brief Envía el comando de carga de claves (LOAD KEYS) al POS.
  *
  * Este comando solicita al POS la carga de claves. Se calcula el LRC del comando antes
  * de enviarlo para garantizar la integridad de los datos. 
  *
  * @param e Puntero al evento LVGL que dispara el envío del comando.
  *
  * @note La función utiliza send_command_with_ack para enviar el comando y gestionar la respuesta.
  */
 static void send_loadkeys_comand(lv_event_t *e) {
     uint8_t loadkeys_command[] = {0x02, '0', '8', '0', '0', 0x03};
     loadkeys_command[5] = calculate_lrc(loadkeys_command + 1, 5);
     printf("Enviando comando LOAD KEYS...\n");
     send_command_with_ack(loadkeys_command, sizeof(loadkeys_command));
 }
 
 /**
  * @brief Crea la interfaz para el tab de comandos POS.
  *
  * Esta función genera una interfaz en forma de pestaña que contiene botones para
  * enviar diferentes comandos al POS (POLLING, INIT, INIT RESP y LOAD KEYS).
  * Se utiliza un contenedor con layout flex para distribuir los botones uniformemente.
  *
  * @param parent Puntero al objeto LVGL que actuará como contenedor del tab.
  *
  * @note Esta función facilita la extensión futura de la interfaz de comandos, permitiendo
  *       agregar nuevos botones o modificar el layout de forma sencilla.
  * \todo Ampliar la funcionalidad para incluir más comandos o personalizar la interfaz de comandos
  *       con estilos y animaciones que permitan interpretar visualmente el estado de la comunicación.
  */
 static void create_operation_cmd_pos_tab(lv_obj_t *parent) {
     // Crear y configurar un contenedor centrado sin scroll.
     lv_obj_t *container = lv_obj_create(parent);
     lv_obj_set_size(container, 500, 300);
     lv_obj_center(container);
     lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
     
     // Configurar layout flex para distribuir botones en fila.
     lv_obj_set_layout(container, LV_LAYOUT_FLEX);
     lv_obj_set_style_flex_flow(container, LV_FLEX_FLOW_ROW, 0);
     lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
     lv_obj_set_style_pad_all(container, 10, 0);
 
     // Botón para enviar el comando POLLING.
     lv_obj_t *btn_polling = lv_btn_create(container);
     lv_obj_set_size(btn_polling, 100, 50);
     lv_obj_add_event_cb(btn_polling, send_polling_command, LV_EVENT_CLICKED, NULL);
     lv_obj_t *label_polling = lv_label_create(btn_polling);
     lv_label_set_text(label_polling, "Polling");
     lv_obj_center(label_polling);
 
     // Botón para enviar el comando INIT.
     lv_obj_t *btn_init = lv_btn_create(container);
     lv_obj_set_size(btn_init, 100, 50);
     lv_obj_add_event_cb(btn_init, send_init_command, LV_EVENT_CLICKED, NULL);
     lv_obj_t *label_init = lv_label_create(btn_init);
     lv_label_set_text(label_init, "Init");
     lv_obj_center(label_init);
 
     // Botón para enviar la respuesta de INIT (INIT RESP).
     lv_obj_t *btn_init_resp = lv_btn_create(container);
     lv_obj_set_size(btn_init_resp, 100, 50);
     lv_obj_add_event_cb(btn_init_resp, send_init_response, LV_EVENT_CLICKED, NULL);
     lv_obj_t *label_init_resp = lv_label_create(btn_init_resp);
     lv_label_set_text(label_init_resp, "Init Resp");
     lv_obj_center(label_init_resp);
 
     // Botón para enviar el comando LOAD KEYS.
     lv_obj_t *btn_loadkeys = lv_btn_create(container);
     lv_obj_set_size(btn_loadkeys, 100, 50);
     lv_obj_add_event_cb(btn_loadkeys, send_loadkeys_comand, LV_EVENT_CLICKED, NULL);
     lv_obj_t *label_loadkeys = lv_label_create(btn_loadkeys);
     lv_label_set_text(label_loadkeys, "Load Keys");
     lv_obj_center(label_loadkeys);
 }
 
 
 
 // ------------------------------------------------------------------------------
 // Estructura principal del UI con LVGL
 // ------------------------------------------------------------------------------
 
 /**
  * @brief Crea la estructura principal de la interfaz de usuario.
  *
  * Esta función inicializa el contenedor principal que abarca toda la pantalla,
  * y organiza en él dos áreas principales:
  *   - Header: una barra superior que contiene el reloj y el icono de Wi‑Fi.
  *   - Área de Contenido: donde se cargarán las diferentes pantallas de la aplicación.
  *
  * Los elementos se configuran para que no sean desplazables y se ajusten al tamaño
  * de la pantalla. Esta función es el punto de partida para definir la arquitectura
  * visual de la aplicación y se puede ampliar para incluir otros elementos globales.
  *
  * @note En futuros desarrollos se podrá incluir la personalización del header (p. ej.,
  *       agregar notificaciones o menús contextuales) o extender el área de contenido
  *       con nuevos contenedores de navegación.
  * \todo Personalizar el header con notificaciones o menús contextuales a la conexión con el POS.
  */
 void create_main_structure(void) {
     // Crea el contenedor principal que ocupa toda la pantalla.
     lv_obj_t *main_container = lv_obj_create(lv_scr_act());
     lv_obj_set_size(main_container, LV_HOR_RES, LV_VER_RES);
     lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);
 
     // --- Header (Barra de notificaciones con reloj) ---
     // Se crea el header y se posiciona en la parte superior, con un fondo oscuro.
     global_header = lv_obj_create(main_container);
     lv_obj_set_size(global_header, LV_HOR_RES, 30);  // Altura configurable según necesidades.
     lv_obj_align(global_header, LV_ALIGN_TOP_MID, 0, -28);
     lv_obj_set_style_bg_color(global_header, lv_color_hex3(0x333), 0);
     lv_obj_set_style_pad_all(global_header, 0, 0);
 
     // Se crea el label para el reloj y se centra en el header.
     global_clock_label = lv_label_create(global_header);
     lv_label_set_text(global_clock_label, "00:00");  // Valor inicial, se actualizará dinámicamente.
     lv_obj_set_style_text_color(global_clock_label, lv_color_white(), 0);
     lv_obj_align(global_clock_label, LV_ALIGN_CENTER, 0, 0);
 
     // Se crea un label para el icono de Wi‑Fi, inicialmente en estado desconectado.
     wifi_status_icon = lv_label_create(global_header);
     lv_label_set_text(wifi_status_icon, LV_SYMBOL_CLOSE);
     lv_obj_set_style_text_color(wifi_status_icon, lv_color_white(), 0);
     lv_obj_align(wifi_status_icon, LV_ALIGN_RIGHT_MID, 0, 0);
 
     // --- Área de Contenido ---
     // Se crea el contenedor donde se inyectarán las pantallas y se configura para no desplazarse.
     global_content = lv_obj_create(main_container);
     lv_obj_set_style_pad_all(global_content, 0, 0);
     lv_obj_set_style_border_width(global_content, 0, 0);
     lv_obj_set_style_border_color(global_content, lv_color_black(), 0);
     lv_obj_set_size(global_content, LV_HOR_RES, LV_VER_RES - 20);
     lv_obj_align(global_content, LV_ALIGN_BOTTOM_MID, 0, 25);
     lv_obj_clear_flag(global_content, LV_OBJ_FLAG_SCROLLABLE);
 }
 
 
 
 // ------------------------------------------------------------------------------
 // Funciones para el Manejo del Reloj y Epoch Time
 // ------------------------------------------------------------------------------
 
 /**
  * @brief Guarda el valor actual del epoch en la memoria no volátil (NVS).
  *
  * Esta función abre la partición "storage" en modo lectura/escritura y almacena
  * el valor del epoch (como un entero de 32 bits) bajo la clave "last_epoch". Si
  * se detecta algún error durante el proceso, se maneja a través de las macros de
  * verificación de errores de ESP-IDF.
  *
  * @param epoch Valor de tiempo (epoch) a guardar.
  *
  * @note Se recomienda llamar a esta función periódicamente para mantener
  *       persistente el tiempo actual entre reinicios.
  * \todo Implementar un mecanismo de guardado periódico para evitar pérdida de datos 
  *       ni tampoco un execeso de escrituras.
  */
 void save_epoch(time_t epoch) {
     nvs_handle_t my_handle;
     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err == ESP_OK) {
         // Se guarda el epoch como un valor de 32 bits.
         err = nvs_set_u32(my_handle, "last_epoch", (uint32_t)epoch);
         if (err == ESP_OK) {
             nvs_commit(my_handle);
         }
         nvs_close(my_handle);
     }
 }
 
 /**
  * @brief Recupera el último valor del epoch almacenado en la NVS.
  *
  * Esta función intenta abrir la partición "storage" en modo solo lectura y
  * recuperar el valor asociado a la clave "last_epoch". Si la clave no existe o
  * ocurre algún error, se utiliza un valor predeterminado.
  *
  * @return time_t Último epoch almacenado o un valor predeterminado en caso de error.
  *
  * @note Esta función es crucial para la persistencia del tiempo entre reinicios.
  */
 time_t load_epoch(void) {
     nvs_handle_t my_handle;
     uint32_t stored_epoch = 0;
     esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
     if (err == ESP_OK) {
         if (nvs_get_u32(my_handle, "last_epoch", &stored_epoch) != ESP_OK) {
             // Valor por defecto: 19/02/2025 17:00:00 (epoch 1739984400)
             stored_epoch = 1739984400;
         }
         nvs_close(my_handle);
     } else {
         // En caso de error, se utiliza el valor por defecto.
         stored_epoch = 1739984400;
     }
     return (time_t)stored_epoch;
 }
 
 /**
  * @brief Actualiza el reloj de la interfaz y sincroniza el epoch.
  *
  * Esta función es llamada periódicamente mediante un timer de LVGL. Primero verifica
  * si SNTP está sincronizado; de ser así, se obtiene la hora real. En caso contrario,
  * se incrementa manualmente el valor del epoch simulado. Luego, se formatea la fecha y
  * hora para actualizar el label del header y se guarda el nuevo epoch en la NVS.
  *
  * @param timer Puntero al timer de LVGL que invoca esta función.
  *
  * @note Esta función es esencial para mantener la información temporal actualizada en
  *       la UI y la persistencia del tiempo. Puede ampliarse para manejar ajustes de zona horaria
  *       o sincronización más compleja.
  */
 static void update_clock_cb(lv_timer_t * timer) {
     time_t now;
     // Verifica si SNTP está sincronizado y utiliza la hora real.
     if (sntp_initialized && sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
         ESP_LOGI(TAG, "SNTP sincronizado");
         time(&now);
         simulated_epoch = now;  // Sincroniza el epoch simulado con la hora real.
     } else {
         // Si SNTP no está disponible, incrementa manualmente el epoch.
         simulated_epoch++;
         now = simulated_epoch;
     }
     
     struct tm timeinfo;
     localtime_r(&now, &timeinfo);
     char datetime_str[20];  // Asegúrate de que el buffer es suficientemente grande.
     strftime(datetime_str, sizeof(datetime_str), "%d/%m/%Y %H:%M:%S", &timeinfo);
     
     // Actualiza el label del header con la fecha y hora formateadas.
     lv_label_set_text(global_clock_label, datetime_str);
     
     // Guarda el nuevo valor del epoch en la NVS para persistencia.
     save_epoch(simulated_epoch);
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 /**
  * @brief Función principal de la aplicación.
  *
  * Esta función es el punto de entrada de la aplicación y se encarga de inicializar
  * y configurar los componentes fundamentales del sistema. Entre sus tareas se incluyen:
  *
  * 1. **Inicialización de la memoria no volátil (NVS):**
  *    - Se configura para almacenar datos persistentes, lo cual es esencial para guardar
  *      configuraciones y el estado del sistema entre reinicios.
  *
  * 2. **Configuración del entorno temporal:**
  *    - Se establece la zona horaria para Chile utilizando `setenv` y `tzset`, lo que permite
  *      que el sistema maneje correctamente la fecha y hora locales.
  *
  * 3. **Inicialización de la conectividad Wi‑Fi:**
  *    - Se configura el servicio Wi‑Fi, incluyendo el driver, modos de operación y la
  *      asignación de eventos necesarios para gestionar la conexión a redes inalámbricas.
  *
  * 4. **Inicialización de la comunicación UART:**
  *    - Se configura el UART para la comunicación serial, facilitando el intercambio de datos
  *      con otros dispositivos o para fines de depuración.
  *
  * 5. **Carga del tiempo (epoch):**
  *    - Se recupera el valor del epoch almacenado en la NVS o se utiliza un valor predeterminado,
  *      lo que permite la persistencia del tiempo a través de reinicios.
  *
  * 6. **Inicialización de la interfaz gráfica con LVGL:**
  *    - Se configuran los buffers y controladores de LVGL.
  *    - Se inicializa el panel LCD RGB y se registran los callbacks de eventos para el LCD.
  *    - Se inicializa el bus I2C y el controlador táctil (touch) asociado al panel.
  *    - Se crea y registra el driver de LVGL, y se inicia la tarea de LVGL que se encarga de:
  *         - Incrementar el tick de LVGL.
  *         - Manejar las interacciones táctiles.
  *         - Refrescar la pantalla.
  *
  * 7. **Creación de tareas para la recepción y procesamiento de datos UART:**
  *    - Se asigna una tarea para recibir bytes por UART en el Core 0.
  *    - Se asigna otra tarea para procesar los comandos recibidos, sin fijar un núcleo específico.
  *
  * 8. **Inicialización y actualización de la interfaz de usuario:**
  *    - Se informa mediante logs que el sistema y la interfaz están listos.
  *    - Se bloquea el acceso a LVGL para realizar cambios de forma segura.
  *    - Se limpia la pantalla actual y se crea la estructura principal del UI, que incluye:
  *         - El header (barra superior con reloj e icono de Wi‑Fi).
  *         - El área de contenido para cargar las diferentes pantallas de la aplicación.
  *    - Se carga la pantalla principal (o la pantalla deseada) en el área de contenido.
  *    - Se configura un timer para actualizar el reloj cada segundo.
  *
  * 9. **Liberación del bloqueo de LVGL:**
  *    - Se libera el mutex para permitir que otras tareas interactúen con la interfaz gráfica.
  *
  * @note Esta función se puede ampliar en el futuro para incorporar nuevas funcionalidades,
  *       tales como, manejo avanzado de eventos, integración de sensores adicionales, procesamiento
  *       de datos de transacciones para su posterior registro en una base de datos, o mejoras en la
  *       interfaz gráfica para incluir más pantallas y elementos interactivos.
  * 
  * \todo Ampliar la función para incluir más funcionalidades como manejo avanzado de eventos, integración
  *       de sensores adicionales, procesamiento de datos de transacciones y mejoras en la interfaz gráfica.
  * \todo Se puede extender para incluir implementaciones de seguridad, gestión de memoria y optimizaciones.
  * \todo Incluir manejo de bluetooth para incorporar impresión de ticket de pago.
  * \todo Incluir lógica del modbus para el control de accionadores y sensores.
  * \todo Diseñar una rutina que transmita periódicamente por MQTT los datos de las transacciones.
  * \todo Implementar una función de reconexión wifi después de un reinicio o tiempo sin conexión. Que escanee
  *       las redes disponibles y se conecte a la más fuerte conocida.
  * 
  */
 void app_main(void)
 {
     // Inicializa la memoria no volátil (NVS) para almacenar datos persistentes.
     init_nvs();
 
     // Configura la zona horaria para Chile y actualiza el entorno de tiempo.
     setenv("TZ", "CLT3CLST,m10.1.0/0,m3.1.0/0", 1);
     tzset();
 
     // Inicializa el servicio Wi‑Fi (configura driver, modos y eventos).
     wifi_service_init();
 
     // Inicializa la comunicación UART para el intercambio de datos seriales.
     init_uart();
 
     // Carga el valor del epoch (tiempo) previamente guardado o usa un valor por defecto.
     simulated_epoch = load_epoch();
     
     // Declara buffers y estructuras de controlador para LVGL (interfaz gráfica).
     static lv_disp_draw_buf_t disp_buf;
     static lv_disp_drv_t disp_drv;
     
     // Inicializa el panel LCD RGB y registra los callbacks de eventos del panel.
     esp_lcd_panel_handle_t panel_handle = init_lcd_panel();
     register_lcd_event_callbacks(panel_handle, &disp_drv);
     
     // Inicializa el bus I2C y el controlador táctil (touch) asociado al panel LCD.
     init_i2c();
     esp_lcd_touch_handle_t tp = init_touch(panel_handle);
     
     // Inicializa el driver de LVGL, creando la pantalla de visualización.
     lv_disp_t *disp = init_lvgl(panel_handle);
     
     // Inicia la tarea de LVGL (incluye tick timer, manejo de touch y refresco de pantalla).
     start_lvgl_task(disp, tp);
     
     // Crea la tarea para recibir datos por UART (asignada al core 0).
     xTaskCreatePinnedToCore(uart_RX_task, "uart_RX_task", 4096, NULL, 2, NULL, 0); // Core 0
 
     // Crea la tarea encargada de procesar los comandos recibidos por UART (sin asignación fija de núcleo).
     xTaskCreate(command_processing_task, "command_processing_task", 4096, NULL, 1, NULL);
 
     // Registro informativo para indicar que el sistema y la interfaz se han inicializado correctamente.
     ESP_LOGI(TAG, "Sistema inicializado. Interfaz lista.");
     
     // Bloquea el acceso a LVGL para realizar cambios en la interfaz de forma segura.
     if(lvgl_lock(-1)) {
         // Limpia la pantalla actual.
         lv_obj_clean(lv_scr_act());
         
         // Crea la estructura principal de la UI, que incluye el header (barra superior) y el área de contenido.
         create_main_structure();
         
         // Carga la pantalla principal (u otra pantalla deseada) en el área de contenido.
         switch_screen(create_main_screen);
 
         // Crea un timer que llama periódicamente a update_clock_cb para actualizar el reloj cada segundo.
         lv_timer_create(update_clock_cb, 1000, NULL);
 
         // Libera el bloqueo de LVGL tras actualizar la UI.
         lvgl_unlock();
     }
 }
 
 