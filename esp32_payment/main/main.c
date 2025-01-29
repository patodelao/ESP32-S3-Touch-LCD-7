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
#include "driver/uart.h"

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

#define UART_NUM 1  
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define UART_BUFFER_SIZE 1024
#define MAX_COMMANDS 100


static lv_obj_t *received_label;

typedef struct {
    char command[5];
    char monto[10];
    char ticket_number[21];
    char impresion[1];
    char enviar_msj[1];
    char codigo_respuesta[3];
    char comercio[13];
    char terminal_id[9];
    char autorizacion[16];
    char ultimos_4[5];
    char operacion[16];
    char tipo_tarjeta[16];
    char fecha_contable[9];
    char numero_cuenta[16];
    char abreviacion_tarjeta[16];
    char fecha_transaccion[9];
    char hora_transaccion[7];
} CommandData;

CommandData command_history[MAX_COMMANDS];
size_t command_count = 0;







// Declaraciones de funciones de inicialización del sistema
void init_nvs();
static void init_uart(void);
esp_lcd_panel_handle_t init_lcd_panel();
void init_i2c();
esp_lcd_touch_handle_t init_touch(esp_lcd_panel_handle_t panel_handle);
lv_disp_t *init_lvgl(esp_lcd_panel_handle_t panel_handle);
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp);
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv);
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT       -1
#define PIN_NUM_HSYNC          46
#define PIN_NUM_VSYNC          3
#define PIN_NUM_DE             5
#define PIN_NUM_PCLK           7
#define PIN_NUM_DATA0          14 // B3
#define PIN_NUM_DATA1          38 // B4
#define PIN_NUM_DATA2          18 // B5
#define PIN_NUM_DATA3          17 // B6
#define PIN_NUM_DATA4          10 // B7
#define PIN_NUM_DATA5          39 // G2
#define PIN_NUM_DATA6          0 // G3
#define PIN_NUM_DATA7          45 // G4
#define PIN_NUM_DATA8          48 // G5
#define PIN_NUM_DATA9          47 // G6
#define PIN_NUM_DATA10         21 // G7
#define PIN_NUM_DATA11         1  // R3
#define PIN_NUM_DATA12         2  // R4
#define PIN_NUM_DATA13         42 // R5
#define PIN_NUM_DATA14         41 // R6
#define PIN_NUM_DATA15         40 // R7
#define PIN_NUM_DISP_EN        -1

// The pixel number in horizontal and vertical
#define LCD_H_RES              800
#define LCD_V_RES              480

#if CONFIG_DOUBLE_FB
#define LCD_NUM_FB             2
#else
#define LCD_NUM_FB             1
#endif // CONFIG_DOUBLE_FB

#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

#define MAX_RETRIES 3  // Número máximo de reintentos ACK / NAK 
#define TIMEOUT_MS 10000  // Tiempo de espera para ACK (10 segundos)
static esp_timer_handle_t ack_timer = NULL;
static uint8_t retry_count = 0;
static char last_transaction_command[256];  // Último comando enviado

static SemaphoreHandle_t lvgl_mux = NULL;   
static QueueHandle_t uart_event_queue = NULL; // cola para eventos de UART
static QueueHandle_t command_queue = NULL; // cola para procesar comandos

static lv_obj_t *textarea;



// Función para inicializar UART
static void init_uart(void) {
    printf("Inicializando UART %d...\n", UART_NUM);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    // Configuración de UART
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Instalar el driver UART
    esp_err_t ret = uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_event_queue, 0);
    if (ret == ESP_OK) {
        printf("Driver UART instalado correctamente.\n");
    } else {
        printf("Error al instalar el driver UART: %s\n", esp_err_to_name(ret));
    }
}


// Almacenar comandos en el historial
void store_command(const CommandData *command) {
    if (command_count < MAX_COMMANDS) {
        command_history[command_count++] = *command;
    } else {
        uart_write_bytes(UART_NUM, "Historial lleno. Comando descartado.\n", 37);
    }
}


void process_field(const char *field, int field_number, const char *command, CommandData *command_data) {
    if (strcmp(command, "0200") == 0) {
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->monto, field, sizeof(command_data->monto) - 1);
                break;
            case 3:
                strncpy(command_data->ticket_number, field, sizeof(command_data->ticket_number) - 1);
                break;
            case 4:
                strncpy(command_data->impresion, field, sizeof(command_data->impresion) - 1);
                break;
            case 5:
                strncpy(command_data->enviar_msj, field, sizeof(command_data->enviar_msj) - 1);
                break;
        }
    } else if (strcmp(command, "0210") == 0) {
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3:
                strncpy(command_data->comercio, field, sizeof(command_data->comercio) - 1);
                break;
            case 4:
                strncpy(command_data->terminal_id, field, sizeof(command_data->terminal_id) - 1);
                break;
            case 5:
                strncpy(command_data->ticket_number, field, sizeof(command_data->ticket_number) - 1);
                break;
            case 6:
                strncpy(command_data->autorizacion, field, sizeof(command_data->autorizacion) - 1);
                break;
            case 7:
                strncpy(command_data->monto, field, sizeof(command_data->monto) - 1);
                break;
            case 8:
                strncpy(command_data->ultimos_4, field, sizeof(command_data->ultimos_4) - 1);
                break;
            case 9:
                strncpy(command_data->operacion, field, sizeof(command_data->operacion) - 1);
                break;
            case 10:
                strncpy(command_data->tipo_tarjeta, field, sizeof(command_data->tipo_tarjeta) - 1);
                break;
            case 11:
                strncpy(command_data->fecha_contable, field, sizeof(command_data->fecha_contable) - 1);
                break;
            case 12:
                strncpy(command_data->numero_cuenta, field, sizeof(command_data->numero_cuenta) - 1);
                break;
            case 13:
                strncpy(command_data->abreviacion_tarjeta, field, sizeof(command_data->abreviacion_tarjeta) - 1);
                break;
            case 14:
                strncpy(command_data->fecha_transaccion, field, sizeof(command_data->fecha_transaccion) - 1);
                break;
            case 15:
                strncpy(command_data->hora_transaccion, field, sizeof(command_data->hora_transaccion) - 1);
                break;
        }
    }
}


// Función para inicializar el sistema de almacenamiento no volátil
uint8_t calculate_lrc(const uint8_t *data, size_t length) {
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++) {
        lrc ^= data[i];
    }
    return lrc;
}

// Generar una cadena alfanumérica aleatoria de la longitud especificada
char* generateAlphanumericString(size_t length) {
    const char charset[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char *dest = malloc(length + 1);
    if (dest) {
        --length; // ajustar para el carácter nulo de fin de cadena
        for (size_t n = 0; n < length; n++) {
            int key = rand() % (int)(sizeof charset - 1);
            dest[n] = charset[key];
        }
        dest[length] = '\0';
    }
    return dest;
}





// Función de callback del timer (se activa si no llega el ACK a tiempo)
static void ack_timeout_callback(void *arg) {
    if (retry_count < MAX_RETRIES) {
        printf("No se recibió ACK. Reintentando envío #%d...\n", retry_count + 1);
        retry_count++;
        uart_write_bytes(UART_NUM, last_transaction_command, strlen(last_transaction_command));
        esp_timer_start_once(ack_timer, TIMEOUT_MS * 1000);  // Reiniciar el timer pasados 10 seg
    } else {
        printf("Error: No se recibió ACK después de %d intentos. Cancelando.\n", MAX_RETRIES);
        retry_count = 0;  // Resetear el contador de reintentos
    }
}

// **Inicializar el timer**
void init_ack_timer() {
    esp_timer_create_args_t timer_args = {
        .callback = &ack_timeout_callback,
        .name = "ack_timer"
    };
    esp_timer_create(&timer_args, &ack_timer);
}


// Función para generar y enviar un comando de transacción
void generate_transaction_command(const char *monto) {
    uint8_t STX = 0x02;
    uint8_t ETX = 0x03;

    char *N_ticket = generateAlphanumericString(20);
    const char *codigo_cmd = "0200";
    const char *impresion_campo = "1";
    const char *msj_status = "1";

    char command[256];
    snprintf(command, sizeof(command), "%s|%s|%s|%s|%s", codigo_cmd, monto, N_ticket, impresion_campo, msj_status);

    size_t command_length = strlen(command) + 3;  // ETX + LRC
    uint8_t *formatted_command = malloc(command_length);
    if (!formatted_command) {
        printf("Error: No se pudo asignar memoria.\n");
        free(N_ticket);
        return;
    }

    size_t index = 0;
    memcpy(&formatted_command[index], command, strlen(command));
    index += strlen(command);
    formatted_command[index++] = ETX;  // Agregar ETX

    uint8_t lrc = calculate_lrc(formatted_command, index);
    formatted_command[index++] = lrc;  // Agregar LRC

    // Guardar el comando enviado
    strncpy(last_transaction_command, (const char *)formatted_command, sizeof(last_transaction_command) - 1);

    // Enviar el comando
    uart_write_bytes(UART_NUM, (const char *)formatted_command, index);

    // Iniciar el timer de 10 segundos
    retry_count = 0;
    esp_timer_start_once(ack_timer, TIMEOUT_MS * 1000);

    free(N_ticket);
    free(formatted_command);
}

// Evento para confirmar y generar el comando
static void confirm_mount_event_handler(lv_event_t *e) {
    const char *amount = lv_textarea_get_text(textarea);

    if (strlen(amount) > 0) {
        generate_transaction_command(amount);
        lv_textarea_set_text(textarea, "");
        printf("Monto confirmado: %s\n", amount);
    } else {
        printf("El campo de monto está vacío.\n");
    }
}

// Evento para procesar la entrada del teclado
static void keypad_event_handler(lv_event_t *e) {
    lv_obj_t *btnm = lv_event_get_target(e);
    const char *txt = lv_btnmatrix_get_btn_text(btnm, lv_btnmatrix_get_selected_btn(btnm));

    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        lv_textarea_del_char(textarea);
    } else if (strcmp(txt, LV_SYMBOL_NEW_LINE) == 0) {
        confirm_mount_event_handler(NULL);
    } else {
        lv_textarea_add_text(textarea, txt);
    }
}

// Crear la interfaz de la botonera numérica
void create_keypad(void) {
    // Crear un textarea para mostrar el monto ingresado
    textarea = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(textarea, true);
    lv_obj_align(textarea, LV_ALIGN_TOP_MID, 0, 10);
    lv_textarea_set_placeholder_text(textarea, "Ingrese monto");

    // Mapa de la botonera
    static const char *btnm_map[] = {
        "1", "2", "3", "\n",
        "4", "5", "6", "\n",
        "7", "8", "9", "\n",
        LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_NEW_LINE, ""
    };

    // Crear la botonera
    lv_obj_t *btnm = lv_btnmatrix_create(lv_scr_act());
    lv_obj_set_size(btnm, 400, 300);
    lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_btnmatrix_set_map(btnm, btnm_map);
    lv_obj_add_event_cb(btnm, keypad_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
}

// **Procesar los comandos recibidos por UART**
bool process_uart_data(const uint8_t *data, size_t length) {
    uint8_t temp_message[UART_BUFFER_SIZE] = {0};
    size_t temp_index = 0;
    char command[5] = {0};
    CommandData current_command = {0};

    for (size_t i = 0; i < length; i++) {
        if (data[i] == 0x02) { // Inicio del comando
            temp_index = 0;
            memset(temp_message, 0, sizeof(temp_message));
            memset(&current_command, 0, sizeof(current_command));
        } else if (data[i] == 0x03) { // Fin del comando
            if (temp_index == 0 || temp_index >= UART_BUFFER_SIZE - 1) {
                return false; // Comando vacío o demasiado largo
            }

            uint8_t lrc_calculated = calculate_lrc(temp_message, temp_index);
            if (i + 1 >= length) {
                return false; // LRC fuera de rango
            }
            uint8_t lrc_received = data[i + 1];

            if (lrc_calculated != lrc_received) {
                return false; // LRC incorrecto
            }

            temp_message[temp_index] = '\0';
            char *field = strtok((char *)temp_message, "|");

            int field_count = 0;
            while (field != NULL) {
                if (field_count == 0) {
                    strncpy(command, field, sizeof(command) - 1);
                }
                process_field(field, ++field_count, command, &current_command);
                field = strtok(NULL, "|");
            }

            store_command(&current_command);
            return true; // Comando procesado correctamente
        } else { // Datos intermedios
            if (temp_index >= sizeof(temp_message) - 1) {
                return false; // Buffer lleno, comando inválido
            }
            temp_message[temp_index++] = data[i];
        }
    }

    return false; // Si el bucle termina sin encontrar un fin de comando
}



// **Manejar respuestas UART**
static void command_processing_task(void *param) {
    uint8_t command_buffer[UART_BUFFER_SIZE];

    while (1) {
        if (xQueueReceive(command_queue, command_buffer, portMAX_DELAY)) {
            printf("Procesando comando: %s\n", command_buffer);

            // Si es ACK, detener el timer y resetear el contador
            if (strncmp((char *)command_buffer, "ACK", 3) == 0) {
                printf("ACK recibido. Deteniendo timer.\n");
                esp_timer_stop(ack_timer);
                retry_count = 0;  // Resetear contador
            } else if (strncmp((char *)command_buffer, "NAK", 3) == 0) {
                printf("NAK recibido. Reintentando...\n");
                ack_timeout_callback(NULL);  // Simular un timeout inmediato
            }
        }
    }
}




// **Task de recepción de datos UART**
static void uart_RX_task(void *param) {
    uint8_t data_buffer[UART_BUFFER_SIZE] = {0};
    size_t bytes_read = 0;

    while (1) {
        uart_event_t event;
        if (xQueueReceive(uart_event_queue, &event, pdMS_TO_TICKS(10))) {
            switch (event.type) {
                case UART_DATA: {
                    while ((bytes_read = uart_read_bytes(UART_NUM, data_buffer, sizeof(data_buffer) - 1, pdMS_TO_TICKS(10))) > 0) {
                        data_buffer[bytes_read] = '\0'; 
                        printf("Mensaje recibido: %s\n", data_buffer);

                        if (!xQueueSend(command_queue, data_buffer, pdMS_TO_TICKS(100))) {
                            printf("Error: no se pudo enviar el mensaje a la cola\n");
                        }
                    }
                    break;
                }
                default:
                    printf("Evento UART no manejado: %d\n", event.type);
                    break;
            }
        }

        while ((bytes_read = uart_read_bytes(UART_NUM, data_buffer, sizeof(data_buffer) - 1, 0)) > 0) {
            data_buffer[bytes_read] = '\0'; 
            printf("Procesando mensaje fuera de evento: %s\n", data_buffer);

            if (!xQueueSend(command_queue, data_buffer, pdMS_TO_TICKS(100))) {
                printf("Error: no se pudo enviar el mensaje a la cola\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





























static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
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
/*
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
*/
static void lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
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

    init_ack_timer();

    // Inicializar UART
    init_uart();


    command_queue = xQueueCreate(10, UART_BUFFER_SIZE);
    if (command_queue == NULL) {
    printf("Error: no se pudo crear la cola de comandos\n");
    }


    // Iniciar la tarea de eventos UART en el Core 0
    xTaskCreatePinnedToCore(uart_RX_task, "uart_RX_task", 4096, NULL, 2, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(command_processing_task, "Command Task", 4096, NULL, 1, NULL, tskNO_AFFINITY); // Sin núcleo fijo

    

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    // Initialize the LCD panel
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel();


    // Register event callbacks
    register_lcd_event_callbacks(panel_handle, &disp_drv);

    // Initialize I2C
    init_i2c();

    // Initialize GPIO
    //gpio_init();    


    // Initialize touch controller
    esp_lcd_touch_handle_t tp = init_touch(panel_handle);

  
    // Initialize LVGL
    lv_disp_t *disp = init_lvgl(panel_handle);

    start_lvgl_task(disp, tp);

    ESP_LOGI(TAG, "Sistema inicializado. Interfaz lista.");

    if (lvgl_lock(-1)) {
        lv_obj_clean(lv_scr_act());  // Limpia la pantalla actual
        create_keypad();  // Crea el widget de la botonera
        lvgl_unlock();
    }

    //vTaskStartScheduler();


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
        .num_fbs = LCD_NUM_FB,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = PIN_NUM_DISP_EN,
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
        .x_max = LCD_V_RES,
        .y_max = LCD_H_RES,
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
    return disp;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_lvgl_task(lv_disp_t *disp, esp_lcd_touch_handle_t tp)
{
    
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
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
    ESP_LOGI(TAG, "Create LVGL task en core 1");
    xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, disp, 2, NULL, 1);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void register_lcd_event_callbacks(esp_lcd_panel_handle_t panel_handle, lv_disp_drv_t *disp_drv)
{
    ESP_LOGI(TAG, "Register event callbacks");

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_event,
    };

    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, disp_drv));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
