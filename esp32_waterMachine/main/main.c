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
#include "time.h"

#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4

static const char *TAG = "demo";

#define UART_NUM 1  
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define UART_BUFFER_SIZE 1024
#define MAX_COMMANDS 100

#define MAX_RETRIES 3  // Número máximo de intentos de reenvío
#define ACK_TIMEOUT_MS 5000 // Tiempo de espera para recibir ACK en milisegundos


static lv_obj_t *received_label;

#include <stdint.h>

typedef struct {
    char command[5];               // Código de comando
    char codigo_respuesta[3];      // 2 caracteres + '\0'
    char codigo_comercio[13];      // 12 caracteres + '\0'
    char terminal_id[9];           // 8 caracteres + '\0'
    char ticket_number[21];        // 20 caracteres + '\0'
    char autorizacion[7];          // 6 caracteres + '\0'
    char monto[10];                // 9 caracteres + '\0'
    char ultimos_4[5];             // 4 caracteres + '\0'
    char operacion[7];             // 6 caracteres + '\0'
    char tipo_tarjeta[3];          // 2 caracteres + '\0'
    char fecha_transaccion[9];     // 8 caracteres + '\0'
    char hora_transaccion[7];      // 6 caracteres + '\0'
    char comercio_prestador[13];   // 12 caracteres + '\0' (para ventas multicomercio)
    char abreviacion_tarjeta[5];   // 4 caracteres + '\0'

    // Datos opcionales para tarjeta de débito
    char fecha_contable[7];        // 6 caracteres + '\0'
    char numero_cuenta[20];        // 19 caracteres + '\0'

    // Datos opcionales para cuotas
    char tipo_cuota[3];            // 2 caracteres + '\0'
    char numero_cuota[3];          // 2 caracteres + '\0'
    char monto_cuota[13];          // 12 caracteres + '\0'
    char glosa_tipo_cuota[31];     // 30 caracteres + '\0'

    // Campo de impresión
    char campo_impresion[1];          // 0 o 1 (indica si incluye impresión)
    char impresion_formateada[1441]; // Hasta 1440 caracteres + '\0'

    char enviar_msj[1];               // 0 o 1 (indica si se debe enviar mensaje)
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
void init_task(void *param);






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

static SemaphoreHandle_t lvgl_mux = NULL;   // semaphore for LVGL
static QueueHandle_t uart_event_queue = NULL; // cola para eventos de UART
static QueueHandle_t command_queue = NULL; // cola para procesar comandos
static QueueHandle_t ack_queue = NULL; // cola para ACKs


static lv_obj_t *textarea; // Objeto de entrada de texto
static lv_obj_t *led_indicator;  // Objeto del LED
static lv_obj_t *led_label;  // Etiqueta para el número de intentos







// **Función para actualizar el indicador LED con el estado actual**
void update_led_indicator(int status, int attempts) {
    if (lvgl_lock(-1)) {
        if (status == 2) {  // ACK recibido (verde)
            lv_obj_set_style_bg_color(led_indicator, lv_color_make(0, 255, 0), LV_PART_MAIN);
        } else if (status == 3) {  // NAK recibido (rojo)
            lv_obj_set_style_bg_color(led_indicator, lv_color_make(255, 0, 0), LV_PART_MAIN);
        } else {  // Estado neutro (gris)
            lv_obj_set_style_bg_color(led_indicator, lv_color_make(200, 200, 200), LV_PART_MAIN);
        }

        // Actualizar número de intentos
        char attempt_str[4];
        snprintf(attempt_str, sizeof(attempt_str), "%d", attempts);
        lv_label_set_text(led_label, attempt_str);

        // **Forzar actualización de la interfaz**
        lv_obj_invalidate(led_indicator);
        lv_refr_now(NULL);  // Fuerza la actualización de LVGL

        lvgl_unlock();
    }
}



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
        // Crear una task para manejar el polling
    //xTaskCreate(init_task, "init_task", 4096, NULL, 2, NULL);
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
    if (strcmp(command, "0200") == 0) {  // solicitud de transacción de venta
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
                strncpy(command_data->campo_impresion, field, sizeof(command_data->campo_impresion) - 1);
                break;
            case 5:
                strncpy(command_data->enviar_msj, field, sizeof(command_data->enviar_msj) - 1);
                break;
        }
    } else if (strcmp(command, "0210") == 0) { // respuesta de solicitud de transanción venta
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
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
    } else if (strcmp(command, "0270") == 0) { // solicitud de venta
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
                strncpy(command_data->campo_impresion, field, sizeof(command_data->campo_impresion) - 1);
                break;
            case 5:
                strncpy(command_data->enviar_msj, field, sizeof(command_data->enviar_msj) - 1);
                break;
            case 6:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
                break;
            case 7:
                strncpy(command_data->monto, field, sizeof(command_data->monto) - 1);
                break;
        }
    } else if (strcmp(command, "0271") == 0) {
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
                break;
            case 4:
                strncpy(command_data->terminal_id, field, sizeof(command_data->terminal_id) - 1);
                break;
            case 5:
                strncpy(command_data->ticket_number, field, sizeof(command_data->ticket_number) - 1);
                break;
            case 6:
                strncpy(command_data->monto, field, sizeof(command_data->monto) - 1);
                break;
            case 7:
                strncpy(command_data->ultimos_4, field, sizeof(command_data->ultimos_4) - 1);
                break;
            case 8:
                strncpy(command_data->operacion, field, sizeof(command_data->operacion) - 1);
                break;
            case 9:
                strncpy(command_data->tipo_tarjeta, field, sizeof(command_data->tipo_tarjeta) - 1);
                break;
            case 10:
                strncpy(command_data->fecha_contable, field, sizeof(command_data->fecha_contable) - 1);
                break;
            case 11:
                strncpy(command_data->numero_cuenta, field, sizeof(command_data->numero_cuenta) - 1);
                break;
            case 12:
                strncpy(command_data->abreviacion_tarjeta, field, sizeof(command_data->abreviacion_tarjeta) - 1);
                break;
            case 13:
                strncpy(command_data->fecha_transaccion, field, sizeof(command_data->fecha_transaccion) - 1);
                break;
            case 14:
                strncpy(command_data->hora_transaccion, field, sizeof(command_data->hora_transaccion) - 1);
                break;
            case 15:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1); //comercio prestador
                break;
            case 16:
                strncpy(command_data->impresion_formateada, field, sizeof(command_data->impresion_formateada) - 1); //campo de impresión
                break;
            case 17:
                strncpy(command_data->tipo_cuota, field, sizeof(command_data->tipo_cuota) - 1);
                break;
            case 18:
                strncpy(command_data->numero_cuota, field, sizeof(command_data->numero_cuota) - 1);
                break;
            case 19:
                strncpy(command_data->monto_cuota, field, sizeof(command_data->monto_cuota) - 1);
                break;
            case 20:
                strncpy(command_data->glosa_tipo_cuota, field, sizeof(command_data->glosa_tipo_cuota) - 1);
                break;
        }
    } else if (strcmp(command, "0250") == 0) { //datos de última venta
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2: 
                strncpy(command_data->campo_impresion, field, sizeof(command_data->campo_impresion) - 1);
                break;
        }
    } else if (strcmp(command, "0260") == 0) { // respues a solicitud de datos de última venta
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2: 
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3: 
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
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
            case 16:
                strncpy(command_data->impresion_formateada, field, sizeof(command_data->impresion_formateada) - 1);
                break;
            case 17:
                strncpy(command_data->tipo_cuota, field, sizeof(command_data->tipo_cuota) - 1);
                break;
            case 18:
                strncpy(command_data->numero_cuota, field, sizeof(command_data->numero_cuota) - 1);
                break;
            case 19:
                strncpy(command_data->monto_cuota, field, sizeof(command_data->monto_cuota) - 1);
                break;
            case 20:
                strncpy(command_data->glosa_tipo_cuota, field, sizeof(command_data->glosa_tipo_cuota) - 1);
                break;
            
        }
    } else if(strcmp(command, "0900") == 0){    // solicitud mensajes intermedios desde el POS para una venta
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
        }
    } else if(strcmp(command, "0500") == 0){    // solicitud de cierre
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
        }
    } else if(strcmp(command, "0510") == 0){    // respuesta de solicitud de cierre
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
                break;
            case 4:
                strncpy(command_data->terminal_id, field, sizeof(command_data->terminal_id) - 1);
                break;
            case 5:
                strncpy(command_data->impresion_formateada, field, sizeof(command_data->impresion_formateada) - 1);
                break;
        }
    } else if(strcmp(command, "0800") == 0){ //solicitud de carga de llaves
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
        }
    } else if(strcmp(command, "0810") == 0){ //respuesta de solicitud de carga de llaves
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->codigo_respuesta, field, sizeof(command_data->codigo_respuesta) - 1);
                break;
            case 3:
                strncpy(command_data->comercio_prestador, field, sizeof(command_data->comercio_prestador) - 1);
                break;
            case 4:
                strncpy(command_data->terminal_id, field, sizeof(command_data->terminal_id) - 1);
                break;
        }
    } else if(strcmp(command, "0100") == 0){ //solicitud de polling
        switch (field_number) {
            case 1:
                strncpy(command_data->command, field, sizeof(command_data->command) - 1);
                break;
            case 2:
                strncpy(command_data->campo_impresion, field, sizeof(command_data->campo_impresion) - 1);
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

static void transaction_task(void *param) {
    uint8_t *command = (uint8_t *)param;
    size_t length = strlen((char *)command);
    xQueueReset(ack_queue); // Limpiar la cola de ACKs
    
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        printf("Intento %d de %d: Enviando comando...\n", attempt, MAX_RETRIES);
        
        // Enviar el comando por UART
        uart_write_bytes(UART_NUM, (const char *)command, length);

        // **Actualizar LED a estado neutro y mostrar intentos**
        update_led_indicator(0, attempt);

        // Esperar ACK/NAK sin bloquear la interfaz
        uint8_t received_byte;
        if (xQueueReceive(ack_queue, &received_byte, pdMS_TO_TICKS(ACK_TIMEOUT_MS))) {
            if (received_byte == 0x06) {  // ACK recibido
                printf("ACK recibido! Comando confirmado.\n");
                update_led_indicator(2, attempt);  // LED verde
                    xQueueReset(ack_queue);

                break;
            } else if (received_byte == 0x15) {  // NAK recibido
                printf("NAK recibido! Reintentando...\n");
                update_led_indicator(3, attempt);  // LED rojo
            }
        } else {
            printf("Tiempo de espera agotado en intento %d.\n", attempt);
            update_led_indicator(3, attempt);  // LED rojo

        }
    }

    // Liberar memoria asignada para el comando
    free(command);
    vTaskDelete(NULL);  // Terminar la tarea
}

// Generar una solicitud de transacción de venta con el monto ingresado
void generate_transaction_command(const char *monto) {
    uint8_t STX = 0x02;
    uint8_t ETX = 0x03;

    char *N_ticket = generateAlphanumericString(20);
    const char *codigo_cmd = "0200";
    const char *campo_impresion_campo = "1";
    const char *msj_status = "1";

    char command[256];
    snprintf(command, sizeof(command), "%s|%s|%s|%s|%s", codigo_cmd, monto, N_ticket, campo_impresion_campo, msj_status);

    size_t command_length = strlen(command) + 3;
    uint8_t *formatted_command = malloc(command_length);
    if (!formatted_command) {
        printf("Error: No se pudo asignar memoria para el comando.\n");
        free(N_ticket);
        return;
    }

    size_t index = 0;
    formatted_command[index++] = STX;
    memcpy(&formatted_command[index], command, strlen(command));
    index += strlen(command);
    formatted_command[index++] = ETX;

    uint8_t lrc = calculate_lrc(formatted_command+1, index);
    formatted_command[index++] = lrc;

    // **Crear la tarea para enviar el comando de manera asíncrona**
    xTaskCreate(transaction_task, "transaction_task", 4096, formatted_command, 2, NULL);

    free(N_ticket);  // Liberar memoria del ticket
}

// Evento para confirmar y generar el comando
static void confirm_mount_event_handler(lv_event_t *e) {
    const char *amount = lv_textarea_get_text(textarea);

    if (strlen(amount) > 0) {
        if(lvgl_lock(-1)) {
        lv_textarea_set_text(textarea, "");
        lvgl_unlock();
        }
        generate_transaction_command(amount); // se envía comando por UART

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

    // Crear la botonera centrada
    lv_obj_t *btnm = lv_btnmatrix_create(lv_scr_act());
    lv_obj_set_size(btnm, 250, 300);  // Tamaño estándar
    lv_obj_align(btnm, LV_ALIGN_CENTER, 0, 20);  // Centrado en la pantalla

    lv_btnmatrix_set_map(btnm, btnm_map);
    lv_obj_add_event_cb(btnm, keypad_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

    // **Crear el LED en la esquina superior derecha**
    led_indicator = lv_obj_create(lv_scr_act());
    lv_obj_set_size(led_indicator, 50, 50);  // Tamaño cuadrado
    lv_obj_align(led_indicator, LV_ALIGN_TOP_RIGHT, -10, 10);  // Alinear en la esquina superior derecha
    lv_obj_set_style_bg_color(led_indicator, lv_color_make(200, 200, 200), LV_PART_MAIN);  // Color gris neutro

    // Crear la etiqueta para el número de intentos dentro del LED
    led_label = lv_label_create(led_indicator);
    lv_label_set_text(led_label, "0");
    lv_obj_center(led_label);  // Alinear el texto en el centro del LED
}


// **Procesar los comandos recibidos por UART**
int process_uart_data(const uint8_t *data, size_t length) {
    uint8_t temp_message[UART_BUFFER_SIZE] = {0};
    size_t temp_index = 0;
    char command[5] = {0};
    CommandData current_command = {0};

    for (size_t i = 0; i < length; i++) {
        printf("Byte recibido: 0x%02X\n", data[i]);  // Ver cada byte recibido en HEX

        if (data[i] == 0x06) {
            printf("ACK recibido. Enviando confirmación...\n");
            //uart_write_bytes(UART_NUM, "ACK recibido\n", strlen("ACK recibido\n"));
            return 2;
        } 
        else if (data[i] == 0x15) {
            printf("NAK recibido. Enviando confirmación...\n");
            //uart_write_bytes(UART_NUM, "NAK recibido\n", strlen("NAK recibido\n"));
            return 3;
        } 

        if (data[i] == 0x02) { // Inicio del comando
            temp_index = 0;
            memset(temp_message, 0, sizeof(temp_message));
            memset(&current_command, 0, sizeof(current_command));
        } 
        else if (data[i] == 0x03) { // Fin del comando
            if (temp_index == 0 || temp_index >= UART_BUFFER_SIZE - 1) {
                return 0; // Comando vacío o demasiado largo
            }

            uint8_t lrc_calculated = calculate_lrc(temp_message, temp_index);
            if (i + 1 >= length) {
                return 10; // LRC fuera de rango
            }
            uint8_t lrc_received = data[i + 1];

            if (lrc_calculated != lrc_received) {
                return 0; // LRC incorrecto
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
            return 1; // Comando procesado correctamente
        } 
        else { // Datos intermedios
            if (temp_index >= sizeof(temp_message) - 1) {
                return 9; // Buffer lleno, comando inválido
            }
            temp_message[temp_index++] = data[i];
        }
    }

    return 0; // Si el bucle termina sin encontrar un fin de comando
}


// **Task para procesar comandos y responder con ACK/NAK**
static void command_processing_task(void *param) {
    uint8_t command_buffer[UART_BUFFER_SIZE];

    while (1) {
        // Esperar a que un comando esté en la cola
        if (xQueueReceive(command_queue, command_buffer, portMAX_DELAY)) {
            printf("Procesando comando: %s\n", command_buffer);

            // Procesar comando y verificar LRC
            int is_valid = process_uart_data(command_buffer, strlen((char *)command_buffer));
            const char *response = NULL;
            if(is_valid == 0)  {
                printf("Error: LRC incorrecto\n");
                response = "NAK: LRC incorrecto\n";
            } else if(is_valid == 1) {
                printf("Comando procesado correctamente\n");
                response = "ACK: Comando procesado correctamente\n";
            } else if(is_valid == 2) {
                printf("ACK recibido\n");
                response = "ACK recibido\n";
            } else if(is_valid == 3) {
                printf("NAK recibido\n");
                response = "NAK recibido\n";
            } else if (is_valid == 9) {
                printf("Error: Buffer lleno\n");
                response = "NAK: Buffer lleno\n";
            } else if (is_valid == 10) {
                printf("Error: LRC fuera de rango\n");
                response = "NAK: LRC fuera de rango\n";
            } else {
                printf("Error: Comando inválido\n");
                response = "NAK: Comando inválido\n";
            }
            
            // Enviar ACK si es válido, NAK si es inválido
            uart_write_bytes(UART_NUM, response, strlen(response));
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

                        // **Buscar ACK (0x06)**
                        for (size_t i = 0; i < bytes_read; i++) {
                            if (data_buffer[i] == 0x06 || data_buffer[i] == 0x15) {
                                printf("ACK/NAK detectado en UART: 0x%02X. Enviándolo a la cola.\n", data_buffer[i]);
                                uint8_t ack_byte = data_buffer[i];  // se guarda el ACK o NAK
                                xQueueSend(ack_queue, &ack_byte, portMAX_DELAY);
                            }

                        }

                        // Enviar mensaje a la cola de procesamiento
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





// **Task para inicializar la comunicación con el POS**
void init_task(void *param) {
    uint8_t init_command[] = {0x02, '0', '0', '7', '0', 0x03};
    init_command[5] = calculate_lrc(init_command + 1, 4);
    uint8_t pulling_command[] = {0x02, '0', '1', '0', '0', 0x03};
    pulling_command[5] = calculate_lrc(pulling_command + 1, 4);
    
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        printf("Intento %d de %d: Enviando comando de inicialización (0070)...\n", attempt, MAX_RETRIES);
        uart_write_bytes(UART_NUM, (const char *)init_command, sizeof(init_command));
        
        uint8_t received_byte;
        if (xQueueReceive(ack_queue, &received_byte, pdMS_TO_TICKS(ACK_TIMEOUT_MS))) {
            if (received_byte == 0x06) {
                printf("ACK recibido! Inicialización exitosa.\n");
                update_led_indicator(2, attempt);
                
                // Enviar comando de pulling
                printf("Enviando comando de pulling (0100)...\n");
                uart_write_bytes(UART_NUM, (const char *)pulling_command, sizeof(pulling_command));
                
                if (xQueueReceive(ack_queue, &received_byte, pdMS_TO_TICKS(ACK_TIMEOUT_MS))) {
                    if (received_byte == 0x06) {
                        printf("ACK recibido! Pulling exitoso.\n");
                        update_led_indicator(2, attempt);
                    } else {
                        printf("NAK recibido en pulling. Error de COM con POS.\n");
                        update_led_indicator(3, attempt);
                    }
                } else {
                    printf("Timeout esperando ACK en pulling. Error de COM con POS.\n");
                    update_led_indicator(3, attempt);
                }
                vTaskDelete(NULL);
                return;
            } else {
                printf("NAK recibido en inicialización. Error de COM con POS.\n");
                update_led_indicator(3, attempt);
                break;
            }
        } else {
            printf("Timeout esperando ACK en inicialización. Error de COM con POS.\n");
            update_led_indicator(3, attempt);
            break;
        }
    }
    vTaskDelete(NULL);
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














#define MAX_ITEMS 10 // Define el número máximo de ítems que quieres permitir


static uint32_t product_count = 1; // Declarar product_count aquí
static lv_obj_t * main_page;
static lv_obj_t * sub_page;
static lv_obj_t * menu;
static lv_obj_t * cont_arr[MAX_ITEMS]; // Arreglo para almacenar las referencias de los contenedores
static uint32_t cont_index = 0;
static lv_obj_t *float_btn_add;
static lv_obj_t *float_btn_del;
static lv_obj_t *float_btn_del_all;


static void save_button_event_cb(lv_event_t * e)
{
    lv_obj_t * sub_page = lv_event_get_user_data(e);
    lv_obj_t * name_ta = lv_obj_get_child(sub_page, 0);
    const char * name = lv_textarea_get_text(name_ta);

    if (strlen(name) == 0) {
        printf("El nombre no puede estar vacío\n");
        return;
    }

    // Buscar el contenedor asociado a la subpágina actual y actualizar su nombre
    for (uint32_t i = 0; i < cont_index; i++) {
        if (lv_menu_get_cur_main_page(menu) == sub_page) {
            lv_obj_t * label = lv_obj_get_child(cont_arr[i], 0);
            lv_label_set_text(label, name);
            return;
        }
    }
}



static void delete_last_item(lv_event_t * e)
{
    if (cont_index == 0) {
        printf("No hay productos para eliminar\n");
        return;
    }

    lv_obj_del(cont_arr[cont_index - 1]); // Eliminar el último contenedor añadido
    cont_arr[cont_index - 1] = NULL; // Limpiar la referencia en el arreglo
    cont_index--;

    // Retroceder el contador de nombres de productos si hay productos creados
    if (product_count > 1) {
        product_count--;
    }

    
}

static void delete_all_items(lv_event_t * e)
{
    while (cont_index > 0) {
        lv_obj_del(cont_arr[cont_index - 1]);
        cont_arr[cont_index - 1] = NULL;
        cont_index--;
    }
    
    product_count=1;
}

static void text_area_focused(lv_event_t * e)
{
    lv_obj_t * kb = lv_event_get_user_data(e);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
}

static void text_area_defocused(lv_event_t * e)
{
    lv_obj_t * kb = lv_event_get_user_data(e);
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t * create_textarea(lv_obj_t * parent, const char * placeholder)
{
    lv_obj_t * ta = lv_textarea_create(parent);
    lv_textarea_set_placeholder_text(ta, placeholder);
    lv_textarea_set_one_line(ta, true);

    lv_obj_t * kb = lv_keyboard_create(lv_scr_act());
    lv_keyboard_set_textarea(kb, ta);
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(ta, text_area_focused, LV_EVENT_FOCUSED, kb);
    lv_obj_add_event_cb(ta, text_area_defocused, LV_EVENT_DEFOCUSED, kb);

    return ta;
}

static void clear_textareas(lv_event_t * e)
{
    lv_obj_t * sub_page = lv_event_get_user_data(e);
    lv_obj_t * name_ta = lv_obj_get_child(sub_page, 0);
    lv_obj_t * price_ta = lv_obj_get_child(sub_page, 1);
    lv_obj_t * desc_ta = lv_obj_get_child(sub_page, 2);
    
    lv_textarea_set_text(name_ta, "");
    lv_textarea_set_text(price_ta, "");
    lv_textarea_set_text(desc_ta, "");
}

static void exit_without_saving_event_cb(lv_event_t * e)
{
    lv_menu_set_page(menu, main_page);
}
/* Event callback for menu page change */
void menu_event_cb(lv_event_t * e)
{
    lv_obj_t * menu = lv_event_get_target(e);
    lv_obj_t * active_page = lv_menu_get_cur_main_page(menu);
    
    if (active_page == sub_page) {
        lv_obj_add_flag(float_btn_add, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(float_btn_del, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(float_btn_del_all, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(float_btn_add, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(float_btn_del, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(float_btn_del_all, LV_OBJ_FLAG_HIDDEN);
    }
}

static void create_new_product(lv_event_t * e)
{
    if (cont_index >= MAX_ITEMS) {
        printf("No se pueden añadir más productos\n");
        return;
    }

    // Crear una nueva subpágina para el producto
    lv_obj_t * new_sub_page = lv_menu_page_create(menu, NULL);

    lv_obj_t * name_ta = create_textarea(new_sub_page, "Nombre");
    lv_obj_t * price_ta = create_textarea(new_sub_page, "Precio");
    lv_obj_t * desc_ta = create_textarea(new_sub_page, "Descripción");

    lv_obj_t * save_btn = lv_btn_create(new_sub_page);
    lv_obj_t * label = lv_label_create(save_btn);
    lv_label_set_text(label, "Guardar");
    lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, new_sub_page);

    // Crear contenedor en la página principal con nombre predeterminado
    lv_obj_t * cont = lv_menu_cont_create(main_page);
    lv_obj_t * cont_label = lv_label_create(cont);

    //static uint32_t product_count = 1;
    static char default_name[20];
    snprintf(default_name, sizeof(default_name), "Producto %" PRIu32, product_count++);

    lv_label_set_text(cont_label, default_name);
    lv_menu_set_load_page_event(menu, cont, new_sub_page);

    lv_obj_scroll_to_view_recursive(cont, LV_ANIM_ON);

    // Guardar la referencia del contenedor en el arreglo
    cont_arr[cont_index] = cont;
    cont_index++;
}



void lv_example_menu_4(void)
{
    /*Create a menu object*/
    menu = lv_menu_create(lv_scr_act());
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t * cont;
    lv_obj_t * label;

    /*Create a sub page*/
    sub_page = lv_menu_page_create(menu, NULL);  // Asigna a la variable global

    lv_obj_t * name_ta = create_textarea(sub_page, "Nombre");
    lv_obj_t * price_ta = create_textarea(sub_page, "Precio");
    lv_obj_t * desc_ta = create_textarea(sub_page, "Descripción");

    lv_obj_t * save_btn = lv_btn_create(sub_page);
    label = lv_label_create(save_btn);
    lv_label_set_text(label, "Guardar");
    lv_obj_add_event_cb(save_btn, save_button_event_cb, LV_EVENT_CLICKED, sub_page);

    lv_obj_t * clear_btn = lv_btn_create(sub_page);
    label = lv_label_create(clear_btn);
    lv_label_set_text(label, "Limpiar");
    lv_obj_add_event_cb(clear_btn, clear_textareas, LV_EVENT_CLICKED, sub_page);

    lv_obj_t * exit_btn = lv_btn_create(sub_page);
    label = lv_label_create(exit_btn);
    lv_label_set_text(label, "Salir sin guardar");
    lv_obj_add_event_cb(exit_btn, exit_without_saving_event_cb, LV_EVENT_CLICKED, NULL);

    /*Create a main page*/
    main_page = lv_menu_page_create(menu, NULL);

    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Producto 1");
    lv_menu_set_load_page_event(menu, cont, sub_page);

    lv_menu_set_page(menu, main_page);

    /* Usar variables globales en lugar de declarar variables locales */
    float_btn_add = lv_btn_create(lv_scr_act());
    lv_obj_set_size(float_btn_add, 50, 50);
    lv_obj_add_flag(float_btn_add, LV_OBJ_FLAG_FLOATING);
    lv_obj_align(float_btn_add, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(float_btn_add, create_new_product, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_add, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_add, LV_SYMBOL_PLUS, 0);
    lv_obj_set_style_text_font(float_btn_add, lv_theme_get_font_large(float_btn_add), 0);

    float_btn_del = lv_btn_create(lv_scr_act());
    lv_obj_set_size(float_btn_del, 50, 50);
    lv_obj_add_flag(float_btn_del, LV_OBJ_FLAG_FLOATING);
    lv_obj_align(float_btn_del, LV_ALIGN_BOTTOM_RIGHT, -70, -10);
    lv_obj_add_event_cb(float_btn_del, delete_last_item, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_del, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_del, LV_SYMBOL_MINUS, 0);
    lv_obj_set_style_text_font(float_btn_del, lv_theme_get_font_large(float_btn_del), 0);

    float_btn_del_all = lv_btn_create(lv_scr_act());
    lv_obj_set_size(float_btn_del_all, 50, 50);
    lv_obj_add_flag(float_btn_del_all, LV_OBJ_FLAG_FLOATING);
    lv_obj_align(float_btn_del_all, LV_ALIGN_BOTTOM_RIGHT, -130, -10);
    lv_obj_add_event_cb(float_btn_del_all, delete_all_items, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_radius(float_btn_del_all, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_img_src(float_btn_del_all, LV_SYMBOL_TRASH, 0);
    lv_obj_set_style_text_font(float_btn_del_all, lv_theme_get_font_large(float_btn_del_all), 0);

    /* Add event handler for page switching */
    lv_obj_add_event_cb(menu, menu_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

















void app_main(void)
{
    // Initialize NVS
    init_nvs();

    // Inicializar UART
    init_uart();


    command_queue = xQueueCreate(10, UART_BUFFER_SIZE); // Cola para comandos recibidos
    ack_queue = xQueueCreate(5, sizeof(uint8_t));  // Cola para almacenar ACKs

    if (!command_queue || !ack_queue) {
        printf("Error: No se pudo crear las colas de comandos o ACK.\n");
    }


    // Iniciar la tarea de eventos UART en el Core 0
    //xTaskCreatePinnedToCore(uart_RX_task, "uart_RX_task", 4096, NULL, 2, NULL, 0); // Core 0
    //xTaskCreatePinnedToCore(command_processing_task, "Command Task", 4096, NULL, 1, NULL, tskNO_AFFINITY); // Sin núcleo fijo
    //xTaskCreatePinnedToCore(init_task, "init_task", 4096, NULL, 2, NULL, 1);
    

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

    start_lvgl_task(disp, tp); // Iniciar la tarea de LVGL en core 1 

    ESP_LOGI(TAG, "Sistema inicializado. Interfaz lista.");

    if (lvgl_lock(-1)) {
        lv_obj_clean(lv_scr_act());  // Limpia la pantalla actual
        lv_example_menu_4();  // Crea la interfaz de menú        
        lvgl_unlock();
    }



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