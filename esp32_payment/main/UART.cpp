#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define UART_BUFFER_SIZE 1024
#define MAX_COMMANDS 100

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

uint8_t calculate_lrc(const uint8_t *data, size_t length) {
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++) {
        lrc ^= data[i];
    }
    return lrc;
}

void store_command(const CommandData *command) {
    if (command_count < MAX_COMMANDS) {
        command_history[command_count++] = *command;
    } else {
        printf("Historial de comandos lleno. No se puede almacenar más.");
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

void test_unified_task(const uint8_t *data, size_t length) {
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
            uint8_t lrc_calculated = calculate_lrc(data, i + 1); // Incluir STX hasta ETX
            uint8_t lrc_received = data[i + 1];

            if (lrc_calculated != lrc_received) {
                printf("LRC inválido. Calculado: 0x%02X, Recibido: 0x%02X\n", lrc_calculated, lrc_received);
                return;
            }

            temp_message[temp_index] = '\0';
            char *field = strtok((char *)temp_message, "|");
            printf("Mensaje recibido:\n");

            int field_count = 0;
            while (field != NULL) {
                if (field_count == 0) {
                    strncpy(command, field, sizeof(command) - 1);
                }
                process_field(field, ++field_count, command, &current_command);
                field = strtok(NULL, "|");
            }

            store_command(&current_command);
        } else {                                            // el caso en que no sea inicio ni fin de comando, se copia en el buffer
        
            if (temp_index < sizeof(temp_message) - 1) {
                temp_message[temp_index++] = data[i];
            } else {
                printf("Buffer temporal lleno, descartando datos.\n");
            }
        }
    }
}




// Generar datos aleatorios para simular la transacción
void generate_transaction_command(const char *amount) {
    char command[256];

    snprintf(command, sizeof(command), "0210|%s|123456789012|TERMINAL1|AUTH123|CREDITO|20230101|ACCT123|VISA|20230101|123000", amount);
    printf("Comando generado: %s\n", command);

    // Aquí puedes enviar el comando por UART si es necesario.
    // uart_write_bytes(UART_NUM, command, strlen(command));
}



int main() {
    // Ejemplo de prueba 1
uint8_t example1[] = {
    0x02, 0x30, 0x32, 0x30, 0x30, 0x7C, 0x31, 0x30, 0x30, 0x30, 0x7C, 0x31, 0x32, 0x33, 0x7C, 0x50, 0x03, 0x42
};
    printf("Prueba 1:\n");
    test_unified_task(example1, sizeof(example1));

    // Ejemplo de prueba 2
uint8_t example2[] = {0x02, '0', '2', '1', '0', 0x7C, '0', '0', 0x7C, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '1', '2', 0x7C, 'T', 'E', 'R', 'M', 'I', 'D', '1', '2', 0x7C, 'T', 'I', 'C', 'K', 0x7C, 'A', 'U', 'T', 'H', 0x7C, '1', '0', '0', '0', 0x7C, '1', '2', '3', '4', 0x7C, 'O', 'P', 'E', 'R', 0x7C, 'C', 'R', 'E', 'D', 'I', 'T', 0x7C, '2', '0', '2', '3', '0', '1', '0', '1', 0x7C, 'A', 'C', 'C', 'T', 0x7C, 'V', 'I', 'S', 'A', 0x7C, '2', '0', '2', '3', '0', '1', '0', '1', 0x7C, '1', '2', '3', '0', '0', 0x03, 0x44};
    printf("\nPrueba 2:\n");
    test_unified_task(example2, sizeof(example2));

uint8_t example3[] = {0x02, 0x30, 0x32, 0x30, 0x30, 0x7C, 0x33, 0x33, 0x33,0x7C, 0x78 ,0x44, 0x34,0x70 ,0x4F, 0x6B, 0x77, 0x57, 0x75, 0x4C, 0x70, 0x44, 0x39, 0x7A, 0x57, 0x46, 0x71, 0x47, 0x50, 0x7C, 0x31, 0x7C, 0x31, 0x03, 0x75 };
    printf("\nPrueba 3:\n");
    test_unified_task(example3, sizeof(example3));


    return 0;
}
