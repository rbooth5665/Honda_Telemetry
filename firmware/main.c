#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"

//Handshake pin, UART TX and RX definitions
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16

//ECU wakeup frame, response buffer, and buffer validation
const uint8_t WAKEUP_FRAME[] = {0xFE, 0x04, 0x72, 0x8C};
const uint8_t WAKEUP_RESPONSE[] = {0x0E, 0x04, 0x72, 0x7C};
uint8_t WAKEUP_RESPONSE_BUFFER[8];

//Data request frame, raw response buffer, cleaned response buffer
const uint8_t DIAG_FRAME[] = {0x72, 0x05, 0x71, 0x11, 0x07};
uint8_t RAW_RESPONSE_BUFFER[31];
uint8_t CLEAN_RESPONSE_BUFFER[26];
uint8_t PAYLOAD_BUFFER[21];

/*
UART definition for hex transmission:
  Baud,   size,       stop,    parity 
{10400, 8 Bits, 1 Stop bit, No parity}

Configuration of driver and structure
*/
const int uart_buffer_size = (1024 * 2);
uart_config_t uart_config = {
    .baud_rate = 10400,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1
};

/*
Enum states to carry machine state from:
1. Connect
2. polling
3. Disconnect
*/
enum States {
    STATE_ECU_CONNECTION,
    STATE_POLLING,
    STATE_DISCONNECTING
};

//------------------------Initial Setup Methods------------------------//
//wakup method for ECU initialization
void handshake_connect() {
    //sets pin to output for handshake
    gpio_set_direction(UART_TX_PIN, GPIO_MODE_OUTPUT);
    
    //sets idle to low to wake ECU, then returns to high
    gpio_set_level(UART_TX_PIN, 0);
    vTaskDelay(70/portTICK_PERIOD_MS);  
    gpio_set_level(UART_TX_PIN, 1);
    vTaskDelay(130/portTICK_PERIOD_MS);
}
//UART setup for communication
void uart_setup() {
    //Initialize UART driver and UART config, sets pins for UART2
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

//------------------------Frame Helper Methods------------------------//
//checksum method takes frame array and returns the Honda checksum
int checksum(uint8_t* frame, int frame_size) {
    int checksum = 0;

    //sums frame bytes
    for(int i = 0; i < frame_size; i++) {
        checksum += frame [i];
    }

    //computes intermediate steps
    checksum = checksum ^ 0xFF;
    checksum = checksum + 1;
    checksum = checksum & 0xFF;

    return checksum;
}
//frame validation utilizes checksum against the last byte of frame
bool validate_frame(uint8_t* frame, int frame_size) {
    //calcuates checksum of frame
    int chk = checksum(frame, frame_size - 1);

    //compares calculated checksum against given frame
    if(chk == frame[frame_size - 1]) {
        return true;
    }
    return false;
}
//sends frame via UART
void send_frame(uint8_t* frame, int frame_size) {
    //flush buffer then send frame
    uart_flush(UART_NUM_2);
    uart_write_bytes(UART_NUM_2, (const char*)frame, frame_size);
}
//receive frame function, returns length of frame
int receive_frame(uint8_t* buffer, int buffer_size) { 

    //Moves UART data into buffer array
    int len = uart_read_bytes(UART_NUM_2, buffer, buffer_size, 100 / portTICK_PERIOD_MS);

    //returns length of frame receieved or 0
    return len;
}
//cleans the echo from data request, fills the clean array with 26 data bytes
void clean_frame(uint8_t* raw, int raw_length, uint8_t* clean) {
    //Removes echo request from raw frame
    int offset = 5;
    for(int i = offset; i < raw_length; i++) {
        clean[i - offset] = raw[i];
    }
    printf("Clean_frame performed.\n");
}
//removes the 5 bytes of header from data frame, fills array with 21 byte payload
void strip_header(uint8_t* unstripped, int unstripped_length, uint8_t* stripped) {
    //removes 4 byte initializer and checksum byte
    int offset = 4;
    for(int i = offset; i < unstripped_length - 1; i++) {
        stripped[i - offset] = unstripped[i];
    }
    printf("Strip_header performed.\n");
}

//------------------------Frame Sending Methods------------------------//
//sends ECU wakeup hex, reads response and validates. Returns True or False
bool ecu_wakeup(int retry) {
    int len = 0;
    //while len is 0 and retry > 0 continue sending frame
    while(len != 8 && retry > 0) {
        //sends wakeup frame to ECU
        send_frame((uint8_t*)WAKEUP_FRAME, sizeof(WAKEUP_FRAME));

        //read echo and response into wakeup buffer
        len = uart_read_bytes(UART_NUM_2, (uint8_t*)WAKEUP_RESPONSE_BUFFER, sizeof(WAKEUP_RESPONSE_BUFFER), 100 / portTICK_PERIOD_MS);
        retry--;

        //waits 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //checks received length
    if(len != 8) {
        printf("ECU Frame sending failed with Length: %d\n", len);
        return false;
    }
    else {
        //compares response buffer to wakeup frame
        int offset = sizeof(WAKEUP_FRAME);
        for(int i = 0; i < sizeof(WAKEUP_RESPONSE); i++) {
            //buffer is 4 byte echo followed by 4 byte response
            if(WAKEUP_RESPONSE[i] != WAKEUP_RESPONSE_BUFFER[i + offset]) {
                printf("Received ECU handshake is different than expected.\n");
                return false;
            }
        }
    }

    printf("ECU Wakeup successful\n");
    return true;
}
//sends data polling hex, fills buffer array with all 26 bytes
bool poll() {
    //sends data request frame to ECU
    send_frame((uint8_t*)DIAG_FRAME, sizeof(DIAG_FRAME));

    //reads echo and response into diag buffer
    int len = uart_read_bytes(UART_NUM_2, (uint8_t*)RAW_RESPONSE_BUFFER, sizeof(RAW_RESPONSE_BUFFER), 100 / portTICK_PERIOD_MS);
    
    //checks received length
    if(len != sizeof(RAW_RESPONSE_BUFFER)) {
        printf("Polling failed, length not the expected 31 bytes\n");
        return false;
    }
    else {
        //takes the full 31 bytes into raw buffer, parses to 26 bytes, validates, parses to 21 bytes
        clean_frame(RAW_RESPONSE_BUFFER, sizeof(RAW_RESPONSE_BUFFER), CLEAN_RESPONSE_BUFFER);

        //verifies the checksum of the clean 26 byte frame. If it fails, return false
        if(!validate_frame(CLEAN_RESPONSE_BUFFER, sizeof(CLEAN_RESPONSE_BUFFER))) {
            printf("Checksum validation for cleaned 26 byte frame failed.\n");
            return false;
        }
        //Strips remaining 5 header bytes and fills the payload into payload buffer
        else {
            strip_header(CLEAN_RESPONSE_BUFFER, sizeof(CLEAN_RESPONSE_BUFFER), PAYLOAD_BUFFER);
        }
    }
    //if length check and checksum validation passes
    printf("Data polled successfully, passed to 21 bytes.\n");
    return true;
}

void app_main() {
    handshake_connect();
    uart_setup();

    if(ecu_wakeup(30)) {
        while(1) {
            //polls data, returns true if bytes are received. Bytes are now stored in PAYLOAD_BUFFER
            if(poll()) {
                //prints all 21 bytes in a single line
                for(int i = 0; i < sizeof(PAYLOAD_BUFFER); i++) {
                    printf("%02X ", PAYLOAD_BUFFER[i]);
                }
                printf("\n");
           }
        }
    }
    else {
        printf("ECU Handshake Failed\n");
    }
}