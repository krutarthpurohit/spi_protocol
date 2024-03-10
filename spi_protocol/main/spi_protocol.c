/* Including required library files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"


/* Defining pins for ESP32 which uses MISO, MOSI, CS, SCLK */
#define ESP_HOST     VSPI_HOST                              // Selecting host(ESP32) to work in VSPI mode
#define PIN_NUM_MISO GPIO_NUM_32                            // MISO pin is present at GPIO_NUM_32
#define PIN_NUM_MOSI GPIO_NUM_23                            // MOSI pin is present at GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18                            // SCLK pin is present at GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5                             // CS' pin is present at GPIO_NUM_5

/* Declaring the funtions which are used in the program */
void vSpiInit(void);
void spi_read_data(uint8_t addr);
void spi_write_data(uint8_t addr, uint8_t data);

#define SPI_TAG "spi_protocol"

esp_err_t ret;
spi_device_handle_t spi;

void vSpiInit(void)
{
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);                   // Setting the CS' pin to work in OUTPUT mode

    spi_bus_config_t buscfg = {                                         // Provide details to the SPI_bus_sturcture of pins and maximum data size
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512 * 8                                      // 4095 bytes is the max size of data that can be sent because of hardware limitations
    };

    spi_device_interface_config_t devcfg = {
        // configure device_structure
        .clock_speed_hz = 12 * 1000 * 1000,                             // Clock out at 12 MHz
        .mode = 0,                                                      // SPI mode 0: CPOL:-0 and CPHA:-0
        .spics_io_num = PIN_NUM_CS,                                     // This field is used to specify the GPIO pin that is to be used as CS'
        .queue_size = 7,                                                // We want to be able to queue 7 transactions at a time
    };

    ret = spi_bus_initialize(ESP_HOST, &buscfg, SPI_DMA_CH_AUTO);       // Initialize the SPI bus
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(ESP_HOST, &devcfg, &spi);                  // Attach the Slave device to the SPI bus
    ESP_ERROR_CHECK(ret);
}

void spi_write_data(uint8_t addr, uint8_t data)     // Function to write data at given address
{
    /*If MSB of addr is set the host will read data from slave.
     *If MSB of addr is clear the host will write data on slave.
     */
    spi_transaction_t trans_desc = {
        // Configure the transaction_structure
        .flags = SPI_TRANS_USE_TXDATA,                                  // Set the Tx flag
        .tx_data = {addr, data},                                        // The host will sent the address first followed by data we provided
        .length = 16,                                                   // Length of the address + data is 16 bits
    };
    gpio_set_level(PIN_NUM_CS, 0);                                      // Lower the CS' line to select the slave
    ret = spi_device_polling_transmit(spi, &trans_desc);                // spi_device_polling_transmit starts to transmit entire 'trans_desc' structure.
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI read operation failed because SPI bus not initialized\n");
        ESP_LOGE(SPI_TAG, "Run \"spi_start 1\" to initialize the spi bus");
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);                                 // Once data is transferred, we provide the delay and then higher the CS'
    gpio_set_level(PIN_NUM_CS, 1);                                      // After CS' is high, the slave sill get unselected
}

void spi_read_data(uint8_t addr)               // Function to read data at given address
{
    /*If MSB of addr is set the host will read data from slave.
     *If MSB of addr is clear the host will write data on slave.
     */
    uint8_t instruction_to_read_data[2] = {0x80 | addr, 0xFF};             // This line sets the MSB of addr vatiable, as a command to read the loaction.
    spi_transaction_t trans_desc = {                                       // Configure the transaction_structure
                                    .flags = SPI_TRANS_USE_RXDATA,         // Set the Rx flag
                                    .tx_buffer = instruction_to_read_data, // Host need to first transmit the (command + address) to slave which the host wants to read
                                    .rxlength = 16,                        // 8*2 = 16 bit data transfer (MAX)
                                    .length = 16};
    gpio_set_level(PIN_NUM_CS, 0);
    ret = spi_device_polling_transmit(spi, &trans_desc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI write operation failed because SPI bus not initialized\n");
        ESP_LOGE(SPI_TAG, "Run \"spi_start 1\" command to initialize the spi bus");
    }
    printf("Data Read %d- %d\n", addr, trans_desc.rx_data[1]);              // Host can fetch the data that received from the slave from inbuild structure member- rx_data directly
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_CS, 1);
}

int app_main(void)
{
    vSpiInit();

    uint8_t addr = 0x22; // address = 0x22
    uint8_t data = 0xAA; // data = 0xAA

    spi_read_data(addr); // Read the value present at address (0x22)
    spi_write_data(addr, data); // Write the value '0xAA' at the location 0x22
    
    return 0;
}