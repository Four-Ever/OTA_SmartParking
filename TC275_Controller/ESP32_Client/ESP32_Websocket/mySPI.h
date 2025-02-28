#ifndef __MY_SPI_H__
#define __MY_SPI_H__

#include <Arduino.h>
// #include <SPI.h>
#include "driver/spi_slave.h"

// #define PIN_MISO 19
// #define PIN_MOSI 23
// #define PIN_SCLK 18
// #define PIN_SS   5  // Slave Select (CS)



// #define SPI_CLK   1000000 // 1 MHz
// #define SPI_INTERVAL 10  // 10ms


// void my_post_setup_cb(spi_slave_transaction_t *trans);
// void my_post_trans_cb(spi_slave_transaction_t *trans);
// void spi_task(void *pvParameters);


// void my_post_trans_cb(spi_slave_transaction_t *trans) {
//   // Serial.print("[SPI] Received Data: ");
  
  
//   Serial.println();
// }

// void spi_task(void *pvParameters) 
// {
//   spi_slave_transaction_t trans;
//   memset(&trans,1,sizeof(trans));
//   trans.length = 8 * BUFFER_SIZE;  // 32 바이트
//   trans.tx_buffer = tx_buffer;
//   trans.rx_buffer = rx_buffer;
//   while (1) 
//   {
//     if (spi_slave_transmit(VSPI_HOST, &trans, portMAX_DELAY) == ESP_OK) {
//       process_read_datas(rx_buffer);
//     }
//   }
// }

// void init_spi(){
//   spi_bus_config_t buscfg = {
//     .mosi_io_num = PIN_MOSI,
//     .miso_io_num = PIN_MISO,
//     .sclk_io_num = PIN_SCLK,
//     .quadwp_io_num = -1,
//     .quadhd_io_num = -1
//   };

//   spi_slave_interface_config_t slvcfg = {
//     .spics_io_num = PIN_SS,
//     .flags = 0,
//     .queue_size = 32,
//     .mode = 0, 
//     .post_setup_cb = NULL,
//     .post_trans_cb = my_post_trans_cb,
//     // .dma_chan = 2 
//   };

//   // SPI Slave 초기화
//   spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
//   xTaskCreatePinnedToCore(spi_task, "spi_task", 4096, NULL, 1, NULL, 1);
//   Serial.println("SPI Slave Initialized");

    // SPI 송신
  //   spiCommand(vspi);

  //   // txBuf1을 초기화 (0으로 채움)
  //   txBuf1 = 0;

// }


#endif 